import numpy as np
import random
from scipy.spatial import KDTree
import heapq
import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan 
from rclpy.node import Node



class DifferentialDriveRobot:
    def __init__(self, wheel_radius, wheel_separation, max_speed, max_angular_speed):
        self.wheel_radius = wheel_radius
        self.wheel_separation = wheel_separation
        self.max_speed = max_speed
        self.max_angular_speed = max_angular_speed
    
    def compute_wheel_velocities(self, linear_velocity, angular_velocity):
        # forsikrer oss om at hastighetene er innenfor robotens begrensninger
        linear_velocity = max(min(linear_velocity, self.max_speed), -self.max_speed)
        angular_velocity = max(min(angular_velocity, self.max_angular_speed), -self.max_angular_speed)

        # beregning av hjulhastigheter basert på diff. drive kinematics
        v_left = (linear_velocity - (self.wheel_separation / 2.0) * angular_velocity) / self.wheel_radius
        v_right = (linear_velocity + (self.wheel_separation / 2.0) * angular_velocity) / self.wheel_radius
        
        return v_left, v_right

    def simulate_motion(self, start_pose, linear_velocity, angular_velocity, dt):
        # simulering av bevegelsen til robot fra startposisjon, gitt hastigheter og tidstrinn
        x, y, theta = start_pose
        
        # forhindrer at vi går over robotens kapasiteter
        linear_velocity = np.clip(linear_velocity, -self.max_speed, self.max_speed)
        angular_velocity = np.clip(angular_velocity, -self.max_angular_speed, self.max_angular_speed)
        
        # tilnærmet ny positur etter tiden dt
        if angular_velocity != 0:
            # beregning av radius til "Instantaneous center of curvature" (ICC)
            radius = linear_velocity / angular_velocity
            
            # kalkulerer ICC posisjon
            icc_x = x - radius * np.sin(theta)
            icc_y = y + radius * np.cos(theta)
            
            # kalkulering av ny pose gjennom ICC
            delta_theta = angular_velocity * dt
            new_x = np.cos(delta_theta) * (x - icc_x) - np.sin(delta_theta) * (y - icc_y) + icc_x
            new_y = np.sin(delta_theta) * (x - icc_x) + np.cos(delta_theta) * (y - icc_y) + icc_y
            new_theta = theta + delta_theta
        else:
            new_x = x + linear_velocity * np.cos(theta) * dt
            new_y = y + linear_velocity * np.sin(theta) * dt
            new_theta = theta
        
        # normalisering av vinkel
        new_theta = (new_theta + np.pi) % (2 * np.pi) - np.pi
        return new_x, new_y, new_theta


class OccupancyGrid:
    def __init__(self, width, height, resolution):
        # basert på størrelse til nettet samt størrelse til hver celle i nettet
        self.width = width
        self.height = height
        self.resolution = resolution
        
        self.grid = -np.ones((int(height / resolution), int(width / resolution)))

    def update_cell(self, x, y, value):
        # konverterer robot posisjon til rutenet koordinater, og oppdaterer celleverdi
        grid_x = int(x / self.resolution)
        grid_y = int(y / self.resolution)
        if 0 <= grid_x < self.grid.shape[1] and 0 <= grid_y < self.grid.shape[0]:
            self.grid[grid_y, grid_x] = value

    def is_occupied(self, x, y):
        # kontrolerer om cellen er opptatt
        grid_x = int(x / self.resolution)
        grid_y = int(y / self.resolution)
        if 0 <= grid_x < self.grid.shape[1] and 0 <= grid_y < self.grid.shape[0]:
            return self.grid[grid_y, grid_x] == 1
        else:
            return False  

    def is_free(self, x, y):
        # kontrolerer om cellen ikke er opptatt
        grid_x = int(x / self.resolution)
        grid_y = int(y / self.resolution)
        if 0 <= grid_x < self.grid.shape[1] and 0 <= grid_y < self.grid.shape[0]:
            return self.grid[grid_y, grid_x] == 0
        else:
            return False  

    def get_cell(self, x, y):
        # konverterer koordinatene, og gir celle verdi
        grid_x = int(x / self.resolution)
        grid_y = int(y / self.resolution)
        if 0 <= grid_x < self.grid.shape[1] and 0 <= grid_y < self.grid.shape[0]:
            return self.grid[grid_y, grid_x]
        else:
            return -1  # Out of bounds - celler er satt til ukjent
    

class PRMPlanner:
    def __init__(self, robot, occupancy_grid, num_samples, connection_distance):
        self.robot = robot
        self.occupancy_grid = occupancy_grid
        self.num_samples = num_samples
        self.connection_distance = connection_distance
        self.nodes = []
        self.edges = {}
        self.kdtree = None

    def sample_free(self):
        # tester tilfeldige frie punkter i rommet
        while len(self.nodes) < self.num_samples:
            x = random.uniform(0, self.occupancy_grid.width)
            y = random.uniform(0, self.occupancy_grid.height)
            if self.occupancy_grid.is_free(x, y):
                self.nodes.append((x, y))

    def connect_nodes(self):
        # kobler til noder innenfor ei viss radius
        self.kdtree = KDTree(self.nodes)
        for idx, node in enumerate(self.nodes):
            neighbors = self.kdtree.query_ball_point(node, self.connection_distance)
            for neighbor_idx in neighbors:
                if neighbor_idx != idx:
                    if self.is_path_free(node, self.nodes[neighbor_idx]):
                        self.edges.setdefault(idx, []).append(neighbor_idx)

    def interpolate_path(self, start, end, num_points=10):
    # interpolerer lineært mellom start og slutt
        x_coords = np.linspace(start[0], end[0], num_points)
        y_coords = np.linspace(start[1], end[1], num_points)
        return zip(x_coords, y_coords)

    def is_path_free(self, start, end):
        path = self.interpolate_path(start, end)

        for point in path:
          if not self.occupancy_grid.is_free(point[0], point[1]):
             # om punkt på veien ikke er ledig, gi False
               return False

        # om alle punktene er kollisjonsfri, gi True
        return True


    def plan_path(self, start, goal):
        start_idx = self.kdtree.query(start)[1]
        goal_idx = self.kdtree.query(goal)[1]

        node_path = self.graph_search(start_idx, goal_idx)

        if node_path is not None:
            path = [self.nodes[idx] for idx in node_path]
            return path
        else:
            return None


    def heuristic(self, a, b):
        # euklidisk avstand som heuristisk
        return np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

    def graph_search(self, start_idx, goal_idx):
        # initialiser det åpne settet med en startindeks
        open_set = []
        heapq.heappush(open_set, (0, start_idx))

        # følg veien som er valgt
        came_from = {}

        # kosten fra start til node
        g_score = {node: float("inf") for node in range(len(self.nodes))}
        g_score[start_idx] = 0  

        # estimert totalkostnad fra start til mål, gjennom node
        f_score = {node: float("inf") for node in range(len(self.nodes))}
        f_score[start_idx] = self.heuristic(self.nodes[start_idx], self.nodes[goal_idx])

        while open_set:
            #Nåværende node er den med lavest f_score
            current = heapq.heappop(open_set)[1]

            # om måøet er nådd, rekonstruer og retuner banen
            if current == goal_idx:
                total_path = [current]
                while current in came_from:
                    current = came_from[current]
                    total_path.insert(0, current)
                return total_path

            for neighbor in self.edges.get(current, []):
                tentative_g_score = g_score[current] + self.heuristic(self.nodes[current], self.nodes[neighbor])

                if tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(self.nodes[neighbor], self.nodes[goal_idx])

                    if neighbor not in [i[1] for i in open_set]:
                        heapq.heappush(open_set, (f_score[neighbor], neighbor))

        # hvis open_set er tom og målet ikke ble nådd, returner None
        return None
    
class RobotNavigationNode(Node):
    def __init__(self):
        super().__init__('robot_navigation_node')

        wheel_radius = 0.027  # radius til hjul i meter
        wheel_separation = 0.266  # distansen mellom senteret av hjulene i meter
        max_speed = 0.22  # maksimum lineær hastighet i m/s
        max_angular_speed = 0.25  # maksimum vinkel hastighet i rad/s

        # initialisering av robot, occupancy grid og PRM planner
        self.robot = DifferentialDriveRobot(wheel_radius, wheel_separation, max_speed, max_angular_speed)
        # fikk trøbbel når jeg prøvde å kalle disse inn ovenfra, 
        # så endte opp med å måtte bruke egen definerte variabler
        grid_width = 10.0
        grid_height = 10.0
        grid_resolution = 0.05
        
        self.occupancy_grid = OccupancyGrid(grid_width, grid_height, grid_resolution)
        
        num_samples = 500
        connection_distance = 1.5
        
        self.planner = PRMPlanner(self.robot, self.occupancy_grid, num_samples, connection_distance)



        # publisering av robothastighetskommando
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # henter inn data fra laser_scan noden
        self.laser_scan_subscriber = self.create_subscription(LaserScan, '/scan', self.laser_scan_callback, 10)


    def laser_scan_callback(self, msg):
        #finner nærmeste hinder og vinkel.
        closest_distance = min(msg.ranges)
        angle_of_closest_obstacle = msg.ranges.index(closest_distance) - 180  # juster basert på LiDAR setup (foreløpig 180 grader)

    
        obstacle_threshold = 0.254  # gitt i meter
        max_linear_velocity = self.robot.max_speed
        max_angular_velocity = self.robot.max_angular_speed

        
        linear_velocity = max_linear_velocity
        angular_velocity = 0.0

        if closest_distance < obstacle_threshold:
            linear_velocity = 0.0  
            
            angle_proportion = (90 - abs(angle_of_closest_obstacle)) / 90
            distance_proportion = (obstacle_threshold - closest_distance) / obstacle_threshold
            angular_velocity = max_angular_velocity * angle_proportion * distance_proportion
            angular_velocity = np.clip(angular_velocity, -max_angular_velocity, max_angular_velocity)
            if angle_of_closest_obstacle > 0:
                angular_velocity = -angular_velocity  # snu mot høyre om hindring er på venstre side og vice versa

        self.get_logger().info(f'Closest distance: {closest_distance}') # for debugging
        self.get_logger().info(f'Angle of closest obstacle: {angle_of_closest_obstacle}')
        #Viser seg at roboten ikke snur seg nok, og ikke kjapt nok. som får stakkaren til å rykke fram og tilbake i lang tid

        #linear_velocity = float()
        #angular_velocity = float()

        # genererer en twist-melding for hastighetene
        velocity_command = Twist()
        velocity_command.linear.x = linear_velocity
        velocity_command.angular.z = angular_velocity

        # publisering av disse hastighetene
        self.velocity_publisher.publish(velocity_command)


        


def main(args=None):
    rclpy.init(args=args)

    robot_navigation_node = RobotNavigationNode()

    try:
        rclpy.spin(robot_navigation_node)
    except KeyboardInterrupt:
        pass
    finally:
        robot_navigation_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
