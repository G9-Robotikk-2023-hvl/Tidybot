from numpy import deg2rad
from roboticstoolbox import jtraj, IK_NR
from TidyArm_plain import TidyArm

robot:TidyArm = TidyArm()

ets = robot.ets()

# Make an IK solver
solver = IK_NR(pinv=True)

seq = [
    deg2rad(180),
    deg2rad(60),
    deg2rad(60),
    deg2rad(60)
]

# Make a goal pose
endPoint = robot.fkine(seq)
print(endPoint)

# Solve the IK problem
solution = solver.solve(ets, endPoint)
print(solution)

if solution.success:
    print("found solution")
else:
    print("could not find solution")
    
traj = jtraj(robot.startPose, solution.q, 100)
# print(traj)
    
robot.plot(traj.q, loop=True, backend="pyplot")