from roboticstoolbox import jtraj
from TidyArm import TidyArm

robot:TidyArm = TidyArm()

traj = jtraj(robot.start, [1,1,1,1], 100)

robot.plot(traj.q, loop=True)