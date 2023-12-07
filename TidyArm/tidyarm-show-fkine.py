from TidyArm_plain import TidyArm

robot:TidyArm = TidyArm()

point = robot.fkine(robot.startPose)
print(point)
