from numpy import arctan, deg2rad, ndarray, pi, sqrt
from roboticstoolbox import DHRobot, RevoluteDH

# Defining units
m  = 1
mm = m * 10**-3


class TidyArm(DHRobot):
    """
    # Tidy Arm
    
    tidy arm is just an extension of the DHRobot class
    
    it applies the DH Matrix:
    
    |__Type__|__Θ_____|___D__|_a________________|_α____|_offset_____________|
    |--------|--------|------|------------------|------|--------------------|
    |Revolute|J1      |L1    |0                 |π/2   |0                   |
    |Revolute|J2      |0     |L2                |0     |0                   |
    |Revolute|J3      |0     |L3                |0     |0                   |
    |Revolute|J4      |0     |L4 + L5           |0     |0                   |
    
    ## Parameters:
    - `l1`: Base height
    - `l2`: Link 1
    - `l3`: Link 2
    - `l4`: Link 3
    - `l4`: Link 4
    """
    
    # the starting pose of Tidybot
    startPose:ndarray
    
    def __init__(self,
        l1:float   = 0.0454,
        l2:float   = 0.20,
        l3:float = 0.20,
        l4:float   = 0.226,
        l5:float   = 0.082
    ):
        DHMatrix = [
            RevoluteDH(d= l1, a= 0    , alpha= pi/2, offset=   0                                    ),
            RevoluteDH(d= 0 , a= l2   , alpha= 0   , offset=   0, qlim=[deg2rad( -90), deg2rad( 90)]),
            RevoluteDH(d= 0 , a= l3   , alpha= 0   , offset=   0, qlim=[deg2rad(-102), deg2rad(303)]),
            RevoluteDH(d= 0 , a= l4+l5, alpha= 0   , offset=   0, qlim=[deg2rad(-180), deg2rad(180)])
        ]
        super().__init__(DHMatrix, name="TidyArm")
        
        self.startPose = [0 for j in DHMatrix]