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
    |Revolute|J1      |L1    |0                 |π/2   |π                   |
    |Revolute|J2      |0     |√( L2^2 + L2.1^2 )|0     |atan(L2_1/L2)       |
    |Revolute|J3      |0     |L3                |0     |-atan(L2_1/L2) + π/2|
    |Revolute|J4      |0     |L4                |0     |0                   |
    
    ## Parameters:
    - `l1`: Base height in meters (default: 177 mm)
    - `l2`: Link 1 length in meters (default: 128 mm)
    - `l2_1`: Link 1 length after angle in meters (default: 24 mm)
    - `l3`: Link 2 length in meters (default: 124 mm)
    - `l4`: Link 3 length in meters (default: 126 mm)
    """
    
    # the starting pose of Tidybot
    startPose:ndarray
    
    def __init__(self,
        l1:float   =  0.0454,
        l2:float   = 0.20,
        l2_1:float = 0.20,
        l3:float   = 0.226,
        l4:float   = 0.082
    ):
        Beta = arctan(l2_1/l2)
        hyp = sqrt(l2**2+l2_1**2)
        DHMatrix = [
            RevoluteDH(d= l1, a= 0  , alpha= pi/2, offset=  pi                                             ),
            RevoluteDH(d= 0 , a= hyp, alpha= 0   , offset=  Beta       , qlim=[deg2rad( -90), deg2rad( 90)]),
            RevoluteDH(d= 0 , a= l3 , alpha= 0   , offset= -Beta + pi/2, qlim=[deg2rad(-102), deg2rad(303)]),
            RevoluteDH(d= 0 , a= l4 , alpha= 0   , offset=  0          , qlim=[deg2rad(-180), deg2rad(180)])
        ]
        super().__init__(DHMatrix, name="TidyArm")
        
        self.startPose = [0 for j in DHMatrix]
        
        L1 = 0.0454;
        L2 = 0.20;
        L3 = 0.20;
        L4 = 0.226;
        L5 = 0.082