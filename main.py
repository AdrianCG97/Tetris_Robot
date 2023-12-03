import pybullet as pb
import time
import mengine as m
import numpy as np
import os
import open3d as o3d
import random


default_euler = np.array([np.pi, 0, 0])
s_x = .50
s_y = .50

board_angle = 0
spawn_angle = -np.pi/2

class TetrisBlock:
    def __init__(self, body, type):
        self.block = body
        self.type = type

    def getGrabPoint(self):
        pos, orient = self.block.get_base_pos_orient()
        euler = m.get_euler(orient)
        yaw = m.get_euler(orient)[2]

        x_g = pos[0]
        y_g = pos[1]
        z_g = pos[2] + .0     # .01 to grab, .02 to see point
        grab_pos = [x_g, y_g, z_g]  #TODO delete
        m.Shape(m.Sphere(.01), static=True, mass=0, position=grab_pos, collision=False) #TODO delete

        if(self.type == 'T'):
            d = .03
            x_g = x_g + (d * np.cos(yaw + np.pi/2))
            y_g = y_g + (d * np.sin(yaw + np.pi/2))

        elif(self.type == 'I'):
            x_g = x_g
            y_g = y_g

        elif(self.type == 'L'):
            d = .015
            x_g = x_g + (d * np.cos(yaw + 5*np.pi/8))
            y_g = y_g + (d * np.sin(yaw + 5*np.pi/8))

        elif(self.type == 'J'):
            d = .015
            x_g = x_g + (d * np.cos(yaw + 3*np.pi/8))
            y_g = y_g + (d * np.sin(yaw + 3*np.pi/8))

        elif(self.type == 'S'):
            d = .05
            x_g = x_g + (d * np.cos(yaw - 7*np.pi/8))
            y_g = y_g + (d * np.sin(yaw - 7*np.pi/8))
            orient = m.get_quaternion([euler[0], euler[1], yaw + np.pi/2])

        elif(self.type == 'Z'):
            d = .05
            x_g = x_g + (d * np.cos(yaw + 7*np.pi/8))
            y_g = y_g + (d * np.sin(yaw + 7*np.pi/8))
            orient = m.get_quaternion([euler[0], euler[1], yaw + np.pi/2])
        
        grab_pos = [x_g, y_g, z_g]
        m.Shape(m.Sphere(.01), static=True, mass=0, position=grab_pos, collision=False) #TODO delete

        return grab_pos, orient
    
def getBlockCoordinates(blockType, rot):
    coordinates = []

    if(blockType == 'T'):
        coordinates = np.array([[0,0],[1,0],[1,1],[2,0]])

    elif(blockType == 'I'):
        coordinates = np.array([[0,0],[0,1],[0,2],[0,3]])

    elif(blockType == 'L'):
        coordinates = np.array([[0,0],[0,1],[0,2],[1,0]])

    elif(blockType == 'J'):
        coordinates = np.array([[0,0],[0,1],[0,2],[-1,0]])

    elif(blockType == 'S'):
        coordinates = np.array([[0,0],[1,0],[1,1],[2,1]])
        
    elif(blockType == 'Z'):
        coordinates = np.array([[0,0],[-1,1],[0,1],[1,0]])

    elif(blockType == 'O'):
        coordinates = np.array([[0,0],[1,0],[1,0],[1,1]])

    if(rot == 0):
        return coordinates
    
    # Rotate
    for r in range(rot):
        temp = np.zeros(4,2)
        temp[:,0] =  coordinates[:,1]
        temp[:,1] = -coordinates[:,0]

    return coordinates

class TetrisBoard:
    def __init__(self, x_pos, y_pos, z_pos):
        self.x_blocks = 10
        self.y_blocks = 20
        self.block_size = .04
        self.x_pos = x_pos  #center of board
        self.y_pos = y_pos  #center of board
        self.z_pos = z_pos
        self.board = np.zeros((self.x_blocks,self.y_blocks))

    # Returns position of center of block
    def get_square_pos(self,x,y):
        x_center = self.x_pos - (self.block_size*self.x_blocks/2) + (x*self.block_size) + (self.block_size/2)
        y_center = self.y_pos - (self.block_size*self.y_blocks/2) + (y*self.block_size) + (self.block_size/2)
        z_center = self.z_pos + (self.block_size/2)

        return [x_center, y_center, z_center]
    
    def addBlock(self, blockType):
        
        best_position = []
        best_rotation = 0 
        min_score     = 9999999

        for col in range(self.x_blocks):
            for rot in range(4):
                for row in range(self.y_blocks):

                    score = 0
                    #Check block doesn't collide
                    coord = getBlockCoordinates(blockType, rot)
                    for c in range(4):
                        x = col + coord[c,0]
                        y = row + coord[c,1]

                        if(x < 0 or x > self.x_blocks or y < 0 or y > self.y_blocks):
                            score = 9999999  # invalid
                        if(self.board[x,y] == 1):
                            score = 9999999  # invalid
                        
                        score = score + y

                    # Look for dead spaces
                    # TODO
                     
                    # Get score
                    if(score < min_score):
                        min_score = score
                        best_rotation = rot
                        best_position = [row, col]


                         




def get_point_cloud(obj):
    """Returns object's point cloud and normals."""
    # Create two cameras
    camera1 = m.Camera(camera_pos=[s_x, s_y, 1], look_at_pos=obj.get_base_pos_orient()[0], fov=60,
                       camera_width=1920 // 4, camera_height=1080 // 4)
    camera2 = m.Camera(camera_pos=[s_x, s_y, 1], look_at_pos=obj.get_base_pos_orient()[0], fov=60,
                       camera_width=1920 // 4, camera_height=1080 // 4)
    # Show the object
    obj.change_visual(link=obj.base, rgba=[1, 1, 1, 1])
    # Capture a point cloud from the camera
    pc1, rgba1 = camera1.get_point_cloud(body=obj)
    pc2, rgba2 = camera2.get_point_cloud(body=obj)
    pc = np.concatenate([pc1, pc2], axis=0)
    # rgba = np.concatenate([rgba1, rgba2], axis=0)
    # Visualize the point cloud
    # m.DebugPoints(pc, points_rgb=rgba[:, :3], size=10)
    # Hide the object
    obj.change_visual(link=obj.base, rgba=[1, 1, 1, 0.75])

    # Create open3d point cloud from array of points
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pc)

    # Estimate normals for each point
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
    normals = np.asarray(pcd.normals)
    return pc, normals

def generate_block(type, position=[0,0,0.8]):
    # Create a body
    r_yaw = 0 #np.random.uniform(-np.pi, np.pi)
    # print("Random yaw: ", r_yaw)
    block_body = 0
    print("Type: ", type)

    if(type == 'I'):
        position[1] = position[1] - .05 
        block_body = m.URDF(filename='./Tetris_I_description/urdf/Tetris_I.xacro', static=False, position=position,
                      orientation=m.get_quaternion(np.array([0, 0, r_yaw])))
        
    elif(type == 'J'):
        block_body = m.URDF(filename='./Tetris_J_description/urdf/Tetris_J.xacro', static=False, position=position, 
                      orientation=m.get_quaternion(np.array([0, 0, r_yaw])))
        
    elif(type == 'T'):
        block_body = m.URDF(filename='./Tetris_T_description/urdf/Tetris_T.xacro', static=False, position=position, 
                      orientation=m.get_quaternion(np.array([0, 0, r_yaw])))
        
    elif(type == 'L'):
        block_body = m.URDF(filename='./Tetris_L_description/urdf/Tetris_L.xacro', static=False, position=position, 
                      orientation=m.get_quaternion(np.array([0, 0, r_yaw])))
        
    elif(type == 'O'):
        block_body = m.URDF(filename='./Tetris_O_description/urdf/Tetris_O.xacro', static=False, position=position, 
                      orientation=m.get_quaternion(np.array([0, 0, r_yaw])))
        
    elif(type == 'S'):
        block_body = m.URDF(filename='./Tetris_S_description/urdf/Tetris_S.xacro', static=False, position=position, 
                      orientation=m.get_quaternion(np.array([0, 0, r_yaw])))
        
    elif(type == 'Z'):
        block_body = m.URDF(filename='./Tetris_Z_description/urdf/Tetris_Z.xacro', static=False, position=position, 
                      orientation=m.get_quaternion(np.array([0, 0, r_yaw])))

    block_body.set_whole_body_frictions(lateral_friction=2000, spinning_friction=2000, rolling_friction=220)
    m.step_simulation(50)

    block = TetrisBlock(block_body, type)

    return block

def moveTo(robot, pos=None, orient=None, joint_angles=None):
    """Move robot to a given ee_pose or joint angles. If both are given, ee_pose is used."""
    if pos is not None:
        joint_angles = robot.ik(robot.end_effector, target_pos=pos, target_orient=orient,
                                use_current_joint_angles=True)
    if joint_angles is None:
        return

    robot.control(joint_angles,  set_instantly=False)

    while np.linalg.norm(robot.get_joint_angles(robot.controllable_joints) - joint_angles) > 0.03: #originally 0.03
        m.step_simulation(realtime=True)
    return

def moveToSpawn(robot):
    # Move end effector to a starting position using IK
    moveTo(robot, pos=[s_x, s_y, 1], orient=m.get_quaternion(np.array([np.pi, 0, 0])))

def startPos(angle):
    print("Move to angle:", angle)
    # Move to start position with current angle
    start_joints = robot.get_joint_angles()
    print(start_joints)
    joint_angles = [-1.6593, -1.4655,  1.3258, -2.2778, 1.3295,  1.4550,  1.5538]
    joint_angles[0] = start_joints[0]
    robot.control(joint_angles)
    m.step_simulation(steps=100, realtime=True)
    start_joints = robot.get_joint_angles()
    print(start_joints)

    # Rotate to target angle
    joint_angles[0] = angle
    robot.control(joint_angles)
    m.step_simulation(steps=100, realtime=True)
    start_joints = robot.get_joint_angles()
    print(start_joints)

def getTypeOfBlock():
    pass #TODO

def getAntipodalGrasp():
    pass #TODO

def moveInDirection():
    # Translate end effector along a vector
    pass

def grabBlock(obj):
    # position, orientation = pb.getBasePositionAndOrientation(obj.block.body, physicsClientId=obj.block.id)
    position, orientation = obj.getGrabPoint()  # delete later

    # Draw dot on top of block
    pos_up = (position[0], position[1], 1)
    m.Shape(m.Sphere(.01), static=True, mass=0, position=pos_up, orientation=orientation, collision=False)

    # Move above block
    gripper_orient = default_euler + [0, 0, np.pi/2 + m.get_euler(orientation)[-1]]
    moveTo(robot, pos=pos_up, orient=gripper_orient)
    print("ANGLES: ", robot.get_joint_angles())

    # Open gripper
    robot.set_gripper_position([3]*2)
    m.step_simulation(steps=50, realtime=True)

    # Move down
    pos_up = (position[0], position[1], .78)
    # m.Shape(m.Sphere(.01), static=True, mass=0, position=pos_up, orientation=orientation, collision=False)
    moveTo(robot, pos=position, orient=gripper_orient, )
    
    # Grab block
    robot.set_gripper_position([0]*2, force=500)
    m.step_simulation(steps=100, realtime=True)

    # Move up
    moveTo(robot, pos=pos_up, orient=gripper_orient)

def placeBlock(board, block, rot, x, y):
    # Check position/rotation/block combination is valid
        #TODO

    # Get target center and orientation of block 
    target_pos = board.get_square_pos(x,y)
    
    # Move above target location
    pos_up = (target_pos[0], target_pos[1], 1)
    print("Pos Up: ", pos_up)
    gripper_orient = default_euler + [0, 0, np.pi/2]
    m.Shape(m.Sphere(.01), static=True, mass=0, position=pos_up, collision=False)
    moveTo(robot, pos=pos_up, orient=gripper_orient)

    # TODO rotate end effector

    # Lower and release
    pos_down = (target_pos[0], target_pos[1], target_pos[2]+.03)
    print("Pos Up: ", pos_down)
    gripper_orient = default_euler + [0, 0, np.pi/2]
    m.Shape(m.Sphere(.01), static=True, mass=0, position=pos_down, collision=False)
    moveTo(robot, pos=pos_down, orient=gripper_orient)

    robot.set_gripper_position([1]*2)
    m.step_simulation(steps=50, realtime=True)
    pos, ori = robot.get_link_pos_orient(robot.end_effector)
    moveTo(robot, pos + [0, 0, 0.1], ori)


def createTray(size, pos, edge):
    base_x = size[0]
    base_y = size[1]
    base_z = size[2]
    edge_s = edge
    b_pos_x = pos[0]
    b_pos_y = pos[1]
    b_pos_z = pos[2]
    base0 = m.Shape(m.Box(np.array([base_x, base_y, base_z])), static=True, 
                    position=np.array([b_pos_x, b_pos_y, b_pos_z + base_z]), collision=True, rgba=[.6, .6, .6, 1])
    base1 = m.Shape(m.Box(np.array([base_x, edge_s, 2*base_z])), static=True, 
                    position=np.array([b_pos_x, b_pos_y + base_y + edge_s, b_pos_z + 2*base_z]), collision=True, rgba=[.5, .5, .5, 1])
    base2 = m.Shape(m.Box(np.array([edge_s, base_y + (2*edge_s), 2*base_z])), static=True, 
                    position=np.array([b_pos_x + base_x + edge_s, b_pos_y, b_pos_z + 2*base_z]), collision=True, rgba=[.5, .5, .5, 1])
    base3 = m.Shape(m.Box(np.array([base_x, edge_s, 2*base_z])), static=True, 
                    position=np.array([b_pos_x, b_pos_y-(base_y + edge_s), b_pos_z + 2*base_z]), collision=True, rgba=[.5, .5, .5, 1])
    base4 = m.Shape(m.Box(np.array([edge_s, base_y + (2*edge_s), 2*base_z])), static=True, 
                    position=np.array([b_pos_x-( base_x + edge_s), b_pos_y, b_pos_z + 2*base_z]), collision=True, rgba=[.5, .5, .5, 1])
    
def moveToTest(robot, pos=None, orient=None, joint_angles=None):
    """Move robot to a given ee_pose or joint angles. If both are given, ee_pose is used."""
    if pos is not None:
        joint_angles = robot.ik(robot.end_effector, target_pos=pos, target_orient=orient,
                                use_current_joint_angles=True)
    if joint_angles is None:
        return

    m.Shape(m.Sphere(.01), static=True, mass=0, position=pos, orientation=orient, collision=False)
    robot.control(joint_angles,  set_instantly=False)

    while np.linalg.norm(robot.get_joint_angles(robot.controllable_joints) - joint_angles) > 0.03: #originally 0.03
        m.step_simulation(realtime=True)
        # print("Current joints: ",robot.get_motor_joint_states())
        # print(np.linalg.norm(robot.get_joint_angles(robot.controllable_joints) - joint_angles))

    time.sleep(3)
    return

def getAngle(x, y):
    a = np.arctan2(y,x-5) - np.pi
    print("Angle: ", a)
    return np.deg2rad(a)

def testMovement():
    y_up = 0.9
    y_down = 0.82
    gripper_orient = default_euler + [0, 0, np.pi/2]

    # Move to center
    startPos(getAngle(0, 0))
    # startPos(board_angle)
    moveToTest(robot, pos=[0,0,y_up], orient=gripper_orient)
    moveToTest(robot, pos=[0,0,y_down], orient=gripper_orient)
    moveToTest(robot, pos=[0,0,y_up], orient=gripper_orient)

    # Move to Corner 1
    x = -.2
    y = -.4
    startPos(getAngle(x, y))
    moveToTest(robot, pos=[x,y,y_up], orient=gripper_orient)
    moveToTest(robot, pos=[x,y,y_down], orient=gripper_orient)
    moveToTest(robot, pos=[x,y,y_up], orient=gripper_orient)

    # Move to Corner 2
    x = -.2
    y = .4
    startPos(getAngle(x, y))
    moveToTest(robot, pos=[x,y,y_up], orient=gripper_orient)
    moveToTest(robot, pos=[x,y,y_down], orient=gripper_orient)
    moveToTest(robot, pos=[x,y,y_up], orient=gripper_orient)

    # Move to Corner 3
    x = .2
    y = .4
    startPos(getAngle(x, y))
    moveToTest(robot, pos=[x,y,y_up], orient=gripper_orient)
    moveToTest(robot, pos=[x,y,y_down], orient=gripper_orient)
    moveToTest(robot, pos=[x,y,y_up], orient=gripper_orient)

    # Move to Corner 4
    x = .2
    y = -.4
    startPos(getAngle(x, y))
    moveToTest(robot, pos=[x,y,y_up], orient=gripper_orient)
    moveToTest(robot, pos=[x,y,y_down], orient=gripper_orient)
    moveToTest(robot, pos=[x,y,y_up], orient=gripper_orient)

    # Move to Corner 4
    x = .2
    y =  0
    startPos(getAngle(x, y))
    moveToTest(robot, pos=[x,y,y_up], orient=gripper_orient)
    moveToTest(robot, pos=[x,y,y_down], orient=gripper_orient)
    moveToTest(robot, pos=[x,y,y_up], orient=gripper_orient)


def startGame(board):
    gameOver = False
    all_blocks = []
    # Set of characters
    block_types = ['I', 'L', 'J']  # TODO add 'O'


    # Move to center
    # Move to block spawn position
    startPos(spawn_angle)

    while(not gameOver):
        # Spawn new random block
        block_type = random.choice(block_types)
        active_block = generate_block(block_type, [s_x, s_y, .8])

        # Detect block type and run tetris solver to get target position
        # TODO

        # Grab block
        grabBlock(active_block)

        # Place block
        placeBlock(board, active_block, 0, 9, 10)

        # Make adjustments

        # Return to base
        # Move to block spawn position
        startPos(spawn_angle)

        time.sleep(5)

if __name__ == "__main__":

    # Create environment
    env = m.Env()
    ground = m.Ground()

    # Create Table
    table = m.URDF(filename=os.path.join(m.directory, 'table', 'table.urdf'), static=True, position=[0.25, 0, 0],
                orientation=m.get_quaternion(np.array([0, 0, np.pi/2])))

    # Create Panda manipulator
    robot = m.Robot.Panda(position=[0.4, 0, 0.76], orientation=m.get_quaternion(np.array([0, 0, np.pi/2])))
    robot.motor_gains = 0.02
    robot.control([-1.6593, -1.4655,  1.3258, -2.2778, 1.3295,  1.4550,  1.5538], set_instantly=True)
    robot.update_joint_limits()

    print("Robot Limits:")
    print(robot.lower_limits)
    print(robot.upper_limits, "\n\n")

    # Board game
    createTray([.2, .4, 0.01], [0,0,.75], .01)
    board = TetrisBoard(0,0,.75)

    # Spawn
    createTray([.15, .15, 0.01], [s_x, s_y, .75], .01)

    # begin simulation
    pb.setGravity(0, 0, -9.8)
    pb.setRealTimeSimulation(1)


    startGame(board)

    block = generate_block('I', [s_x, s_y, .8])
##################################3
    # testMovement()



    # time.sleep(100)

    # # viewMatrix (describes position and orientation)
    # viewMatrix = pb.computeViewMatrix(
    #     cameraEyePosition=[.5, .7, 1.2],  # Camera Position
    #     cameraTargetPosition=[.5, .7, 0], # Point we want camera to face
    #     cameraUpVector=[0, 1, 0])       # Orientation of camera

    # # projectionMatrix (describes camera's intrinsic properties (FOV, aspect ratio, etc))
    # projectionMatrix = pb.computeProjectionMatrixFOV(
    #     fov=45.0,
    #     aspect=1.0,
    #     nearVal=0.1,
    #     farVal=3.1)

    # width, height, rgbImg, depthImg, segImg = pb.getCameraImage(
    #     width=224, 
    #     height=224,
    #     viewMatrix=viewMatrix,
    #     projectionMatrix=projectionMatrix)

    # generate_block()


    # Move to block spawn position
    startPos(spawn_angle)

    # Grab block and return to base position
    grabBlock(block)

    # Rotate to board
    startPos(board_angle)

    # Place block and return to base position
    placeBlock(board, block, 0, 9, 10)
    m.step_simulation(steps=100, realtime=True)
    startPos(board_angle)
    time.sleep(10)