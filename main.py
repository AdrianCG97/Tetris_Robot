import pybullet as pb
import time
import mengine as m
import numpy as np
import os
import open3d as o3d


default_euler = np.array([np.pi, 0, 0])
s_x = 0
s_y = .62 -.25

board_angle = -np.pi/2
spawn_angle = -np.pi

class TetrisBlocks:
    def __init__(self, body, type):
        self.body = body
        self.type = type

class TetrisBoard:
    def __init__(self, x_pos, y_pos, z_pos):
        self.x_blocks = 10
        self.y_blocks = 20
        self.block_size = .04
        self.x_pos = x_pos  #center of board
        self.y_pos = y_pos  #center of board
        self.z_pos = z_pos

    # Returns position of center of block
    def get_square_pos(self,x,y):
        x_center = self.x_pos - (self.block_size*self.x_blocks/2) + (x*self.block_size) + (self.block_size/2)
        y_center = self.y_pos - (self.block_size*self.y_blocks/2) + (y*self.block_size) + (self.block_size/2)
        z_center = self.z_pos + (self.block_size/2)

        return [x_center, y_center, z_center]

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

def generate_block(type):
    # Create a body
    rand_loc = np.random.uniform(-.2, .2, (2, 1))
    print(rand_loc)

    block = 0

    if(type == 'I'):
        block = m.URDF(filename='./Tetris_I_description/urdf/Tetris_I.xacro', static=False, position=[0, -.2, .9], 
                      orientation=m.get_quaternion(np.array([0, 0, 0])))
        
    elif(type == 'J'):
        block = m.URDF(filename='./Tetris_J_description/urdf/Tetris_J.xacro', static=False, position=[.5, -.6, .8], 
                      orientation=m.get_quaternion(np.array([0, 0, 0])))
        
    elif(type == 'T'):
        block = m.URDF(filename='./Tetris_T_description/urdf/Tetris_T.xacro', static=False, position=[0, 0, .8], 
                      orientation=m.get_quaternion(np.array([0, 0, 0])))

    block.set_whole_body_frictions(lateral_friction=2000, spinning_friction=2000, rolling_friction=0)
    m.step_simulation(50)

    return block

def moveTo(robot, pos=None, orient=None, joint_angles=None):
    """Move robot to a given ee_pose or joint angles. If both are given, ee_pose is used."""
    if pos is not None:
        joint_angles = robot.ik(robot.end_effector, target_pos=pos, target_orient=orient,
                                use_current_joint_angles=True)
    if joint_angles is None:
        return

    robot.control(joint_angles)
    while np.linalg.norm(robot.get_joint_angles(robot.controllable_joints) - joint_angles) > 0.03: #originally 0.03
        m.step_simulation(realtime=True)
        # print("Current joints: ",robot.get_motor_joint_states())
        # print(np.linalg.norm(robot.get_joint_angles(robot.controllable_joints) - joint_angles))
    return

def moveToSpawn(robot):
    # Move end effector to a starting position using IK
    moveTo(robot, pos=[s_x, s_y, 1], orient=m.get_quaternion(np.array([np.pi, 0, 0])))
    # target_joint_angles = robot.ik(robot.end_effector,
    #                             target_pos=[.5, 0.7, 1], target_orient=m.get_quaternion(np.array([np.pi, 0, 0])))
    # robot.control(target_joint_angles)
    print("move done!!!!!!!!!!!!!!!!!!!!!!!!!!!1")

def startPos(angle):
    # Move to start position with current angle
    start_joints = robot.get_joint_angles()

    joint_angles = [-1.6593, -1.4655,  1.3258, -2.2778, 1.3295,  1.4550,  1.5538]
    joint_angles[0] = start_joints[0]
    robot.control(joint_angles)
    m.step_simulation(steps=100, realtime=True)

    # Rotate to target angle
    joint_angles[0] = angle
    robot.control(joint_angles)
    m.step_simulation(steps=100, realtime=True)

def getTypeOfBlock():
    pass #TODO

def getAntipodalGrasp():
    pass #TODO

def moveInDirection():
    # Translate end effector along a vector
    pass

def grabBlock(obj):
    position, orientation = pb.getBasePositionAndOrientation(obj.body, physicsClientId=obj.id)

    position = (position[0], position[1] +.03, position[2])  # delete later

    pos_up = (position[0], position[1], 1)
    m.Shape(m.Sphere(.01), static=True, mass=0, position=pos_up, orientation=orientation, collision=False)
    print("Position: ", pos_up)
    print("Orientation: ", orientation)

    pos_up = (position[0], position[1], 1)
    # Move above block
    gripper_orient = default_euler + [0, 0, np.pi/2 + m.get_euler(orientation)[-1]]
    moveTo(robot, pos=pos_up, orient=gripper_orient)
    print("ANGLES: ", robot.get_joint_angles())

    # Open gripper
    robot.set_gripper_position([1]*2)
    m.step_simulation(steps=50, realtime=True)
    
    # Move down
    # pos_up = (position[0], position[1], )
    moveTo(robot, pos=position, orient=gripper_orient)
    
    # Grab block
    robot.set_gripper_position([0]*2, force=5000)
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
    

if __name__ == "__main__":

    # Create environment
    env = m.Env()
    ground = m.Ground()

    # Create Table
    table = m.URDF(filename=os.path.join(m.directory, 'table', 'table.urdf'), static=True, position=[0.25, 0, 0],
                orientation=m.get_quaternion(np.array([0, 0, np.pi/2])))

    # Create Panda manipulator
    robot = m.Robot.Panda(position=[0.5, 0, 0.76], orientation=m.get_quaternion(np.array([0, 0, np.pi])))
    robot.motor_gains = 0.03
    robot.control([-1.6593, -1.4655,  1.3258, -2.2778, 1.3295,  1.4550,  1.5538], set_instantly=True)
    robot.update_joint_limits()

    pb.setDebugObjectColor(1, 0, 0)  # Set color for collision shapes (R,G,B)
    # pb.setDebugObjectMeshScale(0, [1, 1, 1])  # Set mesh scale for collision shapes
    pb.configureDebugVisualizer(pb.COV_ENABLE_RENDERING, 1)
    pb.configureDebugVisualizer(pb.COV_ENABLE_GUI, 1)
    # pb.configureDebugVisualizer(pb.COV_ENABLE_WIREFRAME, 1)

    # Board game
    createTray([.2, .4, 0.01], [0,0,.75], .01)
    board = TetrisBoard(0,0,.75)

    # # Spawn
    # createTray([.15, .15, 0.01], [s_x, s_y, .75], .01)
    
    block = generate_block('I')

    # begin simulation
    pb.setGravity(0, 0, -9.8)

    pb.setRealTimeSimulation(1)


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
    # moveToSpawn(robot)
    startPos(spawn_angle)
    print("IN SPAWN!!!!!!!!!!!!")
    time.sleep(5)

    # Grab block and return to base position
    grabBlock(block)

    # Rotate to board
    startPos(board_angle)
    print("IN BOARD!!!!!!!!!!!!")
    time.sleep(5)

    # Place block and return to base position
    placeBlock(board, block, 0, 0, 1.5)
    m.step_simulation(steps=100, realtime=True)
    startPos(board_angle)
    time.sleep(10)