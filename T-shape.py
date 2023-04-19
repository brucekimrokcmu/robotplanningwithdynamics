import pybullet as p
import time
import pybullet_data
import utils

planner_path_fpath, obstacles_fpath, cpp_fpath = utils.get_file_paths()

physicsClient = p.connect(p.GUI) #or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)

planeId = p.loadURDF("plane.urdf")

# Drop car into environment
startPos = [0,0,0.5]
startOrientation = p.getQuaternionFromEuler([0,0,0])
car = p.loadURDF("car.urdf",startPos, startOrientation)

obstacles = []

# Create a box object
startOrientation = p.getQuaternionFromEuler([0,0,0])

# Middle Right Line
box_extents = [0.1, 1.5, 0.2]
box = p.createCollisionShape(p.GEOM_BOX, halfExtents=box_extents)
box_body_1 = p.createMultiBody(
                    baseMass=1,
                    baseInertialFramePosition=[0, 0, 0],
                    baseCollisionShapeIndex=box, 
                    basePosition=[-1, -2, 0.2])
obstacles.append((box_body_1, box_extents))

# Middle Left Line
box_body_1 = p.createMultiBody(
                    baseMass=1,
                    baseInertialFramePosition=[0, 0, 0],
                    baseCollisionShapeIndex=box, 
                    basePosition=[1, -2, 0.2])
obstacles.append((box_body_1, box_extents))

# Right down curve
box_curve_extents = [0.1, 0.1, 0.2]
box_curve1 = p.createCollisionShape(p.GEOM_BOX, halfExtents=box_curve_extents)
box_body_2 = p.createMultiBody(
                    baseMass=1,
                    baseInertialFramePosition=[0, 0, 0],
                    baseCollisionShapeIndex=box_curve1, 
                    basePosition=[1.1, -0.4, 0.2])
obstacles.append((box_body_2, box_curve_extents))
box_body_2 = p.createMultiBody(
                    baseMass=1,
                    baseInertialFramePosition=[0, 0, 0],
                    baseCollisionShapeIndex=box_curve1, 
                    basePosition=[1.2, -0.2, 0.2])
obstacles.append((box_body_2, box_curve_extents))
box_body_2 = p.createMultiBody(
                    baseMass=1,
                    baseInertialFramePosition=[0, 0, 0],
                    baseCollisionShapeIndex=box_curve1, 
                    basePosition=[1.3, 0.0, 0.2])
obstacles.append((box_body_2, box_curve_extents))
box_body_2 = p.createMultiBody(
                    baseMass=1,
                    baseInertialFramePosition=[0, 0, 0],
                    baseCollisionShapeIndex=box_curve1, 
                    basePosition=[1.4, 0.2, 0.2])
obstacles.append((box_body_2, box_curve_extents))

# Left Down Curve
box_body_2 = p.createMultiBody(
                    baseMass=1,
                    baseInertialFramePosition=[0, 0, 0],
                    baseCollisionShapeIndex=box_curve1, 
                    basePosition=[-1.1, -0.4, 0.2])
obstacles.append((box_body_2, box_curve_extents))
box_body_2 = p.createMultiBody(
                    baseMass=1,
                    baseInertialFramePosition=[0, 0, 0],
                    baseCollisionShapeIndex=box_curve1, 
                    basePosition=[-1.2, -0.2, 0.2])
obstacles.append((box_body_2, box_curve_extents))
box_body_2 = p.createMultiBody(
                    baseMass=1,
                    baseInertialFramePosition=[0, 0, 0],
                    baseCollisionShapeIndex=box_curve1, 
                    basePosition=[-1.3, 0.0, 0.2])
obstacles.append((box_body_2, box_curve_extents))
box_body_2 = p.createMultiBody(
                    baseMass=1,
                    baseInertialFramePosition=[0, 0, 0],
                    baseCollisionShapeIndex=box_curve1, 
                    basePosition=[-1.4, 0.2, 0.2])
obstacles.append((box_body_2, box_curve_extents))

box2_extents = [1.5, 0.1, 0.2]
box2 = p.createCollisionShape(p.GEOM_BOX, halfExtents=box2_extents)
# Up right Line
box_body_1 = p.createMultiBody(
                    baseMass=1,
                    baseInertialFramePosition=[0, 0, 0],
                    baseCollisionShapeIndex=box2, 
                    basePosition=[3.0, 0.2, 0.2])
obstacles.append((box_body_1, box2_extents))

# Middle Left Line
box_body_1 = p.createMultiBody(
                    baseMass=1,
                    baseInertialFramePosition=[0, 0, 0],
                    baseCollisionShapeIndex=box2, 
                    basePosition=[-3.0, 0.2, 0.2])
obstacles.append((box_body_1, box2_extents))

box3_extents = [4.2, 0.1, 0.2]
box3 = p.createCollisionShape(p.GEOM_BOX, halfExtents=box3_extents)
box_body_1 = p.createMultiBody(
                    baseMass=1,
                    baseInertialFramePosition=[0, 0, 0],
                    baseCollisionShapeIndex=box3, 
                    basePosition=[0, 1.5, 0.2])
obstacles.append((box_body_1, box3_extents))

# Port all of the obstacles into a txt file with their coordinates
obstacle_extents_arr = []
for obstacle, extents in obstacles:
    position, orientation = p.getBasePositionAndOrientation(obstacle)
    obstacle_extents_arr.append((position, extents))
utils.create_obstacles_txt_file(obstacle_extents_arr, obstacles_fpath)

# TODO: execute planner here
# if os.system("g++ " + cpp_fpath) == 0:
#     print("Executed planner")
# else:
#     print('Could not execute planner ' + cpp_fpath)
#     #Exception('Could not execute planner ' + cpp_fpath)

# Read input from Planner:
target_states = utils.read_plan_from_file(planner_path_fpath)

# Run the simulation
while(True):
    for current_state in target_states:
        for i in range (p.getNumJoints(car)):
            jointInfo = p.getJointInfo(car, i)
            jointName = jointInfo[1]

            # Set steering angle
            if "bar_joint" in jointName.decode("utf-8"):
                p.setJointMotorControl2(car, i, p.POSITION_CONTROL,targetPosition=current_state.steering_angle)
            
            # Set velocity
            if "wheel_joint" in jointName.decode("utf-8"):
                p.setJointMotorControl2(car, i, p.VELOCITY_CONTROL, targetVelocity=current_state.velocity, force=100)

            # Don't think we can Set orientation
            #if "base_scan_joint" in jointName.decode("utf-8"):
                #if "steer_joint" in jointName.decode("utf-8"):
            #    p.setJointMotorControl2(car, i, p.POSITION_CONTROL,targetPosition=current_state.orientation)

            p.stepSimulation()
        #time.sleep(.5)

    # Disconnect from the simulation environment
    p.disconnect()