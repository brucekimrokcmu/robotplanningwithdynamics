import pybullet as p
import time
import pybullet_data
import utils
import random
import os

# Example usage: 
# python scene5-map.py -p output.txt -o obstacles.txt -c compile.sh

NUM_OBSTACLES = 7
MAP_SIZE = 10
BOUNDARY_HEIGHT = 0.5
HEIGHT = 0.1

planner_path_fpath, obstacles_fpath, cpp_fpath = utils.get_file_paths()

physicsClient = p.connect(p.GUI) #or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)

planeId = p.loadURDF("plane.urdf")


# Create a box object
startOrientation = p.getQuaternionFromEuler([0,0,0])

# (basePosition, baseCollisionShapeIndex, extents)
# box_extents = [0.1, 1.5, 0.2]
# box1 = p.createCollisionShape(p.GEOM_BOX, halfExtents=box_extents)
box_curve_extents = [0.2, 0.1, HEIGHT]
box_curve1 = p.createCollisionShape(p.GEOM_BOX, halfExtents=box_curve_extents)
# box2_extents = [1.5, 0.1, 0.2]
# box2 = p.createCollisionShape(p.GEOM_BOX, halfExtents=box2_extents)
# box3_extents = [4.2, 0.1, 0.2]
# box3 = p.createCollisionShape(p.GEOM_BOX, halfExtents=box3_extents)

# Draw map boundary

# b = p.createCollisionShape(p.GEOM_BOX, halfExtents=[MAP_SIZE/2,MAP_SIZE/2, BOUNDARY_HEIGHT/2])
# boundary = p.createMultiBody(
#                     baseMass=1,
#                     baseInertialFramePosition=[0, 0, 0],
#                     baseCollisionShapeIndex=b, 
#                     basePosition=[MAP_SIZE/2, MAP_SIZE/2, 0])


bottom_b = p.createCollisionShape(p.GEOM_BOX, halfExtents=[MAP_SIZE/2,0,BOUNDARY_HEIGHT/6])
bottom_boundary = p.createMultiBody(
                    baseMass=0,
                    baseInertialFramePosition=[0, 0, 0],
                    baseCollisionShapeIndex=bottom_b, 
                    basePosition=[MAP_SIZE/2, -0.1, 0])

top_b = p.createCollisionShape(p.GEOM_BOX, halfExtents=[MAP_SIZE/2,0,BOUNDARY_HEIGHT/6])
top_boundary = p.createMultiBody(
                    baseMass=0,
                    baseInertialFramePosition=[0, 0, 0],
                    baseCollisionShapeIndex=top_b, 
                    basePosition=[MAP_SIZE/2, MAP_SIZE+0.1, 0])

left_b = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0,MAP_SIZE/2,BOUNDARY_HEIGHT/6])
left_boundary = p.createMultiBody(
                    baseMass=0,
                    baseInertialFramePosition=[0, 0, 0],
                    baseCollisionShapeIndex=left_b, 
                    basePosition=[-0.1, MAP_SIZE/2, 0])

right_b = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0,MAP_SIZE/2,BOUNDARY_HEIGHT/6])
right_boundary = p.createMultiBody(
                    baseMass=0,
                    baseInertialFramePosition=[0, 0, 0],
                    baseCollisionShapeIndex=right_b, 
                    basePosition=[MAP_SIZE+0.1, MAP_SIZE/2, 0])

obstacle_configs = []

# Draw curves
seg1_extents = [1.5,0.05,BOUNDARY_HEIGHT/6]
seg1 = p.createCollisionShape(p.GEOM_BOX, halfExtents=seg1_extents)
p.createMultiBody(
                baseMass=0,
                baseInertialFramePosition=[0, 0, 0],
                baseCollisionShapeIndex=seg1, 
                basePosition=[1.5, 4, 0])
obstacle_configs.append(([1.5, 4, 0], seg1_extents))
p.createMultiBody(
                baseMass=0,
                baseInertialFramePosition=[0, 0, 0],
                baseCollisionShapeIndex=seg1, 
                basePosition=[1.5, 6, 0])
obstacle_configs.append(([1.5, 6, 0], seg1_extents))

seg2_extents = [0.25,0.05,BOUNDARY_HEIGHT/6]
seg2 = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.25,0.05,BOUNDARY_HEIGHT/6])
x = 3.25
y_bottom = 4.1
y_top = 6.1
while (x < 6):
    p.createMultiBody(
                    baseMass=0,
                    baseInertialFramePosition=[0, 0, 0],
                    baseCollisionShapeIndex=seg2, 
                    basePosition=[x, y_bottom, 0])
    p.createMultiBody(
                    baseMass=0,
                    baseInertialFramePosition=[0, 0, 0],
                    baseCollisionShapeIndex=seg2, 
                    basePosition=[x, y_top, 0])

    obstacle_configs.append(([x, y_bottom, 0], seg2_extents))
    obstacle_configs.append(([x, y_top, 0], seg2_extents))

    x += 0.5
    y_bottom += 0.2

print('x,y_bottom: ', x,y_bottom)

# last two curves
p.createMultiBody(
                baseMass=0,
                baseInertialFramePosition=[0, 0, 0],
                baseCollisionShapeIndex=seg1, 
                basePosition=[7.75, 5, 0])
obstacle_configs.append(([7.75, 5, 0], seg1_extents))
p.createMultiBody(
                baseMass=0,
                baseInertialFramePosition=[0, 0, 0],
                baseCollisionShapeIndex=seg1, 
                basePosition=[7.75, 6.3, 0])
obstacle_configs.append(([7.75, 6.3, 0], seg1_extents))


# Drop car into environment
startPos = [0.5,5,0.16]
startOriention = p.getQuaternionFromEuler([0,0,0])
car = p.loadURDF("car.urdf",startPos, startOrientation)

goal_x = 9.5
goal_y = 5

box_goal = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.5, 0.5, 0.1])
box_body_goal = p.createMultiBody(
                    baseMass=1,
                    baseInertialFramePosition=[0, 0, 0],
                    baseVisualShapeIndex=box_goal, 
                    basePosition=[goal_x, goal_y, 0])

# Port all of the obstacles into a txt file with their coordinates
utils.create_map_txt_file(
    obstacle_configs,
    (startPos[0], startPos[1]),
    (goal_x, goal_y),
    obstacles_fpath)

print("Added all obstacles to map")

# execute planner here
if os.system("./" + cpp_fpath) == 0:
    print("Complied planner")
else:
    Exception('Could not compile planner ' + cpp_fpath)

if os.system("./GUSTOut.out Map1 " + planner_path_fpath) == 0:
    print("Executed planner")
else:
    #print('Could not execute planner ' + cpp_fpath)
    Exception('Could not execute planner ' + planner_path_fpath)

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
                p.setJointMotorControl2(car, i, p.VELOCITY_CONTROL, targetVelocity=current_state.velocity, force=10)

            p.stepSimulation()
        #time.sleep(1./240)
    time.sleep(30)
    #p.stepSimulation()
    # Disconnect from the simulation environment
    #p.disconnect()
