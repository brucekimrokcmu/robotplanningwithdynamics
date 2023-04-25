import pybullet as p
import time
import pybullet_data
import utils
import random
import os

# Example usage: 
# python random-map.py -p output.txt -o obstacles.txt -c compile.sh

NUM_OBSTACLES = 2
MAP_SIZE = 6
BOUNDARY_HEIGHT = 0.5
HEIGHT = 0.1

planner_path_fpath, obstacles_fpath, cpp_fpath = utils.get_file_paths()

physicsClient = p.connect(p.GUI) #or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)

planeId = p.loadURDF("plane.urdf")

obstacles = []

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
                    basePosition=[MAP_SIZE/2, 0, 0])

top_b = p.createCollisionShape(p.GEOM_BOX, halfExtents=[MAP_SIZE/2,0,BOUNDARY_HEIGHT/6])
top_boundary = p.createMultiBody(
                    baseMass=0,
                    baseInertialFramePosition=[0, 0, 0],
                    baseCollisionShapeIndex=top_b, 
                    basePosition=[MAP_SIZE/2, MAP_SIZE, 0])

left_b = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0,MAP_SIZE/2,BOUNDARY_HEIGHT/6])
left_boundary = p.createMultiBody(
                    baseMass=0,
                    baseInertialFramePosition=[0, 0, 0],
                    baseCollisionShapeIndex=left_b, 
                    basePosition=[0, MAP_SIZE/2, 0])

right_b = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0,MAP_SIZE/2,BOUNDARY_HEIGHT/6])
right_boundary = p.createMultiBody(
                    baseMass=0,
                    baseInertialFramePosition=[0, 0, 0],
                    baseCollisionShapeIndex=right_b, 
                    basePosition=[MAP_SIZE, MAP_SIZE/2, 0])


# Use box3 as the collision shape
obstacles_coordinates = []
num_iter = 1
while len(obstacles_coordinates) < NUM_OBSTACLES:
    rand_x = random.uniform(0.5,MAP_SIZE-0.5)
    rand_y = random.uniform(0.5,MAP_SIZE-0.5)

    # Skip if we have already generated this point
    if [rand_x,rand_y,HEIGHT] in obstacles_coordinates:
        continue

    # Check if it collides with any of the existing obstacles
    collides = False
    x_ext = box_curve_extents[0]
    y_ext = box_curve_extents[1]
    
    # (x_min, y_max) is top left coordinate
    # (x_max, y_min) is bottom right coordinate
    x_min = rand_x-x_ext
    x_max = rand_x+x_ext
    y_min = rand_y-y_ext
    y_max = rand_y+y_ext

    for x,y,HEIGHT in obstacles_coordinates:
        obst_x_min = x - MAP_SIZE/2
        obst_x_max = x + MAP_SIZE/2
        obst_y_min = y - MAP_SIZE/2
        obst_y_max = y + MAP_SIZE/2
        
        # If one rectangle is on left side of other
        if x_min > obst_x_max or obst_x_min > x_max:
            collides = True
            break
    
        # If one rectangle is above other
        if y_max > obst_y_max or obst_y_min > y_max:
            collides = True
            break

        if collides:
            continue

    obstacles_coordinates.append([rand_x,rand_y,HEIGHT])
    
#print(obstacles_coordinates)
obstacle_configs = [(coor, box_curve1, box_curve_extents) for coor in obstacles_coordinates]

for basePosition, baseCollisionShapeIndex, extents in obstacle_configs:
    box = p.createMultiBody(
                baseMass=0,
                baseInertialFramePosition=[0, 0, 0],
                baseCollisionShapeIndex=baseCollisionShapeIndex, 
                basePosition=basePosition)
    obstacles.append((box, extents))

# Randomly generate start and goal positions

def collidesWithObstacle(x,y,obstacles):
    x_ext = box_curve_extents[0]
    y_ext = box_curve_extents[1]

    for obstx,obsty,_ in obstacles:
        x_ext = box_curve_extents[0]+0.1 # buffer
        y_ext = box_curve_extents[1]+0.1

        x_min = obstx-x_ext
        x_max = obstx+x_ext
        y_min = obsty-y_ext
        y_max = obsty+y_ext

        # postion overlaps with obstacle
        if x >= x_min and x <= x_max and y >= y_min and y <= y_max:
            return True
    return False


# Randomly generate start state
start_x = 0.5 #random.uniform(0.5,MAP_SIZE-0.5)
start_y = 0.5 #random.uniform(0.5,MAP_SIZE-0.5)

while collidesWithObstacle(start_x, start_y, obstacles_coordinates):
    start_x = random.uniform(0.5,MAP_SIZE-0.5)
    start_y = random.uniform(0.5,MAP_SIZE-0.5)

# Drop car into environment
startPos = [start_x,start_y,0.16]
startOriention = p.getQuaternionFromEuler([0,0,0])
car = p.loadURDF("car.urdf",startPos, startOrientation)

# Randomly generate goal state
goal_x = 5#random.uniform(0.2,MAP_SIZE-0.2)
goal_y = 5 #random.uniform(0.2,MAP_SIZE-0.2)

while collidesWithObstacle(goal_x, goal_y, obstacles_coordinates) or (goal_x==start_x and goal_y==start_y):
    goal_x = random.uniform(0.2,MAP_SIZE-0.2)
    goal_y = random.uniform(0.2,MAP_SIZE-0.2)

# box_goal = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.01, 0.01, 1])
# box_body_goal = p.createMultiBody(
#                     baseMass=1,
#                     baseInertialFramePosition=[0, 0, 0],
#                     baseVisualShapeIndex=box_goal, 
#                     basePosition=[goal_x, goal_y, 0])

# Port all of the obstacles into a txt file with their coordinates
obstacle_extents_arr = []
for obstacle, extents in obstacles:
    position, orientation = p.getBasePositionAndOrientation(obstacle)
    obstacle_extents_arr.append((position, extents))
utils.create_map_txt_file(
    obstacle_extents_arr,
    (start_x, start_y),
    (goal_x, goal_y),
    obstacles_fpath)

print("Added all obstacles to map")

# TODO: execute planner here
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

p.setTimeStep(1)
time.sleep(5)
# Run the simulation
start = time.time()
while(True):
    now = time.time()
    duration = now - start
    idx = round(duration)
    # for current_state in target_states:
    if(idx >= len(target_states)):
        time.sleep(60)
    current_state = target_states[idx]
    
    # print(idx)
    for i in range (p.getNumJoints(car)):
        jointInfo = p.getJointInfo(car, i)
        jointName = jointInfo[1]

        # Set steering angle
        if "bar_joint" in jointName.decode("utf-8"):
            p.setJointMotorControl2(car, i, p.POSITION_CONTROL,targetPosition=current_state.steering_angle)
        
        # Set velocity
        if "wheel_joint" in jointName.decode("utf-8"):
            velocity = 6.28*(current_state.velocity / (0.12*3.14)) 
            p.setJointMotorControl2(car, i, p.VELOCITY_CONTROL, targetVelocity=velocity, force=100)

        # Don't think we can Set orientation
        #if "base_scan_joint" in jointName.decode("utf-8"):
            #if "steer_joint" in jointName.decode("utf-8"):
        #    p.setJointMotorControl2(car, i, p.POSITION_CONTROL,targetPosition=current_state.orientation)

    p.stepSimulation()
        # time.sleep(0.01)
        #time.sleep(.5)
    # time.sleep(60)
    #p.stepSimulation()
    # Disconnect from the simulation environment
    #p.disconnect()
