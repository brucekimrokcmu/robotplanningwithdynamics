import pybullet as p
import time
import pybullet_data
import utils
import random
import os

# Example usage: 
# python simple-map.py -p output.txt -o obstacles.txt -c compile.sh

NUM_OBSTACLES = 7
MAP_SIZE = 15
BOUNDARY_HEIGHT = 0.5
HEIGHT = 0.1

planner_path_fpath, obstacles_fpath, cpp_fpath = utils.get_file_paths()

physicsClient = p.connect(p.GUI) #or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)



planeId = p.loadURDF("plane.urdf")
# p.changeDynamics(planeId, -1, rollingFriction=0.01)

# Create a box object
startOrientation = p.getQuaternionFromEuler([0,0,0])

box_curve_extents = [0.2, 0.1, HEIGHT]
box_curve1 = p.createCollisionShape(p.GEOM_BOX, halfExtents=box_curve_extents)
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
seg1_extents = [2.5,0.05,BOUNDARY_HEIGHT/6]
seg1 = p.createCollisionShape(p.GEOM_BOX, halfExtents=seg1_extents)
p.createMultiBody(
                baseMass=0,
                baseInertialFramePosition=[0, 0, 0],
                baseCollisionShapeIndex=seg1, 
                basePosition=[2.5, 4, 0])
obstacle_configs.append(([2.5, 4, 0], seg1_extents))
p.createMultiBody(
                baseMass=0,
                baseInertialFramePosition=[0, 0, 0],
                baseCollisionShapeIndex=seg1, 
                basePosition=[2.5, 6, 0])
obstacle_configs.append(([2.5, 6, 0], seg1_extents))

p.createMultiBody(
                baseMass=0,
                baseInertialFramePosition=[0, 0, 0],
                baseCollisionShapeIndex=seg1, 
                basePosition=[7.5, 3, 0])
obstacle_configs.append(([7.5, 3, 0], seg1_extents))
p.createMultiBody(
                baseMass=0,
                baseInertialFramePosition=[0, 0, 0],
                baseCollisionShapeIndex=seg1, 
                basePosition=[7.5, 7, 0])
obstacle_configs.append(([7.5, 7, 0], seg1_extents))

# Drop car into environment
startPos = [0.5,5,0.16]
startOriention = p.getQuaternionFromEuler([0,0,0])
car = p.loadURDF("car.urdf",startPos, startOrientation)

car_mass =  10.0
p.changeDynamics(car, -1, mass=car_mass)


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

run_time = 60.0
num_steps = len(target_states)
# print(num_steps)
time_step = run_time/num_steps

for i, current_state in enumerate(target_states):
    
    t = i*time_step
    
    if i < num_steps-1:
        next_state = target_states[i+1]
        t_norm = t / time_step
        current_state = current_state.interpolate(next_state, t_norm)

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
    time.sleep(time_step)
time.sleep(300)
    #p.stepSimulation()
    # Disconnect from the simulation environment
    #p.disconnect()
