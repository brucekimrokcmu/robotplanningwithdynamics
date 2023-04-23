import pandas as pd
import argparse

class State:
    def __init__(self, x, y, velocity, steering_angle, orientation):
        self.x = x
        self.y = y
        self.velocity = velocity
        self.steering_angle = steering_angle
        self.orientation = orientation

    def __str__(self):
        s = 'x: ' + str(round(self.x,2)) + "\ty: " + str(round(self.y))
        s += '\tv: ' + str(round(self.velocity))
        s += '\tpsi: ' + str(round(self.steering_angle))
        s += '\ttheta: ' + str(round(self.orientation))
        return s

# Read Plan from file into array of states
def read_plan_from_file(fpath):
    with open(fpath,'r') as input_file:
        data_str = input_file.read()
        data_array = data_str.split('\n\n') # Split on all instances of double new lines

        target_states = []

        # Output file contains blocks:
        # 1) Map ID, 2) Obstacle boundaries, 3) Region decomposition boundaries
        # 4) Robot states, 5) (x,y) sampled points
        states_rows = data_array[3]
        for row in states_rows.split('\n'):   
            state_values = row.split(',')
            current_state = State(
                float(state_values[0]),
                float(state_values[1]),
                float(state_values[2]),
                float(state_values[3]),
                float(state_values[4])
            )
            target_states.append(current_state)

        return target_states

def create_obstacles_txt_file(obstacle_extents_arr, fpath):
    # x, y, half_extent_x, half_extent_y
    obstacles_dict = []
    for position, extents in obstacle_extents_arr:
        coor = list(position)[:2]
        coor.extend(extents[:2])
        obstacles_dict.append(coor)

    df = pd.DataFrame(obstacles_dict)
    df.to_csv(fpath, sep=',', header=False, index=False)

# Arguments for planned path txt file and obstacles txt file
def get_file_paths():
    parser = argparse.ArgumentParser()
    parser.add_argument('-p', '--path', help='file path for the planned path')
    parser.add_argument('-o', '--obstacles', help='file path for obstacles file')
    parser.add_argument('-c', '--cpp', help='filepath for c++ planner.')

    args = parser.parse_args()
    planner_path_fpath = args.path if args.path else Exception(
        'Need file path for planned path.')
    obstacles_fpath = args.obstacles if args.obstacles else Exception(
        'Need file path for obstacles.')
    cpp_fpath = args.cpp if args.obstacles else Exception(
        'Need file path for cpp planner.')
    return planner_path_fpath, obstacles_fpath, cpp_fpath