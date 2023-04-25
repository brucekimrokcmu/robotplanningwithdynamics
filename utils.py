import argparse

class State:
    def __init__(self, x, y, orientation, velocity,steering_angle):
        self.x = x
        self.y = y
        self.orientation = orientation
        self.velocity = velocity
        self.steering_angle = steering_angle
        

    def __str__(self):
        s = 'x: ' + str(round(self.x,2)) + "\ty: " + str(round(self.y,2))
        s += '\tv: ' + str(round(self.velocity,2))
        s += '\tpsi: ' + str(round(self.steering_angle,2))
        s += '\ttheta: ' + str(round(self.orientation,2))
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

def create_map_txt_file(obstacle_extents_arr, start, goal, fpath):
    # x, y, half_extent_x, half_extent_y
    with open(fpath, 'w') as outfile:
        # start position first
        outfile.write(str(round(start[0],3)) + ',' + str(round(start[1],3)) + '\n\n')
        # goal position next
        outfile.write(str(round(goal[0],3)) + ',' + str(round(goal[1],3)) + '\n\n')

        # all obstacles
        for i,(position,extents) in enumerate(obstacle_extents_arr):
            coor = list(position)[:2]
            coor.extend(extents[:2])
            out_str = str(round(coor[0],3)) + ',' +  str(round(coor[1],3))
            out_str += ',' +  str(round(extents[0],3)) + ',' +  str(round(extents[1],3))

            if (i < len(obstacle_extents_arr)-1):
                out_str += '\n'
            outfile.write(out_str)

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