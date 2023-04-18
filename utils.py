import pandas as pd

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
def read_plan_from_file(fname):
    plan_df = pd.read_csv(fname, header=None)
    plan_df.columns = ['x', 'y', 'theta', 'v', 'psi']

    target_states = []
    for index,row in plan_df.iterrows():
        s = State(
            row['x'],
            row['y'],
            row['v'],
            row['psi'],
            row['theta']
        )
        target_states.append(s)
    return target_states