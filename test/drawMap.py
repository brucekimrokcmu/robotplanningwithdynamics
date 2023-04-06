import matplotlib.pyplot as plt

# Open the text file
with open('../output.txt', 'r') as f:
    name = f.readline()
    obs_num = f.readline()
    obstacles = []
    for i in obs_num:
        line = f.readline()
        x, x_extent, y, y_extent = map(float, line.strip().split(',')[:-1])
        obstacles.append((x, x_extent, y, y_extent))
    # Read the lines and parse the rectangle coordinates
    rectangles = []
    for line in f.readlines():
        if line.strip():
            x, x_extent, y, y_extent = map(float, line.strip().split(',')[:-1])
            rectangles.append((x, x_extent, y, y_extent))

# Create a figure and axis object
fig, ax = plt.subplots()

for obs in obstacles:
    x, x_extent, y, y_extent = obs
    ax.add_patch(plt.Rectangle((x, y), x_extent, y_extent, alpha=0.3))

# Draw each rectangle on the axis
for rect in rectangles:
    x, x_extent, y, y_extent = rect
    ax.add_patch(plt.Rectangle((x, y), x_extent, y_extent, alpha=0.5, fill=False))

# Set the axis limits and labels
ax.set_xlim(0, 10)
ax.set_ylim(0, 10)
ax.set_xlabel('X-axis')
ax.set_ylabel('Y-axis')

# Show the plot
plt.show()