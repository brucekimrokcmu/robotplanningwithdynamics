import matplotlib.pyplot as plt

# Open the text file
with open('../output.txt', 'r') as f:
    lines = f.readlines()[1:]
    # Read the lines and parse the rectangle coordinates
    rectangles = []
    for line in lines:
        if line.strip():
            x, x_extent, y, y_extent = map(float, line.strip().split(',')[:-1])
            rectangles.append((x, x_extent, y, y_extent))

# Create a figure and axis object
fig, ax = plt.subplots()

# Draw each rectangle on the axis
for rect in rectangles:
    x, x_extent, y, y_extent = rect
    ax.add_patch(plt.Rectangle((x, y), x_extent, y_extent, alpha=0.5))

# Set the axis limits and labels
ax.set_xlim(0, 100)
ax.set_ylim(0, 100)
ax.set_xlabel('X-axis')
ax.set_ylabel('Y-axis')

# Show the plot
plt.show()