import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
def draw_connected_arcs(states, obstacles=None):
    """
    Draws connected arcs for given states and visualizes them.
    
    Parameters:
        states (list): List of states, where each state is (x, y, theta, tau, r, u).
        obstacles (list): Placeholder for obstacles (not used in this version).
    """
    fig, ax = plt.subplots(figsize=(8, 8))

    # Initialize the starting point
    current_x, current_y, current_theta = states[0][0], states[0][1], states[0][2]

    for state in states:
        x, y, theta, tau, r, u = state
        print(f"State: x={x}, y={y}, theta={theta}, tau={tau}, r={r}, u={u}")
        ax.scatter(current_x, current_y, color="black")  # Plot the current state's position

        # Skip invalid arcs
        if u == -1.0:
            continue
        sign = 1 if u == 0 else -1  # Determine direction based on u

        # Calculate the new orientation
        theta_new = theta + sign * tau / r

        # Calculate the new position
        x_new = x - sign * r * np.sin(theta) + sign * r * np.sin(theta_new)
        y_new = y + sign * r * np.cos(theta) - sign * r * np.cos(theta_new)

# Generate points for visualization
        num_points = 100
        t = np.linspace(0, tau, num_points)
        angles = current_theta + (t / r) * (1 if u == 0 else -1)
        arc_x = current_x - sign *r * np.sin(current_theta) + sign *r * np.sin(angles)
        arc_y = current_y + sign *r * np.cos(current_theta) - sign *r * np.cos(angles)

        # Plot the arc
        ax.plot(arc_x, arc_y, color="blue")

        # Update current state
        current_x, current_y, current_theta = x_new, y_new, theta_new

    # Mark the start and end points
    ax.scatter([states[0][0]], [states[0][1]], color="red", label="Start Point")
    ax.scatter([states[-1][0]], [states[-1][1]], color="purple", label="End Point")
    ax.scatter(8, 8, color="green", label="Goal Point")
    # Add obstacles to the plot
    for obs in obstacles:
        rect = patches.Rectangle(
            (obs["x"], obs["y"]),  # lower-left corner
            obs["width"],  # width
            obs["height"],  # height
            linewidth=2,
            edgecolor="black",
            facecolor="gray",  # obstacle color
            alpha=0.5  # transparency
        )
        ax.add_patch(rect)
    # Set equal scaling and labels
    ax.set_aspect('equal', adjustable='box')
    ax.set_title("Connected Circular Arcs")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.grid(True)
    plt.show()
def read_file_to_list(filename):
    data = []
    for i in range(1000):
      data.append([])
    with open(filename, 'r') as file:
        index=-1
        for line in file:
            if line[0]=='=':
              index+=1
            # Split the line by commas and convert to float if possible
            else:
              values = [float(x) if x.replace('.', '', 1).replace('-', '', 1).isdigit() else x for x in line.strip().split(',')]
              data[index].append(values)
    return data

if __name__=="__main__":
  # Example usage
  file_path = "path.txt"  # Replace with your file path
  state_list = read_file_to_list(file_path)
  obstacles = [
    {"x": 6, "y": 2, "width": 1, "height": 2},
     {"x": 6, "y": 4.5, "width": 1, "height": 2},
    {"x": 2.5, "y": 4, "width": 1, "height": 2}
 #   {"x": -.5, "y": -.35, "width": .4, "height": .7},
 #   {"x": .2, "y": -.35, "width": .4, "height": .7}
  ]
  goal=[8,8]
  path_distances = [
        ((path[-1][0] - goal[0])**2 + (path[-1][1] - goal[1])**2)**0.5
        if len(path) > 1 else float('inf')  # Avoid issues with single-point paths
        for path in state_list
  ]

  # Sort paths by distance to the goal in ascending order and select the nearest `n`
  nearest_goal_paths = [
        path for _, path in sorted(zip(path_distances, state_list))[:]
  ]

  draw_connected_arcs(nearest_goal_paths[999],obstacles)
