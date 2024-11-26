import matplotlib.pyplot as plt
import numpy as np

def draw_connected_arcs(states):
    """
    Parameters:
    - stats: List of tuples, where each tuple contains (x, y, theta, tau, r, u)
    """
    fig, ax = plt.subplots(figsize=(8, 8))

    # Initialize the starting point
    current_x, current_y = states[0][0], states[0][1]

    for state in states:
        x, y, theta, tau, r, u = state

        # Update starting point for this arc
        x, y = current_x, current_y

        # Calculate the direction orthogonal to theta
        orthogonal_theta = theta + (np.pi / 2 if u == 1 else -np.pi / 2)

        # Find the center of the circle
        cx = x + r * np.cos(orthogonal_theta)
        cy = y + r * np.sin(orthogonal_theta)

        # Calculate the starting angle and end angle
        start_angle = np.arctan2(y - cy, x - cx)
        end_angle = start_angle + tau

        # Generate points for the arc
        angles = np.linspace(start_angle, end_angle, 100)
        arc_x = cx + r * np.cos(angles)
        arc_y = cy + r * np.sin(angles)

        # Plot the arc
        ax.plot(arc_x, arc_y, label=f"Arc {state}", color="blue")
        
        # Update the current position to the endpoint of the arc
        current_x, current_y = arc_x[-1], arc_y[-1]

    # Mark the start and end points
    ax.scatter([states[0][0]], [states[0][1]], color="red", label="Start Point")
    ax.scatter([current_x], [current_y], color="green", label="End Point")

    # Set equal scaling and labels
    ax.set_aspect('equal', adjustable='box')
    ax.set_title("Connected Circular Arcs")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.grid(True)
    plt.show()
if __name__=="__main__":
  # Example usage
  states = [
      (1, 1, np.pi/4, np.pi/2, 2, 1),  # (x, y, theta, tau, r, u)
      (0, 0, np.pi/3, np.pi, 3, 0),
      (0, 0, -np.pi/4, np.pi/3, 2, 1)
  ]
  draw_connected_arcs(states)
