import seaborn as sns
import numpy as np
import matplotlib.pyplot as plt

def visualize_value_function(states, values):
    # Assume the environment has lower bound -0.5 and upper bound 0.5 in 2D
    # We will divide the grid into cells of width 0.1
    value_grid = np.zeros((10, 10))
    value_grid_count = np.zeros((10, 10))
    
    for state, value in zip(states, values):
        x, y = state
        cellx = int(x//0.1) + 5
        celly = int(y//0.1) + 5
        if x == 0.5:
            cellx = 9
        if y == 0.5:
            celly = 9
        value_grid[cellx, celly] += value
        value_grid_count[cellx, celly] += 1

    value_grid /= value_grid_count
    ticklabels = ["{:.1f}".format(0.1*x-0.5) for x in range(10)]

    ax = sns.heatmap(value_grid, cmap="plasma", xticklabels=ticklabels, yticklabels=ticklabels)
    ax.invert_yaxis()
    plt.title("Value Function")
    plt.show()

if __name__ == '__main__':
    with open("values.txt") as f:
        states = []
        values = []
        lines = f.readlines()
        for line in lines:
            [x, y, prob] = line.split(",")
            states.append((float(x), float(y)))
            values.append(float(prob))

        visualize_value_function(states, values)