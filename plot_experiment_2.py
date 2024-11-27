import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import make_interp_spline

if __name__ == '__main__':
    means = np.zeros(4)
    sds = np.zeros(4)
    sample_sizes = [1000, 10000, 100000, 1000000]
    for index, n in enumerate(sample_sizes):
        data = []
        with open(f"output{n}.txt") as f:
            lines = f.readlines()
            for line in lines:
                data.append(float(line))
        data = np.array(data)
        means[index] = np.mean(data)
        sds[index] = np.std(data)

    log_sample_sizes = np.log10(sample_sizes)
    spline = make_interp_spline(log_sample_sizes, means)

    x = np.linspace(3, 6, 500)
    y = spline(x)

    fig, ax = plt.subplots()
    ax.errorbar(log_sample_sizes, means, sds, fmt='o')
    ax.plot(x, y)
    ax.set_ylabel("Mean Difference in Probability")
    ax.set_xlabel("Log(Sample Size)")
    ax.set_title("Mean difference: Actual vs Expected Probability")
    plt.show()

    
    