import matplotlib.pyplot as plt
import numpy as np

if __name__ == '__main__':
    means = np.zeros(3)
    sds = np.zeros(3)
    sample_sizes = [1000, 10000, 100000]
    for index, n in enumerate(sample_sizes):
        data = []
        with open(f"output{n}.txt") as f:
            lines = f.readlines()
            for line in lines:
                data.append(float(line))
        data = np.array(data)
        means[index] = np.mean(data)
        sds[index] = np.sqrt(np.sum((data - means[index]))**2/20)


    print(means, sds)
    fig, ax = plt.subplots()
    ax.errorbar(np.log10(sample_sizes), means, sds, linestyle='None', fmt='o')
    ax.set_ylabel("Mean difference: Actual vs Expected Probability")
    ax.set_xlabel("Log(Sample Size)")
    ax.set_title("Experiment 2")
    plt.show()

    
    