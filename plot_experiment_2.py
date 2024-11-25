import matplotlib.pyplot as plt
import math

def openFile(filepath):
    with open(filepath) as f:
        lines = f.readlines()
        mean = 0
        sd = 0
        for line in lines:
            mean += float(line)

        mean /= len(lines)
        for line in lines:
            sd += (float(line) - mean)**2
        
        sd = math.sqrt(sd/len(lines))
        return mean, sd

if __name__ == '__main__':
    means = []
    sds = []
    sample_sizes = [1000, 10000, 100000, 1000000]
    for n in sample_sizes:
        for i in range(20):
            mean, sd = openFile(f"experiments/output{n}.txt")
            mean.append(mean)
            sds.append(sd)

    fig, ax = plt.subplots()
    ax.errorbar(sample_sizes, means, sds, linestyle='None', fmt='o')
    ax.set_ylabel("Mean difference: Actual vs Expected Probability")
    ax.set_xlabel("Log(Sample Size)")
    ax.set_title("Experiment 2")
    plt.show()

    
    