import matplotlib.pyplot as plt
import math

if __name__ == '__main__':
    with open('output.txt') as f:
        lines = f.readlines()
        means = []
        sds = []
        sample_sizes = []
        for line in lines:
            [n, mean, sd] = line.rstrip().split(",")
            means.append(float(mean))
            sds.append(float(sd))
            sample_sizes.append(math.log10(int(n)))

    fig, ax = plt.subplots()
    print(sample_sizes, means, sds)
    ax.errorbar(sample_sizes, means, sds, linestyle='None', fmt='o')
    ax.set_ylabel("Mean difference: Actual vs Expected Probability")
    ax.set_xlabel("Log(Sample Size)")
    ax.set_title("Experiment 2")
    plt.show()

    
    