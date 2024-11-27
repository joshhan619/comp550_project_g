## Running ProjectG
1. Run `make`
2. Once compiled, the usage for the ProjectG executable is `./ProjectG n m b --verbose`
    - n: sample size of SMR
    - m: number of sample transitions per state
    - b: binary variable where:
        - b=0 simulates one path and outputs expected probability of each state in the roadmap to values.txt. Also outputs the simulated path to path.txt.
        - b=1 simulates 1000 paths and computes the actual probability of success for the start state. Used to experiment with the sample size.
3. The `experiment.sh` script can be executed to get data on the difference between the actual and expected probabilities of success on different sample sizes. Output is saved in several output*.txt files. The python script `plot_experiment_2.py` will make a graph displaying the results in the output files.
4. `plot_states.py` will take the path saved to path.txt and create a visualization of the path.
5. `visualize_value_function.py` takes the values saved to values.txt and graph a heatmap of the averaged values.
