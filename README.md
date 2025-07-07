
# COMP550 Project G: Planning Under Uncertainty

**Joshua Han (jh273@rice.edu) & Alan Huang (ah212@rice.du)**

> For a brief intro plz look at [slide](../comp550_project_g/Document/COMP550%20Project%20G%20-%20Joshua,%20Alan.pdf), for detail result see [document](../comp550_project_g/Document/COMP550%20Project%20G%20Report.pdf)
---
## I: Project Statement

In real-world robotics, **actuation error** — the inevitable variation in a robot's response to commands — poses a significant challenge, especially when avoiding collisions is critical. Ignoring this uncertainty can lead to dangerous situations. Our project tackles this by implementing and analyzing the **Stochastic Motion Roadmap (SMR)**, a robust robot motion planner designed to account for these small movement errors.

---

## II: Our Approach

Our project implements the **Stochastic Motion Roadmap (SMR)**, inspired by Alterovitz et al. (2007). Like PRM, SMR builds a roadmap by sampling points as vertices. However, it explicitly models uncertainty:

* **Modeling Uncertainty:** We estimate state transition probabilities between vertices by sampling random motions for each control, assuming a **Gaussian noise distribution** for the robot's movement. These probabilities, $P(t | s, u)$, represent the chance of moving from state $s$ to state $t$ with control $u$.
* **Optimal Policy through MDP:** Unlike deterministic planners, SMR seeks an **optimal policy** $\pi: Q \rightarrow U$, which dictates the best action ($u$) for each state ($s$). This is framed as a Markov Decision Process (MDP), where states are roadmap vertices and controls are discrete actions.
* **Value Iteration:** A reward function assigns 1 to goal states and 0 otherwise. We use **value iteration** to compute the **value function** $ps(i)$, representing the expected probability of success for state $i$ to reach the goal. The optimal policy is derived by selecting actions that maximize this success probability.

We specifically model a **steerable needle robot**, behaving like a Dubins bang-bang car with Gaussian noise. It has two controls: forward-left ($u=0$) and forward-right ($u=1$). We iteratively execute the optimal action to navigate the robot. Our experiments evaluate SMR's goal-reaching success and how it improves with increased sampled vertices.

---

## III: Experiment Description

We conducted three key experiments using the Stochastic Motion Roadmap:

For a comprehensive description of our experimental setup, detailed results, and a deeper analysis, please refer to our [full project report](../comp550_project_g/Document/COMP550%20Project%20G%20Report.pdf).

### 1. Planning in Diverse Environments

We generated SMRs (with $n=50,000$ samples and $m=20$ transitions) in various environments. By simulating multiple paths, we observed how SMR adapts and finds solutions in different spatial configurations.

### 2. Impact of Motion Variance (Sigma)

We investigated how changes in the variance of the robot's random motion (Gaussian noise distributions) affect the SMR's behavior. This included scenarios with low, high, and unbalanced variance between the left and right steering controls, assessing their effect on actual and expected success probabilities.

### 3. Sample Size vs. Accuracy

We examined how the **sample size** ($n$) influences the difference between the actual and expected probabilities of success. For four sample sizes ($10^3, 10^4, 10^5, 10^6$), we generated SMRs, simulated 1000 paths, and calculated the mean and standard deviation of this difference over 20 runs.

---

## IV: Analysis

Our experiments revealed key characteristics of the Stochastic Motion Roadmap's behavior under uncertainty:

### Different Environments

The SMR consistently demonstrated its ability to find diverse and often creative solutions in varied environments, leveraging its inherent randomness. It adapted to challenges like navigating around obstacles, passing through narrow passages, and finding wide circular paths in constrained spaces, showcasing its flexibility when planning under uncertainty.

### Sigma (Uncertainty Levels)

Adjusting the standard deviation of robot motion revealed a clear trade-off:
* **High uncertainty** often led to faster task completion and uncovered novel, less conventional paths.
* **Low uncertainty** resulted in higher iteration costs but produced more optimal and controlled paths with a higher likelihood of success.
* **Unbalanced uncertainty** created unique path behaviors, demonstrating how targeted uncertainty can influence exploration and solution diversity.

These findings highlight the importance of carefully tuning uncertainty levels to balance efficiency, solution diversity, and path quality.

### Sample Size

Our analysis showed that increasing the sample size improves the SMR's approximation of the underlying state-action space, with diminishing returns. While larger samples reduced the variability in results, we consistently observed that the actual probability of success for the start state was lower than the expected probability calculated from value iteration, plateauing around -0.14. This suggests potential areas for refinement in our path simulation methodology.

---

## Running ProjectG

To run and experiment with ProjectG, follow these steps:

1.  **Compile:**
    Run `make` in the project root directory.

2.  **Execute ProjectG:**
    Once compiled, the executable `ProjectG` can be run with the following command-line arguments:
    `./ProjectG n m b --verbose`
    * **`n`**: Sample size of the SMR (number of vertices in the roadmap).
    * **`m`**: Number of sample transitions per state.
    * **`b`**: Binary variable to control simulation mode:
        * `b=0`: Simulates a single path. Outputs the expected probability of each state in the roadmap to `values.txt` and the simulated path to `path.txt`.
        * `b=1`: Simulates 1000 paths and computes the actual probability of success for the start state. This mode is used for experimenting with sample size.

3.  **Experiment with Sample Size:**
    Execute the `experiment.sh` script to gather data on the difference between actual and expected probabilities of success for various sample sizes. The output will be saved in `output*.txt` files.
    The Python script `plot_experiment_2.py` can then be used to generate a graph visualizing these results.

4.  **Visualize Paths:**
    The `plot_states.py` script takes the path saved in `path.txt` and creates a visual representation of the simulated path.

5.  **Visualize Value Function:**
    The `visualize_value_function.py` script reads the values saved in `values.txt` and generates a heatmap representing the averaged value function across the state space.

---