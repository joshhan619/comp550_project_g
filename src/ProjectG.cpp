#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/util/RandomNumbers.h>
#include <vector>
#include <unordered_map>
#include <set>
#include <utility>
#include <iostream>
#include <math.h>
#include <tuple>
#include <fstream>

namespace ob = ompl::base;

using Transition = std::pair<const ob::State *, double>;

struct Rectangle
{
    // Coordinate of the lower left corner of the rectangle
    double x, y;
    // The width (x-axis extent) of the rectangle
    double width;
    // The height (y-axis extent) of the rectangle
    double height;
};
std::vector<Rectangle> obstacles;
struct Edge {
    ob::State *source;
    ob::State *target;
    double prob;
};

struct Graph {
    std::vector<ob::State *> vertices;
    std::vector<Edge> edges;
};

bool isCollisionFree(const ob::State *state) {
    double x = state->as<ob::SE2StateSpace::StateType>()->getX();
    double y = state->as<ob::SE2StateSpace::StateType>()->getY();

    // Check for going out of bounds
    if (x < -0.5 || x > 0.5 || y < -0.5 || y > 0.5) {
        return false;
    }

    // Check for obstacle collisions
    for(size_t i = 0; i < obstacles.size(); ++i)
    {
        if (x >= obstacles[i].x && x <= obstacles[i].x + obstacles[i].width)
            if (y >= obstacles[i].y && y <= obstacles[i].y + obstacles[i].height)
                return false;
    }
    return true;
}

void propagateCircularArc(const ob::State *s, const int u, const double r, const double delta, ob::State *q) {
    auto sState = s->as<ob::SE2StateSpace::StateType>();
    double theta = sState->getYaw();
    int sign = (u == 0) ? 1 : -1;
    double theta_new = theta + sign * delta / r;
    double x_new = sState->getX() - sign * r * std::sin(theta) + sign * r * std::sin(theta_new); 
    double y_new = sState->getY() + sign * r * std::cos(theta) - sign * r * std::cos(theta_new);

    q->as<ob::SE2StateSpace::StateType>()->setX(x_new);
    q->as<ob::SE2StateSpace::StateType>()->setY(y_new);
    q->as<ob::SE2StateSpace::StateType>()->setYaw(theta_new);
}

bool isCollisionFreePath(const ob::State *s1, const ob::State *s2, const int u, const double r, const double delta, ob::SpaceInformationPtr si)
{
    const double stepSize = si->getStateSpace()->getMaximumExtent() / 100.0;
    int numSteps = static_cast<int>(si->distance(s1, s2) / stepSize);

    ob::State *q = si->allocState();
    for (int i = 1; i <= numSteps; i++) {
        propagateCircularArc(s1, u, r, i*delta/numSteps, q);

        if (!si->isValid(q)) {
            si->freeState(q);
            return false;
        }
    }

    si->freeState(q);
    return true;
}

void getTransitions(std::vector<ob::State *> &V, const ob::State *s, const int u, const ob::SpaceInformationPtr si, const int m, ob::State *OBS_STATE, std::map<ob::State *, double> &transitions)
{
    ompl::RNG rng;
    transitions.clear();
    ob::State* q = si->allocState();

    double delta_0 = 0.5;
    double r_0 = 2.5;
    double sigma_delta_0 = 0.1;
    double sigma_delta_1 = 0.2;
    double sigma_r_0 = 0.5;
    double sigma_r_1 = 1.0;
    for (int i = 0; i < m; i++) 
    {
        // Generate sample transitions for needle
        // Sample distance delta and radius r from Gaussian distributions
        double delta = 0.0;
        double r = 0.0;
        if (u == 0) {
            delta = rng.gaussian(delta_0, sigma_delta_0);
            r = rng.gaussian(r_0, sigma_r_0);
        } else {
            delta = rng.gaussian(delta_0, sigma_delta_1);
            r = rng.gaussian(r_0, sigma_r_1);
        }

        // Calculate new sample state q
        propagateCircularArc(s, u, r, delta, q);
        
        ob::State *t = nullptr;
        if (isCollisionFreePath(s, q, u, r, delta, si)) {
            // Find Vertex t that minimizes dist(q, t)
            double minDistance = std::numeric_limits<double>::infinity();
            for (ob::State *v : V) {
                double dist = si->distance(q, v);
                if (dist < minDistance) {
                    minDistance = dist;
                    t = v;
                }
            }
        } else {
            // Set Vertex t to obstacle state
            t = OBS_STATE;
        }
        
        // Find if transitions already contains Vertex t
        if (transitions.count(t) > 0) {
            // Update probability of transitions from Vertex t
            transitions[t] += 1.0/m;
        } else {
            // Initialize probability of transitions from Vertex t
            transitions[t] = 1.0/m;
        }
    }
    si->freeState(q);
}

std::unordered_map<int, Graph> buildSMR(ob::SpaceInformationPtr si, ob::State *start, ob::State *goal, int n, const std::vector<int> &U, int m, ob::State *OBS_STATE) {
    std::unordered_map<int, Graph> SMR; // Map of graphs for each action u

    // Initialize vertices set
    std::vector<ob::State *> V;
    V.push_back(start);
    V.push_back(goal);

    // Sample n collision free states
    ob::State *q = si->allocState();
    auto sampler = si->allocStateSampler();
    while (V.size() < static_cast<size_t>(n+2)) {
        sampler->sampleUniform(q);
        if (si->isValid(q)) {
            V.push_back(si->cloneState(q));
        }
    }
    si->freeState(q);

    // Build graphs for each action u
    for (int u : U) {
        Graph graph;
        graph.vertices = V;

        // Add edges for each vertex and action
        std::map<ob::State *, double> transitions;
        for (ob::State *s : V) {
            getTransitions(V, s, u, si, m, OBS_STATE, transitions);
            for (auto &transition : transitions) {
                graph.edges.push_back({s, transition.first, transition.second});
            }
        }

        SMR[u] = graph;
    }

    return SMR;
}

bool isGoalReachable(ob::State *q, ob::State *goal, double radius) {
    double qX = q->as<ob::SE2StateSpace::StateType>()->getX();
    double qY = q->as<ob::SE2StateSpace::StateType>()->getY();

    double goalX = goal->as<ob::SE2StateSpace::StateType>()->getX();
    double goalY = goal->as<ob::SE2StateSpace::StateType>()->getY();

    return pow(qX-goalX, 2) + pow(qY-goalY, 2) <= pow(radius, 2);
}

std::tuple<std::unordered_map<ob::State *, int>, std::unordered_map<ob::State *, double>> querySMR(std::unordered_map<int, Graph> SMR, ob::State *goal, ob::State *OBS_STATE, double radius) {
    // Build transition probability matrix
    std::unordered_map<int, std::map<ob::State *, std::map<ob::State *, double>>> prob;
     
    for (const auto roadmap: SMR) {
        int u = roadmap.first;
        std::map<ob::State *, std::map<ob::State *, double>> matrix;
        for (auto edge : roadmap.second.edges) {
            (matrix[edge.source])[edge.target] = edge.prob;
        }
        prob[u] = matrix;
    }

    // Value iteration

    // Initialize values
    std::unordered_map<ob::State *, double> values;
    std::unordered_map<ob::State *, double> reward;
    std::vector<ob::State *> &states = SMR[0].vertices;
    for (const auto &state : states) {
        values[state] = 0;
        if (isGoalReachable(state, goal, radius)) {
            reward[state] = 1;
        }
    }

    // Initialize data structure to hold the best actions for each state
    std::unordered_map<ob::State *, int> best_actions;
    
    int max_iterations = states.size();
    bool stop = false;
    std::unordered_map<ob::State *, double> new_values;
    int iter = 0;
    double threshold = 1e-7;
    while (!stop && iter < max_iterations) {
        iter++;
        double max_change_in_value = 0;
        for (const auto &state : states) {
            // It is assumed that when the robot reaches a goal or obstacle state, it stops
            if (reward[state] == 1 || state == OBS_STATE) {
                new_values[state] = reward[state];
                if (new_values[state] - values[state] > max_change_in_value) {
                    max_change_in_value = new_values[state] - values[state];
                }
                continue;
            }

            double maxExpectedValue = 0;
            for (const auto roadmap: SMR) {
                int u = roadmap.first;
                double exp_val = 0;
                for (auto pair : prob[u][state]) {
                    // pair.first accesses a state that stransitions from the current state
                    // pair.second provides the probability from the current state to the next
                    ob::State *next = pair.first;
                    double probability = pair.second;
                    exp_val += probability*values[next];
                }
                maxExpectedValue = std::max(maxExpectedValue, exp_val);

                // If better action is found, update best_actions
                if (maxExpectedValue == exp_val) {
                    best_actions[state] = u;
                }
            }
            new_values[state] = reward[state] + maxExpectedValue;

            if (new_values[state] - values[state] > max_change_in_value) {
                max_change_in_value = new_values[state] - values[state];
            }
        }
        values = new_values;
        // Stop if the max change between the old and new value for every state is below threshold
        if (max_change_in_value < threshold) {
            break;
        }
    }
    std::cout << "Finished in " << iter << " iterations." << std::endl;

    if (iter == max_iterations) {
        std::cerr << "Warning: Value iteration reached max iterations before convergence" << std::endl;
    }

    // Return the optimal policy, which is the best action at each state
    return std::make_tuple(best_actions, values);
}
void makeEnvironment(std::vector<Rectangle> &  obstacles )
{
    
    // TODO: Fill in the vector of rectangles with your street environment.
    Rectangle obs1;
    obs1.x =-.5;
    obs1.y=-.5;
    obs1.width=1;
    obs1.height=.05;
    obstacles.push_back(obs1);
   
    Rectangle obs2;
    obs2.x =-.5;
    obs2.y=.45;
    obs2.width=1;
    obs2.height=.05;
    obstacles.push_back(obs2);
    
    // Rectangle obs3;
    // obs3.x =-.5;
    // obs3.y=-.35;
    // obs3.width=.4;
    // obs3.height=.7;
    // obstacles.push_back(obs3);
    
    // Rectangle obs4;
    // obs4.x =.2;
    // obs4.y=-.35;
    // obs4.width=.4;
    // obs4.height=.7;
    // obstacles.push_back(obs4);
}
std::vector<ob::State *> extractPathFromPolicy(
    ob::SpaceInformationPtr si, 
    ob::State *start, ob::State *goal, 
    std::unordered_map<ob::State *, int> &best_actions, 
    std::unordered_map<int, Graph> &SMR,
    bool verbose,
    int policy ){

    std::vector<ob::State *> path;
    ob::State *current = start;

    while (true) {
        path.push_back(current);

        // Stop if goal is reached
        if (si->distance(current, goal) < 0.01) { // Adjust tolerance as needed
            break;
        }
        
        // Get the best action for the current state
        if (best_actions.find(current) == best_actions.end()) {
            std::cerr << "Error: No action for current state! Path computation aborted." << std::endl;
            break;
        }
        ob::State *next = nullptr;
        int action = best_actions[current];
        if(policy==1){
            // Policy1 choose random vertex but avoid obstcle state
            
            
            for (const auto &edge : SMR[action].edges) {
                if (edge.source == current && edge.target->as<ob::SE2StateSpace::StateType>()->getY()!=9999) {
                    next = const_cast<ob::State *>(edge.target); // Remove const for assignment
                }
            }
        }
        else if(policy==2)
        {
        
            // Policy2 random choose next vertex
            ompl::RNG rng; // Persistent RNG instance
            rng.shuffle(SMR[action].edges.begin(), SMR[action].edges.end());
            next= const_cast<ob::State *>(SMR[action].edges[0].target);
       
        }
        else if(policy==3)
        {
        // Find the next state by maximizing transition probability
        //Policy3 : largest Prob

            double maxProb = 1;
            for (const auto &edge : SMR[action].edges) {
                if (edge.source == current && edge.prob < maxProb) {
                    maxProb = edge.prob;
                    next = const_cast<ob::State *>(edge.target); // Remove const for assignment
                }
            }
        }
        else
            std::cout<<"No such policy"<<std::endl;

        if (!next) {
            if (verbose) {
                std::cerr << "Error: No valid next state found! Path computation aborted." << std::endl;
            }
            break;
        }

        current = next;
    }

    return path;
}

std::tuple<int, int,int, bool> parseArguments(int argc, char* argv[]) {
    // Check if the required arguments are provided
    if (argc < 4) {
        throw std::invalid_argument("Usage: ./program (sample size) (sample transitions) [--verbose]");
    }

    int n, m, policy;
    bool verbose = false;

    try {
        // First argument is n
        n = std::stoi(argv[1]);
    } catch (const std::exception& e) {
        throw std::invalid_argument("The first argument must be an integer (n).");
    }

    try {
        // First argument is n
        m = std::stoi(argv[2]);
    } catch (const std::exception& e) {
        throw std::invalid_argument("The second argument must be an integer (n).");
    }
    try {
        // First argument is n
        policy = std::stoi(argv[3]);
    } catch (const std::exception& e) {
        throw std::invalid_argument("The third argument must be an integer (n).");
    }
    // Check for the optional verbose flag
    if (argc > 3 && std::string(argv[4]) == "--verbose") {
        verbose = true;
    }

    return {n, m, policy,verbose};
}

int main(int argc, char* argv[]) {
    // Parse arguments
    auto parsedArgs = parseArguments(argc, argv);
    int n = std::get<0>(parsedArgs);
    int m = std::get<1>(parsedArgs);
    int policy = std::get<2>(parsedArgs);
    bool verbose = std::get<3>(parsedArgs);
    std::cout << "n: " << n << ", m: " << m << ", verbose: " << (verbose ? "true" : "false") << std::endl;
    
    // Create space
    auto space = std::make_shared<ob::SE2StateSpace>();
    ob::RealVectorBounds bounds(2);
    bounds.setLow(-0.5);
    bounds.setHigh(.5);
    space->setBounds(bounds);

    // Create space information
    auto si = std::make_shared<ob::SpaceInformation>(space);
    si->setStateValidityChecker([](const ob::State *state) { return isCollisionFree(state); });

    // Create start and goal positions
    ob::State *start = si->allocState();
    ob::State *goal = si->allocState();

    start->as<ob::SE2StateSpace::StateType>()->setX(-.5);
    start->as<ob::SE2StateSpace::StateType>()->setY(-.4);
    start->as<ob::SE2StateSpace::StateType>()->setYaw(0.0);

    goal->as<ob::SE2StateSpace::StateType>()->setX(.5);
    goal->as<ob::SE2StateSpace::StateType>()->setY(.4);
    goal->as<ob::SE2StateSpace::StateType>()->setYaw(0.0);

    // Create environment
    obstacles.clear();
    makeEnvironment(obstacles);

    // Define actions
    std::vector<int> U = {0, 1}; // 0 means turn left, 1 means turn right

    // Set obstacle state
    ob::State *OBS_STATE = si->allocState();
    OBS_STATE->as<ob::SE2StateSpace::StateType>()->setX(9999);
    OBS_STATE->as<ob::SE2StateSpace::StateType>()->setY(9999);

    // Build SMR
    std::unordered_map<int, Graph> smr = buildSMR(si, start, goal, n, U, m, OBS_STATE);

    // Output results
    if (verbose) {
        for (const auto roadmap : smr) {
            int u = roadmap.first;
            Graph graph = roadmap.second;
            for (const auto &e : graph.edges) {
                auto source = e.source->as<ob::SE2StateSpace::StateType>();
                auto target = e.target->as<ob::SE2StateSpace::StateType>();
                std::string sStr = "(" + std::to_string(source->getX()) + ", " + std::to_string(source->getY()) + ")";
                std::string tStr = "(" + std::to_string(target->getX()) + std::to_string(target->getY()) + ")";
                std::cout << "Transition prob from " << sStr << " to " << tStr << " is " << e.prob << " with action u=" << u << std::endl;
                if (e.target == goal) {
                    std::cout << "This path leads to the goal" << std::endl;
                }
            }
        }
    }

    // Obtain optimal policy by querying SMR using value iteration
    std::unordered_map<ob::State *, int> best_actions;
    std::unordered_map<ob::State *, double> values;
    double radius = 0.1;
    tie(best_actions, values) = querySMR(smr, goal, OBS_STATE, radius);
    // std::ofstream valuesFile("values.txt");
    // if (valuesFile.is_open()) {
    //     for (auto *v: smr.at(0).vertices) {
    //         double x = v->as<ob::SE2StateSpace::StateType>()->getX();
    //         double y = v->as<ob::SE2StateSpace::StateType>()->getY();
    //         valuesFile << x << "," << y << "," << values[v] << std::endl;
    //     }
    //     std::cout << "Written to values.txt" << std::endl;
    // }

    if (verbose) {
        std::cout << "Optimal policy obtained" << std::endl;
        for (const auto pair : best_actions) {
            auto se2state = pair.first->as<ob::SE2StateSpace::StateType>();
            std::cout << "Best action for state (" << std::to_string(se2state->getX()) << ", " << std::to_string(se2state->getY()) << ") is u=" << pair.second << std::endl;
        }

        for (const auto pair : values) {
            if (pair.first != start) {
                continue;
            }
            auto se2state = pair.first->as<ob::SE2StateSpace::StateType>();
            std::cout << "Probability for state (" << se2state->getX() << ", " << se2state->getY() << ") to reach goal: " << pair.second << std::endl;
        }
    }

    // Experiment 2: Test SMR's sensitivity to sample size n
   
    int successCount = 0;
    for (int k = 0; k < 1000; k++) {
        // Extract and print the path
        std::vector<ob::State *> path = extractPathFromPolicy(si, start, goal, best_actions, smr, verbose,policy);
        if (!path.empty() && isGoalReachable(path.back(), goal, radius)) {
            // This path reached the goal
            successCount++;
        }

        if (verbose) {
            std::cout << "Path from start to goal:" << std::endl;
            for (const auto &state : path) {
                double x = state->as<ob::SE2StateSpace::StateType>()->getX();
                double y = state->as<ob::SE2StateSpace::StateType>()->getY();
                double yaw = state->as<ob::SE2StateSpace::StateType>()->getYaw();
                std::cout << "(" << x << ", " << y << ", " << yaw << ")" << std::endl;
            }
        }
    }
    double actualProb = successCount / 1000.0;
    std::cout << "n = " << n << ". Actual probability is " << actualProb << ". Expected probability is " << values[start] << std::endl;

    // Save actual and expected differences
    std::ofstream outputFile("output" + std::to_string(n) + ".txt");
    if (outputFile.is_open()) {
        outputFile << actualProb - values[start] << std::endl;
        outputFile.close();
        std::cout << "Written to output" + std::to_string(n) + ".txt." << std::endl;
    }
    
    // Free memory in roadmap
    for (auto *v: smr.at(0).vertices) {
        if (v) {
            si->freeState(v);
        }
    }
    obstacles.clear();

    if (OBS_STATE) {
        si->freeState(OBS_STATE);
    }

    return 0;
}
