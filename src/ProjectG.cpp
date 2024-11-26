#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/util/RandomNumbers.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/datastructures/NearestNeighborsSqrtApprox.h>
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
    std::shared_ptr<ompl::NearestNeighbors<ob::State *>> vertices;
    std::vector<Edge> edges;
};

class NeedleStateSpace: public ob::CompoundStateSpace
{
    public:
    NeedleStateSpace()
    {
        addSubspace(std::make_shared<ob::SE2StateSpace>(), 1.0);
        // Set the weight of the orientation theta to alpha
        addSubspace(std::make_shared<ob::RealVectorStateSpace>(1), 1.0);
        getSubspaces()[0]->as<ob::SE2StateSpace>()->setSubspaceWeight(1, alpha);
    }

    double distance(const ob::State *state1, const ob::State *state2) const
    {
        const auto *cstate1 = static_cast<const ob::CompoundState *>(state1);
        const auto *cstate2 = static_cast<const ob::CompoundState *>(state2);
        double se2_dist = components_[0]->distance(cstate1->components[0], cstate2->components[0]);
        double M = 0;

        double b1 = cstate1->as<ob::RealVectorStateSpace::StateType>(1)->values[0];
        double b2 = cstate2->as<ob::RealVectorStateSpace::StateType>(1)->values[0];
        if (b1 != b2) {
            M = 1;
        }
        return se2_dist + M;
    }

    protected:
    double alpha{0.2}; // Default hyperparameter
};

// Assuming state is of type NeedleSpace::StateType
std::tuple<double, double, double> getCoord(const ob::State *state) {
    auto *needleState = state->as<NeedleStateSpace::StateType>()->as<ob::SE2StateSpace::StateType>(0);
    double x = needleState->getX();
    double y = needleState->getY();
    double theta = needleState->getYaw();

    return std::make_tuple(x, y, theta);
}

bool isCollisionFree(const ob::State *state) {
    double x, y;
    std::tie(x, y, std::ignore) = getCoord(state);

    // Check for going out of bounds
    if (x < 0 || x > 10 || y < 0 || y > 10) {
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
    double x, y, theta;
    std::tie(x, y, theta) = getCoord(s);
    int sign = (u == 0) ? 1 : -1;
    double theta_new = theta + sign * delta / r;
    double x_new = x - sign * r * std::sin(theta) + sign * r * std::sin(theta_new); 
    double y_new = y + sign * r * std::cos(theta) - sign * r * std::cos(theta_new);

    q->as<NeedleStateSpace::StateType>()->as<ob::SE2StateSpace::StateType>(0)->setX(x_new);
    q->as<NeedleStateSpace::StateType>()->as<ob::SE2StateSpace::StateType>(0)->setY(y_new);
    q->as<NeedleStateSpace::StateType>()->as<ob::SE2StateSpace::StateType>(0)->setYaw(theta_new);
    q->as<NeedleStateSpace::StateType>()->as<ob::RealVectorStateSpace::StateType>(1)->values[0] = u;
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

void getTransitions(
    std::shared_ptr<ompl::NearestNeighbors<ob::State *>> nn, 
    const ob::State *s, 
    const int u, 
    const ob::SpaceInformationPtr si, 
    const int m, 
    ob::State *OBS_STATE, 
    std::vector<std::tuple<ob::State *, double>> &transitions)
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
    double prob_step = 1.0/m;
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
            t = nn->nearest(q);
        } else {
            // Set Vertex t to obstacle state
            t = OBS_STATE;
        }
        transitions.push_back(std::make_tuple(t, prob_step));
    }
    si->freeState(q);
}

std::unordered_map<int, Graph> buildSMR(
    ob::SpaceInformationPtr si, 
    int n, 
    const std::vector<int> &U, 
    int m, 
    ob::State *OBS_STATE,
    std::shared_ptr<ompl::NearestNeighbors<ob::State *>> nn) 
{
    std::unordered_map<int, Graph> SMR; // Map of graphs for each action u

    // Sample n collision free states
    ob::State *q = si->allocState();
    auto sampler = si->allocStateSampler();
    while (nn->size() < static_cast<size_t>(n)) {
        sampler->sampleUniform(q);

        // Set the turning direction to a binary integer 
        double real_b = q->as<NeedleStateSpace::StateType>()->as<ob::RealVectorStateSpace::StateType>(1)->values[0];
        q->as<NeedleStateSpace::StateType>()->as<ob::RealVectorStateSpace::StateType>(1)->values[0] = floor(real_b);
        if (si->isValid(q)) {
            nn->add(si->cloneState(q));
        }
    }
    si->freeState(q);

    // Build graphs for each action u
    std::vector<ob::State *> V;
    nn->list(V);
    for (int u : U) {
        Graph graph;
        graph.vertices = nn;

        // Add edges for each vertex and action
        std::vector<std::tuple<ob::State *, double>> transitions;
        for (ob::State *s : V) {
            getTransitions(nn, s, u, si, m, OBS_STATE, transitions);
            for (auto &transition : transitions) {
                graph.edges.push_back({s, std::get<0>(transition), std::get<1>(transition)});
            }
        }

        SMR[u] = graph;
    }

    return SMR;
}

bool isGoalReachable(ob::State *q, ob::State *goal, double radius, ob::SpaceInformationPtr si) {
    return si->distance(q, goal) <= radius;
}

std::tuple<std::unordered_map<ob::State *, int>, std::unordered_map<ob::State *, double>> querySMR(
    std::unordered_map<int, Graph> SMR, 
    ob::State *goal, 
    ob::State *OBS_STATE, 
    double radius, 
    ob::SpaceInformationPtr si)
{
    // Build transition probability matrix
    std::unordered_map<int, std::unordered_map<ob::State *, std::vector<std::pair<ob::State *, double>>>> prob;
    for (const auto roadmap: SMR) {
        int u = roadmap.first;
        for (auto edge : roadmap.second.edges) {
            prob[u][edge.source].emplace_back(edge.target, edge.prob);
        }
    }

    // Initialize values and rewards
    std::unordered_map<ob::State *, double> values, reward;
    std::vector<ob::State *> states;
    SMR.at(0).vertices->list(states);

    for (const auto &state : states) {
        values[state] = 0.0;
        reward[state] = isGoalReachable(state, goal, radius, si) ? 1.0 : 0.0;
    }

    // Initialize data structure to hold the best actions for each state
    std::unordered_map<ob::State *, int> best_actions;
    
    int max_iterations = static_cast<int>(states.size());
    std::unordered_map<ob::State *, double> new_values;
    int iter = 0;
    double threshold = 1e-7;
    double gamma = 0.00001; // penalty hyperparameter
    while (iter++ < max_iterations) {
        double max_change_in_value = 0.0;

        for (const auto &state : states) {
            double R = reward[state];
            // It is assumed that when the robot reaches a goal or obstacle state, it stops
            if (R == 1 || state == OBS_STATE) {
                new_values[state] = R;
                max_change_in_value = std::max(std::abs(new_values[state] - values[state]), max_change_in_value);
                continue;
            }

            double maxExpectedValue = -1.0;
            for (const auto &actionPair : prob) {
                int u = actionPair.first;
                double exp_val = 0.0;
                for (const auto &pair : actionPair.second.at(state)) {
                    // pair.first accesses a state that transitions from the current state
                    // pair.second provides the probability from the current state to the next
                    ob::State *next = pair.first;
                    double probability = pair.second;
                    exp_val += probability*(values[next]-gamma);
                }

                // If better action is found, update best_actions
                if (exp_val > maxExpectedValue) {
                    maxExpectedValue = exp_val;
                    best_actions[state] = u;
                }
            }
            new_values[state] = R + maxExpectedValue;
            max_change_in_value = std::max(std::abs(new_values[state] - values[state]), max_change_in_value);
        }
        values.swap(new_values);

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
    // Fill in the vector of rectangles with your street environment.
    Rectangle obs1;
    obs1.x = 3;
    obs1.y = 6;
    obs1.width = 3;
    obs1.height = 2;
    obstacles.push_back(obs1);
   
    Rectangle obs2;
    obs2.x = 3;
    obs2.y = 3;
    obs2.width = 2;
    obs2.height = 1;
    obstacles.push_back(obs2);

    // Rectangle obs1;
    // obs1.x =-.5;
    // obs1.y=-.5;
    // obs1.width=1;
    // obs1.height=.05;
    // obstacles.push_back(obs1);
   
    // Rectangle obs2;
    // obs2.x =-.5;
    // obs2.y=.45;
    // obs2.width=1;
    // obs2.height=.05;
    // obstacles.push_back(obs2);
    
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

    int max_iterations = 1000;
    int iter = 0;
    while (iter < max_iterations) {
        iter++;
        path.push_back(current);

        // Stop if goal is reached
        if (si->distance(current, goal) < 0.01) { // Adjust tolerance as needed
            break;
        }
        
        // Get the best action for the current state
        if (best_actions.find(current) == best_actions.end() && verbose) {
            std::cerr << "Error: No action for current state! Path computation aborted." << std::endl;
            break;
        }
        ob::State *next = nullptr;
        int action = best_actions[current];
        if(policy==1){
            // Policy1 choose random vertex but avoid obstcle state
            
            
            for (const auto &edge : SMR[action].edges) {
                double y;
                std::tie(std::ignore, y, std::ignore) = getCoord(edge.target);
                if (edge.source == current && y != 9999) {
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

 std::vector<std::tuple<ob::State *, double, double, int>> simulatePath(
    std::shared_ptr<ompl::NearestNeighborsSqrtApprox<ob::State *>> nn, 
    std::unordered_map<ob::State *, int> best_actions, 
    ob::SpaceInformationPtr si,
    ob::State *start,
    ob::State *goal,
    double radius) 
{
    std::vector<std::tuple<ob::State *, double, double, int>> path;

    double delta = 0.0;
    double r = 0.0;
    ompl::RNG rng;

    double delta_0 = 0.5;
    double r_0 = 2.5;
    double sigma_delta_0 = 0.1;
    double sigma_delta_1 = 0.2;
    double sigma_r_0 = 0.5;
    double sigma_r_1 = 1.0;
    int u = best_actions[start];

    ob::State *current = start;

    int max_iterations = 100; // Arbitrary stopping point
    int iter = 0;
    ob::State *temp = si->allocState();
    while (iter++ < max_iterations) {
        if (u == 0) {
            delta = rng.gaussian(delta_0, sigma_delta_0);
            r = rng.gaussian(r_0, sigma_r_0);
        } else {
            delta = rng.gaussian(delta_0, sigma_delta_1);
            r = rng.gaussian(r_0, sigma_r_1);
        }

        path.push_back(std::make_tuple(current, delta, r, u));
        
        propagateCircularArc(current, u, r, delta, temp);
        
        if (!isCollisionFreePath(current, temp, u, r, delta, si)) {
            // Stop when collision detected
            path.push_back(std::make_tuple(si->cloneState(temp), -1, -1, -1));
            si->freeState(temp);
            return path;
        }
        
        if (isGoalReachable(temp, goal, radius, si)) {
            // Stop when a successful path is found
            path.push_back(std::make_tuple(si->cloneState(temp), -1, -1, -1));
            si->freeState(temp);
            return path;
        }
        ob::State *near = nn->nearest(temp);
        if (best_actions.find(near) == best_actions.end()) {
            // The nearest vertex in the roadmap is in the goal but the point itself is not in the goal.
            // best_actions will not have an optimal policy for goal states.
            // For this edge case, choose a random action.
            ompl::RNG rng;
            u = floor(rng.uniformReal(0, 2));
        } else {
            u = best_actions.at(near);
        }
        current = si->cloneState(temp);
    }
    if (current != start) {
        si->freeState(current);
    }
    si->freeState(temp);
    return path;
}

std::tuple<int, int,int, bool> parseArguments(int argc, char* argv[]) {
    // Check if the required arguments are provided
    if (argc < 4) {
        throw std::invalid_argument("Usage: ./program (sample size) (sample transitions) (experiment bool) [--verbose]");
    }

    int n, m, run_experiment;
    bool verbose = false;

    try {
        // First argument is n
        n = std::stoi(argv[1]);
    } catch (const std::exception& e) {
        throw std::invalid_argument("The first argument must be an integer (n).");
    }

    try {
        // Second argument is m
        m = std::stoi(argv[2]);
    } catch (const std::exception& e) {
        throw std::invalid_argument("The second argument must be an integer (n).");
    }
    try {
        // Third argument will start the experiment
        run_experiment = std::stoi(argv[3]);
    } catch (const std::exception& e) {
        throw std::invalid_argument("The third argument must be an integer (n).");
    }
    // Check for the optional verbose flag
    if (argc > 4 && std::string(argv[4]) == "--verbose") {
        verbose = true;
    }

    return {n, m, run_experiment, verbose};
}

int main(int argc, char* argv[]) {
    // Parse arguments
    auto parsedArgs = parseArguments(argc, argv);
    int n = std::get<0>(parsedArgs);
    int m = std::get<1>(parsedArgs);
    int run_experiment = std::get<2>(parsedArgs);
    bool verbose = std::get<3>(parsedArgs);
    std::cout << "n: " << n << ", m: " << m << ", verbose: " << (verbose ? "true" : "false") << std::endl;
    
    // Create space
    auto space = std::make_shared<NeedleStateSpace>();
    ob::RealVectorBounds bounds(2);
    bounds.setLow(0);
    bounds.setHigh(10);
    space->getSubspace(0)->as<ob::SE2StateSpace>()->setBounds(bounds);
    
    ob::RealVectorBounds bBounds(1);
    bBounds.setLow(0);
    bBounds.setHigh(2);
    space->getSubspace(1)->as<ob::RealVectorStateSpace>()->setBounds(bBounds);

    // Create space information
    auto si = std::make_shared<ob::SpaceInformation>(space);
    si->setStateValidityChecker([](const ob::State *state) { return isCollisionFree(state); });

    // Create start and goal positions
    ob::State *start = si->allocState();
    ob::State *goal = si->allocState();

    start->as<NeedleStateSpace::StateType>()->as<ob::SE2StateSpace::StateType>(0)->setX(0);
    start->as<NeedleStateSpace::StateType>()->as<ob::SE2StateSpace::StateType>(0)->setY(4);
    start->as<NeedleStateSpace::StateType>()->as<ob::SE2StateSpace::StateType>(0)->setYaw(0);
    start->as<NeedleStateSpace::StateType>()->as<ob::RealVectorStateSpace::StateType>(1)->values[0] = 0;

    goal->as<NeedleStateSpace::StateType>()->as<ob::SE2StateSpace::StateType>(0)->setX(8);
    goal->as<NeedleStateSpace::StateType>()->as<ob::SE2StateSpace::StateType>(0)->setY(8);
    goal->as<NeedleStateSpace::StateType>()->as<ob::SE2StateSpace::StateType>(0)->setYaw(M_PI);
    goal->as<NeedleStateSpace::StateType>()->as<ob::RealVectorStateSpace::StateType>(1)->values[0] = 0;

    // Create environment
    obstacles.clear();
    makeEnvironment(obstacles);

    // Initialize NearestNeighbors
    auto nn = std::make_shared<ompl::NearestNeighborsSqrtApprox<ob::State *>>();
    nn->setDistanceFunction([&](const ob::State *a, const ob::State *b){
        return space->distance(a, b);
    });
    // Add start and goal states to vertices
    nn->add(start);

    // Define actions
    std::vector<int> U = {0, 1}; // 0 means turn left, 1 means turn right

    // Set obstacle state
    ob::State *OBS_STATE = si->allocState();
    OBS_STATE->as<NeedleStateSpace::StateType>()->as<ob::SE2StateSpace::StateType>(0)->setX(9999);
    OBS_STATE->as<NeedleStateSpace::StateType>()->as<ob::SE2StateSpace::StateType>(0)->setY(9999);
    OBS_STATE->as<NeedleStateSpace::StateType>()->as<ob::SE2StateSpace::StateType>(0)->setYaw(0);
    OBS_STATE->as<NeedleStateSpace::StateType>()->as<ob::RealVectorStateSpace::StateType>(1)->values[0] = 0;

    // Build SMR
    std::unordered_map<int, Graph> smr = buildSMR(si, n, U, m, OBS_STATE, nn);
    std::vector<ob::State *> V;
    smr[0].vertices->list(V);

    // Output results
    if (verbose) {
        for (const auto roadmap : smr) {
            int u = roadmap.first;
            Graph graph = roadmap.second;
            for (const auto &e : graph.edges) {
                double sx, sy, tx, ty;
                std::tie(sx, sy, std::ignore) = getCoord(e.source);
                std::tie(tx, ty, std::ignore) = getCoord(e.target);
                std::string sStr = "(" + std::to_string(sx) + ", " + std::to_string(sy) + ")";
                std::string tStr = "(" + std::to_string(tx) + ", " + std::to_string(ty) + ")";
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
    double radius = 1;
    std::tie(best_actions, values) = querySMR(smr, goal, OBS_STATE, radius, si);

    if (run_experiment <= 0) {
        std::ofstream valuesFile("values.txt");
        if (valuesFile.is_open()) {
            for (auto *v: V) {
                double x, y;
                std::tie(x, y, std::ignore) = getCoord(v);
                valuesFile << x << "," << y << "," << values[v] << std::endl;
            }
            valuesFile.close();
            std::cout << "Written to values.txt" << std::endl;
        }
    }

    if (verbose) {
        std::cout << "Optimal policy obtained" << std::endl;
        for (const auto pair : best_actions) {
            double x, y;
            std::tie(x, y, std::ignore) = getCoord(pair.first);
            std::cout << "Best action for state (" << std::to_string(x) << ", " << std::to_string(y) << ") is u=" << pair.second << std::endl;
        }

        for (const auto pair : values) {
            if (pair.first != start) {
                continue;
            }
            double x, y;
            std::tie(x, y, std::ignore) = getCoord(pair.first);
            std::cout << "Probability for state (" << x << ", " << y << ") to reach goal: " << pair.second << std::endl;
        }
    }

    std::vector<std::tuple<ob::State *, double, double, int>> path;
    if (run_experiment > 0) {
        // Experiment 2: Test SMR's sensitivity to sample size n
        int successCount = 0;
        for (int k = 0; k < 1000; k++) {
            // Extract and print the path
            path = simulatePath(nn, best_actions, si, start, goal, radius); 
            if (!path.empty()) {
                ob::State *state = std::get<0>(path.back());
                if (isGoalReachable(state, goal, radius, si)) {
                    // This path reached the goal
                    successCount++;
                }
            }

            if (verbose) {
                std::cout << "Path from start to goal:" << std::endl;
                for (const auto &tuple : path) {
                    ob::State *state = std::get<0>(tuple);
                    double delta = std::get<1>(tuple);
                    double r = std::get<2>(tuple);
                    double x, y, theta;
                    std::tie(x, y, theta) = getCoord(state);
                    std::cout << "(" << x << ", " << y << ", " << theta << ")" << " with arc length " << delta << " and radius " << r << std::endl;
                }
            }
            // Free memory in path
            for (auto tuple: path) {
                ob::State *state = std::get<0>(tuple);
                if (state && state != start) {
                    si->freeState(state);
                }
            }
        }
        double actualProb = successCount / 1000.0;
        std::cout << "n = " << n << ". Actual probability is " << actualProb << ". Expected probability is " << values[start] << std::endl;

        // Save actual and expected differences
        std::ofstream outputFile("output" + std::to_string(n) + ".txt", std::ofstream::app);
        if (outputFile.is_open()) {
            outputFile << actualProb - values[start] << std::endl;
            outputFile.close();
            std::cout << "Written to output" + std::to_string(n) + ".txt." << std::endl;
        }
    } else {
        // Simulate a path once and save path
        path = simulatePath(nn, best_actions, si, start, goal, radius); 
        std::ofstream pathFile("path.txt");
        if (pathFile.is_open()) {
            for (auto &tuple: path) {
                ob::State *state = std::get<0>(tuple);
                double x, y, theta;
                std::tie(x, y, theta) = getCoord(state);
                double delta = std::get<1>(tuple);
                double r = std::get<2>(tuple);
                int u = std::get<3>(tuple);
                pathFile << x << "," << y << "," << theta << "," << delta << "," << r << "," << u << std::endl;
            }
            pathFile.close();
            std::cout << "Written to path.txt" << std::endl;
        }

        // Free memory in path
        for (auto tuple: path) {
            ob::State *state = std::get<0>(tuple);
            if (state && state != start) {
                si->freeState(state);
            }
        }
    }
    
    // Free memory in roadmap
    for (auto *v: V) {
        if (v) {
            si->freeState(v);
        }
    }
    obstacles.clear();

    if (OBS_STATE) {
        si->freeState(OBS_STATE);
    }

    // Free goal state
    if (goal) {
        si->freeState(goal);
    }

    return 0;
}
