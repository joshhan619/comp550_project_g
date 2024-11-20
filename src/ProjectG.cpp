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

namespace ob = ompl::base;

using Transition = std::pair<const ob::State *, double>;

struct Edge {
    const ob::State *source;
    const ob::State *target;
    double prob;
};

struct Graph {
    std::vector<ob::State *> vertices;
    std::vector<Edge> edges;
};

// TODO: Implement isCollisionFree with obstacles
bool isCollisionFree(const ob::State *state) {
    return true;
}

void applyControlInput(const ob::SE2StateSpace::StateType *start, ob::SE2StateSpace::StateType *result, double t, int u) {
    double x = start->getX();
    double y = start->getY();
    double theta = start->getYaw();

    // Adjust turning angle based on control input
    double turnAngle = (u == 0) ? -M_PI_4 : M_PI_4; // Turn left or right by 45 degrees
    double adjustedTheta = theta + t * turnAngle;

    // Generate the new state
    result->setX(x + t * std::cos(adjustedTheta)); // Move forward with rotation
    result->setY(y + t * std::sin(adjustedTheta));
    result->setYaw(adjustedTheta);
}

bool isCollisionFreePath(const ob::State *s1, const ob::State *s2, const int u, ob::SpaceInformationPtr si)
{
    const double stepSize = si->getStateSpace()->getMaximumExtent() / 100.0;
    int numSteps = static_cast<int>(si->distance(s1, s2) / stepSize);

    auto *intermediateState = si->allocState();
    const auto *start = s1->as<ob::SE2StateSpace::StateType>();

    for (int i = 0; i <= numSteps; ++i) {
        double t = static_cast<double>(i) / numSteps;

        // Apply control input to generate intermediate states
        auto *intermediate = intermediateState->as<ob::SE2StateSpace::StateType>();
        applyControlInput(start, intermediate, t, u);

        // Check if the intermediate state is collision-free
        if (!si->isValid(intermediateState)) {
            si->freeState(intermediateState);
            return false;
        }
    }

    si->freeState(intermediateState);
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
    auto sState = s->as<ob::SE2StateSpace::StateType>();
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
        double theta = sState->getYaw();
        int sign = (u == 0) ? 1 : -1;
        double theta_new = theta + sign * delta / r;
        double x_new = sState->getX() - sign * r * std::sin(theta) + sign * r * std::sin(theta_new); 
        double y_new = sState->getY() + sign * r * std::cos(theta) - sign * r * std::cos(theta_new);
    
        q->as<ob::SE2StateSpace::StateType>()->setX(x_new);
        q->as<ob::SE2StateSpace::StateType>()->setY(y_new);
        q->as<ob::SE2StateSpace::StateType>()->setYaw(theta_new);
        
        ob::State *t = nullptr;
        if (isCollisionFreePath(s, q, u, si)) {
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

std::unordered_map<int, Graph> buildSMR(ob::SpaceInformationPtr si, int n, const std::vector<int> &U, int m, ob::State *OBS_STATE) {
    std::unordered_map<int, Graph> SMR; // Map of graphs for each action u

    // Initialize vertices set
    std::vector<ob::State *> V;

    // Sample n collision free states
    ob::State *q = si->allocState();
    auto sampler = si->allocStateSampler();
    while (V.size() < static_cast<size_t>(n)) {
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
            for (const auto &transition : transitions) {
                graph.edges.push_back({s, transition.first, transition.second});
            }
        }

        SMR[u] = graph;
    }


    return SMR;
}

// TODO: Implement querySMR
// void querySMR(const std::unordered_map<int, Graph> roadmap)
// {
//     double value = 0;
// }

int main() {
    auto space = std::make_shared<ob::SE2StateSpace>();
    ob::RealVectorBounds bounds(2);
    bounds.setLow(0);
    bounds.setHigh(1);
    space->setBounds(bounds);

    // Create space information
    auto si = std::make_shared<ob::SpaceInformation>(space);
    si->setStateValidityChecker([](const ob::State *state) { return isCollisionFree(state); });

    // Define actions
    std::vector<int> U = {0, 1}; // 0 means turn left, 1 means turn right

    // Parameters
    int n = 100;  // Number of nodes
    int m = 10;   // Number of sample points per transition

    // Build SMR
    ob::State *OBS_STATE = si->allocState();
    std::unordered_map<int, Graph> smr = buildSMR(si, n, U, m, OBS_STATE);

    // Output results
    for (const auto roadmap : smr) {
        int u = roadmap.first;
        Graph graph = roadmap.second;
        std::cout << "Action " << u << std::endl;
        std::cout << "Vertices: " << graph.vertices.size() << std::endl;
        std::cout << "Edges: " << graph.edges.size() << std::endl;
        for (const auto &e : graph.edges) {
            std::cout << "Edge Probability: " << e.prob << std::endl;
        }
    }

    // Free memory in roadmap
    for (auto *v: smr.at(0).vertices) {
        if (v) {
            si->freeState(v);
        }
    }

    if (OBS_STATE) {
        si->freeState(OBS_STATE);
    }

    return 0;
}