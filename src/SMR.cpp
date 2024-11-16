#include "ompl/geometric/planners/SMR/SMR.h"
#include "ompl/geometric/planners/SMR/ConnectionStrategy.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/datastructures/PDF.h"
#include "ompl/tools/config/SelfConfig.h"
#include "ompl/tools/config/MagicConstants.h"
#include <boost/graph/astar_search.hpp>
#include <boost/graph/incremental_components.hpp>
#include <boost/property_map/vector_property_map.hpp>
#include <boost/foreach.hpp>
#include <thread>
#include <typeinfo>

#include "GoalVisitor.hpp"

#include "ompl/control/spaces/DiscreteControlSpace.h"

#define foreach BOOST_FOREACH

namespace ompl
{
    namespace magic
    {
        /** \brief The number of steps to take for a random bounce
            motion generated as part of the expansion step of SMR. */
        static const unsigned int MAX_RANDOM_BOUNCE_STEPS = 5;

        /** \brief The time in seconds for a single roadmap building operation (dt)*/
        static const double ROADMAP_BUILD_TIME = 0.2;

        /** \brief The number of nearest neighbors to consider by
            default in the construction of the SMR roadmap */
        static const unsigned int DEFAULT_NEAREST_NEIGHBORS = 10;
    }  // namespace magic
}  // namespace ompl

ompl::control::SMR::SMR(const base::SpaceInformationPtr &si)
  : base::Planner(si, "SMR")
  , stateProperty_(boost::get(vertex_state_t(), g_))
  , totalConnectionAttemptsProperty_(boost::get(vertex_total_connection_attempts_t(), g_))
  , successfulConnectionAttemptsProperty_(boost::get(vertex_successful_connection_attempts_t(), g_))
  , weightProperty_(boost::get(boost::edge_weight, g_))
  , disjointSets_(boost::get(boost::vertex_rank, g_), boost::get(boost::vertex_predecessor, g_))
{
    specs_.recognizedGoal = base::GOAL_SAMPLEABLE_REGION;
    specs_.approximateSolutions = true;
    specs_.optimizingPaths = true;
    specs_.multithreaded = true;

    
    Planner::declareParam<unsigned int>("max_nearest_neighbors", this, &SMR::setMaxNearestNeighbors,
                                        &SMR::getMaxNearestNeighbors, std::string("8:1000"));

    addPlannerProgressProperty("iterations INTEGER", [this] { return getIterationCount(); });
    addPlannerProgressProperty("best cost REAL", [this] { return getBestCost(); });
    addPlannerProgressProperty("milestone count INTEGER", [this] { return getMilestoneCountString(); });
    addPlannerProgressProperty("edge count INTEGER", [this] { return getEdgeCountString(); });
}

ompl::control::SMR::~SMR()
{
    freeMemory();
}

void ompl::control::SMR::setup()
{
    Planner::setup();
    if (!nn_)
    {
        specs_.multithreaded = false;  // temporarily set to false since nn_ is used only in single thread
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Vertex>(this));
        specs_.multithreaded = true;
        nn_->setDistanceFunction([this](const Vertex a, const Vertex b) { return distanceFunction(a, b); });
    }
    if (!connectionStrategy_)
        connectionStrategy_ = KStrategy<Vertex>(magic::DEFAULT_NEAREST_NEIGHBORS, nn_);
    if (!connectionFilter_)
        connectionFilter_ = [](const Vertex &, const Vertex &) { return true; };

    // Setup optimization objective
    //
    // If no optimization objective was specified, then default to
    // optimizing path length as computed by the distance() function
    // in the state space.
    if (pdef_)
    {
        if (pdef_->hasOptimizationObjective())
            opt_ = pdef_->getOptimizationObjective();
        else
        {
            opt_ = std::make_shared<base::PathLengthOptimizationObjective>(si_);
            opt_->setCostThreshold(opt_->infiniteCost());
        }
    }
    else
    {
        OMPL_INFORM("%s: problem definition is not set, deferring setup completion...", getName().c_str());
        setup_ = false;
    }
}

void ompl::control::SMR::setMaxNearestNeighbors(unsigned int k)
{
    if (!nn_)
    {
        specs_.multithreaded = false;  // temporarily set to false since nn_ is used only in single thread
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Vertex>(this));
        specs_.multithreaded = true;
        nn_->setDistanceFunction([this](const Vertex a, const Vertex b) { return distanceFunction(a, b); });
    }
    if (!userSetConnectionStrategy_)
        connectionStrategy_ = KStrategy<Vertex>(k, nn_);
    if (isSetup())
        setup();
}

unsigned int ompl::control::SMR::getMaxNearestNeighbors() const
{
    const auto strategy = connectionStrategy_.target<KStrategy<Vertex>>();
    return strategy ? strategy->getNumNeighbors() : 0u;
}


void ompl::control::SMR::setProblemDefinition(const base::ProblemDefinitionPtr &pdef)
{
    Planner::setProblemDefinition(pdef);
    clearQuery();
}

void ompl::control::SMR::clearQuery()
{
    startM_.clear();
    goalM_.clear();
    pis_.restart();
}

void ompl::control::SMR::clear()
{
    Planner::clear();
    sampler_.reset();
    simpleSampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();
    clearQuery();

    iterations_ = 0;
    bestCost_ = base::Cost(std::numeric_limits<double>::quiet_NaN());
}

void ompl::control::SMR::freeMemory()
{
    foreach (Vertex v, boost::vertices(g_))
        si_->freeState(stateProperty_[v]);
    g_.clear();
}

void ompl::control::SMR::expandRoadmap(double expandTime)
{
    expandRoadmap(base::timedPlannerTerminationCondition(expandTime));
}

void ompl::control::SMR::expandRoadmap(const base::PlannerTerminationCondition &ptc)
{
    if (!simpleSampler_)
        simpleSampler_ = si_->allocStateSampler();

    std::vector<base::State *> states(magic::MAX_RANDOM_BOUNCE_STEPS);
    si_->allocStates(states);
    expandRoadmap(ptc, states);
    si_->freeStates(states);
}

void ompl::control::SMR::expandRoadmap(const base::PlannerTerminationCondition &ptc,
                                         std::vector<base::State *> &workStates)
{
    // construct a probability distribution over the vertices in the roadmap
    // as indicated in
    //  "Probabilistic Roadmaps for Path Planning in High-Dimensional Configuration Spaces"
    //        Lydia E. Kavraki, Petr Svestka, Jean-Claude Latombe, and Mark H. Overmars

    PDF<Vertex> pdf;
    foreach (Vertex v, boost::vertices(g_))
    {
        const unsigned long int t = totalConnectionAttemptsProperty_[v];
        pdf.add(v, (double)(t - successfulConnectionAttemptsProperty_[v]) / (double)t);
    }

    if (pdf.empty())
        return;

    while (!ptc)
    {
        iterations_++;
        Vertex v = pdf.sample(rng_.uniform01());
        unsigned int s =
            si_->randomBounceMotion(simpleSampler_, stateProperty_[v], workStates.size(), workStates, false);
        if (s > 0)
        {
            s--;
            Vertex last = addMilestone(si_->cloneState(workStates[s]));

            graphMutex_.lock();
            for (unsigned int i = 0; i < s; ++i)
            {
                // add the vertex along the bouncing motion
                Vertex m = boost::add_vertex(g_);
                stateProperty_[m] = si_->cloneState(workStates[i]);
                totalConnectionAttemptsProperty_[m] = 1;
                successfulConnectionAttemptsProperty_[m] = 0;
                disjointSets_.make_set(m);

                // add the edge to the parent vertex
                const base::Cost weight = opt_->motionCost(stateProperty_[v], stateProperty_[m]);
                const Graph::edge_property_type properties(weight);
                boost::add_edge(v, m, properties, g_);
                uniteComponents(v, m);

                // add the vertex to the nearest neighbors data structure
                nn_->add(m);
                v = m;
            }

            // if there are intermediary states or the milestone has not been connected to the initially sampled vertex,
            // we add an edge
            if (s > 0 || !sameComponent(v, last))
            {
                // add the edge to the parent vertex
                const base::Cost weight = opt_->motionCost(stateProperty_[v], stateProperty_[last]);
                const Graph::edge_property_type properties(weight);
                boost::add_edge(v, last, properties, g_);
                uniteComponents(v, last);
            }
            graphMutex_.unlock();
        }
    }
}

void ompl::control::SMR::growRoadmap(double growTime)
{
    growRoadmap(base::timedPlannerTerminationCondition(growTime));
}

void ompl::control::SMR::growRoadmap(const base::PlannerTerminationCondition &ptc)
{
    if (!isSetup())
        setup();
    if (!sampler_)
        sampler_ = si_->allocValidStateSampler();

    base::State *workState = si_->allocState();
    growRoadmap(ptc, workState);
    si_->freeState(workState);
}

void ompl::control::SMR::growRoadmap(const base::PlannerTerminationCondition &ptc, base::State *workState)
{
    /* grow roadmap in the regular fashion -- sample valid states, add them to the roadmap, add valid connections */
    while (!ptc)
    {
        iterations_++;
        // search for a valid state
        bool found = false;
        while (!found && !ptc)
        {
            unsigned int attempts = 0;
            do
            {
                found = sampler_->sample(workState);
                attempts++;
            } while (attempts < magic::FIND_VALID_STATE_ATTEMPTS_WITHOUT_TERMINATION_CHECK && !found);
        }
        // add it as a milestone
        if (found)
            addMilestone(si_->cloneState(workState));
    }
}

void ompl::control::SMR::checkForSolution(const base::PlannerTerminationCondition &ptc, base::PathPtr &solution)
{
    auto *goal = static_cast<base::GoalSampleableRegion *>(pdef_->getGoal().get());
    while (!ptc && !addedNewSolution_)
    {
        // Check for any new goal states
        if (goal->maxSampleCount() > goalM_.size())
        {
            const base::State *st = pis_.nextGoal();
            if (st != nullptr)
                goalM_.push_back(addMilestone(si_->cloneState(st)));
        }

        // Check for a solution
        addedNewSolution_ = maybeConstructSolution(startM_, goalM_, solution);
        // Sleep for 1ms
        if (!addedNewSolution_)
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

bool ompl::control::SMR::maybeConstructSolution(const std::vector<Vertex> &starts, const std::vector<Vertex> &goals,
                                                  base::PathPtr &solution)
{
    base::Goal *g = pdef_->getGoal().get();
    base::Cost sol_cost(opt_->infiniteCost());
    foreach (Vertex start, starts)
    {
        foreach (Vertex goal, goals)
        {
            // we lock because the connected components algorithm is incremental and may change disjointSets_
            graphMutex_.lock();
            bool same_component = sameComponent(start, goal);
            graphMutex_.unlock();

            if (same_component && g->isStartGoalPairValid(stateProperty_[goal], stateProperty_[start]))
            {
                base::PathPtr p = constructSolution(start, goal);
                if (p)
                {
                    base::Cost pathCost = p->cost(opt_);
                    if (opt_->isCostBetterThan(pathCost, bestCost_))
                        bestCost_ = pathCost;
                    // Check if optimization objective is satisfied
                    if (opt_->isSatisfied(pathCost))
                    {
                        solution = p;
                        return true;
                    }
                    if (opt_->isCostBetterThan(pathCost, sol_cost))
                    {
                        solution = p;
                        sol_cost = pathCost;
                    }
                }
            }
        }
    }

    return false;
}

bool ompl::control::SMR::addedNewSolution() const
{
    return addedNewSolution_;
}

ompl::base::PlannerStatus ompl::control::SMR::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    auto *goal = dynamic_cast<base::GoalSampleableRegion *>(pdef_->getGoal().get());

    if (goal == nullptr)
    {
        OMPL_ERROR("%s: Unknown type of goal", getName().c_str());
        return base::PlannerStatus::UNRECOGNIZED_GOAL_TYPE;
    }

    // Add the valid start states as milestones
    while (const base::State *st = pis_.nextStart())
        startM_.push_back(addMilestone(si_->cloneState(st)));

    if (startM_.empty())
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    if (!goal->couldSample())
    {
        OMPL_ERROR("%s: Insufficient states in sampleable goal region", getName().c_str());
        return base::PlannerStatus::INVALID_GOAL;
    }

    // Ensure there is at least one valid goal state
    if (goal->maxSampleCount() > goalM_.size() || goalM_.empty())
    {
        const base::State *st = goalM_.empty() ? pis_.nextGoal(ptc) : pis_.nextGoal();
        if (st != nullptr)
            goalM_.push_back(addMilestone(si_->cloneState(st)));

        if (goalM_.empty())
        {
            OMPL_ERROR("%s: Unable to find any valid goal states", getName().c_str());
            return base::PlannerStatus::INVALID_GOAL;
        }
    }

    unsigned long int nrStartStates = boost::num_vertices(g_);
    OMPL_INFORM("%s: Starting planning with %lu states already in datastructure", getName().c_str(), nrStartStates);

    // Reset addedNewSolution_ member and create solution checking thread
    addedNewSolution_ = false;
    base::PathPtr sol;
    std::thread slnThread([this, &ptc, &sol] { checkForSolution(ptc, sol); });

    // construct new planner termination condition that fires when the given ptc is true, or a solution is found
    base::PlannerTerminationCondition ptcOrSolutionFound([this, &ptc] { return ptc || addedNewSolution(); });

    constructRoadmap(ptcOrSolutionFound);

    // Ensure slnThread is ceased before exiting solve
    slnThread.join();

    OMPL_INFORM("%s: Created %u states", getName().c_str(), boost::num_vertices(g_) - nrStartStates);

    if (sol)
    {
        base::PlannerSolution psol(sol);
        psol.setPlannerName(getName());
        // if the solution was optimized, we mark it as such
        psol.setOptimized(opt_, bestCost_, addedNewSolution());
        pdef_->addSolutionPath(psol);
    }
    else
    {
        // Return an approximate solution.
        ompl::base::Cost diff = constructApproximateSolution(startM_, goalM_, sol);
        if (!opt_->isFinite(diff))
        {
            OMPL_INFORM("Closest path is still start and goal");
            return base::PlannerStatus::TIMEOUT;
        }
        OMPL_INFORM("Using approximate solution, heuristic cost-to-go is %f", diff.value());
        pdef_->addSolutionPath(sol, true, diff.value(), getName());
        return base::PlannerStatus::APPROXIMATE_SOLUTION;
    }

    return sol ? base::PlannerStatus::EXACT_SOLUTION : base::PlannerStatus::TIMEOUT;
}

void ompl::control::SMR::getTransitions(const std::vector<Vertex> &vertices, const Vertex s, const int u, std::map<Vertex*, double> &transitions)
{
    transitions.clear();
    ompl::base::State* qState;
    si_->allocState(qState);

    double delta_0 = 0.5;
    double r_0 = 2.5;
    double sigma_delta_0 = 0.1;
    double sigma_delta_1 = 0.2;
    double sigma_r_0 = 0.5;
    double sigma_r_1 = 1.0;
    auto xState = stateProperty_[s]->as<ompl::base::SE2StateSpace::StateType>();
    auto tState 
    for (int i = 0; i < m_; i++) 
    {
        // Generate sample transitions for needle
        // Sample distance delta and radius r from Gaussian distributions
        double delta = 0.0;
        double r = 0.0;
        if (u == 0) {
            delta = rng_.gaussian(delta_0, sigma_delta_0);
            r = rng_.gaussian(r_0, sigma_r_0);
        } else {
            delta = rng_.gaussian(delta_0, sigma_delta_1);
            r = rng_.gaussian(r_0, sigma_r_1);
        }

        // Calculate new sample state q
        double theta = xState->getYaw();
        int sign = (u == 0) ? 1 : -1;
        double theta_new = theta + sign * delta / r;
        double x_new = xState->getX() - sign * r * std::sin(theta) + sign * r * std::sin(theta_new); 
        double y_new = yState->getY() + sign * r * std::cos(theta) - sign * r * std::cos(theta_new);
    
        qState->as<ompl::base::SE2StateSpace::StateType>()->setX(x_new);
        qState->as<ompl::base::SE2StateSpace::StateType>()->setY(y_new);
        qState->as<ompl::base::SE2StateSpace::StateType>()->setYaw(theta_new);

        // Check if the path from x to q is valid
        bool valid = si_->checkMotion(xState, qState);
        std::lock_guard<std::mutex> _(graphMutex_);
        Vertex t = boost::add_vertex(g_);
        if (valid) {
            // TODO: Find Vertex t that minimizes dist(q, t) and save its state to tState
            // Use Nearest Neighbors
        } else {
            // TODO: Set Vertex t to obstacle state
        }
        
        // Find if transitions already contains Vertex t
        if (transitions.contains(*t)) {
            // Update probability of transitions from Vertex t
            transitions[*t] = transitions[*t] + 1.0/m_;
        } else {
            // Initialize probability of transitions from Vertex t
            transitions[*t] = 1.0/m_;
        }
    }

    return transitionProb;
}

void ompl::control::SMR::constructRoadmap(const base::PlannerTerminationCondition &ptc)
{
    if (!isSetup())
        setup();
    if (!sampler_)
        sampler_ = si_->allocValidStateSampler();
    if (!simpleSampler_)
        simpleSampler_ = si_->allocStateSampler();

    // SMR Learning phase
    base::State *qState = si_->allocState();
    while (milestoneCount() < n_) 
    {
        iterations_++;
        bool found = false;
        while (!found) 
        {
            unsigned int attempts = 0;
            do
            {
                found = sampler_->sample(qState);
                attempts++;
            } while (attempts < magic::FIND_VALID_STATE_ATTEMPTS_WITHOUT_TERMINATION_CHECK && !found);
        }
        if (found) {
            std::lock_guard<std::mutex> _(graphMutex_);

            Vertex m = boost::add_vertex(g_);
            stateProperty_[m] = si_->cloneState(qState);
            totalConnectionAttemptsProperty_[m] = 1;
            successfulConnectionAttemptsProperty_[m] = 0;
            nn_->add(m);
        }
    }
    si_->freeState(qState);

    const std::vector<Vertex> &vertices = new std::vector<>();
    nn_->list(vertices);
    const int upperBound = siC_->getControlSpace()->as<ompl::control::DiscreteControlSpace>()->getUpperBound();
    const int lowerBound = siC_->getControlSpace()->as<ompl::control::DiscreteControlSpace>()->getLowerBound();

    std::map<Vertex*, double> transitions;
    foreach (Vertex s, vertices)
        for (int u = lowerBound; u <= upperBound; u++) {
            getTransitions(vertices, s, u, transitions);
            for (const auto &[t, p] : transitions) {
                // TODO: Add (s, t, p) to the set of edges
            }
        }

    // SMR Query phase
    // TODO: Implement Markov decision process to find optimal policy
}

ompl::control::SMR::Vertex ompl::control::SMR::addMilestone(base::State *state)
{
    std::lock_guard<std::mutex> _(graphMutex_);

    Vertex m = boost::add_vertex(g_);
    stateProperty_[m] = state;
    totalConnectionAttemptsProperty_[m] = 1;
    successfulConnectionAttemptsProperty_[m] = 0;

    // Initialize to its own (dis)connected component.
    disjointSets_.make_set(m);

    // Which milestones will we attempt to connect to?
    const std::vector<Vertex> &neighbors = connectionStrategy_(m);

    foreach (Vertex n, neighbors)
        if (connectionFilter_(n, m))
        {
            totalConnectionAttemptsProperty_[m]++;
            totalConnectionAttemptsProperty_[n]++;
            if (si_->checkMotion(stateProperty_[n], stateProperty_[m]))
            {
                successfulConnectionAttemptsProperty_[m]++;
                successfulConnectionAttemptsProperty_[n]++;
                const base::Cost weight = opt_->motionCost(stateProperty_[n], stateProperty_[m]);
                const Graph::edge_property_type properties(weight);
                boost::add_edge(n, m, properties, g_);
                uniteComponents(n, m);
            }
        }


    nn_->add(m);

    return m;
}

void ompl::control::SMR::uniteComponents(Vertex m1, Vertex m2)
{
    disjointSets_.union_set(m1, m2);
}

bool ompl::control::SMR::sameComponent(Vertex m1, Vertex m2)
{
    return boost::same_component(m1, m2, disjointSets_);
}

ompl::base::Cost ompl::control::SMR::constructApproximateSolution(const std::vector<Vertex> &starts,
                                                                    const std::vector<Vertex> &goals,
                                                                    base::PathPtr &solution)
{
    std::lock_guard<std::mutex> _(graphMutex_);
    base::Goal *g = pdef_->getGoal().get();
    base::Cost closestVal(opt_->infiniteCost());
    bool approxPathJustStart = true;

    foreach (Vertex start, starts)
    {
        foreach (Vertex goal, goals)
        {
            base::Cost heuristicCost(costHeuristic(start, goal));
            if (opt_->isCostBetterThan(heuristicCost, closestVal))
            {
                closestVal = heuristicCost;
                approxPathJustStart = true;
            }
            if (!g->isStartGoalPairValid(stateProperty_[goal], stateProperty_[start]))
            {
                continue;
            }
            base::PathPtr p;
            boost::vector_property_map<Vertex> prev(boost::num_vertices(g_));
            boost::vector_property_map<base::Cost> dist(boost::num_vertices(g_));
            boost::vector_property_map<base::Cost> rank(boost::num_vertices(g_));

            try
            {
                // Consider using a persistent distance_map if it's slow
                boost::astar_search(
                    g_, start, [this, goal](Vertex v) { return costHeuristic(v, goal); },
                    boost::predecessor_map(prev)
                        .distance_map(dist)
                        .rank_map(rank)
                        .distance_compare(
                            [this](base::Cost c1, base::Cost c2) { return opt_->isCostBetterThan(c1, c2); })
                        .distance_combine([this](base::Cost c1, base::Cost c2) { return opt_->combineCosts(c1, c2); })
                        .distance_inf(opt_->infiniteCost())
                        .distance_zero(opt_->identityCost())
                        .visitor(AStarGoalVisitor<Vertex>(goal)));
            }
            catch (AStarFoundGoal &)
            {
            }

            Vertex closeToGoal = start;
            for (auto vp = vertices(g_); vp.first != vp.second; vp.first++)
            {
                // We want to get the distance of each vertex to the goal.
                // Boost lets us get cost-to-come, cost-to-come+dist-to-goal,
                // but not just dist-to-goal.
                ompl::base::Cost dist_to_goal(costHeuristic(*vp.first, goal));
                if (opt_->isFinite(rank[*vp.first]) && opt_->isCostBetterThan(dist_to_goal, closestVal))
                {
                    closeToGoal = *vp.first;
                    closestVal = dist_to_goal;
                    approxPathJustStart = false;
                }
            }
            if (closeToGoal != start)
            {
                auto p(std::make_shared<PathGeometric>(si_));
                for (Vertex pos = closeToGoal; prev[pos] != pos; pos = prev[pos])
                    p->append(stateProperty_[pos]);
                p->append(stateProperty_[start]);
                p->reverse();

                solution = p;
            }
        }
    }
    if (approxPathJustStart)
    {
        return opt_->infiniteCost();
    }
    return closestVal;
}

ompl::base::PathPtr ompl::control::SMR::constructSolution(const Vertex &start, const Vertex &goal)
{
    std::lock_guard<std::mutex> _(graphMutex_);
    boost::vector_property_map<Vertex> prev(boost::num_vertices(g_));

    try
    {
        // Consider using a persistent distance_map if it's slow
        boost::astar_search(
            g_, start, [this, goal](Vertex v) { return costHeuristic(v, goal); },
            boost::predecessor_map(prev)
                .distance_compare([this](base::Cost c1, base::Cost c2) { return opt_->isCostBetterThan(c1, c2); })
                .distance_combine([this](base::Cost c1, base::Cost c2) { return opt_->combineCosts(c1, c2); })
                .distance_inf(opt_->infiniteCost())
                .distance_zero(opt_->identityCost())
                .visitor(AStarGoalVisitor<Vertex>(goal)));
    }
    catch (AStarFoundGoal &)
    {
    }

    if (prev[goal] == goal)
        throw Exception(name_, "Could not find solution path");

    auto p(std::make_shared<PathGeometric>(si_));
    for (Vertex pos = goal; prev[pos] != pos; pos = prev[pos])
        p->append(stateProperty_[pos]);
    p->append(stateProperty_[start]);
    p->reverse();

    return p;
}

void ompl::control::SMR::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    // Explicitly add start and goal states:
    for (unsigned long i : startM_)
        data.addStartVertex(
            base::PlannerDataVertex(stateProperty_[i], const_cast<SMR *>(this)->disjointSets_.find_set(i)));

    for (unsigned long i : goalM_)
        data.addGoalVertex(
            base::PlannerDataVertex(stateProperty_[i], const_cast<SMR *>(this)->disjointSets_.find_set(i)));

    // Adding edges and all other vertices simultaneously
    foreach (const Edge e, boost::edges(g_))
    {
        const Vertex v1 = boost::source(e, g_);
        const Vertex v2 = boost::target(e, g_);
        data.addEdge(base::PlannerDataVertex(stateProperty_[v1]), base::PlannerDataVertex(stateProperty_[v2]));

        // Add the reverse edge, since we're constructing an undirected roadmap
        data.addEdge(base::PlannerDataVertex(stateProperty_[v2]), base::PlannerDataVertex(stateProperty_[v1]));

        // Add tags for the newly added vertices
        data.tagState(stateProperty_[v1], const_cast<SMR *>(this)->disjointSets_.find_set(v1));
        data.tagState(stateProperty_[v2], const_cast<SMR *>(this)->disjointSets_.find_set(v2));
    }
}

ompl::base::Cost ompl::control::SMR::costHeuristic(Vertex u, Vertex v) const
{
    return opt_->motionCostHeuristic(stateProperty_[u], stateProperty_[v]);
}