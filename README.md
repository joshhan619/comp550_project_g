** TODO **

ProjectGNeedle.cpp
1. Planning code + environment setup

SMR.cpp
1. Find the closest vertex t to the sampled point q using nearest neighbors (getTransitions)
2. Define an obstacle state and set vertex t to it if the path from s to q is not collision free (getTransitions)
3. For each transition obtained from getTransitions, add an edge to the graph (constructRoadmap)
4. Implement the query phase with Markov Decision Process (constructRoadmap)