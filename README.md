# Supercharger Route Planner
## Solution Description:
### GOAL: Construct the minimum path through the network of supercharging stations
### Main Idea:
- Build out a graph starting at the starting node
- Create edge weights based on travel_time + charging_time
- Travese the graph to determine the shortest path
### Build an adjacency list:
- For each node, calculate the great circle distance between every other node
- If within the max range, then add node to adjacency list for that specific node
### Build the graph based on the starting node:
- Start at the starting node and conduct a BFS through the adjacency list
- Kept track of the seen nodes that get popped off the queue
- Avoided cycles by keeping track of nodes reachable by the parent node
    - Idea is that if the node is reachable by the parent node, there is already a route added to the graph
- Added a distance heuristic to ensure that the node is not deviating too far from the projected path
- Cost (edge weight) was assigned in one of two ways:
    - If next charger charges faster than the source, only add enough charge to reach the next charger
    - If the next charger charges slower than the source, charge fully and then go to that charger
    - Chose this cost algorithm to minimize the number of stops and minmize time spent at each stop
### Run Dijkstra's:
- Finally, with a built graph, we can run Dijkstra's to determine the shortest path
- Build the path and output the built path

### RESULTS:
- Was able to successfully create a very fast and fairly accurate approximation algorithm
- Used a distance heuristic in the graph building stage to significantly improve the runtime
- The distance heurisitc is an approximate guess of how far out of path the vehicle can deviate
- Consistenly ended up with results are close to the checker_osx
