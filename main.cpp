/*
Mayank Kishore - Tesla Supercharger Challenge
Solution Description:
    GOAL: Construct the minimum path through the tesla network of supercharging stations

    Main Idea:
        - Build out a graph starting at the starting node
        - Create edge weights based on travel_time + charging_time
        - Travese the graph to determine the shortest path

    Build an adjacency list:
        - For each node, calculate the great circle distance between every other node
        - If within the max range, then add node to adjacency list for that specific node
    Build the graph based on the starting node:
        - Start at the starting node and conduct a BFS through the adjacency list
        - Kept track of the seen nodes that get popped off the queue
        - Avoided cycles by keeping track of nodes reachable by the parent node
            - Idea is that if the node is reachable by the parent node, there is already a route added to the graph
        - Added a distance heuristic to ensure that the node is not deviating too far from the projected path
        - Cost (edge weight) was assigned in one of two ways:
            - If next charger charges faster than the source, only add enough charge to reach the next charger
            - If the next charger charges slower than the source, charge fully and then go to that charger
            - Chose this cost algorithm to minimize the number of stops and minmize time spent at each stop
    Run Dijkstra's:
        - Finally, with a built graph, we can run Dijkstra's to determine the shortest path
        - Build the path and output the built path
    
    RESULTS:
        - Was able to successfully create a very fast and fairly accurate approximation algorithm
        - Used a distance heuristic in the graph building stage to significantly improve the runtime
        - The distance heurisitc is an approximate guess of how far out of path the vehicle can deviate
        - Consistenly ended up with results are close to the checker_osx

    CONCLUSION
        - Really enjoyed working on this challenge as it mirrors a real life problem Tesla faces
        - Am proud of my results and the optimizations that I introduced with this algorithm
        - Would have liked to more robustly check my distance heurisitc number
        - Traded building out every potential path for a more optimal approach
        - This challenge was really exciting and I would love to work on more real world challenge's at Tesla!
*/


#include "network.h"
#include <vector>
#include <cmath>
#include <unordered_map>
#include <unordered_set>
#include <queue>
using namespace std;

#define EARTH_RADIUS 6356.752; // The earth radius is a constant

class optimal_route {
    // Both of these are taken as inputs in the constructor and can be changed if necessary
    const int max_charge;
    const int max_speed;

    public:
        optimal_route(int charge, int speed) : max_charge(charge), max_speed(speed) {}

        // Building out the adjacency list 
        void network_of_chargers(const array<row, 303> &network) {
            // Mapping of node name -> row for ease of use
            vector<row> nodes;
            for (const row &node : network) {
                name_to_node[node.name] = node;
                nodes.push_back(node);
            }
            for (int i = 0; i < network.size(); i++) {
                row node = network[i];
                for (int j = i; j < network.size(); j++) {
                    row adjacent_node = network[j];
                    if (node.name == adjacent_node.name) {
                        continue;
                    }
                    long double distance_to_station = great_circle_distance(node.lat, node.lon, adjacent_node.lat, adjacent_node.lon);
                    if (distance_to_station <= max_charge) {
                        if (adjacency_list.find(node.name) == adjacency_list.end()) 
                            adjacency_list.insert({node.name, {}});
                        if (adjacency_list.find(adjacent_node.name) == adjacency_list.end())
                            adjacency_list.insert({adjacent_node.name, {}});
                        adjacency_list[node.name].push_back(adjacent_node);
                        adjacency_list[adjacent_node.name].push_back(node);
                    }
                }
            }
        }

        // Find the shortest path
        void find_shortest_path(string start_node, string end_node) {
            build_potential_paths(start_node, end_node);
            unordered_map<string,double> shortest_path_tree = djikstra(start_node);
            gen_path(start_node, end_node);
        }

    private:
        unordered_map<string, vector<row> > adjacency_list;
        unordered_map<string, row> name_to_node; // mapping of name to node
        unordered_map<string, unordered_map<string, double> > graph;
        unordered_map<string, string> shortest_path;

        double deg2rad(double deg) {
            return (deg * M_PI / 180.0);
        }
        
        // Using the Haversine formula: https://en.wikipedia.org/wiki/Great-circle_distance
        long double great_circle_distance(long double latitude1, long double longitude1, long double latitude2, long double longitude2) {
            long double lat1 = deg2rad(latitude1);
            long double lon1 = deg2rad(longitude1);
            long double lat2 = deg2rad(latitude2);
            long double lon2 = deg2rad(longitude2);

            long double d_lat = abs(lat1 - lat2);
            long double d_lon = abs(lon1 - lon2);

            long double a = pow(sin(d_lat / 2), 2) + cos(lat1) * cos(lat2) * pow(sin(d_lon / 2), 2);

            long double d_sigma = 2 * asin(sqrt(a));

            return d_sigma * EARTH_RADIUS;
        }

        // Draws a line of best fit from start to end and gets shortest distance from point to line
        long double distance_heuristic(row start, row end, row next_node) {
            // y = m1 * x + b1
            double m1 = (end.lon - start.lon) / (end.lat - start.lat);
            double b1 = start.lon - (m1 * (start.lat));
            // y = m2 * x + b2 (perpendicular line)
            double m2 = -1 * (1/m1);
            double b2 = next_node.lon - (m2 * next_node.lat);
            double x_on_line = (b2 - b1) / (m1 - m2);
            double y_on_line = (m1 * x_on_line) + b1;

            // distance between charger and line of best fit
            long double distance = great_circle_distance(x_on_line, y_on_line, next_node.lat, next_node.lon);
            return distance;
        }

        // Building the graph
        void build_potential_paths(string start_node, string end_node) {
            row start = name_to_node[start_node];
            row end = name_to_node[end_node];
            long double start_to_end_distance = great_circle_distance(start.lat, start.lon, end.lat, end.lon);
            double next_charge = max_charge;
            row curr_node;
            unordered_set<string> parents{start.name};
            unordered_set<string> seen;
            tuple<row, double, unordered_set<string> > contents;
            queue<tuple<row, double, unordered_set<string> >> reachable;
            reachable.push(make_tuple(start, next_charge, parents));

            // BFS through all of the nodes to build paths
            while (reachable.size() > 0) {
                contents = reachable.front();
                reachable.pop();
                curr_node = get<0>(contents);       // current node to bfs through
                next_charge = get<1>(contents);     // the charge the car is arriving to this node with
                parents = get<2>(contents);         // the parent nodes to avoid loops
                seen.insert(curr_node.name);
                if (graph.find(curr_node.name) == graph.end())
                    graph.insert({curr_node.name, {}});
                for (const row &next_node : adjacency_list[curr_node.name]) {
                    long double distance = distance_heuristic(start, end, next_node);
                    long double allowed_deviation = ((start_to_end_distance / 20) + max_charge);
                    
                    // If seen before, or if in parents node, or if outside allowed deviation, don't reassign a cost
                    if ((seen.find(next_node.name) == seen.end()) && (parents.find(next_node.name) == parents.end()) && (distance < allowed_deviation)) {
                        long double distance_to_next = great_circle_distance(curr_node.lat, curr_node.lon, next_node.lat, next_node.lon);
                        double travel_time = (distance_to_next / max_speed);
                        double charge_needed = 0;
                        double curr_charge = next_charge;
                        if (curr_node.name == start.name) {                         // edge case for starting node
                            charge_needed = 0; 
                            curr_charge = max_charge - distance_to_next;
                        } else if (next_node.rate > curr_node.rate) {               // min case (fill minimum needed to reach next node)
                            charge_needed = distance_to_next - curr_charge;
                            curr_charge = 0;
                        } else {                                                    // max case (fill up completely and then drive)
                            charge_needed = max_charge - curr_charge;
                            curr_charge = max_charge - distance_to_next;
                        }
                        if (charge_needed < 0)                                      // if it is negative, then there is no need to stop
                            continue;
                        double charge_time = (charge_needed) / curr_node.rate;
                        double total_time = travel_time + charge_time;
                        unordered_set<string> temp_parents = parents;
                        // Keeping track of parent paths to avoid cycles
                        for (const row &parent_node : adjacency_list[curr_node.name]) {
                            temp_parents.insert(parent_node.name);
                        } 
                        reachable.push(make_tuple(next_node, curr_charge, temp_parents));
                        if (graph.find(curr_node.name) != graph.end()) {
                            if (graph[curr_node.name].find(next_node.name) == graph[curr_node.name].end()) {
                                graph[curr_node.name][next_node.name] = -1 * numeric_limits<double>::infinity();
                            }
                        }
                        // Ensuring the total time is representative
                        graph[curr_node.name][next_node.name] = max(graph[curr_node.name][next_node.name], total_time);
                    }
                }
            }
        }

        // Getting the current minimum node for dijkstra's
        string min_value(unordered_set<string> visited, unordered_map<string,double> distance){
            string min_val = "";
            double minimum = numeric_limits<double>::infinity();
            for (const string &v : visited) {
                if (minimum > distance[v]) {
                    min_val = v;
                    minimum = distance[v];
                }
            }
            return min_val;
        }

        // Running dijksta's given a starting node to generate all paths
        unordered_map<string,double> djikstra(string start_node) {
            unordered_map<string,double> distance;
            unordered_set<string> visited;
            
            distance[start_node] = 0;
            for (const auto &node : graph) {
                if (node.first != start_node) {
                    distance[node.first] = numeric_limits<double>::infinity();
                }
                visited.insert(node.first);
            }
            
            while (visited.size() > 0){
                string min_node = min_value(visited, distance);
                visited.erase(min_node);
                for (const auto &neighbor : graph[min_node]) {
                    if (graph[min_node].find(neighbor.first) == graph[min_node].end()) {
                        continue;
                    }
                    double alternate_path = distance[min_node] + neighbor.second;
                    if (alternate_path < distance[neighbor.first]) {
                        distance[neighbor.first] = alternate_path;
                        shortest_path[neighbor.first] = min_node;   // Used to keep track of shortest path in reverse
                    }
                }
            }
            return distance;
        }

        // Building the shortest path from start to end
        void gen_path(string start_node, string end_node) {
            vector<string> locations;
            vector<long double> times;
            string curr = end_node; 
            while (curr != start_node) { // build the path until the start value
                locations.insert(locations.begin(), curr);
                if (shortest_path.find(curr) == shortest_path.end()) { // if it gets to the node with no parent, then it is not possible
                    std::cout << "Error: Invalid Input" << std::endl; 
                    return;  
                }
                curr = shortest_path[curr]; // go to the parent of the current
            } 
            locations.insert(locations.begin(), start_node);
            calc_charging_times(locations, times);

            // Printing out the shortest path and the corresponding times
            cout << locations[0] << ", ";
            for (auto i = 1; i < times.size(); i++){
                cout << locations[i] << ", " << to_string(times[i]) << ", ";
            }
            cout << locations[locations.size() - 1];
        }

        // Calculate the time spent at each charger
        void calc_charging_times(vector<string> path, vector<long double> &times) {
            for (int i = path.size() - 1; i > 0; i--) {
                row one = name_to_node[path[i-1]];
                row two = name_to_node[path[i]];
                long double distance = great_circle_distance(one.lat, one.lon, two.lat, two.lon);
                times.insert(times.begin(), graph[path[i-1]][path[i]] - (distance / max_speed));
            }
        }        
};

int main(int argc, char** argv)
{
    if (argc != 3)
    {
        std::cout << "Error: requires initial and final supercharger names" << std::endl;        
        return -1;
    }
    
    std::string initial_charger_name = argv[1];
    std::string goal_charger_name = argv[2];

    optimal_route object(320, 105);
    object.network_of_chargers(network);
    object.find_shortest_path(initial_charger_name, goal_charger_name);
    return 0;
}