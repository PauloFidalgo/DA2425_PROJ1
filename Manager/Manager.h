//
// Created by paulo on 21-02-2025.
//

#ifndef MANAGER_H
#define MANAGER_H
#include "../Model/Graph.h"
#include "../Model/Node.h"
#include "../Parser/Parser.h"
#include <queue>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <limits>
#include <algorithm>

/**
 * @class Manager
 * @brief Manages route planning functionalities for a graph-based navigation system.
 */
class Manager {
private:
    mutable bool first_time_writing = true; ///< Flag to indicate if it's the first time writing to the file.
    Graph<Node *> graph; ///< The graph representing the navigation system.
    std::unordered_map<int, Node *> nodes; ///< The map of node IDs to Node objects.

    /**
     * @brief Reads the input file and processes each instruction.
     */
    void readBatch() const;

    /**
     * @brief Writes the output to a file.
     * @param str The string to write to the file.
     */
    void writeBatch(const std::string &str) const;
public:
    /**
     * @brief Constructs a Manager object.
     */
    Manager(std::string choice);

    /**
     * @brief Determines the best (fastest) route between a source and destination.
     * @param source The source node ID.
     * @param destination The destination node ID.
     */
    void drive_only_independent_route(int source, int destination) const;

    /**
     * @brief Finds a restricted driving route from a source node to a destination node, avoiding specified nodes and segments, and optionally including a specific node.
     * @param source The source node ID.
     * @param destination The destination node ID.
     * @param avoid_nodes The set of nodes to avoid.
     * @param avoid_segments The list of segments to avoid.
     * @param include_node The node that must be included in the route.
     */
    void restricted_route(int source, int destination, const std::unordered_set<int> &avoid_nodes, const std::vector<std::pair<int, int>> &avoid_segments, int include_node) const;
    
    /**
     * @brief Finds the best route from a source node to a destination node, considering both driving and walking.
     *
     * This function calculates the optimal route from a source node to a destination node by first driving to a parking node
     * and then walking to the destination. It considers a maximum walking time and avoids specified nodes and segments.
     *
     * @param source The ID of the source node.
     * @param destination The ID of the destination node.
     * @param max_walking_time The maximum allowed walking time.
     * @param avoid_nodes A set of node IDs to avoid in the route.
     * @param avoid_segments A vector of node ID pairs representing segments to avoid in the route.
     */
    void drive_and_walk_route(int source, int destination, int max_walking_time, const std::unordered_set<int>& avoid_nodes, const std::vector<std::pair<int, int>>& avoid_segments) const;
};

#endif //MANAGER_H
