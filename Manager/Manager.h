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

class Manager {
private:
    mutable bool first_time_writing = true;
    Graph<Node*> graph;
    std::unordered_map<int, Node *> nodes;
    void readBatch() const;
    void writeBatch(const std::string &str) const;
public:
    Manager(std::string choice);
    void drive_only_independent_route(int source, int destination) const;
    void restricted_route(int source, int destination, const std::unordered_set<int> &avoid_nodes, const std::vector<std::pair<int, int>> &avoid_segments, int include_node) const;
    void drive_and_walk_route(int source, int destination, int max_walking_time, const std::unordered_set<int>& avoid_nodes, const std::vector<std::pair<int, int>>& avoid_segments) const;
};

#endif //MANAGER_H
