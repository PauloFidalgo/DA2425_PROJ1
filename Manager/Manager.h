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
#include <limits>
#include <algorithm>

class Manager {
private:
    Graph<Node*> graph;
    std::unordered_map<int, Node *> nodes;
    void readBatch() const;
public:
    Manager();
    void drive_only_independent_route(int source, int destination) const;
};

#endif //MANAGER_H
