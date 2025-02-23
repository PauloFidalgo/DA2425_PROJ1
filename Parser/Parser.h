//
// Created by paulo on 21-02-2025.
//

#ifndef PARSER_H
#define PARSER_H
#include "../Model/Graph.h"
#include "../Model/Node.h"
#include <fstream>
#include <iostream>
#include <utility>
#include <sstream>
#include <fstream>
#include <unordered_map>

using namespace std;

class Parser {
private:
    std::string prefix;
    unordered_map<std::string, Node*> nodes;
    unordered_map<int, Node *> nodes_int;
    void readLocations(Graph<Node *> &graph, const string &fileName);
    void readDistances(Graph<Node*> &graph, const string &fileName);

public:
    explicit Parser(const std::string &choice);
    Graph<Node*> parseGraph();
    unordered_map<int, Node *> getNodesInt() const;
};

#endif //PARSER_H
