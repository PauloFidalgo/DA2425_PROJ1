//
// Created by paulo on 21-02-2025.
//

#include "Parser.h"


void Parser::readLocations(Graph<Node*> &graph, const string &fileName) {
    fstream iff;
    try {
        iff.open(fileName, ios::in);
        std::string line, location, str_id, code, str_parking;
        int id;
        bool parking;

        getline(iff, line);

        while (getline(iff, line)) {
            stringstream iss(line);
            getline(iss, location, ',');
            getline(iss, str_id, ',');
            getline(iss, code, ',');
            getline(iss, str_parking);

            id = stoi(str_id);
            parking = stoi(str_parking) != 0;

            Node* node = new Node(id, location, code, parking);
            nodes[code] = node;
            graph.addVertex(node);
        }
    } catch (const ifstream::failure& e) {
        cout << "Error opening file" << endl;
    }

    iff.close();
}

void Parser::readDistances(Graph<Node*> &graph, const string &fileName) {
    fstream iff;
    try {
        iff.open(fileName, ios::in);
        std::string line, code1, code2, str_driving, str_walking;
        int driving, walking;

        getline(iff, line);

        while (getline(iff, line)) {
            stringstream iss(line);
            getline(iss, code1, ',');
            getline(iss, code2, ',');
            getline(iss, str_driving, ',');
            getline(iss, str_walking);

            try {
                driving = (str_driving != "X") ? std::stoi(str_driving) : -1;
            } catch (...) {
                driving = -1;
            }

            try {
                walking = (str_walking != "X") ? std::stoi(str_walking) : -1;
            } catch (...) {
                walking = -1;
            }

            auto starting_point = nodes.find(code1);
            auto ending_point = nodes.find(code2);
            if (starting_point != nodes.end() && ending_point != nodes.end()) {
                graph.addBidirectionalEdge(starting_point->second, ending_point->second, {walking, driving});
            }

        }
    } catch (const ifstream::failure& e) {
        cout << "Error opening file" << endl;
    }

    iff.close();
}

Graph<Node *> Parser::parseGraph() {
    Graph<Node *> graph;
    readLocations(graph, "../Dataset/Locations.csv");
    readDistances(graph, "../Dataset/Distances.csv");

    return graph;
}
