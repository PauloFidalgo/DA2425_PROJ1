//
// Created by paulo on 21-02-2025.
//

#ifndef NODE_H
#define NODE_H
#include <string>


class Node {
    std::string location;
    int id;
    std::string code;
    bool parking;

public:
    Node(int id, std::string  location, std::string  code, bool parking);
    int getId() const { return id; };
    bool getParking() const { return parking; };
};

#endif //NODE_H
