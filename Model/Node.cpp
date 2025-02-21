//
// Created by paulo on 21-02-2025.
//

#include "../Model/Node.h"

#include <string>
#include <utility>


Node::Node(int id, std::string  location,std::string  code, bool parking) : id(id), location(std::move(location)), code(std::move(code)), parking(parking) {};




