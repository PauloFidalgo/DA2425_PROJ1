//
// Created by paulo on 21-02-2025.
//

#include "Manager.h"

Manager::Manager() {
    Parser parser;
    this->graph = parser.parseGraph();
}
