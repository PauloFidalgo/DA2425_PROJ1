cmake_minimum_required(VERSION 3.30)
project(Project_1)

set(CMAKE_CXX_STANDARD 20)

add_executable(Project_1 main.cpp
        Parser/Parser.cpp
        Parser/Parser.h
        Model/Graph.h
        Model/MutablePriorityQueue.h
        Model/Node.cpp
        Model/Node.h
        Manager/Manager.cpp
        Manager/Manager.h)
