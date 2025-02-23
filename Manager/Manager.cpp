//
// Created by paulo on 21-02-2025.
//

#include "Manager.h"

#include <unordered_set>

Manager::Manager() {
    Parser parser = Parser();
    this->graph = parser.parseGraph();
    this->nodes = parser.getNodesInt();

    readBatch();
}

void extractPattern(const std::string &line, std::string &mode)
{
    std::istringstream lineStream(line);
    std::string key;
    if (std::getline(lineStream, key, ':'))
    {
        std::getline(lineStream, mode);
    }
}

void Manager::readBatch() const
{
    fstream batchFile;
    batchFile.open("../Batch/input.txt", ios::in);
    if (!batchFile.is_open())
    {
        cout << "Error opening file" << endl;
        return;
    }

    string line;
    while (getline(batchFile, line))
    {
        string mode;

        if (line.find("Mode:") != std::string::npos) {
            extractPattern(line, mode);
            if (mode == "driving")
            {
                string tmp;
                int int_source = -1, int_destination = -1;
                std::unordered_set<int> avoid_nodes;
                std::vector<std::pair<int, int>> avoid_segments;
                int include_node = -1;

                if (getline(batchFile, line) && line.find("Source:") != std::string::npos)
                {
                    extractPattern(line, tmp);
                    int_source = stoi(tmp);
                }

                if (getline(batchFile, line) && line.find("Destination:") != std::string::npos)
                {
                    extractPattern(line, tmp);
                    int_destination = stoi(tmp);
                }

                if (getline(batchFile, line) && line.find("AvoidNodes:") != std::string::npos)
                {
                    extractPattern(line, tmp);
                    if (!tmp.empty())
                    {
                        std::istringstream ss(tmp);
                        std::string node;
                        while (std::getline(ss, node, ','))
                        {
                            avoid_nodes.insert(stoi(node));
                        }
                    }
                    if (getline(batchFile, line) && line.find("AvoidSegments:") != std::string::npos)
                    {
                        extractPattern(line, tmp);

                        if (!tmp.empty())
                        {
                            std::istringstream ss(tmp);
                            std::string segment;
                            while (std::getline(ss, segment, ')'))
                            {
                                if (!segment.empty() && segment[0] == ',')
                                    segment = segment.substr(1);
                                segment.erase(std::remove(segment.begin(), segment.end(), '('), segment.end());
                                segment.erase(std::remove(segment.begin(), segment.end(), ' '), segment.end()); // Remove any spaces
                                if (!segment.empty())
                                {
                                    std::istringstream segment_ss(segment);
                                    std::string node1, node2;
                                    if (std::getline(segment_ss, node1, ',') && std::getline(segment_ss, node2, ','))
                                    {
                                        avoid_segments.emplace_back(stoi(node1), stoi(node2));
                                    }
                                }
                            }
                        }
                    }

                    if (getline(batchFile, line) && line.find("IncludeNode:") != std::string::npos)
                    {
                        extractPattern(line, tmp);
                        if (!tmp.empty())
                        {
                            include_node = stoi(tmp);
                        }
                    }
                } 


                if (avoid_nodes.empty() && avoid_segments.empty() && include_node == -1)
                {
                    drive_only_independent_route(int_source, int_destination);
                }
                else
                {
                    restricted_route(int_source, int_destination, avoid_nodes, avoid_segments, include_node);
                }
            }
            else if (mode == "driving-walking")
            {
                string tmp;
                int int_source = -1, int_destination = -1;
                int max_walking_time = -1;
                std::unordered_set<int> avoid_nodes;
                std::vector<std::pair<int, int>> avoid_segments;

                if (getline(batchFile, line) && line.find("Source:") != std::string::npos)
                {
                    extractPattern(line, tmp);
                    int_source = stoi(tmp);
                }

                if (getline(batchFile, line) && line.find("Destination:") != std::string::npos)
                {
                    extractPattern(line, tmp);
                    int_destination = stoi(tmp);
                }

                if (getline(batchFile, line) && line.find("MaxWalkTime:") != std::string::npos)
                {
                    extractPattern(line, tmp);
                    max_walking_time = stoi(tmp);
                }
                if (getline(batchFile, line) && line.find("AvoidNodes:") != std::string::npos) {
                    extractPattern(line, tmp);
                    if (!tmp.empty())
                    {
                        std::istringstream ss(tmp);
                        std::string node;
                        while (std::getline(ss, node, ','))
                        {
                            avoid_nodes.insert(stoi(node));
                        }
                    }
                }
                if (getline(batchFile, line) && line.find("AvoidSegments:") != std::string::npos)
                {
                    extractPattern(line, tmp);

                    if (!tmp.empty())
                    {
                        std::istringstream ss(tmp);
                        std::string segment;
                        while (std::getline(ss, segment, ')'))
                        {
                            if (!segment.empty() && segment[0] == ',')
                                segment = segment.substr(1);
                            segment.erase(std::remove(segment.begin(), segment.end(), '('), segment.end());
                            segment.erase(std::remove(segment.begin(), segment.end(), ' '), segment.end()); // Remove any spaces
                            if (!segment.empty())
                            {
                                std::istringstream segment_ss(segment);
                                std::string node1, node2;
                                if (std::getline(segment_ss, node1, ',') && std::getline(segment_ss, node2, ','))
                                {
                                    avoid_segments.emplace_back(stoi(node1), stoi(node2));
                                }
                            }
                        }
                    }
                }
                drive_and_walk_route(int_source, int_destination, max_walking_time, avoid_nodes, avoid_segments);
            }
        }
    }
    batchFile.close();
}

void Manager::writeBatch(const std::string &str) const
{
    std::ios_base::openmode mode = std::ios::out;
    if (!first_time_writing)
    {
        mode |= std::ios::app;
    }

    std::fstream batchFile;
    batchFile.open("../Batch/output.txt", mode);
    if (!batchFile.is_open())
    {
        std::cout << "Error opening file for writing" << std::endl;
        return;
    }
    batchFile << str;
    batchFile.close();

    first_time_writing = false;
}

std::pair<int, std::vector<int>> dijkstra(const Graph<Node *> &graph, Node *source, Node *destination, const std::unordered_map<int, Node *> &nodes, unordered_set<Node*> &visited, bool walking = false)
{
    auto source_vertex = graph.findVertex(source);
    auto destination_vertex = graph.findVertex(destination);
    if (source_vertex == nullptr || destination_vertex == nullptr)
        return {std::numeric_limits<int>::max(), {}};

    std::unordered_map<int, int> dist;
    std::unordered_map<int, int> prev;
    auto cmp = [&dist](Node *left, Node *right)
    { return dist[left->getId()] > dist[right->getId()]; };
    std::priority_queue<Node *, std::vector<Node *>, decltype(cmp)> pq(cmp);

    for (auto vertex : graph.getVertexSet()) {
        if (visited.find(vertex->getInfo()) == visited.end()) {
            vertex->setVisited(false);
        } else {
            vertex->setVisited(true);
        }
    }


    for (const auto &pair : nodes)
    {
        dist[pair.first] = std::numeric_limits<int>::max();
        prev[pair.first] = -1;
    }

    dist[source->getId()] = 0;
    pq.push(source);

    while (!pq.empty())
    {
        Node *u = pq.top();
        pq.pop();

        if (u->getId() == destination->getId())
            break;

        auto current = graph.findVertex(u);


        if (current)
        {
            current->setVisited(true);
            for (const auto &neighbor : current->getAdj())
            {
                auto v = neighbor->getDest();

                if (v->isVisited()) continue;
                
                int weight = walking ? neighbor->getWalk() : neighbor->getDrive();
                if (weight == -1)
                    continue;

                int alt = dist[u->getId()] + weight;
                if (alt < dist[v->getInfo()->getId()])
                {
                    dist[v->getInfo()->getId()] = alt;
                    prev[v->getInfo()->getId()] = u->getId();
                    pq.push(v->getInfo());
                }
            }
        }
    }

    std::vector<int> path;
    for (int at = destination->getId(); at != -1; at = prev[at])
    {
        path.push_back(at);
    }

    std::reverse(path.begin(), path.end());

    if (path.size() == 1 && path[0] != source->getId())
    {
        return {std::numeric_limits<int>::max(), {}};
    }

    return {dist[destination->getId()], path};
}

void Manager::drive_only_independent_route(int source, int destination) const
{
    auto source_itr = nodes.find(source);
    auto destination_itr = nodes.find(destination);

    std::string output = "Source:" + std::to_string(source) + "\n" +
                         "Destination:" + std::to_string(destination) + "\n";

    if (source_itr == nodes.end() || destination_itr == nodes.end())
    {
        output += "BestDrivingRoute:none\n";
        output += "AlternativeDrivingRoute:none\n\n";
        writeBatch(output);
        return;
    }

    std::unordered_set<Node *> visited;

    std::pair<int, std::vector<int>> best_route = dijkstra(graph, source_itr->second, destination_itr->second, nodes, visited);
    if (best_route.second.empty())
    {
        output += "BestDrivingRoute:none\n";
        output += "AlternativeDrivingRoute:none\n\n";
        writeBatch(output);
        return;
    }

    output += "BestDrivingRoute:";
    for (size_t i = 0; i < best_route.second.size(); ++i)
    {
        output += std::to_string(best_route.second[i]);
        if (i < best_route.second.size() - 1)
            output += ",";
    }
    output += "(" + std::to_string(best_route.first) + ")\n";

    for (auto node : best_route.second)
    {
        if (node != source && node != destination)
        {
            auto v = nodes.find(node);
            if (v != nodes.end())
            {
                visited.insert(v->second);
            }
        }
    }

    std::pair<int, std::vector<int>> alternative_route = dijkstra(graph, source_itr->second, destination_itr->second, nodes, visited);
    if (alternative_route.second.empty())
    {
        output += "AlternativeDrivingRoute:none\n\n";
        writeBatch(output);
        return;
    }

    output += "AlternativeDrivingRoute:";
    for (size_t i = 0; i < alternative_route.second.size(); ++i)
    {
        output += std::to_string(alternative_route.second[i]);
        if (i < alternative_route.second.size() - 1)
            output += ",";
    }
    output += "(" + std::to_string(alternative_route.first) + ")\n\n";

    writeBatch(output);
}


void Manager::restricted_route(int source, int destination, const std::unordered_set<int>& avoid_nodes, const std::vector<std::pair<int, int>>& avoid_segments, int include_node) const
{
    auto source_itr = nodes.find(source);
    auto destination_itr = nodes.find(destination);

    std::string output = "Source:" + std::to_string(source) + "\n" +
                         "Destination:" + std::to_string(destination) + "\n";

    if (source_itr == nodes.end() || destination_itr == nodes.end())
    {
        output += "RestrictedDrivingRoute:none\n\n";
        writeBatch(output);
        return;
    }


    std::unordered_set<Node*> visited;
    for (auto node : avoid_nodes) {
        auto it = nodes.find(node);
        if (it != nodes.end()) {
            visited.insert(it->second);
        }
    }

    // Modify the graph to exclude avoid_segments
    Graph<Node*> modified_graph = graph;
    for (const auto& segment : avoid_segments)
    {
        auto node1_itr = nodes.find(segment.first);
        auto node2_itr = nodes.find(segment.second);
        if (node1_itr != nodes.end() && node2_itr != nodes.end())
        {
            modified_graph.removeEdge(node1_itr->second, node2_itr->second);
        }
    }

    // If include_node is specified, find the route via include_node
    std::pair<int, std::vector<int>> route;
    if (include_node != -1)
    {
        auto include_node_itr = nodes.find(include_node);
        if (include_node_itr == nodes.end())
        {
            output += "RestrictedDrivingRoute:none\n\n";
            writeBatch(output);
            return;
        }

        auto first_leg = dijkstra(modified_graph, source_itr->second, include_node_itr->second, nodes, visited);
        if (first_leg.second.empty())
        {
            output += "RestrictedDrivingRoute:none\n\n";
            writeBatch(output);
            return;
        }

        auto second_leg = dijkstra(modified_graph, include_node_itr->second, destination_itr->second, nodes, visited);
        if (second_leg.second.empty())
        {
            output += "RestrictedDrivingRoute:none\n\n";
            writeBatch(output);
            return;
        }

        // Combine the two legs
        route.first = first_leg.first + second_leg.first;
        route.second = first_leg.second;
        route.second.insert(route.second.end(), second_leg.second.begin() + 1, second_leg.second.end());
    }
    else
    {
        route = dijkstra(modified_graph, source_itr->second, destination_itr->second, nodes, visited);
    }

    if (route.second.empty())
    {
        output += "RestrictedDrivingRoute:none\n\n";
        writeBatch(output);
        return;
    }

    output += "RestrictedDrivingRoute:";
    for (size_t i = 0; i < route.second.size(); ++i)
    {
        output += std::to_string(route.second[i]);
        if (i < route.second.size() - 1)
            output += ",";
    }
    output += "(" + std::to_string(route.first) + ")\n\n";

    writeBatch(output);
}

void Manager::drive_and_walk_route(int source, int destination, int max_walking_time, const std::unordered_set<int>& avoid_nodes, const std::vector<std::pair<int, int>>& avoid_segments) const
{
    auto source_itr = nodes.find(source);
    auto destination_itr = nodes.find(destination);

    std::string output = "Source:" + std::to_string(source) + "\n" +
                         "Destination:" + std::to_string(destination) + "\n";

    if (source_itr == nodes.end() || destination_itr == nodes.end())
    {
        output += "DrivingRoute:none\n";
        output += "ParkingNode:none\n";
        output += "WalkingRoute:none\n";
        output += "TotalTime:\n";
        output += "Message:Invalid source or destination node\n";
        writeBatch(output);
        return;
    }

    // Modify the graph to exclude avoid_segments
    Graph<Node*> modified_graph = graph;
    for (const auto& segment : avoid_segments)
    {
        auto node1_itr = nodes.find(segment.first);
        auto node2_itr = nodes.find(segment.second);
        if (node1_itr != nodes.end() && node2_itr != nodes.end())
        {
            modified_graph.removeEdge(node1_itr->second, node2_itr->second);
        }
    }

    // Mark the avoid nodes as visited
    std::unordered_set<Node*> visited;
    for (auto node : avoid_nodes) {
        auto it = nodes.find(node);
        if (it != nodes.end()) {
            visited.insert(it->second);
        }
    }

    // Find all parking nodes, do not include avoid ones
    std::vector<Node *> parking_nodes;
    for (const auto &pair : nodes)
    {
        auto it = visited.find(pair.second);
        if (it == visited.end() && pair.second->getParking()) {
            parking_nodes.push_back(pair.second);
        }
    }

    std::pair<int, std::vector<int>> best_driving_route;
    std::pair<int, std::vector<int>> best_walking_route;
    Node* best_parking_node = nullptr;
    int best_total_time = std::numeric_limits<int>::max();
    std::vector<std::tuple<int, std::pair<int, std::vector<int>>, std::pair<int, std::vector<int>>>> alternative_routes;

    // Try each parking node as the transition point from driving to walking
    for (Node *parking_node : parking_nodes)
    {
        if (parking_node->getId() == source || parking_node->getId() == destination)
        {
            continue;
        }

        // Find the driving route to the parking node
        auto driving_route = dijkstra(modified_graph, source_itr->second, parking_node, nodes, visited);
        if (driving_route.second.empty())
        {
            continue;
        }

        // Find the walking route from the parking node to the destination
        auto walking_route = dijkstra(modified_graph, parking_node, destination_itr->second, nodes, visited, true);
        if (walking_route.second.empty())
        {
            continue;
        }

        // Combine the driving and walking routes
        int total_time = driving_route.first + walking_route.first;
        if (total_time < best_total_time && walking_route.first <= max_walking_time)
        {
            best_total_time = total_time;
            best_driving_route = driving_route;
            best_walking_route = walking_route;
            best_parking_node = parking_node;
        }
        else if (walking_route.first > max_walking_time)
        {
            alternative_routes.emplace_back(total_time, driving_route, walking_route);
        }
    }

    if (best_driving_route.second.empty() || best_walking_route.second.empty())
    {
        output += "DrivingRoute:none\n";
        output += "ParkingNode:none\n";
        output += "WalkingRoute:none\n";
        output += "TotalTime:\n";
        if (parking_nodes.empty())
        {
            output += "Message:No parking nodes available\n\n";
        }
        else if (best_driving_route.second.empty() && alternative_routes.empty())
        {
            output += "Message:No valid driving route found\n\n";
        }
        else if (best_walking_route.second.empty())
        {
            output += "Message:No valid walking route found or walking time exceeds maximum limit\n\n";
        }
        else
        {
            output += "Message:Unknown error\n\n";
        }
        // Provide alternative routes
        if (!alternative_routes.empty())
        {
            output += "Alternative routes:\n";
            output += "Source:" + std::to_string(source) + "\n" +
                         "Destination:" + std::to_string(destination) + "\n";

            std::sort(alternative_routes.begin(), alternative_routes.end());
                for (size_t i = 0; i < std::min(alternative_routes.size(), size_t(2)); ++i)
                {
                    auto [total_time, driving_route, walking_route] = alternative_routes[i];
                    output += "DrivingRoute" + std::to_string(i + 1) + ":";
                    for (size_t j = 0; j < driving_route.second.size(); ++j)
                    {
                        output += std::to_string(driving_route.second[j]);
                        if (j < driving_route.second.size() - 1)
                            output += ",";
                    }
                    output += "(" + std::to_string(driving_route.first) + ")\n";

                    int parking_node = walking_route.second[0];
                    output += "ParkingNode" + std::to_string(i + 1) + ":" + std::to_string(parking_node) + "\n";

                    output += "WalkingRoute" + std::to_string(i + 1) + ":";
                    for (size_t j = 0; j < walking_route.second.size(); ++j)
                    {
                        output += std::to_string(walking_route.second[j]);
                        if (j < walking_route.second.size() - 1)
                            output += ",";
                    }
                    output += "(" + std::to_string(walking_route.first) + ")\n";

                    output += "TotalTime" + std::to_string(i + 1) + ":" + std::to_string(total_time) + "\n";
                }
            output += "\n";
        }
    }
    else
    {
        output += "DrivingRoute:";
        for (size_t i = 0; i < best_driving_route.second.size(); ++i)
        {
            output += std::to_string(best_driving_route.second[i]);
            if (i < best_driving_route.second.size() - 1)
                output += ",";
        }
        output += "(" + std::to_string(best_driving_route.first) + ")\n";

        output += "ParkingNode:" + std::to_string(best_parking_node->getId()) + "\n";

        output += "WalkingRoute:";
        for (size_t i = 0; i < best_walking_route.second.size(); ++i)
        {
            output += std::to_string(best_walking_route.second[i]);
            if (i < best_walking_route.second.size() - 1)
                output += ",";
        }
        output += "(" + std::to_string(best_walking_route.first) + ")\n";

        output += "TotalTime:" + std::to_string(best_total_time) + "\n\n";
    }

    writeBatch(output);
}