//
// Created by paulo on 21-02-2025.
//

#include "Manager.h"

/**
 * @brief Constructs a Manager object.
 * @param choice The choice of input file.
 */
Manager::Manager(std::string choice) {
    Parser parser = Parser(choice);
    this->graph = parser.parseGraph();
    this->nodes = parser.getNodesInt();

    readBatch();
}

/**
 * @brief Extracts the pattern from a line.
 * @param line The input line.
 * @param mode The extracted mode.
 */
void extractPattern(const std::string &line, std::string &mode)
{
    std::istringstream lineStream(line);
    std::string key;
    if (std::getline(lineStream, key, ':'))
    {
        std::getline(lineStream, mode);
    }
}

/**
 * @brief Reads the input file and processes each instruction.
 */
void Manager::readBatch() const
{
    fstream batchFile;
    batchFile.open("../Batch/input.txt", ios::in);
    if (!batchFile.is_open())
    {
        cout << "Error opening file" << endl;
        return;
    }

    bool restricted = false;

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
                        restricted = true;
                        extractPattern(line, tmp);
                        if (!tmp.empty())
                        {
                            include_node = stoi(tmp);
                        }
                    }
                } 


                if (avoid_nodes.empty() && avoid_segments.empty() && include_node == -1)
                {
                    drive_only_independent_route(int_source, int_destination, restricted);
                    restricted = false;
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

/**
 * @brief Writes the output to a file.
 * @param str The string to write to the file.
 */
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

/**
 * @brief Implements Dijkstra's algorithm to find the shortest path between two nodes in the graph.
 *
 * @param graph The graph representing the navigation system.
 * @param source The source node.
 * @param destination The destination node.
 * @param nodes The map of node IDs to Node objects.
 * @param visited The set of nodes that have been visited.
 * @param walking A flag indicating whether to use walking weights instead of driving weights.
 * @return A pair containing the total distance and the path as a vector of node IDs.
 *
 * @complexity The complexity is O((V + E) log V), where V is the number of vertices and E is the number of edges.
 */
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

/**
 * @brief Determines the best (fastest) route between a source and destination.
 * @param source The source node ID.
 * @param destination The destination node ID.
 * @param restricted Boolean to know if the function is restricted or not
 */
void Manager::drive_only_independent_route(int source, int destination, bool restricted) const
{
    auto source_itr = nodes.find(source);
    auto destination_itr = nodes.find(destination);

    std::string output = "Source:" + std::to_string(source) + "\n" +
                         "Destination:" + std::to_string(destination) + "\n";

    if (source_itr == nodes.end() || destination_itr == nodes.end() || (source == destination))
    {
        if (!restricted) {
            output += "BestDrivingRoute:none\n";
            output += "AlternativeDrivingRoute:none\n\n";
        } else {
            output += "RestrictedDrivingRoute:none\n";
        }

        writeBatch(output);
        return;
    }

    std::unordered_set<Node *> visited;

    std::pair<int, std::vector<int>> best_route = dijkstra(graph, source_itr->second, destination_itr->second, nodes, visited);
    if (best_route.second.empty())
    {
        if (!restricted) {
            output += "BestDrivingRoute:none\n";
            output += "AlternativeDrivingRoute:none\n\n";
        } else {
            output += "RestrictedDrivingRoute:none\n";
        }

        writeBatch(output);
        return;
    }

    output += restricted ? "RestrictedDrivingRoute:" : "BestDrivingRoute:";
    for (size_t i = 0; i < best_route.second.size(); ++i)
    {
        output += std::to_string(best_route.second[i]);
        if (i < best_route.second.size() - 1)
            output += ",";
    }
    output += "(" + std::to_string(best_route.first) + ")\n";
    output += restricted ? "\n" : "";

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

    if (!restricted) {
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
    }

    writeBatch(output);
}

/**
 * @brief Finds a restricted driving route from a source node to a destination node, avoiding specified nodes and segments, and optionally including a specific node.
 * @param source The source node ID.
 * @param destination The destination node ID.
 * @param avoid_nodes The set of nodes to avoid.
 * @param avoid_segments The list of segments to avoid.
 * @param include_node The node that must be included in the route.
 */
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

    std::vector<pair<Edge<Node*>*, pair<int,int>>> original_weights;
    for (const auto &segment : avoid_segments)
    {
        auto node1_itr = nodes.find(segment.first);
        if (node1_itr != nodes.end())
        {
            auto source_vertex = graph.findVertex(node1_itr->second);
            if (source_vertex) {
                for (auto outgoing_edge : source_vertex->getAdj()) {
                    if (outgoing_edge->getDest()->getInfo()->getId() == segment.second) {
                        pair<int, int> original = outgoing_edge->getWeight();
                        pair<Edge<Node *> *, pair<int, int>> edge_dist = {outgoing_edge, original};
                        original_weights.push_back(edge_dist);
                        outgoing_edge->setWeight({-1, -1});
                    }
                }
            }
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

        auto first_leg = dijkstra(graph, source_itr->second, include_node_itr->second, nodes, visited);
        if (first_leg.second.empty())
        {
            output += "RestrictedDrivingRoute:none\n\n";
            writeBatch(output);
            return;
        }

        auto second_leg = dijkstra(graph, include_node_itr->second, destination_itr->second, nodes, visited);
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
        route = dijkstra(graph, source_itr->second, destination_itr->second, nodes, visited);
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

    for (auto originals : original_weights) {
        originals.first->setWeight(originals.second);
    }
}


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

    // Modify the graph to avoid segments
    std::vector<pair<Edge<Node *> *, pair<int, int>>> original_weights;
    for (const auto &segment : avoid_segments)
    {
        auto node1_itr = nodes.find(segment.first);
        if (node1_itr != nodes.end())
        {
            auto source_vertex = graph.findVertex(node1_itr->second);
            if (source_vertex)
            {
                for (auto outgoing_edge : source_vertex->getAdj())
                {
                    if (outgoing_edge->getDest()->getInfo()->getId() == segment.second)
                    {
                        pair<int, int> original = outgoing_edge->getWeight();
                        pair<Edge<Node *> *, pair<int, int>> edge_dist = {outgoing_edge, original};
                        original_weights.push_back(edge_dist);
                        outgoing_edge->setWeight({-1, -1});
                    }
                }
            }
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

    bool no_valid_walking_route = false;
    bool walking_time_exceeds_limit = false;

    // Try each parking node as the transition point from driving to walking
    for (Node *parking_node : parking_nodes)
    {
        if (parking_node->getId() == source || parking_node->getId() == destination)
        {
            continue;
        }

        // Find the driving route to the parking node
        auto driving_route = dijkstra(graph, source_itr->second, parking_node, nodes, visited);
        if (driving_route.second.empty())
        {
            continue;
        }

        // Find the walking route from the parking node to the destination
        auto walking_route = dijkstra(graph, parking_node, destination_itr->second, nodes, visited, true);
        if (walking_route.second.empty())
        {
            no_valid_walking_route = true;
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
            walking_time_exceeds_limit = true;
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
        else if (no_valid_walking_route)
        {
            output += "Message:No valid walking route found\n";
        }
        else if (walking_time_exceeds_limit)
        {
            output += "Message:Walking time exceeds maximum limit\n";
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

    for (auto originals : original_weights)
    {
        originals.first->setWeight(originals.second);
    }
}