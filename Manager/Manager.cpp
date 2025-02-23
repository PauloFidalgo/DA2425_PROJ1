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

        if (line.find("Mode:") != std::string::npos)
        {
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

std::pair<int, std::vector<int>> dijkstra(const Graph<Node *> &graph, Node *source, Node *destination, const std::unordered_map<int, Node *> &nodes, unordered_set<Node*> &visited)
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
        current->setVisited(true);

        if (current)
        {
            for (const auto &neighbor : current->getAdj())
            {
                auto v = neighbor->getDest();

                if (v->isVisited()) continue;

                int weight = neighbor->getDrive();
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
        output += "RestrictedDrivingRoute:none\n";
        writeBatch(output);
        return;
    }

    std::unordered_set<Node*> visited;

    // Modify the graph to exclude avoid_nodes and avoid_segments
    Graph<Node*> modified_graph = graph;
    for (int node_id : avoid_nodes)
    {
        auto node_itr = nodes.find(node_id);
        if (node_itr != nodes.end())
        {
            modified_graph.removeVertex(node_itr->second);
        }
    }

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
            output += "RestrictedDrivingRoute:none\n";
            writeBatch(output);
            return;
        }

        auto first_leg = dijkstra(modified_graph, source_itr->second, include_node_itr->second, nodes, visited);
        if (first_leg.second.empty())
        {
            output += "RestrictedDrivingRoute:none\n";
            writeBatch(output);
            return;
        }

        auto second_leg = dijkstra(modified_graph, include_node_itr->second, destination_itr->second, nodes, visited);
        if (second_leg.second.empty())
        {
            output += "RestrictedDrivingRoute:none\n";
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
        output += "RestrictedDrivingRoute:none\n";
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
    output += "(" + std::to_string(route.first) + ")\n";

    writeBatch(output);
}