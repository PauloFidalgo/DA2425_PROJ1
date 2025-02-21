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

                if (getline(batchFile, line) && line.find("Source:") != std::string::npos)
                {
                    extractPattern(line, tmp);
                    int int_source = stoi(tmp);

                    if (getline(batchFile, line) && line.find("Destination:") != std::string::npos)
                    {
                        extractPattern(line, tmp);
                        int int_destination = stoi(tmp);

                        drive_only_independent_route(int_source, int_destination);
                    }
                }
            }
        }
    }
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

    std::cout << "Source:" << source << std::endl;
    std::cout << "Destination:" << destination << std::endl;

    if (source_itr == nodes.end() || destination_itr == nodes.end())
    {
        std::cout << "BestDrivingRoute:none\n";
        std::cout << "AlternativeDrivingRoute:none\n\n";
        return;
    }

    std::unordered_set<Node*> visited;

    std::pair<int,std::vector<int>> best_route = dijkstra(graph, source_itr->second, destination_itr->second, nodes, visited);
    if (best_route.second.empty())
    {
        std::cout << "BestDrivingRoute:none\n";
        std::cout << "AlternativeDrivingRoute:none\n\n";
        return;
    }

    std::cout << "BestDrivingRoute:";
    for (size_t i = 0; i < best_route.second.size(); ++i)
    {
        std::cout << best_route.second[i];
        if (i < best_route.second.size() - 1)
            std::cout << ",";
    }

    std::cout << "(" << best_route.first << ")" << endl;

    for (auto node : best_route.second) {
        if (node != source && node != destination) {
            auto v = nodes.find(node);
            if (v != nodes.end()) {
                visited.insert(v->second);
            }
        }
    }

    std::pair<int,std::vector<int>> alternative_route = dijkstra(graph, source_itr->second, destination_itr->second, nodes, visited);
    if (alternative_route.second.empty())
    {
        std::cout << "AlternativeDrivingRoute:none\n\n";
        return;
    }

    std::cout << "AlternativeDrivingRoute:";
    for (size_t i = 0; i < alternative_route.second.size(); ++i)
    {
        std::cout << alternative_route.second[i];
        if (i < alternative_route.second.size() - 1)
            std::cout << ",";
    }

    std::cout << "(" << alternative_route.first << ")" << endl << endl;
}