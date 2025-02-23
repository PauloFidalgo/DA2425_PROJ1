# Project Documentation

## Overview

This project involves route planning functionalities for a graph-based navigation system. The main functionalities include:
1. Independent Route Planning
2. Restricted Route Planning
3. Environmentally-Friendly Route Planning (driving and walking)

## Main Functions

### 1. `Manager::drive_only_independent_route`

**Description**: This function determines the best (fastest) route between a source and destination. It also identifies a best alternative independent route, ensuring the two routes share no intermediate nodes or segments, except for the source and destination.

**Complexity**: The complexity is O((V + E) log V) due to the use of Dijkstra's algorithm for finding the shortest path.

**How it works**:
- Uses Dijkstra's algorithm to find the shortest path from the source to the destination.
- Removes the nodes and edges used in the best route (except for the source and destination) and then uses Dijkstra's algorithm again to find the next best route.

### 2. `Manager::restricted_route`

**Description**: This function computes the fastest route with specific routing restrictions, such as excluding specific nodes and segments from the graph, and including a specific node that the route must pass through.

**Complexity**: The complexity is O((V + E) log V) due to the use of Dijkstra's algorithm for finding the shortest path.

**How it works**:
- Modifies the graph by removing the nodes and edges specified in the restrictions.
- If an include node is specified, finds the route via the include node by splitting the route into two legs.
- Uses Dijkstra's algorithm to find the shortest path for each leg and combines the results.

### 3. `Manager::drive_and_walk_route`

**Description**: This function plans routes that combine driving and walking, allowing users to drive the first section of the route, park the vehicle, and then walk the remaining distance to the destination.

**Complexity**: The complexity is O((V + E) log V) due to the use of Dijkstra's algorithm for finding the shortest path.

**How it works**:
- Finds all parking nodes in the graph.
- For each parking node, finds the driving route to the parking node and the walking route from the parking node to the destination.
- Combines the driving and walking routes and selects the best route that meets the user's requirements.
- If no suitable route is found, provides up to 2 alternative routes that approximate the user's requirements, sorted by overall travel time.

### 4. `Manager::readBatch`

**Description**: This function reads the input file and processes each instruction to perform the appropriate route planning functionality.

**Complexity**: The complexity depends on the number of instructions and the complexity of the route planning functions called.

**How it works**:
- Reads each line from the input file.
- Extracts the mode and parameters for each instruction.
- Calls the appropriate route planning function based on the mode and parameters.

### 5. `Manager::writeBatch`

**Description**: This function writes the output to a file, either overwriting the file or appending to it based on a flag.

**Complexity**: The complexity is O(1) for each write operation.

**How it works**:
- Opens the output file in the appropriate mode (overwrite or append).
- Writes the output string to the file.
- Closes the file.

### 6. `dijkstra`

**Description**: This function implements Dijkstra's algorithm to find the shortest path between two nodes in the graph.

**Complexity**: The complexity is O((V + E) log V).

**How it works**:
- Initializes the distance to all nodes as infinity and the distance to the source node as 0.
- Uses a priority queue to select the node with the smallest distance.
- Updates the distances to the adjacent nodes and repeats the process until the destination node is reached or all nodes are processed.