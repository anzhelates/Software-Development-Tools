#pragma once

#include <vector>
#include <queue>
#include <stack>
#include <limits>
#include <stdexcept>
#include "Graph.h"
#include "Vehicle.h"

/**
 * @file GraphAlgorithm.h
 * @brief Implementation of BFS, DFS, Dijkstra's algorithm and isConnected method
 * @class GraphAlgorithms
 * @brief Static methods for common graph algorithms
 */
class GraphAlgorithms {
public:
    /**
     * @brief Performs a Breadth-First Search starting from a given vertex ID
     * @tparam TVertex The type of data stored in graph vertices
     * @tparam TEdge The type of data stored on graph edges
     * @param graph A constant reference to the graph to be traversed
     * @param startId The ID of the starting vertex
     * @return std::vector<int> A vector of vertex IDs in the order they were visited
     * @throws std::out_of_range If the startId is invalid
     */
    template <typename TVertex, typename TEdge>
    static std::vector<int> BFS(const Graph<TVertex, TEdge>& graph, int startId) {
        std::vector<int> order;
        int n = graph.getNumberOfVertices();
        if (startId < 0 || startId >= n) {
            throw std::out_of_range("BFS startId is out of range");
        }

        std::vector<bool> visited(n, false);
        std::queue<int> q;

        visited[startId] = true;
        q.push(startId);

        while (!q.empty()) {
            int current = q.front(); q.pop();
            order.push_back(current);

            std::vector<int> neighbors = graph.getNeighbors(current);
            for (int neighbor : neighbors) {
                if (neighbor >= 0 && neighbor < n && !visited[neighbor]) {
                    visited[neighbor] = true;
                    q.push(neighbor);
                }
            }
        }
        return order;
    }

    /**
     * @brief Performs a Depth-First Search starting from a given vertex ID
     * @tparam TVertex The type of data stored in graph vertices
     * @tparam TEdge The type of data stored on graph edges
     * @param graph A constant reference to the graph to be traversed
     * @param startId The ID of the starting vertex
     * @return std::vector<int> A vector of vertex IDs in the order they were visited
     * @throws std::out_of_range If the startId is invalid
     */
    template <typename TVertex, typename TEdge>
    static std::vector<int> DFS(const Graph<TVertex, TEdge>& graph, int startId) {
        std::vector<int> order;
        int n = graph.getNumberOfVertices();
        if (startId < 0 || startId >= n) {
            throw std::out_of_range("DFS startId is out of range");
        }

        std::vector<bool> visited(n, false);
        std::stack<int> st;
        st.push(startId);

        while (!st.empty()) {
            int current = st.top(); st.pop();
            if (!visited[current]) {
                visited[current] = true;
                order.push_back(current);
                std::vector<int> neighbors = graph.getNeighbors(current);
                for (int i = static_cast<int>(neighbors.size()) - 1; i >= 0; --i) {
                    int neighbor = neighbors[i];
                    if (!visited[neighbor]) st.push(neighbor);
                }
            }
        }
        return order;
    }

    /**
     * @brief Implements Dijkstra's algorithm to find the shortest paths from a starting vertex
     * @details Edge weight is calculated as the travel time using the specified vehicle
     * @tparam TVertex The type of data stored in graph vertices
     * @tparam TEdge The type of data stored on graph edges
     * @param graph A reference to the graph
     * @param startId The ID of the starting vertex
     * @param vehicle A reference to the vehicle used for travel time calculation
     * @return std::vector<double> A vector of the shortest "time" distances from startId to all other vertices
     * @throws std::out_of_range If the startId is invalid
     *
     * @example GraphAlgorithm.h
     * @code
     * // Example
     * Car myCar("Car", 100, 8);
     * try {
     *     std::vector<double> times = GraphAlgorithms::Dijkstra(graph, 0, myCar);
     *     std::cout << "Time to vertex 1: " << times[1] << " hours." << std::endl;
     * } catch (const std::out_of_range& e) {
     *     std::cerr << "Error: Invalid start vertex" << std::endl;
     * }
     * @endcode
     */
    template <typename TVertex, typename TEdge>
    static std::vector<double> Dijkstra(const Graph<TVertex, TEdge>& graph, int startId, const Vehicle& vehicle) {
        int n = graph.getNumberOfVertices();
        std::vector<double> dist(n, std::numeric_limits<double>::infinity());

        if (startId < 0 || startId >= n) {
            throw std::out_of_range("Dijkstra startId is out of range");
        }

        dist[startId] = 0.0;

        using PDI = std::pair<double, int>;
        std::priority_queue<PDI, std::vector<PDI>, std::greater<PDI>> pq;

        pq.push({0.0, startId});

        while (!pq.empty()) {
            double currentDist = pq.top().first;
            int currentVertex = pq.top().second;
            pq.pop();

            if (currentDist > dist[currentVertex]) {
                continue;
            }

            std::vector<TEdge*> outgoingEdges = graph.getEdgesFrom(currentVertex);

            for (TEdge* edge : outgoingEdges) {
                if (!edge || !edge->isActive()) {
                    continue;
                }

                int neighborVertex = edge->getDestination()->getId();
                double travelTime = edge->calculateTravelTime(vehicle);

                if (travelTime == std::numeric_limits<double>::infinity()) {
                    continue;
                }

                double newDist = dist[currentVertex] + travelTime;

                if (newDist < dist[neighborVertex]) {
                    dist[neighborVertex] = newDist;
                    pq.push({newDist, neighborVertex});
                }
            }
        }
        return dist;
    }

    /**
     * @brief Checks if the graph is connected
     * @details For directed graphs, checks for strong connectivity. For undirected, checks for standard connectivity
     * @tparam TVertex The type of data stored in graph vertices
     * @tparam TEdge The type of data stored on graph edges
     * @param graph A reference to the graph
     * @return True if connected (or strongly connected), false otherwise
     */
    template <typename TVertex, typename TEdge>
    static bool isConnected(const Graph<TVertex, TEdge>& graph) {
        int n = graph.getNumberOfVertices();
        if (n < 2) return true;

        std::vector<bool> visited(n, false);
        std::stack<int> st;
        st.push(0);
        visited[0] = true;
        int count = 1;

        while (!st.empty()) {
            int current = st.top(); st.pop();

            for (int neighbor : graph.getNeighbors(current)) {
                if (!visited[neighbor]) {
                    visited[neighbor] = true;
                    st.push(neighbor);
                    count++;
                }
            }
        }

        if (!graph.isDirected()) {
            return count == n;
        }

        for (int i = 0; i < n; ++i) {
            std::fill(visited.begin(), visited.end(), false);
            std::stack<int> st2;
            st2.push(i);
            visited[i] = true;
            int reachableCount = 1;

            while (!st2.empty()) {
                int current = st2.top(); st2.pop();
                for (int neighbor : graph.getNeighbors(current)) {
                    if (!visited[neighbor]) {
                        visited[neighbor] = true;
                        st2.push(neighbor);
                        reachableCount++;
                    }
                }
            }
            if (reachableCount != n) return false;
        }
        return true;
    }
};