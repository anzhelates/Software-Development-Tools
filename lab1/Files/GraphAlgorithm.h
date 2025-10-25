#pragma once

#include <vector>
#include <queue>
#include <stack>
#include <limits>
#include "Graph.h"
#include "Vehicle.h"

class GraphAlgorithms {
public:
    template <typename TVertex, typename TEdge>
    static std::vector<int> BFS(const Graph<TVertex, TEdge>& graph, int startId) {
        std::vector<int> order;
        int n = graph.getNumberOfVertices();
        if (startId < 0 || startId >= n) return order;

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

    template <typename TVertex, typename TEdge>
    static std::vector<int> DFS(const Graph<TVertex, TEdge>& graph, int startId) {
        std::vector<int> order;
        int n = graph.getNumberOfVertices();
        if (startId < 0 || startId >= n) return order;

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

    template <typename TVertex, typename TEdge>
    static std::vector<double> Dijkstra(const Graph<TVertex, TEdge>& graph, int startId, const Vehicle& vehicle) {
        int n = graph.getNumberOfVertices();
        std::vector<double> dist(n, std::numeric_limits<double>::infinity());

        if (startId < 0 || startId >= n) {
            return dist;
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