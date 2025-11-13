#pragma once

#include "Graph.h"
#include <list>
#include <vector>
#include <stdexcept>

/**
 * @file AdjacencyList.h
 * @brief Defines the AdjacencyList class for Graph representation using adjacency list
 * @class AdjacencyList
 * @brief Represents a Graph using an adjacency list
 * @tparam TVertex The type of data stored in a vertex
 * @tparam TEdge The type of data stored in an edge
 */
template <typename TVertex, typename TEdge>
class AdjacencyList : public Graph<TVertex, TEdge> {
private:
    /**
     * @struct AdjEntry
     * @brief A helper structure to store an entry in the adjacency list
     */
    struct AdjEntry {
        int m_to; ///< ID of the destination vertex
        TEdge* m_edge; ///< Pointer to the edge connecting the vertices

        /**
         * @brief Constructs an AdjEntry
         * @param to The ID of the destination vertex
         * @param edge A pointer to the edge object
         */
        AdjEntry(int to, TEdge* edge) : m_to(to), m_edge(edge) {}
    };

    /// @brief The adjacency list, each index corresponds to a vertex ID
    std::vector<std::list<AdjEntry>> m_adjList;

public:
    /**
     * @brief Constructs an AdjacencyList graph
     * @param directed Indicates whether the graph is directed or not
     */
    explicit AdjacencyList(bool directed = true) : Graph<TVertex, TEdge>(directed) {}
    ~AdjacencyList() override = default;

    /**
     * @brief Adds a new vertex to the graph
     * @param vertex A pointer to the vertex to add
     * @return The ID assigned to the new vertex
     * @throws std::invalid_argument If the vertex pointer is null
     */
    int addVertex(TVertex* vertex) override {
        if (!vertex) {
            throw std::invalid_argument("Vertex pointer cannot be null");
        }
        int id = this->m_vertices.size();
        vertex->setId(id);
        this->m_vertices.push_back(vertex);
        m_adjList.resize(this->m_vertices.size());
        return id;
    }

    /**
     * @brief Adds a new edge to the graph
     * @param edge A pointer to the edge to add
     * @throws std::invalid_argument If the edge pointer is null, or its source/destination is null
     * @throws std::out_of_range If the edge's vertex IDs are invalid for the current graph size
     *
     * @example
     * @code
     * AdjacencyList<MyVertex, MyEdge> graph(true); // directed = true
     *
     * int A = graph.addVertex(new MyVertex("A")); // 0
     * int B = graph.addVertex(new MyVertex("B")); // 1
     * int C = graph.addVertex(new MyVertex("C")); // 2
     *
     * graph.addEdge(new MyEdge(graph.getVertexById(A), graph.getVertexById(B), 5)); // A -> B
     * graph.addEdge(new MyEdge(graph.getVertexById(A), graph.getVertexById(C), 7)); // A -> C
     *
     * // Getting neighbors of the vertex A
     * auto neighbors = graph.getNeighbors(A); // {1, 2}
     * @endcode
     */
    void addEdge(TEdge* edge) override {
        if (!edge || !edge->getSource() || !edge->getDestination()) {
            throw std::invalid_argument("Edge, its source, or its destination cannot be null");
        }
        edge->markActive();
        this->m_edges.push_back(edge);
        int from = edge->getSource()->getId();
        int to = edge->getDestination()->getId();

        if (from < 0 || from >= static_cast<int>(m_adjList.size()) || to < 0 || to >= static_cast<int>(m_adjList.size())) {
            throw std::out_of_range("Edge vertex ID is out of graph bounds");
        }

        m_adjList[from].push_back(AdjEntry(to, edge));
        if (!this->m_directed) {
            m_adjList[to].push_back(AdjEntry(from, edge));
        }
    }

    /**
     * @brief Removes an edge from the graph, marks it as inactive
     * @param edge A pointer to the edge to remove
     * @throws std::invalid_argument If the edge pointer is null
     */
    void removeEdge(TEdge* edge) override {
        if (!edge) {
            throw std::invalid_argument("Cannot remove a null edge");
        }
        if (!edge->getSource() || !edge->getDestination()) return;

        int from = edge->getFrom();
        int to = edge->getTo();

        if (from >= 0 && from < static_cast<int>(m_adjList.size())) {
            for (auto it = m_adjList[from].begin(); it != m_adjList[from].end();) {
                if (it->m_edge == edge) it = m_adjList[from].erase(it);
                else ++it;
            }
        }
        if (!this->m_directed && to >= 0 && to < static_cast<int>(m_adjList.size())) {
            for (auto it = m_adjList[to].begin(); it != m_adjList[to].end();) {
                if (it->m_edge == edge) it = m_adjList[to].erase(it);
                else ++it;
            }
        }
        edge->markInactive();
    }

    /**
     * @brief Removes a vertex from the graph by marking it as inactive
     * @details All edges connected to this vertex are also marked as inactive
     * @param id The ID of the vertex to remove
     * @throws std::out_of_range If the vertex ID is invalid
     */
    void removeVertex(int id) override {
        if (id < 0 || id >= static_cast<int>(this->m_vertices.size())) {
            throw std::out_of_range("Vertex ID is out of range");
        }

        for (int i = 0; i < static_cast<int>(m_adjList.size()); ++i) {
            if (i == id) continue;
            for (auto it = m_adjList[i].begin(); it != m_adjList[i].end();) {
                if (it->m_to == id) {
                    if (it->m_edge) it->m_edge->markInactive();
                    it = m_adjList[i].erase(it);
                }
                else ++it;
            }
        }

        for (const auto& entry : m_adjList[id]) {
            if (entry.m_edge) entry.m_edge->markInactive();
        }

        m_adjList[id].clear();

        if (this->m_vertices[id]) this->m_vertices[id]->markInactive();
    }

    /**
     * @brief Gets the IDs of all neighboring vertices for a given vertex
     * @param id The ID of the source vertex
     * @return A vector of IDs of neighbor vertices
     * @throws std::out_of_range If the vertex ID is invalid
     */
    std::vector<int> getNeighbors(int id) const override {
        if (id < 0 || id >= static_cast<int>(m_adjList.size())) {
            throw std::out_of_range("Vertex ID is out of range");
        }
        std::vector<int> neighbors;
        for (const auto& entry : m_adjList[id]) {
            if (entry.m_edge && entry.m_edge->isActive())
                neighbors.push_back(entry.m_to);
        }
        return neighbors;
    }

    /**
     * @brief Retrieves the edge between two vertices
     * @param fromId The ID of the source vertex
     * @param toId The ID of the destination vertex
     * @return A pointer to the edge if it exists and is active, otherwise nullptr
     * @throws std::out_of_range If either fromId or toId is invalid
     */
    TEdge* getEdge(int fromId, int toId) const override {
        if (fromId < 0 || fromId >= static_cast<int>(m_adjList.size()) || toId < 0 || toId >= static_cast<int>(m_adjList.size())) {
            throw std::out_of_range("Vertex ID is out of range");
        }
        for (const auto& entry : m_adjList[fromId]) {
            if (entry.m_to == toId && entry.m_edge && entry.m_edge->isActive())
                return entry.m_edge;
        }
        return nullptr;
    }

    /**
     * @brief Retrieves all outgoing edges from a specific vertex
     * @param fromId The ID of the source vertex
     * @return A vector of pointers to all active outgoing edges
     * @throws std::out_of_range If the fromId is invalid
     */
    std::vector<TEdge*> getEdgesFrom(int fromId) const override {
        if (fromId < 0 || fromId >= static_cast<int>(this->m_adjList.size())) {
            throw std::out_of_range("Vertex ID is out of range");
        }
        std::vector<TEdge*> result;
        for (const auto& entry : this->m_adjList[fromId]) {
            if (entry.m_edge && entry.m_edge->isActive()) {
                result.push_back(entry.m_edge);
            }
        }
        return result;
    }
};