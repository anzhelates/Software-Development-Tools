#pragma once

#include "Graph.h"
#include <vector>
#include <stdexcept>

/**
 * @file AdjacencyMatrix.h
 * @class AdjacencyMatrix
 * @brief Represents a graph using an adjacency matrix
 * @tparam TVertex The type of data stored in a vertex
 * @tparam TEdge The type of data stored in an edge
 */
template <typename TVertex, typename TEdge>
class AdjacencyMatrix : public Graph<TVertex, TEdge> {
private:
    /// @brief The vector represents the adjacency matrix
    std::vector<std::vector<TEdge*>> m_adjMatrix;

public:
    /**
     * @brief Constructs a graph based on adjacency matrix
     * @param directed Checks whether the graph is directed
     */
    explicit AdjacencyMatrix(bool directed = true) : Graph<TVertex, TEdge>(directed) {}

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
        int id = static_cast<int>(this->m_vertices.size());
        vertex->setId(id);
        this->m_vertices.push_back(vertex);

        m_adjMatrix.resize(this->m_vertices.size());
        for (auto& row : m_adjMatrix) {
            row.resize(this->m_vertices.size(), nullptr);
        }
        return id;
    }

    /**
     * @brief Adds a new edge to the graph
     * @param edge A pointer to the edge to add
     * @throws std::invalid_argument If the edge pointer is null, or its source/destination is null
     * @throws std::out_of_range If the edge's vertex IDs are invalid for the current matrix size
     */
    void addEdge(TEdge* edge) override {
        if (!edge || !edge->getSource() || !edge->getDestination()) {
            throw std::invalid_argument("Edge, its source, or its destination cannot be null");
        }
        int from = edge->getSource()->getId();
        int to = edge->getDestination()->getId();
        if (from < 0 || from >= static_cast<int>(this->m_vertices.size()) || to < 0 || to >= static_cast<int>(this->m_vertices.size())) {
            throw std::out_of_range("Edge vertex ID is out of matrix bounds");
        }

        if (m_adjMatrix[from][to] != nullptr) return;
        edge->markActive();
        this->m_edges.push_back(edge);
        m_adjMatrix[from][to] = edge;
        if (!this->m_directed) {
            m_adjMatrix[to][from] = edge;
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

        int from = edge->getFrom();
        int to = edge->getTo();

        if (from >= 0 && from < static_cast<int>(m_adjMatrix.size()) &&
            to >= 0 && to < static_cast<int>(m_adjMatrix.size())) {
            if (m_adjMatrix[from][to] == edge) m_adjMatrix[from][to] = nullptr;
            if (!this->m_directed && m_adjMatrix[to][from] == edge) m_adjMatrix[to][from] = nullptr;
            }
        edge->markInactive();
    }

    /**
     * @brief Removes a vertex from the graph, marks it as inactive
     * @param id The ID of the vertex to remove
     * @throws std::out_of_range If the vertex ID is invalid
     */
    void removeVertex(int id) override {
        if (id < 0 || id >= static_cast<int>(this->m_vertices.size())) {
            throw std::out_of_range("Vertex ID is out of range");
        }

        for (int i = 0; i < static_cast<int>(m_adjMatrix.size()); ++i) {
            if (m_adjMatrix[id][i] != nullptr) m_adjMatrix[id][i]->markInactive();
            if (m_adjMatrix[i][id] != nullptr) m_adjMatrix[i][id]->markInactive();
            if (i >= 0 && i < static_cast<int>(m_adjMatrix.size())) {
                m_adjMatrix[id][i] = nullptr;
                m_adjMatrix[i][id] = nullptr;
            }
        }
        if (this->m_vertices[id]) this->m_vertices[id]->markInactive();
    }

    /**
     * @brief Gets the IDs of all neighboring vertices for a given vertex
     * @param id The ID of the source vertex
     * @return A vector of IDs of neighbor vertices
     * @throws std::out_of_range If the vertex ID is invalid
     */
    std::vector<int> getNeighbors(int id) const override {
        if (id < 0 || id >= static_cast<int>(m_adjMatrix.size())) {
            throw std::out_of_range("Vertex ID is out of range");
        }
        std::vector<int> neighbors;
        for (int i = 0; i < static_cast<int>(m_adjMatrix[id].size()); ++i) {
            if (m_adjMatrix[id][i] != nullptr && m_adjMatrix[id][i]->isActive()) {
                neighbors.push_back(i);
            }
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
        if (fromId < 0 || fromId >= static_cast<int>(m_adjMatrix.size())
            || toId < 0 || toId >= static_cast<int>(m_adjMatrix.size())) {
            throw std::out_of_range("Vertex ID is out of range");
        }
        TEdge* e = m_adjMatrix[fromId][toId];
        if (e && e->isActive()) return e;
        return nullptr;
    }

    /**
     * @brief Retrieves all outgoing edges from a specific vertex
     * @param fromId The ID of the source vertex
     * @return A vector of pointers to all active outgoing edges
     * @throws std::out_of_range If the fromId is invalid
     */
    std::vector<TEdge*> getEdgesFrom(int fromId) const override {
        if (fromId < 0 || fromId >= static_cast<int>(m_adjMatrix.size())) {
            throw std::out_of_range("Vertex ID is out of range");
        }
        std::vector<TEdge*> result;
        for (size_t to = 0; to < m_adjMatrix[fromId].size(); ++to) {
            TEdge* edge = m_adjMatrix[fromId][to];
            if (edge && edge->isActive()) {
                result.push_back(edge);
            }
        }
        return result;
    }
};