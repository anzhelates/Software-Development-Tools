#pragma once

#include <vector>
#include <stdexcept>
#include "Vertex.h"
#include "Edge.h"

/**
 * @file Graph.h
 * @class Graph
 * @brief An abstract base class for graph data structures
 * The graph manages the memory of the vertices and edges added to it
 * @tparam TVertex The type of data stored in a vertex
 * @tparam TEdge The type of data stored in an edge
 */
template <typename TVertex, typename TEdge>
class Graph {
    protected:
        std::vector<TVertex*> m_vertices; ///< Container for all vertices in the graph
        std::vector<TEdge*> m_edges; ///< Container for all edges in the graph
        bool m_directed; ///< Checks whether the graph is directed or not

    public:
        /**
         * @brief Constructs a graph
         * @param directed Set to true for a directed graph, false for an undirected graph
         */
        explicit Graph(bool directed = true) : m_directed(directed) {}

        /**
         * @brief Virtual destructor
         * @details Cleans up all vertices and edges owned by the graph
         */
        virtual ~Graph() {
            for (auto* vertex : m_vertices) { delete vertex; }
            for (auto* edge : m_edges) { delete edge; }
            m_vertices.clear();
            m_edges.clear();
        }

        /**
         * @brief Pure virtual function to add a vertex to the graph
         * @param vertex A pointer to the vertex to add
         * @return The ID assigned to the vertex
         */
        virtual int addVertex(TVertex* vertex) = 0;

        /**
         * @brief Pure virtual function to add an edge to the graph
         * @param edge A pointer to the edge to add
         */
        virtual void addEdge(TEdge* edge) = 0;

        /**
         * @brief Pure virtual function to remove a vertex from the graph
         * @param id The ID of the vertex to remove
         */
        virtual void removeVertex(int id) = 0;

        /**
         * @brief Pure virtual function to remove an edge from the graph
         * @param edge A pointer to the edge to remove
         */
        virtual void removeEdge(TEdge* edge) = 0;

        /**
         * @brief Checks if the graph is directed
         * @return True if the graph is directed, false otherwise
         */
        bool isDirected() const { return m_directed; }

        /**
         * @brief Gets the total number of vertices in the graph
         * @return The number of vertices
         */
        int getNumberOfVertices() const { return static_cast<int>(m_vertices.size()); }

        /**
         * @brief Gets a vertex by its unique ID
         * @param id The ID of the vertex
         * @return A pointer to the vertex if found
         * @throws std::out_of_range If the ID is invalid
         */
        TVertex* getVertexById(int id) const {
            if (id < 0 || id >= static_cast<int>(m_vertices.size())) {
                throw std::out_of_range("Vertex ID is out of range");
            }
            return m_vertices[id];
        }

        /**
         * @brief Pure virtual function to get the neighbors of a vertex
         * @param id The ID of the vertex
         * @return A vector of IDs of the neighboring vertices
         */
        virtual std::vector<int> getNeighbors(int id) const = 0;

        /**
         * @brief Pure virtual function to get the edge between two vertices
         * @param fromId The ID of the source vertex
         * @param toId The ID of the destination vertex
         * @return A pointer to the edge if it exists, otherwise nullptr
         */
        virtual TEdge* getEdge(int fromId, int toId) const = 0;

        /**
         * @brief Pure virtual function to get all outgoing edges from a vertex
         * @param fromId The ID of the source vertex
         * @return A vector of pointers to the outgoing edges
         */
        virtual std::vector<TEdge*> getEdgesFrom(int fromId) const = 0;

        /**
         * @brief Gets all vertices in the graph
         * @return A const reference to the vector of vertex pointers
         */
        const std::vector<TVertex*>& getVertices() const { return m_vertices; }

        /**
         * @brief Gets all edges in the graph
         * @return A const reference to the vector of edge pointers
         */
        const std::vector<TEdge*>& getEdges() const { return m_edges; }
};