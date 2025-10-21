#pragma once

#include <vector>
#include "Vertex.h"
#include "Edge.h"

template <typename TVertex, typename TEdge>
class Graph {
protected:
    std::vector<TVertex*> m_vertices;
    std::vector<TEdge*> m_edges;
    bool m_directed;

public:
    explicit Graph(bool directed = true) : m_directed(directed) {}
    virtual ~Graph() {
        for (auto* vertex : m_vertices) { delete vertex; }
        for (auto* edge : m_edges) { delete edge; }
        m_vertices.clear();
        m_edges.clear();
    }
    virtual int addVertex(TVertex* vertex) = 0;
    virtual void addEdge(TEdge* edge) = 0;
    virtual void removeVertex(int id) = 0;
    virtual void removeEdge(TEdge* edge) = 0;
    bool isDirected() const { return m_directed; }
    int getNumberOfVertices() const { return static_cast<int>(m_vertices.size()); }
    TVertex* getVertexById(int id) const {
        if (id >= 0 && id < static_cast<int>(m_vertices.size())) return m_vertices[id];
        return nullptr;
    }
    virtual std::vector<int> getNeighbors(int id) const = 0;
    virtual TEdge* getEdge(int fromId, int toId) const = 0;
    virtual std::vector<TEdge*> getEdgesFrom(int fromId) const = 0;
    const std::vector<TVertex*>& getVertices() const { return m_vertices; }
    const std::vector<TEdge*>& getEdges() const { return m_edges; }
};