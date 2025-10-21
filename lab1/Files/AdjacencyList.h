#pragma once

#include "Graph.h"
#include <list>
#include <vector>

template <typename TVertex, typename TEdge>
class AdjacencyList : public Graph<TVertex, TEdge> {
private:
    struct AdjEntry {
        int m_to;
        TEdge* m_edge;
        AdjEntry(int to, TEdge* edge) : m_to(to), m_edge(edge) {}
    };

    std::vector<std::list<AdjEntry>> m_adjList;

public:
    explicit AdjacencyList(bool directed = true) : Graph<TVertex, TEdge>(directed) {}

    ~AdjacencyList() override = default;

    int addVertex(TVertex* vertex) override {
        if (!vertex) return -1;
        int id = this->m_vertices.size();
        vertex->setId(id);
        this->m_vertices.push_back(vertex);
        m_adjList.resize(this->m_vertices.size());
        return id;
    }

    void addEdge(TEdge* edge) override {
        if (!edge || !edge->getSource() || !edge->getDestination()) return;
        edge->markActive();
        this->m_edges.push_back(edge);
        int from = edge->getSource()->getId();
        int to = edge->getDestination()->getId();
        m_adjList[from].push_back(AdjEntry(to, edge));
        if (!this->m_directed) {
            m_adjList[to].push_back(AdjEntry(from, edge));
        }
    }

    void removeEdge(TEdge* edge) override {
        if (!edge || !edge->getSource() || !edge->getDestination()) return;

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

    void removeVertex(int id) override {
        if (id < 0 || id >= static_cast<int>(this->m_vertices.size())) return;

        for (int i = 0; i < static_cast<int>(m_adjList.size()); ++i) {
            if (i == id) continue;
            for (auto it = m_adjList[i].begin(); it != m_adjList[i].end();) {
                if (it->m_to == id) it = m_adjList[i].erase(it);
                else ++it;
            }
        }

        for (const auto& entry : m_adjList[id]) {
            if (entry.m_edge) entry.m_edge->markInactive();
        }

        m_adjList[id].clear();

        if (this->m_vertices[id]) this->m_vertices[id]->markInactive();
    }

    std::vector<int> getNeighbors(int id) const override {
        std::vector<int> neighbors;
        if (id >= 0 && id < static_cast<int>(m_adjList.size())) {
            for (const auto& entry : m_adjList[id]) {
                if (entry.m_edge && entry.m_edge->isActive())
                    neighbors.push_back(entry.m_to);
            }
        }
        return neighbors;
    }

    TEdge* getEdge(int fromId, int toId) const override {
        if (fromId >= 0 && fromId < static_cast<int>(m_adjList.size())) {
            for (const auto& entry : m_adjList[fromId]) {
                if (entry.m_to == toId && entry.m_edge && entry.m_edge->isActive())
                    return entry.m_edge;
            }
        }
        return nullptr;
    }

    std::vector<TEdge*> getEdgesFrom(int fromId) const override {
        std::vector<TEdge*> result;
        if (fromId >= 0 && fromId < static_cast<int>(this->m_adjList.size())) {
            for (const auto& entry : this->m_adjList[fromId]) {
                if (entry.m_edge && entry.m_edge->isActive()) {
                    result.push_back(entry.m_edge);
                }
            }
        }
        return result;
    }
};