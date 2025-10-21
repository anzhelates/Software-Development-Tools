#pragma once

#include <string>
#include <vector>
#include "Vehicle.h"

template <typename TVertex, typename TEdge> class Graph;
template <typename TVertex, typename TEdge> class AdjacencyList;
template <typename TVertex, typename TEdge> class AdjacencyMatrix;

class Vertex {
private:
    int m_id = -1;
    std::string m_name;
    std::vector<Vehicle*> m_availableVehicles;
    bool m_active = true;

protected:
    void setId(int id) { m_id = id; }

public:
    Vertex() : m_name() {}
    explicit Vertex(const std::string& name) : m_name(name) {}
    virtual ~Vertex() {
        for (auto* vehicle : m_availableVehicles) {
            delete vehicle;
        }
        m_availableVehicles.clear();
    }

    int getId() const { return m_id; }
    const std::string& getName() const { return m_name; }
    const std::vector<Vehicle*>& getAvailableVehicles() const { return m_availableVehicles; }
    void addVehicle(Vehicle* vehicle) { if(vehicle) m_availableVehicles.push_back(vehicle); }
    void markInactive() { m_active = false; }
    bool isActive() const { return m_active; }
    template <typename TVertex, typename TEdge> friend class Graph;
    template <typename TVertex, typename TEdge> friend class AdjacencyList;
    template <typename TVertex, typename TEdge> friend class AdjacencyMatrix;
};