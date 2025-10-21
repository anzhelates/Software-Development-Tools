#pragma once

#include <vector>
#include "Edge.h"
#include "Vehicle.h"
#include "Vertex.h"

class Route {
private:
    Vertex* m_startVertex = nullptr;
    Vertex* m_endVertex = nullptr;
    const Vehicle* m_vehicle = nullptr;
    std::vector<Edge*> m_path;

public:
    Route() = default;
    Route(Vertex* start, Vertex* end, const Vehicle* vehicle)
        : m_startVertex(start), m_endVertex(end), m_vehicle(vehicle) {}

    void setVehicle(const Vehicle* vehicle) { m_vehicle = vehicle; }
    void setStart(Vertex* start) { m_startVertex = start; }
    void setEnd(Vertex* end) { m_endVertex = end; }

    Vertex* getStart() const { return m_startVertex; }
    Vertex* getEnd() const { return m_endVertex; }
    const Vehicle* getVehicle() const { return m_vehicle; }

    void addEdge(Edge* edge) { if (edge) m_path.push_back(edge); }
    const std::vector<Edge*>& getPath() const { return m_path; }

    double totalTime(const Vehicle& vehicle) const {
        double total = 0.0;
        for (auto* edge : m_path) {
            total += edge->calculateTravelTime(vehicle);
        }
        return total;
    }

    double totalDistance() const {
        double total = 0.0;
        for (auto* edge : m_path) {
            total += edge->getDistance();
        }
        return total;
    }

    double totalFuel(const Vehicle& vehicle) const {
        double total = 0.0;
        for (auto* edge : m_path) {
            double distance = edge->getDistance();
            total += vehicle.calculateFuel(distance);
        }
        return total;
    }
};