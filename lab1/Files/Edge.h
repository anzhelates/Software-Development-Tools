#pragma once

#include <string>
#include <vector>
#include "Vertex.h"
#include "Vehicle.h"
#include "EnumClasses.h"

struct Obstacle {
    std::string name;
    double delayHours = 0.0;
    ObstacleCause cause = ObstacleCause::CUSTOM_DELAY;
};

class Edge {
private:
    Vertex* m_source;
    Vertex* m_destination;
    double m_distance;
    RoadType m_type;
    RoadCharacteristic m_characteristic;
    std::vector<Obstacle*> m_obstacles;
    bool m_active = false;

public:
    Edge(Vertex* source, Vertex* destination, double distance,
        RoadType type = RoadType::ROAD, RoadCharacteristic characteristic = RoadCharacteristic::STANDARD);

    virtual ~Edge();
    void addObstacle(Obstacle* obstacle);

    Vertex* getSource() const { return m_source; }
    Vertex* getDestination() const { return m_destination; }
    double getDistance() const { return m_distance; }
    RoadType getType() const { return m_type; }
    RoadCharacteristic getCharacteristic() const { return m_characteristic; }
    std::vector<Obstacle*> getObstacles() const { return m_obstacles; }

    int getFrom() const { return m_source ? m_source->getId() : -1; }
    int getTo() const { return m_destination ? m_destination->getId() : -1; }

    double calculateTravelTime(const Vehicle& vehicle) const;

    void markInactive() { m_active = false; }
    void markActive() { m_active = true; }
    bool isActive() const { return m_active; }
};