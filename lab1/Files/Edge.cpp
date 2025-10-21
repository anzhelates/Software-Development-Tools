#include "Edge.h"
#include <limits>

Edge::Edge(Vertex* source, Vertex* destination, double distance,
           RoadType type, RoadCharacteristic characteristic)
    : m_source(source), m_destination(destination), m_distance(distance), m_type(type), m_characteristic(characteristic), m_active(true) {}

Edge::~Edge() {
    for (auto* obstacle : m_obstacles) {
        delete obstacle;
    }
    m_obstacles.clear();
}

void Edge::addObstacle(Obstacle* obstacle) {
    if (obstacle) m_obstacles.push_back(obstacle);
}

double Edge::calculateTravelTime(const Vehicle& vehicle) const {
    if (!m_active || !vehicle.canUse(m_type)) {
        return std::numeric_limits<double>::infinity();
    }

    double delay = 0.0;
    double final_speed = vehicle.getSpeed(ObstacleCause::CUSTOM_DELAY, m_characteristic);

    for (const auto* obstacle : m_obstacles) {
        if (!obstacle) continue;

        delay += obstacle->delayHours;
        double speed_with_obstacle = vehicle.getSpeed(obstacle->cause, m_characteristic);

        final_speed = std::min(final_speed, speed_with_obstacle);
    }
    return m_distance / final_speed + delay;
}