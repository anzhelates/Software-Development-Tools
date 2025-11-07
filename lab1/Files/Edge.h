#pragma once

#include <string>
#include <vector>
#include "Vertex.h"
#include "Vehicle.h"
#include "EnumClasses.h"

/**
 * @file Edge.h
 * @brief Declares the Edge class representing a connection between two cities with travel conditions
 * @struct Obstacle
 * @brief Represents an obstacle that can affect travel time
 */
struct Obstacle {
    std::string name; ///< The name of the obstacle
    double delayHours = 0.0; ///< A fixed time delay in hours caused by the obstacle
    ObstacleCause cause = ObstacleCause::CUSTOM_DELAY; ///< The type of obstacle that is used to calculate speed reduction
};

/**
 * @class Edge
 * @brief Represents an edge between two vertices
 */
class Edge {
    private:
        Vertex* m_source; ///< Pointer to the source vertex
        Vertex* m_destination; ///< Pointer to the destination vertex
        double m_distance; ///< The distance of the edge in km
        RoadType m_type; ///< The type of the path (ROAD, AIR,...)
        RoadCharacteristic m_characteristic; ///< The characteristic of the path (HIGHWAY,...)
        std::vector<Obstacle*> m_obstacles; ///< A list of obstacles
        bool m_active = false; ///< Check whether the edge is usable

    public:
        /**
         * @brief Constructs an Edge
         * @param source Pointer to the source vertex
         * @param destination Pointer to the destination vertex
         * @param distance The distance in km
         * @param type The type of the road
         * @param characteristic The characteristic of the road
         */
        Edge(Vertex* source, Vertex* destination, double distance,
             RoadType type = RoadType::ROAD, RoadCharacteristic characteristic = RoadCharacteristic::STANDARD);

        /**
         * @brief Virtual destructor (implemented in Edge.cpp)
         */
        virtual ~Edge();

        /**
         * @brief Adds an obstacle to the edge
         * @param obstacle A pointer to the Obstacle to add
         */
        void addObstacle(Obstacle* obstacle);

        Vertex* getSource() const { return m_source; } /** @brief gets the source vertex */
        Vertex* getDestination() const { return m_destination; } /** @brief gets the destination vertex */
        double getDistance() const { return m_distance; } /** @brief Gets the distance of the edge */
        RoadType getType() const { return m_type; }  /** @brief gets the road type */
        RoadCharacteristic getCharacteristic() const { return m_characteristic; } /** @brief Gets the road characteristic */
        std::vector<Obstacle*> getObstacles() const { return m_obstacles; } /** @brief Gets the list of obstacles on the edge */

        /** @brief getFrom() gets the ID of the source vertex, getTo() gets the ID of the destination vertex
         * @return the integer ID, or -1 if source/destination is null */
        int getFrom() const { return m_source ? m_source->getId() : -1; }
        int getTo() const { return m_destination ? m_destination->getId() : -1; }

        /**
         * @brief Calculates the total time to travel this edge on a given vehicle
         * @param vehicle The vehicle to calculate travel time for
         * @return The total travel time in hours
         */
        double calculateTravelTime(const Vehicle& vehicle) const;

        /** @brief markInactive() marks the edge as inactive (cannot be used)
         * markActive() marks the edge as active (can be used)
         * isActive() checks if the edge is active
         */
        void markInactive() { m_active = false; }
        void markActive() { m_active = true; }
        bool isActive() const { return m_active; }
};