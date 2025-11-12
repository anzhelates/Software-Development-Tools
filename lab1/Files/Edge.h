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

        Vertex* getSource() const { return m_source; } /** @brief Gets the source vertex */
        Vertex* getDestination() const { return m_destination; } /** @brief Gets the destination vertex */
        double getDistance() const { return m_distance; } /** @brief Gets the distance of the edge */
        RoadType getType() const { return m_type; }  /** @brief Gets the road type (ROAD, RAIL, AIR, WATER) */
        RoadCharacteristic getCharacteristic() const { return m_characteristic; } /** @brief Gets the road characteristic (HIGHWAY, CITY_STREET, DIRT_ROAD, PARK_ROAD, STANDARD, DENSELY_POPULATED_CITY, OTHER) */
        std::vector<Obstacle*> getObstacles() const { return m_obstacles; } /** @brief Gets the list of obstacles on the edge (TRAFFIC_JAM, ACCIDENT, CONSTRUCTION, WEATHER_STORM, WEATHER_SNOW, WEATHER_WIND, WEATHER_ICE, CUSTOM_DELAY) */

        /** @brief Gets the ID of the source vertex
         * @return the integer ID, or -1 if source is null */
        int getFrom() const { return m_source ? m_source->getId() : -1; }
        /** @brief Gets the ID of the destination vertex
         * @return the integer ID, or -1 if destination is null */
        int getTo() const { return m_destination ? m_destination->getId() : -1; }

        /**
         * @brief Calculates the total time to travel this edge on a given vehicle
         * @param vehicle The vehicle to calculate travel time for
         * @return The total travel time in hours
         */
        double calculateTravelTime(const Vehicle& vehicle) const;

        /** @brief Marks the edge as inactive (cannot be used) */
        void markInactive() { m_active = false; }
        /** Marks the edge as active (can be used) */
        void markActive() { m_active = true; }
        /** Checks if the edge is active
         * @return true if the edge is active, false otherwise */
        bool isActive() const { return m_active; }
};