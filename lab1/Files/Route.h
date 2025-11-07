#pragma once

#include <vector>
#include <stdexcept>
#include "Edge.h"
#include "Vehicle.h"
#include "Vertex.h"

/**
 * @file Route.h
 * @brief Declares the Route class describing a path, distance, and traversal data between cities
 * @class Route
 * @brief Represents a calculated path from a start vertex to an end vertex
 * @details Provides methods to calculate total time, distance, and fuel consumption
 */

class Route {
    private:
        Vertex* m_startVertex = nullptr; ///< The starting vertex of the route
        Vertex* m_endVertex = nullptr; ///< The ending vertex of the route
        const Vehicle* m_vehicle = nullptr; ///< The vehicle used for the route
        std::vector<Edge*> m_path; ///< The sequence of edges that form the path

    public:
        Route() = default;

        /**
         * @brief Constructs a Route with start, end, and vehicle
         * @param start The starting vertex
         * @param end The ending vertex
         * @param vehicle The vehicle to be used
         * @throws std::invalid_argument If start or end vertex pointers are null
         */
        Route(Vertex* start, Vertex* end, const Vehicle* vehicle)
            : m_startVertex(start), m_endVertex(end), m_vehicle(vehicle) {
            if (!start || !end) {
                throw std::invalid_argument("Route start and end vertices cannot be null");
            }
        }

        /** @brief Setters and getters */
        void setVehicle(const Vehicle* vehicle) { m_vehicle = vehicle; }
        void setStart(Vertex* start) {
            if (!start) throw std::invalid_argument("Start vertex cannot be null");
            m_startVertex = start;
        }
        void setEnd(Vertex* end) {
            if (!end) throw std::invalid_argument("End vertex cannot be null");
            m_endVertex = end;
        }

        Vertex* getStart() const { return m_startVertex; }
        Vertex* getEnd() const { return m_endVertex; }
        const Vehicle* getVehicle() const { return m_vehicle; }

        /** @brief Adds an edge to the end of the path */
        void addEdge(Edge* edge) { if (edge) m_path.push_back(edge); }
        /** @brief Gets the complete path */
        const std::vector<Edge*>& getPath() const { return m_path; }

        /**
         * @brief Calculates the total travel time for the route
         * @param vehicle The vehicle used for travel
         * @return The total time in hours
         *
         * @example Route.h
         * @code
         * // Example
         * Vertex* cityA = new Vertex("CityA");
         * Vertex* cityB = new Vertex("CityB");
         * Car myCar("MyCar", 100, 8);
         * Route myRoute(cityA, cityB, &myCar);
         * Edge* edgeAB = new Edge(cityA, cityB, 120.0, RoadType::ROAD, RoadCharacteristic::STANDARD);
         * myRoute.addEdge(edgeAB);
         * double time = myRoute.totalTime(myCar);
         * std::cout << "Total time: " << time << " hours." << std::endl;
         * @endcode
         */
        double totalTime(const Vehicle& vehicle) const {
           double total = 0.0;
             for (auto* edge : m_path) {
                  if (edge) total += edge->calculateTravelTime(vehicle);
             }
             return total;
        }

        /**
         * @brief Calculates the total distance of the route
         * @return The total distance in km
         */
        double totalDistance() const {
            double total = 0.0;
            for (auto* edge : m_path) {
                if (edge) total += edge->getDistance();
            }
            return total;
        }

        /**
         * @brief Calculates the total fuel consumption for the route
         * @param vehicle The vehicle used for the calculation
         * @return The total fuel consumed in liters
         */
        double totalFuel(const Vehicle& vehicle) const {
             double total = 0.0;
             for (auto* edge : m_path) {
                 if (edge) {
                    double distance = edge->getDistance();
                    total += vehicle.calculateFuel(distance);
                 }
             }
             return total;
        }
};