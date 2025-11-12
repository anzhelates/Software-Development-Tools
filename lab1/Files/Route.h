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

        /**
         * @brief Sets the vehicle for the particular route
         * @param vehicle Pointer to the vehicle to assign
         */
        void setVehicle(const Vehicle* vehicle) { m_vehicle = vehicle; }
        /**
         * @brief Sets the starting vertex of the route
         * @param start Pointer to the start vertex
         * @throws std::invalid_argument if the start vertex if nullptr
         */
        void setStart(Vertex* start) {
            if (!start) throw std::invalid_argument("Start vertex cannot be null");
            m_startVertex = start;
        }
        /**
         * @brief Sets the ending vertex of the route
         * @param end Pointer to the end vertex
         * @throws std::invalid_argument if end is nullptr
         */
        void setEnd(Vertex* end) {
            if (!end) throw std::invalid_argument("End vertex cannot be null");
            m_endVertex = end;
        }
        /**
         * @brief Gets the starting vertex of the route
         * @return Pointer to the start vertex
         */
        Vertex* getStart() const { return m_startVertex; }
        /**
         * @brief Gets the ending vertex of the route
         * @return Pointer to the end vertex
         */
        Vertex* getEnd() const { return m_endVertex; }
        /**
         * @brief Gets the vehicle assigned to this route
         * @return Pointer to the assigned vehicle
         */
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
         * @example
         * @code
         * City* cityA = new City("A", 500000);
         * City* cityB = new City("B", 1200000);
         * City* cityC = new City("C", 900000);
         *
         * Car car("Car", 100, 7.5);
         * Route route(cityA, cityC, &car);
         * // Add the route A -> B -> C
         * Edge* AB = new Road(cityA, cityB, 120);
         * Edge* BC = new Road(cityB, cityC, 80);
         * route.addEdge(AB);
         * route.addEdge(BC);
         *
         * double time = route.totalTime(car); // 2.0 h (≈120/100 + 80/100)
         * double distance = route.totalDistance(); // 200 km
         * double fuel = route.totalFuel(car); // 15 l (≈7.5 l/100 km * 200)
         *
         * std::cout << "Time: " << time << " h, Distance: " << distance << " km, Fuel: " << fuel << " l" << std::endl;
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