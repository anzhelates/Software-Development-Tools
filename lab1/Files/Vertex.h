#pragma once

#include <string>
#include <vector>
#include <stdexcept>
#include "Vehicle.h"

template <typename TVertex, typename TEdge> class Graph;
template <typename TVertex, typename TEdge> class AdjacencyList;
template <typename TVertex, typename TEdge> class AdjacencyMatrix;

/**
 * @file Vertex.h
 * @brief Declares the Vertex class representing a graph node
 * @class Vertex
 * @brief Represents a vertex in a graph
 * @details A vertex has a unique ID, a name, and can have a list of available vehicles
 * It also has an active status
 */
class Vertex {
    private:
        int m_id = -1; ///< The unique ID of the vertex, assigned by the graph
        std::string m_name; ///< The name of the vertex (might be a city name)
        std::vector<Vehicle*> m_availableVehicles; ///< Vehicles available at this vertex
        bool m_active = true; ///< Checks whether the vertex is currently usable

    protected:
        /**
         * @brief Sets the ID of the vertex
         * @param id The new ID for the vertex
         */
        void setId(int id) { m_id = id; }

    public:
        /**
         * @brief Default constructor for Vertex
         * @details Creates a vertex with an empty name and default values
         * The ID is set to -1 and the vertex is active by default
         */
        Vertex() : m_name() {}

        /**
         * @brief Constructs a vertex with a name
         * @param name The name of the vertex
         * @throws std::invalid_argument If the name is empty
         */
        explicit Vertex(const std::string& name) : m_name(name) {
            if (name.empty()) {
                throw std::invalid_argument("Vertex name cannot be empty");
            }
        }

        /**
         * @brief Virtual Vertex destructor
         * @details Deletes all available vehicles owned by this vertex
         */
        virtual ~Vertex() {
            for (auto* vehicle : m_availableVehicles) {
                delete vehicle;
            }
            m_availableVehicles.clear();
        }

        /**
         * @brief Gets the ID of the vertex
         * @return The ID of the vertex
         */
        int getId() const { return m_id; }
        /**
         * @brief Gets the name of the vertex
         * @return The name of the vertex
         */
        const std::string& getName() const { return m_name; }
        /**
         * @brief Gets the vehicles available for the particular vertex (City)
         * @return The vehicles available for the vertex (City)
         */
        const std::vector<Vehicle*>& getAvailableVehicles() const { return m_availableVehicles; }

        /**
         * @brief Adds a vehicle to the list of available vehicles at this vertex
         * @param vehicle A pointer to the vehicle to add
         */
        void addVehicle(Vehicle* vehicle) { if(vehicle) m_availableVehicles.push_back(vehicle); }

        /** @brief Marks the vertex as inactive */
        void markInactive() { m_active = false; }
        /**
         * @brief Checks whether the vertex is active
         * @return True if the vertex is active, false otherwise
         */
        bool isActive() const { return m_active; }

        /** @brief Friend class declarations to allow Graph implementations to call setId */
        template <typename TVertexT, typename TEdgeT> friend class Graph;
        template <typename TVertex, typename TEdge> friend class AdjacencyList;
        template <typename TVertex, typename TEdge> friend class AdjacencyMatrix;
};