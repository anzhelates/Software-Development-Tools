#pragma once

#include <string>
#include "Vertex.h"

/**
 * @file City.h
 * @brief Declares the City class representing a city node
 * @class City
 * @brief Represents a city, inherits from Vertex
 */
class City : public Vertex {
private:
    /// @brief The population of the city
    long m_population;
public:
    /**
     * @brief Constructs a City object
     * @param name The name of the city
     * @param population The population of the city
     */
    City(const std::string& name, long population) : Vertex(name), m_population(population)  {}

    /**
     * @brief Gets the population of the city
     * @return The population
     */
    long getPopulation() const { return m_population; };

    /**
     * @brief Checks if the city is considered densely populated
     * @return True if the population is 1000000 or more, false otherwise
     *
     * @example City.h
     * @code
     * // Example
     * City kyiv("Kyiv", 2800000);
     * bool isDense = kyiv.isDenselyPopulated(); // returns true
     *
     * City lviv("Lviv", 720000);
     * bool isDense2 = lviv.isDenselyPopulated(); // returns false
     * @endcode
     */
    bool isDenselyPopulated() const {
        return m_population >= 1000000;
    }
};