#pragma once

#include <string>
#include "Vertex.h"

class City : public Vertex {
private:
    long m_population;
public:
    City(const std::string& name, long population) : Vertex(name), m_population(population)  {}

    long getPopulation() const { return m_population; };
    bool isDenselyPopulated() const {
        return m_population >= 1000000;
    }
};