#include "doctest.h"
#include "AdjacencyMatrix.h"
#include "City.h"
#include "Edge.h"
#include <algorithm>
#include <stdexcept>

/**
 * @file AdjacencyMatrixTest.cpp
 * @brief Unit tests for the AdjacencyMatrix class
 * @brief Unit tests for the AdjacencyMatrix class
 */

/**
 * @brief Helper function to create a new Edge and mark it active
 * @param from Pointer to the source vertex
 * @param to Pointer to the destination vertex
 * @param dist Distance of the edge (default 10.0)
 * @param type Road type of the edge (default ROAD)
 * @param rc Road characteristic of the edge (default STANDARD)
 * @return Pointer to the newly created Edge
 */
static Edge* makeEdge(Vertex* from, Vertex* to, double dist = 10.0, RoadType type = RoadType::ROAD, RoadCharacteristic rc = RoadCharacteristic::STANDARD) {
    Edge* edge = new Edge(from, to, dist, type, rc);
    edge->markActive();
    return edge;
}

/**
 * @brief Helper function to create a new City (which is a Vertex)
 * @param name The name of the city
 * @param population The population of the city (default 1000)
 * @return Pointer to the newly created City
 */
static City* makeCity(const std::string& name, long population = 1000) {
    return new City(name, population);
}

/**
 * @brief Tests vertex/edge addition in a directed graph
 */
TEST_CASE("addVertex, addEdge, directed") {
    AdjacencyMatrix<City, Edge> g(true);
    auto* a = makeCity("A");
    auto* b = makeCity("B");

    int id_a = g.addVertex(a);
    int id_b = g.addVertex(b);

    auto* e = makeEdge(a, b, 8.0);
    g.addEdge(e);

    CHECK(g.getEdge(id_a, id_b) == e);
    CHECK(g.getEdge(id_b, id_a) == nullptr);

    CHECK(a->isActive());
    CHECK(b->isActive());
    CHECK(e->isActive());
}

/**
 * @brief Tests 'getEdge' and 'getNeighbors' methods in an undirected graph
 */
TEST_CASE("undirected") {
    AdjacencyMatrix<City, Edge> g(false);
    auto* a = makeCity("A");
    auto* b = makeCity("B");

    int id_a = g.addVertex(a);
    int id_b = g.addVertex(b);

    auto* e = makeEdge(a, b, 12.0);
    g.addEdge(e);

    CHECK(g.getEdge(id_a, id_b) == e);
    CHECK(g.getEdge(id_b, id_a) == e);

    auto nA = g.getNeighbors(id_a);
    auto nB = g.getNeighbors(id_b);
    CHECK(nA.size() == 1);
    CHECK(nB.size() == 1);
    CHECK(nA[0] == id_b);
    CHECK(nB[0] == id_a);
}

/**
 * @brief Tests the 'removeEdge' and 'removeVertex' methods
 */
TEST_CASE("removeEdge and removeVertex") {
    AdjacencyMatrix<City, Edge> g(false);
    auto* a = makeCity("A");
    auto* b = makeCity("B");
    auto* c = makeCity("C");

    int id_a = g.addVertex(a);
    int id_b = g.addVertex(b);
    int id_c = g.addVertex(c);

    auto* e1 = makeEdge(a, b);
    auto* e2 = makeEdge(b, c);
    g.addEdge(e1);
    g.addEdge(e2);

    g.removeEdge(e1);
    CHECK_FALSE(e1->isActive());
    CHECK(g.getEdge(id_a, id_b) == nullptr);

    g.removeVertex(id_b);
    CHECK_FALSE(b->isActive());
    CHECK_FALSE(e2->isActive());
    CHECK(g.getEdge(id_a, id_b) == nullptr);
    CHECK(g.getEdge(id_b, id_c) == nullptr);
}

/**
 * @brief Tests neighbor search and edge distance summation
 */
TEST_CASE("edges sum and neighbors") {
    AdjacencyMatrix<City, Edge> g(true);
    auto* a = makeCity("A");
    auto* b = makeCity("B");
    auto* c = makeCity("C");

    int id_a = g.addVertex(a);
    int id_b = g.addVertex(b);
    int id_c = g.addVertex(c);

    auto* e1 = makeEdge(a, b, 10.0);
    auto* e2 = makeEdge(a, c, 20.0);
    g.addEdge(e1);
    g.addEdge(e2);

    auto nA = g.getNeighbors(id_a);
    CHECK(nA.size() == 2);
    CHECK(std::find(nA.begin(), nA.end(), id_b) != nA.end());
    CHECK(std::find(nA.begin(), nA.end(), id_c) != nA.end());

    double sum = 0;
    for (auto* edge : g.getEdges()) {
        if (edge->isActive()) sum += edge->getDistance();
    }
    CHECK(sum == doctest::Approx(30.0));
}

/**
 * @brief Tests boundary and error conditions for various graph methods
 * @throws std::out_of_range Confirms thrown for out-of-bounds IDs
 * @throws std::invalid_argument Confirms thrown for null pointers
 */
TEST_CASE("boundary and error conditions") {
    AdjacencyMatrix<City, Edge> g(true);
    auto* a = makeCity("A");
    int id_a = g.addVertex(a);

    CHECK_THROWS_AS(g.getEdge(-1, id_a), std::out_of_range);
    CHECK_THROWS_AS(g.getEdge(id_a, 100), std::out_of_range);
    CHECK_THROWS_AS(g.getEdgesFrom(-1), std::out_of_range);
    CHECK_THROWS_AS(g.getEdgesFrom(100), std::out_of_range);
    CHECK_THROWS_AS(g.getNeighbors(-1), std::out_of_range);
    CHECK_THROWS_AS(g.getNeighbors(100), std::out_of_range);
    CHECK_THROWS_AS(g.getVertexById(-1), std::out_of_range);
    CHECK_THROWS_AS(g.getVertexById(100), std::out_of_range);
    CHECK_THROWS_AS(g.removeVertex(-1), std::out_of_range);
    CHECK_THROWS_AS(g.removeVertex(100), std::out_of_range);

    CHECK_THROWS_AS(g.addVertex(nullptr), std::invalid_argument);
    CHECK_THROWS_AS(g.addEdge(nullptr), std::invalid_argument);
    CHECK_THROWS_AS(g.removeEdge(nullptr), std::invalid_argument);

    CHECK(g.getNumberOfVertices() == 1);
    CHECK(g.getVertexById(id_a) == a);
}