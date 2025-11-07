#include "doctest.h"
#include "AdjacencyList.h"
#include "City.h"
#include "Edge.h"
#include <algorithm>
#include <stdexcept>

/**
 * @file AdjacencyListTest.cpp
 * @brief Unit tests for the AdjacencyList class.
 */

/**
 * @brief Helper function to create a new Edge
 * @param from Pointer to the source vertex
 * @param to Pointer to the destination vertex
 * @param dist Distance of the edge (default 10.0)
 * @param type Road type of the edge (default ROAD)
 * @param rc Road characteristic of the edge (default STANDARD)
 * @return Pointer to the newly created Edge
 */
static Edge* makeEdge(Vertex* from, Vertex* to, double dist = 10.0, RoadType type = RoadType::ROAD, RoadCharacteristic rc = RoadCharacteristic::STANDARD) {
    Edge* edge = new Edge(from, to, dist, type, rc);
    return edge;
}

/**
 * @brief Helper function to create a new City
 * @param name The name of the city
 * @param population The population of the city (default 1000)
 * @return Pointer to the newly created City
 */
static City* makeCity(const std::string& name, long population = 1000) {
    return new City(name, population);
}

/**
 * @brief Tests basic vertex operations: addVertex, getVertexById and getNumberOfVertices
 */
TEST_CASE("basic vertex operations") {
    AdjacencyList<City, Edge> g(false);

    CHECK(g.getNumberOfVertices() == 0);

    auto* c1 = makeCity("A", 1000);
    auto* c2 = makeCity("B", 2000);

    int id1 = g.addVertex(c1);
    int id2 = g.addVertex(c2);

    CHECK(id1 == 0);
    CHECK(id2 == 1);
    CHECK(g.getVertexById(id1)->getName() == "A");
    CHECK(g.getVertexById(id1)->getPopulation() == 1000);
    CHECK(g.getNumberOfVertices() == 2);
    CHECK(g.getVertexById(id1)->isActive());
    CHECK(g.getVertexById(id2)->isActive());
}

/**
 * @brief Tests edge operations in an undirected graph
 */
TEST_CASE("addEdge, getNeighbors, getEdge and getEdgesFrom undirected") {
    AdjacencyList<City, Edge> g(false);

    auto* c1 = makeCity("A", 1000);
    auto* c2 = makeCity("B", 2000);
    auto* c3 = makeCity("C", 1500);

    int id1 = g.addVertex(c1);
    int id2 = g.addVertex(c2);
    int id3 = g.addVertex(c3);

    auto* e1 = makeEdge(c1, c2, 5.0, RoadType::ROAD);
    auto* e2 = makeEdge(c2, c3, 7.0, RoadType::RAIL);

    g.addEdge(e1);
    g.addEdge(e2);

    /**
     * @brief Checks 'getNeighbors' for undirected graph
     * @details Verifies that if A is connected to B, B is also connected to A
     */
    SUBCASE("getNeighbors") {
        auto n1 = g.getNeighbors(id1);
        auto n2 = g.getNeighbors(id2);
        auto n3 = g.getNeighbors(id3);

        CHECK(n1.size() == 1);
        CHECK(n1[0] == id2);
        CHECK(n2.size() == 2);
        CHECK(std::find(n2.begin(), n2.end(), id1) != n2.end());
        CHECK(std::find(n2.begin(), n2.end(), id3) != n2.end());
        CHECK(n3.size() == 1);
        CHECK(n3[0] == id2);
    }
    /**
     * @brief Checks 'getEdge' for undirected graph
     * @details Verifies 'getEdge(A, B)' and 'getEdge(B, A)' return the same edge
     */
    SUBCASE("getEdge") {
        CHECK(g.getEdge(id1, id2) == e1);
        CHECK(g.getEdge(id2, id1) == e1);
        CHECK(g.getEdge(id1, id3) == nullptr);
    }
    /**
     * @brief Checks 'getEdgesFrom' for undirected graph
     */
    SUBCASE("getEdgesFrom") {
        auto edgesFrom2 = g.getEdgesFrom(id2);
        CHECK(edgesFrom2.size() == 2);
        CHECK((edgesFrom2[0] == e1 || edgesFrom2[1] == e1));
        CHECK((edgesFrom2[0] == e2 || edgesFrom2[1] == e2));
    }
}

/**
 * @brief Tests edge operations in a directed graph
 * @details Verifies that 'getNeighbors' and 'getEdge' match the edge direction
 */
TEST_CASE("directed edge") {
    AdjacencyList<City, Edge> g(true);

    auto* a = makeCity("A", 1000);
    auto* b = makeCity("B", 2000);
    int id_a = g.addVertex(a);
    int id_b = g.addVertex(b);

    auto* e = makeEdge(a, b, 12.0, RoadType::ROAD);
    g.addEdge(e);

    auto nA = g.getNeighbors(id_a);
    auto nB = g.getNeighbors(id_b);

    CHECK(nA.size() == 1);
    CHECK(nA[0] == id_b);
    CHECK(nB.empty());

    CHECK(g.getEdge(id_a, id_b)->getDistance() == doctest::Approx(12.0));
    CHECK(g.getEdge(id_b, id_a) == nullptr);
}

/**
 * @brief Tests the 'removeEdge' and 'removeVertex' methods
 */
TEST_CASE("removeEdge and removeVertex") {
    AdjacencyList<City, Edge> g(false);
    auto* a = makeCity("A", 1000);
    auto* b = makeCity("B", 2000);
    auto* c = makeCity("C", 1500);

    int id_a = g.addVertex(a);
    int id_b = g.addVertex(b);
    int id_c = g.addVertex(c);

    auto* e1 = makeEdge(a, b, 5.0);
    auto* e2 = makeEdge(b, c, 7.0);
    auto* e3 = makeEdge(a, c, 15.0);
    g.addEdge(e1);
    g.addEdge(e2);
    g.addEdge(e3);

    g.removeEdge(e1);
    CHECK_FALSE(e1->isActive());
    CHECK(g.getEdge(id_a, id_b) == nullptr);

    g.removeVertex(id_b);
    CHECK_FALSE(b->isActive());
    CHECK_FALSE(e2->isActive());
    CHECK(g.getEdge(id_a, id_c) == e3);
}

/**
 * @brief Tests 'getEdge' for correctness and handling of invalid IDs
 * @throws std::out_of_range Checks that this exception is thrown for invalid IDs
 */
TEST_CASE("getEdge correctness and invalid IDs") {
    AdjacencyList<City, Edge> g(true);
    auto* a = makeCity("A", 1000);
    auto* b = makeCity("B", 2000);
    int id_a = g.addVertex(a);
    int id_b = g.addVertex(b);

    auto* e = makeEdge(a, b);
    g.addEdge(e);

    CHECK(g.getEdge(id_a, id_b) == e);
    CHECK(g.getEdge(id_b, id_a) == nullptr);

    CHECK_THROWS_AS(g.getEdge(5, 1), std::out_of_range);
    CHECK_THROWS_AS(g.getEdge(-1, 0), std::out_of_range);
    CHECK_THROWS_AS(g.getEdge(id_a, 100), std::out_of_range);
}

/**
 * @brief Tests the sum of all edge distances in the graph
 */
TEST_CASE("edge distances sum") {
    AdjacencyList<City, Edge> g(false);
    auto* a = makeCity("A", 1000);
    auto* b = makeCity("B", 2000);
    auto* c = makeCity("C", 1500);

    g.addVertex(a);
    g.addVertex(b);
    g.addVertex(c);

    auto* e1 = makeEdge(a, b, 10.0);
    auto* e2 = makeEdge(b, c, 20.0);
    auto* e3 = makeEdge(a, c, 25.0);

    g.addEdge(e1);
    g.addEdge(e2);
    g.addEdge(e3);

    double total = 0;
    for (auto* edge : g.getEdges()) {
        if (edge->isActive()) total += edge->getDistance();
    }
    CHECK(total == doctest::Approx(55.0));
}

/**
 * @brief Tests boundary and error conditions for various graph methods
 * @throws std::out_of_range Confirms thrown for out-of-bounds IDs
 * @throws std::invalid_argument Confirms thrown for null pointers
 */
TEST_CASE("boundary and error conditions") {
    AdjacencyList<City, Edge> g(false);
    auto* a = makeCity("A");
    int id_a = g.addVertex(a);

    CHECK_THROWS_AS(g.getNeighbors(-1), std::out_of_range);
    CHECK_THROWS_AS(g.getNeighbors(100), std::out_of_range);
    CHECK_THROWS_AS(g.getEdge(-1, id_a), std::out_of_range);
    CHECK_THROWS_AS(g.getEdge(id_a, 100), std::out_of_range);
    CHECK_THROWS_AS(g.getEdgesFrom(-1), std::out_of_range);
    CHECK_THROWS_AS(g.getEdgesFrom(100), std::out_of_range);
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