#include "doctest.h"
#include "GraphAlgorithm.h"
#include "AdjacencyList.h"
#include "AdjacencyMatrix.h"
#include "City.h"
#include "Edge.h"
#include "EnumClasses.h"
#include "Car.h"
#include "LandVehicle.h"
#include <algorithm>
#include <limits>
#include <stdexcept>

/**
 * @file GraphAlgorithmTest.cpp
 * @brief Unit tests for the GraphAlgorithms class
 * @details Tests for BFS, DFS, Dijkstra's algorithm, and connectivity checks (isConnected)
 */

/**
 * @brief Helper function to build a simple 3-vertex graph
 * @tparam TGraph The type of graph (AdjacencyList or AdjacencyMatrix)
 * @param g A reference to the graph object
 * @param[out] idA The integer ID assigned to vertex 'A'
 * @param[out] idB The integer ID assigned to vertex 'B'
 * @param[out] idC The integer ID assigned to vertex 'C'
 * @details Adds three cities (A, B, C) and three edges (A-B, B-C, A-C) to the given graph
 */
template<typename TGraph>
void buildSimpleGraph(TGraph& g, int& idA, int& idB, int& idC) {
    City* a = new City("A", 1000);
    City* b = new City("B", 2000);
    City* c = new City("C", 1500);

    idA = g.addVertex(a);
    idB = g.addVertex(b);
    idC = g.addVertex(c);

    g.addEdge(new Edge(a, b, 10.0, RoadType::ROAD));
    g.addEdge(new Edge(b, c, 20.0, RoadType::ROAD));
    g.addEdge(new Edge(a, c, 50.0, RoadType::ROAD));
}


TEST_SUITE("BFSTestSuite") {
    /**
     * @brief Tests BFS on an empty graph
     * @throws std::out_of_range Thrown because the startId does not exist in the graph
     */
    TEST_CASE("empty graph") {
        AdjacencyList<City, Edge> g(true);
        CHECK_THROWS_AS(GraphAlgorithms::BFS(g, 0), std::out_of_range);
    }

    /**
     * @brief Tests BFS on a graph with a single vertex
     */
    TEST_CASE("single vertex") {
        AdjacencyList<City, Edge> g(true);
        City* a = new City("A", 1000);
        int idA = g.addVertex(a);
        auto res = GraphAlgorithms::BFS(g, idA);
        REQUIRE(res.size() == 1);
        CHECK(res[0] == idA);
    }

    /**
     * @brief Tests BFS on a simple 3-node connected graph
     */
    TEST_CASE("simple undirected graph") {
        AdjacencyList<City, Edge> g(false);
        int idA, idB, idC;
        buildSimpleGraph(g, idA, idB, idC);

        auto res = GraphAlgorithms::BFS(g, idA);
        CHECK(res.size() == g.getNumberOfVertices());
        CHECK(std::find(res.begin(), res.end(), idA) != res.end());
        CHECK(std::find(res.begin(), res.end(), idB) != res.end());
        CHECK(std::find(res.begin(), res.end(), idC) != res.end());
    }

    /**
     * @brief Tests BFS with invalid start IDs
     * @throws std::out_of_range Expected for both sub-checks
     */
    TEST_CASE("invalid start ID") {
        AdjacencyList<City, Edge> g(false);
        int idA, idB, idC;
        buildSimpleGraph(g, idA, idB, idC);

        CHECK_THROWS_AS(GraphAlgorithms::BFS(g, -1), std::out_of_range);
        CHECK_THROWS_AS(GraphAlgorithms::BFS(g, 100), std::out_of_range);
    }
}

TEST_SUITE("DFSTestSuite") {
    /**
     * @brief Tests DFS on an empty graph
     * @throws std::out_of_range Thrown because the startId does not exist in the graph
     */
    TEST_CASE("empty graph") {
        AdjacencyMatrix<City, Edge> g(true);
        CHECK_THROWS_AS(GraphAlgorithms::DFS(g, 0), std::out_of_range);
    }

    /**
     * @brief Tests DFS on a simple 3-node connected graph
     */
    TEST_CASE("simple graph") {
        AdjacencyMatrix<City, Edge> g(false);
        int idA, idB, idC;
        buildSimpleGraph(g, idA, idB, idC);

        auto res = GraphAlgorithms::DFS(g, idA);
        CHECK(res.size() == g.getNumberOfVertices());
        CHECK(std::find(res.begin(), res.end(), idA) != res.end());
        CHECK(std::find(res.begin(), res.end(), idB) != res.end());
        CHECK(std::find(res.begin(), res.end(), idC) != res.end());
    }

    /**
     * @brief Tests DFS with invalid start IDs
     * @throws std::out_of_range Expected for both sub-checks
     */
    TEST_CASE("invalid start ID") {
        AdjacencyMatrix<City, Edge> g(false);
        int idA, idB, idC;
        buildSimpleGraph(g, idA, idB, idC);

        CHECK_THROWS_AS(GraphAlgorithms::DFS(g, -1), std::out_of_range);
        CHECK_THROWS_AS(GraphAlgorithms::DFS(g, 100), std::out_of_range);
    }
}

TEST_SUITE("DijkstraTestSuite") {
    /**
     * @brief Tests Dijkstra's algorithm in a directed graph with an unreachable vertex
     */
    TEST_CASE("unreachable vertices") {
        AdjacencyList<City, Edge> g(true);
        City* a = new City("A", 1000);
        City* b = new City("B", 2000);
        int idA = g.addVertex(a);
        int idB = g.addVertex(b);

        Car car("Car", 100, 8);
        auto dist = GraphAlgorithms::Dijkstra(g, idA, car);

        REQUIRE(dist.size() == 2);
        CHECK(dist[idA] == doctest::Approx(0.0));
        CHECK(dist[idB] == std::numeric_limits<double>::infinity());
    }

    /**
     * @brief Tests Dijkstra's algorithm on a simple undirected graph
     */
    TEST_CASE("shortest path undirected") {
        AdjacencyList<City, Edge> g(false);
        int idA, idB, idC;
        buildSimpleGraph(g, idA, idB, idC);

        Car car("Car", 100, 8);
        auto dist = GraphAlgorithms::Dijkstra(g, idA, car);

        REQUIRE(dist.size() == 3);
        CHECK(dist[idA] == doctest::Approx(0.0));
        CHECK(dist[idB] == doctest::Approx(0.1));
        CHECK(dist[idC] == doctest::Approx(0.3));
    }

    /**
     * @brief Tests Dijkstra's algorithm on an edge with an obstacle
     */
    TEST_CASE("obstacle impact") {
        AdjacencyList<City, Edge> g(true);
        City* a = new City("A", 1000);
        City* b = new City("B", 2000);
        int idA = g.addVertex(a);
        int idB = g.addVertex(b);

        Edge* ab = new Edge(a, b, 100.0, RoadType::ROAD);
        ab->addObstacle(new Obstacle{"Traffic jam", 2.0, ObstacleCause::TRAFFIC_JAM});
        g.addEdge(ab);

        Car car("Car", 100, 8);
        auto dist = GraphAlgorithms::Dijkstra(g, idA, car);

        REQUIRE(dist.size() == 2);
        CHECK(dist[idB] == doctest::Approx(4.5));
    }

    /**
     * @brief Tests Dijkstra's algorithm on a directed graph with no forward path
    */
    TEST_CASE("directed graph unreachable") {
        AdjacencyList<City, Edge> g(true);
        City* a = new City("A", 1000);
        City* b = new City("B", 2000);

        int idA = g.addVertex(a);
        int idB = g.addVertex(b);

        g.addEdge(new Edge(b, a, 10.0, RoadType::ROAD));

        Car car("Car", 100, 8);
        auto dist = GraphAlgorithms::Dijkstra(g, idA, car);
        CHECK(dist[idB] == std::numeric_limits<double>::infinity());
    }

    /**
     * @brief Tests Dijkstra's algorithm on an edge with multiple obstacles
     */
    TEST_CASE("multiple obstacles on one edge") {
        AdjacencyList<City, Edge> g(true);
        City* a = new City("A", 1000);
        City* b = new City("B", 2000);
        int idA = g.addVertex(a);
        int idB = g.addVertex(b);

        Edge* ab = new Edge(a, b, 100.0, RoadType::ROAD);
        ab->addObstacle(new Obstacle{"Jam", 2.0, ObstacleCause::TRAFFIC_JAM});
        ab->addObstacle(new Obstacle{"Construction", 1.0, ObstacleCause::CONSTRUCTION});
        g.addEdge(ab);

        Car car("Car", 100, 8);
        auto dist = GraphAlgorithms::Dijkstra(g, idA, car);

        CHECK(dist[idB] == doctest::Approx(5.5));
    }

    /**
     * @brief Tests Dijkstra's algorithm with different vehicle types
     */
    TEST_CASE("bike slower than car") {
        AdjacencyList<City, Edge> g(false);
        City* a = new City("A", 1000);
        City* b = new City("B", 2000);
        int idA = g.addVertex(a);
        int idB = g.addVertex(b);

        g.addEdge(new Edge(a, b, 100.0, RoadType::ROAD));

        Car car("Car", 100, 8);
        LandVehicle bike("Bike", 20, 0);

        auto distBike = GraphAlgorithms::Dijkstra(g, idA, bike);
        auto distCar = GraphAlgorithms::Dijkstra(g, idA, car);

        // travelTime(bike) = 100.0 / 20.0 = 5.0
        // travelTime(car) = 100.0 / 100.0 = 1.0
        CHECK(distBike[idB] > distCar[idB]);
        CHECK(distBike[idB] == doctest::Approx(5.0));
    }

    /**
     * @brief Tests Dijkstra's algorithm with invalid start IDs
     * @throws std::out_of_range Expected for both sub-checks
     */
    TEST_CASE("invalid start ID") {
        AdjacencyList<City, Edge> g(false);
        int idA, idB, idC;
        buildSimpleGraph(g, idA, idB, idC);
        Car car("Car", 100, 8);

        CHECK_THROWS_AS(GraphAlgorithms::Dijkstra(g, -1, car), std::out_of_range);
        CHECK_THROWS_AS(GraphAlgorithms::Dijkstra(g, 100, car), std::out_of_range);
    }
}

TEST_SUITE("isConnectedTestSuite") {

    /**
     * @brief Tests 'isConnected' on a connected undirected graph
     */
    TEST_CASE("undirected connected") {
        AdjacencyList<City, Edge> g(false);
        City* a = new City("A", 1000);
        City* b = new City("B", 2000);
        City* c = new City("C", 1500);
        g.addVertex(a);
        g.addVertex(b);
        g.addVertex(c);

        g.addEdge(new Edge(a, b, 10.0));
        g.addEdge(new Edge(b, c, 5.0));

        CHECK(GraphAlgorithms::isConnected(g));
    }

    /**
     * @brief Tests 'isConnected' on a disconnected undirected graph
     */
    TEST_CASE("undirected disconnected") {
        AdjacencyList<City, Edge> g(false);
        City* a = new City("A", 1000);
        City* b = new City("B", 2000);
        City* c = new City("C", 1500);
        g.addVertex(a);
        g.addVertex(b);
        g.addVertex(c);

        g.addEdge(new Edge(a, b, 10.0));
        CHECK_FALSE(GraphAlgorithms::isConnected(g));
    }

    /**
     * @brief Tests 'isConnected' on a strongly connected directed graph
     */
    TEST_CASE("directed strongly connected") {
        AdjacencyList<City, Edge> g(true);
        City* a = new City("A", 1000);
        City* b = new City("B", 2000);
        City* c = new City("C", 1500);
        g.addVertex(a);
        g.addVertex(b);
        g.addVertex(c);

        g.addEdge(new Edge(a, b, 10.0));
        g.addEdge(new Edge(b, c, 5.0));
        g.addEdge(new Edge(c, a, 7.0));

        CHECK(GraphAlgorithms::isConnected(g));
    }

    /**
     * @brief Tests 'isConnected' on a weakly connected directed graph
     */
    TEST_CASE("directed weakly connected") {
        AdjacencyList<City, Edge> g(true);
        City* a = new City("A", 1000);
        City* b = new City("B", 2000);
        City* c = new City("C", 1500);
        g.addVertex(a);
        g.addVertex(b);
        g.addVertex(c);

        g.addEdge(new Edge(a, b, 10.0));
        g.addEdge(new Edge(b, c, 5.0));
        CHECK_FALSE(GraphAlgorithms::isConnected(g));
    }

    /**
     * @brief Tests 'isConnected' on a disconnected directed graph
     */
    TEST_CASE("directed disconnected") {
        AdjacencyList<City, Edge> g(true);
        City* a = new City("A", 1000);
        City* b = new City("B", 2000);
        City* c = new City("C", 1500);

        g.addVertex(a);
        g.addVertex(b);
        g.addVertex(c);

        g.addEdge(new Edge(a, b, 10.0, RoadType::ROAD));
        CHECK_FALSE(GraphAlgorithms::isConnected(g));
    }
}