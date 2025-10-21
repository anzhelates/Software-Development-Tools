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

template<typename TGraph>
TGraph buildSimpleGraph(bool directed = false) {
    TGraph g(directed);

    City* a = new City("A", 1000);
    City* b = new City("B", 2000);
    City* c = new City("C", 1500);

    g.addVertex(a);
    g.addVertex(b);
    g.addVertex(c);

    g.addEdge(new Edge(a, b, 10.0, RoadType::ROAD));
    g.addEdge(new Edge(b, c, 20.0, RoadType::ROAD));
    g.addEdge(new Edge(a, c, 50.0, RoadType::ROAD));

    return g;
}

TEST_SUITE("BFSTestSuite") {
    TEST_CASE("empty graph") {
        AdjacencyList<City, Edge> g;
        auto res = GraphAlgorithms::BFS(g, 0);
        CHECK(res.empty());
    }

    TEST_CASE("single vertex") {
        AdjacencyList<City, Edge> g;
        City* a = new City("A", 1000);
        int idA = g.addVertex(a);
        auto res = GraphAlgorithms::BFS(g, idA);
        REQUIRE(res.size() == 1);
        CHECK(res[0] == idA);
    }

    TEST_CASE("simple undirected graph") {
        auto g = buildSimpleGraph<AdjacencyList<City, Edge>>(false);
        auto res = GraphAlgorithms::BFS(g, 0);
        CHECK(res.size() == 3);
        CHECK(std::find(res.begin(), res.end(), 0) != res.end());
        CHECK(std::find(res.begin(), res.end(), 1) != res.end());
        CHECK(std::find(res.begin(), res.end(), 2) != res.end());
    }
}

TEST_SUITE("DFSTestSuite") {
    TEST_CASE("empty graph") {
        AdjacencyMatrix<City, Edge> g;
        auto res = GraphAlgorithms::DFS(g, 0);
        CHECK(res.empty());
    }

    TEST_CASE("simple graph") {
        auto g = buildSimpleGraph<AdjacencyMatrix<City, Edge>>(false);
        auto res = GraphAlgorithms::DFS(g, 0);
        CHECK(res.size() == 3);
        CHECK(std::find(res.begin(), res.end(), 0) != res.end());
        CHECK(std::find(res.begin(), res.end(), 1) != res.end());
        CHECK(std::find(res.begin(), res.end(), 2) != res.end());
    }
}

TEST_SUITE("DijkstraTestSuite") {
    TEST_CASE("unreachable vertices") {
        AdjacencyList<City, Edge> g;
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

    TEST_CASE("shortest path undirected") {
        auto g = buildSimpleGraph<AdjacencyList<City, Edge>>(false);
        Car car("Car", 100, 8);
        auto dist = GraphAlgorithms::Dijkstra(g, 0, car);
        REQUIRE(dist.size() == 3);
        CHECK(dist[0] == doctest::Approx(0.0));
        CHECK(dist[1] < dist[2]);
    }

    TEST_CASE("obstacle impact") {
        AdjacencyList<City, Edge> g;
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
        CHECK(dist[idB] > 1.0);
    }

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

    TEST_CASE("multiple obstacles on one edge") {
        AdjacencyList<City, Edge> g;
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
        CHECK(dist[idB] > 1.0);
    }

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
        CHECK(distBike[idB] > distCar[idB]);
    }
}

TEST_SUITE("isConnectedTestSuite") {
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