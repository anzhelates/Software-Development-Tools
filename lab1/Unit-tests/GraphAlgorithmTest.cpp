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
    TEST_CASE("empty graph") {
        AdjacencyList<City, Edge> g(true);
        auto res = GraphAlgorithms::BFS(g, 0);
        CHECK(res.empty());
    }

    TEST_CASE("single vertex") {
        AdjacencyList<City, Edge> g(true);
        City* a = new City("A", 1000);
        int idA = g.addVertex(a);
        auto res = GraphAlgorithms::BFS(g, idA);
        REQUIRE(res.size() == 1);
        CHECK(res[0] == idA);
    }

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

    TEST_CASE("invalid start ID") {
        AdjacencyList<City, Edge> g(false);
        int idA, idB, idC;
        buildSimpleGraph(g, idA, idB, idC);

        auto res_neg = GraphAlgorithms::BFS(g, -1);
        CHECK(res_neg.empty());
        auto res_oob = GraphAlgorithms::BFS(g, 100);
        CHECK(res_oob.empty());
    }
}

TEST_SUITE("DFSTestSuite") {
    TEST_CASE("empty graph") {
        AdjacencyMatrix<City, Edge> g(true);
        auto res = GraphAlgorithms::DFS(g, 0);
        CHECK(res.empty());
    }

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

    TEST_CASE("invalid start ID") {
        AdjacencyMatrix<City, Edge> g(false);
        int idA, idB, idC;
        buildSimpleGraph(g, idA, idB, idC);

        auto res_neg = GraphAlgorithms::DFS(g, -1);
        CHECK(res_neg.empty());
        auto res_oob = GraphAlgorithms::DFS(g, 100);
        CHECK(res_oob.empty());
    }
}

TEST_SUITE("DijkstraTestSuite") {
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

    TEST_CASE("shortest path undirected") {
        AdjacencyList<City, Edge> g(false);
        int idA, idB, idC;
        buildSimpleGraph(g, idA, idB, idC);

        Car car("Car", 100, 8);
        auto dist = GraphAlgorithms::Dijkstra(g, idA, car);

        REQUIRE(dist.size() == 3);

        /*
            distances: 10.0, 20.0, 50.0
            dist[A] = 0.0
            dist[B] = travelTime(A->B) = (distance) 10.0 / (speed) 100.0 = 0.1
            dist[C] = min(travelTime(A->C), travelTime(A->B->C))
            travelTime(A->C) = (distance) 50.0 / (speed) 100.0 = 0.5
            travelTime(A->B->C) = travelTime(A->B) + travelTime(B->C) = 0.1 + ((distance) 20.0 / (speed) 100.0) = 0.1 + 0.2 = 0.3
            min(0.5, 0.3) = 0.3
        */
        CHECK(dist[idA] == doctest::Approx(0.0));
        CHECK(dist[idB] == doctest::Approx(0.1));
        CHECK(dist[idC] == doctest::Approx(0.3));
    }

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
        /*
            Traffic jam factor for Car = 0.4
            travelTime = (distance / speed) + delayHours
            travelTime = (100.0 / 40.0) + 2.0 = 2.5 + 2.0 = 4.5
        */
        CHECK(dist[idB] == doctest::Approx(4.5));
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

        /*
            delayHours = (traffic jam delay) 2.0 + (construction delay) 1.0 = 3.0
            speed = min(speed_normal, speed_jam, speed_construction)
            traffic jam speed = 100 / 0.4 = 40, construction speed = 80 => speed = 40.
            travelTime = ((distance) 100.0 / (speed) 40.0) + (dalayHours) 3.0 = 5.5
        */
        CHECK(dist[idB] == doctest::Approx(5.5));
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

        // travelTime(bike) = 100.0 / 20.0 = 5.0
        // travelTime(car) = 100.0 / 100.0 = 1.0
        CHECK(distBike[idB] > distCar[idB]);
        CHECK(distBike[idB] == doctest::Approx(5.0));
    }

    TEST_CASE("invalid start ID") {
        AdjacencyList<City, Edge> g(false);
        int idA, idB, idC;
        buildSimpleGraph(g, idA, idB, idC);
        Car car("Car", 100, 8);

        auto dist_neg = GraphAlgorithms::Dijkstra(g, -1, car);

        REQUIRE(dist_neg.size() == g.getNumberOfVertices());
        CHECK(dist_neg[idA] == std::numeric_limits<double>::infinity());
        CHECK(dist_neg[idB] == std::numeric_limits<double>::infinity());
        CHECK(dist_neg[idC] == std::numeric_limits<double>::infinity());

        auto dist_oob = GraphAlgorithms::Dijkstra(g, 100, car);
        REQUIRE(dist_oob.size() == g.getNumberOfVertices());
        CHECK(dist_oob[idA] == std::numeric_limits<double>::infinity());
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