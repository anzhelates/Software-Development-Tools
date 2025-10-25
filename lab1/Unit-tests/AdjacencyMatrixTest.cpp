#include "doctest.h"

#include "AdjacencyMatrix.h"
#include "City.h"
#include "Edge.h"
#include <algorithm>

static Edge* makeEdge(Vertex* from, Vertex* to, double dist = 10.0, RoadType type = RoadType::ROAD, RoadCharacteristic rc = RoadCharacteristic::STANDARD) {
    Edge* edge = new Edge(from, to, dist, type, rc);
    edge->markActive();
    return edge;
}
static City* makeCity(const std::string& name, long population = 1000) {
    return new City(name, population);
}

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

TEST_CASE("edges sum and invalid ID") {
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

TEST_CASE("boundary and error conditions") {
    AdjacencyMatrix<City, Edge> g(true);
    auto* a = makeCity("A");
    int id_a = g.addVertex(a);

    CHECK(g.getEdge(-1, id_a) == nullptr);
    CHECK(g.getEdge(id_a, 100) == nullptr);
    CHECK(g.getEdgesFrom(-1).empty());
    CHECK(g.getEdgesFrom(100).empty());
    CHECK(g.getNeighbors(-1).empty());
    CHECK(g.getNeighbors(100).empty());
    CHECK(g.getVertexById(-1) == nullptr);
    CHECK(g.getVertexById(100) == nullptr);

    CHECK(g.addVertex(nullptr) == -1);
    g.addEdge(nullptr);
    g.removeEdge(nullptr);

    g.removeVertex(-1);
    g.removeVertex(100);

    CHECK(g.getNumberOfVertices() == 1);
    CHECK(g.getVertexById(id_a) == a);
}