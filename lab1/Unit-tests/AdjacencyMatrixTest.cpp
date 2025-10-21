#include "doctest.h"

#include "AdjacencyList.h"
#include "AdjacencyMatrix.h"
#include "City.h"
#include "Edge.h"
#include "GraphAlgorithm.h"
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

    g.addVertex(a);
    g.addVertex(b);

    auto* e = makeEdge(a, b, 8.0);
    g.addEdge(e);

    CHECK(g.getEdge(a->getId(), b->getId()) == e);
    CHECK(g.getEdge(b->getId(), a->getId()) == nullptr);

    CHECK(a->isActive());
    CHECK(b->isActive());
    CHECK(e->isActive());
}

TEST_CASE("undirected") {
    AdjacencyMatrix<City, Edge> g(false);
    auto* a = makeCity("A");
    auto* b = makeCity("B");

    g.addVertex(a);
    g.addVertex(b);

    auto* e = makeEdge(a, b, 12.0);
    g.addEdge(e);

    CHECK(g.getEdge(a->getId(), b->getId()) == e);
    CHECK(g.getEdge(b->getId(), a->getId()) == e);

    auto nA = g.getNeighbors(a->getId());
    auto nB = g.getNeighbors(b->getId());
    CHECK(nA.size() == 1);
    CHECK(nB.size() == 1);
    CHECK(nA[0] == b->getId());
    CHECK(nB[0] == a->getId());
}

TEST_CASE("removeEdge and removeVertex") {
    AdjacencyMatrix<City, Edge> g(false);
    auto* a = makeCity("A");
    auto* b = makeCity("B");
    auto* c = makeCity("C");

    g.addVertex(a);
    g.addVertex(b);
    g.addVertex(c);

    auto* e1 = makeEdge(a, b);
    auto* e2 = makeEdge(b, c);
    g.addEdge(e1);
    g.addEdge(e2);

    g.removeEdge(e1);
    CHECK_FALSE(e1->isActive());
    CHECK(g.getEdge(a->getId(), b->getId()) == nullptr);

    g.removeVertex(b->getId());
    CHECK_FALSE(b->isActive());
    CHECK_FALSE(e2->isActive());
    CHECK(g.getEdge(a->getId(), b->getId()) == nullptr);
    CHECK(g.getEdge(b->getId(), c->getId()) == nullptr);
}

TEST_CASE("getNeighbors, edges sum and invalid ID") {
    AdjacencyMatrix<City, Edge> g(true);
    auto* a = makeCity("A");
    auto* b = makeCity("B");
    auto* c = makeCity("C");
    g.addVertex(a);
    g.addVertex(b);
    g.addVertex(c);

    auto* e1 = makeEdge(a, b, 10.0);
    auto* e2 = makeEdge(a, c, 20.0);
    g.addEdge(e1);
    g.addEdge(e2);

    auto nA = g.getNeighbors(a->getId());
    CHECK(nA.size() == 2);
    CHECK(std::find(nA.begin(), nA.end(), b->getId()) != nA.end());
    CHECK(std::find(nA.begin(), nA.end(), c->getId()) != nA.end());

    double sum = 0;
    for (auto* edge : g.getEdges()) {
        if (edge->isActive()) sum += edge->getDistance();
    }
    CHECK(sum == 30.0);

    CHECK(g.getNeighbors(-1).empty());
    CHECK(g.getNeighbors(100).empty());
}