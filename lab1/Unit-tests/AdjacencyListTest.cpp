#include "doctest.h"

#include "AdjacencyList.h"
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

TEST_CASE("addEdge, getNeighbors, getEdge and getEdgesFrom undirected") {
    AdjacencyList<City, Edge> g(false);

    auto* c1 = makeCity("A", 1000);
    auto* c2 = makeCity("B", 2000);
    auto* c3 = makeCity("C", 1500);

    g.addVertex(c1);
    g.addVertex(c2);
    g.addVertex(c3);

    auto* e1 = makeEdge(c1, c2, 5.0, RoadType::ROAD);
    auto* e2 = makeEdge(c2, c3, 7.0, RoadType::RAIL);

    g.addEdge(e1);
    g.addEdge(e2);

    SUBCASE("getNeighbors") {
        auto n1 = g.getNeighbors(c1->getId());
        auto n2 = g.getNeighbors(c2->getId());
        auto n3 = g.getNeighbors(c3->getId());

        CHECK(n1.size() == 1);
        CHECK(n1[0] == c2->getId());

        CHECK(n2.size() == 2);
        CHECK(std::find(n2.begin(), n2.end(), c1->getId()) != n2.end());
        CHECK(std::find(n2.begin(), n2.end(), c3->getId()) != n2.end());

        CHECK(n3.size() == 1);
        CHECK(n3[0] == c2->getId());
    }

    SUBCASE("getEdge") {
        CHECK(g.getEdge(c1->getId(), c2->getId()) == e1);
        CHECK(g.getEdge(c2->getId(), c1->getId()) == e1);
        CHECK(g.getEdge(c1->getId(), c3->getId()) == nullptr);
    }

    SUBCASE("getEdgesFrom") {
        auto edgesFrom2 = g.getEdgesFrom(c2->getId());
        CHECK(edgesFrom2.size() == 2);
        CHECK((edgesFrom2[0] == e1 || edgesFrom2[1] == e1));
        CHECK((edgesFrom2[0] == e2 || edgesFrom2[1] == e2));
    }
}

TEST_CASE("directed edge") {
    AdjacencyList<City, Edge> g(true);

    auto* a = makeCity("A", 1000);
    auto* b = makeCity("B", 2000);
    g.addVertex(a);
    g.addVertex(b);

    auto* e = makeEdge(a, b, 12.0, RoadType::ROAD);
    g.addEdge(e);

    auto nA = g.getNeighbors(a->getId());
    auto nB = g.getNeighbors(b->getId());

    CHECK(nA.size() == 1);
    CHECK(nA[0] == b->getId());
    CHECK(nB.empty());

    CHECK(g.getEdge(a->getId(), b->getId())->getDistance() == 12.0);
    CHECK(g.getEdge(b->getId(), a->getId()) == nullptr);
}

TEST_CASE("removeEdge and removeVertex") {
    AdjacencyList<City, Edge> g(false);
    auto* a = makeCity("A", 1000);
    auto* b = makeCity("B", 2000);
    auto* c = makeCity("C", 1500);

    g.addVertex(a);
    g.addVertex(b);
    g.addVertex(c);

    auto* e1 = makeEdge(a, b, 5.0);
    auto* e2 = makeEdge(b, c, 7.0);
    auto* e3 = makeEdge(a, c, 15.0);

    g.addEdge(e1);
    g.addEdge(e2);
    g.addEdge(e3);

    g.removeEdge(e1);
    CHECK_FALSE(e1->isActive());
    CHECK(g.getEdge(a->getId(), b->getId()) == nullptr);

    g.removeVertex(b->getId());
    CHECK_FALSE(b->isActive());
    CHECK_FALSE(e2->isActive());
    CHECK(g.getEdge(a->getId(), c->getId()) == e3);
}

TEST_CASE("getEdge correctness and invalid IDs") {
    AdjacencyList<City, Edge> g(true);
    auto* a = makeCity("A", 1000);
    auto* b = makeCity("B", 2000);
    g.addVertex(a);
    g.addVertex(b);

    auto* e = makeEdge(a, b);
    g.addEdge(e);

    CHECK(g.getEdge(a->getId(), b->getId()) == e);
    CHECK(g.getEdge(b->getId(), a->getId()) == nullptr);
    CHECK(g.getEdge(5, 1) == nullptr);
    CHECK(g.getEdge(-1, 0) == nullptr);
}

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
    CHECK(total == 55.0);
}