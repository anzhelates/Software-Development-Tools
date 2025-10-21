#define DOCTEST_CONFIG_IMPLEMENT
#include "doctest.h"

#include <iostream>
#include <vector>
#include <string>
#include "AdjacencyList.h"
#include "AdjacencyMatrix.h"
#include "City.h"
#include "Edge.h"
#include "GraphAlgorithm.h"
#include "Route.h"
#include "Car.h"
#include "AirVehicle.h"
#include "Underground.h"
#include "WaterVehiсle.h"

int main(int argc, char** argv) {
    using GraphType = AdjacencyList<City, Edge>;

    doctest::Context context;
    context.applyCommandLine(argc, argv);
    int res = context.run();
    if(context.shouldExit()) {
        return res;
    }

    GraphType graph(false);

    City* kyiv = new City("Kyiv", 3000000);
    City* lviv = new City("Lviv", 720000);
    City* odessa = new City("Odessa", 1000000);

    graph.addVertex(kyiv);
    graph.addVertex(lviv);
    graph.addVertex(odessa);

    Car* car = new Car("Toyota", 80.0, 7.5);
    AirVehicle* plane = new AirVehicle("Boeing 737", 850.0, 2600.0);
    Underground* metro = new Underground("Kyiv Metro", 40.0, 50.0);
    WaterVehicle* boat = new WaterVehicle("Titanic", 40.0, 30);

    kyiv->addVehicle(car);
    lviv->addVehicle(plane);
    odessa->addVehicle(boat);
    kyiv->addVehicle(metro);

    Edge* roadEdge = new Edge(kyiv, lviv, 540.0, RoadType::ROAD, RoadCharacteristic::HIGHWAY);
    Edge* airEdge = new Edge(kyiv, odessa, 475.0, RoadType::AIR, RoadCharacteristic::OTHER);
    Edge* waterEdge = new Edge(lviv, odessa, 700.0, RoadType::WATER, RoadCharacteristic::OTHER);
    roadEdge->markActive();
    airEdge->markActive();
    waterEdge->markActive();

    graph.addEdge(roadEdge);
    graph.addEdge(airEdge);
    graph.addEdge(waterEdge);

    std::cout << "BFS from Kyiv:\n";
    std::vector<int> bfsOrder = GraphAlgorithms::BFS(graph, kyiv->getId());
    for (int id : bfsOrder) {
        std::cout << "- " << graph.getVertexById(id)->getName() << "\n";
    }

    std::cout << "DFS from Kyiv:\n";
    std::vector<int> dfsOrder = GraphAlgorithms::DFS(graph, kyiv->getId());
    for (int id : dfsOrder) {
        std::cout << "- " << graph.getVertexById(id)->getName() << "\n";
    }

    std::cout << "The shortest time with Car (Toyota) from Kyiv:\n";
    std::vector<double> dist = GraphAlgorithms::Dijkstra(graph, kyiv->getId(), *car);
    for (int i = 0; i < static_cast<int>(dist.size()); ++i) {
        std::cout << graph.getVertexById(i)->getName() << ": " << dist[i] << " h\n";
    }

    Route route(kyiv, lviv, car);
    route.addEdge(roadEdge);

    std::cout << "Total distance: " << route.totalDistance() << " km\n";
    std::cout << "Total time taken: " << route.totalTime(*car) << " h\n";
    std::cout << "Total fuel consumption: " << route.totalFuel(*car) << " l\n";

    return 0;
}