#include "doctest.h"
#include "Edge.h"
#include "LandVehicle.h"
#include "RailVehicle.h"
#include "AirVehicle.h"
#include "WaterVehicle.h"
#include "Car.h"
#include "Underground.h"
#include "Route.h"
#include "City.h"
#include <limits>

/**
 * @file Short_tests_Vehicle_Edge_Route.cpp
 * @brief Unit tests for Vehicle, Edge, and Route classes
 */

/**
 * @brief Helper function to create a new City vertex
 * @param name The name of the city
 * @param population The population of the city (default 1000)
 * @return A pointer to the newly created City object
 */
static City* makeCity(const std::string& name, long population = 1000) {
    return new City(name, population);
}

TEST_SUITE("VehicleTestSuite") {
    /**
     * @brief Tests the Vehicle's getSpeed method with various road characteristics
     */
    TEST_CASE("speed with different road characteristics") {
        Car car("Car", 100.0, 8.0);

        CHECK(car.getSpeed(ObstacleCause::CUSTOM_DELAY, RoadCharacteristic::STANDARD) == doctest::Approx(100.0));
        CHECK(car.getSpeed(ObstacleCause::CUSTOM_DELAY, RoadCharacteristic::HIGHWAY) == doctest::Approx(120.0));
        CHECK(car.getSpeed(ObstacleCause::CUSTOM_DELAY, RoadCharacteristic::CITY_STREET) == doctest::Approx(80.0));
        CHECK(car.getSpeed(ObstacleCause::CUSTOM_DELAY, RoadCharacteristic::DIRT_ROAD) == doctest::Approx(60.0));
        CHECK(car.getSpeed(ObstacleCause::CUSTOM_DELAY, RoadCharacteristic::DENSELY_POPULATED_CITY) == doctest::Approx(70.0));
    }

    /**
     * @brief Tests the Vehicle's 'canUse' method for different road and vehicle types
     */
    TEST_CASE("canUse for different types") {
        Car car("Car", 100, 8);
        Underground metro("Metro", 80, 5);
        AirVehicle plane("Plane", 900, 3000);
        WaterVehicle ship("Ship", 40, 1500);

        CHECK(car.canUse(RoadType::ROAD) == true);
        CHECK(car.canUse(RoadType::RAIL) == false);
        CHECK(metro.canUse(RoadType::RAIL) == true);
        CHECK(metro.canUse(RoadType::ROAD) == false);
        CHECK(plane.canUse(RoadType::AIR) == true);
        CHECK(ship.canUse(RoadType::WATER) == true);
    }
}

TEST_SUITE("EdgeTestSuite") {
    /**
     * @brief Tests the Edge's 'calculateTravelTime' method
     */
    TEST_CASE("calculateTravelTime") {
        City* c1 = new City("A", 500000);
        City* c2 = new City("B", 200000);
        Car* car = new Car("Car", 100.0, 8.0);

        Edge* edge = new Edge(c1, c2, 120.0, RoadType::ROAD, RoadCharacteristic::HIGHWAY);

        /**
         * @brief Checks travel time calculation without any obstacles
         */
        SUBCASE("without obstacles") {
            // speed on highway = 120.0
            // travelTime = 120.0 / 120.0 = 1.0
            CHECK(edge->calculateTravelTime(*car) == doctest::Approx(1.0));
        }

        /**
         * @brief Checks travel time calculation with an obstacle
         */
        SUBCASE("with obstacles") {
            auto* obs = new Obstacle{"Traffic jam", 0.5, ObstacleCause::TRAFFIC_JAM};
            edge->addObstacle(obs);

            // ((distance) 120.0 / (100.0 * (highway factor) 1.3 * (traffic jam factor) 0.4)) + (delay) 0.5 = 2.30769 + 0.5 = 2.80769
            CHECK(edge->calculateTravelTime(*car) == doctest::Approx(2.80769));
        }

        /**
         * @brief Checks that travel time is infinity if the vehicle cannot use the edge
         */
        SUBCASE("vehicle cannot use this road") {
            AirVehicle plane("Plane", 900, 3000);
            CHECK(edge->calculateTravelTime(plane) == std::numeric_limits<double>::infinity());
        }

        delete edge;
        delete car;
        delete c1;
        delete c2;
    }
}

TEST_SUITE("RouteTestSuite") {
    /**
     * @brief Tests that totalTime returns infinity if a route contains an unusable edge
     */
    TEST_CASE("edge not usable by vehicle") {
        City* A = makeCity("A");
        City* B = makeCity("B");
        Car car("Car",100,8);
        Edge* AB = new Edge(A,B,50.0,RoadType::RAIL);
        Route route(A,B,&car);
        route.addEdge(AB);
        CHECK(route.totalTime(car) == std::numeric_limits<double>::infinity());

        delete AB;
        delete A;
        delete B;
    }

    /**
     * @brief Tests the calculation of total distance, time, and fuel for a multi-edge route
     */
    TEST_CASE("total distance, time, fuel") {
        City* A = makeCity("A", 1000000);
        City* B = makeCity("B", 500000);
        City* C = makeCity("C", 200000);

        Car car("Car", 100.0, 8.0);

        Edge* AB = new Edge(A, B, 100.0, RoadType::ROAD, RoadCharacteristic::STANDARD);
        Edge* BC = new Edge(B, C, 50.0, RoadType::ROAD, RoadCharacteristic::STANDARD);

        Route route(A, C, &car);
        route.addEdge(AB);
        route.addEdge(BC);

        CHECK(route.totalDistance() == doctest::Approx(150.0));
        CHECK(route.totalTime(car) == doctest::Approx(1.5));
        CHECK(route.totalFuel(car) == doctest::Approx(12.0));

        delete AB;
        delete BC;
        delete A;
        delete B;
        delete C;
    }

    /**
     * @brief Confirms that a route with even one unusable edge results in infinite total time
     */
    TEST_CASE("contains an unusable edge leads to infinity totalTime") {
        City* A = makeCity("A");
        City* B = makeCity("B");
        Car car("Car", 100.0, 8.0);

        Edge* AB = new Edge(A, B, 50.0, RoadType::RAIL);
        AB->markActive();

        Route route(A, B, &car);
        route.addEdge(AB);

        CHECK(route.totalTime(car) == std::numeric_limits<double>::infinity());

        delete AB;
        delete A;
        delete B;
    }
}