#pragma once
#include "Vehicle.h"
#include <string>

/**
 * @file LandVehicle.h
 * @class LandVehicle
 * @brief Represents a vehicle that travels on land
 */
class LandVehicle : public Vehicle {
public:
    /**
     * @brief Constructs a LandVehicle
     * @param name The name of the vehicle
     * @param speed The base speed in km/h
     * @param fuel The fuel efficiency in liters per 100 km
     */
    LandVehicle(const std::string& name, double speed, double fuel)
        : Vehicle(name, speed, fuel) {}

    /**
     * @brief Calculates the effective speed considering obstacles and road types
     * @param cause The cause of the obstacle (TRAFFIC_JAM,...)
     * @param road The characteristic of the road (HIGHWAY,...)
     * @return The speed in km/h
     */
    double getSpeed(ObstacleCause cause, RoadCharacteristic road) const override {
        double factor = 1.0;
        switch (cause) {
            case ObstacleCause::TRAFFIC_JAM: factor *= 0.6; break;
            case ObstacleCause::ACCIDENT: factor *= 0.5; break;
            case ObstacleCause::CONSTRUCTION: factor *= 0.8; break;
            case ObstacleCause::WEATHER_STORM: factor *= 0.7; break;
            case ObstacleCause::WEATHER_SNOW: factor *= 0.6; break;
            case ObstacleCause::WEATHER_WIND: factor *= 0.9; break;
            default: break;
        }

        switch (road) {
            case RoadCharacteristic::STANDARD: factor *= 1.0; break;
            case RoadCharacteristic::HIGHWAY: factor *= 1.2; break;
            case RoadCharacteristic::CITY_STREET: factor *= 0.8; break;
            case RoadCharacteristic::DIRT_ROAD: factor *= 0.6; break;
            case RoadCharacteristic::PARK_ROAD: factor *= 0.5; break;
            case RoadCharacteristic::DENSELY_POPULATED_CITY: factor *= 0.7; break;
            default: break;
        }
        return m_speed * factor;
    }

    /**
     * @brief Checks whether the vehicle can use a specific type of path
     * @param type The type of the path
     * @return True if the path type is ROAD, false otherwise
     */
    bool canUse(RoadType type) const override {
        return type == RoadType::ROAD;
    }
};