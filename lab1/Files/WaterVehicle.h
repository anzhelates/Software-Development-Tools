#pragma once
#include "Vehicle.h"
#include <string>

/**
 * @file WaterVehicle.h
 * @class WaterVehicle
 * @brief Represents a vehicle that travels on water
 */
class WaterVehicle : public Vehicle {
public:
    /**
     * @brief Constructs a WaterVehicle
     * @param name The name of the vehicle
     * @param speed The base speed in km/h
     * @param fuel The fuel efficiency
     */
    WaterVehicle(const std::string& name, double speed, int fuel)
        : Vehicle(name, speed, fuel) {}

    /**
     * @brief Calculates the effective speed considering weather obstacles
     * @param cause The cause of the obstacle
     * @param characteristic The characteristic of the path (ignored here)
     * @return The speed
     */
    double getSpeed(ObstacleCause cause, RoadCharacteristic characteristic) const override {
        double factor = 1.0;

        switch (cause) {
            case ObstacleCause::WEATHER_STORM: factor *= 0.5; break;
            case ObstacleCause::WEATHER_WIND: factor *= 0.7; break;
            case ObstacleCause::WEATHER_ICE: factor *= 0.6; break;
            case ObstacleCause::CUSTOM_DELAY: factor *= 0.9; break;
            default: break;
        }
        return m_speed * factor;
    }

    /**
     * @brief Checks whether the vehicle can use a specific type of path
     * @param type The type of the path
     * @return True if the path type is WATER, false otherwise
     */
    bool canUse(RoadType type) const override {
        return type == RoadType::WATER;
    }
};