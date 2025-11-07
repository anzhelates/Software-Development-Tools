#pragma once
#include "Vehicle.h"
#include <string>

/**
 * @file RailVehicle.h
 * @class RailVehicle
 * @brief Represents a vehicle that travels on rails
 */
class RailVehicle : public Vehicle {
public:
    /**
     * @brief Constructs a RailVehicle
     * @param name The name of the vehicle
     * @param speed The base speed in km/h
     * @param fuel The fuel efficiency
     */
    RailVehicle(const std::string& name, double speed, double fuel)
        : Vehicle(name, speed, fuel) {}

    /**
     * @brief Calculates the effective speed considering obstacles
     * @param cause The cause of the obstacle
     * @param characteristic The characteristic of the path (ignored here)
     * @return The speed in km/h
     */
    double getSpeed(ObstacleCause cause, RoadCharacteristic characteristic) const override {
        double factor = 1.0;
        switch (cause) {
            case ObstacleCause::CONSTRUCTION: factor *= 0.8; break;
            case ObstacleCause::ACCIDENT: factor *= 0.5; break;
            case ObstacleCause::CUSTOM_DELAY: factor *= 0.9; break;
            default: break;
        }
        return m_speed * factor;
    }

    /**
     * @brief Checks whether the vehicle can use a specific type of path
     * @param type The type of the path
     * @return True if the path type is RAIL, false otherwise
     */
    bool canUse(RoadType type) const override {
        return type == RoadType::RAIL;
    }
};