#pragma once
#include "Vehicle.h"
#include <string>

/**
 * @file AirVehicle.h
 * @brief Declares the AirVehicle class representing an air-based transport
 * @class AirVehicle
 * @brief Represents a vehicle that can fly
 */
class AirVehicle : public Vehicle {
public:
    /**
     * @brief Constructs an AirVehicle
     * @param name The name of the air vehicle
     * @param speed The speed in km/h
     * @param fuel The fuel efficiency in liters per 100 km
     */
    AirVehicle(const std::string& name, double speed, double fuel)
        : Vehicle(name, speed, fuel) {}

    /**
     * @brief Calculates the speed considering obstacles
     * @param cause Specifies the obstacle
     * @param characteristic Road characteristic (ignored here)
     * @return The speed in km/h
     */
    double getSpeed(ObstacleCause cause, RoadCharacteristic characteristic) const override {
        double factor = 1.0;
        switch (cause) {
            case ObstacleCause::WEATHER_WIND: factor *= 0.8; break;
            case ObstacleCause::WEATHER_STORM: factor *= 0.5; break;
            case ObstacleCause::WEATHER_SNOW: factor *= 0.6; break;
            case ObstacleCause::CUSTOM_DELAY: factor *= 0.9; break;
            default: break;
        }
        return m_speed * factor;
    }

    /**
     * @brief Checks whether the vehicle can use a specific type of path
     * @param type The type of the path
     * @return True if the path type is AIR, false otherwise
     */
    bool canUse(RoadType type) const override {
        return type == RoadType::AIR;
    }
};