#pragma once
#include "RailVehicle.h"

/**
 * @file Underground.h
 * @class Underground
 * @brief Represents an underground train, a specific type of rail vehicle
 */
class Underground : public RailVehicle {
public:
    /**
     * @brief Constructs an Underground vehicle
     * @param name The name or line of the underground
     * @param speed The base speed in km/h
     * @param fuel The fuel consumption
     */
    Underground(const std::string& name, double speed = 80.0, double fuel = 100.0)
        : RailVehicle(name, speed, fuel) {}

    /**
     * @brief Calculates the effective speed considering obstacles
     * @param cause The cause of the obstacle
     * @param characteristic The characteristic of the path (ignored here)
     * @return The adjusted speed in km/h
     */
    double getSpeed(ObstacleCause cause, RoadCharacteristic characteristic) const override {
        double factor = 1.0;
        switch (cause) {
            case ObstacleCause::ACCIDENT:
                factor *= 0.5;
            break;
            case ObstacleCause::CONSTRUCTION:
                factor *= 0.8;
            break;
            case ObstacleCause::CUSTOM_DELAY:
                factor *= 0.9;
            break;
            default: break;
        }
        return m_speed * factor;
    }
};