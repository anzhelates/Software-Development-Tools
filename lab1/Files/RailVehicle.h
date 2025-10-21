#pragma once
#include "Vehicle.h"
#include <string>

class RailVehicle : public Vehicle {
public:
    RailVehicle(const std::string& name, double speed, double fuel)
        : Vehicle(name, speed, fuel) {}

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

    bool canUse(RoadType type) const override {
        return type == RoadType::RAIL;
    }
};