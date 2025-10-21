#pragma once
#include "Vehicle.h"
#include <string>

class AirVehicle : public Vehicle {
public:
    AirVehicle(const std::string& name, double speed, double fuel)
        : Vehicle(name, speed, fuel) {}

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

    bool canUse(RoadType type) const override {
        return type == RoadType::AIR;
    }
};