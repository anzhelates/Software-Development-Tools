#pragma once
#include "Vehicle.h"
#include <string>

class WaterVehicle : public Vehicle {
public:
    WaterVehicle(const std::string& name, double speed, int fuel)
        : Vehicle(name, speed, fuel) {}

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

    bool canUse(RoadType type) const override {
        return type == RoadType::WATER;
    }
};