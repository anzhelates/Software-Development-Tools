#pragma once
#include "Vehicle.h"
#include <string>

class LandVehicle : public Vehicle {
public:
    LandVehicle(const std::string& name, double speed, double fuel)
        : Vehicle(name, speed, fuel) {}

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

    bool canUse(RoadType type) const override {
        return type == RoadType::ROAD;
    }
};