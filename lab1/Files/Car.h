#pragma once
#include "LandVehicle.h"

class Car : public LandVehicle {
public:
    Car(const std::string& name, double speed, double fuel)
        : LandVehicle(name, speed, fuel) {}

    double getSpeed(ObstacleCause cause, RoadCharacteristic road) const override {
        double factor = 1.0;

        if (cause == ObstacleCause::TRAFFIC_JAM) {
            factor *= 0.4;
        } else {
            return LandVehicle::getSpeed(cause, road);
        }

        switch (road) {
            case RoadCharacteristic::HIGHWAY: factor *= 1.3; break;
            case RoadCharacteristic::CITY_STREET: factor *= 0.7; break;
            case RoadCharacteristic::DIRT_ROAD: factor *= 0.8; break;
            default: break;
        }
        return m_speed * factor;
    }
};