#pragma once
#include "RailVehicle.h"

class Underground : public RailVehicle {
public:
    Underground(const std::string& name, double speed = 80.0, double fuel = 100.0)
        : RailVehicle(name, speed, fuel) {}

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