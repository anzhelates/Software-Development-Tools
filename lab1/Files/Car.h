#pragma once
#include "LandVehicle.h"

/**
 * @file Car.h
 * @brief Declares the Car class representing a land vehicle
 * @class Car
 * @brief Represents a car, inherits from LandVehicle
 */
class Car : public LandVehicle {
public:
    /**
     * @brief Constructs a Car
     * @param name The name of the car
     * @param speed The speed in km/h
     * @param fuel The fuel efficiency in liters per 100 km
     */
    Car(const std::string& name, double speed, double fuel)
        : LandVehicle(name, speed, fuel) {}

    /**
     * @brief Calculates the speed considering obstacles and road type
     * @param cause The cause of the obstacle
     * @param road The characteristic of the road
     * @return The speed in km/h
     */
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