#pragma once

#include <string>
#include <stdexcept>
#include "EnumClasses.h"

class City;

/**
 * @file Vehicle.h
 * @brief Declares the Vehicle class for all vehicle types
 * @class Vehicle
 * @brief An abstract base class for all types of vehicles
 * @details Common interface for vehicles, including calculating speed under various conditions and fuel consumption
 */
class Vehicle {
    protected:
        std::string m_name;
        double m_speed;
        double m_fuelEfficiency;

    public:
        /**
         * @brief Constructs a Vehicle
         * @param name The name of the vehicle
         * @param speed The speed in km/h
         * @param fuelEfficiency The fuel consumption in liters per 100 km
         * @throws std::invalid_argument If speed or fuelEfficiency is negative
         */
        Vehicle(const std::string& name, double speed, double fuelEfficiency)
            : m_name(name), m_speed(speed), m_fuelEfficiency(fuelEfficiency) {
            if (speed < 0) {
                throw std::invalid_argument("Vehicle speed cannot be negative");
            }
            if (fuelEfficiency < 0) {
                throw std::invalid_argument("Vehicle fuel efficiency cannot be negative");
            }
        }

        virtual ~Vehicle() = default;

        /** @brief Get name of the vehicle */
        const std::string& getName() const { return m_name; }

        /**
         * @brief Pure virtual function to calculate the vehicle's effective speed
         * @param cause The cause of an obstacle affecting speed
         * @param characteristic The characteristic of the path
         * @return The calculated speed in km/h
         */
        virtual double getSpeed(ObstacleCause cause, RoadCharacteristic characteristic) const = 0;

        /**
         * @brief Checks whether the vehicle can use a certain RoadType
         * @param type The type of the path
         * @return True if the vehicle can use the path, false otherwise
         */
        virtual bool canUse(RoadType type) const = 0;

        /**
         * @brief Calculates the fuel needed to travel a given distance
         * @param distance The distance to travel in km
         * @return The fuel consumed in liters
         * @throws std::invalid_argument If distance is negative
         */
        virtual double calculateFuel(double distance) const {
            if (distance < 0) {
                throw std::invalid_argument("Distance cannot be negative");
            }
            return (distance / 100.0) * m_fuelEfficiency;
        }
};