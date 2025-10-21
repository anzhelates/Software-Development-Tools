#pragma once

#include <string>
#include "EnumClasses.h"

class City;

class Vehicle {
protected:
    std::string m_name;
    double m_speed;
    double m_fuelEfficiency;

public:
    Vehicle(const std::string& name, double speed, double fuelEfficiency)
        : m_name(name), m_speed(speed), m_fuelEfficiency(fuelEfficiency) {}

    virtual ~Vehicle() = default;
    const std::string& getName() const { return m_name; }
    virtual double getSpeed(ObstacleCause cause, RoadCharacteristic characteristic) const = 0;
    virtual bool canUse(RoadType type) const = 0;
    virtual double calculateFuel(double distance) const {
        return (distance / 100.0) * m_fuelEfficiency;
    }
};