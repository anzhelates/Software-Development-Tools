#pragma once

/**
 * @file EnumClasses.h
 * @brief Contains all enum classes RoadType, RoadCharacteristic and ObstacleCause
 */

/**
 * @enum RoadType
 * @brief Defines the physical type of path
 */
enum class RoadType { ROAD, RAIL, AIR, WATER };

/**
 * @enum RoadCharacteristic
 * @brief Defines specific characteristics of a path that can affect travel speed
 */
enum class RoadCharacteristic { HIGHWAY, CITY_STREET, DIRT_ROAD, PARK_ROAD, STANDARD, DENSELY_POPULATED_CITY, OTHER };

/**
 * @enum ObstacleCause
 * @brief Defines the cause of an obstacle
 */
enum class ObstacleCause { TRAFFIC_JAM, ACCIDENT, CONSTRUCTION, WEATHER_STORM, WEATHER_SNOW, WEATHER_WIND, WEATHER_ICE, CUSTOM_DELAY };