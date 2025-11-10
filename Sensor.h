// Sensor.h
#pragma once
#include "Traffic.h"
#include <random>

class Sensor {
public:
    std::default_random_engine gen;
    std::normal_distribution<double> noise{0.0, 0.5}; // 0.5 m or m/s noise

    double MeasureDistance(const Traffic& traf) {
        return traf.Distance() + noise(gen);
    }

    double MeasureRelSpeed(const Traffic& traf) {
        return traf.RelSpeed() + noise(gen) * 0.1;
    }
};
