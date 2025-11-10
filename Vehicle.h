// Vehicle.h
#pragma once
#include <cmath>

class Vehicle {
public:
    double x, y, yaw, v;      // state
    double L;                 // wheelbase
    double throttle, brake, steer;

    Vehicle(double x0=0, double y0=0, double yaw0=0, double v0=0, double L_=2.5)
        : x(x0), y(y0), yaw(yaw0), v(v0), L(L_), throttle(0), brake(0), steer(0) {}

    void Update(double dt) {
        double a = throttle * 2.0 - brake * 5.0;  // simple accel/brake map
        v += a * dt;
        v = std::max(0.0, v);
        x += v * std::cos(yaw) * dt;
        y += v * std::sin(yaw) * dt;
        yaw += (v / L) * std::tan(steer) * dt;
    }
};
