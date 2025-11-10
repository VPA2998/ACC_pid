// Traffic.h
#pragma once
#include "Vehicle.h"
#include <vector>

class Traffic {
public:
    Vehicle ego;
    Vehicle lead;
    double roadLength;
    double t;

    Traffic(double lead_init_gap=40.0) {
        ego = Vehicle(0, 0, 0, 20);
        lead = Vehicle(lead_init_gap, 0, 0, 22);
        roadLength = 1000.0;
        t = 0.0;
    }

    void Update(double dt) {
        // Simple lead vehicle behavior: slow down between 5–10s
        if (t > 5.0 && t < 10.0) lead.throttle = 0.0, lead.brake = 0.2;
        else if (t >= 10.0) lead.brake = 0.0, lead.throttle = 0.2;

        lead.Update(dt);
        ego.Update(dt);
        t += dt;
    }

    double Distance() const { return lead.x - ego.x; }
    double RelSpeed() const { return lead.v - ego.v; }
};
