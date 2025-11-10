#include <iostream>
#include <fstream>
#include <iomanip>
#include <cmath>
#include <algorithm>
#include <cstdlib>
#include <string>
#include "Traffic.h"
#include "Sensor.h"
#include "ACC.h"

int main(int argc, char** argv) {
    double dt = 0.05;
    double sim_time = 60.0;
    double v_set = 27.0;
    double headway = 1.2;
    double mu = 0.9;
    double init_gap = 50.0;
    double lead_accel = 0.0;
    double v_ego0 = 27.0;   // NEW: initial ego speed
    double v_lead0 = 27.0;  // NEW: initial lead speed
    double Kp_c = 0.4, Ki_c = 0.05, Kd_c = 0.01;
    double Kp_h = 0.6, Ki_h = 0.08, Kd_h = 0.02;

    // --- CLI parameters ---
    if (argc > 1) v_set = atof(argv[1]);
    if (argc > 2) headway = atof(argv[2]);
    if (argc > 3) mu = atof(argv[3]);
    if (argc > 4) sim_time = atof(argv[4]);
    if (argc > 5) init_gap = atof(argv[5]);
    if (argc > 6) lead_accel = atof(argv[6]);
    if (argc > 7) v_ego0 = atof(argv[7]);   // NEW
    if (argc > 8) v_lead0 = atof(argv[8]);  // NEW
    if (argc > 11) { Kp_c = atof(argv[9]); Ki_c = atof(argv[10]); Kd_c = atof(argv[11]); }
    if (argc > 14) { Kp_h = atof(argv[12]); Ki_h = atof(argv[13]); Kd_h = atof(argv[14]); }

    std::cout << std::fixed << std::setprecision(2);
    std::cout << "Running ACC simulation...\n"
              << "v_set=" << v_set << " m/s, headway=" << headway
              << " s, mu=" << mu
              << ", sim_time=" << sim_time
              << " s, init_gap=" << init_gap
              << " m, lead_accel=" << lead_accel
              << " m/s², ego0=" << v_ego0
              << " m/s, lead0=" << v_lead0 << " m/s\n";

    // --- Initialize system ---
    Traffic traffic;
    Sensor sensor;
    ACC acc(Kp_c, Ki_c, Kd_c, Kp_h, Ki_h, Kd_h);
    acc.mu = mu;
    acc.headway = headway;

    // --- Initial vehicle states ---
    traffic.lead.x = init_gap;
    traffic.lead.v = v_lead0;   // NEW
    traffic.ego.v = v_ego0;     // NEW

    std::ofstream log("acc_sim.csv");
    log << std::fixed << std::setprecision(3);
    log << "time,ego_x,ego_v,lead_x,lead_v,dist,rel_v,a_cmd,throttle,brake,acc_mode\n";

    for (double t = 0; t <= sim_time; t += dt) {
        // Lead dynamics
        traffic.lead.v += lead_accel * dt;
        traffic.lead.v = std::max(0.0, traffic.lead.v);
        traffic.lead.x += traffic.lead.v * dt;

        // Sensor readings
        double dist = sensor.MeasureDistance(traffic);
        double rel_v = traffic.lead.v - traffic.ego.v;

        // ACC control
        double a_cmd = acc.Step(dist, traffic.ego.v, traffic.lead.v, v_set, dt);

        // Throttle/Brake mapping
        double throttle = 0.0, brake = 0.0;
        if (acc.mode == ACCMode::Override) {
            throttle = 0.0; brake = 1.0;
        } else if (a_cmd >= 0.0) {
            throttle = std::clamp(a_cmd / acc.a_max, 0.0, 1.0);
        } else {
            brake = std::clamp(-a_cmd / (-acc.a_min), 0.0, 1.0);
        }

        traffic.ego.throttle = throttle;
        traffic.ego.brake = brake;
        traffic.Update(dt);

        // Log data
        log << t << "," << traffic.ego.x << "," << traffic.ego.v << ","
            << traffic.lead.x << "," << traffic.lead.v << ","
            << dist << "," << rel_v << "," << a_cmd << ","
            << throttle << "," << brake << ","
            << acc.ModeString() << "\n";
    }

    log.close();
    std::cout << "\n✅ Simulation complete. Results saved to acc_sim.csv\n";
    return 0;
}
