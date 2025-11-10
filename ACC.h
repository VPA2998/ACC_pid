#pragma once
#include <algorithm>
#include <cmath>
#include <string>

enum class ACCMode { Cruise, Headway, Override };

class ACC {
public:
    // PID gains
    double Kp_c, Ki_c, Kd_c;  // Cruise (speed control)
    double Kp_h, Ki_h, Kd_h;  // Headway (distance control)
    double Kp_v, Ki_v;        // Inner speed loop (shared)

    // Parameters
    double headway;        // time gap [s]
    double d0;             // standstill distance [m]
    double mu;             // road friction
    double safety_factor;  // safety coefficient
    double d_safety;       // minimal buffer distance [m]
    double a_max, a_min;   // acceleration limits
    double margin;         // transition margin

    // State variables
    ACCMode mode;
    double integ_c, integ_h, integ_v;
    double prev_err_c, prev_err_h, prev_err_v;

    // --- Constructor with tunable gains ---
    ACC(double Kp_c_=0.4, double Ki_c_=0.05, double Kd_c_=0.01,
        double Kp_h_=0.6, double Ki_h_=0.08, double Kd_h_=0.02,
        double Kp_v_=0.8, double Ki_v_=0.1,
        double headway_=1.2, double d0_=5.0,
        double mu_=0.9, double safety_factor_=0.9,
        double d_safety_=5.0, double a_max_=2.0,
        double a_min_=-8.0, double margin_=10.0)
        : Kp_c(Kp_c_), Ki_c(Ki_c_), Kd_c(Kd_c_),
          Kp_h(Kp_h_), Ki_h(Ki_h_), Kd_h(Kd_h_),
          Kp_v(Kp_v_), Ki_v(Ki_v_),
          headway(headway_), d0(d0_), mu(mu_),
          safety_factor(safety_factor_), d_safety(d_safety_),
          a_max(a_max_), a_min(a_min_), margin(margin_),
          mode(ACCMode::Cruise),
          integ_c(0.0), integ_h(0.0), integ_v(0.0),
          prev_err_c(0.0), prev_err_h(0.0), prev_err_v(0.0) {}

    // --- Helper functions ---
    double AvailableDecel() const {
        const double g = 9.81;
        return mu * g * safety_factor;
    }

    double BrakingDistance(double v_ego, double v_lead) const {
        double dv = v_ego - v_lead;
        if (dv <= 0) return 0.0;
        return (dv * dv) / (2 * AvailableDecel()) + d_safety;
    }

    double DesiredSpacing(double v_ego) const {
        return d0 + headway * v_ego;
    }

    // --- Cruise PID ---
    double CruisePID(double v_set, double v_ego, double dt) {
        double err = v_set - v_ego;
        integ_c += err * dt;
        double deriv = (err - prev_err_c) / dt;
        prev_err_c = err;
        double a = Kp_c * err + Ki_c * integ_c + Kd_c * deriv;
        return std::clamp(a, a_min, a_max);
    }

    // --- Headway outer PID ---
    double HeadwayOuter(double dist, double v_ego, double dt) {
        double d_des = DesiredSpacing(v_ego);
        double err = dist - d_des;
        integ_h += err * dt;
        double deriv = (err - prev_err_h) / dt;
        prev_err_h = err;
        return Kp_h * err + Ki_h * integ_h + Kd_h * deriv;
    }

    // --- Inner PI for speed ---
    double SpeedInner(double v_sp, double v_ego, double dt) {
        double err = v_sp - v_ego;
        integ_v += err * dt;
        double a = Kp_v * err + Ki_v * integ_v;
        return std::clamp(a, a_min, a_max);
    }

    // --- Mode switching logic ---
    void UpdateMode(double dist, double v_ego, double v_lead) {
        double d_brake = BrakingDistance(v_ego, v_lead);
        double d_des   = DesiredSpacing(v_ego);

        switch (mode) {
            case ACCMode::Cruise:
                if (dist < d_des + 5.0) mode = ACCMode::Headway;
                break;
            case ACCMode::Headway:
                if (dist < d_brake) mode = ACCMode::Override;
                else if (dist > d_des + 15.0) mode = ACCMode::Cruise;
                break;
            case ACCMode::Override:
                if (dist > d_brake + 5.0) mode = ACCMode::Headway;
                break;
        }
    }

    // --- Main control step ---
    double Step(double dist, double v_ego, double v_lead, double v_set, double dt) {
        UpdateMode(dist, v_ego, v_lead);

        if (mode == ACCMode::Override)
            return a_min;  // Full brake

        if (mode == ACCMode::Cruise)
            return CruisePID(v_set, v_ego, dt);

        double dv = HeadwayOuter(dist, v_ego, dt);
        double v_sp = v_ego + dv;
        return SpeedInner(v_sp, v_ego, dt);
    }

    std::string ModeString() const {
        switch (mode) {
            case ACCMode::Cruise: return "Cruise";
            case ACCMode::Headway: return "Headway";
            case ACCMode::Override: return "Override";
            default: return "Unknown";
        }
    }
};
