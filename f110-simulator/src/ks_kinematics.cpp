#include <cmath>

#include "f1tenth_simulator/car_state.hpp"
#include "f1tenth_simulator/ks_kinematics.hpp"

using namespace racecar_simulator;

// Implementation based off of Kinematic Single Track Dynamics defined in CommonRoad: Vehicle Models
// https://gitlab.lrz.de/tum-cps/commonroad-vehicle-models/blob/master/vehicleModels_commonRoad.pdf

CarState KSKinematics::update(
        const CarState start,
        double accel,
        double steer_angle_vel,
        CarParams p,
        double dt) {

    CarState end;

    // compute first derivatives of state
    double x_dot = start.v_x * std::cos(start.theta) - start.v_y * std::sin(start.theta);
    double y_dot = start.v_x * std::sin(start.theta) + start.v_y * std::cos(start.theta);
    double vx_dot = accel;
    double vy_dot = 0; // Change if lateral velocity is considered
    double steer_ang_dot = steer_angle_vel;
    double theta_dot = start.v_x / p.wheelbase * std::tan(start.steer_angle);

    // update state
    end.x = start.x + x_dot * dt;
    end.y = start.y + y_dot * dt;
    end.theta = start.theta + theta_dot * dt;
    end.v_x = start.v_x + vx_dot * dt;
    end.v_y = start.v_y + vy_dot * dt;
    end.steer_angle = start.steer_angle + steer_ang_dot * dt;
    end.angular_velocity = start.angular_velocity;
    end.slip_angle = std::atan2(end.v_y, end.v_x);

    return end;
}
