#include <cmath>

#include "f1tenth_simulator/car_state.hpp"
#include "f1tenth_simulator/std_kinematics.hpp"
#include <iostream>

using namespace racecar_simulator;

// Implementation based off of Single Track Drift Dynamics defined in CommonRoad: Vehicle Models
// https://gitlab.lrz.de/tum-cps/commonroad-vehicle-models/-/blob/master/PYTHON/vehiclemodels/vehicle_dynamics_std.py
// and
// https://gitlab.ethz.ch/ics/crs/-/blob/main/software/src/crs/dynamic_models/pacejka_model/src/pacejka_continuous.cpp?ref_type=heads

CarState STDKinematics::update_pacejka(
        const CarState start,
        double accel,
        double steer_angle_vel,
        CarParams p,
        double dt) {
    /* Model for the single track dynamics of the vehicle
    It uses the Pacejka tire model for the lateral tire forces
    
    state
    start.x = x-position in global frame [m]
    start.y = y-position in global frame [m]
    start.theta = orientation in global frame [rad]
    start.v_x = longitudinal velocity [m/s]
    start.v_y = lateral velocity [m/s]
    start.steer_angle = steering angle [rad]
    start.angular_velocity = angular velocity [rad/s]
    start.slip_angle = slip angle [rad]
    start.omega_fw = front wheel angular velocity [rad/s]
    start.omega_rw = rear wheel angular velocity [rad/s]
    start.st_dyn = true if using ST dynamics, false if using kinematic model
    
    input 
    accel = acceleration [m/s^2]
    steer_angle_vel = steering angle velocity [rad/s] */

    CarState end;

    // local params
    double v_b = 3; // m/s
    double v_s = 1; // m/s
    double v_min = v_b-2*v_s; // m/s
    double g = 9.81; // m/s^2

    // in the original code constraints were applied here, but I think they are already applied outside
    // therefore we skip to the DYNAMICS computation directly
    
    // lateral tire slip angles
    double alpha_f;
    double alpha_r;
    double start_vx = start.v_x;
    double start_vy = start.v_y;
    if (start.v_x >= v_min){
        alpha_f = std::atan2((-start_vy - p.l_f * start.angular_velocity), start_vx) + start.steer_angle;
        alpha_r = std::atan2((-start_vy + p.l_r * start.angular_velocity), start_vx);
    } else {
        alpha_f = 0;
        alpha_r = 0;
    }

    // compute vertical tire forces (load transfer due to acceleration)
    double F_zf = p.mass * (-accel * p.h_cg + g * p.l_r) / (p.l_f + p.l_r);
    double F_zr = p.mass * (accel * p.h_cg + g * p.l_f) / (p.l_f + p.l_r);

    // combined lateral slip forces according to Pacejka
    double F_yf = p.friction_coeff * p.D_f * F_zf * std::sin(
        p.C_f * std::atan(
            p.B_f * alpha_f - p.E_f*(p.B_f * alpha_f - std::atan(
                p.B_f * alpha_f))));
    double F_yr = p.friction_coeff * p.D_r * F_zr * std::sin(
        p.C_r * std::atan(
            p.B_r * alpha_r - p.E_r*(p.B_r * alpha_r - std::atan(
                p.B_r * alpha_r))));
    // Linear tire model
    // F_yf = p.friction_coeff * F_zf * p.cs_f * alpha_f;
    // F_yr = p.friction_coeff * F_zr * p.cs_r * alpha_r;

    // compute first derivatives of state
    double x_dot = start_vx * std::cos(start.theta) - start_vy * std::sin(start.theta);
    double y_dot = start_vx * std::sin(start.theta) + start_vy * std::cos(start.theta);
    double vx_dot = accel + (1/p.mass) * (- F_yf * std::sin(start.steer_angle)) + start_vy * start.angular_velocity;
    double steer_angle_dot = steer_angle_vel;
    double theta_dot = start.angular_velocity;
    double vy_dot = (1/p.mass) * (F_yr + F_yf * std::cos(start.steer_angle)) - start_vx * start.angular_velocity;
    double theta_ddot = (1/p.I_z) * (-F_yr * p.l_r + F_yf * p.l_f * std::cos(start.steer_angle));

    double end_vx = start_vx + vx_dot * dt;
    double end_vy = start_vy + vy_dot * dt;

    // update state
    end.x = start.x + x_dot * dt;
    end.y = start.y + y_dot * dt;
    end.theta = start.theta + theta_dot * dt;
    end.v_x = end_vx;
    end.v_y = end_vy;
    end.steer_angle = start.steer_angle + steer_angle_dot * dt;
    end.angular_velocity = start.angular_velocity + theta_ddot * dt;
    end.slip_angle = std::atan2(end_vy, end_vx);

    // MIX with kinematic model at low speeds
    // kinematic system dynamics
    CarState kin_end = update_k(
                start,
                accel,
                steer_angle_vel,
                p,
                dt);
    
    // weights for mixing
    double w_std = 0.5 * (1 + std::tanh((start.v_x - v_b) / v_s));
    double w_kin = 1 - w_std;
    if (start.v_x < v_min) {
        w_std = 0;
        w_kin = 1;
    }

    // mix states
    end.x = w_std*end.x + w_kin*kin_end.x;
    end.y = w_std*end.y + w_kin*kin_end.y;
    end.theta = w_std*end.theta + w_kin*kin_end.theta;
    end.v_x = w_std*end.v_x + w_kin*kin_end.v_x;
    end.v_y = w_std*end.v_y + w_kin*kin_end.v_y;
    end.steer_angle = w_std*end.steer_angle + w_kin*kin_end.steer_angle;
    end.angular_velocity = w_std*end.angular_velocity + w_kin*kin_end.angular_velocity;
    end.slip_angle = w_std*end.slip_angle + w_kin*kin_end.slip_angle;
    end.st_dyn = w_std*end.st_dyn + w_kin*kin_end.st_dyn;
    end.st_dyn = false;

    return end;
}

CarState STDKinematics::update_linear(
        const CarState start,
        double accel,
        double steer_angle_vel,
        CarParams p,
        double dt) {
    /* Model for the single track dynamics of the vehicle
    It uses the Pacejka tire model for the lateral tire forces
    
    state
    start.x = x-position in global frame [m]
    start.y = y-position in global frame [m]
    start.theta = orientation in global frame [rad]
    start.v_x = longitudinal velocity [m/s]
    start.v_y = lateral velocity [m/s]
    start.steer_angle = steering angle [rad]
    start.angular_velocity = angular velocity [rad/s]
    start.slip_angle = slip angle [rad]
    start.omega_fw = front wheel angular velocity [rad/s]
    start.omega_rw = rear wheel angular velocity [rad/s]
    start.st_dyn = true if using ST dynamics, false if using kinematic model
    
    input 
    accel = acceleration [m/s^2]
    steer_angle_vel = steering angle velocity [rad/s] */

    CarState end;

    // local params
    double v_b = 3; // m/s
    double v_s = 1; // m/s
    double v_min = v_b-2*v_s; // m/s
    double g = 9.81; // m/s^2

    // in the original code constraints were applied here, but I think they are already applied outside
    // therefore we skip to the DYNAMICS computation directly
    
    // lateral tire slip angles
    double alpha_f;
    double alpha_r;
    double start_vx = start.v_x;
    double start_vy = start.v_y;
    if (start.v_x >= v_min){
        alpha_f = std::atan2((-start_vy - p.l_f * start.angular_velocity), start_vx) + start.steer_angle;
        alpha_r = std::atan2((-start_vy + p.l_r * start.angular_velocity), start_vx);
    } else {
        alpha_f = 0;
        alpha_r = 0;
    }

    // compute vertical tire forces (load transfer due to acceleration)
    double F_zf = p.mass * (-accel * p.h_cg + g * p.l_r) / (p.l_f + p.l_r);
    double F_zr = p.mass * (accel * p.h_cg + g * p.l_f) / (p.l_f + p.l_r);

    // Linear tire model
    double F_yf = p.friction_coeff * F_zf * p.cs_f * alpha_f;
    double F_yr = p.friction_coeff * F_zr * p.cs_r * alpha_r;

    // compute first derivatives of state
    double x_dot = start_vx * std::cos(start.theta) - start_vy * std::sin(start.theta);
    double y_dot = start_vx * std::sin(start.theta) + start_vy * std::cos(start.theta);
    double vx_dot = accel + (1/p.mass) * (- F_yf * std::sin(start.steer_angle)) + start_vy * start.angular_velocity;
    double steer_angle_dot = steer_angle_vel;
    double theta_dot = start.angular_velocity;
    double vy_dot = (1/p.mass) * (F_yr + F_yf * std::cos(start.steer_angle)) - start_vx * start.angular_velocity;
    double theta_ddot = (1/p.I_z) * (-F_yr * p.l_r + F_yf * p.l_f * std::cos(start.steer_angle));

    double end_vx = start_vx + vx_dot * dt;
    double end_vy = start_vy + vy_dot * dt;

    // update state
    end.x = start.x + x_dot * dt;
    end.y = start.y + y_dot * dt;
    end.theta = start.theta + theta_dot * dt;
    end.v_x = end_vx;
    end.v_y = end_vy;
    end.steer_angle = start.steer_angle + steer_angle_dot * dt;
    end.angular_velocity = start.angular_velocity + theta_ddot * dt;
    end.slip_angle = std::atan2(end_vy, end_vx);

    // MIX with kinematic model at low speeds
    // kinematic system dynamics
    CarState kin_end = update_k(
                start,
                accel,
                steer_angle_vel,
                p,
                dt);
    
    // weights for mixing
    double w_std = 0.5 * (1 + std::tanh((start.v_x - v_b) / v_s));
    double w_kin = 1 - w_std;
    if (start.v_x < v_min) {
        w_std = 0;
        w_kin = 1;
    }

    // mix states
    end.x = w_std*end.x + w_kin*kin_end.x;
    end.y = w_std*end.y + w_kin*kin_end.y;
    end.theta = w_std*end.theta + w_kin*kin_end.theta;
    end.v_x = w_std*end.v_x + w_kin*kin_end.v_x;
    end.v_y = w_std*end.v_y + w_kin*kin_end.v_y;
    end.steer_angle = w_std*end.steer_angle + w_kin*kin_end.steer_angle;
    end.angular_velocity = w_std*end.angular_velocity + w_kin*kin_end.angular_velocity;
    end.slip_angle = w_std*end.slip_angle + w_kin*kin_end.slip_angle;
    end.st_dyn = w_std*end.st_dyn + w_kin*kin_end.st_dyn;
    end.st_dyn = false;

    return end;
}


CarState STDKinematics::update_k(
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
    double steer_angle_dot = steer_angle_vel;
    double theta_dot = start.v_x * std::tan(start.steer_angle) / p.wheelbase ;
    double slip_angle_dot = (1 / (1 + std::pow(((p.l_r/p.wheelbase) * std::tan(start.steer_angle)), 2))) *
            (p.l_r / (p.wheelbase * std::pow(std::cos(start.steer_angle), 2))) * steer_angle_vel;
    double theta_double_dot = accel * std::tan(start.steer_angle) * std::cos(start.slip_angle)/ p.wheelbase  +
            start.v_x * steer_angle_vel / (p.wheelbase * std::pow(std::cos(start.steer_angle), 2)) -
            start.v_y * std::tan(start.steer_angle) * slip_angle_dot / p.wheelbase;

    // update state
    end.x = start.x + x_dot * dt;
    end.y = start.y + y_dot * dt;
    end.theta = start.theta + theta_dot * dt;
    end.v_x = start.v_x + vx_dot * dt;
    end.slip_angle = start.slip_angle + slip_angle_dot * dt;
    end.v_y = std::tan(end.slip_angle) * end.v_x;
    end.steer_angle = start.steer_angle + steer_angle_dot * dt;
    end.angular_velocity = start.angular_velocity + theta_double_dot * dt;
    end.st_dyn = false;

    return end;

}
