#pragma once

namespace racecar_simulator {

struct CarState {
    double x; // x position
    double y; // y position
    double theta; // orientation
    double v_x; // velocity
    double v_y; // lateral velocity
    double steer_angle; // steering angle
    double angular_velocity; // angular velocity
    double slip_angle; // slip angle
    bool st_dyn;
};

}
