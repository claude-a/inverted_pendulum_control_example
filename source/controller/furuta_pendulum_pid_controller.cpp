#include "furuta_pendulum_pid_controller.hpp"

FurutaPendulum_PID_Controller::FurutaPendulum_PID_Controller()
    : kp_theta(KP_THETA_DEFAULT)
{
}

float FurutaPendulum_PID_Controller::calculate_manipulation(
    float theta,
    float alpha,
    float dtheta,
    float dalpha
)
{
    return -kp_theta * theta;
}
