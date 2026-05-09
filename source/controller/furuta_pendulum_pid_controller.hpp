#ifndef FURUTA_PENDULUM_PID_CONTROLLER_HPP
#define FURUTA_PENDULUM_PID_CONTROLLER_HPP

const float KP_THETA_DEFAULT = 0.1f;

class FurutaPendulum_PID_Controller {
public:
    FurutaPendulum_PID_Controller() : kp_theta(KP_THETA_DEFAULT) {}

    float calculate_manipulation(
        float theta,
        float alpha,
        float dtheta,
        float dalpha
    ) {
        return -kp_theta * theta;
    }

private:
    float kp_theta;
};

#endif // FURUTA_PENDULUM_PID_CONTROLLER_HPP
