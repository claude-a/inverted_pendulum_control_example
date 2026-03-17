#ifndef FURUTA_PENDULUM_PID_CONTROLLER_HPP
#define FURUTA_PENDULUM_PID_CONTROLLER_HPP

#include <cmath>
#include <limits>
#include <algorithm>

// Converted from Python: FurutaPendulum_PID_Controller
// C++11 header-only class declaration and inline constructor.

class FurutaPendulum_PID_Controller {
public:
    // Module-level defaults (from original Python)
    static constexpr double TS_DEFAULT = 0.005;
    static constexpr double TS_MIN = 1e-6;
    static constexpr double V_LIMIT_DEFAULT = 12.0;
    static constexpr double ALPHA_REF_LIMIT_RAD_DEFAULT = 12.0 * 3.14159265358979323846 / 180.0;
    static constexpr double THETA_TO_ALPHA_SIGN_DEFAULT = 1.0;
    static constexpr double KP_THETA_DEFAULT = 0.1;
    static constexpr double KI_THETA_DEFAULT = 0.0;
    static constexpr double KD_THETA_DEFAULT = 0.05;
    static constexpr double KP_ALPHA_DEFAULT = 100.0;
    static constexpr double KI_ALPHA_DEFAULT = 0.0;
    static constexpr double KD_ALPHA_DEFAULT = 10.0;
    static constexpr double DTHETA_TAU_DEFAULT = 0.02;
    static constexpr double DALPHA_TAU_DEFAULT = 0.02;
    static constexpr double EPS_VALUE = 1e-12;

    // Public state and configuration (mirrors Python attributes)
    double Ts;

    // References (can be changed later)
    double theta_ref_rad;
    double alpha_ref_rad;

    // Voltage saturation used for anti-windup
    double v_limit;

    // Limit for commanded pendulum tilt generated from theta control
    double alpha_ref_limit_rad;

    // Mapping sign: if theta is positive, we want alpha_ref_cmd to be positive.
    double theta_to_alpha_sign;

    // Tuning gains
    double kp_theta;
    double ki_theta;
    double kd_theta;

    double kp_alpha;
    double ki_alpha;
    double kd_alpha;

    // Integrator states
    double _int_theta;
    double _int_alpha;

    // Simple derivative filtering (on measurement)
    double _dalpha_filt;
    double _dalpha_tau;
    double _dtheta_filt;
    double _dtheta_tau;

    // Constructor: mirrors Python __init__ signature. Uses NaN to represent "None" for alpha_ref_limit_rad.
    FurutaPendulum_PID_Controller(
        double Ts_ = TS_DEFAULT,
        double theta_ref_rad_ = 0.0,
        double alpha_ref_rad_ = 0.0,
        double v_limit_ = V_LIMIT_DEFAULT,
        double alpha_ref_limit_rad_ = std::numeric_limits<double>::quiet_NaN(),
        double theta_to_alpha_sign_ = THETA_TO_ALPHA_SIGN_DEFAULT
    ) {
        // Sample time
        Ts = static_cast<double>(Ts_);

        // References
        theta_ref_rad = static_cast<double>(theta_ref_rad_);
        alpha_ref_rad = static_cast<double>(alpha_ref_rad_);

        // Voltage saturation
        v_limit = std::abs(static_cast<double>(v_limit_));

        // Alpha ref limit: if NaN (used as None), use default
        if (std::isnan(alpha_ref_limit_rad_)) {
            alpha_ref_limit_rad = std::abs(ALPHA_REF_LIMIT_RAD_DEFAULT);
        } else {
            alpha_ref_limit_rad = std::abs(static_cast<double>(alpha_ref_limit_rad_));
        }

        // Mapping sign
        theta_to_alpha_sign = (theta_to_alpha_sign_ >= 0.0) ? 1.0 : -1.0;

        // Tuning gains (start conservative; tune as needed)
        kp_theta = KP_THETA_DEFAULT;
        ki_theta = KI_THETA_DEFAULT;
        kd_theta = KD_THETA_DEFAULT;

        kp_alpha = KP_ALPHA_DEFAULT;
        ki_alpha = KI_ALPHA_DEFAULT;
        kd_alpha = KD_ALPHA_DEFAULT;

        // Integrator states
        _int_theta = 0.0;
        _int_alpha = 0.0;

        // Derivative filtering
        _dalpha_filt = 0.0;
        _dalpha_tau = DALPHA_TAU_DEFAULT;
        _dtheta_filt = 0.0;
        _dtheta_tau = DTHETA_TAU_DEFAULT;
    }

    // Additional methods that existed in Python should be declared here as needed.
    // No dynamic memory allocation and no while-loops are used in this header.
};

#endif // FURUTA_PENDULUM_PID_CONTROLLER_HPP
