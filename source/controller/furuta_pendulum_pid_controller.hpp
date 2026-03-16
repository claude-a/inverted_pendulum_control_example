#ifndef FURUTA_PENDULUM_PID_CONTROLLER_HPP
#define FURUTA_PENDULUM_PID_CONTROLLER_HPP

#include <cmath>

// Header generated to mirror Python class FurutaPendulum_PID_Controller
// Contains public member variables to match Python attribute access.

class FurutaPendulum_PID_Controller {
public:
  // Defaults (from Python source)
  static constexpr double TS_DEFAULT = 0.005;
  static constexpr double TS_MIN = 1e-6;

  static constexpr double V_LIMIT_DEFAULT = 12.0;
  static constexpr double ALPHA_REF_LIMIT_RAD_DEFAULT = 12.0 * 3.14159265358979323846 / 180.0; // 12 deg in rad
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

  // Constructor mirrors Python __init__ signature and default values
  FurutaPendulum_PID_Controller(
      double Ts = TS_DEFAULT,
      double theta_ref_rad = 0.0,
      double alpha_ref_rad = 0.0,
      double v_limit = V_LIMIT_DEFAULT,
      double alpha_ref_limit_rad = ALPHA_REF_LIMIT_RAD_DEFAULT,
      double theta_to_alpha_sign = THETA_TO_ALPHA_SIGN_DEFAULT);

  ~FurutaPendulum_PID_Controller() = default;

  // Public members (to mirror Python attributes)
  double Ts;                     // sampling time [s]

  // References (can be changed later)
  double theta_ref_rad;
  double alpha_ref_rad;

  // Voltage saturation used for anti-windup
  double v_limit;

  // Limit for commanded pendulum tilt generated from theta control
  double alpha_ref_limit_rad;

  // Mapping sign: if theta is positive, alpha_ref_cmd positive (or -1)
  double theta_to_alpha_sign;

  // === Tuning gains ===
  // Theta PID gains
  double kp_theta;
  double ki_theta;
  double kd_theta;

  // Pendulum PID gains (motor voltage output)
  double kp_alpha;
  double ki_alpha;
  double kd_alpha;

  // Integrator states
  double _int_theta;
  double _int_alpha;

  // Simple derivative filtering (on measurement)
  double _dalpha_filt;
  double _dalpha_tau; // [s]
  double _dtheta_filt;
  double _dtheta_tau; // [s]

  // The implementation (.cpp) should initialize members consistently with the
  // Python constructor behavior (abs for limits, sign mapping, etc.).

private:
  // No private data at this time. All attributes are public to mirror Python.
};

#endif // FURUTA_PENDULUM_PID_CONTROLLER_HPP
