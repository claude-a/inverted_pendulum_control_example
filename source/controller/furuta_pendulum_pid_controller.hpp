#ifndef FURUTA_PENDULUM_PID_CONTROLLER_HPP
#define FURUTA_PENDULUM_PID_CONTROLLER_HPP

#include <cmath>
#include <cstddef>

// C++11 header-only declaration mirroring the Python
// class FurutaPendulum_PID_Controller

class FurutaPendulum_PID_Controller {
public:
  // Default / tuning constants (mirrors Python defaults)
  static constexpr double TS_DEFAULT = 0.005;
  static constexpr double TS_MIN = 1e-6;
  static constexpr double V_LIMIT_DEFAULT = 12.0;
  static constexpr double ALPHA_REF_LIMIT_RAD_DEFAULT = 0.20943951023931953; // 12 deg in rad
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

  // Constructor mirrors Python __init__ signature. The parameter
  // alpha_ref_limit_rad uses a negative sentinel to indicate "not provided"
  // (Python used None to trigger the default).
  FurutaPendulum_PID_Controller(
      double Ts = TS_DEFAULT,
      double theta_ref_rad = 0.0,
      double alpha_ref_rad = 0.0,
      double v_limit = V_LIMIT_DEFAULT,
      double alpha_ref_limit_rad = -1.0,
      double theta_to_alpha_sign = THETA_TO_ALPHA_SIGN_DEFAULT) noexcept;

  // Public members mirror the Python attributes so callers can read/write
  // them similarly to the Python implementation.

  // Sampling time
  double Ts;

  // References
  double theta_ref_rad;
  double alpha_ref_rad;

  // Voltage saturation used for anti-windup
  double v_limit;

  // Limit for commanded pendulum tilt generated from theta control
  double alpha_ref_limit_rad;

  // Mapping sign: if theta is positive, alpha_ref_cmd should be positive
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

  // Reset method mirrors the newly added Python method that clears internal states
  void reset() noexcept;

  // Destructor defaulted
  ~FurutaPendulum_PID_Controller() = default;

private:
  // No dynamic memory and no while loops in this class (per constraints).
};

#endif // FURUTA_PENDULUM_PID_CONTROLLER_HPP
