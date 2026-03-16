#ifndef FURUTA_PENDULUM_PID_CONTROLLER_HPP
#define FURUTA_PENDULUM_PID_CONTROLLER_HPP

#include <cmath>
#include <cstdint>

namespace furuta_pid {

// Defaults (match Python constants)
static constexpr double TS_DEFAULT = 0.005;
static constexpr double TS_MIN = 1e-6;

static constexpr double V_LIMIT_DEFAULT = 12.0;
static constexpr double ALPHA_REF_LIMIT_RAD_DEFAULT = 12.0 * M_PI / 180.0;
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

class FurutaPendulum_PID_Controller {
public:
  // Constructor mirrors Python signature and behavior. alpha_ref_limit_rad
  // defaults to ALPHA_REF_LIMIT_RAD_DEFAULT (Python uses None -> default).
  FurutaPendulum_PID_Controller(
      double Ts = TS_DEFAULT,
      double theta_ref_rad = 0.0,
      double alpha_ref_rad = 0.0,
      double v_limit = V_LIMIT_DEFAULT,
      double alpha_ref_limit_rad = ALPHA_REF_LIMIT_RAD_DEFAULT,
      double theta_to_alpha_sign = THETA_TO_ALPHA_SIGN_DEFAULT)
      : Ts(static_cast<double>(Ts)),
        theta_ref_rad(static_cast<double>(theta_ref_rad)),
        alpha_ref_rad(static_cast<double>(alpha_ref_rad)),
        v_limit(std::fabs(v_limit)),
        alpha_ref_limit_rad(std::fabs(alpha_ref_limit_rad)),
        theta_to_alpha_sign((theta_to_alpha_sign >= 0.0) ? 1.0 : -1.0),
        // PID gains (tunable)
        kp_theta(KP_THETA_DEFAULT),
        ki_theta(KI_THETA_DEFAULT),
        kd_theta(KD_THETA_DEFAULT),
        kp_alpha(KP_ALPHA_DEFAULT),
        ki_alpha(KI_ALPHA_DEFAULT),
        kd_alpha(KD_ALPHA_DEFAULT),
        // integrator and filter initial states
        _int_theta(0.0),
        _int_alpha(0.0),
        _dalpha_filt(0.0),
        _dalpha_tau(DALPHA_TAU_DEFAULT),
        _dtheta_filt(0.0),
        _dtheta_tau(DTHETA_TAU_DEFAULT) {}

  // Public member variables mirror the Python attributes for simplicity and
  // direct access from calling code.

  // Sampling time
  double Ts;

  // References (can be changed later)
  double theta_ref_rad;
  double alpha_ref_rad;

  // Voltage saturation used for anti-windup
  double v_limit;

  // Limit for commanded pendulum tilt generated from theta control
  double alpha_ref_limit_rad;

  // Mapping sign for theta->alpha
  double theta_to_alpha_sign;

  // === Tuning gains ===
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

  // No dynamic allocation or while-loops are used in this header-only
  // representation. Any algorithms operating on these members should follow
  // the same constraints.
};

} // namespace furuta_pid

#endif // FURUTA_PENDULUM_PID_CONTROLLER_HPP
