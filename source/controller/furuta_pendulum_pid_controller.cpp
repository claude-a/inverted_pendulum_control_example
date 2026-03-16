#include "furuta_pendulum_pid_controller.hpp"
#include <cmath>

// Implementation of FurutaPendulum_PID_Controller constructor
FurutaPendulum_PID_Controller::FurutaPendulum_PID_Controller(
    double Ts,
    double theta_ref_rad,
    double alpha_ref_rad,
    double v_limit,
    double alpha_ref_limit_rad,
    double theta_to_alpha_sign)
    : Ts(static_cast<double>(Ts)),
      theta_ref_rad(static_cast<double>(theta_ref_rad)),
      alpha_ref_rad(static_cast<double>(alpha_ref_rad)),
      v_limit(std::fabs(v_limit)),
      alpha_ref_limit_rad(std::fabs(alpha_ref_limit_rad)),
      theta_to_alpha_sign(theta_to_alpha_sign >= 0.0 ? 1.0 : -1.0),
      kp_theta(KP_THETA_DEFAULT),
      ki_theta(KI_THETA_DEFAULT),
      kd_theta(KD_THETA_DEFAULT),
      kp_alpha(KP_ALPHA_DEFAULT),
      ki_alpha(KI_ALPHA_DEFAULT),
      kd_alpha(KD_ALPHA_DEFAULT),
      _int_theta(0.0),
      _int_alpha(0.0),
      _dalpha_filt(0.0),
      _dalpha_tau(DALPHA_TAU_DEFAULT),
      _dtheta_filt(0.0),
      _dtheta_tau(DTHETA_TAU_DEFAULT) {
  // Constructor mirrors Python initialization semantics:
  // - Ts, theta_ref_rad, alpha_ref_rad stored as doubles
  // - v_limit and alpha_ref_limit_rad use absolute value
  // - theta_to_alpha_sign mapped to +1 or -1 based on sign
  // All integrators and derivative filters initialized to zero; gains set to defaults.
}
