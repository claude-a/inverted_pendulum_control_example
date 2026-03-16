#include "furuta_pendulum_pid_controller.hpp"

#include <cmath>

// Implementation of FurutaPendulum_PID_Controller methods

FurutaPendulum_PID_Controller::FurutaPendulum_PID_Controller(
    double Ts,
    double theta_ref_rad,
    double alpha_ref_rad,
    double v_limit,
    double alpha_ref_limit_rad,
    double theta_to_alpha_sign) noexcept {
  // Sampling time
  this->Ts = static_cast<double>(Ts);

  // References
  this->theta_ref_rad = static_cast<double>(theta_ref_rad);
  this->alpha_ref_rad = static_cast<double>(alpha_ref_rad);

  // Voltage saturation used for anti-windup
  this->v_limit = std::fabs(v_limit);

  // alpha_ref_limit_rad: Python used None to select default. Here a negative
  // sentinel means "not provided". If negative, use default.
  if (alpha_ref_limit_rad < 0.0) {
    this->alpha_ref_limit_rad = ALPHA_REF_LIMIT_RAD_DEFAULT;
  } else {
    this->alpha_ref_limit_rad = std::fabs(alpha_ref_limit_rad);
  }

  // Mapping sign: mirror Python's behavior
  this->theta_to_alpha_sign = (theta_to_alpha_sign >= 0.0) ? 1.0 : -1.0;

  // === Tuning gains (initialize to Python defaults) ===
  this->kp_theta = KP_THETA_DEFAULT;
  this->ki_theta = KI_THETA_DEFAULT;
  this->kd_theta = KD_THETA_DEFAULT;

  this->kp_alpha = KP_ALPHA_DEFAULT;
  this->ki_alpha = KI_ALPHA_DEFAULT;
  this->kd_alpha = KD_ALPHA_DEFAULT;

  // Integrator states
  this->_int_theta = 0.0;
  this->_int_alpha = 0.0;

  // Simple derivative filtering (on measurement)
  this->_dalpha_filt = 0.0;
  this->_dalpha_tau = DALPHA_TAU_DEFAULT;
  this->_dtheta_filt = 0.0;
  this->_dtheta_tau = DTHETA_TAU_DEFAULT;
}

// Reset method mirrors the Python reset() which clears internal states
void FurutaPendulum_PID_Controller::reset() noexcept {
  this->_int_theta = 0.0;
  this->_int_alpha = 0.0;
  this->_dalpha_filt = 0.0;
  this->_dtheta_filt = 0.0;
}
