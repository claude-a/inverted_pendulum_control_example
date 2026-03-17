#include "furuta_pendulum_pid_controller.hpp"

FurutaPendulum_PID_Controller::FurutaPendulum_PID_Controller(
    double Ts_ = TS_DEFAULT, double theta_ref_rad_ = 0.0,
    double alpha_ref_rad_ = 0.0, double v_limit_ = V_LIMIT_DEFAULT,
    double alpha_ref_limit_rad_ = std::numeric_limits<double>::quiet_NaN(),
    double theta_to_alpha_sign_ = THETA_TO_ALPHA_SIGN_DEFAULT) {
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
  if (theta_to_alpha_sign_ >= 0) {
    theta_to_alpha_sign = 1.0;
  } else {
    theta_to_alpha_sign = -1.0;
  }

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
