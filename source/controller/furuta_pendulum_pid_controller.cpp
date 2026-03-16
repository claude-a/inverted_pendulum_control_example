#include "furuta_pendulum_pid_controller.hpp"
#include <cmath>

FurutaPendulum_PID_Controller::FurutaPendulum_PID_Controller(FLOAT Ts,
                                                             FLOAT theta_ref_rad,
                                                             FLOAT alpha_ref_rad,
                                                             FLOAT v_limit,
                                                             FLOAT alpha_ref_limit_rad,
                                                             FLOAT theta_to_alpha_sign)
    : Ts(static_cast<FLOAT>(Ts)),
      theta_ref_rad(static_cast<FLOAT>(theta_ref_rad)),
      alpha_ref_rad(static_cast<FLOAT>(alpha_ref_rad)),
      v_limit(static_cast<FLOAT>(std::abs(v_limit))),
      alpha_ref_limit_rad(static_cast<FLOAT>(
          (alpha_ref_limit_rad < 0.0) ? ALPHA_REF_LIMIT_RAD_DEFAULT
                                      : std::abs(alpha_ref_limit_rad)))),
      theta_to_alpha_sign(theta_to_alpha_sign >= 0.0 ? 1.0 : -1.0),
      kp_theta(KP_THETA_DEFAULT), ki_theta(KI_THETA_DEFAULT), kd_theta(KD_THETA_DEFAULT),
      kp_alpha(KP_ALPHA_DEFAULT), ki_alpha(KI_ALPHA_DEFAULT), kd_alpha(KD_ALPHA_DEFAULT),
      _int_theta(0.0), _int_alpha(0.0),
      _dalpha_filt(0.0), _dalpha_tau(DALPHA_TAU_DEFAULT),
      _dtheta_filt(0.0), _dtheta_tau(DTHETA_TAU_DEFAULT) {
  // Enforce minimum timestep
  if (this->Ts < TS_MIN) {
    this->Ts = TS_MIN;
  }
}

void FurutaPendulum_PID_Controller::reset_integrators() {
  _int_theta = 0.0;
  _int_alpha = 0.0;
}

void FurutaPendulum_PID_Controller::set_dalpha_tau(FLOAT tau) {
  // Keep tau non-negative; tiny values allowed
  _dalpha_tau = (tau < 0.0) ? 0.0 : tau;
}

void FurutaPendulum_PID_Controller::set_dtheta_tau(FLOAT tau) {
  _dtheta_tau = (tau < 0.0) ? 0.0 : tau;
}

FurutaPendulum_PID_Controller::FLOAT FurutaPendulum_PID_Controller::int_theta() const {
  return _int_theta;
}

FurutaPendulum_PID_Controller::FLOAT FurutaPendulum_PID_Controller::int_alpha() const {
  return _int_alpha;
}

FurutaPendulum_PID_Controller::FLOAT FurutaPendulum_PID_Controller::dalpha_filt() const {
  return _dalpha_filt;
}

FurutaPendulum_PID_Controller::FLOAT FurutaPendulum_PID_Controller::dtheta_filt() const {
  return _dtheta_filt;
}

FurutaPendulum_PID_Controller::FLOAT FurutaPendulum_PID_Controller::dalpha_tau() const {
  return _dalpha_tau;
}

FurutaPendulum_PID_Controller::FLOAT FurutaPendulum_PID_Controller::dtheta_tau() const {
  return _dtheta_tau;
}
