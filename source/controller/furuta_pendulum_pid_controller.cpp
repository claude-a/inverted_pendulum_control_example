#include "furuta_pendulum_pid_controller.hpp"
#include <algorithm>
#include <stdexcept>

namespace furuta_pid {

static inline double clamp_double(double v, double lo, double hi) {
  return (v < lo) ? lo : (v > hi) ? hi : v;
}

void reset(FurutaPendulum_PID_Controller &c) {
  c._int_theta = 0.0;
  c._int_alpha = 0.0;
  c._dalpha_filt = 0.0;
  c._dtheta_filt = 0.0;
}

// Update derivative filter state for a measured derivative value.
// Uses first-order low-pass: y_new = a * y_prev + (1-a) * meas_dot,
// where a = tau/(tau+Ts). Safely handles very small Ts/tau.
static inline double update_derivative_filter(double prev_filt, double meas_dot,
                                              double tau, double Ts) {
  const double denom = tau + Ts;
  if (denom <= EPS_VALUE) {
    // If numerically degenerate, fall back to direct measurement
    return meas_dot;
  }
  const double a = tau / denom;
  const double b = Ts / denom; // = 1 - a
  return a * prev_filt + b * meas_dot;
}

// Run the theta PID which generates an alpha reference command (rad).
// - theta_meas_rad: measured base angle [rad]
// - dtheta_meas_rad: measured derivative of base angle [rad/s]
// Returns alpha_ref_cmd (rad) clamped to controller.alpha_ref_limit_rad
double update_theta_controller(FurutaPendulum_PID_Controller &controller,
                               double theta_meas_rad,
                               double dtheta_meas_rad) {
  // Error (reference - measurement)
  const double err = controller.theta_ref_rad - theta_meas_rad;

  // Integrator update (theta integrator integrates error)
  // Use simple Euler integration (no dynamic allocation, no while)
  const double int_increment = controller.ki_theta * err * controller.Ts;
  controller._int_theta += int_increment;

  // Derivative on measurement (filtered)
  controller._dtheta_filt = update_derivative_filter(controller._dtheta_filt,
                                                      dtheta_meas_rad,
                                                      controller._dtheta_tau,
                                                      controller.Ts);

  // PID output: note derivative on measurement is subtracted (same sign as Python)
  const double p_term = controller.kp_theta * err;
  const double d_term = controller.kd_theta * controller._dtheta_filt;
  const double theta_pid_output = p_term + controller._int_theta - d_term;

  // Map theta PID output to alpha command with sign mapping
  double alpha_ref_cmd = controller.theta_to_alpha_sign * theta_pid_output;

  // Clamp alpha reference to limits
  alpha_ref_cmd = clamp_double(alpha_ref_cmd, -controller.alpha_ref_limit_rad,
                               controller.alpha_ref_limit_rad);

  return alpha_ref_cmd;
}

// Run the alpha PID which generates motor voltage command [V].
// - alpha_ref_cmd: commanded pendulum tilt [rad]
// - alpha_meas_rad: measured pendulum tilt [rad]
// - dalpha_meas_rad: measured pendulum tilt derivative [rad/s]
// Returns voltage command clamped to +/- v_limit
double update_alpha_controller(FurutaPendulum_PID_Controller &controller,
                               double alpha_ref_cmd, double alpha_meas_rad,
                               double dalpha_meas_rad) {
  const double err = alpha_ref_cmd - alpha_meas_rad;

  // Integrator update for alpha
  const double int_increment = controller.ki_alpha * err * controller.Ts;
  controller._int_alpha += int_increment;

  // Derivative on measurement (filtered)
  controller._dalpha_filt = update_derivative_filter(controller._dalpha_filt,
                                                      dalpha_meas_rad,
                                                      controller._dalpha_tau,
                                                      controller.Ts);

  const double p_term = controller.kp_alpha * err;
  const double d_term = controller.kd_alpha * controller._dalpha_filt;
  double v_cmd = p_term + controller._int_alpha - d_term;

  // Saturation
  const double v_sat = clamp_double(v_cmd, -controller.v_limit, controller.v_limit);

  // Anti-windup: if saturated and integrator pushed further into saturation,
  // undo last integrator increment. This is a simple conditional back-calculation
  // style without adding extra state.
  if (v_cmd != v_sat) {
    // Undo integrator increment to prevent wind-up
    controller._int_alpha -= int_increment;
    v_cmd = v_sat;
  }

  return v_cmd;
}

// Convenience function that runs full two-stage control loop and returns voltage.
// Inputs:
//  - theta_meas_rad, dtheta_meas_rad: base (actuated) angle and derivative
//  - alpha_meas_rad, dalpha_meas_rad: pendulum angle and derivative
// Output: motor voltage command [V]
// This mirrors the Python structure where theta PID produces alpha_ref_cmd that
// the alpha PID then tracks.
double compute_control(FurutaPendulum_PID_Controller &controller,
                       double theta_meas_rad, double dtheta_meas_rad,
                       double alpha_meas_rad, double dalpha_meas_rad) {
  // Compute alpha reference from theta controller
  const double alpha_ref_cmd = update_theta_controller(controller, theta_meas_rad,
                                                       dtheta_meas_rad);

  // Optionally store the commanded alpha in the public member so calling code
  // can inspect it (header exposes alpha_ref_rad). Keep original behavior by
  // updating the attribute but not modifying user-provided reference logic.
  controller.alpha_ref_rad = alpha_ref_cmd;

  // Compute voltage from alpha controller
  const double v_cmd = update_alpha_controller(controller, alpha_ref_cmd,
                                                alpha_meas_rad, dalpha_meas_rad);
  return v_cmd;
}

} // namespace furuta_pid
