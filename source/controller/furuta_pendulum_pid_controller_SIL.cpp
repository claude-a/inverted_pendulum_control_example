#include <pybind11/pybind11.h>
#include "furuta_pendulum_pid_controller.hpp"

namespace py = pybind11;

using furuta_pid::FurutaPendulum_PID_Controller;

// Forward declarations for functions implemented in the library .cpp
namespace furuta_pid {
double update_theta_controller(FurutaPendulum_PID_Controller &controller,
                               double theta_meas_rad,
                               double dtheta_meas_rad);

double update_alpha_controller(FurutaPendulum_PID_Controller &controller,
                               double alpha_ref_cmd,
                               double alpha_meas_rad,
                               double dalpha_meas_rad);

double compute_control(FurutaPendulum_PID_Controller &controller,
                       double theta_meas_rad,
                       double dtheta_meas_rad,
                       double alpha_meas_rad,
                       double dalpha_meas_rad);

void reset(FurutaPendulum_PID_Controller &c);
} // namespace furuta_pid

namespace furuta_pid_SIL {

static FurutaPendulum_PID_Controller controller;

// Initialize/reset the controller instance to defaults
void initialize() { controller = FurutaPendulum_PID_Controller(); }

// Wrapper: run theta controller and return commanded alpha_ref_cmd [rad]
double update_theta(double theta_meas_rad, double dtheta_meas_rad) {
  return furuta_pid::update_theta_controller(controller, theta_meas_rad,
                                              dtheta_meas_rad);
}

// Wrapper: run alpha controller and return motor voltage [V]
double update_alpha(double alpha_ref_cmd, double alpha_meas_rad,
                     double dalpha_meas_rad) {
  return furuta_pid::update_alpha_controller(controller, alpha_ref_cmd,
                                              alpha_meas_rad, dalpha_meas_rad);
}

// Wrapper: run full compute_control (theta->alpha->voltage)
double compute_control(double theta_meas_rad, double dtheta_meas_rad,
                       double alpha_meas_rad, double dalpha_meas_rad) {
  return furuta_pid::compute_control(controller, theta_meas_rad,
                                     dtheta_meas_rad, alpha_meas_rad,
                                     dalpha_meas_rad);
}

// Reset internal integrators / filters
void reset_controller() { furuta_pid::reset(controller); }

// Simple setters/getters for common parameters and gains
void set_Ts(double v) { controller.Ts = v; }
double get_Ts() { return controller.Ts; }

void set_theta_ref_rad(double v) { controller.theta_ref_rad = v; }
double get_theta_ref_rad() { return controller.theta_ref_rad; }

void set_alpha_ref_rad(double v) { controller.alpha_ref_rad = v; }
double get_alpha_ref_rad() { return controller.alpha_ref_rad; }

void set_v_limit(double v) { controller.v_limit = std::fabs(v); }
double get_v_limit() { return controller.v_limit; }

void set_alpha_ref_limit_rad(double v) { controller.alpha_ref_limit_rad = std::fabs(v); }
double get_alpha_ref_limit_rad() { return controller.alpha_ref_limit_rad; }

void set_theta_to_alpha_sign(double v) { controller.theta_to_alpha_sign = (v >= 0.0) ? 1.0 : -1.0; }
double get_theta_to_alpha_sign() { return controller.theta_to_alpha_sign; }

void set_theta_gains(double kp, double ki, double kd) {
  controller.kp_theta = kp;
  controller.ki_theta = ki;
  controller.kd_theta = kd;
}

void get_theta_gains(double &kp, double &ki, double &kd) {
  kp = controller.kp_theta;
  ki = controller.ki_theta;
  kd = controller.kd_theta;
}

void set_alpha_gains(double kp, double ki, double kd) {
  controller.kp_alpha = kp;
  controller.ki_alpha = ki;
  controller.kd_alpha = kd;
}

void get_alpha_gains(double &kp, double &ki, double &kd) {
  kp = controller.kp_alpha;
  ki = controller.ki_alpha;
  kd = controller.kd_alpha;
}

// Expose some internal states for inspection
double get_int_theta() { return controller._int_theta; }
double get_int_alpha() { return controller._int_alpha; }

double get_dtheta_filt() { return controller._dtheta_filt; }
double get_dalpha_filt() { return controller._dalpha_filt; }

} // namespace furuta_pid_SIL

PYBIND11_MODULE(FurutaPendulumPIDControllerSIL, m) {
  m.doc() = "Furuta pendulum PID controller SIL bindings";

  m.def("initialize", &furuta_pid_SIL::initialize, "Initialize controller");
  m.def("reset", &furuta_pid_SIL::reset_controller, "Reset integrators/filters");

  m.def("update_theta", &furuta_pid_SIL::update_theta,
        py::arg("theta_meas_rad"), py::arg("dtheta_meas_rad"),
        "Run theta PID and return alpha_ref_cmd [rad]");

  m.def("update_alpha", &furuta_pid_SIL::update_alpha,
        py::arg("alpha_ref_cmd"), py::arg("alpha_meas_rad"),
        py::arg("dalpha_meas_rad"), "Run alpha PID and return motor voltage [V]");

  m.def("compute_control", &furuta_pid_SIL::compute_control,
        py::arg("theta_meas_rad"), py::arg("dtheta_meas_rad"),
        py::arg("alpha_meas_rad"), py::arg("dalpha_meas_rad"),
        "Run full two-stage controller and return motor voltage [V]");

  // Parameters
  m.def("set_Ts", &furuta_pid_SIL::set_Ts);
  m.def("get_Ts", &furuta_pid_SIL::get_Ts);

  m.def("set_theta_ref_rad", &furuta_pid_SIL::set_theta_ref_rad);
  m.def("get_theta_ref_rad", &furuta_pid_SIL::get_theta_ref_rad);

  m.def("set_alpha_ref_rad", &furuta_pid_SIL::set_alpha_ref_rad);
  m.def("get_alpha_ref_rad", &furuta_pid_SIL::get_alpha_ref_rad);

  m.def("set_v_limit", &furuta_pid_SIL::set_v_limit);
  m.def("get_v_limit", &furuta_pid_SIL::get_v_limit);

  m.def("set_alpha_ref_limit_rad", &furuta_pid_SIL::set_alpha_ref_limit_rad);
  m.def("get_alpha_ref_limit_rad", &furuta_pid_SIL::get_alpha_ref_limit_rad);

  m.def("set_theta_to_alpha_sign", &furuta_pid_SIL::set_theta_to_alpha_sign);
  m.def("get_theta_to_alpha_sign", &furuta_pid_SIL::get_theta_to_alpha_sign);

  // Gains
  m.def("set_theta_gains", &furuta_pid_SIL::set_theta_gains,
        py::arg("kp"), py::arg("ki"), py::arg("kd"));
  m.def("get_theta_gains", &furuta_pid_SIL::get_theta_gains,
        py::arg("kp"), py::arg("ki"), py::arg("kd"));

  m.def("set_alpha_gains", &furuta_pid_SIL::set_alpha_gains,
        py::arg("kp"), py::arg("ki"), py::arg("kd"));
  m.def("get_alpha_gains", &furuta_pid_SIL::get_alpha_gains,
        py::arg("kp"), py::arg("ki"), py::arg("kd"));

  // Internal state inspection
  m.def("get_int_theta", &furuta_pid_SIL::get_int_theta);
  m.def("get_int_alpha", &furuta_pid_SIL::get_int_alpha);
  m.def("get_dtheta_filt", &furuta_pid_SIL::get_dtheta_filt);
  m.def("get_dalpha_filt", &furuta_pid_SIL::get_dalpha_filt);
}
