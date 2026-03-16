#include <pybind11/pybind11.h>

#include "furuta_pendulum_pid_controller.hpp"

namespace furuta_pendulum_pid_controller_SIL {

namespace py = pybind11;

static FurutaPendulum_PID_Controller ctrl;

void initialize() { ctrl = FurutaPendulum_PID_Controller(); }

// Re-create controller with constructor arguments
void configure(FurutaPendulum_PID_Controller::FLOAT Ts,
               FurutaPendulum_PID_Controller::FLOAT theta_ref_rad,
               FurutaPendulum_PID_Controller::FLOAT alpha_ref_rad,
               FurutaPendulum_PID_Controller::FLOAT v_limit,
               FurutaPendulum_PID_Controller::FLOAT alpha_ref_limit_rad,
               FurutaPendulum_PID_Controller::FLOAT theta_to_alpha_sign) {
  ctrl = FurutaPendulum_PID_Controller(Ts, theta_ref_rad, alpha_ref_rad, v_limit,
                                       alpha_ref_limit_rad, theta_to_alpha_sign);
}

// Methods mirroring the C++ class
void reset_integrators() { ctrl.reset_integrators(); }

void set_dalpha_tau(FurutaPendulum_PID_Controller::FLOAT tau) { ctrl.set_dalpha_tau(tau); }

void set_dtheta_tau(FurutaPendulum_PID_Controller::FLOAT tau) { ctrl.set_dtheta_tau(tau); }

// Accessors for internal state
FurutaPendulum_PID_Controller::FLOAT int_theta() { return ctrl.int_theta(); }
FurutaPendulum_PID_Controller::FLOAT int_alpha() { return ctrl.int_alpha(); }
FurutaPendulum_PID_Controller::FLOAT dalpha_filt() { return ctrl.dalpha_filt(); }
FurutaPendulum_PID_Controller::FLOAT dtheta_filt() { return ctrl.dtheta_filt(); }
FurutaPendulum_PID_Controller::FLOAT dalpha_tau() { return ctrl.dalpha_tau(); }
FurutaPendulum_PID_Controller::FLOAT dtheta_tau() { return ctrl.dtheta_tau(); }

// Getters and setters for public configuration/state
FurutaPendulum_PID_Controller::FLOAT get_Ts() { return ctrl.Ts; }
void set_Ts(FurutaPendulum_PID_Controller::FLOAT v) { ctrl.Ts = v < FurutaPendulum_PID_Controller::TS_MIN ? FurutaPendulum_PID_Controller::TS_MIN : v; }

FurutaPendulum_PID_Controller::FLOAT get_theta_ref_rad() { return ctrl.theta_ref_rad; }
void set_theta_ref_rad(FurutaPendulum_PID_Controller::FLOAT v) { ctrl.theta_ref_rad = v; }

FurutaPendulum_PID_Controller::FLOAT get_alpha_ref_rad() { return ctrl.alpha_ref_rad; }
void set_alpha_ref_rad(FurutaPendulum_PID_Controller::FLOAT v) { ctrl.alpha_ref_rad = v; }

FurutaPendulum_PID_Controller::FLOAT get_v_limit() { return ctrl.v_limit; }
void set_v_limit(FurutaPendulum_PID_Controller::FLOAT v) { ctrl.v_limit = std::abs(v); }

FurutaPendulum_PID_Controller::FLOAT get_alpha_ref_limit_rad() { return ctrl.alpha_ref_limit_rad; }
void set_alpha_ref_limit_rad(FurutaPendulum_PID_Controller::FLOAT v) { ctrl.alpha_ref_limit_rad = std::abs(v); }

FurutaPendulum_PID_Controller::FLOAT get_theta_to_alpha_sign() { return ctrl.theta_to_alpha_sign; }
void set_theta_to_alpha_sign(FurutaPendulum_PID_Controller::FLOAT v) { ctrl.theta_to_alpha_sign = v >= 0.0 ? 1.0 : -1.0; }

// PID gains
FurutaPendulum_PID_Controller::FLOAT get_kp_theta() { return ctrl.kp_theta; }
void set_kp_theta(FurutaPendulum_PID_Controller::FLOAT v) { ctrl.kp_theta = v; }
FurutaPendulum_PID_Controller::FLOAT get_ki_theta() { return ctrl.ki_theta; }
void set_ki_theta(FurutaPendulum_PID_Controller::FLOAT v) { ctrl.ki_theta = v; }
FurutaPendulum_PID_Controller::FLOAT get_kd_theta() { return ctrl.kd_theta; }
void set_kd_theta(FurutaPendulum_PID_Controller::FLOAT v) { ctrl.kd_theta = v; }

FurutaPendulum_PID_Controller::FLOAT get_kp_alpha() { return ctrl.kp_alpha; }
void set_kp_alpha(FurutaPendulum_PID_Controller::FLOAT v) { ctrl.kp_alpha = v; }
FurutaPendulum_PID_Controller::FLOAT get_ki_alpha() { return ctrl.ki_alpha; }
void set_ki_alpha(FurutaPendulum_PID_Controller::FLOAT v) { ctrl.ki_alpha = v; }
FurutaPendulum_PID_Controller::FLOAT get_kd_alpha() { return ctrl.kd_alpha; }
void set_kd_alpha(FurutaPendulum_PID_Controller::FLOAT v) { ctrl.kd_alpha = v; }

PYBIND11_MODULE(FurutaPendulumPIDControllerSIL, m) {
  m.doc() = "SIL bindings for FurutaPendulum_PID_Controller";

  m.def("initialize", &initialize, "Initialize the controller instance (default ctor)");
  m.def("configure", &configure, "Configure controller by constructor args",
        py::arg("Ts"), py::arg("theta_ref_rad"), py::arg("alpha_ref_rad"), py::arg("v_limit"),
        py::arg("alpha_ref_limit_rad"), py::arg("theta_to_alpha_sign"));

  m.def("reset_integrators", &reset_integrators, "Reset internal integrators to zero");
  m.def("set_dalpha_tau", &set_dalpha_tau, "Set dalpha filter time constant", py::arg("tau"));
  m.def("set_dtheta_tau", &set_dtheta_tau, "Set dtheta filter time constant", py::arg("tau"));

  m.def("int_theta", &int_theta, "Get internal theta integrator state");
  m.def("int_alpha", &int_alpha, "Get internal alpha integrator state");
  m.def("dalpha_filt", &dalpha_filt, "Get filtered dalpha measurement");
  m.def("dtheta_filt", &dtheta_filt, "Get filtered dtheta measurement");
  m.def("dalpha_tau", &dalpha_tau, "Get dalpha filter time constant");
  m.def("dtheta_tau", &dtheta_tau, "Get dtheta filter time constant");

  // Config/state accessors
  m.def("get_Ts", &get_Ts);
  m.def("set_Ts", &set_Ts, py::arg("Ts"));
  m.def("get_theta_ref_rad", &get_theta_ref_rad);
  m.def("set_theta_ref_rad", &set_theta_ref_rad, py::arg("val"));
  m.def("get_alpha_ref_rad", &get_alpha_ref_rad);
  m.def("set_alpha_ref_rad", &set_alpha_ref_rad, py::arg("val"));
  m.def("get_v_limit", &get_v_limit);
  m.def("set_v_limit", &set_v_limit, py::arg("val"));
  m.def("get_alpha_ref_limit_rad", &get_alpha_ref_limit_rad);
  m.def("set_alpha_ref_limit_rad", &set_alpha_ref_limit_rad, py::arg("val"));
  m.def("get_theta_to_alpha_sign", &get_theta_to_alpha_sign);
  m.def("set_theta_to_alpha_sign", &set_theta_to_alpha_sign, py::arg("val"));

  // Gains
  m.def("get_kp_theta", &get_kp_theta);
  m.def("set_kp_theta", &set_kp_theta, py::arg("val"));
  m.def("get_ki_theta", &get_ki_theta);
  m.def("set_ki_theta", &set_ki_theta, py::arg("val"));
  m.def("get_kd_theta", &get_kd_theta);
  m.def("set_kd_theta", &set_kd_theta, py::arg("val"));

  m.def("get_kp_alpha", &get_kp_alpha);
  m.def("set_kp_alpha", &set_kp_alpha, py::arg("val"));
  m.def("get_ki_alpha", &get_ki_alpha);
  m.def("set_ki_alpha", &set_ki_alpha, py::arg("val"));
  m.def("get_kd_alpha", &get_kd_alpha);
  m.def("set_kd_alpha", &set_kd_alpha, py::arg("val"));
}

} // namespace furuta_pendulum_pid_controller_SIL
