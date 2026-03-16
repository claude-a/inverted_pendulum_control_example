#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "furuta_pendulum_pid_controller.hpp"

namespace furuta_pendulum_pid_controller_SIL {

namespace py = pybind11;

static FurutaPendulum_PID_Controller ctrl;

void initialize() { ctrl = FurutaPendulum_PID_Controller(); }

void construct(double Ts,
               double theta_ref_rad,
               double alpha_ref_rad,
               double v_limit,
               double alpha_ref_limit_rad,
               double theta_to_alpha_sign) {
  ctrl = FurutaPendulum_PID_Controller(Ts,
                                       theta_ref_rad,
                                       alpha_ref_rad,
                                       v_limit,
                                       alpha_ref_limit_rad,
                                       theta_to_alpha_sign);
}

// References
void set_references(double theta_ref_rad, double alpha_ref_rad) {
  ctrl.theta_ref_rad = theta_ref_rad;
  ctrl.alpha_ref_rad = alpha_ref_rad;
}

py::dict get_references() {
  py::dict d;
  d["theta_ref_rad"] = ctrl.theta_ref_rad;
  d["alpha_ref_rad"] = ctrl.alpha_ref_rad;
  return d;
}

// Limits
void set_limits(double v_limit, double alpha_ref_limit_rad) {
  ctrl.v_limit = std::fabs(v_limit);
  ctrl.alpha_ref_limit_rad = std::fabs(alpha_ref_limit_rad);
}

py::dict get_limits() {
  py::dict d;
  d["v_limit"] = ctrl.v_limit;
  d["alpha_ref_limit_rad"] = ctrl.alpha_ref_limit_rad;
  return d;
}

// Gains
void set_gains(double kp_theta, double ki_theta, double kd_theta,
               double kp_alpha, double ki_alpha, double kd_alpha) {
  ctrl.kp_theta = kp_theta;
  ctrl.ki_theta = ki_theta;
  ctrl.kd_theta = kd_theta;
  ctrl.kp_alpha = kp_alpha;
  ctrl.ki_alpha = ki_alpha;
  ctrl.kd_alpha = kd_alpha;
}

py::dict get_gains() {
  py::dict d;
  d["kp_theta"] = ctrl.kp_theta;
  d["ki_theta"] = ctrl.ki_theta;
  d["kd_theta"] = ctrl.kd_theta;
  d["kp_alpha"] = ctrl.kp_alpha;
  d["ki_alpha"] = ctrl.ki_alpha;
  d["kd_alpha"] = ctrl.kd_alpha;
  return d;
}

// Internal states (integrators, derivative filters)
void set_state(double int_theta,
               double int_alpha,
               double dalpha_filt,
               double dtheta_filt) {
  ctrl._int_theta = int_theta;
  ctrl._int_alpha = int_alpha;
  ctrl._dalpha_filt = dalpha_filt;
  ctrl._dtheta_filt = dtheta_filt;
}

py::dict get_state() {
  py::dict d;
  d["_int_theta"] = ctrl._int_theta;
  d["_int_alpha"] = ctrl._int_alpha;
  d["_dalpha_filt"] = ctrl._dalpha_filt;
  d["_dtheta_filt"] = ctrl._dtheta_filt;
  d["_dalpha_tau"] = ctrl._dalpha_tau;
  d["_dtheta_tau"] = ctrl._dtheta_tau;
  return d;
}

// Expose a convenience reset for integrators and filters
void reset_states() {
  ctrl._int_theta = 0.0;
  ctrl._int_alpha = 0.0;
  ctrl._dalpha_filt = 0.0;
  ctrl._dtheta_filt = 0.0;
}

PYBIND11_MODULE(FurutaPendulumPIDControllerSIL, m) {
  m.doc() = "SIL wrapper for FurutaPendulum_PID_Controller";

  m.def("initialize", &initialize, "Initialize internal controller to defaults");
  m.def("construct", &construct, "Construct controller with parameters",
        py::arg("Ts") = FurutaPendulum_PID_Controller::TS_DEFAULT,
        py::arg("theta_ref_rad") = 0.0,
        py::arg("alpha_ref_rad") = 0.0,
        py::arg("v_limit") = FurutaPendulum_PID_Controller::V_LIMIT_DEFAULT,
        py::arg("alpha_ref_limit_rad") = FurutaPendulum_PID_Controller::ALPHA_REF_LIMIT_RAD_DEFAULT,
        py::arg("theta_to_alpha_sign") = FurutaPendulum_PID_Controller::THETA_TO_ALPHA_SIGN_DEFAULT);

  m.def("set_references", &set_references, "Set reference angles",
        py::arg("theta_ref_rad"), py::arg("alpha_ref_rad"));
  m.def("get_references", &get_references, "Get reference angles");

  m.def("set_limits", &set_limits, "Set saturation limits",
        py::arg("v_limit"), py::arg("alpha_ref_limit_rad"));
  m.def("get_limits", &get_limits, "Get saturation limits");

  m.def("set_gains", &set_gains, "Set PID gains",
        py::arg("kp_theta"), py::arg("ki_theta"), py::arg("kd_theta"),
        py::arg("kp_alpha"), py::arg("ki_alpha"), py::arg("kd_alpha"));
  m.def("get_gains", &get_gains, "Get PID gains");

  m.def("set_state", &set_state, "Set internal integrator and filter states",
        py::arg("int_theta"), py::arg("int_alpha"), py::arg("dalpha_filt"), py::arg("dtheta_filt"));
  m.def("get_state", &get_state, "Get internal integrator and filter states");
  m.def("reset_states", &reset_states, "Reset integrators and filters to zero");
}

} // namespace furuta_pendulum_pid_controller_SIL