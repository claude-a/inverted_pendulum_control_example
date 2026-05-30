#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>

#include "furuta_pendulum_pid_controller.hpp"

namespace furuta_pendulum_pid_controller_SIL {

namespace py = pybind11;

FurutaPendulum_PID_Controller controller;

void initialize(void) { controller = FurutaPendulum_PID_Controller(); }

// Class: FurutaPendulum_PID_Controller
// Method: reset
void reset() {
  controller.reset();
}

// Class: FurutaPendulum_PID_Controller
// Method: set_theta_reference_rad
void set_theta_reference_rad(float theta_ref_rad) {
  controller.set_theta_reference_rad(theta_ref_rad);
}

// Class: FurutaPendulum_PID_Controller
// Method: set_theta_reference_deg
void set_theta_reference_deg(float theta_ref_deg) {
  controller.set_theta_reference_deg(theta_ref_deg);
}

// Class: FurutaPendulum_PID_Controller
// Method: set_alpha_reference_rad
void set_alpha_reference_rad(float alpha_ref_rad) {
  controller.set_alpha_reference_rad(alpha_ref_rad);
}

// Class: FurutaPendulum_PID_Controller
// Method: set_alpha_reference_deg
void set_alpha_reference_deg(float alpha_ref_deg) {
  controller.set_alpha_reference_deg(alpha_ref_deg);
}

// Class: FurutaPendulum_PID_Controller
// Method: calculate_manipulation
float calculate_manipulation(float theta, float alpha, float dtheta,
                            float dalpha) {
  return controller.calculate_manipulation(theta, alpha, dtheta, dalpha);
}

PYBIND11_MODULE(FurutaPendulumPIDControllerSIL, m) {
  m.def("initialize", &initialize, "Initialize the module");
  m.def("reset", &reset, "reset method");
  m.def("set_theta_reference_rad", &set_theta_reference_rad, "set_theta_reference_rad method");
  m.def("set_theta_reference_deg", &set_theta_reference_deg, "set_theta_reference_deg method");
  m.def("set_alpha_reference_rad", &set_alpha_reference_rad, "set_alpha_reference_rad method");
  m.def("set_alpha_reference_deg", &set_alpha_reference_deg, "set_alpha_reference_deg method");
  m.def("calculate_manipulation", &calculate_manipulation,
        "calculate_manipulation method");
}

} // namespace furuta_pendulum_pid_controller_SIL
