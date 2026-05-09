#include <pybind11/pybind11.h>

#include "furuta_pendulum_pid_controller.hpp"

namespace furuta_pendulum_pid_controller_SIL {

namespace py = pybind11;

FurutaPendulum_PID_Controller controller;

void initialize(void) { controller = FurutaPendulum_PID_Controller(); }

// Class: FurutaPendulum_PID_Controller
// Method: calculate_manipulation
float calculate_manipulation(float theta, float alpha, float dtheta, float dalpha) {
  return controller.calculate_manipulation(theta, alpha, dtheta, dalpha);
}

PYBIND11_MODULE(FurutaPendulum_PID_ControllerSIL, m) {
  m.def("initialize", &initialize, "Initialize the module");
  m.def("calculate_manipulation", &calculate_manipulation, "calculate_manipulation method");
}

} // namespace furuta_pendulum_pid_controller_SIL
