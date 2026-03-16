#include <pybind11/pybind11.h>

#include "furuta_pendulum_pid_controller.hpp"

namespace furuta_pendulum_pid_controller_SIL {

namespace py = pybind11;

static FurutaPendulum_PID_Controller controller;

void initialize(void) { controller = FurutaPendulum_PID_Controller(); }

PYBIND11_MODULE(FurutaPendulumPIDControllerSIL, m) {
  m.doc() = "SIL wrapper for FurutaPendulum_PID_Controller";
  m.def("initialize", &initialize, "Initialize the module");
}

} // namespace furuta_pendulum_pid_controller_SIL
