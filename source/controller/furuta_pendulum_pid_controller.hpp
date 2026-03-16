#ifndef FURUTA_PENDULUM_PID_CONTROLLER_HPP
#define FURUTA_PENDULUM_PID_CONTROLLER_HPP

#include <cmath>

class FurutaPendulum_PID_Controller {
public:
  typedef double FLOAT;

  // Defaults (mirroring Python constants)
  static constexpr FLOAT TS_DEFAULT = 0.005;
  static constexpr FLOAT TS_MIN = 1e-6;
  static constexpr FLOAT V_LIMIT_DEFAULT = 12.0;
  static constexpr FLOAT ALPHA_REF_LIMIT_RAD_DEFAULT = 12.0 * 3.14159265358979323846 / 180.0; // 12 deg in rad
  static constexpr FLOAT THETA_TO_ALPHA_SIGN_DEFAULT = 1.0;
  static constexpr FLOAT KP_THETA_DEFAULT = 0.1;
  static constexpr FLOAT KI_THETA_DEFAULT = 0.0;
  static constexpr FLOAT KD_THETA_DEFAULT = 0.05;
  static constexpr FLOAT KP_ALPHA_DEFAULT = 100.0;
  static constexpr FLOAT KI_ALPHA_DEFAULT = 0.0;
  static constexpr FLOAT KD_ALPHA_DEFAULT = 10.0;
  static constexpr FLOAT DTHETA_TAU_DEFAULT = 0.02;
  static constexpr FLOAT DALPHA_TAU_DEFAULT = 0.02;
  static constexpr FLOAT EPS_VALUE = 1e-12;

  // Constructor mirrors the Python __init__ signature. Use negative alpha_ref_limit_rad to indicate "None".
  FurutaPendulum_PID_Controller(FLOAT Ts = TS_DEFAULT,
                                FLOAT theta_ref_rad = 0.0,
                                FLOAT alpha_ref_rad = 0.0,
                                FLOAT v_limit = V_LIMIT_DEFAULT,
                                FLOAT alpha_ref_limit_rad = -1.0,
                                FLOAT theta_to_alpha_sign = THETA_TO_ALPHA_SIGN_DEFAULT);

  // Defaulted special members
  ~FurutaPendulum_PID_Controller() = default;
  FurutaPendulum_PID_Controller(const FurutaPendulum_PID_Controller&) = default;
  FurutaPendulum_PID_Controller& operator=(const FurutaPendulum_PID_Controller&) = default;
  FurutaPendulum_PID_Controller(FurutaPendulum_PID_Controller&&) = default;
  FurutaPendulum_PID_Controller& operator=(FurutaPendulum_PID_Controller&&) = default;

  // Public configuration/state (mirrors Python attributes)
  FLOAT Ts;
  FLOAT theta_ref_rad;
  FLOAT alpha_ref_rad;
  FLOAT v_limit;
  FLOAT alpha_ref_limit_rad;
  FLOAT theta_to_alpha_sign;

  // PID tuning gains
  FLOAT kp_theta;
  FLOAT ki_theta;
  FLOAT kd_theta;

  FLOAT kp_alpha;
  FLOAT ki_alpha;
  FLOAT kd_alpha;

  // Methods to manage internal state and configuration
  void reset_integrators();
  void set_dalpha_tau(FLOAT tau);
  void set_dtheta_tau(FLOAT tau);

  // Accessors for internal states (read-only)
  FLOAT int_theta() const;
  FLOAT int_alpha() const;
  FLOAT dalpha_filt() const;
  FLOAT dtheta_filt() const;
  FLOAT dalpha_tau() const;
  FLOAT dtheta_tau() const;

private:
  // Internal states (mirror Python "private" attributes)
  FLOAT _int_theta;
  FLOAT _int_alpha;
  FLOAT _dalpha_filt;
  FLOAT _dalpha_tau;
  FLOAT _dtheta_filt;
  FLOAT _dtheta_tau;
};

#endif // FURUTA_PENDULUM_PID_CONTROLLER_HPP
