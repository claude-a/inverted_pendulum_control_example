#ifndef FURUTA_PENDULUM_PID_CONTROLLER_HPP
#define FURUTA_PENDULUM_PID_CONTROLLER_HPP

#include <cmath>
#include <algorithm>

const float TS_DEFAULT = 0.005f;
const float TS_MIN = 1e-6f;
const float V_LIMIT_DEFAULT = 12.0f;
const float ALPHA_REF_LIMIT_RAD_DEFAULT = 12.0f * M_PI / 180.0f;
const float THETA_TO_ALPHA_SIGN_DEFAULT = 1.0f;
const float KP_THETA_DEFAULT = 0.1f;
const float KI_THETA_DEFAULT = 0.0f;
const float KD_THETA_DEFAULT = 0.05f;
const float KP_ALPHA_DEFAULT = 100.0f;
const float KI_ALPHA_DEFAULT = 0.0f;
const float KD_ALPHA_DEFAULT = 10.0f;
const float DTHETA_TAU_DEFAULT = 0.02f;
const float DALPHA_TAU_DEFAULT = 0.02f;
const float EPS_VALUE = 1e-12f;

inline float _wrap_to_pi(float angle_rad) {
    return fmodf(angle_rad + M_PI, 2.0f * M_PI) - M_PI;
}

class FurutaPendulum_PID_Controller {
private:
    float Ts;
    float theta_ref_rad;
    float alpha_ref_rad;
    float v_limit;
    float alpha_ref_limit_rad;
    float theta_to_alpha_sign;
    
    float kp_theta;
    float ki_theta;
    float kd_theta;
    
    float kp_alpha;
    float ki_alpha;
    float kd_alpha;
    
    float _int_theta;
    float _int_alpha;
    
    float _dalpha_filt;
    float _dalpha_tau;
    float _dtheta_filt;
    float _dtheta_tau;
    
    static bool _should_integrate_u(float u_cmd, float u_cmd_sat, float err, float u_lim) {
        if (u_cmd == u_cmd_sat) {
            return true;
        }
        if (u_cmd_sat >= u_lim - EPS_VALUE && err > 0.0f) {
            return false;
        }
        if (u_cmd_sat <= -u_lim + EPS_VALUE && err < 0.0f) {
            return false;
        }
        return true;
    }

public:
    FurutaPendulum_PID_Controller(
        float Ts = TS_DEFAULT,
        float theta_ref_rad = 0.0f,
        float alpha_ref_rad = 0.0f,
        float v_limit = V_LIMIT_DEFAULT,
        float alpha_ref_limit_rad = ALPHA_REF_LIMIT_RAD_DEFAULT,
        float theta_to_alpha_sign = THETA_TO_ALPHA_SIGN_DEFAULT
    ) {
        this->Ts = Ts;
        this->theta_ref_rad = theta_ref_rad;
        this->alpha_ref_rad = alpha_ref_rad;
        this->v_limit = fabsf(v_limit);
        this->alpha_ref_limit_rad = fabsf(alpha_ref_limit_rad);
        this->theta_to_alpha_sign = (theta_to_alpha_sign >= 0.0f) ? 1.0f : -1.0f;
        
        this->kp_theta = KP_THETA_DEFAULT;
        this->ki_theta = KI_THETA_DEFAULT;
        this->kd_theta = KD_THETA_DEFAULT;
        
        this->kp_alpha = KP_ALPHA_DEFAULT;
        this->ki_alpha = KI_ALPHA_DEFAULT;
        this->kd_alpha = KD_ALPHA_DEFAULT;
        
        this->_int_theta = 0.0f;
        this->_int_alpha = 0.0f;
        
        this->_dalpha_filt = 0.0f;
        this->_dalpha_tau = DALPHA_TAU_DEFAULT;
        this->_dtheta_filt = 0.0f;
        this->_dtheta_tau = DTHETA_TAU_DEFAULT;
    }
    
    void reset() {
        _int_theta = 0.0f;
        _int_alpha = 0.0f;
        _dalpha_filt = 0.0f;
        _dtheta_filt = 0.0f;
    }
    
    void set_theta_reference_rad(float theta_ref_rad) {
        this->theta_ref_rad = theta_ref_rad;
    }
    
    void set_theta_reference_deg(float theta_ref_deg) {
        this->theta_ref_rad = theta_ref_deg * M_PI / 180.0f;
    }
    
    void set_alpha_reference_rad(float alpha_ref_rad) {
        this->alpha_ref_rad = alpha_ref_rad;
    }
    
    void set_alpha_reference_deg(float alpha_ref_deg) {
        this->alpha_ref_rad = alpha_ref_deg * M_PI / 180.0f;
    }

    float calculate_manipulation(
        float theta,
        float alpha,
        float dtheta,
        float dalpha
    ) {
        float Ts_local = Ts;
        if (Ts_local <= TS_MIN) {
            Ts_local = TS_MIN;
        }
        
        float theta_meas = theta;
        float alpha_meas = alpha;
        float dtheta_meas = dtheta;
        float dalpha_meas = dalpha;
        
        float e_theta = _wrap_to_pi(theta_meas - theta_ref_rad);
        
        float dtheta_used;
        float tau_th = _dtheta_tau;
        if (tau_th > 0.0f) {
            float a = tau_th / (tau_th + Ts_local);
            float b = Ts_local / (tau_th + Ts_local);
            _dtheta_filt = a * _dtheta_filt + b * dtheta_meas;
            dtheta_used = _dtheta_filt;
        } else {
            dtheta_used = dtheta_meas;
        }
        
        float dalpha_used;
        float tau_al = _dalpha_tau;
        if (tau_al > 0.0f) {
            float a = tau_al / (tau_al + Ts_local);
            float b = Ts_local / (tau_al + Ts_local);
            _dalpha_filt = a * _dalpha_filt + b * dalpha_meas;
            dalpha_used = _dalpha_filt;
        } else {
            dalpha_used = dalpha_meas;
        }
        
        float alpha_offset_cmd = theta_to_alpha_sign * (
            kp_theta * e_theta
            + ki_theta * _int_theta
            + kd_theta * dtheta_used
        );
        
        float alpha_ref_cmd = _wrap_to_pi(alpha_ref_rad + alpha_offset_cmd);
        float alpha_ref_cmd_sat = std::max(
            -alpha_ref_limit_rad,
            std::min(alpha_ref_limit_rad, alpha_ref_cmd)
        );
        
        float e_alpha = _wrap_to_pi(alpha_ref_cmd_sat - alpha_meas);
        
        float v_unsat = (kp_alpha * e_alpha)
            + (ki_alpha * _int_alpha)
            - (kd_alpha * dalpha_used);
        
        float v_sat = std::max(-v_limit, std::min(v_limit, v_unsat));
        
        if (_should_integrate_u(alpha_ref_cmd, alpha_ref_cmd_sat, e_theta, alpha_ref_limit_rad)) {
            _int_theta += e_theta * Ts_local;
        }
        
        if (_should_integrate_u(v_unsat, v_sat, e_alpha, v_limit)) {
            _int_alpha += e_alpha * Ts_local;
        }
        
        return v_sat;
    }
};

#endif // FURUTA_PENDULUM_PID_CONTROLLER_HPP
