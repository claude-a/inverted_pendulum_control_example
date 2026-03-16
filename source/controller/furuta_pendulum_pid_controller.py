from __future__ import annotations

import os
import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parents[1]))
sys.path.append(str(Path(__file__).resolve().parents[2]))

import math

TS_DEFAULT = 0.005
TS_MIN = 1e-6

V_LIMIT_DEFAULT = 12.0
ALPHA_REF_LIMIT_RAD_DEFAULT = 12.0 * math.pi / 180.0
THETA_TO_ALPHA_SIGN_DEFAULT = 1.0
KP_THETA_DEFAULT = 0.1
KI_THETA_DEFAULT = 0.0
KD_THETA_DEFAULT = 0.05
KP_ALPHA_DEFAULT = 100.0
KI_ALPHA_DEFAULT = 0.0
KD_ALPHA_DEFAULT = 10.0
DTHETA_TAU_DEFAULT = 0.02
DALPHA_TAU_DEFAULT = 0.02
EPS_VALUE = 1e-12


class FurutaPendulum_PID_Controller:
    def __init__(
        self,
        Ts: float = TS_DEFAULT,
        theta_ref_rad: float = 0.0,
        alpha_ref_rad: float = 0.0,
        v_limit: float = V_LIMIT_DEFAULT,
        alpha_ref_limit_rad: float | None = None,
        theta_to_alpha_sign: float = THETA_TO_ALPHA_SIGN_DEFAULT,
    ) -> None:
        self.Ts = float(Ts)

        # References (can be changed later)
        self.theta_ref_rad = float(theta_ref_rad)
        self.alpha_ref_rad = float(alpha_ref_rad)

        # Voltage saturation used for anti-windup
        self.v_limit = float(abs(v_limit))

        # Limit for commanded pendulum tilt generated from theta control.
        # Keeping this small is important: this controller intentionally uses
        # a *small* pendulum lean to create restoring motion.
        if alpha_ref_limit_rad is None:
            alpha_ref_limit_rad = ALPHA_REF_LIMIT_RAD_DEFAULT
        self.alpha_ref_limit_rad = float(abs(alpha_ref_limit_rad))

        # Mapping sign: if theta is positive, we want alpha_ref_cmd to be positive.
        # If your plant uses opposite sign conventions, set this to -1.0.
        self.theta_to_alpha_sign = 1.0 if theta_to_alpha_sign >= 0.0 else -1.0

        # === Tuning gains (start conservative; tune as needed) ===
        # Theta PID output is interpreted as a commanded pendulum angle offset [rad].
        # (theta > 0) -> (alpha_ref_cmd > 0) by construction.
        self.kp_theta = KP_THETA_DEFAULT
        self.ki_theta = KI_THETA_DEFAULT
        self.kd_theta = KD_THETA_DEFAULT

        # Pendulum PID outputs motor voltage [V] to track alpha_ref_cmd.
        self.kp_alpha = KP_ALPHA_DEFAULT
        self.ki_alpha = KI_ALPHA_DEFAULT
        self.kd_alpha = KD_ALPHA_DEFAULT

        # Integrator states
        self._int_theta = 0.0
        self._int_alpha = 0.0

        # Simple derivative filtering (on measurement)
        self._dalpha_filt = 0.0
        self._dalpha_tau = DALPHA_TAU_DEFAULT  # [s]
        self._dtheta_filt = 0.0
        self._dtheta_tau = DTHETA_TAU_DEFAULT  # [s]
