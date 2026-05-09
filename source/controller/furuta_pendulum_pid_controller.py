from __future__ import annotations

import os
import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parents[1]))
sys.path.append(str(Path(__file__).resolve().parents[2]))


class FurutaPendulum_PID_Controller:

    def __init__(
        self
    ) -> None:

        pass

    def calculate_manipulation(
        self,
        theta: float,
        alpha: float,
        dtheta: float,
        dalpha: float,
    ) -> float:

        return 0
