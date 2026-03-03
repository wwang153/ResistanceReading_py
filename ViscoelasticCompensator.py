import numpy as np
import math


class Compensator:

    def __init__(self, mode, p0, p1, q0, q1, data_rate):

        self.mode = mode.lower()

        self.p0 = p0
        self.p1 = p1
        self.q0 = q0
        self.q1 = q1

        self.Ts = 1.0 / data_rate

        # runtime state
        self.t = 0.0
        self.R0 = None
        self.Lprev = None
        self.initialized = False

        # ---- NEW: R0 averaging ----
        self.R0_buffer = []
        self.R0_sample_count = 500

    # --------------------------------------------------
    # MATLAB-style G(t)
    # --------------------------------------------------
    def G(self, t):
        return (self.q0/self.p0 +
                (self.q1/self.p1 - self.q0/self.p0) *
                np.exp(-self.p0/self.p1 * t))

    # --------------------------------------------------
    # MATLAB-style J(t)
    # --------------------------------------------------
    def J(self, t):
        return (self.p0/self.q0 +
                (self.p1/self.q1 - self.p0/self.q0) *
                np.exp(-self.q0/self.q1 * t))

    # --------------------------------------------------
    # Main streaming update
    # --------------------------------------------------
    def update(self, R_measured):

        # handle invalid reading
        if R_measured is None or math.isnan(R_measured):
            return math.nan

        # first valid sample
        self.R0_buffer.append(R_measured)

        if len(self.R0_buffer) < self.R0_sample_count:
            return 0.0  # still collecting baseline

        # Compute average R0
        self.R0 = sum(self.R0_buffer) / len(self.R0_buffer)

        dR = R_measured - self.R0

        self.t += self.Ts

        # ---------------------------
        # First iteration (S1)
        # ---------------------------
        if self.Lprev is None:

            if self.mode == "creep":
                S1 = np.exp(self.p0/self.p1 * self.t) * dR * self.Ts
                G0 = self.G(0)
            else:
                S1 = np.exp(self.q0/self.q1 * self.t) * dR * self.Ts
                J0 = self.J(0)

            self.Lprev = S1
            return dR

        # ---------------------------
        # Recursive Li update
        # ---------------------------
        if self.mode == "creep":

            Li = self.Lprev + \
                 np.exp(self.p0/self.p1 * self.t) * dR * self.Ts


            Value = dR * G0 - \
                    (self.q1/self.p1 - self.q0/self.p0) * \
                    (self.p0/self.p1) * \
                    np.exp(-self.p0/self.p1 * self.t) * Li

        else:  # relaxation

            Li = self.Lprev + \
                 np.exp(self.q0/self.q1 * self.t) * dR * self.Ts

            Value = dR * J0 - \
                    (self.p1/self.q1 - self.p0/self.q0) * \
                    (self.q0/self.q1) * \
                    np.exp(-self.q0/self.q1 * self.t) * Li

        self.Lprev = Li

        return Value

    # --------------------------------------------------
    # Optional reset
    # --------------------------------------------------
    def reset(self):
        self.t = 0.0
        self.R0 = None
        self.Lprev = None
        self.initialized = False