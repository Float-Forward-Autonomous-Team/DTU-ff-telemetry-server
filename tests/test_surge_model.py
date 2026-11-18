"""src/surge_model.py on synthetic days with known parameters, and its use in the strategy engine.

    python -m unittest tests.test_surge_model
"""
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

import surge_model as sm  # noqa: E402
from strategy_engine import surge_eta_grid  # noqa: E402


def synthetic_day(a=180.0, b=1.8, c_turn=4000.0, noise_w=0.0, seed=0):
    """A day on the 1 s grid: power steps held for 60-120 s, a turn of 0.25 rad/s every 40 s,
    speed in steady state of P = a u^b + c r^2 u."""
    rng = np.random.default_rng(seed)
    levels = rng.uniform(1500, 4500, 40)
    P = np.concatenate([np.full(int(rng.integers(60, 121)), p) for p in levels])
    n = len(P)
    t = np.arange(n, dtype=float)
    r = np.where((t % 40) < 6, 0.25, 0.0)
    u = np.empty(n)
    for i in range(n):  # steady-state speed for this power and yaw rate (bisection)
        lo, hi = 0.5, 15.0
        for _ in range(60):
            mid = (lo + hi) / 2
            lo, hi = (mid, hi) if a * mid ** b + c_turn * r[i] ** 2 * mid < P[i] else (lo, mid)
        u[i] = (lo + hi) / 2
    return {"t": t, "name": "synthetic", "yaw_source": "synthetic",
            "motor_power": P + rng.normal(0, noise_w, n) if noise_w else P, "gps_speed": u, "r": r}


class TestFit(unittest.TestCase):
    def test_recovers_free_exponent_law(self):
        d = synthetic_day()
        fit = sm.fit_power_law(sm.plateau_table(d, sm.plateaus(d)))
        self.assertAlmostEqual(fit["free"]["b"], 1.8, delta=0.02)
        self.assertAlmostEqual(fit["free"]["a"], 180.0, delta=5.0)
        self.assertAlmostEqual(fit["free"]["c_turn"], 4000.0, delta=200.0)
        self.assertGreater(fit["free"]["r2"], 0.999)
        # the cubic law can't follow a b = 1.8 boat as well
        self.assertLess(fit["cubic"]["r2"], fit["free"]["r2"])

    def test_recovers_cubic_law(self):
        d = synthetic_day(a=40.0, b=3.0, c_turn=7000.0)
        fit = sm.fit_power_law(sm.plateau_table(d, sm.plateaus(d)))
        self.assertAlmostEqual(fit["cubic"]["k"], 40.0, delta=0.5)
        self.assertAlmostEqual(fit["cubic"]["c_turn"], 7000.0, delta=300.0)
        self.assertAlmostEqual(fit["free"]["b"], 3.0, delta=0.02)

    def test_noise_does_not_bias_much(self):
        d = synthetic_day(noise_w=150.0, seed=3)
        fit = sm.fit_power_law(sm.plateau_table(d, sm.plateaus(d)))
        self.assertAlmostEqual(fit["free"]["b"], 1.8, delta=0.1)


class TestPlateaus(unittest.TestCase):
    def test_steps_become_plateaus(self):
        d = synthetic_day()
        segs = sm.plateaus(d)
        self.assertGreaterEqual(len(segs), 30)
        for i0, i1 in segs:
            self.assertGreaterEqual((i1 - i0) * sm.DT, sm.PLATEAU_MIN_S)

    def test_no_propulsion_no_plateau(self):
        d = synthetic_day()
        d["gps_speed"] = np.full(len(d["t"]), 0.5)  # moored, power spikes only
        self.assertEqual(sm.plateaus(d), [])


class TestValidation(unittest.TestCase):
    def test_true_model_has_no_error(self):
        d = synthetic_day()
        model = lambda u, r: sm.p_surge(u, r, 180.0, 1.8, 4000.0)  # noqa: E731
        pe = sm.power_error(d, model)
        self.assertLess(pe["rmse_w"], 1.0)
        self.assertAlmostEqual(pe["range_error_pct"], 0.0, delta=0.01)
        fw = sm.forward_replay(d, model, mass_kg=318, eta=0.37)
        # the synthetic speed jumps instantly at each step and turn, the replay has inertia
        # (tau ~4 s), so RMSE isn't 0; but no bias and the same distance
        self.assertLess(abs(fw["bias_mps"]), 0.02)
        self.assertAlmostEqual(fw["distance_error_pct"], 0.0, delta=0.5)

    def test_too_expensive_model_is_conservative(self):
        d = synthetic_day()
        pe = sm.power_error(d, lambda u, r: 1.1 * sm.p_surge(u, r, 180.0, 1.8, 4000.0))
        self.assertAlmostEqual(pe["range_error_pct"], -9.1, delta=0.1)  # 1/1.1 - 1


class TestStrategyHook(unittest.TestCase):
    def test_eta_grid_reproduces_curve_inside_range_only(self):
        v = np.array([1.0, 2.0, 4.0, 5.0, 8.0])
        R = np.array([30.0, 60.0, 190.0, 240.0, 450.0])
        old = np.full(5, 0.4)
        surge = {"a": 180.0, "b": 1.8, "valid_u_range_mps": [3.4, 5.6]}
        eta = surge_eta_grid(surge, v, R, old)
        for i in (2, 3):
            self.assertAlmostEqual(R[i] * v[i] / eta[i], 180.0 * v[i] ** 1.8, delta=1.0)
        for i in (0, 1, 4):
            self.assertEqual(eta[i], 0.4)


if __name__ == "__main__":
    unittest.main()
