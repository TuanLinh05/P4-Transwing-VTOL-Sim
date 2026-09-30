"""
Tests for metrics computation.
"""
import unittest
from Simulation.experiments.metrics import (
    FlightSample,
    compute_hover_metrics,
    compute_transition_metrics,
    compute_mission_metrics,
    compute_landing_metrics
)

class TestMetrics(unittest.TestCase):
    def setUp(self):
        self.samples = [
            FlightSample(
                t_s=0.0, phase="hover", mode="QLOITER", armed=True,
                north_m=0.0, east_m=0.0, relative_alt_m=10.0,
                roll_deg=0.0, pitch_deg=0.0, yaw_deg=0.0,
                airspeed_mps=0.0, groundspeed_mps=0.0, vertical_speed_mps=0.0,
                throttle_pct=0.5, servo_outputs=(1500, 1500), mission_current=0,
                battery_voltage_v=12.0, battery_current_a=10.0
            )
        ]

    def test_compute_hover_metrics(self):
        metrics = compute_hover_metrics(self.samples, target_alt=10.0)
        self.assertEqual(metrics["position_rmse_m"], 0.0)
        self.assertEqual(metrics["altitude_rmse_m"], 0.0)

if __name__ == "__main__":
    unittest.main()
