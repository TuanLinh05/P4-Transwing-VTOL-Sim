"""
Tests for thresholds evaluation.
"""
import unittest
from Simulation.experiments.thresholds import evaluate_metric

class TestThresholds(unittest.TestCase):
    def test_evaluate_metric_max_pass(self):
        result = evaluate_metric("hover_rmse", 1.0, {"max": 2.0})
        self.assertEqual(result.status, "pass")

    def test_evaluate_metric_max_fail(self):
        result = evaluate_metric("hover_rmse", 3.0, {"max": 2.0})
        self.assertEqual(result.status, "fail")

if __name__ == "__main__":
    unittest.main()
