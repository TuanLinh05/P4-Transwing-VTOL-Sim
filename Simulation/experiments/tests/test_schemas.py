"""
Tests for schemas.
"""
import unittest
from Simulation.experiments.schemas import RunPhase, SafetyLimits

class TestSchemas(unittest.TestCase):
    def test_run_phase(self):
        self.assertEqual(RunPhase.PREPARE, "prepare")
        
    def test_safety_limits(self):
        limits = SafetyLimits(120.0, 500.0, 60.0, 60.0)
        self.assertEqual(limits.max_altitude_m, 120.0)

if __name__ == "__main__":
    unittest.main()
