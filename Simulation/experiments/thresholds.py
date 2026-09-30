"""
Thresholds engine module.
"""
from dataclasses import dataclass
from typing import Literal
import json

@dataclass(frozen=True)
class CheckResult:
    metric: str
    value: float | bool | None
    status: Literal["pass", "fail", "unsupported"]
    limit: object
    reason: str

def evaluate_metric(name: str, value: object, rule: dict) -> CheckResult:
    if value is None:
        return CheckResult(name, None, "unsupported", rule, "No valid samples")
    if "max" in rule:
        passed = float(value) <= float(rule["max"])
    elif "min" in rule:
        passed = float(value) >= float(rule["min"])
    elif "equals" in rule:
        passed = value == rule["equals"]
    else:
        raise ValueError(f"Unsupported threshold rule for {name}")
    return CheckResult(name, value, "pass" if passed else "fail", rule, "")

def evaluate_all(metrics: dict, threshold_profile: dict) -> list[CheckResult]:
    results = []
    for key, rule in threshold_profile.items():
        if key in ["schema_version", "profile"]:
            continue
        val = metrics.get(key)
        results.append(evaluate_metric(key, val, rule))
    return results

def load_threshold_profile(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)
