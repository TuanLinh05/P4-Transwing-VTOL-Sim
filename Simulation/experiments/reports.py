"""
Reports generation module.
"""
import json
import csv
from pathlib import Path

def generate_json_report(path: Path, metrics: dict) -> None:
    with open(path, "w", encoding="utf-8") as f:
        json.dump(metrics, f, indent=2)

def generate_csv_summary(path: Path, runs_data: list[dict]) -> None:
    if not runs_data:
        return
    keys = runs_data[0].keys()
    with open(path, "w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=keys)
        writer.writeheader()
        for run in runs_data:
            writer.writerow(run)

def generate_markdown_report(path: Path, summary: dict) -> None:
    with open(path, "w", encoding="utf-8") as f:
        f.write(f"# Experiment Summary\n\n")
        for k, v in summary.items():
            f.write(f"- **{k}**: {v}\n")

def generate_verdict(path: Path, results: list) -> None:
    passed = all(r.status == "pass" for r in results)
    verdict = {
        "passed": passed,
        "results": [{"metric": r.metric, "status": r.status, "value": r.value} for r in results]
    }
    with open(path, "w", encoding="utf-8") as f:
        json.dump(verdict, f, indent=2)
