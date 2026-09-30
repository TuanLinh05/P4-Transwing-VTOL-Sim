"""
Command line interface for experiments.
"""
import argparse
import sys
from pathlib import Path

def run_command(args):
    print(f"Running experiment with request: {args.request}")

def list_command(args):
    print(f"Listing completed runs in {args.results_dir}")

def compare_command(args):
    print(f"Comparing runs: {args.cases}")

def main():
    parser = argparse.ArgumentParser(description="P4 Transwing VTOL Experiment Runner")
    subparsers = parser.add_subparsers(dest="command", required=True)

    run_parser = subparsers.add_parser("run", help="Run an experiment")
    run_parser.add_argument("request", type=Path, help="Path to experiment request JSON")
    run_parser.set_defaults(func=run_command)

    list_parser = subparsers.add_parser("list", help="List completed runs")
    list_parser.add_argument("--results-dir", type=Path, default=Path("Simulation/results"))
    list_parser.set_defaults(func=list_command)

    compare_parser = subparsers.add_parser("compare", help="Compare runs")
    compare_parser.add_argument("cases", nargs="+", help="Run IDs or cases to compare")
    compare_parser.set_defaults(func=compare_command)

    args = parser.parse_args()
    args.func(args)

if __name__ == "__main__":
    main()
