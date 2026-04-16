from __future__ import annotations

import argparse
from pathlib import Path
import sys


REPO_ROOT = Path(__file__).resolve().parents[1]
ROS2_PKG_ROOT = REPO_ROOT / "ros2_ws" / "src" / "space_rover_autonomy"

if str(ROS2_PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(ROS2_PKG_ROOT))

from space_rover_autonomy.simulation.validation import run_validation_suite


def main() -> None:
    parser = argparse.ArgumentParser(description="Run a reproducible Gazebo-oriented validation suite for rover autonomy.")
    parser.add_argument(
        "--config",
        type=Path,
        default=REPO_ROOT / "configs" / "gazebo_validation_suite.yaml",
        help="Path to the YAML validation suite configuration.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=REPO_ROOT / "outputs",
        help="Directory for JSON summaries and metric plots.",
    )
    args = parser.parse_args()

    summary = run_validation_suite(args.config, args.output_dir)
    overall = summary["aggregate"]["overall"]
    print(
        "Validation suite complete: "
        f"success_rate={overall.get('success_rate', 0.0):.3f}, "
        f"energy_efficiency={overall.get('energy_efficiency', 0.0):.3f}, "
        f"path_optimality={overall.get('path_optimality', 0.0):.3f}"
    )


if __name__ == "__main__":
    main()
