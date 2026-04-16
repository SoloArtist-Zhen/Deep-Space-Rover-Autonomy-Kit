import sys
import tempfile
import unittest
from pathlib import Path

import yaml


REPO_ROOT = Path(__file__).resolve().parents[1]
ROS2_PKG_ROOT = REPO_ROOT / "ros2_ws" / "src" / "space_rover_autonomy"

if str(ROS2_PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(ROS2_PKG_ROOT))

from space_rover_autonomy.simulation.validation import run_validation_suite


class ValidationSuiteTest(unittest.TestCase):
    def test_reproducible_validation_exports_metrics(self) -> None:
        config = {
            "suite_name": "test_validation",
            "sim_backend": "gazebo_harmonic",
            "output_prefix": "test_validation",
            "reproducibility": {"seeds": [3], "max_cycles": 12, "goal_tolerance": 2.0},
            "scenarios": [
                {
                    "name": "unit_nominal",
                    "terrain": {"size": 40, "crater_density": 1.0, "roughness_gain": 1.0, "elevation_scale": 1.0},
                    "sensors": {
                        "camera_noise_std": 4.0,
                        "camera_blur_sigma": 0.6,
                        "camera_delay_steps": 0,
                        "lidar_noise_std": 0.01,
                        "lidar_dropout_base": 0.03,
                        "lidar_delay_steps": 0,
                    },
                    "resources": {
                        "battery_level": 0.82,
                        "compute_load": 0.28,
                        "thermal_margin": 0.86,
                        "comms_bandwidth": 0.74,
                        "wheel_health": 0.95,
                        "memory_load": 0.22,
                    },
                }
            ],
        }
        with tempfile.TemporaryDirectory() as temp_dir:
            temp_path = Path(temp_dir)
            config_path = temp_path / "suite.yaml"
            config_path.write_text(yaml.safe_dump(config))
            summary = run_validation_suite(config_path, temp_path)
            self.assertIn("aggregate", summary)
            self.assertIn("overall", summary["aggregate"])
            self.assertIn("success_rate", summary["aggregate"]["overall"])
            self.assertEqual(len(summary["trials"]), 1)
            self.assertTrue((temp_path / "test_validation_suite.json").exists())
            self.assertTrue((temp_path / "fig25_validation_success_rate.png").exists())


if __name__ == "__main__":
    unittest.main()
