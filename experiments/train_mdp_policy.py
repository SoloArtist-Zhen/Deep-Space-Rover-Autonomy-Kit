from pathlib import Path
import sys


REPO_ROOT = Path(__file__).resolve().parents[1]
ROS2_PKG_ROOT = REPO_ROOT / "ros2_ws" / "src" / "space_rover_autonomy"

if str(ROS2_PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(ROS2_PKG_ROOT))

from space_rover_autonomy.decision_making import SimulationBasedTrainer


if __name__ == "__main__":
    trainer = SimulationBasedTrainer()
    trainer.train(output_dir=REPO_ROOT / "outputs")
