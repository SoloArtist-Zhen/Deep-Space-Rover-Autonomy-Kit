import sys
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
ROS2_PKG_ROOT = REPO_ROOT / "ros2_ws" / "src" / "space_rover_autonomy"

if str(ROS2_PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(ROS2_PKG_ROOT))

from space_rover_autonomy.decision_making import RoverAction, RoverExplorationMDP, SimulationBasedTrainer
from space_rover_autonomy.system import build_demo_scenario


class HierarchicalAutonomyStackTest(unittest.TestCase):
    def test_demo_cycle_uses_mdp_decision_policy(self) -> None:
        system, tasks, mission_goal = build_demo_scenario(seed=4)
        cycle = system.run_cycle("test_mission", tasks, mission_goal)
        self.assertGreaterEqual(len(cycle.mission_plan.scheduled_tasks), 1)
        self.assertEqual(cycle.path_plan.planner_name, "mdp_mcts_belief")
        self.assertTrue(cycle.path_plan.action_sequence)
        self.assertIn(cycle.path_plan.action_sequence[0], {"move_north", "move_south", "move_west", "move_east", "scan", "replan"})
        self.assertIn(cycle.control_command.mode, {"track_path", "hold", "scan", "replan"})
        self.assertIn("semantic_class_names", cycle.world_model.environment_representation)
        self.assertEqual(
            cycle.world_model.environment_representation["semantic_grid"].shape,
            cycle.world_model.elevation.shape,
        )

    def test_resource_constraints_can_force_conserve_mode(self) -> None:
        system, tasks, mission_goal = build_demo_scenario(seed=7)
        sensor_data = system.sensor_suite.read_all()
        sensor_data["resource_state"].battery_level = 0.05
        sensor_data["resource_state"].thermal_margin = 0.08
        cycle = system.run_cycle("critical_resources", tasks, mission_goal)
        self.assertEqual(cycle.behavior.autonomy_mode, "conserve")
        self.assertEqual(cycle.path_plan.planner_name, "mdp_mcts_belief")

    def test_scan_action_reduces_uncertainty(self) -> None:
        system, tasks, mission_goal = build_demo_scenario(seed=9)
        sensor_data = system.sensor_suite.read_all()
        world_model = system.world_model_builder.build(
            elevation=sensor_data["elevation_map"],
            rover_pose=sensor_data["rover_pose"],
            goal=mission_goal,
            science_targets=[task.target for task in tasks],
            resource_state=sensor_data["resource_state"],
            rgb_image=sensor_data["rgb_image"],
            lidar_height_map=sensor_data["lidar_height_map"],
            lidar_validity=sensor_data["lidar_validity"],
        )
        env = RoverExplorationMDP(world_model, mission_goal, sensor_data["resource_state"], max_steps=8)
        state = env.initial_state()
        next_state, reward, _, _ = env.step(state, RoverAction.SCAN)
        self.assertLess(next_state.mean_uncertainty, state.mean_uncertainty)
        self.assertGreater(reward, -1.0)

    def test_foundation_perception_outputs_semantic_occupancy(self) -> None:
        system, tasks, mission_goal = build_demo_scenario(seed=5)
        sensor_data = system.sensor_suite.read_all()
        world_model = system.world_model_builder.build(
            elevation=sensor_data["elevation_map"],
            rover_pose=sensor_data["rover_pose"],
            goal=mission_goal,
            science_targets=[task.target for task in tasks],
            resource_state=sensor_data["resource_state"],
            rgb_image=sensor_data["rgb_image"],
            lidar_height_map=sensor_data["lidar_height_map"],
            lidar_validity=sensor_data["lidar_validity"],
        )
        env_repr = world_model.environment_representation
        self.assertEqual(env_repr["semantic_probabilities"].shape[:2], world_model.elevation.shape)
        self.assertEqual(env_repr["semantic_probabilities"].shape[2], len(env_repr["semantic_class_names"]))
        self.assertTrue(((env_repr["occupancy_probability"] >= 0.0) & (env_repr["occupancy_probability"] <= 1.0)).all())
        self.assertTrue(((env_repr["perception_uncertainty"] >= 0.0) & (env_repr["perception_uncertainty"] <= 1.0)).all())

    def test_training_pipeline_builds_policy_prior(self) -> None:
        trainer = SimulationBasedTrainer(map_size=32, episodes=3, max_steps=10, planner_iterations=10, planner_horizon=4)
        summary = trainer.train()
        self.assertEqual(len(summary["episodes"]), 3)
        self.assertGreater(summary["policy_table_size"], 0)
        self.assertEqual(len(summary["returns"]), 3)


if __name__ == "__main__":
    unittest.main()
