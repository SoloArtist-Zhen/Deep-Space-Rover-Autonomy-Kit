# Deep Space Rover Hierarchical Autonomy

## Code-Structure Diagram

```text
deep_space_rover_kit/
├── configs/
│   └── research_mission_profile.yaml          # Publication-facing mission specification
├── docs/
│   ├── architecture.md                        # Architecture/data-flow document
│   ├── rhea_method.md                         # Publication-facing method section draft
│   └── simulation_validation_framework.md     # Gazebo validation design
├── experiments/
│   ├── run_hierarchical_demo.py               # Pure-Python research demo entrypoint
│   ├── run_validation_suite.py                # Reproducible validation benchmark
│   └── train_mdp_policy.py                    # Simulation-based MDP policy training
├── legacy/
│   └── prototype/                             # Archived baseline implementation
│       ├── baseline_main.py
│       └── modules/
├── outputs/                                   # Figures + JSON summaries
├── ros2_ws/
│   └── src/
│       └── space_rover_autonomy/
│           ├── config/autonomy_stack.yaml     # ROS2 parameters
│           ├── config/gazebo_validation.yaml  # Gazebo validation parameters
│           ├── launch/hierarchical_autonomy.launch.py
│           ├── launch/gazebo_validation.launch.py
│           ├── package.xml
│           ├── setup.py
│           └── space_rover_autonomy/
│               ├── core/
│               │   ├── models.py             # Mission/world/path/control data contracts
│               │   └── topics.py             # ROS2 topic graph
│               ├── interfaces/
│               │   ├── sensors.py            # RGB / LiDAR / state sensor adapters
│               │   └── actuators.py          # Extensible actuator adapters
│               ├── perception/
│               │   ├── foundation_features.py# DINOv2 feature extraction
│               │   ├── semantic_fusion.py    # Semantic occupancy + LiDAR fusion
│               │   ├── terrain_analysis.py   # Supporting geometric operators
│               │   └── world_model_builder.py
│               ├── mission_planning/
│               │   ├── task_scheduler.py     # Task ordering and feasibility filtering
│               │   └── mission_planner.py    # Mission-level constraints and active task
│               ├── decision_making/
│               │   ├── behavior_manager.py   # Task execution mode switching
│               │   ├── mdp.py                # Belief-state MDP / POMDP-lite simulator
│               │   ├── mcts.py               # MCTS policy + learned prior table
│               │   ├── training.py           # Simulation-based policy training
│               │   ├── planners.py           # Classical planner baselines
│               │   └── resource_aware_planner.py
│               ├── motion_control/
│               │   ├── attitude_controller.py
│               │   ├── path_follower.py
│               │   └── motion_controller.py
│               ├── nodes/
│               │   ├── perception_node.py
│               │   ├── mission_planning_node.py
│               │   ├── decision_node.py
│               │   └── motion_control_node.py
│               ├── simulation/terrain.py
│               ├── simulation/validation.py   # Reproducible validation harness
│               ├── system.py                 # End-to-end orchestration for experiments
│               └── visualization/plots.py
├── tests/
│   └── test_hierarchical_stack.py
└── main.py                                    # Root launcher for demo cycle
```

## Three-Layer Autonomy Mapping

1. Mission Planning
   - `mission_planning/task_scheduler.py`
   - `mission_planning/mission_planner.py`
   - Responsibility: prioritize science/relay tasks under energy, bandwidth, and deadline constraints.
2. Decision Making
   - `decision_making/behavior_manager.py`
   - `decision_making/mdp.py`
   - `decision_making/mcts.py`
   - `decision_making/training.py`
   - `decision_making/resource_aware_planner.py`
   - Responsibility: maintain a belief-state MDP over rover pose, obstacle belief, energy, and uncertainty; choose `move`, `scan`, or `replan` via MCTS; and improve priors through simulation-based training.
3. Motion Control
   - `motion_control/path_follower.py`
   - `motion_control/attitude_controller.py`
   - `motion_control/motion_controller.py`
   - Responsibility: convert the selected path into velocity, steering, and body-attitude commands for the actuator layer.

Perception is intentionally decoupled from the three layers and implemented as a supporting subsystem:

- `interfaces/sensors.py`
- `perception/foundation_features.py`
- `perception/semantic_fusion.py`
- `perception/world_model_builder.py`

## Data Flow

```text
RGB Camera + LiDAR + State Sensors
  -> DINOv2 Feature Extraction
  -> Semantic Occupancy Fusion
  -> /autonomy/perception/world_model
  -> /autonomy/resources/state

WorldModel + ResourceState
  -> Mission Planning Layer
  -> /autonomy/mission/plan

MissionPlan + WorldModel + ResourceState
  -> Decision Making Layer
  -> Belief-State MDP
  -> State = [position, obstacle belief, remaining energy, uncertainty]
  -> Actions = [move_north, move_south, move_west, move_east, scan, replan]
  -> /autonomy/decision/behavior
  -> /autonomy/decision/path_plan

PathPlan + WorldModel + ResourceState
  -> Motion Control Layer
  -> /autonomy/control/command
  -> /autonomy/control/actuator_feedback
```

## MDP / POMDP-Lite Semantics

- Unknown terrain is represented through `obstacle_belief`, `observed_mask`, and `uncertainty_map`.
- Passive observation reveals only a local neighborhood around the rover.
- `scan` is an active information-gathering action that reduces local uncertainty.
- `replan` rebuilds a global route from the current belief map rather than from the hidden ground-truth map.
- The decision layer therefore behaves as a POMDP-lite belief-space planner while keeping the interface compatible with the rest of the autonomy stack.

## Foundation-Model Perception Stack

- Vision stream: DINOv2 patch embeddings from `foundation_features.py`.
- LiDAR stream: noisy height map plus validity/confidence mask.
- Fusion output:
  - semantic occupancy grid
  - occupancy probability map
  - traversability map
  - perception uncertainty map
  - structured environment representation for planning
- If no local DINOv2 checkpoint is available, the code falls back to an offline random-initialized DINOv2-compatible model so the repository remains executable without network access.

## Simulation-Based Training Pipeline

1. `SimulationBasedTrainer` generates random lunar terrains and reachable start-goal pairs.
2. `MCTSDecisionPolicy` runs episodes on the belief-state MDP.
3. State-action-return tuples update `TabularPolicyPrior`, which biases future MCTS expansions.
4. Training artifacts are exported to `outputs/mdp_training_summary.json` and `outputs/fig23_mdp_training_returns.png`.

## Gazebo-Oriented Validation Pipeline

1. `configs/gazebo_validation_suite.yaml` defines the seed set, terrain regimes, and sensor corruption levels.
2. `simulation/validation.py` expands each scenario into deterministic trials and runs the autonomy stack in closed loop.
3. `launch/gazebo_validation.launch.py` reuses the same parameters for ROS2 or Gazebo execution with `use_sim_time`.
4. The benchmark exports:
   - success rate
   - energy efficiency
   - path optimality
5. Results are written to JSON manifests plus summary plots for paper-ready reporting.

## Research Positioning

- `legacy/prototype/` preserves the lightweight baseline for ablation and reproducibility.
- `ros2_ws/src/space_rover_autonomy/space_rover_autonomy/system.py` provides a deterministic experiment path for papers when ROS2 is unavailable.
- ROS2 nodes in `nodes/` reuse the same core logic, so simulation and deployment share the same autonomy contracts rather than two divergent code paths.
