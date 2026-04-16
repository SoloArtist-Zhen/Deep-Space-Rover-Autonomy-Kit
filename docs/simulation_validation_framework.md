# Gazebo Validation Framework for Deep-Space Rover Autonomy

## Overview

This repository now includes a **Gazebo-oriented validation framework** for publication-grade evaluation of rover autonomy. The framework is designed around **Gazebo Harmonic + ROS2**, with a deterministic offline experiment harness that uses the same terrain, sensor, and seed specifications. The purpose is to keep the autonomy stack identical across:

1. ROS2 node-level validation in simulation.
2. Reproducible multi-seed experiment sweeps.
3. Paper-ready metric aggregation and visualization.

The main design choice is to standardize all experiments through a single suite specification:

- backend: `gazebo_harmonic`
- suite config: `configs/gazebo_validation_suite.yaml`
- ROS2 sim config: `ros2_ws/src/space_rover_autonomy/config/gazebo_validation.yaml`
- launch entry: `ros2_ws/src/space_rover_autonomy/launch/gazebo_validation.launch.py`
- deterministic runner: `experiments/run_validation_suite.py`

## Architecture

```text
Gazebo / ROS2 Validation Stack

Gazebo Harmonic world
  -> randomized terrain profile
  -> simulated rover state
  -> ROS2 time

Sensor degradation layer
  -> RGB blur + additive noise + delay
  -> LiDAR noise + dropout + delay
  -> delayed registration against rover motion

Perception layer
  -> foundation-model semantic occupancy
  -> uncertainty map
  -> world model

Autonomy layer
  -> mission planning
  -> belief-state decision making
  -> motion control

Experiment harness
  -> multi-seed sweep
  -> metrics aggregation
  -> JSON manifests
  -> figure generation
```

## Why Gazebo

Gazebo is chosen over Isaac Sim for this repository because:

1. The stack is already ROS2-oriented, so Gazebo integration is lower-friction.
2. Headless execution and seeded runs are easier to standardize for paper experiments.
3. The same autonomy nodes can be launched without introducing a separate middleware layer.

Isaac Sim remains a valid future backend for photorealistic transfer studies, but Gazebo is the primary reproducible benchmark backend here.

## Terrain Randomization

The validation framework uses procedural terrain generation with three tunable factors:

1. `crater_density`
2. `roughness_gain`
3. `elevation_scale`

The terrain generator starts from the existing synthetic lunar terrain model and then adds randomized crater and boulder fields. This produces a controlled difficulty ladder while keeping the generation fully deterministic under a fixed seed.

For each experiment seed:

$$
\mathcal{Z}_{seed} = f_{terrain}(seed, \rho_{crater}, \rho_{rough}, s_{elev})
$$

The resulting height field is reused consistently across:

- RGB rendering
- LiDAR simulation
- autonomy rollouts
- metric computation

## Sensor Noise and Delay Model

The validation framework explicitly models two classes of sensing corruption.

### Noise

RGB observations include:

1. additive Gaussian image noise
2. optical blur

LiDAR observations include:

1. range or height noise
2. dropout probability
3. validity degradation

### Delay

Sensor delay is modeled as **registration lag** with respect to the rover's previous motion. In practice, delayed data are spatially shifted against the most recent executed motion, which approximates stale observation alignment under latency.

This is implemented in:

- `simulation/perception_sensing.py`
- `nodes/perception_node.py`

The result is a controlled way to evaluate robustness under stale perception without requiring a custom simulator plugin.

## Metrics

The framework reports the three required mission-level metrics:

1. **Success rate**
   - fraction of trials that reach the goal within tolerance before energy or collision termination
2. **Energy efficiency**
   - ratio between reference-path energy and autonomy execution energy
3. **Path optimality**
   - ratio between reference-path cost and executed-path cost

Higher is better for all three metrics.

More precisely:

$$
\text{SuccessRate} = \frac{1}{N} \sum_{i=1}^{N} \mathbb{I}[\text{trial}_i \text{ succeeds}]
$$

$$
\text{EnergyEfficiency} = \min\left(1,\frac{E_{\text{ref}}}{E_{\text{exec}}}\right)
$$

$$
\text{PathOptimality} = \min\left(1,\frac{J_{\text{ref}}}{J_{\text{exec}}}\right)
$$

where the reference path is computed by weighted A* over the clean reference world model.

## Reproducibility Protocol

The experiment harness guarantees reproducibility through:

1. explicit seed lists in `configs/gazebo_validation_suite.yaml`
2. deterministic terrain generation
3. deterministic sensor perturbation generation
4. automatic config snapshot export
5. hashed manifest export

Each run produces:

- `outputs/gazebo_validation_manifest.json`
- `outputs/gazebo_validation_config_snapshot.yaml`
- `outputs/gazebo_validation_suite.json`
- `outputs/fig25_validation_success_rate.png`
- `outputs/fig26_validation_energy_efficiency.png`
- `outputs/fig27_validation_path_optimality.png`

## Running the Suite

### Reproducible offline benchmark

```bash
python3 experiments/run_validation_suite.py
```

### ROS2 launch for Gazebo-oriented validation

```bash
ros2 launch space_rover_autonomy gazebo_validation.launch.py \
  run_gazebo:=true \
  gazebo_world:=/path/to/generated_world.sdf
```

The launch path is designed for ROS2/Gazebo experiments. The Python runner is designed for deterministic metric sweeps and paper tables.

## Recommended Publishable Protocol

For publication-quality results:

1. expand the seed list from the default quick-run set to at least `30` seeds per scenario
2. report mean and standard deviation for each metric
3. include nominal, rugged, and delayed or degraded sensing regimes
4. fix all suite YAML files under version control
5. archive manifests and output summaries with the paper artifact

## Positioning

This framework is appropriate for papers that need:

1. reproducible autonomy validation
2. robustness evaluation under terrain and sensing shifts
3. ROS2-compatible simulation workflows
4. benchmark-ready metrics for deep-space rover exploration
