
# Deep Space Rover Autonomy Kit

>行星车自主导航与智能控制开源起步包。特性：
>**月表地形**合成（多尺度噪声 + 陨坑 + 岩块）
>**风险感知**路径规划对比：A*、RRT*、蚁群优化（ACO）、强化学习（Q-learning）
>基于地形梯度的**姿态稳定代理**与**能耗评估**
>**多车协同覆盖**（Voronoi 分区 + 局部 A* 推进）
>自动生成 **19 张高级图** 和 `benchmarks.json` 指标
>


---

## 1. 开始（Quick Start）

```bash
python main.py
```

运行后在 `outputs/` 目录下生成全部图像与指标文件。若你是从压缩包解压，已内置一套示例输出，可直接浏览图片并对照下文说明。

---

## 2. 科学目标与研究问题

- **目标**：在未知/部分已知月面地形上，实现 **安全、高效、可复现** 的行星车自主导航与协同探索，并提供**可量化**的对比基线，便于替换为 SOTA 算法（对标 ICRA/IROS、RA-L/T-RO、JFR）。
- **关键问题**：
  1) 如何基于**坡度/粗糙度**构建**滑移风险**与**通行代价**？  
  2) 规划器（A*/RRT*/ACO/强化学习）在**能耗/长度/风险**三维指标下的**帕累托表现**？  
  3) 多车如何在通信简化条件下实现**高覆盖率**？  

---

## 3. 方法概览

1. **地形生成**（`modules/terrain.py`）  
   - 多尺度 Perlin-like 噪声 + **负高斯坑（陨坑）** + **正高斯包（岩块）** 合成高程图 `Z`；  
   - 计算**坡度图** `slope`，合成**滑移风险图** `risk`（坡度 + 粗糙度加权，岩块区域再加权）；  
   - 基于 `slope` 与 `Z` 形成**通行代价图** `cost`。

2. **规划器族群**（`modules/planners.py`）  
   - **A\***：风险感知网格搜索；  
   - **RRT\***：在**障碍掩膜**上进行采样式规划；  
   - **ACO**：蚁群在代价图上以信息素/代价引导；  
   - **Q-learning**：在未知代价下试探学习贪婪策略。

3. **姿态/能耗评估**（`modules/controllers.py`）  
   - 用地形梯度（坡度）估计**roll/pitch** 代理；  
   - **能耗** = 路径步长对代价/坡度的积分近似。

4. **多车协同覆盖**（`modules/multiagent.py`）  
   - **Voronoi** 分区确定职责区；按栅格间隔生成目标点，**局部 A\*** 逐步前推；  
   - 统计覆盖率曲线。

---

## 4. 结果与图片说明（outputs/）

> 下列所有文件均由 `python main.py` 自动生成，数值随随机种子略有不同。

<img width="960" height="720" alt="fig01_terrain_surface" src="https://github.com/user-attachments/assets/82168a4c-a4b7-43cb-9f4e-4070fb013763" />

### 图 1：`fig01_terrain_surface.png` — 3D 高保真月表地形
- **看点**：多尺度纹理 + 陨坑/岩块的形态，适合测试**轮地相互作用**与**俯仰稳定**。  
- **研究意义**：在无真实 DEM 时，提供**可控统计特性**的地形；后续可替换为 LRO/HiRISE DEM。
  
<img width="1020" height="765" alt="fig02_slope" src="https://github.com/user-attachments/assets/3722c5b6-a73b-42ba-b635-7e98c2906933" />

### 图 2：`fig02_slope.png` — 坡度（梯度范数）热图
- **看点**：高坡区更亮，通常伴随高滑移/翻覆风险。  
- **研究意义**：为**风险图/代价图**提供核心物理量。

<img width="1020" height="765" alt="fig03_risk" src="https://github.com/user-attachments/assets/9e767cde-6af5-49ef-824e-724f91a42db5" />

### 图 3：`fig03_risk.png` — 滑移风险图
- **看点**：坡度 + 粗糙度综合，岩块区**加权惩罚**；明亮区域即高风险。  
- **研究意义**：可对标**traction-aware** 方法，检验“避险”能力。

  <img width="1020" height="765" alt="fig04_cost" src="https://github.com/user-attachments/assets/c819eb4d-a7f4-471c-8a6c-99f30547a77c" />

### 图 4：`fig04_cost.png` — 通行代价图
- **看点**：将**能耗/风险**合成统一代价域。  
- **研究意义**：为 A*/ACO 等提供统一优化目标。

  <img width="1020" height="765" alt="fig05_astar_path" src="https://github.com/user-attachments/assets/32a43efa-92a5-4f06-9c86-1f882a425602" />

### 图 5：`fig05_astar_path.png` — 风险感知 A* 路径
- **看点**：在代价图上叠加路径；通常趋向**绕开高坡/岩块**。  
- **研究意义**：网格基线，后续可替换为 **D\* Lite / Anytime Repairing A\*** 以支持**在线重规划**。

  <img width="1020" height="765" alt="fig06_rrt_path" src="https://github.com/user-attachments/assets/5f04190c-5c49-42b4-b976-aab499276ad1" />

### 图 6：`fig06_rrt_path.png` — RRT\* 在障碍掩膜上的路径
- **看点**：采样式对比基线；障碍由 `risk>阈值` 产生。  
- **研究意义**：为 **BIT\***、**RRTX** 等 SOTA 提供替换位。

<img width="1020" height="765" alt="fig07_aco_path" src="https://github.com/user-attachments/assets/17480f38-6993-4596-a38a-45c86581c74c" />



## 5. 指标文件（`outputs/benchmarks.json`）
- `best_planner`：本次运行能耗最优的规划器名称；  
- `energy_best`：其能耗值；  
- `pareto`：各规划器的（长度、能耗、平均风险）列表，可生成帕累托前沿；  
- `coverage_final`：多车最终覆盖率（0–1）。

---

## 6. 目录结构
```
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
└── main.py                                    # Root 
