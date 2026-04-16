# Reliability-Aware Hybrid Explainable Autonomy (RHEA) for Deep-Space Rover Exploration

This document proposes a publication-oriented autonomy framework that extends the current hierarchical stack toward a deployable deep-space rover system. The design targets unknown planetary environments, limited onboard resources, intermittent sensing quality, and the need for interpretable decisions during long-delay operations.

## III. Method

### A. Overview

We propose **RHEA**, a reliability-aware hybrid autonomy framework that couples model-based planning with learned policy priors and explicit explainability. RHEA follows the existing three-layer organization of mission planning, decision making, and motion control, but augments the decision loop with a reliability estimator and an explanation generator. The key idea is to optimize exploration utility not only with respect to terrain geometry and science value, but also with respect to the **expected trustworthiness of the autonomy stack itself**.

At each decision epoch, the rover constructs a structured world model from semantic occupancy, uncertainty, and resource state; updates reliability beliefs for perception, planning, and control; allocates mission resources across candidate science tasks; and then invokes a hybrid decision module in which a learned policy/value prior guides a reliability-constrained tree search. Only the first action of the selected plan is executed, after which the system replans with updated observations and explanation traces.

Relative to a classical cost-map planner, RHEA differs in four aspects:

1. It operates in belief space rather than on a fully observed deterministic map.
2. It uses a learned prior to bias search toward promising actions in unknown terrain.
3. It explicitly penalizes low-confidence or failure-prone autonomy states.
4. It returns a human-auditable explanation tuple for every high-level decision.

### B. Reliability-Aware State Representation

Let the rover state at time step $t$ be

$$
\mathbf{s}_t =
\left[
\mathbf{p}_t,\;
\mathbf{M}_t,\;
\mathbf{U}_t,\;
\mathbf{r}_t,\;
\mathbf{q}_t,\;
\mathcal{T}_t
\right],
$$

where $\mathbf{p}_t \in \mathbb{Z}^2$ is the rover position, $\mathbf{M}_t$ is the semantic occupancy belief map, $\mathbf{U}_t$ is the perception uncertainty map, $\mathbf{r}_t$ is the resource vector, $\mathbf{q}_t$ is the reliability vector, and $\mathcal{T}_t$ is the active mission task set.

The resource vector is defined as

$$
\mathbf{r}_t = [E_t, C_t, H_t, B_t, W_t, M_t],
$$

where $E_t$ is remaining energy, $C_t$ is compute availability, $H_t$ is thermal margin, $B_t$ is communications bandwidth, $W_t$ is wheel health, and $M_t$ is memory availability.

The reliability vector is defined at the subsystem level as

$$
\mathbf{q}_t =
\left[
q_t^{\text{perc}},
q_t^{\text{plan}},
q_t^{\text{ctrl}}
\right], \qquad
q_t^k \in [0,1].
$$

Each component models the probability that subsystem $k$ will remain behaviorally consistent over the next receding-horizon interval. Reliability is updated online from diagnostic evidence:

$$
q_{t+1}^k = (1-\alpha) q_t^k + \alpha \, \sigma(-\ell_t^k),
$$

where $\ell_t^k$ is a fault evidence score and $\sigma(\cdot)$ is the logistic function. Example evidence terms include perception out-of-distribution indicators, planner rollout disagreement, actuator slip, and resource saturation.

### C. Hybrid Planning-Learning Architecture

RHEA combines three elements:

1. **Resource-constrained mission planning** to select feasible science and relay tasks.
2. **Reliability-aware belief-space planning** to evaluate action sequences under uncertainty.
3. **Learned policy/value priors** to accelerate search and adapt online behavior.

The action space is

$$
\mathcal{A} =
\{
\texttt{move\_north},
\texttt{move\_south},
\texttt{move\_east},
\texttt{move\_west},
\texttt{scan},
\texttt{replan},
\texttt{sample},
\texttt{wait}
\}.
$$

The learned module provides a policy prior $\pi_\theta(a \mid \mathbf{s}_t)$ and a value estimate $V_\phi(\mathbf{s}_t)$, while the model-based component uses semantic occupancy, uncertainty, and resource dynamics to simulate rollouts. The planner then performs a constrained tree search whose node score is

$$
\mathrm{Score}(\mathbf{s}, a)
=
Q(\mathbf{s}, a)
+ c_{\text{puct}} \, \pi_\theta(a \mid \mathbf{s})
\frac{\sqrt{N(\mathbf{s})}}{1 + N(\mathbf{s}, a)}
- \lambda_{\rho} \, \widehat{F}(\mathbf{s}, a)
- \lambda_{u} \, \widehat{U}(\mathbf{s}, a),
$$

where $Q(\mathbf{s}, a)$ is the Monte Carlo return estimate, $\widehat{F}(\mathbf{s}, a)$ is predicted failure risk derived from $\mathbf{q}_t$, and $\widehat{U}(\mathbf{s}, a)$ is the expected epistemic uncertainty after executing action $a$.

This hybrid formulation preserves the safety and long-horizon reasoning of model-based planning while inheriting adaptation from learning.

### D. Reliability-Aware Decision Objective

RHEA solves the following constrained optimization problem over policy $\pi$:

$$
\pi^\star
=
\arg\max_{\pi}
\mathbb{E}_{\pi}
\left[
\sum_{t=0}^{T}
\gamma^t
\left(
r_t^{\text{sci}}
+ \lambda_i r_t^{\text{info}}
- \lambda_e c_t^{\text{energy}}
- \lambda_h c_t^{\text{hazard}}
- \lambda_u c_t^{\text{uncert}}
- \lambda_{\rho} c_t^{\text{unrel}}
\right)
\right]
$$

subject to

$$
\sum_{t=0}^{T} c_t^{\text{energy}} \le E_{\max},
\qquad
C_t \ge C_{\min},
\qquad
H_t \ge H_{\min},
\qquad
B_t \ge B_{\min},
\qquad
R_t \ge R_{\min},
$$

where $R_t$ is the aggregate autonomy reliability,

$$
R_t = \prod_{k \in \{\text{perc}, \text{plan}, \text{ctrl}\}} (q_t^k)^{w_k}.
$$

The unreliability cost is defined as

$$
c_t^{\text{unrel}} = 1 - R_t,
$$

and the information reward is used to prefer active sensing in unknown terrain:

$$
r_t^{\text{info}} =
\mathcal{H}(\mathbf{U}_t) - \mathcal{H}(\mathbf{U}_{t+1}),
$$

where $\mathcal{H}(\cdot)$ denotes map-wise entropy or mean uncertainty.

### E. Resource-Constrained Mission Planning

At the mission layer, candidate tasks are selected through a constrained knapsack-orienteering objective. Let $z_i \in \{0,1\}$ indicate whether task $i$ is selected. The planner solves

$$
\max_{\mathbf{z}}
\sum_{i=1}^{N}
z_i
\left(
v_i
+ \lambda_n n_i
- \lambda_d d_i
- \lambda_{\rho} \rho_i
\right),
$$

subject to

$$
\sum_i z_i e_i \le \bar{E},
\qquad
\sum_i z_i b_i \le \bar{B},
\qquad
\sum_i z_i \tau_i \le \bar{T},
\qquad
\rho_i \le 1 - R_{\min}.
$$

Here $v_i$ is science value, $n_i$ is novelty, $d_i$ is task urgency penalty, $e_i$ is expected energy cost, $b_i$ is required bandwidth, $\tau_i$ is execution time, and $\rho_i$ is the forecast reliability burden of task $i$. This makes resource budgeting an explicit optimization layer rather than a post hoc heuristic.

### F. Explainable Autonomy Layer

RHEA produces an explanation tuple for each committed high-level action:

$$
\mathcal{E}_t =
\left(
\kappa_t^{\text{task}},
\kappa_t^{\text{terrain}},
\kappa_t^{\text{resource}},
\kappa_t^{\text{reliability}},
\kappa_t^{\text{counterfactual}}
\right).
$$

The explanation is generated from a decomposed action score

$$
Q^{\text{hyb}}(\mathbf{s}, a)
=
Q^{\text{task}}
+ Q^{\text{info}}
- \lambda_h Q^{\text{hazard}}
- \lambda_e Q^{\text{energy}}
- \lambda_{\rho} Q^{\text{unrel}},
$$

where each term is logged separately. A counterfactual explanation is then constructed by comparing the chosen action $a^\star$ with the highest-scoring rejected alternative $\tilde{a}$:

$$
\Delta(a^\star, \tilde{a}) =
Q^{\text{hyb}}(\mathbf{s}, a^\star) - Q^{\text{hyb}}(\mathbf{s}, \tilde{a}).
$$

The exported rationale can therefore take the form: "choose `scan` instead of `move_east` because uncertainty reduction and reliability margin outweigh short-term path progress." This is useful for science operators, fault diagnosis, and post-mission auditing.

## Mathematical Formulation Summary

### Belief Dynamics

The partially observed environment is modeled as a belief MDP:

$$
b_{t+1} = \tau(b_t, a_t, o_{t+1}),
$$

where $b_t$ contains semantic occupancy, uncertainty, resource state, and subsystem reliability.

### Hybrid Policy

The operational policy is

$$
\pi(a_t \mid b_t)
\propto
\mathrm{MCTS}
\left(
b_t,\;
\pi_\theta,\;
V_\phi,\;
\widehat{F},\;
\widehat{U}
\right).
$$

The learned prior shapes search, while the planner enforces long-horizon constraint handling.

### Reliability Margin

The reliability margin is

$$
\delta_t = R_t - R_{\min}.
$$

If $\delta_t < 0$, the decision layer enters a guarded mode and restricts the action set to $\{\texttt{scan}, \texttt{replan}, \texttt{wait}\}$ or other low-risk actions.

## Algorithm Pseudocode

### Algorithm 1: Online Reliability-Aware Hybrid Autonomy

```text
Input:
  sensor observations o_t
  task set T
  resource budgets r_0
  learned prior π_θ, value V_φ
  reliability estimator f_ρ

Initialize belief b_0 and reliability vector q_0

for t = 0, 1, ..., T do
  1. Update world model:
       b_t <- BeliefUpdate(b_{t-1}, a_{t-1}, o_t)
  2. Estimate subsystem reliability:
       q_t <- f_ρ(o_t, diagnostics_t, resource_t)
  3. Schedule feasible mission tasks:
       T_t <- ResourceConstrainedScheduler(T, b_t, r_t, q_t)
  4. Run hybrid planner:
       for k = 1 to K simulations do
         expand tree using prior π_θ(a | b)
         simulate rollout with reliability-aware transition model
         backpropagate science, uncertainty, energy, and reliability return
       end for
  5. Select action:
       a_t <- argmax_a Score(b_t, a)
  6. Generate explanation:
       E_t <- Explain(a_t, rejected alternatives, score decomposition)
  7. Execute first action and update resources
  8. If reliability margin is violated:
       switch to guarded action subset and trigger replanning
end for

Return:
  mission outcome, trajectory, explanations, reliability trace
```

### Algorithm 2: Simulation-Based Training of the Hybrid Prior

```text
Input:
  terrain generator G
  task generator Ψ
  initial policy prior π_θ
  value network V_φ

for episode = 1, ..., N do
  1. Sample terrain, hazards, and science targets from G and Ψ
  2. Roll out RHEA in the simulated environment
  3. Store tuples:
       (b_t, a_t, R_t^rollout, reliability trace, explanation tags)
  4. Update π_θ and V_φ using PPO-style or actor-critic loss
  5. Calibrate reliability estimator with observed subsystem failures
end for

Return:
  trained prior, value model, and calibrated reliability model
```

## Experimental Design

### A. Objectives

The experiments should answer four questions:

1. Does hybrid planning plus learning improve exploration success in unknown terrain?
2. Does reliability-aware decision making reduce failure under degraded sensing and resources?
3. Does mission-level resource allocation improve science return per unit energy?
4. Are the generated explanations faithful and useful for human analysis?

### B. Baselines

The minimum baseline suite should include:

1. **A*-baseline**: weighted A* over a deterministic cost map with no uncertainty reasoning and no learned prior.
2. **Rule-based baseline**: handcrafted state machine with thresholds for hazard, battery, and scan triggering.
3. **Hybrid without reliability**: same planner and learner as RHEA, but remove reliability terms from reward and constraints.
4. **Hybrid without explanations**: same decision policy as RHEA, but no score decomposition or counterfactual rationale.

### C. Environment Protocol

Use procedurally generated lunar or Martian terrains with controlled difficulty:

1. Map sizes: `64 x 64`, `96 x 96`, and `128 x 128`.
2. Hazard density: low, medium, and high crater or rock coverage.
3. Observation quality: nominal, LiDAR dropout, vision blur, and mixed degradation.
4. Resource regimes: nominal, energy-limited, thermal-limited, and compute-limited.
5. Mission types: single-goal navigation, multi-target science collection, and relay-plus-science mixed missions.

For each condition, evaluate at least `30` random seeds and report mean plus standard deviation.

### D. Metrics

Recommended evaluation metrics are:

1. Mission success rate.
2. Science return.
3. Energy consumed per successful mission.
4. Path efficiency relative to shortest feasible path.
5. Mean risk and collision rate.
6. Reliability violation rate, defined as the fraction of steps with $R_t < R_{\min}$.
7. Uncertainty reduction per meter traveled.
8. Explanation fidelity, measured as agreement between logged score decomposition and actual action ranking.
9. Reliability calibration, measured with Brier score or expected calibration error.

### E. Ablation Studies

The paper should include at least the following ablations:

1. Remove the learned prior and use pure reliability-aware search.
2. Remove reliability penalties and keep hybrid planning-learning only.
3. Remove mission-level resource optimization and use greedy task ordering.
4. Remove explanation generation and measure only policy performance.
5. Replace semantic occupancy with geometric occupancy only.

These ablations isolate which gains come from learning, reliability, resource awareness, and semantic perception.

### F. Statistical Analysis

Use paired statistical tests across matched seeds. Report confidence intervals for success rate and science return. For explanation fidelity and calibration metrics, report both global averages and failure-case subsets, since reliability-aware methods are expected to show their strongest advantage in degraded conditions.

## Expected Positioning Against Baselines

Relative to A*, RHEA should improve robustness in partially observed maps because it reasons over uncertainty, active sensing, and subsystem reliability rather than treating the map as fixed. Relative to a rule-based controller, RHEA should improve task prioritization and long-horizon coordination because resource allocation and failure risk are optimized jointly instead of through static thresholds.

The central claim suitable for an IEEE Transactions on Robotics submission is therefore:

> A reliability-aware hybrid autonomy stack can increase science productivity and mission robustness in unknown planetary terrain while producing auditable explanations for each high-level decision.
