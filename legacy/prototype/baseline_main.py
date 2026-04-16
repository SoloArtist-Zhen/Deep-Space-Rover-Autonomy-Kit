
import numpy as np, json
from pathlib import Path
from modules.terrain import lunar_terrain, slope_map, slip_risk, cost_map
from modules.planners import astar, rrt_star, aco_path, qlearn_nav
from modules.controllers import follow_path, energy_of_path
from modules.viz import plot_surface, heatmap, overlay_path, series, scatter_xy
from modules.multiagent import coverage_sim

OUT = Path("outputs"); OUT.mkdir(exist_ok=True, parents=True)

def choose_points(H,W):
    start=(int(0.1*H), int(0.1*W))
    goal=(int(0.85*H), int(0.85*W))
    return start, goal

def run_all():
    Z = lunar_terrain(size=128, seed=10)
    slope, gx, gy = slope_map(Z, scale_xy=1.0)
    risk = slip_risk(Z, slope)
    cost = cost_map(Z, slope, alpha=0.7, beta=0.3)

    plot_surface(Z, "High-fidelity Lunar Terrain (synthetic)", OUT/"fig01_terrain_surface.png")
    heatmap(slope, "Slope map (gradient magnitude)", OUT/"fig02_slope.png")
    heatmap(risk, "Slip risk map", OUT/"fig03_risk.png")
    heatmap(cost, "Traversal cost map", OUT/"fig04_cost.png")

    H,W = Z.shape
    start, goal = choose_points(H,W)

    path_astar = astar(cost, start, goal, w=1.0)
    overlay_path(cost, path_astar, "Risk-aware A* over cost map", OUT/"fig05_astar_path.png")

    obs = risk > 0.75
    path_rrt = rrt_star(obs.astype(float), start, goal, iters=600, step=3, radius=6)
    overlay_path(obs.astype(float), path_rrt, "RRT* path over obstacle mask", OUT/"fig06_rrt_path.png")

    path_aco, conv = aco_path(cost, start, goal, n_ants=35, iters=22, evap=0.35, q=40.0)
    overlay_path(cost, path_aco, "ACO path over cost map", OUT/"fig07_aco_path.png")
    series(conv, title="ACO convergence (best cost vs. iteration)", yl="Path cost", xl="Iteration", path=OUT/"fig08_aco_convergence.png")

    path_ql, rets = qlearn_nav(cost, start, goal, episodes=60, alpha=0.7, gamma=0.95, eps=0.2)
    overlay_path(cost, path_ql, "Q-learning greedy policy path", OUT/"fig09_qlearn_path.png")
    series(rets, title="Q-learning return per episode", yl="Return", xl="Episode", path=OUT/"fig10_qlearn_return.png")

    candidates = [("A*", path_astar), ("RRT*", path_rrt), ("ACO", path_aco), ("QL", path_ql)]
    metrics = []
    for name, pth in candidates:
        E, prof = energy_of_path(cost, pth, slope, k_e=1.0)
        metrics.append((name, E, len(pth), prof))
    metrics = [m for m in metrics if m[2]>0]
    best = sorted(metrics, key=lambda x: x[1])[0] if metrics else ("A*",1e9,0,None)
    best_name, best_E, _, _ = best
    best_path = dict(candidates)[best_name]

    follow = follow_path(Z, best_path, v=1.0, kp_yaw=1.0, dt=0.1)
    if len(follow["t"])>0:
        att = follow["att"]; t = follow["t"]
        series(att[:,0], t, title="Attitude: roll", yl="rad", xl="s", path=OUT/"fig11_roll.png")
        series(att[:,1], t, title="Attitude: pitch", yl="rad", xl="s", path=OUT/"fig12_pitch.png")
        series(att[:,2], t, title="Attitude: yaw", yl="rad", xl="s", path=OUT/"fig13_yaw.png")

    risk_along = [risk[p] for p in best_path] if len(best_path)>0 else []
    series(risk_along, title="Slip risk along best path", yl="risk", xl="step", path=OUT/"fig14_risk_along.png")

    lens=[]; Eng=[]; Riskm=[]
    for name,pth in candidates:
        if len(pth)==0: continue
        E,_ = energy_of_path(cost, pth, slope)
        r = np.mean([risk[p] for p in pth])
        lens.append(len(pth)); Eng.append(E); Riskm.append(r)
    scatter_xy(lens, Eng, title="Pareto: path length vs energy", xl="Length (steps)", yl="Energy", path=OUT/"fig15_pareto_len_energy.png")
    scatter_xy(Eng, Riskm, title="Pareto: energy vs risk", xl="Energy", yl="Mean risk", path=OUT/"fig16_pareto_energy_risk.png")

    seeds = [(int(0.1*H), int(0.9*W)), (int(0.5*H), int(0.5*W)), (int(0.9*H), int(0.1*W)), (int(0.85*H), int(0.85*W))]
    paths, covered, cov_curve, lab = coverage_sim(cost, seeds, steps=500, stride=5)
    heatmap(lab, "Voronoi region partition (4 rovers)", OUT/"fig17_voronoi.png")
    heatmap(covered.astype(float), "Final coverage map", OUT/"fig18_coverage.png")
    series(cov_curve, title="Coverage over time (multi-rover)", yl="covered ratio", xl="step", path=OUT/"fig19_coverage_curve.png")

    bench = {
        "best_planner": best_name,
        "energy_best": float(best_E),
        "pareto": {
            "length": list(map(int,lens)),
            "energy": list(map(float,Eng)),
            "risk_mean": list(map(float,Riskm))
        },
        "coverage_final": float(cov_curve[-1] if len(cov_curve)>0 else 0.0)
    }
    (OUT/"benchmarks.json").write_text(json.dumps(bench, indent=2))

if __name__=="__main__":
    run_all()
