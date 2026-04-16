
import numpy as np
from modules.planners import astar

def voronoi_partition(seeds, H, W):
    lab = np.zeros((H,W), dtype=int)
    for i in range(H):
        for j in range(W):
            dmin=1e9; idx=0
            for k,(x,y) in enumerate(seeds):
                d = (i-x)**2 + (j-y)**2
                if d<dmin: dmin=d; idx=k
            lab[i,j]=idx
    return lab

def coverage_sim(cost, seeds, steps=500, stride=5):
    H,W = cost.shape
    lab = voronoi_partition(seeds, H, W)
    targets = []
    for r in range(len(seeds)):
        pts = []
        for i in range(0,H,stride):
            for j in range(0,W,stride):
                if lab[i,j]==r: pts.append((i,j))
        targets.append(pts)
    covered = np.zeros((H,W), dtype=bool)
    agents = [seeds[k] for k in range(len(seeds))]
    paths = [[] for _ in agents]
    cov_curve=[]
    for t in range(steps):
        for k in range(len(agents)):
            if len(targets[k])==0: continue
            au = agents[k]
            dmin=1e9; tgt=None; idx=None
            for ti,p in enumerate(targets[k]):
                if covered[p]: continue
                d = abs(p[0]-au[0])+abs(p[1]-au[1])
                if d<dmin: dmin=d; tgt=p; idx=ti
            if tgt is None: continue
            pth = astar(cost, agents[k], tgt, w=1.0)
            if len(pth)>2: pth = pth[1:3]
            if len(pth)>0:
                agents[k] = pth[-1]
                paths[k].extend(pth)
                covered[agents[k]] = True
        cov_curve.append(covered.mean())
    return paths, covered, np.array(cov_curve), lab
