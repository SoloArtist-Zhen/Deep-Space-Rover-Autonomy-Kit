
import numpy as np, heapq, math

def astar(cost, start, goal, w=1.0):
    H,W = cost.shape
    def h(p):
        return math.hypot(p[0]-goal[0], p[1]-goal[1])
    g = np.full((H,W), np.inf)
    parent = -np.ones((H,W,2), dtype=int)
    g[start] = 0
    pq = [(h(start), start)]
    visited = np.zeros((H,W), dtype=bool)
    while pq:
        f,p = heapq.heappop(pq)
        if visited[p]: continue
        visited[p] = True
        if p == goal: break
        for dx,dy in [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]:
            nx,ny = p[0]+dx, p[1]+dy
            if nx<0 or ny<0 or nx>=H or ny>=W: continue
            step = math.hypot(dx,dy)
            ng = g[p] + step*(1 + cost[nx,ny])
            if ng < g[nx,ny]:
                g[nx,ny] = ng
                parent[nx,ny] = p
                heapq.heappush(pq, (ng + w*h((nx,ny)), (nx,ny)))
    if parent[goal][0] == -1: return []
    path = [goal]; cur = goal
    while tuple(cur) != start:
        cur = tuple(parent[cur]); path.append(cur)
    path.reverse()
    return path

def rrt_star(obs_map, start, goal, iters=600, step=3, radius=6):
    H,W = obs_map.shape
    nodes = [start]
    parent = {start: None}
    cost = {start: 0.0}
    def dist(a,b): return math.hypot(a[0]-b[0], a[1]-b[1])
    def steer(a,b,step):
        if dist(a,b) <= step: return b
        th = math.atan2(b[1]-a[1], b[0]-a[0])
        return (int(round(a[0]+step*math.cos(th))), int(round(a[1]+step*math.sin(th))))
    def collision(a,b):
        x0,y0 = a; x1,y1 = b
        n=max(abs(x1-x0),abs(y1-y0))
        for i in range(n+1):
            xi = int(round(x0+(x1-x0)*i/n)); yi=int(round(y0+(y1-y0)*i/n))
            if xi<0 or yi<0 or xi>=H or yi>=W: return True
            if obs_map[xi,yi]>0.6: return True
        return False
    for _ in range(iters):
        rnd = (np.random.randint(0,H), np.random.randint(0,W)) if np.random.rand()>0.1 else goal
        dmin=1e9; nearest=nodes[0]
        for nd in nodes:
            d=dist(nd,rnd)
            if d<dmin: dmin=d; nearest=nd
        new = steer(nearest, rnd, step)
        if new in parent: continue
        if collision(nearest,new): continue
        best_p = nearest; best_c = cost[nearest] + dist(nearest,new)
        for nd in nodes:
            if dist(nd,new) < radius and not collision(nd,new):
                cc = cost[nd] + dist(nd,new)
                if cc < best_c:
                    best_c = cc; best_p = nd
        parent[new]=best_p; cost[new]=best_c; nodes.append(new)
        for nd in nodes:
            if nd==new: continue
            if dist(nd,new)<radius and not collision(nd,new):
                cc = cost[new]+dist(new,nd)
                if cc < cost.get(nd,1e9):
                    parent[nd]=new; cost[nd]=cc
        if dist(new,goal)<radius and not collision(new,goal):
            parent[goal]=new; cost[goal]=cost[new]+dist(new,goal); nodes.append(goal); break
    if goal not in parent: return []
    path=[goal]; cur=goal
    while cur is not None:
        cur = parent[cur]; 
        if cur is not None: path.append(cur)
    path.reverse()
    return path

def aco_path(cost, start, goal, n_ants=40, iters=25, evap=0.4, q=50.0):
    H,W = cost.shape
    tau = np.ones((H,W))*1e-3
    def nb(p):
        x,y=p; c=[]
        for dx,dy in [(-1,0),(1,0),(0,-1),(0,1)]:
            nx,ny=x+dx,y+dy
            if 0<=nx<H and 0<=ny<W: c.append((nx,ny))
        return c
    best=None; best_len=1e9; conv=[]
    for it in range(iters):
        paths=[]; lengths=[]
        for k in range(n_ants):
            p=start; trace=[p]; L=0.0; fail=False
            for _ in range(H*W//2):
                choices=nb(p)
                if len(choices)==0: fail=True; break
                w=[(tau[c]+1e-9)/(1.0+cost[c]) for c in choices]
                s=sum(w); w=[x/s for x in w]
                idx = np.random.choice(len(choices), p=w)
                nxt=choices[idx]
                L += 1.0 + cost[nxt]
                trace.append(nxt); p=nxt
                if p==goal: break
            if p!=goal: fail=True
            if not fail:
                paths.append(trace); lengths.append(L)
                if L<best_len: best_len=L; best=trace
        tau = (1-evap)*tau
        for tr,L in zip(paths,lengths):
            dep = q/(L+1e-9)
            for pt in tr: tau[pt]+=dep
        conv.append(best_len if best_len<1e9 else np.nan)
    return best if best is not None else [], np.array(conv)

def qlearn_nav(cost, start, goal, episodes=60, alpha=0.7, gamma=0.95, eps=0.2):
    H,W = cost.shape
    Q = np.zeros((H,W,4))
    acts=[(-1,0),(1,0),(0,-1),(0,1)]
    def step(s,a):
        x,y=s; dx,dy=acts[a]; nx,ny=x+dx,y+dy
        if nx<0 or ny<0 or nx>=H or ny>=W: return s, -1.0, False
        r = - (0.5 + cost[nx,ny])
        ns = (nx,ny)
        if ns==goal: return ns, 5.0, True
        return ns, r, False
    returns=[]
    for ep in range(episodes):
        s = start; done=False; G=0.0; steps=0
        while not done and steps< H*W//2:
            if np.random.rand()<eps: a=np.random.randint(4)
            else: a=int(np.argmax(Q[s[0],s[1]]))
            ns, r, done = step(s,a)
            G += r
            Q[s[0],s[1],a] = (1-alpha)*Q[s[0],s[1],a] + alpha*(r + gamma*np.max(Q[ns[0],ns[1]]))
            s = ns; steps+=1
        returns.append(G)
    s=start; path=[s]; visited=set([s])
    for _ in range(H*W//3):
        a=int(np.argmax(Q[s[0],s[1]]))
        s=(s[0]+acts[a][0], s[1]+acts[a][1])
        if s==path[-1] or s in visited: break
        path.append(s); visited.add(s)
        if s==goal: break
    if path[-1]!=goal: path=[]
    return path, np.array(returns)
