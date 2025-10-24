
import numpy as np

def _lerp(a,b,t): return a + t*(b-a)

def _fade(t): return t*t*t*(t*(t*6-15)+10)

def _grad(hash_, x, y):
    h = hash_ & 3
    u = x if h < 2 else y
    v = y if h < 2 else x
    return (u if (h&1)==0 else -u) + (v if (h&2)==0 else -v)

def perlin2D(shape=(128,128), res=(8,8), seed=0, amp=1.0):
    np.random.seed(seed)
    (h, w) = shape
    delta = (res[0]/h, res[1]/w)
    grid = np.random.randint(0, 255, (res[0]+1, res[1]+1), dtype=np.int32)
    noise = np.zeros((h,w))
    for i in range(h):
        for j in range(w):
            x = i*delta[0]; y = j*delta[1]
            xi = int(x); yi = int(y)
            xf = x - xi; yf = y - yi
            u = _fade(xf); v = _fade(yf)
            n00 = _grad(grid[xi, yi], xf, yf)
            n01 = _grad(grid[xi, yi+1], xf, yf-1)
            n10 = _grad(grid[xi+1, yi], xf-1, yf)
            n11 = _grad(grid[xi+1, yi+1], xf-1, yf-1)
            x1 = _lerp(n00, n10, u)
            x2 = _lerp(n01, n11, u)
            noise[i,j] = _lerp(x1, x2, v)
    noise = (noise - noise.min())/(noise.max()-noise.min()+1e-12)
    return amp*noise

def add_gaussian_bumps(Z, num=20, scale=6, depth=0.6, seed=1, invert=False):
    np.random.seed(seed)
    H, W = Z.shape
    xs = np.arange(H); ys = np.arange(W)
    X,Y = np.meshgrid(xs, ys, indexing='ij')
    for _ in range(num):
        cx = np.random.randint(0,H)
        cy = np.random.randint(0,W)
        sx = np.random.randint(2,scale)
        sy = np.random.randint(2,scale)
        A = (np.random.rand()*0.8+0.2)*depth
        if invert: A = -A
        bump = A*np.exp(-((X-cx)**2/(2*sx**2) + (Y-cy)**2/(2*sy**2)))
        Z += bump
    Z = (Z - Z.min())/(Z.max()-Z.min()+1e-12)
    return Z

def lunar_terrain(size=128, seed=0):
    Z = perlin2D((size,size),(8,8),seed=seed,amp=0.6)
    Z += 0.5*perlin2D((size,size),(16,16),seed=seed+1,amp=0.4)
    Z = (Z - Z.min())/(Z.max()-Z.min()+1e-12)
    Z = add_gaussian_bumps(Z, num=12, scale=10, depth=0.7, seed=seed+2, invert=True)
    Z = add_gaussian_bumps(Z, num=25, scale=5, depth=0.5, seed=seed+3, invert=False)
    Z = (Z - Z.min())/(Z.max()-Z.min()+1e-12)
    return Z

def slope_map(Z, scale_xy=1.0):
    gx, gy = np.gradient(Z*1.0, scale_xy, scale_xy)
    slope = np.sqrt(gx*gx + gy*gy)
    return slope, gx, gy

def slip_risk(Z, slope, rock_thresh=0.75):
    rough = np.abs(Z - 0.5)
    risk = 0.6*slope/ (slope.max()+1e-12) + 0.4*(rough/ (rough.max()+1e-12))
    risk[Z>rock_thresh] *= 1.5
    return (risk - risk.min())/(risk.max()-risk.min()+1e-12)

def cost_map(Z, slope, alpha=0.7, beta=0.3):
    c = alpha*(slope/ (slope.max()+1e-12)) + beta* ((Z - Z.min())/(Z.max()-Z.min()+1e-12))
    return (c - c.min())/(c.max()-c.min()+1e-12)
