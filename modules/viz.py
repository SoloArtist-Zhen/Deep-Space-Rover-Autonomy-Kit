
import numpy as np, matplotlib.pyplot as plt

def ensure_ax3d():
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
    fig = plt.figure(figsize=(6,4.5))
    ax = fig.add_subplot(111, projection='3d')
    return fig, ax

def plot_surface(Z, title, path=None):
    fig, ax = ensure_ax3d()
    H,W = Z.shape
    X,Y = np.meshgrid(np.arange(W), np.arange(H))
    ax.plot_surface(X, Y, Z, linewidth=0, antialiased=False)
    ax.set_title(title)
    if path:
        fig.tight_layout(); fig.savefig(path, dpi=160)
        plt.close(fig)

def heatmap(A, title, path=None):
    fig, ax = plt.subplots(figsize=(6,4.5))
    im = ax.imshow(A, origin='lower')
    ax.set_title(title); fig.colorbar(im, ax=ax, fraction=0.046, pad=0.04)
    if path:
        fig.tight_layout(); fig.savefig(path, dpi=170); plt.close(fig)

def overlay_path(Z, path_xy, title, path=None):
    fig, ax = plt.subplots(figsize=(6,4.5))
    im = ax.imshow(Z, origin='lower')
    if len(path_xy)>0:
        xs=[p[1] for p in path_xy]; ys=[p[0] for p in path_xy]
        ax.plot(xs, ys)
        ax.plot(xs[0], ys[0], marker='o')
        ax.plot(xs[-1], ys[-1], marker='s')
    ax.set_title(title); 
    fig.colorbar(im, ax=ax, fraction=0.046, pad=0.04)
    if path:
        fig.tight_layout(); fig.savefig(path, dpi=170); plt.close(fig)

def series(y, x=None, title="", yl="", xl="", path=None):
    import numpy as np, matplotlib.pyplot as plt
    if x is None: x = np.arange(len(y))
    fig, ax = plt.subplots(figsize=(6,3.2))
    ax.plot(x, y)
    ax.set_title(title); ax.set_ylabel(yl); ax.set_xlabel(xl); ax.grid(True)
    if path:
        fig.tight_layout(); fig.savefig(path, dpi=170); plt.close(fig)

def scatter_xy(x, y, title="", xl="", yl="", path=None):
    fig, ax = plt.subplots(figsize=(5.5,4.2))
    ax.scatter(x,y)
    ax.set_title(title); ax.set_xlabel(xl); ax.set_ylabel(yl); ax.grid(True)
    if path:
        fig.tight_layout(); fig.savefig(path, dpi=170); plt.close(fig)
