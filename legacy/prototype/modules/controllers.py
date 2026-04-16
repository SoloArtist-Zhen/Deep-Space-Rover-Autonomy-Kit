
import numpy as np

def gradient_at(Z, p):
    x,y=p
    x=int(np.clip(x,1,Z.shape[0]-2)); y=int(np.clip(y,1,Z.shape[1]-2))
    gx = (Z[x+1,y]-Z[x-1,y])/2.0
    gy = (Z[x,y+1]-Z[x,y-1])/2.0
    return gx,gy

def attitude_from_slope(gx,gy):
    roll = np.arctan2(gy, 1.0)
    pitch = np.arctan2(gx, 1.0)
    yaw = 0.0
    return roll, pitch, yaw

def follow_path(Z, path, v=1.0, kp_yaw=1.2, dt=0.1):
    if len(path)<2: return {"t":[], "att":[], "pos":[]}
    pos_tr=[]; att_tr=[]; tvec=[]
    yaw=0.0
    for k,(x,y) in enumerate(path):
        if k < len(path)-1:
            nx,ny = path[k+1]
            hd = np.arctan2(ny-y, nx-x)
        else:
            hd = yaw
        e = (hd - yaw + np.pi)%(2*np.pi)-np.pi
        yaw += kp_yaw*e*dt
        gx,gy = gradient_at(Z,(x,y))
        roll,pitch,_ = attitude_from_slope(gx,gy)
        pos_tr.append((x,y)); att_tr.append((roll,pitch,yaw)); tvec.append(k*dt)
    out = {"t":np.array(tvec), "att":np.array(att_tr), "pos":np.array(pos_tr)}
    return out

def energy_of_path(cost, path, slope, k_e=1.0):
    if len(path)<2: return 1e9, None
    E=0.0; prof=[]
    for (x,y) in path:
        c = cost[x,y] + 0.2*slope[x,y]
        E += (1.0 + c)
        prof.append(E)
    return E, np.array(prof)
