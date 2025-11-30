# D:\pytorch learning\venv\Scripts\python.exe pytorch_DuarteFP64.py

import numpy as np
import torch
import time

torch.set_default_dtype(torch.float64)
torch.set_default_device("cuda")  # use torch.set_default_device("cup") for computation on cpu

nB = 1000 # Grid size for bond holdings

# --- Load Tauchen output from file ---
logy_grid = np.loadtxt("logy_grid.txt")
Py = np.loadtxt("P.txt")

logy_grid = torch.tensor(logy_grid, dtype=torch.float64)
Py = torch.tensor(Py, dtype=torch.float64)

# Grid size for endowment
ny = len(logy_grid)

# Model parameters
β = torch.tensor(0.953, dtype=torch.float64)
γ = torch.tensor(2.0, dtype=torch.float64)
r = torch.tensor(0.017, dtype=torch.float64)
θ = torch.tensor(0.282, dtype=torch.float64)

MAX_ITER = 10000  # Maximum number of iterations
tol = 1e-7  # Tolerance for value iteration

# Grid for bond holdings
Bgrid = torch.linspace(-.45, .45, nB, dtype=torch.float64)
ygrid = torch.exp(logy_grid)

# Initialize value functions and bond price
Vd = torch.zeros([ny, 1], dtype=torch.float64)
Vc = torch.zeros((ny, nB), dtype=torch.float64)
V = torch.zeros((ny, nB), dtype=torch.float64)
Q = torch.ones((ny, nB), dtype=torch.float64) * torch.tensor(0.95, dtype=torch.float64)

# Reshape grids to facilitate broacasting operations. Dimensions are:
# 0: endowment dimension
# 1: Bondholdings dimension
# 2: Bondholdings in the next period dimension
y = torch.reshape(ygrid, [-1, 1, 1])
B = torch.reshape(Bgrid, [1, -1, 1])
Bnext = torch.reshape(Bgrid, [1, 1, -1])

zero_ind = nB // 2  # Index of zero bond holdings

# Utility function
def u(c):
    return c**(1 - γ) / (1 - γ)

# Endowment in autarky
ymean = torch.mean(ygrid)
def_y = torch.min(0.969 * ymean, ygrid)

def iterate(V, Vc, Vd, Q):
        EV = torch.matmul(Py, V)
        EVd = torch.matmul(Py, Vd)
        EVc = torch.matmul(Py, Vc)

        # bellman target for Vd
        Vd_target = u(def_y) + β * (θ * EVc[:, zero_ind] + (1 - θ) * EVd[:, 0])
        Vd_target = torch.reshape(Vd_target, [-1, 1])
        
        # bond price next period
        Qnext = torch.reshape(Q, [ny, 1, nB])

        # Bellman target for Vc
        c = torch.relu(y - Qnext * Bnext + B)
        EV = torch.reshape(EV, [ny, 1, nB])
        m = u(c) + β * EV
        Vc_target = torch.max(m, dim=2, out=None)[0]

        # Update bond price--- risk neutral pricing
        default_states = (Vd_target > Vc_target).double()
        default_prob = torch.matmul(Py, default_states)
        Q_target = (1 - default_prob) / (1 + r)

        V_target = torch.max(Vc_target, Vd_target)

        return V_target, Vc_target, Vd_target, Q_target, default_prob

iterate = torch.jit.trace(iterate, (V, Vc, Vd, Q))  # Jit compilation


V_ = V.cpu().numpy()


start_time = time.time()

for counter in range(MAX_ITER):
    
    V, Vc, Vd, Q, default_prob = iterate(V, Vc, Vd, Q)
    
    if counter % 50 == 0 and counter > 10:
        
        V_new_ = V.cpu().numpy()
        dist = np.abs(V_ - V_new_).max()
        V_ = V_new_

        elapsed = time.time() - start_time
        print(f"{counter} ~ diff = {dist:.8e} ~ elapsed: {elapsed:.3f}s")
        
        if dist < tol:
            print("Converged in", counter, "iterations. Total time:", time.time() - start_time, "seconds.")
            break

V_np = V.cpu().numpy()
Q_np = Q.cpu().numpy()

np.savetxt("V.txt", V_np, delimiter='\t', fmt="%.10e")
np.savetxt("Q.txt", Q_np, delimiter='\t', fmt="%.10e")

