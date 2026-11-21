import time
import numpy as np
import matplotlib.pyplot as plt
from scipy.linalg import solve_continuous_are

# --- Parametres du robot ---
I = 0.0225      # inertie du pendule autour du pivot (kg m^2)
J = 0.00027     # inertie de la roue (kg m^2)
mgl = 0.88      # m*g*l (N m)
U_MAX = 0.5     # couple maximal (N m)
DT, T = 0.005, 10.0
PUSH, PUSH_T = 1.0, 3.0   # poussee de 1 N m pendant 50 ms a t = 3 s
N_SEEDS = 10              # nombre d'essais par condition

# --- LQR ---
A = np.array([[0, 1, 0], [mgl / I, 0, 0], [-mgl / I, 0, 0]])
B = np.array([[0], [-1 / I], [1 / J + 1 / I]])
Q = np.diag([100.0, 10.0, 0.1])
R = np.array([[100.0]])
P = solve_continuous_are(A, B, Q, R)
K = (np.linalg.inv(R) @ B.T @ P).flatten()

# --- PID (PD) ---
Kp = 2 * mgl
Kd = 2 * 0.7 * np.sqrt(I * (Kp - mgl))

def pid(th, w, wh):
    return Kp * th + Kd * w

def lqr(th, w, wh):
    return -(K[0] * th + K[1] * w + K[2] * wh)

def run(ctrl, noise_deg=0.0, delay_ms=0, seed=0):
    """Retourne True si le pendule ne depasse jamais 30 degres."""
    rng = np.random.default_rng(seed)
    sig_th = np.deg2rad(noise_deg)      # bruit sur l'angle (rad)
    sig_w = 0.04 * noise_deg            # bruit sur la vitesse (rad/s)
    n_delay = int(round(delay_ms / (DT * 1000)))
    buf = [0.0] * n_delay
    th, w, wh = np.deg2rad(5.0), 0.0, 0.0
    for k in range(int(T / DT)):
        t = k * DT
        th_m, w_m = th, w
        if noise_deg > 0:
            th_m = th + rng.normal(0, sig_th)
            w_m = w + rng.normal(0, sig_w)
        u_cmd = float(np.clip(ctrl(th_m, w_m, wh), -U_MAX, U_MAX))
        if n_delay > 0:
            buf.append(u_cmd)
            u = buf.pop(0)
        else:
            u = u_cmd
        d = PUSH if PUSH_T <= t < PUSH_T + 0.05 else 0.0
        acc = (mgl * np.sin(th) - u + d) / I
        wacc = u / J - acc
        w += acc * DT
        th += w * DT
        wh += wacc * DT
        if abs(np.rad2deg(th)) > 30:
            return False
    return True

def success_rate(ctrl, **kw):
    return float(np.mean([run(ctrl, seed=s, **kw) for s in range(N_SEEDS)]))

def margin(levels, rates):
    """Plus grand niveau avec 100 % de reussite (en partant de 0)."""
    best = None
    for lv, r in zip(levels, rates):
        if r < 1.0:
            break
        best = lv
    return best

t0 = time.time()
controllers = {"PID": pid, "LQR": lqr}

# --- Experience 1 : bruit de capteur (sans retard) ---
noise_levels = [0, 2, 4, 6, 8, 12, 16, 24]   # ecart-type du bruit sur l'angle (deg)
noise_res = {n: [success_rate(c, noise_deg=lv) for lv in noise_levels]
             for n, c in controllers.items()}

# --- Experience 2 : retard (avec un petit bruit de 0,5 deg) ---
delays = list(range(0, 95, 5))               # retard (ms)
delay_res = {n: [success_rate(c, noise_deg=0.5, delay_ms=d) for d in delays]
             for n, c in controllers.items()}

print("Temps de calcul : %.0f s" % (time.time() - t0))
for n in controllers:
    print(n, "- bruit maximal avec 100 pourcent de reussite :", margin(noise_levels, noise_res[n]), "deg")
    print(n, "- marge de retard (100 pourcent de reussite) :", margin(delays, delay_res[n]), "ms")

fig, ax = plt.subplots(1, 2, figsize=(11, 4))
for n in controllers:
    ax[0].plot(noise_levels, noise_res[n], marker="o", label=n)
    ax[1].plot(delays, delay_res[n], marker="o", label=n)
ax[0].set_xlabel("sensor noise on angle (deg, std)")
ax[0].set_ylabel("success rate")
ax[0].set_title("Noise tolerance")
ax[1].set_xlabel("actuation delay (ms)")
ax[1].set_ylabel("success rate")
ax[1].set_title("Delay tolerance")
for a in ax:
    a.set_ylim(-0.05, 1.05)
    a.legend()
plt.tight_layout()
plt.savefig("results_noise_delay.png", dpi=150)
plt.show()
