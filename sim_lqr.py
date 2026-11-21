import numpy as np
import matplotlib.pyplot as plt
from scipy.linalg import solve_continuous_are

# --- Parametres du robot (calcules a partir de CoppeliaSim) ---
I = 0.0225      # inertie du pendule autour du pivot (kg m^2)
J = 0.00027     # inertie de la roue (kg m^2)
mgl = 0.88      # m*g*l (N m)
U_MAX = 0.5     # couple maximal du moteur (N m)
DT, T = 0.005, 10.0
PUSH, PUSH_T = 1.0, 3.0   # poussee de 1 N m pendant 50 ms a t = 3 s

# --- Modele lineaire pour le LQR : x = [angle, vitesse angulaire, vitesse roue] ---
A = np.array([[0, 1, 0], [mgl / I, 0, 0], [-mgl / I, 0, 0]])
B = np.array([[0], [-1 / I], [1 / J + 1 / I]])
Q = np.diag([100.0, 10.0, 0.1])
R = np.array([[100.0]])
P = solve_continuous_are(A, B, Q, R)
K = (np.linalg.inv(R) @ B.T @ P).flatten()
print("Gains LQR K =", K)

# --- Gains PID (PD) ---
Kp = 2 * mgl
Kd = 2 * 0.7 * np.sqrt(I * (Kp - mgl))

def pid(th, w, wh):
    return Kp * th + Kd * w

def lqr(th, w, wh):
    return -(K[0] * th + K[1] * w + K[2] * wh)

def run(ctrl):
    th, w, wh = np.deg2rad(5.0), 0.0, 0.0
    L = {"t": [], "th": [], "u": [], "wh": []}
    for k in range(int(T / DT)):
        t = k * DT
        u = float(np.clip(ctrl(th, w, wh), -U_MAX, U_MAX))
        d = PUSH if PUSH_T <= t < PUSH_T + 0.05 else 0.0
        acc = (mgl * np.sin(th) - u + d) / I
        wacc = u / J - acc
        w += acc * DT
        th += w * DT
        wh += wacc * DT
        L["t"].append(t)
        L["th"].append(np.rad2deg(th))
        L["u"].append(u)
        L["wh"].append(wh)
    return {key: np.array(v) for key, v in L.items()}

def metrics(L):
    after = L["t"] >= PUSH_T
    th_a, t_a = L["th"][after], L["t"][after]
    out = np.where(np.abs(th_a) > 1.0)[0]
    settle = 0.0 if len(out) == 0 else float(t_a[out[-1]] - PUSH_T)
    return {
        "success": bool(np.max(np.abs(L["th"])) < 30),
        "settling_s": round(settle, 2),
        "max_angle_deg": round(float(np.max(np.abs(th_a))), 2),
        "rms_torque": round(float(np.sqrt(np.mean(L["u"] ** 2))), 4),
        "final_wheel_speed": round(float(L["wh"][-1]), 1),
    }

results = {"PID": run(pid), "LQR": run(lqr)}
for name, L in results.items():
    print(name, metrics(L))

fig, ax = plt.subplots(3, 1, sharex=True, figsize=(8, 8))
for name, L in results.items():
    ax[0].plot(L["t"], L["th"], label=name)
    ax[1].plot(L["t"], L["u"], label=name)
    ax[2].plot(L["t"], L["wh"], label=name)
ax[0].set_ylabel("angle (deg)")
ax[1].set_ylabel("torque (N m)")
ax[2].set_ylabel("wheel speed (rad/s)")
ax[2].set_xlabel("time (s)")
ax[0].legend()
plt.tight_layout()
plt.savefig("results_pid_vs_lqr.png", dpi=150)
plt.show()
