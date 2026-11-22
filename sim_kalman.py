import time
import numpy as np
import matplotlib.pyplot as plt
from scipy.linalg import solve_continuous_are

# --- Robot parameters ---
I = 0.0225      # pendulum inertia about the pivot (kg m^2)
J = 0.00027     # wheel inertia (kg m^2)
mgl = 0.88      # m*g*l (N m)
U_MAX = 0.5     # torque limit (N m)
DT, T = 0.005, 10.0
PUSH, PUSH_T = 1.0, 3.0   # push of 1 N m for 50 ms at t = 3 s
N_SEEDS = 10

# --- Linear model: x = [angle, angular velocity, wheel speed] ---
A = np.array([[0, 1, 0], [mgl / I, 0, 0], [-mgl / I, 0, 0]])
B = np.array([[0], [-1 / I], [1 / J + 1 / I]])
Ad = np.eye(3) + A * DT      # discrete model used by the filter
Bd = B * DT

# --- LQR ---
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

# --- Kalman filter tuning (process noise) ---
Q_KF = np.diag([1e-6, 0.1, 0.1]) * DT

def run(ctrl, noise_deg=0.0, delay_ms=0, seed=0, use_kf=False):
    """Returns True if the pendulum never exceeds 30 degrees."""
    rng = np.random.default_rng(seed)
    sig = np.array([np.deg2rad(noise_deg), 0.04 * noise_deg, 0.01])
    R_KF = np.diag(np.maximum(sig, 1e-4) ** 2)
    n_delay = int(round(delay_ms / (DT * 1000)))
    buf = [0.0] * n_delay
    x = np.array([np.deg2rad(5.0), 0.0, 0.0])    # true state
    xh = np.zeros(3)                             # estimated state
    Pk = np.eye(3)
    u_prev = 0.0
    for k in range(int(T / DT)):
        t = k * DT
        z = x.copy()                             # measurement
        if noise_deg > 0:
            z[0] += rng.normal(0, sig[0])
            z[1] += rng.normal(0, sig[1])
        if use_kf:
            if k == 0:
                xh = z.copy()
            else:
                xh = Ad @ xh + (Bd * u_prev).flatten()   # prediction
                Pk = Ad @ Pk @ Ad.T + Q_KF
                G = Pk @ np.linalg.inv(Pk + R_KF)        # Kalman gain
                xh = xh + G @ (z - xh)                   # correction
                Pk = (np.eye(3) - G) @ Pk
            m = xh
        else:
            m = z
        u_cmd = float(np.clip(ctrl(m[0], m[1], m[2]), -U_MAX, U_MAX))
        if n_delay > 0:
            buf.append(u_cmd)
            u = buf.pop(0)
        else:
            u = u_cmd
        u_prev = u
        d = PUSH if PUSH_T <= t < PUSH_T + 0.05 else 0.0
        th, w, wh = x
        acc = (mgl * np.sin(th) - u + d) / I
        wacc = u / J - acc
        w2 = w + acc * DT
        x = np.array([th + w2 * DT, w2, wh + wacc * DT])
        if abs(np.rad2deg(x[0])) > 30:
            return False
    return True

def success_rate(ctrl, **kw):
    return float(np.mean([run(ctrl, seed=s, **kw) for s in range(N_SEEDS)]))

def margin(levels, rates):
    best = None
    for lv, r in zip(levels, rates):
        if r < 1.0:
            break
        best = lv
    return best

configs = {
    "PID": (pid, False),
    "PID + Kalman": (pid, True),
    "LQR": (lqr, False),
    "LQR + Kalman": (lqr, True),
}

t0 = time.time()
noise_levels = [0, 2, 4, 6, 8, 12, 16, 24, 32]
delays = list(range(0, 95, 5))
noise_res = {n: [success_rate(c, noise_deg=lv, use_kf=kf) for lv in noise_levels]
             for n, (c, kf) in configs.items()}
delay_res = {n: [success_rate(c, noise_deg=0.5, delay_ms=d, use_kf=kf) for d in delays]
             for n, (c, kf) in configs.items()}

print("Run time: %.0f s" % (time.time() - t0))
for n in configs:
    print(n, "| max noise (deg):", margin(noise_levels, noise_res[n]),
          "| delay margin (ms):", margin(delays, delay_res[n]))

fig, ax = plt.subplots(1, 2, figsize=(12, 4.5))
styles = {"PID": "C0-", "PID + Kalman": "C0--", "LQR": "C1-", "LQR + Kalman": "C1--"}
for n in configs:
    ax[0].plot(noise_levels, noise_res[n], styles[n], marker="o", label=n)
    ax[1].plot(delays, delay_res[n], styles[n], marker="o", label=n)
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
plt.savefig("results_kalman.png", dpi=150)
plt.show()
