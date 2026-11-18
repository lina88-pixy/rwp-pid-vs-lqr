import numpy as np
import matplotlib.pyplot as plt

# Placeholder parameters (we will replace them with your real ones)
I = 0.01      # pendulum inertia about the pivot (kg m^2)
J = 0.001     # wheel inertia (kg m^2)
mgl = 0.2     # m*g*l (N m)
u_max = 0.5   # max wheel torque (N m)

# PD gains: stable only if Kp > mgl
Kp = 2 * mgl
Kd = 2 * 0.7 * np.sqrt(I * (Kp - mgl))

dt, T = 0.005, 10.0
th, w, wheel = np.deg2rad(5), 0.0, 0.0   # start 5 degrees from upright
t_log, th_log, u_log, wheel_log = [], [], [], []

for k in range(int(T / dt)):
    u = np.clip(Kp * th + Kd * w, -u_max, u_max)
    acc = (mgl * np.sin(th) - u) / I
    wheel_acc = u / J - acc
    w += acc * dt
    th += w * dt
    wheel += wheel_acc * dt
    t_log.append(k * dt)
    th_log.append(np.rad2deg(th))
    u_log.append(u)
    wheel_log.append(wheel)

fig, ax = plt.subplots(3, 1, sharex=True)
ax[0].plot(t_log, th_log); ax[0].set_ylabel("angle (deg)")
ax[1].plot(t_log, u_log); ax[1].set_ylabel("torque (N m)")
ax[2].plot(t_log, wheel_log); ax[2].set_ylabel("wheel speed (rad/s)")
ax[2].set_xlabel("time (s)")
plt.show()

