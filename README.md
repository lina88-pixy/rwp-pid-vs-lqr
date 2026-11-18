# rwp-ros2-pid-vs-lqr

PID vs LQR (+ Kalman filter) for balancing a reaction wheel pendulum in ROS 2 Humble and CoppeliaSim, evaluated under sensor noise and actuation delay.

## Research question
How much sensor noise and actuation delay can PID and LQR controllers
tolerate before a reaction wheel pendulum falls, and does a Kalman filter
widen that margin?

## Hypotheses
1. LQR outperforms PID in the nominal (noise-free, delay-free) case.
2. LQR loses more stability margin to delay than PID does.
3. A Kalman filter restores performance under sensor noise.

(Results: see "Findings" below. Each hypothesis is marked supported or not.)

## Metrics
- **Success rate:** fraction of runs where the pendulum angle never exceeds 30°.
- **Settling time:** time to return within ±1° after a push.
- **Max angle:** peak pendulum angle after a push.
- **RMS torque:** root-mean-square of the wheel torque command (control effort).
- **Delay margin:** largest actuation delay with 100% success.
  
## Findings
Coming soon
