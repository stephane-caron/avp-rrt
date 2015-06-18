# AVP-RRT

This repository contains the code used in the paper:

Pham, Quang-Cuong, Stéphane Caron, and Yoshihiko Nakamura.  [Kinodynamic
Planning in the Configuration Space via Admissible Velocity
Propagation.](http://www.normalesup.org/~pham/docs/kinodynamic.pdf) *Robotics:
Science and Systems*. 2013.

## Requirements

- [OpenRAVE](http://openrave.org)
- Optional: [IPython](http://ipython.org/)
- Optional: [Slycot](https://github.com/avventi/Slycot) (if you want to use
  LQR-RRT)

## Usage

Run `make vip-rrt.py` to open IPython with an AVP-RRT instance.

Plots for the torque-limited pendulum:

- `make 11-05`: (11, 5)-Nm torque limit
- `make 11-07`: (11, 7)-Nm torque limit
- `make 13-05`: (13, 5)-Nm torque limit

Plots for RRT parameter identification:

- `make param-dur`: vary the maximum duration of local trajectories
- `make param-mod`: vary the steer-to-goal frequency (in number of steps)
- `make param-trj`: vary the number of trajectories tried per extension
