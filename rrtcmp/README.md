# rrtcmp

This module implements three RRT variants.

## AVP-RRT

The algorithm introduced in

Pham, Quang-Cuong, Stéphane Caron, and Yoshihiko Nakamura. 
[Kinodynamic Planning in the Configuration Space via Admissible Velocity
Propagation.](http://www.normalesup.org/~pham/docs/kinodynamic.pdf)
*Robotics: Science and Systems*. 2013.

The class name is `VIP_RRT`, its former name, which we do not change for
backward compatibility with the benchmarking dump files.

## *k*-NN RRT

The algorithm from 

LaValle, Steven M., and James J. Kuffner. "Randomized kinodynamic
planning." *The International Journal of Robotics Research* 20.5 (2001): 378-400.

upgraded with the *k*-Nearest Neighbors heuristic. (Disable with *k*=1.)

## LQR-RRT

The algorithm from

Perez, Alejandro, et al. "Lqr-rrt\*: Optimal sampling-based motion planning
with automatically derived extension heuristics." *IEEE International
Conference on Robotics and Automation* (ICRA), 2012.
