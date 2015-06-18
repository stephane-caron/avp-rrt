# AVP-RRT

This repository contains the code used in the paper:

    Pham, Quang-Cuong, Stéphane Caron, and Yoshihiko Nakamura. 
    [Kinodynamic Planning in the Configuration Space via Admissible Velocity
    Propagation.](http://www.normalesup.org/~pham/docs/kinodynamic.pdf)
    *Robotics: Science and Systems*. 2013.

## Requirements

- [OpenRAVE](http://openrave.org)
- Optional: [IPython](http://ipython.org/)
- Optional: [Slycot](https://github.com/avventi/Slycot) (if you want to use
  LQR-RRT)

## Usage

See the Makefile:

    $ make
    Usage:                                                         
                                                                   
        make vip-rrt.py -- run AVP-RRT planner in an ipython shell 
                                                                   
    Plots for the torque-limited pendulum:                         
                                                                   
        make 11-05 -- (11, 5)-Nm torque limit                      
        make 11-07 -- (11, 7)-Nm torque limit                      
        make 13-05 -- (13, 5)-Nm torque limit                      
                                                                   
    Plots for RRT parameter identification:                        
                                                                   
        make param-dur -- max-traj-duration                        
        make param-mod -- steer-to-goal-modulo                     
        make param-trj -- traj-per-extension                       
                                                                   
    Other:                                                         
                                                                   
        make clean -- clean temporary files                        
        make help -- this message  
