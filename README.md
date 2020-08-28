This repository contains benchmarking instances for the aircraft conflict resolution problem (ACRP) in air traffic control.

The ACRP consists of finding minimal deviation conflict-free trajectories for a set of aircraft by adjusting their velocities (speed and heading) within specified control bounds. The ACRP is often formulated in two-dimensional (2D) space under the assumption of uniform motion laws. The main decision variables of the 2D ACRP are the speed control rate and the heading control deviation of aircraft. 

The 2D ACRP can be represented as a nonconvex mixed-integer problem.

![Data and variables 2D ACRP](https://github.com/acrp-lib/acrp-lib/blob/master/datavariables.PNG)

![Nonconvex 2D ACRP](https://github.com/acrp-lib/acrp-lib/blob/master/nonconvex.PNG)

A reproducible model of this formulation is available in the __Code__ folder of this repository and details on this formulation can be found at http://arxiv.org/abs/1911.12997

Send your questions and comments about this repository to d.rey@unsw.edu.au
