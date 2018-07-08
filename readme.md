___

### Localization and Mapping

The second half of this Coursera course: https://www.coursera.org/learn/robotics-learning

Mapping:MATLAB sandbox to visualise a 2D robot exploring the inside of a building
using LIDAR and building up its own map using occupancy grid log-odd updates.
Its trajectory is pre-planned and does not collide with walls. 

Localisation: using the known map and a series of LIDAR measurements, we
can then estimate the position and attitude of our robot in the building by
using a particle filter. We can use these snapshots to infer its trajectory.

Author(s): UPenn School of Engineering and Applied Science

Contributor: Omar Kadhim - Summer 2018

IMPORTANTLY: this is an educational library and the vast majority of the
code, from the equations of motion, to the differential equation solver and
the visualisation libraries are provided by the UPenn course here: 
https://www.coursera.org/learn/robotics-flight/ 

___

### Update Log:

Rev1: First upload
____

### Getting Started:

Ensure all files are in the same directory. 

For the mapping example run mapping_test.m

For the localization example run localization_test.m

For the localization excercise I've created a visualisation plot and a gif 
for you to download and play around with. This is a challenging problem with 
many parameters to optimiseincluding the resolution of LIDAR measurements, the 
variance of motion and number of particles (position search space). Try
varying them in particleLocalization.m and see how sensitive the performance
of the algorithm is.
_____

