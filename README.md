# slam_backend README
This is the SLAVE README. Read it after reading the README in the `slam` repo

## iSAM2-based backend interface for 2D Pose Graph SLAM

Build by cloning this into the src directory of your catkin workspace and then doing catkin_make.

### Requires GTSAM.
To install GTSAM, follow the instructions (https://bitbucket.org/gtborg/gtsam), cloning into a folder outside your catkin workspace.

$ mkdir build

$ cd build

$ cmake ..

$ ccmake .. (set GTSAM_WITH_EIGEN_MKL and GTSAM_WITH_EIGEN_MKL_OPENMP to OFF)

$ make check (optional, runs unit tests)

$ make install

## To Install SLAM Backend

Clone into your catkin workspace source folder and then catkin_make.

# Debugging
Installing gtsam on Ubuntu 16.04:
In file: /home/student/Downloads/gtsam-3.2.1/gtsam/base/FastSet.h
add the line `#include <boost/serialization/serialization.hpp>`
before the line `#include <boost/serialization/set.hpp>`

based on https://svn.boost.org/trac/boost/ticket/12126