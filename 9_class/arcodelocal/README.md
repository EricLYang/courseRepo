# aruco_positioning_system

The system is try to use AR code for labeling. 需要打印ARCODE,请见opencv官网ARUCO


# tutorial of how to make
  you can download with:
    git clone https://EricLYang@bitbucket.org/EricLYang/arcodelocal.git

  Then go the build folder:
   rm -r *

  Then do:
  cmake ..

  Finally do:
  make -j4

# tutorial of how to launch
You should install ROS (or you cannot cmake.., or make).

First check the launch folder, the file aruco_mapping.launch is the one you supposed to use:
roscore
and then:
roslaunch aruco_mapping.launch

Before, keep in mind, you should check the images to be received in aruco_mapping.launch:
<remap from="/image_raw" to="what is topic name"/>

please remap to you topic name.

BTW, change the camera parameters in data/geniusF100.ini with your camera calibration results, in CCNY case, we use 7*6 board with 0.108m width:
rosrun camera_calibration cameracalibrator.py --size 7x6 --square 0.108 right:=/stereo/right/image_raw left:=/stereo/left/image_raw

Then enojoy.

# asasas

[ROS](http://ros.org) package

Aruco mapping package, see [Wiki](http://wiki.ros.org/aruco_mapping) 
