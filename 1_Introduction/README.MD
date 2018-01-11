### FIRST CLASS AS INTRODUCTION

This course cover the basic introduction of SLAM, about the definition of the problem, the reason why it was proposed, the framework, and the [ROS](http://www.ros.org/) platform which we mainly test on.

I attach the follow usefull links for your consideration:
- Albert-Ludwigs-Universität Freiburg [SLAM Course](http://ais.informatik.uni-freiburg.de/teaching/ws13/mapping/) Documents
- ICRA 2016 [SLAM tutorial](http://www.dis.uniroma1.it/~labrococo/tutorial_icra_2016/)
- CCNY [Advanced Mobile ROBOTICS](http://www-ee.ccny.cuny.edu/www/web/jxiao/G3300-s17.htm) Course
- Using [MPRT](https://www.mrpt.org/) platform to simulate [SLAM](https://www.mrpt.org/List_of_SLAM_algorithms)
- Open SLAM [resource](http://openslam.org/)
- [Amazing Source](http://lfa.mobivap.uva.es/~fradelg/phd/index.html)

Open Source Projects:

> -  [PTAM](https://github.com/lrse/sptam)
> -  [ORB-SLAM](https://github.com/raulmur/ORB_SLAM)
> -  [LSD-SLAM](https://github.com/tum-vision/lsd_slam)
> -  [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2)
> -  [OKVIS](https://github.com/ethz-asl/okvis)
> -  [DVO: Dense Visual Odometry](https://github.com/tum-vision/dvo_slam)
> -  [SVO: Semi-Direct Monocular Visual Odometry](https://github.com/uzh-rpg/rpg_svo)
> -  [RGBD-SLAM](https://github.com/felixendres/rgbdslam_v2)
### Reference Papers

> Please check the [reference Folder](https://github.com/EricLYang/courseRepo/tree/master/1_Introduction/referencePaper), two very important papers are:
> - Hugh Durrant-Whyte & Tim Bailey. “Simultaneous Localisation and Mapping (SLAM): Part I The Essential Algorithms“, (2006) ([PDF](https://github.com/EricLYang/courseRepo/blob/master/1_Introduction/referencePaper/10.1.1.128.4195.pdf))
> - Tim Bailey & Hugh Durrant-Whyte and . “Simultaneous Localisation and Mapping (SLAM): Part II State of the Art“, (2006) ([PDF](https://github.com/EricLYang/courseRepo/blob/master/1_Introduction/referencePaper/10.1.1.108.4153.pdf))


### Build By Yourself and RUN

1. Build the project

> - cd build
> - rm -r *
> - cmake ..
> - make -j4

2. Download the data from [TUM](https://vision.in.tum.de/data/datasets/rgbd-dataset/download)

> - cd  ..
> - wget https://vision.in.tum.de/rgbd/dataset/freiburg1/rgbd_dataset_freiburg1_room.bag

3. Run 

> - roscore
> - rosbag play rgbd_dataset_freiburg1_room.bag
> - ./RGBD_demo /camera/rgb/image_color /camera/depth/image /camera/rgb/camera_info /camera/depth/camera_info
> - rosrun rviz rviz
