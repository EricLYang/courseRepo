### CourseRepo
This folder contains the files for SLAM Robotics & Autonomous Driving course, this course is mostly based on: Ubuntu (I use [14.04](http://releases.ubuntu.com/14.04/)) with [ROS Indigo](http://wiki.ros.org/indigo).

1. Install ROS 安装ROS版本可以根据个人情况而定,没有特定要求

Before starting, please follow the given instruction to install ROS:
- For indigo: [Link](http://wiki.ros.org/indigo/Installation), if you use ubuntu, follow this [link](http://wiki.ros.org/indigo/Installation/Ubuntu)

- For other distribution of ROS, please search google with: ROS + [dirstribution](http://wiki.ros.org/Distributions) + install

2. Install Opencv

Please follow this [intsruction](http://www.samontab.com/web/2014/06/installing-opencv-2-4-9-in-ubuntu-14-04-lts/)

If you need a different distribution, execute

> - git checkout distribution

### Pre-materials
#### How to creat a ROS Workspace, it is more convenient than catkin_workspace
1. First step:

> cd ~/ 

mkdir ROS_WORKSPACE

2. Edit .bashrc

> gedit .bashrc # open .bashrc under ~/

Then, paste the following behind source /opt/ros/indigo/setup.bash:

> ROS_WORKSPACE=$HOME/ROS_WORKSPACE
 
> export ROS_WORKSPACE 

> ROS_PACKAGE_PATH=$ROS_WORKSPACE:$ROS_PACKAGE_PATH

> export ROS_PACKAGE_PATH 

Open a new terminal, try:
> roscd

### CMake Tutorial
1. 一本很好的书,感谢作者
> - [书链接](https://github.com/Akagi201/learning-cmake/tree/master/docs)

2. 一个很好的教程
> - [链接](https://github.com/Akagi201/learning-cmake)

### Add SSH key to your github
Follow this instruction, 
> https://help.github.com/articles/adding-a-new-ssh-key-to-your-github-account/

If not working, go to your folder with use interface or go to the /path/of/the/folder in terminal execute:
> ll

Then, direct to .git/config, change
> url = http://github.com/path/to/repository
to
> url = ssh://git@github.com/path/to/repository
Then, it should work.

#### 资料

更新了阅读资料百度网盘[链接](https://pan.baidu.com/s/1miXK6ow​) 密码 ​​​tpb3

需要资料的话,请留言,或者平台问答.

## Course Contents
#### [第一讲: SLAM概论和架构](https://github.com/EricLYang/courseRepo/tree/master/1_Introduction)
> 

#### [第二讲: SLAM基本理论一：坐标系、刚体运动和李群](https://github.com/EricLYang/courseRepo/tree/master/2_class)  
> 

#### [第三讲: SLAM基本理论二：从贝叶斯开始学滤波器](https://github.com/EricLYang/courseRepo/tree/master/3_class)  

#### [第四讲: SLAM基本理论三：图优化](https://github.com/EricLYang/courseRepo/tree/master/4_class)  
