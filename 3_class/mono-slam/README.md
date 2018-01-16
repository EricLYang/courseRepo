# Mono-Slam Implementation in ROS
这是ROS MONO SLAM (EKF) 的实现,基于ROS. 复现[Andrew Davison](https://github.com/EricLYang/courseRepo/blob/master/3_class/referencePaper/davison_etal_pami2007.pdf)的工作,
详情参考原作者的论文:[Ludovico O. Russo](https://github.com/EricLYang/courseRepo/blob/master/3_class/mono-slam/reference/Master%20Thesis%20Ludovico%20Russo%20-%20oneside.pdf). 有详细的介绍,值得阅读.

# 安装 Install

1. 首先安装configlib:

> - sudo apt-get install libconfig++-dev

2. 编译

> - cd build
> - rm -rf *
> - cmake ..
> - make -j8

这样就可以了.

## Usage 使用

Please revise the run.sh file, and change the part of your camera config file and the image message topic:

> - bash run.sh

我提供的一个数据集,请见百度网盘链接(微信群里),使用的话:

> - rosbag play test.bag


