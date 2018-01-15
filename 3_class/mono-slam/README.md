# Mono-Slam Implementation in ROS
这是ROS MONO SLAM (EKF) 的实现,基于ROS. 复现[Andrew Davison](https://www.doc.ic.ac.uk/~ajd/Publications/davison_etal_pami2007.pdf)的工作,
详情参考原作者的论文:[Ludovico O. Russo](https://github.com/ludusrusso). 有详细的介绍,值得阅读.

# 安装 Install

1. 首先安装configlib:

> - sudo apt-get install libconfig++-dev

2. 编译

> - cd build
> - rm -rf *
> - cmake ..
> - make -j8

这样就可以了

## Usage:


