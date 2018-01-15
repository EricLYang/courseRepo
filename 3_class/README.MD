### EKF滤波器内容

滤波器的主要核心思想参见课件,需要清楚预测和观测的两部分区别.

1. 给除了一些参考论文,这些都是很出色的工作,详细看一下:

>- [Reference]()

2.  分清楚Indirect Approach 和 Direct Approach区别

> - [PDF]()



### The implementation Of using Matlab for VINS (EKF) 视惯融合MATLAB版本

有一个VINS的MATLAB版本,这个是开源的,大家可以试一下,这个是完整实现了Indirect Approach,用的数据是[KITTI](http://www.cvlibs.net/datasets/kitti/raw_data.php)数据.
源代码链接[link](https://www.mathworks.com/matlabcentral/fileexchange/43218-visual-inertial-odometry?s_tid=gn_loc_drop)

1. 论文,请参见链接
> - [PDF]()

2. 这个里面用到的各项技巧都是其他论文里面或多或少涉及,特别是Sliding Window这个概念

3. 结果很tricky,尝试一下自己改变参数

### MONO-SLAM的实例

请见[mono-slam文件夹]()
