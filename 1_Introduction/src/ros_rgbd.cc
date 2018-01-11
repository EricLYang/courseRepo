#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include "geometry_msgs/Point32.h"
#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include<opencv2/core/core.hpp>

ros::Publisher pub;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

using namespace std;

void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD, const sensor_msgs::CameraInfoConstPtr& info_depth_msg)
{
	// Solve all of perception here...
	cv::Mat image_color = cv_bridge::toCvCopy(msgRGB)->image;
	cv::Mat image_depth = cv_bridge::toCvCopy(msgD)->image;
	cvtColor(image_color,image_color, CV_RGB2BGR);

	// colorize the image
	cv::Vec3b color(255,0,0);
	for(int y=0;y<image_color.rows;y++) {
		for(int x=0;x<image_color.cols;x++) {
			float depth = image_depth.at<short int>(cv::Point(x,y));// / 1000.0;

			if( depth == 0) {
				image_color.at<cv::Vec3b>(cv::Point(x,y)) = cv::Vec3b(0,0,255);
			}
			if( depth > 1.00) {
				image_color.at<cv::Vec3b>(cv::Point(x,y)) = cv::Vec3b(255,0,0);
			}
		}
	}

	//
	image_color.at<cv::Vec3b>(cv::Point(320,240)) = cv::Vec3b(255,255,255);

	// get camera intrinsics
	float fx = info_depth_msg->K[0];
	float fy = info_depth_msg->K[4];
	float cx = info_depth_msg->K[2];
	float cy = info_depth_msg->K[5];

	// produce a point cloud
	sensor_msgs::PointCloud2 pointcloud_msg_r;

        // produce a point cloud
	PointCloud::Ptr pointcloud_msg (new PointCloud);
	pointcloud_msg->header = image_depth_msg->header;

	pcl::PointXYZ pt;
	for(int y=0;y<image_color.rows;y+=4) {
		for(int x=0;x<image_color.cols;x+=4) {
			float depth = image_depth.at<short int>(cv::Point(x,y)) / 1000.0;

			if(depth>0) {
				pt.x = (x - cx) * depth / fx;
				pt.y = (y - cy) * depth / fy;
				pt.z = depth;
				//cout << pt.x<<" "<<pt.y<<" "<<pt.z<<endl;
				pointcloud_msg->points.push_back (pt);
			}
		}
	}
        //transfer pcl to pcls message
        //pcl::toROSMsg(pointcloud_msg, pointcloud_msg_r);
	//pointcloud_msg_r.height = 1;
        //pointcloud_msg_r.header = msgD->header;
	//pointcloud_msg_r.width = pointcloud_msg.points.size();
	pub.publish (pointcloud_msg);

        ROS_WARN("Data received......");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();

    ROS_DEBUG("Start a simple RGBD Receiving And Publishing Test");

    ros::NodeHandle nh;
    
    cout << " out " << argv[1] << " " << argv[2] << " " << argv[3] ;

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, argv[1], 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, argv[2], 1);
    message_filters::Subscriber<sensor_msgs::CameraInfo> info_camera_sub(nh, argv[3], 1);
    
    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2> ("/RGBD/point_cloud", 1);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub, depth_sub, info_camera_sub);
   
    sync.registerCallback(boost::bind(&GrabRGBD, _1, _2, _3));

    ros::spin();

    ros::shutdown();

    return 0;
}

