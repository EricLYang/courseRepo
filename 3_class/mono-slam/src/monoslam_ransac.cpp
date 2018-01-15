#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

#include <std_msgs/Float32.h>

#include "RosVSLAMRansac.hpp"
#include "utils.hpp"

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <fstream>


using namespace cv;
using namespace std;
using namespace geometry_msgs;



namespace enc = sensor_msgs::image_encodings;

static const char WINDOW[] = "Image window";




class ImageConverter
{

  ros::NodeHandle nh_;
  ros::Publisher camera_poses_pub;
  ros::Publisher camera_pose_pub;




  ros::Publisher features_pub;
  ros::Publisher camera_pub;

  ros::Publisher odometry_pub;


  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  
  ros::Publisher quality_index_pub;



private:
	RosVSLAM slam;
	int first;
	PoseArray poses;
	int numFeatures;
	tf::TransformListener tf_listener;

	ros::Subscriber odom_subscriber;

	nav_msgs::Odometry latest_odom_;
	nav_msgs::Odometry last_odom_;
	bool received_odom_;

	float v_speed_, w_speed_;
	fstream fs;

public:
  ImageConverter(char *file = NULL)
    : it_(nh_), first(1), slam(file)
  {
	  v_speed_ = w_speed_ = 100;
  	poses.header.frame_id = "/world"; 
    image_pub_ = it_.advertise("/monoslam/imgproc", 1);
    image_sub_ = it_.subscribe("/camera/image_raw", 1, &ImageConverter::imageCb, this);

    odom_subscriber = nh_.subscribe("/pose", 1, &ImageConverter::odom_callback, this);


    cv::namedWindow(WINDOW);
	camera_poses_pub  = nh_.advertise<nav_msgs::Path>("/monoslam/camera_poses", 1000);
	camera_pose_pub  = nh_.advertise<geometry_msgs::Pose>("/monoslam/camera_pose", 1000);
	
	odometry_pub = nh_.advertise<nav_msgs::Odometry>("/monoslam/visual_odometry", 1000);


	camera_pub  = nh_.advertise<visualization_msgs::MarkerArray>("/monoslam/camera", 1000);

	features_pub  = nh_.advertise<visualization_msgs::MarkerArray>("/monoslam/features", 1000);



	quality_index_pub = nh_.advertise<std_msgs::Float32>("/monoslam/quality_index",1000);

  }





  ~ImageConverter()
  {
    cv::destroyWindow(WINDOW);
    fs.close();
  }

  void odom_callback(const nav_msgs::Odometry::ConstPtr& odom_msg)
  {
	  v_speed_ = odom_msg->twist.twist.linear.x;
	  w_speed_ = odom_msg->twist.twist.angular.z;
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
	//std::cout << "New Frame Caputerd" << endl;
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    
	Mat frame;
	int scale = 1;
	cv::resize(cv_ptr->image,cv_ptr->image,cv::Size(cv_ptr->image.size().width/scale, cv_ptr->image.size().height/scale));
	
	if (first) {

		first = 0;

		slam.captureNewFrame(cv_ptr->image);


    	slam.findNewFeatures();

		frame = slam.returnImageDrawed();
	} else {
		slam.captureNewFrame(cv_ptr->image, msg->header.stamp.toSec());



		slam.predict();
		slam.update();
		
		poses.poses.push_back(slam.getCameraPose());
		camera_poses_pub.publish(slam.getCameraPath());

		geometry_msgs::Pose camPose = slam.getCameraPose();

		camera_pose_pub.publish(camPose);

		features_pub.publish(slam.getFeatures());
		camera_pub.publish(slam.ActualCameraRepr());
		

		odometry_pub.publish(slam.getVisualOdometry(msg->header.stamp));
		frame = slam.returnImageDrawed();
	}
	
		

	cv_ptr->image = frame;
    image_pub_.publish(cv_ptr->toImageMsg());

    char name[100];
    static int num = 0;

    /*
    	sprintf(name, "frame%03d.jpg", num++);
    	cv::imwrite(name, frame);
    */
  }
};

int main(int argc, char** argv)
{

	ros::init(argc, argv, "Mono_SLAM");
	char *file = NULL;
	if (argc > 1) file = argv[1];


	std::cout << "Starting monoslam_exec" << endl;
	
  	ImageConverter ic(file);
  ros::spin();
  return 0;
}


