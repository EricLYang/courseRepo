#include "vslamRansac.hpp"

#include <ros/ros.h>

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>


#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <nav_msgs/Path.h>

#include <eigen3/Eigen/Dense>
using namespace Eigen;


class RosVSLAM : public VSlamFilter
{
private:
	nav_msgs::Path cameraPath;
	void updatePath();
	std::vector<Quaternionf> vQuat;
	std::vector<Matrix3f> vCov;

public:
	RosVSLAM(char *file = NULL);
	geometry_msgs::Pose getCameraPose();
	visualization_msgs::MarkerArray getFeatures();
	visualization_msgs::MarkerArray ActualCameraRepr();

	void predict(float v_x = 0, float w_z = 0);
	void update(float v_x = 0, float w_z = 0);
	nav_msgs::Path getCameraPath();
	nav_msgs::Odometry getVisualOdometry(ros::Time stamp);
};


