#include "RosVSLAMRansac.hpp"
#include <sstream>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>


 // #define DRAW_COV


RosVSLAM::RosVSLAM(char *file): VSlamFilter(file) {
	this->cameraPath.header.frame_id = "/world";
	this->cameraPath.header.stamp = ros::Time::now();
}

geometry_msgs::Pose RosVSLAM::getCameraPose() {
	geometry_msgs::Pose p;

	VectorXf state = getState();

	p.position.x = state(0)*map_scale;
	p.position.y = state(1)*map_scale;
	p.position.z = state(2)*map_scale;
	p.orientation.x = state(4);
	p.orientation.y = state(5);
	p.orientation.z = state(6);
	p.orientation.w = state(3);
	return p;
}

void RosVSLAM::updatePath() {
	geometry_msgs::PoseStamped pose;

	VectorXf state = getState();

	pose.pose.position.x = state(0)*map_scale;
	pose.pose.position.y = state(1)*map_scale;
	pose.pose.position.z = state(2)*map_scale;
	pose.pose.orientation.x = state(4);
	pose.pose.orientation.y = state(5);
	pose.pose.orientation.z = state(6);
	pose.pose.orientation.w = state(3);
	this->cameraPath.poses.push_back(pose);
}

void RosVSLAM::predict(float v_x, float w_z) {
	this->VSlamFilter::predict();
	this->updatePath();
}


void RosVSLAM::update(float v_speed_, float w_speed_) {
	this->VSlamFilter::update(v_speed_,w_speed_);
	this->updatePath();
}

nav_msgs::Odometry RosVSLAM::getVisualOdometry(ros::Time stamp){
	VectorXf state = getState();
	nav_msgs::Odometry odom;
	odom.header.stamp = stamp;
	odom.twist.twist.linear.x = state(7);
	odom.twist.twist.linear.y = state(8);
	odom.twist.twist.linear.z = state(9);

	odom.twist.twist.angular.x = state(10);
	odom.twist.twist.angular.y = state(11);
	odom.twist.twist.angular.z = state(12);

	MatrixXf S = Sigma.block<6,6>(7,7);

	for(int i = 0; i < 6; i++) {
		for(int j = 0; j < 6; j++){
			odom.twist.covariance[i+6*j] = S(i,j);
		}
	}

	return odom;

}



visualization_msgs::MarkerArray RosVSLAM::ActualCameraRepr() {

	visualization_msgs::MarkerArray camera;

	visualization_msgs::Marker camera_marker;
	camera_marker.header.frame_id = "/world";
	camera_marker.header.stamp = ros::Time::now();
	camera_marker.ns  = "camera_marker";
	camera_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
	camera_marker.mesh_resource = "package://vslam/mesh/test.dae";
    camera_marker.action = visualization_msgs::Marker::ADD;
    camera_marker.pose.orientation.w = 1.0;
	camera_marker.id = 0;
    camera_marker.scale.x = 0.1;
    camera_marker.scale.y = 0.1;
	camera_marker.scale.z = 0.1;
	camera_marker.color.b = 1.0f;
    camera_marker.color.a = 1.0;

	VectorXf state = getState();

	std::cout << map_scale << std::endl;
	camera_marker.pose.position.x = state(0)*map_scale;
	camera_marker.pose.position.y = state(1)*map_scale;
	camera_marker.pose.position.z = state(2)*map_scale;
	camera_marker.pose.orientation.x = state(4);
	camera_marker.pose.orientation.y = state(5);
	camera_marker.pose.orientation.z = state(6);
	camera_marker.pose.orientation.w = state(3);


	Matrix3f Cov = Sigma.block<3,3>(0,0);
    SelfAdjointEigenSolver<MatrixXf> eigenSolver(Cov);
    Vector3f eigs = eigenSolver.eigenvalues();
    Matrix3f vecs = eigenSolver.eigenvectors();
    Quaternionf q(vecs);

	visualization_msgs::Marker CameraCov;
	CameraCov.header.frame_id = "/world";
	CameraCov.header.stamp = ros::Time::now();

	CameraCov.ns  = "CameraCov";
	CameraCov.action = visualization_msgs::Marker::ADD;
   // pointsXYZ.pose.orientation.w = 1.0;
	CameraCov.id = 0;
	CameraCov.type = visualization_msgs::Marker::SPHERE;
	CameraCov.scale.x = 9*eigs(0)*map_scale;
	CameraCov.scale.y = 9*eigs(1)*map_scale;
	CameraCov.scale.z = 9*eigs(2)*map_scale;
	CameraCov.pose.orientation.w = q.w();
	CameraCov.pose.orientation.x = q.x();
	CameraCov.pose.orientation.y = q.y();
	CameraCov.pose.orientation.z = q.z();
	CameraCov.pose.position.x = state(0)*map_scale;
	CameraCov.pose.position.y = state(1)*map_scale;
	CameraCov.pose.position.z = state(2)*map_scale;
	CameraCov.color.b = 1.0f;
	CameraCov.color.a = 0.3;
	CameraCov.lifetime.sec = 1;

	camera.markers.push_back(CameraCov);
	camera.markers.push_back(camera_marker);



	return camera;
}



visualization_msgs::MarkerArray RosVSLAM::getFeatures() {
	visualization_msgs::Marker pointsINV;
	pointsINV.header.frame_id = "/world";
	pointsINV.header.stamp = ros::Time::now();
	pointsINV.ns  = "points_and_lines";
	pointsINV.action = visualization_msgs::Marker::ADD;
	pointsINV.pose.orientation.w = 1.0;
	pointsINV.id = 0;
	pointsINV.type = visualization_msgs::Marker::SPHERE_LIST;
	pointsINV.scale.x = 0.1;
	pointsINV.scale.y = 0.1;
	pointsINV.scale.z = 0.1;
	pointsINV.color.r = 0.0f;
	pointsINV.color.g = 0.0f;
	pointsINV.color.b = 0.0f;
	pointsINV.color.a = 1.0;


	visualization_msgs::MarkerArray points;

	for (int i = 0; i < this->patches.size(); ++i) {
		int pos = this->patches[i].position_in_state;
		Vector3f d;
		Matrix3f Cov;
		if (!patches[i].isXYZ()) {
			MatrixXf Jf;
			d = depth2XYZ(mu.segment<6>(pos), Jf);
			geometry_msgs::Point p;
			p.x = d(0)*map_scale;
			p.y = d(1)*map_scale;
			p.z = d(2)*map_scale;
			pointsINV.points.push_back(p);
			Cov = Jf*Sigma.block<6,6>(pos,pos)*Jf.transpose();

	        SelfAdjointEigenSolver<MatrixXf> eigenSolver(Cov);
	        Vector3f eigs = eigenSolver.eigenvalues();
	        Matrix3f vecs = eigenSolver.eigenvectors();
	        Quaternionf q(vecs);

	    	visualization_msgs::Marker pointsINV;
	    	pointsINV.header.frame_id = "/world";
	    	pointsINV.header.stamp = ros::Time::now();
	    	char name[20];
	    	sprintf(name, "F%d",i);
	    	std::stringstream ss;
	    	ss << i;

	    	pointsINV.ns  = name;
	    	pointsINV.action = visualization_msgs::Marker::ADD;
	       // pointsXYZ.pose.orientation.w = 1.0;
	    	pointsINV.id = i;
	        pointsINV.type = visualization_msgs::Marker::SPHERE;
	        pointsINV.scale.x = eigs(0)*map_scale;
	        pointsINV.scale.y = eigs(1)*map_scale;
	        pointsINV.scale.z = eigs(2)*map_scale;
	        pointsINV.pose.orientation.w = q.w();
	        pointsINV.pose.orientation.x = q.x();
	        pointsINV.pose.orientation.y = q.y();
	        pointsINV.pose.orientation.z = q.z();
	        pointsINV.pose.position.x = d(0)*map_scale;
	        pointsINV.pose.position.y = d(1)*map_scale;
	        pointsINV.pose.position.z = d(2)*map_scale;
	        pointsINV.color.g = 1.0f;
	        pointsINV.color.a = 0.5;
	        pointsINV.lifetime.sec = 1;

	    	// points.markers.push_back(pointsINV);

		} else {
			d = mu.segment<3>(pos);
			Cov = Sigma.block<3,3>(pos,pos);
			Cov.eigenvalues();
	        SelfAdjointEigenSolver<MatrixXf> eigenSolver(Cov);
	        Vector3f eigs = eigenSolver.eigenvalues();
	        Matrix3f vecs = eigenSolver.eigenvectors();
	        Quaternion<float> q(vecs);

	        visualization_msgs::Marker pointsXYZ;
	    	pointsXYZ.header.frame_id = "/world";
	    	pointsXYZ.header.stamp = ros::Time::now();
	    	char name[20];
	    	sprintf(name, "F%d",i);
	    	std::stringstream ss;
	    	ss << i;
	    	pointsXYZ.ns = name;
	    	pointsXYZ.action = visualization_msgs::Marker::ADD;
	        pointsXYZ.id = i;
	        pointsXYZ.type = visualization_msgs::Marker::SPHERE;
	        pointsXYZ.scale.x = eigs(0)*map_scale;
	        pointsXYZ.scale.y = eigs(1)*map_scale;
	        pointsXYZ.scale.z = eigs(2)*map_scale;
	        pointsXYZ.pose.orientation.w = q.w();
	        pointsXYZ.pose.orientation.x = q.x();
	        pointsXYZ.pose.orientation.y = q.y();
	        pointsXYZ.pose.orientation.z = q.z();
	        pointsXYZ.pose.position.x = d(0)*map_scale;
	        pointsXYZ.pose.position.y = d(1)*map_scale;
	        pointsXYZ.pose.position.z = d(2)*map_scale;
	        pointsXYZ.color.r = 1.0f;
	        pointsXYZ.color.a = 0.5;
	        pointsXYZ.lifetime.sec = 1;
	    	//points.markers.push_back(pointsXYZ);

	        visualization_msgs::Marker textpointsXYZ;
	        textpointsXYZ.header.frame_id = "/world";
	        textpointsXYZ.header.stamp = ros::Time::now();
	    	sprintf(name, "name%d",i);
	    	textpointsXYZ.ns = name;
	    	textpointsXYZ.text = ss.str();
	    	textpointsXYZ.action = visualization_msgs::Marker::ADD;
	       // pointsXYZ.pose.orientation.w = 1.0;
	    	textpointsXYZ.id = 0;
	    	textpointsXYZ.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	    	textpointsXYZ.scale.x = 1;
	        textpointsXYZ.scale.y = 1;
	        textpointsXYZ.scale.z = 1;
	        textpointsXYZ.pose.position.x = d(0)*map_scale;
	        textpointsXYZ.pose.position.y = d(1)*map_scale;
	        textpointsXYZ.pose.position.z = d(2)*map_scale;
	        textpointsXYZ.color.r = 0.0f;
	        textpointsXYZ.color.g = 0.0f;
	        textpointsXYZ.color.b = 0.0f;
	        textpointsXYZ.color.a = 1;
	        textpointsXYZ.lifetime.sec = 1;
	    	// points.markers.push_back(textpointsXYZ);







			geometry_msgs::Point p;
			p.x = d(0)*map_scale;
			p.y = d(1)*map_scale;
			p.z = d(2)*map_scale;
			pointsINV.points.push_back(p);


		}

    }

	points.markers.push_back(pointsINV);




	static int count_OLD = 0;
	char name[100];
	visualization_msgs::Marker pointsOLD;
	pointsOLD.header.frame_id = "/world";
	pointsOLD.header.stamp = ros::Time::now();
	pointsOLD.action = visualization_msgs::Marker::ADD;
	pointsOLD.pose.orientation.w = 1.0;
	pointsOLD.id = 0;
	pointsOLD.type = visualization_msgs::Marker::SPHERE_LIST;
	pointsOLD.scale.x = 0.1;
	pointsOLD.scale.y = 0.1;
	pointsOLD.scale.z = 0.1;
	pointsOLD.color.r = 0.0f;
	pointsOLD.color.g = 0.0f;
	pointsOLD.color.b = 0.0f;
	pointsOLD.color.a = 1.0;



	std::cout << "size = " <<deleted_patches.size() << std::endl;
	if (deleted_patches.size() > 1000) {
		for (int i = 0; i < deleted_patches.size(); i++) {
			sprintf(name, "OLD_points%d", count_OLD++);
			pointsOLD.ns  = name;
			geometry_msgs::Point p;
			p.x = deleted_patches[i].XYZ_pos(0)*map_scale;
			p.y = deleted_patches[i].XYZ_pos(1)*map_scale;
			p.z = deleted_patches[i].XYZ_pos(2)*map_scale;
			pointsOLD.points.push_back(p);
		}
		deleted_patches.clear();
	} else {
		for (int i = 0; i < deleted_patches.size(); i++) {
			pointsOLD.ns  = "OLD_points";
			geometry_msgs::Point p;
			p.x = deleted_patches[i].XYZ_pos(0)*map_scale;
			p.y = deleted_patches[i].XYZ_pos(1)*map_scale;
			p.z = deleted_patches[i].XYZ_pos(2)*map_scale;
			pointsOLD.points.push_back(p);
		}
	}

	points.markers.push_back(pointsOLD);


    return points;
}

nav_msgs::Path RosVSLAM::getCameraPath() {
	return this->cameraPath;
}
