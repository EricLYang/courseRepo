#ifndef __CAMERA_MODEL__
#define __CAMERA_MODEL__

#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>



typedef struct _camConfig_ {
	float fx, fy, u0,v0,k1,k2,k3,p1,p2;
} camConfig;


using namespace Eigen;

class CameraModel {
    float fx, fy;
    float u0, v0;
    float k1, k2, k3, p1, p2;
    
private:
	Matrix2f diff_distort_undistort(Vector2f hn);
	Matrix2f projectionJacobian;
	Matrix3f unprojectionJacobian;

public:
    
    // Camera fatta a mano wide angle
    CameraModel(
             float _fx = 592.2860,
             float _fy = 584.9968,
             float _u0 = 362.1059,
             float _v0 = 275.9642,
             float _k1 =  -0.3954,
             float _k2 = 0.5521,
             float _k3 = 0,
             float _p1 = -0.0075,
             float _p2 =  0.0140);



	void setParam(camConfig camParams);
    Vector3f unproject(Vector2f hd, bool computeJacobian = false);
    Matrix3f getUnprojectionJacobian();

    Vector2f project(Vector3f h, bool computeJacobian = false);
    Matrix2f getProjectionJacobian();

};

#endif

