#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>

using namespace Eigen;

class CamModel {
    float fx, fy, k1, k2;
    float u0, v0;
    
public:
    
    // Camera fatta a mano wide angle
    CamModel(
             float _fx = 558.665908,
             float _fy = 554.953227,
             float _u0 = 346.781365,
             float _v0 = 241.238318,
             float _k1 = -0.433618,
             float _k2 = 0.220758);

    /*
     *
     *
     * Quelli della camera rossa stereo
    CamModel(
                float _fx = 884.805305,
                float _fy = 880.495678,
                float _u0 = 320.125098,
                float _v0 = 234.720010,
                float _k1 = -0.105199,
                float _k2 = -0.028570);
*/
  /*
    CamModel(
              float _fx = 862.350016,
              float _fy = 856.474667,
              float _u0 = 312.041077 -0.5,
              float _v0 = 212.607881 -0.5,
              float _k1 = -0.098650,
              float _k2 = 0.009240);*/





/*
CamModel(

             float _fx = 1443.6375/4,
             float _fy = 1418.4542/4,
             float _u0 = 617.56/4,
             float _v0 = 474.686/4,
             float _k1 = 0.013002,
             float _k2 = -0.024809);*/
    CamModel(cv::Mat parameters);

    Vector3f UndistortAndDeproject(Vector2f hd, MatrixXf &J);
    Vector2f projectAndDistort(Vector3f h, MatrixXf &J);
	Vector2f projectAndDistort(Vector3f h);
};
