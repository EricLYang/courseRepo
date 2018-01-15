#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

#include <eigen3/Eigen/Dense>


using namespace Eigen;

void plotFeatures(cv::Mat img, std::vector<cv::Point2f> features);

MatrixXf vConcat(MatrixXf m1, MatrixXf m2);
MatrixXf hConcat(MatrixXf m1, MatrixXf m2);
VectorXf Concat(VectorXf m1, VectorXf m2);




