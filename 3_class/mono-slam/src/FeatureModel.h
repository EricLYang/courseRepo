/*
 * FeatureModel.h
 *
 *  Created on: Jul 22, 2013
 *      Author: ludovicorusso
 */

#ifndef FEATUREMODEL_H_
#define FEATUREMODEL_H_

#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>

#include "MotionModel.h"
#include "cameraModel.hpp"
#include "FeaturesState.h"


using namespace Eigen;
using namespace std;
using namespace cv;


class FeatureModel {
private:
	MatrixXf H_matrix;
	Mat Patch;

	Matrix<float, 6, 13> computeJx();
	Matrix<float, 6, 6>  computeJm();
	Matrix3f computeSigma_mm(float sigma_pixel = 1, float sigma_rho = 0.5);

	const CameraModel & camera;
	const MotionModel & motion;
	const FeaturesState & features;


public:
	FeatureModel(const CameraModel & _camera);
	virtual ~FeatureModel();

	bool isInInverseDepth;

	Vector2f imageLocation;

	VectorXf f;

	Vector2f z;
	Vector2f h;
	MatrixXf Sigma_ii;
	vector<MatrixXf> Sigma_ij;

	Matrix3f get3DCovariance();
	Matrix2f get2DCovariance();

	Vector2f get2DPosition();
	Vector3f get3DPosition();

	Vector2f predict();
};

#endif /* FEATUREMODEL_H_ */
