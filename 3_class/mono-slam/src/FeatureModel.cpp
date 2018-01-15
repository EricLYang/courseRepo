/*
 * FeatureModel.cpp
 *
 *  Created on: Jul 22, 2013
 *      Author: ludovicorusso
 */

#include "FeatureModel.h"

FeatureModel::FeatureModel(	const CamModel & _camera,
								const MotionModel & _motion,
								const FeaturesState & _featuresState,
								const Mat & frame,
								const Point2f & pf,
								int patchSize,
								float rho_0,
								float sigma_rho):
								camera(_camera),
								motion(_motion),
								features(_featuresState)
	{
    Mat newPatch(cv::Mat(frame, cv::Rect(pf.x-patchSize/2, pf.y-patchSize/2, patchSize,patchSize)));
    this->imageLocation << pf.x, pf.y;

    this->Patch = newPatch.clone();


    Vector2f hd = (Vector2f << (float)pf.x, (float)pf.y);

    // TODO: convert using motionmodel
    Vector3f r = motion->get_r();
    Quatenionf q = motion->get_q();


    Vector3f hC = this->camera.unproject(hd, true);
    Matrix3f Jac_hCHd = this->camera.getUnprojectionJacobian();

    Matrix3f Rot = quat2rot(q);

    Vector3f hW = Rot*hC;

    float hx = hW(0);
    float hy = hW(1);
    float hz = hW(2);



    float theta = atan2(hx,hz);
    float phi = atan2(-hy, sqrt(hx*hx+hz*hz));

	// Updating state and Sigma
    MatrixXf Jx = this->computeJx();
	MatrixXf Jm = this->computeJm();
	Matrix2f Sigma_mm = this->computeSigma_mm(sigma_pixel, sigma_pixel);

	this->f = (VectorXf(6) << r, theta, phi, rho_0);
	this->Sigma_ii = Jm*Sigma_mm*Jm.transpose() + Jx*motion->getSigma_xx()*Jx.transpose();

	this->motion->updateVariaceBlocks(Jx);
	this->features->updateVariaceBlocks(Jx);
	this->features.push_back((*this));
    return 1;

}

FeatureModel::~FeatureModel() {
	// TODO Auto-generated destructor stub
}

Matrix3f FeatureModel::computeSigma_mm(float sigma_pixel, float sigma_rho) {
	Matrix3f Sigma_mm = pow(sigma_pixel,2)*Matrix3f::Identity();
	Sigma_mm(2,2) = pow(sigma_rho,2);
	return Sigma_mm;
}

Matrix<float, 6, 13> FeatureModel::computeJx() {
	// TODO
	Matrix<float, 6, 13> Jx = Matrix<float, 6, 13>::Zero();
	Jx.block<3,3>(0,0) = Matrix3f::Identity();

	MatrixXf Jf_hW = d_f_d_hW(hW);
    MatrixXf J_hW_q = dRq_times_d_dq(q,hC);
    Jx.block<6,4>(0,3) = Jf_hW*J_hW_q;
    return Jx;
}

Matrix<float, 6, 3> FeatureModel::computeJm() {

	// TODO
	Matrix<float, 6, 3> Jm = Matrix<float, 6, 6>::Zero();
    MatrixXf Jf_hW = d_f_d_hW(hW);
    Jm.block<6,2>(0,0) = Jf_hW*Rot*Jac_hCHd;
    Js.block<6,1>(0, 2) << 0,0,0,0,0,1;
    return Jm;
}

