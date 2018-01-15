/*
 * ConfigVSLAM.h
 *
 *  Created on: Jul 10, 2013
 *      Author: ludovico
 */

#ifndef CONFIGVSLAM_H_
#define CONFIGVSLAM_H_

#include <cstddef>
#include "camModel.hpp"



class ConfigVSLAM {
public:
	ConfigVSLAM(char *file = NULL);

	float sigma_vx, sigma_vy, sigma_vz;
	float sigma_wx, sigma_wy, sigma_wz;

	float rho_0;
	float sigma_rho_0;

	int window_size;
	int sigma_pixel;
	
	int kernel_size;
	int sigma_size;
	
	camConfig camParams;
	int scale;

	float T_camera;

	int nInitFeatures;
	int min_features;
	int max_features;

	int forsePlane;

};

#endif /* CONFIGVSLAM_H_ */
