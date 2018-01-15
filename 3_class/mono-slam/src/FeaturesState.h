/*
 * FeaturesState.h
 *
 *  Created on: Jul 22, 2013
 *      Author: ludovicorusso
 */

#ifndef FEATURESSTATE_H_
#define FEATURESSTATE_H_

#include <eigen3/Eigen/Dense>
#include "FeatureModel.h"

using namespace Eigen;
using namespace std;


class FeaturesState: public vector<FeatureModel>{
public:
	FeaturesState();
	virtual ~FeaturesState();
	void updateVariaceBlocks(MatrixXf Jx);
};

#endif /* FEATURESSTATE_H_ */
