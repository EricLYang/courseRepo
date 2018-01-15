#include "utils.hpp"



void plotFeatures(cv::Mat img, std::vector<cv::Point2f> features) {
    int fontFace = cv::FONT_HERSHEY_SCRIPT_SIMPLEX;
    double fontScale = 0.5;
    int thickness = 3;
    char buffer [33];
    
    for (int i = 0; i < features.size(); i++ ) {
        sprintf(buffer,"%d",i);
        std::string s(buffer);
        cv::circle(img, features[i], 10, CV_RGB(255, 32, 32));
        cv::putText(img, s, features[i], fontFace, fontScale, CV_RGB(32,255, 32), thickness, 8);
    }
}

MatrixXf vConcat(MatrixXf m1, MatrixXf m2) {
	if (m1.rows() == 0) return m2;
	else {
		MatrixXf concatMatrix(m1.rows()+m2.rows(), m1.cols());
		concatMatrix << m1,m2;
		return concatMatrix;
	}

}

VectorXf Concat(VectorXf m1, VectorXf  m2) {
	if (m1.rows() == 0) return m2;
	else {
		VectorXf concatMatrix(m1.rows()+m2.rows());
		concatMatrix << m1,m2;
		return concatMatrix;
	}

}

MatrixXf hConcat(MatrixXf  m1,  MatrixXf m2) {
	if (m1.cols() == 0) return m2;
	else {
		MatrixXf concatMatrix(m1.rows(), m1.cols() + m2.cols());
		concatMatrix << m1,m2;
		return concatMatrix;
	}

}
