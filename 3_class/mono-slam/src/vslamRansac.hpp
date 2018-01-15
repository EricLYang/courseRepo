#define DEBUG

#include <opencv2/opencv.hpp>
#include "camModel.hpp"

#include "Patch.hpp"

#include <eigen3/Eigen/Dense>

#include "ConfigVSLAM.h"



class VSlamFilter {
    CamModel cam;
    
	float dT;

    MatrixXf Vmax; //Covariance of speed (linear and rotation)

    MatrixXf Kt;
    MatrixXf Ft;
    MatrixXf Fnt;
    MatrixXf Ht; // measures prediction
    VectorXf h_out;
    VectorXf tot_h_out;

    MatrixXf St; // innovation covariance matrix
    
    
    
    cv::Mat frame, old_frame;
    cv::Mat originalImg;
    cv::Mat drawedImg;
    
    
    void drawPrediction();
    void drawUpdate(cv::Point f);
    
protected:
    float map_scale; // scale of the map

    std::vector<Patch> patches;
    std::vector<Patch> deleted_patches;

    VectorXf mu;
    MatrixXf Sigma;
	int nFeatures;
    float T_camera;
    ConfigVSLAM config;
	

    int windowsSize;
    int sigma_pixel;
    int sigma_pixel_2;

	int kernel_min_size;
	int scale;
	int sigma_size;

	int nInitFeatures;

	int camera_state_dim;

	float vz_odom;
	MatrixXf Hvz_odom;
	
	
	double old_ts;

	
public:



	float feature_index;

    VSlamFilter(char *file = NULL);
    int addFeature(cv::Point2f pf);

    void removeFeature(int index);

    void predict(float v_x = 0, float w_z = 0);
    void update(float v_x = 0, float w_z = 0);
    
    
    void captureNewFrame(cv::Mat newFrame);
    void captureNewFrame(cv::Mat newFrame, double time_stamp);
    
    cv::Mat returnImageDrawed();
    
    VectorXf getState();
    
    void findNewFeatures(int num = -1);

    void convert2XYZ(int index);
    void findFeaturesConvertible2XYZ();
    Vector3f depth2XYZ(VectorXf f, MatrixXf &J);

    int numOfFeatures();


};

Vector3f inverse2XYZ(VectorXf f, Vector3f r, MatrixXf &J_hp_f , Matrix3f R = Matrix3f::Identity());
Vector3f inverse2XYZ(VectorXf f, Vector3f r);
