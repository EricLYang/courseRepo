//#define EIGEN_DEFAULT_IO_FORMAT Eigen::IOFormat(2,1)

#include <ros/ros.h>

#include "vslamRansac.hpp"
#include "utils.hpp"


#include <iostream>
#include <iomanip>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <fstream>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Cholesky>

#define STATE_DIM 14

/**
 * Prototipes of the used funcitons
 *
 */





/**
 *
 * @param St
 * @param sigma_size
 * @return
 */
/* Compute the ellispoides parameters from features prediction covariance and store it
*  in a matrix of which each row represent a features ellipoide in form (a,b,theta), where
*  a and b are the semiaxis and theta are the rotation angle
*/
MatrixXi computeEllipsoidParameters(MatrixXf St, int sigma_size);


/**
 * Converts a Vector3f to a Quaternion
 * @param vec Vector to convert
 * @return Quaternion
 */
Vector4f vec2quat(Vector3f vec);

/**
 * Computes the rotation matrix associated to a quaternion
 * @param quat The quaternion
 * @return The rotation matrix
 */
Matrix3f quat2rot(Vector4f quat);

/**
 * Computes the quaternion complement associated to a given quaternion
 * @param quat The quaternion
 * @return The complement of quat
 */
Vector4f quatComplement(Vector4f quat);

Matrix3f diffQuat2rot(Vector4f quat, int index); // compute de partial derivate matrix of q2r(q) for q_index

Vector4f quatZero();
Vector4f quatCrossProduct(Vector4f h, Vector4f g);


// return Jacobian d(g(x))/dx
MatrixXf dg_x_dx(VectorXf X_old, float dT);


// compute the Jacobian between q(t) and q(t-1) knowing the rotation h = quat(w*T)
Matrix4f Jacobian_qt_qt1(Vector4f h);

// Computer the Jacobian df/dhW
MatrixXf d_f_d_hW(Vector3f hW);

// compute the Jacobian between q(t) and w(t-1) knowing the actual rotation q, w and dT
MatrixXf Jacobian_qt_w(Vector4f q, Vector3f w, float dT);

// return the Jacobian d(q^c)/d(q)
Matrix4f d_qbar_q();

// return Jacobian d(R(q)d)/dq
MatrixXf dRq_times_d_dq(Vector4f q, Vector3f d);

void normalizeQuaternion(VectorXf &mu, MatrixXf &Sigma);

/* state camera Prediction
	This function predicts the State of the camera using kwnoledge on Motion 
	and the time distance from the last update.
	
	Xv: last state of the camera
	dT: time distance
	
	return: Predicted state of the camera
*/
VectorXf fv(VectorXf Xv, double dT);


bool isInsideImage(Vector2f hi, cv::Size size, int windowsSize);



// VSLAM Class Implementation


int VSlamFilter::numOfFeatures() {
	return this->patches.size();
}

VectorXf VSlamFilter::getState() {
	VectorXf state(STATE_DIM);
	Vector4f qr;
	Matrix3f Rot;
	//Rot << 0, -1, 0, 0,0,-1,1,0,0;
	//qr << 0.5, 0.5, -0.5, 0.5;
	state = mu.segment<STATE_DIM>(0);

    return state;
}

VSlamFilter::VSlamFilter(char *file) : config(file) {

	old_ts = -1;
	float sigma_vv = 0.0004, sigma_ww = 0.0004;
	
	kernel_min_size = config.kernel_size;
	sigma_pixel = config.sigma_pixel;
	sigma_pixel_2 = sigma_pixel*sigma_pixel;
	sigma_size = config.sigma_size;
	windowsSize = config.window_size;

   	dT = 1;
	T_camera = config.T_camera;
	scale = config.scale;
	cam.setParam(config.camParams);


    mu = VectorXf::Zero(STATE_DIM); mu(3) = 1; mu(13) = 1; // initialization mu

    Vmax = MatrixXf::Identity(6,6);
	Vmax(0,0) = config.sigma_vx*config.sigma_vx;
	Vmax(1,1) = config.sigma_vy*config.sigma_vy;
	Vmax(2,2) = config.sigma_vz*config.sigma_vz;
	Vmax(3,3) = config.sigma_wx*config.sigma_wx;
	Vmax(4,4) = config.sigma_wy*config.sigma_wy;
	Vmax(5,5) = config.sigma_wz*config.sigma_wz;


	Sigma = 0.0000000004*MatrixXf::Identity(STATE_DIM,STATE_DIM);
	Sigma(13,13) = 0.09;

	Sigma.block<3,3>(7,7) = sigma_vv*sigma_vv*MatrixXf::Identity(3,3);
	Sigma.block<3,3>(10,10) = sigma_ww*sigma_ww*MatrixXf::Identity(3,3);

    nInitFeatures = config.nInitFeatures;

}



void VSlamFilter::captureNewFrame(cv::Mat newFrame, double time_stamp) {
	if (old_ts > 0) {
		dT = (time_stamp - old_ts);
		std::cout << "my Dt = " << dT*1000 << std::endl;
	}
	
	old_ts = time_stamp;
	captureNewFrame(newFrame);
}

void VSlamFilter::captureNewFrame(cv::Mat newFrame) {
	cv::resize(newFrame,newFrame,cv::Size(newFrame.size().width/scale, newFrame.size().height/scale));
	old_frame = frame.clone();
    originalImg = newFrame.clone();
    drawedImg = newFrame.clone();
    cvtColor( newFrame, frame, CV_BGR2GRAY );

}


int VSlamFilter::addFeature(cv::Point2f pf) {
    if (pf.x > windowsSize/2 && pf.y > windowsSize/2 && pf.x < frame.size().width - windowsSize/2 && pf.y < frame.size().height - windowsSize/2) {
	

    	int pos = mu.size();
        Patch newPat(cv::Mat(frame, cv::Rect(pf.x-windowsSize/2, pf.y-windowsSize/2, windowsSize,windowsSize)), pf, pos);
        patches.push_back(newPat);

		
        Vector2f hd;
        hd << (float)pf.x, (float)pf.y;
    
        Vector3f r = mu.segment<3>(0);
        Vector4f q = mu.segment<4>(3);
        
        MatrixXf Jac_hCHd;
        Vector3f hC = cam.UndistortAndDeproject(hd, Jac_hCHd);
        
        Matrix3f Rot = quat2rot(q);

        Vector3f hW = Rot*hC;

        float hx = hW(0);
        float hy = hW(1);
        float hz = hW(2);

        float ro = config.rho_0;


        float theta = atan2(hx,hz);
        float phi = atan2(-hy, sqrt(hx*hx+hz*hz));		

		// Updating state and Sigma

		VectorXf f(6);
		f << r, theta, phi, ro;
		mu = Concat(mu,f);


        int nOld = Sigma.rows();
        MatrixXf Js = MatrixXf::Zero(nOld+6, nOld+3);
        Js.block(0,0,nOld,nOld) = MatrixXf::Identity(nOld,nOld);
        Js.block<3,3>(nOld,0) = MatrixXf::Identity(3,3);
		
		


        MatrixXf Jf_hW = d_f_d_hW(hW);
        MatrixXf J_hW_q = dRq_times_d_dq(q,hC);

        Js.block<6,4>(nOld,3) = Jf_hW*J_hW_q;
        Js.block<6,2>(nOld,nOld) = Jf_hW*Rot*Jac_hCHd;
        Js.block<6,1>(nOld, nOld+2) << 0,0,0,0,0,1;

        MatrixXf S = sigma_pixel_2*MatrixXf::Identity(nOld+3, nOld+3);
        S.block(0,0, nOld, nOld) = Sigma;

        S(nOld + 2, nOld + 2) =  config.sigma_rho_0;

		Sigma = Js*S*Js.transpose();
        return 1;
    }
    return 0;
}

void VSlamFilter::removeFeature(int index) {
    int pos = patches[index].position_in_state;
    
    Patch p = patches[index];


    int first = pos;
    
    int size = (p.isXYZ()?3:6);


    if (p.n_find > 5) {
		if (p.isXYZ()) {
			p.XYZ_pos = mu.segment<3>(pos);
		} else {
			MatrixXf J;
			p.XYZ_pos = depth2XYZ(mu.segment<6>(pos), J);
		}
	    deleted_patches.push_back(p);
    }


    int dim = Sigma.cols();

    int last = mu.rows() - pos - size;

    if (index != patches.size() - 1) {
        mu = Concat(mu.head(first), mu.tail(last));
        Sigma = vConcat(Sigma.topRows(first), Sigma.bottomRows(last));
        Sigma = hConcat(Sigma.leftCols(first), Sigma.rightCols(last));
    } else {
        mu = mu.head(pos).eval();
        Sigma = Sigma.block(0,0,pos,pos).eval();
    }

    for(int i = index + 1; i < patches.size(); i++) {
    	patches[i].change_position(-size);
    }

    patches.erase(patches.begin()+index);
}


void VSlamFilter::predict(float v_x, float w_z) {
	Ft = dg_x_dx(mu.segment<13>(0), dT);
    const int n = Sigma.cols();
    
    MatrixXf Ft_complete = MatrixXf::Identity(n,n);
    Ft_complete.block<13,13>(0,0) = Ft;
    MatrixXf Q;
    Q = Ft.middleCols<6>(7)*(Vmax/dT/dT)*Ft.middleCols<6>(7).transpose();


    MatrixXf Qtot = MatrixXf::Zero(n,n);
    Qtot.block<13,13>(0,0) = Q;
    Sigma = (Ft_complete*Sigma*Ft_complete.transpose() + Qtot).eval();

    mu.segment<13>(0) = fv(mu.segment<13>(0), dT);
    Vector3f r = mu.segment<3>(0);
    Vector4f q = mu.segment<4>(3);
    Vector3f v = mu.segment<3>(7);
    Vector3f w = mu.segment<3>(10);
    Vector4f h = vec2quat(w);

    map_scale = mu(13);



    Vector4f qc = quatComplement(q); // quaternion Complement
    Matrix3f RotCW = quat2rot(qc);

    // blurring components


    Matrix3f RotCW_blurring = quat2rot(quatComplement(quatCrossProduct(q,vec2quat(w*T_camera*dT))));
    Vector3f r_blurring = r + v*T_camera*dT;
    Vector2f hi_out_blurred;


    VectorXf hi_out;
    MatrixXf Hit;
    
    int featurer_counter = 0;
    
    for (int i = 0; i < patches.size(); i++) {
		int pos = patches[i].position_in_state;
        if (!patches[i].isXYZ()) {                                // Feature in Inverse Depth Form
            Hit = MatrixXf::Zero(2, mu.rows());
            VectorXf f = mu.segment<6>(pos);

            if (f(5) <= 0) {
            	ROS_ERROR("Feature %d ad infinity: %f", i, f(5));
            	patches[i].setRemove();
            	patches[i].setIsInInnovation(false);
            	continue;
            }
            MatrixXf J_hW_f;
            Vector3f d = inverse2XYZ(f, r, J_hW_f);
            Vector3f hC = RotCW*d;
            MatrixXf J_h_hC;
            hi_out = cam.projectAndDistort(hC, J_h_hC);
            bool flag = isInsideImage(hi_out, frame.size(), windowsSize) && hC(2) >= 0 && f(5) > 0;

			patches[i].setIsInInnovation(flag);
            if (!flag) continue;
			
			featurer_counter++;
            MatrixXf d_h_q = dRq_times_d_dq(qc,d)*d_qbar_q();
                        
            Hit.middleCols<3>(0) = -f(5)*J_h_hC*RotCW;
            Hit.middleCols<4>(3) = J_h_hC*d_h_q;
            Hit.middleCols<6>(pos) = J_h_hC*RotCW*J_hW_f;
            patches[i].h = hi_out;
			patches[i].H = Hit;


			// Blurring
			hi_out_blurred = cam.projectAndDistort(RotCW_blurring*inverse2XYZ(f,r_blurring, J_hW_f), J_h_hC);
			patches[i].blur(hi_out, hi_out_blurred,kernel_min_size );


//#define TEST_BLURR
#ifdef TEST_BLURR
			for (float delta_T_camera = 0; delta_T_camera <= 1; delta_T_camera +=0.1) {
			    Matrix3f RotCW_blurring = quat2rot(quatComplement(quatCrossProduct(q,vec2quat(w*delta_T_camera))));
			    Vector3f r_blurring = r + v*delta_T_camera;
				MatrixXf df_blur = cam.projectAndDistort(RotCW_blurring*inverse2XYZ(f,r_blurring, J_hW_f), J_h_hC);
				patches[i].blurTest(hi_out, df_blur ,kernel_min_size, i, 10*delta_T_camera);
			}
#endif



        } else {                                // Feature in Inverse Depth Form
            Hit = MatrixXf::Zero(2, mu.rows());
            Vector3f y = mu.segment<3>(pos);

            Vector3f d = y-r;
            Vector3f hC = RotCW*d;
            MatrixXf J_h_hC;
            hi_out = cam.projectAndDistort(hC, J_h_hC);
            bool flag = isInsideImage(hi_out, frame.size(), windowsSize) && hC(2) >= 0;

			patches[i].setIsInInnovation(flag);
            if (!flag) continue;

			featurer_counter++;
            MatrixXf d_h_q = dRq_times_d_dq(qc,d)*d_qbar_q();

            Hit.middleCols<3>(0) = -J_h_hC*RotCW;
            Hit.middleCols<4>(3) = J_h_hC*d_h_q;
            Hit.middleCols<3>(pos) = J_h_hC*RotCW;
            patches[i].h = hi_out;
			patches[i].H = Hit;


			// Blurring
			hi_out_blurred = cam.projectAndDistort(RotCW_blurring*(y-r_blurring), J_h_hC);
			patches[i].blur(hi_out, hi_out_blurred,kernel_min_size );

#ifdef TEST_BLURR
			for (float delta_T_camera = 0; delta_T_camera <= 1; delta_T_camera +=0.1) {
			    Matrix3f RotCW_blurring = quat2rot(quatComplement(quatCrossProduct(q,vec2quat(w*delta_T_camera))));
			    Vector3f r_blurring = r + v*delta_T_camera;
				MatrixXf df_blur = cam.projectAndDistort(RotCW_blurring*(y-r_blurring), J_h_hC);
				patches[i].blurTest(hi_out, hi_out_blurred,kernel_min_size, i, 10*delta_T_camera);
			}
#endif


        }
    }

    VectorXf temp_h_out;
    MatrixXf temp_Ht;

	for (int i = 0, j = 0; i < patches.size(); i++) {
		if (patches[i].patchIsInInnovation()) {
			temp_h_out = Concat(temp_h_out,patches[i].h);
			temp_Ht = vConcat(temp_Ht,patches[i].H);
			patches[i].position_in_z = 2*j;
			j++;
		}
	}    
	
    h_out = temp_h_out;
    Ht = temp_Ht;
    

    if (h_out.rows() > 0) {
        St = (Ht*Sigma*Ht.transpose() + sigma_pixel_2*MatrixXf::Identity(Ht.rows(), Ht.rows())).eval();
        this->drawPrediction();
    }
}


Vector3f VSlamFilter::depth2XYZ(VectorXf f, MatrixXf &J) {

	const float theta = f(3);
    const float phi = f(4);
    const float ro = f(5);
    Vector3f m;
    m(0) = sin(theta)*cos(phi);
    m(1) = -sin(phi);
    m(2) = cos(theta)*cos(phi);

    Vector3f y = f.segment<3>(0) + m/ro;



    J = MatrixXf::Zero(3,6);
    J.middleCols<3>(0) = Matrix3f::Identity();

    J(0,3) = cos(theta)*cos(phi)/ro;
    J(1,3) = 0;
    J(2,3) = -sin(theta)*cos(phi)/ro;

    J(0,4) = -sin(theta)*sin(phi)/ro;
    J(1,4) = -cos(phi)/ro;
    J(2,4) = -cos(theta)*sin(phi)/ro;

    J.col(5) = -m/(ro*ro);
    return y;

}


void VSlamFilter::convert2XYZ(int index) {
	int pos = patches[index].position_in_state;
	int size = 6;
	int dx = 3;
	VectorXf f = mu.segment<6>(pos);
	const float theta = f(3);
    const float phi = f(4);
    const float ro = f(5);
    Vector3f m;
    m(0) = sin(theta)*cos(phi);
    m(1) = -sin(phi);
    m(2) = cos(theta)*cos(phi);

    Vector3f y = f.segment<3>(0) + m/ro;

    Vector3f d = y - mu.segment<3>(0);

    // controllo sul L_i

    float sigma = Sigma(pos+5,pos+5);

    float t = d.transpose()*m;

    float L = 4*sigma*abs(t)/(ro*ro*d.squaredNorm());
    if (L <= 0) {
    	patches[index].setRemove();
    }
    if (L < 0.01) {


        MatrixXf J_hp_f = MatrixXf::Zero(3,6);
        J_hp_f.middleCols<3>(0) = Matrix3f::Identity();


        J_hp_f(0,3) = cos(theta)*cos(phi)/ro;
        J_hp_f(1,3) = 0;
        J_hp_f(2,3) = -sin(theta)*cos(phi)/ro;

        J_hp_f(0,4) = -sin(theta)*sin(phi)/ro;
        J_hp_f(1,4) = -cos(phi)/ro;
        J_hp_f(2,4) = -cos(theta)*sin(phi)/ro;

        J_hp_f.col(5) = -m/(ro*ro);


        int first = pos;
        int last = mu.rows() - pos - 3;


        if (index != patches.size() - 1) {
            mu = Concat(mu.head(first), mu.tail(last));
            mu.segment<3>(pos) = y;
        } else {
            mu = Concat(mu.head(first), y);
        }

        int n = Sigma.cols();
        MatrixXf J = MatrixXf::Zero(n-3,n);
        J.topLeftCorner(pos,pos) = MatrixXf::Identity(pos,pos);
        J.block<3,6>(pos,pos) = J_hp_f;
        if (index != patches.size() - 1) {
        	J.bottomRightCorner(n-pos-6,n-pos-6) = MatrixXf::Identity(n-pos-6,n-pos-6);
        }

        Sigma = J*Sigma*J.transpose();

        patches[index].convertInXYZ();

        for(int i = index + 1; i < patches.size(); i++) {
        	patches[i].change_position(-3);
        }
    }
}


void VSlamFilter::findFeaturesConvertible2XYZ() {
	for (int i = 0; i < patches.size(); i++) {
		if (!patches[i].isXYZ()) convert2XYZ(i);
	}

}



void VSlamFilter::findNewFeatures(int num) {
	ROS_INFO("Finding new Feature");

	if (num <= 0) num = nInitFeatures;

    int winSize = 2*windowsSize+1;
    cv::Mat mask = cv::Mat(frame.size(),CV_8UC1 );
    mask.setTo(cv::Scalar(0));
	int estrem = windowsSize;
    cv::Mat(mask, cv::Rect(estrem, estrem,mask.size().width-2*estrem, mask.size().height - 2*estrem)).setTo(cv::Scalar(255));



    for (int i = 0; i < patches.size(); i++) {
    	if (patches[i].center.x > windowsSize && 
    		patches[i].center.y > windowsSize && 
    		patches[i].center.x < mask.size().width - windowsSize && 
    		patches[i].center.y < mask.size().height - windowsSize) 
    		{
    		int x = (patches[i].center.x-winSize/2 > 0 ? 
    				patches[i].center.x-winSize/2 : 0);
    		int y = (patches[i].center.y-winSize/2 > 0 ? 
    				patches[i].center.y-winSize/2 : 0);
    		int width = winSize - 
    			(patches[i].center.x+winSize/2 <= frame.size().width ? 
    			0 : frame.size().width - patches[i].center.x-winSize/2);
    		
			int height = winSize - 
    			(patches[i].center.y+winSize/2 <= frame.size().height ? 
    			0 : frame.size().height - patches[i].center.y-winSize/2);
    			
    		cv::Rect roi = cv::Rect(x,y, width, height);
    		cv::Mat(mask, roi).setTo(cv::Scalar(0));

    	}
    }
    std::vector<cv::Point2f> features;
	
	
    goodFeaturesToTrack(frame,features,num,0.01f,5,mask);

    for (int i = 0; i < features.size(); i++) {
	cv::Point2f newFeature = cv::Point2f(features[i].x, features[i].y);
        this->addFeature(newFeature);
    }
    
	ROS_DEBUG("Found %d Feature[s]", features.size());

}

void VSlamFilter::update(float v_x, float w_z) {

    cv::Point2f locF;
    int matchedFeatures = 0;
    int searchedFeatures = 0;

	
    for (int i = 0; i < patches.size(); i++) {
        if (patches[i].patchIsInInnovation()) {
            patches[i].findMatch(frame, St.block<2,2>(patches[i].position_in_z,patches[i].position_in_z),sigma_size, false);
            searchedFeatures++;
        }
    }

#ifdef USE_RANSAC

	int nhyp = 10000;
	float p = 0.99;
	float th = 2*sigma_pixel;
	
	srand(time(NULL));
	
	int num_zli = 0;
	
	std::vector<int> ransacindex;


	for (int i = 0; i < patches.size(); i++) {
		if (patches[i].patchIsInInnovation())
			ransacindex.push_back(i);
	}


	for (int i = 0; i < nhyp && ransacindex.size() > 0; i++) {
		int actual_num_zli = 0;

		int posRansac = rand()%ransacindex.size();
		int selectPatch = ransacindex[posRansac];
		ransacindex.erase(ransacindex.begin() + posRansac);

		MatrixXf S_i = patches[selectPatch].H*Sigma*patches[selectPatch].H.transpose() + sigma_pixel_2*MatrixXf::Identity(patches[selectPatch].H.rows(), patches[selectPatch].H.rows());
        MatrixXf K_i = Sigma*patches[selectPatch].H.transpose()*S_i.inverse();
		VectorXf mu_i = mu + K_i*(patches[selectPatch].z - patches[selectPatch].h);
		
		Vector3f r = mu_i.segment<3>(0);
		Vector4f q = mu_i.segment<4>(3)/mu_i.segment<4>(3).norm();
    	Matrix3f RotCW = quat2rot(quatComplement(q));	
		
    	int searched_features = 0;
		for (int i = 0; i < patches.size(); i++) {
			if (!patches[i].patchIsInInnovation()) continue;
			Vector2f hi_i;
			int pos = patches[i].position_in_state;

			if (!patches[i].isXYZ()) {
				VectorXf f = mu_i.segment<6>(pos);
				Vector3f d = inverse2XYZ(f, r);
				hi_i = cam.projectAndDistort(RotCW*d);
			} else {
	            Vector3f y = mu.segment<3>(pos);
	            Vector3f d = y-r;
	            Vector3f hC = RotCW*d;
	            MatrixXf J_hf_hC;
	            hi_i = cam.projectAndDistort(hC, J_hf_hC);
			}

			patches[i].setIsInLi((patches[i].z - hi_i).norm() <= th);
			if (patches[i].patchIsInLi()) actual_num_zli++;
			searched_features++;

    	}
    	
    	if (actual_num_zli > num_zli) {
    		num_zli = actual_num_zli;
    		nhyp = log(1-p)/(log(num_zli/(matchedFeatures+0.0f)));
    		for (int i = 0; i < patches.size(); i++) {
    			if (patches[i].patchIsInInnovation()) patches[i].ConfirmIsInLi();
    		}
    	}
	}
	

	VectorXf z_li;
    MatrixXf H_li;
    VectorXf h_li;

	for (int i = 0, j = 0; i < patches.size(); i++) {
		if (patches[i].patchIsInLi()) {
			h_li = Concat(h_li,patches[i].h);
			H_li = vConcat(H_li,patches[i].H);
			z_li = Concat(z_li, patches[i].z);
			patches[i].position_in_z = 2*j;
			j++;
		}
	}    


	MatrixXf Sigma_tmp = Sigma;
	VectorXf mu_tmp = mu;

    if (z_li.rows() > 0) {
    	int p = H_li.rows();
    	St = H_li*Sigma_tmp*H_li.transpose() + sigma_pixel_2*MatrixXf::Identity(p,p);
    	//Kt = Sigma*H_li.transpose()*St.inverse();
    	//MatrixXf Stinv = MatrixXf::Identity(p,p);
    	//St.llt().solveInPlace(Stinv);
    	Kt = Sigma_tmp*H_li.transpose()*St.inverse();//

    	//	MatrixXf Stinv = St.lu().solve(Matrix<float,Dynamic,Dynamic>::Identity(p,p));
    	//	Kt = Sigma*H_li.transpose()*Stinv;

    	mu_tmp = mu + Kt*(z_li - h_li);
    	Sigma_tmp = ((MatrixXf::Identity(Sigma_tmp.rows(),Sigma_tmp.rows()) - Kt*H_li)*Sigma_tmp);
       	normalizeQuaternion(mu_tmp,Sigma_tmp);
    } else {
        ROS_ERROR("No Matching li");
    }

    
    //////////////////////////////////////////////////////////
	th = 1;
    if (1) {
		Vector3f r = mu.segment<3>(0);
		Vector4f q = mu.segment<4>(3);
		Vector4f qc = quatComplement(q);
		Matrix3f RotCW = quat2rot(qc);
		Matrix2f S_hi;
		Vector2f hi_hi;



		int counter = 0;
		for (int i = 0; i < patches.size(); i++) {
			if (!patches[i].patchIsInLi() && patches[i].patchIsInInnovation()) {
				int pos = patches[i].position_in_state;
				if (!patches[i].isXYZ()) {
					MatrixXf Hi_hi = MatrixXf::Zero(2, mu.rows());
					VectorXf f = mu.segment<6>(pos);
					MatrixXf J_hp_f;
					Vector3f d = inverse2XYZ(f, r, J_hp_f);
					MatrixXf J_hf_hi;
					patches[i].h = cam.projectAndDistort(RotCW*d, J_hf_hi);

		            MatrixXf d_h_q = dRq_times_d_dq(qc,d)*d_qbar_q();
		            Hi_hi.middleCols<3>(0) = -f(5)*J_hf_hi*RotCW;
		            Hi_hi.middleCols<4>(3) = J_hf_hi*d_h_q;
		            Hi_hi.middleCols<6>(pos) = J_hf_hi*RotCW*J_hp_f;
					patches[i].H = Hi_hi;


				} else {
					MatrixXf Hi_hi = MatrixXf::Zero(2, mu.rows());
		            Vector3f y = mu.segment<3>(pos);
		            Vector3f d = y-r;
		            Vector3f hC = RotCW*d;
		            MatrixXf J_hf_hC;
		            patches[i].h = cam.projectAndDistort(hC, J_hf_hC);

		            MatrixXf d_h_q = dRq_times_d_dq(qc,d)*d_qbar_q();

		            Hi_hi.middleCols<3>(0) = -J_hf_hC*RotCW;
		            Hi_hi.middleCols<4>(3) = J_hf_hC*d_h_q;
		            Hi_hi.middleCols<3>(pos) = J_hf_hC*RotCW;
					patches[i].H = Hi_hi;

				}

				S_hi = patches[i].H*Sigma_tmp*patches[i].H.transpose();
				patches[i].setIsInHi((patches[i].h - patches[i].z).transpose()*S_hi.inverse()*(patches[i].h - patches[i].z) <= th);
				counter++;
			}
		}
    }


    VectorXf z_hi;
    MatrixXf H_hi;
    VectorXf h_hi;
	for (int i = 0, j = 0; i < patches.size(); i++) {
		if (patches[i].patchIsInHi()) {
			h_hi = Concat(h_hi,  patches[i].h);
			H_hi = vConcat(H_hi, patches[i].H);
			z_hi = Concat(z_hi,  patches[i].z);
			patches[i].position_in_z = 2*j;
			j++;
		}
	}    

#endif


#ifndef USE_RANSAC
    VectorXf z_hi;
    MatrixXf H_hi;
    VectorXf h_hi;
	for (int i = 0, j = 0; i < patches.size(); i++) {
		if (patches[i].patchIsInInnovation()) {
			h_hi = Concat(h_hi,  patches[i].h);
			H_hi = vConcat(H_hi, patches[i].H);
			z_hi = Concat(z_hi,  patches[i].z);
			patches[i].position_in_z = 2*j;
			j++;
		}
	}
#endif



	bool flag_correction = false;
	Matrix3f corr_ass_sigma = 0.00001*Matrix3f::Identity();

	if (config.forsePlane) {
		Vector3f corr_ass = Vector3f::Zero();
		MatrixXf corr_ass_H = MatrixXf::Zero(3,mu_tmp.size());
		corr_ass_H(0,1) = 1;
		corr_ass_H(1,4) = 1;
		corr_ass_H(2,6) = 1;

		Vector3f h_corr_ass;
		h_corr_ass << mu_tmp(1), mu_tmp(4),  mu_tmp(6);
		h_hi = Concat(h_hi,  h_corr_ass);
		z_hi = Concat(z_hi,  corr_ass);
		H_hi = vConcat(H_hi, corr_ass_H);
		flag_correction = true;
	}




    if (z_hi.rows() > 0) {
    	int p = H_hi.rows();
    	St = H_hi*Sigma_tmp*H_hi.transpose();

    	MatrixXf St_error = sigma_pixel_2*MatrixXf::Identity(p,p);
    	//if (flag_odom) St_error(p-1,p-1) = 0.2;
    	if (flag_correction) St_error.block<3,3>(p-3,p-3) = corr_ass_sigma;

    	St += St_error;

    	Kt = Sigma_tmp*H_hi.transpose()*St.inverse();//lu().solve(Matrix<float,Dynamic,Dynamic>::Identity(p,p));
    	mu_tmp = mu_tmp + Kt*(z_hi - h_hi);
        Sigma_tmp = ((MatrixXf::Identity(Sigma_tmp.rows(),Sigma_tmp.rows()) - Kt*H_hi)*Sigma_tmp).eval();
       	normalizeQuaternion(mu_tmp,Sigma_tmp);
    }
    
    if (1) {
    	mu = mu_tmp;
    	Sigma = Sigma_tmp;
    }



    for (int i = 0; i < patches.size(); i++) {
    	patches[i].drawUpdate(drawedImg, i);
    }


	for (int i = patches.size()-1; i >= 0; --i){
		patches[i].update_quality_index();
		if (patches[i].mustBeRemove()) this->removeFeature(i);
	}

    int nVisibleFeature = 0;
    for (int i = 0; i < patches.size(); i++) if (patches[i].patchIsInInnovation()) nVisibleFeature++;

    if (nVisibleFeature < config.min_features) {
    	if (patches.size() > config.max_features) this->removeFeature(0);
    	this->findNewFeatures(5);
    }

	findFeaturesConvertible2XYZ();
}


void VSlamFilter::drawUpdate(cv::Point f) {
    cv::rectangle(drawedImg, cv::Point(f.x - windowsSize/2, f.y - windowsSize/2), cv::Point(f.x + windowsSize/2, f.y + windowsSize/2), CV_RGB(0,0,255), 1);
    cv::circle(drawedImg, f, 2, CV_RGB(0,0,255),2);
    
	/*std::stringstream text;
	text << 1;
	cv::putText(drawedImg,text.str(), f,cv::FONT_HERSHEY_SIMPLEX, 2, CV_RGB(0,0,255));
	*/
}


void VSlamFilter::drawPrediction() {
    
    MatrixXi sigma_mis = computeEllipsoidParameters(St, sigma_size);
    for (int i = 0; i < St.rows()/2; i++) {
        cv::Point2i predictFeature(h_out(2*i), h_out(2*i+1));
        cv::ellipse(drawedImg, predictFeature, cv::Size(sigma_mis(i,0),sigma_mis(i,1)), (double)sigma_mis(i,2), 0, 360, CV_RGB(32, 32, 255));
    }
}

cv::Mat VSlamFilter::returnImageDrawed() {
    return drawedImg.clone();
}
MatrixXi computeEllipsoidParameters(MatrixXf St, int sigma_size) {
    int nFeatures = St.cols()/2;
    MatrixXi sigma_mis(nFeatures,3);
    for (int i = 0; i < nFeatures; i++) {
        SelfAdjointEigenSolver<MatrixXf> eigenSolver(St.block<2,2>(2*i,2*i));
        Vector2f eigs = eigenSolver.eigenvalues();
        Matrix2f vecs = eigenSolver.eigenvectors();
 
        sigma_mis(i,0) = (eigs(0) > 0 ? (int)sigma_size*sqrt((double)eigs(0)) : 1);
        sigma_mis(i,1) = (eigs(1) > 0 ? (int)sigma_size*sqrt((double)eigs(1)) : 1);
        sigma_mis(i,2) = (int)(180/3.14*atan2((double)vecs(1,0), (double)vecs(0,0)));
    }
    return sigma_mis;
}




/*************************/

Vector4f vec2quat(Vector3f vec) {
    float alpha = vec.norm();
    
    if (alpha != 0) {
        Vector4f quat;
        quat(0) = cos(alpha/2);
        quat.segment<3>(1) = vec*sin(alpha/2)/alpha;
        return quat;
    }
    else {
        return quatZero();
    }
}

Vector4f quatZero() {
    Vector4f quat;
    quat << 1,0,0,0;
    return quat;
}

Matrix3f quat2rot(Vector4f quat) {
    float qr = quat(0);
    float qi = quat(1);
    float qj = quat(2);
    float qk = quat(3);
    
    const float w = quat(0);
    const float x = quat(1);
    const float y = quat(2);
    const float z = quat(3);

    Matrix3f rot;

	rot <<	 qr*qr + qi*qi - qj*qj - qk*qk, 		-2*qr*qk+2*qi*qj,					 2*qr*qj+2*qi*qk,
		 2*qr*qk+2*qi*qj,				 qr*qr - qi*qi + qj*qj - qk*qk,				-2*qr*qi+2*qj*qk,
		-2*qr*qj+2*qi*qk,				 2*qr*qi+2*qj*qk,					 qr*qr - qi*qi - qj*qj + qk*qk;
    /*
	rot <<	 qr*qr + qi*qi - qj*qj - qk*qk, 		-2*qr*qk+2*qi*qj,					 2*qr*qj+2*qi*qk,
		 2*qr*qk+2*qi*qj,				 qr*qr - qi*qi + qj*qj - qk*qk,				-2*qr*qi+2*qj*qk,
		-2*qr*qj+2*qi*qk,				 2*qr*qi+2*qj*qk,					 qr*qr - qi*qi - qj*qj + qk*qk;


	const float norm = quat.segment<3>(1).norm();
	Vector3f v = quat.segment<3>(1);
	Matrix3f skew;
	skew << 0, -z, y, z, 0, -x, -y, x, 0;
	rot = Matrix3f::Identity()*(1 - 2*norm*norm) + 2*(v*v.transpose() + w*skew);
*/
    return rot;
}

Matrix4f YupsilonMatric(Vector4f q1) {
	Matrix4f res;
    	const float r1=q1(0);
    	const float x1=q1(1);
    	const float y1=q1(2);
   	const float z1=q1(3);
	res <<  r1, -x1, -y1, -z1,
		x1,  r1, -z1,  y1,
		y1,  z1,  r1, -x1,
		z1, -y1,  x1,  r1;
	
	return res;
		
}


Matrix4f YupsilonMatricComplementar(Vector4f q1) {
	Matrix4f res;
    	const float r1=q1(0);
    	const float x1=q1(1);
    	const float y1=q1(2);
   	const float z1=q1(3);
	res <<  r1, -x1, -y1, -z1,
		x1,  r1,  z1, -y1,
		y1, -z1,  r1,  x1,
		z1,  y1, -x1,  r1;

	return res;
		
}

Vector4f quatCrossProduct(Vector4f q1, Vector4f q2) {
    Vector4f q;
    // First quaternion q1 (x1 y1 z1 r1)
    const float r1=q1(0);
    const float x1=q1(1);
    const float y1=q1(2);
    const float z1=q1(3);
    
    
    // Second quaternion q2 (x2 y2 z2 r2)
    const float r2=q2(0);
    const float x2=q2(1);
    const float y2=q2(2);
    const float z2=q2(3);
    
    q(0) = r1*r2 - x1*x2 - y1*y2 - z1*z2;   // r component
    q(1) = x1*r2 + r1*x2 + y1*z2 - z1*y2;   // x component
    q(2) = r1*y2 - x1*z2 + y1*r2 + z1*x2;   // y component
    q(3) = r1*z2 + x1*y2 - y1*x2 + z1*r2;   // z component


    
    return YupsilonMatric(q1)*q2;
}



Vector3f inverse2XYZ(VectorXf f, Vector3f r, MatrixXf &J_hp_f, Matrix3f R) {
    const float theta = f(3);
    const float phi = f(4);
    const float ro = f(5);
    
    Vector3f m;
    m(0) = sin(theta)*cos(phi);
    m(1) = -sin(phi);
    m(2) = cos(theta)*cos(phi);
    
    J_hp_f = MatrixXf(3,6);
    J_hp_f.middleCols<3>(0) = ro*Matrix3f::Identity();

    J_hp_f(0,3) = cos(theta)*cos(phi);
    J_hp_f(1,3) = 0;
    J_hp_f(2,3) = -sin(theta)*cos(phi);
    
    J_hp_f(0,4) = -sin(theta)*sin(phi);
    J_hp_f(1,4) = -cos(phi);
    J_hp_f(2,4) = -cos(theta)*sin(phi);
    
    J_hp_f.col(5) = f.segment<3>(0)-r;
    
    
    //J_hp_f= R*J_hp_f;

    
    MatrixXf ret;
    //ret =  R*(ro*(f.segment<3>(0)-r) + m);
    ret =  (ro*(f.segment<3>(0)-r) + m);
    return ret;
}

Vector3f inverse2XYZ(VectorXf f, Vector3f r) {
    const float theta = f(3);
    const float phi = f(4);
    const float ro = f(5);
    
    Vector3f m;
    m(0) = sin(theta)*cos(phi);
    m(1) = -sin(phi);
    m(2) = cos(theta)*cos(phi);
  
    return ro*(f.segment<3>(0)-r) + m;
}

// return Jacobian d(g(x))/dx
MatrixXf dg_x_dx(VectorXf mu, float dT) {
	Vector3f r = mu.segment<3>(0);
    Vector4f q = mu.segment<4>(3);
    Vector3f v = mu.segment<3>(7);
    Vector3f w = mu.segment<3>(10);
    Vector4f h = vec2quat(dT*w); // h = quat(w)

	MatrixXf Ft;
    Ft = MatrixXf::Identity(13,13);
    Ft.block<4,4>(3,3) = Jacobian_qt_qt1(h);
    Ft.block<4,3>(3,10) = Jacobian_qt_w(q,w,dT);
	Ft.block<3,3>(0,7) = dT*MatrixXf::Identity(3,3);

	return Ft;
}



// compute the Jacobian between q(t) and q(t-1) knowing the rotation h = quat(w*T)
Matrix4f Jacobian_qt_qt1(Vector4f h) {
    const float hr=h(0);
    const float hx=h(1);
    const float hy=h(2);
    const float hz=h(3);
    
    Matrix4f ret;
    ret <<  hr, -hx, -hy, -hz,
            hx, hr, hz, -hy,
            hy, -hz, hr, hx,
            hz, hy, -hx, hr;

    return YupsilonMatricComplementar(h);    
}

// compute the Jacobian between q(t) and w(t-1) knowing the actual rotation q, w and dT
MatrixXf Jacobian_qt_w(Vector4f q, Vector3f w, float dT) {
    const float qr=q(0);
    const float qx=q(1);
    const float qy=q(2);
    const float qz=q(3);
    
    const float n = w.norm();
    const float s = sin(dT*n/2);
    const float c = cos(dT*n/2);
    const float Sinc = (n == 0 ? 1: 2*sin(dT*n/2)/(dT*n));
    
    Vector3f n_w;
    if (n > 0) n_w = w/n;
    
    Matrix4f t1;
	t1 << 	qr, -qx, -qy, -qz,
           	qx, qr, -qz, qy,
        	qy, qz, qr, -qx,
            qz, -qy, qx, qr;

	t1 = YupsilonMatric(q);
    
    MatrixXf t2 = MatrixXf::Zero(4,3);
    t2.row(0) = -dT*0.5*s*n_w.transpose();
    t2.middleRows<3>(1) = dT*0.5*(Sinc*Matrix3f::Identity() + (c-Sinc)*n_w*n_w.transpose());

	MatrixXf ret;
	ret = t1*t2;
    return ret;
}

Matrix3f diffQuat2rot(Vector4f quat, int index) {
    Matrix3f dR;
    const float q0_2 = 2*quat(0);
    const float qx_2 = 2*quat(1);
    const float qy_2 = 2*quat(2);
    const float qz_2 = 2*quat(3);

    const float w = quat(0);
    const float x = quat(1);
    const float y = quat(2);
    const float z = quat(3);
    
    if (index == 0) {
        dR << 	 q0_2, 	-qz_2,	 qy_2,
        		 qz_2,	 q0_2,	-qx_2,
        		-qy_2,	 qx_2,	 q0_2;

 		/*dR << 	 0, 	-qz_2,	 qy_2,
        		 qz_2,	 0,	-qx_2,
        		-qy_2,	 qx_2,	 0;*/
        
    } else if (index == 1) {
        dR << 	qx_2, 	 qy_2, 	 qz_2,
        		qy_2,  	-qx_2, 	-q0_2,
        		qz_2,	 q0_2,	-qx_2;

		/*dR << 	0, 	 qy_2, 	 qz_2,
        		qy_2,  	-2*qx_2, -q0_2,
        		qz_2,	 q0_2,	-2*qx_2;*/
        
    } else if (index == 2) {
        dR <<	-qy_2,     qx_2,      q0_2,
        		 qx_2,     qy_2,      qz_2,
        		-q0_2,     qz_2,     -qy_2;

  		/*dR <<	-2*qy_2,     qx_2,      q0_2,
        		 qx_2,     0,      	qz_2,
        		-q0_2,     qz_2,     -2*qy_2;*/
        
    } else if (index == 3) {
        dR <<	-qz_2,		-q0_2, 	qx_2,
        		 q0_2,      -qz_2,  qy_2,
        		 qx_2,       qy_2,  qz_2;

        /*dR <<	-2*qz_2,		-q0_2, 	qx_2,
        		 q0_2,      -2*qz_2,  qy_2,
        		 qx_2,       qy_2,  0;*/
    }
    return dR;
}

Vector4f quatComplement(Vector4f quat) {
    Vector4f qC;
    qC << quat(0), -quat(1), -quat(2), -quat(3);
    return qC;
}

// state camera Update
VectorXf fv(VectorXf Xv, double dT) {


	Vector3f v = Xv.segment<3>(7);
	Vector3f w = Xv.segment<3>(10);

    Xv.segment<3>(0) += dT*v;
    Xv.segment<4>(3) = quatCrossProduct( Xv.segment<4>(3),vec2quat(dT*w));

    Xv.segment<3>(7) = v;
	Xv.segment<3>(10) = w;
    return Xv;
}

Matrix4f d_qbar_q() {
    Matrix4f J = -Matrix4f::Identity();
    J(0,0) = 1;
    return J;
}


// Compute the Jacobian df/dhW
MatrixXf d_f_d_hW(Vector3f hW) {
    const float hx = hW(0);
    const float hy = hW(1);
    const float hz = hW(2);
    
    // d_Theta_hW
    MatrixXf J_Theta_hW(1,3);
    float normal = hx*hx+hz*hz;
    J_Theta_hW(0,0) = hz/normal;
    J_Theta_hW(0,1) = 0;
    J_Theta_hW(0,2) = -hx/normal;

    
    // d_Phi_hW
    float normal2 = hx*hx+hy*hy+hz*hz;
    MatrixXf J_Phi_hW(1,3);
    J_Phi_hW(0,0) = hx*hy/sqrt(normal)/normal2;
    J_Phi_hW(0,1) = -sqrt(normal)/normal2;
    J_Phi_hW(0,2) = hz*hy/sqrt(normal)/normal2;
    
    
    MatrixXf J_f_hW = MatrixXf::Zero(6,3);
    J_f_hW.row(3) = J_Theta_hW;
    J_f_hW.row(4) = J_Phi_hW;

    return J_f_hW;
}


void normalizeQuaternion(VectorXf &mu, MatrixXf &Sigma) {
    Vector4f q = mu.segment<4>(3);
    
    const float norma = q.norm();
    mu.segment<4>(3) = q/norma;

    MatrixXf Q;
    Q = norma*norma*Matrix4f::Identity() - q*q.transpose();
    
    Q = (Q*(1/(norma*norma*norma))).eval();

    MatrixXf Qc = MatrixXf::Identity(Sigma.rows(), Sigma.cols());
    Qc.block<4,4>(3,3) = Q;

	Sigma = (Qc*Sigma*Qc.transpose()).eval();

}



bool isInsideImage(Vector2f hi, cv::Size size, int windowsSize) {
    float i = hi(0);
    float j = hi(1);
    
    if (i > windowsSize/2 && j > windowsSize/2 && i < size.width - windowsSize/2 && j < size.height - windowsSize/2) {
        return true;
    }
    return false;

}

MatrixXf dRq_times_d_dq(Vector4f q, Vector3f d) {
    MatrixXf diff_Rq_times_dq(3,4);
    for (int j = 0; j < 4; j++) {
    	diff_Rq_times_dq.col(j) = diffQuat2rot(q,j)*d;
    }
    return diff_Rq_times_dq;
}

