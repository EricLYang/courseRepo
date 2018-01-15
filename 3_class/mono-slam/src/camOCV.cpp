#include "camOCV.hpp"

CamModel::CamModel(float _fx, float _fy, float _u0, float _v0, float _k1, float _k2):
        fx(_fx),
        fy(_fy),
        u0(_u0),
        v0(_v0),
        k1(_k1),
        k2(_k2)
    {}
    
    
    
    
    
CamModel::CamModel(cv::Mat parameters):
    fx(parameters.at<float>(0)),
    fy(parameters.at<float>(1)),
    u0(parameters.at<float>(2)),
    v0(parameters.at<float>(3)),
    k1(parameters.at<float>(4)),
    k2(parameters.at<float>(5))
{
    std::cout << "Cam Model " << fx << " " << fy << " " << u0 << " "<< v0 << " " << k1 << " " << k2 << std::endl;
}



Vector2f CamModel::projectAndDistort(Vector3f h, MatrixXf &J) {
    
    float x = h(0);
    float y = h(1);
    float z = h(2);
    
    
    // Compute point retina undistor
    float inv_z = 1/z;

    float x1 = x/z;
    float y1 = y/z;
    
    
    
    MatrixXf Jacobian_RetinaUnd_hC(2,3);
    Jacobian_RetinaUnd_hC << 1/z, 0, -x/z/z, 0, 1/z, -y/z/z;
    
    // Compute point retina distort

    float ru = sqrt(x1*x1 + y1*y1);
    float ru_2 = ru*ru;
    float r = ru/(1+k1*ru_2+k2*ru_2*ru_2);
    
    for (int i = 0; i < 100; i++) {
        r = r - (r + k1*pow(r,3.0) + k2*pow(r,5.0) - ru)/(1 + 3*k1*pow(r,2.0)+ 5*k2*pow(r,4.0));
    }
    
    float r_2 = r*r;

    float l = 1 + k1*r_2 + k2*r_2*r_2;
    float x2 = x1/l;
    float y2 = y1/l;
    
    
    float m = 2*k1 + 4*k2*r_2;
    MatrixXf Jacobian_Undistort_Distort(2,2);
    Jacobian_Undistort_Distort(0,0) = l + m*x2*x2;
    Jacobian_Undistort_Distort(1,1) = l + m*y2*y2;
    Jacobian_Undistort_Distort(0,1) = Jacobian_Undistort_Distort(1,0) =  m*x2*y2;

    
    
    // Compute point pixel distort
    
    Vector2f hd;
    hd << fx*x2 + u0, fy*y2 + v0;

    MatrixXf Jacobian_Pixel_Retina(2,2);
    Jacobian_Pixel_Retina << fx, 0, 0, fy;
    
    
    // ************************************ //
        
    J = Jacobian_Pixel_Retina*Jacobian_Undistort_Distort.inverse()*Jacobian_RetinaUnd_hC;
    

    return hd;
}

Vector2f CamModel::projectAndDistort(Vector3f h) {
    
    float x = h(0);
    float y = h(1);
    float z = h(2);
    
    
    // Compute point retina undistor
    float inv_z = 1/z;

    float x1 = x*inv_z;
    float y1 = y*inv_z;

    
    // Compute point retina distort

    float ru = sqrt(x1*x1 + y1*y1);
    float ru_2 = ru*ru;
    float r = ru/(1+k1*ru_2+k2*ru_2*ru_2);
    
    for (int i = 0; i < 100; i++) {
        r = r - (r + k1*pow(r,3.0) + k2*pow(r,5.0) - ru)/(1 + 3*k1*pow(r,2.0)+ 5*k2*pow(r,4.0));
    }
    
    float r_2 = r*r;

    float l = 1 + k1*r_2 + k2*r_2*r_2;
    float x2 = x1/l;
    float y2 = y1/l;
    
    
    float m = 2*k1 + 4*k2*r_2;

    
    // Compute point pixel distort
    
    Vector2f hd;
    hd  << fx*x2 + u0, fy*y2 + v0;

    return hd;
}

Vector3f CamModel::UndistortAndDeproject(Vector2f hd, MatrixXf &J){
    
    // Compute retina distort
    float x2 = (hd(0) - u0)/fx;
    float y2 = (hd(1) - v0)/fy;

    MatrixXf Jacobian_Retina_Pixel(2,2);
    Jacobian_Retina_Pixel << 1/fx, 0, 0, 1/fy;

    // Compute retina undistort
    
    float r_2 = x2*x2 + y2*y2;
    float l = 1+k1*r_2+k2*r_2*r_2;
    float x1 = x2*l;
    float y1 = y2*l;
    
    float m = 2*k1 + 4*k2*r_2;
    MatrixXf Jacobian_Undistort_Distort(2,2);
    Jacobian_Undistort_Distort(0,0) = l + m*x2*x2;
    Jacobian_Undistort_Distort(1,1) = l + m*y2*y2;
    Jacobian_Undistort_Distort(0,1) = Jacobian_Undistort_Distort(1,0) =  m*x2*y2;
    
    // Compute 3D Line
    
    Vector3f hC;
    hC << x1, y1, 1;
    
    MatrixXf Jacobian_hC_RetinaUnd(3,2);
    Jacobian_hC_RetinaUnd << 1, 0, 0, 1, 0, 0;
    
    J = Jacobian_hC_RetinaUnd*Jacobian_Undistort_Distort*Jacobian_Retina_Pixel;

    return hC;
}



