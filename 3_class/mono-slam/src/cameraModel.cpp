#include "cameraModel.hpp"


void CameraModel::setParam(camConfig camParams) {
	fx = camParams.fx;
	fy = camParams.fy;
	u0 = camParams.u0;
	v0 = camParams.v0;
	k1 = camParams.k1;
	k2 = camParams.k2;
	k3 = camParams.k3;
	p1 = camParams.p1;
	p2 = camParams.p2;
	
}


Matrix2f CameraModel::diff_distort_undistort(Vector2f hn) {

	float r_2 = hn(0)*hn(0) + hn(1)*hn(1);
	float l = 1 + k1*r_2 + k2*r_2*r_2 + k3*r_2*r_2*r_2;

	Vector2f hn_contr;
    hn_contr << hn(1),hn(0);

    Vector2f pv;
    pv << p1,p2;

    Vector2f pv_contr;
    pv_contr << p2,p1;

    Matrix2f j_matrix;
    j_matrix << p2*hn(0), 0, 0, p1*hn(1);


    float f = k1 + 2*k2*r_2 + 3*k3*r_2*r_2;

    Matrix2f Jacobian_Distort_Undistort = l*Matrix2f::Identity() + 2*f*hn*hn.transpose() + 2*pv*hn_contr.transpose() + 2*pv_contr*hn.transpose() + 4*j_matrix;
    return Jacobian_Distort_Undistort;

}



CameraModel::CameraModel(float _fx, float _fy, float _u0, float _v0, float _k1, float _k2, float _k3, float _p1, float _p2):
        fx(_fx), fy(_fy),
        u0(_u0), v0(_v0),
        k1(_k1), k2(_k2), k3(_k3),
        p1(_p1), p2(_p2)
    { }
    


Vector2f CameraModel::project(Vector3f h1, bool computeJacobian) {

	const float x = h1(0);
    const float y = h1(1);
    const float z = h1(2);

    // compute retina projection
    const float x1 = x/z;
    const float y1 = y/z;



    // Distort retina point
    const float r_2 = x1*x1+y1*y1;
    const float l = 1 + k1*r_2 + k2*r_2*r_2 + k3*r_2*r_2*r_2;
    const float x2 = x1*l + 2*p1*x1*y1 + p2*(r_2 + 2*x1*x1);
    const float y2 = y1*l + 2*p2*x1*y1 + p1*(r_2 + 2*y1*y1);

    // project in image RF
    Vector2f hd = (Vector2f << fx*x2 + u0, fy*y2 + v0);


    if (computeJacobian) {
        Vector2f hn = (Vector2f << x1,y1);
    	Matrix<float, 2, 3> Jacobian_RetinaUnd_hC = (Matrix<float, 2, 3> << 1/z, 0, -x/z/z, 0, 1/z, -y/z/z);
    	Matrix2f Jacobian_Pixel_Retina = (Matrix2f  << fx, 0, 0, fy);
    	this->projectionJacobian = Jacobian_Pixel_Retina*diff_distort_undistort(hn)*Jacobian_RetinaUnd_hC;
    }

    return hd;
}


Matrix2f CameraModel::getProjectionJacobian() {
	return this->projectionJacobian;
}




Vector3f CameraModel::unproject(Vector2f hd, bool computeJacobian){
    
    // Compute retina distort
    const float x2 = (hd(0) - u0)/fx;
    const float y2 = (hd(1) - v0)/fy;
    
    // Compute retina undistort using iterative solution
    float x1 = x2;
    float y1 = y2;

    float r_2;
    float l;

    float dx,dy;


    const int n_iter = 20;
    for (int i = 0; i < n_iter; ++i) {
    	r_2 = x1*x1 + y1*y1;
    	l = 1 + k1*r_2 + k2*r_2*r_2 + k3*r_2*r_2*r_2;

        dx = 2*p1*x1*y1 + p2*(r_2 + 2*x1*x1);
        dy = 2*p2*x1*y1 + p1*(r_2 + 2*y1*y1);

        x1 = (x2 - dx)/l;
        y1 = (y2 - dy)/l;
    }

    // Epipolar line normal vector
    Vector3f hC = (Vector3f << x1, y1, 1);

    if (computeJacobian) {
        Vector2f hn = (Vector2f << x1,y1);
        Matrix2f Jacobian_Retina_Pixel = (Matrix2f << 1/fx, 0, 0, 1/fy);
        Matrix<float, 3, 2> Jacobian_hC_RetinaUnd = (Matrix<float, 3, 2> << 1, 0, 0, 1, 0, 0);
        this->unprojectionJacobian = Jacobian_hC_RetinaUnd*diff_distort_undistort(hn).inverse()*Jacobian_Retina_Pixel;
    }

    return hC;
}

Matrix3f CameraModel::getUnprojectionJacobian() {
	return this->unprojectionJacobian;
}




