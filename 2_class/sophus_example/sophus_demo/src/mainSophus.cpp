#include <iostream>
#include <Eigen/Core>
#include <sophus/so3.h>
#include <sophus/se3.h>

using namespace std;
//using namespace Eigen;
//using namespace Sophus;


int main(int argc, char **argv) {
    //沿着Z轴旋转90度的旋转矩阵
    Eigen::AngleAxisd A1(M_PI / 2, Eigen::Vector3d(0, 0, 1));//以（0,0,1）为旋转轴，旋转180度
    Eigen::Matrix3d R1 = A1.matrix();
    Eigen::Quaterniond Q1(A1);


    //一、初始化的李群（SO3）的几种方式

    //1.使用旋转矩阵初始化李群
    Sophus::SO3 SO3_R(R1);
    //注意：尽管SO(3)是对应一个矩阵,但是输出SO(3)时,实际上是以so(3)形式输出,从输出的结果可以看到,其输出的值与旋转角对应的值相同,这也证证实了SO(3)对应的李代数so(3)就是旋转角。
    cout << "SO(3) SO3_R from Matrix" << SO3_R << endl << endl;

    //2.使用四元数初始化李群
    Sophus::SO3 SO3_Q(Q1);
    cout << "SO(3) SO3_Q from Quaterion" << SO3_Q << endl << endl;

    /****************************************************************************
     3.1 使用旋转角（轴角）的各个元素对应的代数值来初始化李群

     注意：直接使用旋转角AngleAxis或是旋转角度对应的向量(Vector3d=AngleAxis.axis()*AngleAxis.angle())对李群进行初始化是不行的，因为SO3李群没有对应的构造函数。
    也即是使用下列方法是错误的：

     Sophus::SO3 SO3_A(A1);//直接使用旋转角对李群初始化
     Sophus::SO3 SO3_A(A1.axis()*A1.angle());//直接使用旋转角度对应的向量(Vector3d=AngleAxis.axis()*AngleAxis.angle())对李群进行初始化

     只能使用旋转角对应的向量的每一个维度进行赋值，对应于SO3的这样一个构造函数SO3(double rot_x, double rot_y, double rot_z);

    *******************************************************************************/

    //3.1.1 使用旋转角度对应的向量(Vector3d=AngleAxis.axis()*AngleAxis.angle())中的各个元素对李群进行初始化
    Sophus::SO3 SO3_A1((A1.axis() * A1.angle())(0), (A1.axis() * A1.angle())(1), (A1.axis() * A1.angle())(2));
    cout << "SO(3) SO3_A1 from AngelAxis1" << SO3_A1 << endl << endl;

    //3.1.2 使用旋转角度对应的向量(Vector3d=AngleAxis.axis()*AngleAxis.angle())中的各个元素对李群进行初始化
    Sophus::SO3 SO3_A2(M_PI / 2 * 0, M_PI / 2 * 0, M_PI / 2 * 1);
    cout << "SO(3) SO3_A2 from AngleAixs2" << SO3_A2 << endl << endl;

    //3.2 由于旋转角（轴角）与李代数so(3)对应,所以直接使用旋转角的值获得se(3),进而再通过Sophus::SO3::exp()获得对应的SO(3)
    Eigen::Vector3d V1(0, 0, M_PI / 2);//so3在Eigen中用Vector3d表示
    Sophus::SO3 SO3_V1 = Sophus::SO3::exp(V1);
    cout << "SO(3) SO3_V1 from SO3::exp()" << SO3_V1 << endl << endl;


    //二、SO(3)与so(3)的相互转换，以及so3对应的hat和vee操作

    Eigen::Vector3d so3_V1 = SO3_V1.log();//so(3)在Sophus(Eigen)中用vector3d表示,使用对数映射获得李群对应的李代数
    cout << "so(3) so3_V1 from SO3_V1" << so3_V1.transpose() << endl << endl;


    Sophus::SO3 SO3_V2 = Sophus::SO3::exp(so3_V1);//使用指数映射将李代数转化为李群
    cout << "SO(3) so3_V2 from so3_V1" <<SO3_V2 << endl << endl;


    Eigen::Matrix3d M_so3_V1 = Sophus::SO3::hat(so3_V1);//hat为向量到其对应的反对称矩阵
    cout << "so3 hat=\n" << M_so3_V1 << endl << endl;

    Eigen::Vector3d V_M = Sophus::SO3::vee(M_so3_V1);//vee为反对称矩阵对应的向量
    cout << "so3 vee=\n" << V_M << endl << endl;

    //三、增量扰动模型
    Eigen::Vector3d update_so3(1e-4,0,0);//假设更新量为这么多
    Eigen::Matrix3d update_matrix=Sophus::SO3::exp(update_so3).matrix();//将李群转换为旋转矩阵
    cout<<"SO3 update Matrix=\n"<<update_matrix<<endl<<endl;

    Sophus::SO3 SO3_updated=Sophus::SO3::exp(update_so3)*SO3_R;
    cout<<"SO3 updated = \n"<<SO3_updated<<endl;

    Eigen::Matrix3d SO3_updated_matrix=SO3_updated.matrix();//将李群转换为旋转矩阵
    cout<<"SO3 updated Matrix = \n"<<SO3_updated_matrix<<endl<<endl;


//******************************************************************分割线***********************************************************************************
    cout<<"************************************分割线*************************************************"<<endl<<endl;

    Eigen::AngleAxisd A2(M_PI/2,Eigen::Vector3d(0,0,1));
    Eigen::Matrix3d R2=A2.matrix();
    Eigen::Quaterniond Q2(A2);
    Sophus::SO3 SO3_2(R2);

    //一、初始化李代数的几种方式
    Eigen::Vector3d t(1,0,0);

    //1. 使用旋转矩阵和平移向量来初始化SE3
    Sophus::SE3 SE_Rt(R2,t);
    cout<<"SE3 SE_Rt from  Rotation_Matrix and Transform=\n"<<SE_Rt<<endl<<endl;//注意尽管SE(3)是对应一个4*4的矩阵,但是输出SE(3)时是以一个六维向量输出的,其中前前三位为对应的so3,后3维度为实际的平移量t，而不是se3中的平移分量
    //2. 使用四元数和平移向量来初始化SE3
    Sophus::SE3 SE_Qt(Q2,t);
    cout<<"SE3 SE_Qt from  Quaterion and Transform=\n"<<SE_Qt<<endl<<endl;
    //3. 使用SO3和平移向量来初始化SE3
    Sophus::SE3 SE_St(SO3_2,t);
    cout<<"SE3 SE_St from  SO3 and Transform=\n"<<SE_St<<endl<<endl;

    //二、SE(3)与se(3)的相互转换，以及se3对应的hat和vee操作
    Sophus::Vector6d se3_Rt=SE_Rt.log();//se(3)在Sophus中用Vector6d表示,使用对数映射获得李群对应的李代数
    cout<<"se(3) se3_Rt from SE3_Rt\n"<<se3_Rt<<endl<<endl;//se3输出的是一个六维度向量,其中前3维是平移分量,后3维度是旋转分量

    Sophus::SE3 SE3_Rt2=Sophus::SE3::exp(se3_Rt);//使用指数映射将李代数转化为李群
    cout<<"SE(3) SO3_Rt2 from se3_Rt"<<SE3_Rt2<<endl<<endl;

    Sophus::Matrix4d M_se3_Rt=Sophus::SE3::hat(se3_Rt);
    cout<<"se(3) hat=\n"<<M_se3_Rt<<endl<<endl;

    Sophus::Vector6d V_M_se3=Sophus::SE3::vee(M_se3_Rt);
    cout<<"se(3) vee=\n"<<V_M_se3<<endl<<endl;




    //三、增量扰动模型

    Sophus::Vector6d update_se3=Sophus::Vector6d::Zero();
    update_se3(0)=1e-4d;

    cout<<"update_se3\n"<<update_se3.transpose()<<endl<<endl;

    Eigen::Matrix4d update_matrix2=Sophus::SE3::exp(update_se3).matrix();//将李群转换为旋转矩阵
    cout<<"update matrix=\n"<<update_matrix2<<endl<<endl;

    Sophus::SE3 SE3_updated=Sophus::SE3::exp(update_se3)*SE3_Rt2;
    cout<<"SE3 updated=\n"<<SE3_updated<<endl<<endl;

    Eigen::Matrix4d SE3_updated_matrix=SE3_updated.matrix();//将李群转换为旋转矩阵
    cout<<"SE3 updated Matrix=\n"<<SE3_updated_matrix<<endl<<endl;

    return 0;

}
