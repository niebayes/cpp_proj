#include <iostream>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "sophus/so3.hpp"
#include "sophus/se3.hpp"

// #include "pcl/2d/convolution.h"

using namespace Eigen;
using namespace std;
// using namespace cv;

int main()
{
     Eigen::Matrix3d R = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d(0, 0, 1)).toRotationMatrix();
     Sophus::SO3 SO3_R(R);              // Sophus::SO(3)可以直接从旋转矩阵构造
     Sophus::SO3 SO3_v(0, 0, M_PI / 2); // 亦可从旋转向量构造
     Eigen::Quaterniond q(R);           // 或者四元数
     Sophus::SO3 SO3_q(q);
     // 上述表达方式都是等价的
     // 输出 SO(3) 时，以 so(3) 形式输出
     cout << "SO(3) from matrix: " << SO3_R << endl;
     cout << "SO(3) from vector: " << SO3_v << endl;
     cout << "SO(3) from quaternion :" << SO3_q << endl;
     // Mat image;
     // image = cv::imread("itsuki.jpg");
     // imshow("image", image);

     // MatrixXd m = MatrixXd::Random(3, 3);
     // m = (m + MatrixXd::Constant(3, 3, 1.2)) * 50;
     // cout << "m =" << endl
     //      << m << endl;
     // VectorXd v(3);
     // v << 1, 2, 3;
     // cout << "m * v =" << endl
     //      << m * v << endl;
}