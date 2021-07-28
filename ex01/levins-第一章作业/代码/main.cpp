#include <iostream>
#include "sophus/so3.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>

using namespace std;
using namespace Eigen;

// using Sophus for update
void update_Matrix_Sophus(const Vector3d & update, Matrix3d & matrix3D){
    Sophus::SO3d SO3d_R(matrix3D);
    matrix3D = (SO3d_R * Sophus::SO3d::exp(update)).matrix();

}
// using Rodrigues' formula for exp(w^) without Sophus
void update_Matrix(const Vector3d & update, Matrix3d & matrix3D){
    double theta = update.norm();
    Vector3d norm = update.normalized();
    Matrix3d skewMatrix = Matrix3d::Zero();
    skewMatrix(0,1) = -norm.z();
    skewMatrix(0,2) = norm.y();
    skewMatrix(1,0) = norm.z();
    skewMatrix(1,2) = -norm.x();
    skewMatrix(2,0) = -norm.y();
    skewMatrix(2,1) = norm.x();
    //Rodrigues' formula
    Matrix3d R_update = cos(theta) * Matrix3d::Identity()
                        + (1 - cos(theta)) * norm * norm.transpose()
                        + sin(theta) * skewMatrix;

    matrix3D = matrix3D * R_update;
}
void update_Quaternion(const Vector3d & update, Quaterniond & q){
    Quaterniond q_update = {1, update.x()/2, update.y()/2, update.z()/2};
    q = q * q_update;
    q.normalize();
}

int main() {

    Vector3d update = {0.01, 0.02, 0.03};
    // original rotation matrix
    Matrix3d R = AngleAxisd(M_PI_2, Vector3d{0, 0, 1}).toRotationMatrix();
    Quaterniond q(R);

    // updated rotation matrix
//    update_Matrix_Sophus(update, R);
    update_Matrix(update, R);
    cout << "use Rotation Matrix to update :\n" << R << endl;

    update_Quaternion(update, q);
    cout << "use Quaternion to update :\n" << q.toRotationMatrix() << endl;

    cout << "\ndifference:\n" << q.toRotationMatrix() - R << endl;

    ////conclusion: order of magnitude at least -6, so the difference can be neglected
//    update.normalize();
    cout << update << endl;
    Matrix3d mat = Sophus::SO3d::hat(update);
    cout << Matrix3d::Identity() - mat * mat << endl;
    update *= 100;
    cout << pow(update.norm(),2) * (Matrix3d::Identity() - update * update.transpose()) << endl;
    return 0;
}
