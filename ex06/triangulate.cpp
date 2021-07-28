//
// Created by hyj on 18-11-11.
//
#include <iostream>
#include <vector>
#include <random>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <chrono>
#include <fstream>

struct Pose
{
    Pose(Eigen::Matrix3d R, Eigen::Vector3d t) : Rwc(R), qwc(R), twc(t){};
    Eigen::Matrix3d Rwc;
    Eigen::Quaterniond qwc;
    Eigen::Vector3d twc;
    
    Eigen::Vector2d uv; // 这帧图像观测到的特征坐标
};
void run(double sigma = 0.1, int interval = 1)
{
    int poseNums = 100;
    double radius = 8;
    double fx = 1.;
    double fy = 1.;
    std::vector<Pose> camera_pose;
    for (int n = 0; n < poseNums; ++n)
    {
        double theta = n * 2 * M_PI / (poseNums * 4); // 1/4 圆弧
        // 绕 z轴 旋转
        Eigen::Matrix3d R;
        R = Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ());
        Eigen::Vector3d t = Eigen::Vector3d(radius * cos(theta) - radius, radius * sin(theta), 1 * sin(2 * theta));
        camera_pose.push_back(Pose(R, t));
    }

    // 随机数生成 1 个 三维特征点
    std::default_random_engine generator;
    std::uniform_real_distribution<double> xy_rand(-4, 4.0);
    std::uniform_real_distribution<double> z_rand(8., 10.);
    double tx = xy_rand(generator);
    double ty = xy_rand(generator);
    double tz = z_rand(generator);

    Eigen::Vector3d Pw(tx, ty, tz);
    // 这个特征从第三帧相机开始被观测，i=3
    int start_frame_id = 3;
    int end_frame_id = poseNums;
    for (int i = start_frame_id; i < end_frame_id; ++i)
    {
        Eigen::Matrix3d Rcw = camera_pose[i].Rwc.transpose();
        Eigen::Vector3d Pc = Rcw * (Pw - camera_pose[i].twc);

        double x = Pc.x();
        double y = Pc.y();
        double z = Pc.z();

        double w_sigma = sigma; // variance of Gaussian noise
        std::normal_distribution<double> noise(0., w_sigma);
        camera_pose[i].uv = Eigen::Vector2d(x / z + noise(generator), y / z + noise(generator));
    }

    /// TODO::homework; 请完成三角化估计深度的代码
    // 遍历所有的观测数据，并三角化
    Eigen::Vector3d P_est; // 结果保存到这个变量
    P_est.setZero();
    /* your code begin */

    // step 1 construct matrix D
    Eigen::MatrixX4d D;
    for (int i = start_frame_id; i < end_frame_id; i += interval)
    {
        double u = camera_pose[i].uv[0];
        double v = camera_pose[i].uv[1];
        Eigen::Matrix3d Rcw = camera_pose[i].Rwc.transpose();
        Eigen::Vector3d tcw = -Rcw * camera_pose[i].twc;

        D.conservativeResize(D.rows() + 2, Eigen::NoChange);
        Eigen::Matrix<double, 3, 4> P;
        P << Rcw, tcw;
        D.row(D.rows() - 2) = u * P.row(2) - P.row(0);
        D.row(D.rows() - 1) = v * P.row(2) - P.row(1);
    }
    // step 2 compute D^T * D and apply SVD
    Eigen::MatrixXd DTD_temp = D.transpose() * D;
    Eigen::MatrixXd DTD = 0.5 * (DTD_temp.transpose() + DTD_temp);
    // DTD is symmetric so we can use SelfAdjointEVD
    auto start = std::chrono::steady_clock::now();
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes(DTD);
    auto end = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    std::cout << "EVD used time = " << 1e3 * elapsed_seconds.count() << " ms" << std::endl;
    auto eigenVal = saes.eigenvalues();
    std::cout << ((saes.info() == 0) ? "success" : "fail") << std::endl;
    std::cout << eigenVal << std::endl;
    // std::cout << saes.eigenvectors().col(0) << std::endl;
    Eigen::Vector4d P_est4 = saes.eigenvectors().col(0);
    // std::cout << P_est4 << std::endl;
    P_est = (P_est4 / P_est4(3, 0)).head(3);

    // same result for SVD
    auto start1 = std::chrono::steady_clock::now();
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(DTD, Eigen::ComputeThinU | Eigen::ComputeThinV);
    auto end1 = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed_seconds1 = end1 - start1;
    std::cout << "SVD used time = " << 1e3 * elapsed_seconds1.count() << " ms" << std::endl;
    std::cout << svd.singularValues() << std::endl;
    // std::cout << svd.matrixU() << '\n'
    //   << svd.matrixV() << std::endl;

    /* your code end */
    std::cout << "ground truth: \n"
              << Pw.transpose() << std::endl;
    std::cout << "your result: \n"
              << P_est.transpose() << std::endl;
    // TODO:: 请如课程讲解中提到的判断三角化结果好坏的方式，绘制奇异值比值变化曲线

    double rate = eigenVal[1] / eigenVal[0];
    std::cout << "scecond smallest sigular value compared with smallest one :" << rate << std::endl;
    std::ofstream result_csv;
    result_csv.open("result.csv", std::ios_base::app | std::ios_base::out);
    result_csv << std::abs(rate) << ",";
    result_csv.close();

    if (std::abs(eigenVal[1] / eigenVal[0]) < 1e2)
    {
        std::cout << "-------------solution not reliable-------------\n"
                  << std::endl;
    }
    else
        std::cout << "-------------good solution-------------\n"
                  << std::endl;
}

int main()
{

    auto different_noise_level = std::vector<double>{0, 1e-3, 0.5 * 1e-2, 0.7 * 1e-2, 1e-2, 0.5 * 1e-1, 1e-1};
    auto intervals = std::vector<int>{1, 2, 3, 4, 5, 6, 7};
    std::ofstream result_csv;

    result_csv.open("result.csv", std::ios_base::app);
    for (auto &e : different_noise_level)
        result_csv << e << ",";
    result_csv << std::endl;

    // for (auto item : different_noise_level)
    // {
    //     std::cout << "-----std_variance of Gauss noise is " << item << " ----------" << std::endl;
    //     run(item);
    // }

    for (auto item : intervals)
    {
        std::cout << "-----interval is " << item << " ----------" << std::endl;
        run(1 * 1e-1, item);
    }
    result_csv.close();
    return 0;
}
