#include <iostream>
#include <vector>
#include <random>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

using namespace std;

struct Pose
{
    Pose(Eigen::Matrix3d R, Eigen::Vector3d t) : Rwc(R), qwc(R), twc(t) {};
    Eigen::Matrix3d Rwc;
    Eigen::Vector3d twc;
    Eigen::Quaterniond qwc;
};


int main()
{
    // 假设三维点在每个相机时刻均可以观测到
    int feature_num = 20;
    int pose_num = 10;
    int ndim = pose_num*6 + feature_num*3;
    double fx = 1.0, fy = 1.0;
    double radius = 8.0;
    Eigen::MatrixXd H(ndim, ndim);
    H.setZero();

    vector<Pose> camera_pose;
    // 假设相机做圆弧运动
    for (int n = 0; n < pose_num; n++)
    {
        double theta = n*2*M_PI/(pose_num*4);  // 1/4圆弧
        Eigen::Matrix3d R;
        R = Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ());
        Eigen::Vector3d t;
        t = Eigen::Vector3d(radius*cos(theta), radius*sin(theta), 1*sin(2*theta));  // 沿Z轴做正弦运动
        camera_pose.push_back(Pose(R, t));
    }

    // 随机数生成三维点
    default_random_engine generator;
    vector<Eigen::Vector3d> points;
    for (int j = 0; j < feature_num; j++)
    {
        uniform_real_distribution<double> xy_rand(-4.0, 4.0);
        uniform_real_distribution<double> z_rand(8.0, 10.0);
        double Px = xy_rand(generator);
        double Py = xy_rand(generator);
        double Pz = z_rand(generator);
        Eigen::Vector3d Pw(Px, Py, Pz);
        points.push_back(Pw);

        for (int i = 0; i < pose_num; i++)
        {
            /**
             * Twc = [Rwc twc]
             *       [  0   1]
             * Tcw = Twc.inverse() = [Rwc.inverse() -Rwc.inverse()*twc*1]
             *                       [           0                   1]
             */
            Eigen::Matrix3d Rcw = camera_pose[i].Rwc.transpose();
            Eigen::Vector3d Pc = Rcw*(Pw-camera_pose[i].twc);
            double x = Pc.x();
            double y = Pc.y();
            double z = Pc.z();
            double z_2 = z*z;
            Eigen::Matrix<double, 2, 3> J_uv_Pc;
            J_uv_Pc << fx/z, 0, -fx*x/z_2, 0, fy/z, -fy*y/z_2;
            // 优化特征点空间位置
            Eigen::Matrix<double, 2, 3> J_uv_Pw = J_uv_Pc*Rcw; 
            // 优化相机位姿
            Eigen::Matrix<double, 2, 6> J_uv_kesi;
            J_uv_kesi << fx/z, 0, -fx*x/z_2, -fx*x*y/z_2, fx*(1+x*x/z_2), -fx*y/z,
                         0, fy/z, -fy*y/z_2, -fy*(1+y*y/z_2), fy*x*y/z_2, fy*x/z;
            
            H.block(i*6, i*6, 6, 6) += J_uv_kesi.transpose() * J_uv_kesi; // pose-pose
            H.block(6*pose_num + j*3, 6*pose_num + j*3, 3, 3) += J_uv_Pw.transpose() * J_uv_Pw;  // landmark-landmark
            H.block(i*6, 6*pose_num + j*3, 6, 3) += J_uv_kesi.transpose() * J_uv_Pw;  // pose-landmark(upper)
            H.block(6*pose_num + j*3, i*6, 3, 6) += J_uv_Pw.transpose() * J_uv_kesi;  // pose-landmark(lower)
        }
        
    }
    
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(H, Eigen::ComputeThinU | Eigen::ComputeThinV);
    cout << svd.singularValues() << endl;

    /*
        // 最后7维的值接近于0，表示零空间的维度为7
        103.573
        102.619
        101.66
        ...
        ...
        ...
        0.00281533
        0.00116278
        0.000216946
        1.54151e-16
        9.79149e-17
        7.45022e-17
        5.05922e-17
        2.83315e-17
        2.03184e-17
        9.62462e-18
     */
    
    return 0;
}


