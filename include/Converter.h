/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef CONVERTER_H
#define CONVERTER_H

#include<opencv2/core/core.hpp>

#include<Eigen/Dense>
#include"Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include"Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"
#include"Thirdparty/g2o/g2o/types/se2.h"

namespace ORB_SLAM2
{

class Converter
{
public:
    static std::vector<cv::Mat> toDescriptorVector(const cv::Mat &Descriptors);

    static g2o::SE2 toSE2(const cv::Mat &cvT);
    
    static double normalize_angle(double theta)
    {
        if (theta >= -M_PI && theta < M_PI)
        return theta;

        double multiplier = floor(theta / (2*M_PI));
        theta = theta - multiplier*2*M_PI;
        if (theta >= M_PI)
            theta -= 2*M_PI;
        if (theta < -M_PI)
            theta += 2*M_PI;

        return theta;
    }

    static inline Eigen::Matrix3d skew(const Vector3d&v)
    {
        Eigen::Matrix3d m;
        m.fill(0.);
        m(0,1)  = -v(2);
        m(0,2)  =  v(1);
        m(1,2)  = -v(0);
        m(1,0)  =  v(2);
        m(2,0) = -v(1);
        m(2,1) = v(0);
        return m;
    }

    static g2o::SE3Quat toSE3Quat(const cv::Mat &cvT);
    static g2o::SE3Quat toSE3Quat(const g2o::Sim3 &gSim3);

    static cv::Mat toCvMat(const g2o::SE3Quat &SE3);
    static cv::Mat toCvMat(const g2o::Sim3 &Sim3);
    static cv::Mat toCvMat(const Eigen::Matrix<double,4,4> &m);
    static cv::Mat toCvMat(const Eigen::Matrix3d &m);
    static cv::Mat toCvMat(const Eigen::Matrix<double,3,1> &m);
    static cv::Mat toCvSE3(const Eigen::Matrix<double,3,3> &R, const Eigen::Matrix<double,3,1> &t);

    static Eigen::Matrix<double,3,1> toVector3d(const cv::Mat &cvVector);
    static Eigen::Matrix<double,3,1> toVector3d(const cv::Point3f &cvPoint);
    static Eigen::Matrix<double,3,3> toMatrix3d(const cv::Mat &cvMat3);

    static std::vector<float> toQuaternion(const cv::Mat &M);
};
namespace Constants
{
   static cv::Mat bTc = (cv::Mat_<float>(4,4) <<
               9.9792816252667338e-03f, 6.5348103708624539e-03f,9.9992885256485176e-01f, 2.2648368490900000e-01f,
               -9.9982014658446139e-01f, 1.6192923276330706e-02f,9.8723715283343672e-03f, -5.1141940356500000e-02f,
               -1.6127257115523985e-02f, -9.9984753112121250e-01f,6.6952288046080444e-03f, 9.1600000000000004e-01f, 
               0.0f, 0.0f, 0.0f,1.0f);
};

}// namespace ORB_SLAM

#endif // CONVERTER_H
