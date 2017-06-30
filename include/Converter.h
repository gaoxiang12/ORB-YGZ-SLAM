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

#ifndef YGZ_CONVERTER_H_
#define YGZ_CONVERTER_H_

#include "Common.h"
#include "IMU/NavState.h"
#include "IMU/imudata.h"
#include "IMU/IMUPreintegrator.h"

// ORB里用的一些Convert函数，虽然我自己不用，但是有一些ORB里的旧代码用了，我也懒得改。。

namespace ygz {

    class Converter {
    public:
        // 从IMU integration 数据转换为 NaviState
        static void updateNS(NavState &ns, const IMUPreintegrator &imupreint, const Vector3d &gw);

        // 矩阵形式的描述转向量形式的描述
        static std::vector<cv::Mat> toDescriptorVector(const cv::Mat &Descriptors);

        /** cv::Mat to g2o::SE3Quat */
        static g2o::SE3Quat toSE3Quat(const cv::Mat &cvT);

        /** unimplemented */
        static g2o::SE3Quat toSE3Quat(const g2o::Sim3 &gSim3);

        // 各种其他矩阵转cv::Mat
        // 讲道理 矩阵干脆都用eigen存就行了
        static cv::Mat toCvMat(const SE3d &SE3);

        static cv::Mat toCvMat(const SE3f &SE3);

        static cv::Mat toCvMat(const g2o::SE3Quat &SE3);

        static cv::Mat toCvMat(const g2o::Sim3 &Sim3);

        static cv::Mat toCvMat(const Eigen::Matrix<double, 4, 4> &m);

        static cv::Mat toCvMat(const Eigen::Matrix<float, 4, 4> &m);

        static cv::Mat toCvMat(const Eigen::Matrix3d &m);

        static cv::Mat toCvMat(const Eigen::Matrix3f &m);

        static cv::Mat toCvMat(const Eigen::Matrix<double, 3, 1> &m);

        static cv::Mat toCvMat(const Eigen::Matrix<float, 3, 1> &m);

        static cv::Mat toCvSE3(const Eigen::Matrix<double, 3, 3> &R, const Eigen::Matrix<double, 3, 1> &t);

        // Mat 转 Eigen
        static Eigen::Matrix<double, 3, 1> toVector3d(const cv::Mat &cvVector);

        static Eigen::Matrix<double, 3, 1> toVector3d(const cv::Point3f &cvPoint);

        static Eigen::Matrix<double, 3, 3> toMatrix3d(const cv::Mat &cvMat3);

        static std::vector<float> toQuaternion(const cv::Mat &M);
    };

}// namespace ygz

#endif // CONVERTER_H
