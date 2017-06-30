#ifndef YGZ_COMMON_H_
#define YGZ_COMMON_H_

// 常用的一些头文件 

// std 
#include <vector>
#include <list>
#include <memory>
#include <string>
#include <iostream>
#include <set>
#include <unordered_map>
#include <map>
#include <algorithm>
#include <mutex>
#include <thread>

using namespace std;

// for Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector> // for vector of Eigen objects 
#include <Eigen/Dense>  // linear algebra

using Eigen::Vector2f;
using Eigen::Vector3d;
using Eigen::Vector2f;
using Eigen::Vector3f;
using Eigen::Vector4f;
using Eigen::Matrix2f;
using Eigen::Matrix3f;
using Eigen::Matrix4f;
using Eigen::Matrix2d;
using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::Quaternionf;
// other things I need in optimiztion 
typedef Eigen::Matrix<double, 6, 1> Vector6d;

// for Sophus
#include <Thirdparty/sophus/sophus/se3.hpp>
#include <Thirdparty/sophus/sophus/so3.hpp>

using Sophus::SO3f;
using Sophus::SE3f;
using Sophus::SO3d;
using Sophus::SE3d;

// for g2o
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"
#include "Thirdparty/g2o/g2o/types/se3quat.h"
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

// for cv
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

using cv::Mat;

// pangolin
#include <pangolin/pangolin.h>

// glog
#include <glog/logging.h>

#endif