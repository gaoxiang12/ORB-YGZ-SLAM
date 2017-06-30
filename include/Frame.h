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

#ifndef YGZ_FRAME_H_
#define YGZ_FRAME_H_

// ORB 里的帧

#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include "ORBVocabulary.h"
#include "ORBextractor.h"

#include "IMU/imudata.h"
#include "IMU/NavState.h"
#include "IMU/IMUPreintegrator.h"

namespace ygz {

    // 格点大小，这个最好能随着图像大小做一些改变
    #define FRAME_GRID_ROWS 48
    #define FRAME_GRID_COLS 64

    class MapPoint;

    class KeyFrame;

    class ORBextractor;

    class Frame {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef enum {
            Monocular = 0, Stereo, RGBD
        } SensorType;

    public:

        Frame();

        // Copy constructor.
        Frame(const Frame &frame);

        // Constructor for stereo cameras.
        Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp, ORBextractor *extractorLeft,
              ORBextractor *extractorRight, ORBVocabulary *voc, Matrix3f &K, cv::Mat &distCoef, const float &bf,
              const float &thDepth);

        // Constructor for RGB-D cameras.
        Frame(const cv::Mat &imGray, const cv::Mat &imDepth, const double &timeStamp, ORBextractor *extractor,
              ORBVocabulary *voc, Matrix3f &K, cv::Mat &distCoef, const float &bf, const float &thDepth);

        // Constructor for Monocular cameras.
        Frame(const cv::Mat &imGray, const double &timeStamp, ORBextractor *extractor, ORBVocabulary *voc, Matrix3f &K,
              cv::Mat &distCoef, const float &bf, const float &thDepth);

        // Constructor for Monocular VI
        Frame(const cv::Mat &imGray, const double &timeStamp, const std::vector<IMUData> &vimu, ORBextractor *extractor,
              ORBVocabulary *voc,
              Matrix3f &K, cv::Mat &distCoef, const float &bf, const float &thDepth, KeyFrame *pLastKF = NULL);

        // 提取特征的所有步骤
        void ExtractFeatures();

        // Extract ORB on the image. 0 for left image and 1 for right image.
        // 提取的关键点存放在mvKeys和mDescriptors中
        // ORB是直接调orbExtractor提取的
        void ExtractORB(int flag, const cv::Mat &im);

        // 计算图像金字塔
        void ComputeImagePyramid();

        // Compute Bag of Words representation.
        // 存放在mBowVec中
        void ComputeBoW();

        // Set the camera pose.
        void SetPose(const SE3f &Tcw);

        // Computes rotation, translation and camera center matrices from the camera pose.
        void UpdatePoseMatrices();

        // Returns the camera center.
        inline Vector3f GetCameraCenter() {
            return mOw;
        }

        // Returns inverse of rotation
        inline Matrix3f GetRotationInverse() {
            return mRwc;
        }

        // Check if a MapPoint is in the frustum of the camera
        // and fill variables of the MapPoint to be used by the tracking
        // 判断路标点是否在视野中
        // 会检测像素坐标、距离和视线角
        bool isInFrustum(MapPoint *pMP, float viewingCosLimit);

        // Compute the cell of a keypoint (return false if outside the grid)
        // 检测某个关键帧是否在某个网格内
        bool PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY);

        // 获取一定区域内的点，需要事先用AssignFeaturesToGrid指定填充网格
        vector<size_t> GetFeaturesInArea(const float &x, const float &y, const float &r, const int minLevel = -1,
                                         const int maxLevel = -1) const;

        // Search a match for each keypoint in the left image to a keypoint in the right image.
        // If there is a match, depth is computed and the right coordinate associated to the left keypoint is stored.
        // 左右目之间的双目匹配。先用ORB特征匹配，再进行微调
        // 需要双目已经做了Rectify,极线水平，误差允许在+-2像素之间
        void ComputeStereoMatches();

        // Associate a "right" coordinate to a keypoint if there is valid depth in the depthmap.
        // 通过 RGBD 数据提供伪双目数据，便于统一接口
        void ComputeStereoFromRGBD(const cv::Mat &imDepth);

        // Backprojects a keypoint (if stereo/depth info available) into 3D world coordinates.
        // 计算第i个特征在相机系的坐标
        Vector3f UnprojectStereo(const int &i);

        // Computes image bounds for the undistorted image (called in the constructor).
        // 计算一些图像的边界(在ygz中不使用，因为默认图像已经是缩放过的)
        void ComputeImageBounds(const cv::Mat &imLeft);

        // Assign keypoints to the grid for speed up feature matching (called in the constructor).
        void AssignFeaturesToGrid();

        // coordinate transform: world, camera, pixel
        inline Vector3f World2Camera(const Vector3f &p_w, const SE3f &T_c_w) const {
            return T_c_w * p_w;
        }

        inline Vector3f Camera2World(const Vector3f &p_c, const SE3f &T_c_w) const {
            return T_c_w.inverse() * p_c;
        }

        inline Vector2f Camera2Pixel(const Vector3f &p_c) const {
            return Vector2f(
                    fx * p_c(0, 0) / p_c(2, 0) + cx,
                    fy * p_c(1, 0) / p_c(2, 0) + cy
            );
        }

        inline Vector3f Pixel2Camera(const Vector2f &p_p, double depth = 1) const {
            return Vector3f(
                    (p_p(0, 0) - cx) * depth / fx,
                    (p_p(1, 0) - cy) * depth / fy,
                    depth
            );
        }

        inline Vector3f Pixel2World(const Vector2f &p_p, const SE3f &T_c_w, double depth = 1) const {
            return Camera2World(Pixel2Camera(p_p, depth), T_c_w);
        }

        Vector2f World2Pixel(const Vector3f &p_w, const SE3f &T_c_w) const {
            return Camera2Pixel(World2Camera(p_w, T_c_w));
        }

        // IMU related
        // compute the imu preintegration
        void ComputeIMUPreIntSinceLastFrame(const Frame *pLastF, IMUPreintegrator &imupreint) const;

        // update the pose matrix from navigation state
        void UpdatePoseFromNS(const SE3d &Tbc);

        // update the nav state from camera pose
        void UpdateNSFromPose();

        // set initial navigation, and set bias to zero
        void SetInitialNavStateAndBias(const NavState &ns);

        // update the navigation status using preintegration
        void UpdateNavState(const IMUPreintegrator &imupreint, const Vector3d &gw);

        // get navigation state
        NavState GetNavState(void) const {
            return mNavState;
        }

        // set navigation state
        void SetNavState(const NavState &ns) {
            mNavState = ns;
        }

        // set gyro bias
        void SetNavStateBiasGyr(const Vector3d &bg);

        // set accelerate bias
        void SetNavStateBiasAcc(const Vector3d &ba);


    public:
        // Vocabulary used for relocalization.
        ORBVocabulary *mpORBvocabulary = nullptr;     // 字典

        // Feature extractor. The right is used only in the stereo case.
        ORBextractor *mpORBextractorLeft = nullptr;
        ORBextractor *mpORBextractorRight = nullptr;     // 两个图像的特征提取器

        cv::Mat mImGray;    // 灰度图/双目左图
        cv::Mat mImRight;   // 双目右图
        cv::Mat mImDepth;   // RGBD深度图

        // 图像金字塔
        vector<cv::Mat> mvImagePyramid;

        // Frame timestamp.
        double mTimeStamp = -1;

        // Calibration matrix and OpenCV distortion parameters.
        // 内参
        Matrix3f mK = Matrix3f::Zero();
        static float fx;
        static float fy;
        static float cx;
        static float cy;
        static float invfx;
        static float invfy;

        static Mat map1, map2;  // for undistortion
        cv::Mat mDistCoef;  // 畸变参数
        static bool mbNeedUndistort;        // 如果传进来的图像是去过畸变的，就没必要再做一次

        // Stereo baseline multiplied by fx.
        float mbf = 0;       // 双目中的基线乘焦距

        // Stereo baseline in meters.
        float mb;                   // 基线

        // Threshold close/far points. Close points are inserted from 1 view.
        // Far points are inserted as in the monocular case from 2 views.
        float mThDepth;             // 远点的阈值

        // Number of KeyPoints.
        int N; ///< KeyPoints数量

        // Vector of keypoints (original for visualization) and undistorted (actually used by the system).
        // In the stereo case, mvKeysUn is redundant as images must be rectified.
        // In the RGB-D case, RGB images can be distorted.
        // mvKeys:原始左图像提取出的特征点
        // mvKeysRight:原始右图像提取出的特征点
        std::vector<cv::KeyPoint> mvKeys, mvKeysRight;
        vector<int> mvMatchedFrom;  // 标识每个特征点是从哪个Keyframe选取的

        // 是否已经提取了特征
        bool mbFeatureExtracted = false;    // flag to indicate if the ORB features are detected

        // Corresponding stereo coordinate and depth for each keypoint.
        // "Monocular" keypoints have a negative value.
        // 对于双目，mvuRight存储了左目像素点在右目中的对应点的横坐标
        // mvDepth对应的深度
        // 单目摄像头，这两个容器中存的都是-1
        std::vector<float> mvuRight;
        std::vector<float> mvDepth;

        // Bag of Words Vector structures.
        DBoW2::BowVector mBowVec;
        DBoW2::FeatureVector mFeatVec;

        // ORB descriptor, each row associated to a keypoint.
        // 左目摄像头和右目摄像头特征点对应的描述子（矩阵形式，第i行表示第i个特征）
        cv::Mat mDescriptors, mDescriptorsRight;

        // MapPoints associated to keypoints, NULL pointer if no association.
        // 每个特征点对应的MapPoint
        std::vector<MapPoint *> mvpMapPoints;

        // Flag to identify outlier associations.
        // 观测不到Map中的3D点
        std::vector<bool> mvbOutlier;

        // Keypoints are assigned to cells in a grid to reduce matching complexity when projecting MapPoints.
        // Grid的倒数,用来快速确认Grid的位置
        static float mfGridElementWidthInv;
        static float mfGridElementHeightInv;

        // 这是grid，将图像分成格子，保证提取的特征点比较均匀
        // #define FRAME_GRID_ROWS 48
        // #define FRAME_GRID_COLS 64
        std::vector<std::size_t> mGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS];
        bool mbGridSet = false;       // Grid是否已经被设置

        // Camera pose.
        SE3f mTcw;  // World 到 Camera
        bool mbPoseSet = false;      // Pose 是否已经设置

        // Current and Next Frame id.
        static long unsigned int nNextId; ///< Next Frame id.
        long unsigned int mnId = 0; ///< Current Frame id.

        // Reference Keyframe.
        KeyFrame *mpReferenceKF = nullptr;//指针，指向参考关键帧

        // Scale pyramid info.
        int mnScaleLevels = 0;//图像提金字塔的层数
        float mfScaleFactor = 0;//图像提金字塔的尺度因子
        float mfLogScaleFactor; // 对数scale缩放值

        // 各层金字塔的参数
        vector<float> mvScaleFactors;               // 缩放倍数
        vector<float> mvInvScaleFactors;            // 倒数
        vector<float> mvLevelSigma2;                // 平方
        vector<float> mvInvLevelSigma2;             // 倒数平方

        // Undistorted Image Bounds (computed once).
        // 用于确定画格子时的边界
        static float mnMinX;
        static float mnMaxX;
        static float mnMinY;
        static float mnMaxY;

        static bool mbInitialComputations;

        // IMU Data from last Frame to this Frame
        std::vector<IMUData> mvIMUDataSinceLastFrame;

        // For pose optimization, use as prior and prior information(inverse covariance)
        // 用IMU确定出来的先验
        Matrix<double, 15, 15> mMargCovInv = Matrix<double, 15, 15>::Zero();
        NavState mNavStatePrior;

        // 传感器类型
        SensorType mSensor;

        // Rotation, translation and camera center
        // 相机的一些量，都可以从 Tcw 里推导出来
        Matrix3f mRcw; ///< Rotation from world to camera
        Vector3f mtcw; ///< Translation from world to camera
        Matrix3f mRwc; ///< Rotation from camera to world
        Vector3f mOw; //==mtwc,Translation from camera to world

        // IMU 用的量，Rwb, twb, v, ba, bg,共15维，存储在NavState当中
        NavState mNavState; // Navigation state used in VIO
    };

}// namespace ygz

#endif // FRAME_H
