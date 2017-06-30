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


#ifndef YGZ_TRACKING_H_
#define YGZ_TRACKING_H_

#include "Frame.h"
#include "ORBVocabulary.h"
#include "KeyFrameDatabase.h"
#include "ORBextractor.h"
#include "Initializer.h"
#include "MapDrawer.h"
#include "SparseImageAlign.h"


#include "IMU/imudata.h"
#include "IMU/IMUPreintegrator.h"
#include "IMU/configparam.h"


// Tracking 线程
// 改的最多的地方
namespace ygz {

    class Viewer;

    class FrameDrawer;

    class Map;

    class LocalMapping;

    class LoopClosing;

    class System;

    class Tracking {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        Tracking(System *pSys, ORBVocabulary *pVoc, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Map *pMap,
                 KeyFrameDatabase *pKFDB, const string &strSettingPath, const int sensor, ConfigParam *pParams);

        ~Tracking();

        // Preprocess the input and call Track(). Extract features and performs stereo matching.
        SE3f GrabImageStereo(const cv::Mat &imRectLeft, const cv::Mat &imRectRight, const double &timestamp);

        SE3f GrabImageRGBD(const cv::Mat &imRGB, const cv::Mat &imD, const double &timestamp);

        SE3f GrabImageMonocular(const cv::Mat &im, const double &timestamp);

        void SetLocalMapper(LocalMapping *pLocalMapper);

        void SetLoopClosing(LoopClosing *pLoopClosing);

        void SetViewer(Viewer *pViewer);

        // Load new settings
        // The focal lenght should be similar or scale prediction will fail when projecting points
        // TODO: Modify MapPoint::PredictScale to take into account focal lenght
        void ChangeCalibration(const string &strSettingPath);

        // Use this function if you have deactivated local mapping and you only want to localize the camera.
        void InformOnlyTracking(const bool &flag);

    public:

        // Tracking states
        enum eTrackingState {
            SYSTEM_NOT_READY = -1,
            NO_IMAGES_YET = 0,
            NOT_INITIALIZED = 1,
            OK = 2,
            LOST = 3
        };

        eTrackingState mState;
        eTrackingState mLastProcessedState;

        // Input sensor
        int mSensor;

        // Current Frame
        Frame mCurrentFrame;
        cv::Mat mImGray;

        // Initialization Variables (Monocular)
        // 初始化时前两帧相关变量
        std::vector<int> mvIniLastMatches;
        std::vector<int> mvIniMatches;// 跟踪初始化时前两帧之间的匹配
        std::vector<cv::Point2f> mvbPrevMatched;
        std::vector<Vector3f> mvIniP3D;
        Frame mInitialFrame;

        // Lists used to recover the full camera trajectory at the end of the execution.
        // Basically we store the reference keyframe for each frame and its relative transformation
        list <SE3f> mlRelativeFramePoses;
        list<KeyFrame *> mlpReferences;
        list<double> mlFrameTimes;
        list<bool> mlbLost;

        // True if local mapping is deactivated and we are performing only localization
        bool mbOnlyTracking;

        void Reset();

        ConfigParam *mpParams = nullptr;      // VIO params

        // Whether use IMU
        bool mbUseIMU = false;
        bool mbVisionWeak = false;

        // Predict the NavState of Current Frame by IMU
        void PredictNavStateByIMU(bool trackLastKF);

        IMUPreintegrator mIMUPreIntInTrack;

        bool TrackLocalMapWithIMU(bool bTrackLastKF = false);

        bool TrackLocalMapDirectWithIMU(bool bTrackLastKF = false);

        SE3f GrabImageMonoVI(const cv::Mat &im, const std::vector<IMUData> &vimu, const double &timestamp);

        // IMU Data since last KF. Append when new data is provided
        // Should be cleared in 1. initialization beginning, 2. new keyframe created.
        std::vector<IMUData> mvIMUSinceLastKF;

        IMUPreintegrator
        GetIMUPreIntSinceLastKF(Frame *pCurF, KeyFrame *pLastKF, const std::vector<IMUData> &vIMUSInceLastKF);

        IMUPreintegrator GetIMUPreIntSinceLastFrame(Frame *pCurF, Frame *pLastF);


    protected:

        // Main tracking function. It is independent of the input sensor.
        void Track();

        // Map initialization for stereo and RGB-D
        void StereoInitialization();

        // Map initialization for monocular
        void MonocularInitialization();

        void CreateInitialMapMonocular();

        void CheckReplacedInLastFrame();

        bool TrackReferenceKeyFrame();

        void UpdateLastFrame();

        // 根据速度设置当前帧位姿，然后再从mLastFrame中寻找匹配点的投影
        // 投影用 ORBmatcher::SearchByProjection 实现
        // 注意会设置当前帧与地图点的关联
        bool TrackWithMotionModel();

        // 用SVO中的sparse alignment 来更新当前帧位姿
        // 但是这里不会处理特征点的关联和投影关系
        bool TrackWithSparseAlignment(bool bMapUpdated);

        bool Relocalization();

        // 特征点法的 Local Map 追踪
        // 与局部地图的特征匹配
        void UpdateLocalMap();

        void UpdateLocalPoints();

        void UpdateLocalKeyFrames();

        bool TrackLocalMap();

        void SearchLocalPoints();

        // 直接法的 Local Map 追踪
        // 与局部地图的直接匹配
        bool TrackLocalMapDirect();

        void SearchLocalPointsDirect();

        // 从地图观测中选取近的几个观测
        /**
         * @param[in] observations 地图点的观测数据
         * @param[in] n 选取的数量
         */
        vector<pair<KeyFrame *, size_t> > SelectNearestKeyframe(const map<KeyFrame *, size_t> &observations, int n = 5);

        bool NeedNewKeyFrame();

        void CreateNewKeyFrame();

        // In case of performing only localization, this flag is true when there are no matches to
        // points in the map. Still tracking will continue if there are enough matches with temporal points.
        // In that case we are doing visual odometry. The system will try to do relocalization to recover
        // "zero-drift" localization to the map.
        bool mbVO = false;

        //Other Thread Pointers
        LocalMapping *mpLocalMapper = nullptr;
        LoopClosing *mpLoopClosing = nullptr;

        //ORB
        // orb特征提取器，不管单目还是双目，mpORBextractorLeft都要用到
        // 如果是双目，则要用到mpORBextractorRight
        // 如果是单目，在初始化的时候使用mpIniORBextractor而不是mpORBextractorLeft，
        // mpIniORBextractor属性中提取的特征点个数是mpORBextractorLeft的两倍
        ORBextractor *mpORBextractorLeft = nullptr, *mpORBextractorRight = nullptr;
        ORBextractor *mpIniORBextractor = nullptr;

        //BoW
        ORBVocabulary *mpORBVocabulary = nullptr;
        KeyFrameDatabase *mpKeyFrameDB = nullptr;

        // Initalization (only for monocular)
        Initializer *mpInitializer = nullptr;

        //Local Map
        KeyFrame *mpReferenceKF = nullptr;
        std::vector<KeyFrame *> mvpLocalKeyFrames;
        std::vector<MapPoint *> mvpLocalMapPoints;
        set<MapPoint *> mvpDirectMapPointsCache;     // 缓存之前匹配到的地图点
        int mnCacheHitTh = 150;   // cache 命中点的阈值

        // System
        System *mpSystem = nullptr;

        //Drawers
        Viewer *mpViewer = nullptr;
        FrameDrawer *mpFrameDrawer = nullptr;
        MapDrawer *mpMapDrawer = nullptr;

        //Map
        Map *mpMap = nullptr;

        //Calibration matrix
        Matrix3f mK = Matrix3f::Identity();
        cv::Mat mDistCoef;
        float mbf = 0;

        //New KeyFrame rules (according to fps)
        int mMinFrames = 0;
        int mMaxFrames = 0;

        // Threshold close/far points
        // Points seen as close by the stereo/RGBD sensor are considered reliable
        // and inserted from just one frame. Far points requiere a match in two keyframes.
        float mThDepth = 0;

        // For RGB-D inputs only. For some datasets (e.g. TUM) the depthmap values are scaled.
        float mDepthMapFactor = 1;

        //Current matches in frame
        int mnMatchesInliers = 0;

        //Last Frame, KeyFrame and Relocalisation Info
        KeyFrame *mpLastKeyFrame = nullptr;
        Frame mLastFrame;
        unsigned int mnLastKeyFrameId;
        unsigned int mnLastRelocFrameId;

        //Motion Model
        SE3f mVelocity;
        bool mbVelocitySet = false;

        //Color order (true RGB, false BGR, ignored if grayscale)
        bool mbRGB = false;

        list<MapPoint *> mlpTemporalPoints;  // 双目和RGBD中会生成一些额外的地图点以使匹配更加稳定

        // sparse image alignment
        SparseImgAlign *mpAlign = nullptr;

        bool mbDirectFailed = false;    // 直接方法是否失败了？
    };

} //namespace ygz

#endif // TRACKING_H
