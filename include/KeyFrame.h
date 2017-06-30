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

#ifndef YGZ_KEYFRAME_H_
#define YGZ_KEYFRAME_H_

#include "Common.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include "ORBVocabulary.h"
#include "ORBextractor.h"

#include "IMU/imudata.h"
#include "IMU/NavState.h"
#include "IMU/IMUPreintegrator.h"

namespace ygz {

    class Map;

    class MapPoint;

    class Frame;

    class KeyFrameDatabase;

    /* KeyFrame
     * 关键帧，和普通的Frame不一样，但是可以由Frame来构造
     * Frame的数据都是公开访问的，但Keyframe多数操作是带锁的
     * 许多数据会被三个线程同时访问，所以用锁的地方很普遍
     */
    class KeyFrame {
    public:

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        // 从Frame转KeyFrame的构造函数
        KeyFrame(Frame &F, Map *pMap, KeyFrameDatabase *pKFDB);

        // VIO constructor
        KeyFrame(Frame &F, Map *pMap, KeyFrameDatabase *pKFDB, std::vector<IMUData> vIMUData, KeyFrame *pLastKF = NULL);

        // Pose functions
        // KeyFrame的Pose可能被多个线程更新，因此全部要上锁
        void SetPose(const SE3f &Tcw);

        SE3f GetPose();

        SE3f GetPoseInverse();

        Vector3f GetCameraCenter();

        Vector3f GetStereoCenter();

        Matrix3f GetRotation();

        Vector3f GetTranslation();

        // Bag of Words Representation
        void ComputeBoW();

        // Covisibility graph functions
        void AddConnection(KeyFrame *pKF, const int &weight);

        // cov graph operations
        void EraseConnection(KeyFrame *pKF);

        void UpdateConnections();

        void UpdateBestCovisibles();

        std::set<KeyFrame *> GetConnectedKeyFrames();

        std::vector<KeyFrame *> GetVectorCovisibleKeyFrames();

        std::vector<KeyFrame *> GetBestCovisibilityKeyFrames(const int &N);

        std::vector<KeyFrame *> GetCovisiblesByWeight(const int &w);

        int GetWeight(KeyFrame *pKF);

        // Spanning tree functions
        void AddChild(KeyFrame *pKF);

        void EraseChild(KeyFrame *pKF);

        void ChangeParent(KeyFrame *pKF);

        std::set<KeyFrame *> GetChilds();

        KeyFrame *GetParent();

        bool hasChild(KeyFrame *pKF);

        // Loop Edges
        void AddLoopEdge(KeyFrame *pKF);

        std::set<KeyFrame *> GetLoopEdges();

        // MapPoint observation functions
        void AddMapPoint(MapPoint *pMP, const size_t &idx);

        void EraseMapPointMatch(const size_t &idx);

        void EraseMapPointMatch(MapPoint *pMP);

        void ReplaceMapPointMatch(const size_t &idx, MapPoint *pMP);

        std::set<MapPoint *> GetMapPoints();

        std::vector<MapPoint *> GetMapPointMatches();

        int TrackedMapPoints(const int &minObs);

        MapPoint *GetMapPoint(const size_t &idx);

        // KeyPoint functions
        std::vector<size_t> GetFeaturesInArea(const float &x, const float &y, const float &r) const;

        Vector3f UnprojectStereo(int i);

        // Image
        bool IsInImage(const float &x, const float &y) const;

        // Enable/Disable bad flag changes
        void SetNotErase();

        void SetErase();

        // Set/check bad flag
        void SetBadFlag();

        bool isBad();

        // Compute Scene Depth (q=2 median). Used in monocular.
        float ComputeSceneMedianDepth(const int q);

        static bool weightComp(int a, int b) {
            return a > b;
        }

        static bool lId(KeyFrame *pKF1, KeyFrame *pKF2) {
            return pKF1->mnId < pKF2->mnId;
        }

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

        inline Vector3f Pixel2Camera(const Vector2f &p_p, float depth = 1) const {
            return Vector3f(
                    (p_p(0, 0) - cx) * depth / fx,
                    (p_p(1, 0) - cy) * depth / fy,
                    depth
            );
        }

        inline Vector3f Pixel2World(const Vector2f &p_p, const SE3f &T_c_w, float depth = 1) const {
            return Camera2World(Pixel2Camera(p_p, depth), T_c_w);
        }

        inline Vector2f World2Pixel(const Vector3f &p_w, const SE3f &T_c_w) const {
            return Camera2Pixel(World2Camera(p_w, T_c_w));
        }


        KeyFrame *GetPrevKeyFrame(void);

        KeyFrame *GetNextKeyFrame(void);

        void SetPrevKeyFrame(KeyFrame *pKF);

        void SetNextKeyFrame(KeyFrame *pKF);

        std::vector<IMUData> GetVectorIMUData(void);

        void AppendIMUDataToFront(KeyFrame *pPrevKF);

        void ComputePreInt(void);

        const IMUPreintegrator &GetIMUPreInt(void);

        void UpdateNavStatePVRFromTcw(const SE3d &Tcw, const SE3d &Tbc);

        void UpdatePoseFromNS(const SE3d &Tbc);

        void UpdateNavState(const IMUPreintegrator &imupreint, const Vector3d &gw);

        void SetNavState(const NavState &ns);

        const NavState &GetNavState(void);

        void SetNavStateVel(const Vector3d &vel);

        void SetNavStatePos(const Vector3d &pos);

        void SetNavStateRot(const Matrix3d &rot);

        void SetNavStateRot(const SO3d &rot);

        void SetNavStateBiasGyr(const Vector3d &bg);

        void SetNavStateBiasAcc(const Vector3d &ba);

        void SetNavStateDeltaBg(const Vector3d &dbg);

        void SetNavStateDeltaBa(const Vector3d &dba);

        void SetInitialNavStateAndBias(const NavState &ns);

        // Variables used by loop closing
        NavState mNavStateGBA;       //mTcwGBA
        NavState mNavStateBefGBA;    //mTcwBefGBA

    public:

        // The following variables are accesed from only 1 thread or never change (no mutex needed).
        // nNextID名字改为nLastID更合适，表示上一个KeyFrame的ID号
        static long unsigned int nNextId;
        // 在nNextID的基础上加1就得到了mnID，为当前KeyFrame的ID号
        long unsigned int mnId =0;
        // 每个KeyFrame基本属性是它是一个Frame，KeyFrame初始化的时候需要Frame，

        // mnFrameId记录了该KeyFrame是由哪个Frame初始化的
        const long unsigned int mnFrameId;

        // 时间
        const double mTimeStamp =0;

        // Grid (to speed up feature matching)
        // 和Frame类中的定义相同
        const int mnGridCols;
        const int mnGridRows;
        const float mfGridElementWidthInv;
        const float mfGridElementHeightInv;

        // Variables used by the tracking
        long unsigned int mnTrackReferenceForFrame = 0;
        long unsigned int mnFuseTargetForKF = 0;

        // Variables used by the local mapping
        long unsigned int mnBALocalForKF = 0;
        long unsigned int mnBAFixedForKF = 0;

        // Variables used by the keyframe database
        long unsigned int mnLoopQuery = 0;
        int mnLoopWords = 0;
        float mLoopScore = 0;
        long unsigned int mnRelocQuery = 0;
        int mnRelocWords = 0;
        float mRelocScore = 0;

        // Variables used by loop closing
        SE3f mTcwGBA;
        SE3f mTcwBefGBA;
        long unsigned int mnBAGlobalForKF;

        // Calibration parameters
        const float fx, fy, cx, cy, invfx, invfy, mbf, mb, mThDepth;

        // Number of KeyPoints
        const int N;

        // KeyPoints, stereo coordinate and descriptors (all associated by an index)
        // 特征点位置
        const std::vector<cv::KeyPoint> mvKeys;
        const std::vector<float> mvuRight; // negative value for monocular points
        const std::vector<float> mvDepth; // negative value for monocular points
        const cv::Mat mDescriptors;

        //BoW
        DBoW2::BowVector mBowVec; ///< Vector of words to represent images
        DBoW2::FeatureVector mFeatVec; ///< Vector of nodes with indexes of local features

        // Pose relative to parent (this is computed when bad flag is activated)
        SE3f mTcp;

        // Scale
        const int mnScaleLevels;
        const float mfScaleFactor;
        const float mfLogScaleFactor;
        const std::vector<float> mvScaleFactors;
        const std::vector<float> mvLevelSigma2;
        const std::vector<float> mvInvLevelSigma2;

        // Image bounds and calibration
        const int mnMinX;
        const int mnMinY;
        const int mnMaxX;
        const int mnMaxY;
        const Matrix3f mK;

        // image pyramid
        vector<cv::Mat> mvImagePyramid;

    protected:
        // The following variables need to be accessed trough a mutex to be thread safe.
        // data used in imu
        std::mutex mMutexPrevKF;
        std::mutex mMutexNextKF;
        KeyFrame *mpPrevKeyFrame;
        KeyFrame *mpNextKeyFrame;

        // P, V, R, bg, ba, delta_bg, delta_ba (delta_bx is for optimization update)
        std::mutex mMutexNavState;
        NavState mNavState;

        // IMU Data from lask KeyFrame to this KeyFrame
        std::mutex mMutexIMUData;
        std::vector<IMUData> mvIMUData;
        IMUPreintegrator mIMUPreInt;

        // SE3 Pose and camera center
        SE3f Tcw;
        SE3f Twc;
        Vector3f Ow;
        Vector3f Cw; // Stereo middel point. Only for visualization

        // MapPoints associated to keypoints
        std::vector<MapPoint *> mvpMapPoints;

        // BoW
        KeyFrameDatabase *mpKeyFrameDB;
        ORBVocabulary *mpORBvocabulary;

        // Grid over the image to speed up feature matching
        std::vector<std::vector<std::vector<size_t> > > mGrid;

        std::map<KeyFrame *, int> mConnectedKeyFrameWeights; ///< 与该关键帧连接的关键帧与权重
        std::vector<KeyFrame *> mvpOrderedConnectedKeyFrames; ///< 排序后的关键帧
        std::vector<int> mvOrderedWeights; ///< 排序后的权重(从大到小)

        // Spanning Tree and Loop Edges
        // std::set是集合，相比vector，进行插入数据这样的操作时会自动排序
        bool mbFirstConnection = false;
        KeyFrame *mpParent = nullptr;
        std::set<KeyFrame *> mspChildrens;
        std::set<KeyFrame *> mspLoopEdges;

        // Bad flags
        bool mbNotErase = false;
        bool mbToBeErased = false;
        bool mbBad = false;

        float mHalfBaseline =0; // Only for visualization

        Map *mpMap =nullptr;

        std::mutex mMutexPose;
        std::mutex mMutexConnections;
        std::mutex mMutexFeatures;

    };

} //namespace ygz

#endif // KEYFRAME_H
