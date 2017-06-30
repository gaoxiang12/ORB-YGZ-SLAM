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

#ifndef YGZ_LOCALMAPPING_H_
#define YGZ_LOCALMAPPING_H_

#include "Common.h"

// 后端建图和优化线程

namespace ygz {

    class Tracking;

    class LoopClosing;

    class Map;

    class KeyFrame;

    class KeyFrameDatabase;

    class ConfigParam;

    class MapPoint;

    class LocalMapping {
    public:
        ConfigParam *mpParams= nullptr; // IMU配置参数
        bool mbUseIMU;

        std::thread *mptLocalMappingVIOInit =nullptr;   // 单目 初始化线程

        // KeyFrames in Local Window, for Local BA
        // Insert in ProcessNewKeyFrame()
        void AddToLocalWindow(KeyFrame *pKF);

        void DeleteBadInLocalWindow(void);

        mutex mMutexVINSIniting;
        bool mbVINSIniting;

        bool GetVINSIniting(void);

        void SetVINSIniting(bool flag);

        bool mbResetVINSInit;

        bool GetResetVINSInit(void);

        bool SetResetVINSInit(bool flag);

        void VINSInitThread(void);

        bool TryInitVIO(void);

        bool GetVINSInited(void);

        void SetVINSInited(bool flag);

        bool GetFirstVINSInited(void);

        void SetFirstVINSInited(bool flag);

        Vector3d GetGravityVec(void);

        double GetVINSInitScale(void) { return mnVINSInitScale; }

        bool GetMapUpdateFlagForTracking();

        void SetMapUpdateFlagInTracking(bool bflag);

        KeyFrame *GetMapUpdateKF();

        std::mutex mMutexUpdatingInitPoses;

        bool GetUpdatingInitPoses(void);

        void SetUpdatingInitPoses(bool flag);

        // 初始化相关
    protected:
        double mnStartTime;
        bool mbFirstTry;
        double mnVINSInitScale;
        Vector3d mGravityVec; // gravity vector in world frame

        std::mutex mMutexVINSInitFlag;
        bool mbVINSInited;

        std::mutex mMutexFirstVINSInitFlag;
        bool mbFirstVINSInited;

        unsigned int mnLocalWindowSize;
        std::list<KeyFrame *> mlLocalKeyFrames;

        std::mutex mMutexMapUpdateFlag;
        bool mbMapUpdateFlagForTracking;
        KeyFrame *mpMapUpdateKF;

        bool mbUpdatingInitPoses;

        std::mutex mMutexCopyInitKFs;
        bool mbCopyInitKFs;

        bool GetFlagCopyInitKFs() {
            unique_lock<mutex> lock(mMutexCopyInitKFs);
            return mbCopyInitKFs;
        }

        void SetFlagCopyInitKFs(bool flag) {
            unique_lock<mutex> lock(mMutexCopyInitKFs);
            mbCopyInitKFs = flag;
        }

    public:
        LocalMapping(Map *pMap, const float bMonocular, ConfigParam *pParams);

        void SetLoopCloser(LoopClosing *pLoopCloser);

        void SetTracker(Tracking *pTracker);

        // Main function
        void Run();

        void InsertKeyFrame(KeyFrame *pKF);

        // Thread Synch
        void RequestStop();

        void RequestReset();

        bool Stop();

        void Release();

        bool isStopped();

        bool stopRequested();

        bool AcceptKeyFrames();

        void SetAcceptKeyFrames(bool flag);

        bool SetNotStop(bool flag);

        void InterruptBA();

        void RequestFinish();

        bool isFinished();

        int KeyframesInQueue() {
            unique_lock<std::mutex> lock(mMutexNewKFs);
            return mlNewKeyFrames.size();
        }

    protected:

        bool CheckNewKeyFrames();

        void ProcessNewKeyFrame();

        void CreateNewMapPoints();

        void MapPointCulling();

        void SearchInNeighbors();

        void KeyFrameCulling();

        Matrix3f ComputeF12(KeyFrame *&pKF1, KeyFrame *&pKF2);

        bool mbMonocular;

        void ResetIfRequested();

        bool mbResetRequested;
        std::mutex mMutexReset;

        bool CheckFinish();

        void SetFinish();

        bool mbFinishRequested;
        bool mbFinished;
        std::mutex mMutexFinish;

        Map *mpMap =nullptr;

        LoopClosing *mpLoopCloser =nullptr;
        Tracking *mpTracker =nullptr;

        std::list<KeyFrame *> mlNewKeyFrames; ///< 等待处理的关键帧列表

        KeyFrame *mpCurrentKeyFrame =nullptr;

        std::list<MapPoint *> mlpRecentAddedMapPoints;

        std::mutex mMutexNewKFs;

        bool mbAbortBA;
        bool mbStopped;
        bool mbStopRequested;
        bool mbNotStop;
        std::mutex mMutexStop;

        bool mbAcceptKeyFrames;
        std::mutex mMutexAccept;
    };

} //namespace ygz

#endif // LOCALMAPPING_H
