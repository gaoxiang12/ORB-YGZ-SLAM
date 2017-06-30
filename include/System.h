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


#ifndef YGZ_SYSTEM_H_
#define YGZ_SYSTEM_H_

#include "Common.h"
#include "ORBVocabulary.h"
#include "IMU/imudata.h"
#include "IMU/configparam.h"

// System
// 协调各个线程一同工作

namespace ygz {

    class Viewer;

    class FrameDrawer;

    class Map;

    class MapPoint;

    class Tracking;

    class LocalMapping;

    class LoopClosing;

    class MapDrawer;

    class KeyFrameDatabase;

    class System {
    public:

        // Input sensor
        enum eSensor {
            MONOCULAR = 0,
            STEREO = 1,
            RGBD = 2
        };

    public:

        // Initialize the SLAM system. It launches the Local Mapping, Loop Closing and Viewer threads.
        System(const string &strVocFile, const string &strSettingsFile, const eSensor sensor,
               const bool bUseViewer = true, ConfigParam *pParams = NULL);

        ~System();

        // Proccess the given stereo frame. Images must be synchronized and rectified.
        // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
        // Returns the camera pose (empty if tracking fails).
        cv::Mat TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp);

        // Process the given rgbd frame. Depthmap must be registered to the RGB frame.
        // Input image: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
        // Input depthmap: Float (CV_32F).
        // Returns the camera pose (empty if tracking fails).
        cv::Mat TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp);

        // Proccess the given monocular frame
        // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
        // Returns the camera pose (empty if tracking fails).
        cv::Mat TrackMonocular(const cv::Mat &im, const double &timestamp);

        /**
         *@brief Track Monocular with IMU
         * @param[in] im input image
         * @param[in] vimu the imu measurements from last image frame
         * @param[in] timestamp time
         * @return the esitmated pose
        */
        cv::Mat TrackMonoVI(const cv::Mat &im, const std::vector<IMUData> &vimu, const double &timestamp);


        // This stops local mapping thread (map building) and performs only camera tracking.
        void ActivateLocalizationMode();

        // This resumes local mapping thread and performs SLAM again.
        void DeactivateLocalizationMode();

        // Returns true if there have been a big map change (loop closure, global BA)
        // since last call to this function
        bool MapChanged();

        // Reset the system (clear map)
        void Reset();

        // All threads will be requested to finish.
        // It waits until all threads have finished.
        // This function must be called before saving the trajectory.
        void Shutdown();

        // Save camera trajectory in the TUM RGB-D dataset format.
        // Only for stereo and RGB-D. This method does not work for monocular.
        // Call first Shutdown()
        // See format details at: http://vision.in.tum.de/data/datasets/rgbd-dataset
        void SaveTrajectoryTUM(const string &filename);

        // Save keyframe poses in the TUM RGB-D dataset format.
        // This method works for all sensor input.
        // Call first Shutdown()
        // See format details at: http://vision.in.tum.de/data/datasets/rgbd-dataset
        void SaveKeyFrameTrajectoryTUM(const string &filename);

        // Save camera trajectory in the KITTI dataset format.
        // Only for stereo and RGB-D. This method does not work for monocular.
        // Call first Shutdown()
        // See format details at: http://www.cvlibs.net/datasets/kitti/eval_odometry.php
        void SaveTrajectoryKITTI(const string &filename);

        // save keyframe trajectory
        void SaveKeyFrameTrajectoryNavState(const string &filename);

        // TODO: Save/Load functions
        // SaveMap(const string &filename);
        // LoadMap(const string &filename);

        // Information from most recent processed frame
        // You can call this right after TrackMonocular (or stereo or RGBD)
        int GetTrackingState();

        std::vector<MapPoint *> GetTrackedMapPoints();

        std::vector<cv::KeyPoint> GetTrackedKeyPointsUn();

        // check if local map can accept keyframes (not busy)
        bool GetLocalMapAcceptKF();

        // Proccess the given monocular frame with IMU measurements
        // IMU measurements should be between LAST and CURRENT frame
        // (i.e. the timestamp of IMU measrements should be smaller than the timestamp of image)
        // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
        // Returns the camera pose (empty if tracking fails).

        ConfigParam *mpParams =nullptr;  // IMU related params

    private:

        // Input sensor
        eSensor mSensor;

        // ORB vocabulary used for place recognition and feature matching.
        ORBVocabulary *mpVocabulary =nullptr;

        // KeyFrame database for place recognition (relocalization and loop detection).
        KeyFrameDatabase *mpKeyFrameDatabase =nullptr;

        // Map structure that stores the pointers to all KeyFrames and MapPoints.
        Map *mpMap =nullptr;

        // Tracker. It receives a frame and computes the associated camera pose.
        // It also decides when to insert a new keyframe, create some new MapPoints and
        // performs relocalization if tracking fails.
        Tracking *mpTracker =nullptr;

        // Local Mapper. It manages the local map and performs local bundle adjustment.
        LocalMapping *mpLocalMapper =nullptr;

        // Loop Closer. It searches loops with every new keyframe. If there is a loop it performs
        // a pose graph optimization and full bundle adjustment (in a new thread) afterwards.
        LoopClosing *mpLoopCloser =nullptr;

        // The viewer draws the map and the current camera pose. It uses Pangolin.
        Viewer *mpViewer =nullptr;

        FrameDrawer *mpFrameDrawer =nullptr;
        MapDrawer *mpMapDrawer =nullptr;

        // System threads: Local Mapping, Loop Closing, Viewer.
        // The Tracking thread "lives" in the main execution thread that creates the System object.
        std::thread *mptLocalMapping =nullptr;
        std::thread *mptLoopClosing =nullptr;
        std::thread *mptViewer =nullptr;

        // Reset flag
        std::mutex mMutexReset;
        bool mbReset =false;

        // Change mode flags
        std::mutex mMutexMode;
        bool mbActivateLocalizationMode;
        bool mbDeactivateLocalizationMode;

        // Tracking state
        int mTrackingState;
        std::vector<MapPoint *> mTrackedMapPoints;
        std::vector<cv::KeyPoint> mTrackedKeyPoints;
        std::mutex mMutexState;
    };

}// namespace ygz

#endif // SYSTEM_H
