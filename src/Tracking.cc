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


#include "Tracking.h"
#include "ORBmatcher.h"
#include "FrameDrawer.h"
#include "Converter.h"
#include "Map.h"
#include "Initializer.h"
#include "FrameDrawer.h"
#include "MapDrawer.h"
#include "Optimizer.h"
#include "PnPsolver.h"
#include "System.h"
#include "LocalMapping.h"
#include "Viewer.h"

using namespace std;

namespace ygz {

    SE3f Tracking::GrabImageMonoVI(const cv::Mat &im, const std::vector<IMUData> &vimu, const double &timestamp) {
        mvIMUSinceLastKF.insert(mvIMUSinceLastKF.end(), vimu.begin(), vimu.end());
        mImGray = im;

        if (mImGray.channels() == 3) {
            if (mbRGB)
                cvtColor(mImGray, mImGray, CV_RGB2GRAY);
            else
                cvtColor(mImGray, mImGray, CV_BGR2GRAY);
        } else if (mImGray.channels() == 4) {
            if (mbRGB)
                cvtColor(mImGray, mImGray, CV_RGBA2GRAY);
            else
                cvtColor(mImGray, mImGray, CV_BGRA2GRAY);
        }

        if (mState == NOT_INITIALIZED || mState == NO_IMAGES_YET)
            mCurrentFrame = Frame(mImGray, timestamp, vimu, mpIniORBextractor, mpORBVocabulary, mK, mDistCoef, mbf,
                                  mThDepth);
        else
            mCurrentFrame = Frame(mImGray, timestamp, vimu, mpORBextractorLeft, mpORBVocabulary, mK, mDistCoef, mbf,
                                  mThDepth/*,mpLastKeyFrame*/);

        Track();

        return mCurrentFrame.mTcw;
    }

//-------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------

    Tracking::Tracking(
            System *pSys, ORBVocabulary *pVoc, FrameDrawer *pFrameDrawer,
            MapDrawer *pMapDrawer, Map *pMap, KeyFrameDatabase *pKFDB, const string &strSettingPath, const int sensor,
            ConfigParam *pParams) :
            mState(NO_IMAGES_YET), mSensor(sensor), mbOnlyTracking(false), mbVO(false), mpORBVocabulary(pVoc),
            mpKeyFrameDB(pKFDB), mpInitializer(static_cast<Initializer *>(NULL)), mpSystem(pSys), mpViewer(NULL),
            mpFrameDrawer(pFrameDrawer), mpMapDrawer(pMapDrawer), mpMap(pMap), mnLastRelocFrameId(0) {
        mpParams = pParams;

        // Load camera parameters from settings file

        cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
        float fx = fSettings["Camera.fx"];
        float fy = fSettings["Camera.fy"];
        float cx = fSettings["Camera.cx"];
        float cy = fSettings["Camera.cy"];

        mK = Matrix3f::Identity();
        mK(0, 0) = fx;
        mK(1, 1) = fy;
        mK(0, 2) = cx;
        mK(1, 2) = cy;

        int bUseDistK6 = fSettings["Camera.bUseDistK6"];
        if (bUseDistK6 == 1) {
            cv::Mat DistCoef(8, 1, CV_32F);
            DistCoef.at<float>(0) = fSettings["Camera.k1"];
            DistCoef.at<float>(1) = fSettings["Camera.k2"];
            DistCoef.at<float>(2) = fSettings["Camera.p1"];
            DistCoef.at<float>(3) = fSettings["Camera.p2"];
            DistCoef.at<float>(4) = fSettings["Camera.k3"];
            DistCoef.at<float>(5) = fSettings["Camera.k4"];
            DistCoef.at<float>(6) = fSettings["Camera.k5"];
            DistCoef.at<float>(7) = fSettings["Camera.k6"];
            DistCoef.copyTo(mDistCoef);

            for (size_t i = 0; i < 8; i++)
                if (DistCoef.at<float>(i) != 0)
                    Frame::mbNeedUndistort = true;
        } else {
            cv::Mat DistCoef(4, 1, CV_32F);
            DistCoef.at<float>(0) = fSettings["Camera.k1"];
            DistCoef.at<float>(1) = fSettings["Camera.k2"];
            DistCoef.at<float>(2) = fSettings["Camera.p1"];
            DistCoef.at<float>(3) = fSettings["Camera.p2"];
            for (size_t i = 0; i < 4; i++)
                if (DistCoef.at<float>(i) != 0)
                    Frame::mbNeedUndistort = true;

            const float k3 = fSettings["Camera.k3"];
            if (k3 != 0) {
                DistCoef.resize(5);
                DistCoef.at<float>(4) = k3;
                Frame::mbNeedUndistort = true;
            }
            DistCoef.copyTo(mDistCoef);
        }

        mbf = fSettings["Camera.bf"];

        float fps = fSettings["Camera.fps"];
        if (fps == 0)
            fps = 30;

        // Max/Min Frames to insert keyframes and to check relocalisation
        if (mSensor == System::RGBD)
            mMinFrames = 0;
        else
            mMinFrames = 0;
        mMaxFrames = fps;

        cout << endl << "Camera Parameters: " << endl;
        cout << "- fx: " << fx << endl;
        cout << "- fy: " << fy << endl;
        cout << "- cx: " << cx << endl;
        cout << "- cy: " << cy << endl;
        cout << "- k1: " << mDistCoef.at<float>(0) << endl;
        cout << "- k2: " << mDistCoef.at<float>(1) << endl;
        if (mDistCoef.rows == 5)
            cout << "- k3: " << mDistCoef.at<float>(4) << endl;
        if (mDistCoef.rows == 8) {
            cout << "- k3: " << mDistCoef.at<float>(4) << endl;
            cout << "- k4: " << mDistCoef.at<float>(5) << endl;
            cout << "- k5: " << mDistCoef.at<float>(6) << endl;
            cout << "- k6: " << mDistCoef.at<float>(7) << endl;
        }
        cout << "- p1: " << mDistCoef.at<float>(2) << endl;
        cout << "- p2: " << mDistCoef.at<float>(3) << endl;
        cout << "- fps: " << fps << endl;


        int nRGB = fSettings["Camera.RGB"];
        mbRGB = nRGB;

        if (mbRGB)
            cout << "- color order: RGB (ignored if grayscale)" << endl;
        else
            cout << "- color order: BGR (ignored if grayscale)" << endl;

        // Load ORB parameters

        int nFeatures = fSettings["ORBextractor.nFeatures"];
        float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
        int nLevels = fSettings["ORBextractor.nLevels"];
        int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
        int fMinThFAST = fSettings["ORBextractor.minThFAST"];

        mpORBextractorLeft = new ORBextractor(nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);

        if (sensor == System::STEREO)
            mpORBextractorRight = new ORBextractor(nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);

        if (sensor == System::MONOCULAR)
            mpIniORBextractor = new ORBextractor(2 * nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);

        cout << endl << "ORB Extractor Parameters: " << endl;
        cout << "- Number of Features: " << nFeatures << endl;
        cout << "- Scale Levels: " << nLevels << endl;
        cout << "- Scale Factor: " << fScaleFactor << endl;
        cout << "- Initial Fast Threshold: " << fIniThFAST << endl;
        cout << "- Minimum Fast Threshold: " << fMinThFAST << endl;

        if (sensor == System::STEREO || sensor == System::RGBD) {
            mThDepth = mbf * (float) fSettings["ThDepth"] / fx;
            cout << endl << "Depth Threshold (Close/Far Points): " << mThDepth << endl;
        }

        if (sensor == System::RGBD) {
            mDepthMapFactor = fSettings["DepthMapFactor"];
            if (fabs(mDepthMapFactor) < 1e-5)
                mDepthMapFactor = 1;
            else
                mDepthMapFactor = 1.0f / mDepthMapFactor;
        }

        mpAlign = new ygz::SparseImgAlign(nLevels - 1, 1);
        mbUseIMU = mpParams->GetUseIMUFlag();

        int nCacheHitTh = fSettings["Tracking.CacheFeatures"];
        if (nCacheHitTh) {
            mnCacheHitTh = nCacheHitTh;
        }
    }

    Tracking::~Tracking() {
        if (mpAlign)
            delete mpAlign;
    }


    void Tracking::SetLocalMapper(LocalMapping *pLocalMapper) {
        mpLocalMapper = pLocalMapper;
    }

    void Tracking::SetLoopClosing(LoopClosing *pLoopClosing) {
        mpLoopClosing = pLoopClosing;
    }

    void Tracking::SetViewer(Viewer *pViewer) {
        mpViewer = pViewer;
    }


    SE3f Tracking::GrabImageStereo(const cv::Mat &imRectLeft, const cv::Mat &imRectRight, const double &timestamp) {
        mImGray = imRectLeft;
        cv::Mat imGrayRight = imRectRight;

        if (mImGray.channels() == 3) {
            if (mbRGB) {
                cvtColor(mImGray, mImGray, CV_RGB2GRAY);
                cvtColor(imGrayRight, imGrayRight, CV_RGB2GRAY);
            } else {
                cvtColor(mImGray, mImGray, CV_BGR2GRAY);
                cvtColor(imGrayRight, imGrayRight, CV_BGR2GRAY);
            }
        } else if (mImGray.channels() == 4) {
            if (mbRGB) {
                cvtColor(mImGray, mImGray, CV_RGBA2GRAY);
                cvtColor(imGrayRight, imGrayRight, CV_RGBA2GRAY);
            } else {
                cvtColor(mImGray, mImGray, CV_BGRA2GRAY);
                cvtColor(imGrayRight, imGrayRight, CV_BGRA2GRAY);
            }
        }

        mCurrentFrame = Frame(mImGray, imGrayRight, timestamp, mpORBextractorLeft, mpORBextractorRight, mpORBVocabulary,
                              mK, mDistCoef, mbf, mThDepth);

        Track();

        return mCurrentFrame.mTcw;
    }


    SE3f Tracking::GrabImageRGBD(const cv::Mat &imRGB, const cv::Mat &imD, const double &timestamp) {
        mImGray = imRGB;
        cv::Mat imDepth = imD;

        if (mImGray.channels() == 3) {
            if (mbRGB)
                cvtColor(mImGray, mImGray, CV_RGB2GRAY);
            else
                cvtColor(mImGray, mImGray, CV_BGR2GRAY);
        } else if (mImGray.channels() == 4) {
            if (mbRGB)
                cvtColor(mImGray, mImGray, CV_RGBA2GRAY);
            else
                cvtColor(mImGray, mImGray, CV_BGRA2GRAY);
        }

        if ((fabs(mDepthMapFactor - 1.0f) > 1e-5) || imDepth.type() != CV_32F)
            imDepth.convertTo(imDepth, CV_32F, mDepthMapFactor);

        mCurrentFrame = Frame(mImGray, imDepth, timestamp, mpORBextractorLeft, mpORBVocabulary, mK, mDistCoef, mbf,
                              mThDepth);

        Track();

        return mCurrentFrame.mTcw;
    }


    SE3f Tracking::GrabImageMonocular(const cv::Mat &im, const double &timestamp) {
        mImGray = im;

        if (mImGray.channels() == 3) {
            if (mbRGB)
                cvtColor(mImGray, mImGray, CV_RGB2GRAY);
            else
                cvtColor(mImGray, mImGray, CV_BGR2GRAY);
        } else if (mImGray.channels() == 4) {
            if (mbRGB)
                cvtColor(mImGray, mImGray, CV_RGBA2GRAY);
            else
                cvtColor(mImGray, mImGray, CV_BGRA2GRAY);
        }

        if (mState == NOT_INITIALIZED || mState == NO_IMAGES_YET)
            mCurrentFrame = Frame(mImGray, timestamp, mpIniORBextractor, mpORBVocabulary, mK, mDistCoef, mbf, mThDepth);
        else
            mCurrentFrame = Frame(mImGray, timestamp, mpORBextractorLeft, mpORBVocabulary, mK, mDistCoef, mbf,
                                  mThDepth);

        Track();

        return mCurrentFrame.mTcw;
    }

    void Tracking::Track() {
        if (mState == NO_IMAGES_YET) {
            mState = NOT_INITIALIZED;
        }

        mLastProcessedState = mState;

        // Get Map Mutex -> Map cannot be changed
        unique_lock<mutex> lock(mpMap->mMutexMapUpdate);

        // Different operation, according to whether the map is updated
        bool bMapUpdated = false;
        if (mpLocalMapper->GetMapUpdateFlagForTracking()) {
            bMapUpdated = true;
            mpLocalMapper->SetMapUpdateFlagInTracking(false);
        }

        if (mpLoopClosing->GetMapUpdateFlagForTracking()) {
            LOG(INFO) << "Tracking noted that map is updated by loop closing" << endl;
            bMapUpdated = true;
            mpLoopClosing->SetMapUpdateFlagInTracking(false);
        }
        if (mpLocalMapper->GetFirstVINSInited()) {
            for (list<SE3f>::iterator lit = mlRelativeFramePoses.begin(), lend = mlRelativeFramePoses.end();
                 lit != lend; lit++) {
                lit->translation() *= mpLocalMapper->GetVINSInitScale();
            }
        }

        if (mState == NOT_INITIALIZED) {
            mCurrentFrame.ExtractFeatures();

            if (mSensor == System::STEREO || mSensor == System::RGBD)
                StereoInitialization();
            else
                MonocularInitialization();

            mpFrameDrawer->Update(this);

            if (mState != OK)
                return;
        } else {
            // System is initialized. Track Frame.
            bool bOK;

            // Initial camera pose estimation using motion model or relocalization (if tracking is lost)
            // mbOnlyTracking 等于 false表示正常VO模式（有地图更新），mbOnlyTracking等于true表示用户手动选择定位模式
            if (!mbOnlyTracking) {
                // Local Mapping is activated. This is the normal behaviour, unless
                // you explicitly activate the "only tracking" mode.

                if (mState == OK) {
                    // Local Mapping might have changed some MapPoints tracked in last frame
                    // 检查并更新上一帧被替换的MapPoints
                    // 更新Fuse函数和SearchAndFuse函数替换的MapPoints
                    CheckReplacedInLastFrame();

                    // 运动模型是空的或刚完成重定位
                    // mnLastRelocFrameId上一次重定位的那一帧
                    if (mbVelocitySet == false) {
                        bOK = TrackReferenceKeyFrame();
                    } else {
                        // 根据恒速模型设定当前帧的初始位姿
                        // 通过投影的方式在参考帧中找当前帧特征点的匹配点
                        // 优化每个特征点所对应3D点的投影误差即可得到位姿

                        //// we modify this with sparse direct alignment, in which we don't need to extract new features or do feature matching
                        bOK = TrackWithSparseAlignment(mpLocalMapper->GetFirstVINSInited() || bMapUpdated);

                        if (bOK == false)
                            bOK = TrackWithMotionModel();
                        if (!bOK)
                            bOK = TrackReferenceKeyFrame();
                    }
                } else {
                    if (mbUseIMU == false) {
                        LOG(INFO) << "Try relocalization" << endl;
                        bOK = Relocalization();
                    } else {
                        //TODO: add re-localization. Need to re-initialize P/V/R/bg/ba
                        LOG(ERROR) << "TODO: add relocalization. Lost, reset." << endl;
                        // mpSystem->Reset();
                        return;
                    }
                }
            } else {
                // TODO(jingpang): Localization Mode is currently not considered in Visual-Inertial code...

                // Localization Mode: Local Mapping is deactivated
                if (mState == LOST) {
                    bOK = Relocalization();
                } else {
                    if (!mbVO) {
                        // In last frame we tracked enough MapPoints in the map

                        if (mbVelocitySet) {
                            // bOK = TrackWithMotionModel();
                            bOK = TrackWithSparseAlignment(mpLocalMapper->GetFirstVINSInited() || bMapUpdated);
                        } else {
                            bOK = TrackReferenceKeyFrame();
                        }
                    } else {
                        // In last frame we tracked mainly "visual odometry" points.

                        // We compute two camera poses, one from motion model and one doing relocalization.
                        // If relocalization is sucessfull we choose that solution, otherwise we retain
                        // the "visual odometry" solution.

                        bool bOKMM = false;
                        bool bOKReloc = false;
                        vector<MapPoint *> vpMPsMM;
                        vector<bool> vbOutMM;
                        SE3f TcwMM;
                        if (mbVelocitySet) {
                            bOKMM = TrackWithMotionModel();
                            vpMPsMM = mCurrentFrame.mvpMapPoints;
                            vbOutMM = mCurrentFrame.mvbOutlier;
                            TcwMM = mCurrentFrame.mTcw;
                        }
                        bOKReloc = Relocalization();

                        if (bOKMM && !bOKReloc) {
                            mCurrentFrame.SetPose(TcwMM);
                            mCurrentFrame.mvpMapPoints = vpMPsMM;
                            mCurrentFrame.mvbOutlier = vbOutMM;

                            if (mbVO) {
                                for (int i = 0; i < mCurrentFrame.N; i++) {
                                    if (mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i]) {
                                        mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                                    }
                                }
                            }
                        } else if (bOKReloc) {
                            mbVO = false;
                        }

                        bOK = bOKReloc || bOKMM;
                    }
                }
            }

            mCurrentFrame.mpReferenceKF = mpReferenceKF;

            // If we have an initial estimation of the camera pose and matching. Track the local map.
            // 步骤2.2：在帧间匹配得到初始的姿态后，现在对local map进行跟踪得到更多的匹配，并优化当前位姿
            // local map:当前帧、当前帧的MapPoints、当前关键帧与其它关键帧共视关系
            // 在步骤2.1中主要是两两跟踪（恒速模型跟踪上一帧、跟踪参考帧），这里搜索局部关键帧后搜集所有局部MapPoints，
            // 然后将局部MapPoints和当前帧进行投影匹配，得到更多匹配的MapPoints后进行Pose优化
            if (!mbOnlyTracking) {
                if (bOK) {
                    if (!mbUseIMU || !mpLocalMapper->GetVINSInited()) {
                        // 不使用IMU或IMU未初始化
                        if (mCurrentFrame.mbFeatureExtracted) {
                            // 已提特征， 用特征点法与 local map 进行比较
                            bOK = TrackLocalMap();
                        } else {
                            // 未提特征，用Direct方法比较
                            bOK = TrackLocalMapDirect();
                            if (!bOK) {
                                // Track local map 数量少，可能是前一步位姿估计的不准，也可能是视觉上匹配的就是太少
                                // clear the features and try search in local map
                                mCurrentFrame.N = 0;
                                mCurrentFrame.mBowVec.clear();
                                mCurrentFrame.mvKeys.clear();
                                mCurrentFrame.mvpMapPoints.clear();
                                mCurrentFrame.ExtractFeatures();
                                if (mbVelocitySet)  // 如果有速度就重设一下速度
                                    bOK = TrackWithMotionModel();
                                bOK = TrackLocalMap();
                                if ( bOK == false ) {
                                    LOG(WARNING)<<"Still failed, abort"<<endl;
                                }
                            }
                        }
                    } else {
                        // 使用IMU且IMU已经初始化
                        if (mCurrentFrame.mbFeatureExtracted) {
                            // 已提特征， 用特征点法与 local map 进行比较
                            bOK = TrackLocalMapWithIMU(mpLocalMapper->GetFirstVINSInited() || bMapUpdated);

                            if (!bOK)
                                LOG(INFO) << "Track Local map with imu failed." << endl;
                        } else {
                            // 未提特征，用Direct方法比较
                            bOK = TrackLocalMapDirectWithIMU(mpLocalMapper->GetFirstVINSInited() || bMapUpdated);

                            if (!bOK) {
                                LOG(INFO) << "Track Local map direct with IMU failed. VINS first inited: "
                                          << mpLocalMapper->GetFirstVINSInited() << ", mapupdated:" << bMapUpdated
                                          << endl;
                            }
                        }
                    }
                }
            } else {
                // TODO(jingpang): Localization Mode is currently not considered in Visual-Inertial code...

                // mbVO true means that there are few matches to MapPoints in the map. We cannot retrieve
                // a local map and therefore we do not perform TrackLocalMap(). Once the system relocalizes
                // the camera we will use the local map again.
                if (bOK && !mbVO)
                    bOK = TrackLocalMap();
            }

            if (bOK) {
                mState = OK;
                mbVisionWeak = false;
            } else {
                if (mbUseIMU && mpLocalMapper->GetVINSInited()) {
                    mbVisionWeak = true;
                    LOG(INFO) << "Set vision weak = true" << endl;
                } else {
                    // without imu or imu not inited, set lost
                    LOG(INFO) << "Set state to Lost" << endl;
                    mState = LOST;
                }
            }

            // Update drawer
            mpFrameDrawer->Update(this);

            // If tracking were good, check if we insert a keyframe
            if (bOK) {
                // Update motion model
                // 步骤2.3：更新恒速运动模型TrackWithMotionModel中的mVelocity
                if (mLastFrame.mbPoseSet) {
                    SE3f LastTwc = mLastFrame.mTcw.inverse();
                    mVelocity = mCurrentFrame.mTcw * LastTwc;
                    mbVelocitySet = true;
                } else {
                    mVelocity = SE3f();
                    mbVelocitySet = false;
                }

                mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw.cast<double>());

                // Clean VO matches
                // 步骤2.4：清除UpdateLastFrame中为当前帧临时添加的MapPoints
                for (int i = 0; i < mCurrentFrame.N; i++) {
                    MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];
                    if (pMP) {
                        if (pMP->Observations() < 1) {
                            mCurrentFrame.mvbOutlier[i] = false;
                            mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
                        }
                    }
                }

                // Delete temporal MapPoints
                // 步骤2.5：清除临时的MapPoints，这些MapPoints在TrackWithMotionModel的UpdateLastFrame函数里生成（仅双目和rgbd）
                // 步骤2.4中只是在当前帧中将这些MapPoints剔除，这里从MapPoints数据库中删除
                // 这里生成的仅仅是为了提高双目或rgbd摄像头的帧间跟踪效果，用完以后就扔了，没有添加到地图中
                for (list<MapPoint *>::iterator lit = mlpTemporalPoints.begin(), lend = mlpTemporalPoints.end();
                     lit != lend; lit++) {
                    MapPoint *pMP = *lit;
                    delete pMP;
                }
                mlpTemporalPoints.clear();

                // Check if we need to insert a new keyframe
                // 步骤2.6：检测并插入关键帧，对于双目会产生新的MapPoints
                if (NeedNewKeyFrame())
                    CreateNewKeyFrame();
                // TODO(jingpang): Add mbCreateNewKFAfterReloc here to consider KeyFrame create after relocalization

                // We allow points with high innovation (considererd outliers by the Huber Function)
                // pass to the new keyframe, so that bundle adjustment will finally decide
                // if they are outliers or not. We don't want next frame to estimate its position
                // with those points so we discard them in the frame.
                // 删除那些在bundle adjustment中检测为outlier的3D map点
                for (int i = 0; i < mCurrentFrame.N; i++) {
                    if (mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])
                        mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
                }

                // Clear First-Init flag
                if (mpLocalMapper->GetFirstVINSInited()) {
                    mpLocalMapper->SetFirstVINSInited(false);
                }
            } else // vision track failed
            {
                if (mbUseIMU && mpLocalMapper->GetVINSInited()) {
                    // if we have IMU, we can still do things like imu propatation
                    // also show the pose
                    LOG(INFO) << "Try using imu predicted pose" << endl;
                    // reset the pose to IMU prediction
                    PredictNavStateByIMU(true);
                    mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw.cast<double>());
                    mCurrentFrame.mpReferenceKF = mpLastKeyFrame;

                    // check keyframe
                    if (NeedNewKeyFrame())
                        CreateNewKeyFrame();
                }
            }

            // Reset if the camera get lost soon after initialization
            // 跟踪失败，且地图中没有许多关键帧，表示离初始化不久，Reset重新进行初始化
            if (mState == LOST) {
                if (mpMap->KeyFramesInMap() <= 5)
                    // if(!mpLocalMapper->GetVINSInited())
                {
                    cout << "Track lost soon after initialisation, reseting..." << endl;
                    mpSystem->Reset();
                    return;
                }
            }

            if (!mCurrentFrame.mpReferenceKF)
                mCurrentFrame.mpReferenceKF = mpReferenceKF;

            // 保存上一帧的数据，用于 sparse alignment
            if (mCurrentFrame.N > 30)
                mLastFrame = Frame(mCurrentFrame);
        }

        // Store frame pose information to retrieve the complete camera trajectory afterwards.
        // 步骤3：记录位姿信息，用于轨迹复现
        if (mCurrentFrame.mbPoseSet) {
            // 计算相对姿态T_currentFrame_referenceKeyFrame
            SE3f Tcr = mCurrentFrame.mTcw * mCurrentFrame.mpReferenceKF->GetPoseInverse();
            mlRelativeFramePoses.push_back(Tcr);
            mlpReferences.push_back(mpReferenceKF);
            mlFrameTimes.push_back(mCurrentFrame.mTimeStamp);
            mlbLost.push_back(mState == LOST);
        } else {
            // This can happen if tracking is lost
            mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
            mlpReferences.push_back(mlpReferences.back());
            mlFrameTimes.push_back(mlFrameTimes.back());
            mlbLost.push_back(mState == LOST);
        }

    }

    /**
     * @brief 双目和rgbd的地图初始化
     *
     * 由于具有深度信息，直接生成MapPoints
     */
    void Tracking::StereoInitialization() {
        if (mCurrentFrame.N > 500) {
            // Set Frame pose to the origin
            mCurrentFrame.SetPose(SE3f());

            // Create KeyFrame
            KeyFrame *pKFini = new KeyFrame(mCurrentFrame, mpMap, mpKeyFrameDB);

            // Insert KeyFrame in the map
            mpMap->AddKeyFrame(pKFini);

            // Create MapPoints and asscoiate to KeyFrame
            for (int i = 0; i < mCurrentFrame.N; i++) {
                float z = mCurrentFrame.mvDepth[i];
                if (z > 0) {
                    Vector3f x3D = mCurrentFrame.UnprojectStereo(i);
                    MapPoint *pNewMP = new MapPoint(x3D, pKFini, mpMap);
                    pNewMP->AddObservation(pKFini, i);
                    pKFini->AddMapPoint(pNewMP, i);
                    pNewMP->ComputeDistinctiveDescriptors();
                    pNewMP->UpdateNormalAndDepth();
                    mpMap->AddMapPoint(pNewMP);

                    mCurrentFrame.mvpMapPoints[i] = pNewMP;
                }
            }

            cout << "New map created with " << mpMap->MapPointsInMap() << " points" << endl;

            mpLocalMapper->InsertKeyFrame(pKFini);

            mLastFrame = Frame(mCurrentFrame);
            mnLastKeyFrameId = mCurrentFrame.mnId;
            mpLastKeyFrame = pKFini;

            mvpLocalKeyFrames.push_back(pKFini);
            mvpLocalMapPoints = mpMap->GetAllMapPoints();
            mpReferenceKF = pKFini;
            mCurrentFrame.mpReferenceKF = pKFini;

            mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

            mpMap->mvpKeyFrameOrigins.push_back(pKFini);

            mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw.cast<double>());

            mState = OK;
        }
    }

    void Tracking::MonocularInitialization() {

        if (!mpInitializer) {
            // Set Reference Frame
            if (mCurrentFrame.mvKeys.size() > 100) {
                mInitialFrame = Frame(mCurrentFrame);
                mLastFrame = Frame(mCurrentFrame);
                mvbPrevMatched.resize(mCurrentFrame.mvKeys.size());
                for (size_t i = 0; i < mCurrentFrame.mvKeys.size(); i++)
                    mvbPrevMatched[i] = mCurrentFrame.mvKeys[i].pt;

                if (mpInitializer)
                    delete mpInitializer;

                mpInitializer = new Initializer(mCurrentFrame, 1.0, 200);

                fill(mvIniMatches.begin(), mvIniMatches.end(), -1);

                return;
            }
        } else {
            // Try to initialize
            if ((int) mCurrentFrame.mvKeys.size() <= 100) {
                delete mpInitializer;
                mpInitializer = static_cast<Initializer *>(NULL);
                fill(mvIniMatches.begin(), mvIniMatches.end(), -1);
                return;
            }

            // Find correspondences
            ORBmatcher matcher(0.9, true);
            int nmatches = matcher.SearchForInitialization(mInitialFrame, mCurrentFrame, mvbPrevMatched, mvIniMatches,
                                                           100);

            // Check if there are enough correspondences
            if (nmatches < 100) {
                delete mpInitializer;
                mpInitializer = static_cast<Initializer *>(NULL);
                return;
            }

            Matrix3f Rcw; // Current Camera Rotation
            Vector3f tcw; // Current Camera Translation
            vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)

            if (mpInitializer->Initialize(mCurrentFrame, mvIniMatches, Rcw, tcw, mvIniP3D, vbTriangulated)) {
                for (size_t i = 0, iend = mvIniMatches.size(); i < iend; i++) {
                    if (mvIniMatches[i] >= 0 && !vbTriangulated[i]) {
                        mvIniMatches[i] = -1;
                        nmatches--;
                    }
                }

                // Set Frame Poses
                mCurrentFrame.SetPose(SE3f(Rcw, tcw));

                CreateInitialMapMonocular();

                // mMinFrames = 10;    // 初始化完毕后，设置插关键帧的最小间隔
            }
        }
    }

/**
 * @brief CreateInitialMapMonocular
 *
 * 为单目摄像头三角化生成MapPoints
 */
    void Tracking::CreateInitialMapMonocular() {
        // The first imu package include 2 parts for KF1 and KF2
        vector<IMUData> vimu1, vimu2;
        if (mbUseIMU) {
            for (size_t i = 0; i < mvIMUSinceLastKF.size(); i++) {
                IMUData imu = mvIMUSinceLastKF[i];
                if (imu._t < mInitialFrame.mTimeStamp)
                    vimu1.push_back(imu);
                else
                    vimu2.push_back(imu);
            }
        }

        // Create KeyFrames
        //KeyFrame* pKFini = new KeyFrame(mInitialFrame,mpMap,mpKeyFrameDB);
        //KeyFrame* pKFcur = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);
        KeyFrame *pKFini = new KeyFrame(mInitialFrame, mpMap, mpKeyFrameDB, vimu1, NULL);
        pKFini->ComputePreInt();
        KeyFrame *pKFcur = new KeyFrame(mCurrentFrame, mpMap, mpKeyFrameDB, vimu2, pKFini);
        pKFcur->ComputePreInt();

        // Clear IMUData buffer
        mvIMUSinceLastKF.clear();

        pKFini->ComputeBoW();
        pKFcur->ComputeBoW();

        // Insert KFs in the map
        mpMap->AddKeyFrame(pKFini);
        mpMap->AddKeyFrame(pKFcur);

        // Create MapPoints and asscoiate to keyframes
        for (size_t i = 0; i < mvIniMatches.size(); i++) {
            if (mvIniMatches[i] < 0)
                continue;

            //Create MapPoint.
            Vector3f worldPos = mvIniP3D[i];

            MapPoint *pMP = new MapPoint(worldPos, pKFcur, mpMap);
            pKFini->AddMapPoint(pMP, i);
            pKFcur->AddMapPoint(pMP, mvIniMatches[i]);

            pMP->AddObservation(pKFini, i);
            pMP->AddObservation(pKFcur, mvIniMatches[i]);

            pMP->ComputeDistinctiveDescriptors();
            pMP->UpdateNormalAndDepth();

            //Fill Current Frame structure
            mCurrentFrame.mvpMapPoints[mvIniMatches[i]] = pMP;
            mCurrentFrame.mvbOutlier[mvIniMatches[i]] = false;

            //Add to Map
            mpMap->AddMapPoint(pMP);

        }

        // Update Connections
        pKFini->UpdateConnections();
        pKFcur->UpdateConnections();

        // Bundle Adjustment
        cout << "New Map created with " << mpMap->MapPointsInMap() << " points" << endl;

        Optimizer::GlobalBundleAdjustemnt(mpMap, 20);

        // Set median depth to 1
        float medianDepth = pKFini->ComputeSceneMedianDepth(2);
        float invMedianDepth = 1.0f / medianDepth;

        if (medianDepth < 0 || pKFcur->TrackedMapPoints(1) < 100) {
            cout << "Wrong initialization, reseting..." << endl;
            Reset();
            return;
        }

        // Scale initial baseline
        SE3f Tc2w = pKFcur->GetPose();
        Tc2w.translation() = Tc2w.translation() * invMedianDepth;
        // Tc2w.col(3).rowRange(0,3) = Tc2w.col(3).rowRange(0,3)*invMedianDepth;
        pKFcur->SetPose(Tc2w);

        // Scale points
        vector<MapPoint *> vpAllMapPoints = pKFini->GetMapPointMatches();
        for (size_t iMP = 0; iMP < vpAllMapPoints.size(); iMP++) {
            if (vpAllMapPoints[iMP]) {
                MapPoint *pMP = vpAllMapPoints[iMP];
                pMP->SetWorldPos(pMP->GetWorldPos() * invMedianDepth);
                pMP->UpdateNormalAndDepth();
            }
        }

        mpLocalMapper->InsertKeyFrame(pKFini);
        mpLocalMapper->InsertKeyFrame(pKFcur);

        mCurrentFrame.SetPose(pKFcur->GetPose());
        mnLastKeyFrameId = mCurrentFrame.mnId;
        mpLastKeyFrame = pKFcur;

        mvpLocalKeyFrames.push_back(pKFcur);
        mvpLocalKeyFrames.push_back(pKFini);
        mvpLocalMapPoints = mpMap->GetAllMapPoints();
        mpReferenceKF = pKFcur;
        mCurrentFrame.mpReferenceKF = pKFcur;

        mLastFrame = Frame(mCurrentFrame);

        mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

        mpMapDrawer->SetCurrentCameraPose(pKFcur->GetPose().cast<double>());

        mpMap->mvpKeyFrameOrigins.push_back(pKFini);

        mState = OK;
    }

/**
 * @brief 检查上一帧中的MapPoints是否被替换
 *
 * Local Mapping线程可能会将关键帧中某些MapPoints进行替换，由于tracking中需要用到mLastFrame，这里检查并更新上一帧中被替换的MapPoints
 * @see LocalMapping::SearchInNeighbors()
 */
    void Tracking::CheckReplacedInLastFrame() {
        for (int i = 0; i < mLastFrame.N; i++) {
            MapPoint *pMP = mLastFrame.mvpMapPoints[i];

            if (pMP) {
                MapPoint *pRep = pMP->GetReplaced();
                if (pRep) {
                    mLastFrame.mvpMapPoints[i] = pRep;
                }
            }
        }
    }

/**
 * @brief 对参考关键帧的MapPoints进行跟踪
 *
 * 1. 计算当前帧的词包，将当前帧的特征点分到特定层的nodes上
 * 2. 对属于同一node的描述子进行匹配
 * 3. 根据匹配对估计当前帧的姿态
 * 4. 根据姿态剔除误匹配
 * @return 如果匹配数大于10，返回true
 */
    bool Tracking::TrackReferenceKeyFrame() {
        // When tracking with reference keyframe, we need features and key points, direct method cannot work
        if (mCurrentFrame.mbFeatureExtracted == false) {
            mCurrentFrame.ExtractFeatures(); // 未提特征则加入新的特征
        } else {
            // 已经提过就算了
        }

        // We perform first an ORB matching with the reference keyframe
        // If enough matches are found we setup a PnP solver
        ORBmatcher matcher(0.7, false);
        vector<MapPoint *> vpMapPointMatches;
        int nmatches = matcher.SearchByBoW(mpReferenceKF, mCurrentFrame, vpMapPointMatches);

        if (nmatches < 15) {
            LOG(INFO) << "track reference frame failed, matches: " << nmatches << endl;
            return false;
        }

        mCurrentFrame.mvpMapPoints = vpMapPointMatches;
        mCurrentFrame.SetPose(mLastFrame.mTcw);

        Optimizer::PoseOptimization(&mCurrentFrame);
        if (mbUseIMU && mpLocalMapper->GetVINSInited()) {
            NavState ns = mCurrentFrame.GetNavState();
            SE3d Twb = (mpParams->GetSE3Tbc() * mCurrentFrame.mTcw.cast<double>()).inverse();
            ns.Set_Pos(Twb.translation());
            ns.Set_Rot(Twb.so3());
            mCurrentFrame.SetNavState(ns);
        }

        // Discard outliers
        int nmatchesMap = 0;
        for (int i = 0; i < mCurrentFrame.N; i++) {
            if (mCurrentFrame.mvpMapPoints[i]) {
                if (mCurrentFrame.mvbOutlier[i]) {
                    MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];

                    mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
                    mCurrentFrame.mvbOutlier[i] = false;
                    pMP->mbTrackInView = false;
                    pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                    nmatches--;
                } else if (mCurrentFrame.mvpMapPoints[i]->Observations() > 0)
                    nmatchesMap++;
            }
        }

        return nmatchesMap >= 10;
    }

/**
 * @brief 双目或rgbd摄像头根据深度值为上一帧产生新的MapPoints
 *
 * 在双目和rgbd情况下，选取一些深度小一些的点（可靠一些） \n
 * 可以通过深度值产生一些新的MapPoints
 */
    void Tracking::UpdateLastFrame() {
        // Update pose according to reference keyframe
        // cout<<"calling update last frame"<<endl;
        KeyFrame *pRef = mLastFrame.mpReferenceKF;
        SE3f Tlr = mlRelativeFramePoses.back();
        mLastFrame.SetPose(Tlr * pRef->GetPose());

        if (mbUseIMU && mpLocalMapper->GetVINSInited()) {
            // Update navstate: T_w_b = T_w_c * Tcb
            // Velocity should change direction according to rotation change, to make sure velocity in body frame keep unchanged.
            // Vb_pre = Vb_new: Rbw_pre*Vwb_pre = Rbw_new*Vwb_new

            SE3d Tclw = (Tlr * pRef->GetPose()).cast<double>();
            SE3d Twbl = (mpParams->GetSE3Tbc() * Tclw).inverse();
            NavState nsl = mLastFrame.GetNavState();
            SO3d Rwb_pre = nsl.Get_R();
            Vector3d Vwb_new = Twbl.so3() * Rwb_pre.inverse() * nsl.Get_V();
            nsl.Set_Pos(Twbl.translation());
            nsl.Set_Rot(Twbl.so3());
            nsl.Set_Vel(Vwb_new);
            mLastFrame.SetNavState(nsl);
        }

        if (mnLastKeyFrameId == mLastFrame.mnId || mSensor == System::MONOCULAR || !mbOnlyTracking)
            return;

        // Create "visual odometry" MapPoints
        // We sort points according to their measured depth by the stereo/RGB-D sensor
        vector<pair<float, int> > vDepthIdx;
        vDepthIdx.reserve(mLastFrame.N);
        for (int i = 0; i < mLastFrame.N; i++) {
            float z = mLastFrame.mvDepth[i];
            if (z > 0) {
                vDepthIdx.push_back(make_pair(z, i));
            }
        }

        if (vDepthIdx.empty())
            return;

        sort(vDepthIdx.begin(), vDepthIdx.end());

        // We insert all close points (depth<mThDepth)
        // If less than 100 close points, we insert the 100 closest ones.
        int nPoints = 0;
        for (size_t j = 0; j < vDepthIdx.size(); j++) {
            int i = vDepthIdx[j].second;

            bool bCreateNew = false;

            MapPoint *pMP = mLastFrame.mvpMapPoints[i];
            if (!pMP)
                bCreateNew = true;
            else if (pMP->Observations() < 1) {
                bCreateNew = true;
            }

            if (bCreateNew) {
                Vector3f x3D = mLastFrame.UnprojectStereo(i);
                MapPoint *pNewMP = new MapPoint(x3D, mpMap, &mLastFrame, i);

                mLastFrame.mvpMapPoints[i] = pNewMP;

                mlpTemporalPoints.push_back(pNewMP);
                nPoints++;
            } else {
                nPoints++;
            }

            if (vDepthIdx[j].first > mThDepth && nPoints > 100)
                break;
        }
    }


/**
 * @brief 根据匀速度模型对上一帧的MapPoints进行跟踪
 *
 * 1. 非单目情况，需要对上一帧产生一些新的MapPoints（临时）
 * 2. 将上一帧的MapPoints投影到当前帧的图像平面上，在投影的位置进行区域匹配
 * 3. 根据匹配对估计当前帧的姿态
 * 4. 根据姿态剔除误匹配
 * @return 如果匹配数大于10，返回true
 * @see V-B Initial Pose Estimation From Previous Frame
 *
 * @note this is not used in current version
 */
    bool Tracking::TrackWithMotionModel() {
        if (mCurrentFrame.mbFeatureExtracted == false)
            mCurrentFrame.ExtractFeatures();

        ORBmatcher matcher(0.9, true);

        // Update last frame pose according to its reference keyframe
        // Create "visual odometry" points if in Localization Mode
        UpdateLastFrame();
        mCurrentFrame.SetPose(mVelocity * mLastFrame.mTcw);

        fill(mCurrentFrame.mvpMapPoints.begin(), mCurrentFrame.mvpMapPoints.end(), static_cast<MapPoint *>(NULL));

        // Project points seen in previous frame
        int th;
        if (mSensor != System::STEREO)
            th = 15;
        else
            th = 7;
        int nmatches = matcher.SearchByProjection(mCurrentFrame, mLastFrame, th, mSensor == System::MONOCULAR);

        // If few matches, uses a wider window search
        if (nmatches < 20) {
            LOG(INFO) << "Search a wider window" << endl;
            fill(mCurrentFrame.mvpMapPoints.begin(), mCurrentFrame.mvpMapPoints.end(), static_cast<MapPoint *>(NULL));
            nmatches = matcher.SearchByProjection(mCurrentFrame, mLastFrame, 2 * th, mSensor == System::MONOCULAR);
        }

        if (nmatches < 20) {
            LOG(INFO) << "Match is not enough, failed." << endl;
            return false;
        }

        // Optimize frame pose with all matches
        Optimizer::PoseOptimization(&mCurrentFrame);

        // Discard outliers
        int nmatchesMap = 0;
        for (int i = 0; i < mCurrentFrame.N; i++) {
            if (mCurrentFrame.mvpMapPoints[i]) {
                if (mCurrentFrame.mvbOutlier[i]) {
                    MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];

                    mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
                    mCurrentFrame.mvbOutlier[i] = false;
                    pMP->mbTrackInView = false;
                    pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                    nmatches--;
                } else if (mCurrentFrame.mvpMapPoints[i]->Observations() > 0)
                    nmatchesMap++;
            }
        }

        if (mbOnlyTracking) {
            mbVO = nmatchesMap < 10;
            return nmatches > 20;
        }

        return nmatchesMap >= 10;
    }

    bool Tracking::TrackLocalMap() {
        // We have an estimation of the camera pose and some map points tracked in the frame.
        // We retrieve the local map and try to find matches to points in the local map.

        if (mvpLocalMapPoints.size() == 0)
            UpdateLocalMap();

        if (mCurrentFrame.mbFeatureExtracted == false) {
            mCurrentFrame.N = 0;
            mCurrentFrame.ExtractFeatures();
        }

        // 更新局部地图点和局部关键帧，关键帧由共视关系决定，地图点则由共视的关键帧生成
        SearchLocalPoints();

        // Optimize Pose
        Optimizer::PoseOptimization(&mCurrentFrame);
        mnMatchesInliers = 0;

        // Update MapPoints Statistics
        for (int i = 0; i < mCurrentFrame.N; i++) {
            if (mCurrentFrame.mvpMapPoints[i]) {
                if (!mCurrentFrame.mvbOutlier[i]) {
                    mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                    if (!mbOnlyTracking) {
                        if (mCurrentFrame.mvpMapPoints[i]->Observations() > 0)
                            mnMatchesInliers++;
                    } else
                        mnMatchesInliers++;
                } else if (mSensor == System::STEREO)
                    mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);

            }
        }

        UpdateLocalMap();

        // Decide if the tracking was succesful
        // More restrictive if there was a relocalization recently
        if (mCurrentFrame.mnId < mnLastRelocFrameId + mMaxFrames && mnMatchesInliers < 50) {
            return false;
        }

        if (mnMatchesInliers < 30) {
            LOG(WARNING) << "track local map failed, inliers: " << mnMatchesInliers << endl;
            return false;
        } else {
            return true;
        }
    }

    bool Tracking::TrackLocalMapWithIMU(bool bTrackLastKF) {
        // We have an estimation of the camera pose and some map points tracked in the frame.
        // We retrieve the local map and try to find matches to points in the local map.
        if (mvpLocalMapPoints.size() == 0)
            UpdateLocalMap();

        if (mCurrentFrame.mbFeatureExtracted == false) {
            mCurrentFrame.N = 0;
            mCurrentFrame.ExtractFeatures();
        }

        // 更新局部地图点和局部关键帧，关键帧由共视关系决定，地图点则由共视的关键帧生成
        SearchLocalPoints();

        // Optimize Pose, consider IMU variables here in PoseOptimization()
        //Optimizer::PoseOptimization(&mCurrentFrame);
        if (bTrackLastKF) {
            //fortest, to delete
            IMUPreintegrator imupreint = GetIMUPreIntSinceLastKF(&mCurrentFrame, mpLastKeyFrame, mvIMUSinceLastKF);
            if ((imupreint.getDeltaP() - mIMUPreIntInTrack.getDeltaP()).norm() > 1e-4)
                LOG(ERROR) << "preint dif P" << endl;
            if ((imupreint.getDeltaV() - mIMUPreIntInTrack.getDeltaV()).norm() > 1e-4)
                LOG(ERROR) << "preint dif V" << endl;
            if (Sophus::SO3(imupreint.getDeltaR().inverse() * mIMUPreIntInTrack.getDeltaR()).log().norm() > 1e-4)
                LOG(ERROR) << "preint dif R" << endl;
            if ((imupreint.getDeltaTime() - mIMUPreIntInTrack.getDeltaTime()) > 1e-4)
                LOG(ERROR) << "preint dif time" << endl;

            Optimizer::PoseOptimization(&mCurrentFrame, mpLastKeyFrame, mIMUPreIntInTrack,
                                        mpLocalMapper->GetGravityVec(), true);
        } else {
            //fortest, to delete
            IMUPreintegrator imupreint = GetIMUPreIntSinceLastFrame(&mCurrentFrame, &mLastFrame);
            if ((imupreint.getDeltaP() - mIMUPreIntInTrack.getDeltaP()).norm() > 1e-4)
                LOG(ERROR) << "preint dif P" << endl;
            if ((imupreint.getDeltaV() - mIMUPreIntInTrack.getDeltaV()).norm() > 1e-4)
                LOG(ERROR) << "preint dif V" << endl;
            if (Sophus::SO3(imupreint.getDeltaR().inverse() * mIMUPreIntInTrack.getDeltaR()).log().norm() > 1e-4)
                LOG(ERROR) << "preint dif R" << endl;
            if ((imupreint.getDeltaTime() - mIMUPreIntInTrack.getDeltaTime()) > 1e-4)
                LOG(ERROR) << "preint dif time" << endl;

            Optimizer::PoseOptimization(&mCurrentFrame, &mLastFrame, mIMUPreIntInTrack, mpLocalMapper->GetGravityVec(),
                                        true);
        }

        mnMatchesInliers = 0;

        // Update MapPoints Statistics
        for (int i = 0; i < mCurrentFrame.N; i++) {
            if (mCurrentFrame.mvpMapPoints[i]) {
                if (!mCurrentFrame.mvbOutlier[i]) {
                    mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                    if (!mbOnlyTracking) {
                        if (mCurrentFrame.mvpMapPoints[i]->Observations() > 0)
                            mnMatchesInliers++;
                    } else
                        mnMatchesInliers++;
                } else if (mSensor == System::STEREO)
                    mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);

            }
        }


        UpdateLocalMap();

        // Decide if the tracking was succesful
        // More restrictive if there was a relocalization recently
        if (mCurrentFrame.mnId < mnLastRelocFrameId + mMaxFrames && mnMatchesInliers < 50)
            return false;

        if (mnMatchesInliers < 30)
            return false;
        else
            return true;
    }

    bool Tracking::TrackLocalMapDirectWithIMU(bool bTrackLastKF) {
        // project the local map points into current frame
        // 这步把 local map points 投影至当前帧并确定其位置
        if (mCurrentFrame.mbFeatureExtracted == true)  // 此帧已经提了特征，用特征点法的local mapping来处理
        {
            return TrackLocalMapWithIMU(bTrackLastKF);
        }

        SearchLocalPointsDirect();
        UpdateLocalKeyFrames();

        // 优化当前帧的位姿，确定 inlier
        // Optimize pose
        if (bTrackLastKF) {
            Optimizer::PoseOptimization(&mCurrentFrame, mpLastKeyFrame, mIMUPreIntInTrack,
                                        mpLocalMapper->GetGravityVec(), true);
        } else {
            Optimizer::PoseOptimization(&mCurrentFrame, &mLastFrame, mIMUPreIntInTrack, mpLocalMapper->GetGravityVec(),
                                        true);
        }

        mnMatchesInliers = 0;

        // 更新地图点的统计信息
        for (int i = 0; i < mCurrentFrame.N; i++) {
            if (mCurrentFrame.mvpMapPoints[i]) {
                if (!mCurrentFrame.mvbOutlier[i]) {
                    // inlier
                    mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                    if (!mbOnlyTracking) {
                        if (mCurrentFrame.mvpMapPoints[i]->Observations() > 0)
                            mnMatchesInliers++;
                    } else
                        mnMatchesInliers++;
                } else if (mSensor == System::STEREO) {
                    mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
                } else {
                    // 这个投影是outlier，从cache里移除
                    auto iter = mvpDirectMapPointsCache.find(mCurrentFrame.mvpMapPoints[i]);
                    if (iter != mvpDirectMapPointsCache.end())
                        mvpDirectMapPointsCache.erase(iter);
                }
            }
        }

        // always return true in VIO because pose can be constrained by IMU
        if (mCurrentFrame.mnId < mnLastRelocFrameId + mMaxFrames && mnMatchesInliers < 50) {
            LOG(INFO) << "Track local map direct with imu failed, inliers: " << mnMatchesInliers << endl;
            return false;
        }

        if (mnMatchesInliers < 30) {
            LOG(INFO) << "Track local map direct with imu failed, inliers: " << mnMatchesInliers << endl;
            return false;
        } else {
            return true;
        }
    }


    bool Tracking::NeedNewKeyFrame() {
        // 前面是排除一些不需要添加关键帧的检查
        // 步骤1：如果用户在界面上选择重定位，那么将不插入关键帧
        // 由于插入关键帧过程中会生成MapPoint，因此用户选择重定位后地图上的点云和关键帧都不会再增加
        if (mbOnlyTracking)
            return false;

        // While updating initial poses
        if (mpLocalMapper->GetUpdatingInitPoses()) {
            LOG(WARNING) << "mpLocalMapper->GetUpdatingInitPoses, no new KF" << endl;
            return false;
        }

        // If Local Mapping is freezed by a Loop Closure do not insert keyframes
        // 如果局部地图被闭环检测使用，则不插入关键帧
        if (mpLocalMapper->isStopped() || mpLocalMapper->stopRequested())
            return false;

        const int nKFs = mpMap->KeyFramesInMap();

        // Do not insert keyframes if not enough frames have passed from last relocalisation
        // 步骤2：判断是否距离上一次插入关键帧的时间太短
        // mCurrentFrame.mnId是当前帧的ID
        // mnLastRelocFrameId是最近一次重定位帧的ID
        // mMaxFrames等于图像输入的帧率
        // 如果关键帧比较少，则考虑插入关键帧
        if (mCurrentFrame.mnId < mnLastRelocFrameId + mMaxFrames && nKFs > mMaxFrames)
            return false;

        // Tracked MapPoints in the reference keyframe
        // 步骤3：得到参考关键帧跟踪到的MapPoints数量
        // 在 UpdateLocalKeyFrames 函数中会将与当前关键帧共视程度最高的关键帧设定为当前帧的参考关键帧
        int nMinObs = 3;
        if (nKFs <= 2)
            nMinObs = 2;
        int nRefMatches = mpReferenceKF->TrackedMapPoints(nMinObs);

        // Local Mapping accept keyframes?
        // 步骤4：查询局部地图管理器是否繁忙
        bool bLocalMappingIdle = mpLocalMapper->AcceptKeyFrames();

        // Check how many "close" points are being tracked and how many could be potentially created.
        // Stereo & RGB-D: Ratio of close "matches to map"/"total matches"
        // "total matches = matches to map + visual odometry matches"
        // Visual odometry matches will become MapPoints if we insert a keyframe.
        // This ratio measures how many MapPoints we could create if we insert a keyframe.
        // 步骤5：对于双目或RGBD摄像头，统计总的可以添加的MapPoints数量和跟踪到地图中的MapPoints数量
        int nNonTrackedClose = 0;
        int nTrackedClose = 0;
        if (mSensor != System::MONOCULAR) {
            for (int i = 0; i < mCurrentFrame.N; i++) {
                if (mCurrentFrame.mvDepth[i] > 0 && mCurrentFrame.mvDepth[i] < mThDepth) {
                    if (mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
                        nTrackedClose++;
                    else
                        nNonTrackedClose++;
                }
            }
        }

        bool bNeedToInsertClose = 0;        // 注意直接法中，nonTrackedClose将接近零（只有一些在pose optimization中outlier才会统计），因此单目need to insert close将一直为零
        bNeedToInsertClose = (nTrackedClose < 100) && (nNonTrackedClose > 70);

        // Thresholds
        // 步骤6：决策是否需要插入关键帧
        // Thresholds
        // 设定inlier阈值，和之前帧特征点匹配的inlier比例
        float thRefRatio = 0.75f;
        if (nKFs < 2)
            thRefRatio = 0.4f;

        if (mSensor == System::MONOCULAR)
            thRefRatio = 0.9f;

        double timegap = 0.5, largetimegap = 3.0;


        // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
        // 很长时间没有插入关键帧
        //const bool c1a = mCurrentFrame.mnId>=mnLastKeyFrameId+mMaxFrames;
        const bool c1a = (mCurrentFrame.mTimeStamp - mpLastKeyFrame->mTimeStamp >= largetimegap);
        // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
        // 一定时间没有插入关键帧 且 localMapper处于空闲状态
        const bool c1b = (mCurrentFrame.mnId >= mnLastKeyFrameId + mMinFrames && bLocalMappingIdle);
        // Condition 1c: tracking is weak
        // 跟踪快跪了，或可以添加许多新特征点，0.25和0.3是一个比较低的阈值
        const bool c1c = (mnMatchesInliers < 50) || bNeedToInsertClose;

        // Condition 2: Few tracked points compared to reference keyframe. Lots of visual odometry compared to map matches.
        // 阈值比c1c要高，与最近的一个关键帧相比，追踪的点数较少，但仍能保持追踪
        const bool c2 = ((mnMatchesInliers < nRefMatches * thRefRatio || bNeedToInsertClose) && mnMatchesInliers > 15);

        // 有IMU的情况下，一定时间间隔下加入新关键帧，防止bias漂走
        const bool cTimeGap =
                mbUseIMU && ((mCurrentFrame.mTimeStamp - mpLastKeyFrame->mTimeStamp) >= timegap) && bLocalMappingIdle;

        // 限制运动的阈值，最好是发生运动之后再插入关键帧?
        // const bool c3 = (mpReferenceKF->GetPose().inverse()*mCurrentFrame.mTcw).log().norm()>0.1 ;

        if (((c1a || c1b || c1c) && c2) || cTimeGap) {
            // c1中任意一个成立且c2成立
            // If the mapping accepts keyframes, insert keyframe.
            // Otherwise send a signal to interrupt BA
            if (bLocalMappingIdle) {
                return true;
            } else {
                // 打断local BA并插入新的点
                mpLocalMapper->InterruptBA();
                // if(mSensor!=System::MONOCULAR)   // why != monocular?
                {
                    // 队列里不能阻塞太多关键帧
                    // tracking插入关键帧不是直接插入，而且先插入到mlNewKeyFrames中，
                    // 然后localmapper再逐个pop出来插入到mspKeyFrames
                    if (mpLocalMapper->KeyframesInQueue() < 3)
                        return true;
                    else
                        return false;
                }
                // else
                // return false;
            }
        } else
            return false;
    }

    void Tracking::CreateNewKeyFrame() {

        if (!mpLocalMapper->SetNotStop(true))
            return;

        // if we have not extracted features, we do feature extraction here
        if (mCurrentFrame.mbFeatureExtracted == false) {
            // this key frame is generated with direct tracking
            mCurrentFrame.ExtractFeatures();
            // update the map point cache ?
        } else {
            // this is generated with feature matching, which may occur in monocular initialization or track with reference keyframe
        }

        // 根据此帧生成关键帧
        KeyFrame *pKF = nullptr;
        if (mbUseIMU) {
            pKF = new KeyFrame(mCurrentFrame, mpMap, mpKeyFrameDB, mvIMUSinceLastKF, mpLastKeyFrame);
            // Set initial NavState for KeyFrame
            pKF->SetInitialNavStateAndBias(mCurrentFrame.GetNavState());
            // Compute pre-integrator
            pKF->ComputePreInt();
            // Clear IMUData buffer
            mvIMUSinceLastKF.clear();
        } else {
            pKF = new KeyFrame(mCurrentFrame, mpMap, mpKeyFrameDB);
        }

        mpReferenceKF = pKF;
        mCurrentFrame.mpReferenceKF = pKF;

        if (mSensor != System::MONOCULAR) {
            // in RGBD and stereo mode, we can generate new feature with only one key-frame
            mCurrentFrame.UpdatePoseMatrices();
            // We sort points by the measured depth by the stereo/RGBD sensor.
            // We create all those MapPoints whose depth < mThDepth.
            // If there are less than 100 close points we create the 100 closest.
            vector<pair<float, int> > vDepthIdx;
            vDepthIdx.reserve(mCurrentFrame.N);
            for (int i = 0; i < mCurrentFrame.N; i++) {
                float z = mCurrentFrame.mvDepth[i];
                if (z > 0) {
                    vDepthIdx.push_back(make_pair(z, i));
                }
            }

            if (!vDepthIdx.empty()) {
                sort(vDepthIdx.begin(), vDepthIdx.end());

                int nPoints = 0;
                for (size_t j = 0; j < vDepthIdx.size(); j++) {
                    int i = vDepthIdx[j].second;

                    bool bCreateNew = false;

                    MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];

                    // 这个点之前只有一次观测或没有看到过，但是有合法的depth
                    if (!pMP)
                        bCreateNew = true;
                    else if (pMP->Observations() < 1) {
                        bCreateNew = true;
                        mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
                    }

                    if (bCreateNew) {
                        Vector3f x3D = mCurrentFrame.UnprojectStereo(i);
                        MapPoint *pNewMP = new MapPoint(x3D, pKF, mpMap);
                        pNewMP->AddObservation(pKF, i);
                        pKF->AddMapPoint(pNewMP, i);
                        pNewMP->ComputeDistinctiveDescriptors();
                        pNewMP->UpdateNormalAndDepth();
                        mpMap->AddMapPoint(pNewMP);
                        mvpDirectMapPointsCache.insert(pNewMP);
                        mCurrentFrame.mvpMapPoints[i] = pNewMP;
                        nPoints++;
                    } else {
                        nPoints++;
                    }

                    if (vDepthIdx[j].first > mThDepth)
                        break;
                }
            }
        }

        mpLocalMapper->InsertKeyFrame(pKF);
        mpLocalMapper->SetNotStop(false);

        mnLastKeyFrameId = mCurrentFrame.mnId;
        mpLastKeyFrame = pKF;

        if (mpMap->GetAllKeyFrames().size() > 10)
            mMinFrames = 10;
    }

/**
 * @brief 对Local MapPoints进行跟踪
 *
 * 在局部地图中查找在当前帧视野范围内的点，将视野范围内的点和当前帧的特征点进行投影匹配
 */
    void Tracking::SearchLocalPoints() {
        // Do not search map points already matched
        for (vector<MapPoint *>::iterator vit = mCurrentFrame.mvpMapPoints.begin(), vend = mCurrentFrame.mvpMapPoints.end();
             vit != vend; vit++) {
            MapPoint *pMP = *vit;
            if (pMP) {
                if (pMP->isBad()) {
                    *vit = static_cast<MapPoint *>(NULL);
                } else {
                    pMP->IncreaseVisible();
                    pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                    pMP->mbTrackInView = false;
                }
            }
        }

        int nToMatch = 0;

        // Project points in frame and check its visibility
        for (vector<MapPoint *>::iterator vit = mvpLocalMapPoints.begin(), vend = mvpLocalMapPoints.end();
             vit != vend; vit++) {
            MapPoint *pMP = *vit;
            if (pMP->mnLastFrameSeen == mCurrentFrame.mnId)
                continue;
            if (pMP->isBad())
                continue;
            // Project (this fills MapPoint variables for matching)
            if (mCurrentFrame.isInFrustum(pMP, 0.5)) {
                pMP->IncreaseVisible();
                nToMatch++;
            }
        }

        if (nToMatch > 0) {
            ORBmatcher matcher(0.8);
            bool checkLevel = true;
            int th = 1;
            if (mSensor == System::RGBD)
                th = 3;
            // If the camera has been relocalised recently, perform a coarser search
            if (mCurrentFrame.mnId < mnLastRelocFrameId + 2)
                th = 5;
            if (mbDirectFailed) { // 直接法失败时，位姿不可靠，也使用较大的窗口来搜
                checkLevel = false;
                th = 5;
            }
            int cnt = matcher.SearchByProjection(mCurrentFrame, mvpLocalMapPoints, th, false );
            LOG(INFO) << "Search by projection returns " << cnt << endl;
        }
    }

/**
 * @brief 更新LocalMap
 *
 * 局部地图包括： \n
 * - K1个关键帧、K2个临近关键帧和参考关键帧
 * - 由这些关键帧观测到的MapPoints
 */
    void Tracking::UpdateLocalMap() {
        // This is for visualization
        mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

        // Update
        UpdateLocalKeyFrames();
        UpdateLocalPoints();
    }

/**
 * @brief 更新局部关键帧，called by UpdateLocalMap()
 *
 * 遍历当前帧的MapPoints，将观测到这些MapPoints的关键帧和相邻的关键帧取出，更新mvpLocalKeyFrames
 */
    void Tracking::UpdateLocalPoints() {
        mvpLocalMapPoints.clear();

        for (vector<KeyFrame *>::const_iterator itKF = mvpLocalKeyFrames.begin(), itEndKF = mvpLocalKeyFrames.end();
             itKF != itEndKF; itKF++) {
            KeyFrame *pKF = *itKF;
            const vector<MapPoint *> vpMPs = pKF->GetMapPointMatches();

            for (vector<MapPoint *>::const_iterator itMP = vpMPs.begin(), itEndMP = vpMPs.end();
                 itMP != itEndMP; itMP++) {
                MapPoint *pMP = *itMP;
                if (!pMP)
                    continue;
                if (pMP->mnTrackReferenceForFrame == mCurrentFrame.mnId)
                    continue;
                if (!pMP->isBad()) {
                    mvpLocalMapPoints.push_back(pMP);
                    pMP->mnTrackReferenceForFrame = mCurrentFrame.mnId;
                }
            }
        }

        // 从last keyframe中也取一份
        for (MapPoint *mp: mpLastKeyFrame->GetMapPointMatches()) {
            if (mp == nullptr || mp->isBad() || mp->mnTrackReferenceForFrame == mCurrentFrame.mnId)
                continue;
            mvpLocalMapPoints.push_back(mp);
        }
    }


    void Tracking::UpdateLocalKeyFrames() {
        // Each map point vote for the keyframes in which it has been observed
        map<KeyFrame *, int> keyframeCounter;
        for (int i = 0; i < mCurrentFrame.N; i++) {
            if (mCurrentFrame.mvpMapPoints[i]) {
                MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];
                if (!pMP->isBad()) {
                    const map<KeyFrame *, size_t> observations = pMP->GetObservations();
                    for (map<KeyFrame *, size_t>::const_iterator it = observations.begin(), itend = observations.end();
                         it != itend; it++)
                        keyframeCounter[it->first]++;
                } else {
                    mCurrentFrame.mvpMapPoints[i] = NULL;
                }
            }
        }

        if (keyframeCounter.empty())
            return;

        int max = 0;
        KeyFrame *pKFmax = static_cast<KeyFrame *>(NULL);

        mvpLocalKeyFrames.clear();
        mvpLocalKeyFrames.reserve(3 * keyframeCounter.size());

        // All keyframes that observe a map point are included in the local map. Also check which keyframe shares most points
        for (map<KeyFrame *, int>::const_iterator it = keyframeCounter.begin(), itEnd = keyframeCounter.end();
             it != itEnd; it++) {
            KeyFrame *pKF = it->first;

            if (pKF->isBad())
                continue;

            if (it->second > max) {
                max = it->second;
                pKFmax = pKF;
            }

            mvpLocalKeyFrames.push_back(it->first);
            pKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
        }


        // Include also some not-already-included keyframes that are neighbors to already-included keyframes
        for (vector<KeyFrame *>::const_iterator itKF = mvpLocalKeyFrames.begin(), itEndKF = mvpLocalKeyFrames.end();
             itKF != itEndKF; itKF++) {
            // Limit the number of keyframes
            if (mvpLocalKeyFrames.size() > 80)
                break;

            KeyFrame *pKF = *itKF;

            const vector<KeyFrame *> vNeighs = pKF->GetBestCovisibilityKeyFrames(10);

            for (vector<KeyFrame *>::const_iterator itNeighKF = vNeighs.begin(), itEndNeighKF = vNeighs.end();
                 itNeighKF != itEndNeighKF; itNeighKF++) {
                KeyFrame *pNeighKF = *itNeighKF;
                if (!pNeighKF->isBad()) {
                    if (pNeighKF->mnTrackReferenceForFrame != mCurrentFrame.mnId) {
                        mvpLocalKeyFrames.push_back(pNeighKF);
                        pNeighKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
                        break;
                    }
                }
            }

            const set<KeyFrame *> spChilds = pKF->GetChilds();
            for (set<KeyFrame *>::const_iterator sit = spChilds.begin(), send = spChilds.end(); sit != send; sit++) {
                KeyFrame *pChildKF = *sit;
                if (!pChildKF->isBad()) {
                    if (pChildKF->mnTrackReferenceForFrame != mCurrentFrame.mnId) {
                        mvpLocalKeyFrames.push_back(pChildKF);
                        pChildKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
                        break;
                    }
                }
            }

            KeyFrame *pParent = pKF->GetParent();
            if (pParent) {
                if (pParent->mnTrackReferenceForFrame != mCurrentFrame.mnId) {
                    mvpLocalKeyFrames.push_back(pParent);
                    pParent->mnTrackReferenceForFrame = mCurrentFrame.mnId;
                    break;
                }
            }
        }

        if (pKFmax) {
            mpReferenceKF = pKFmax;
            mCurrentFrame.mpReferenceKF = mpReferenceKF;
        }
    }

    bool Tracking::Relocalization() {
        // in relocalization we need to extract the features, direct method cannot help us
        if (mCurrentFrame.mbFeatureExtracted == false) {
            mCurrentFrame.ExtractFeatures();
        }

        // Compute Bag of Words Vector
        // mCurrentFrame.ComputeBoW();

        // Relocalization is performed when tracking is lost
        // Track Lost: Query KeyFrame Database for keyframe candidates for relocalisation
        vector<KeyFrame *> vpCandidateKFs = mpKeyFrameDB->DetectRelocalizationCandidates(&mCurrentFrame);

        if (vpCandidateKFs.empty()) {
            return false;
        }

        const int nKFs = vpCandidateKFs.size();

        // We perform first an ORB matching with each candidate
        // If enough matches are found we setup a PnP solver
        ORBmatcher matcher(0.75, true);
        // ORBmatcher matcher(0.9,false);

        vector<PnPsolver *> vpPnPsolvers;
        vpPnPsolvers.resize(nKFs);

        vector<vector<MapPoint *> > vvpMapPointMatches;
        vvpMapPointMatches.resize(nKFs);

        vector<bool> vbDiscarded;
        vbDiscarded.resize(nKFs);

        int nCandidates = 0;

        for (int i = 0; i < nKFs; i++) {
            KeyFrame *pKF = vpCandidateKFs[i];
            if (pKF->isBad())
                vbDiscarded[i] = true;
            else {
                int nmatches = matcher.SearchByBoW(pKF, mCurrentFrame, vvpMapPointMatches[i]);
                if (nmatches < 10) {
                    vbDiscarded[i] = true;
                    continue;
                } else {
                    PnPsolver *pSolver = new PnPsolver(mCurrentFrame, vvpMapPointMatches[i]);
                    pSolver->SetRansacParameters(0.99, 10, 300, 4, 0.5, 5.991);
                    vpPnPsolvers[i] = pSolver;
                    nCandidates++;
                }
            }
        }

        // Alternatively perform some iterations of P4P RANSAC
        // Until we found a camera pose supported by enough inliers
        bool bMatch = false;
        ORBmatcher matcher2(0.9, true);

        while (nCandidates > 0 && !bMatch) {
            for (int i = 0; i < nKFs; i++) {
                if (vbDiscarded[i])
                    continue;

                // Perform 5 Ransac Iterations
                vector<bool> vbInliers;
                int nInliers;
                bool bNoMore;

                PnPsolver *pSolver = vpPnPsolvers[i];
                cv::Mat Tcw = pSolver->iterate(5, bNoMore, vbInliers, nInliers);

                // If Ransac reachs max. iterations discard keyframe
                if (bNoMore) {
                    vbDiscarded[i] = true;
                    nCandidates--;
                }

                // If a Camera Pose is computed, optimize
                if (!Tcw.empty()) {
                    auto tmp = Converter::toSE3Quat(Tcw);
                    mCurrentFrame.mTcw = SE3d(tmp.rotation(), tmp.translation()).cast<float>();
                    mCurrentFrame.mbPoseSet = true;

                    set<MapPoint *> sFound;

                    const int np = vbInliers.size();

                    for (int j = 0; j < np; j++) {
                        if (vbInliers[j]) {
                            mCurrentFrame.mvpMapPoints[j] = vvpMapPointMatches[i][j];
                            sFound.insert(vvpMapPointMatches[i][j]);
                        } else
                            mCurrentFrame.mvpMapPoints[j] = NULL;
                    }

                    int nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                    if (nGood < 10) {
                        continue;
                    }

                    for (int io = 0; io < mCurrentFrame.N; io++)
                        if (mCurrentFrame.mvbOutlier[io])
                            mCurrentFrame.mvpMapPoints[io] = static_cast<MapPoint *>(NULL);

                    // If few inliers, search by projection in a coarse window and optimize again
                    if (nGood < 50) {
                        int nadditional = matcher2.SearchByProjection(mCurrentFrame, vpCandidateKFs[i], sFound, 10,
                                                                      100);

                        if (nadditional + nGood >= 50) {
                            nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                            // If many inliers but still not enough, search by projection again in a narrower window
                            // the camera has been already optimized with many points
                            if (nGood > 30 && nGood < 50) {
                                sFound.clear();
                                for (int ip = 0; ip < mCurrentFrame.N; ip++)
                                    if (mCurrentFrame.mvpMapPoints[ip])
                                        sFound.insert(mCurrentFrame.mvpMapPoints[ip]);
                                nadditional = matcher2.SearchByProjection(mCurrentFrame, vpCandidateKFs[i], sFound, 3,
                                                                          64);

                                // Final optimization
                                if (nGood + nadditional >= 50) {
                                    nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                                    for (int io = 0; io < mCurrentFrame.N; io++)
                                        if (mCurrentFrame.mvbOutlier[io])
                                            mCurrentFrame.mvpMapPoints[io] = NULL;
                                }
                            }
                        }
                    }


                    // If the pose is supported by enough inliers stop ransacs and continue
                    if (nGood >= 50) {
                        bMatch = true;
                        break;
                    }
                }
            }
        }

        if (!bMatch) {
            return false;
        } else {
            mnLastRelocFrameId = mCurrentFrame.mnId;
            return true;
        }

    }

    void Tracking::Reset() {
        cout << "System Reseting" << endl;
        if (mpViewer) {
            mpViewer->RequestStop();
            while (!mpViewer->isStopped())
                usleep(3000);
        }

        // Reset Local Mapping
        cout << "Reseting Local Mapper...";
        mpLocalMapper->RequestReset();
        cout << " done" << endl;

        // Reset Loop Closing
        cout << "Reseting Loop Closing...";
        mpLoopClosing->RequestReset();
        cout << " done" << endl;

        // Clear BoW Database
        cout << "Reseting Database...";
        mpKeyFrameDB->clear();
        cout << " done" << endl;

        // Clear Map (this erase MapPoints and KeyFrames)
        mpMap->clear();

        mvpLocalKeyFrames.clear();
        mvpLocalMapPoints.clear();
        mvpDirectMapPointsCache.clear();

        KeyFrame::nNextId = 0;
        Frame::nNextId = 0;
        mState = NO_IMAGES_YET;

        if (mpInitializer) {
            delete mpInitializer;
            mpInitializer = static_cast<Initializer *>(NULL);
        }

        mlRelativeFramePoses.clear();
        mlpReferences.clear();
        mlFrameTimes.clear();
        mlbLost.clear();

        if (mpViewer)
            mpViewer->Release();

        mMinFrames = 0;
    }

    void Tracking::ChangeCalibration(const string &strSettingPath) {
        cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
        float fx = fSettings["Camera.fx"];
        float fy = fSettings["Camera.fy"];
        float cx = fSettings["Camera.cx"];
        float cy = fSettings["Camera.cy"];

        mK = Matrix3f::Identity();
        mK(0, 0) = fx;
        mK(1, 1) = fy;
        mK(0, 2) = cx;
        mK(1, 2) = cy;

        int bUseDistK6 = fSettings["Camera.bUseDistK6"];
        if (bUseDistK6 == 1) {
            cv::Mat DistCoef(8, 1, CV_32F);
            DistCoef.at<float>(0) = fSettings["Camera.k1"];
            DistCoef.at<float>(1) = fSettings["Camera.k2"];
            DistCoef.at<float>(2) = fSettings["Camera.p1"];
            DistCoef.at<float>(3) = fSettings["Camera.p2"];
            DistCoef.at<float>(4) = fSettings["Camera.k3"];
            DistCoef.at<float>(5) = fSettings["Camera.k4"];
            DistCoef.at<float>(6) = fSettings["Camera.k5"];
            DistCoef.at<float>(7) = fSettings["Camera.k6"];
            DistCoef.copyTo(mDistCoef);
        } else {
            cv::Mat DistCoef(4, 1, CV_32F);
            DistCoef.at<float>(0) = fSettings["Camera.k1"];
            DistCoef.at<float>(1) = fSettings["Camera.k2"];
            DistCoef.at<float>(2) = fSettings["Camera.p1"];
            DistCoef.at<float>(3) = fSettings["Camera.p2"];
            const float k3 = fSettings["Camera.k3"];
            if (k3 != 0) {
                DistCoef.resize(5);
                DistCoef.at<float>(4) = k3;
            }
            DistCoef.copyTo(mDistCoef);
        }

        mbf = fSettings["Camera.bf"];

        Frame::mbInitialComputations = true;
    }

    void Tracking::InformOnlyTracking(const bool &flag) {
        mbOnlyTracking = flag;
    }

    void Tracking::PredictNavStateByIMU(bool bTrackLastKF) {
        // Map updated, optimize with last KeyFrame
        if (bTrackLastKF) {
            // Compute IMU Pre-integration
            mIMUPreIntInTrack = GetIMUPreIntSinceLastKF(&mCurrentFrame, mpLastKeyFrame, mvIMUSinceLastKF);

            // Get initial NavState&pose from Last KeyFrame
            mCurrentFrame.SetInitialNavStateAndBias(mpLastKeyFrame->GetNavState());
            mCurrentFrame.UpdateNavState(mIMUPreIntInTrack, mpLocalMapper->GetGravityVec());
            mCurrentFrame.UpdatePoseFromNS(ConfigParam::GetSE3Tbc());
        }
            // Map not updated, optimize with last Frame
        else {
            // Compute IMU Pre-integration
            mIMUPreIntInTrack = GetIMUPreIntSinceLastFrame(&mCurrentFrame, &mLastFrame);

            // Get initial pose from Last Frame
            mCurrentFrame.SetInitialNavStateAndBias(mLastFrame.GetNavState());
            mCurrentFrame.UpdateNavState(mIMUPreIntInTrack, mpLocalMapper->GetGravityVec());
            mCurrentFrame.UpdatePoseFromNS(ConfigParam::GetSE3Tbc());
        }
    }

    IMUPreintegrator
    Tracking::GetIMUPreIntSinceLastKF(Frame *pCurF, KeyFrame *pLastKF, const std::vector<IMUData> &vIMUSInceLastKF) {
        // Reset pre-integrator first
        IMUPreintegrator IMUPreInt;
        IMUPreInt.reset();

        Vector3d bg = pLastKF->GetNavState().Get_BiasGyr();
        Vector3d ba = pLastKF->GetNavState().Get_BiasAcc();

        // remember to consider the gap between the last KF and the first IMU
        {
            const IMUData &imu = vIMUSInceLastKF.front();
            double dt = std::max(0.0d, imu._t - pLastKF->mTimeStamp);
            IMUPreInt.update(imu._g - bg, imu._a - ba, dt);
        }
        // integrate each imu
        for (size_t i = 0; i < vIMUSInceLastKF.size(); i++) {
            const IMUData &imu = vIMUSInceLastKF[i];
            double nextt;
            if (i == vIMUSInceLastKF.size() - 1)
                nextt = pCurF->mTimeStamp;         // last IMU, next is this KeyFrame
            else
                nextt = vIMUSInceLastKF[i + 1]._t;  // regular condition, next is imu data

            // delta time
            double dt = std::max(0.0, nextt - imu._t);
            // update pre-integrator
            IMUPreInt.update(imu._g - bg, imu._a - ba, dt);
        }

        return IMUPreInt;
    }

    IMUPreintegrator Tracking::GetIMUPreIntSinceLastFrame(Frame *pCurF, Frame *pLastF) {
        // Reset pre-integrator first
        IMUPreintegrator IMUPreInt;
        IMUPreInt.reset();

        pCurF->ComputeIMUPreIntSinceLastFrame(pLastF, IMUPreInt);

        return IMUPreInt;
    }


    bool Tracking::TrackWithSparseAlignment(bool bTrackLastKF) {
        // 更新上一帧的信息（因为可能被其他线程修改）
        UpdateLastFrame();

        if (!mbUseIMU || !mpLocalMapper->GetVINSInited()) {
            // 不使用IMU时，利用速度模型估计当前帧位姿
            mCurrentFrame.SetPose(mVelocity * mLastFrame.mTcw);
        } else {
            // 使用IMU计算初始位姿
            // predict pose with IMU
            PredictNavStateByIMU(bTrackLastKF);
        }

        // check if last frame have enough observations
        size_t inliers_in_last_frame = 0;
        for (int i = 0; i < mLastFrame.N; i++)
            if (mLastFrame.mvpMapPoints[i] && mLastFrame.mvpMapPoints[i]->isBad() == false &&
                mLastFrame.mvbOutlier[i] == false)
                inliers_in_last_frame++;
        if (inliers_in_last_frame < 30) {
            LOG(WARNING) << "Last frame have less observations: " << inliers_in_last_frame
                         << ", sparse alignment may have a erroneous result, return back to feature method." << endl;
            return false;
        }

        SE3f TCR;
        size_t ret = mpAlign->run(&mLastFrame, &mCurrentFrame, TCR);

        if (ret == false) {
            LOG(INFO) << "Failed. return back to feature methods" << endl;
            mCurrentFrame.SetPose(mVelocity * mLastFrame.mTcw);
            return false;
        }

        mCurrentFrame.SetPose(TCR * mLastFrame.mTcw);
        if (mbUseIMU && mpLocalMapper->GetVINSInited()) {
            // Set the navigation state
            NavState ns = mCurrentFrame.GetNavState();
            SE3d Twb = (mpParams->GetSE3Tbc() * mCurrentFrame.mTcw.cast<double>()).inverse();
            ns.Set_Pos(Twb.translation());
            ns.Set_Rot(Twb.so3());
            mCurrentFrame.SetNavState(ns);
        }
        return true;
    }

    bool Tracking::TrackLocalMapDirect() {
        // project the local map points into current frame, then search with direct align
        // 这步把 local map points 投影至当前帧并确定其位置
        if (mCurrentFrame.mbFeatureExtracted == true)  // 此帧已经提了特征，用特征点法的local mapping来处理
        {
            // if we have extracted features, do it with feature matching
            LOG(INFO) << "This frame already have features, using Track Local Map instead." << endl;
            return TrackLocalMap();
        }

        SearchLocalPointsDirect();
        UpdateLocalKeyFrames();

        // compute the stereo key point and RGBD key point, pose optimizer will use that information
        if (mSensor == System::RGBD)
            mCurrentFrame.ComputeStereoFromRGBD(mCurrentFrame.mImDepth);
        else if (mSensor == System::STEREO)
            mCurrentFrame.ComputeStereoMatches();

        // 优化当前帧的位姿，确定 inlier
        // Optimize pose
        Optimizer::PoseOptimization(&mCurrentFrame);
        mnMatchesInliers = 0;

        // 更新地图点的统计信息
        for (int i = 0; i < mCurrentFrame.N; i++) {
            if (mCurrentFrame.mvpMapPoints[i]) {
                if (!mCurrentFrame.mvbOutlier[i]) {
                    // inlier
                    mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                    if (!mbOnlyTracking) {
                        if (mCurrentFrame.mvpMapPoints[i]->Observations() > 0)
                            mnMatchesInliers++;
                    } else
                        mnMatchesInliers++;
                } else if (mSensor == System::STEREO) {
                    mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
                    // 这个投影是outlier，从cache里移除
                    auto iter = mvpDirectMapPointsCache.find(mCurrentFrame.mvpMapPoints[i]);
                    if (iter != mvpDirectMapPointsCache.end())
                        mvpDirectMapPointsCache.erase(iter);
                } else {
                    // 这个投影是outlier，从cache里移除
                    auto iter = mvpDirectMapPointsCache.find(mCurrentFrame.mvpMapPoints[i]);
                    if (iter != mvpDirectMapPointsCache.end())
                        mvpDirectMapPointsCache.erase(iter);
                }
            }
        }

        // 如果刚进行过重定位，且inlier数量太少（相比于没有重定位时更加严格）
        if (mCurrentFrame.mnId < mnLastRelocFrameId + mMaxFrames && mnMatchesInliers < 50) {
            mbDirectFailed = true;
            return false;
        }

        // 为保证lastframe帧有足够的观测量，这里最好定高一些
        if (mnMatchesInliers < 30) {
            LOG(WARNING) << "Track Local Map direct failed" << endl;
            mbDirectFailed = true;
            return false;
        } else {
            mbDirectFailed = false;
            return true;
        }
    }

    void Tracking::SearchLocalPointsDirect() {
        int cntSuccess = 0;
        // use grid to evaluate the coverage of feature points
        const int grid_size = 5;
        const int grid_rows = mCurrentFrame.mvImagePyramid[0].rows / grid_size;
        const int grid_cols = mCurrentFrame.mvImagePyramid[0].cols / grid_size;
        vector<bool> grid(grid_rows * grid_cols, false);

        ORBmatcher matcher;
        if (!mvpDirectMapPointsCache.empty()) {
            // 缓存不空，则在缓存中搜索
            for (auto iter = mvpDirectMapPointsCache.begin(); iter != mvpDirectMapPointsCache.end();) {
                MapPoint *mp = *iter;
                if (mp->isBad() || mCurrentFrame.isInFrustum(mp, 0.5) == false)      // 坏蛋点或者在视野外
                {
                    iter = mvpDirectMapPointsCache.erase(iter);
                    continue;
                }

                int gx = static_cast<int> ( mp->mTrackProjX / grid_size );
                int gy = static_cast<int> ( mp->mTrackProjY / grid_size );
                int k = gy * grid_cols + gx;

                if (grid[k] == true) {
                    iter++;         // 这个点不知道能不能匹配，所以先保留在cache里头
                    continue;        // already exist a projection
                }

                // try align it with current frame
                auto obs = mp->GetObservations();
                auto obs_sorted = SelectNearestKeyframe(obs, 5);
                vector<Vector2f> matched_pixels;
                for (auto &o: obs_sorted) {
                    Vector2f px_curr(mp->mTrackProjX, mp->mTrackProjY);
                    int level = mp->mnTrackScaleLevel;

                    if (matcher.FindDirectProjection(o.first, &mCurrentFrame, mp, px_curr, level)) {
                        if (px_curr[0] < 20 || px_curr[1] < 20
                            || px_curr[0] >= mCurrentFrame.mvImagePyramid[0].cols - 20
                            || px_curr[1] >= mCurrentFrame.mvImagePyramid[0].rows - 20)
                            continue;   // 丢弃位于太边缘的地方的点，否则在创建关键帧，计算描述子时可能导致溢出
                        matched_pixels.push_back(px_curr);
                        mCurrentFrame.mvMatchedFrom.push_back(o.first->mnId);
                        break;
                    }
                }
                if (!matched_pixels.empty()) {
                    // 有成功追踪到的点，取平均像素位置为测量
                    Vector2f px_ave(0, 0);
                    for (Vector2f &p: matched_pixels)
                        px_ave += p;
                    px_ave = px_ave / matched_pixels.size();

                    // insert a feature and assign it to a map point
                    mCurrentFrame.mvKeys.push_back(
                            cv::KeyPoint(cv::Point2f(px_ave[0], px_ave[1]), 7, -1, 0, 0)
                    );

                    mCurrentFrame.mvpMapPoints.push_back(mp);
                    mCurrentFrame.mvDepth.push_back(-1);
                    mCurrentFrame.mvbOutlier.push_back(false);

                    int gx = static_cast<int> ( px_ave[0] / grid_size );
                    int gy = static_cast<int> ( px_ave[1] / grid_size );
                    int k = gy * grid_cols + gx;
                    grid[k] = true;

                    iter++;
                    cntSuccess++;
                } else {
                    // 一次都没匹配上
                    iter = mvpDirectMapPointsCache.erase(iter);
                }
            }
        }

        if (cntSuccess > mnCacheHitTh) {
            // 从缓存中就得到了足够的匹配点，直接返回
            // we matched enough points in cache, then do pose optimization
            mCurrentFrame.N = mCurrentFrame.mvKeys.size();
            mCurrentFrame.mvuRight.resize(mCurrentFrame.N, -1);
            return;
        }

        // 否则，更新地图点，并从地图点中拿到更多的点
        // no enough projections, search more in local map points
        UpdateLocalMap();

        int rejected = 0;
        int outside = 0;
        for (MapPoint *mp: mvpLocalMapPoints) {
            if (mvpDirectMapPointsCache.find(mp) != mvpDirectMapPointsCache.end()) {
                // 已经在缓存中（同时说明已经匹配）
                // already in cache (means already matched)
                continue;
            }
            // 后续和上面是一样的
            if (mp->isBad()) // 坏蛋点
                continue;
            if (mCurrentFrame.isInFrustum(mp, 0.5) == false) // 在视野外或视线角太大
            {
                outside++;
                continue;
            }

            // 如果上面那个判断通过，那么这个地图点在此帧的投影位置就有一个大致的估计
            // 比较这个点的某次观测与当前帧的图像
            ORBmatcher matcher;
            // 我们比较最近一些观测
            auto obs = mp->GetObservations();
            auto obs_sorted = SelectNearestKeyframe(obs, 5);
            vector<Vector2f> matched_pixels;
            for (auto &o: obs_sorted) {
                Vector2f px_curr(mp->mTrackProjX, mp->mTrackProjY);
                int level = mp->mnTrackScaleLevel;
                if (matcher.FindDirectProjection(o.first, &mCurrentFrame, mp, px_curr, level)) {
                    if (px_curr[0] < 20 || px_curr[1] < 20
                        || px_curr[0] >= mCurrentFrame.mvImagePyramid[0].cols - 20
                        || px_curr[1] >= mCurrentFrame.mvImagePyramid[0].rows - 20)
                        continue;   // 丢弃位于太边缘的地方的点，否则在创建关键帧，计算描述子时可能导致溢出
                    matched_pixels.push_back(px_curr);
                    mCurrentFrame.mvMatchedFrom.push_back(o.first->mnId);

                    break;
                }
            }

            if (!matched_pixels.empty()) {
                // 有成功追踪到的点，取平均像素位置为测量
                Vector2f px_ave(0, 0);
                for (Vector2f &p: matched_pixels)
                    px_ave += p;
                px_ave = px_ave / matched_pixels.size();

                // insert a feature and assign it to a map point
                mCurrentFrame.mvKeys.push_back(
                        cv::KeyPoint(cv::Point2f(px_ave[0], px_ave[1]), 7, -1, 0, 0)
                );

                mCurrentFrame.mvpMapPoints.push_back(mp);
                mCurrentFrame.mvDepth.push_back(-1);
                mCurrentFrame.mvbOutlier.push_back(false);

                mvpDirectMapPointsCache.insert(mp);
            } else {
                // 一次都没匹配上，该点无效
                rejected++;
            }
        }

        mCurrentFrame.N = mCurrentFrame.mvKeys.size();
        mCurrentFrame.mvuRight.resize(mCurrentFrame.N, -1);
    }

    vector<std::pair<KeyFrame *, size_t> >
    Tracking::SelectNearestKeyframe(const std::map<KeyFrame *, size_t> &observations, int n) {
        vector<std::pair<KeyFrame *, size_t> > s;
        for (auto &o: observations) {
            if (!o.first->isBad() && o.first != mpLastKeyFrame)
                s.push_back(make_pair(o.first, o.second));
        }
        // 按照id排序
        // 然而这里选最近的点会容易导致飘移
        // sort( s.begin(), s.end(),
        // [](const pair<KeyFrame*, size_t>& p1, const pair<KeyFrame*, size_t>& p2) {return p1.first->mnId > p2.first->mnId; } );
        sort(s.begin(), s.end(),
             [](const pair<KeyFrame *, size_t> &p1, const pair<KeyFrame *, size_t> &p2) {
                 return p1.first->mnId > p2.first->mnId;
             });

        if ((int) s.size() < n)
            return s;
        else
            return vector<std::pair<KeyFrame *, size_t> >(s.begin(), s.begin() + n);
    }


} //namespace ygz
