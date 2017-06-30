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

#include "Frame.h"
#include "Converter.h"
#include "ORBmatcher.h"

namespace ygz {

    long unsigned int Frame::nNextId = 0;
    bool Frame::mbNeedUndistort = false;
    bool Frame::mbInitialComputations = true;
    float Frame::cx = 0, Frame::cy = 0, Frame::fx = 0, Frame::fy = 0, Frame::invfx = 0, Frame::invfy = 0;
    float Frame::mnMinX = 0, Frame::mnMinY = 0, Frame::mnMaxX = 0, Frame::mnMaxY = 0;
    float Frame::mfGridElementWidthInv = 0, Frame::mfGridElementHeightInv = 0;
    Mat Frame::map1, Frame::map2;

    // ---------------------------------------------------------------------------------------
    // IMU related
    void Frame::SetInitialNavStateAndBias(const NavState &ns) {
        mNavState = ns;
        // Set bias as bias+delta_bias, and reset the delta_bias term
        mNavState.Set_BiasGyr(ns.Get_BiasGyr() + ns.Get_dBias_Gyr());
        mNavState.Set_BiasAcc(ns.Get_BiasAcc() + ns.Get_dBias_Acc());
        mNavState.Set_DeltaBiasGyr(Vector3d::Zero());
        mNavState.Set_DeltaBiasAcc(Vector3d::Zero());
    }

    void Frame::SetNavStateBiasGyr(const Vector3d &bg) {
        mNavState.Set_BiasGyr(bg);
    }

    void Frame::SetNavStateBiasAcc(const Vector3d &ba) {
        mNavState.Set_BiasAcc(ba);
    }

    void Frame::UpdateNavState(const IMUPreintegrator &imupreint, const Vector3d &gw) {
        Converter::updateNS(mNavState, imupreint, gw);
    }

    void Frame::ComputeIMUPreIntSinceLastFrame(const Frame *pLastF, IMUPreintegrator &IMUPreInt) const {
        // Reset pre-integrator first
        IMUPreInt.reset();

        const std::vector<IMUData> &vIMUSInceLastFrame = mvIMUDataSinceLastFrame;

        Vector3d bg = pLastF->GetNavState().Get_BiasGyr();
        Vector3d ba = pLastF->GetNavState().Get_BiasAcc();

        // remember to consider the gap between the last KF and the first IMU
        {
            const IMUData &imu = vIMUSInceLastFrame.front();
            double dt = std::max(0., imu._t - pLastF->mTimeStamp);
            IMUPreInt.update(imu._g - bg, imu._a - ba, dt);
        }
        // integrate each imu
        for (size_t i = 0; i < vIMUSInceLastFrame.size(); i++) {
            const IMUData &imu = vIMUSInceLastFrame[i];
            double nextt;
            if (i == vIMUSInceLastFrame.size() - 1) {
                nextt = mTimeStamp;    // last IMU, next is this KeyFrame
            } else {
                nextt = vIMUSInceLastFrame[i + 1]._t;    // regular condition, next is imu data
            }

            // delta time
            double dt = std::max(0., nextt - imu._t);
            // update pre-integrator
            IMUPreInt.update(imu._g - bg, imu._a - ba, dt);
        }
    }

    void Frame::UpdatePoseFromNS(const SE3d &Tbc) {
        const SO3d &Rbc = Tbc.so3();
        const Vector3d &Pbc = Tbc.translation();

        const SO3d &Rwb = mNavState.Get_R();
        const Vector3d &Pwb = mNavState.Get_P();

        SO3d Rcw = (Rwb * Rbc).inverse();
        Vector3d Pwc = Rwb * Pbc + Pwb;
        Vector3d Pcw = -(Rcw * Pwc);
        SetPose(SE3d(Rcw, Pcw).cast<float>());
    }


    // constructor for vio
    Frame::Frame(const cv::Mat &imGray, const double &timeStamp, const std::vector<IMUData> &vimu,
                 ORBextractor *extractor, ORBVocabulary *voc,
                 Matrix3f &K, cv::Mat &distCoef, const float &bf, const float &thDepth, KeyFrame *pLastKF)
            : mpORBvocabulary(voc), mpORBextractorLeft(extractor),
              mpORBextractorRight(static_cast<ORBextractor *> ( NULL )),
              mTimeStamp(timeStamp), mK(K), mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth),
              mImGray(imGray.clone()), mSensor(Monocular) {
        // Copy IMU data
        mvIMUDataSinceLastFrame = vimu;

        // Frame ID
        mnId = nNextId++;

        // Scale Level Info
        mnScaleLevels = mpORBextractorLeft->GetLevels();
        mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
        mfLogScaleFactor = log(mfScaleFactor);
        mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
        mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
        mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
        mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

        // This is done only for the first Frame (or after a change in the calibration)
        if (mbInitialComputations) {
            ComputeImageBounds(imGray);
            mfGridElementWidthInv = static_cast<float> ( FRAME_GRID_COLS ) / static_cast<float> ( mnMaxX - mnMinX );
            mfGridElementHeightInv = static_cast<float> ( FRAME_GRID_ROWS ) / static_cast<float> ( mnMaxY - mnMinY );

            fx = K(0, 0);
            fy = K(1, 1);
            cx = K(0, 2);
            cy = K(1, 2);
            invfx = 1.0f / fx;
            invfy = 1.0f / fy;
            for (size_t i = 0; i < 4; i++)
                if (distCoef.at<float>(i) != 0) {
                    mbNeedUndistort = true;
                }

            mbInitialComputations = false;
        }

        mb = mbf / fx;
        ComputeImagePyramid();
    }


    // Dummy constructor
    Frame::Frame() {}

    // Copy Constructor
    Frame::Frame(const Frame &frame)
            : mpORBvocabulary(frame.mpORBvocabulary), mpORBextractorLeft(frame.mpORBextractorLeft),
              mpORBextractorRight(frame.mpORBextractorRight),
              mTimeStamp(frame.mTimeStamp), mK(frame.mK), mDistCoef(frame.mDistCoef.clone()),
              mbf(frame.mbf), mb(frame.mb), mThDepth(frame.mThDepth), N(frame.N), mvKeys(frame.mvKeys),
              mvKeysRight(frame.mvKeysRight), mvuRight(frame.mvuRight),
              mvDepth(frame.mvDepth), mBowVec(frame.mBowVec), mFeatVec(frame.mFeatVec),
              mDescriptors(frame.mDescriptors.clone()), mDescriptorsRight(frame.mDescriptorsRight.clone()),
              mvpMapPoints(frame.mvpMapPoints), mvbOutlier(frame.mvbOutlier), mnId(frame.mnId),
              mpReferenceKF(frame.mpReferenceKF), mnScaleLevels(frame.mnScaleLevels),
              mfScaleFactor(frame.mfScaleFactor), mfLogScaleFactor(frame.mfLogScaleFactor),
              mvScaleFactors(frame.mvScaleFactors), mvInvScaleFactors(frame.mvInvScaleFactors),
              mvLevelSigma2(frame.mvLevelSigma2), mvInvLevelSigma2(frame.mvInvLevelSigma2), mbPoseSet(false),
              mbFeatureExtracted(false), mImGray(frame.mImGray.clone()), mImRight(frame.mImRight.clone()),
              mImDepth(frame.mImDepth.clone()) {
        for (int i = 0; i < FRAME_GRID_COLS; i++)
            for (int j = 0; j < FRAME_GRID_ROWS; j++) {
                mGrid[i][j] = frame.mGrid[i][j];
            }

        if (frame.mbPoseSet == true) {
            SetPose(frame.mTcw);
        }

        mvIMUDataSinceLastFrame = frame.mvIMUDataSinceLastFrame;
        mNavState = frame.GetNavState();
        mMargCovInv = frame.mMargCovInv;
        mNavStatePrior = frame.mNavStatePrior;

        // copy the image pyramid
        for (const cv::Mat &mat: frame.mvImagePyramid) {
            mvImagePyramid.push_back(mat.clone());
        }
    }

    // Stereo 双目的初始化
    Frame::Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp, ORBextractor *extractorLeft,
                 ORBextractor *extractorRight, ORBVocabulary *voc, Matrix3f &K, cv::Mat &distCoef, const float &bf,
                 const float &thDepth)
            : mpORBvocabulary(voc), mpORBextractorLeft(extractorLeft), mpORBextractorRight(extractorRight),
              mTimeStamp(timeStamp), mK(K), mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth),
              mImGray(imLeft.clone()), mImRight(imRight.clone()), mSensor(Stereo) {
        // Frame ID
        mnId = nNextId++;

        // Scale Level Info
        mnScaleLevels = mpORBextractorLeft->GetLevels();
        mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
        mfLogScaleFactor = log(mfScaleFactor);
        mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
        mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
        mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
        mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

        ComputeImagePyramid();

        // This is done only for the first Frame (or after a change in the calibration)
        if (mbInitialComputations) {
            ComputeImageBounds(imLeft);

            mfGridElementWidthInv = static_cast<float> ( FRAME_GRID_COLS ) / (mnMaxX - mnMinX);
            mfGridElementHeightInv = static_cast<float> ( FRAME_GRID_ROWS ) / (mnMaxY - mnMinY);

            fx = K(0, 0);
            fy = K(1, 1);
            cx = K(0, 2);
            cy = K(1, 2);
            invfx = 1.0f / fx;
            invfy = 1.0f / fy;
            mbInitialComputations = false;
        }
        mb = mbf / fx;

    }

    // RGBD
    Frame::Frame(
            const cv::Mat &imGray, const cv::Mat &imDepth, const double &timeStamp, ORBextractor *extractor,
            ORBVocabulary *voc, Matrix3f &K, cv::Mat &distCoef, const float &bf, const float &thDepth)
            : mpORBvocabulary(voc), mpORBextractorLeft(extractor),
              mTimeStamp(timeStamp), mK(K), mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth),
              mImGray(imGray.clone()), mSensor(RGBD), mImDepth(imDepth) {
        // Frame ID
        mnId = nNextId++;

        // Scale Level Info
        mnScaleLevels = mpORBextractorLeft->GetLevels();
        mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
        mfLogScaleFactor = log(mfScaleFactor);
        mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
        mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
        mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
        mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

        // This is done only for the first Frame (or after a change in the calibration)
        if (mbInitialComputations) {
            ComputeImageBounds(imGray);

            mfGridElementWidthInv = static_cast<float> ( FRAME_GRID_COLS ) / static_cast<float> ( mnMaxX - mnMinX );
            mfGridElementHeightInv = static_cast<float> ( FRAME_GRID_ROWS ) / static_cast<float> ( mnMaxY - mnMinY );

            fx = K(0, 0);
            fy = K(1, 1);
            cx = K(0, 2);
            cy = K(1, 2);
            invfx = 1.0f / fx;
            invfy = 1.0f / fy;
            mbInitialComputations = false;
        }

        mb = mbf / fx;

        ComputeImagePyramid();
    }


    // monocular
    Frame::Frame(
            const cv::Mat &imGray, const double &timeStamp, ORBextractor *extractor, ORBVocabulary *voc, Matrix3f &K,
            cv::Mat &distCoef, const float &bf, const float &thDepth)
            : mpORBvocabulary(voc), mpORBextractorLeft(extractor),
              mpORBextractorRight(static_cast<ORBextractor *> ( NULL )),
              mTimeStamp(timeStamp), mK(K), mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth),
              mImGray(imGray.clone()), mSensor(Monocular) {
        // Frame ID
        mnId = nNextId++;

        // Scale Level Info
        mnScaleLevels = mpORBextractorLeft->GetLevels();
        mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
        mfLogScaleFactor = log(mfScaleFactor);
        mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
        mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
        mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
        mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

        // This is done only for the first Frame (or after a change in the calibration)
        if (mbInitialComputations) {
            ComputeImageBounds(imGray);

            mfGridElementWidthInv = static_cast<float> ( FRAME_GRID_COLS ) / static_cast<float> ( mnMaxX - mnMinX );
            mfGridElementHeightInv = static_cast<float> ( FRAME_GRID_ROWS ) / static_cast<float> ( mnMaxY - mnMinY );

            fx = K(0, 0);
            fy = K(1, 1);
            cx = K(0, 2);
            cy = K(1, 2);
            invfx = 1.0f / fx;
            invfy = 1.0f / fy;

            mbInitialComputations = false;
        }

        mb = mbf / fx;

        ComputeImagePyramid();
    }

    void Frame::AssignFeaturesToGrid() {
        int nReserve = 0.5f * N / (FRAME_GRID_COLS * FRAME_GRID_ROWS);
        for (unsigned int i = 0; i < FRAME_GRID_COLS; i++)
            for (unsigned int j = 0; j < FRAME_GRID_ROWS; j++) {
                mGrid[i][j].reserve(nReserve);
            }

        for (int i = 0; i < N; i++) {
            const cv::KeyPoint &kp = mvKeys[i];

            int nGridPosX, nGridPosY;
            if (PosInGrid(kp, nGridPosX, nGridPosY)) {
                mGrid[nGridPosX][nGridPosY].push_back(i);
            }
        }
        mbGridSet = true;
    }

    void Frame::ExtractORB(int flag, const cv::Mat &im) {
        // flag =0表示左眼，或者rgbd中的gray图，为1表示右眼
        if (flag == 0) {
            if (N > 0 && mbFeatureExtracted == false) // 没提特征然而有追踪点，来自直接法，提取新的fast特征点
            {
                (*mpORBextractorLeft)(this, mvKeys, mDescriptors, ORBextractor::DSO_KEYPOINT);
            } else {
                // 该帧还没有提特征，可能来自重定位或初始化，使用原版ORB提取
                (*mpORBextractorLeft)( this,mvKeys,mDescriptors,ORBextractor::ORBSLAM_KEYPOINT );
                // 当然也可以尝试DSO的方法
                // (*mpORBextractorLeft)(this, mvKeys, mDescriptors, ORBextractor::DSO_KEYPOINT);
            }
        } else {
            // 右眼
            (*mpORBextractorRight)(this, mvKeysRight, mDescriptorsRight, ORBextractor::ORBSLAM_KEYPOINT, false);
        }
    }

    void Frame::SetPose(const SE3f &Tcw) {
        mTcw = Tcw;
        mbPoseSet = true;
        UpdatePoseMatrices();
    }

    void Frame::UpdatePoseMatrices() {
        mRcw = mTcw.rotationMatrix();
        mRwc = mRcw.transpose();
        mtcw = mTcw.translation();
        mOw = -(mRcw.transpose() * mtcw);
    }

    bool Frame::isInFrustum(MapPoint *pMP, float viewingCosLimit) {
        pMP->mbTrackInView = false;

        // 3D in absolute coordinates
        Vector3f P = pMP->GetWorldPos();

        // 3D in camera coordinates
        const Vector3f Pc = mRcw * P + mtcw;
        const float &PcX = Pc[0];
        const float &PcY = Pc[1];
        const float &PcZ = Pc[2];

        // Check positive depth
        if (PcZ < 0.0f) {
            return false;
        }

        // Project in image and check it is not outside
        const float invz = 1.0f / PcZ;
        const float u = fx * PcX * invz + cx;
        const float v = fy * PcY * invz + cy;

        if (u < mnMinX || u > mnMaxX) {
            return false;
        }
        if (v < mnMinY || v > mnMaxY) {
            return false;
        }

        // Check distance is in the scale invariance region of the MapPoint
        const float maxDistance = pMP->GetMaxDistanceInvariance();
        const float minDistance = pMP->GetMinDistanceInvariance();
        const Vector3f PO = P - mOw;
        const float dist = PO.norm();

        if (dist < minDistance || dist > maxDistance) {
            return false;
        }

        // Check viewing angle
        Vector3f Pn = pMP->GetNormal();
        const float viewCos = PO.dot(Pn) / dist;

        if (viewCos < viewingCosLimit) {
            return false;
        }

        // Predict scale in the image
        const int nPredictedLevel = pMP->PredictScale(dist, this);

        // Data used by the tracking
        pMP->mbTrackInView = true;
        pMP->mTrackProjX = u;
        pMP->mTrackProjXR = u - mbf * invz;
        pMP->mTrackProjY = v;
        pMP->mnTrackScaleLevel = nPredictedLevel;
        pMP->mTrackViewCos = viewCos;

        return true;
    }

    vector<size_t> Frame::GetFeaturesInArea(const float &x, const float &y, const float &r, const int minLevel,
                                            const int maxLevel) const {
        vector<size_t> vIndices;
        vIndices.reserve(N);

        const int nMinCellX = max(0, (int) floor((x - mnMinX - r) * mfGridElementWidthInv));
        if (nMinCellX >= FRAME_GRID_COLS) {
            return vIndices;
        }

        const int nMaxCellX = min((int) FRAME_GRID_COLS - 1, (int) ceil((x - mnMinX + r) * mfGridElementWidthInv));
        if (nMaxCellX < 0) {
            return vIndices;
        }

        const int nMinCellY = max(0, (int) floor((y - mnMinY - r) * mfGridElementHeightInv));
        if (nMinCellY >= FRAME_GRID_ROWS) {
            return vIndices;
        }

        const int nMaxCellY = min((int) FRAME_GRID_ROWS - 1, (int) ceil((y - mnMinY + r) * mfGridElementHeightInv));
        if (nMaxCellY < 0) {
            return vIndices;
        }

        const bool bCheckLevels = (minLevel > 0) || (maxLevel >= 0);

        for (int ix = nMinCellX; ix <= nMaxCellX; ix++) {
            for (int iy = nMinCellY; iy <= nMaxCellY; iy++) {
                const vector<size_t> vCell = mGrid[ix][iy];
                if (vCell.empty()) {
                    continue;
                }

                for (size_t j = 0, jend = vCell.size(); j < jend; j++) {
                    const cv::KeyPoint &kpUn = mvKeys[vCell[j]];
                    if (bCheckLevels) {
                        if (kpUn.octave < minLevel) {
                            continue;
                        }
                        if (maxLevel >= 0)
                            if (kpUn.octave > maxLevel) {
                                continue;
                            }
                    }

                    const float distx = kpUn.pt.x - x;
                    const float disty = kpUn.pt.y - y;

                    if (fabs(distx) < r && fabs(disty) < r) {
                        vIndices.push_back(vCell[j]);
                    }
                }
            }
        }

        return vIndices;
    }

    bool Frame::PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY) {
        posX = round((kp.pt.x - mnMinX) * mfGridElementWidthInv);
        posY = round((kp.pt.y - mnMinY) * mfGridElementHeightInv);

        //Keypoint's coordinates are undistorted, which could cause to go out of the image
        if (posX < 0 || posX >= FRAME_GRID_COLS || posY < 0 || posY >= FRAME_GRID_ROWS) {
            return false;
        }

        return true;
    }

    void Frame::ComputeBoW() {
        if (mBowVec.empty()) {
            vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mDescriptors);
            mpORBvocabulary->transform(vCurrentDesc, mBowVec, mFeatVec, 4);
        }
    }

    void Frame::ComputeImageBounds(const cv::Mat &imLeft) {
        mnMinX = 0.0f;
        mnMaxX = imLeft.cols;
        mnMinY = 0.0f;
        mnMaxY = imLeft.rows;
    }

    void Frame::ComputeStereoMatches() {
        mvuRight = vector<float>(N, -1.0f);
        mvDepth = vector<float>(N, -1.0f);

        const int thOrbDist = (ORBmatcher::TH_HIGH + ORBmatcher::TH_LOW) / 2;

        const int nRows = mpORBextractorLeft->mvImagePyramid[0].rows;

        //Assign keypoints to row table
        vector<vector<size_t> > vRowIndices(nRows, vector<size_t>());

        for (int i = 0; i < nRows; i++) {
            vRowIndices[i].reserve(200);
        }

        const int Nr = mvKeysRight.size();

        for (int iR = 0; iR < Nr; iR++) {
            const cv::KeyPoint &kp = mvKeysRight[iR];
            const float &kpY = kp.pt.y;
            const float r = 2.0f * mvScaleFactors[mvKeysRight[iR].octave];
            const int maxr = ceil(kpY + r);
            const int minr = floor(kpY - r);

            for (int yi = minr; yi <= maxr; yi++) {
                vRowIndices[yi].push_back(iR);
            }
        }

        // Set limits for search
        const float minZ = mb;
        const float minD = 0;
        const float maxD = mbf / minZ;

        // For each left keypoint search a match in the right image
        vector<pair<int, int> > vDistIdx;
        vDistIdx.reserve(N);

        for (int iL = 0; iL < N; iL++) {
            const cv::KeyPoint &kpL = mvKeys[iL];
            const int &levelL = kpL.octave;
            const float &vL = kpL.pt.y;
            const float &uL = kpL.pt.x;

            const vector<size_t> &vCandidates = vRowIndices[vL];

            if (vCandidates.empty()) {
                continue;
            }

            const float minU = uL - maxD;
            const float maxU = uL - minD;

            if (maxU < 0) {
                continue;
            }

            int bestDist = ORBmatcher::TH_HIGH;
            size_t bestIdxR = 0;

            const cv::Mat &dL = mDescriptors.row(iL);

            // Compare descriptor to right keypoints
            for (size_t iC = 0; iC < vCandidates.size(); iC++) {
                const size_t iR = vCandidates[iC];
                const cv::KeyPoint &kpR = mvKeysRight[iR];

                if (kpR.octave < levelL - 1 || kpR.octave > levelL + 1) {
                    continue;
                }

                const float &uR = kpR.pt.x;

                if (uR >= minU && uR <= maxU) {
                    const cv::Mat &dR = mDescriptorsRight.row(iR);
                    const int dist = ORBmatcher::DescriptorDistance(dL, dR);

                    if (dist < bestDist) {
                        bestDist = dist;
                        bestIdxR = iR;
                    }
                }
            }

            // Subpixel match by correlation
            if (bestDist < thOrbDist) {
                // coordinates in image pyramid at keypoint scale
                const float uR0 = mvKeysRight[bestIdxR].pt.x;
                const float scaleFactor = mvInvScaleFactors[kpL.octave];
                const float scaleduL = round(kpL.pt.x * scaleFactor);
                const float scaledvL = round(kpL.pt.y * scaleFactor);
                const float scaleduR0 = round(uR0 * scaleFactor);

                // sliding window search
                const int w = 5;
                cv::Mat IL = mpORBextractorLeft->mvImagePyramid[kpL.octave].rowRange(scaledvL - w,
                                                                                     scaledvL + w + 1).colRange(
                        scaleduL - w, scaleduL + w + 1);
                IL.convertTo(IL, CV_32F);
                IL = IL - IL.at<float>(w, w) * cv::Mat::ones(IL.rows, IL.cols, CV_32F);

                int bestDist = INT_MAX;
                int bestincR = 0;
                const int L = 5;
                vector<float> vDists;
                vDists.resize(2 * L + 1);

                const float iniu = scaleduR0 + L - w;
                const float endu = scaleduR0 + L + w + 1;
                if (iniu < 0 || endu >= mpORBextractorRight->mvImagePyramid[kpL.octave].cols) {
                    continue;
                }

                for (int incR = -L; incR <= +L; incR++) {
                    cv::Mat IR = mpORBextractorRight->mvImagePyramid[kpL.octave].rowRange(scaledvL - w,
                                                                                          scaledvL + w + 1).colRange(
                            scaleduR0 + incR - w, scaleduR0 + incR + w + 1);
                    IR.convertTo(IR, CV_32F);
                    IR = IR - IR.at<float>(w, w) * cv::Mat::ones(IR.rows, IR.cols, CV_32F);

                    float dist = cv::norm(IL, IR, cv::NORM_L1);
                    if (dist < bestDist) {
                        bestDist = dist;
                        bestincR = incR;
                    }

                    vDists[L + incR] = dist;
                }

                if (bestincR == -L || bestincR == L) {
                    continue;
                }

                // Sub-pixel match (Parabola fitting)
                const float dist1 = vDists[L + bestincR - 1];
                const float dist2 = vDists[L + bestincR];
                const float dist3 = vDists[L + bestincR + 1];

                const float deltaR = (dist1 - dist3) / (2.0f * (dist1 + dist3 - 2.0f * dist2));

                if (deltaR < -1 || deltaR > 1) {
                    continue;
                }

                // Re-scaled coordinate
                float bestuR = mvScaleFactors[kpL.octave] * ((float) scaleduR0 + (float) bestincR + deltaR);

                float disparity = (uL - bestuR);

                if (disparity >= minD && disparity < maxD) {
                    if (disparity <= 0) {
                        disparity = 0.01;
                        bestuR = uL - 0.01;
                    }
                    mvDepth[iL] = mbf / disparity;
                    mvuRight[iL] = bestuR;
                    vDistIdx.push_back(pair<int, int>(bestDist, iL));
                }
            }
        }

        sort(vDistIdx.begin(), vDistIdx.end());
        const float median = vDistIdx[vDistIdx.size() / 2].first;
        const float thDist = 1.5f * 1.4f * median;

        for (int i = vDistIdx.size() - 1; i >= 0; i--) {
            if (vDistIdx[i].first < thDist) {
                break;
            } else {
                mvuRight[vDistIdx[i].second] = -1;
                mvDepth[vDistIdx[i].second] = -1;
            }
        }
    }


    void Frame::ComputeStereoFromRGBD(const cv::Mat &imDepth) {
        mvuRight = vector<float>(N, -1);
        mvDepth = vector<float>(N, -1);

        for (int i = 0; i < N; i++) {
            const cv::KeyPoint &kp = mvKeys[i];

            const float &v = kp.pt.y;
            const float &u = kp.pt.x;
            const float d = imDepth.at<float>(v, u);

            if (d > 0) {
                mvDepth[i] = d;
                mvuRight[i] = kp.pt.x - mbf / d;
            }
        }
    }

    Vector3f Frame::UnprojectStereo(const int &i) {
        const float z = mvDepth[i];
        if (z > 0) {
            const float u = mvKeys[i].pt.x;
            const float v = mvKeys[i].pt.y;
            const float x = (u - cx) * z * invfx;
            const float y = (v - cy) * z * invfy;
            Vector3f x3Dc(x, y, z);
            return mRwc * x3Dc + mOw;
        } else {
            return Vector3f(0, 0, 0);
        }
    }

    void Frame::ExtractFeatures() {
        if (mbFeatureExtracted == true) {
            LOG (INFO) << "Frame " << mnId << " feature already extracted." << endl;
            return;
        }

        if (mSensor == Monocular || mSensor == RGBD) {
            // 单目和RGBD只需提一张图
            ExtractORB(0, mImGray);
        } else {
            // 双目左右都要提
            thread threadLeft(&Frame::ExtractORB, this, 0, mImGray);
            thread threadRight(&Frame::ExtractORB, this, 1, mImRight);
            threadLeft.join();
            threadRight.join();
        }

        N = mvKeys.size();

        if (mvKeys.empty()) {
            LOG (WARNING) << "Cannot extract features, please check your image!" << endl;
            return;
        }

        if (mSensor == Monocular) {
            // Set no stereo information
            mvuRight = vector<float>(N, -1);
            mvDepth = vector<float>(N, -1);
        } else if (mSensor == Stereo) {
            // stereo mode, match the right key points
            ComputeStereoMatches();
        } else {
            // RGBD
            ComputeStereoFromRGBD(mImDepth);
        }

        // set the map points
        if (mvpMapPoints.size() == 0) {
            mvpMapPoints = vector<MapPoint *>(N, static_cast<MapPoint *> ( NULL ));
            mvbOutlier = vector<bool>(N, false);
        } else {
            // we already have associated map points
            size_t size_before = mvpMapPoints.size();
            mvpMapPoints.resize(N);
            mvbOutlier.resize(N);
            for (int i = size_before; i < N; i++) {
                mvpMapPoints[i] = nullptr;
                mvbOutlier[i] = false;
            }
        }

        AssignFeaturesToGrid();
        ComputeBoW();
        mbFeatureExtracted = true;
    }

    void Frame::ComputeImagePyramid() {
        // Undistort the image
        if (mbNeedUndistort) {
            if (map1.empty()) {
                // init the undistortion map
                cv::initUndistortRectifyMap(
                        Converter::toCvMat(mK),
                        mDistCoef, Mat(), Converter::toCvMat(mK),
                        cv::Size(mImGray.cols, mImGray.rows),
                        CV_16SC2, map1, map2
                );
            }

            if (mSensor == Monocular || mSensor == Stereo || mSensor == RGBD) {
                cv::Mat img_undistorted;
                cv::remap(mImGray, img_undistorted, map1, map2, cv::INTER_LINEAR); // 似乎不能把dst和src设成一样的。。
                mImGray = img_undistorted;
            }
            // RGBD 似乎不用去畸变?
            if (mSensor == Stereo) {
                // also distort the right camera
                cv::Mat img_undistorted;
                cv::remap(mImRight, img_undistorted, map1, map2, cv::INTER_LINEAR);
                mImRight = img_undistorted;
            }

            if (mSensor == RGBD) {
                // also distort the depth image
                cv::Mat img_undistorted;
                cv::remap(mImDepth, img_undistorted, map1, map2, cv::INTER_LINEAR);
                mImDepth = img_undistorted;
            }
        }

        mpORBextractorLeft->ComputePyramid(mImGray);

        // 把图像金字塔拷贝出来
        mvImagePyramid.resize(mpORBextractorLeft->GetLevels());
        for (int l = 0; l < mpORBextractorLeft->GetLevels(); l++) {
            mvImagePyramid[l] = mpORBextractorLeft->mvImagePyramid[l].clone();
        }
    }
} //namespace ygz
