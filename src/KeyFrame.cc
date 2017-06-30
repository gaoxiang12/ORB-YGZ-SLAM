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
#include <iomanip>

#include "KeyFrame.h"
#include "KeyFrameDatabase.h"
#include "Converter.h"
#include "ORBmatcher.h"
#include "Map.h"

namespace ygz {

    long unsigned int KeyFrame::nNextId = 0;

    void KeyFrame::UpdateNavStatePVRFromTcw(const SE3d &Tcw, const SE3d &Tbc) {
        unique_lock<mutex> lock(mMutexNavState);
        SE3d Twb = (Tbc * Tcw).inverse();
        Matrix3d Rwb = Twb.rotationMatrix();
        Vector3d Pwb = Twb.translation();

        Matrix3d Rw1 = mNavState.Get_RotMatrix();
        Vector3d Vw1 = mNavState.Get_V();
        Vector3d Vw2 = Rwb * Rw1.transpose() * Vw1;   // bV1 = bV2 ==> Rwb1^T*wV1 = Rwb2^T*wV2 ==> wV2 = Rwb2*Rwb1^T*wV1

        mNavState.Set_Pos(Pwb);
        mNavState.Set_Rot(Rwb);
        mNavState.Set_Vel(Vw2);
    }

    void KeyFrame::SetInitialNavStateAndBias(const NavState &ns) {
        unique_lock<mutex> lock(mMutexNavState);
        mNavState = ns;
        // Set bias as bias+delta_bias, and reset the delta_bias term
        mNavState.Set_BiasGyr(ns.Get_BiasGyr() + ns.Get_dBias_Gyr());
        mNavState.Set_BiasAcc(ns.Get_BiasAcc() + ns.Get_dBias_Acc());
        mNavState.Set_DeltaBiasGyr(Vector3d::Zero());
        mNavState.Set_DeltaBiasAcc(Vector3d::Zero());
    }

    KeyFrame *KeyFrame::GetPrevKeyFrame(void) {
        unique_lock<mutex> lock(mMutexPrevKF);
        return mpPrevKeyFrame;
    }

    KeyFrame *KeyFrame::GetNextKeyFrame(void) {
        unique_lock<mutex> lock(mMutexNextKF);
        return mpNextKeyFrame;
    }

    void KeyFrame::SetPrevKeyFrame(KeyFrame *pKF) {
        unique_lock<mutex> lock(mMutexPrevKF);
        mpPrevKeyFrame = pKF;
    }

    void KeyFrame::SetNextKeyFrame(KeyFrame *pKF) {
        unique_lock<mutex> lock(mMutexNextKF);
        mpNextKeyFrame = pKF;
    }

    std::vector<IMUData> KeyFrame::GetVectorIMUData(void) {
        unique_lock<mutex> lock(mMutexIMUData);
        return mvIMUData;
    }

    void KeyFrame::AppendIMUDataToFront(KeyFrame *pPrevKF) {
        std::vector<IMUData> vimunew = pPrevKF->GetVectorIMUData();
        {
            unique_lock<mutex> lock(mMutexIMUData);
            vimunew.insert(vimunew.end(), mvIMUData.begin(), mvIMUData.end());
            mvIMUData = vimunew;
        }
    }

    void KeyFrame::UpdatePoseFromNS(const SE3d &Tbc) {
        Matrix3d Rbc_ = Tbc.rotationMatrix().cast<double>();
        Vector3d Pbc_ = Tbc.translation();

        Matrix3d Rwb_ = mNavState.Get_RotMatrix();
        Vector3d Pwb_ = mNavState.Get_P();

        Matrix3d Rcw_ = (Rwb_ * Rbc_).transpose();
        Vector3d Pwc_ = Rwb_ * Pbc_ + Pwb_;
        Vector3d Pcw_ = -Rcw_ * Pwc_;

        SE3d Tcw_ = SE3d(Rcw_, Pcw_);

        SetPose(Tcw_.cast<float>());
    }

    void KeyFrame::UpdateNavState(const IMUPreintegrator &imupreint, const Vector3d &gw) {
        unique_lock<mutex> lock(mMutexNavState);

        Matrix3d dR = imupreint.getDeltaR();
        Vector3d dP = imupreint.getDeltaP();
        Vector3d dV = imupreint.getDeltaV();
        double dt = imupreint.getDeltaTime();

        Vector3d Pwbpre = mNavState.Get_P();
        Matrix3d Rwbpre = mNavState.Get_RotMatrix();
        Vector3d Vwbpre = mNavState.Get_V();

        Matrix3d Rwb = Rwbpre * dR;
        Vector3d Pwb = Pwbpre + Vwbpre * dt + 0.5 * gw * dt * dt + Rwbpre * dP;
        Vector3d Vwb = Vwbpre + gw * dt + Rwbpre * dV;

        // Here assume that the pre-integration is re-computed after bias updated, so the bias term is ignored
        mNavState.Set_Pos(Pwb);
        mNavState.Set_Vel(Vwb);
        mNavState.Set_Rot(Rwb);
    }

    void KeyFrame::SetNavState(const NavState &ns) {
        unique_lock<mutex> lock(mMutexNavState);
        mNavState = ns;
    }

    const NavState &KeyFrame::GetNavState(void) {
        unique_lock<mutex> lock(mMutexNavState);
        return mNavState;
    }

    void KeyFrame::SetNavStateBiasGyr(const Vector3d &bg) {
        unique_lock<mutex> lock(mMutexNavState);
        mNavState.Set_BiasGyr(bg);
    }

    void KeyFrame::SetNavStateBiasAcc(const Vector3d &ba) {
        unique_lock<mutex> lock(mMutexNavState);
        mNavState.Set_BiasAcc(ba);
    }

    void KeyFrame::SetNavStateVel(const Vector3d &vel) {
        unique_lock<mutex> lock(mMutexNavState);
        mNavState.Set_Vel(vel);
    }

    void KeyFrame::SetNavStatePos(const Vector3d &pos) {
        unique_lock<mutex> lock(mMutexNavState);
        mNavState.Set_Pos(pos);
    }

    void KeyFrame::SetNavStateRot(const Matrix3d &rot) {
        unique_lock<mutex> lock(mMutexNavState);
        mNavState.Set_Rot(rot);
    }

    void KeyFrame::SetNavStateRot(const SO3d &rot) {
        unique_lock<mutex> lock(mMutexNavState);
        mNavState.Set_Rot(rot);
    }

    void KeyFrame::SetNavStateDeltaBg(const Vector3d &dbg) {
        unique_lock<mutex> lock(mMutexNavState);
        mNavState.Set_DeltaBiasGyr(dbg);
    }

    void KeyFrame::SetNavStateDeltaBa(const Vector3d &dba) {
        unique_lock<mutex> lock(mMutexNavState);
        mNavState.Set_DeltaBiasAcc(dba);
    }

    const IMUPreintegrator &KeyFrame::GetIMUPreInt(void) {
        unique_lock<mutex> lock(mMutexIMUData);
        return mIMUPreInt;
    }

    void KeyFrame::ComputePreInt(void) {
        unique_lock<mutex> lock(mMutexIMUData);
        if (mpPrevKeyFrame == NULL) {
            if (mnId != 0) {
                cerr << "previous KeyFrame is NULL, pre-integrator not changed. id: " << mnId << endl;
            }
            return;
        } else {
            // Reset pre-integrator first
            mIMUPreInt.reset();

            if (mvIMUData.empty())
                return;

            // IMU pre-integration integrates IMU data from last to current, but the bias is from last
            Vector3d bg = mpPrevKeyFrame->GetNavState().Get_BiasGyr();
            Vector3d ba = mpPrevKeyFrame->GetNavState().Get_BiasAcc();

            // remember to consider the gap between the last KF and the first IMU
            {
                const IMUData &imu = mvIMUData.front();
                double dt = std::max(0., imu._t - mpPrevKeyFrame->mTimeStamp);
                mIMUPreInt.update(imu._g - bg, imu._a - ba, dt);
            }
            // integrate each imu
            for (size_t i = 0; i < mvIMUData.size(); i++) {
                const IMUData &imu = mvIMUData[i];
                double nextt;
                if (i == mvIMUData.size() - 1)
                    nextt = mTimeStamp;         // last IMU, next is this KeyFrame
                else
                    nextt = mvIMUData[i + 1]._t;  // regular condition, next is imu data

                // delta time
                double dt = std::max(0., nextt - imu._t);
                // update pre-integrator
                mIMUPreInt.update(imu._g - bg, imu._a - ba, dt);
            }
        }
    }

// VIO constructor
    KeyFrame::KeyFrame(Frame &F, Map *pMap, KeyFrameDatabase *pKFDB, std::vector<IMUData> vIMUData, KeyFrame *pPrevKF) :
            mnFrameId(F.mnId), mTimeStamp(F.mTimeStamp), mnGridCols(FRAME_GRID_COLS), mnGridRows(FRAME_GRID_ROWS),
            mfGridElementWidthInv(F.mfGridElementWidthInv), mfGridElementHeightInv(F.mfGridElementHeightInv),
            mnTrackReferenceForFrame(0), mnFuseTargetForKF(0), mnBALocalForKF(0), mnBAFixedForKF(0),
            mnLoopQuery(0), mnLoopWords(0), mnRelocQuery(0), mnRelocWords(0), mnBAGlobalForKF(0),
            fx(F.fx), fy(F.fy), cx(F.cx), cy(F.cy), invfx(F.invfx), invfy(F.invfy),
            mbf(F.mbf), mb(F.mb), mThDepth(F.mThDepth), N(F.N), mvKeys(F.mvKeys),
            mvuRight(F.mvuRight), mvDepth(F.mvDepth), mDescriptors(F.mDescriptors.clone()),
            mBowVec(F.mBowVec), mFeatVec(F.mFeatVec), mnScaleLevels(F.mnScaleLevels), mfScaleFactor(F.mfScaleFactor),
            mfLogScaleFactor(F.mfLogScaleFactor), mvScaleFactors(F.mvScaleFactors), mvLevelSigma2(F.mvLevelSigma2),
            mvInvLevelSigma2(F.mvInvLevelSigma2), mnMinX(F.mnMinX), mnMinY(F.mnMinY), mnMaxX(F.mnMaxX),
            mnMaxY(F.mnMaxY), mK(F.mK), mvpMapPoints(F.mvpMapPoints), mpKeyFrameDB(pKFDB),
            mpORBvocabulary(F.mpORBvocabulary), mbFirstConnection(true), mpParent(NULL), mbNotErase(false),
            mbToBeErased(false), mbBad(false), mHalfBaseline(F.mb / 2), mpMap(pMap) {
        mvIMUData = vIMUData;

        if (pPrevKF) {
            pPrevKF->SetNextKeyFrame(this);
        }
        mpPrevKeyFrame = pPrevKF;
        mpNextKeyFrame = NULL;

        mnId = nNextId++;

        mGrid.resize(mnGridCols);
        for (int i = 0; i < mnGridCols; i++) {
            mGrid[i].resize(mnGridRows);
            for (int j = 0; j < mnGridRows; j++)
                mGrid[i][j] = F.mGrid[i][j];
        }

        SetPose(F.mTcw);
        for (cv::Mat &im: F.mvImagePyramid) {
            mvImagePyramid.push_back(im);
        }
    }

    // regular constructor
    KeyFrame::KeyFrame(Frame &F, Map *pMap, KeyFrameDatabase *pKFDB) :
            mnFrameId(F.mnId), mTimeStamp(F.mTimeStamp), mnGridCols(FRAME_GRID_COLS), mnGridRows(FRAME_GRID_ROWS),
            mfGridElementWidthInv(F.mfGridElementWidthInv), mfGridElementHeightInv(F.mfGridElementHeightInv),
            mnTrackReferenceForFrame(0), mnFuseTargetForKF(0), mnBALocalForKF(0), mnBAFixedForKF(0),
            mnLoopQuery(0), mnLoopWords(0), mnRelocQuery(0), mnRelocWords(0), mnBAGlobalForKF(0),
            fx(F.fx), fy(F.fy), cx(F.cx), cy(F.cy), invfx(F.invfx), invfy(F.invfy),
            mbf(F.mbf), mb(F.mb), mThDepth(F.mThDepth), N(F.N), mvKeys(F.mvKeys),
            mvuRight(F.mvuRight), mvDepth(F.mvDepth), mDescriptors(F.mDescriptors.clone()),
            mBowVec(F.mBowVec), mFeatVec(F.mFeatVec), mnScaleLevels(F.mnScaleLevels), mfScaleFactor(F.mfScaleFactor),
            mfLogScaleFactor(F.mfLogScaleFactor), mvScaleFactors(F.mvScaleFactors), mvLevelSigma2(F.mvLevelSigma2),
            mvInvLevelSigma2(F.mvInvLevelSigma2), mnMinX(F.mnMinX), mnMinY(F.mnMinY), mnMaxX(F.mnMaxX),
            mnMaxY(F.mnMaxY), mK(F.mK), mvpMapPoints(F.mvpMapPoints), mpKeyFrameDB(pKFDB),
            mpORBvocabulary(F.mpORBvocabulary), mbFirstConnection(true), mpParent(NULL), mbNotErase(false),
            mbToBeErased(false), mbBad(false), mHalfBaseline(F.mb / 2), mpMap(pMap), mpPrevKeyFrame(nullptr),
            mpNextKeyFrame(nullptr) {
        mnId = nNextId++;

        mGrid.resize(mnGridCols);
        for (int i = 0; i < mnGridCols; i++) {
            mGrid[i].resize(mnGridRows);
            for (int j = 0; j < mnGridRows; j++)
                mGrid[i][j] = F.mGrid[i][j];
        }

        SetPose(F.mTcw);
        for (cv::Mat &im: F.mvImagePyramid) {
            mvImagePyramid.push_back(im);
        }

    }

    void KeyFrame::ComputeBoW() {
        if (mBowVec.empty() || mFeatVec.empty()) {
            vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mDescriptors);
            // Feature vector associate features with nodes in the 4th level (from leaves up)
            // We assume the vocabulary tree has 6 levels, change the 4 otherwise
            mpORBvocabulary->transform(vCurrentDesc, mBowVec, mFeatVec, 4);
        }
    }

    void KeyFrame::SetPose(const SE3f &Tcw_) {
        unique_lock<mutex> lock(mMutexPose);
        Tcw = Tcw_;
        Matrix3f Rcw = Tcw.rotationMatrix();
        Vector3f tcw = Tcw.translation();
        Matrix3f Rwc = Rcw.transpose();
        Ow = -Rwc * tcw;

        Twc = SE3f(Rwc, Ow);
        Vector3f center(mHalfBaseline, 0, 0);
        Cw = (Twc * center);
    }

    SE3f KeyFrame::GetPose() {
        unique_lock<mutex> lock(mMutexPose);
        return Tcw;
    }

    SE3f KeyFrame::GetPoseInverse() {
        unique_lock<mutex> lock(mMutexPose);
        return Twc;
    }

    Vector3f KeyFrame::GetCameraCenter() {
        unique_lock<mutex> lock(mMutexPose);
        return Ow;
    }

    Vector3f KeyFrame::GetStereoCenter() {
        unique_lock<mutex> lock(mMutexPose);
        return Cw;
    }

    Matrix3f KeyFrame::GetRotation() {
        unique_lock<mutex> lock(mMutexPose);
        return Tcw.rotationMatrix();
    }

    Vector3f KeyFrame::GetTranslation() {
        unique_lock<mutex> lock(mMutexPose);
        return Tcw.translation();
    }

    void KeyFrame::AddConnection(KeyFrame *pKF, const int &weight) {
        {
            unique_lock<mutex> lock(mMutexConnections);
            if (!mConnectedKeyFrameWeights.count(pKF))
                mConnectedKeyFrameWeights[pKF] = weight;
            else if (mConnectedKeyFrameWeights[pKF] != weight)
                mConnectedKeyFrameWeights[pKF] = weight;
            else
                return;
        }

        UpdateBestCovisibles();
    }

    void KeyFrame::UpdateBestCovisibles() {
        unique_lock<mutex> lock(mMutexConnections);
        vector<pair<int, KeyFrame *> > vPairs;
        vPairs.reserve(mConnectedKeyFrameWeights.size());
        for (map<KeyFrame *, int>::iterator mit = mConnectedKeyFrameWeights.begin(), mend = mConnectedKeyFrameWeights.end();
             mit != mend; mit++)
            vPairs.push_back(make_pair(mit->second, mit->first));

        sort(vPairs.begin(), vPairs.end());
        list < KeyFrame * > lKFs;
        list<int> lWs;
        for (size_t i = 0, iend = vPairs.size(); i < iend; i++) {
            lKFs.push_front(vPairs[i].second);
            lWs.push_front(vPairs[i].first);
        }

        mvpOrderedConnectedKeyFrames = vector<KeyFrame *>(lKFs.begin(), lKFs.end());
        mvOrderedWeights = vector<int>(lWs.begin(), lWs.end());
    }

    set<KeyFrame *> KeyFrame::GetConnectedKeyFrames() {
        unique_lock<mutex> lock(mMutexConnections);
        set<KeyFrame *> s;
        for (map<KeyFrame *, int>::iterator mit = mConnectedKeyFrameWeights.begin();
             mit != mConnectedKeyFrameWeights.end(); mit++)
            s.insert(mit->first);
        return s;
    }

    vector<KeyFrame *> KeyFrame::GetVectorCovisibleKeyFrames() {
        unique_lock<mutex> lock(mMutexConnections);
        return mvpOrderedConnectedKeyFrames;
    }

    vector<KeyFrame *> KeyFrame::GetBestCovisibilityKeyFrames(const int &N) {
        unique_lock<mutex> lock(mMutexConnections);
        if ((int) mvpOrderedConnectedKeyFrames.size() < N)
            return mvpOrderedConnectedKeyFrames;
        else
            return vector<KeyFrame *>(mvpOrderedConnectedKeyFrames.begin(), mvpOrderedConnectedKeyFrames.begin() + N);

    }

    vector<KeyFrame *> KeyFrame::GetCovisiblesByWeight(const int &w) {
        unique_lock<mutex> lock(mMutexConnections);

        if (mvpOrderedConnectedKeyFrames.empty())
            return vector<KeyFrame *>();

        vector<int>::iterator it = upper_bound(mvOrderedWeights.begin(), mvOrderedWeights.end(), w,
                                               KeyFrame::weightComp);
        if (it == mvOrderedWeights.end())
            return vector<KeyFrame *>();
        else {
            int n = it - mvOrderedWeights.begin();
            return vector<KeyFrame *>(mvpOrderedConnectedKeyFrames.begin(), mvpOrderedConnectedKeyFrames.begin() + n);
        }
    }

    int KeyFrame::GetWeight(KeyFrame *pKF) {
        unique_lock<mutex> lock(mMutexConnections);
        if (mConnectedKeyFrameWeights.count(pKF))
            return mConnectedKeyFrameWeights[pKF];
        else
            return 0;
    }

    void KeyFrame::AddMapPoint(MapPoint *pMP, const size_t &idx) {
        unique_lock<mutex> lock(mMutexFeatures);
        mvpMapPoints[idx] = pMP;
    }

    void KeyFrame::EraseMapPointMatch(const size_t &idx) {
        unique_lock<mutex> lock(mMutexFeatures);
        mvpMapPoints[idx] = static_cast<MapPoint *>(NULL);
    }

    void KeyFrame::EraseMapPointMatch(MapPoint *pMP) {
        int idx = pMP->GetIndexInKeyFrame(this);
        if (idx >= 0)
            mvpMapPoints[idx] = static_cast<MapPoint *>(NULL);
    }


    void KeyFrame::ReplaceMapPointMatch(const size_t &idx, MapPoint *pMP) {
        mvpMapPoints[idx] = pMP;
    }

    set<MapPoint *> KeyFrame::GetMapPoints() {
        unique_lock<mutex> lock(mMutexFeatures);
        set<MapPoint *> s;
        for (size_t i = 0, iend = mvpMapPoints.size(); i < iend; i++) {
            if (!mvpMapPoints[i])
                continue;
            MapPoint *pMP = mvpMapPoints[i];
            if (!pMP->isBad())
                s.insert(pMP);
        }
        return s;
    }

    int KeyFrame::TrackedMapPoints(const int &minObs) {
        unique_lock<mutex> lock(mMutexFeatures);

        int nPoints = 0;
        const bool bCheckObs = minObs > 0;
        for (int i = 0; i < N; i++) {
            MapPoint *pMP = mvpMapPoints[i];
            if (pMP) {
                if (!pMP->isBad()) {
                    if (bCheckObs) {
                        if (mvpMapPoints[i]->Observations() >= minObs)
                            nPoints++;
                    } else
                        nPoints++;
                }
            }
        }

        return nPoints;
    }

    vector<MapPoint *> KeyFrame::GetMapPointMatches() {
        unique_lock<mutex> lock(mMutexFeatures);
        return mvpMapPoints;
    }

    MapPoint *KeyFrame::GetMapPoint(const size_t &idx) {
        unique_lock<mutex> lock(mMutexFeatures);
        return mvpMapPoints[idx];
    }

    void KeyFrame::UpdateConnections() {
        map<KeyFrame *, int> KFcounter;

        vector<MapPoint *> vpMP;

        {
            unique_lock<mutex> lockMPs(mMutexFeatures);
            vpMP = mvpMapPoints;
        }

        //For all map points in keyframe check in which other keyframes are they seen
        //Increase counter for those keyframes
        for (vector<MapPoint *>::iterator vit = vpMP.begin(), vend = vpMP.end(); vit != vend; vit++) {
            MapPoint *pMP = *vit;

            if (!pMP)
                continue;

            if (pMP->isBad())
                continue;

            map<KeyFrame *, size_t> observations = pMP->GetObservations();

            for (map<KeyFrame *, size_t>::iterator mit = observations.begin(), mend = observations.end();
                 mit != mend; mit++) {
                if (mit->first->mnId == mnId)
                    continue;
                KFcounter[mit->first]++;
            }
        }

        // This should not happen
        if (KFcounter.empty())
            return;

        //If the counter is greater than threshold add connection
        //In case no keyframe counter is over threshold add the one with maximum counter
        // 共视超过15个点就被认为是有connection了。。。然而直接法中共视超多的
        int nmax = 0;
        KeyFrame *pKFmax = NULL;
        int th = 15;
        // int th = 30;

        vector<pair<int, KeyFrame *> > vPairs;
        vPairs.reserve(KFcounter.size());
        for (map<KeyFrame *, int>::iterator mit = KFcounter.begin(), mend = KFcounter.end(); mit != mend; mit++) {
            if (mit->second > nmax) {
                nmax = mit->second;
                pKFmax = mit->first;
            }
            if (mit->second >= th) {
                vPairs.push_back(make_pair(mit->second, mit->first));
                (mit->first)->AddConnection(this, mit->second);
            }
        }

        if (vPairs.empty()) {
            vPairs.push_back(make_pair(nmax, pKFmax));
            pKFmax->AddConnection(this, nmax);
        }

        sort(vPairs.begin(), vPairs.end());
        list < KeyFrame * > lKFs;
        list<int> lWs;
        for (size_t i = 0; i < vPairs.size(); i++) {
            lKFs.push_front(vPairs[i].second);
            lWs.push_front(vPairs[i].first);
        }

        {
            unique_lock<mutex> lockCon(mMutexConnections);

            // mspConnectedKeyFrames = spConnectedKeyFrames;
            mConnectedKeyFrameWeights = KFcounter;
            mvpOrderedConnectedKeyFrames = vector<KeyFrame *>(lKFs.begin(), lKFs.end());
            mvOrderedWeights = vector<int>(lWs.begin(), lWs.end());

            if (mbFirstConnection && mnId != 0) {
                mpParent = mvpOrderedConnectedKeyFrames.front();
                mpParent->AddChild(this);
                mbFirstConnection = false;
            }

        }
    }

    void KeyFrame::AddChild(KeyFrame *pKF) {
        unique_lock<mutex> lockCon(mMutexConnections);
        mspChildrens.insert(pKF);
    }

    void KeyFrame::EraseChild(KeyFrame *pKF) {
        unique_lock<mutex> lockCon(mMutexConnections);
        mspChildrens.erase(pKF);
    }

    void KeyFrame::ChangeParent(KeyFrame *pKF) {
        unique_lock<mutex> lockCon(mMutexConnections);
        mpParent = pKF;
        pKF->AddChild(this);
    }

    set<KeyFrame *> KeyFrame::GetChilds() {
        unique_lock<mutex> lockCon(mMutexConnections);
        return mspChildrens;
    }

    KeyFrame *KeyFrame::GetParent() {
        unique_lock<mutex> lockCon(mMutexConnections);
        return mpParent;
    }

    bool KeyFrame::hasChild(KeyFrame *pKF) {
        unique_lock<mutex> lockCon(mMutexConnections);
        return mspChildrens.count(pKF);
    }

    void KeyFrame::AddLoopEdge(KeyFrame *pKF) {
        unique_lock<mutex> lockCon(mMutexConnections);
        mbNotErase = true;
        mspLoopEdges.insert(pKF);
    }

    set<KeyFrame *> KeyFrame::GetLoopEdges() {
        unique_lock<mutex> lockCon(mMutexConnections);
        return mspLoopEdges;
    }

    void KeyFrame::SetNotErase() {
        unique_lock<mutex> lock(mMutexConnections);
        mbNotErase = true;
    }

    void KeyFrame::SetErase() {
        {
            unique_lock<mutex> lock(mMutexConnections);
            if (mspLoopEdges.empty()) {
                mbNotErase = false;
            }
        }

        if (mbToBeErased) {
            SetBadFlag();
        }
    }

    void KeyFrame::SetBadFlag() {
        // Test log
        if (mbBad) {
            vector<KeyFrame *> vKFinMap = mpMap->GetAllKeyFrames();
            std::set<KeyFrame *> KFinMap(vKFinMap.begin(), vKFinMap.end());
            if (KFinMap.count(this)) {
                mpMap->EraseKeyFrame(this);
            }
            mpKeyFrameDB->erase(this);
            return;
        }

        {
            unique_lock<mutex> lock(mMutexConnections);
            if (mnId == 0) {
                LOG(INFO) << "Don't erase the first key frame" << endl;
                return;
            } else if (mbNotErase) {
                mbToBeErased = true;
                return;
            }
        }

        for (map<KeyFrame *, int>::iterator mit = mConnectedKeyFrameWeights.begin(), mend = mConnectedKeyFrameWeights.end();
             mit != mend; mit++)
            mit->first->EraseConnection(this);

        for (size_t i = 0; i < mvpMapPoints.size(); i++)
            if (mvpMapPoints[i])
                mvpMapPoints[i]->EraseObservation(this);
        {
            unique_lock<mutex> lock(mMutexConnections);
            unique_lock<mutex> lock1(mMutexFeatures);

            mConnectedKeyFrameWeights.clear();
            mvpOrderedConnectedKeyFrames.clear();

            // Update Spanning Tree
            set<KeyFrame *> sParentCandidates;
            sParentCandidates.insert(mpParent);

            // Assign at each iteration one children with a parent (the pair with highest covisibility weight)
            // Include that children as new parent candidate for the rest
            while (!mspChildrens.empty()) {
                bool bContinue = false;

                int max = -1;
                KeyFrame *pC;
                KeyFrame *pP;

                for (set<KeyFrame *>::iterator sit = mspChildrens.begin(), send = mspChildrens.end();
                     sit != send; sit++) {
                    KeyFrame *pKF = *sit;
                    if (pKF->isBad())
                        continue;

                    // Check if a parent candidate is connected to the keyframe
                    vector<KeyFrame *> vpConnected = pKF->GetVectorCovisibleKeyFrames();
                    for (size_t i = 0, iend = vpConnected.size(); i < iend; i++) {
                        for (set<KeyFrame *>::iterator spcit = sParentCandidates.begin(), spcend = sParentCandidates.end();
                             spcit != spcend; spcit++) {
                            if (vpConnected[i]->mnId == (*spcit)->mnId) {
                                int w = pKF->GetWeight(vpConnected[i]);
                                if (w > max) {
                                    pC = pKF;
                                    pP = vpConnected[i];
                                    max = w;
                                    bContinue = true;
                                }
                            }
                        }
                    }
                }

                if (bContinue) {
                    pC->ChangeParent(pP);
                    sParentCandidates.insert(pC);
                    mspChildrens.erase(pC);
                } else
                    break;
            }

            // If a children has no covisibility links with any parent candidate, assign to the original parent of this KF
            if (!mspChildrens.empty())
                for (set<KeyFrame *>::iterator sit = mspChildrens.begin(); sit != mspChildrens.end(); sit++) {
                    (*sit)->ChangeParent(mpParent);
                }

            mpParent->EraseChild(this);
            mTcp = Tcw * mpParent->GetPoseInverse();

            // Update Prev/Next KeyFrame for prev/next
            KeyFrame *pPrevKF = GetPrevKeyFrame();
            KeyFrame *pNextKF = GetNextKeyFrame();
            if (pPrevKF)
                pPrevKF->SetNextKeyFrame(pNextKF);
            if (pNextKF)
                pNextKF->SetPrevKeyFrame(pPrevKF);
            SetPrevKeyFrame(NULL);
            SetNextKeyFrame(NULL);
            // TODO
            if (pPrevKF && pNextKF) {
                // Update IMUData for NextKF
                pNextKF->AppendIMUDataToFront(this);
                // Re-compute pre-integrator
                pNextKF->ComputePreInt();
            }

            mbBad = true;
        }


        mpMap->EraseKeyFrame(this);
        mpKeyFrameDB->erase(this);
    }

    bool KeyFrame::isBad() {
        unique_lock<mutex> lock(mMutexConnections);
        return mbBad;
    }

    void KeyFrame::EraseConnection(KeyFrame *pKF) {
        bool bUpdate = false;
        {
            unique_lock<mutex> lock(mMutexConnections);
            if (mConnectedKeyFrameWeights.count(pKF)) {
                mConnectedKeyFrameWeights.erase(pKF);
                bUpdate = true;
            }
        }

        if (bUpdate)
            UpdateBestCovisibles();
    }

    vector<size_t> KeyFrame::GetFeaturesInArea(const float &x, const float &y, const float &r) const {
        vector<size_t> vIndices;
        vIndices.reserve(N);

        const int nMinCellX = max(0, (int) floor((x - mnMinX - r) * mfGridElementWidthInv));
        if (nMinCellX >= mnGridCols)
            return vIndices;

        const int nMaxCellX = min((int) mnGridCols - 1, (int) ceil((x - mnMinX + r) * mfGridElementWidthInv));
        if (nMaxCellX < 0)
            return vIndices;

        const int nMinCellY = max(0, (int) floor((y - mnMinY - r) * mfGridElementHeightInv));
        if (nMinCellY >= mnGridRows)
            return vIndices;

        const int nMaxCellY = min((int) mnGridRows - 1, (int) ceil((y - mnMinY + r) * mfGridElementHeightInv));
        if (nMaxCellY < 0)
            return vIndices;

        for (int ix = nMinCellX; ix <= nMaxCellX; ix++) {
            for (int iy = nMinCellY; iy <= nMaxCellY; iy++) {
                const vector<size_t> vCell = mGrid[ix][iy];
                for (size_t j = 0, jend = vCell.size(); j < jend; j++) {
                    const cv::KeyPoint &kpUn = mvKeys[vCell[j]];
                    const float distx = kpUn.pt.x - x;
                    const float disty = kpUn.pt.y - y;

                    if (fabs(distx) < r && fabs(disty) < r)
                        vIndices.push_back(vCell[j]);
                }
            }
        }

        return vIndices;
    }

    bool KeyFrame::IsInImage(const float &x, const float &y) const {
        return (x >= mnMinX && x < mnMaxX && y >= mnMinY && y < mnMaxY);
    }

    Vector3f KeyFrame::UnprojectStereo(int i) {
        const float z = mvDepth[i];
        if (z > 0) {
            const float u = mvKeys[i].pt.x;
            const float v = mvKeys[i].pt.y;
            const float x = (u - cx) * z * invfx;
            const float y = (v - cy) * z * invfy;
            Vector3f x3Dc(x, y, z);

            unique_lock<mutex> lock(mMutexPose);
            return Twc * x3Dc;
        } else
            return Vector3f(0, 0, 0);
    }

    float KeyFrame::ComputeSceneMedianDepth(const int q) {
        vector<MapPoint *> vpMapPoints;
        SE3f Tcw_;
        {
            unique_lock<mutex> lock(mMutexFeatures);
            unique_lock<mutex> lock2(mMutexPose);
            vpMapPoints = mvpMapPoints;
            Tcw_ = Tcw;
        }

        vector<float> vDepths;
        vDepths.reserve(N);
        Vector4f r = Tcw_.matrix().row(2);
        Vector3f Rcw2 = r.head<3>();
        float zcw = Tcw_.matrix()(2, 3);

        for (int i = 0; i < N; i++) {
            if (mvpMapPoints[i]) {
                MapPoint *pMP = mvpMapPoints[i];
                Vector3f x3Dw = pMP->GetWorldPos();
                float z = Rcw2.dot(x3Dw) + zcw;
                vDepths.push_back(z);
            }
        }
        sort(vDepths.begin(), vDepths.end());

        return vDepths[(vDepths.size() - 1) / q];
    }

} //namespace ygz
