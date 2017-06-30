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

#include "LocalMapping.h"
#include "LoopClosing.h"
#include "ORBmatcher.h"
#include "Optimizer.h"
#include "Converter.h"

#include <iomanip>

namespace ygz {

    class KeyFrameInit {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        double mTimeStamp;
        KeyFrameInit *mpPrevKeyFrame;
        SE3d Twc;
        IMUPreintegrator mIMUPreInt;
        std::vector<IMUData> mvIMUData;
        Vector3d bg;


        KeyFrameInit(KeyFrame &kf) :
                mTimeStamp(kf.mTimeStamp), mpPrevKeyFrame(NULL), Twc(kf.GetPoseInverse().cast<double>()),
                mIMUPreInt(kf.GetIMUPreInt()), mvIMUData(kf.GetVectorIMUData()), bg(0, 0, 0) {
        }

        void ComputePreInt(void) {
            if (mpPrevKeyFrame == NULL) {
                return;
            } else {
                // Reset pre-integrator first
                mIMUPreInt.reset();

                if (mvIMUData.empty())
                    return;

                // remember to consider the gap between the last KF and the first IMU
                {
                    const IMUData &imu = mvIMUData.front();
                    double dt = std::max(0., imu._t - mpPrevKeyFrame->mTimeStamp);
                    mIMUPreInt.update(imu._g - bg, imu._a, dt);  // Acc bias not considered here
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
                    mIMUPreInt.update(imu._g - bg, imu._a, dt);
                }
            }
        }

    };

    bool LocalMapping::GetVINSIniting(void) {
        unique_lock<mutex> lock(mMutexVINSIniting);
        return mbVINSIniting;
    }

    void LocalMapping::SetVINSIniting(bool flag) {
        unique_lock<mutex> lock(mMutexVINSIniting);
        mbVINSIniting = flag;
    }

    bool LocalMapping::GetResetVINSInit(void) {
        unique_lock<mutex> lock(mMutexVINSIniting);
        return mbResetVINSInit;
    }

    bool LocalMapping::SetResetVINSInit(bool flag) {
        unique_lock<mutex> lock(mMutexVINSIniting);
        mbResetVINSInit = flag;
        return true;
    }

    bool LocalMapping::GetUpdatingInitPoses(void) {
        unique_lock<mutex> lock(mMutexUpdatingInitPoses);
        return mbUpdatingInitPoses;
    }

    void LocalMapping::SetUpdatingInitPoses(bool flag) {
        unique_lock<mutex> lock(mMutexUpdatingInitPoses);
        mbUpdatingInitPoses = flag;
    }

    KeyFrame *LocalMapping::GetMapUpdateKF() {
        unique_lock<mutex> lock(mMutexMapUpdateFlag);
        return mpMapUpdateKF;
    }

    bool LocalMapping::GetMapUpdateFlagForTracking() {
        unique_lock<mutex> lock(mMutexMapUpdateFlag);
        return mbMapUpdateFlagForTracking;
    }

    void LocalMapping::SetMapUpdateFlagInTracking(bool bflag) {
        unique_lock<mutex> lock(mMutexMapUpdateFlag);
        mbMapUpdateFlagForTracking = bflag;
        if (bflag) {
            mpMapUpdateKF = mpCurrentKeyFrame;
        }
    }

    bool LocalMapping::GetVINSInited(void) {
        unique_lock<mutex> lock(mMutexVINSInitFlag);
        return mbVINSInited;
    }

    void LocalMapping::SetVINSInited(bool flag) {
        unique_lock<mutex> lock(mMutexVINSInitFlag);
        mbVINSInited = flag;
    }

    bool LocalMapping::GetFirstVINSInited(void) {
        unique_lock<mutex> lock(mMutexFirstVINSInitFlag);
        return mbFirstVINSInited;
    }

    void LocalMapping::SetFirstVINSInited(bool flag) {
        unique_lock<mutex> lock(mMutexFirstVINSInitFlag);
        mbFirstVINSInited = flag;
        LOG(INFO) << "set first vins inited : " << flag << endl;
    }

    Vector3d LocalMapping::GetGravityVec() {
        return mGravityVec;
    }

    void LocalMapping::VINSInitThread() {
        unsigned long initedid = 0;
        cout << "Start VINSInitThread" << endl;
        SetVINSIniting(true);
        while (1) {
            if (KeyFrame::nNextId > 2)
                if (!GetVINSInited() && mpCurrentKeyFrame->mnId > initedid) {
                    initedid = mpCurrentKeyFrame->mnId;

                    bool tmpbool = TryInitVIO();
                    if (tmpbool) {
                        //SetFirstVINSInited(true);
                        //SetVINSInited(true);
                        cout << "VINS inited, quit VINS init thread" << endl;
                        break;
                    }
                }
            usleep(3000);
            if (isFinished()) {
                cout << "LocalMapping finished, quit VINS init thread" << endl;
                break;
            }
            if (GetResetVINSInit()) {
                SetResetVINSInit(false);
                cout << "Resetting, quit VINS init thread" << endl;
                break;
            }
        }
        SetVINSIniting(false);
        cout << "Quit VINSInitThread" << endl;
    }

    bool LocalMapping::TryInitVIO(void) {
        if (mpMap->KeyFramesInMap() <= mnLocalWindowSize)
            return false;

        static bool fopened = false;
        static ofstream fgw, fscale, fbiasa, fbiasg;
        if (!fopened) {
            fgw.open("/home/jp/opensourcecode/ORB_SLAM2/tmp/gw.txt");
            fscale.open("/home/jp/opensourcecode/ORB_SLAM2/tmp/scale.txt");
            fbiasa.open("/home/jp/opensourcecode/ORB_SLAM2/tmp/biasa.txt");
            fbiasg.open("/home/jp/opensourcecode/ORB_SLAM2/tmp/biasg.txt");
            if (fgw.is_open() && fscale.is_open() && fbiasa.is_open() && fbiasg.is_open())
                fopened = true;
            else {
                cerr << "file open error in TryInitVIO" << endl;
                fopened = false;
            }
            fgw << std::fixed << std::setprecision(6);
            fscale << std::fixed << std::setprecision(6);
            fbiasa << std::fixed << std::setprecision(6);
            fbiasg << std::fixed << std::setprecision(6);
        }

        Optimizer::GlobalBundleAdjustemnt(mpMap, 10);

        // Extrinsics
        SE3d Tbc = ConfigParam::GetSE3Tbc();
        SE3d Tcb = Tbc.inverse();
        Matrix3d Rcb = Tcb.rotationMatrix();
        Vector3d pcb = Tcb.translation();


        // Wait KeyFrame Culling.
        // 1. if KeyFrame Culling is running, wait until finished.
        // 2. if KFs are being copied, then don't run KeyFrame Culling (in KeyFrameCulling function)
        while (GetFlagCopyInitKFs()) {
            usleep(3000);
        }

        SetFlagCopyInitKFs(true);

        // Use all KeyFrames in map to compute
        vector<KeyFrame *> vScaleGravityKF = mpMap->GetAllKeyFrames();
        int N = vScaleGravityKF.size();
        KeyFrame *pNewestKF = vScaleGravityKF[N - 1];
        vector<SE3d> vTwc;
        vector<IMUPreintegrator> vIMUPreInt;
        // Store initialization-required KeyFrame data
        vector<KeyFrameInit *> vKFInit;

        for (int i = 0; i < N; i++) {
            KeyFrame *pKF = vScaleGravityKF[i];
            vTwc.push_back(pKF->GetPoseInverse().cast<double>());
            vIMUPreInt.push_back(pKF->GetIMUPreInt());
            KeyFrameInit *pkfi = new KeyFrameInit(*pKF);
            if (i != 0) {
                pkfi->mpPrevKeyFrame = vKFInit[i - 1];
            }
            vKFInit.push_back(pkfi);
        }

        SetFlagCopyInitKFs(false);

        // Step 1.
        // Try to compute initial gyro bias, using optimization with Gauss-Newton
        Vector3d bgest = Optimizer::OptimizeInitialGyroBias(vTwc, vIMUPreInt);
        //Vector3d bgest = Optimizer::OptimizeInitialGyroBias(vScaleGravityKF);
        LOG(WARNING) << "bgest: " << bgest.transpose() << endl;

        // Update biasg and pre-integration in LocalWindow. Remember to reset back to zero
        for (int i = 0; i < N; i++) {
            vKFInit[i]->bg = bgest;
        }
        for (int i = 0; i < N; i++) {
            vKFInit[i]->ComputePreInt();
        }

        // Solve A*x=B for x=[s,gw] 4x1 vector
        MatrixXd A(3 * (N - 2), 4);
        A.setZero();
        VectorXd B(3 * (N - 2));
        B.setZero();
        Matrix3d I3 = Matrix3d::Identity();

        // Step 2.
        // Approx Scale and Gravity vector in 'world' frame (first KF's camera frame)
        for (int i = 0; i < N - 2; i++) {
            const IMUPreintegrator &imupreint2 = vKFInit[i + 1]->mIMUPreInt;
            const IMUPreintegrator &imupreint3 = vKFInit[i + 2]->mIMUPreInt;
            // Delta time between frames
            double dt12 = imupreint2.getDeltaTime();//pKF2->GetIMUPreInt().getDeltaTime();
            double dt23 = imupreint3.getDeltaTime();//pKF3->GetIMUPreInt().getDeltaTime();
            // Pre-integrated measurements
            const Vector3d &dp12 = imupreint2.getDeltaP();
            const Vector3d &dv12 = imupreint2.getDeltaV();
            const Vector3d &dp23 = imupreint3.getDeltaP();

            // Pose of camera in world frame
            const SE3d &Twc1 = vTwc[i]; //pKF1->GetPoseInverse()
            const SE3d &Twc2 = vTwc[i + 1]; //pKF2->GetPoseInverse()
            const SE3d &Twc3 = vTwc[i + 2]; //pKF3->GetPoseInverse()
            // Position of camera center
            const Vector3d &pc1 = Twc1.translation();
            const Vector3d &pc2 = Twc2.translation();
            const Vector3d &pc3 = Twc3.translation();
            // Rotation of camera, Rwc
            const Matrix3d Rc1 = Twc1.rotationMatrix();
            const Matrix3d Rc2 = Twc2.rotationMatrix();
            const Matrix3d Rc3 = Twc3.rotationMatrix();

            // Stack to A/B matrix
            // lambda*s + beta*g = gamma
            Vector3d lambda = (pc2 - pc1) * dt23 + (pc2 - pc3) * dt12;
            Matrix3d beta = 0.5 * I3 * (dt12 * dt12 * dt23 + dt12 * dt23 * dt23);
            Vector3d gamma = (Rc3 - Rc2) * pcb * dt12 + (Rc1 - Rc2) * pcb * dt23 + Rc1 * Rcb * dp12 * dt23 -
                             Rc2 * Rcb * dp23 * dt12 - Rc1 * Rcb * dv12 * dt12 * dt23;
            A.block<3, 1>(3 * i, 0) = lambda;
            A.block<3, 3>(3 * i, 1) = beta;
            B.segment<3>(3 * i) = gamma;
            // Tested the formulation in paper, -gamma. Then the scale and gravity vector is -xx
        }
        // Use svd to compute A*x=B, x=[s,gw] 4x1 vector
        JacobiSVD<MatrixXd> svd1(A, ComputeThinU | ComputeThinV);
        VectorXd x = svd1.solve(B);

        //    // x=[s,gw] 4x1 vector
        double sstar = x[0];    // scale should be positive
        //    cv::Mat gwstar = x.rowRange(1,4);   // gravity should be about ~9.8
        Vector3d gwstar = x.segment<3>(1);

        LOG(WARNING) << x.transpose() << ", gw:" << gwstar.transpose() << ", |gw|=" << gwstar.norm() << endl;


        // Step 3.
        // Use gravity magnitude 9.8 as constraint
        // gI = [0;0;1], the normalized gravity vector in an inertial frame, NED type with no orientation.
        Vector3d gI(0, 0, 1.0);
        Vector3d GI = gI * ConfigParam::GetG();
        // Normalized approx. gravity vecotr in world frame
        Vector3d gwn = gwstar / gwstar.norm();

        // vhat = (gI x gw) / |gI x gw|
        Vector3d gIxgwn = gI.cross(gwn);
        double normgIxgwn = gIxgwn.norm();
        Vector3d vhat = gIxgwn / normgIxgwn;
        double theta = std::atan2(normgIxgwn, gI.dot(gwn));
        Matrix3d Rwi = SO3::exp(vhat * theta).matrix();

        // Solve C*x=D for x=[s,dthetaxy,ba] (1+2+3)x1 vector
        MatrixXd C(3 * (N - 2), 6);
        C.setZero();
        VectorXd D(3 * (N - 2));
        D.setZero();

        for (int i = 0; i < N - 2; i++) {
            const IMUPreintegrator &imupreint2 = vKFInit[i + 1]->mIMUPreInt;
            const IMUPreintegrator &imupreint3 = vKFInit[i + 2]->mIMUPreInt;
            // Delta time between frames
            double dt12 = imupreint2.getDeltaTime();
            double dt23 = imupreint3.getDeltaTime();
            // Pre-integrated measurements
            const Vector3d &dp12 = imupreint2.getDeltaP();
            const Vector3d &dv12 = imupreint2.getDeltaV();
            const Vector3d &dp23 = imupreint3.getDeltaP();
            const Matrix3d &Jpba12 = imupreint2.getJPBiasa();
            const Matrix3d &Jvba12 = imupreint2.getJVBiasa();
            const Matrix3d &Jpba23 = imupreint3.getJPBiasa();
            // Pose of camera in world frame
            const SE3d &Twc1 = vTwc[i]; //pKF1->GetPoseInverse()
            const SE3d &Twc2 = vTwc[i + 1]; //pKF2->GetPoseInverse()
            const SE3d &Twc3 = vTwc[i + 2]; //pKF3->GetPoseInverse()
            // Position of camera center
            const Vector3d &pc1 = Twc1.translation();
            const Vector3d &pc2 = Twc2.translation();
            const Vector3d &pc3 = Twc3.translation();
            // Rotation of camera, Rwc
            const Matrix3d Rc1 = Twc1.rotationMatrix();
            const Matrix3d Rc2 = Twc2.rotationMatrix();
            const Matrix3d Rc3 = Twc3.rotationMatrix();
            // Stack to C/D matrix
            // lambda*s + phi*dthetaxy + zeta*ba = psi
            Vector3d lambda = (pc2 - pc1) * dt23 + (pc2 - pc3) * dt12;
            Matrix3d phi = -0.5 * (dt12 * dt12 * dt23 + dt12 * dt23 * dt23) * Rwi * SO3::hat(GI);
            Matrix3d zeta = Rc2 * Rcb * Jpba23 * dt12
                            + Rc1 * Rcb * Jvba12 * dt12 * dt23
                            - Rc1 * Rcb * Jpba12 * dt23;
            Vector3d psi = (Rc1 - Rc2) * pcb * dt23 + Rc1 * Rcb * dp12 * dt23 - (Rc2 - Rc3) * pcb * dt12
                           - Rc2 * Rcb * dp23 * dt12 - Rc1 * Rcb * dv12 * dt23 * dt12 -
                           0.5 * Rwi * GI * (dt12 * dt12 * dt23 + dt12 * dt23 * dt23);
            C.block<3, 1>(3 * i, 0) = lambda;
            C.block<3, 2>(3 * i, 1) = phi.block<3, 2>(0, 0);
            C.block<3, 3>(3 * i, 3) = zeta;
            D.segment<3>(3 * i) = psi;
        }

        // Use svd to compute C*x=D, x=[s,dthetaxy,ba] 6x1 vector
        JacobiSVD<MatrixXd> svd2(C, ComputeThinU | ComputeThinV);
        VectorXd y = svd2.solve(D);

        double s_ = y(0);
        Matrix<double, 2, 1> dthetaxy = y.segment<2>(1);
        Vector3d dbiasa_ = y.segment<3>(3); //y.rowRange(3,6);

        LOG(WARNING) << y.transpose() << ", s:" << s_ << ", dthetaxy:" << dthetaxy.transpose() << ", dbiasa:"
                     << dbiasa_.transpose() << endl;

        // dtheta = [dx;dy;0]
        Vector3d dtheta(0., 0., 0.);
        dtheta.head(2) = dthetaxy;
        // Rwi_ = Rwi*exp(dtheta)
        Eigen::Matrix3d Rwi_ = Rwi * SO3::exp(dtheta).matrix();
        Vector3d gw = Rwi_ * GI;
        LOG(WARNING) << "gw: " << gw.transpose() << ", |gw|=" << gw.norm() << endl;

        // Debug log
        if (fopened) {
            cout << "Time: " << mpCurrentKeyFrame->mTimeStamp - mnStartTime << ", sstar: " << sstar << ", s: " << s_
                 << endl;

            fgw << mpCurrentKeyFrame->mTimeStamp << " "
                << gw[0] << " " << gw[1] << " " << gw[2] << " "
                << gwstar[0] << " " << gwstar[1] << " " << gwstar[2] << " "
                << endl;
            fscale << mpCurrentKeyFrame->mTimeStamp << " "
                   << s_ << " " << sstar << " " << endl;
            fbiasa << mpCurrentKeyFrame->mTimeStamp << " "
                   << dbiasa_(0) << " " << dbiasa_(1) << " " << dbiasa_(2) << " " << endl;
            fbiasg << mpCurrentKeyFrame->mTimeStamp << " "
                   << bgest(0) << " " << bgest(1) << " " << bgest(2) << " " << endl;
        }


        // ********************************
        // TODO(jingpang): Add some logic or strategy to confirm initialization status
        bool bVIOInited = false;
        if (mbFirstTry) {
            mbFirstTry = false;
            mnStartTime = mpCurrentKeyFrame->mTimeStamp;
        }
        if (pNewestKF->mTimeStamp - mnStartTime >= ConfigParam::GetVINSInitTime()) {
            bVIOInited = true;
        }
        LOG(INFO) << pNewestKF->mTimeStamp << ", " << mpCurrentKeyFrame->mTimeStamp << ", time elapsed: "
                  << pNewestKF->mTimeStamp - mnStartTime << endl;
        // When failed. Or when you're debugging.
        // Reset biasg to zero, and re-compute imu-preintegrator.
        if (!bVIOInited) {
        } else {
            // Set NavState , scale and bias for all KeyFrames
            // Scale
            double scale = s_;
            mnVINSInitScale = s_;
            // gravity vector in world frame
            mGravityVec = gw;

            // Update NavState for the KeyFrames not in vScaleGravityKF
            // Update Tcw-type pose for these KeyFrames, need mutex lock

            // Stop local mapping, and
            RequestStop();

            // Wait until Local Mapping has effectively stopped
            while (!isStopped() && !isFinished()) {
                usleep(1000);
            }

            SetUpdatingInitPoses(true);
            {
                unique_lock<mutex> lock(mpMap->mMutexMapUpdate);

                for (vector<KeyFrame *>::const_iterator vit = vScaleGravityKF.begin(), vend = vScaleGravityKF.end();
                     vit != vend; vit++) {
                    KeyFrame *pKF = *vit;
                    if (pKF->isBad())
                        continue;
                    // Position and rotation of visual SLAM
                    const Vector3d wPc = pKF->GetPoseInverse().translation().cast<double>();                   // wPc
                    const Matrix3d Rwc = pKF->GetPoseInverse().rotationMatrix().cast<double>();            // Rwc
                    // Set position and rotation of navstate
                    const Vector3d wPb = scale * wPc + Rwc * pcb;
                    pKF->SetNavStatePos(wPb);
                    pKF->SetNavStateRot(Rwc * Rcb);
                    // Update bias of Gyr & Acc
                    pKF->SetNavStateBiasGyr(bgest);
                    pKF->SetNavStateBiasAcc(dbiasa_);
                    // Set delta_bias to zero. (only updated during optimization)
                    pKF->SetNavStateDeltaBg(Vector3d::Zero());
                    pKF->SetNavStateDeltaBa(Vector3d::Zero());
                    // Step 4.
                    // compute velocity
                    if (pKF != vScaleGravityKF.back()) {
                        KeyFrame *pKFnext = pKF->GetNextKeyFrame();
                        // IMU pre-int between pKF ~ pKFnext
                        const IMUPreintegrator &imupreint = pKFnext->GetIMUPreInt();
                        // Time from this(pKF) to next(pKFnext)
                        double dt = imupreint.getDeltaTime();                                       // deltaTime
                        const Vector3d &dp = imupreint.getDeltaP();       // deltaP
                        const Matrix3d &Jpba = imupreint.getJPBiasa();    // J_deltaP_biasa
                        const Vector3d wPcnext = pKFnext->GetPoseInverse().translation().cast<double>();           // wPc next
                        const Matrix3d Rwcnext = pKFnext->GetPoseInverse().rotationMatrix().cast<double>();    // Rwc next

                        Eigen::Vector3d vel = -1. / dt * (scale * (wPc - wPcnext) + (Rwc - Rwcnext) * pcb +
                                                          Rwc * Rcb * (dp + Jpba * dbiasa_) + 0.5 * gw * dt * dt);
                        pKF->SetNavStateVel(vel);
                    } else {
                        // If this is the last KeyFrame, no 'next' KeyFrame exists
                        KeyFrame *pKFprev = pKF->GetPrevKeyFrame();
                        const IMUPreintegrator &imupreint_prev_cur = pKF->GetIMUPreInt();
                        double dt = imupreint_prev_cur.getDeltaTime();
                        Eigen::Matrix3d Jvba = imupreint_prev_cur.getJVBiasa();
                        Eigen::Vector3d dv = imupreint_prev_cur.getDeltaV();
                        Eigen::Vector3d velpre = pKFprev->GetNavState().Get_V();
                        Eigen::Matrix3d rotpre = pKFprev->GetNavState().Get_RotMatrix();
                        Eigen::Vector3d veleig = velpre + gw * dt + rotpre * (dv + Jvba * dbiasa_);
                        pKF->SetNavStateVel(veleig);
                    }
                }

                // Re-compute IMU pre-integration at last.
                for (vector<KeyFrame *>::const_iterator vit = vScaleGravityKF.begin(), vend = vScaleGravityKF.end();
                     vit != vend; vit++) {
                    KeyFrame *pKF = *vit;
                    if (pKF->isBad())
                        continue;
                    pKF->ComputePreInt();
                }

                // Update poses (multiply metric scale)
                vector<KeyFrame *> mspKeyFrames = mpMap->GetAllKeyFrames();
                for (std::vector<KeyFrame *>::iterator sit = mspKeyFrames.begin(), send = mspKeyFrames.end();
                     sit != send; sit++) {
                    KeyFrame *pKF = *sit;
                    SE3f Tcw = pKF->GetPose();
                    Tcw.translation() *= scale;
                    pKF->SetPose(Tcw);
                }
                vector<MapPoint *> mspMapPoints = mpMap->GetAllMapPoints();
                for (std::vector<MapPoint *>::iterator sit = mspMapPoints.begin(), send = mspMapPoints.end();
                     sit != send; sit++) {
                    MapPoint *pMP = *sit;
                    pMP->UpdateScale(scale);
                }
                LOG(INFO) << std::endl << "... Map scale updated ..." << std::endl << std::endl;

                // Update NavStates
                if (pNewestKF != mpCurrentKeyFrame) {
                    KeyFrame *pKF;

                    // step1. bias&d_bias
                    pKF = pNewestKF;
                    do {
                        pKF = pKF->GetNextKeyFrame();

                        // Update bias of Gyr & Acc
                        pKF->SetNavStateBiasGyr(bgest);
                        pKF->SetNavStateBiasAcc(dbiasa_);
                        // Set delta_bias to zero. (only updated during optimization)
                        pKF->SetNavStateDeltaBg(Eigen::Vector3d::Zero());
                        pKF->SetNavStateDeltaBa(Eigen::Vector3d::Zero());
                    } while (pKF != mpCurrentKeyFrame);

                    // step2. re-compute pre-integration
                    pKF = pNewestKF;
                    do {
                        pKF = pKF->GetNextKeyFrame();

                        pKF->ComputePreInt();
                    } while (pKF != mpCurrentKeyFrame);

                    // step3. update pos/rot
                    pKF = pNewestKF;
                    do {
                        pKF = pKF->GetNextKeyFrame();

                        // Update rot/pos
                        // Position and rotation of visual SLAM
                        Vector3d wPc = pKF->GetPoseInverse().translation().cast<double>();                   // wPc
                        Matrix3d Rwc = pKF->GetPoseInverse().rotationMatrix().cast<double>();            // Rwc
                        Vector3d wPb = wPc + Rwc * pcb;
                        pKF->SetNavStatePos(wPb);
                        pKF->SetNavStateRot(Rwc * Rcb);

                        if (pKF != mpCurrentKeyFrame) {
                            KeyFrame *pKFnext = pKF->GetNextKeyFrame();
                            // IMU pre-int between pKF ~ pKFnext
                            const IMUPreintegrator &imupreint = pKFnext->GetIMUPreInt();
                            // Time from this(pKF) to next(pKFnext)
                            double dt = imupreint.getDeltaTime();                                       // deltaTime
                            const Vector3d &dp = imupreint.getDeltaP();       // deltaP
                            const Matrix3d &Jpba = imupreint.getJPBiasa();    // J_deltaP_biasa
                            const Vector3d wPcnext = pKFnext->GetPoseInverse().translation().cast<double>();           // wPc next
                            const Matrix3d Rwcnext = pKFnext->GetPoseInverse().rotationMatrix().cast<double>();    // Rwc next

                            Vector3d vel = -1. / dt * ((wPc - wPcnext) + (Rwc - Rwcnext) * pcb +
                                                       Rwc * Rcb * (dp + Jpba * dbiasa_) + 0.5 * gw * dt * dt);
                            pKF->SetNavStateVel(vel);
                        } else {
                            // If this is the last KeyFrame, no 'next' KeyFrame exists
                            KeyFrame *pKFprev = pKF->GetPrevKeyFrame();
                            const IMUPreintegrator &imupreint_prev_cur = pKF->GetIMUPreInt();
                            double dt = imupreint_prev_cur.getDeltaTime();
                            Eigen::Matrix3d Jvba = imupreint_prev_cur.getJVBiasa();
                            Eigen::Vector3d dv = imupreint_prev_cur.getDeltaV();
                            //
                            Eigen::Vector3d velpre = pKFprev->GetNavState().Get_V();
                            Eigen::Matrix3d rotpre = pKFprev->GetNavState().Get_RotMatrix();
                            Eigen::Vector3d veleig = velpre + gw * dt + rotpre * (dv + Jvba * dbiasa_);
                            pKF->SetNavStateVel(veleig);
                        }

                    } while (pKF != mpCurrentKeyFrame);

                }

                LOG(INFO) << std::endl << "... Map NavState updated ..." << std::endl << std::endl;

                SetFirstVINSInited(true);
                SetVINSInited(true);

            }
            SetUpdatingInitPoses(false);

            // Release LocalMapping
            Release();


            // Run global BA after inited
            unsigned long nGBAKF = mpCurrentKeyFrame->mnId;
            Optimizer::GlobalBundleAdjustmentNavState(mpMap, mGravityVec, 10, NULL, nGBAKF, false);
            LOG(INFO) << "finish global BA after vins init" << endl;

            // Update pose
            // Stop local mapping, and
            RequestStop();
            LOG(INFO) << "request local mapping stop" << endl;
            // Wait until Local Mapping has effectively stopped
            while (!isStopped() && !isFinished()) {
                usleep(1000);
            }
            LOG(INFO) << "local mapping stopped" << endl;

            {
                unique_lock<mutex> lock(mpMap->mMutexMapUpdate);

                // Correct keyframes starting at map first keyframe
                list < KeyFrame * > lpKFtoCheck(mpMap->mvpKeyFrameOrigins.begin(), mpMap->mvpKeyFrameOrigins.end());

                while (!lpKFtoCheck.empty()) {
                    KeyFrame *pKF = lpKFtoCheck.front();
                    const set<KeyFrame *> sChilds = pKF->GetChilds();
                    SE3d Twc = pKF->GetPoseInverse().cast<double>();
                    for (set<KeyFrame *>::const_iterator sit = sChilds.begin(); sit != sChilds.end(); sit++) {
                        KeyFrame *pChild = *sit;
                        if (pChild->mnBAGlobalForKF != nGBAKF) {
                            LOG(INFO) << "correct KF after gBA in VI init: " << pChild->mnId << endl;
                            SE3f Tchildc = pChild->GetPose() * Twc.cast<float>();
                            pChild->mTcwGBA = Tchildc * pKF->mTcwGBA;//*Tcorc*pKF->mTcwGBA;
                            pChild->mnBAGlobalForKF = nGBAKF;

                            // Set NavStateGBA and correct the P/V/R
                            pChild->mNavStateGBA = pChild->GetNavState();
                            SE3d TwbGBA = (Tbc * pChild->mTcwGBA.cast<double>()).inverse();
                            Matrix3d RwbGBA = TwbGBA.rotationMatrix();
                            Vector3d PwbGBA = TwbGBA.translation();
                            Matrix3d Rw1 = pChild->mNavStateGBA.Get_RotMatrix();
                            Vector3d Vw1 = pChild->mNavStateGBA.Get_V();
                            Vector3d Vw2 = RwbGBA * Rw1.transpose() *
                                           Vw1;   // bV1 = bV2 ==> Rwb1^T*wV1 = Rwb2^T*wV2 ==> wV2 = Rwb2*Rwb1^T*wV1
                            pChild->mNavStateGBA.Set_Pos(PwbGBA);
                            pChild->mNavStateGBA.Set_Rot(RwbGBA);
                            pChild->mNavStateGBA.Set_Vel(Vw2);
                        }
                        lpKFtoCheck.push_back(pChild);
                    }

                    pKF->mTcwBefGBA = pKF->GetPose();
                    //pKF->SetPose(pKF->mTcwGBA);
                    pKF->mNavStateBefGBA = pKF->GetNavState();
                    pKF->SetNavState(pKF->mNavStateGBA);
                    pKF->UpdatePoseFromNS(Tbc);

                    lpKFtoCheck.pop_front();
                }

                // Correct MapPoints
                const vector<MapPoint *> vpMPs = mpMap->GetAllMapPoints();

                for (size_t i = 0; i < vpMPs.size(); i++) {
                    MapPoint *pMP = vpMPs[i];

                    if (pMP->isBad())
                        continue;

                    if (pMP->mnBAGlobalForKF == nGBAKF) {
                        // If optimized by Global BA, just update
                        pMP->SetWorldPos(pMP->mPosGBA);
                    } else {
                        // Update according to the correction of its reference keyframe
                        KeyFrame *pRefKF = pMP->GetReferenceKeyFrame();

                        if (pRefKF->mnBAGlobalForKF != nGBAKF)
                            continue;

                        // Map to non-corrected camera
                        Matrix3f Rcw = pRefKF->mTcwBefGBA.rotationMatrix();
                        Vector3f tcw = pRefKF->mTcwBefGBA.translation();
                        Vector3f Xc = Rcw * pMP->GetWorldPos() + tcw;

                        // Backproject using corrected camera
                        SE3f Twc = pRefKF->GetPoseInverse();
                        Matrix3f Rwc = Twc.rotationMatrix();
                        Vector3f twc = Twc.translation();

                        pMP->SetWorldPos(Rwc * Xc + twc);
                    }
                }

                cout << "Map updated!" << endl;

                // Map updated, set flag for Tracking
                SetMapUpdateFlagInTracking(true);

                // Release LocalMapping
                Release();
            }

        }

        for (int i = 0; i < N; i++) {
            if (vKFInit[i])
                delete vKFInit[i];
        }

        return bVIOInited;
    }

    void LocalMapping::AddToLocalWindow(KeyFrame *pKF) {
        mlLocalKeyFrames.push_back(pKF);
        if (mlLocalKeyFrames.size() > mnLocalWindowSize) {
            mlLocalKeyFrames.pop_front();
        }
    }

    void LocalMapping::DeleteBadInLocalWindow(void) {
        std::list<KeyFrame *>::iterator lit = mlLocalKeyFrames.begin();
        while (lit != mlLocalKeyFrames.end()) {
            KeyFrame *pKF = *lit;
            if (!pKF) LOG(ERROR) << "pKF null in DeleteBadInLocalWindow?" << endl; //Test log
            if (pKF->isBad()) {
                lit = mlLocalKeyFrames.erase(lit);
            } else {
                lit++;
            }
        }
    }

    LocalMapping::LocalMapping(Map *pMap, const float bMonocular, ConfigParam *pParams) :
            mbMonocular(bMonocular), mbResetRequested(false), mbFinishRequested(false), mbFinished(true), mpMap(pMap),
            mbAbortBA(false), mbStopped(false), mbStopRequested(false), mbNotStop(false), mbAcceptKeyFrames(true) {
        mpParams = pParams;
        mnLocalWindowSize = mpParams->GetLocalWindowSize();
        mbUseIMU = mpParams->GetUseIMUFlag();
        LOG(INFO) << "mnLocalWindowSize:" << mnLocalWindowSize << ", mbUseIMU:" << mbUseIMU << endl;

        mbVINSInited = false;
        mbFirstTry = true;
        mbFirstVINSInited = false;
        mbVINSIniting = false;
        mbResetVINSInit = false;

        mbUpdatingInitPoses = false;
        mbCopyInitKFs = false;

        //Thread for VINS initialization
        if (mbUseIMU)
            mptLocalMappingVIOInit = new thread(&LocalMapping::VINSInitThread, this);
        else
            mptLocalMappingVIOInit = NULL;
    }

    void LocalMapping::SetLoopCloser(LoopClosing *pLoopCloser) {
        mpLoopCloser = pLoopCloser;
    }

    void LocalMapping::SetTracker(Tracking *pTracker) {
        mpTracker = pTracker;
    }

    void LocalMapping::Run() {

        mbFinished = false;

        while (1) {
            // Tracking will see that Local Mapping is busy
            SetAcceptKeyFrames(false);

            // Check if there are keyframes in the queue
            if (CheckNewKeyFrames()) {
                // BoW conversion and insertion in Map
                // VI-A keyframe insertion
                ProcessNewKeyFrame();

                // Check recent MapPoints
                // VI-B recent map points culling
                MapPointCulling();

                // Triangulate new MapPoints
                // VI-C new map points creation
                CreateNewMapPoints();

                if (!CheckNewKeyFrames()) {
                    // Find more matches in neighbor keyframes and fuse point duplications
                    SearchInNeighbors();
                }

                mbAbortBA = false;

                // 已经处理完队列中的最后的一个关键帧，并且闭环检测没有请求停止LocalMapping
                if (!CheckNewKeyFrames() && !stopRequested()) {
                    // VI-D Local BA
                    if (mpMap->KeyFramesInMap() > 2) {
                        if (!mbUseIMU || (mbUseIMU && !GetVINSInited())) {
                            Optimizer::LocalBundleAdjustment(mpCurrentKeyFrame, &mbAbortBA, mpMap, this);
                        } else {
                            Optimizer::LocalBundleAdjustmentNavState(mpCurrentKeyFrame, mlLocalKeyFrames, &mbAbortBA,
                                                                     mpMap, mGravityVec, this);
                            SetMapUpdateFlagInTracking(true);
                        }

                    }

                    //HERE, need to add a thread for Visual-Inertial initialization

                    // Check redundant local Keyframes
                    // 检测并剔除当前帧相邻的关键帧中冗余的关键帧
                    // 剔除的标准是：该关键帧的90%的MapPoints可以被其它关键帧观测到
                    // trick!
                    // Tracking中先把关键帧交给LocalMapping线程
                    // 并且在Tracking中InsertKeyFrame函数的条件比较松，交给LocalMapping线程的关键帧会比较密
                    // 在这里再删除冗余的关键帧
                    KeyFrameCulling();
                }

                mpLoopCloser->InsertKeyFrame(mpCurrentKeyFrame);
            } else if (Stop()) {
                // Safe area to stop
                while (isStopped() && !CheckFinish()) {
                    usleep(1000);
                }
                if (CheckFinish())
                    break;
            }

            ResetIfRequested();

            // Tracking will see that Local Mapping is busy
            SetAcceptKeyFrames(true);

            if (CheckFinish())
                break;

            usleep(1000);
        }

        SetFinish();
    }

    /**
     * @brief 插入关键帧
     *
     * 将关键帧插入到地图中，以便将来进行局部地图优化
     * 这里仅仅是将关键帧插入到列表中进行等待
     * @param pKF KeyFrame
     */
    void LocalMapping::InsertKeyFrame(KeyFrame *pKF) {
        unique_lock<mutex> lock(mMutexNewKFs);
        // 将关键帧插入到列表中
        mlNewKeyFrames.push_back(pKF);
        mbAbortBA = true;
    }

    /**
     * @brief 查看列表中是否有等待被插入的关键帧
     * @return 如果存在，返回true
     */
    bool LocalMapping::CheckNewKeyFrames() {
        unique_lock<mutex> lock(mMutexNewKFs);
        return (!mlNewKeyFrames.empty());
    }

    /**
     * @brief 处理列表中的关键帧
     *
     * - 计算Bow
     * - 关联当前关键帧至MapPoints，并更新MapPoints的平均观测方向和观测距离范围
     * - 插入关键帧，更新Covisibility图和Ensenssial图
     */
    void LocalMapping::ProcessNewKeyFrame() {
        {
            unique_lock<mutex> lock(mMutexNewKFs);
            // 从列表中获得一个等待被插入的关键帧
            mpCurrentKeyFrame = mlNewKeyFrames.front();
            mlNewKeyFrames.pop_front();
        }

        // Associate MapPoints to the new keyframe and update normal and descriptor
        const vector<MapPoint *> vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();

        for (size_t i = 0; i < vpMapPointMatches.size(); i++) {
            MapPoint *pMP = vpMapPointMatches[i];
            if (pMP) {
                if (!pMP->isBad()) {
                    // NOTE
                    if (!pMP->IsInKeyFrame(mpCurrentKeyFrame)) {
                        pMP->AddObservation(mpCurrentKeyFrame, i);
                        // 获得该点的平均观测方向和观测距离范围
                        pMP->UpdateNormalAndDepth();
                        // 加入关键帧后，更新3d点的最佳描述子
                        pMP->ComputeDistinctiveDescriptors();
                    } else // this can only happen for new stereo points inserted by the Tracking
                    {
                        // 将所有点放到mlpRecentAddedMapPoints，等待检查
                        mlpRecentAddedMapPoints.push_back(pMP);
                    }
                }
            }
        }

        // Update links in the Covisibility Graph
        // 插入关键帧后，更新Covisibility图和Ensenssial图(tree)
        mpCurrentKeyFrame->UpdateConnections();

        // Delete bad KF in LocalWindow
        DeleteBadInLocalWindow();
        // Add Keyframe to LocalWindow
        AddToLocalWindow(mpCurrentKeyFrame);

        // Insert Keyframe in Map
        mpMap->AddKeyFrame(mpCurrentKeyFrame);
    }

    void LocalMapping::MapPointCulling() {
        // Check Recent Added MapPoints
        list<MapPoint *>::iterator lit = mlpRecentAddedMapPoints.begin();
        const unsigned long int nCurrentKFid = mpCurrentKeyFrame->mnId;

        int nThObs;
        if (mbMonocular)
            nThObs = 2;
        else
            nThObs = 3;
        const int cnThObs = nThObs;

        // 遍历等待检查的所有点
        while (lit != mlpRecentAddedMapPoints.end()) {
            MapPoint *pMP = *lit;
            if (pMP->isBad()) {
                // 坏点直接删除
                lit = mlpRecentAddedMapPoints.erase(lit);
            } else if (pMP->GetFoundRatio() < 0.25f) {
                // VI-B 条件1，能找到该点的帧不应该少于理论上观测到该点的帧的1/4
                // IncreaseFound, IncreaseVisible。注意不一定是关键帧。
                pMP->SetBadFlag();
                lit = mlpRecentAddedMapPoints.erase(lit);
            } else if (((int) nCurrentKFid - (int) pMP->mnFirstKFid) >= 2 && pMP->Observations() <= cnThObs) {
                // VI-B 条件2，从该点建立开始，到现在已经过了不小于2帧，但是观测到该点的关键帧数不超过2
                // 那么该点检验不合格
                pMP->SetBadFlag();
                lit = mlpRecentAddedMapPoints.erase(lit);
            } else if (((int) nCurrentKFid - (int) pMP->mnFirstKFid) >= 3)
                // 从建立该点开始，过了3帧，检测合格
                lit = mlpRecentAddedMapPoints.erase(lit);
            else
                lit++;
        }
    }

    // 找到当前关键帧在图中邻接的一些关键帧
    // 对于每一个邻接的关键帧，根据当前关键帧和该邻接关键帧的姿态，算出两幅图像的基本矩阵
    // 搜索当前关键帧和邻接关键帧之间未成为3d点的特征点匹配
    //         在搜索特征点匹配时，先获分别获得这两个帧在数据库中所有二级节点对应的特征点集
    //         获取两个帧中所有相等的二级节点对，在每个二级节点对
    //         能够得到一对特征点集合，分别对应当前关键帧和相邻关键帧
    //         在两个集合中利用极线约束(基本矩阵)和描述子距离来寻求特征点匹配对
    //         可以在满足极线约束的条件下搜索描述子距离最小并且不超过一定阈值的匹配对
    // 获得未成为3d点的特征点匹配集合后，通过三角化获得3d坐标
    // 在通过平行、重投影误差、尺度一致性等检查后，则建立一个对应的3d点MapPoint对象
    // 需要标记新的3d点与两个关键帧之间的观测关系，还需要计算3d点的平均观测方向以及最佳的描述子
    // 最后将新产生的3d点放到检测队列中等待检验
    void LocalMapping::CreateNewMapPoints() {
        // Retrieve neighbor keyframes in covisibility graph
        // 在当前插入的关键帧中，找到权重前nn的邻接关键帧
        int nn = 10;
        if (mbMonocular)
            nn = 20;
        const vector<KeyFrame *> vpNeighKFs = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(nn);

        ORBmatcher matcher(0.6, false);

        Matrix3f Rcw1 = mpCurrentKeyFrame->GetRotation();
        Matrix3f Rwc1 = Rcw1.transpose();
        Vector3f tcw1 = mpCurrentKeyFrame->GetTranslation();
        Eigen::Matrix<float, 3, 4> Tcw1;
        Tcw1.block<3, 3>(0, 0) = Rcw1;
        Tcw1.block<3, 1>(0, 3) = tcw1;
        Vector3f Ow1 = mpCurrentKeyFrame->GetCameraCenter();

        const float &fx1 = mpCurrentKeyFrame->fx;
        const float &fy1 = mpCurrentKeyFrame->fy;
        const float &cx1 = mpCurrentKeyFrame->cx;
        const float &cy1 = mpCurrentKeyFrame->cy;
        const float &invfx1 = mpCurrentKeyFrame->invfx;
        const float &invfy1 = mpCurrentKeyFrame->invfy;

        const float ratioFactor = 1.5f * mpCurrentKeyFrame->mfScaleFactor;

        int nnew = 0;

        // Search matches with epipolar restriction and triangulate
        // 遍历所有找到权重前nn的相邻关键帧
        for (size_t i = 0; i < vpNeighKFs.size(); i++) {
            if (i > 0 && CheckNewKeyFrames())
                return;

            // 当前邻接的关键帧
            KeyFrame *pKF2 = vpNeighKFs[i];

            // Check first that baseline is not too short
            // 邻接的关键帧在世界坐标系中的坐标
            Vector3f Ow2 = pKF2->GetCameraCenter();
            // 基线，两个关键帧的位移
            Vector3f vBaseline = Ow2 - Ow1;
            // 基线长度
            const float baseline = vBaseline.norm();

            if (!mbMonocular) {
                if (baseline < pKF2->mb)
                    continue;
            } else {
                // 邻接关键帧的场景深度中值
                const float medianDepthKF2 = pKF2->ComputeSceneMedianDepth(2);
                // 景深与距离的比例
                const float ratioBaselineDepth = baseline / medianDepthKF2;
                // 如果特别远(比例特别小)，那么不考虑当前邻接的关键帧
                if (ratioBaselineDepth < 0.01)
                    continue;
            }

            // Compute Fundamental Matrix
            Matrix3f F12 = ComputeF12(mpCurrentKeyFrame, pKF2);

            // Search matches that fullfil epipolar constraint
            vector<pair<size_t, size_t> > vMatchedIndices;
            matcher.SearchForTriangulation(mpCurrentKeyFrame, pKF2, F12, vMatchedIndices, false);

            Matrix3f Rcw2 = pKF2->GetRotation();
            Matrix3f Rwc2 = Rcw2.transpose();
            Vector3f tcw2 = pKF2->GetTranslation();
            Matrix<float, 3, 4> Tcw2;
            Tcw2.block<3, 3>(0, 0) = Rcw2;
            Tcw2.block<3, 1>(0, 3) = tcw2;

            const float &fx2 = pKF2->fx;
            const float &fy2 = pKF2->fy;
            const float &cx2 = pKF2->cx;
            const float &cy2 = pKF2->cy;
            const float &invfx2 = pKF2->invfx;
            const float &invfy2 = pKF2->invfy;

            // Triangulate each match
            const int nmatches = vMatchedIndices.size();
            for (int ikp = 0; ikp < nmatches; ikp++) {
                // 当前匹配对在当前关键帧中的索引
                const int &idx1 = vMatchedIndices[ikp].first;

                // 当前匹配对在邻接关键帧中的索引
                const int &idx2 = vMatchedIndices[ikp].second;

                // 当前匹配在当前关键帧中的特征点
                const cv::KeyPoint &kp1 = mpCurrentKeyFrame->mvKeys[idx1];
                const float kp1_ur = mpCurrentKeyFrame->mvuRight[idx1];
                bool bStereo1 = kp1_ur >= 0;

                // 当前匹配在邻接关键帧中的特征点
                const cv::KeyPoint &kp2 = pKF2->mvKeys[idx2];
                const float kp2_ur = pKF2->mvuRight[idx2];
                bool bStereo2 = kp2_ur >= 0;

                // Check parallax between rays
                Vector3f xn1((kp1.pt.x - cx1) * invfx1, (kp1.pt.y - cy1) * invfy1, 1.0);
                Vector3f xn2((kp2.pt.x - cx2) * invfx2, (kp2.pt.y - cy2) * invfy2, 1.0);

                Vector3f ray1 = Rwc1 * xn1;
                Vector3f ray2 = Rwc2 * xn2;
                const float cosParallaxRays = ray1.dot(ray2) / (ray1.norm() * ray2.norm());

                float cosParallaxStereo = cosParallaxRays + 1;
                float cosParallaxStereo1 = cosParallaxStereo;
                float cosParallaxStereo2 = cosParallaxStereo;

                if (bStereo1)
                    cosParallaxStereo1 = cos(2 * atan2(mpCurrentKeyFrame->mb / 2, mpCurrentKeyFrame->mvDepth[idx1]));
                else if (bStereo2)
                    cosParallaxStereo2 = cos(2 * atan2(pKF2->mb / 2, pKF2->mvDepth[idx2]));

                cosParallaxStereo = min(cosParallaxStereo1, cosParallaxStereo2);

                Vector3f x3D;
                if (cosParallaxRays < cosParallaxStereo && cosParallaxRays > 0 &&
                    (bStereo1 || bStereo2 || cosParallaxRays < 0.9998)) {
                    // Linear Triangulation Method
                    Matrix4f A;
                    A.row(0) = xn1[0] * Tcw1.row(2) - Tcw1.row(0);
                    A.row(1) = xn1[1] * Tcw1.row(2) - Tcw1.row(1);
                    A.row(2) = xn2[0] * Tcw2.row(2) - Tcw2.row(0);
                    A.row(3) = xn2[1] * Tcw2.row(2) - Tcw2.row(1);

                    Eigen::JacobiSVD<Matrix4f> SVD(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
                    Matrix4f V = SVD.matrixV();
                    if (V(3, 3) == 0)
                        continue;
                    x3D = V.block<3, 1>(0, 3) / V(3, 3);

                } else if (bStereo1 && cosParallaxStereo1 < cosParallaxStereo2) {
                    x3D = mpCurrentKeyFrame->UnprojectStereo(idx1);
                } else if (bStereo2 && cosParallaxStereo2 < cosParallaxStereo1) {
                    x3D = pKF2->UnprojectStereo(idx2);
                } else
                    continue; //No stereo and very low parallax

                Vector3f x3Dt = x3D;

                //Check triangulation in front of cameras
                float z1 = Rcw1.row(2).dot(x3Dt) + tcw1[2];
                if (z1 <= 0)
                    continue;

                float z2 = Rcw2.row(2).dot(x3Dt) + tcw2[2];
                if (z2 <= 0)
                    continue;

                //Check reprojection error in first keyframe
                const float &sigmaSquare1 = mpCurrentKeyFrame->mvLevelSigma2[kp1.octave];
                const float x1 = Rcw1.row(0).dot(x3Dt) + tcw1[0];
                const float y1 = Rcw1.row(1).dot(x3Dt) + tcw1[1];
                const float invz1 = 1.0 / z1;

                if (!bStereo1) {
                    float u1 = fx1 * x1 * invz1 + cx1;
                    float v1 = fy1 * y1 * invz1 + cy1;
                    float errX1 = u1 - kp1.pt.x;
                    float errY1 = v1 - kp1.pt.y;
                    if ((errX1 * errX1 + errY1 * errY1) > 5.991 * sigmaSquare1)
                        continue;
                } else {
                    float u1 = fx1 * x1 * invz1 + cx1;
                    float u1_r = u1 - mpCurrentKeyFrame->mbf * invz1;
                    float v1 = fy1 * y1 * invz1 + cy1;
                    float errX1 = u1 - kp1.pt.x;
                    float errY1 = v1 - kp1.pt.y;
                    float errX1_r = u1_r - kp1_ur;
                    if ((errX1 * errX1 + errY1 * errY1 + errX1_r * errX1_r) > 7.8 * sigmaSquare1)
                        continue;
                }

                //Check reprojection error in second keyframe
                const float sigmaSquare2 = pKF2->mvLevelSigma2[kp2.octave];
                const float x2 = Rcw2.row(0).dot(x3Dt) + tcw2[0];
                const float y2 = Rcw2.row(1).dot(x3Dt) + tcw2[1];
                const float invz2 = 1.0 / z2;
                if (!bStereo2) {
                    float u2 = fx2 * x2 * invz2 + cx2;
                    float v2 = fy2 * y2 * invz2 + cy2;
                    float errX2 = u2 - kp2.pt.x;
                    float errY2 = v2 - kp2.pt.y;
                    if ((errX2 * errX2 + errY2 * errY2) > 5.991 * sigmaSquare2)
                        continue;
                } else {
                    float u2 = fx2 * x2 * invz2 + cx2;
                    float u2_r = u2 - mpCurrentKeyFrame->mbf * invz2;
                    float v2 = fy2 * y2 * invz2 + cy2;
                    float errX2 = u2 - kp2.pt.x;
                    float errY2 = v2 - kp2.pt.y;
                    float errX2_r = u2_r - kp2_ur;
                    if ((errX2 * errX2 + errY2 * errY2 + errX2_r * errX2_r) > 7.8 * sigmaSquare2)
                        continue;
                }

                //Check scale consistency
                Vector3f normal1 = x3D - Ow1;
                float dist1 = normal1.norm();

                Vector3f normal2 = x3D - Ow2;
                float dist2 = normal2.norm();

                if (dist1 == 0 || dist2 == 0)
                    continue;

                const float ratioDist = dist2 / dist1;
                const float ratioOctave =
                        mpCurrentKeyFrame->mvScaleFactors[kp1.octave] / pKF2->mvScaleFactors[kp2.octave];

                /*if(fabs(ratioDist-ratioOctave)>ratioFactor)
                    continue;*/
                if (ratioDist * ratioFactor < ratioOctave || ratioDist > ratioOctave * ratioFactor)
                    continue;

                // Triangulation is succesfull
                MapPoint *pMP = new MapPoint(x3D, mpCurrentKeyFrame, mpMap);

                pMP->AddObservation(mpCurrentKeyFrame, idx1);
                pMP->AddObservation(pKF2, idx2);

                mpCurrentKeyFrame->AddMapPoint(pMP, idx1);
                pKF2->AddMapPoint(pMP, idx2);

                pMP->ComputeDistinctiveDescriptors();

                pMP->UpdateNormalAndDepth();

                mpMap->AddMapPoint(pMP);

                // 将新产生的点放入检测队列
                mlpRecentAddedMapPoints.push_back(pMP);

                nnew++;
            }
        }
    }

    // 产生新的3d点后，该3d点有可能会被其他关键帧找到
    // 找到当前关键帧一二级邻接关键帧集合k12
    // 判断并处理当前关键帧的3d点与k12的关系
    // 判断并处理k12的所有3d点与当前关键帧的关系
    // 处理3d点和关键帧的关系时，先将3d点投影到关键帧上，在投影点附近搜索匹配特征点
    // 如果找到匹配的特征点，那么判断该特征点是否成为了3d点，如果已经成为3d点，那么将两个3d点进行合并，注意观测它的关键帧也要合并
    // 如果没有成为3d点，那么就添加这个3d点与关键帧还有特征点之间的关系
    // 处理完3d点和关键帧的关系后需要更新相应的图和树结构
    // 还需要更新所涉及3d点的观测方向，与引用关键帧的距离，以及最合适的描述子
    void LocalMapping::SearchInNeighbors() {
        // Retrieve neighbor keyframes
        // 获得当前关键帧在covisibility图中权重排名前nn的邻接关键帧
        int nn = 10;
        if (mbMonocular)
            nn = 20;
        const vector<KeyFrame *> vpNeighKFs = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(nn);

        // 获得相邻关键帧和二级相邻关键帧的集合，存储在vpTargetKFs中
        vector<KeyFrame *> vpTargetKFs;
        for (vector<KeyFrame *>::const_iterator vit = vpNeighKFs.begin(), vend = vpNeighKFs.end(); vit != vend; vit++) {
            KeyFrame *pKFi = *vit;
            if (pKFi->isBad() || pKFi->mnFuseTargetForKF == mpCurrentKeyFrame->mnId)
                continue;
            vpTargetKFs.push_back(pKFi);
            pKFi->mnFuseTargetForKF = mpCurrentKeyFrame->mnId;

            // Extend to some second neighbors
            const vector<KeyFrame *> vpSecondNeighKFs = pKFi->GetBestCovisibilityKeyFrames(5);
            for (vector<KeyFrame *>::const_iterator vit2 = vpSecondNeighKFs.begin(), vend2 = vpSecondNeighKFs.end();
                 vit2 != vend2; vit2++) {
                KeyFrame *pKFi2 = *vit2;
                if (pKFi2->isBad() || pKFi2->mnFuseTargetForKF == mpCurrentKeyFrame->mnId ||
                    pKFi2->mnId == mpCurrentKeyFrame->mnId)
                    continue;
                vpTargetKFs.push_back(pKFi2);
            }
        }


        // Search matches by projection from current KF in target KFs
        ORBmatcher matcher;
        vector<MapPoint *> vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();
        for (vector<KeyFrame *>::iterator vit = vpTargetKFs.begin(), vend = vpTargetKFs.end(); vit != vend; vit++) {
            KeyFrame *pKFi = *vit;

            // 判断当前关键帧中的3d点是否能在一级邻接和二级邻接的关键帧中找到
            // 如果是则进行处理
            // 如果3d点能匹配关键帧的特征点，并且该点已经成为了3d点，那么将两个3d点合并
            // 如果3d点能匹配关键帧的特征点，并且该点不是3d点，那么更新3d点与关键帧的关系
            matcher.Fuse(pKFi, vpMapPointMatches);
        }

        // Search matches by projection from target KFs in current KF
        // 用于存储一级邻接和二级邻接关键帧所有3d点的集合
        vector<MapPoint *> vpFuseCandidates;
        vpFuseCandidates.reserve(vpTargetKFs.size() * vpMapPointMatches.size());

        // 遍历每一个一二级邻接关键帧
        for (vector<KeyFrame *>::iterator vitKF = vpTargetKFs.begin(), vendKF = vpTargetKFs.end();
             vitKF != vendKF; vitKF++) {
            KeyFrame *pKFi = *vitKF;

            vector<MapPoint *> vpMapPointsKFi = pKFi->GetMapPointMatches();

            // 遍历当前一二级邻接关键帧中所有的3d点
            for (vector<MapPoint *>::iterator vitMP = vpMapPointsKFi.begin(), vendMP = vpMapPointsKFi.end();
                 vitMP != vendMP; vitMP++) {
                MapPoint *pMP = *vitMP;
                if (!pMP)
                    continue;

                // 判断3d点是否为坏点，或者是否已经加进集合vpFuseCandidates
                if (pMP->isBad() || pMP->mnFuseCandidateForKF == mpCurrentKeyFrame->mnId)
                    continue;

                // 加入集合，并标记已经加入
                pMP->mnFuseCandidateForKF = mpCurrentKeyFrame->mnId;
                vpFuseCandidates.push_back(pMP);
            }
        }

        // 判断所有一二级关键帧的所有3d点是否能在当前关键帧中找到
        // 如果能，则进行相应处理
        // 如果3d点能匹配关键帧的特征点，并且该点已经成为了3d点，那么将两个3d点合并
        // 如果3d点能匹配关键帧的特征点，并且该点不是3d点，那么更新3d点与关键帧的关系
        matcher.Fuse(mpCurrentKeyFrame, vpFuseCandidates);


        // Update points
        vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();
        for (size_t i = 0, iend = vpMapPointMatches.size(); i < iend; i++) {
            MapPoint *pMP = vpMapPointMatches[i];
            if (pMP) {
                if (!pMP->isBad()) {
                    // 在所有找到pMP的关键帧中，获得最佳的描述子
                    pMP->ComputeDistinctiveDescriptors();

                    // 更新平均观测方向和观测距离
                    pMP->UpdateNormalAndDepth();
                }
            }
        }

        // Update connections in covisibility graph

        // 更新covisibility图
        mpCurrentKeyFrame->UpdateConnections();
    }

    // 根据姿态计算两个关键帧之间的基本矩阵
    Matrix3f LocalMapping::ComputeF12(KeyFrame *&pKF1, KeyFrame *&pKF2) {
        Matrix3f R1w = pKF1->GetRotation();
        Vector3f t1w = pKF1->GetTranslation();
        Matrix3f R2w = pKF2->GetRotation();
        Vector3f t2w = pKF2->GetTranslation();

        Matrix3f R12 = R1w * R2w.transpose();
        Vector3f t12 = -1 * R1w * R2w.transpose() * t2w + t1w;

        Matrix3f t12x = SO3f::hat(t12);

        const Matrix3f &K1 = pKF1->mK;
        const Matrix3f &K2 = pKF2->mK;

        return K1.transpose().inverse() * t12x * R12 * K2.inverse();
    }

    void LocalMapping::RequestStop() {
        unique_lock<mutex> lock(mMutexStop);
        mbStopRequested = true;
        unique_lock<mutex> lock2(mMutexNewKFs);
        mbAbortBA = true;
    }

    bool LocalMapping::Stop() {
        unique_lock<mutex> lock(mMutexStop);
        if (mbStopRequested && !mbNotStop) {
            mbStopped = true;
            cout << "Local Mapping STOP" << endl;
            return true;
        }

        return false;
    }

    bool LocalMapping::isStopped() {
        unique_lock<mutex> lock(mMutexStop);
        return mbStopped;
    }

    bool LocalMapping::stopRequested() {
        unique_lock<mutex> lock(mMutexStop);
        return mbStopRequested;
    }

    void LocalMapping::Release() {
        unique_lock<mutex> lock(mMutexStop);
        unique_lock<mutex> lock2(mMutexFinish);
        if (mbFinished)
            return;
        mbStopped = false;
        mbStopRequested = false;
        for (list<KeyFrame *>::iterator lit = mlNewKeyFrames.begin(), lend = mlNewKeyFrames.end(); lit != lend; lit++)
            delete *lit;
        mlNewKeyFrames.clear();

        cout << "Local Mapping RELEASE" << endl;
    }

    bool LocalMapping::AcceptKeyFrames() {
        unique_lock<mutex> lock(mMutexAccept);
        return mbAcceptKeyFrames;
    }

    void LocalMapping::SetAcceptKeyFrames(bool flag) {
        unique_lock<mutex> lock(mMutexAccept);
        mbAcceptKeyFrames = flag;
    }

    bool LocalMapping::SetNotStop(bool flag) {
        unique_lock<mutex> lock(mMutexStop);

        if (flag && mbStopped)
            return false;

        mbNotStop = flag;

        return true;
    }

    void LocalMapping::InterruptBA() {
        mbAbortBA = true;
    }

    /**
     * @brief 关键帧剔除
     *
     * 在Covisibility Graph中的关键帧，其90%以上的MapPoints能被其他关键帧（至少3个）观测到，则认为该关键帧为冗余关键帧。
     * @see VI-E Local Keyframe Culling
     */
    void LocalMapping::KeyFrameCulling() {
        // Check redundant keyframes (only local keyframes)
        // A keyframe is considered redundant if the 90% of the MapPoints it sees, are seen
        // in at least other 3 keyframes (in the same or finer scale)
        // We only consider close stereo points

        if (GetFlagCopyInitKFs())
            return;
        SetFlagCopyInitKFs(true);

        // TODO 用直接法追踪时，经常会有共视情况出现，需要调整这里的阈值以防太多东西都被culling掉
        // 之前的原则是：若90%以上的MapPoints能被其他关键帧观测到，就认为该关键帧冗余。
        vector<KeyFrame *> vpLocalKeyFrames = mpCurrentKeyFrame->GetVectorCovisibleKeyFrames();

        for (vector<KeyFrame *>::iterator vit = vpLocalKeyFrames.begin(), vend = vpLocalKeyFrames.end();
             vit != vend; vit++) {
            KeyFrame *pKF = *vit;
            if (pKF->mnId == 0)
                continue;

            // TODO check this
            if ((mpCurrentKeyFrame->mnId - pKF->mnId) <= 10)
                continue;  // don't remove nearby key-frames in vio

            if (mbUseIMU) {
                // Don't drop the KF before current KF
                if (pKF->GetNextKeyFrame() == mpCurrentKeyFrame)
                    continue;

                if (pKF->mTimeStamp >= mpCurrentKeyFrame->mTimeStamp - 0.15)
                    continue;
            }


            const vector<MapPoint *> vpMapPoints = pKF->GetMapPointMatches();

            int nObs = 3;
            const int thObs = nObs;
            int nRedundantObservations = 0;
            int nMPs = 0;
            for (size_t i = 0, iend = vpMapPoints.size(); i < iend; i++) {
                MapPoint *pMP = vpMapPoints[i];
                if (pMP) {
                    if (!pMP->isBad()) {
                        if (!mbMonocular) {
                            if (pKF->mvDepth[i] > pKF->mThDepth || pKF->mvDepth[i] < 0)
                                continue;
                        }

                        nMPs++;
                        if (pMP->Observations() > thObs) {
                            const int &scaleLevel = pKF->mvKeys[i].octave;
                            const map<KeyFrame *, size_t> observations = pMP->GetObservations();
                            int nObs = 0;
                            for (map<KeyFrame *, size_t>::const_iterator mit = observations.begin(), mend = observations.end();
                                 mit != mend; mit++) {
                                KeyFrame *pKFi = mit->first;
                                if (pKFi == pKF)
                                    continue;
                                const int &scaleLeveli = pKFi->mvKeys[mit->second].octave;

                                if (scaleLeveli <= scaleLevel + 1) {
                                    nObs++;
                                    if (nObs >= thObs)
                                        break;
                                }
                            }
                            if (nObs >= thObs) {
                                nRedundantObservations++;
                            }
                        }
                    }
                }
            }

            if (nRedundantObservations > 0.9 * nMPs) {
                pKF->SetBadFlag();
            }
        }

        SetFlagCopyInitKFs(false);
    }

    void LocalMapping::RequestReset() {
        {
            unique_lock<mutex> lock(mMutexReset);
            mbResetRequested = true;
        }

        while (1) {
            {
                unique_lock<mutex> lock2(mMutexReset);
                if (!mbResetRequested)
                    break;
            }
            usleep(3000);
        }
    }

    void LocalMapping::ResetIfRequested() {
        unique_lock<mutex> lock(mMutexReset);
        if (mbResetRequested) {
            if (GetVINSIniting()) {
                // Wait VINS init finish
                cout << "Reset, wait VINS init finish" << endl;
                SetResetVINSInit(true);
                while (GetVINSIniting()) {
                    usleep(10000);
                }
            }

            if (mptLocalMappingVIOInit) {
                cout << "Detach and delete VINS init thread" << endl;
                mptLocalMappingVIOInit->detach();
                delete mptLocalMappingVIOInit;
                mptLocalMappingVIOInit = new thread(&LocalMapping::VINSInitThread, this);
            }

            mlNewKeyFrames.clear();
            mlpRecentAddedMapPoints.clear();
            mbResetRequested = false;

            mlLocalKeyFrames.clear();

            // Add resetting init flags
            mbVINSInited = false;
            mbFirstTry = true;
        }
    }

    void LocalMapping::RequestFinish() {
        unique_lock<mutex> lock(mMutexFinish);
        mbFinishRequested = true;
    }

    bool LocalMapping::CheckFinish() {
        unique_lock<mutex> lock(mMutexFinish);
        return mbFinishRequested;
    }

    void LocalMapping::SetFinish() {
        unique_lock<mutex> lock(mMutexFinish);
        mbFinished = true;
        unique_lock<mutex> lock2(mMutexStop);
        mbStopped = true;
    }

    bool LocalMapping::isFinished() {
        unique_lock<mutex> lock(mMutexFinish);
        return mbFinished;
    }

} //namespace ygz
