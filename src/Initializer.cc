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

#include "Initializer.h"

#include "Thirdparty/DBoW2/DUtils/Random.h"

#include "Optimizer.h"
#include "ORBmatcher.h"
#include "Converter.h"


namespace ygz {

    Initializer::Initializer(const Frame &ReferenceFrame, float sigma, int iterations) {
        mK = ReferenceFrame.mK;

        mvKeys1 = ReferenceFrame.mvKeys;

        mSigma = sigma;
        mSigma2 = sigma * sigma;
        mMaxIterations = iterations;
    }

    /**
     * @brief 并行地计算基础矩阵和单应性矩阵，选取其中一个模型，恢复出最开始两帧之间的相对姿态以及点云
     */
    bool Initializer::Initialize(
            const Frame &CurrentFrame, const vector<int> &vMatches12, Matrix3f &R21, Vector3f &t21,
            vector<Vector3f> &vP3D, vector<bool> &vbTriangulated) {
        // Fill structures with current keypoints and matches with reference frame
        // Reference Frame: 1, Current Frame: 2
        mvKeys2 = CurrentFrame.mvKeys;

        mvMatches12.clear();
        mvMatches12.reserve(mvKeys2.size());
        mvbMatched1.resize(mvKeys1.size());
        for (size_t i = 0, iend = vMatches12.size(); i < iend; i++) {
            if (vMatches12[i] >= 0) {
                mvMatches12.push_back(make_pair(i, vMatches12[i]));
                mvbMatched1[i] = true;
            } else
                mvbMatched1[i] = false;
        }

        const int N = mvMatches12.size();

        // Indices for minimum set selection
        vector<size_t> vAllIndices;
        vAllIndices.reserve(N);
        vector<size_t> vAvailableIndices;

        for (int i = 0; i < N; i++) {
            vAllIndices.push_back(i);
        }

        // Generate sets of 8 points for each RANSAC iteration
        mvSets = vector<vector<size_t> >(mMaxIterations, vector<size_t>(8, 0));

        DUtils::Random::SeedRandOnce(0);

        for (int it = 0; it < mMaxIterations; it++) {
            vAvailableIndices = vAllIndices;

            // Select a minimum set
            for (size_t j = 0; j < 8; j++) {
                int randi = DUtils::Random::RandomInt(0, vAvailableIndices.size() - 1);
                int idx = vAvailableIndices[randi];

                mvSets[it][j] = idx;

                vAvailableIndices[randi] = vAvailableIndices.back();
                vAvailableIndices.pop_back();
            }
        }

        // Launch threads to compute in parallel a fundamental matrix and a homography
        vector<bool> vbMatchesInliersH, vbMatchesInliersF;
        float SH, SF;
        Matrix3f H, F;

        thread threadH(&Initializer::FindHomography, this, ref(vbMatchesInliersH), ref(SH), ref(H));
        thread threadF(&Initializer::FindFundamental, this, ref(vbMatchesInliersF), ref(SF), ref(F));

        // Wait until both threads have finished
        threadH.join();
        threadF.join();

        // Compute ratio of scores
        float RH = SH / (SH + SF);

        // Try to reconstruct from homography or fundamental depending on the ratio (0.40-0.45)
        if (RH > 0.40)
            return ReconstructH(vbMatchesInliersH, H, mK, R21, t21, vP3D, vbTriangulated, 1.0, 50);
        else //if(pF_HF>0.6)
            return ReconstructF(vbMatchesInliersF, F, mK, R21, t21, vP3D, vbTriangulated, 1.0, 50);

        return false;
    }


    void Initializer::FindHomography(vector<bool> &vbMatchesInliers, float &score, Matrix3f &H21) {
        // Number of putative matches
        const int N = mvMatches12.size();

        // Normalize coordinates
        vector<Vector2f> vPn1, vPn2;
        Matrix3f T1, T2;
        Normalize(mvKeys1, vPn1, T1);
        Normalize(mvKeys2, vPn2, T2);
        Matrix3f T2inv = T2.inverse();

        // Best Results variables
        score = 0.0;
        vbMatchesInliers = vector<bool>(N, false);

        // Iteration variables
        vector<Vector2f> vPn1i(8);
        vector<Vector2f> vPn2i(8);
        Matrix3f H21i, H12i;
        vector<bool> vbCurrentInliers(N, false);
        float currentScore;

        // Perform all RANSAC iterations and save the solution with highest score
        for (int it = 0; it < mMaxIterations; it++) {
            // Select a minimum set
            for (size_t j = 0; j < 8; j++) {
                int idx = mvSets[it][j];

                vPn1i[j] = vPn1[mvMatches12[idx].first];
                vPn2i[j] = vPn2[mvMatches12[idx].second];
            }

            Matrix3f Hn = ComputeH21(vPn1i, vPn2i);
            H21i = T2inv * Hn * T1;
            H12i = H21i.inverse();

            currentScore = CheckHomography(H21i, H12i, vbCurrentInliers, mSigma);

            if (currentScore > score) {
                H21 = H21i;
                vbMatchesInliers = vbCurrentInliers;
                score = currentScore;
            }
        }

    }


    /**
     * @brief 计算基础矩阵
     *
     * 假设场景为非平面情况下通过前两帧求取Fundamental矩阵(current frame 2 到 reference frame 1),并得到该模型的评分
     * 流程和FindHomograp  hy相似
     */
    void Initializer::FindFundamental(vector<bool> &vbMatchesInliers, float &score, Matrix3f &F21) {
        // Number of putative matches
        const int N = vbMatchesInliers.size();

        // Normalize coordinates
        vector<Vector2f> vPn1, vPn2;
        Matrix3f T1, T2;
        Normalize(mvKeys1, vPn1, T1);
        Normalize(mvKeys2, vPn2, T2);
        Matrix3f T2t = T2.transpose();

        // Best Results variables
        score = 0.0;
        vbMatchesInliers = vector<bool>(N, false);

        // Iteration variables
        vector<Vector2f> vPn1i(8);
        vector<Vector2f> vPn2i(8);
        Matrix3f F21i;
        vector<bool> vbCurrentInliers(N, false);
        float currentScore;

        // Perform all RANSAC iterations and save the solution with highest score
        for (int it = 0; it < mMaxIterations; it++) {
            // Select a minimum set
            for (int j = 0; j < 8; j++) {
                int idx = mvSets[it][j];

                vPn1i[j] = vPn1[mvMatches12[idx].first];
                vPn2i[j] = vPn2[mvMatches12[idx].second];
            }

            Matrix3f Fn = ComputeF21(vPn1i, vPn2i);

            F21i = T2t * Fn * T1;

            currentScore = CheckFundamental(F21i, vbCurrentInliers, mSigma);

            if (currentScore > score) {
                F21 = F21i;
                vbMatchesInliers = vbCurrentInliers;
                score = currentScore;
            }
        }
    }


    // |x'|     | h1 h2 h3 ||x|
    // |y'| = a | h4 h5 h6 ||y|  简写: x' = a H x, a为一个尺度因子
    // |1 |     | h7 h8 h9 ||1|
    // 使用DLT(direct linear tranform)求解该模型
    // x' = a H x
    // ---> (x') 叉乘 (H x)  = 0
    // ---> Ah = 0
    // A = | 0  0  0 -x -y -1 xy' yy' y'|  h = | h1 h2 h3 h4 h5 h6 h7 h8 h9 |
    //     |-x -y -1  0  0  0 xx' yx' x'|
    // 通过SVD求解Ah = 0，A'A最小特征值对应的特征向量即为解

    /**
     * @brief 从特征点匹配求homography（normalized DLT）
     *
     * @param  vP1 归一化后的点, in reference frame
     * @param  vP2 归一化后的点, in current frame
     * @return     单应矩阵
     * @see        Multiple View Geometry in Computer Vision - Algorithm 4.2 p109
     */
    Matrix3f Initializer::ComputeH21(const vector<Vector2f> &vP1, const vector<Vector2f> &vP2) {
        const int N = vP1.size();
        Eigen::MatrixXf A(2 * N, 9);
        for (size_t i = 0; i < N; i++) {
            const float u1 = vP1[i][0];
            const float v1 = vP1[i][1];
            const float u2 = vP2[i][0];
            const float v2 = vP2[i][1];

            A(2 * i, 0) = 0.0;
            A(2 * i, 1) = 0.0;
            A(2 * i, 2) = 0.0;
            A(2 * i, 3) = -u1;
            A(2 * i, 4) = -v1;
            A(2 * i, 5) = -1;
            A(2 * i, 6) = v2 * u1;
            A(2 * i, 7) = v2 * v1;
            A(2 * i, 8) = v2;

            A(2 * i + 1, 0) = u1;
            A(2 * i + 1, 1) = v1;
            A(2 * i + 1, 2) = 1;
            A(2 * i + 1, 3) = 0.0;
            A(2 * i + 1, 4) = 0.0;
            A(2 * i + 1, 5) = 0.0;
            A(2 * i + 1, 6) = -u2 * u1;
            A(2 * i + 1, 7) = -u2 * v1;
            A(2 * i + 1, 8) = -u2;

        }

        Eigen::JacobiSVD<Eigen::MatrixXf> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::MatrixXf V = svd.matrixV();
        // Eigen的SVD是U*S*V^T = A ，所以取V最后一列

        Matrix3f ret;
        ret << V(0, 8), V(1, 8), V(2, 8),
                V(3, 8), V(4, 8), V(5, 8),
                V(6, 8), V(7, 8), V(8, 8);
        return ret;       // V 最后一列
    }

    // x'Fx = 0 整理可得：Af = 0
    // A = | x'x x'y x' y'x y'y y' x y 1 |, f = | f1 f2 f3 f4 f5 f6 f7 f8 f9 |
    // 通过SVD求解Af = 0，A'A最小特征值对应的特征向量即为解
    /**
     * @brief 从特征点匹配求fundamental matrix（normalized 8点法）
     * @param  vP1 归一化后的点, in reference frame
     * @param  vP2 归一化后的点, in current frame
     * @return     基础矩阵
     * @see        Multiple View Geometry in Computer Vision - Algorithm 11.1 p282 (中文版 p191)
     */
    Matrix3f Initializer::ComputeF21(const vector<Vector2f> &vP1, const vector<Vector2f> &vP2) {
        const int N = vP1.size();
        Eigen::MatrixXf A(N, 9);
        for (size_t i = 0; i < N; i++) {
            const float u1 = vP1[i][0];
            const float v1 = vP1[i][1];
            const float u2 = vP2[i][0];
            const float v2 = vP2[i][1];

            A(i, 0) = u2 * u1;
            A(i, 1) = u2 * v1;
            A(i, 2) = u2;
            A(i, 3) = v2 * u1;
            A(i, 4) = v2 * v1;
            A(i, 5) = v2;
            A(i, 6) = u1;
            A(i, 7) = v1;
            A(i, 8) = 1;
        }

        Eigen::JacobiSVD<Eigen::MatrixXf> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::MatrixXf V = svd.matrixV();
        Matrix3f Fpre;
        Fpre << V(0, 8), V(1, 8), V(2, 8),
                V(3, 8), V(4, 8), V(5, 8),
                V(6, 8), V(7, 8), V(8, 8);       // 最后一列转成3x3
        Eigen::JacobiSVD<Eigen::Matrix3f> svd_F(Fpre, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Vector3f sigma = svd_F.singularValues();
        return svd_F.matrixU() * Eigen::DiagonalMatrix<float, 3>(sigma[0], sigma[1], 0) *
               svd_F.matrixV().transpose(); // 第3个奇异值设零
    }

    /**
     * @brief 对给定的homography matrix打分
     *
     * @see
     * - Author's paper - IV. AUTOMATIC MAP INITIALIZATION （2）
     * - Multiple View Geometry in Computer Vision - symmetric transfer errors: 4.2.2 Geometric distance
     * - Multiple View Geometry in Computer Vision - model selection 4.7.1 RANSAC
     *
     * 实际上算的是第二个图到第一个图的H重投影
     */
    float Initializer::CheckHomography(const Matrix3f &H21, const Matrix3f &H12, vector<bool> &vbMatchesInliers,
                                       float sigma) {
        const int N = mvMatches12.size();

        const float h11 = H21(0, 0);
        const float h12 = H21(0, 1);
        const float h13 = H21(0, 2);
        const float h21 = H21(1, 0);
        const float h22 = H21(1, 1);
        const float h23 = H21(1, 2);
        const float h31 = H21(2, 0);
        const float h32 = H21(2, 1);
        const float h33 = H21(2, 2);

        const float h11inv = H12(0, 0);
        const float h12inv = H12(0, 1);
        const float h13inv = H12(0, 2);
        const float h21inv = H12(1, 0);
        const float h22inv = H12(1, 1);
        const float h23inv = H12(1, 2);
        const float h31inv = H12(2, 0);
        const float h32inv = H12(2, 1);
        const float h33inv = H12(2, 2);

        vbMatchesInliers.resize(N);

        float score = 0;

        const float th = 5.991;

        const float invSigmaSquare = 1.0 / (sigma * sigma);

        for (int i = 0; i < N; i++) {
            bool bIn = true;

            const cv::KeyPoint &kp1 = mvKeys1[mvMatches12[i].first];
            const cv::KeyPoint &kp2 = mvKeys2[mvMatches12[i].second];

            const float u1 = kp1.pt.x;
            const float v1 = kp1.pt.y;
            const float u2 = kp2.pt.x;
            const float v2 = kp2.pt.y;

            // Reprojection error in first image
            // x2in1 = H12*x2

            const float w2in1inv = 1.0 / (h31inv * u2 + h32inv * v2 + h33inv);
            const float u2in1 = (h11inv * u2 + h12inv * v2 + h13inv) * w2in1inv;
            const float v2in1 = (h21inv * u2 + h22inv * v2 + h23inv) * w2in1inv;

            const float squareDist1 = (u1 - u2in1) * (u1 - u2in1) + (v1 - v2in1) * (v1 - v2in1);

            const float chiSquare1 = squareDist1 * invSigmaSquare;

            if (chiSquare1 > th)
                bIn = false;
            else
                score += th - chiSquare1;

            // Reprojection error in second image
            // x1in2 = H21*x1

            const float w1in2inv = 1.0 / (h31 * u1 + h32 * v1 + h33);
            const float u1in2 = (h11 * u1 + h12 * v1 + h13) * w1in2inv;
            const float v1in2 = (h21 * u1 + h22 * v1 + h23) * w1in2inv;

            const float squareDist2 = (u2 - u1in2) * (u2 - u1in2) + (v2 - v1in2) * (v2 - v1in2);

            const float chiSquare2 = squareDist2 * invSigmaSquare;

            if (chiSquare2 > th)
                bIn = false;
            else
                score += th - chiSquare2;

            if (bIn)
                vbMatchesInliers[i] = true;
            else
                vbMatchesInliers[i] = false;
        }

        return score;
    }


    /**
     * @brief 对给定的fundamental matrix打分
     *
     * @see
     * - Author's paper - IV. AUTOMATIC MAP INITIALIZATION （2）
     * - Multiple View Geometry in Computer Vision - symmetric transfer errors: 4.2.2 Geometric distance
     * - Multiple View Geometry in Computer Vision - model selection 4.7.1 RANSAC
     */
    float Initializer::CheckFundamental(const Matrix3f &F21, vector<bool> &vbMatchesInliers, float sigma) {
        const int N = mvMatches12.size();

        const float f11 = F21(0, 0);
        const float f12 = F21(0, 1);
        const float f13 = F21(0, 2);
        const float f21 = F21(1, 0);
        const float f22 = F21(1, 1);
        const float f23 = F21(1, 2);
        const float f31 = F21(2, 0);
        const float f32 = F21(2, 1);
        const float f33 = F21(2, 2);

        vbMatchesInliers.resize(N);

        float score = 0;

        const float th = 3.841;
        const float thScore = 5.991;

        const float invSigmaSquare = 1.0 / (sigma * sigma);

        for (int i = 0; i < N; i++) {
            bool bIn = true;

            const cv::KeyPoint &kp1 = mvKeys1[mvMatches12[i].first];
            const cv::KeyPoint &kp2 = mvKeys2[mvMatches12[i].second];

            const float u1 = kp1.pt.x;
            const float v1 = kp1.pt.y;
            const float u2 = kp2.pt.x;
            const float v2 = kp2.pt.y;

            // Reprojection error in second image
            // l2=F21x1=(a2,b2,c2)

            const float a2 = f11 * u1 + f12 * v1 + f13;
            const float b2 = f21 * u1 + f22 * v1 + f23;
            const float c2 = f31 * u1 + f32 * v1 + f33;

            const float num2 = a2 * u2 + b2 * v2 + c2;

            const float squareDist1 = num2 * num2 / (a2 * a2 + b2 * b2);

            const float chiSquare1 = squareDist1 * invSigmaSquare;

            if (chiSquare1 > th)
                bIn = false;
            else
                score += thScore - chiSquare1;

            // Reprojection error in second image
            // l1 =x2tF21=(a1,b1,c1)

            const float a1 = f11 * u2 + f21 * v2 + f31;
            const float b1 = f12 * u2 + f22 * v2 + f32;
            const float c1 = f13 * u2 + f23 * v2 + f33;

            const float num1 = a1 * u1 + b1 * v1 + c1;

            const float squareDist2 = num1 * num1 / (a1 * a1 + b1 * b1);

            const float chiSquare2 = squareDist2 * invSigmaSquare;

            if (chiSquare2 > th)
                bIn = false;
            else
                score += thScore - chiSquare2;

            if (bIn)
                vbMatchesInliers[i] = true;
            else
                vbMatchesInliers[i] = false;
        }

        return score;
    }

    bool Initializer::ReconstructF(
            vector<bool> &vbMatchesInliers, Matrix3f &F21, Matrix3f &K,
            Matrix3f &R21, Vector3f &t21, vector<Vector3f> &vP3D,
            vector<bool> &vbTriangulated, float minParallax, int minTriangulated) {
        int N = 0;
        for (size_t i = 0, iend = vbMatchesInliers.size(); i < iend; i++)
            if (vbMatchesInliers[i])
                N++;

        // Compute Essential Matrix from Fundamental Matrix
        Matrix3f E21 = K.transpose() * F21 * K;

        Matrix3f R1, R2;
        Vector3f t;

        // Recover the 4 motion hypotheses
        DecomposeE(E21, R1, R2, t);

        Vector3f t1 = t;
        Vector3f t2 = -t;

        // Reconstruct with the 4 hyphoteses and check
        vector<Vector3f> vP3D1, vP3D2, vP3D3, vP3D4;
        vector<bool> vbTriangulated1, vbTriangulated2, vbTriangulated3, vbTriangulated4;
        float parallax1, parallax2, parallax3, parallax4;

        int nGood1 = CheckRT(R1, t1, mvKeys1, mvKeys2, mvMatches12, vbMatchesInliers, K, vP3D1, 16.0 * mSigma2,
                             vbTriangulated1, parallax1);
        int nGood2 = CheckRT(R2, t1, mvKeys1, mvKeys2, mvMatches12, vbMatchesInliers, K, vP3D2, 16.0 * mSigma2,
                             vbTriangulated2, parallax2);
        int nGood3 = CheckRT(R1, t2, mvKeys1, mvKeys2, mvMatches12, vbMatchesInliers, K, vP3D3, 16.0 * mSigma2,
                             vbTriangulated3, parallax3);
        int nGood4 = CheckRT(R2, t2, mvKeys1, mvKeys2, mvMatches12, vbMatchesInliers, K, vP3D4, 16.0 * mSigma2,
                             vbTriangulated4, parallax4);

        int maxGood = max(nGood1, max(nGood2, max(nGood3, nGood4)));

        int nMinGood = max(static_cast<int>(0.9 * N), minTriangulated);

        int nsimilar = 0;
        if (nGood1 > 0.7 * maxGood)
            nsimilar++;
        if (nGood2 > 0.7 * maxGood)
            nsimilar++;
        if (nGood3 > 0.7 * maxGood)
            nsimilar++;
        if (nGood4 > 0.7 * maxGood)
            nsimilar++;

        // If there is not a clear winner or not enough triangulated points reject initialization
        if (maxGood < nMinGood || nsimilar > 1) {
            return false;
        }

        // If best reconstruction has enough parallax initialize
        if (maxGood == nGood1) {
            if (parallax1 > minParallax) {
                vP3D = vP3D1;
                vbTriangulated = vbTriangulated1;
                R21 = R1;
                t21 = t1;
                return true;
            }
        } else if (maxGood == nGood2) {
            if (parallax2 > minParallax) {
                vP3D = vP3D2;
                vbTriangulated = vbTriangulated2;
                R21 = R2;
                t21 = t1;
                return true;
            }
        } else if (maxGood == nGood3) {
            if (parallax3 > minParallax) {
                vP3D = vP3D3;
                vbTriangulated = vbTriangulated3;
                R21 = R1;
                t21 = t2;
                return true;
            }
        } else if (maxGood == nGood4) {
            if (parallax4 > minParallax) {
                vP3D = vP3D4;
                vbTriangulated = vbTriangulated4;
                R21 = R2;
                t21 = t2;
                return true;
            }
        }

        return false;
    }

    // H矩阵分解常见有两种方法：Faugeras SVD-based decomposition 和 Zhang SVD-based decomposition
    // 参考文献：Motion and structure from motion in a piecewise plannar environment
    // 这篇参考文献和下面的代码使用了Faugeras SVD-based decomposition算法
    /**
     * @brief 从H恢复R t
     *
     * @see
     * - Faugeras et al, Motion and structure from motion in a piecewise planar environment. International Journal of Pattern Recognition and Artificial Intelligence, 1988.
     * - Deeper understanding of the homography decomposition for vision-based control
     */
    bool Initializer::ReconstructH(
            vector<bool> &vbMatchesInliers, Matrix3f &H21, Matrix3f &K,
            Matrix3f &R21, Vector3f &t21, vector<Vector3f> &vP3D, vector<bool> &vbTriangulated, float minParallax,
            int minTriangulated) {
        int N = 0;
        for (size_t i = 0, iend = vbMatchesInliers.size(); i < iend; i++)
            if (vbMatchesInliers[i])
                N++;

        // We recover 8 motion hypotheses using the method of Faugeras et al.
        // Motion and structure from motion in a piecewise planar environment.
        // International Journal of Pattern Recognition and Artificial Intelligence, 1988

        Matrix3f invK = K.inverse();
        Matrix3f A = invK * H21 * K;
        Eigen::JacobiSVD<Matrix3f> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);

        Matrix3f V = svd.matrixV();
        Matrix3f U = svd.matrixU();

        Eigen::Vector3f sigma = svd.singularValues();
        float d1 = (sigma[0]);
        float d2 = (sigma[1]);
        float d3 = (sigma[2]);
        float s = U.determinant() * V.determinant();

        if (d1 / d2 < 1.00001 || d2 / d3 < 1.00001) {
            return false;
        }

        vector<Matrix3f> vR;
        vector<Vector3f> vt, vn;
        vR.reserve(8);
        vt.reserve(8);
        vn.reserve(8);

        //n'=[x1 0 x3] 4 posibilities e1=e3=1, e1=1 e3=-1, e1=-1 e3=1, e1=e3=-1
        float aux1 = sqrt((d1 * d1 - d2 * d2) / (d1 * d1 - d3 * d3));
        float aux3 = sqrt((d2 * d2 - d3 * d3) / (d1 * d1 - d3 * d3));
        float x1[] = {aux1, aux1, -aux1, -aux1};
        float x3[] = {aux3, -aux3, aux3, -aux3};

        //case d'=d2
        float aux_stheta = sqrt((d1 * d1 - d2 * d2) * (d2 * d2 - d3 * d3)) / ((d1 + d3) * d2);

        float ctheta = (d2 * d2 + d1 * d3) / ((d1 + d3) * d2);
        float stheta[] = {aux_stheta, -aux_stheta, -aux_stheta, aux_stheta};

        for (int i = 0; i < 4; i++) {
            Matrix3f Rp = Matrix3f::Identity();
            Rp(0, 0) = ctheta;
            Rp(0, 2) = -stheta[i];
            Rp(2, 0) = stheta[i];
            Rp(2, 2) = ctheta;
            Matrix3f R = s * U * Rp * V.transpose();
            vR.push_back(R);

            Vector3f tp;
            tp[0] = x1[i];
            tp[1] = 0;
            tp[2] = -x3[i];
            tp *= d1 - d3;

            Vector3f t = U * tp;
            t.normalize();
            vt.push_back(t);

            Vector3f np;
            np[0] = x1[i];
            np[1] = 0;
            np[2] = x3[i];

            Vector3f n = V * np;
            if (n[2] < 0)
                n = -n;
            vn.push_back(n);
        }

        //case d'=-d2
        float aux_sphi = sqrt((d1 * d1 - d2 * d2) * (d2 * d2 - d3 * d3)) / ((d1 - d3) * d2);

        float cphi = (d1 * d3 - d2 * d2) / ((d1 - d3) * d2);
        float sphi[] = {aux_sphi, -aux_sphi, -aux_sphi, aux_sphi};

        for (int i = 0; i < 4; i++) {
            Matrix3f Rp = Matrix3f::Identity();
            Rp(0, 0) = cphi;
            Rp(0, 2) = sphi[i];
            Rp(1, 1) = -1;
            Rp(2, 0) = sphi[i];
            Rp(2, 2) = -cphi;

            Matrix3f R = s * U * Rp * V.transpose();
            vR.push_back(R);

            Vector3f tp;
            tp[0] = x1[i];
            tp[1] = 0;
            tp[2] = x3[i];
            tp *= d1 + d3;

            Vector3f t = U * tp;
            t.normalize();
            vt.push_back(t);

            Vector3f np;
            np[0] = x1[i];
            np[1] = 0;
            np[2] = x3[i];

            Vector3f n = V * np;
            if (n[2] < 0)
                n = -n;
            vn.push_back(n);
        }


        int bestGood = 0;
        int secondBestGood = 0;
        int bestSolutionIdx = -1;
        float bestParallax = -1;
        vector<Vector3f> bestP3D;
        vector<bool> bestTriangulated;

        // Instead of applying the visibility constraints proposed in the Faugeras' paper (which could fail for points seen with low parallax)
        // We reconstruct all hypotheses and check in terms of triangulated points and parallax
        for (size_t i = 0; i < 8; i++) {
            float parallaxi;
            vector<Vector3f> vP3Di;
            vector<bool> vbTriangulatedi;
            int nGood = CheckRT(vR[i], vt[i], mvKeys1, mvKeys2, mvMatches12, vbMatchesInliers, K, vP3Di, 4.0 * mSigma2,
                                vbTriangulatedi, parallaxi);

            if (nGood > bestGood) {
                secondBestGood = bestGood;
                bestGood = nGood;
                bestSolutionIdx = i;
                bestParallax = parallaxi;
                bestP3D = vP3Di;
                bestTriangulated = vbTriangulatedi;
            } else if (nGood > secondBestGood) {
                secondBestGood = nGood;
            }
        }


        if (secondBestGood < 0.75 * bestGood && bestParallax >= minParallax && bestGood > minTriangulated &&
            bestGood > 0.9 * N) {
            R21 = vR[bestSolutionIdx];
            t21 = vt[bestSolutionIdx];
            vP3D = bestP3D;
            vbTriangulated = bestTriangulated;

            return true;
        }

        return false;
    }

    // Trianularization: 已知匹配特征点对{x x'} 和 各自相机矩阵{P P'}, 估计三维点 X
    // x' = P'X  x = PX
    // 它们都属于 x = aPX模型
    //                         |X|
    // |x|     |p1 p2  p3  p4 ||Y|     |x|    |--p0--||.|
    // |y| = a |p5 p6  p7  p8 ||Z| ===>|y| = a|--p1--||X|
    // |z|     |p9 p10 p11 p12||1|     |z|    |--p2--||.|
    // 采用DLT的方法：x叉乘PX = 0
    // |yp2 -  p1|     |0|
    // |p0 -  xp2| X = |0|
    // |xp1 - yp0|     |0|
    // 两个点:
    // |yp2   -  p1  |     |0|
    // |p0    -  xp2 | X = |0| ===> AX = 0
    // |y'p2' -  p1' |     |0|
    // |p0'   - x'p2'|     |0|
    // 变成程序中的形式：
    // |xp2  - p0 |     |0|
    // |yp2  - p1 | X = |0| ===> AX = 0
    // |x'p2'- p0'|     |0|
    // |y'p2'- p1'|     |0|
    /**
     * @brief 给定投影矩阵P1,P2和图像上的点kp1,kp2，从而恢复3D坐标
     *
     * @param kp1 特征点, in reference frame
     * @param kp2 特征点, in current frame
     * @param P1  投影矩阵P1
     * @param P2  投影矩阵P2
     * @param x3D 三维点
     * @see       Multiple View Geometry in Computer Vision - 12.2 Linear triangulation methods p312
     */
    void Initializer::Triangulate(
            const Vector2f &kp1, const Vector2f &kp2,
            const Eigen::Matrix<float, int(3), int(4)> &P1,
            const Eigen::Matrix<float, int(3), int(4)> &P2,
            Vector3f &x3D) {
        Eigen::Matrix4f A;
        A.block<1, 4>(0, 0) = kp1[0] * P1.block<1, 4>(2, 0) - P1.block<1, 4>(0, 0);
        A.block<1, 4>(1, 0) = kp1[1] * P1.block<1, 4>(2, 0) - P1.block<1, 4>(1, 0);
        A.block<1, 4>(2, 0) = kp2[0] * P2.block<1, 4>(2, 0) - P2.block<1, 4>(0, 0);
        A.block<1, 4>(3, 0) = kp2[1] * P2.block<1, 4>(2, 0) - P2.block<1, 4>(1, 0);
        Eigen::JacobiSVD<Eigen::Matrix4f> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
        x3D = svd.matrixV().block<3, 1>(0, 3) / svd.matrixV()(3, 3);
    }

    void Initializer::Normalize(const vector<cv::KeyPoint> &vKeys, vector<Vector2f> &vNormalizedPoints, Matrix3f &T) {
        float meanX = 0;
        float meanY = 0;
        const int N = vKeys.size();

        vNormalizedPoints.resize(N);

        for (int i = 0; i < N; i++) {
            meanX += vKeys[i].pt.x;
            meanY += vKeys[i].pt.y;
        }

        meanX = meanX / N;
        meanY = meanY / N;

        float meanDevX = 0;
        float meanDevY = 0;

        for (int i = 0; i < N; i++) {
            vNormalizedPoints[i][0] = vKeys[i].pt.x - meanX;
            vNormalizedPoints[i][1] = vKeys[i].pt.y - meanY;

            meanDevX += fabs(vNormalizedPoints[i][0]);
            meanDevY += fabs(vNormalizedPoints[i][1]);
        }

        meanDevX = meanDevX / N;
        meanDevY = meanDevY / N;

        float sX = 1.0 / meanDevX;
        float sY = 1.0 / meanDevY;

        for (int i = 0; i < N; i++) {
            vNormalizedPoints[i][0] = vNormalizedPoints[i][0] * sX;
            vNormalizedPoints[i][1] = vNormalizedPoints[i][1] * sY;
        }

        T = Matrix3f::Identity();
        T(0, 0) = sX;
        T(1, 1) = sY;
        T(0, 2) = -meanX * sX;
        T(1, 2) = -meanY * sY;
    }


    int Initializer::CheckRT(
            const Matrix3f &R, const Vector3f &t,
            const vector<cv::KeyPoint> &vKeys1,
            const vector<cv::KeyPoint> &vKeys2,
            const vector<Match> &vMatches12, vector<bool> &vbMatchesInliers,
            const Matrix3f &K, vector<Vector3f> &vP3D, float th2, vector<bool> &vbGood, float &parallax) {
        // Calibration parameters
        const double fx = K(0, 0);
        const double fy = K(1, 1);
        const double cx = K(0, 2);
        const double cy = K(1, 2);

        vbGood = vector<bool>(vKeys1.size(), false);
        vP3D.resize(vKeys1.size());

        vector<float> vCosParallax;
        vCosParallax.reserve(vKeys1.size());

        // Camera 1 Projection Matrix K[I|0]
        Eigen::Matrix<float, 3, 4> P1;
        P1.setZero();
        P1.block<3, 3>(0, 0) = K;
        Vector3f O1(0, 0, 0);

        // Camera 2 Projection Matrix K[R|t]
        Eigen::Matrix<float, 3, 4> P2;
        P2.block<3, 3>(0, 0) = R;
        P2.block<3, 1>(0, 3) = t;
        P2 = K * P2;
        Vector3f O2 = -R.transpose() * t;

        int nGood = 0;

        for (size_t i = 0, iend = vMatches12.size(); i < iend; i++) {
            if (!vbMatchesInliers[i])
                continue;

            const cv::KeyPoint &kp1 = vKeys1[vMatches12[i].first];
            const cv::KeyPoint &kp2 = vKeys2[vMatches12[i].second];
            Vector3f p3dC1;

            Triangulate(
                    Vector2f(kp1.pt.x, kp1.pt.y),
                    Vector2f(kp2.pt.x, kp2.pt.y),
                    P1, P2, p3dC1
            );

            if (!isfinite(p3dC1[0]) || !isfinite(p3dC1[1]) || !isfinite(p3dC1[2])) {
                vbGood[vMatches12[i].first] = false;
                continue;
            }

            // Check parallax
            Vector3f normal1 = p3dC1 - O1;
            float dist1 = normal1.norm();

            Vector3f normal2 = p3dC1 - O2;
            float dist2 = normal2.norm();

            float cosParallax = normal1.dot(normal2) / (dist1 * dist2);

            // Check depth in front of first camera (only if enough parallax, as "infinite" points can easily go to negative depth)
            if (p3dC1[2] <= 0 && cosParallax < 0.99998)
                continue;

            // Check depth in front of second camera (only if enough parallax, as "infinite" points can easily go to negative depth)
            Vector3f p3dC2 = R * p3dC1 + t;

            if (p3dC2[2] <= 0 && cosParallax < 0.99998)
                continue;

            // Check reprojection error in first image
            float im1x, im1y;
            float invZ1 = 1.0 / p3dC1[2];
            im1x = fx * p3dC1[0] * invZ1 + cx;
            im1y = fy * p3dC1[1] * invZ1 + cy;

            float squareError1 = (im1x - kp1.pt.x) * (im1x - kp1.pt.x) + (im1y - kp1.pt.y) * (im1y - kp1.pt.y);

            if (squareError1 > th2)
                continue;

            // Check reprojection error in second image
            float im2x, im2y;
            float invZ2 = 1.0 / p3dC2[2];
            im2x = fx * p3dC2[0] * invZ2 + cx;
            im2y = fy * p3dC2[1] * invZ2 + cy;

            float squareError2 = (im2x - kp2.pt.x) * (im2x - kp2.pt.x) + (im2y - kp2.pt.y) * (im2y - kp2.pt.y);

            if (squareError2 > th2)
                continue;

            vCosParallax.push_back(cosParallax);
            vP3D[vMatches12[i].first] = p3dC1;
            nGood++;

            if (cosParallax < 0.99998)
                vbGood[vMatches12[i].first] = true;
        }

        if (nGood > 0) {
            sort(vCosParallax.begin(), vCosParallax.end());

            size_t idx = min(50, int(vCosParallax.size() - 1));
            parallax = acos(vCosParallax[idx]) * 180 / CV_PI;
        } else
            parallax = 0;

        return nGood;
    }

    void Initializer::DecomposeE(
            const Matrix3f &E, Matrix3f &R1, Matrix3f &R2, Vector3f &t) {
        Eigen::JacobiSVD<Matrix3f> svd(E, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Matrix3f U = svd.matrixU(), V = svd.matrixV();
        t = U.block<3, 1>(0, 2);
        t = t / t.norm();

        Matrix3f W;
        W.setZero();
        W(0, 1) = -1;
        W(1, 0) = 1;
        W(2, 2) = 1;

        R1 = U * W * V.transpose();
        if (R1.determinant() < 0)
            R1 = -R1;

        R2 = U * W.transpose() * V.transpose();
        if (R2.determinant() < 0)
            R2 = -R2;

    }

} //namespace ygz
