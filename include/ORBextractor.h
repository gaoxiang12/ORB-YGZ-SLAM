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

#ifndef YGZ_ORBEXTRACTOR_H_
#define YGZ_ORBEXTRACTOR_H_

#include "Common.h"

// ORB 提取器

namespace ygz {

    class Frame;

    // ORB里用的四叉树均匀化
    class ExtractorNode {
    public:
        ExtractorNode() : bNoMore(false) {}

        void DivideNode(ExtractorNode &n1, ExtractorNode &n2, ExtractorNode &n3, ExtractorNode &n4);

        std::vector<cv::KeyPoint> vKeys;
        cv::Point2i UL, UR, BL, BR;
        std::list<ExtractorNode>::iterator lit;
        bool bNoMore;
    };

    class ORBextractor {
    public:

        enum {
            HARRIS_SCORE = 0, FAST_SCORE = 1
        };

        // select the keypoint method, support original ORB-SLAM, Grid FAST in SVO and a dynamic grid FAST (DSO like)
        // 使用哪种方法提取特征点，支持原版ORB,SVO里的网格FAST，以及类DSO的变网格FAST
        // 相对来说，原版 ORB-SLAM 的特征提取重复性较好，但比较慢。后两个快一些，但重复性差一些
        typedef enum {
            ORBSLAM_KEYPOINT, FAST_KEYPOINT, DSO_KEYPOINT
        } KeyPointMethod;

        ORBextractor(int nfeatures, float scaleFactor, int nlevels,
                     int iniThFAST, int minThFAST);

        ~ORBextractor() {}

        // Compute the ORB features and descriptors on an image.
        // ORB are dispersed on the image using an octree.
        // Mask is ignored in the current implementation.
        // 调用接口，给定图像，建立金字塔并计算关键点，接口与OpenCV相容
        void operator()(
                cv::InputArray image, cv::InputArray mask,
                std::vector<cv::KeyPoint> &keypoints,
                cv::OutputArray descriptors
        );

        // detect features for frame
        // 给某个单独的Frame调用的接口
        void operator()(Frame *frame,
                        std::vector<cv::KeyPoint> &keypoints,
                        cv::OutputArray descriptors,
                        KeyPointMethod method,
                        bool leftEye = true     // 是否是左眼（如果是双目的左眼，就要考虑现有的特征点，如果是右眼就可以随便提
        );

        int inline GetLevels() {
            return nlevels;
        }

        float inline GetScaleFactor() {
            return scaleFactor;
        }

        std::vector<float> inline GetScaleFactors() {
            return mvScaleFactor;
        }

        std::vector<float> inline GetInverseScaleFactors() {
            return mvInvScaleFactor;
        }

        std::vector<float> inline GetScaleSigmaSquares() {
            return mvLevelSigma2;
        }

        std::vector<float> inline GetInverseScaleSigmaSquares() {
            return mvInvLevelSigma2;
        }

        std::vector<cv::Mat> mvImagePyramid;

        void ComputePyramid(cv::Mat image);

    protected:
        // use the original fast lib to compute keypoints, faster than Opencv's implementation
        void ComputeKeyPointsFast(std::vector<std::vector<cv::KeyPoint>> &allKeypoints,
                                  std::vector<cv::KeyPoint> &exist_kps);

        // compute keypoints using orb slam's octree based implementation
        void ComputeKeyPointsOctTree(std::vector<std::vector<cv::KeyPoint> > &allKeypoints);

        // compute keypoints using dso's pixel selector
        // 注意这里计算的allkeypoints都是在当前金字塔分辨率下的，在提描述的时候才会乘scale
        // 但是dso可能会强行选出一些梯度不好的地方，导致align的时候出错
        void
        ComputeKeyPointsDSO(std::vector<std::vector<cv::KeyPoint>> &allKeypoints, std::vector<cv::KeyPoint> &exist_kps);

        // Single level DSO
        // 单层的DSO，只有原始图像分辨率下的特征点
        void ComputeKeyPointsDSOSingleLevel(
                std::vector<cv::KeyPoint> &allKeypoints,
                std::vector<cv::KeyPoint> &exist_kps
        );

        // 将提取的特征点按照四叉树均匀分布
        std::vector<cv::KeyPoint> DistributeOctTree(const std::vector<cv::KeyPoint> &vToDistributeKeys, const int &minX,
                                                    const int &maxX, const int &minY, const int &maxY,
                                                    const int &nFeatures, const int &level);

        // 老版 Opencv 的特征点
        void ComputeKeyPointsOld(std::vector<std::vector<cv::KeyPoint> > &allKeypoints);

        std::vector<cv::Point> pattern;

        // Shi-Tomasi 分数，这个分数越高则特征越优先
        float ShiTomasiScore(const cv::Mat &img, const int &u, const int &v) const;

        int nfeatures =0;
        double scaleFactor =0;
        int nlevels =0;
        int iniThFAST =0;
        int minThFAST =0;

        std::vector<int> mnFeaturesPerLevel;

        std::vector<int> umax;
        std::vector<float> mvScaleFactor;
        std::vector<float> mvInvScaleFactor;
        std::vector<float> mvLevelSigma2;
        std::vector<float> mvInvLevelSigma2;

        // grid fast
        /// 注意这里网格大小对特征重复性影响非常明显，一定不要调得太大！
        /// 特征重复性会直接影响到新地图点的生成。在5的时候可以生成大约100+个点，10的时候就只有20-50个点了,20时一般为个位数
        /// 然而网格太小则使得地图点过多，影响性能
        const int mnCellSize = 5;   // fixed grid size
        int mnGridCols =0;
        int mnGridRows =0;
        std::vector<bool> mvbGridOccupancy;

        cv::Mat map1, map2; // for distortion
        cv::Mat mOccupancy;

        int mnGridSize;     // dynamic grid size used in DSO
    };

} //namespace ygz

#endif

