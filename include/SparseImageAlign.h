#ifndef YGZ_SPARSE_IMAGE_ALIGN_
#define YGZ_SPARSE_IMAGE_ALIGN_

#include "Common.h"
#include "NLSSolver.h"
#include "Frame.h"

// 稀疏直接法求解器
// 请注意SVO的直接法用了一种逆向的奇怪解法，它的雅可比是在Ref中估计而不是在Current里估计的，所以迭代过程中雅可比是不动的

namespace ygz {

    /// Optimize the pose of the frame by minimizing the photometric error of feature patches.
    class SparseImgAlign : public NLLSSolver<6, SE3f> {
        static const int patch_halfsize_ = 2;
        static const int patch_size_ = 2 * patch_halfsize_;
        static const int patch_area_ = patch_size_ * patch_size_;
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        cv::Mat resimg_;

        /**
         * @brief constructor
         * @param[in] n_levels total pyramid level
         * @param[in] min_level minimum levels
         * @param[in] n_iter iterations
         * @param[in] methos GaussNewton or LevernbergMarquardt
         * @param[in] display display the residual image
         * @param[in] verbose output the inner computation information
         */
        SparseImgAlign(
                int n_levels,
                int min_level,
                int n_iter = 10,
                Method method = GaussNewton,
                bool display = false,
                bool verbose = false);

        /**
         * 计算 ref 和 current 之间的运动
         * @brief compute the relative motion between ref frame and current frame
         * @param[in] ref_frame the reference
         * @param[in] cur_frame the current frame
         * @param[out] TCR motion from ref to current
         */
        size_t run(
                Frame *ref_frame,
                Frame *cur_frame,
                SE3f &TCR
        );

        /// Return fisher information matrix, i.e. the Hessian of the log-likelihood
        /// at the converged state.
        Matrix<float, 6, 6> getFisherInformation();

    protected:
        Frame *ref_frame_;              //!< reference frame, has depth for gradient pixels.
        Frame *cur_frame_;              //!< only the image is known!
        int level_;                     //!< current pyramid level on which the optimization runs.
        bool display_;                  //!< display residual image.
        int max_level_;                 //!< coarsest pyramid level for the alignment.
        int min_level_;                 //!< finest pyramid level for the alignment.

        // cache:
        Matrix<float, 6, Dynamic, ColMajor> jacobian_cache_;    // 雅可比矩阵，这个东西是固定下来的

        bool have_ref_patch_cache_;
        cv::Mat ref_patch_cache_;
        std::vector<bool> visible_fts_;

        // 在ref中计算雅可比
        void precomputeReferencePatches();

        // 派生出来的虚函数
        virtual float computeResiduals(const SE3f &model, bool linearize_system, bool compute_weight_scale = false);

        virtual int solve();

        virtual void update(const ModelType &old_model, ModelType &new_model);

        virtual void startIteration();

        virtual void finishIteration();

        // *************************************************************************************
        // 一些固定的雅可比
        // xyz 到 相机坐标 的雅可比，平移在前
        // 这里已经取了负号，不要再取一遍！
        inline Eigen::Matrix<float, 2, 6> JacobXYZ2Cam(const Vector3f &xyz) {
            Eigen::Matrix<float, 2, 6> J;
            const float x = xyz[0];
            const float y = xyz[1];
            const float z_inv = 1. / xyz[2];
            const float z_inv_2 = z_inv * z_inv;

            J(0, 0) = -z_inv;           // -1/z
            J(0, 1) = 0.0;              // 0
            J(0, 2) = x * z_inv_2;        // x/z^2
            J(0, 3) = y * J(0, 2);      // x*y/z^2
            J(0, 4) = -(1.0 + x * J(0, 2)); // -(1.0 + x^2/z^2)
            J(0, 5) = y * z_inv;          // y/z

            J(1, 0) = 0.0;              // 0
            J(1, 1) = -z_inv;           // -1/z
            J(1, 2) = y * z_inv_2;        // y/z^2
            J(1, 3) = 1.0 + y * J(1, 2); // 1.0 + y^2/z^2
            J(1, 4) = -J(0, 3);       // -x*y/z^2
            J(1, 5) = -x * z_inv;         // x/z
            return J;
        }

    };

}// namespace ygz 


#endif