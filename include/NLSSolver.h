#ifndef YGZ_NLSSOLVER_H_
#define YGZ_NLSSOLVER_H_

#include "RobustCost.h"
#include "Common.h"

// the nls solver from vikit in svo
// （非常简单的）非线性优化求解器
// 实现在NLSSolver_impl.hpp里
// 随后在SparseImageAlign.h中继承，因为那东西本身就是个非线性优化
// 请勿吐槽代码风格和ORB不一样，我也知道不一样

using namespace Eigen;

namespace ygz {

    /**
     * \brief Abstract Class for solving nonlinear least-squares (NLLS) problems.
     *
     * The function implements two algorithms: Levenberg Marquardt and Gauss Newton
     *
     * Example implementations of this function can be found in the rpl_examples
     * package: img_align_2d.cpp, img_align_3d.cpp
     *
     * Template Parameters:
     * D  : dimension of the residual
     * T  : type of the model, e.g. SE2, SE3
     */

    template<int D, typename T> // D是维度，E是模型类型
    class NLLSSolver {

    public:
        typedef T ModelType;
        // 高斯牛顿或LM优化
        enum Method {
            GaussNewton, LevenbergMarquardt
        };
        enum ScaleEstimatorType {
            UnitScale, TDistScale, MADScale, NormalScale
        };
        enum WeightFunctionType {
            UnitWeight, TDistWeight, TukeyWeight, HuberWeight
        };

    protected:
        Matrix<float, D, D> H_;       //!< Hessian approximation
        Matrix<float, D, 1> Jres_;    //!< Jacobian x Residual
        Matrix<float, D, 1> x_;       //!< update step

        // 是否有先验
        bool have_prior_;
        ModelType prior_;
        Matrix<float, D, D> I_prior_; //!< Prior information matrix (inverse covariance)

        float chi2_;
        float rho_;
        Method method_;

        /// If the flag linearize_system is set, the function must also compute the
        /// Jacobian and set the member variables H_, Jres_
        virtual float
        computeResiduals(const ModelType &model,
                         bool linearize_system,
                         bool compute_weight_scale) = 0;

        /// Solve the linear system H*x = Jres. This function must set the update
        /// step in the member variable x_. Must return true if the system could be
        /// solved and false if it was singular.
        virtual int
        solve() = 0;

        virtual void
        update(const ModelType &old_model, ModelType &new_model) = 0;

        virtual void
        applyPrior(const ModelType &current_model) {}

        virtual void
        startIteration() {}

        virtual void
        finishIteration() {}

        virtual void
        finishTrial() {}

    public:

        /// Damping parameter. If mu > 0, coefficient matrix is positive definite, this
        /// ensures that x is a descent direction. If mu is large, x is a short step in
        /// the steepest direction. This is good if the current iterate is far from the
        /// solution. If mu is small, LM approximates gauss newton iteration and we
        /// have (almost) quadratic convergence in the final stages.
        float mu_init_, mu_;
        float nu_init_, nu_;          //!< Increase factor of mu after fail
        size_t n_iter_init_, n_iter_;  //!< Number of Iterations
        size_t n_trials_;              //!< Number of trials
        size_t n_trials_max_;          //!< Max number of trials
        size_t n_meas_;                //!< Number of measurements
        bool stop_;                  //!< Stop flag
        bool verbose_;               //!< Output Statistics
        float eps_;                   //!< Stop if update norm is smaller than eps
        size_t iter_;                  //!< Current Iteration
        bool error_increased_;


        // robust least squares
        bool use_weights_;
        float scale_;
        robust_cost::ScaleEstimatorPtr scale_estimator_;
        robust_cost::WeightFunctionPtr weight_function_;

        NLLSSolver() :
                have_prior_(false),
                method_(LevenbergMarquardt),
                mu_init_(0.01f),
                mu_(mu_init_),
                nu_init_(2.0),
                nu_(nu_init_),
                n_iter_init_(15),
                n_iter_(n_iter_init_),
                n_trials_(0),
                n_trials_max_(5),
                n_meas_(0),
                stop_(false),
                verbose_(true),
                eps_(0.0000000001),
                iter_(0),
                use_weights_(false),
                scale_(0.0),
                scale_estimator_(NULL),
                weight_function_(NULL) {}

        virtual ~NLLSSolver() {}

        /// Calls the GaussNewton or LevenbergMarquardt optimization strategy
        void optimize(ModelType &model);

        /// Gauss Newton optimization strategy
        void optimizeGaussNewton(ModelType &model);

        /// Levenberg Marquardt optimization strategy
        void optimizeLevenbergMarquardt(ModelType &model);

        /// Specify the robust cost that should be used and the appropriate scale estimator
        void setRobustCostFunction(
                ScaleEstimatorType scale_estimator,
                WeightFunctionType weight_function);

        /// Add prior to optimization.
        void setPrior(
                const ModelType &prior,
                const Matrix<float, D, D> &Information);

        /// Reset all parameters to restart the optimization
        void reset();

        /// Get the squared error
        const float &getChi2() const;

        /// The Information matrix is equal to the inverse covariance matrix.
        const Matrix<float, D, D> &getInformationMatrix() const;
    };


    inline float norm_max(const Eigen::VectorXf &v) {
        float max = -1;
        for (int i = 0; i < v.size(); i++) {
            float abs = fabs(v[i]);
            if (abs > max) {
                max = abs;
            }
        }
        return max;
    }

#include "NLSSolver_impl.hpp"

}

#endif
