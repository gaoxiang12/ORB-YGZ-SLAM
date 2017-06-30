#ifndef YGZ_IMUDATA_H_
#define YGZ_IMUDATA_H_

#include "Common.h"

namespace ygz {

    using namespace Eigen;

    class IMUData {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        IMUData(const double &gx, const double &gy, const double &gz,
                const double &ax, const double &ay, const double &az,
                const double &t) : _g(gx, gy, gz), _a(ax, ay, az), _t(t) {}

        // covariance of measurement
        static Matrix3d _gyrMeasCov;
        static Matrix3d _accMeasCov;

        static Matrix3d getGyrMeasCov(void) { return _gyrMeasCov; }

        static Matrix3d getAccMeasCov(void) { return _accMeasCov; }

        // covariance of bias random walk
        static Matrix3d _gyrBiasRWCov;
        static Matrix3d _accBiasRWCov;

        static Matrix3d getGyrBiasRWCov(void) { return _gyrBiasRWCov; }

        static Matrix3d getAccBiasRWCov(void) { return _accBiasRWCov; }

        static double _gyrBiasRw2;
        static double _accBiasRw2;

        static double getGyrBiasRW2(void) { return _gyrBiasRw2; }

        static double getAccBiasRW2(void) { return _accBiasRw2; }

        // Raw data of imu
        Vector3d _g;    //gyr data
        Vector3d _a;    //acc data
        double _t;      //timestamp
    };

}

#endif // IMUDATA_H
