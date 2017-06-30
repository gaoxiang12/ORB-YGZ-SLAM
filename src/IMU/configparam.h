#ifndef YGZ_CONFIGPARAM_H_
#define YGZ_CONFIGPARAM_H_

#include "Common.h"

namespace ygz {

    class ConfigParam {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        ConfigParam(std::string configfile);

        static bool GetUseIMUFlag() { return _bUseIMU; }

        double _testDiscardTime;

        static SE3d GetSE3Tbc() { return _SE3Tbc; }

        static Matrix4d GetEigTbc();

        static Mat GetMatTbc();

        static Matrix4d GetEigT_cb();

        static Mat GetMatT_cb();

        static int GetLocalWindowSize();

        static double GetImageDelayToIMU();

        static bool GetAccMultiply9p8();

        static double GetG() { return _g; }

        std::string _bagfile;
        std::string _imageTopic;
        std::string _imuTopic;

        static std::string getTmpFilePath();

        static std::string _tmpFilePath;

        static double GetVINSInitTime() { return _nVINSInitTime; }

    private:
        static SE3d _SE3Tbc;
        static Matrix4d _EigTbc;
        static Mat _MatTbc;
        static Matrix4d _EigTcb;
        static Mat _MatTcb;
        static int _LocalWindowSize;
        static double _ImageDelayToIMU;
        static bool _bAccMultiply9p8;

        static double _g;
        static double _nVINSInitTime;
        static bool _bUseIMU;

    };

}

#endif // CONFIGPARAM_H
