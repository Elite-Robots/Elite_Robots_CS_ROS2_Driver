#ifndef __KIN_HPP__
#define __KIN_HPP__

#include <stdlib.h>
#include <string.h>
#include <array>
#include <cmath>
#include <iostream>
#include <memory>
#include <mutex>
#include <vector>

#include "kdl/chain.hpp"
#include "kdl/chainfksolverpos_recursive.hpp"
#include "kdl/chainiksolverpos_lma.hpp"
#include "kdl/chainjnttojacsolver.hpp"
#include "kdl/frames_io.hpp"

#define PI1 3.14
#define UNUSED(x) (void)(x)

namespace KDL {
class ChainIkSolverPos;
class ChainFkSolverPos_recursive;
class Chain;
class ChainJntToJacSolver;
class JntArray;
class Frame;
}  // namespace KDL

#define AXIS_COUNT 6  // 默认机械臂6自由度

#ifndef ROBOT_STRUCTURE_TYPE
#define ROBOT_STRUCTURE_TYPE int
#endif

#define MIN_LEN 1.0e-9

typedef double JOINTS[AXIS_COUNT];
typedef double VEC3D[3];
typedef double PNT3D[3];

typedef struct _rframe {
    VEC3D X;
    VEC3D Y;
    VEC3D Z;
    PNT3D O;
    double scale;
} RFRAME;

typedef double MAT4X4[4][4];

// ================== 基类接口（给 pluginlib 用） ==================
//在moveit2中可以不用写base类
class KinematicsBase {
   public:
    virtual ~KinematicsBase() = default;
    virtual void updateKinConfig(const double dh_a[AXIS_COUNT], const double dh_d[AXIS_COUNT],
                                 const double dh_alpha[AXIS_COUNT]) = 0;

    virtual void forward(const JOINTS joints, MAT4X4 mat,
                         const std::array<double, 6> &tcp_offset_xyzrxryrz = std::array<double, 6>{}) = 0;

    virtual int inverse(MAT4X4 mat, const JOINTS &joints_near, JOINTS &joints,
                        const std::array<double, 6> &tcp_offset_xyzRPY = std::array<double, 6>{}) = 0;
};
class Kinematics : public KinematicsBase {
   public:
    Kinematics();
    ~Kinematics() override;

    bool isDefault(void);

    void updateKinConfig(const double dh_a[AXIS_COUNT], const double dh_d[AXIS_COUNT], const double dh_alpha[AXIS_COUNT]) override;

    // api interface
    void forward(const JOINTS joints, MAT4X4 mat,
                 const std::array<double, 6> &tcp_offset_xyzrxryrz = std::array<double, 6>{}) override;

    int inverse(MAT4X4 mat, const JOINTS &joints_near, JOINTS &joints,
                const std::array<double, 6> &tcp_offset_xyzRPY = std::array<double, 6>{}) override;

    // 直接用外部构造好的 KDL Chain
    void updateKdlChain(const KDL::Chain &chain);

   private:
    std::unique_ptr<KDL::ChainIkSolverPos> _ikSolver;
    std::unique_ptr<KDL::ChainFkSolverPos_recursive> _fkSolver;
    std::unique_ptr<KDL::ChainJntToJacSolver> _jacSolver;
    std::unique_ptr<KDL::Chain> _robotChain;
    int _jointCount;

    std::mutex _dataMutex;
    double _dhA[AXIS_COUNT];
    double _dhD[AXIS_COUNT];
    double _dhAlpha[AXIS_COUNT];
};

#endif  // __KIN_HPP__
