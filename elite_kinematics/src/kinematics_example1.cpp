
#include "elite_kinematics/kinematics_example1.hpp"
#include <pluginlib/class_list_macros.hpp>
#include "Log.hpp"
static const RFRAME base_frame = {
    .X = {-1.0, 0.0, 0.0},
    .Y = {0.0, -1.0, 0.0},
    .Z = {0.0, 0.0, 1.0},
    .O = {0},
    .scale = 1.0,
};

static const RFRAME flange_frame = {
    .X = {0.0, 0.0, 1.0},
    .Y = {-1.0, 0.0, 0.0},
    .Z = {0.0, -1.0, 0.0},
    .O = {0},
    .scale = 1.0,
};

static void KDLJointsToJoints(const KDL::JntArray &kdl_joints, JOINTS joints) {
    for (int i = 0; i < AXIS_COUNT; i++) {
        joints[i] = kdl_joints(i);
    }
}

static KDL::JntArray JointsToKDLJoints(const JOINTS joints) {
    KDL::JntArray kdl_joints(AXIS_COUNT);
    for (int i = 0; i < AXIS_COUNT; i++) {
        kdl_joints(i) = joints[i];
    }
    return kdl_joints;
}

static void KDLFrameToMat4x4(const KDL::Frame &frame, MAT4X4 mat4x4) {
    mat4x4[0][0] = frame.M.data[0];
    mat4x4[0][1] = frame.M.data[1];
    mat4x4[0][2] = frame.M.data[2];
    mat4x4[0][3] = frame.p.data[0];

    mat4x4[1][0] = frame.M.data[3];
    mat4x4[1][1] = frame.M.data[4];
    mat4x4[1][2] = frame.M.data[5];
    mat4x4[1][3] = frame.p.data[1];

    mat4x4[2][0] = frame.M.data[6];
    mat4x4[2][1] = frame.M.data[7];
    mat4x4[2][2] = frame.M.data[8];
    mat4x4[2][3] = frame.p.data[2];

    mat4x4[3][0] = 0;
    mat4x4[3][1] = 0;
    mat4x4[3][2] = 0;
    mat4x4[3][3] = 1;
}

static KDL::Frame Mat4x4ToKDLFrame(const MAT4X4 mat4x4) {
    KDL::Frame frame = KDL::Frame::Identity();
    frame.M.data[0] = mat4x4[0][0];
    frame.M.data[1] = mat4x4[0][1];
    frame.M.data[2] = mat4x4[0][2];
    frame.p.data[0] = mat4x4[0][3];

    frame.M.data[3] = mat4x4[1][0];
    frame.M.data[4] = mat4x4[1][1];
    frame.M.data[5] = mat4x4[1][2];
    frame.p.data[1] = mat4x4[1][3];

    frame.M.data[6] = mat4x4[2][0];
    frame.M.data[7] = mat4x4[2][1];
    frame.M.data[8] = mat4x4[2][2];
    frame.p.data[2] = mat4x4[2][3];
    return frame;
}

Kinematics::Kinematics() {
    _jointCount = AXIS_COUNT;
    _ikSolver = nullptr;
    _fkSolver = nullptr;
    _jacSolver = nullptr;
    _robotChain = nullptr;
    memset(_dhA, 0, sizeof(_dhA));
    memset(_dhD, 0, sizeof(_dhD));
    memset(_dhAlpha, 0, sizeof(_dhAlpha));
}

Kinematics::~Kinematics() {
    _ikSolver.reset();
    _fkSolver.reset();
    _jacSolver.reset();
    _robotChain.reset();
}

void Kinematics::forward(const JOINTS joints, MAT4X4 mat, const std::array<double, 6> &tcp_offset_xyzRPY) {
    KDL::JntArray kdl_joints = JointsToKDLJoints(joints);
    KDL::Frame end_effector_frame;

    if (_fkSolver == nullptr) {
        ELITE_LOG_ERROR("Please set Kinematics config first by Function: setKDLconfig !");
        return;
    }
    if (tcp_offset_xyzRPY.size() != 6) {
        ELITE_LOG_ERROR("TCP offset must have 6 elements!");
        return;
    }
    _fkSolver->JntToCart(kdl_joints, end_effector_frame);

    KDL::Rotation rot;

    // RPY
    rot = KDL::Rotation::EulerZYX(tcp_offset_xyzRPY[5], tcp_offset_xyzRPY[4], tcp_offset_xyzRPY[3]);

    KDL::Frame tcp_offset(rot, KDL::Vector(tcp_offset_xyzRPY[0], tcp_offset_xyzRPY[1], tcp_offset_xyzRPY[2]));
    KDL::Frame tcp_frame = end_effector_frame * tcp_offset;

    KDLFrameToMat4x4(tcp_frame, mat);
}

int Kinematics::inverse(MAT4X4 mat, const JOINTS &joints_near, JOINTS &joints, const std::array<double, 6> &tcp_offset_xyzRPY) {
    if (_ikSolver == nullptr) {
        return -1;
    }
    KDL::Rotation rot;
    rot = KDL::Rotation::RPY(tcp_offset_xyzRPY[3], tcp_offset_xyzRPY[4], tcp_offset_xyzRPY[5]);
    KDL::Frame tcp_offset(rot, KDL::Vector(tcp_offset_xyzRPY[0], tcp_offset_xyzRPY[1], tcp_offset_xyzRPY[2]));

    KDL::JntArray kdl_joints_near = JointsToKDLJoints(joints_near);
    KDL::Frame frame = Mat4x4ToKDLFrame(mat);
    frame = frame * tcp_offset.Inverse();
    KDL::JntArray ret_joints(_jointCount);
    int result = _ikSolver->CartToJnt(kdl_joints_near, frame, ret_joints);
    if (result == KDL::SolverI::E_NOERROR) {
        KDLJointsToJoints(ret_joints, joints);
    }
    return (result == KDL::SolverI::E_NOERROR) ? 0 : -1;
}

bool Kinematics::isDefault(void) { return false; }

void Kinematics::updateKinConfig(const double dh_a[AXIS_COUNT], const double dh_d[AXIS_COUNT], const double dh_alpha[AXIS_COUNT]) {
    _dataMutex.lock();

    // 自动释放之前的资源
    _ikSolver.reset();
    _fkSolver.reset();
    _jacSolver.reset();
    _robotChain.reset();

    _robotChain = std::make_unique<KDL::Chain>();

    for (int i = 0; i < _jointCount; i++) {
        _dhA[i] = dh_a[i];
        _dhD[i] = dh_d[i];
        _dhAlpha[i] = dh_alpha[i];
    }

    // 根据新的 DH 参数配置机器人链
    for (int i = 0; i < _jointCount; i++) {
        KDL::Frame rot = KDL::Frame(KDL::Rotation(KDL::Vector(1, 0, 0), KDL::Vector(0, cos(_dhAlpha[i]), sin(_dhAlpha[i])),
                                                  KDL::Vector(0, -sin(_dhAlpha[i]), cos(_dhAlpha[i]))));
        KDL::Frame trans = KDL::Frame(KDL::Vector(_dhA[i], 0, _dhD[i]));
        _robotChain->addSegment(KDL::Segment("Link" + std::to_string(i), KDL::Joint(KDL::Joint::None), rot * trans));
        _robotChain->addSegment(KDL::Segment("Link" + std::to_string(i), KDL::Joint(KDL::Joint::RotZ)));
    }

    _ikSolver = std::make_unique<KDL::ChainIkSolverPos_LMA>(*(_robotChain), 1E-10, 500);
    _fkSolver = std::make_unique<KDL::ChainFkSolverPos_recursive>(*(_robotChain));
    _jacSolver = std::make_unique<KDL::ChainJntToJacSolver>(*(_robotChain));

    _dataMutex.unlock();
}

void Kinematics::updateKdlChain(const KDL::Chain &chain) {
    std::lock_guard<std::mutex> lock(_dataMutex);

    _ikSolver.reset();
    _fkSolver.reset();
    _jacSolver.reset();
    _robotChain = std::make_unique<KDL::Chain>(chain);
    _jointCount = chain.getNrOfJoints();

    _ikSolver = std::make_unique<KDL::ChainIkSolverPos_LMA>(*_robotChain, 1E-10, 500);
    _fkSolver = std::make_unique<KDL::ChainFkSolverPos_recursive>(*_robotChain);
    _jacSolver = std::make_unique<KDL::ChainJntToJacSolver>(*_robotChain);
}
// =============== 导出为 pluginlib 插件 ===============
PLUGINLIB_EXPORT_CLASS(Kinematics, KinematicsBase)
