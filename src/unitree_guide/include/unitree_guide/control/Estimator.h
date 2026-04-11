/*
* 这个文件定义了一个基于卡尔曼滤波的状态估计器类 Estimator，用于估计四足机器人的状态，包括位置、速度、脚的位置和速度等。
* 这个类的主要功能是根据机器人当前的状态和传感器测量值，使用卡尔曼滤波算法来估计机器人的状态，并提供接口供其他模块调用。
* 主要成员变量包括状态向量 x_hat_、输入向量 u_、测量向量 y_、预测测量向量 y_hat_、状态转移矩阵 A、输入矩阵 B、输出矩阵 C，以及协方差矩阵 P、Q、R 等。
* 主要成员函数包括构造函数、状态获取函数（如 getPosition()、getVelocity()、getFootPos() 等）和 update() 函数，其中 update() 函数实现了卡尔曼滤波的预测和更新步骤。
*/

#ifndef ESTIMATOR_H
#define ESTIMATOR_H
#include <memory>
#include <kdl/frames.hpp>
#include <unitree_guide/common/mathTypes.h>
#include <unitree_guide/robot/QuadrupedRobot.h>
#include <unitree_guide/control/LowPassFilter.h>
#include <unitree_guide/common/DebugData.h>

struct CtrlInterfaces;
class WaveGenerator;
class QuadrupedRobot;
struct CtrlComponent;

class Estimator {
public:
    explicit Estimator(CtrlInterfaces &ctrl_interfaces, CtrlComponent &ctrl_component);
    const EstimatorDebugData &getDebugData() const { return debug_data_; }

    ~Estimator() = default;

    /**
     * Get the estimated robot central position
     * @return robot central position
     * 估计机身位置
     */
    Vec3 getPosition() {
        return x_hat_.segment(0, 3);
    }

    /**
     * Get the estimated robot central velocity
     * @return robot central velocity
     * 估计机身速度
     */
    Vec3 getVelocity() {
        return x_hat_.segment(3, 3);
    }

    /**
     * Get the estimated foot position in world frame
     * @param index leg index
     * @return foot position in world frame
     * 估计单个足端在世界坐标系下的位置
     */
    Vec3 getFootPos(const int index) {
        return getPosition() + rotation_ * Vec3(foot_poses_[index].p.data);
    }

    /**
     * Get the estimated feet velocity in world frame
     * @return feet velocity in world frame
     * 估计四只足端在世界坐标系下的位置
     */
    Vec34 getFeetPos() {
        Vec34 feet_pos;
        for (int i(0); i < 4; ++i) {
            feet_pos.col(i) = getFootPos(i);
        }
        return feet_pos;
    }

    /**
     * Get the estimated feet velocity in world frame
     * @return feet velocity in world frame
     * 估计四只足端在世界坐标系下的速度
     */
    // Vec34 getFeetVel() {
    //     const std::vector<KDL::Vector> feet_vel = robot_model_->getFeet2BVelocities();
    //     Vec34 result;
    //     for (int i(0); i < 4; ++i) {
    //         result.col(i) = Vec3(feet_vel[i].data) + getVelocity();
    //     }
    //     return result;
    // }

    Vec34 getFeetVel() {
        const std::vector<KDL::Vector> feet_vel_body_list = robot_model_->getFeet2BVelocities();
        const std::vector<KDL::Frame> feet_pos_body_list = robot_model_->getFeet2BPositions();
        Vec34 result;
        const Vec3 body_vel_world = getVelocity();

        for (int i(0); i < 4; ++i) {
            const Vec3 foot_pos_body = Vec3(feet_pos_body_list[i].p.data);
            const Vec3 foot_vel_body = Vec3(feet_vel_body_list[i].data);

            // [修改] 返回“世界系足端速度”
            // 原代码少了 body->world 旋转，也少了 omega x r 项
            result.col(i) = body_vel_world +
                            rotation_ * (gyro_.cross(foot_pos_body) + foot_vel_body);
        }
        return result;
    }


    /**
     * Get the estimated foot position in body frame
     * @return
     * 足端相对机身的位置
     */
    Vec34 getFeetPos2Body() {
        Vec34 foot_pos;
        const Vec3 body_pos = getPosition();
        for (int i = 0; i < 4; i++) {
            foot_pos.col(i) = getFootPos(i) - body_pos;
        }
        return foot_pos;
    }

    /*
     * 旋转矩阵
     */
    RotMat getRotation() {
        return rotation_;
    }

    /*
     * imu测算的机身坐标下的角速度
     */
    Vec3 getGyro() {
        return gyro_;
    }

    /*
     * imu测算的世界坐标下的角速度
     */
    [[nodiscard]] Vec3 getGyroGlobal() const {
        return rotation_ * gyro_;
    }

    /*
    * 当前机身的偏航角yaw
    */
    [[nodiscard]] double getYaw() const;

    /*
     * 当前机身偏航角yaw的变化率，导数
     */
    [[nodiscard]] double getDYaw() const {
        return getGyroGlobal()(2);
    }

    void update();

private:
    CtrlInterfaces &ctrl_interfaces_;
    std::shared_ptr<QuadrupedRobot> &robot_model_;
    std::shared_ptr<WaveGenerator> &wave_generator_;

    Eigen::Matrix<double, 18, 1> x_hat_; // The state of estimator, position(3)+velocity(3)+feet position(3x4)

    Eigen::Matrix<double, 3, 1> u_; // The input of estimator

    Eigen::Matrix<double, 28, 1> y_; // The measurement value of output y
    Eigen::Matrix<double, 28, 1> y_hat_; // The prediction of output y
    Eigen::Matrix<double, 18, 18> A; // The transtion matrix of estimator
    Eigen::Matrix<double, 18, 3> B; // The input matrix
    Eigen::Matrix<double, 28, 18> C; // The output matrix

    // Covariance Matrix
   Eigen::Matrix<double, 18, 18> P; // Prediction covariance，协方差矩阵
    Eigen::Matrix<double, 18, 18> Ppriori; // Priori prediction covariance，先验协方差矩阵
    Eigen::Matrix<double, 18, 18> Q; // Dynamic simulation covariance，动态仿真协方差矩阵
    Eigen::Matrix<double, 28, 28> R; // Measurement covariance，测量噪声协方差矩阵
    Eigen::Matrix<double, 18, 18> QInit_; // Initial value of Dynamic simulation covariance，动态仿真协方差矩阵初始值
    Eigen::Matrix<double, 28, 28> RInit_; // Initial value of Measurement covariance，测量噪声协方差矩阵初始值
    Eigen::Matrix<double, 18, 1> Qdig; // adjustable process noise covariance，可调节的过程噪声协方差
    Eigen::Matrix<double, 3, 3> Cu; // The covariance of system input u，系统输入u的协方差矩阵

    // Output Measurement
    Eigen::Matrix<double, 12, 1> feet_pos_body_; // The feet positions to body, in the global coordinate
    Eigen::Matrix<double, 12, 1> feet_vel_body_; // The feet velocity to body, in the global coordinate
    Eigen::Matrix<double, 4, 1> feet_h_; // The Height of each foot, in the global coordinate

    Eigen::Matrix<double, 28, 28> S; // _S = C*P*C.T + R，残差协方差矩阵
    Eigen::PartialPivLU<Eigen::Matrix<double, 28, 28> > Slu; // _S.lu()，S矩阵的LU分解，用于求逆
    Eigen::Matrix<double, 28, 1> Sy; // _Sy = _S.inv() * (y - yhat)，残差加权矩阵，S.inv()=S^{-1}
    Eigen::Matrix<double, 28, 18> Sc; // _Sc = _S.inv() * C，残差加权矩阵与输出矩阵的乘积
    Eigen::Matrix<double, 28, 28> SR; // _SR = _S.inv() * R，残差加权矩阵与测量噪声协方差矩阵的乘积
    Eigen::Matrix<double, 28, 18> STC; // _STC = (_S.transpose()).inv() * C，残差加权矩阵转置与输出矩阵的乘积，(S^T)^{-1}C
    Eigen::Matrix<double, 18, 18> IKC; // _IKC = I - KC，卡尔曼增益矩阵K与输出矩阵C的乘积的差，I为单位矩阵
    EstimatorDebugData debug_data_;


    Vec3 g_;
    double dt_;

    RotMat rotation_;
    Vec3 acceleration_;
    Vec3 gyro_;

    std::vector<KDL::Frame> foot_poses_;
    std::vector<KDL::Vector> foot_vels_;
    std::vector<std::shared_ptr<LowPassFilter> > low_pass_filters_;

    double large_variance_;
};


#endif //ESTIMATOR_H
