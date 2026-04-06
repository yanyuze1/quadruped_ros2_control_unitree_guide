/*
 * 行走状态定义，StateTrotting 类继承自 FSMState 类，表示四足机器人处于行走状态时的行为逻辑，包括进入状态、持续执行和离开状态的函数，以及检查是否需要切换状态的函数。
 * 当机器人处于行走状态时，通常会根据外部输入或传感器数据动态调整机身姿态和位置。
 * 进入状态时先把关节 kp/kd 调高，让站立更硬一点。
 * 记录当前四只脚的位置作为参考支撑形状。
 * 运行时把摇杆输入映射成行走的目标位置和偏航。
 * 通过步态生成器计算步态参数，并通过平衡控制器计算每条腿的力，再通过机器人模型把足端力变成关节力矩输出。 
 */

#ifndef STATETROTTING_H
#define STATETROTTING_H
#include <unitree_guide/control/BalanceCtrl.h>
#include <unitree_guide/gait/GaitGenerator.h>
#include <unitree_guide/common/DebugData.h>
#include <FSM/FSMState.h>

class StateTrotting final : public FSMState {
public:
    explicit StateTrotting(CtrlInterfaces &ctrl_interfaces,
                           CtrlComponent &ctrl_component);

    void enter() override;

    void run(const rclcpp::Time &time,
             const rclcpp::Duration &period) override;

    void exit() override;

    FSMStateName checkChange() override;
    const StateTrottingDebugData &getDebugData() const { return debug_data_; }

private:
    void getUserCmd();

    void calcCmd();

    /**
    * Calculate the torque command
    */
    void calcTau();

    /**
    * Calculate the joint space velocity and acceleration
    */
    void calcQQd();

    /**
    * Calculate the PD gain for the joints
    */
    void calcGain() const;

    /**
     * Check whether the robot should take a step or not
     * @return
     */
    bool checkStepOrNot();

    std::shared_ptr<Estimator> &estimator_;
    std::shared_ptr<QuadrupedRobot> &robot_model_;
    std::shared_ptr<BalanceCtrl> &balance_ctrl_;
    std::shared_ptr<WaveGenerator> &wave_generator_;

    GaitGenerator gait_generator_;

    // Robot State
    Vec3 pos_body_, vel_body_;
    RotMat B2G_RotMat, G2B_RotMat;

    // Robot command
    Vec3 pcd_;
    Vec3 vel_target_, v_cmd_body_;
    double dt_;
    double yaw_cmd_{}, d_yaw_cmd_{}, d_yaw_cmd_past_{};
    Vec3 w_cmd_global_;
    Vec34 pos_feet_global_goal_, vel_feet_global_goal_;
    RotMat Rd;

    // Control Parameters
    double gait_height_;
    Vec3 pos_error_, vel_error_;
    Mat3 Kpp, Kdp, Kd_w_;
    double kp_w_;
    Mat3 Kp_swing_, Kd_swing_;
    Vec2 v_x_limit_, v_y_limit_, w_yaw_limit_;

    bool yaw_rate_mode_{false};
    bool yaw_rate_mode_last_{false};
    double yaw_hold_deadband_{0.05};
    StateTrottingDebugData debug_data_;

};


#endif //STATETROTTING_H
