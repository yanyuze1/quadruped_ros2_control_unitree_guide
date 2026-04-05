/**
 * 自由站立状态定义，StateFreeStand 类继承自 FSMState 类，表示四足机器人处于自由站立状态时的行为逻辑，包括进入状态、持续执行和离开状态的函数，以及检查是否需要切换状态的函数。
 * 当机器人处于自由站立状态时，通常会根据外部输入或传感器数据动态调整机身姿态和位置。
 * 进入状态时先把关节 kp/kd 调高，让站立更硬一点。
 * 记录当前四只脚的位置作为参考支撑形状。
 * 运行时把摇杆输入映射成 roll / pitch / yaw / height 目标。
 * 通过运动学计算新的四腿目标关节角，并直接下发位置命令。
 * 它不负责走路，只负责“站着调身体”。
 */

#ifndef STATEFREESTAND_H
#define STATEFREESTAND_H
#include <robot/QuadrupedRobot.h>

#include <FSM/FSMState.h>

struct CtrlComponent;

class StateFreeStand final : public FSMState {
public:
    explicit StateFreeStand(CtrlInterfaces &ctrl_interfaces,
                            CtrlComponent &ctrl_component);

    void enter() override;

    void run(const rclcpp::Time &time,
             const rclcpp::Duration &period) override;

    void exit() override;

    FSMStateName checkChange() override;

private:
    std::shared_ptr<QuadrupedRobot> &robot_model_;

    void calc_body_target(float row, float pitch, float yaw, float height);

    float row_max_, row_min_;
    float pitch_max_, pitch_min_;
    float yaw_max_, yaw_min_;
    float height_max_, height_min_;

    std::vector<KDL::JntArray> init_joint_pos_;
    std::vector<KDL::JntArray> target_joint_pos_;

    KDL::Frame fr_init_pos_;
    std::vector<KDL::Frame> init_foot_pos_; // 4 feet position in fr-foot frame
};

#endif //STATEFREESTAND_H
