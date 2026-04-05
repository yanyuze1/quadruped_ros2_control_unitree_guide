/*
 * 固定站立状态定义，BaseFixedStand 类继承自 FSMState，表示四足机器人处于固定站立状态时的行为逻辑，包括进入状态、持续执行和离开状态的函数，以及检查是否需要切换状态的函数。
 * 当机器人处于固定站立状态时，通常会将关节位置设置为预设的目标位置，并通过 PD 控制器进行控制，以保持机器人在该位置上稳定站立。
*/

#pragma once

#include "FSM/FSMState.h"

class BaseFixedStand : public FSMState
{
public:
    BaseFixedStand(CtrlInterfaces& ctrl_interfaces,
                   const std::vector<double>& target_pos,
                   double kp,
                   double kd);

    void enter() override;

    void run(const rclcpp::Time& time,
             const rclcpp::Duration& period) override;

    void exit() override;

    FSMStateName checkChange() override;

protected:
    double target_pos_[12] = {};
    double start_pos_[12] = {};
    rclcpp::Time start_time_;

    double kp_, kd_;

    double duration_ = 600; // steps
    double percent_ = 0; //%
    double phase = 0.0;
};
