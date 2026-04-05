/**
 * 急停状态定义，StatePassive 类继承自 FSMState，表示四足机器人处于被动状态时的行为逻辑，包括进入状态、持续执行和离开状态的函数，以及检查是否需要切换状态的函数。
 * 当机器人处于被动状态时，通常会关闭所有控制命令输出，使机器人保持静止状态，以确保安全。
 */

#ifndef STATEPASSIVE_H
#define STATEPASSIVE_H
#include <FSM/FSMState.h>

class StatePassive final : public FSMState
{
public:
    explicit StatePassive(CtrlInterfaces& ctrl_interfaces);

    void enter() override;

    void run(const rclcpp::Time& time,
             const rclcpp::Duration& period) override;

    void exit() override;

    FSMStateName checkChange() override;
};


#endif //STATEPASSIVE_H
