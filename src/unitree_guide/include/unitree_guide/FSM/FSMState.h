/*
 * 状态机状态管理定义
    *所有状态都由enter、run、exit三种状态构成
    *enter：进入状态时执行一次的函数，通常用于状态初始化
    *run：状态持续执行的函数，在update()中被调用，通常用于状态的主要逻辑
    *exit：离开状态时执行一次的函数，通常用于状态清理
    *checkChange：检查是否需要切换状态的函数，在run()中被调用，返回下一个状态的枚举值，如果不需要切换则返回当前状态的枚举值
*/

#ifndef FSMSTATE_H
#define FSMSTATE_H

#include <string>
#include <utility>
#include <common/enumClass.h>
#include <unitree_guide/CtrlInterfaces.h>
#include <rclcpp/time.hpp>

class FSMState
{
public:
    virtual ~FSMState() = default;

    FSMState(const FSMStateName& state_name, std::string state_name_string, CtrlInterfaces& ctrl_interfaces)
        : state_name(state_name),
          state_name_string(std::move(state_name_string)),
          ctrl_interfaces_(ctrl_interfaces)
    {
    }

    virtual void enter() = 0;

    virtual void run(const rclcpp::Time& time,
                     const rclcpp::Duration& period) = 0;

    virtual void exit() = 0;

    virtual FSMStateName checkChange() { return FSMStateName::INVALID; }

    FSMStateName state_name;
    std::string state_name_string;

protected:
    CtrlInterfaces& ctrl_interfaces_;
};

#endif //FSMSTATE_H
