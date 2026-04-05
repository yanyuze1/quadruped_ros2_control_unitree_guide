/*
* 固定站立状态定义，StateFixedStand 类继承自 BaseFixedStand 类，表示四足机器人处于固定站立状态时的行为逻辑，包括进入状态、持续执行和离开状态的函数，以及检查是否需要切换状态的函数。
* 当机器人处于固定站立状态时，通常会将关节位置设置为预设的目标位置，并通过 PD 控制器进行控制，以保持机器人在该位置上稳定站立。
*/

#ifndef STATEFIXEDSTAND_H
#define STATEFIXEDSTAND_H

#include <controller_common/FSM/BaseFixedStand.h>

class StateFixedStand final : public BaseFixedStand {
public:
    explicit StateFixedStand(CtrlInterfaces &ctrl_interfaces,
                             const std::vector<double> &target_pos,
                             double kp,
                             double kd);

    FSMStateName checkChange() override;
};


#endif //STATEFIXEDSTAND_H
