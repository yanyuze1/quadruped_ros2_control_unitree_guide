//
// Created by Ercong Huang on 26-6-1
//

#ifndef STATECMDVEL_H
#define STATECMDVEL_H
#include "unitree_guide_controller/FSM/StateTrotting.h"
#include "unitree_guide_controller/control/CmdVelCommand.h"
#include "controller_common/FSM/FSMState.h"
#include <realtime_tools/realtime_buffer.hpp>

class StateCmdVel final : public FSMState {
public:
    explicit StateCmdVel(CtrlInterfaces &ctrl_interfaces,
                          CtrlComponent &ctrl_component,
                          realtime_tools::RealtimeBuffer<CmdVelCommand> &cmd_vel_buffer);

    void enter() override;
    void run(const rclcpp::Time &time,
             const rclcpp::Duration &period) override;
    void exit() override;
    FSMStateName checkChange() override;

private:
    static float normalizeToInput(double value, const Vec2 &limits);
    void updateControlInputFromCmdVel(const rclcpp::Time &time);

    StateTrotting trotting_state_;
    realtime_tools::RealtimeBuffer<CmdVelCommand> &cmd_vel_buffer_;    

    Vec2 v_x_limit_;
    Vec2 v_y_limit_;
    Vec2 w_yaw_limit_;
    double cmd_vel_timeout_{0.25};
};


#endif //STATECMDVEL_H
    