#include "unitree_guide_controller/FSM/StateCmdVel.h"

#include "unitree_guide_controller/common/mathTools.h"

StateCmdVel::StateCmdVel(
    CtrlInterfaces &ctrl_interfaces,
    CtrlComponent &ctrl_component,
    realtime_tools::RealtimeBuffer<CmdVelCommand> &cmd_vel_buffer)
    : FSMState(FSMStateName::CMDVEL, "cmd_vel", ctrl_interfaces),
      trotting_state_(ctrl_interfaces, ctrl_component),
      cmd_vel_buffer_(cmd_vel_buffer)
{
    v_x_limit_ << -0.2, 0.2;
    v_y_limit_ << -0.2, 0.2;
    w_yaw_limit_ << -0.4, 0.4;
}

void StateCmdVel::enter()
{
    ctrl_interfaces_.control_inputs_.lx = 0.0;
    ctrl_interfaces_.control_inputs_.ly = 0.0;
    ctrl_interfaces_.control_inputs_.rx = 0.0;
    ctrl_interfaces_.control_inputs_.ry = 0.0;

    trotting_state_.enter();
}

void StateCmdVel::run(const rclcpp::Time &time, const rclcpp::Duration &period)
{
    updateControlInputFromCmdVel(time);
    trotting_state_.run(time, period);
}

void StateCmdVel::exit()
{
    ctrl_interfaces_.control_inputs_.lx = 0.0;
    ctrl_interfaces_.control_inputs_.ly = 0.0;
    ctrl_interfaces_.control_inputs_.rx = 0.0;
    ctrl_interfaces_.control_inputs_.ry = 0.0;

    trotting_state_.exit();
}

FSMStateName StateCmdVel::checkChange()
{
    switch (ctrl_interfaces_.control_inputs_.command) {
        case 1:
            return FSMStateName::PASSIVE;
        case 2:
            return FSMStateName::FIXEDSTAND;
        default:
            return FSMStateName::CMDVEL;
    }
}

float StateCmdVel::normalizeToInput(const double value, const Vec2 &limits)
{
    const double clamped = saturation(value, limits);
    const double normalized =
        2.0 * (clamped - limits(0)) / (limits(1) - limits(0)) - 1.0;

    return static_cast<float>(saturation(normalized, Vec2(-1.0, 1.0)));
}

void StateCmdVel::updateControlInputFromCmdVel(const rclcpp::Time &time)
{
    double vx = 0.0;
    double vy = 0.0;
    double yaw_rate = 0.0;

    const CmdVelCommand *cmd = cmd_vel_buffer_.readFromRT();
    if (cmd != nullptr && cmd->valid) {
        const double age = (time - cmd->stamp).seconds();
        if (age >= 0.0 && age <= cmd_vel_timeout_) {
            vx = cmd->twist.linear.x;
            vy = cmd->twist.linear.y;
            yaw_rate = cmd->twist.angular.z;
        }
    }

    ctrl_interfaces_.control_inputs_.ly = normalizeToInput(vx, v_x_limit_);
    ctrl_interfaces_.control_inputs_.lx = normalizeToInput(-vy, v_y_limit_);
    ctrl_interfaces_.control_inputs_.rx = normalizeToInput(-yaw_rate, w_yaw_limit_);
    ctrl_interfaces_.control_inputs_.ry = 0.0;
}