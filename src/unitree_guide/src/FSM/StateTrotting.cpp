/*
 * 机器人行走状态，使用 VMC（虚拟模型控制）方法来计算四条腿的支撑力，然后通过逆运动学求出关节角度，最后 PD 控制器把关节拉到目标角度上。这个状态的作用是让机器人以一种比较自然的方式行走，同时允许用户通过遥控器来控制机器人的移动速度和转向速度。
 */

#include "unitree_guide/FSM/StateTrotting.h"

#include <cmath>

#include <unitree_guide/common/mathTools.h>
#include <unitree_guide/control/CtrlComponent.h>
#include <unitree_guide/control/Estimator.h>
#include <unitree_guide/gait/WaveGenerator.h>

StateTrotting::StateTrotting(CtrlInterfaces &ctrl_interfaces,
                             CtrlComponent &ctrl_component) : FSMState(FSMStateName::TROTTING, "trotting",
                                                                       ctrl_interfaces),
                                                              estimator_(ctrl_component.estimator_),
                                                              robot_model_(ctrl_component.robot_model_),
                                                              balance_ctrl_(ctrl_component.balance_ctrl_),
                                                              wave_generator_(ctrl_component.wave_generator_),
                                                              gait_generator_(ctrl_component) {
    gait_height_ = 0.08; // 抬腿高度(m)
    // vmc控制参数
    Kpp = Vec3(70, 70, 70).asDiagonal();
    Kdp = Vec3(10, 10, 10).asDiagonal();
    kp_w_ = 780;
    Kd_w_ = Vec3(70, 70, 70).asDiagonal();
    Kp_swing_ = Vec3(400, 400, 400).asDiagonal();
    Kd_swing_ = Vec3(10, 10, 10).asDiagonal();

    v_x_limit_ << -0.2, 0.2;
    v_y_limit_ << -0.2, 0.2;
    w_yaw_limit_ << -0.4, 0.4;
    dt_ = 1.0 / ctrl_interfaces_.frequency_;
}

void StateTrotting::enter() {
    // 状态机入口
    pcd_ = estimator_->getPosition();
    pcd_(2) = -estimator_->getFeetPos2Body()(2, 0);
    v_cmd_body_.setZero();  // 速度指令清零
    yaw_cmd_ = estimator_->getYaw();
    yaw_cmd_ = std::atan2(std::sin(yaw_cmd_), std::cos(yaw_cmd_));
    Rd = rotz(yaw_cmd_);
    w_cmd_global_.setZero();

    yaw_rate_mode_ = false;
    yaw_rate_mode_last_ = false;

    debug_data_ = StateTrottingDebugData();

    ctrl_interfaces_.control_inputs_.command = 0;
    gait_generator_.restart();
}

void StateTrotting::run(const rclcpp::Time &/*time*/, const rclcpp::Duration &/*period*/) {
    pos_body_ = estimator_->getPosition();
    vel_body_ = estimator_->getVelocity();

    B2G_RotMat = estimator_->getRotation();
    G2B_RotMat = B2G_RotMat.transpose();

    getUserCmd();
    calcCmd();

    gait_generator_.setGait(vel_target_.segment(0, 2), w_cmd_global_(2), gait_height_);
    gait_generator_.generate(pos_feet_global_goal_, vel_feet_global_goal_);

    calcTau();
    calcQQd();

    if (checkStepOrNot()){
        wave_generator_->status_ = WaveStatus::WAVE_ALL;
    } else {
        wave_generator_->status_ = WaveStatus::STANCE_ALL;
    }

    debug_data_.phase = wave_generator_->phase_;
    for (int i = 0; i < 4; ++i) {
        debug_data_.contact(i) = static_cast<double>(wave_generator_->contact_(i));
    }
    debug_data_.wave_status = static_cast<int>(wave_generator_->status_);

    calcGain();

}

void StateTrotting::exit() {
    wave_generator_->status_ = WaveStatus::SWING_ALL;
}

FSMStateName StateTrotting::checkChange() {
    switch (ctrl_interfaces_.control_inputs_.command) {
        case 1:
            return FSMStateName::PASSIVE;
        case 2:
            return FSMStateName::FIXEDSTAND;
        default:
            return FSMStateName::TROTTING;
    }
}

void StateTrotting::getUserCmd() {
    /* Movement */
    v_cmd_body_(0) = invNormalize(ctrl_interfaces_.control_inputs_.ly, v_x_limit_(0), v_x_limit_(1));
    v_cmd_body_(1) = -invNormalize(ctrl_interfaces_.control_inputs_.lx, v_y_limit_(0), v_y_limit_(1));
    v_cmd_body_(2) = 0;

    /* Turning */
    d_yaw_cmd_ = -invNormalize(ctrl_interfaces_.control_inputs_.rx, w_yaw_limit_(0), w_yaw_limit_(1));
    d_yaw_cmd_ = 0.90 * d_yaw_cmd_past_ + (1 - 0.90) * d_yaw_cmd_;
    d_yaw_cmd_past_ = d_yaw_cmd_;

    debug_data_.v_cmd_body = v_cmd_body_;
    debug_data_.d_yaw_cmd = d_yaw_cmd_;
}

void StateTrotting::calcCmd() {
    /* Movement */
    vel_target_ = B2G_RotMat * v_cmd_body_;

    vel_target_(0) =
            saturation(vel_target_(0), Vec2(vel_body_(0) - 0.2, vel_body_(0) + 0.2));
    vel_target_(1) =
            saturation(vel_target_(1), Vec2(vel_body_(1) - 0.2, vel_body_(1) + 0.2));

    pcd_(0) = saturation(pcd_(0) + vel_target_(0) * dt_,
                         Vec2(pos_body_(0) - 0.05, pos_body_(0) + 0.05));
    pcd_(1) = saturation(pcd_(1) + vel_target_(1) * dt_,
                         Vec2(pos_body_(1) - 0.05, pos_body_(1) + 0.05));

    vel_target_(2) = 0;

    const double current_yaw =
            std::atan2(std::sin(estimator_->getYaw()), std::cos(estimator_->getYaw()));

    yaw_rate_mode_last_ = yaw_rate_mode_;
    yaw_rate_mode_ = std::fabs(d_yaw_cmd_) > yaw_hold_deadband_;

    if (yaw_rate_mode_) {
        yaw_cmd_ = current_yaw;
    } else if (yaw_rate_mode_last_) {
        yaw_cmd_ = current_yaw;
    }

    /* Turning */
    Rd = rotz(yaw_cmd_);
    w_cmd_global_.setZero();
    w_cmd_global_(2) = d_yaw_cmd_;

    debug_data_.vel_target = vel_target_;
    debug_data_.pcd = pcd_;
    debug_data_.yaw_cmd = yaw_cmd_;
    debug_data_.yaw_est = estimator_->getYaw();
    debug_data_.yaw_error = std::atan2(std::sin(yaw_cmd_ - debug_data_.yaw_est),
                                       std::cos(yaw_cmd_ - debug_data_.yaw_est));
    debug_data_.d_yaw_est = estimator_->getDYaw();
}

void StateTrotting::calcTau() {
    pos_error_ = pcd_ - pos_body_;
    vel_error_ = vel_target_ - vel_body_;

    Vec3 dd_pcd = Kpp * pos_error_ + Kdp * vel_error_;
    Vec3 rot_error = rotMatToExp(Rd * G2B_RotMat);
    if (yaw_rate_mode_) {
        // 转向时关闭 yaw 位置误差，避免累计朝向导致“反向纠偏”
        rot_error(2) = 0.0;
    }
    // Vec3 d_wbd = kp_w_ * rotMatToExp(Rd * G2B_RotMat) +
    Vec3 d_wbd = kp_w_ * rot_error +
                 Kd_w_ * (w_cmd_global_ - estimator_->getGyroGlobal());

    dd_pcd(0) = saturation(dd_pcd(0), Vec2(-3, 3));
    dd_pcd(1) = saturation(dd_pcd(1), Vec2(-3, 3));
    dd_pcd(2) = saturation(dd_pcd(2), Vec2(-5, 5));

    d_wbd(0) = saturation(d_wbd(0), Vec2(-40, 40));
    d_wbd(1) = saturation(d_wbd(1), Vec2(-30, 30));
    d_wbd(2) = saturation(d_wbd(2), Vec2(-10, 10));

    const Vec34 pos_feet_body_global = estimator_->getFeetPos2Body();
    Vec34 force_feet_global =
            -balance_ctrl_->calF(dd_pcd, d_wbd, B2G_RotMat, pos_feet_body_global, wave_generator_->contact_);


    Vec34 pos_feet_global = estimator_->getFeetPos();
    Vec34 vel_feet_global = estimator_->getFeetVel();

    for (int i(0); i < 4; ++i) {
        if (wave_generator_->contact_(i) == 0) {
            force_feet_global.col(i) =
                    Kp_swing_ * (pos_feet_global_goal_.col(i) - pos_feet_global.col(i)) +
                    Kd_swing_ * (vel_feet_global_goal_.col(i) - vel_feet_global.col(i));
        }
    }

    Vec34 force_feet_body_ = G2B_RotMat * force_feet_global;

    debug_data_.pos_body = pos_body_;
    debug_data_.vel_body = vel_body_;
    debug_data_.pos_error = pos_error_;
    debug_data_.vel_error = vel_error_;
    debug_data_.dd_pcd = dd_pcd;
    debug_data_.rot_error = rot_error;
    debug_data_.d_wbd = d_wbd;
    debug_data_.gyro_global = estimator_->getGyroGlobal();
    debug_data_.pos_feet_global_goal = pos_feet_global_goal_;
    debug_data_.vel_feet_global_goal = vel_feet_global_goal_;
    debug_data_.pos_feet_global = pos_feet_global;
    debug_data_.vel_feet_global = vel_feet_global;
    debug_data_.force_feet_global = force_feet_global;
    debug_data_.force_feet_body = force_feet_body_;
    debug_data_.tau_cmd.setZero();

    std::vector<KDL::JntArray> current_joints = robot_model_->current_joint_pos_;
    for (int i = 0; i < 4; i++) {
        KDL::JntArray torque = robot_model_->getTorque(force_feet_body_.col(i), i);
        for (int j = 0; j < 3; j++) {
            const int idx = i * 3 + j;
            ctrl_interfaces_.joint_torque_command_interface_[i * 3 + j].get().set_value(torque(j));
            debug_data_.tau_cmd(idx) = torque(j);
        }
    }
}

void StateTrotting::calcQQd() {
    const std::vector<KDL::Frame> pos_feet_body = robot_model_->getFeet2BPositions();

    Vec34 pos_feet_target, vel_feet_target;
    for (int i(0); i < 4; ++i) {
        pos_feet_target.col(i) = G2B_RotMat * (pos_feet_global_goal_.col(i) - pos_body_);
        vel_feet_target.col(i) = G2B_RotMat * (vel_feet_global_goal_.col(i) - vel_body_);
    }

    Vec12 q_goal = robot_model_->getQ(pos_feet_target);
    Vec12 qd_goal = robot_model_->getQd(pos_feet_body, vel_feet_target);

    debug_data_.q_goal = q_goal;
    debug_data_.qd_goal = qd_goal;

    for (int i = 0; i < 12; i++) {
        ctrl_interfaces_.joint_position_command_interface_[i].get().set_value(q_goal(i));
        ctrl_interfaces_.joint_velocity_command_interface_[i].get().set_value(qd_goal(i));

        debug_data_.q_state(i) =
            ctrl_interfaces_.joint_position_state_interface_[i].get().get_value();
        debug_data_.qd_state(i) =
            ctrl_interfaces_.joint_velocity_state_interface_[i].get().get_value();
        debug_data_.tau_state(i) =
            ctrl_interfaces_.joint_effort_state_interface_[i].get().get_value();

        debug_data_.q_error(i) = debug_data_.q_goal(i) - debug_data_.q_state(i);
        debug_data_.qd_error(i) = debug_data_.qd_goal(i) - debug_data_.qd_state(i);
        debug_data_.tau_error(i) = debug_data_.tau_cmd(i) - debug_data_.tau_state(i);
    }
}

void StateTrotting::calcGain() const {
    for (int i(0); i < 4; ++i) {
        if (wave_generator_->contact_(i) == 0) {
            // swing gain
            for (int j = 0; j < 3; j++) {
                ctrl_interfaces_.joint_kp_command_interface_[i * 3 + j].get().set_value(3);
                ctrl_interfaces_.joint_kd_command_interface_[i * 3 + j].get().set_value(2);
            }
        } else {
            // stable gain
            for (int j = 0; j < 3; j++) {
                ctrl_interfaces_.joint_kp_command_interface_[i * 3 + j].get().set_value(0.8);
                ctrl_interfaces_.joint_kd_command_interface_[i * 3 + j].get().set_value(0.8);
            }
        }
    }
}

bool StateTrotting::checkStepOrNot() {
    if (fabs(v_cmd_body_(0)) > 0.03 || fabs(v_cmd_body_(1)) > 0.03 ||
        fabs(pos_error_(0)) > 0.08 || fabs(pos_error_(1)) > 0.08 ||
        fabs(vel_error_(0)) > 0.05 || fabs(vel_error_(1)) > 0.05 ||
        fabs(d_yaw_cmd_) > 0.05) {
        return true;
    }
    return false;
}
