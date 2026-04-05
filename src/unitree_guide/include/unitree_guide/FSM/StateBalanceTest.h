/*
* 平衡测试状态定义，StateBalanceTest 类继承自 FSMState
    * 进入状态时锁定四脚全支撑
    * 根据摇杆输入改变机身目标位置和目标偏航
    * 用 PD 方式生成期望机身线加速度和角加速度
    * 交给 BalanceCtrl 算四条腿该出多大力
    * 再通过机器人模型把足端力变成关节力矩输出
*/

#ifndef STATEBALANCETEST_H
#define STATEBALANCETEST_H

#include <unitree_guide/common/mathTypes.h>

#include <FSM/FSMState.h>


class WaveGenerator;
class BalanceCtrl;
class QuadrupedRobot;
class Estimator;
struct CtrlComponent;

class StateBalanceTest final : public FSMState {
public:
    explicit StateBalanceTest(CtrlInterfaces &ctrl_interfaces,
                              CtrlComponent &ctrl_component);

    void enter() override;

    void run(const rclcpp::Time &time,
             const rclcpp::Duration &period) override;

    void exit() override;

    FSMStateName checkChange() override;

private:
    void calcTorque();

    std::shared_ptr<Estimator> &estimator_;
    std::shared_ptr<QuadrupedRobot> &robot_model_;
    std::shared_ptr<BalanceCtrl> &balance_ctrl_;
    std::shared_ptr<WaveGenerator> &wave_generator_;

    Vec3 pcd_, pcd_init_;
    RotMat Rd_;
    RotMat init_rotation_;

    double kp_w_;
    Mat3 Kp_p_, Kd_p_, Kd_w_;
    Vec3 dd_pcd_, d_wbd_;

    float _xMax, _xMin;
    float _yMax, _yMin;
    float _zMax, _zMin;
    float _yawMax, _yawMin;
};


#endif //STATEBALANCETEST_H
