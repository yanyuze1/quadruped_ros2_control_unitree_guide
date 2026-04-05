/*
* 打包核心控制模块
*/

#ifndef CTRLCOMPONENT_H
#define CTRLCOMPONENT_H
#include <unitree_guide/gait/WaveGenerator.h>

#include <BalanceCtrl.h>
#include <Estimator.h>

struct CtrlComponent {
    std::shared_ptr<QuadrupedRobot> robot_model_;   // 机器人模型，负责运动学/动力学相关计算
    std::shared_ptr<Estimator> estimator_;          // 状态评估器，负责估计机器人位置、速度、姿态等状态信息
    std::shared_ptr<BalanceCtrl> balance_ctrl_;     // 平衡控制器，负责根据状态信息计算控制命令以保持机器人平衡
    std::shared_ptr<WaveGenerator> wave_generator_; // 步态生成器，负责生成机器人的步态参数，如步长、步频等

    CtrlComponent() = default;
};
#endif //CTRLCOMPONENT_H
