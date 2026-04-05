/*
* 基于二次规划QP的平衡控制器，根据机器人期望的机身线加速度、角加速度，以及当前四脚接触情况，求解四条腿应该输出的足端力
*/

#ifndef BALANCECTRL_H
#define BALANCECTRL_H

#include <memory>

#include <unitree_guide/common/mathTypes.h>
#include <unitree_guide/common/DebugData.h>
class QuadrupedRobot;

class BalanceCtrl {
public:
    const BalanceCtrlDebugData &getDebugData() const { return debug_data_; }

    // 获取机器人的物理参数
    explicit BalanceCtrl(const std::shared_ptr<QuadrupedRobot>& robot);

    ~BalanceCtrl() = default;

    /**
     * Calculate the desired feet end force     计算所需的足端力
     * @param ddPcd desired body acceleration   机器人期望的线加速度
     * @param dWbd desired body angular acceleration    机器人期望的角加速度
     * @param rot_matrix current body rotation matrix   当前机器人旋转矩阵
     * @param feet_pos_2_body feet positions to body under world frame  世界坐标系下，机器人足底相对机器人质心的位置向量(3x4矩阵)
     * @param contact feet contact  四个足底接触状态(四个脚四维向量，0为离地，1为触地)
     * @return  返回四个脚足端力向量(3x4矩阵对应每个脚的三个自由度)
     */
    Vec34 calF(const Vec3 &ddPcd, const Vec3 &dWbd, const RotMat &rot_matrix,
               const Vec34 &feet_pos_2_body, const VecInt4 &contact);

private:
    BalanceCtrlDebugData debug_data_;

    // 雅可比矩阵/映射矩阵A的计算函数(将足端力映射为机器人的合力和合力矩)
    void calMatrixA(const Vec34 &feet_pos_2_body, const RotMat &rotM);
    // 目标向量bd的计算函数(机器人为达成目标加速度的期望合力和合力矩，包括重力补偿)
    void calVectorBd(const Vec3 &ddPcd, const Vec3 &dWbd, const RotMat &rotM);
    // QP求解器约束条件的计算函数(等式约束:悬空脚力为0；不等式约束：触地脚满足摩擦锥和支持力为正)   
    void calConstraints(const VecInt4 &contact);
    // 求解QP
    void solveQP();
    
    Mat12 G_, W_, U_;   // G：目标函数的二次项矩阵；W：足端力大小权重矩阵；U：足端力改变量的权重矩阵。
    Mat6 S_;            // 动力学方程权重矩阵
    Mat3 Ib_;           // 机器人的惯性张量
    Vec6 bd_;           // 六自由度旋量目标(合力 + 合力矩)
    Vec3 g_, pcb_;      // g：重力向量；pcb：当前质心位置向量
    Vec12 F_, F_prev_, g0T_;    // F：当前足端力；F_prev：上一时刻的足端力；g0T：g0的转置
    double mass_, alpha_, beta_, friction_ratio_;   // mass：机器人的总质量；alpha：权重矩阵W的权重系数；beta：权重矩阵U的权重系数；friction_ratio：足端和地面摩擦系数
    Eigen::MatrixXd CE_, CI_;   // CE：等式约束矩阵；CI：不等式约束矩阵
    Eigen::VectorXd ce0_, ci0_; // ce0：等式约束的常数项；ci0：不等式约束的常数项
    Eigen::Matrix<double, 6, 12> A_;    // 雅可比矩阵A(6x12矩阵，12个关节映射为机器人6个空间力/力矩)
    Eigen::Matrix<double, 5, 3> friction_mat_;  // 摩擦四棱锥不等式约束矩阵
};


#endif //BALANCECTRL_H
