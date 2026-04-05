/*
 * 这套控制器里的“四足机器人模型封装类”，作用是把 URDF/KDL 运动学链、当前关节状态，以及常用的运动学/力学计算统一包装起来，提供给各个状态和算法模块调用。
 * 这样做的好处是：
 * 1. 代码复用：把所有与机器人运动学/力学相关的计算都集中在一个类里，其他模块只需要调用这个类的接口，不需要重复实现相同的计算逻辑。
 * 2. 代码清晰：把机器人模型相关的代码和控制算法的代码分开，逻辑更清晰，维护起来也更方便。
 * 3. 易于扩展：如果以后需要增加新的功能，比如增加新的传感器输入或者新的控制算法，只需要在这个类里添加相应的接口，不需要修改其他模块的代码。
 * 4. 统一接口：通过这个类提供的接口，其他模块可以更方便地获取机器人当前的状态和计算结果，减少了模块之间的耦合，提高了代码的可维护性和可扩展性。
*/


#ifndef QUADRUPEDROBOT_H
#define QUADRUPEDROBOT_H
#include <string>
#include <kdl_parser/kdl_parser.hpp>
#include <unitree_guide/common/mathTypes.h>

#include <RobotLeg.h>


struct CtrlInterfaces;

class QuadrupedRobot {
public:
    explicit QuadrupedRobot(CtrlInterfaces &ctrl_interfaces, const std::string &robot_description,
                            const std::vector<std::string> &feet_names, const std::string &base_name);

    ~QuadrupedRobot() = default;

    /**
     * Calculate the joint positions based on the foot end position
     * @param pEe_list vector of foot-end position
     * @return
     */
    [[nodiscard]] std::vector<KDL::JntArray> getQ(const std::vector<KDL::Frame> &pEe_list) const;

    [[nodiscard]] Vec12 getQ(const Vec34 &vecP) const;

    Vec12 getQd(const std::vector<KDL::Frame> &pos, const Vec34 &vel);

    /**
     * Calculate the foot end position based on joint positions
     * @return vector of foot-end position
     */
    [[nodiscard]] std::vector<KDL::Frame> getFeet2BPositions() const;

    /**
     * Calculate the foot end position based on joint positions
     * @param index leg index
     * @return foot-end position
     */
    [[nodiscard]] KDL::Frame getFeet2BPositions(int index) const;

    /**
     * Calculate the Jacobian matrix based on joint positions
     * @param index leg index
     * @return Jacobian matrix
     */
    [[nodiscard]] KDL::Jacobian getJacobian(int index) const;

    /**
     * Calculate the torque based on joint positions, joint velocities and external force
     * @param force external force
     * @param index leg index
     * @return torque
     */
    [[nodiscard]] KDL::JntArray getTorque(
        const Vec3 &force, int index) const;

    /**
    * Calculate the torque based on joint positions, joint velocities and external force
    * @param force external force
    * @param index leg index
    * @return torque
    */
    [[nodiscard]] KDL::JntArray getTorque(
        const KDL::Vector &force, int index) const;

    /**
     * Calculate the foot end velocity
     * @param index leg index
     * @return velocity vector
     */
    [[nodiscard]] KDL::Vector getFeet2BVelocities(int index) const;

    /**
     * Calculate all foot end velocity
     * @return list of foot end velocity
     */
    [[nodiscard]] std::vector<KDL::Vector> getFeet2BVelocities() const;

    double mass_ = 0;
    Vec34 feet_pos_normal_stand_;
    std::vector<KDL::JntArray> current_joint_pos_;
    std::vector<KDL::JntArray> current_joint_vel_;

    void update();

private:
    CtrlInterfaces &ctrl_interfaces_;
    std::vector<std::shared_ptr<RobotLeg> > robot_legs_;

    KDL::Chain fr_chain_;
    KDL::Chain fl_chain_;
    KDL::Chain rr_chain_;
    KDL::Chain rl_chain_;
};


#endif //QUADRUPEDROBOT_H
