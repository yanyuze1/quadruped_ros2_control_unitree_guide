/**
 * 用于debug
 */
#ifndef DEBUGDATA_H
#define DEBUGDATA_H

#include <unitree_guide/common/mathTypes.h>

struct BalanceCtrlDebugData {
    Vec3 dd_pcd;
    Vec3 d_wbd;
    Vec3 bd_force;
    Vec3 bd_torque;
    Vec34 force;

    BalanceCtrlDebugData() {
        dd_pcd.setZero();
        d_wbd.setZero();
        bd_force.setZero();
        bd_torque.setZero();
        force.setZero();
    }
};

struct EstimatorDebugData {
    Vec3 position;
    Vec3 velocity;
    Vec3 gyro;
    Vec3 gyro_global;
    Vec3 acceleration;
    double yaw;
    double dyaw;

    EstimatorDebugData() : yaw(0.0), dyaw(0.0) {
        position.setZero();
        velocity.setZero();
        gyro.setZero();
        gyro_global.setZero();
        acceleration.setZero();
    }
};

#endif // DEBUGDATA_H
