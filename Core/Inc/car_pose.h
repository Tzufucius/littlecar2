#ifndef __CAR_POSE_H__
#define __CAR_POSE_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include "sensor_ops.h"
#include "sensor_wit.h"

    extern const volatile WIT_Data_t *carpose_imu;
    extern const volatile OPS_Pose_t *carpose_ops;

    void CarPose_Init(void);

#ifdef __cplusplus
}
#endif

#endif

// 示例用法
// #include "car_pose.h"

// if ((carpose_ops != NULL) && (carpose_ops->valid != 0U))
// {
//     float x = carpose_ops->pos_x_mm;
// }

// if ((carpose_imu != NULL) && (carpose_imu->angle_deg.valid != 0U))
// {
//     float yaw = carpose_imu->angle_deg.z;
// }