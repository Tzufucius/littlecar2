#include "car_pose.h"
#include <stddef.h>

const volatile WIT_Data_t *carpose_imu = NULL;
const volatile OPS_Pose_t *carpose_ops = NULL;

void CarPose_Init(void)
{
  carpose_imu = WIT_GetData();
  carpose_ops = OPS_GetPoseRef();
}
