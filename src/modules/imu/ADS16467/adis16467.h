//
// Created by SuperChen on 2024/11/25.
//

#ifndef RTTHREAD_ADIS16467_H
#define RTTHREAD_ADIS16467_H

#include <rtthread.h>
#include <rtdevice.h>
#include "imu.h"

#ifdef __cplusplus
extern "C" {
#endif

extern struct imu_ops imu_ops;
extern float ADIS16467_g_norm;   // 通过校准得出的重力加速度,数据融合时会用到

/* Re-implement this function to define customized rotation */
__attribute__((weak)) void ADIS16467_acc_rotate_to_frd(float* data);
/* Re-implement this function to define customized rotation */
__attribute__((weak)) void ADIS16467_gyro_rotate_to_frd(float* data);

#ifdef __cplusplus
}
#endif

#endif //RTTHREAD_ADIS16467_H
