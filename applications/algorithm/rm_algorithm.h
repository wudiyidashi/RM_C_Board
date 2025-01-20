 /**
 * @file rm_algorithm.h
 * @author ChuShicheng
 * @author modified by neozng
 * @brief  RM电控算法库,仅被应用层调用
 * @date 2023-09-04
 */

 /*
 * Change Logs:
 * Date            Author          Notes
 * 2023-09-04      ChuShicheng     first version
 */
#ifndef _RM_ALGORITHM_H
#define _RM_ALGORITHM_H

#include <rtthread.h>
#include <board.h>
#include "user_lib.h"
#include "arm_math.h"
#ifdef BSP_USING_PID
#include "pid.h"
#endif /* BSP_USING_PID */
#ifdef BSP_USING_QUATERNIONESF
#include "QuaternionEKF.h"
#endif /* BSP_USING_QUATERNIONESF */
#ifdef BSP_USING_KALMAN_FILTER
#include "kalman_filter.h"
#endif /* BSP_USING_KALMAN_FILTER */
#ifdef BSP_USING_RAMP
#include "ramp.h"
#endif /* BSP_USING_RAMP */

#endif /* _RM_ALGORITHM_H */
