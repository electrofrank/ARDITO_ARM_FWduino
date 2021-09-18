/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: ArmonyController_2020a.h
 *
 * Code generated for Simulink model 'ArmonyController_2020a'.
 *
 * Model version                  : 1.8
 * Simulink Coder version         : 9.4 (R2020b) 29-Jul-2020
 * C/C++ source code generated on : Mon Jul 19 13:16:06 2021
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef RTW_HEADER_ArmonyController_2020a_h_
#define RTW_HEADER_ArmonyController_2020a_h_
#include <stddef.h>
#include <math.h>
#include <string.h>
#ifndef ArmonyController_2020a_COMMON_INCLUDES_
#define ArmonyController_2020a_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                             /* ArmonyController_2020a_COMMON_INCLUDES_ */

/* Model Code Variants */

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* Forward declaration for rtModel */
typedef struct tag_RTM_ArmonyController_2020_T RT_MODEL_ArmonyController_202_T;

/* Block signals (default storage) */
typedef struct {
  real_T Merge[6];                     /* '<S7>/Merge' */
  real_T Merge_p[6];                   /* '<S6>/Merge' */
} B_ArmonyController_2020a_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real_T L1;                           /* '<S1>/Data Store Memory2' */
  real_T L2;                           /* '<S1>/Data Store Memory3' */
  real_T L3;                           /* '<S1>/Data Store Memory4' */
} DW_ArmonyController_2020a_T;

/* Real-time Model Data Structure */
struct tag_RTM_ArmonyController_2020_T {
  const char_T * volatile errorStatus;
  B_ArmonyController_2020a_T *blockIO;
  DW_ArmonyController_2020a_T *dwork;
};

/* Model entry point functions */
extern void ArmonyController_2020a_initialize(RT_MODEL_ArmonyController_202_T *
  const ArmonyController_2020a_M, real_T ArmonyController_2020a_U_position_EE[6],
  real_T ArmonyController_2020a_U_acceleration_EE[6], real_T
  ArmonyController_2020a_U_max_speed_EE[6], real_T
  ArmonyController_2020a_U_velocity[6], real_T
  *ArmonyController_2020a_U_reference_frame, real_T
  *ArmonyController_2020a_U_control_flag_IN, real_T
  ArmonyController_2020a_U_feedback_encoder[6], real_T
  *ArmonyController_2020a_U_controller_status_IN, real_T
  ArmonyController_2020a_Y_delta_q[6], real_T ArmonyController_2020a_Y_q_d_dot[6],
  real_T ArmonyController_2020a_Y_q_dot_max[6], boolean_T
  ArmonyController_2020a_Y_direction[6], real_T
  ArmonyController_2020a_Y_abs_q_dot[6], real_T
  *ArmonyController_2020a_Y_control_flag_OUT, real_T
  *ArmonyController_2020a_Y_controller_status_OUT, real_T
  ArmonyController_2020a_Y_p_EE[6]);
extern void ArmonyController_2020a_step(RT_MODEL_ArmonyController_202_T *const
  ArmonyController_2020a_M, real_T ArmonyController_2020a_U_position_EE[6],
  real_T ArmonyController_2020a_U_velocity[6], real_T
  ArmonyController_2020a_U_reference_frame, real_T
  ArmonyController_2020a_U_control_flag_IN, real_T
  ArmonyController_2020a_U_feedback_encoder[6], real_T
  ArmonyController_2020a_U_controller_status_IN, real_T
  ArmonyController_2020a_Y_delta_q[6], real_T ArmonyController_2020a_Y_q_d_dot[6],
  real_T ArmonyController_2020a_Y_q_dot_max[6], boolean_T
  ArmonyController_2020a_Y_direction[6], real_T
  ArmonyController_2020a_Y_abs_q_dot[6], real_T
  *ArmonyController_2020a_Y_control_flag_OUT, real_T
  *ArmonyController_2020a_Y_controller_status_OUT, real_T
  ArmonyController_2020a_Y_p_EE[6]);
extern void ArmonyController_2020a_terminate(RT_MODEL_ArmonyController_202_T *
  const ArmonyController_2020a_M);

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S8>/Display' : Unused code path elimination
 * Block '<S8>/Display1' : Unused code path elimination
 * Block '<S8>/Display2' : Unused code path elimination
 * Block '<S8>/Display3' : Unused code path elimination
 * Block '<S8>/Display4' : Unused code path elimination
 * Block '<S8>/Display5' : Unused code path elimination
 * Block '<S8>/Display6' : Unused code path elimination
 * Block '<S11>/Gain' : Unused code path elimination
 * Block '<S7>/Scope' : Unused code path elimination
 */

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'ArmonyController_2020a'
 * '<S1>'   : 'ArmonyController_2020a/ArmController'
 * '<S2>'   : 'ArmonyController_2020a/ArmController/position_control'
 * '<S3>'   : 'ArmonyController_2020a/ArmController/position_direct_kinematics'
 * '<S4>'   : 'ArmonyController_2020a/ArmController/position_error'
 * '<S5>'   : 'ArmonyController_2020a/ArmController/speed_direction'
 * '<S6>'   : 'ArmonyController_2020a/ArmController/velocity_control'
 * '<S7>'   : 'ArmonyController_2020a/ArmController/position_control/KinematicController'
 * '<S8>'   : 'ArmonyController_2020a/ArmController/position_control/KinematicController/CoreController '
 * '<S9>'   : 'ArmonyController_2020a/ArmController/position_control/KinematicController/HoldPosition'
 * '<S10>'  : 'ArmonyController_2020a/ArmController/position_control/KinematicController/ReachabilityCheck'
 * '<S11>'  : 'ArmonyController_2020a/ArmController/position_control/KinematicController/CoreController /Radians to Degrees'
 * '<S12>'  : 'ArmonyController_2020a/ArmController/position_control/KinematicController/CoreController /inverseOrientation'
 * '<S13>'  : 'ArmonyController_2020a/ArmController/position_control/KinematicController/CoreController /positionCartesian'
 * '<S14>'  : 'ArmonyController_2020a/ArmController/position_control/KinematicController/CoreController /wristPositionOrientation'
 * '<S15>'  : 'ArmonyController_2020a/ArmController/position_control/KinematicController/CoreController /inverseOrientation/MATLAB Function'
 * '<S16>'  : 'ArmonyController_2020a/ArmController/position_control/KinematicController/CoreController /positionCartesian/MATLAB Function'
 * '<S17>'  : 'ArmonyController_2020a/ArmController/position_control/KinematicController/CoreController /wristPositionOrientation/MATLAB Function'
 * '<S18>'  : 'ArmonyController_2020a/ArmController/position_control/KinematicController/CoreController /wristPositionOrientation/MATLAB Function1'
 * '<S19>'  : 'ArmonyController_2020a/ArmController/position_control/KinematicController/ReachabilityCheck/MATLAB Function'
 * '<S20>'  : 'ArmonyController_2020a/ArmController/position_direct_kinematics/MATLAB Function'
 * '<S21>'  : 'ArmonyController_2020a/ArmController/position_direct_kinematics/T0_6'
 * '<S22>'  : 'ArmonyController_2020a/ArmController/position_direct_kinematics/T0_6/MATLAB Function'
 * '<S23>'  : 'ArmonyController_2020a/ArmController/velocity_control/global_frame'
 * '<S24>'  : 'ArmonyController_2020a/ArmController/velocity_control/local_frame'
 * '<S25>'  : 'ArmonyController_2020a/ArmController/velocity_control/global_frame/MATLAB Function'
 * '<S26>'  : 'ArmonyController_2020a/ArmController/velocity_control/local_frame/MATLAB Function'
 * '<S27>'  : 'ArmonyController_2020a/ArmController/velocity_control/local_frame/MATLAB Function1'
 */
#endif                                /* RTW_HEADER_ArmonyController_2020a_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
