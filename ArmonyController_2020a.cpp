/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: ArmonyController_2020a.c
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

#include "ArmonyController_2020a.h"
#include <stdio.h>
#define NumBitsPerChar                 8U

extern real_T rt_atan2d_snf(real_T u0, real_T u1);

/* Forward declaration for local functions */
static real_T ArmonyController_2020a_det(const real_T x[9]);
static real_T rtGetNaN(void);
static real32_T rtGetNaNF(void);

/*===========*
 * Constants *
 *===========*/
#define RT_PI                          3.14159265358979323846
#define RT_PIF                         3.1415927F
#define RT_LN_10                       2.30258509299404568402
#define RT_LN_10F                      2.3025851F
#define RT_LOG10E                      0.43429448190325182765
#define RT_LOG10EF                     0.43429449F
#define RT_E                           2.7182818284590452354
#define RT_EF                          2.7182817F

/*
 * UNUSED_PARAMETER(x)
 *   Used to specify that a function parameter (argument) is required but not
 *   accessed by the function body.
 */
#ifndef UNUSED_PARAMETER
#if defined(__LCC__)
#define UNUSED_PARAMETER(x)                                      /* do nothing */
#else

/*
 * This is the semi-ANSI standard way of indicating that an
 * unused function parameter is required.
 */
#define UNUSED_PARAMETER(x)            (void) (x)
#endif
#endif

extern real_T rtInf;
extern real_T rtMinusInf;
extern real_T rtNaN;
extern real32_T rtInfF;
extern real32_T rtMinusInfF;
extern real32_T rtNaNF;
static void rt_InitInfAndNaN(size_t realSize);
static boolean_T rtIsInf(real_T value);
static boolean_T rtIsInfF(real32_T value);
static boolean_T rtIsNaN(real_T value);
static boolean_T rtIsNaNF(real32_T value);
typedef struct {
  struct {
    uint32_T wordH;
    uint32_T wordL;
  } words;
} BigEndianIEEEDouble;

typedef struct {
  struct {
    uint32_T wordL;
    uint32_T wordH;
  } words;
} LittleEndianIEEEDouble;

typedef struct {
  union {
    real32_T wordLreal;
    uint32_T wordLuint;
  } wordL;
} IEEESingle;

real_T rtInf;
real_T rtMinusInf;
real_T rtNaN;
real32_T rtInfF;
real32_T rtMinusInfF;
real32_T rtNaNF;
static real_T rtGetInf(void);
static real32_T rtGetInfF(void);
static real_T rtGetMinusInf(void);
static real32_T rtGetMinusInfF(void);

/*
 * Initialize rtNaN needed by the generated code.
 * NaN is initialized as non-signaling. Assumes IEEE.
 */
static real_T rtGetNaN(void)
{
  size_t bitsPerReal = sizeof(real_T) * (NumBitsPerChar);
  real_T nan = 0.0;
  if (bitsPerReal == 32U) {
    nan = rtGetNaNF();
  } else {
    union {
      LittleEndianIEEEDouble bitVal;
      real_T fltVal;
    } tmpVal;

    tmpVal.bitVal.words.wordH = 0xFFF80000U;
    tmpVal.bitVal.words.wordL = 0x00000000U;
    nan = tmpVal.fltVal;
  }

  return nan;
}

/*
 * Initialize rtNaNF needed by the generated code.
 * NaN is initialized as non-signaling. Assumes IEEE.
 */
static real32_T rtGetNaNF(void)
{
  IEEESingle nanF = { { 0 } };

  nanF.wordL.wordLuint = 0xFFC00000U;
  return nanF.wordL.wordLreal;
}

/*
 * Initialize the rtInf, rtMinusInf, and rtNaN needed by the
 * generated code. NaN is initialized as non-signaling. Assumes IEEE.
 */
static void rt_InitInfAndNaN(size_t realSize)
{
  (void) (realSize);
  rtNaN = rtGetNaN();
  rtNaNF = rtGetNaNF();
  rtInf = rtGetInf();
  rtInfF = rtGetInfF();
  rtMinusInf = rtGetMinusInf();
  rtMinusInfF = rtGetMinusInfF();
}

/* Test if value is infinite */
static boolean_T rtIsInf(real_T value)
{
  return (boolean_T)((value==rtInf || value==rtMinusInf) ? 1U : 0U);
}

/* Test if single-precision value is infinite */
static boolean_T rtIsInfF(real32_T value)
{
  return (boolean_T)(((value)==rtInfF || (value)==rtMinusInfF) ? 1U : 0U);
}

/* Test if value is not a number */
static boolean_T rtIsNaN(real_T value)
{
  boolean_T result = (boolean_T) 0;
  size_t bitsPerReal = sizeof(real_T) * (NumBitsPerChar);
  if (bitsPerReal == 32U) {
    result = rtIsNaNF((real32_T)value);
  } else {
    union {
      LittleEndianIEEEDouble bitVal;
      real_T fltVal;
    } tmpVal;

    tmpVal.fltVal = value;
    result = (boolean_T)((tmpVal.bitVal.words.wordH & 0x7FF00000) == 0x7FF00000 &&
                         ( (tmpVal.bitVal.words.wordH & 0x000FFFFF) != 0 ||
                          (tmpVal.bitVal.words.wordL != 0) ));
  }

  return result;
}

/* Test if single-precision value is not a number */
static boolean_T rtIsNaNF(real32_T value)
{
  IEEESingle tmp;
  tmp.wordL.wordLreal = value;
  return (boolean_T)( (tmp.wordL.wordLuint & 0x7F800000) == 0x7F800000 &&
                     (tmp.wordL.wordLuint & 0x007FFFFF) != 0 );
}

/*
 * Initialize rtInf needed by the generated code.
 * Inf is initialized as non-signaling. Assumes IEEE.
 */
static real_T rtGetInf(void)
{
  size_t bitsPerReal = sizeof(real_T) * (NumBitsPerChar);
  real_T inf = 0.0;
  if (bitsPerReal == 32U) {
    inf = rtGetInfF();
  } else {
    union {
      LittleEndianIEEEDouble bitVal;
      real_T fltVal;
    } tmpVal;

    tmpVal.bitVal.words.wordH = 0x7FF00000U;
    tmpVal.bitVal.words.wordL = 0x00000000U;
    inf = tmpVal.fltVal;
  }

  return inf;
}

/*
 * Initialize rtInfF needed by the generated code.
 * Inf is initialized as non-signaling. Assumes IEEE.
 */
static real32_T rtGetInfF(void)
{
  IEEESingle infF;
  infF.wordL.wordLuint = 0x7F800000U;
  return infF.wordL.wordLreal;
}

/*
 * Initialize rtMinusInf needed by the generated code.
 * Inf is initialized as non-signaling. Assumes IEEE.
 */
static real_T rtGetMinusInf(void)
{
  size_t bitsPerReal = sizeof(real_T) * (NumBitsPerChar);
  real_T minf = 0.0;
  if (bitsPerReal == 32U) {
    minf = rtGetMinusInfF();
  } else {
    union {
      LittleEndianIEEEDouble bitVal;
      real_T fltVal;
    } tmpVal;

    tmpVal.bitVal.words.wordH = 0xFFF00000U;
    tmpVal.bitVal.words.wordL = 0x00000000U;
    minf = tmpVal.fltVal;
  }

  return minf;
}

/*
 * Initialize rtMinusInfF needed by the generated code.
 * Inf is initialized as non-signaling. Assumes IEEE.
 */
static real32_T rtGetMinusInfF(void)
{
  IEEESingle minfF;
  minfF.wordL.wordLuint = 0xFF800000U;
  return minfF.wordL.wordLreal;
}

/* Function for MATLAB Function: '<S23>/MATLAB Function' */
static real_T ArmonyController_2020a_det(const real_T x[9])
{
  real_T A[9];
  real_T b_y;
  real_T smax;
  real_T y;
  int32_T c;
  int32_T c_ix;
  int32_T c_k;
  int32_T d;
  int32_T ijA;
  int32_T ix;
  int32_T iy;
  int32_T j;
  int8_T ipiv[3];
  boolean_T isodd;
  memcpy(&A[0], &x[0], 9U * sizeof(real_T));
  ipiv[0] = 1;
  ipiv[1] = 2;
  for (j = 0; j < 2; j++) {
    c = j << 2;
    iy = 0;
    ix = c;
    smax = fabs(A[c]);
    for (c_k = 2; c_k <= 3 - j; c_k++) {
      ix++;
      b_y = fabs(A[ix]);
      if (b_y > smax) {
        iy = c_k - 1;
        smax = b_y;
      }
    }

    if (A[c + iy] != 0.0) {
      if (iy != 0) {
        iy += j;
        ipiv[j] = (int8_T)(iy + 1);
        smax = A[j];
        A[j] = A[iy];
        A[iy] = smax;
        smax = A[j + 3];
        A[j + 3] = A[iy + 3];
        A[iy + 3] = smax;
        smax = A[j + 6];
        A[j + 6] = A[iy + 6];
        A[iy + 6] = smax;
      }

      iy = (c - j) + 3;
      for (ix = c + 1; ix < iy; ix++) {
        A[ix] /= A[c];
      }
    }

    iy = c;
    ix = c + 3;
    for (c_k = 0; c_k <= 1 - j; c_k++) {
      if (A[ix] != 0.0) {
        smax = -A[ix];
        c_ix = c + 1;
        d = (iy - j) + 6;
        for (ijA = iy + 4; ijA < d; ijA++) {
          A[ijA] += A[c_ix] * smax;
          c_ix++;
        }
      }

      ix += 3;
      iy += 3;
    }
  }

  isodd = (ipiv[0] > 1);
  y = A[0] * A[4] * A[8];
  if (ipiv[1] > 2) {
    isodd = !isodd;
  }

  if (isodd) {
    y = -y;
  }

  return y;
}

real_T rt_atan2d_snf(real_T u0, real_T u1)
{
  real_T y;
  int32_T u0_0;
  int32_T u1_0;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    if (u0 > 0.0) {
      u0_0 = 1;
    } else {
      u0_0 = -1;
    }

    if (u1 > 0.0) {
      u1_0 = 1;
    } else {
      u1_0 = -1;
    }

    y = atan2(u0_0, u1_0);
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = atan2(u0, u1);
  }

  return y;
}

/* Model step function */
void ArmonyController_2020a_step(RT_MODEL_ArmonyController_202_T *const
  ArmonyController_2020a_M, real_T ArmonyController_2020a_U_pose_EE[6],
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
  ArmonyController_2020a_Y_p_EE[6])
{


  B_ArmonyController_2020a_T *ArmonyController_2020a_B =
    ArmonyController_2020a_M->blockIO;
  DW_ArmonyController_2020a_T *ArmonyController_2020a_DW =
    ArmonyController_2020a_M->dwork;

  real_T J[36];
  real_T a[36];
  real_T rtb_MatrixConcatenate[24];
  real_T T0_1[16];
  real_T T0_2[16];
  real_T T0_3[16];
  real_T T0_4[16];
  real_T T0_5[16];
  real_T T0_5_0[16];
  real_T c3_0[16];
  real_T smax_0[16];
  real_T rtb_angles[12];
  real_T R_in[9];
  real_T rtb_R0_3[9];
  real_T smax_1[9];
  real_T rtb_VectorConcatenate_l[6];
  real_T q4[4];
  real_T q5[4];
  real_T q6[4];
  real_T rtb_R0_3_0[3];
  real_T rtb_R0_3_1[3];
  real_T c3;
  real_T c_b_idx_1;
  real_T c_b_idx_2;
  real_T ct_tmp;
  real_T d_b_idx_0;
  real_T e_b_idx_0;
  real_T p_6;
  real_T p_6_idx_0;
  real_T p_6_idx_1;
  real_T q1_idx_0;
  real_T q1_idx_1;
  real_T rtb_angles_tmp;
  real_T rtb_angles_tmp_0;
  real_T rtb_angles_tmp_1;
  real_T smax;
  real_T st_idx_0;
  real_T st_idx_0_tmp;
  real_T st_idx_1;
  real_T st_idx_2;
  int32_T c_ix;
  int32_T c_j;
  int32_T e;
  int32_T exitg1;
  int32_T i;
  int32_T ijA;
  int32_T ix;
  int32_T iy;
  int32_T pipk;
  int8_T ipiv[6];
  int8_T p[6];
  int8_T ipiv_0;


  /* SwitchCase: '<S1>/control_selection' incorporates:
   *  Inport: '<Root>/control_flag_IN'
   */
  ct_tmp = trunc(ArmonyController_2020a_U_control_flag_IN);
  if (rtIsNaN(ct_tmp) || rtIsInf(ct_tmp)) {
    ct_tmp = 0.0;
  } else {
    ct_tmp = fmod(ct_tmp, 4.294967296E+9);
  }
         

  switch ((int32_T)ArmonyController_2020a_U_control_flag_IN)
  {

   case 0:
    /* Outputs for IfAction SubSystem: '<S1>/velocity_control' incorporates:
     *  ActionPort: '<S6>/Action Port'
     */
    /* SwitchCase: '<S6>/Switch Case' incorporates:
     *  Inport: '<Root>/reference_frame'
     *  MATLAB Function: '<S23>/MATLAB Function'
     *  MATLAB Function: '<S24>/MATLAB Function'
     *  MATLAB Function: '<S24>/MATLAB Function1'
     */
    ct_tmp = trunc(ArmonyController_2020a_U_reference_frame);
    if (rtIsNaN(ct_tmp) || rtIsInf(ct_tmp)) {
      ct_tmp = 0.0;
    } else {
      ct_tmp = fmod(ct_tmp, 4.294967296E+9);
    }


    switch (ct_tmp < 0.0 ? -(int32_T)(uint32_T)-ct_tmp : (int32_T)(uint32_T)
            ct_tmp) {
     case 0:
      /* Outputs for IfAction SubSystem: '<S6>/global_frame' incorporates:
       *  ActionPort: '<S23>/Action Port'
       */
      /* MATLAB Function: '<S23>/MATLAB Function' incorporates:
       *  DataStoreRead: '<S23>/Data Store Read'
       *  Inport: '<Root>/feedback_encoder'
       */
      smax = cos(ArmonyController_2020a_U_feedback_encoder[0]);
      c3 = sin(ArmonyController_2020a_U_feedback_encoder[0]);
      T0_1[0] = smax;
      T0_1[4] = 0.0;
      T0_1[8] = c3;
      T0_1[12] = 0.0;
      T0_1[1] = c3;
      T0_1[5] = 0.0;
      T0_1[9] = -smax;
      T0_1[13] = 0.0;
      ct_tmp = cos(ArmonyController_2020a_U_feedback_encoder[1]);
      smax_0[0] = ct_tmp;
      st_idx_0_tmp = sin(ArmonyController_2020a_U_feedback_encoder[1]);
      smax_0[4] = -st_idx_0_tmp;
      smax_0[8] = 0.0;
      smax_0[12] = ArmonyController_2020a_DW->L1 * ct_tmp;
      smax_0[1] = st_idx_0_tmp;
      smax_0[5] = ct_tmp;
      smax_0[9] = 0.0;
      smax_0[13] = ArmonyController_2020a_DW->L1 * sin
        (ArmonyController_2020a_U_feedback_encoder[1]);
      T0_1[2] = 0.0;
      T0_1[3] = 0.0;
      smax_0[2] = 0.0;
      smax_0[3] = 0.0;
      q4[0] = 0.0;
      q5[0] = 0.0;

      /* MATLAB Function: '<S23>/MATLAB Function' */
      T0_1[6] = 1.0;
      T0_1[7] = 0.0;
      smax_0[6] = 0.0;
      smax_0[7] = 0.0;
      q4[1] = 0.0;
      q5[1] = 1.0;

      /* MATLAB Function: '<S23>/MATLAB Function' */
      T0_1[10] = 0.0;
      T0_1[11] = 0.0;
      smax_0[10] = 1.0;
      smax_0[11] = 0.0;
      q4[2] = 0.0;
      q5[2] = 0.0;

      /* MATLAB Function: '<S23>/MATLAB Function' */
      T0_1[14] = 0.0;
      T0_1[15] = 1.0;
      smax_0[14] = 0.0;
      smax_0[15] = 1.0;
      q4[3] = 1.0;
      q5[3] = 0.0;

      /* MATLAB Function: '<S23>/MATLAB Function' incorporates:
       *  DataStoreRead: '<S23>/Data Store Read1'
       *  DataStoreRead: '<S23>/Data Store Read2'
       *  Inport: '<Root>/feedback_encoder'
       */
      c3_0[0] = cos(ArmonyController_2020a_U_feedback_encoder[2]);
      c3_0[4] = 0.0;
      c3_0[8] = sin(ArmonyController_2020a_U_feedback_encoder[2]);
      c3_0[12] = 0.0;
      c3_0[1] = sin(ArmonyController_2020a_U_feedback_encoder[2]);
      c3_0[5] = 0.0;
      c3_0[9] = -cos(ArmonyController_2020a_U_feedback_encoder[2]);
      c3_0[13] = 0.0;
      i = 0;
      for (c_j = 0; c_j < 4; c_j++) {
        iy = 0;
        for (ix = 0; ix < 4; ix++) {
          pipk = iy + c_j;
          T0_2[pipk] = 0.0;
          T0_2[pipk] += smax_0[iy] * T0_1[c_j];
          T0_2[pipk] += smax_0[iy + 1] * T0_1[c_j + 4];
          T0_2[pipk] += smax_0[iy + 2] * T0_1[c_j + 8];
          T0_2[pipk] += smax_0[iy + 3] * T0_1[c_j + 12];
          iy += 4;
        }

        c3_0[i + 2] = q5[c_j];
        c3_0[i + 3] = q4[c_j];
        i += 4;
      }

      smax = cos(ArmonyController_2020a_U_feedback_encoder[3]);
      c3 = sin(ArmonyController_2020a_U_feedback_encoder[3]);
      T0_5[0] = smax;
      T0_5[4] = 0.0;
      T0_5[8] = c3;
      T0_5[12] = 0.0;
      T0_5[1] = c3;
      T0_5[5] = 0.0;
      T0_5[9] = -smax;
      T0_5[13] = 0.0;
      T0_5[2] = 0.0;
      T0_5[6] = -1.0;
      T0_5[10] = 0.0;
      T0_5[14] = ArmonyController_2020a_DW->L2;
      i = 0;
      for (c_j = 0; c_j < 4; c_j++) {
        iy = 0;
        for (ix = 0; ix < 4; ix++) {
          pipk = iy + c_j;
          T0_3[pipk] = 0.0;
          T0_3[pipk] += c3_0[iy] * T0_2[c_j];
          T0_3[pipk] += c3_0[iy + 1] * T0_2[c_j + 4];
          T0_3[pipk] += c3_0[iy + 2] * T0_2[c_j + 8];
          T0_3[pipk] += c3_0[iy + 3] * T0_2[c_j + 12];
          iy += 4;
        }

        T0_5[i + 3] = q4[c_j];
        i += 4;
      }

      smax = cos(ArmonyController_2020a_U_feedback_encoder[4]);
      c3 = sin(ArmonyController_2020a_U_feedback_encoder[4]);
      smax_0[0] = smax;
      smax_0[4] = 0.0;
      smax_0[8] = c3;
      smax_0[12] = 0.0;
      smax_0[1] = c3;
      smax_0[5] = 0.0;
      smax_0[9] = -smax;
      smax_0[13] = 0.0;
      i = 0;
      for (c_j = 0; c_j < 4; c_j++) {
        iy = 0;
        for (ix = 0; ix < 4; ix++) {
          pipk = iy + c_j;
          T0_4[pipk] = 0.0;
          T0_4[pipk] += T0_5[iy] * T0_3[c_j];
          T0_4[pipk] += T0_5[iy + 1] * T0_3[c_j + 4];
          T0_4[pipk] += T0_5[iy + 2] * T0_3[c_j + 8];
          T0_4[pipk] += T0_5[iy + 3] * T0_3[c_j + 12];
          iy += 4;
        }

        smax_0[i + 2] = q5[c_j];
        smax_0[i + 3] = q4[c_j];
        i += 4;
      }

      smax = sin(ArmonyController_2020a_U_feedback_encoder[5]);
      c3 = cos(ArmonyController_2020a_U_feedback_encoder[5]);
      c3_0[0] = c3;
      c3_0[4] = -smax;
      c3_0[8] = 0.0;
      c3_0[12] = 0.0;
      c3_0[1] = smax;
      c3_0[5] = c3;
      c3_0[9] = 0.0;
      c3_0[13] = 0.0;
      c3_0[2] = 0.0;
      c3_0[6] = 0.0;
      c3_0[10] = 1.0;
      c3_0[14] = ArmonyController_2020a_DW->L3;
      i = 0;
      for (c_j = 0; c_j < 4; c_j++) {
        iy = 0;
        for (ix = 0; ix < 4; ix++) {
          pipk = iy + c_j;
          T0_5[pipk] = 0.0;
          T0_5[pipk] += smax_0[iy] * T0_4[c_j];
          T0_5[pipk] += smax_0[iy + 1] * T0_4[c_j + 4];
          T0_5[pipk] += smax_0[iy + 2] * T0_4[c_j + 8];
          T0_5[pipk] += smax_0[iy + 3] * T0_4[c_j + 12];
          iy += 4;
        }

        c3_0[i + 3] = q4[c_j];
        i += 4;
      }

      i = 0;
      for (c_j = 0; c_j < 4; c_j++) {
        for (iy = 0; iy < 4; iy++) {
          pipk = iy + i;
          T0_5_0[pipk] = 0.0;
          T0_5_0[pipk] += c3_0[i] * T0_5[iy];
          T0_5_0[pipk] += c3_0[i + 1] * T0_5[iy + 4];
          T0_5_0[pipk] += c3_0[i + 2] * T0_5[iy + 8];
          T0_5_0[pipk] += c3_0[i + 3] * T0_5[iy + 12];
        }

        i += 4;
      }

      smax = T0_5_0[12] - T0_2[12];
      d_b_idx_0 = T0_5_0[12] - T0_3[12];
      e_b_idx_0 = T0_5_0[12] - T0_4[12];
      J[3] = 0.0;
      J[9] = T0_1[8];
      J[15] = T0_2[8];
      J[21] = T0_3[8];
      J[27] = T0_4[8];
      J[33] = T0_5[8];
      p_6_idx_0 = T0_5_0[12] - T0_5[12];

      /* MATLAB Function: '<S23>/MATLAB Function' */
      c_b_idx_1 = T0_5_0[13] - T0_2[13];
      c3 = T0_5_0[13] - T0_3[13];
      st_idx_1 = T0_5_0[13] - T0_4[13];
      J[4] = 0.0;
      J[10] = T0_1[9];
      J[16] = T0_2[9];
      J[22] = T0_3[9];
      J[28] = T0_4[9];
      J[34] = T0_5[9];
      p_6_idx_1 = T0_5_0[13] - T0_5[13];

      /* MATLAB Function: '<S23>/MATLAB Function' */
      c_b_idx_2 = T0_5_0[14] - T0_2[14];
      q1_idx_1 = T0_5_0[14] - T0_3[14];
      st_idx_2 = T0_5_0[14] - T0_4[14];
      p_6 = T0_5_0[14] - T0_5[14];
      J[5] = 1.0;
      J[11] = 0.0;
      J[17] = T0_2[10];
      J[23] = T0_3[10];
      J[29] = T0_4[10];
      J[35] = T0_5[10];
      J[0] = 0.0 * T0_5_0[14] - T0_5_0[13];
      J[1] = T0_5_0[12] - 0.0 * T0_5_0[14];
      J[2] = 0.0 * T0_5_0[13] - 0.0 * T0_5_0[12];
      J[6] = T0_5_0[14] * T0_1[9] - T0_5_0[13] * 0.0;
      J[7] = T0_5_0[12] * 0.0 - T0_5_0[14] * T0_1[8];
      J[8] = T0_5_0[13] * T0_1[8] - T0_5_0[12] * T0_1[9];
      J[12] = T0_2[9] * c_b_idx_2 - T0_2[10] * c_b_idx_1;
      J[13] = T0_2[10] * smax - T0_2[8] * c_b_idx_2;
      J[14] = T0_2[8] * c_b_idx_1 - T0_2[9] * smax;
      J[18] = T0_3[9] * q1_idx_1 - T0_3[10] * c3;
      J[19] = T0_3[10] * d_b_idx_0 - T0_3[8] * q1_idx_1;
      J[20] = T0_3[8] * c3 - T0_3[9] * d_b_idx_0;
      J[24] = T0_4[9] * st_idx_2 - T0_4[10] * st_idx_1;
      J[25] = T0_4[10] * e_b_idx_0 - T0_4[8] * st_idx_2;
      J[26] = T0_4[8] * st_idx_1 - T0_4[9] * e_b_idx_0;
      J[30] = T0_5[9] * p_6 - T0_5[10] * p_6_idx_1;
      J[31] = T0_5[10] * p_6_idx_0 - T0_5[8] * p_6;
      J[32] = T0_5[8] * p_6_idx_1 - T0_5[9] * p_6_idx_0;
      i = 0;
      c_j = 0;
      for (iy = 0; iy < 3; iy++) {
        rtb_R0_3[i] = J[c_j];
        rtb_R0_3[i + 1] = J[c_j + 1];
        rtb_R0_3[i + 2] = J[c_j + 2];
        i += 3;
        c_j += 6;
      }

      if (ArmonyController_2020a_det(rtb_R0_3) == 0.0) {
        /* Merge: '<S6>/Merge' */
        for (i = 0; i < 6; i++) {
          ArmonyController_2020a_B->Merge_p[i] = 0.0;
        }
      } else {
        i = 0;
        c_j = 0;
        for (iy = 0; iy < 3; iy++) {
          rtb_R0_3[i] = J[c_j + 21];
          rtb_R0_3[i + 1] = J[c_j + 22];
          rtb_R0_3[i + 2] = J[c_j + 23];
          i += 3;
          c_j += 6;
        }

        if (ArmonyController_2020a_det(rtb_R0_3) == 0.0) {
          /* Merge: '<S6>/Merge' */
          for (i = 0; i < 6; i++) {
            ArmonyController_2020a_B->Merge_p[i] = 0.0;
          }
        } else {
          memset(&a[0], 0, 36U * sizeof(real_T));
          for (i = 0; i < 6; i++) {
            ipiv[i] = (int8_T)(i + 1);
          }

          for (c_j = 0; c_j < 5; c_j++) {
            pipk = c_j * 7;
            iy = 0;
            ix = pipk;
            smax = fabs(J[pipk]);
            for (i = 2; i <= 6 - c_j; i++) {
              ix++;
              c3 = fabs(J[ix]);
              if (c3 > smax) {
                iy = i - 1;
                smax = c3;
              }
            }

            if (J[pipk + iy] != 0.0) {
              if (iy != 0) {
                iy += c_j;
                ipiv[c_j] = (int8_T)(iy + 1);
                ix = c_j;
                for (i = 0; i < 6; i++) {
                  smax = J[ix];
                  J[ix] = J[iy];
                  J[iy] = smax;
                  ix += 6;
                  iy += 6;
                }
              }

              iy = (pipk - c_j) + 6;
              for (ix = pipk + 1; ix < iy; ix++) {
                J[ix] /= J[pipk];
              }
            }

            iy = pipk;
            ix = pipk + 6;
            for (i = 0; i <= 4 - c_j; i++) {
              if (J[ix] != 0.0) {
                smax = -J[ix];
                c_ix = pipk + 1;
                e = (iy - c_j) + 12;
                for (ijA = iy + 7; ijA < e; ijA++) {
                  J[ijA] += J[c_ix] * smax;
                  c_ix++;
                }
              }

              ix += 6;
              iy += 6;
            }
          }

          for (i = 0; i < 6; i++) {
            p[i] = (int8_T)(i + 1);
          }

          for (c_j = 0; c_j < 5; c_j++) {
            ipiv_0 = ipiv[c_j];
            if (ipiv_0 > c_j + 1) {
              pipk = p[ipiv_0 - 1];
              p[ipiv_0 - 1] = p[c_j];
              p[c_j] = (int8_T)pipk;
            }
          }

          for (c_j = 0; c_j < 6; c_j++) {
            pipk = p[c_j] - 1;
            a[c_j + 6 * pipk] = 1.0;
            for (iy = c_j; iy + 1 < 7; iy++) {
              i = 6 * pipk + iy;
              if (a[i] != 0.0) {
                for (ix = iy + 1; ix + 1 < 7; ix++) {
                  c_ix = 6 * pipk + ix;
                  a[c_ix] -= a[i] * J[6 * iy + ix];
                }
              }
            }
          }

          for (c_j = 0; c_j < 6; c_j++) {
            pipk = 6 * c_j;
            for (iy = 5; iy >= 0; iy--) {
              ix = 6 * iy;
              i = iy + pipk;
              ct_tmp = a[i];
              if (ct_tmp != 0.0) {
                a[i] = ct_tmp / J[iy + ix];
                for (i = 0; i < iy; i++) {
                  c_ix = i + pipk;
                  a[c_ix] -= a[iy + pipk] * J[i + ix];
                }
              }
            }
          }

          /* Merge: '<S6>/Merge' incorporates:
           *  Inport: '<Root>/velocity'
           */
          for (i = 0; i < 6; i++) {
            ArmonyController_2020a_B->Merge_p[i] = 0.0;
            c_j = 0;
            for (iy = 0; iy < 6; iy++) {
              ArmonyController_2020a_B->Merge_p[i] += a[c_j + i] *
                ArmonyController_2020a_U_velocity[iy];
              c_j += 6;
            }
          }
        }
      }

      /* End of Outputs for SubSystem: '<S6>/global_frame' */
      break;

     case 1:
      /* Outputs for IfAction SubSystem: '<S6>/local_frame' incorporates:
       *  ActionPort: '<S24>/Action Port'
       */
      /* MATLAB Function: '<S24>/MATLAB Function' incorporates:
       *  DataStoreRead: '<S24>/Data Store Read'
       *  Inport: '<Root>/feedback_encoder'
       *  MATLAB Function: '<S24>/MATLAB Function1'
       */
      p_6 = cos(ArmonyController_2020a_U_feedback_encoder[1]);
      e_b_idx_0 = sin(ArmonyController_2020a_U_feedback_encoder[1]);
      p_6_idx_0 = cos(ArmonyController_2020a_U_feedback_encoder[2]);
      c_b_idx_1 = sin(ArmonyController_2020a_U_feedback_encoder[2]);
      c3 = sin(ArmonyController_2020a_U_feedback_encoder[3]);
      smax = cos(ArmonyController_2020a_U_feedback_encoder[3]);
      ct_tmp = cos(ArmonyController_2020a_U_feedback_encoder[0]);
      smax_0[0] = ct_tmp;
      smax_0[4] = 0.0;
      st_idx_0_tmp = sin(ArmonyController_2020a_U_feedback_encoder[0]);
      smax_0[8] = st_idx_0_tmp;
      smax_0[12] = 0.0;
      smax_0[1] = st_idx_0_tmp;
      smax_0[5] = 0.0;
      smax_0[9] = -ct_tmp;
      smax_0[13] = 0.0;
      T0_5[0] = p_6;
      T0_5[4] = -e_b_idx_0;
      T0_5[8] = 0.0;
      T0_5[12] = ArmonyController_2020a_DW->L1 * p_6;
      T0_5[1] = e_b_idx_0;
      T0_5[5] = p_6;
      T0_5[9] = 0.0;
      T0_5[13] = ArmonyController_2020a_DW->L1 * e_b_idx_0;
      smax_0[2] = 0.0;
      smax_0[3] = 0.0;
      T0_5[2] = 0.0;
      T0_5[3] = 0.0;
      q4[0] = 0.0;
      q5[0] = 0.0;

      /* MATLAB Function: '<S24>/MATLAB Function' */
      smax_0[6] = 1.0;
      smax_0[7] = 0.0;
      T0_5[6] = 0.0;
      T0_5[7] = 0.0;
      q4[1] = 0.0;
      q5[1] = 1.0;

      /* MATLAB Function: '<S24>/MATLAB Function' */
      smax_0[10] = 0.0;
      smax_0[11] = 0.0;
      T0_5[10] = 1.0;
      T0_5[11] = 0.0;
      q4[2] = 0.0;
      q5[2] = 0.0;

      /* MATLAB Function: '<S24>/MATLAB Function' */
      smax_0[14] = 0.0;
      smax_0[15] = 1.0;
      T0_5[14] = 0.0;
      T0_5[15] = 1.0;
      q4[3] = 1.0;
      q5[3] = 0.0;

      /* MATLAB Function: '<S24>/MATLAB Function' incorporates:
       *  DataStoreRead: '<S24>/Data Store Read1'
       */
      T0_1[0] = p_6_idx_0;
      T0_1[4] = 0.0;
      T0_1[8] = c_b_idx_1;
      T0_1[12] = 0.0;
      T0_1[1] = c_b_idx_1;
      T0_1[5] = 0.0;
      T0_1[9] = -p_6_idx_0;
      T0_1[13] = 0.0;
      i = 0;
      for (c_j = 0; c_j < 4; c_j++) {
        iy = 0;
        for (ix = 0; ix < 4; ix++) {
          pipk = iy + c_j;
          c3_0[pipk] = 0.0;
          c3_0[pipk] += T0_5[iy] * smax_0[c_j];
          c3_0[pipk] += T0_5[iy + 1] * smax_0[c_j + 4];
          c3_0[pipk] += T0_5[iy + 2] * smax_0[c_j + 8];
          c3_0[pipk] += T0_5[iy + 3] * smax_0[c_j + 12];
          iy += 4;
        }

        T0_1[i + 2] = q5[c_j];
        T0_1[i + 3] = q4[c_j];
        i += 4;
      }

      T0_2[0] = smax;
      T0_2[4] = 0.0;
      T0_2[8] = -c3;
      T0_2[12] = 0.0;
      T0_2[1] = c3;
      T0_2[5] = 0.0;
      T0_2[9] = smax;
      T0_2[13] = 0.0;
      T0_2[2] = 0.0;
      T0_2[6] = -1.0;
      T0_2[10] = 0.0;
      T0_2[14] = ArmonyController_2020a_DW->L2;
      i = 0;
      for (c_j = 0; c_j < 4; c_j++) {
        iy = 0;
        for (ix = 0; ix < 4; ix++) {
          pipk = iy + c_j;
          smax_0[pipk] = 0.0;
          smax_0[pipk] += T0_1[iy] * c3_0[c_j];
          smax_0[pipk] += T0_1[iy + 1] * c3_0[c_j + 4];
          smax_0[pipk] += T0_1[iy + 2] * c3_0[c_j + 8];
          smax_0[pipk] += T0_1[iy + 3] * c3_0[c_j + 12];
          iy += 4;
        }

        T0_2[i + 3] = q4[c_j];
        i += 4;
      }

      i = 0;
      for (c_j = 0; c_j < 4; c_j++) {
        for (iy = 0; iy < 4; iy++) {
          ix = iy + i;
          c3_0[ix] = 0.0;
          c3_0[ix] += T0_2[i] * smax_0[iy];
          c3_0[ix] += T0_2[i + 1] * smax_0[iy + 4];
          c3_0[ix] += T0_2[i + 2] * smax_0[iy + 8];
          c3_0[ix] += T0_2[i + 3] * smax_0[iy + 12];
        }

        i += 4;
      }

      /* Selector: '<S24>/R0_3' */
      i = 0;
      c_j = 0;
      for (iy = 0; iy < 3; iy++) {
        rtb_R0_3[i] = c3_0[c_j];
        rtb_R0_3[i + 1] = c3_0[c_j + 1];
        rtb_R0_3[i + 2] = c3_0[c_j + 2];
        i += 3;
        c_j += 4;
      }

      /* End of Selector: '<S24>/R0_3' */
      for (i = 0; i < 3; i++) {
        /* Product: '<S24>/Matrix Multiply' */
        rtb_R0_3_0[i] = 0.0;

        /* Product: '<S24>/Matrix Multiply1' incorporates:
         *  Product: '<S24>/Matrix Multiply'
         */
        rtb_R0_3_1[i] = 0.0;
        d_b_idx_0 = rtb_R0_3[i];

        /* Product: '<S24>/Matrix Multiply' incorporates:
         *  Inport: '<Root>/velocity'
         *  Selector: '<S24>/R0_3'
         */
        rtb_R0_3_0[i] += d_b_idx_0 * ArmonyController_2020a_U_velocity[0];

        /* Product: '<S24>/Matrix Multiply1' incorporates:
         *  Inport: '<Root>/velocity'
         *  Product: '<S24>/Matrix Multiply'
         */
        rtb_R0_3_1[i] += d_b_idx_0 * ArmonyController_2020a_U_velocity[3];
        d_b_idx_0 = rtb_R0_3[i + 3];

        /* Product: '<S24>/Matrix Multiply' incorporates:
         *  Inport: '<Root>/velocity'
         *  Selector: '<S24>/R0_3'
         */
        rtb_R0_3_0[i] += d_b_idx_0 * ArmonyController_2020a_U_velocity[1];

        /* Product: '<S24>/Matrix Multiply1' incorporates:
         *  Inport: '<Root>/velocity'
         *  Product: '<S24>/Matrix Multiply'
         */
        rtb_R0_3_1[i] += d_b_idx_0 * ArmonyController_2020a_U_velocity[4];
        d_b_idx_0 = rtb_R0_3[i + 6];

        /* Product: '<S24>/Matrix Multiply' incorporates:
         *  Inport: '<Root>/velocity'
         *  Selector: '<S24>/R0_3'
         */
        rtb_R0_3_0[i] += d_b_idx_0 * ArmonyController_2020a_U_velocity[2];

        /* Product: '<S24>/Matrix Multiply1' incorporates:
         *  Inport: '<Root>/velocity'
         *  Product: '<S24>/Matrix Multiply'
         */
        rtb_R0_3_1[i] += d_b_idx_0 * ArmonyController_2020a_U_velocity[5];

        /* Product: '<S24>/Matrix Multiply' */
        rtb_VectorConcatenate_l[i] = rtb_R0_3_0[i];

        /* Product: '<S24>/Matrix Multiply1' incorporates:
         *  Product: '<S24>/Matrix Multiply'
         */
        rtb_VectorConcatenate_l[i + 3] = rtb_R0_3_1[i];
      }

      /* MATLAB Function: '<S24>/MATLAB Function1' incorporates:
       *  DataStoreRead: '<S24>/Data Store Read2'
       *  Inport: '<Root>/feedback_encoder'
       *  Product: '<S24>/Matrix Multiply'
       *  Selector: '<S24>/R0_3'
       */
      T0_1[0] = ct_tmp;
      T0_1[4] = 0.0;
      T0_1[8] = st_idx_0_tmp;
      T0_1[12] = 0.0;
      T0_1[1] = st_idx_0_tmp;
      T0_1[5] = 0.0;
      T0_1[9] = -cos(ArmonyController_2020a_U_feedback_encoder[0]);
      T0_1[13] = 0.0;
      smax_0[0] = p_6;
      smax_0[4] = -sin(ArmonyController_2020a_U_feedback_encoder[1]);
      smax_0[8] = 0.0;
      smax_0[12] = ArmonyController_2020a_DW->L1 * cos
        (ArmonyController_2020a_U_feedback_encoder[1]);
      smax_0[1] = e_b_idx_0;
      smax_0[5] = p_6;
      smax_0[9] = 0.0;
      smax_0[13] = ArmonyController_2020a_DW->L1 * sin
        (ArmonyController_2020a_U_feedback_encoder[1]);
      T0_1[2] = 0.0;
      T0_1[3] = 0.0;
      smax_0[2] = 0.0;
      smax_0[3] = 0.0;
      q4[0] = 0.0;
      q5[0] = 0.0;

      /* MATLAB Function: '<S24>/MATLAB Function1' */
      T0_1[6] = 1.0;
      T0_1[7] = 0.0;
      smax_0[6] = 0.0;
      smax_0[7] = 0.0;
      q4[1] = 0.0;
      q5[1] = 1.0;

      /* MATLAB Function: '<S24>/MATLAB Function1' */
      T0_1[10] = 0.0;
      T0_1[11] = 0.0;
      smax_0[10] = 1.0;
      smax_0[11] = 0.0;
      q4[2] = 0.0;
      q5[2] = 0.0;

      /* MATLAB Function: '<S24>/MATLAB Function1' */
      T0_1[14] = 0.0;
      T0_1[15] = 1.0;
      smax_0[14] = 0.0;
      smax_0[15] = 1.0;
      q4[3] = 1.0;
      q5[3] = 0.0;

      /* MATLAB Function: '<S24>/MATLAB Function1' incorporates:
       *  DataStoreRead: '<S24>/Data Store Read3'
       *  DataStoreRead: '<S24>/Data Store Read4'
       *  Inport: '<Root>/feedback_encoder'
       */
      c3_0[0] = p_6_idx_0;
      c3_0[4] = 0.0;
      c3_0[8] = c_b_idx_1;
      c3_0[12] = 0.0;
      c3_0[1] = c_b_idx_1;
      c3_0[5] = 0.0;
      c3_0[9] = -cos(ArmonyController_2020a_U_feedback_encoder[2]);
      c3_0[13] = 0.0;
      i = 0;
      for (c_j = 0; c_j < 4; c_j++) {
        iy = 0;
        for (ix = 0; ix < 4; ix++) {
          pipk = iy + c_j;
          T0_2[pipk] = 0.0;
          T0_2[pipk] += smax_0[iy] * T0_1[c_j];
          T0_2[pipk] += smax_0[iy + 1] * T0_1[c_j + 4];
          T0_2[pipk] += smax_0[iy + 2] * T0_1[c_j + 8];
          T0_2[pipk] += smax_0[iy + 3] * T0_1[c_j + 12];
          iy += 4;
        }

        c3_0[i + 2] = q5[c_j];
        c3_0[i + 3] = q4[c_j];
        i += 4;
      }

      smax_0[0] = smax;
      smax_0[4] = 0.0;
      smax_0[8] = c3;
      smax_0[12] = 0.0;
      smax_0[1] = c3;
      smax_0[5] = 0.0;
      smax_0[9] = -smax;
      smax_0[13] = 0.0;
      smax_0[2] = 0.0;
      smax_0[6] = -1.0;
      smax_0[10] = 0.0;
      smax_0[14] = ArmonyController_2020a_DW->L2;
      i = 0;
      for (c_j = 0; c_j < 4; c_j++) {
        iy = 0;
        for (ix = 0; ix < 4; ix++) {
          pipk = iy + c_j;
          T0_3[pipk] = 0.0;
          T0_3[pipk] += c3_0[iy] * T0_2[c_j];
          T0_3[pipk] += c3_0[iy + 1] * T0_2[c_j + 4];
          T0_3[pipk] += c3_0[iy + 2] * T0_2[c_j + 8];
          T0_3[pipk] += c3_0[iy + 3] * T0_2[c_j + 12];
          iy += 4;
        }

        smax_0[i + 3] = q4[c_j];
        i += 4;
      }

      c3_0[0] = cos(ArmonyController_2020a_U_feedback_encoder[4]);
      c3_0[4] = 0.0;
      e_b_idx_0 = sin(ArmonyController_2020a_U_feedback_encoder[4]);
      c3_0[8] = e_b_idx_0;
      c3_0[12] = 0.0;
      c3_0[1] = e_b_idx_0;
      c3_0[5] = 0.0;
      c3_0[9] = -cos(ArmonyController_2020a_U_feedback_encoder[4]);
      c3_0[13] = 0.0;
      i = 0;
      for (c_j = 0; c_j < 4; c_j++) {
        iy = 0;
        for (ix = 0; ix < 4; ix++) {
          pipk = iy + c_j;
          T0_4[pipk] = 0.0;
          T0_4[pipk] += smax_0[iy] * T0_3[c_j];
          T0_4[pipk] += smax_0[iy + 1] * T0_3[c_j + 4];
          T0_4[pipk] += smax_0[iy + 2] * T0_3[c_j + 8];
          T0_4[pipk] += smax_0[iy + 3] * T0_3[c_j + 12];
          iy += 4;
        }

        c3_0[i + 2] = q5[c_j];
        c3_0[i + 3] = q4[c_j];
        i += 4;
      }

      smax_0[0] = cos(ArmonyController_2020a_U_feedback_encoder[5]);
      smax_0[4] = -sin(ArmonyController_2020a_U_feedback_encoder[5]);
      smax_0[8] = 0.0;
      smax_0[12] = 0.0;
      smax_0[1] = sin(ArmonyController_2020a_U_feedback_encoder[5]);
      smax_0[5] = cos(ArmonyController_2020a_U_feedback_encoder[5]);
      smax_0[9] = 0.0;
      smax_0[13] = 0.0;
      smax_0[2] = 0.0;
      smax_0[6] = 0.0;
      smax_0[10] = 1.0;
      smax_0[14] = ArmonyController_2020a_DW->L3;
      i = 0;
      for (c_j = 0; c_j < 4; c_j++) {
        iy = 0;
        for (ix = 0; ix < 4; ix++) {
          pipk = iy + c_j;
          T0_5[pipk] = 0.0;
          T0_5[pipk] += c3_0[iy] * T0_4[c_j];
          T0_5[pipk] += c3_0[iy + 1] * T0_4[c_j + 4];
          T0_5[pipk] += c3_0[iy + 2] * T0_4[c_j + 8];
          T0_5[pipk] += c3_0[iy + 3] * T0_4[c_j + 12];
          iy += 4;
        }

        smax_0[i + 3] = q4[c_j];
        i += 4;
      }

      i = 0;
      for (c_j = 0; c_j < 4; c_j++) {
        for (iy = 0; iy < 4; iy++) {
          pipk = iy + i;
          T0_5_0[pipk] = 0.0;
          T0_5_0[pipk] += smax_0[i] * T0_5[iy];
          T0_5_0[pipk] += smax_0[i + 1] * T0_5[iy + 4];
          T0_5_0[pipk] += smax_0[i + 2] * T0_5[iy + 8];
          T0_5_0[pipk] += smax_0[i + 3] * T0_5[iy + 12];
        }

        i += 4;
      }

      smax = T0_5_0[12] - T0_2[12];
      d_b_idx_0 = T0_5_0[12] - T0_3[12];
      e_b_idx_0 = T0_5_0[12] - T0_4[12];
      J[3] = 0.0;
      J[9] = st_idx_0_tmp;
      J[15] = T0_2[8];
      J[21] = T0_3[8];
      J[27] = T0_4[8];
      J[33] = T0_5[8];
      p_6_idx_0 = T0_5_0[12] - T0_5[12];

      /* MATLAB Function: '<S24>/MATLAB Function1' */
      c_b_idx_1 = T0_5_0[13] - T0_2[13];
      c3 = T0_5_0[13] - T0_3[13];
      st_idx_1 = T0_5_0[13] - T0_4[13];
      J[4] = 0.0;
      J[10] = -ct_tmp;
      J[16] = T0_2[9];
      J[22] = T0_3[9];
      J[28] = T0_4[9];
      J[34] = T0_5[9];
      p_6_idx_1 = T0_5_0[13] - T0_5[13];

      /* MATLAB Function: '<S24>/MATLAB Function1' */
      c_b_idx_2 = T0_5_0[14] - T0_2[14];
      q1_idx_1 = T0_5_0[14] - T0_3[14];
      st_idx_2 = T0_5_0[14] - T0_4[14];
      p_6 = T0_5_0[14] - T0_5[14];
      J[5] = 1.0;
      J[11] = 0.0;
      J[17] = T0_2[10];
      J[23] = T0_3[10];
      J[29] = T0_4[10];
      J[35] = T0_5[10];
      J[0] = 0.0 * T0_5_0[14] - T0_5_0[13];
      J[1] = T0_5_0[12] - 0.0 * T0_5_0[14];
      J[2] = 0.0 * T0_5_0[13] - 0.0 * T0_5_0[12];
      J[6] = T0_5_0[14] * -ct_tmp - T0_5_0[13] * 0.0;
      J[7] = T0_5_0[12] * 0.0 - T0_5_0[14] * st_idx_0_tmp;
      J[8] = T0_5_0[13] * st_idx_0_tmp - T0_5_0[12] * -ct_tmp;
      J[12] = T0_2[9] * c_b_idx_2 - T0_2[10] * c_b_idx_1;
      J[13] = T0_2[10] * smax - T0_2[8] * c_b_idx_2;
      J[14] = T0_2[8] * c_b_idx_1 - T0_2[9] * smax;
      J[18] = T0_3[9] * q1_idx_1 - T0_3[10] * c3;
      J[19] = T0_3[10] * d_b_idx_0 - T0_3[8] * q1_idx_1;
      J[20] = T0_3[8] * c3 - T0_3[9] * d_b_idx_0;
      J[24] = T0_4[9] * st_idx_2 - T0_4[10] * st_idx_1;
      J[25] = T0_4[10] * e_b_idx_0 - T0_4[8] * st_idx_2;
      J[26] = T0_4[8] * st_idx_1 - T0_4[9] * e_b_idx_0;
      J[30] = T0_5[9] * p_6 - T0_5[10] * p_6_idx_1;
      J[31] = T0_5[10] * p_6_idx_0 - T0_5[8] * p_6;
      J[32] = T0_5[8] * p_6_idx_1 - T0_5[9] * p_6_idx_0;
      i = 0;
      c_j = 0;
      for (iy = 0; iy < 3; iy++) {
        rtb_R0_3[i] = J[c_j];
        rtb_R0_3[i + 1] = J[c_j + 1];
        rtb_R0_3[i + 2] = J[c_j + 2];
        i += 3;
        c_j += 6;
      }

      if (ArmonyController_2020a_det(rtb_R0_3) == 0.0) {
        /* Merge: '<S6>/Merge' */
        for (i = 0; i < 6; i++) {
          ArmonyController_2020a_B->Merge_p[i] = 0.0;
        }
      } else {
        i = 0;
        c_j = 0;
        for (iy = 0; iy < 3; iy++) {
          rtb_R0_3[i] = J[c_j + 21];
          rtb_R0_3[i + 1] = J[c_j + 22];
          rtb_R0_3[i + 2] = J[c_j + 23];
          i += 3;
          c_j += 6;
        }

        if (ArmonyController_2020a_det(rtb_R0_3) == 0.0) {
          /* Merge: '<S6>/Merge' */
          for (i = 0; i < 6; i++) {
            ArmonyController_2020a_B->Merge_p[i] = 0.0;
          }
        } else {
          memset(&a[0], 0, 36U * sizeof(real_T));
          for (i = 0; i < 6; i++) {
            ipiv[i] = (int8_T)(i + 1);
          }

          for (c_j = 0; c_j < 5; c_j++) {
            pipk = c_j * 7;
            iy = 0;
            ix = pipk;
            smax = fabs(J[pipk]);
            for (i = 2; i <= 6 - c_j; i++) {
              ix++;
              c3 = fabs(J[ix]);
              if (c3 > smax) {
                iy = i - 1;
                smax = c3;
              }
            }

            if (J[pipk + iy] != 0.0) {
              if (iy != 0) {
                iy += c_j;
                ipiv[c_j] = (int8_T)(iy + 1);
                ix = c_j;
                for (i = 0; i < 6; i++) {
                  smax = J[ix];
                  J[ix] = J[iy];
                  J[iy] = smax;
                  ix += 6;
                  iy += 6;
                }
              }

              iy = (pipk - c_j) + 6;
              for (ix = pipk + 1; ix < iy; ix++) {
                J[ix] /= J[pipk];
              }
            }

            iy = pipk;
            ix = pipk + 6;
            for (i = 0; i <= 4 - c_j; i++) {
              if (J[ix] != 0.0) {
                smax = -J[ix];
                c_ix = pipk + 1;
                e = (iy - c_j) + 12;
                for (ijA = iy + 7; ijA < e; ijA++) {
                  J[ijA] += J[c_ix] * smax;
                  c_ix++;
                }
              }

              ix += 6;
              iy += 6;
            }
          }

          for (i = 0; i < 6; i++) {
            p[i] = (int8_T)(i + 1);
          }

          for (c_j = 0; c_j < 5; c_j++) {
            ipiv_0 = ipiv[c_j];
            if (ipiv_0 > c_j + 1) {
              pipk = p[ipiv_0 - 1];
              p[ipiv_0 - 1] = p[c_j];
              p[c_j] = (int8_T)pipk;
            }
          }

          for (c_j = 0; c_j < 6; c_j++) {
            pipk = p[c_j] - 1;
            a[c_j + 6 * pipk] = 1.0;
            for (iy = c_j; iy + 1 < 7; iy++) {
              i = 6 * pipk + iy;
              if (a[i] != 0.0) {
                for (ix = iy + 1; ix + 1 < 7; ix++) {
                  c_ix = 6 * pipk + ix;
                  a[c_ix] -= a[i] * J[6 * iy + ix];
                }
              }
            }
          }

          for (c_j = 0; c_j < 6; c_j++) {
            pipk = 6 * c_j;
            for (iy = 5; iy >= 0; iy--) {
              ix = 6 * iy;
              i = iy + pipk;
              ct_tmp = a[i];
              if (ct_tmp != 0.0) {
                a[i] = ct_tmp / J[iy + ix];
                for (i = 0; i < iy; i++) {
                  c_ix = i + pipk;
                  a[c_ix] -= a[iy + pipk] * J[i + ix];
                }
              }
            }
          }

          /* Merge: '<S6>/Merge' incorporates:
           *  Concatenate: '<S24>/Vector Concatenate'
           */
          for (i = 0; i < 6; i++) {
            ArmonyController_2020a_B->Merge_p[i] = 0.0;
            c_j = 0;
            for (iy = 0; iy < 6; iy++) {
              ArmonyController_2020a_B->Merge_p[i] += a[c_j + i] *
                rtb_VectorConcatenate_l[iy];
              c_j += 6;
            }
          }
        }
      }

      /* End of Outputs for SubSystem: '<S6>/local_frame' */
      break;
    }

    /* End of SwitchCase: '<S6>/Switch Case' */
    /* End of Outputs for SubSystem: '<S1>/velocity_control' */
    break;


   case 1:
    /* Outputs for IfAction SubSystem: '<S1>/position_control' incorporates:
     *  ActionPort: '<S2>/Action Port'
     */
    /* MATLAB Function: '<S10>/MATLAB Function' incorporates:
     *  DataStoreRead: '<S7>/Data Store Read'
     *  DataStoreRead: '<S7>/Data Store Read1'
     *  DataStoreRead: '<S7>/Data Store Read2'
     */
        printf("Position control \n");

    smax = (ArmonyController_2020a_DW->L1 + ArmonyController_2020a_DW->L2) +
      ArmonyController_2020a_DW->L3;

    /* SwitchCase: '<S7>/Switch Case' incorporates:
     *  Inport: '<Root>/position_EE'
     *  MATLAB Function: '<S10>/MATLAB Function'
     */
    if ((ArmonyController_2020a_U_pose_EE[0] *
         ArmonyController_2020a_U_pose_EE[0] +
         ArmonyController_2020a_U_pose_EE[1] *
         ArmonyController_2020a_U_pose_EE[1]) +
        ArmonyController_2020a_U_pose_EE[2] *
        ArmonyController_2020a_U_pose_EE[2] > smax * smax) {
      /* Outputs for IfAction SubSystem: '<S7>/HoldPosition' incorporates:
       *  ActionPort: '<S9>/Action Port'
       */
      for (i = 0; i < 6; i++) {
        /* Merge: '<S7>/Merge' incorporates:
         *  Inport: '<Root>/feedback_encoder'
         *  Inport: '<S9>/q_prev'
         */
        ArmonyController_2020a_B->Merge[i] =
          ArmonyController_2020a_U_feedback_encoder[i];
      }

      /* End of Outputs for SubSystem: '<S7>/HoldPosition' */
    } else {
      /* Outputs for IfAction SubSystem: '<S7>/CoreController ' incorporates:
       *  ActionPort: '<S8>/Action Port'
       */
      /* MATLAB Function: '<S14>/MATLAB Function1' incorporates:
       *  DataStoreRead: '<S8>/Data Store Read2'
       *  MATLAB Function: '<S14>/MATLAB Function'
       */
      ct_tmp = cos(ArmonyController_2020a_U_pose_EE[4]);
      st_idx_0_tmp = sin(ArmonyController_2020a_U_pose_EE[4]);
      st_idx_0 = ArmonyController_2020a_U_pose_EE[0] -
        ArmonyController_2020a_DW->L3 * st_idx_0_tmp;
      p_6_idx_0 = sin(ArmonyController_2020a_U_pose_EE[5]);
      st_idx_1 = ArmonyController_2020a_U_pose_EE[1] - -ct_tmp * p_6_idx_0 *
        ArmonyController_2020a_DW->L3;
      c_b_idx_1 = cos(ArmonyController_2020a_U_pose_EE[5]);
      st_idx_2 = ArmonyController_2020a_U_pose_EE[2] - c_b_idx_1 * ct_tmp *
        ArmonyController_2020a_DW->L3;

      /* MATLAB Function: '<S13>/MATLAB Function' incorporates:
       *  DataStoreRead: '<S8>/Data Store Read'
       *  DataStoreRead: '<S8>/Data Store Read1'
       *  Inport: '<Root>/feedback_encoder'
       */
      p_6_idx_1 = st_idx_1 * st_idx_1;
      e_b_idx_0 = st_idx_0 * st_idx_0 + p_6_idx_1;
      c3 = (((e_b_idx_0 + st_idx_2 * st_idx_2) - ArmonyController_2020a_DW->L1 *
             ArmonyController_2020a_DW->L1) - ArmonyController_2020a_DW->L2 *
            ArmonyController_2020a_DW->L2) / (2.0 *
        ArmonyController_2020a_DW->L1 * ArmonyController_2020a_DW->L2);
      c_b_idx_2 = sqrt(1.0 - c3 * c3);
      p_6 = rt_atan2d_snf(c_b_idx_2, c3);
      d_b_idx_0 = rt_atan2d_snf(-c_b_idx_2, c3);
      if ((p_6 + 1.5707963267948966 == 0.0) || (p_6 + 1.5707963267948966 ==
           3.1415926535897931) || (d_b_idx_0 + 1.5707963267948966 ==
           3.1415926535897931)) {
        i = 0;
        for (c_j = 0; c_j < 3; c_j++) {
          rtb_angles[i] = ArmonyController_2020a_U_feedback_encoder[c_j];
          rtb_angles[i + 1] = ArmonyController_2020a_U_feedback_encoder[c_j];
          rtb_angles[i + 2] = ArmonyController_2020a_U_feedback_encoder[c_j];
          rtb_angles[i + 3] = ArmonyController_2020a_U_feedback_encoder[c_j];
          i += 4;
        }
      } else {
        if ((st_idx_1 == 0.0) && (st_idx_0 == 0.0)) {
          q1_idx_0 = ArmonyController_2020a_U_feedback_encoder[0];
          q1_idx_1 = ArmonyController_2020a_U_feedback_encoder[0];
        } else {
          q1_idx_0 = rt_atan2d_snf(st_idx_1, st_idx_0);
          q1_idx_1 = rt_atan2d_snf(-st_idx_1, -st_idx_0);
        }

        smax = sqrt(e_b_idx_0);
        rtb_angles[0] = q1_idx_0;
        e_b_idx_0 = ArmonyController_2020a_DW->L2 * c3 +
          ArmonyController_2020a_DW->L1;
        rtb_angles_tmp = ArmonyController_2020a_DW->L2 * c_b_idx_2;
        rtb_angles_tmp_0 = e_b_idx_0 * st_idx_2;
        rtb_angles_tmp_1 = rtb_angles_tmp * st_idx_2;
        rtb_angles[4] = rt_atan2d_snf(rtb_angles_tmp_0 - rtb_angles_tmp * smax,
          e_b_idx_0 * smax + rtb_angles_tmp_1);
        rtb_angles[8] = p_6 + 1.5707963267948966;
        rtb_angles[1] = q1_idx_0;
        c_b_idx_2 = ArmonyController_2020a_DW->L2 * -c_b_idx_2;
        st_idx_0 *= st_idx_0;
        st_idx_2 *= c_b_idx_2;
        rtb_angles[5] = rt_atan2d_snf(rtb_angles_tmp_0 - c_b_idx_2 * smax,
          e_b_idx_0 * sqrt(st_idx_0 + p_6_idx_1) + st_idx_2);
        rtb_angles[9] = d_b_idx_0 + 1.5707963267948966;
        rtb_angles[2] = q1_idx_1;
        st_idx_0 = sqrt(st_idx_0 + st_idx_1 * st_idx_1);
        rtb_angles[6] = rt_atan2d_snf(rtb_angles_tmp_0 + st_idx_0 *
          rtb_angles_tmp, -e_b_idx_0 * smax + rtb_angles_tmp_1);
        rtb_angles[10] = p_6 + 1.5707963267948966;
        rtb_angles[3] = q1_idx_1;
        rtb_angles[7] = rt_atan2d_snf(rtb_angles_tmp_0 + st_idx_0 * c_b_idx_2,
          -(ArmonyController_2020a_DW->L2 * c3 + ArmonyController_2020a_DW->L1) *
          st_idx_0 + st_idx_2);
        rtb_angles[11] = d_b_idx_0 + 1.5707963267948966;
      }

      /* End of MATLAB Function: '<S13>/MATLAB Function' */

      /* SignalConversion generated from: '<S8>/Matrix Concatenate' */
      memcpy(&rtb_MatrixConcatenate[0], &rtb_angles[0], 12U * sizeof(real_T));

      /* MATLAB Function: '<S14>/MATLAB Function' incorporates:
       *  MATLAB Function: '<S14>/MATLAB Function1'
       */
      p_6 = cos(ArmonyController_2020a_U_pose_EE[3]);
      smax = sin(ArmonyController_2020a_U_pose_EE[3]);
      rtb_R0_3[0] = ct_tmp * p_6;
      rtb_R0_3[3] = -ct_tmp * smax;
      rtb_R0_3[6] = st_idx_0_tmp;
      c_b_idx_2 = p_6 * p_6_idx_0;
      rtb_R0_3[1] = c_b_idx_2 * st_idx_0_tmp + c_b_idx_1 * smax;
      p_6 *= c_b_idx_1;
      rtb_R0_3[4] = p_6 - p_6_idx_0 * st_idx_0_tmp * smax;
      rtb_R0_3[7] = -ct_tmp * p_6_idx_0;
      rtb_R0_3[2] = p_6_idx_0 * smax - p_6 * st_idx_0_tmp;
      rtb_R0_3[5] = c_b_idx_1 * st_idx_0_tmp * smax + c_b_idx_2;
      rtb_R0_3[8] = c_b_idx_1 * ct_tmp;

      /* MATLAB Function: '<S12>/MATLAB Function' */
      q4[0] = 0.0;
      q5[0] = 0.0;
      q6[0] = 0.0;
      q4[1] = 0.0;
      q5[1] = 0.0;
      q6[1] = 0.0;
      q4[2] = 0.0;
      q5[2] = 0.0;
      q6[2] = 0.0;
      q4[3] = 0.0;
      q5[3] = 0.0;
      q6[3] = 0.0;
      ix = 0;
      do {
        exitg1 = 0;
        if (ix < 4) {
          smax = cos(rtb_angles[ix]);
          c3 = sin(rtb_angles[ix]);
          p_6_idx_0 = rtb_angles[ix + 4] + rtb_angles[ix + 8];
          c_b_idx_2 = cos(p_6_idx_0);
          p_6 = sin(p_6_idx_0);
          smax_1[0] = smax * c_b_idx_2;
          smax_1[1] = c3;
          smax_1[2] = smax * p_6;
          smax_1[3] = c3 * c_b_idx_2;
          smax_1[4] = -smax;
          smax_1[5] = c3 * p_6;
          smax_1[6] = p_6;
          smax_1[7] = 0.0;
          smax_1[8] = -c_b_idx_2;
          for (i = 0; i < 3; i++) {
            c_j = 0;
            for (iy = 0; iy < 3; iy++) {
              pipk = c_j + i;
              R_in[pipk] = 0.0;
              R_in[pipk] += rtb_R0_3[c_j] * smax_1[i];
              R_in[pipk] += rtb_R0_3[c_j + 1] * smax_1[i + 3];
              R_in[pipk] += rtb_R0_3[c_j + 2] * smax_1[i + 6];
              c_j += 3;
            }
          }

          q4[ix] = rt_atan2d_snf(R_in[7], R_in[6]);
          q5[ix] = rt_atan2d_snf(sqrt(R_in[6] * R_in[6] + R_in[7] * R_in[7]),
            R_in[8]);
          if (q5[ix] == 0.0) {
            i = 0;
            for (c_j = 0; c_j < 3; c_j++) {
              ct_tmp = ArmonyController_2020a_U_feedback_encoder[c_j + 3];
              rtb_angles[i] = ct_tmp;
              rtb_angles[i + 1] = ct_tmp;
              rtb_angles[i + 2] = ct_tmp;
              rtb_angles[i + 3] = ct_tmp;
              i += 4;
            }

            memcpy(&rtb_MatrixConcatenate[12], &rtb_angles[0], 12U * sizeof
                   (real_T));
            exitg1 = 1;
          } else if (q5[ix] == 3.1415926535897931) {
            i = 0;
            for (c_j = 0; c_j < 3; c_j++) {
              ct_tmp = ArmonyController_2020a_U_feedback_encoder[c_j + 3];
              rtb_angles[i] = ct_tmp;
              rtb_angles[i + 1] = ct_tmp;
              rtb_angles[i + 2] = ct_tmp;
              rtb_angles[i + 3] = ct_tmp;
              i += 4;
            }

            memcpy(&rtb_MatrixConcatenate[12], &rtb_angles[0], 12U * sizeof
                   (real_T));
            exitg1 = 1;
          } else {
            q6[ix] = rt_atan2d_snf(R_in[5], -R_in[2]);
            ix++;
          }
        } else {
          rtb_angles[0] = q4[0];
          rtb_angles[4] = q5[0];
          rtb_angles[8] = q6[0];
          rtb_angles[1] = q4[1];
          rtb_angles[5] = q5[1];
          rtb_angles[9] = q6[1];
          rtb_angles[2] = q4[2];
          rtb_angles[6] = q5[2];
          rtb_angles[10] = q6[2];
          rtb_angles[3] = q4[3];
          rtb_angles[7] = q5[3];
          rtb_angles[11] = q6[3];
          memcpy(&rtb_MatrixConcatenate[12], &rtb_angles[0], 12U * sizeof(real_T));
          exitg1 = 1;
        }
      } while (exitg1 == 0);

      /* Merge: '<S7>/Merge' incorporates:
       *  Concatenate: '<S8>/Matrix Concatenate'
       *  Selector: '<S8>/Selector'
       *  SignalConversion generated from: '<S8>/joint_variables'
       */
      i = 0;
      for (c_j = 0; c_j < 6; c_j++) {
        ArmonyController_2020a_B->Merge[c_j] = rtb_MatrixConcatenate[i + 1];
        i += 4;
      }

      /* End of Outputs for SubSystem: '<S7>/CoreController ' */
    }

    /* End of SwitchCase: '<S7>/Switch Case' */
    for (i = 0; i < 6; i++) {
      /* Outport: '<Root>/q_d_dot' incorporates:
       *  SignalConversion generated from: '<S2>/q_d_dot'
       */
      ArmonyController_2020a_Y_q_d_dot[i] = 0.0;

      /* Outport: '<Root>/q_dot_max' incorporates:
       *  Outport: '<Root>/q_d_dot'
       *  SignalConversion generated from: '<S2>/q_d_dot'
       *  SignalConversion generated from: '<S2>/q_dot_max'
       */
      ArmonyController_2020a_Y_q_dot_max[i] = 0.0;
    }

    /* End of Outputs for SubSystem: '<S1>/position_control' */

    break;
  }


  /* End of SwitchCase: '<S1>/control_selection' */
  for (i = 0; i < 6; i++) {
    /* Sum: '<S4>/Subtract' incorporates:
     *  Inport: '<Root>/feedback_encoder'
     *  Merge: '<S7>/Merge'
     */
    p_6 = ArmonyController_2020a_B->Merge[i] -
      ArmonyController_2020a_U_feedback_encoder[i];

    /* Outport: '<Root>/delta_q' incorporates:
     *  Abs: '<S4>/Abs'
     *  Sum: '<S4>/Subtract'
     */
    ArmonyController_2020a_Y_delta_q[i] = fabs(p_6);

    /* Outport: '<Root>/direction' incorporates:
     *  Logic: '<S1>/Logical Operator'
     *  Merge: '<S6>/Merge'
     *  Sum: '<S4>/Subtract'
     *  Switch: '<S4>/Switch'
     *  Switch: '<S5>/Switch'
     */
    ArmonyController_2020a_Y_direction[i] = ((p_6 > 0.0) ||
      (ArmonyController_2020a_B->Merge_p[i] > 0.0));

    /* Outport: '<Root>/abs_q_dot' incorporates:
     *  Abs: '<S5>/Abs'
     *  Merge: '<S6>/Merge'
     *  Sum: '<S4>/Subtract'
     */
    ArmonyController_2020a_Y_abs_q_dot[i] = fabs
      (ArmonyController_2020a_B->Merge_p[i]);
  }

  /* Outport: '<Root>/control_flag_OUT' incorporates:
   *  Inport: '<Root>/control_flag_IN'
   */
  *ArmonyController_2020a_Y_control_flag_OUT =
    ArmonyController_2020a_U_control_flag_IN;

  /* Outport: '<Root>/controller_status_OUT' incorporates:
   *  Inport: '<Root>/controller_status_IN'
   */
  *ArmonyController_2020a_Y_controller_status_OUT =
    ArmonyController_2020a_U_controller_status_IN;

  /* MATLAB Function: '<S21>/MATLAB Function' incorporates:
   *  Inport: '<Root>/feedback_encoder'
   */
  smax = cos(ArmonyController_2020a_U_feedback_encoder[0]);
  e_b_idx_0 = ArmonyController_2020a_U_feedback_encoder[1] +
    ArmonyController_2020a_U_feedback_encoder[2];
  st_idx_1 = sin(e_b_idx_0);
  p_6_idx_0 = sin(ArmonyController_2020a_U_feedback_encoder[4]);
  p_6 = sin(ArmonyController_2020a_U_feedback_encoder[0]);
  d_b_idx_0 = cos(ArmonyController_2020a_U_feedback_encoder[1]);
  c_b_idx_1 = sin(ArmonyController_2020a_U_feedback_encoder[3]);
  ct_tmp = cos(e_b_idx_0);
  st_idx_0_tmp = cos(ArmonyController_2020a_U_feedback_encoder[3]);
  e_b_idx_0 = cos(ArmonyController_2020a_U_feedback_encoder[4]);
  c3 = cos(ArmonyController_2020a_U_feedback_encoder[5]);
  p_6_idx_1 = sin(ArmonyController_2020a_U_feedback_encoder[5]);
  c_b_idx_2 = st_idx_0_tmp * e_b_idx_0;
  T0_1[6] = (c_b_idx_2 * p_6_idx_1 + c_b_idx_1 * c3) * -st_idx_1 - ct_tmp *
    p_6_idx_0 * p_6_idx_1;
  q1_idx_1 = ct_tmp * e_b_idx_0;
  T0_1[10] = st_idx_1 * st_idx_0_tmp * p_6_idx_0 - q1_idx_1;

  /* Selector: '<S3>/extract_position' incorporates:
   *  DataStoreRead: '<S21>/Data Store Read3'
   *  DataStoreRead: '<S21>/Data Store Read4'
   *  DataStoreRead: '<S21>/Data Store Read5'
   *  Inport: '<Root>/feedback_encoder'
   *  MATLAB Function: '<S21>/MATLAB Function'
   *  Outport: '<Root>/p_EE'
   */
  ArmonyController_2020a_Y_p_EE[0] = ((ct_tmp * st_idx_0_tmp * p_6_idx_0 +
    st_idx_1 * e_b_idx_0) * smax + p_6 * c_b_idx_1 * p_6_idx_0) *
    ArmonyController_2020a_DW->L3 + (ArmonyController_2020a_DW->L1 * smax *
    d_b_idx_0 + ArmonyController_2020a_DW->L2 * smax * st_idx_1);
  ArmonyController_2020a_Y_p_EE[1] = ((ct_tmp * st_idx_0_tmp * p_6_idx_0 + sin
    (ArmonyController_2020a_U_feedback_encoder[1] +
     ArmonyController_2020a_U_feedback_encoder[2]) * cos
    (ArmonyController_2020a_U_feedback_encoder[4])) * p_6 - smax * c_b_idx_1 *
    p_6_idx_0) * ArmonyController_2020a_DW->L3 + (ArmonyController_2020a_DW->L1 *
    p_6 * d_b_idx_0 + ArmonyController_2020a_DW->L2 * p_6 * st_idx_1);
  ArmonyController_2020a_Y_p_EE[2] = (st_idx_1 * st_idx_0_tmp * p_6_idx_0 -
    q1_idx_1) * ArmonyController_2020a_DW->L3 + (ArmonyController_2020a_DW->L1 *
    sin(ArmonyController_2020a_U_feedback_encoder[1]) -
    ArmonyController_2020a_DW->L2 * ct_tmp);

  /* MATLAB Function: '<S21>/MATLAB Function' */
  d_b_idx_0 = c_b_idx_2 * c3 - c_b_idx_1 * p_6_idx_1;

  /* MATLAB Function: '<S3>/MATLAB Function' incorporates:
   *  MATLAB Function: '<S21>/MATLAB Function'
   *  Outport: '<Root>/p_EE'
   */
  ArmonyController_2020a_Y_p_EE[3] = rt_atan2d_snf((d_b_idx_0 * ct_tmp -
    st_idx_1 * p_6_idx_0 * c3) * p_6 - (c_b_idx_1 * e_b_idx_0 * c3 +
    st_idx_0_tmp * p_6_idx_1) * smax, ((st_idx_0_tmp * e_b_idx_0 * c3 -
    c_b_idx_1 * p_6_idx_1) * ct_tmp - st_idx_1 * p_6_idx_0 * c3) * smax +
    (c_b_idx_1 * e_b_idx_0 * c3 + st_idx_0_tmp * p_6_idx_1) * p_6);
  ArmonyController_2020a_Y_p_EE[4] = rt_atan2d_snf(-(d_b_idx_0 * st_idx_1 +
    ct_tmp * p_6_idx_0 * c3), sqrt(T0_1[6] * T0_1[6] + T0_1[10] * T0_1[10]));
  ArmonyController_2020a_Y_p_EE[5] = rt_atan2d_snf(T0_1[6], T0_1[10]);
}

/* Model initialize function */
void ArmonyController_2020a_initialize(RT_MODEL_ArmonyController_202_T *const
  ArmonyController_2020a_M, real_T ArmonyController_2020a_U_pose_EE[6],
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
  ArmonyController_2020a_Y_p_EE[6])
{
  B_ArmonyController_2020a_T *ArmonyController_2020a_B =
    ArmonyController_2020a_M->blockIO;
  DW_ArmonyController_2020a_T *ArmonyController_2020a_DW =
    ArmonyController_2020a_M->dwork;

  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* block I/O */
  (void) memset(((void *) ArmonyController_2020a_B), 0,
                sizeof(B_ArmonyController_2020a_T));

  /* states (dwork) */
  (void) memset((void *)ArmonyController_2020a_DW, 0,
                sizeof(DW_ArmonyController_2020a_T));

  /* external inputs */
  (void)memset(&ArmonyController_2020a_U_pose_EE[0], 0, 6U * sizeof(real_T));
  (void)memset(&ArmonyController_2020a_U_acceleration_EE[0], 0, 6U * sizeof
               (real_T));
  (void)memset(&ArmonyController_2020a_U_max_speed_EE[0], 0, 6U * sizeof(real_T));
  (void)memset(&ArmonyController_2020a_U_velocity[0], 0, 6U * sizeof(real_T));
  (void)memset(&ArmonyController_2020a_U_feedback_encoder[0], 0, 6U * sizeof
               (real_T));
  *ArmonyController_2020a_U_reference_frame = 0.0;
  *ArmonyController_2020a_U_control_flag_IN = 0.0;
  *ArmonyController_2020a_U_controller_status_IN = 0.0;

  /* external outputs */
  (void) memset(&ArmonyController_2020a_Y_delta_q[0], 0,
                6U*sizeof(real_T));
  (void) memset(&ArmonyController_2020a_Y_q_d_dot[0], 0,
                6U*sizeof(real_T));
  (void) memset(&ArmonyController_2020a_Y_q_dot_max[0], 0,
                6U*sizeof(real_T));
  (void) memset(&ArmonyController_2020a_Y_direction[0], 0,
                6U*sizeof(boolean_T));
  (void) memset(&ArmonyController_2020a_Y_abs_q_dot[0], 0,
                6U*sizeof(real_T));
  (*ArmonyController_2020a_Y_control_flag_OUT) = 0.0;
  (*ArmonyController_2020a_Y_controller_status_OUT) = 0.0;
  (void) memset(&ArmonyController_2020a_Y_p_EE[0], 0,
                6U*sizeof(real_T));

  {
    int32_T i;

    /* Start for DataStoreMemory: '<S1>/Data Store Memory2' */
    ArmonyController_2020a_DW->L1 = 620.0;

    /* Start for DataStoreMemory: '<S1>/Data Store Memory3' */
    ArmonyController_2020a_DW->L2 = 335.0;

    /* Start for DataStoreMemory: '<S1>/Data Store Memory4' */
    ArmonyController_2020a_DW->L3 = 288.0;

    /* SystemInitialize for IfAction SubSystem: '<S1>/position_control' */
    /* SystemInitialize for IfAction SubSystem: '<S1>/velocity_control' */
    for (i = 0; i < 6; i++) {
      /* SystemInitialize for Merge: '<S6>/Merge' */
      ArmonyController_2020a_B->Merge_p[i] = 0.0;

      /* SystemInitialize for Merge: '<S7>/Merge' incorporates:
       *  Merge: '<S6>/Merge'
       */
      ArmonyController_2020a_B->Merge[i] = 0.0;

      /* SystemInitialize for Outport: '<Root>/q_d_dot' incorporates:
       *  Merge: '<S6>/Merge'
       *  SignalConversion generated from: '<S2>/q_d_dot'
       */
      ArmonyController_2020a_Y_q_d_dot[i] = 0.0;

      /* SystemInitialize for Outport: '<Root>/q_dot_max' incorporates:
       *  Merge: '<S6>/Merge'
       *  SignalConversion generated from: '<S2>/q_dot_max'
       */
      ArmonyController_2020a_Y_q_dot_max[i] = 0.0;
    }

    /* End of SystemInitialize for SubSystem: '<S1>/velocity_control' */
    /* End of SystemInitialize for SubSystem: '<S1>/position_control' */
  }
}

/* Model terminate function */
void ArmonyController_2020a_terminate(RT_MODEL_ArmonyController_202_T *const
  ArmonyController_2020a_M)
{
  /* (no terminate code required) */
  UNUSED_PARAMETER(ArmonyController_2020a_M);
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
