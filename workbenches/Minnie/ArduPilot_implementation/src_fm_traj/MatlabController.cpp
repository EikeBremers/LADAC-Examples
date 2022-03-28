//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: MatlabController.cpp
//
// Code generated for Simulink model 'ArduCopter_MinnieTrajController'.
//
// Model version                  : 1.389
// Simulink Coder version         : 9.0 (R2018b) 24-May-2018
// C/C++ source code generated on : Mon Mar 28 11:53:50 2022
//
// Target selection: ert.tlc
// Embedded hardware selection: ARM Compatible->ARM Cortex
// Code generation objectives:
//    1. Execution efficiency
//    2. RAM efficiency
// Validation result: Not run
//

#include "MatlabController.h"
#define NumBitsPerChar                 8U

extern real32_T rt_atan2f_snf(real32_T u0, real32_T u1);
extern real_T rt_hypotd_snf(real_T u0, real_T u1);
extern real_T rt_atan2d_snf(real_T u0, real_T u1);
extern real_T rt_powd_snf(real_T u0, real_T u1);
extern real32_T rt_hypotf_snf(real32_T u0, real32_T u1);
static real32_T look1_iflf_binlx(real32_T u0, const real32_T bp0[], const
  real32_T table[], uint32_T maxIndex);
static void DCMtoquaternions(const real32_T rtu_M_bg[9], real32_T rty_q_bg[4]);
static void MATLABFunction4(const real32_T rtu_n_g[3], real32_T *rty_phi,
  real32_T *rty_delta);
static void MATLABFunction3(real32_T rtu_phi, real32_T rtu_delta, real32_T
  rty_q_cmd_red[4]);
static void MATLABFunction(const real32_T rtu_omega_Kb[3], const real32_T
  rtu_omega_Kb_dt[3], const real32_T rtu_M_bg[9], const real32_T rtu_n_g_d[3],
  const real32_T rtu_n_g_d_dt[3], const real32_T rtu_n_g_d_dt2[3], real32_T
  rty_n_b_d[3], real32_T rty_n_b_d_dt[3], real32_T rty_n_b_d_dt2[3]);

// Forward declaration for local functions
static real32_T norm(const real32_T x[3]);
extern "C" {
  extern real_T rtGetInf(void);
  extern real32_T rtGetInfF(void);
  extern real_T rtGetMinusInf(void);
  extern real32_T rtGetMinusInfF(void);
}                                      // extern "C"
  extern "C"
{
  extern real_T rtGetNaN(void);
  extern real32_T rtGetNaNF(void);
}                                      // extern "C"

//===========*
//  Constants *
// ===========
#define RT_PI                          3.14159265358979323846
#define RT_PIF                         3.1415927F
#define RT_LN_10                       2.30258509299404568402
#define RT_LN_10F                      2.3025851F
#define RT_LOG10E                      0.43429448190325182765
#define RT_LOG10EF                     0.43429449F
#define RT_E                           2.7182818284590452354
#define RT_EF                          2.7182817F

//
//  UNUSED_PARAMETER(x)
//    Used to specify that a function parameter (argument) is required but not
//    accessed by the function body.
#ifndef UNUSED_PARAMETER
# if defined(__LCC__)
#   define UNUSED_PARAMETER(x)                                   // do nothing
# else

//
//  This is the semi-ANSI standard way of indicating that an
//  unused function parameter is required.
#   define UNUSED_PARAMETER(x)         (void) (x)
# endif
#endif

extern "C" {
  extern real_T rtInf;
  extern real_T rtMinusInf;
  extern real_T rtNaN;
  extern real32_T rtInfF;
  extern real32_T rtMinusInfF;
  extern real32_T rtNaNF;
  extern void rt_InitInfAndNaN(size_t realSize);
  extern boolean_T rtIsInf(real_T value);
  extern boolean_T rtIsInfF(real32_T value);
  extern boolean_T rtIsNaN(real_T value);
  extern boolean_T rtIsNaNF(real32_T value);
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
}                                      // extern "C"
  extern "C"
{
  real_T rtInf;
  real_T rtMinusInf;
  real_T rtNaN;
  real32_T rtInfF;
  real32_T rtMinusInfF;
  real32_T rtNaNF;
}

extern "C" {
  //
  // Initialize rtInf needed by the generated code.
  // Inf is initialized as non-signaling. Assumes IEEE.
  //
  real_T rtGetInf(void)
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

  //
  // Initialize rtInfF needed by the generated code.
  // Inf is initialized as non-signaling. Assumes IEEE.
  //
  real32_T rtGetInfF(void)
  {
    IEEESingle infF;
    infF.wordL.wordLuint = 0x7F800000U;
    return infF.wordL.wordLreal;
  }

  //
  // Initialize rtMinusInf needed by the generated code.
  // Inf is initialized as non-signaling. Assumes IEEE.
  //
  real_T rtGetMinusInf(void)
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

  //
  // Initialize rtMinusInfF needed by the generated code.
  // Inf is initialized as non-signaling. Assumes IEEE.
  //
  real32_T rtGetMinusInfF(void)
  {
    IEEESingle minfF;
    minfF.wordL.wordLuint = 0xFF800000U;
    return minfF.wordL.wordLreal;
  }
}
  extern "C"
{
  //
  // Initialize rtNaN needed by the generated code.
  // NaN is initialized as non-signaling. Assumes IEEE.
  //
  real_T rtGetNaN(void)
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

  //
  // Initialize rtNaNF needed by the generated code.
  // NaN is initialized as non-signaling. Assumes IEEE.
  //
  real32_T rtGetNaNF(void)
  {
    IEEESingle nanF = { { 0 } };

    nanF.wordL.wordLuint = 0xFFC00000U;
    return nanF.wordL.wordLreal;
  }
}

extern "C" {
  //
  // Initialize the rtInf, rtMinusInf, and rtNaN needed by the
  // generated code. NaN is initialized as non-signaling. Assumes IEEE.
  //
  void rt_InitInfAndNaN(size_t realSize)
  {
    (void) (realSize);
    rtNaN = rtGetNaN();
    rtNaNF = rtGetNaNF();
    rtInf = rtGetInf();
    rtInfF = rtGetInfF();
    rtMinusInf = rtGetMinusInf();
    rtMinusInfF = rtGetMinusInfF();
  }

  // Test if value is infinite
  boolean_T rtIsInf(real_T value)
  {
    return (boolean_T)((value==rtInf || value==rtMinusInf) ? 1U : 0U);
  }

  // Test if single-precision value is infinite
  boolean_T rtIsInfF(real32_T value)
  {
    return (boolean_T)(((value)==rtInfF || (value)==rtMinusInfF) ? 1U : 0U);
  }

  // Test if value is not a number
  boolean_T rtIsNaN(real_T value)
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
      result = (boolean_T)((tmpVal.bitVal.words.wordH & 0x7FF00000) ==
                           0x7FF00000 &&
                           ( (tmpVal.bitVal.words.wordH & 0x000FFFFF) != 0 ||
                            (tmpVal.bitVal.words.wordL != 0) ));
    }

    return result;
  }

  // Test if single-precision value is not a number
  boolean_T rtIsNaNF(real32_T value)
  {
    IEEESingle tmp;
    tmp.wordL.wordLreal = value;
    return (boolean_T)( (tmp.wordL.wordLuint & 0x7F800000) == 0x7F800000 &&
                       (tmp.wordL.wordLuint & 0x007FFFFF) != 0 );
  }
}
  static real32_T look1_iflf_binlx(real32_T u0, const real32_T bp0[], const
  real32_T table[], uint32_T maxIndex)
{
  real32_T frac;
  uint32_T iRght;
  uint32_T iLeft;
  uint32_T bpIdx;

  // Column-major Lookup 1-D
  // Search method: 'binary'
  // Use previous index: 'off'
  // Interpolation method: 'Linear point-slope'
  // Extrapolation method: 'Linear'
  // Use last breakpoint for index at or above upper limit: 'off'
  // Remove protection against out-of-range input in generated code: 'off'

  // Prelookup - Index and Fraction
  // Index Search method: 'binary'
  // Extrapolation method: 'Linear'
  // Use previous index: 'off'
  // Use last breakpoint for index at or above upper limit: 'off'
  // Remove protection against out-of-range input in generated code: 'off'
  if (u0 <= bp0[0U]) {
    iLeft = 0U;
    frac = (u0 - bp0[0U]) / (bp0[1U] - bp0[0U]);
  } else if (u0 < bp0[maxIndex]) {
    // Binary Search
    bpIdx = maxIndex >> 1U;
    iLeft = 0U;
    iRght = maxIndex;
    while (iRght - iLeft > 1U) {
      if (u0 < bp0[bpIdx]) {
        iRght = bpIdx;
      } else {
        iLeft = bpIdx;
      }

      bpIdx = (iRght + iLeft) >> 1U;
    }

    frac = (u0 - bp0[iLeft]) / (bp0[iLeft + 1U] - bp0[iLeft]);
  } else {
    iLeft = maxIndex - 1U;
    frac = (u0 - bp0[maxIndex - 1U]) / (bp0[maxIndex] - bp0[maxIndex - 1U]);
  }

  // Column-major Interpolation 1-D
  // Interpolation method: 'Linear point-slope'
  // Use last breakpoint for index at or above upper limit: 'off'
  // Overflow mode: 'wrapping'
  return (table[iLeft + 1U] - table[iLeft]) * frac + table[iLeft];
}

//
// Output and update for atomic system:
//    '<S14>/DCM to quaternions'
//    '<S54>/DCM to quaternions'
//
static void DCMtoquaternions(const real32_T rtu_M_bg[9], real32_T rty_q_bg[4])
{
  real32_T q_0;
  real32_T q_1;
  real32_T q_2;
  real32_T q_3;
  real32_T q_bg_unsigned[4];
  int32_T idx;
  real32_T ex;
  int32_T k;
  q_0 = ((1.0F + rtu_M_bg[0]) + rtu_M_bg[4]) + rtu_M_bg[8];
  if (rtIsNaNF(q_0) || (!(0.0F < q_0))) {
    q_0 = 0.0F;
  }

  q_0 = 0.5F * std::sqrt(q_0);
  q_1 = ((1.0F + rtu_M_bg[0]) - rtu_M_bg[4]) - rtu_M_bg[8];
  if (rtIsNaNF(q_1) || (!(0.0F < q_1))) {
    q_1 = 0.0F;
  }

  q_1 = 0.5F * std::sqrt(q_1);
  q_2 = ((1.0F - rtu_M_bg[0]) + rtu_M_bg[4]) - rtu_M_bg[8];
  if (rtIsNaNF(q_2) || (!(0.0F < q_2))) {
    q_2 = 0.0F;
  }

  q_2 = 0.5F * std::sqrt(q_2);
  q_3 = ((1.0F - rtu_M_bg[0]) - rtu_M_bg[4]) + rtu_M_bg[8];
  if (rtIsNaNF(q_3) || (!(0.0F < q_3))) {
    q_3 = 0.0F;
  }

  q_3 = 0.5F * std::sqrt(q_3);
  q_bg_unsigned[0] = q_0;
  q_bg_unsigned[1] = q_1;
  q_bg_unsigned[2] = q_2;
  q_bg_unsigned[3] = q_3;
  idx = 0;
  ex = q_0;
  for (k = 1; k + 1 < 5; k++) {
    if (ex < q_bg_unsigned[k]) {
      ex = q_bg_unsigned[k];
      idx = k;
    }
  }

  switch (idx) {
   case 0:
    ex = rtu_M_bg[7] - rtu_M_bg[5];
    if (ex < 0.0F) {
      ex = -1.0F;
    } else if (ex > 0.0F) {
      ex = 1.0F;
    } else if (ex == 0.0F) {
      ex = 0.0F;
    } else {
      ex = (rtNaNF);
    }

    q_1 *= ex;
    ex = rtu_M_bg[2] - rtu_M_bg[6];
    if (ex < 0.0F) {
      ex = -1.0F;
    } else if (ex > 0.0F) {
      ex = 1.0F;
    } else if (ex == 0.0F) {
      ex = 0.0F;
    } else {
      ex = (rtNaNF);
    }

    q_2 *= ex;
    ex = rtu_M_bg[3] - rtu_M_bg[1];
    if (ex < 0.0F) {
      ex = -1.0F;
    } else if (ex > 0.0F) {
      ex = 1.0F;
    } else if (ex == 0.0F) {
      ex = 0.0F;
    } else {
      ex = (rtNaNF);
    }

    q_3 *= ex;
    break;

   case 1:
    ex = rtu_M_bg[7] - rtu_M_bg[5];
    if (ex < 0.0F) {
      ex = -1.0F;
    } else if (ex > 0.0F) {
      ex = 1.0F;
    } else if (ex == 0.0F) {
      ex = 0.0F;
    } else {
      ex = (rtNaNF);
    }

    q_0 *= ex;
    ex = rtu_M_bg[3] + rtu_M_bg[1];
    if (ex < 0.0F) {
      ex = -1.0F;
    } else if (ex > 0.0F) {
      ex = 1.0F;
    } else if (ex == 0.0F) {
      ex = 0.0F;
    } else {
      ex = (rtNaNF);
    }

    q_2 *= ex;
    ex = rtu_M_bg[2] + rtu_M_bg[6];
    if (ex < 0.0F) {
      ex = -1.0F;
    } else if (ex > 0.0F) {
      ex = 1.0F;
    } else if (ex == 0.0F) {
      ex = 0.0F;
    } else {
      ex = (rtNaNF);
    }

    q_3 *= ex;
    break;

   case 2:
    ex = rtu_M_bg[2] - rtu_M_bg[6];
    if (ex < 0.0F) {
      ex = -1.0F;
    } else if (ex > 0.0F) {
      ex = 1.0F;
    } else if (ex == 0.0F) {
      ex = 0.0F;
    } else {
      ex = (rtNaNF);
    }

    q_0 *= ex;
    ex = rtu_M_bg[3] + rtu_M_bg[1];
    if (ex < 0.0F) {
      ex = -1.0F;
    } else if (ex > 0.0F) {
      ex = 1.0F;
    } else if (ex == 0.0F) {
      ex = 0.0F;
    } else {
      ex = (rtNaNF);
    }

    q_1 *= ex;
    ex = rtu_M_bg[7] + rtu_M_bg[5];
    if (ex < 0.0F) {
      ex = -1.0F;
    } else if (ex > 0.0F) {
      ex = 1.0F;
    } else if (ex == 0.0F) {
      ex = 0.0F;
    } else {
      ex = (rtNaNF);
    }

    q_3 *= ex;
    break;

   case 3:
    ex = rtu_M_bg[3] - rtu_M_bg[1];
    if (ex < 0.0F) {
      ex = -1.0F;
    } else if (ex > 0.0F) {
      ex = 1.0F;
    } else if (ex == 0.0F) {
      ex = 0.0F;
    } else {
      ex = (rtNaNF);
    }

    q_0 *= ex;
    ex = rtu_M_bg[2] + rtu_M_bg[6];
    if (ex < 0.0F) {
      ex = -1.0F;
    } else if (ex > 0.0F) {
      ex = 1.0F;
    } else if (ex == 0.0F) {
      ex = 0.0F;
    } else {
      ex = (rtNaNF);
    }

    q_1 *= ex;
    ex = rtu_M_bg[7] + rtu_M_bg[5];
    if (ex < 0.0F) {
      ex = -1.0F;
    } else if (ex > 0.0F) {
      ex = 1.0F;
    } else if (ex == 0.0F) {
      ex = 0.0F;
    } else {
      ex = (rtNaNF);
    }

    q_2 *= ex;
    break;
  }

  rty_q_bg[0] = q_0;
  rty_q_bg[1] = q_1;
  rty_q_bg[2] = q_2;
  rty_q_bg[3] = q_3;
  q_1 = 1.29246971E-26F;
  q_2 = std::abs(rty_q_bg[0]);
  if (q_2 > 1.29246971E-26F) {
    q_0 = 1.0F;
    q_1 = q_2;
  } else {
    q_3 = q_2 / 1.29246971E-26F;
    q_0 = q_3 * q_3;
  }

  q_2 = std::abs(rty_q_bg[1]);
  if (q_2 > q_1) {
    q_3 = q_1 / q_2;
    q_0 = q_0 * q_3 * q_3 + 1.0F;
    q_1 = q_2;
  } else {
    q_3 = q_2 / q_1;
    q_0 += q_3 * q_3;
  }

  q_2 = std::abs(rty_q_bg[2]);
  if (q_2 > q_1) {
    q_3 = q_1 / q_2;
    q_0 = q_0 * q_3 * q_3 + 1.0F;
    q_1 = q_2;
  } else {
    q_3 = q_2 / q_1;
    q_0 += q_3 * q_3;
  }

  q_2 = std::abs(rty_q_bg[3]);
  if (q_2 > q_1) {
    q_3 = q_1 / q_2;
    q_0 = q_0 * q_3 * q_3 + 1.0F;
    q_1 = q_2;
  } else {
    q_3 = q_2 / q_1;
    q_0 += q_3 * q_3;
  }

  q_0 = q_1 * std::sqrt(q_0);
  if (rtIsNaNF(q_0) || (!(2.22044605E-16F < q_0))) {
    q_0 = 2.22044605E-16F;
  }

  rty_q_bg[0] /= q_0;
  rty_q_bg[1] /= q_0;
  rty_q_bg[2] /= q_0;
  rty_q_bg[3] /= q_0;
}

real32_T rt_atan2f_snf(real32_T u0, real32_T u1)
{
  real32_T y;
  int32_T u0_0;
  int32_T u1_0;
  if (rtIsNaNF(u0) || rtIsNaNF(u1)) {
    y = (rtNaNF);
  } else if (rtIsInfF(u0) && rtIsInfF(u1)) {
    if (u0 > 0.0F) {
      u0_0 = 1;
    } else {
      u0_0 = -1;
    }

    if (u1 > 0.0F) {
      u1_0 = 1;
    } else {
      u1_0 = -1;
    }

    y = std::atan2((real32_T)u0_0, (real32_T)u1_0);
  } else if (u1 == 0.0F) {
    if (u0 > 0.0F) {
      y = RT_PIF / 2.0F;
    } else if (u0 < 0.0F) {
      y = -(RT_PIF / 2.0F);
    } else {
      y = 0.0F;
    }
  } else {
    y = std::atan2(u0, u1);
  }

  return y;
}

//
// Output and update for atomic system:
//    '<S14>/MATLAB Function4'
//    '<S19>/MATLAB Function4'
//
static void MATLABFunction4(const real32_T rtu_n_g[3], real32_T *rty_phi,
  real32_T *rty_delta)
{
  real32_T minval;
  if ((!rtIsNaNF(-rtu_n_g[2])) && (1.0F > -rtu_n_g[2])) {
    minval = -rtu_n_g[2];
  } else {
    minval = 1.0F;
  }

  *rty_delta = rt_atan2f_snf(rtu_n_g[1], rtu_n_g[0]);
  if (rtIsNaNF(minval) || (!(-1.0F < minval))) {
    minval = -1.0F;
  }

  *rty_phi = std::acos(minval);
}

//
// Output and update for atomic system:
//    '<S23>/MATLAB Function3'
//    '<S23>/MATLAB Function8'
//
static void MATLABFunction3(real32_T rtu_phi, real32_T rtu_delta, real32_T
  rty_q_cmd_red[4])
{
  real32_T x;
  real32_T xy;
  real32_T b_x;
  real32_T b_x_tmp;
  x = std::sin(rtu_delta);
  b_x_tmp = std::cos(rtu_delta);
  xy = std::sqrt(x * x + -b_x_tmp * -b_x_tmp);
  if (xy <= 0.01) {
    x = 1.0F;
    xy = std::sqrt(-b_x_tmp * -b_x_tmp + 1.0F);
  }

  b_x = 1.0F / xy;
  xy = std::sin(rtu_phi / 2.0F);
  rty_q_cmd_red[0] = std::cos(rtu_phi / 2.0F);
  rty_q_cmd_red[1] = b_x * x * xy;
  rty_q_cmd_red[2] = b_x * -b_x_tmp * xy;
  rty_q_cmd_red[3] = b_x * 0.0F * xy;
}

// Function for MATLAB Function: '<S58>/MATLAB Function'
static real32_T norm(const real32_T x[3])
{
  real32_T y;
  real32_T scale;
  real32_T absxk;
  real32_T t;
  scale = 1.29246971E-26F;
  absxk = std::abs(x[0]);
  if (absxk > 1.29246971E-26F) {
    y = 1.0F;
    scale = absxk;
  } else {
    t = absxk / 1.29246971E-26F;
    y = t * t;
  }

  absxk = std::abs(x[1]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  absxk = std::abs(x[2]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  return scale * std::sqrt(y);
}

//
// Output and update for atomic system:
//    '<S58>/MATLAB Function'
//    '<S58>/MATLAB Function1'
//
static void MATLABFunction(const real32_T rtu_omega_Kb[3], const real32_T
  rtu_omega_Kb_dt[3], const real32_T rtu_M_bg[9], const real32_T rtu_n_g_d[3],
  const real32_T rtu_n_g_d_dt[3], const real32_T rtu_n_g_d_dt2[3], real32_T
  rty_n_b_d[3], real32_T rty_n_b_d_dt[3], real32_T rty_n_b_d_dt2[3])
{
  real32_T absxk;
  int32_T exponent;
  real32_T y;
  real32_T scale;
  real32_T b_absxk;
  real32_T t;
  real32_T rtu_omega_Kb_0[3];
  real32_T rtu_omega_Kb_dt_0[3];
  real32_T tmp[9];
  real32_T rtu_M_bg_0[9];
  int32_T i;
  int32_T i_0;
  int32_T rtu_M_bg_tmp;
  int32_T rtu_M_bg_tmp_0;
  for (i = 0; i < 3; i++) {
    rty_n_b_d[i] = 0.0F;
    rty_n_b_d[i] += rtu_M_bg[i] * rtu_n_g_d[0];
    rty_n_b_d[i] += rtu_M_bg[i + 3] * rtu_n_g_d[1];
    rty_n_b_d[i] += rtu_M_bg[i + 6] * rtu_n_g_d[2];
  }

  rtu_omega_Kb_0[0] = -(rtu_omega_Kb[1] * rty_n_b_d[2] - rtu_omega_Kb[2] *
                        rty_n_b_d[1]);
  rtu_omega_Kb_0[1] = -(rtu_omega_Kb[2] * rty_n_b_d[0] - rtu_omega_Kb[0] *
                        rty_n_b_d[2]);
  rtu_omega_Kb_0[2] = -(rtu_omega_Kb[0] * rty_n_b_d[1] - rtu_omega_Kb[1] *
                        rty_n_b_d[0]);
  for (i = 0; i < 3; i++) {
    rty_n_b_d_dt[i] = rtu_omega_Kb_0[i] + (rtu_M_bg[i + 6] * rtu_n_g_d_dt[2] +
      (rtu_M_bg[i + 3] * rtu_n_g_d_dt[1] + rtu_M_bg[i] * rtu_n_g_d_dt[0]));
  }

  rtu_omega_Kb_dt_0[0] = -(rtu_omega_Kb_dt[1] * rty_n_b_d[2] - rtu_omega_Kb_dt[2]
    * rty_n_b_d[1]);
  rtu_omega_Kb_dt_0[1] = -(rtu_omega_Kb_dt[2] * rty_n_b_d[0] - rtu_omega_Kb_dt[0]
    * rty_n_b_d[2]);
  rtu_omega_Kb_dt_0[2] = -(rtu_omega_Kb_dt[0] * rty_n_b_d[1] - rtu_omega_Kb_dt[1]
    * rty_n_b_d[0]);
  rtu_omega_Kb_0[0] = rtu_omega_Kb[1] * rty_n_b_d_dt[2] - rtu_omega_Kb[2] *
    rty_n_b_d_dt[1];
  rtu_omega_Kb_0[1] = rtu_omega_Kb[2] * rty_n_b_d_dt[0] - rtu_omega_Kb[0] *
    rty_n_b_d_dt[2];
  rtu_omega_Kb_0[2] = rtu_omega_Kb[0] * rty_n_b_d_dt[1] - rtu_omega_Kb[1] *
    rty_n_b_d_dt[0];
  tmp[0] = 0.0F;
  tmp[3] = -rtu_omega_Kb[2];
  tmp[6] = rtu_omega_Kb[1];
  tmp[1] = rtu_omega_Kb[2];
  tmp[4] = 0.0F;
  tmp[7] = -rtu_omega_Kb[0];
  tmp[2] = -rtu_omega_Kb[1];
  tmp[5] = rtu_omega_Kb[0];
  tmp[8] = 0.0F;
  for (i = 0; i < 3; i++) {
    absxk = 0.0F;
    for (i_0 = 0; i_0 < 3; i_0++) {
      rtu_M_bg_tmp = i + 3 * i_0;
      rtu_M_bg_0[rtu_M_bg_tmp] = 0.0F;
      rtu_M_bg_tmp_0 = 3 * i_0 + i;
      rtu_M_bg_0[rtu_M_bg_tmp] = rtu_M_bg_0[rtu_M_bg_tmp_0] + rtu_M_bg[3 * i_0] *
        tmp[3 * i];
      rtu_M_bg_0[rtu_M_bg_tmp] = rtu_M_bg[3 * i_0 + 1] * tmp[3 * i + 1] +
        rtu_M_bg_0[rtu_M_bg_tmp_0];
      rtu_M_bg_0[rtu_M_bg_tmp] = rtu_M_bg[3 * i_0 + 2] * tmp[3 * i + 2] +
        rtu_M_bg_0[rtu_M_bg_tmp_0];
      absxk += rtu_M_bg_0[rtu_M_bg_tmp_0] * rtu_n_g_d_dt[i_0];
    }

    rty_n_b_d_dt2[i] = ((rtu_omega_Kb_dt_0[i] - rtu_omega_Kb_0[i]) + absxk) +
      (rtu_M_bg[i + 6] * rtu_n_g_d_dt2[2] + (rtu_M_bg[i + 3] * rtu_n_g_d_dt2[1]
        + rtu_M_bg[i] * rtu_n_g_d_dt2[0]));
  }

  if (rty_n_b_d[2] > 0.0F) {
    rty_n_b_d[2] = 0.0F;
    t = std::abs(rty_n_b_d[0]);
    if ((!rtIsInfF(t)) && (!rtIsNaNF(t))) {
      if (t <= 1.17549435E-38F) {
        absxk = 1.4013E-45F;
      } else {
        std::frexp(t, &exponent);
        absxk = std::ldexp(1.0F, exponent - 24);
      }
    } else {
      absxk = (rtNaNF);
    }

    scale = 1.29246971E-26F;
    if (t > 1.29246971E-26F) {
      y = 1.0F;
      scale = t;
    } else {
      t /= 1.29246971E-26F;
      y = t * t;
    }

    b_absxk = std::abs(rty_n_b_d[1]);
    if (b_absxk > scale) {
      t = scale / b_absxk;
      y = y * t * t + 1.0F;
      scale = b_absxk;
    } else {
      t = b_absxk / scale;
      y += t * t;
    }

    y = scale * std::sqrt(y);
    absxk += y;
    rty_n_b_d[0] += (rty_n_b_d[0] / absxk - rty_n_b_d[0]) * 2.0F;
    rty_n_b_d[1] += (rty_n_b_d[1] / absxk - rty_n_b_d[1]) * 2.0F;
    if (norm(rty_n_b_d) < 0.9) {
      rty_n_b_d[0] = -1.0F;
      absxk = norm(rty_n_b_d);
      rty_n_b_d[0] /= absxk;
      rty_n_b_d[1] /= absxk;
      rty_n_b_d[2] /= absxk;
    }

    rty_n_b_d_dt[0] = -rty_n_b_d_dt[0] * 0.2F;
    rty_n_b_d_dt2[0] = -rty_n_b_d_dt2[0] * 0.2F;
    rty_n_b_d_dt[1] = -rty_n_b_d_dt[1] * 0.2F;
    rty_n_b_d_dt2[1] = -rty_n_b_d_dt2[1] * 0.2F;
    rty_n_b_d_dt[2] = -rty_n_b_d_dt[2] * 0.2F;
    rty_n_b_d_dt2[2] = -rty_n_b_d_dt2[2] * 0.2F;
  }
}

void MatlabControllerClass::emxInit_real_T(emxArray_real_T **pEmxArray, int32_T
  numDimensions)
{
  emxArray_real_T *emxArray;
  int32_T i;
  *pEmxArray = (emxArray_real_T *)malloc(sizeof(emxArray_real_T));
  emxArray = *pEmxArray;
  emxArray->data = (real_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int32_T *)malloc(sizeof(int32_T) * numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

void MatlabControllerClass::emxEnsureCapacity_real_T(emxArray_real_T *emxArray,
  int32_T oldNumel)
{
  int32_T newNumel;
  int32_T i;
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  newNumel = 1;
  for (i = 0; i < emxArray->numDimensions; i++) {
    newNumel *= emxArray->size[i];
  }

  if (newNumel > emxArray->allocatedSize) {
    i = emxArray->allocatedSize;
    if (i < 16) {
      i = 16;
    }

    while (i < newNumel) {
      if (i > 1073741823) {
        i = MAX_int32_T;
      } else {
        i <<= 1;
      }
    }

    newData = calloc((uint32_T)i, sizeof(real_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(real_T) * oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = (real_T *)newData;
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
}

void MatlabControllerClass::emxFree_real_T(emxArray_real_T **pEmxArray)
{
  if (*pEmxArray != (emxArray_real_T *)NULL) {
    if (((*pEmxArray)->data != (real_T *)NULL) && (*pEmxArray)->canFreeData) {
      free((*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_real_T *)NULL;
  }
}

void MatlabControllerClass::emxInit_int32_T(emxArray_int32_T **pEmxArray,
  int32_T numDimensions)
{
  emxArray_int32_T *emxArray;
  int32_T i;
  *pEmxArray = (emxArray_int32_T *)malloc(sizeof(emxArray_int32_T));
  emxArray = *pEmxArray;
  emxArray->data = (int32_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int32_T *)malloc(sizeof(int32_T) * numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

void MatlabControllerClass::emxEnsureCapacity_int32_T(emxArray_int32_T *emxArray,
  int32_T oldNumel)
{
  int32_T newNumel;
  int32_T i;
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  newNumel = 1;
  for (i = 0; i < emxArray->numDimensions; i++) {
    newNumel *= emxArray->size[i];
  }

  if (newNumel > emxArray->allocatedSize) {
    i = emxArray->allocatedSize;
    if (i < 16) {
      i = 16;
    }

    while (i < newNumel) {
      if (i > 1073741823) {
        i = MAX_int32_T;
      } else {
        i <<= 1;
      }
    }

    newData = calloc((uint32_T)i, sizeof(int32_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(int32_T) * oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = (int32_T *)newData;
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
}

// Function for MATLAB Function: '<S104>/trajFromWaypoints'
real_T MatlabControllerClass::xnrm2_i(int32_T n, const emxArray_real_T *x,
  int32_T ix0)
{
  real_T y;
  real_T scale;
  int32_T kend;
  real_T absxk;
  real_T t;
  int32_T k;
  y = 0.0;
  if (n >= 1) {
    if (n == 1) {
      y = std::abs(x->data[ix0 - 1]);
    } else {
      scale = 3.3121686421112381E-170;
      kend = (ix0 + n) - 1;
      for (k = ix0; k <= kend; k++) {
        absxk = std::abs(x->data[k - 1]);
        if (absxk > scale) {
          t = scale / absxk;
          y = y * t * t + 1.0;
          scale = absxk;
        } else {
          t = absxk / scale;
          y += t * t;
        }
      }

      y = scale * std::sqrt(y);
    }
  }

  return y;
}

real_T rt_hypotd_snf(real_T u0, real_T u1)
{
  real_T y;
  real_T a;
  a = std::abs(u0);
  y = std::abs(u1);
  if (a < y) {
    a /= y;
    y *= std::sqrt(a * a + 1.0);
  } else if (a > y) {
    y /= a;
    y = std::sqrt(y * y + 1.0) * a;
  } else {
    if (!rtIsNaN(y)) {
      y = a * 1.4142135623730951;
    }
  }

  return y;
}

// Function for MATLAB Function: '<S104>/trajFromWaypoints'
void MatlabControllerClass::xscal(int32_T n, real_T a, emxArray_real_T *x,
  int32_T ix0)
{
  int32_T b;
  int32_T k;
  b = (ix0 + n) - 1;
  for (k = ix0; k <= b; k++) {
    x->data[k - 1] *= a;
  }
}

// Function for MATLAB Function: '<S104>/trajFromWaypoints'
void MatlabControllerClass::xgeqp3(emxArray_real_T *A, emxArray_real_T *tau,
  emxArray_int32_T *jpvt)
{
  int32_T m;
  int32_T n;
  int32_T mn;
  emxArray_real_T *work;
  emxArray_real_T *vn1;
  emxArray_real_T *vn2;
  int32_T i_i;
  int32_T nmi;
  int32_T b_n;
  int32_T yk;
  int32_T ix;
  real_T smax;
  real_T s;
  int32_T b_ix;
  int32_T iy;
  int32_T knt;
  int32_T c_ix;
  int32_T e;
  int32_T b_ia;
  int32_T d_ix;
  int32_T exitg1;
  boolean_T exitg2;
  m = A->size[0];
  n = A->size[1];
  b_n = A->size[0];
  mn = A->size[1];
  if (b_n < mn) {
    mn = b_n;
  }

  yk = tau->size[0];
  tau->size[0] = mn;
  emxEnsureCapacity_real_T(tau, yk);
  if (A->size[1] < 1) {
    b_n = 0;
  } else {
    b_n = A->size[1];
  }

  yk = jpvt->size[0] * jpvt->size[1];
  jpvt->size[0] = 1;
  jpvt->size[1] = b_n;
  emxEnsureCapacity_int32_T(jpvt, yk);
  if (b_n > 0) {
    jpvt->data[0] = 1;
    yk = 1;
    for (i_i = 2; i_i <= b_n; i_i++) {
      yk++;
      jpvt->data[i_i - 1] = yk;
    }
  }

  if ((A->size[0] != 0) && (A->size[1] != 0)) {
    emxInit_real_T(&work, 1);
    b_n = A->size[1];
    yk = work->size[0];
    work->size[0] = b_n;
    emxEnsureCapacity_real_T(work, yk);
    for (yk = 0; yk < b_n; yk++) {
      work->data[yk] = 0.0;
    }

    emxInit_real_T(&vn1, 1);
    emxInit_real_T(&vn2, 1);
    b_n = A->size[1];
    yk = vn1->size[0];
    vn1->size[0] = b_n;
    emxEnsureCapacity_real_T(vn1, yk);
    b_n = A->size[1];
    yk = vn2->size[0];
    vn2->size[0] = b_n;
    emxEnsureCapacity_real_T(vn2, yk);
    b_n = 1;
    for (yk = 0; yk < n; yk++) {
      vn1->data[yk] = xnrm2_i(m, A, b_n);
      vn2->data[yk] = vn1->data[yk];
      b_n += m;
    }

    for (b_n = 0; b_n < mn; b_n++) {
      i_i = b_n * m + b_n;
      nmi = n - b_n;
      yk = (m - b_n) - 1;
      if (nmi < 1) {
        knt = 0;
      } else {
        knt = 1;
        if (nmi > 1) {
          ix = b_n;
          smax = std::abs(vn1->data[b_n]);
          for (b_ix = 2; b_ix <= nmi; b_ix++) {
            ix++;
            s = std::abs(vn1->data[ix]);
            if (s > smax) {
              knt = b_ix;
              smax = s;
            }
          }
        }
      }

      ix = (b_n + knt) - 1;
      if (ix + 1 != b_n + 1) {
        b_ix = m * ix;
        iy = m * b_n;
        for (knt = 0; knt < m; knt++) {
          smax = A->data[b_ix];
          A->data[b_ix] = A->data[iy];
          A->data[iy] = smax;
          b_ix++;
          iy++;
        }

        b_ix = jpvt->data[ix];
        jpvt->data[ix] = jpvt->data[b_n];
        jpvt->data[b_n] = b_ix;
        vn1->data[ix] = vn1->data[b_n];
        vn2->data[ix] = vn2->data[b_n];
      }

      if (b_n + 1 < m) {
        smax = A->data[i_i];
        tau->data[b_n] = 0.0;
        if (1 + yk > 0) {
          s = xnrm2_i(yk, A, i_i + 2);
          if (s != 0.0) {
            s = rt_hypotd_snf(A->data[i_i], s);
            if (A->data[i_i] >= 0.0) {
              s = -s;
            }

            if (std::abs(s) < 1.0020841800044864E-292) {
              knt = -1;
              do {
                knt++;
                xscal(yk, 9.9792015476736E+291, A, i_i + 2);
                s *= 9.9792015476736E+291;
                smax *= 9.9792015476736E+291;
              } while (!(std::abs(s) >= 1.0020841800044864E-292));

              s = rt_hypotd_snf(smax, xnrm2_i(yk, A, i_i + 2));
              if (smax >= 0.0) {
                s = -s;
              }

              tau->data[b_n] = (s - smax) / s;
              xscal(yk, 1.0 / (smax - s), A, i_i + 2);
              for (ix = 0; ix <= knt; ix++) {
                s *= 1.0020841800044864E-292;
              }

              smax = s;
            } else {
              tau->data[b_n] = (s - A->data[i_i]) / s;
              xscal(yk, 1.0 / (A->data[i_i] - s), A, i_i + 2);
              smax = s;
            }
          }
        }

        A->data[i_i] = smax;
      } else {
        tau->data[b_n] = 0.0;
      }

      if (b_n + 1 < n) {
        smax = A->data[i_i];
        A->data[i_i] = 1.0;
        b_ix = ((b_n + 1) * m + b_n) + 1;
        if (tau->data[b_n] != 0.0) {
          ix = yk + 1;
          knt = i_i + yk;
          while ((ix > 0) && (A->data[knt] == 0.0)) {
            ix--;
            knt--;
          }

          knt = nmi - 2;
          exitg2 = false;
          while ((!exitg2) && (knt + 1 > 0)) {
            nmi = knt * m + b_ix;
            iy = nmi;
            do {
              exitg1 = 0;
              if (iy <= (nmi + ix) - 1) {
                if (A->data[iy - 1] != 0.0) {
                  exitg1 = 1;
                } else {
                  iy++;
                }
              } else {
                knt--;
                exitg1 = 2;
              }
            } while (exitg1 == 0);

            if (exitg1 == 1) {
              exitg2 = true;
            }
          }

          nmi = knt;
        } else {
          ix = 0;
          nmi = -1;
        }

        if (ix > 0) {
          if (nmi + 1 != 0) {
            for (knt = 0; knt <= nmi; knt++) {
              work->data[knt] = 0.0;
            }

            knt = 0;
            iy = m * nmi + b_ix;
            d_ix = b_ix;
            while (((m > 0) && (d_ix <= iy)) || ((m < 0) && (d_ix >= iy))) {
              c_ix = i_i;
              s = 0.0;
              e = (d_ix + ix) - 1;
              for (b_ia = d_ix; b_ia <= e; b_ia++) {
                s += A->data[b_ia - 1] * A->data[c_ix];
                c_ix++;
              }

              work->data[knt] += s;
              knt++;
              d_ix += m;
            }
          }

          if (!(-tau->data[b_n] == 0.0)) {
            b_ix--;
            knt = 0;
            for (iy = 0; iy <= nmi; iy++) {
              if (work->data[knt] != 0.0) {
                s = work->data[knt] * -tau->data[b_n];
                d_ix = i_i;
                c_ix = ix + b_ix;
                for (e = b_ix; e < c_ix; e++) {
                  A->data[e] += A->data[d_ix] * s;
                  d_ix++;
                }
              }

              knt++;
              b_ix += m;
            }
          }
        }

        A->data[i_i] = smax;
      }

      for (i_i = b_n + 1; i_i < n; i_i++) {
        if (vn1->data[i_i] != 0.0) {
          smax = std::abs(A->data[A->size[0] * i_i + b_n]) / vn1->data[i_i];
          smax = 1.0 - smax * smax;
          if (smax < 0.0) {
            smax = 0.0;
          }

          s = vn1->data[i_i] / vn2->data[i_i];
          s = s * s * smax;
          if (s <= 1.4901161193847656E-8) {
            if (b_n + 1 < m) {
              vn1->data[i_i] = xnrm2_i(yk, A, (m * i_i + b_n) + 2);
              vn2->data[i_i] = vn1->data[i_i];
            } else {
              vn1->data[i_i] = 0.0;
              vn2->data[i_i] = 0.0;
            }
          } else {
            vn1->data[i_i] *= std::sqrt(smax);
          }
        }
      }
    }

    emxFree_real_T(&vn2);
    emxFree_real_T(&vn1);
    emxFree_real_T(&work);
  }
}

void MatlabControllerClass::emxFree_int32_T(emxArray_int32_T **pEmxArray)
{
  if (*pEmxArray != (emxArray_int32_T *)NULL) {
    if (((*pEmxArray)->data != (int32_T *)NULL) && (*pEmxArray)->canFreeData) {
      free((*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_int32_T *)NULL;
  }
}

// Function for MATLAB Function: '<S104>/trajFromWaypoints'
void MatlabControllerClass::lusolve(const emxArray_real_T *A, emxArray_real_T
  *B_9)
{
  int32_T n;
  emxArray_real_T *b_A;
  emxArray_int32_T *ipiv;
  int32_T mmj;
  int32_T b_n;
  int32_T yk;
  int32_T b_c;
  int32_T ix;
  real_T smax;
  real_T s;
  int32_T iy;
  int32_T c_ix;
  int32_T e;
  int32_T ijA;
  int32_T b_kAcol;
  emxInit_real_T(&b_A, 2);
  n = A->size[1];
  yk = b_A->size[0] * b_A->size[1];
  b_A->size[0] = A->size[0];
  b_A->size[1] = A->size[1];
  emxEnsureCapacity_real_T(b_A, yk);
  b_kAcol = A->size[0] * A->size[1] - 1;
  for (yk = 0; yk <= b_kAcol; yk++) {
    b_A->data[yk] = A->data[yk];
  }

  emxInit_int32_T(&ipiv, 2);
  b_kAcol = A->size[1];
  b_n = A->size[1];
  if (b_kAcol < b_n) {
    b_n = b_kAcol;
  }

  if (b_n < 1) {
    b_n = 0;
  }

  yk = ipiv->size[0] * ipiv->size[1];
  ipiv->size[0] = 1;
  ipiv->size[1] = b_n;
  emxEnsureCapacity_int32_T(ipiv, yk);
  if (b_n > 0) {
    ipiv->data[0] = 1;
    yk = 1;
    for (b_kAcol = 2; b_kAcol <= b_n; b_kAcol++) {
      yk++;
      ipiv->data[b_kAcol - 1] = yk;
    }
  }

  if (A->size[1] >= 1) {
    b_kAcol = A->size[1] - 1;
    yk = A->size[1];
    if (b_kAcol < yk) {
      yk = b_kAcol;
    }

    for (b_kAcol = 0; b_kAcol < yk; b_kAcol++) {
      mmj = n - b_kAcol;
      b_c = (n + 1) * b_kAcol;
      if (mmj < 1) {
        b_n = -1;
      } else {
        b_n = 0;
        if (mmj > 1) {
          ix = b_c;
          smax = std::abs(b_A->data[b_c]);
          for (iy = 2; iy <= mmj; iy++) {
            ix++;
            s = std::abs(b_A->data[ix]);
            if (s > smax) {
              b_n = iy - 1;
              smax = s;
            }
          }
        }
      }

      if (b_A->data[b_c + b_n] != 0.0) {
        if (b_n != 0) {
          iy = b_kAcol + b_n;
          ipiv->data[b_kAcol] = iy + 1;
          ix = b_kAcol;
          for (b_n = 0; b_n < n; b_n++) {
            smax = b_A->data[ix];
            b_A->data[ix] = b_A->data[iy];
            b_A->data[iy] = smax;
            ix += n;
            iy += n;
          }
        }

        iy = b_c + mmj;
        for (b_n = b_c + 1; b_n < iy; b_n++) {
          b_A->data[b_n] /= b_A->data[b_c];
        }
      }

      ix = b_c + n;
      b_n = ix + 1;
      for (iy = 0; iy <= mmj - 2; iy++) {
        smax = b_A->data[ix];
        if (b_A->data[ix] != 0.0) {
          c_ix = b_c + 1;
          e = mmj + b_n;
          for (ijA = b_n; ijA < e - 1; ijA++) {
            b_A->data[ijA] += b_A->data[c_ix] * -smax;
            c_ix++;
          }
        }

        ix += n;
        b_n += n;
      }
    }
  }

  for (yk = 0; yk <= n - 2; yk++) {
    if (yk + 1 != ipiv->data[yk]) {
      smax = B_9->data[yk];
      B_9->data[yk] = B_9->data[ipiv->data[yk] - 1];
      B_9->data[ipiv->data[yk] - 1] = smax;
    }
  }

  emxFree_int32_T(&ipiv);
  for (yk = 0; yk < n; yk++) {
    b_kAcol = n * yk;
    if (B_9->data[yk] != 0.0) {
      for (mmj = yk + 1; mmj < n; mmj++) {
        B_9->data[mmj] -= b_A->data[mmj + b_kAcol] * B_9->data[yk];
      }
    }
  }

  for (yk = A->size[1] - 1; yk + 1 > 0; yk--) {
    b_kAcol = n * yk;
    if (B_9->data[yk] != 0.0) {
      B_9->data[yk] /= b_A->data[yk + b_kAcol];
      for (mmj = 0; mmj < yk; mmj++) {
        B_9->data[mmj] -= b_A->data[mmj + b_kAcol] * B_9->data[yk];
      }
    }
  }

  emxFree_real_T(&b_A);
}

// Function for MATLAB Function: '<S104>/trajFromWaypoints'
void MatlabControllerClass::mldivide_i(const emxArray_real_T *A, const
  emxArray_real_T *B_8, emxArray_real_T *Y)
{
  emxArray_real_T *b_A;
  emxArray_real_T *tau;
  emxArray_int32_T *jpvt;
  int32_T rankR;
  int32_T minmn;
  int32_T maxmn;
  emxArray_real_T *b_B;
  real_T wj;
  int32_T c_i;
  uint32_T b_idx_0;
  int32_T u1;
  boolean_T exitg1;
  emxInit_real_T(&b_A, 2);
  emxInit_real_T(&tau, 1);
  emxInit_int32_T(&jpvt, 2);
  emxInit_real_T(&b_B, 1);
  if ((A->size[0] == 0) || (A->size[1] == 0) || (B_8->size[0] == 0)) {
    b_idx_0 = (uint32_T)A->size[1];
    minmn = Y->size[0];
    Y->size[0] = (int32_T)b_idx_0;
    emxEnsureCapacity_real_T(Y, minmn);
    maxmn = (int32_T)b_idx_0;
    for (minmn = 0; minmn < maxmn; minmn++) {
      Y->data[minmn] = 0.0;
    }
  } else if (A->size[0] == A->size[1]) {
    minmn = Y->size[0];
    Y->size[0] = B_8->size[0];
    emxEnsureCapacity_real_T(Y, minmn);
    maxmn = B_8->size[0];
    for (minmn = 0; minmn < maxmn; minmn++) {
      Y->data[minmn] = B_8->data[minmn];
    }

    lusolve(A, Y);
  } else {
    minmn = b_A->size[0] * b_A->size[1];
    b_A->size[0] = A->size[0];
    b_A->size[1] = A->size[1];
    emxEnsureCapacity_real_T(b_A, minmn);
    maxmn = A->size[0] * A->size[1] - 1;
    for (minmn = 0; minmn <= maxmn; minmn++) {
      b_A->data[minmn] = A->data[minmn];
    }

    xgeqp3(b_A, tau, jpvt);
    rankR = 0;
    if (b_A->size[0] < b_A->size[1]) {
      minmn = b_A->size[0];
      maxmn = b_A->size[1];
    } else {
      minmn = b_A->size[1];
      maxmn = b_A->size[0];
    }

    if (minmn > 0) {
      exitg1 = false;
      while ((!exitg1) && (rankR < minmn)) {
        wj = 2.2204460492503131E-15 * (real_T)maxmn;
        if (1.4901161193847656E-8 < wj) {
          wj = 1.4901161193847656E-8;
        }

        if (!(std::abs(b_A->data[b_A->size[0] * rankR + rankR]) <= wj * std::abs
              (b_A->data[0]))) {
          rankR++;
        } else {
          exitg1 = true;
        }
      }
    }

    maxmn = b_A->size[1];
    minmn = Y->size[0];
    Y->size[0] = maxmn;
    emxEnsureCapacity_real_T(Y, minmn);
    for (minmn = 0; minmn < maxmn; minmn++) {
      Y->data[minmn] = 0.0;
    }

    minmn = b_B->size[0];
    b_B->size[0] = B_8->size[0];
    emxEnsureCapacity_real_T(b_B, minmn);
    maxmn = B_8->size[0];
    for (minmn = 0; minmn < maxmn; minmn++) {
      b_B->data[minmn] = B_8->data[minmn];
    }

    minmn = b_A->size[0];
    maxmn = b_A->size[0];
    u1 = b_A->size[1];
    if (maxmn < u1) {
      u1 = maxmn;
    }

    maxmn = u1 - 1;
    for (u1 = 0; u1 <= maxmn; u1++) {
      if (tau->data[u1] != 0.0) {
        wj = b_B->data[u1];
        for (c_i = u1 + 1; c_i < minmn; c_i++) {
          wj += b_A->data[b_A->size[0] * u1 + c_i] * b_B->data[c_i];
        }

        wj *= tau->data[u1];
        if (wj != 0.0) {
          b_B->data[u1] -= wj;
          for (c_i = u1 + 1; c_i < minmn; c_i++) {
            b_B->data[c_i] -= b_A->data[b_A->size[0] * u1 + c_i] * wj;
          }
        }
      }
    }

    for (maxmn = 0; maxmn < rankR; maxmn++) {
      Y->data[jpvt->data[maxmn] - 1] = b_B->data[maxmn];
    }

    for (rankR--; rankR + 1 > 0; rankR--) {
      Y->data[jpvt->data[rankR] - 1] /= b_A->data[b_A->size[0] * rankR + rankR];
      for (u1 = 0; u1 < rankR; u1++) {
        Y->data[jpvt->data[u1] - 1] -= b_A->data[b_A->size[0] * rankR + u1] *
          Y->data[jpvt->data[rankR] - 1];
      }
    }
  }

  emxFree_real_T(&b_B);
  emxFree_int32_T(&jpvt);
  emxFree_real_T(&tau);
  emxFree_real_T(&b_A);
}

// Function for MATLAB Function: '<S104>/trajFromWaypoints'
void MatlabControllerClass::polyder_c(const emxArray_real_T *u, emxArray_real_T *
  a)
{
  int32_T nymax;
  int32_T nlead0;
  int32_T ny;
  int32_T tmp;
  if (u->size[1] < 2) {
    nymax = 1;
  } else {
    nymax = u->size[1] - 1;
  }

  tmp = a->size[0] * a->size[1];
  a->size[0] = 1;
  a->size[1] = nymax;
  emxEnsureCapacity_real_T(a, tmp);
  switch (u->size[1]) {
   case 0:
    a->data[0] = 0.0;
    break;

   case 1:
    a->data[0] = 0.0;
    break;

   default:
    nlead0 = 0;
    ny = 0;
    while ((ny <= nymax - 2) && (u->data[ny] == 0.0)) {
      nlead0++;
      ny++;
    }

    ny = nymax - nlead0;
    tmp = a->size[0] * a->size[1];
    a->size[0] = 1;
    a->size[1] = ny;
    emxEnsureCapacity_real_T(a, tmp);
    for (nymax = 0; nymax < ny; nymax++) {
      a->data[nymax] = u->data[nymax + nlead0];
    }
    break;
  }

  nlead0 = a->size[1] - 2;
  for (ny = 0; ny <= nlead0; ny++) {
    a->data[ny] *= (real_T)((nlead0 - ny) + 1) + 1.0;
  }

  if ((u->size[1] != 0) && (rtIsInf(u->data[u->size[1] - 1]) || rtIsNaN(u->
        data[u->size[1] - 1]))) {
    a->data[a->size[1] - 1] = (rtNaN);
  }
}

// Function for MATLAB Function: '<S104>/trajFromWaypoints'
void MatlabControllerClass::polyInterpolation(const real32_T points[6], real_T
  degree, boolean_T cycle, emxArray_real_T *coeffs, real_T *num_of_splines)
{
  real32_T points_new_data[7];
  emxArray_real_T *pp;
  emxArray_real_T *point_0;
  real_T intermediate_size;
  real_T size_A_mat;
  emxArray_real_T *A;
  emxArray_real_T *b;
  real_T bnd_left;
  real_T bnd_right;
  emxArray_real_T *x;
  int32_T c;
  int32_T d;
  int32_T fb;
  int32_T gb;
  emxArray_real_T *pp_0;
  emxArray_real_T *tmp;
  int32_T loop_ub;
  real32_T points_0[7];
  int32_T i;
  int32_T i_0;
  int32_T i_1;
  int32_T points_new_size_idx_1;
  int32_T degree_idx_1;
  real_T intermediate_size_tmp;
  int32_T intermediate_size_tmp_0;
  int32_T intermediate_size_tmp_1;
  int32_T exitg1;
  if (cycle) {
    for (i = 0; i < 6; i++) {
      points_0[i] = points[i];
    }

    points_0[6] = points[0];
    points_new_size_idx_1 = 7;
    for (i = 0; i < 7; i++) {
      points_new_data[i] = points_0[i];
    }
  } else {
    points_new_size_idx_1 = 6;
    for (i = 0; i < 6; i++) {
      points_new_data[i] = points[i];
    }
  }

  emxInit_real_T(&pp, 2);
  i = pp->size[0] * pp->size[1];
  degree_idx_1 = (int32_T)(degree + 1.0);
  pp->size[0] = degree_idx_1;
  pp->size[1] = degree_idx_1;
  emxEnsureCapacity_real_T(pp, i);
  loop_ub = degree_idx_1 * degree_idx_1 - 1;
  for (i = 0; i <= loop_ub; i++) {
    pp->data[i] = 1.0;
  }

  if (2 > degree_idx_1) {
    d = 1;
    c = 0;
  } else {
    d = 2;
    c = degree_idx_1;
  }

  loop_ub = (c - d) + 1;
  i = d - 1;
  for (i_1 = 0; i_1 < degree_idx_1; i_1++) {
    for (i_0 = 0; i_0 < loop_ub; i_0++) {
      pp->data[(i + i_0) + pp->size[0] * i_1] = 0.0;
    }
  }

  d = 0;
  emxInit_real_T(&pp_0, 2);
  emxInit_real_T(&tmp, 2);
  do {
    exitg1 = 0;
    i = (int32_T)degree;
    i_1 = i - 1;
    if (d <= i_1) {
      bnd_left = ((real_T)pp->size[1] - (1.0 + (real_T)d)) + 1.0;
      if (1.0 > bnd_left) {
        loop_ub = 0;
      } else {
        loop_ub = (int32_T)bnd_left;
      }

      i = pp_0->size[0] * pp_0->size[1];
      pp_0->size[0] = 1;
      pp_0->size[1] = loop_ub;
      emxEnsureCapacity_real_T(pp_0, i);
      for (i = 0; i < loop_ub; i++) {
        pp_0->data[i] = pp->data[pp->size[0] * i + d];
      }

      polyder_c(pp_0, tmp);
      loop_ub = tmp->size[1];
      for (i = 0; i < loop_ub; i++) {
        pp->data[(d + pp->size[0] * i) + 1] = tmp->data[i];
      }

      d++;
    } else {
      exitg1 = 1;
    }
  } while (exitg1 == 0);

  emxFree_real_T(&tmp);
  emxFree_real_T(&pp_0);
  emxInit_real_T(&point_0, 2);
  i_0 = point_0->size[0] * point_0->size[1];
  point_0->size[0] = pp->size[0];
  point_0->size[1] = pp->size[1];
  emxEnsureCapacity_real_T(point_0, i_0);
  loop_ub = pp->size[0] * pp->size[1] - 1;
  for (i_0 = 0; i_0 <= loop_ub; i_0++) {
    point_0->data[i_0] = pp->data[i_0];
  }

  for (d = 0; d <= i_1; d++) {
    bnd_left = (real_T)point_0->size[1] - (1.0 + (real_T)d);
    if (1.0 > bnd_left) {
      c = 0;
    } else {
      c = (int32_T)bnd_left;
    }

    for (i_0 = 0; i_0 < c; i_0++) {
      point_0->data[d + point_0->size[0] * i_0] = 0.0;
    }
  }

  emxInit_real_T(&A, 2);
  intermediate_size_tmp = (((real_T)points_new_size_idx_1 - 1.0) - 1.0) *
    (degree + 1.0);
  size_A_mat = (degree + 1.0) * ((real_T)points_new_size_idx_1 - 1.0);
  i_1 = A->size[0] * A->size[1];
  loop_ub = (int32_T)size_A_mat;
  A->size[0] = loop_ub;
  A->size[1] = loop_ub;
  emxEnsureCapacity_real_T(A, i_1);
  degree_idx_1 = loop_ub * loop_ub - 1;
  for (i_1 = 0; i_1 <= degree_idx_1; i_1++) {
    A->data[i_1] = 0.0;
  }

  emxInit_real_T(&b, 1);
  i_1 = b->size[0];
  b->size[0] = loop_ub;
  emxEnsureCapacity_real_T(b, i_1);
  for (i_1 = 0; i_1 < loop_ub; i_1++) {
    b->data[i_1] = 0.0;
  }

  if (!cycle) {
    bnd_right = (degree + 1.0) / 2.0;
    bnd_left = std::ceil(bnd_right);
    bnd_right = std::floor(bnd_right);
    if (1.0 > bnd_left) {
      i_1 = 0;
    } else {
      i_1 = (int32_T)bnd_left;
    }

    loop_ub = i_1 - 1;
    degree_idx_1 = point_0->size[1] - 1;
    for (i_1 = 0; i_1 <= degree_idx_1; i_1++) {
      for (i_0 = 0; i_0 <= loop_ub; i_0++) {
        A->data[i_0 + A->size[0] * i_1] = point_0->data[point_0->size[0] * i_1 +
          i_0];
      }
    }

    intermediate_size = (bnd_left + 1.0) + intermediate_size_tmp;
    if (intermediate_size > (intermediate_size + bnd_right) - 1.0) {
      d = 0;
    } else {
      d = (int32_T)intermediate_size - 1;
    }

    if (1.0 > bnd_right) {
      i_1 = 0;
    } else {
      i_1 = (int32_T)bnd_right;
    }

    loop_ub = i_1 - 1;
    degree_idx_1 = pp->size[1] - 1;
    for (i_1 = 0; i_1 <= degree_idx_1; i_1++) {
      for (i_0 = 0; i_0 <= loop_ub; i_0++) {
        A->data[(d + i_0) + A->size[0] * ((int32_T)((1.0 + intermediate_size_tmp)
          + (real_T)i_1) - 1)] = pp->data[pp->size[0] * i_1 + i_0];
      }
    }

    b->data[0] = points_new_data[0];
    b->data[(int32_T)(((real_T)(int32_T)size_A_mat - bnd_right) + 1.0) - 1] =
      points_new_data[points_new_size_idx_1 - 1];
    if (degree > 1.0) {
      b->data[1] = points_new_data[1] - points_new_data[0];
    }

    if (degree > 2.0) {
      b->data[(int32_T)(((real_T)b->size[0] - bnd_right) + 2.0) - 1] =
        points_new_data[points_new_size_idx_1 - 1] -
        points_new_data[points_new_size_idx_1 - 2];
    }
  } else {
    bnd_left = 2.0;
    loop_ub = point_0->size[1] - 1;
    for (i_1 = 0; i_1 <= loop_ub; i_1++) {
      A->data[A->size[0] * i_1] = point_0->data[point_0->size[0] * i_1];
    }

    loop_ub = pp->size[1] - 1;
    for (i_1 = 0; i_1 <= loop_ub; i_1++) {
      A->data[1 + A->size[0] * ((int32_T)((intermediate_size_tmp + 1.0) +
        (real_T)i_1) - 1)] = pp->data[pp->size[0] * i_1];
    }

    b->data[0] = points_new_data[0];
    b->data[1] = points_new_data[points_new_size_idx_1 - 1];
    if (2.0 > degree) {
      d = 0;
      c = 0;
    } else {
      d = 1;
      c = i;
    }

    bnd_right = (((intermediate_size_tmp + 2.0) + 1.0) + degree) - 2.0;
    if ((intermediate_size_tmp + 2.0) + 1.0 > bnd_right) {
      gb = 0;
    } else {
      gb = (int32_T)((intermediate_size_tmp + 2.0) + 1.0) - 1;
    }

    loop_ub = point_0->size[1] - 1;
    for (i_1 = 0; i_1 <= loop_ub; i_1++) {
      degree_idx_1 = c - d;
      for (i_0 = 0; i_0 < degree_idx_1; i_0++) {
        A->data[(gb + i_0) + A->size[0] * i_1] = point_0->data[(d + i_0) +
          point_0->size[0] * i_1];
      }
    }

    if (2.0 > degree) {
      d = 0;
      c = 0;
    } else {
      d = 1;
      c = i;
    }

    if ((intermediate_size_tmp + 2.0) + 1.0 > bnd_right) {
      gb = 0;
    } else {
      gb = (int32_T)((intermediate_size_tmp + 2.0) + 1.0) - 1;
    }

    loop_ub = pp->size[1] - 1;
    for (i_1 = 0; i_1 <= loop_ub; i_1++) {
      degree_idx_1 = c - d;
      for (i_0 = 0; i_0 < degree_idx_1; i_0++) {
        A->data[(gb + i_0) + A->size[0] * ((int32_T)((intermediate_size_tmp +
          1.0) + (real_T)i_1) - 1)] = -pp->data[(d + i_0) + pp->size[0] * i_1];
      }
    }
  }

  for (d = 0; d <= points_new_size_idx_1 - 3; d++) {
    intermediate_size_tmp = ((1.0 + (real_T)d) - 1.0) * (degree + 1.0);
    intermediate_size = intermediate_size_tmp + bnd_left;
    size_A_mat = (degree + 1.0) + (intermediate_size_tmp + 1.0);
    loop_ub = pp->size[1] - 1;
    intermediate_size_tmp_0 = (int32_T)(intermediate_size + 1.0);
    for (i_1 = 0; i_1 <= loop_ub; i_1++) {
      A->data[(intermediate_size_tmp_0 + A->size[0] * ((int32_T)
                ((intermediate_size_tmp + 1.0) + (real_T)i_1) - 1)) - 1] =
        pp->data[pp->size[0] * i_1];
    }

    loop_ub = point_0->size[1] - 1;
    intermediate_size_tmp_1 = (int32_T)(intermediate_size + 2.0);
    for (i_1 = 0; i_1 <= loop_ub; i_1++) {
      A->data[(intermediate_size_tmp_1 + A->size[0] * ((int32_T)(size_A_mat +
                 (real_T)i_1) - 1)) - 1] = point_0->data[point_0->size[0] * i_1];
    }

    if (2.0 > degree) {
      c = 0;
      gb = 0;
    } else {
      c = 1;
      gb = i;
    }

    bnd_right = ((intermediate_size + 3.0) + degree) - 2.0;
    if (intermediate_size + 3.0 > bnd_right) {
      fb = 0;
    } else {
      fb = (int32_T)(intermediate_size + 3.0) - 1;
    }

    loop_ub = pp->size[1] - 1;
    for (i_1 = 0; i_1 <= loop_ub; i_1++) {
      degree_idx_1 = gb - c;
      for (i_0 = 0; i_0 < degree_idx_1; i_0++) {
        A->data[(fb + i_0) + A->size[0] * ((int32_T)((intermediate_size_tmp +
          1.0) + (real_T)i_1) - 1)] = pp->data[(c + i_0) + pp->size[0] * i_1];
      }
    }

    if (2.0 > degree) {
      c = 0;
      gb = 0;
    } else {
      c = 1;
      gb = i;
    }

    if (intermediate_size + 3.0 > bnd_right) {
      fb = 0;
    } else {
      fb = (int32_T)(intermediate_size + 3.0) - 1;
    }

    loop_ub = point_0->size[1] - 1;
    for (i_1 = 0; i_1 <= loop_ub; i_1++) {
      degree_idx_1 = gb - c;
      for (i_0 = 0; i_0 < degree_idx_1; i_0++) {
        A->data[(fb + i_0) + A->size[0] * ((int32_T)(size_A_mat + (real_T)i_1) -
          1)] = -point_0->data[(c + i_0) + point_0->size[0] * i_1];
      }
    }

    bnd_right = points_new_data[d + 1];
    b->data[intermediate_size_tmp_0 - 1] = bnd_right;
    b->data[intermediate_size_tmp_1 - 1] = bnd_right;
  }

  emxFree_real_T(&point_0);
  emxFree_real_T(&pp);
  emxInit_real_T(&x, 1);
  mldivide_i(A, b, x);
  i = coeffs->size[0] * coeffs->size[1];
  coeffs->size[0] = 1;
  coeffs->size[1] = x->size[0];
  emxEnsureCapacity_real_T(coeffs, i);
  loop_ub = x->size[0];
  emxFree_real_T(&b);
  emxFree_real_T(&A);
  for (i = 0; i < loop_ub; i++) {
    coeffs->data[i] = x->data[i];
  }

  emxFree_real_T(&x);
  *num_of_splines = points_new_size_idx_1 - 1;
}

// Function for MATLAB Function: '<S91>/trajGetMatch'
boolean_T MatlabControllerClass::anyNonFinite(const creal_T x_data[], const
  int32_T x_size[2])
{
  int32_T nx;
  boolean_T c_p;
  int32_T k;
  nx = x_size[0] * x_size[1];
  c_p = true;
  for (k = 0; k < nx; k++) {
    if (c_p && (rtIsInf(x_data[k].re) || rtIsInf(x_data[k].im) || (rtIsNaN
          (x_data[k].re) || rtIsNaN(x_data[k].im)))) {
      c_p = false;
    }
  }

  return !c_p;
}

// Function for MATLAB Function: '<S91>/trajGetMatch'
boolean_T MatlabControllerClass::ishermitian(const creal_T A_data[], const
  int32_T A_size[2])
{
  boolean_T p;
  int32_T j;
  int32_T i;
  int32_T exitg1;
  boolean_T exitg2;
  p = (A_size[0] == A_size[1]);
  if (p) {
    j = 0;
    exitg2 = false;
    while ((!exitg2) && (j <= A_size[1] - 1)) {
      i = 0;
      do {
        exitg1 = 0;
        if (i <= j) {
          if ((!(A_data[A_size[0] * j + i].re == A_data[A_size[0] * i + j].re)) ||
              (!(A_data[A_size[0] * j + i].im == -A_data[A_size[0] * i + j].im)))
          {
            p = false;
            exitg1 = 1;
          } else {
            i++;
          }
        } else {
          j++;
          exitg1 = 2;
        }
      } while (exitg1 == 0);

      if (exitg1 == 1) {
        exitg2 = true;
      }
    }
  }

  return p;
}

// Function for MATLAB Function: '<S91>/trajGetMatch'
real_T MatlabControllerClass::xzlangeM(const creal_T x_data[], const int32_T
  x_size[2])
{
  real_T y;
  real_T absxk;
  int32_T b;
  int32_T k;
  boolean_T exitg1;
  y = 0.0;
  b = x_size[0] * x_size[1];
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k <= b - 1)) {
    absxk = rt_hypotd_snf(x_data[k].re, x_data[k].im);
    if (rtIsNaN(absxk)) {
      y = (rtNaN);
      exitg1 = true;
    } else {
      if (absxk > y) {
        y = absxk;
      }

      k++;
    }
  }

  return y;
}

// Function for MATLAB Function: '<S91>/MATLAB Function2'
boolean_T MatlabControllerClass::isfinite(real_T x)
{
  return (!rtIsInf(x)) && (!rtIsNaN(x));
}

// Function for MATLAB Function: '<S91>/trajGetMatch'
void MatlabControllerClass::xzlascl(real_T cfrom, real_T cto, creal_T A_data[],
  int32_T A_size[2])
{
  real_T cfromc;
  real_T ctoc;
  boolean_T notdone;
  real_T cfrom1;
  real_T cto1;
  real_T mul;
  int32_T i;
  int32_T i_0;
  int32_T loop_ub;
  int32_T loop_ub_0;
  int32_T tmp;
  int32_T tmp_0;
  cfromc = cfrom;
  ctoc = cto;
  notdone = true;
  while (notdone) {
    cfrom1 = cfromc * 2.0041683600089728E-292;
    cto1 = ctoc / 4.9896007738368E+291;
    if ((std::abs(cfrom1) > std::abs(ctoc)) && (ctoc != 0.0)) {
      mul = 2.0041683600089728E-292;
      cfromc = cfrom1;
    } else if (std::abs(cto1) > std::abs(cfromc)) {
      mul = 4.9896007738368E+291;
      ctoc = cto1;
    } else {
      mul = ctoc / cfromc;
      notdone = false;
    }

    loop_ub = A_size[1];
    for (i_0 = 0; i_0 < loop_ub; i_0++) {
      loop_ub_0 = A_size[0];
      for (i = 0; i < loop_ub_0; i++) {
        tmp = A_size[0] * i_0;
        tmp_0 = i + tmp;
        A_data[tmp_0].re = A_data[tmp + i].re * mul;
        A_data[tmp_0].im = A_data[A_size[0] * i_0 + i].im * mul;
      }
    }
  }
}

// Function for MATLAB Function: '<S91>/trajGetMatch'
void MatlabControllerClass::xzggbal(creal_T A_data[], int32_T A_size[2], int32_T
  *ilo, int32_T *ihi, int32_T rscale_data[], int32_T *rscale_size)
{
  int32_T i;
  int32_T j;
  boolean_T found;
  int32_T ii;
  int32_T nzcount;
  int32_T jj;
  real_T atmp_re;
  real_T atmp_im;
  int32_T atmp_re_tmp;
  int32_T exitg1;
  int32_T exitg2;
  boolean_T exitg3;
  boolean_T exitg4;
  *rscale_size = A_size[0];
  ii = A_size[0];
  for (atmp_re_tmp = 0; atmp_re_tmp < ii; atmp_re_tmp++) {
    rscale_data[atmp_re_tmp] = 1;
  }

  *ilo = 1;
  *ihi = A_size[0];
  if (A_size[0] <= 1) {
    *ihi = 1;
  } else {
    do {
      exitg2 = 0;
      i = 0;
      j = -1;
      found = false;
      ii = *ihi;
      exitg3 = false;
      while ((!exitg3) && (ii > 0)) {
        nzcount = 0;
        i = ii;
        j = *ihi - 1;
        jj = 0;
        exitg4 = false;
        while ((!exitg4) && (jj <= *ihi - 1)) {
          if ((A_data[(A_size[0] * jj + ii) - 1].re != 0.0) || (A_data[(A_size[0]
                * jj + ii) - 1].im != 0.0) || (jj + 1 == ii)) {
            if (nzcount == 0) {
              j = jj;
              nzcount = 1;
              jj++;
            } else {
              nzcount = 2;
              exitg4 = true;
            }
          } else {
            jj++;
          }
        }

        if (nzcount < 2) {
          found = true;
          exitg3 = true;
        } else {
          ii--;
        }
      }

      if (!found) {
        exitg2 = 2;
      } else {
        nzcount = A_size[0];
        jj = A_size[1];
        ii = A_size[0] * A_size[1] - 1;
        if (0 <= ii) {
          memcpy(&rtDW.b_A_data_m[0], &A_data[0], (ii + 1) * sizeof(creal_T));
        }

        if (i != *ihi) {
          for (ii = 0; ii < A_size[0]; ii++) {
            atmp_re_tmp = nzcount * ii;
            atmp_re = rtDW.b_A_data_m[(atmp_re_tmp + i) - 1].re;
            atmp_im = rtDW.b_A_data_m[(nzcount * ii + i) - 1].im;
            rtDW.b_A_data_m[(i + atmp_re_tmp) - 1] = rtDW.b_A_data_m
              [(atmp_re_tmp + *ihi) - 1];
            atmp_re_tmp = (*ihi + atmp_re_tmp) - 1;
            rtDW.b_A_data_m[atmp_re_tmp].re = atmp_re;
            rtDW.b_A_data_m[atmp_re_tmp].im = atmp_im;
          }
        }

        if (j + 1 != *ihi) {
          for (i = 0; i < *ihi; i++) {
            atmp_re_tmp = nzcount * j;
            atmp_re = rtDW.b_A_data_m[atmp_re_tmp + i].re;
            atmp_im = rtDW.b_A_data_m[nzcount * j + i].im;
            rtDW.b_A_data_m[i + atmp_re_tmp] = rtDW.b_A_data_m[(*ihi - 1) *
              nzcount + i];
            atmp_re_tmp = i + nzcount * (*ihi - 1);
            rtDW.b_A_data_m[atmp_re_tmp].re = atmp_re;
            rtDW.b_A_data_m[atmp_re_tmp].im = atmp_im;
          }
        }

        for (atmp_re_tmp = 0; atmp_re_tmp < jj; atmp_re_tmp++) {
          for (i = 0; i < nzcount; i++) {
            A_data[i + A_size[0] * atmp_re_tmp] = rtDW.b_A_data_m[nzcount *
              atmp_re_tmp + i];
          }
        }

        rscale_data[*ihi - 1] = j + 1;
        (*ihi)--;
        if (*ihi == 1) {
          rscale_data[0] = 1;
          exitg2 = 1;
        }
      }
    } while (exitg2 == 0);

    if (exitg2 == 1) {
    } else {
      do {
        exitg1 = 0;
        i = 0;
        j = 0;
        found = false;
        ii = *ilo;
        exitg3 = false;
        while ((!exitg3) && (ii <= *ihi)) {
          nzcount = 0;
          i = *ihi;
          j = ii;
          jj = *ilo;
          exitg4 = false;
          while ((!exitg4) && (jj <= *ihi)) {
            if ((A_data[((ii - 1) * A_size[0] + jj) - 1].re != 0.0) || (A_data
                 [((ii - 1) * A_size[0] + jj) - 1].im != 0.0) || (jj == ii)) {
              if (nzcount == 0) {
                i = jj;
                nzcount = 1;
                jj++;
              } else {
                nzcount = 2;
                exitg4 = true;
              }
            } else {
              jj++;
            }
          }

          if (nzcount < 2) {
            found = true;
            exitg3 = true;
          } else {
            ii++;
          }
        }

        if (!found) {
          exitg1 = 1;
        } else {
          nzcount = A_size[0];
          jj = A_size[1];
          ii = A_size[0] * A_size[1] - 1;
          if (0 <= ii) {
            memcpy(&rtDW.b_A_data_m[0], &A_data[0], (ii + 1) * sizeof(creal_T));
          }

          if (i != *ilo) {
            for (ii = *ilo - 1; ii < A_size[0]; ii++) {
              atmp_re_tmp = nzcount * ii;
              atmp_re = rtDW.b_A_data_m[(atmp_re_tmp + i) - 1].re;
              atmp_im = rtDW.b_A_data_m[(nzcount * ii + i) - 1].im;
              rtDW.b_A_data_m[(i + atmp_re_tmp) - 1] = rtDW.b_A_data_m
                [(atmp_re_tmp + *ilo) - 1];
              atmp_re_tmp = (*ilo + atmp_re_tmp) - 1;
              rtDW.b_A_data_m[atmp_re_tmp].re = atmp_re;
              rtDW.b_A_data_m[atmp_re_tmp].im = atmp_im;
            }
          }

          if (j != *ilo) {
            for (i = 0; i < *ihi; i++) {
              atmp_re = rtDW.b_A_data_m[(j - 1) * nzcount + i].re;
              atmp_im = rtDW.b_A_data_m[(j - 1) * nzcount + i].im;
              rtDW.b_A_data_m[i + nzcount * (j - 1)] = rtDW.b_A_data_m[(*ilo - 1)
                * nzcount + i];
              atmp_re_tmp = i + nzcount * (*ilo - 1);
              rtDW.b_A_data_m[atmp_re_tmp].re = atmp_re;
              rtDW.b_A_data_m[atmp_re_tmp].im = atmp_im;
            }
          }

          for (atmp_re_tmp = 0; atmp_re_tmp < jj; atmp_re_tmp++) {
            for (i = 0; i < nzcount; i++) {
              A_data[i + A_size[0] * atmp_re_tmp] = rtDW.b_A_data_m[nzcount *
                atmp_re_tmp + i];
            }
          }

          rscale_data[*ilo - 1] = j;
          (*ilo)++;
          if (*ilo == *ihi) {
            rscale_data[*ilo - 1] = *ilo;
            exitg1 = 1;
          }
        }
      } while (exitg1 == 0);
    }
  }
}

// Function for MATLAB Function: '<S91>/trajGetMatch'
void MatlabControllerClass::xzlartg(const creal_T f, const creal_T g, real_T *cs,
  creal_T *sn, creal_T *r)
{
  real_T scale;
  int32_T count;
  int32_T rescaledir;
  real_T g2;
  real_T f2s;
  real_T di;
  real_T fs_re;
  real_T fs_im;
  real_T gs_re;
  real_T gs_im;
  real_T g2_tmp;
  boolean_T guard1 = false;
  di = std::abs(f.re);
  scale = di;
  g2_tmp = std::abs(f.im);
  if (g2_tmp > di) {
    scale = g2_tmp;
  }

  g2 = std::abs(g.re);
  f2s = std::abs(g.im);
  if (f2s > g2) {
    g2 = f2s;
  }

  if (g2 > scale) {
    scale = g2;
  }

  fs_re = f.re;
  fs_im = f.im;
  gs_re = g.re;
  gs_im = g.im;
  count = -1;
  rescaledir = 0;
  guard1 = false;
  if (scale >= 7.4428285367870146E+137) {
    do {
      count++;
      fs_re *= 1.3435752215134178E-138;
      fs_im *= 1.3435752215134178E-138;
      gs_re *= 1.3435752215134178E-138;
      gs_im *= 1.3435752215134178E-138;
      scale *= 1.3435752215134178E-138;
    } while (!(scale < 7.4428285367870146E+137));

    rescaledir = 1;
    guard1 = true;
  } else if (scale <= 1.3435752215134178E-138) {
    if ((g.re == 0.0) && (g.im == 0.0)) {
      *cs = 1.0;
      sn->re = 0.0;
      sn->im = 0.0;
      *r = f;
    } else {
      do {
        count++;
        fs_re *= 7.4428285367870146E+137;
        fs_im *= 7.4428285367870146E+137;
        gs_re *= 7.4428285367870146E+137;
        gs_im *= 7.4428285367870146E+137;
        scale *= 7.4428285367870146E+137;
      } while (!(scale > 1.3435752215134178E-138));

      rescaledir = -1;
      guard1 = true;
    }
  } else {
    guard1 = true;
  }

  if (guard1) {
    scale = fs_re * fs_re + fs_im * fs_im;
    g2 = gs_re * gs_re + gs_im * gs_im;
    f2s = g2;
    if (1.0 > g2) {
      f2s = 1.0;
    }

    if (scale <= f2s * 2.0041683600089728E-292) {
      if ((f.re == 0.0) && (f.im == 0.0)) {
        *cs = 0.0;
        r->re = rt_hypotd_snf(g.re, g.im);
        r->im = 0.0;
        g2 = rt_hypotd_snf(gs_re, gs_im);
        sn->re = gs_re / g2;
        sn->im = -gs_im / g2;
      } else {
        scale = std::sqrt(g2);
        *cs = rt_hypotd_snf(fs_re, fs_im) / scale;
        g2 = di;
        if (g2_tmp > di) {
          g2 = g2_tmp;
        }

        if (g2 > 1.0) {
          g2 = rt_hypotd_snf(f.re, f.im);
          fs_re = f.re / g2;
          fs_im = f.im / g2;
        } else {
          f2s = 7.4428285367870146E+137 * f.re;
          di = 7.4428285367870146E+137 * f.im;
          g2 = rt_hypotd_snf(f2s, di);
          fs_re = f2s / g2;
          fs_im = di / g2;
        }

        gs_re /= scale;
        gs_im = -gs_im / scale;
        sn->re = fs_re * gs_re - fs_im * gs_im;
        sn->im = fs_re * gs_im + fs_im * gs_re;
        r->re = (sn->re * g.re - sn->im * g.im) + *cs * f.re;
        r->im = (sn->re * g.im + sn->im * g.re) + *cs * f.im;
      }
    } else {
      f2s = std::sqrt(g2 / scale + 1.0);
      r->re = f2s * fs_re;
      r->im = f2s * fs_im;
      *cs = 1.0 / f2s;
      g2 += scale;
      scale = r->re / g2;
      g2 = r->im / g2;
      sn->re = scale * gs_re - g2 * -gs_im;
      sn->im = scale * -gs_im + g2 * gs_re;
      if (rescaledir > 0) {
        for (rescaledir = 0; rescaledir <= count; rescaledir++) {
          r->re *= 7.4428285367870146E+137;
          r->im *= 7.4428285367870146E+137;
        }
      } else {
        if (rescaledir < 0) {
          for (rescaledir = 0; rescaledir <= count; rescaledir++) {
            r->re *= 1.3435752215134178E-138;
            r->im *= 1.3435752215134178E-138;
          }
        }
      }
    }
  }
}

// Function for MATLAB Function: '<S91>/trajGetMatch'
void MatlabControllerClass::xzgghrd(int32_T ilo, int32_T ihi, creal_T A_data[],
  int32_T A_size[2])
{
  int32_T jrow;
  real_T c;
  creal_T s;
  int32_T jcol;
  int32_T j;
  real_T tmp;
  real_T tmp_0;
  real_T stemp_re;
  real_T stemp_im;
  int32_T stemp_re_tmp;
  int32_T stemp_re_tmp_tmp;
  if ((A_size[0] > 1) && (ihi >= ilo + 2)) {
    for (jcol = ilo - 1; jcol + 1 < ihi - 1; jcol++) {
      for (jrow = ihi - 1; jrow + 1 > jcol + 2; jrow--) {
        xzlartg(A_data[(jrow + A_size[0] * jcol) - 1], A_data[jrow + A_size[0] *
                jcol], &c, &s, &A_data[(jrow + A_size[0] * jcol) - 1]);
        stemp_re_tmp_tmp = jrow + A_size[0] * jcol;
        A_data[stemp_re_tmp_tmp].re = 0.0;
        A_data[stemp_re_tmp_tmp].im = 0.0;
        for (j = jcol + 1; j < A_size[0]; j++) {
          stemp_re_tmp_tmp = A_size[0] * j;
          stemp_re_tmp = stemp_re_tmp_tmp + jrow;
          stemp_re = A_data[stemp_re_tmp - 1].re * c + (A_data[stemp_re_tmp].re *
            s.re - A_data[A_size[0] * j + jrow].im * s.im);
          stemp_im = A_data[(A_size[0] * j + jrow) - 1].im * c + (A_data[A_size
            [0] * j + jrow].im * s.re + A_data[A_size[0] * j + jrow].re * s.im);
          tmp = A_data[(A_size[0] * j + jrow) - 1].im;
          tmp_0 = A_data[(A_size[0] * j + jrow) - 1].re;
          stemp_re_tmp_tmp += jrow;
          A_data[stemp_re_tmp_tmp].re = A_data[A_size[0] * j + jrow].re * c -
            (A_data[(A_size[0] * j + jrow) - 1].re * s.re + A_data[(A_size[0] *
              j + jrow) - 1].im * s.im);
          A_data[stemp_re_tmp_tmp].im = A_data[A_size[0] * j + jrow].im * c -
            (s.re * tmp - s.im * tmp_0);
          stemp_re_tmp_tmp--;
          A_data[stemp_re_tmp_tmp].re = stemp_re;
          A_data[stemp_re_tmp_tmp].im = stemp_im;
        }

        s.re = -s.re;
        s.im = -s.im;
        for (j = 0; j < ihi; j++) {
          stemp_re_tmp = A_size[0] * jrow;
          stemp_re = (A_data[(jrow - 1) * A_size[0] + j].re * s.re - A_data
                      [(jrow - 1) * A_size[0] + j].im * s.im) +
            A_data[stemp_re_tmp + j].re * c;
          stemp_im = (A_data[(jrow - 1) * A_size[0] + j].im * s.re + A_data
                      [(jrow - 1) * A_size[0] + j].re * s.im) + A_data[A_size[0]
            * jrow + j].im * c;
          tmp = A_data[A_size[0] * jrow + j].im;
          tmp_0 = A_data[A_size[0] * jrow + j].re;
          stemp_re_tmp_tmp = j + A_size[0] * (jrow - 1);
          A_data[stemp_re_tmp_tmp].re = A_data[(jrow - 1) * A_size[0] + j].re *
            c - (A_data[A_size[0] * jrow + j].re * s.re + A_data[A_size[0] *
                 jrow + j].im * s.im);
          A_data[stemp_re_tmp_tmp].im = A_data[(jrow - 1) * A_size[0] + j].im *
            c - (s.re * tmp - s.im * tmp_0);
          stemp_re_tmp_tmp = j + stemp_re_tmp;
          A_data[stemp_re_tmp_tmp].re = stemp_re;
          A_data[stemp_re_tmp_tmp].im = stemp_im;
        }
      }
    }
  }
}

// Function for MATLAB Function: '<S91>/trajGetMatch'
real_T MatlabControllerClass::xzlanhs(const creal_T A_data[], const int32_T
  A_size[2], int32_T ilo, int32_T ihi)
{
  real_T f;
  real_T scale;
  real_T sumsq;
  boolean_T firstNonZero;
  real_T reAij;
  real_T imAij;
  real_T temp2;
  int32_T j;
  int32_T b;
  int32_T i;
  f = 0.0;
  if (ilo <= ihi) {
    scale = 0.0;
    sumsq = 0.0;
    firstNonZero = true;
    for (j = ilo; j <= ihi; j++) {
      b = j + 1;
      if (ihi < j + 1) {
        b = ihi;
      }

      for (i = ilo; i <= b; i++) {
        reAij = A_data[((j - 1) * A_size[0] + i) - 1].re;
        imAij = A_data[((j - 1) * A_size[0] + i) - 1].im;
        if (reAij != 0.0) {
          reAij = std::abs(reAij);
          if (firstNonZero) {
            sumsq = 1.0;
            scale = reAij;
            firstNonZero = false;
          } else if (scale < reAij) {
            temp2 = scale / reAij;
            sumsq = sumsq * temp2 * temp2 + 1.0;
            scale = reAij;
          } else {
            temp2 = reAij / scale;
            sumsq += temp2 * temp2;
          }
        }

        if (imAij != 0.0) {
          reAij = std::abs(imAij);
          if (firstNonZero) {
            sumsq = 1.0;
            scale = reAij;
            firstNonZero = false;
          } else if (scale < reAij) {
            temp2 = scale / reAij;
            sumsq = sumsq * temp2 * temp2 + 1.0;
            scale = reAij;
          } else {
            temp2 = reAij / scale;
            sumsq += temp2 * temp2;
          }
        }
      }
    }

    f = scale * std::sqrt(sumsq);
  }

  return f;
}

// Function for MATLAB Function: '<S91>/trajGetMatch'
int32_T MatlabControllerClass::mod(int32_T x)
{
  return x - x / 10 * 10;
}

// Function for MATLAB Function: '<S91>/trajGetMatch'
void MatlabControllerClass::sqrt_j(creal_T *x)
{
  real_T xr;
  real_T absxr;
  xr = x->re;
  if (x->im == 0.0) {
    if (x->re < 0.0) {
      absxr = 0.0;
      xr = std::sqrt(-x->re);
    } else {
      absxr = std::sqrt(x->re);
      xr = 0.0;
    }
  } else if (x->re == 0.0) {
    if (x->im < 0.0) {
      absxr = std::sqrt(-x->im / 2.0);
      xr = -absxr;
    } else {
      absxr = std::sqrt(x->im / 2.0);
      xr = absxr;
    }
  } else if (rtIsNaN(x->re)) {
    absxr = x->re;
  } else if (rtIsNaN(x->im)) {
    absxr = x->im;
    xr = x->im;
  } else if (rtIsInf(x->im)) {
    absxr = std::abs(x->im);
    xr = x->im;
  } else if (rtIsInf(x->re)) {
    if (x->re < 0.0) {
      absxr = 0.0;
      xr = x->im * -x->re;
    } else {
      absxr = x->re;
      xr = 0.0;
    }
  } else {
    absxr = std::abs(x->re);
    xr = std::abs(x->im);
    if ((absxr > 4.4942328371557893E+307) || (xr > 4.4942328371557893E+307)) {
      absxr *= 0.5;
      xr = rt_hypotd_snf(absxr, xr * 0.5);
      if (xr > absxr) {
        absxr = std::sqrt(absxr / xr + 1.0) * std::sqrt(xr);
      } else {
        absxr = std::sqrt(xr) * 1.4142135623730951;
      }
    } else {
      absxr = std::sqrt((rt_hypotd_snf(absxr, xr) + absxr) * 0.5);
    }

    if (x->re > 0.0) {
      xr = x->im / absxr * 0.5;
    } else {
      if (x->im < 0.0) {
        xr = -absxr;
      } else {
        xr = absxr;
      }

      absxr = x->im / xr * 0.5;
    }
  }

  x->re = absxr;
  x->im = xr;
}

// Function for MATLAB Function: '<S91>/trajGetMatch'
void MatlabControllerClass::xzlartg_n(const creal_T f, const creal_T g, real_T
  *cs, creal_T *sn)
{
  real_T scale;
  real_T g2;
  real_T f2s;
  real_T di;
  real_T fs_re;
  real_T fs_im;
  real_T gs_re;
  real_T gs_im;
  real_T g2_tmp;
  boolean_T guard1 = false;
  di = std::abs(f.re);
  scale = di;
  g2_tmp = std::abs(f.im);
  if (g2_tmp > di) {
    scale = g2_tmp;
  }

  g2 = std::abs(g.re);
  f2s = std::abs(g.im);
  if (f2s > g2) {
    g2 = f2s;
  }

  if (g2 > scale) {
    scale = g2;
  }

  fs_re = f.re;
  fs_im = f.im;
  gs_re = g.re;
  gs_im = g.im;
  guard1 = false;
  if (scale >= 7.4428285367870146E+137) {
    do {
      fs_re *= 1.3435752215134178E-138;
      fs_im *= 1.3435752215134178E-138;
      gs_re *= 1.3435752215134178E-138;
      gs_im *= 1.3435752215134178E-138;
      scale *= 1.3435752215134178E-138;
    } while (!(scale < 7.4428285367870146E+137));

    guard1 = true;
  } else if (scale <= 1.3435752215134178E-138) {
    if ((g.re == 0.0) && (g.im == 0.0)) {
      *cs = 1.0;
      sn->re = 0.0;
      sn->im = 0.0;
    } else {
      do {
        fs_re *= 7.4428285367870146E+137;
        fs_im *= 7.4428285367870146E+137;
        gs_re *= 7.4428285367870146E+137;
        gs_im *= 7.4428285367870146E+137;
        scale *= 7.4428285367870146E+137;
      } while (!(scale > 1.3435752215134178E-138));

      guard1 = true;
    }
  } else {
    guard1 = true;
  }

  if (guard1) {
    scale = fs_re * fs_re + fs_im * fs_im;
    g2 = gs_re * gs_re + gs_im * gs_im;
    f2s = g2;
    if (1.0 > g2) {
      f2s = 1.0;
    }

    if (scale <= f2s * 2.0041683600089728E-292) {
      if ((f.re == 0.0) && (f.im == 0.0)) {
        *cs = 0.0;
        g2 = rt_hypotd_snf(gs_re, gs_im);
        sn->re = gs_re / g2;
        sn->im = -gs_im / g2;
      } else {
        scale = std::sqrt(g2);
        *cs = rt_hypotd_snf(fs_re, fs_im) / scale;
        g2 = di;
        if (g2_tmp > di) {
          g2 = g2_tmp;
        }

        if (g2 > 1.0) {
          g2 = rt_hypotd_snf(f.re, f.im);
          fs_re = f.re / g2;
          fs_im = f.im / g2;
        } else {
          f2s = 7.4428285367870146E+137 * f.re;
          di = 7.4428285367870146E+137 * f.im;
          g2 = rt_hypotd_snf(f2s, di);
          fs_re = f2s / g2;
          fs_im = di / g2;
        }

        gs_re /= scale;
        gs_im = -gs_im / scale;
        sn->re = fs_re * gs_re - fs_im * gs_im;
        sn->im = fs_re * gs_im + fs_im * gs_re;
      }
    } else {
      f2s = std::sqrt(g2 / scale + 1.0);
      fs_re *= f2s;
      fs_im *= f2s;
      *cs = 1.0 / f2s;
      g2 += scale;
      fs_re /= g2;
      fs_im /= g2;
      sn->re = fs_re * gs_re - fs_im * -gs_im;
      sn->im = fs_re * -gs_im + fs_im * gs_re;
    }
  }
}

// Function for MATLAB Function: '<S91>/trajGetMatch'
void MatlabControllerClass::xzhgeqz(const creal_T A_data[], const int32_T
  A_size[2], int32_T ilo, int32_T ihi, int32_T *info, creal_T alpha1_data[],
  int32_T *alpha1_size, creal_T beta1_data[], int32_T *beta1_size)
{
  creal_T ctemp;
  real_T anorm;
  real_T b_atol;
  real_T bscale;
  boolean_T failed;
  int32_T j;
  int32_T ifirst;
  int32_T istart;
  int32_T ilast;
  int32_T ilastm1;
  int32_T ifrstm;
  int32_T ilastm;
  int32_T iiter;
  boolean_T goto60;
  boolean_T goto70;
  boolean_T goto90;
  int32_T jp1;
  creal_T ad22;
  real_T temp;
  creal_T shift;
  real_T temp2;
  int32_T jiter;
  int32_T i;
  int32_T b_A_size_idx_0;
  creal_T anorm_0;
  real_T ar;
  real_T ai;
  real_T t1_re;
  real_T t1_im;
  real_T shift_im;
  real_T eshift_re;
  real_T eshift_im;
  int32_T ar_tmp;
  int32_T shift_tmp;
  int32_T shift_tmp_0;
  int32_T shift_tmp_tmp;
  boolean_T guard1 = false;
  boolean_T guard2 = false;
  boolean_T guard3 = false;
  int32_T exitg1;
  boolean_T exitg2;
  b_A_size_idx_0 = A_size[0];
  ilast = A_size[0] * A_size[1] - 1;
  if (0 <= ilast) {
    memcpy(&rtDW.b_A_data[0], &A_data[0], (ilast + 1) * sizeof(creal_T));
  }

  *info = 0;
  if ((A_size[0] == 1) && (A_size[1] == 1)) {
    ihi = 1;
  }

  *alpha1_size = A_size[0];
  ilast = A_size[0];
  if (0 <= ilast - 1) {
    memset(&alpha1_data[0], 0, ilast * sizeof(creal_T));
  }

  *beta1_size = A_size[0];
  ilast = A_size[0];
  for (ar_tmp = 0; ar_tmp < ilast; ar_tmp++) {
    beta1_data[ar_tmp].re = 1.0;
    beta1_data[ar_tmp].im = 0.0;
  }

  eshift_re = 0.0;
  eshift_im = 0.0;
  ctemp.re = 0.0;
  ctemp.im = 0.0;
  anorm = xzlanhs(A_data, A_size, ilo, ihi);
  bscale = 2.2204460492503131E-16 * anorm;
  b_atol = 2.2250738585072014E-308;
  if (bscale > 2.2250738585072014E-308) {
    b_atol = bscale;
  }

  bscale = 2.2250738585072014E-308;
  if (anorm > 2.2250738585072014E-308) {
    bscale = anorm;
  }

  anorm = 1.0 / bscale;
  bscale = 1.0 / std::sqrt((real_T)A_size[0]);
  failed = true;
  for (ilast = ihi; ilast < A_size[0]; ilast++) {
    alpha1_data[ilast] = A_data[A_size[0] * ilast + ilast];
  }

  guard1 = false;
  guard2 = false;
  if (ihi >= ilo) {
    ifirst = ilo;
    istart = ilo;
    ilast = ihi - 1;
    ilastm1 = ihi - 2;
    ifrstm = ilo;
    ilastm = ihi;
    iiter = 0;
    goto60 = false;
    goto70 = false;
    goto90 = false;
    jiter = 0;
    do {
      exitg1 = 0;
      if (jiter <= ((ihi - ilo) + 1) * 30 - 1) {
        if (ilast + 1 == ilo) {
          goto60 = true;
        } else {
          ar_tmp = b_A_size_idx_0 * ilastm1;
          if (std::abs(rtDW.b_A_data[ar_tmp + ilast].re) + std::abs
              (rtDW.b_A_data[b_A_size_idx_0 * ilastm1 + ilast].im) <= b_atol) {
            ar_tmp += ilast;
            rtDW.b_A_data[ar_tmp].re = 0.0;
            rtDW.b_A_data[ar_tmp].im = 0.0;
            goto60 = true;
          } else {
            j = ilastm1;
            guard3 = false;
            exitg2 = false;
            while ((!exitg2) && (j + 1 >= ilo)) {
              if (j + 1 == ilo) {
                guard3 = true;
                exitg2 = true;
              } else if (std::abs(rtDW.b_A_data[(j - 1) * b_A_size_idx_0 + j].re)
                         + std::abs(rtDW.b_A_data[(j - 1) * b_A_size_idx_0 + j].
                                    im) <= b_atol) {
                ar_tmp = j + b_A_size_idx_0 * (j - 1);
                rtDW.b_A_data[ar_tmp].re = 0.0;
                rtDW.b_A_data[ar_tmp].im = 0.0;
                guard3 = true;
                exitg2 = true;
              } else {
                j--;
                guard3 = false;
              }
            }

            if (guard3) {
              ifirst = j + 1;
              goto70 = true;
            }
          }
        }

        if ((!goto60) && (!goto70)) {
          ilast = *alpha1_size;
          for (ar_tmp = 0; ar_tmp < ilast; ar_tmp++) {
            alpha1_data[ar_tmp].re = (rtNaN);
            alpha1_data[ar_tmp].im = 0.0;
          }

          ilast = *beta1_size;
          for (ar_tmp = 0; ar_tmp < ilast; ar_tmp++) {
            beta1_data[ar_tmp].re = (rtNaN);
            beta1_data[ar_tmp].im = 0.0;
          }

          *info = 1;
          exitg1 = 1;
        } else if (goto60) {
          goto60 = false;
          alpha1_data[ilast] = rtDW.b_A_data[b_A_size_idx_0 * ilast + ilast];
          ilast = ilastm1;
          ilastm1--;
          if (ilast + 1 < ilo) {
            failed = false;
            guard2 = true;
            exitg1 = 1;
          } else {
            iiter = 0;
            eshift_re = 0.0;
            eshift_im = 0.0;
            ilastm = ilast + 1;
            if (ifrstm > ilast + 1) {
              ifrstm = ilo;
            }

            jiter++;
          }
        } else {
          if (goto70) {
            goto70 = false;
            iiter++;
            ifrstm = ifirst;
            if (mod(iiter) != 0) {
              ar = rtDW.b_A_data[b_A_size_idx_0 * ilastm1 + ilastm1].re * anorm;
              ai = rtDW.b_A_data[b_A_size_idx_0 * ilastm1 + ilastm1].im * anorm;
              if (ai == 0.0) {
                shift.re = ar / bscale;
                shift.im = 0.0;
              } else if (ar == 0.0) {
                shift.re = 0.0;
                shift.im = ai / bscale;
              } else {
                shift.re = ar / bscale;
                shift.im = ai / bscale;
              }

              ar_tmp = b_A_size_idx_0 * ilast;
              ar = rtDW.b_A_data[ar_tmp + ilast].re * anorm;
              ai = rtDW.b_A_data[b_A_size_idx_0 * ilast + ilast].im * anorm;
              if (ai == 0.0) {
                ad22.re = ar / bscale;
                ad22.im = 0.0;
              } else if (ar == 0.0) {
                ad22.re = 0.0;
                ad22.im = ai / bscale;
              } else {
                ad22.re = ar / bscale;
                ad22.im = ai / bscale;
              }

              t1_re = (shift.re + ad22.re) * 0.5;
              t1_im = (shift.im + ad22.im) * 0.5;
              ar = rtDW.b_A_data[ar_tmp + ilastm1].re * anorm;
              ai = rtDW.b_A_data[b_A_size_idx_0 * ilast + ilastm1].im * anorm;
              if (ai == 0.0) {
                temp = ar / bscale;
                temp2 = 0.0;
              } else if (ar == 0.0) {
                temp = 0.0;
                temp2 = ai / bscale;
              } else {
                temp = ar / bscale;
                temp2 = ai / bscale;
              }

              ar = rtDW.b_A_data[b_A_size_idx_0 * ilastm1 + ilast].re * anorm;
              ai = rtDW.b_A_data[b_A_size_idx_0 * ilastm1 + ilast].im * anorm;
              if (ai == 0.0) {
                ar /= bscale;
                ai = 0.0;
              } else if (ar == 0.0) {
                ar = 0.0;
                ai /= bscale;
              } else {
                ar /= bscale;
                ai /= bscale;
              }

              shift_im = shift.re * ad22.im + shift.im * ad22.re;
              shift.re = ((t1_re * t1_re - t1_im * t1_im) + (temp * ar - temp2 *
                ai)) - (shift.re * ad22.re - shift.im * ad22.im);
              shift.im = ((t1_re * t1_im + t1_im * t1_re) + (temp * ai + temp2 *
                ar)) - shift_im;
              sqrt_j(&shift);
              if ((t1_re - ad22.re) * shift.re + (t1_im - ad22.im) * shift.im <=
                  0.0) {
                shift.re += t1_re;
                shift.im += t1_im;
              } else {
                shift.re = t1_re - shift.re;
                shift.im = t1_im - shift.im;
              }
            } else {
              temp = rtDW.b_A_data[b_A_size_idx_0 * ilastm1 + ilast].re * anorm;
              temp2 = rtDW.b_A_data[b_A_size_idx_0 * ilastm1 + ilast].im * anorm;
              if (temp2 == 0.0) {
                temp /= bscale;
                temp2 = 0.0;
              } else if (temp == 0.0) {
                temp = 0.0;
                temp2 /= bscale;
              } else {
                temp /= bscale;
                temp2 /= bscale;
              }

              eshift_re += temp;
              eshift_im += temp2;
              shift.re = eshift_re;
              shift.im = eshift_im;
            }

            j = ilastm1;
            jp1 = ilastm1 + 1;
            exitg2 = false;
            while ((!exitg2) && (j + 1 > ifirst)) {
              istart = j + 1;
              ar_tmp = b_A_size_idx_0 * j;
              ctemp.re = rtDW.b_A_data[ar_tmp + j].re * anorm - shift.re *
                bscale;
              ctemp.im = rtDW.b_A_data[b_A_size_idx_0 * j + j].im * anorm -
                shift.im * bscale;
              temp = std::abs(ctemp.re) + std::abs(ctemp.im);
              temp2 = (std::abs(rtDW.b_A_data[ar_tmp + jp1].re) + std::abs
                       (rtDW.b_A_data[b_A_size_idx_0 * j + jp1].im)) * anorm;
              t1_re = temp;
              if (temp2 > temp) {
                t1_re = temp2;
              }

              if ((t1_re < 1.0) && (t1_re != 0.0)) {
                temp /= t1_re;
                temp2 /= t1_re;
              }

              if ((std::abs(rtDW.b_A_data[(j - 1) * b_A_size_idx_0 + j].re) +
                   std::abs(rtDW.b_A_data[(j - 1) * b_A_size_idx_0 + j].im)) *
                  temp2 <= temp * b_atol) {
                goto90 = true;
                exitg2 = true;
              } else {
                jp1 = j;
                j--;
              }
            }

            if (!goto90) {
              istart = ifirst;
              ctemp.re = rtDW.b_A_data[((ifirst - 1) * b_A_size_idx_0 + ifirst)
                - 1].re * anorm - shift.re * bscale;
              ctemp.im = rtDW.b_A_data[((ifirst - 1) * b_A_size_idx_0 + ifirst)
                - 1].im * anorm - shift.im * bscale;
              goto90 = true;
            }
          }

          if (goto90) {
            goto90 = false;
            anorm_0.re = rtDW.b_A_data[(istart - 1) * b_A_size_idx_0 + istart].
              re * anorm;
            anorm_0.im = rtDW.b_A_data[(istart - 1) * b_A_size_idx_0 + istart].
              im * anorm;
            xzlartg_n(ctemp, anorm_0, &temp, &ad22);
            j = istart;
            jp1 = istart - 2;
            while (j < ilast + 1) {
              if (j > istart) {
                xzlartg(rtDW.b_A_data[(j + b_A_size_idx_0 * jp1) - 1],
                        rtDW.b_A_data[j + b_A_size_idx_0 * jp1], &temp, &ad22,
                        &rtDW.b_A_data[(j + b_A_size_idx_0 * jp1) - 1]);
                ar_tmp = j + b_A_size_idx_0 * jp1;
                rtDW.b_A_data[ar_tmp].re = 0.0;
                rtDW.b_A_data[ar_tmp].im = 0.0;
              }

              for (jp1 = j - 1; jp1 < ilastm; jp1++) {
                shift_tmp_tmp = b_A_size_idx_0 * jp1;
                shift_tmp = shift_tmp_tmp + j;
                shift_tmp_0 = shift_tmp - 1;
                shift.re = rtDW.b_A_data[shift_tmp_0].re * temp +
                  (rtDW.b_A_data[shift_tmp].re * ad22.re -
                   rtDW.b_A_data[b_A_size_idx_0 * jp1 + j].im * ad22.im);
                shift.im = rtDW.b_A_data[(b_A_size_idx_0 * jp1 + j) - 1].im *
                  temp + (rtDW.b_A_data[b_A_size_idx_0 * jp1 + j].im * ad22.re +
                          rtDW.b_A_data[b_A_size_idx_0 * jp1 + j].re * ad22.im);
                temp2 = rtDW.b_A_data[(b_A_size_idx_0 * jp1 + j) - 1].re;
                ar_tmp = j + shift_tmp_tmp;
                rtDW.b_A_data[ar_tmp].re = rtDW.b_A_data[b_A_size_idx_0 * jp1 +
                  j].re * temp - (rtDW.b_A_data[(b_A_size_idx_0 * jp1 + j) - 1].
                                  re * ad22.re + rtDW.b_A_data[(b_A_size_idx_0 *
                  jp1 + j) - 1].im * ad22.im);
                rtDW.b_A_data[ar_tmp].im = rtDW.b_A_data[shift_tmp].im * temp -
                  (rtDW.b_A_data[shift_tmp_0].im * ad22.re - ad22.im * temp2);
                rtDW.b_A_data[ar_tmp - 1] = shift;
              }

              ad22.re = -ad22.re;
              ad22.im = -ad22.im;
              jp1 = j;
              if (ilast + 1 < j + 2) {
                jp1 = ilast - 1;
              }

              for (i = ifrstm - 1; i < jp1 + 2; i++) {
                shift_tmp = (j - 1) * b_A_size_idx_0 + i;
                shift_tmp_tmp = b_A_size_idx_0 * j;
                shift_tmp_0 = shift_tmp_tmp + i;
                shift.re = (rtDW.b_A_data[shift_tmp].re * ad22.re -
                            rtDW.b_A_data[(j - 1) * b_A_size_idx_0 + i].im *
                            ad22.im) + rtDW.b_A_data[shift_tmp_0].re * temp;
                shift.im = (rtDW.b_A_data[(j - 1) * b_A_size_idx_0 + i].im *
                            ad22.re + rtDW.b_A_data[(j - 1) * b_A_size_idx_0 + i]
                            .re * ad22.im) + rtDW.b_A_data[b_A_size_idx_0 * j +
                  i].im * temp;
                temp2 = rtDW.b_A_data[b_A_size_idx_0 * j + i].re;
                ar_tmp = i + b_A_size_idx_0 * (j - 1);
                rtDW.b_A_data[ar_tmp].re = rtDW.b_A_data[(j - 1) *
                  b_A_size_idx_0 + i].re * temp - (rtDW.b_A_data[b_A_size_idx_0 *
                  j + i].re * ad22.re + rtDW.b_A_data[b_A_size_idx_0 * j + i].im
                  * ad22.im);
                rtDW.b_A_data[ar_tmp].im = rtDW.b_A_data[shift_tmp].im * temp -
                  (rtDW.b_A_data[shift_tmp_0].im * ad22.re - ad22.im * temp2);
                rtDW.b_A_data[i + shift_tmp_tmp] = shift;
              }

              jp1 = j - 1;
              j++;
            }
          }

          jiter++;
        }
      } else {
        guard2 = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  } else {
    guard1 = true;
  }

  if (guard2) {
    if (failed) {
      *info = ilast + 1;
      for (ifirst = 0; ifirst <= ilast; ifirst++) {
        alpha1_data[ifirst].re = (rtNaN);
        alpha1_data[ifirst].im = 0.0;
        beta1_data[ifirst].re = (rtNaN);
        beta1_data[ifirst].im = 0.0;
      }
    } else {
      guard1 = true;
    }
  }

  if (guard1) {
    for (ilast = 0; ilast <= ilo - 2; ilast++) {
      alpha1_data[ilast] = rtDW.b_A_data[b_A_size_idx_0 * ilast + ilast];
    }
  }
}

// Function for MATLAB Function: '<S91>/trajGetMatch'
void MatlabControllerClass::xzlascl_n(real_T cfrom, real_T cto, creal_T A_data[],
  int32_T *A_size)
{
  real_T cfromc;
  real_T ctoc;
  boolean_T notdone;
  real_T cfrom1;
  real_T cto1;
  real_T mul;
  int32_T i;
  int32_T loop_ub;
  cfromc = cfrom;
  ctoc = cto;
  notdone = true;
  while (notdone) {
    cfrom1 = cfromc * 2.0041683600089728E-292;
    cto1 = ctoc / 4.9896007738368E+291;
    if ((std::abs(cfrom1) > std::abs(ctoc)) && (ctoc != 0.0)) {
      mul = 2.0041683600089728E-292;
      cfromc = cfrom1;
    } else if (std::abs(cto1) > std::abs(cfromc)) {
      mul = 4.9896007738368E+291;
      ctoc = cto1;
    } else {
      mul = ctoc / cfromc;
      notdone = false;
    }

    loop_ub = *A_size;
    for (i = 0; i < loop_ub; i++) {
      A_data[i].re *= mul;
      A_data[i].im *= mul;
    }
  }
}

// Function for MATLAB Function: '<S91>/trajGetMatch'
void MatlabControllerClass::xzgeev(const creal_T A_data[], const int32_T A_size
  [2], int32_T *info, creal_T alpha1_data[], int32_T *alpha1_size, creal_T
  beta1_data[], int32_T *beta1_size)
{
  real_T anrm;
  boolean_T ilascl;
  real_T anrmto;
  int32_T ihi;
  int32_T rscale_data[17];
  int32_T loop_ub;
  int32_T At_size[2];
  int32_T rscale_size;
  At_size[0] = A_size[0];
  At_size[1] = A_size[1];
  loop_ub = A_size[0] * A_size[1] - 1;
  if (0 <= loop_ub) {
    memcpy(&rtDW.At_data[0], &A_data[0], (loop_ub + 1) * sizeof(creal_T));
  }

  *info = 0;
  anrm = xzlangeM(A_data, A_size);
  if (!isfinite(anrm)) {
    *alpha1_size = A_size[0];
    loop_ub = A_size[0];
    for (ihi = 0; ihi < loop_ub; ihi++) {
      alpha1_data[ihi].re = (rtNaN);
      alpha1_data[ihi].im = 0.0;
    }

    *beta1_size = A_size[0];
    loop_ub = A_size[0];
    for (ihi = 0; ihi < loop_ub; ihi++) {
      beta1_data[ihi].re = (rtNaN);
      beta1_data[ihi].im = 0.0;
    }
  } else {
    ilascl = false;
    anrmto = anrm;
    if ((anrm > 0.0) && (anrm < 6.7178761075670888E-139)) {
      anrmto = 6.7178761075670888E-139;
      ilascl = true;
    } else {
      if (anrm > 1.4885657073574029E+138) {
        anrmto = 1.4885657073574029E+138;
        ilascl = true;
      }
    }

    if (ilascl) {
      xzlascl(anrm, anrmto, rtDW.At_data, At_size);
    }

    xzggbal(rtDW.At_data, At_size, &loop_ub, &ihi, rscale_data, &rscale_size);
    xzgghrd(loop_ub, ihi, rtDW.At_data, At_size);
    xzhgeqz(rtDW.At_data, At_size, loop_ub, ihi, info, alpha1_data, alpha1_size,
            beta1_data, beta1_size);
    if ((*info == 0) && ilascl) {
      xzlascl_n(anrmto, anrm, alpha1_data, alpha1_size);
    }
  }
}

// Function for MATLAB Function: '<S91>/trajGetMatch'
real_T MatlabControllerClass::xnrm2_g(int32_T n, const creal_T x_data[], int32_T
  ix0)
{
  real_T y;
  real_T scale;
  int32_T kend;
  real_T absxk;
  real_T t;
  int32_T k;
  y = 0.0;
  if (n >= 1) {
    if (n == 1) {
      y = rt_hypotd_snf(x_data[ix0 - 1].re, x_data[ix0 - 1].im);
    } else {
      scale = 3.3121686421112381E-170;
      kend = (ix0 + n) - 1;
      for (k = ix0; k <= kend; k++) {
        absxk = std::abs(x_data[k - 1].re);
        if (absxk > scale) {
          t = scale / absxk;
          y = y * t * t + 1.0;
          scale = absxk;
        } else {
          t = absxk / scale;
          y += t * t;
        }

        absxk = std::abs(x_data[k - 1].im);
        if (absxk > scale) {
          t = scale / absxk;
          y = y * t * t + 1.0;
          scale = absxk;
        } else {
          t = absxk / scale;
          y += t * t;
        }
      }

      y = scale * std::sqrt(y);
    }
  }

  return y;
}

// Function for MATLAB Function: '<S91>/trajGetMatch'
real_T MatlabControllerClass::xdlapy3(real_T x1, real_T x2, real_T x3)
{
  real_T y;
  real_T a;
  real_T b;
  real_T c;
  a = std::abs(x1);
  b = std::abs(x2);
  c = std::abs(x3);
  if ((a > b) || rtIsNaN(b)) {
    y = a;
  } else {
    y = b;
  }

  if (c > y) {
    y = c;
  }

  if (y > 0.0) {
    if (!rtIsInf(y)) {
      a /= y;
      b /= y;
      c /= y;
      y *= std::sqrt((a * a + c * c) + b * b);
    } else {
      y = (a + b) + c;
    }
  } else {
    y = (a + b) + c;
  }

  return y;
}

// Function for MATLAB Function: '<S91>/trajGetMatch'
creal_T MatlabControllerClass::recip(const creal_T y)
{
  creal_T z;
  real_T br;
  real_T brm;
  real_T bim;
  brm = std::abs(y.re);
  bim = std::abs(y.im);
  if (y.im == 0.0) {
    z.re = 1.0 / y.re;
    z.im = 0.0;
  } else if (y.re == 0.0) {
    z.re = 0.0;
    z.im = -1.0 / y.im;
  } else if (brm > bim) {
    brm = y.im / y.re;
    bim = brm * y.im + y.re;
    z.re = 1.0 / bim;
    z.im = -brm / bim;
  } else if (brm == bim) {
    bim = 0.5;
    if (y.re < 0.0) {
      bim = -0.5;
    }

    br = 0.5;
    if (y.im < 0.0) {
      br = -0.5;
    }

    z.re = bim / brm;
    z.im = -br / brm;
  } else {
    brm = y.re / y.im;
    bim = brm * y.re + y.im;
    z.re = brm / bim;
    z.im = -1.0 / bim;
  }

  return z;
}

// Function for MATLAB Function: '<S91>/trajGetMatch'
creal_T MatlabControllerClass::xzlarfg(int32_T n, creal_T *alpha1, creal_T
  x_data[], int32_T ix0)
{
  creal_T tau;
  real_T xnorm;
  int32_T knt;
  int32_T b_k;
  int32_T c_k;
  creal_T alpha1_0;
  real_T ar;
  tau.re = 0.0;
  tau.im = 0.0;
  if (n > 0) {
    xnorm = xnrm2_g(n - 1, x_data, ix0);
    if ((xnorm != 0.0) || (alpha1->im != 0.0)) {
      xnorm = xdlapy3(alpha1->re, alpha1->im, xnorm);
      if (alpha1->re >= 0.0) {
        xnorm = -xnorm;
      }

      if (std::abs(xnorm) < 1.0020841800044864E-292) {
        knt = -1;
        b_k = (ix0 + n) - 2;
        do {
          knt++;
          for (c_k = ix0; c_k <= b_k; c_k++) {
            ar = x_data[c_k - 1].im * 9.9792015476736E+291 + x_data[c_k - 1].re *
              0.0;
            x_data[c_k - 1].re = x_data[c_k - 1].re * 9.9792015476736E+291 -
              x_data[c_k - 1].im * 0.0;
            x_data[c_k - 1].im = ar;
          }

          xnorm *= 9.9792015476736E+291;
          alpha1->re *= 9.9792015476736E+291;
          alpha1->im *= 9.9792015476736E+291;
        } while (!(std::abs(xnorm) >= 1.0020841800044864E-292));

        xnorm = xdlapy3(alpha1->re, alpha1->im, xnrm2_g(n - 1, x_data, ix0));
        if (alpha1->re >= 0.0) {
          xnorm = -xnorm;
        }

        ar = xnorm - alpha1->re;
        if (0.0 - alpha1->im == 0.0) {
          tau.re = ar / xnorm;
          tau.im = 0.0;
        } else if (ar == 0.0) {
          tau.re = 0.0;
          tau.im = (0.0 - alpha1->im) / xnorm;
        } else {
          tau.re = ar / xnorm;
          tau.im = (0.0 - alpha1->im) / xnorm;
        }

        alpha1_0.re = alpha1->re - xnorm;
        alpha1_0.im = alpha1->im;
        *alpha1 = recip(alpha1_0);
        b_k = (ix0 + n) - 2;
        for (c_k = ix0; c_k <= b_k; c_k++) {
          ar = x_data[c_k - 1].im * alpha1->re + x_data[c_k - 1].re * alpha1->im;
          x_data[c_k - 1].re = x_data[c_k - 1].re * alpha1->re - x_data[c_k - 1]
            .im * alpha1->im;
          x_data[c_k - 1].im = ar;
        }

        for (b_k = 0; b_k <= knt; b_k++) {
          xnorm *= 1.0020841800044864E-292;
        }

        alpha1->re = xnorm;
        alpha1->im = 0.0;
      } else {
        ar = xnorm - alpha1->re;
        if (0.0 - alpha1->im == 0.0) {
          tau.re = ar / xnorm;
          tau.im = 0.0;
        } else if (ar == 0.0) {
          tau.re = 0.0;
          tau.im = (0.0 - alpha1->im) / xnorm;
        } else {
          tau.re = ar / xnorm;
          tau.im = (0.0 - alpha1->im) / xnorm;
        }

        alpha1_0.re = alpha1->re - xnorm;
        alpha1_0.im = alpha1->im;
        *alpha1 = recip(alpha1_0);
        knt = (ix0 + n) - 2;
        for (b_k = ix0; b_k <= knt; b_k++) {
          ar = x_data[b_k - 1].im * alpha1->re + x_data[b_k - 1].re * alpha1->im;
          x_data[b_k - 1].re = x_data[b_k - 1].re * alpha1->re - x_data[b_k - 1]
            .im * alpha1->im;
          x_data[b_k - 1].im = ar;
        }

        alpha1->re = xnorm;
        alpha1->im = 0.0;
      }
    }
  }

  return tau;
}

// Function for MATLAB Function: '<S91>/trajGetMatch'
int32_T MatlabControllerClass::ilazlr(int32_T m, int32_T n, const creal_T
  A_data[], int32_T ia0, int32_T lda)
{
  int32_T i;
  int32_T rowleft;
  int32_T rowright;
  int32_T exitg1;
  boolean_T exitg2;
  i = m;
  exitg2 = false;
  while ((!exitg2) && (i > 0)) {
    rowleft = (ia0 + i) - 1;
    rowright = (n - 1) * lda + rowleft;
    do {
      exitg1 = 0;
      if (((lda > 0) && (rowleft <= rowright)) || ((lda < 0) && (rowleft >=
            rowright))) {
        if ((A_data[rowleft - 1].re != 0.0) || (A_data[rowleft - 1].im != 0.0))
        {
          exitg1 = 1;
        } else {
          rowleft += lda;
        }
      } else {
        i--;
        exitg1 = 2;
      }
    } while (exitg1 == 0);

    if (exitg1 == 1) {
      exitg2 = true;
    }
  }

  return i;
}

// Function for MATLAB Function: '<S91>/trajGetMatch'
void MatlabControllerClass::xgemv(int32_T m, int32_T n, const creal_T A_data[],
  int32_T ia0, int32_T lda, const creal_T x_data[], int32_T ix0, creal_T y_data[])
{
  int32_T iy;
  int32_T ix;
  int32_T b;
  int32_T iac;
  int32_T d;
  int32_T ia;
  real_T c_re;
  real_T c_im;
  if (m != 0) {
    for (ix = 0; ix < m; ix++) {
      y_data[ix].re = 0.0;
      y_data[ix].im = 0.0;
    }

    ix = ix0;
    b = (n - 1) * lda + ia0;
    iac = ia0;
    while (((lda > 0) && (iac <= b)) || ((lda < 0) && (iac >= b))) {
      c_re = x_data[ix - 1].re - x_data[ix - 1].im * 0.0;
      c_im = x_data[ix - 1].re * 0.0 + x_data[ix - 1].im;
      iy = 0;
      d = (iac + m) - 1;
      for (ia = iac; ia <= d; ia++) {
        y_data[iy].re += A_data[ia - 1].re * c_re - A_data[ia - 1].im * c_im;
        y_data[iy].im += A_data[ia - 1].re * c_im + A_data[ia - 1].im * c_re;
        iy++;
      }

      ix++;
      iac += lda;
    }
  }
}

// Function for MATLAB Function: '<S91>/trajGetMatch'
void MatlabControllerClass::xgerc(int32_T m, int32_T n, const creal_T alpha1,
  const creal_T x_data[], int32_T iy0, creal_T A_data[], int32_T ia0, int32_T
  lda)
{
  int32_T jA;
  int32_T jy;
  int32_T ix;
  int32_T j;
  int32_T b;
  int32_T ijA;
  real_T temp_re;
  real_T temp_im;
  if ((!(alpha1.re == 0.0)) || (!(alpha1.im == 0.0))) {
    jA = ia0 - 1;
    jy = iy0 - 1;
    for (j = 0; j < n; j++) {
      if ((A_data[jy].re != 0.0) || (A_data[jy].im != 0.0)) {
        temp_re = A_data[jy].re * alpha1.re + A_data[jy].im * alpha1.im;
        temp_im = A_data[jy].re * alpha1.im - A_data[jy].im * alpha1.re;
        ix = 0;
        b = m + jA;
        for (ijA = jA; ijA < b; ijA++) {
          A_data[ijA].re += x_data[ix].re * temp_re - x_data[ix].im * temp_im;
          A_data[ijA].im += x_data[ix].re * temp_im + x_data[ix].im * temp_re;
          ix++;
        }
      }

      jy++;
      jA += lda;
    }
  }
}

// Function for MATLAB Function: '<S91>/trajGetMatch'
void MatlabControllerClass::xzlarf_f(int32_T m, int32_T n, int32_T iv0, const
  creal_T tau, creal_T C_data[], int32_T ic0, int32_T ldc, creal_T work_data[])
{
  int32_T lastv;
  int32_T lastc;
  creal_T tau_0;
  if ((tau.re != 0.0) || (tau.im != 0.0)) {
    lastv = n;
    lastc = iv0 + n;
    while ((lastv > 0) && ((C_data[lastc - 2].re == 0.0) && (C_data[lastc - 2].
             im == 0.0))) {
      lastv--;
      lastc--;
    }

    lastc = ilazlr(m, lastv, C_data, ic0, ldc);
  } else {
    lastv = 0;
    lastc = 0;
  }

  if (lastv > 0) {
    xgemv(lastc, lastv, C_data, ic0, ldc, C_data, iv0, work_data);
    tau_0.re = -tau.re;
    tau_0.im = -tau.im;
    xgerc(lastc, lastv, tau_0, work_data, iv0, C_data, ic0, ldc);
  }
}

// Function for MATLAB Function: '<S91>/trajGetMatch'
int32_T MatlabControllerClass::ilazlc(int32_T m, int32_T n, const creal_T
  A_data[], int32_T ia0, int32_T lda)
{
  int32_T j;
  int32_T coltop;
  int32_T ia;
  int32_T exitg1;
  boolean_T exitg2;
  j = n;
  exitg2 = false;
  while ((!exitg2) && (j > 0)) {
    coltop = (j - 1) * lda + ia0;
    ia = coltop;
    do {
      exitg1 = 0;
      if (ia <= (coltop + m) - 1) {
        if ((A_data[ia - 1].re != 0.0) || (A_data[ia - 1].im != 0.0)) {
          exitg1 = 1;
        } else {
          ia++;
        }
      } else {
        j--;
        exitg1 = 2;
      }
    } while (exitg1 == 0);

    if (exitg1 == 1) {
      exitg2 = true;
    }
  }

  return j;
}

// Function for MATLAB Function: '<S91>/trajGetMatch'
void MatlabControllerClass::xgemv_c(int32_T m, int32_T n, const creal_T A_data[],
  int32_T ia0, int32_T lda, const creal_T x_data[], int32_T ix0, creal_T y_data[])
{
  int32_T ix;
  int32_T b_iy;
  int32_T b;
  int32_T iac;
  int32_T d;
  int32_T ia;
  real_T c_re;
  real_T c_im;
  if (n != 0) {
    for (b_iy = 0; b_iy < n; b_iy++) {
      y_data[b_iy].re = 0.0;
      y_data[b_iy].im = 0.0;
    }

    b_iy = 0;
    b = (n - 1) * lda + ia0;
    iac = ia0;
    while (((lda > 0) && (iac <= b)) || ((lda < 0) && (iac >= b))) {
      ix = ix0 - 1;
      c_re = 0.0;
      c_im = 0.0;
      d = (iac + m) - 1;
      for (ia = iac - 1; ia < d; ia++) {
        c_re += A_data[ia].re * x_data[ix].re + A_data[ia].im * x_data[ix].im;
        c_im += A_data[ia].re * x_data[ix].im - A_data[ia].im * x_data[ix].re;
        ix++;
      }

      y_data[b_iy].re += c_re - 0.0 * c_im;
      y_data[b_iy].im += 0.0 * c_re + c_im;
      b_iy++;
      iac += lda;
    }
  }
}

// Function for MATLAB Function: '<S91>/trajGetMatch'
void MatlabControllerClass::xgerc_c(int32_T m, int32_T n, const creal_T alpha1,
  int32_T ix0, const creal_T y_data[], creal_T A_data[], int32_T ia0, int32_T
  lda)
{
  int32_T jA;
  int32_T jy;
  int32_T ix;
  int32_T j;
  int32_T b;
  int32_T ijA;
  real_T A_data_im;
  real_T temp_re;
  real_T temp_im;
  if ((!(alpha1.re == 0.0)) || (!(alpha1.im == 0.0))) {
    jA = ia0 - 1;
    jy = 0;
    for (j = 0; j < n; j++) {
      if ((y_data[jy].re != 0.0) || (y_data[jy].im != 0.0)) {
        temp_re = y_data[jy].re * alpha1.re + y_data[jy].im * alpha1.im;
        temp_im = y_data[jy].re * alpha1.im - y_data[jy].im * alpha1.re;
        ix = ix0;
        b = m + jA;
        for (ijA = jA; ijA < b; ijA++) {
          A_data_im = A_data[ix - 1].re * temp_im + A_data[ix - 1].im * temp_re;
          A_data[ijA].re += A_data[ix - 1].re * temp_re - A_data[ix - 1].im *
            temp_im;
          A_data[ijA].im += A_data_im;
          ix++;
        }
      }

      jy++;
      jA += lda;
    }
  }
}

// Function for MATLAB Function: '<S91>/trajGetMatch'
void MatlabControllerClass::xzlarf_fy(int32_T m, int32_T n, int32_T iv0, const
  creal_T tau, creal_T C_data[], int32_T ic0, int32_T ldc, creal_T work_data[])
{
  int32_T lastv;
  int32_T lastc;
  creal_T tau_0;
  if ((tau.re != 0.0) || (tau.im != 0.0)) {
    lastv = m;
    lastc = iv0 + m;
    while ((lastv > 0) && ((C_data[lastc - 2].re == 0.0) && (C_data[lastc - 2].
             im == 0.0))) {
      lastv--;
      lastc--;
    }

    lastc = ilazlc(lastv, n, C_data, ic0, ldc);
  } else {
    lastv = 0;
    lastc = 0;
  }

  if (lastv > 0) {
    xgemv_c(lastv, lastc, C_data, ic0, ldc, C_data, iv0, work_data);
    tau_0.re = -tau.re;
    tau_0.im = -tau.im;
    xgerc_c(lastv, lastc, tau_0, iv0, work_data, C_data, ic0, ldc);
  }
}

// Function for MATLAB Function: '<S91>/trajGetMatch'
void MatlabControllerClass::xgehrd(creal_T a_data[], int32_T a_size[2])
{
  creal_T tau_data[16];
  int32_T n;
  creal_T work_data[17];
  int32_T im1n;
  int32_T in;
  creal_T alpha1;
  int32_T loop_ub;
  int8_T c_idx_0;
  creal_T tau_data_0;
  int32_T u0;
  int32_T alpha1_tmp;
  n = a_size[0];
  c_idx_0 = (int8_T)a_size[0];
  if (0 <= c_idx_0 - 1) {
    memset(&work_data[0], 0, c_idx_0 * sizeof(creal_T));
  }

  for (loop_ub = 0; loop_ub <= n - 2; loop_ub++) {
    im1n = loop_ub * n + 2;
    in = (loop_ub + 1) * n;
    alpha1_tmp = a_size[0] * loop_ub;
    alpha1 = a_data[(alpha1_tmp + loop_ub) + 1];
    u0 = loop_ub + 3;
    if (u0 >= n) {
      u0 = n;
    }

    tau_data[loop_ub] = xzlarfg((n - loop_ub) - 1, &alpha1, a_data, loop_ub * n
      + u0);
    alpha1_tmp = (loop_ub + alpha1_tmp) + 1;
    a_data[alpha1_tmp].re = 1.0;
    a_data[alpha1_tmp].im = 0.0;
    xzlarf_f(n, (n - loop_ub) - 1, loop_ub + im1n, tau_data[loop_ub], a_data, in
             + 1, n, work_data);
    tau_data_0.re = tau_data[loop_ub].re;
    tau_data_0.im = -tau_data[loop_ub].im;
    xzlarf_fy((n - loop_ub) - 1, (n - loop_ub) - 1, loop_ub + im1n, tau_data_0,
              a_data, (loop_ub + in) + 2, n, work_data);
    a_data[alpha1_tmp] = alpha1;
  }
}

// Function for MATLAB Function: '<S91>/trajGetMatch'
void MatlabControllerClass::xscal_jo(int32_T n, const creal_T a, creal_T x_data[],
  int32_T ix0, int32_T incx)
{
  int32_T b;
  int32_T k;
  real_T im;
  b = (n - 1) * incx + ix0;
  k = ix0;
  while (((incx > 0) && (k <= b)) || ((incx < 0) && (k >= b))) {
    im = x_data[k - 1].im * a.re + x_data[k - 1].re * a.im;
    x_data[k - 1].re = x_data[k - 1].re * a.re - x_data[k - 1].im * a.im;
    x_data[k - 1].im = im;
    k += incx;
  }
}

// Function for MATLAB Function: '<S91>/trajGetMatch'
void MatlabControllerClass::xscal_j(int32_T n, const creal_T a, creal_T x_data[],
  int32_T ix0)
{
  int32_T b;
  int32_T k;
  real_T im;
  b = (ix0 + n) - 1;
  for (k = ix0; k <= b; k++) {
    im = x_data[k - 1].im * a.re + x_data[k - 1].re * a.im;
    x_data[k - 1].re = x_data[k - 1].re * a.re - x_data[k - 1].im * a.im;
    x_data[k - 1].im = im;
  }
}

// Function for MATLAB Function: '<S91>/trajGetMatch'
creal_T MatlabControllerClass::xzlarfg_e(creal_T *alpha1, creal_T *x)
{
  creal_T tau;
  real_T xnorm;
  int32_T knt;
  int32_T k;
  creal_T alpha1_0;
  real_T ar;
  tau.re = 0.0;
  tau.im = 0.0;
  xnorm = rt_hypotd_snf(x->re, x->im);
  if ((xnorm != 0.0) || (alpha1->im != 0.0)) {
    xnorm = xdlapy3(alpha1->re, alpha1->im, xnorm);
    if (alpha1->re >= 0.0) {
      xnorm = -xnorm;
    }

    if (std::abs(xnorm) < 1.0020841800044864E-292) {
      knt = -1;
      do {
        knt++;
        x->re *= 9.9792015476736E+291;
        x->im *= 9.9792015476736E+291;
        xnorm *= 9.9792015476736E+291;
        alpha1->re *= 9.9792015476736E+291;
        alpha1->im *= 9.9792015476736E+291;
      } while (!(std::abs(xnorm) >= 1.0020841800044864E-292));

      xnorm = xdlapy3(alpha1->re, alpha1->im, rt_hypotd_snf(x->re, x->im));
      if (alpha1->re >= 0.0) {
        xnorm = -xnorm;
      }

      ar = xnorm - alpha1->re;
      if (0.0 - alpha1->im == 0.0) {
        tau.re = ar / xnorm;
        tau.im = 0.0;
      } else if (ar == 0.0) {
        tau.re = 0.0;
        tau.im = (0.0 - alpha1->im) / xnorm;
      } else {
        tau.re = ar / xnorm;
        tau.im = (0.0 - alpha1->im) / xnorm;
      }

      alpha1_0.re = alpha1->re - xnorm;
      alpha1_0.im = alpha1->im;
      *alpha1 = recip(alpha1_0);
      ar = alpha1->re * x->im + alpha1->im * x->re;
      x->re = alpha1->re * x->re - alpha1->im * x->im;
      x->im = ar;
      for (k = 0; k <= knt; k++) {
        xnorm *= 1.0020841800044864E-292;
      }

      alpha1->re = xnorm;
      alpha1->im = 0.0;
    } else {
      ar = xnorm - alpha1->re;
      if (0.0 - alpha1->im == 0.0) {
        tau.re = ar / xnorm;
        tau.im = 0.0;
      } else if (ar == 0.0) {
        tau.re = 0.0;
        tau.im = (0.0 - alpha1->im) / xnorm;
      } else {
        tau.re = ar / xnorm;
        tau.im = (0.0 - alpha1->im) / xnorm;
      }

      alpha1_0.re = alpha1->re - xnorm;
      alpha1_0.im = alpha1->im;
      *alpha1 = recip(alpha1_0);
      ar = alpha1->re * x->im + alpha1->im * x->re;
      x->re = alpha1->re * x->re - alpha1->im * x->im;
      x->im = ar;
      alpha1->re = xnorm;
      alpha1->im = 0.0;
    }
  }

  return tau;
}

// Function for MATLAB Function: '<S91>/trajGetMatch'
int32_T MatlabControllerClass::eml_zlahqr(creal_T h_data[], int32_T h_size[2])
{
  int32_T info;
  int32_T n;
  int32_T itmax;
  int32_T ldh;
  int32_T i;
  real_T SMLNUM;
  int32_T L;
  boolean_T goto140;
  int32_T k;
  real_T tst;
  real_T htmp1;
  real_T ab;
  real_T ba;
  real_T aa;
  creal_T t;
  creal_T u2;
  boolean_T goto70;
  int32_T m;
  int32_T its;
  int32_T b_k;
  int32_T c_j;
  creal_T x;
  creal_T u2_0;
  creal_T u2_1;
  real_T brm;
  real_T v_idx_0_re;
  real_T v_idx_0_im;
  real_T v_idx_1_re;
  real_T v_idx_1_im;
  int32_T u0;
  int32_T u2_tmp;
  int32_T v_idx_0_re_tmp;
  boolean_T exitg1;
  boolean_T exitg2;
  boolean_T exitg3;
  n = h_size[0];
  if (10.0 > h_size[0]) {
    ab = 10.0;
  } else {
    ab = h_size[0];
  }

  itmax = 30 * (int32_T)ab;
  ldh = h_size[0];
  info = 0;
  if (1 != h_size[0]) {
    for (i = 0; i <= n - 4; i++) {
      u0 = i + h_size[0] * i;
      u2_tmp = u0 + 2;
      h_data[u2_tmp].re = 0.0;
      h_data[u2_tmp].im = 0.0;
      u0 += 3;
      h_data[u0].re = 0.0;
      h_data[u0].im = 0.0;
    }

    if (1 <= h_size[0] - 2) {
      u0 = (h_size[0] + h_size[0] * (h_size[0] - 3)) - 1;
      h_data[u0].re = 0.0;
      h_data[u0].im = 0.0;
    }

    for (i = 1; i < n; i++) {
      if (h_data[(i - 1) * h_size[0] + i].im != 0.0) {
        SMLNUM = h_data[(i - 1) * h_size[0] + i].re;
        ba = h_data[(i - 1) * h_size[0] + i].im;
        tst = std::abs(h_data[(i - 1) * h_size[0] + i].re) + std::abs(h_data[(i
          - 1) * h_size[0] + i].im);
        if (ba == 0.0) {
          u2.re = SMLNUM / tst;
          u2.im = 0.0;
        } else if (SMLNUM == 0.0) {
          u2.re = 0.0;
          u2.im = ba / tst;
        } else {
          u2.re = SMLNUM / tst;
          u2.im = ba / tst;
        }

        tst = rt_hypotd_snf(u2.re, u2.im);
        if (-u2.im == 0.0) {
          ba = u2.re / tst;
          htmp1 = 0.0;
        } else if (u2.re == 0.0) {
          ba = 0.0;
          htmp1 = -u2.im / tst;
        } else {
          ba = u2.re / tst;
          htmp1 = -u2.im / tst;
        }

        u0 = i + h_size[0] * (i - 1);
        h_data[u0].re = rt_hypotd_snf(h_data[(i - 1) * h_size[0] + i].re,
          h_data[(i - 1) * h_size[0] + i].im);
        h_data[u0].im = 0.0;
        m = i * ldh;
        L = m + i;
        its = (((n - i) - 1) * ldh + L) + 1;
        k = L;
        while ((ldh > 0) && (k + 1 <= its)) {
          ab = ba * h_data[k].im + htmp1 * h_data[k].re;
          h_data[k].re = ba * h_data[k].re - htmp1 * h_data[k].im;
          h_data[k].im = ab;
          k += ldh;
        }

        its = m;
        v_idx_0_re_tmp = i + 2;
        if (n < v_idx_0_re_tmp) {
          v_idx_0_re_tmp = n;
        }

        k = m + v_idx_0_re_tmp;
        while (its + 1 <= k) {
          ab = ba * h_data[its].im + -htmp1 * h_data[its].re;
          h_data[its].re = ba * h_data[its].re - -htmp1 * h_data[its].im;
          h_data[its].im = ab;
          its++;
        }
      }
    }

    SMLNUM = (real_T)h_size[0] / 2.2204460492503131E-16 *
      2.2250738585072014E-308;
    i = h_size[0] - 1;
    exitg1 = false;
    while ((!exitg1) && (i + 1 >= 1)) {
      L = -1;
      goto140 = false;
      its = 0;
      exitg2 = false;
      while ((!exitg2) && (its <= itmax)) {
        k = i;
        exitg3 = false;
        while ((!exitg3) && (k + 1 > L + 2)) {
          u0 = (k - 1) * h_size[0] + k;
          ab = std::abs(h_data[u0].re);
          ba = ab + std::abs(h_data[(k - 1) * h_size[0] + k].im);
          if (ba <= SMLNUM) {
            exitg3 = true;
          } else {
            m = h_size[0] * k + k;
            htmp1 = std::abs(h_data[m].re) + std::abs(h_data[h_size[0] * k + k].
              im);
            tst = (std::abs(h_data[u0 - 1].re) + std::abs(h_data[((k - 1) *
                     h_size[0] + k) - 1].im)) + htmp1;
            if (tst == 0.0) {
              if (k - 1 >= 1) {
                tst = std::abs(h_data[((k - 2) * h_size[0] + k) - 1].re);
              }

              if (k + 2 <= n) {
                tst += std::abs(h_data[m + 1].re);
              }
            }

            if (ab <= 2.2204460492503131E-16 * tst) {
              tst = std::abs(h_data[m - 1].re) + std::abs(h_data[(h_size[0] * k
                + k) - 1].im);
              if (ba > tst) {
                ab = ba;
                ba = tst;
              } else {
                ab = tst;
              }

              tst = std::abs(h_data[((k - 1) * h_size[0] + k) - 1].re -
                             h_data[h_size[0] * k + k].re) + std::abs(h_data[((k
                - 1) * h_size[0] + k) - 1].im - h_data[h_size[0] * k + k].im);
              if (htmp1 > tst) {
                aa = htmp1;
                htmp1 = tst;
              } else {
                aa = tst;
              }

              tst = aa + ab;
              htmp1 = aa / tst * htmp1 * 2.2204460492503131E-16;
              if ((SMLNUM > htmp1) || rtIsNaN(htmp1)) {
                htmp1 = SMLNUM;
              }

              if (ab / tst * ba <= htmp1) {
                exitg3 = true;
              } else {
                k--;
              }
            } else {
              k--;
            }
          }
        }

        L = k - 1;
        if (k + 1 > 1) {
          u0 = k + h_size[0] * (k - 1);
          h_data[u0].re = 0.0;
          h_data[u0].im = 0.0;
        }

        if (k + 1 >= i + 1) {
          goto140 = true;
          exitg2 = true;
        } else {
          switch (its) {
           case 10:
            t.re = std::abs(h_data[(h_size[0] * k + k) + 1].re) * 0.75 +
              h_data[h_size[0] * k + k].re;
            t.im = h_data[h_size[0] * k + k].im;
            break;

           case 20:
            t.re = std::abs(h_data[(i - 1) * h_size[0] + i].re) * 0.75 +
              h_data[h_size[0] * i + i].re;
            t.im = h_data[h_size[0] * i + i].im;
            break;

           default:
            m = h_size[0] * i + i;
            t = h_data[m];
            u2 = h_data[m - 1];
            sqrt_j(&u2);
            m = (i - 1) * h_size[0] + i;
            x = h_data[m];
            sqrt_j(&x);
            ba = u2.re * x.re - u2.im * x.im;
            ab = u2.re * x.im + u2.im * x.re;
            tst = std::abs(ba) + std::abs(ab);
            if (tst != 0.0) {
              x.re = (h_data[m - 1].re - h_data[h_size[0] * i + i].re) * 0.5;
              x.im = (h_data[((i - 1) * h_size[0] + i) - 1].im - h_data[h_size[0]
                      * i + i].im) * 0.5;
              aa = std::abs(x.re) + std::abs(x.im);
              if ((!(tst > aa)) && (!rtIsNaN(aa))) {
                tst = aa;
              }

              if (x.im == 0.0) {
                t.re = x.re / tst;
                t.im = 0.0;
              } else if (x.re == 0.0) {
                t.re = 0.0;
                t.im = x.im / tst;
              } else {
                t.re = x.re / tst;
                t.im = x.im / tst;
              }

              if (ab == 0.0) {
                u2.re = ba / tst;
                u2.im = 0.0;
              } else if (ba == 0.0) {
                u2.re = 0.0;
                u2.im = ab / tst;
              } else {
                u2.re = ba / tst;
                u2.im = ab / tst;
              }

              htmp1 = u2.re * u2.im + u2.im * u2.re;
              u2.re = (t.re * t.re - t.im * t.im) + (u2.re * u2.re - u2.im *
                u2.im);
              u2.im = (t.re * t.im + t.im * t.re) + htmp1;
              sqrt_j(&u2);
              u2.re *= tst;
              u2.im *= tst;
              if (aa > 0.0) {
                if (x.im == 0.0) {
                  t.re = x.re / aa;
                  t.im = 0.0;
                } else if (x.re == 0.0) {
                  t.re = 0.0;
                  t.im = x.im / aa;
                } else {
                  t.re = x.re / aa;
                  t.im = x.im / aa;
                }

                if (t.re * u2.re + t.im * u2.im < 0.0) {
                  u2.re = -u2.re;
                  u2.im = -u2.im;
                }
              }

              tst = x.re + u2.re;
              aa = x.im + u2.im;
              if (aa == 0.0) {
                if (ab == 0.0) {
                  htmp1 = ba / tst;
                  tst = 0.0;
                } else if (ba == 0.0) {
                  htmp1 = 0.0;
                  tst = ab / tst;
                } else {
                  htmp1 = ba / tst;
                  tst = ab / tst;
                }
              } else if (tst == 0.0) {
                if (ba == 0.0) {
                  htmp1 = ab / aa;
                  tst = 0.0;
                } else if (ab == 0.0) {
                  htmp1 = 0.0;
                  tst = -(ba / aa);
                } else {
                  htmp1 = ab / aa;
                  tst = -(ba / aa);
                }
              } else {
                brm = std::abs(tst);
                htmp1 = std::abs(aa);
                if (brm > htmp1) {
                  brm = aa / tst;
                  tst += brm * aa;
                  htmp1 = (brm * ab + ba) / tst;
                  tst = (ab - brm * ba) / tst;
                } else if (htmp1 == brm) {
                  tst = tst > 0.0 ? 0.5 : -0.5;
                  aa = aa > 0.0 ? 0.5 : -0.5;
                  htmp1 = (ba * tst + ab * aa) / brm;
                  tst = (ab * tst - ba * aa) / brm;
                } else {
                  brm = tst / aa;
                  tst = brm * tst + aa;
                  htmp1 = (brm * ba + ab) / tst;
                  tst = (brm * ab - ba) / tst;
                }
              }

              t.re = h_data[h_size[0] * i + i].re - (ba * htmp1 - ab * tst);
              t.im = h_data[h_size[0] * i + i].im - (ba * tst + ab * htmp1);
            }
            break;
          }

          goto70 = false;
          m = i;
          exitg3 = false;
          while ((!exitg3) && (m > k + 1)) {
            u2_tmp = (m - 1) * h_size[0] + m;
            u2.re = h_data[u2_tmp - 1].re - t.re;
            u2.im = h_data[((m - 1) * h_size[0] + m) - 1].im - t.im;
            tst = (std::abs(u2.re) + std::abs(u2.im)) + std::abs(h_data[u2_tmp].
              re);
            if (u2.im == 0.0) {
              v_idx_0_re = u2.re / tst;
              v_idx_0_im = 0.0;
            } else if (u2.re == 0.0) {
              v_idx_0_re = 0.0;
              v_idx_0_im = u2.im / tst;
            } else {
              v_idx_0_re = u2.re / tst;
              v_idx_0_im = u2.im / tst;
            }

            htmp1 = h_data[u2_tmp].re / tst;
            v_idx_1_re = htmp1;
            v_idx_1_im = 0.0;
            if (std::abs(h_data[((m - 2) * h_size[0] + m) - 1].re) * std::abs
                (htmp1) <= ((std::abs(h_data[((m - 1) * h_size[0] + m) - 1].re)
                             + std::abs(h_data[((m - 1) * h_size[0] + m) - 1].im))
                            + (std::abs(h_data[h_size[0] * m + m].re) + std::abs
                               (h_data[h_size[0] * m + m].im))) * (std::abs
                 (v_idx_0_re) + std::abs(v_idx_0_im)) * 2.2204460492503131E-16)
            {
              goto70 = true;
              exitg3 = true;
            } else {
              m--;
            }
          }

          if (!goto70) {
            u2.re = h_data[h_size[0] * k + k].re - t.re;
            u2.im = h_data[h_size[0] * k + k].im - t.im;
            htmp1 = h_data[(h_size[0] * k + k) + 1].re;
            tst = (std::abs(u2.re) + std::abs(u2.im)) + std::abs(htmp1);
            if (u2.im == 0.0) {
              v_idx_0_re = u2.re / tst;
              v_idx_0_im = 0.0;
            } else if (u2.re == 0.0) {
              v_idx_0_re = 0.0;
              v_idx_0_im = u2.im / tst;
            } else {
              v_idx_0_re = u2.re / tst;
              v_idx_0_im = u2.im / tst;
            }

            v_idx_1_re = htmp1 / tst;
            v_idx_1_im = 0.0;
          }

          for (b_k = m; b_k <= i; b_k++) {
            if (b_k > m) {
              v_idx_0_re_tmp = (b_k - 2) * h_size[0] + b_k;
              v_idx_0_re = h_data[v_idx_0_re_tmp - 1].re;
              v_idx_0_im = h_data[((b_k - 2) * h_size[0] + b_k) - 1].im;
              v_idx_1_re = h_data[v_idx_0_re_tmp].re;
              v_idx_1_im = h_data[(b_k - 2) * h_size[0] + b_k].im;
            }

            u2.re = v_idx_0_re;
            u2.im = v_idx_0_im;
            x.re = v_idx_1_re;
            x.im = v_idx_1_im;
            t = xzlarfg_e(&u2, &x);
            v_idx_0_re = u2.re;
            v_idx_0_im = u2.im;
            v_idx_1_re = x.re;
            v_idx_1_im = x.im;
            if (b_k > m) {
              u0 = b_k + h_size[0] * (b_k - 2);
              h_data[u0 - 1] = u2;
              h_data[u0].re = 0.0;
              h_data[u0].im = 0.0;
            }

            ba = t.re * x.re - t.im * x.im;
            for (v_idx_0_re_tmp = b_k - 1; v_idx_0_re_tmp < n; v_idx_0_re_tmp++)
            {
              u0 = h_size[0] * v_idx_0_re_tmp;
              u2_tmp = u0 + b_k;
              u2.re = (h_data[u2_tmp - 1].re * t.re - h_data[(h_size[0] *
                        v_idx_0_re_tmp + b_k) - 1].im * -t.im) + h_data[u2_tmp].
                re * ba;
              u2.im = (h_data[(h_size[0] * v_idx_0_re_tmp + b_k) - 1].im * t.re
                       + h_data[(h_size[0] * v_idx_0_re_tmp + b_k) - 1].re *
                       -t.im) + h_data[h_size[0] * v_idx_0_re_tmp + b_k].im * ba;
              u0 += b_k;
              u2_tmp = u0 - 1;
              h_data[u2_tmp].re = h_data[(h_size[0] * v_idx_0_re_tmp + b_k) - 1]
                .re - u2.re;
              h_data[u2_tmp].im = h_data[(h_size[0] * v_idx_0_re_tmp + b_k) - 1]
                .im - u2.im;
              h_data[u0].re = h_data[h_size[0] * v_idx_0_re_tmp + b_k].re -
                (u2.re * x.re - u2.im * x.im);
              h_data[u0].im = h_data[h_size[0] * v_idx_0_re_tmp + b_k].im -
                (u2.re * x.im + u2.im * x.re);
            }

            u0 = b_k + 2;
            v_idx_0_re_tmp = i + 1;
            if (u0 < v_idx_0_re_tmp) {
              v_idx_0_re_tmp = u0;
            }

            for (c_j = 0; c_j < v_idx_0_re_tmp; c_j++) {
              u2_tmp = h_size[0] * b_k;
              u2.re = (h_data[(b_k - 1) * h_size[0] + c_j].re * t.re - h_data
                       [(b_k - 1) * h_size[0] + c_j].im * t.im) + h_data[u2_tmp
                + c_j].re * ba;
              u2.im = (h_data[(b_k - 1) * h_size[0] + c_j].im * t.re + h_data
                       [(b_k - 1) * h_size[0] + c_j].re * t.im) + h_data[h_size
                [0] * b_k + c_j].im * ba;
              u0 = c_j + h_size[0] * (b_k - 1);
              h_data[u0].re = h_data[(b_k - 1) * h_size[0] + c_j].re - u2.re;
              h_data[u0].im = h_data[(b_k - 1) * h_size[0] + c_j].im - u2.im;
              u0 = c_j + u2_tmp;
              h_data[u0].re = h_data[h_size[0] * b_k + c_j].re - (u2.re * x.re -
                u2.im * -x.im);
              h_data[u0].im = h_data[h_size[0] * b_k + c_j].im - (u2.re * -x.im
                + u2.im * x.re);
            }

            if ((b_k == m) && (m > k + 1)) {
              tst = rt_hypotd_snf(1.0 - t.re, 0.0 - t.im);
              if (0.0 - t.im == 0.0) {
                ba = (1.0 - t.re) / tst;
                htmp1 = 0.0;
              } else if (1.0 - t.re == 0.0) {
                ba = 0.0;
                htmp1 = (0.0 - t.im) / tst;
              } else {
                ba = (1.0 - t.re) / tst;
                htmp1 = (0.0 - t.im) / tst;
              }

              u2.re = ba;
              u2.im = htmp1;
              ab = h_data[(m - 1) * h_size[0] + m].re * -htmp1 + h_data[(m - 1) *
                h_size[0] + m].im * ba;
              u0 = m + h_size[0] * (m - 1);
              h_data[u0].re = h_data[(m - 1) * h_size[0] + m].re * ba - h_data
                [(m - 1) * h_size[0] + m].im * -htmp1;
              h_data[u0].im = ab;
              if (m + 2 <= i + 1) {
                ab = h_data[(h_size[0] * m + m) + 1].re * htmp1 + h_data
                  [(h_size[0] * m + m) + 1].im * ba;
                u0 = (m + h_size[0] * m) + 1;
                h_data[u0].re = h_data[(h_size[0] * m + m) + 1].re * ba -
                  h_data[(h_size[0] * m + m) + 1].im * htmp1;
                h_data[u0].im = ab;
              }

              for (v_idx_0_re_tmp = m; v_idx_0_re_tmp <= i + 1; v_idx_0_re_tmp++)
              {
                if (m + 1 != v_idx_0_re_tmp) {
                  if (n > v_idx_0_re_tmp) {
                    xscal_jo(n - v_idx_0_re_tmp, u2, h_data, v_idx_0_re_tmp +
                             v_idx_0_re_tmp * ldh, ldh);
                  }

                  u2_1.re = ba;
                  u2_1.im = -htmp1;
                  xscal_j(v_idx_0_re_tmp - 1, u2_1, h_data, 1 + (v_idx_0_re_tmp
                           - 1) * ldh);
                }
              }
            }
          }

          u2 = h_data[(i - 1) * h_size[0] + i];
          if (h_data[(i - 1) * h_size[0] + i].im != 0.0) {
            tst = rt_hypotd_snf(h_data[(i - 1) * h_size[0] + i].re, h_data[(i -
              1) * h_size[0] + i].im);
            u0 = i + h_size[0] * (i - 1);
            h_data[u0].re = tst;
            h_data[u0].im = 0.0;
            if (u2.im == 0.0) {
              ba = u2.re / tst;
              htmp1 = 0.0;
            } else if (u2.re == 0.0) {
              ba = 0.0;
              htmp1 = u2.im / tst;
            } else {
              ba = u2.re / tst;
              htmp1 = u2.im / tst;
            }

            u2.re = ba;
            u2.im = htmp1;
            if (n > i + 1) {
              u2_0.re = ba;
              u2_0.im = -htmp1;
              xscal_jo((n - i) - 1, u2_0, h_data, (i + (i + 1) * ldh) + 1, ldh);
            }

            xscal_j(i, u2, h_data, 1 + i * ldh);
          }

          its++;
        }
      }

      if (!goto140) {
        info = i + 1;
        exitg1 = true;
      } else {
        i = L;
      }
    }
  }

  return info;
}

// Function for MATLAB Function: '<S91>/trajGetMatch'
int32_T MatlabControllerClass::xhseqr(creal_T h_data[], int32_T h_size[2])
{
  int32_T info;
  int32_T istart;
  int32_T jend;
  int32_T j;
  int32_T i;
  int32_T loop_ub;
  int32_T b_h_size_idx_0;
  int32_T b_h_size_idx_1;
  info = eml_zlahqr(h_data, h_size);
  b_h_size_idx_0 = h_size[0];
  b_h_size_idx_1 = h_size[1];
  loop_ub = h_size[0] * h_size[1] - 1;
  if (0 <= loop_ub) {
    memcpy(&rtDW.b_h_data[0], &h_data[0], (loop_ub + 1) * sizeof(creal_T));
  }

  if (3 < h_size[0]) {
    istart = 4;
    if (h_size[0] - 4 < h_size[1] - 1) {
      jend = h_size[0] - 4;
    } else {
      jend = h_size[1] - 1;
    }

    for (j = 0; j <= jend; j++) {
      for (i = istart; i <= h_size[0]; i++) {
        loop_ub = (i + b_h_size_idx_0 * j) - 1;
        rtDW.b_h_data[loop_ub].re = 0.0;
        rtDW.b_h_data[loop_ub].im = 0.0;
      }

      istart++;
    }
  }

  for (loop_ub = 0; loop_ub < b_h_size_idx_1; loop_ub++) {
    for (istart = 0; istart < b_h_size_idx_0; istart++) {
      h_data[istart + h_size[0] * loop_ub] = rtDW.b_h_data[b_h_size_idx_0 *
        loop_ub + istart];
    }
  }

  return info;
}

// Function for MATLAB Function: '<S91>/trajGetMatch'
void MatlabControllerClass::triu(creal_T x_data[], int32_T x_size[2])
{
  int32_T istart;
  int32_T jend;
  int32_T j;
  int32_T i;
  int32_T tmp;
  if (1 < x_size[0]) {
    istart = 2;
    if (x_size[0] - 2 < x_size[1] - 1) {
      jend = x_size[0] - 2;
    } else {
      jend = x_size[1] - 1;
    }

    for (j = 0; j <= jend; j++) {
      for (i = istart; i <= x_size[0]; i++) {
        tmp = (i + x_size[0] * j) - 1;
        x_data[tmp].re = 0.0;
        x_data[tmp].im = 0.0;
      }

      istart++;
    }
  }
}

// Function for MATLAB Function: '<S91>/trajGetMatch'
void MatlabControllerClass::schur(creal_T A_data[], int32_T A_size[2], creal_T
  V_data[], int32_T V_size[2])
{
  int32_T loop_ub;
  int32_T i;
  int8_T b_idx_0;
  int8_T b_idx_1;
  if (anyNonFinite(A_data, A_size)) {
    b_idx_0 = (int8_T)A_size[0];
    b_idx_1 = (int8_T)A_size[1];
    V_size[0] = b_idx_0;
    V_size[1] = b_idx_1;
    loop_ub = b_idx_0 * b_idx_1 - 1;
    for (i = 0; i <= loop_ub; i++) {
      V_data[i].re = (rtNaN);
      V_data[i].im = 0.0;
    }

    triu(V_data, V_size);
  } else {
    xgehrd(A_data, A_size);
    V_size[0] = A_size[0];
    V_size[1] = A_size[1];
    loop_ub = A_size[0] * A_size[1] - 1;
    if (0 <= loop_ub) {
      memcpy(&V_data[0], &A_data[0], (loop_ub + 1) * sizeof(creal_T));
    }

    xhseqr(V_data, V_size);
  }
}

// Function for MATLAB Function: '<S91>/trajGetMatch'
void MatlabControllerClass::mainDiagZeroImag(const creal_T D_data[], const
  int32_T D_size[2], creal_T d_data[], int32_T *d_size)
{
  int32_T k;
  *d_size = D_size[0];
  for (k = 0; k < D_size[0]; k++) {
    d_data[k] = D_data[D_size[0] * k + k];
  }
}

// Function for MATLAB Function: '<S91>/trajGetMatch'
void MatlabControllerClass::eig(const creal_T A_data[], const int32_T A_size[2],
  creal_T V_data[], int32_T *V_size)
{
  int32_T info;
  creal_T beta1_data[17];
  int32_T beta1_size;
  int32_T T_size[2];
  int32_T A_size_0[2];
  real_T brm;
  real_T bim;
  real_T sgnbi;
  real_T d;
  creal_T V_data_0;
  if (anyNonFinite(A_data, A_size)) {
    if ((A_size[0] == 1) && (A_size[1] == 1)) {
      *V_size = 1;
      V_data[0].re = (rtNaN);
      V_data[0].im = 0.0;
    } else {
      *V_size = A_size[0];
      info = A_size[0];
      for (beta1_size = 0; beta1_size < info; beta1_size++) {
        V_data[beta1_size].re = (rtNaN);
        V_data[beta1_size].im = 0.0;
      }
    }
  } else if ((A_size[0] == 1) && (A_size[1] == 1)) {
    *V_size = 1;
    V_data[0] = A_data[0];
  } else if (ishermitian(A_data, A_size)) {
    A_size_0[0] = A_size[0];
    A_size_0[1] = A_size[1];
    info = A_size[0] * A_size[1];
    if (0 <= info - 1) {
      memcpy(&rtDW.A_data[0], &A_data[0], info * sizeof(creal_T));
    }

    schur(rtDW.A_data, A_size_0, rtDW.T_data, T_size);
    mainDiagZeroImag(rtDW.T_data, T_size, V_data, V_size);
  } else {
    xzgeev(A_data, A_size, &info, V_data, V_size, beta1_data, &beta1_size);
    info = *V_size;
    for (beta1_size = 0; beta1_size < info; beta1_size++) {
      if (beta1_data[beta1_size].im == 0.0) {
        if (V_data[beta1_size].im == 0.0) {
          bim = V_data[beta1_size].re / beta1_data[beta1_size].re;
          brm = 0.0;
        } else if (V_data[beta1_size].re == 0.0) {
          bim = 0.0;
          brm = V_data[beta1_size].im / beta1_data[beta1_size].re;
        } else {
          bim = V_data[beta1_size].re / beta1_data[beta1_size].re;
          brm = V_data[beta1_size].im / beta1_data[beta1_size].re;
        }
      } else if (beta1_data[beta1_size].re == 0.0) {
        if (V_data[beta1_size].re == 0.0) {
          bim = V_data[beta1_size].im / beta1_data[beta1_size].im;
          brm = 0.0;
        } else if (V_data[beta1_size].im == 0.0) {
          bim = 0.0;
          brm = -(V_data[beta1_size].re / beta1_data[beta1_size].im);
        } else {
          bim = V_data[beta1_size].im / beta1_data[beta1_size].im;
          brm = -(V_data[beta1_size].re / beta1_data[beta1_size].im);
        }
      } else {
        brm = std::abs(beta1_data[beta1_size].re);
        bim = std::abs(beta1_data[beta1_size].im);
        if (brm > bim) {
          brm = beta1_data[beta1_size].im / beta1_data[beta1_size].re;
          d = brm * beta1_data[beta1_size].im + beta1_data[beta1_size].re;
          bim = (brm * V_data[beta1_size].im + V_data[beta1_size].re) / d;
          brm = (V_data[beta1_size].im - brm * V_data[beta1_size].re) / d;
        } else if (bim == brm) {
          d = beta1_data[beta1_size].re > 0.0 ? 0.5 : -0.5;
          sgnbi = beta1_data[beta1_size].im > 0.0 ? 0.5 : -0.5;
          bim = (V_data[beta1_size].re * d + V_data[beta1_size].im * sgnbi) /
            brm;
          brm = (V_data[beta1_size].im * d - V_data[beta1_size].re * sgnbi) /
            brm;
        } else {
          brm = beta1_data[beta1_size].re / beta1_data[beta1_size].im;
          d = brm * beta1_data[beta1_size].re + beta1_data[beta1_size].im;
          bim = (brm * V_data[beta1_size].re + V_data[beta1_size].im) / d;
          brm = (brm * V_data[beta1_size].im - V_data[beta1_size].re) / d;
        }
      }

      V_data_0.re = bim;
      V_data_0.im = brm;
      V_data[beta1_size] = V_data_0;
    }
  }
}

// Function for MATLAB Function: '<S91>/trajGetMatch'
void MatlabControllerClass::roots(const real_T c[18], creal_T r_data[], int32_T *
  r_size)
{
  int32_T k1;
  int32_T k2;
  int32_T companDim;
  real_T ctmp[18];
  int32_T j;
  creal_T eiga_data[17];
  int32_T a_size[2];
  boolean_T exitg1;
  boolean_T exitg2;
  memset(&r_data[0], 0, 17U * sizeof(creal_T));
  k1 = 1;
  while ((k1 <= 18) && (!(c[k1 - 1] != 0.0))) {
    k1++;
  }

  k2 = 18;
  while ((k2 >= k1) && (!(c[k2 - 1] != 0.0))) {
    k2--;
  }

  if (k1 < k2) {
    companDim = k2 - k1;
    exitg1 = false;
    while ((!exitg1) && (companDim > 0)) {
      j = 0;
      exitg2 = false;
      while ((!exitg2) && (j + 1 <= companDim)) {
        ctmp[j] = c[k1 + j] / c[k1 - 1];
        if (rtIsInf(std::abs(ctmp[j]))) {
          exitg2 = true;
        } else {
          j++;
        }
      }

      if (j + 1 > companDim) {
        exitg1 = true;
      } else {
        k1++;
        companDim--;
      }
    }

    if (companDim < 1) {
      if (1 > 18 - k2) {
        *r_size = 0;
      } else {
        *r_size = 18 - k2;
      }
    } else {
      a_size[0] = companDim;
      a_size[1] = companDim;
      k1 = companDim * companDim - 1;
      if (0 <= k1) {
        memset(&rtDW.a_data[0], 0, (k1 + 1) * sizeof(creal_T));
      }

      for (k1 = 0; k1 <= companDim - 2; k1++) {
        j = companDim * k1;
        rtDW.a_data[j].re = -ctmp[k1];
        rtDW.a_data[j].im = 0.0;
        j = (k1 + j) + 1;
        rtDW.a_data[j].re = 1.0;
        rtDW.a_data[j].im = 0.0;
      }

      j = companDim * (companDim - 1);
      rtDW.a_data[j].re = -ctmp[companDim - 1];
      rtDW.a_data[j].im = 0.0;
      for (k1 = 0; k1 <= 17 - k2; k1++) {
        r_data[k1].re = 0.0;
        r_data[k1].im = 0.0;
      }

      eig(rtDW.a_data, a_size, eiga_data, &k1);
      for (k1 = 0; k1 < companDim; k1++) {
        r_data[(k1 - k2) + 18] = eiga_data[k1];
      }

      *r_size = (companDim - k2) + 18;
    }
  } else if (1 > 18 - k2) {
    *r_size = 0;
  } else {
    *r_size = 18 - k2;
  }
}

// Function for MATLAB Function: '<S91>/trajGetMatch'
void MatlabControllerClass::polyval_k(const real_T p[6], const real_T x_data[],
  const int32_T x_size[2], real_T y_data[], int32_T y_size[2])
{
  int32_T k;
  real_T p_0;
  int32_T i;
  int32_T loop_ub;
  loop_ub = (int8_T)x_size[1];
  for (i = 0; i < loop_ub; i++) {
    y_data[i] = p[0];
  }

  y_size[0] = 1;
  y_size[1] = x_size[1];
  loop_ub = x_size[0] * x_size[1] - 1;
  for (k = 0; k < 5; k++) {
    p_0 = p[k + 1];
    for (i = 0; i <= loop_ub; i++) {
      y_data[i] = x_data[i] * y_data[i] + p_0;
    }
  }
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

    y = atan2((real_T)u0_0, (real_T)u1_0);
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

// Function for MATLAB Function: '<S91>/trajGetMatch'
void MatlabControllerClass::trajGetMatch(const trajectoryBus *traj, const
  real32_T position[3], real_T *section_idx, real_T *error, real_T *t)
{
  real32_T xf;
  real32_T yf;
  real32_T zf;
  emxArray_real_T *section_errors;
  emxArray_real_T *section_t;
  real_T traj_degree;
  int32_T no_of_points;
  real_T P_0[18];
  creal_T ti_data[17];
  real_T T_data[11];
  real_T curr_pos_data[33];
  real_T error_array_data[11];
  int32_T k;
  int32_T i;
  real_T b_section_idx;
  real_T scale;
  real_T absxk;
  real_T b_t;
  real_T tmp_data[11];
  real_T tmp_data_0[11];
  real_T tmp_data_1[11];
  int32_T error_array_size[2];
  int32_T tmp_size[2];
  int32_T tmp_size_0[2];
  int32_T tmp_size_1[2];
  int32_T loop_ub;
  boolean_T exitg1;
  emxInit_real_T(&section_errors, 1);
  xf = position[0];
  yf = position[1];
  zf = position[2];
  i = section_errors->size[0];
  loop_ub = (int32_T)((traj->num_sections_set - 1.0) + 1.0);
  section_errors->size[0] = loop_ub;
  emxEnsureCapacity_real_T(section_errors, i);
  for (i = 0; i < loop_ub; i++) {
    section_errors->data[i] = 0.0;
  }

  emxInit_real_T(&section_t, 1);
  i = section_t->size[0];
  section_t->size[0] = loop_ub;
  emxEnsureCapacity_real_T(section_t, i);
  for (i = 0; i < loop_ub; i++) {
    section_t->data[i] = 0.0;
  }

  traj_degree = traj->polynomial_degree;
  for (k = 0; k < (int32_T)traj->num_sections_set; k++) {
    no_of_points = 0;
    memset(&P_0[0], 0, 18U * sizeof(real_T));
    b_section_idx = 1.0 + (real_T)k;
    if (1.0 + (real_T)k > traj->num_sections_set) {
      b_section_idx = 1.0;
    }

    if (traj_degree == 3.0) {
      no_of_points = 6;
      P_0[12] = (traj->sections[(int32_T)b_section_idx - 1].pos_x[0] *
                 traj->sections[(int32_T)b_section_idx - 1].pos_x[0] * 3.0 +
                 traj->sections[(int32_T)b_section_idx - 1].pos_y[0] *
                 traj->sections[(int32_T)b_section_idx - 1].pos_y[0] * 3.0) +
        traj->sections[(int32_T)b_section_idx - 1].pos_z[0] * traj->sections
        [(int32_T)b_section_idx - 1].pos_z[0] * 3.0;
      P_0[13] = (traj->sections[(int32_T)b_section_idx - 1].pos_x[0] * 5.0 *
                 traj->sections[(int32_T)b_section_idx - 1].pos_x[1] +
                 traj->sections[(int32_T)b_section_idx - 1].pos_y[0] * 5.0 *
                 traj->sections[(int32_T)b_section_idx - 1].pos_y[1]) +
        traj->sections[(int32_T)b_section_idx - 1].pos_z[0] * 5.0 *
        traj->sections[(int32_T)b_section_idx - 1].pos_z[1];
      P_0[14] = ((((traj->sections[(int32_T)b_section_idx - 1].pos_x[1] *
                    traj->sections[(int32_T)b_section_idx - 1].pos_x[1] * 2.0 +
                    traj->sections[(int32_T)b_section_idx - 1].pos_y[1] *
                    traj->sections[(int32_T)b_section_idx - 1].pos_y[1] * 2.0) +
                   traj->sections[(int32_T)b_section_idx - 1].pos_z[1] *
                   traj->sections[(int32_T)b_section_idx - 1].pos_z[1] * 2.0) +
                  traj->sections[(int32_T)b_section_idx - 1].pos_x[0] * 4.0 *
                  traj->sections[(int32_T)b_section_idx - 1].pos_x[2]) +
                 traj->sections[(int32_T)b_section_idx - 1].pos_y[0] * 4.0 *
                 traj->sections[(int32_T)b_section_idx - 1].pos_y[2]) +
        traj->sections[(int32_T)b_section_idx - 1].pos_z[0] * 4.0 *
        traj->sections[(int32_T)b_section_idx - 1].pos_z[2];
      P_0[15] = (((((((real32_T)(traj->sections[(int32_T)b_section_idx - 1].
        pos_x[0] * 3.0 * traj->sections[(int32_T)b_section_idx - 1].pos_x[3] +
        traj->sections[(int32_T)b_section_idx - 1].pos_x[1] * 3.0 *
        traj->sections[(int32_T)b_section_idx - 1].pos_x[2]) - (real32_T)
                      (traj->sections[(int32_T)b_section_idx - 1].pos_x[0] * 3.0)
                      * xf) + (real32_T)(traj->sections[(int32_T)b_section_idx -
        1].pos_y[0] * 3.0 * traj->sections[(int32_T)b_section_idx - 1].pos_y[3]))
                    + (real32_T)(traj->sections[(int32_T)b_section_idx - 1].
        pos_y[1] * 3.0 * traj->sections[(int32_T)b_section_idx - 1].pos_y[2])) -
                   (real32_T)(traj->sections[(int32_T)b_section_idx - 1].pos_y[0]
        * 3.0) * yf) + (real32_T)(traj->sections[(int32_T)b_section_idx - 1].
        pos_z[0] * 3.0 * traj->sections[(int32_T)b_section_idx - 1].pos_z[3])) +
                 (real32_T)(traj->sections[(int32_T)b_section_idx - 1].pos_z[1] *
                            3.0 * traj->sections[(int32_T)b_section_idx - 1].
                            pos_z[2])) - (real32_T)(traj->sections[(int32_T)
        b_section_idx - 1].pos_z[0] * 3.0) * zf;
      P_0[16] = (((((real32_T)(((traj->sections[(int32_T)b_section_idx - 1].
        pos_x[2] * traj->sections[(int32_T)b_section_idx - 1].pos_x[2] +
        traj->sections[(int32_T)b_section_idx - 1].pos_y[2] * traj->sections
        [(int32_T)b_section_idx - 1].pos_y[2]) + traj->sections[(int32_T)
        b_section_idx - 1].pos_z[2] * traj->sections[(int32_T)b_section_idx - 1]
        .pos_z[2]) + traj->sections[(int32_T)b_section_idx - 1].pos_x[1] * 2.0 *
        traj->sections[(int32_T)b_section_idx - 1].pos_x[3]) - (real32_T)
                    (traj->sections[(int32_T)b_section_idx - 1].pos_x[1] * 2.0) *
                    xf) + (real32_T)(traj->sections[(int32_T)b_section_idx - 1].
        pos_y[1] * 2.0 * traj->sections[(int32_T)b_section_idx - 1].pos_y[3])) -
                  (real32_T)(traj->sections[(int32_T)b_section_idx - 1].pos_y[1]
        * 2.0) * yf) + (real32_T)(traj->sections[(int32_T)b_section_idx - 1].
                  pos_z[1] * 2.0 * traj->sections[(int32_T)b_section_idx - 1].
                  pos_z[3])) - (real32_T)(traj->sections[(int32_T)b_section_idx
        - 1].pos_z[1] * 2.0) * zf;
      P_0[17] = (((((real32_T)(traj->sections[(int32_T)b_section_idx - 1].pos_x
        [2] * traj->sections[(int32_T)b_section_idx - 1].pos_x[3]) - (real32_T)
                    traj->sections[(int32_T)b_section_idx - 1].pos_x[2] * xf) +
                   (real32_T)(traj->sections[(int32_T)b_section_idx - 1].pos_y[2]
        * traj->sections[(int32_T)b_section_idx - 1].pos_y[3])) - (real32_T)
                  traj->sections[(int32_T)b_section_idx - 1].pos_y[2] * yf) +
                 (real32_T)(traj->sections[(int32_T)b_section_idx - 1].pos_z[2] *
                            traj->sections[(int32_T)b_section_idx - 1].pos_z[3]))
        - (real32_T)traj->sections[(int32_T)b_section_idx - 1].pos_z[2] * zf;
      if (std::abs(P_0[12]) <= 2.2204460492503131E-16) {
        P_0[12] = 2.2204460492503131E-16;
      }
    }

    if (traj_degree == 5.0) {
      no_of_points = 10;
      P_0[8] = (traj->sections[(int32_T)b_section_idx - 1].pos_x[0] *
                traj->sections[(int32_T)b_section_idx - 1].pos_x[0] * 5.0 +
                traj->sections[(int32_T)b_section_idx - 1].pos_y[0] *
                traj->sections[(int32_T)b_section_idx - 1].pos_y[0] * 5.0) +
        traj->sections[(int32_T)b_section_idx - 1].pos_z[0] * traj->sections
        [(int32_T)b_section_idx - 1].pos_z[0] * 5.0;
      P_0[9] = (traj->sections[(int32_T)b_section_idx - 1].pos_x[0] * 9.0 *
                traj->sections[(int32_T)b_section_idx - 1].pos_x[1] +
                traj->sections[(int32_T)b_section_idx - 1].pos_y[0] * 9.0 *
                traj->sections[(int32_T)b_section_idx - 1].pos_y[1]) +
        traj->sections[(int32_T)b_section_idx - 1].pos_z[0] * 9.0 *
        traj->sections[(int32_T)b_section_idx - 1].pos_z[1];
      P_0[10] = ((((traj->sections[(int32_T)b_section_idx - 1].pos_x[1] *
                    traj->sections[(int32_T)b_section_idx - 1].pos_x[1] * 4.0 +
                    traj->sections[(int32_T)b_section_idx - 1].pos_y[1] *
                    traj->sections[(int32_T)b_section_idx - 1].pos_y[1] * 4.0) +
                   traj->sections[(int32_T)b_section_idx - 1].pos_z[1] *
                   traj->sections[(int32_T)b_section_idx - 1].pos_z[1] * 4.0) +
                  traj->sections[(int32_T)b_section_idx - 1].pos_x[0] * 8.0 *
                  traj->sections[(int32_T)b_section_idx - 1].pos_x[2]) +
                 traj->sections[(int32_T)b_section_idx - 1].pos_y[0] * 8.0 *
                 traj->sections[(int32_T)b_section_idx - 1].pos_y[2]) +
        traj->sections[(int32_T)b_section_idx - 1].pos_z[0] * 8.0 *
        traj->sections[(int32_T)b_section_idx - 1].pos_z[2];
      P_0[11] = ((((traj->sections[(int32_T)b_section_idx - 1].pos_x[0] * 7.0 *
                    traj->sections[(int32_T)b_section_idx - 1].pos_x[3] +
                    traj->sections[(int32_T)b_section_idx - 1].pos_x[1] * 7.0 *
                    traj->sections[(int32_T)b_section_idx - 1].pos_x[2]) +
                   traj->sections[(int32_T)b_section_idx - 1].pos_y[0] * 7.0 *
                   traj->sections[(int32_T)b_section_idx - 1].pos_y[3]) +
                  traj->sections[(int32_T)b_section_idx - 1].pos_y[1] * 7.0 *
                  traj->sections[(int32_T)b_section_idx - 1].pos_y[2]) +
                 traj->sections[(int32_T)b_section_idx - 1].pos_z[0] * 7.0 *
                 traj->sections[(int32_T)b_section_idx - 1].pos_z[3]) +
        traj->sections[(int32_T)b_section_idx - 1].pos_z[1] * 7.0 *
        traj->sections[(int32_T)b_section_idx - 1].pos_z[2];
      P_0[12] = (((((((traj->sections[(int32_T)b_section_idx - 1].pos_x[2] *
                       traj->sections[(int32_T)b_section_idx - 1].pos_x[2] * 3.0
                       + traj->sections[(int32_T)b_section_idx - 1].pos_y[2] *
                       traj->sections[(int32_T)b_section_idx - 1].pos_y[2] * 3.0)
                      + traj->sections[(int32_T)b_section_idx - 1].pos_z[2] *
                      traj->sections[(int32_T)b_section_idx - 1].pos_z[2] * 3.0)
                     + traj->sections[(int32_T)b_section_idx - 1].pos_x[0] * 6.0
                     * traj->sections[(int32_T)b_section_idx - 1].pos_x[4]) +
                    traj->sections[(int32_T)b_section_idx - 1].pos_x[1] * 6.0 *
                    traj->sections[(int32_T)b_section_idx - 1].pos_x[3]) +
                   traj->sections[(int32_T)b_section_idx - 1].pos_y[0] * 6.0 *
                   traj->sections[(int32_T)b_section_idx - 1].pos_y[4]) +
                  traj->sections[(int32_T)b_section_idx - 1].pos_y[1] * 6.0 *
                  traj->sections[(int32_T)b_section_idx - 1].pos_y[3]) +
                 traj->sections[(int32_T)b_section_idx - 1].pos_z[0] * 6.0 *
                 traj->sections[(int32_T)b_section_idx - 1].pos_z[4]) +
        traj->sections[(int32_T)b_section_idx - 1].pos_z[1] * 6.0 *
        traj->sections[(int32_T)b_section_idx - 1].pos_z[3];
      P_0[13] = (((((((((real32_T)((traj->sections[(int32_T)b_section_idx - 1].
        pos_x[0] * 5.0 * traj->sections[(int32_T)b_section_idx - 1].pos_x[5] +
        traj->sections[(int32_T)b_section_idx - 1].pos_x[1] * 5.0 *
        traj->sections[(int32_T)b_section_idx - 1].pos_x[4]) + traj->sections
        [(int32_T)b_section_idx - 1].pos_x[2] * 5.0 * traj->sections[(int32_T)
        b_section_idx - 1].pos_x[3]) - (real32_T)(traj->sections[(int32_T)
        b_section_idx - 1].pos_x[0] * 5.0) * xf) + (real32_T)(traj->sections
        [(int32_T)b_section_idx - 1].pos_y[0] * 5.0 * traj->sections[(int32_T)
        b_section_idx - 1].pos_y[5])) + (real32_T)(traj->sections[(int32_T)
        b_section_idx - 1].pos_y[1] * 5.0 * traj->sections[(int32_T)
        b_section_idx - 1].pos_y[4])) + (real32_T)(traj->sections[(int32_T)
        b_section_idx - 1].pos_y[2] * 5.0 * traj->sections[(int32_T)
        b_section_idx - 1].pos_y[3])) - (real32_T)(traj->sections[(int32_T)
        b_section_idx - 1].pos_y[0] * 5.0) * yf) + (real32_T)(traj->sections
        [(int32_T)b_section_idx - 1].pos_z[0] * 5.0 * traj->sections[(int32_T)
        b_section_idx - 1].pos_z[5])) + (real32_T)(traj->sections[(int32_T)
        b_section_idx - 1].pos_z[1] * 5.0 * traj->sections[(int32_T)
        b_section_idx - 1].pos_z[4])) + (real32_T)(traj->sections[(int32_T)
                  b_section_idx - 1].pos_z[2] * 5.0 * traj->sections[(int32_T)
                  b_section_idx - 1].pos_z[3])) - (real32_T)(traj->sections
        [(int32_T)b_section_idx - 1].pos_z[0] * 5.0) * zf;
      P_0[14] = (((((((real32_T)((((traj->sections[(int32_T)b_section_idx - 1].
        pos_x[3] * traj->sections[(int32_T)b_section_idx - 1].pos_x[3] * 2.0 +
        traj->sections[(int32_T)b_section_idx - 1].pos_y[3] * traj->sections
        [(int32_T)b_section_idx - 1].pos_y[3] * 2.0) + traj->sections[(int32_T)
        b_section_idx - 1].pos_z[3] * traj->sections[(int32_T)b_section_idx - 1]
        .pos_z[3] * 2.0) + traj->sections[(int32_T)b_section_idx - 1].pos_x[1] *
        4.0 * traj->sections[(int32_T)b_section_idx - 1].pos_x[5]) +
        traj->sections[(int32_T)b_section_idx - 1].pos_x[2] * 4.0 *
        traj->sections[(int32_T)b_section_idx - 1].pos_x[4]) - (real32_T)
                      (traj->sections[(int32_T)b_section_idx - 1].pos_x[1] * 4.0)
                      * xf) + (real32_T)(traj->sections[(int32_T)b_section_idx -
        1].pos_y[1] * 4.0 * traj->sections[(int32_T)b_section_idx - 1].pos_y[5]))
                    + (real32_T)(traj->sections[(int32_T)b_section_idx - 1].
        pos_y[2] * 4.0 * traj->sections[(int32_T)b_section_idx - 1].pos_y[4])) -
                   (real32_T)(traj->sections[(int32_T)b_section_idx - 1].pos_y[1]
        * 4.0) * yf) + (real32_T)(traj->sections[(int32_T)b_section_idx - 1].
        pos_z[1] * 4.0 * traj->sections[(int32_T)b_section_idx - 1].pos_z[5])) +
                 (real32_T)(traj->sections[(int32_T)b_section_idx - 1].pos_z[2] *
                            4.0 * traj->sections[(int32_T)b_section_idx - 1].
                            pos_z[4])) - (real32_T)(traj->sections[(int32_T)
        b_section_idx - 1].pos_z[1] * 4.0) * zf;
      P_0[15] = (((((((real32_T)(traj->sections[(int32_T)b_section_idx - 1].
        pos_x[2] * 3.0 * traj->sections[(int32_T)b_section_idx - 1].pos_x[5] +
        traj->sections[(int32_T)b_section_idx - 1].pos_x[3] * 3.0 *
        traj->sections[(int32_T)b_section_idx - 1].pos_x[4]) - (real32_T)
                      (traj->sections[(int32_T)b_section_idx - 1].pos_x[2] * 3.0)
                      * xf) + (real32_T)(traj->sections[(int32_T)b_section_idx -
        1].pos_y[2] * 3.0 * traj->sections[(int32_T)b_section_idx - 1].pos_y[5]))
                    + (real32_T)(traj->sections[(int32_T)b_section_idx - 1].
        pos_y[3] * 3.0 * traj->sections[(int32_T)b_section_idx - 1].pos_y[4])) -
                   (real32_T)(traj->sections[(int32_T)b_section_idx - 1].pos_y[2]
        * 3.0) * yf) + (real32_T)(traj->sections[(int32_T)b_section_idx - 1].
        pos_z[2] * 3.0 * traj->sections[(int32_T)b_section_idx - 1].pos_z[5])) +
                 (real32_T)(traj->sections[(int32_T)b_section_idx - 1].pos_z[3] *
                            3.0 * traj->sections[(int32_T)b_section_idx - 1].
                            pos_z[4])) - (real32_T)(traj->sections[(int32_T)
        b_section_idx - 1].pos_z[2] * 3.0) * zf;
      P_0[16] = (((((real32_T)(((traj->sections[(int32_T)b_section_idx - 1].
        pos_x[4] * traj->sections[(int32_T)b_section_idx - 1].pos_x[4] +
        traj->sections[(int32_T)b_section_idx - 1].pos_y[4] * traj->sections
        [(int32_T)b_section_idx - 1].pos_y[4]) + traj->sections[(int32_T)
        b_section_idx - 1].pos_z[4] * traj->sections[(int32_T)b_section_idx - 1]
        .pos_z[4]) + traj->sections[(int32_T)b_section_idx - 1].pos_x[3] * 2.0 *
        traj->sections[(int32_T)b_section_idx - 1].pos_x[5]) - (real32_T)
                    (traj->sections[(int32_T)b_section_idx - 1].pos_x[3] * 2.0) *
                    xf) + (real32_T)(traj->sections[(int32_T)b_section_idx - 1].
        pos_y[3] * 2.0 * traj->sections[(int32_T)b_section_idx - 1].pos_y[5])) -
                  (real32_T)(traj->sections[(int32_T)b_section_idx - 1].pos_y[3]
        * 2.0) * yf) + (real32_T)(traj->sections[(int32_T)b_section_idx - 1].
                  pos_z[3] * 2.0 * traj->sections[(int32_T)b_section_idx - 1].
                  pos_z[5])) - (real32_T)(traj->sections[(int32_T)b_section_idx
        - 1].pos_z[3] * 2.0) * zf;
      P_0[17] = (((((real32_T)(traj->sections[(int32_T)b_section_idx - 1].pos_x
        [4] * traj->sections[(int32_T)b_section_idx - 1].pos_x[5]) - (real32_T)
                    traj->sections[(int32_T)b_section_idx - 1].pos_x[4] * xf) +
                   (real32_T)(traj->sections[(int32_T)b_section_idx - 1].pos_y[4]
        * traj->sections[(int32_T)b_section_idx - 1].pos_y[5])) - (real32_T)
                  traj->sections[(int32_T)b_section_idx - 1].pos_y[4] * yf) +
                 (real32_T)(traj->sections[(int32_T)b_section_idx - 1].pos_z[4] *
                            traj->sections[(int32_T)b_section_idx - 1].pos_z[5]))
        - (real32_T)traj->sections[(int32_T)b_section_idx - 1].pos_z[4] * zf;
      if (std::abs(P_0[8]) <= 2.2204460492503131E-16) {
        P_0[8] = 2.2204460492503131E-16;
      }
    }

    roots(P_0, ti_data, &i);
    loop_ub = no_of_points + 1;
    if (0 <= no_of_points) {
      memset(&T_data[0], 0, (no_of_points + 1) * sizeof(real_T));
    }

    T_data[no_of_points] = 1.0;
    for (i = 0; i <= no_of_points - 2; i++) {
      absxk = rt_atan2d_snf(ti_data[i].im, ti_data[i].re);
      if ((ti_data[i].re >= 0.0) && (ti_data[i].re <= 1.0) && (absxk < 0.001) &&
          (absxk > -0.001)) {
        T_data[i] = ti_data[i].re;
      }
    }

    error_array_size[0] = 1;
    error_array_size[1] = loop_ub;
    if (0 <= loop_ub - 1) {
      memcpy(&error_array_data[0], &T_data[0], loop_ub * sizeof(real_T));
    }

    polyval_k(traj->sections[(int32_T)b_section_idx - 1].pos_x, error_array_data,
              error_array_size, tmp_data, tmp_size);
    polyval_k(traj->sections[(int32_T)b_section_idx - 1].pos_y, error_array_data,
              error_array_size, tmp_data_0, tmp_size_0);
    polyval_k(traj->sections[(int32_T)b_section_idx - 1].pos_z, error_array_data,
              error_array_size, tmp_data_1, tmp_size_1);
    loop_ub = tmp_size[1];
    for (i = 0; i < loop_ub; i++) {
      curr_pos_data[3 * i] = tmp_data[i];
    }

    loop_ub = tmp_size_0[1];
    for (i = 0; i < loop_ub; i++) {
      curr_pos_data[1 + 3 * i] = tmp_data_0[i];
    }

    loop_ub = tmp_size_1[1];
    for (i = 0; i < loop_ub; i++) {
      curr_pos_data[2 + 3 * i] = tmp_data_1[i];
    }

    for (i = 0; i <= no_of_points; i++) {
      curr_pos_data[3 * i] = (real32_T)curr_pos_data[3 * i] - position[0];
      curr_pos_data[1 + 3 * i] = (real32_T)curr_pos_data[3 * i + 1] - position[1];
      curr_pos_data[2 + 3 * i] = (real32_T)curr_pos_data[3 * i + 2] - position[2];
    }

    error_array_size[1] = (int8_T)tmp_size[1];
    for (no_of_points = 0; no_of_points < tmp_size[1]; no_of_points++) {
      i = no_of_points * 3 + 1;
      b_section_idx = 0.0;
      scale = 3.3121686421112381E-170;
      for (loop_ub = i; loop_ub <= i + 2; loop_ub++) {
        absxk = std::abs(curr_pos_data[loop_ub - 1]);
        if (absxk > scale) {
          b_t = scale / absxk;
          b_section_idx = b_section_idx * b_t * b_t + 1.0;
          scale = absxk;
        } else {
          b_t = absxk / scale;
          b_section_idx += b_t * b_t;
        }
      }

      error_array_data[no_of_points] = scale * std::sqrt(b_section_idx);
    }

    if (error_array_size[1] <= 2) {
      if (error_array_size[1] == 1) {
        section_errors->data[k] = error_array_data[0];
        no_of_points = 1;
      } else if ((error_array_data[0] > error_array_data[1]) || (rtIsNaN
                  (error_array_data[0]) && (!rtIsNaN(error_array_data[1])))) {
        section_errors->data[k] = error_array_data[1];
        no_of_points = 2;
      } else {
        section_errors->data[k] = error_array_data[0];
        no_of_points = 1;
      }
    } else {
      if (!rtIsNaN(error_array_data[0])) {
        no_of_points = 1;
      } else {
        no_of_points = 0;
        i = 2;
        exitg1 = false;
        while ((!exitg1) && (i <= error_array_size[1])) {
          if (!rtIsNaN(error_array_data[i - 1])) {
            no_of_points = i;
            exitg1 = true;
          } else {
            i++;
          }
        }
      }

      if (no_of_points == 0) {
        section_errors->data[k] = error_array_data[0];
        no_of_points = 1;
      } else {
        b_section_idx = error_array_data[no_of_points - 1];
        for (i = no_of_points; i < error_array_size[1]; i++) {
          if (b_section_idx > error_array_data[i]) {
            b_section_idx = error_array_data[i];
            no_of_points = i + 1;
          }
        }

        section_errors->data[k] = b_section_idx;
      }
    }

    section_t->data[k] = T_data[no_of_points - 1];
  }

  if (traj->num_sections_set > 0.0) {
    if (section_errors->size[0] <= 2) {
      if (section_errors->size[0] == 1) {
        *error = section_errors->data[0];
        k = 1;
      } else if ((section_errors->data[0] > section_errors->data[1]) || (rtIsNaN
                  (section_errors->data[0]) && (!rtIsNaN(section_errors->data[1]))))
      {
        *error = section_errors->data[1];
        k = 2;
      } else {
        *error = section_errors->data[0];
        k = 1;
      }
    } else {
      if (!rtIsNaN(section_errors->data[0])) {
        k = 1;
      } else {
        k = 0;
        no_of_points = 2;
        exitg1 = false;
        while ((!exitg1) && (no_of_points <= section_errors->size[0])) {
          if (!rtIsNaN(section_errors->data[no_of_points - 1])) {
            k = no_of_points;
            exitg1 = true;
          } else {
            no_of_points++;
          }
        }
      }

      if (k == 0) {
        *error = section_errors->data[0];
        k = 1;
      } else {
        *error = section_errors->data[k - 1];
        for (no_of_points = k; no_of_points < section_errors->size[0];
             no_of_points++) {
          if (*error > section_errors->data[no_of_points]) {
            *error = section_errors->data[no_of_points];
            k = no_of_points + 1;
          }
        }
      }
    }

    *section_idx = k;
    *t = section_t->data[k - 1];
  } else {
    *section_idx = 1.0;
    *error = 0.0;
    *t = 0.0;
  }

  emxFree_real_T(&section_t);
  emxFree_real_T(&section_errors);
}

// Function for MATLAB Function: '<S91>/MATLAB Function2'
void MatlabControllerClass::polyder_a(const real_T u[6], real_T a_data[],
  int32_T a_size[2])
{
  int32_T nlead0;
  int32_T b_k;
  nlead0 = 0;
  b_k = 0;
  while ((b_k < 4) && (u[b_k] == 0.0)) {
    nlead0++;
    b_k++;
  }

  a_size[0] = 1;
  a_size[1] = 5 - nlead0;
  for (b_k = 0; b_k <= 4 - nlead0; b_k++) {
    a_data[b_k] = u[b_k + nlead0];
  }

  nlead0 = a_size[1] - 2;
  for (b_k = 0; b_k <= nlead0; b_k++) {
    a_data[b_k] *= (real_T)((nlead0 - b_k) + 1) + 1.0;
  }

  if (rtIsInf(u[5]) || rtIsNaN(u[5])) {
    a_data[a_size[1] - 1] = (rtNaN);
  }
}

// Function for MATLAB Function: '<S91>/MATLAB Function2'
real_T MatlabControllerClass::polyval_i(const real_T p_data[], const int32_T
  p_size[2], real_T x)
{
  real_T y;
  int32_T k;
  if (p_size[1] != 0) {
    y = p_data[0];
    for (k = 0; k <= p_size[1] - 2; k++) {
      y = x * y + p_data[k + 1];
    }
  }

  return y;
}

// Function for MATLAB Function: '<S91>/trajGetMatch'
void MatlabControllerClass::polyder_ge(const real_T u_data[], const int32_T
  u_size[2], real_T a_data[], int32_T a_size[2])
{
  int32_T nymax;
  int32_T nlead0;
  int32_T ny;
  real_T tmp;
  if (u_size[1] < 2) {
    nymax = 1;
  } else {
    nymax = u_size[1] - 1;
  }

  a_size[0] = 1;
  a_size[1] = nymax;
  switch (u_size[1]) {
   case 0:
    a_data[0] = 0.0;
    break;

   case 1:
    a_data[0] = 0.0;
    break;

   default:
    nlead0 = 0;
    ny = 0;
    while ((ny <= nymax - 2) && (u_data[ny] == 0.0)) {
      nlead0++;
      ny++;
    }

    ny = nymax - nlead0;
    a_size[0] = 1;
    a_size[1] = ny;
    for (nymax = 0; nymax < ny; nymax++) {
      a_data[nymax] = u_data[nymax + nlead0];
    }
    break;
  }

  nlead0 = a_size[1] - 2;
  for (ny = 0; ny <= nlead0; ny++) {
    a_data[ny] *= (real_T)((nlead0 - ny) + 1) + 1.0;
  }

  if (u_size[1] != 0) {
    tmp = u_data[u_size[1] - 1];
    if (rtIsInf(tmp) || rtIsNaN(tmp)) {
      a_data[a_size[1] - 1] = (rtNaN);
    }
  }
}

// Function for MATLAB Function: '<S91>/trajGetMatch'
real_T MatlabControllerClass::norm_o(const real_T x[3])
{
  real_T y;
  real_T scale;
  real_T absxk;
  real_T t;
  scale = 3.3121686421112381E-170;
  absxk = std::abs(x[0]);
  if (absxk > 3.3121686421112381E-170) {
    y = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    y = t * t;
  }

  absxk = std::abs(x[1]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  absxk = std::abs(x[2]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  return scale * std::sqrt(y);
}

real_T rt_powd_snf(real_T u0, real_T u1)
{
  real_T y;
  real_T tmp;
  real_T tmp_0;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else {
    tmp = std::abs(u0);
    tmp_0 = std::abs(u1);
    if (rtIsInf(u1)) {
      if (tmp == 1.0) {
        y = 1.0;
      } else if (tmp > 1.0) {
        if (u1 > 0.0) {
          y = (rtInf);
        } else {
          y = 0.0;
        }
      } else if (u1 > 0.0) {
        y = 0.0;
      } else {
        y = (rtInf);
      }
    } else if (tmp_0 == 0.0) {
      y = 1.0;
    } else if (tmp_0 == 1.0) {
      if (u1 > 0.0) {
        y = u0;
      } else {
        y = 1.0 / u0;
      }
    } else if (u1 == 2.0) {
      y = u0 * u0;
    } else if ((u1 == 0.5) && (u0 >= 0.0)) {
      y = std::sqrt(u0);
    } else if ((u0 < 0.0) && (u1 > std::floor(u1))) {
      y = (rtNaN);
    } else {
      y = pow(u0, u1);
    }
  }

  return y;
}

// Function for MATLAB Function: '<S91>/trajGetMatch'
void MatlabControllerClass::trajSectionGetFrenetSerretWithG(const section
  *traj_section, real_T vel, real_T g, real_T T[3], real_T B_a[3], real_T N[3],
  real_T *kappa, real_T *tau)
{
  real_T ddot_r_g[3];
  real_T dot_r[3];
  real_T dx_data[5];
  real_T dy_data[5];
  real_T dz_data[5];
  real_T ddx_data[4];
  real_T ddy_data[4];
  real_T ddz_data[4];
  boolean_T isodd;
  real_T A[9];
  int8_T ipiv[3];
  int32_T j;
  int32_T b_c;
  int32_T ix;
  real_T smax;
  real_T s;
  int32_T c_k;
  int32_T iy;
  int32_T c_ix;
  int32_T d;
  int32_T ijA;
  real_T tmp_data[4];
  int32_T dx_size[2];
  int32_T dy_size[2];
  int32_T dz_size[2];
  int32_T ddx_size[2];
  real_T ddot_r_idx_0;
  real_T ddot_r_idx_1;
  real_T ddot_r_idx_2;
  real_T tmp;
  real_T tmp_0;
  real_T ddot_r_g_tmp;
  real_T ddot_r_g_tmp_tmp;
  real_T smax_tmp;
  polyder_a(traj_section->pos_x, dx_data, dx_size);
  polyder_a(traj_section->pos_y, dy_data, dy_size);
  polyder_a(traj_section->pos_z, dz_data, dz_size);
  s = polyval_i(dx_data, dx_size, traj_section->t);
  tmp = polyval_i(dy_data, dy_size, traj_section->t);
  tmp_0 = polyval_i(dz_data, dz_size, traj_section->t);
  dot_r[0] = s;
  dot_r[1] = tmp;
  dot_r[2] = tmp_0;
  polyder_ge(dx_data, dx_size, ddx_data, ddx_size);
  polyder_ge(dy_data, dy_size, ddy_data, dx_size);
  polyder_ge(dz_data, dz_size, ddz_data, dy_size);
  ddot_r_idx_0 = polyval_i(ddx_data, ddx_size, traj_section->t);
  ddot_r_idx_1 = polyval_i(ddy_data, dx_size, traj_section->t);
  ddot_r_idx_2 = polyval_i(ddz_data, dy_size, traj_section->t);
  smax_tmp = norm_o(dot_r);
  smax = smax_tmp * smax_tmp;
  ddot_r_g_tmp_tmp = vel * vel;
  ddot_r_g_tmp = 0.0 / ddot_r_g_tmp_tmp * smax;
  ddot_r_g[0] = ddot_r_g_tmp + ddot_r_idx_0;
  ddot_r_g[1] = ddot_r_g_tmp + ddot_r_idx_1;
  ddot_r_g[2] = -g / ddot_r_g_tmp_tmp * smax + ddot_r_idx_2;
  if (norm_o(ddot_r_g) > 2.2204460492503131E-16) {
    ddot_r_idx_0 = ddot_r_g[0];
    ddot_r_idx_1 = ddot_r_g[1];
    ddot_r_idx_2 = ddot_r_g[2];
  }

  ddot_r_g[0] = tmp * ddot_r_idx_2 - tmp_0 * ddot_r_idx_1;
  ddot_r_g[1] = tmp_0 * ddot_r_idx_0 - s * ddot_r_idx_2;
  ddot_r_g[2] = s * ddot_r_idx_1 - tmp * ddot_r_idx_0;
  if (norm_o(ddot_r_g) < 2.2204460492503131E-16) {
    ddot_r_g[0] = 0.0;
    ddot_r_g[1] = 2.2204460492503131E-16;
    ddot_r_g[2] = 0.0;
  }

  if (smax_tmp > 2.2204460492503131E-16) {
    smax = smax_tmp;
  } else {
    smax = 2.2204460492503131E-16;
  }

  T[0] = s / smax;
  T[1] = tmp / smax;
  T[2] = tmp_0 / smax;
  ddot_r_g_tmp_tmp = norm_o(ddot_r_g);
  if (ddot_r_g_tmp_tmp > 2.2204460492503131E-16) {
    smax = ddot_r_g_tmp_tmp;
  } else {
    smax = 2.2204460492503131E-16;
  }

  polyder_ge(ddx_data, ddx_size, tmp_data, dz_size);
  polyder_ge(ddy_data, dx_size, ddx_data, ddx_size);
  polyder_ge(ddz_data, dy_size, ddy_data, dx_size);
  A[6] = polyval_i(tmp_data, dz_size, traj_section->t);
  A[7] = polyval_i(ddx_data, ddx_size, traj_section->t);
  A[8] = polyval_i(ddy_data, dx_size, traj_section->t);
  B_a[0] = ddot_r_g[0] / smax;
  A[0] = s;
  A[3] = ddot_r_idx_0;
  ipiv[0] = 1;
  B_a[1] = ddot_r_g[1] / smax;
  A[1] = tmp;
  A[4] = ddot_r_idx_1;
  ipiv[1] = 2;
  B_a[2] = ddot_r_g[2] / smax;
  A[2] = tmp_0;
  A[5] = ddot_r_idx_2;
  for (j = 0; j < 2; j++) {
    b_c = j << 2;
    iy = 0;
    ix = b_c;
    smax = std::abs(A[b_c]);
    for (c_k = 2; c_k <= 3 - j; c_k++) {
      ix++;
      s = std::abs(A[ix]);
      if (s > smax) {
        iy = c_k - 1;
        smax = s;
      }
    }

    if (A[b_c + iy] != 0.0) {
      if (iy != 0) {
        iy += j;
        ipiv[j] = (int8_T)(iy + 1);
        smax = A[j];
        A[j] = A[iy];
        A[iy] = smax;
        ix = j + 3;
        iy += 3;
        smax = A[ix];
        A[ix] = A[iy];
        A[iy] = smax;
        ix += 3;
        iy += 3;
        smax = A[ix];
        A[ix] = A[iy];
        A[iy] = smax;
      }

      iy = (b_c - j) + 3;
      for (ix = b_c + 1; ix < iy; ix++) {
        A[ix] /= A[b_c];
      }
    }

    iy = b_c + 4;
    ix = b_c + 3;
    for (c_k = 0; c_k <= 1 - j; c_k++) {
      smax = A[ix];
      if (A[ix] != 0.0) {
        c_ix = b_c + 1;
        d = (iy - j) + 2;
        for (ijA = iy; ijA < d; ijA++) {
          A[ijA] += A[c_ix] * -smax;
          c_ix++;
        }
      }

      ix += 3;
      iy += 3;
    }
  }

  isodd = false;
  if (ipiv[0] > 1) {
    isodd = true;
  }

  smax = A[0] * A[4] * A[8];
  if (ipiv[1] > 2) {
    isodd = !isodd;
  }

  if (isodd) {
    smax = -smax;
  }

  N[0] = B_a[1] * T[2] - B_a[2] * T[1];
  N[1] = B_a[2] * T[0] - B_a[0] * T[2];
  N[2] = B_a[0] * T[1] - B_a[1] * T[0];
  smax_tmp = rt_powd_snf(smax_tmp, 3.0);
  if (!(smax_tmp > 2.2204460492503131E-16)) {
    smax_tmp = 2.2204460492503131E-16;
  }

  *kappa = ddot_r_g_tmp_tmp / smax_tmp;
  smax_tmp = ddot_r_g_tmp_tmp * ddot_r_g_tmp_tmp;
  if (!(smax_tmp > 2.2204460492503131E-16)) {
    smax_tmp = 2.2204460492503131E-16;
  }

  *tau = smax / smax_tmp;
}

// Function for MATLAB Function: '<S91>/MATLAB Function2'
void MatlabControllerClass::split(real_T x[650], int32_T nx, int32_T *nxnew,
  real_T *pathlen)
{
  real_T udelta;
  int32_T n[649];
  int32_T ridx;
  int32_T b_lidx;
  int32_T j;
  *pathlen = x[nx - 1] - x[0];
  if (*pathlen > 0.0) {
    udelta = 10.0 / *pathlen;
    *nxnew = nx;
    memset(&n[0], 0, 649U * sizeof(int32_T));
    for (ridx = 0; ridx <= nx - 2; ridx++) {
      n[ridx] = (int32_T)(std::ceil(std::abs(x[ridx + 1] - x[ridx]) * udelta) -
                          1.0);
      *nxnew += n[ridx];
    }

    if (*nxnew > nx) {
      ridx = *nxnew - 1;
      for (b_lidx = nx - 2; b_lidx + 1 > 0; b_lidx--) {
        x[ridx] = x[b_lidx + 1];
        ridx--;
        udelta = (x[b_lidx + 1] - x[b_lidx]) / (real_T)(n[b_lidx] + 1);
        for (j = n[b_lidx]; j > 0; j--) {
          x[ridx] = (real_T)j * udelta + x[b_lidx];
          ridx--;
        }
      }
    }

    nx = *nxnew;
  } else {
    *nxnew = nx;
  }

  ridx = 0;
  for (b_lidx = 1; b_lidx < nx; b_lidx++) {
    if (std::abs(x[b_lidx] - x[ridx]) > 0.0) {
      ridx++;
      x[ridx] = x[b_lidx];
    } else {
      (*nxnew)--;
    }
  }

  if (*nxnew < 2) {
    x[1] = x[nx - 1];
    *nxnew = 2;
  }
}

// Function for MATLAB Function: '<S91>/MATLAB Function2'
boolean_T MatlabControllerClass::mesh_has_collapsed(const real_T x_data[], const
  int32_T x_size[2])
{
  boolean_T p;
  real_T absxk;
  real_T absxkm1;
  int32_T k;
  real_T absxk_tmp;
  boolean_T exitg1;
  absxk = std::abs(x_data[0]);
  p = false;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k <= x_size[1] - 2)) {
    absxkm1 = absxk;
    absxk_tmp = x_data[k + 1];
    absxk = std::abs(absxk_tmp);
    if ((!(absxkm1 > absxk)) && (!rtIsNaN(absxk))) {
      absxkm1 = absxk;
    }

    if (std::abs(absxk_tmp - x_data[k]) <= 2.2204460492503131E-14 * absxkm1) {
      p = true;
      exitg1 = true;
    } else {
      k++;
    }
  }

  return p;
}

// Function for MATLAB Function: '<S91>/MATLAB Function2'
void MatlabControllerClass::polyval_ik(const real_T p_data[], const int32_T
  p_size[2], const real_T x_data[], const int32_T x_size[2], real_T y_data[],
  int32_T y_size[2])
{
  real_T p;
  int32_T loop_ub;
  int32_T i;
  int32_T loop_ub_0;
  y_size[0] = 1;
  y_size[1] = (int16_T)x_size[1];
  if ((y_size[1] != 0) && (p_size[1] != 0)) {
    y_size[0] = 1;
    loop_ub_0 = y_size[1];
    for (i = 0; i < loop_ub_0; i++) {
      y_data[i] = p_data[0];
    }

    if (0 <= p_size[1] - 2) {
      y_size[0] = 1;
      y_size[1] = x_size[1];
      loop_ub = x_size[0] * x_size[1] - 1;
    }

    for (loop_ub_0 = 0; loop_ub_0 <= p_size[1] - 2; loop_ub_0++) {
      p = p_data[loop_ub_0 + 1];
      for (i = 0; i <= loop_ub; i++) {
        y_data[i] = x_data[i] * y_data[i] + p;
      }
    }
  }
}

// Function for MATLAB Function: '<S91>/MATLAB Function2'
void MatlabControllerClass::power(const real_T a_data[], const int32_T a_size[2],
  real_T y_data[], int32_T y_size[2])
{
  int32_T loop_ub;
  y_size[1] = (int16_T)a_size[1];
  loop_ub = y_size[1] - 1;
  if (0 <= loop_ub) {
    memcpy(&rtDW.z1_data[0], &y_data[0], (loop_ub + 1) * sizeof(real_T));
  }

  for (loop_ub = 0; loop_ub < y_size[1]; loop_ub++) {
    rtDW.z1_data[loop_ub] = a_data[loop_ub] * a_data[loop_ub];
  }

  y_size[0] = 1;
  loop_ub = y_size[1] - 1;
  if (0 <= loop_ub) {
    memcpy(&y_data[0], &rtDW.z1_data[0], (loop_ub + 1) * sizeof(real_T));
  }
}

// Function for MATLAB Function: '<S91>/MATLAB Function2'
void MatlabControllerClass::sqrt_p(real_T x_data[], int32_T x_size[2])
{
  int32_T k;
  for (k = 0; k < x_size[1]; k++) {
    x_data[k] = std::sqrt(x_data[k]);
  }
}

// Function for MATLAB Function: '<S91>/MATLAB Function2'
void MatlabControllerClass::__anon_fcn(const real_T dx_data[], const int32_T
  dx_size[2], const real_T dy_data[], const int32_T dy_size[2], const real_T
  dz_data[], const int32_T dz_size[2], const real_T ts_data[], const int32_T
  ts_size[2], real_T varargout_1_data[], int32_T varargout_1_size[2])
{
  int32_T i;
  int32_T loop_ub;
  int32_T tmp_size[2];
  int32_T tmp_size_0[2];
  int32_T tmp_size_1[2];
  polyval_ik(dx_data, dx_size, ts_data, ts_size, rtDW.tmp_data, tmp_size);
  power(rtDW.tmp_data, tmp_size, rtDW.tmp_data_k, tmp_size_0);
  polyval_ik(dy_data, dy_size, ts_data, ts_size, rtDW.tmp_data, tmp_size);
  power(rtDW.tmp_data, tmp_size, rtDW.tmp_data_c, tmp_size_1);
  polyval_ik(dz_data, dz_size, ts_data, ts_size, rtDW.tmp_data, tmp_size);
  power(rtDW.tmp_data, tmp_size, rtDW.tmp_data_b, tmp_size_1);
  varargout_1_size[0] = 1;
  varargout_1_size[1] = tmp_size_0[1];
  loop_ub = tmp_size_0[0] * tmp_size_0[1];
  for (i = 0; i < loop_ub; i++) {
    varargout_1_data[i] = (rtDW.tmp_data_k[i] + rtDW.tmp_data_c[i]) +
      rtDW.tmp_data_b[i];
  }

  sqrt_p(varargout_1_data, varargout_1_size);
}

// Function for MATLAB Function: '<S91>/MATLAB Function2'
void MatlabControllerClass::transfun(int8_T problem_type, const b_cell_wrap_1
  f_tunableEnvironment[3], const real_T t_data[], const int32_T t_size[2],
  real_T A, real_T B_5, real_T yZERO, boolean_T firstcall, real_T y_data[],
  int32_T y_size[2], boolean_T *tooclose)
{
  int32_T nt;
  real_T BmAd4;
  real_T BpAd2;
  real_T BmAtp75;
  real_T tk2;
  int32_T d_k;
  int32_T x_size[2];
  int16_T b_idx_1;
  nt = t_size[1] - 1;
  x_size[0] = t_size[0];
  x_size[1] = t_size[1];
  switch (problem_type) {
   case 1:
    tk2 = B_5 - A;
    BmAd4 = tk2 * 0.25;
    BpAd2 = (B_5 + A) * 0.5;
    BmAtp75 = tk2 * 0.75;
    for (nt = 0; nt < t_size[1]; nt++) {
      tk2 = t_data[nt] * t_data[nt];
      rtDW.x_data_c[nt] = BmAd4 * t_data[nt] * (3.0 - tk2) + BpAd2;
      rtDW.xt_data[nt] = (1.0 - tk2) * BmAtp75;
    }
    break;

   case 2:
    for (d_k = 0; d_k <= nt; d_k++) {
      tk2 = t_data[d_k] / (1.0 - t_data[d_k]);
      rtDW.x_data_c[d_k] = tk2 * tk2 + A;
      rtDW.xt_data[d_k] = 2.0 * tk2 / ((1.0 - t_data[d_k]) * (1.0 - t_data[d_k]));
    }
    break;

   case 3:
    for (d_k = 0; d_k <= nt; d_k++) {
      tk2 = t_data[d_k] / (1.0 + t_data[d_k]);
      rtDW.x_data_c[d_k] = B_5 - tk2 * tk2;
      rtDW.xt_data[d_k] = -2.0 * tk2 / ((1.0 + t_data[d_k]) * (1.0 + t_data[d_k]));
    }
    break;

   default:
    for (d_k = 0; d_k <= nt; d_k++) {
      tk2 = t_data[d_k] * t_data[d_k];
      rtDW.x_data_c[d_k] = t_data[d_k] / (1.0 - tk2);
      rtDW.xt_data[d_k] = (1.0 + tk2) / ((1.0 - tk2) * (1.0 - tk2));
    }
    break;
  }

  *tooclose = ((!firstcall) && mesh_has_collapsed(rtDW.x_data_c, x_size));
  if (*tooclose) {
    b_idx_1 = (int16_T)t_size[1];
    y_size[0] = 1;
    y_size[1] = b_idx_1;
    d_k = b_idx_1 - 1;
    for (nt = 0; nt <= d_k; nt++) {
      y_data[nt] = yZERO;
    }
  } else {
    __anon_fcn(f_tunableEnvironment[0].f1.data, f_tunableEnvironment[0].f1.size,
               f_tunableEnvironment[1].f1.data, f_tunableEnvironment[1].f1.size,
               f_tunableEnvironment[2].f1.data, f_tunableEnvironment[2].f1.size,
               rtDW.x_data_c, x_size, y_data, y_size);
    nt = y_size[0] * y_size[1];
    y_size[0] = 1;
    d_k = nt - 1;
    for (nt = 0; nt <= d_k; nt++) {
      y_data[nt] *= rtDW.xt_data[nt];
    }
  }
}

// Function for MATLAB Function: '<S91>/MATLAB Function2'
real_T MatlabControllerClass::midpArea(const b_cell_wrap_1 f_tunableEnvironment
  [3], real_T a, real_T b)
{
  real_T q;
  real_T x;
  real_T b_a;
  real_T c_a;
  x = (a + b) / 2.0;
  if ((!rtIsInf(a)) && (!rtIsNaN(a)) && ((!rtIsInf(b)) && (!rtIsNaN(b))) &&
      (rtIsInf(x) || rtIsNaN(x))) {
    x = a / 2.0 + b / 2.0;
  }

  b_a = polyval_i(f_tunableEnvironment[0].f1.data, f_tunableEnvironment[0].
                  f1.size, x);
  c_a = polyval_i(f_tunableEnvironment[1].f1.data, f_tunableEnvironment[1].
                  f1.size, x);
  x = polyval_i(f_tunableEnvironment[2].f1.data, f_tunableEnvironment[2].f1.size,
                x);
  q = std::sqrt((b_a * b_a + c_a * c_a) + x * x) * (b - a);
  return q;
}

// Function for MATLAB Function: '<S91>/MATLAB Function2'
real_T MatlabControllerClass::quadgk(const b_cell_wrap_1 fun_tunableEnvironment
  [3], real_T b)
{
  real_T q;
  boolean_T reversedir;
  real_T B_4;
  int32_T nt;
  boolean_T isinfA;
  boolean_T isinfB;
  int8_T problem_type;
  real_T pathlen;
  real_T q_ok;
  real_T err_ok;
  int32_T ix;
  real_T qsubs;
  real_T tol;
  real_T errsub1norm;
  real_T abserrsubk;
  int32_T c_k;
  real_T absxk;
  static const real_T NODES[15] = { -0.99145537112081261, -0.94910791234275849,
    -0.8648644233597691, -0.74153118559939435, -0.58608723546769115,
    -0.40584515137739718, -0.20778495500789851, 0.0, 0.20778495500789851,
    0.40584515137739718, 0.58608723546769115, 0.74153118559939435,
    0.8648644233597691, 0.94910791234275849, 0.99145537112081261 };

  static const real_T f[15] = { 0.022935322010529221, 0.063092092629978544,
    0.1047900103222502, 0.14065325971552589, 0.16900472663926791,
    0.19035057806478539, 0.20443294007529891, 0.20948214108472779,
    0.20443294007529891, 0.19035057806478539, 0.16900472663926791,
    0.14065325971552589, 0.1047900103222502, 0.063092092629978544,
    0.022935322010529221 };

  static const real_T g[15] = { 0.022935322010529221, -0.066392873538891159,
    0.1047900103222502, -0.13905213177375081, 0.16900472663926791,
    -0.19147947244033353, 0.20443294007529891, -0.20847704258874161,
    0.20443294007529891, -0.19147947244033353, 0.16900472663926791,
    -0.13905213177375081, 0.1047900103222502, -0.066392873538891159,
    0.022935322010529221 };

  int32_T x_size[2];
  int32_T fx_size[2];
  boolean_T tmp;
  boolean_T tmp_0;
  int32_T subs_tmp;
  int32_T subs_tmp_0;
  int32_T subs_tmp_tmp;
  int32_T exitg1;
  if ((0.0 != b) && (0.0 > b)) {
    absxk = std::abs(b);
    if (!rtIsInf(absxk)) {
      if (!(absxk <= 2.2250738585072014E-308)) {
        frexp(absxk, &nt);
      }

      if (!(absxk <= 2.2250738585072014E-308)) {
        frexp(absxk, &ix);
      }
    }
  }

  if (b < 0.0) {
    reversedir = true;
    absxk = b;
    B_4 = 0.0;
  } else {
    reversedir = false;
    absxk = 0.0;
    B_4 = b;
  }

  rtDW.interval[0] = absxk;
  nt = 0;
  rtDW.interval[1] = B_4;
  isinfA = rtIsInf(absxk);
  isinfB = rtIsInf(B_4);
  if (absxk >= B_4) {
    problem_type = 0;
  } else {
    tmp = !isinfA;
    tmp_0 = !isinfB;
    if (tmp && tmp_0) {
      rtDW.interval[0] = -1.0;
      rtDW.interval[1] = 1.0;
      problem_type = 1;
    } else if (tmp && isinfB) {
      rtDW.interval[0] = 0.0;
      rtDW.interval[1] = 1.0;
      problem_type = 2;
    } else if (isinfA && tmp_0) {
      rtDW.interval[0] = -1.0;
      rtDW.interval[1] = 0.0;
      problem_type = 3;
    } else if (isinfA && isinfB) {
      rtDW.interval[0] = -1.0;
      rtDW.interval[1] = 1.0;
      problem_type = 4;
    } else {
      problem_type = 0;
    }
  }

  memset(&rtDW.interval[2], 0, 648U * sizeof(real_T));
  q = 0.0;
  if (problem_type == 0) {
    pathlen = 0.0;
  } else {
    split(rtDW.interval, 2, &nt, &pathlen);
    nt -= 2;
    if (!(pathlen > 0.0)) {
      problem_type = 0;
    }
  }

  if (problem_type == 0) {
    q = midpArea(fun_tunableEnvironment, absxk, B_4);
    if (reversedir) {
      q = -q;
    }
  } else {
    for (ix = 0; ix <= nt; ix++) {
      subs_tmp = ix << 1;
      rtDW.subs[subs_tmp] = rtDW.interval[ix];
      rtDW.subs[1 + subs_tmp] = rtDW.interval[ix + 1];
    }

    q_ok = 0.0;
    err_ok = 0.0;
    isinfA = true;
    do {
      exitg1 = 0;
      x_size[0] = 1;
      x_size[1] = (nt + 1) * 15;
      ix = -1;
      for (c_k = 0; c_k <= nt; c_k++) {
        subs_tmp = c_k << 1;
        tol = rtDW.subs[subs_tmp + 1];
        qsubs = (tol + rtDW.subs[subs_tmp]) / 2.0;
        tol = (tol - rtDW.subs[c_k << 1]) / 2.0;
        for (subs_tmp = 0; subs_tmp < 15; subs_tmp++) {
          ix++;
          rtDW.x_data[ix] = NODES[subs_tmp] * tol + qsubs;
        }
      }

      transfun(problem_type, fun_tunableEnvironment, rtDW.x_data, x_size, absxk,
               B_4, 0.0, isinfA, rtDW.fx_data, fx_size, &isinfB);
      if (isinfB) {
        exitg1 = 1;
      } else {
        qsubs = 0.0;
        ix = -1;
        for (c_k = 0; c_k <= nt; c_k++) {
          rtDW.qsub[c_k] = 0.0;
          rtDW.errsub[c_k] = 0.0;
          for (subs_tmp = 0; subs_tmp < 15; subs_tmp++) {
            ix++;
            rtDW.qsub[c_k] += f[subs_tmp] * rtDW.fx_data[ix];
            rtDW.errsub[c_k] += g[subs_tmp] * rtDW.fx_data[ix];
          }

          subs_tmp = c_k << 1;
          tol = (rtDW.subs[subs_tmp + 1] - rtDW.subs[subs_tmp]) / 2.0;
          rtDW.qsub[c_k] *= tol;
          qsubs += rtDW.qsub[c_k];
          rtDW.errsub[c_k] *= tol;
        }

        q = qsubs + q_ok;
        tol = 1.0E-6 * std::abs(q);
        if ((1.0E-10 > tol) || rtIsNaN(tol)) {
          tol = 1.0E-10;
        }

        qsubs = 2.0 * tol / pathlen;
        errsub1norm = 0.0;
        ix = 0;
        for (c_k = 0; c_k <= nt; c_k++) {
          abserrsubk = std::abs(rtDW.errsub[c_k]);
          subs_tmp = c_k << 1;
          subs_tmp_tmp = subs_tmp + 1;
          if (abserrsubk <= (rtDW.subs[subs_tmp_tmp] - rtDW.subs[subs_tmp]) /
              2.0 * qsubs) {
            err_ok += rtDW.errsub[c_k];
            q_ok += rtDW.qsub[c_k];
          } else {
            errsub1norm += abserrsubk;
            ix++;
            subs_tmp = (ix - 1) << 1;
            rtDW.subs[subs_tmp] = rtDW.subs[c_k << 1];
            rtDW.subs[1 + subs_tmp] = rtDW.subs[subs_tmp_tmp];
          }
        }

        qsubs = std::abs(err_ok) + errsub1norm;
        if ((!isfinite(q)) || (!isfinite(qsubs)) || (ix == 0) || (qsubs <= tol))
        {
          exitg1 = 1;
        } else {
          nt = (ix << 1) - 1;
          if (nt + 1 > 650) {
            exitg1 = 1;
          } else {
            while (ix > 0) {
              c_k = (ix - 1) << 1;
              subs_tmp = c_k + 1;
              subs_tmp_tmp = ix << 1;
              subs_tmp_0 = (subs_tmp_tmp - 1) << 1;
              rtDW.subs[1 + subs_tmp_0] = rtDW.subs[subs_tmp];
              rtDW.subs[subs_tmp_0] = (rtDW.subs[subs_tmp] + rtDW.subs[c_k]) /
                2.0;
              subs_tmp = (subs_tmp_tmp - 2) << 1;
              rtDW.subs[1 + subs_tmp] = rtDW.subs[subs_tmp_0];
              rtDW.subs[subs_tmp] = rtDW.subs[c_k];
              ix--;
            }

            isinfA = false;
          }
        }
      }
    } while (exitg1 == 0);

    if (reversedir) {
      q = -q;
    }
  }

  return q;
}

// Function for MATLAB Function: '<S91>/MATLAB Function2'
void MatlabControllerClass::trajSectionGetArcLength(const real_T
  traj_section_pos_x[6], const real_T traj_section_pos_y[6], const real_T
  traj_section_pos_z[6], real_T varargin_1, real_T *arc_length, real_T
  *arc_length_dt)
{
  real_T t;
  real_T dx_data[5];
  real_T dy_data[5];
  real_T dz_data[5];
  b_cell_wrap_1 tunableEnvironment[3];
  real_T a;
  real_T b_a;
  int32_T loop_ub;
  int32_T i;
  int32_T dx_size[2];
  int32_T dy_size[2];
  int32_T dz_size[2];
  if ((1.1 < varargin_1) || rtIsNaN(varargin_1)) {
    t = 1.1;
  } else {
    t = varargin_1;
  }

  if (!(t > -0.1)) {
    t = -0.1;
  }

  polyder_a(traj_section_pos_x, dx_data, dx_size);
  polyder_a(traj_section_pos_y, dy_data, dy_size);
  polyder_a(traj_section_pos_z, dz_data, dz_size);
  tunableEnvironment[0].f1.size[0] = 1;
  tunableEnvironment[0].f1.size[1] = dx_size[1];
  loop_ub = dx_size[0] * dx_size[1] - 1;
  for (i = 0; i <= loop_ub; i++) {
    tunableEnvironment[0].f1.data[i] = dx_data[i];
  }

  tunableEnvironment[1].f1.size[0] = 1;
  tunableEnvironment[1].f1.size[1] = dy_size[1];
  loop_ub = dy_size[0] * dy_size[1] - 1;
  for (i = 0; i <= loop_ub; i++) {
    tunableEnvironment[1].f1.data[i] = dy_data[i];
  }

  tunableEnvironment[2].f1.size[0] = 1;
  tunableEnvironment[2].f1.size[1] = dz_size[1];
  loop_ub = dz_size[0] * dz_size[1] - 1;
  for (i = 0; i <= loop_ub; i++) {
    tunableEnvironment[2].f1.data[i] = dz_data[i];
  }

  *arc_length = quadgk(tunableEnvironment, t);
  a = polyval_i(dx_data, dx_size, t);
  b_a = polyval_i(dy_data, dy_size, t);
  t = polyval_i(dz_data, dz_size, t);
  *arc_length_dt = std::sqrt((a * a + b_a * b_a) + t * t);
}

// Function for MATLAB Function: '<S54>/MATLAB Function1'
void MatlabControllerClass::quatNormalize(const real32_T q[4], real32_T q_out[4])
{
  real32_T y;
  real32_T scale;
  real32_T absxk;
  real32_T t;
  scale = 1.29246971E-26F;
  absxk = std::abs(q[0]);
  if (absxk > 1.29246971E-26F) {
    y = 1.0F;
    scale = absxk;
  } else {
    t = absxk / 1.29246971E-26F;
    y = t * t;
  }

  absxk = std::abs(q[1]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  absxk = std::abs(q[2]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  absxk = std::abs(q[3]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  y = scale * std::sqrt(y);
  if (rtIsNaNF(y) || (!(2.22044605E-16F < y))) {
    y = 2.22044605E-16F;
  }

  q_out[0] = q[0] / y;
  q_out[1] = q[1] / y;
  q_out[2] = q[2] / y;
  q_out[3] = q[3] / y;
}

// Function for MATLAB Function: '<S49>/MATLAB Function'
void MatlabControllerClass::LSQFromQR(const real32_T A_data[], const int32_T
  A_size[2], const real32_T tau_data[], const int32_T jpvt_data[], real32_T B_3
  [8], int32_T rankA, real32_T Y_data[], int32_T *Y_size)
{
  int32_T b_i;
  real32_T wj;
  int32_T b_j;
  int32_T loop_ub;
  int8_T b_idx_0;
  b_idx_0 = (int8_T)A_size[1];
  *Y_size = b_idx_0;
  if (0 <= b_idx_0 - 1) {
    memset(&Y_data[0], 0, b_idx_0 * sizeof(real32_T));
  }

  for (b_j = 0; b_j < A_size[1]; b_j++) {
    if (tau_data[b_j] != 0.0F) {
      wj = B_3[b_j];
      for (loop_ub = b_j + 1; loop_ub + 1 < 9; loop_ub++) {
        wj += A_data[(b_j << 3) + loop_ub] * B_3[loop_ub];
      }

      wj *= tau_data[b_j];
      if (wj != 0.0F) {
        B_3[b_j] -= wj;
        for (loop_ub = b_j + 1; loop_ub + 1 < 9; loop_ub++) {
          B_3[loop_ub] -= A_data[(b_j << 3) + loop_ub] * wj;
        }
      }
    }
  }

  for (loop_ub = 0; loop_ub < rankA; loop_ub++) {
    Y_data[jpvt_data[loop_ub] - 1] = B_3[loop_ub];
  }

  for (loop_ub = rankA - 1; loop_ub + 1 > 0; loop_ub--) {
    b_j = loop_ub << 3;
    Y_data[jpvt_data[loop_ub] - 1] /= A_data[b_j + loop_ub];
    for (b_i = 0; b_i < loop_ub; b_i++) {
      Y_data[jpvt_data[b_i] - 1] -= A_data[b_j + b_i] * Y_data[jpvt_data[loop_ub]
        - 1];
    }
  }
}

// Function for MATLAB Function: '<S49>/MATLAB Function'
real32_T MatlabControllerClass::xnrm2(int32_T n, const real32_T x_data[],
  int32_T ix0)
{
  real32_T y;
  real32_T scale;
  int32_T kend;
  real32_T absxk;
  real32_T t;
  int32_T k;
  y = 0.0F;
  scale = 1.29246971E-26F;
  kend = (ix0 + n) - 1;
  for (k = ix0; k <= kend; k++) {
    absxk = std::abs(x_data[k - 1]);
    if (absxk > scale) {
      t = scale / absxk;
      y = y * t * t + 1.0F;
      scale = absxk;
    } else {
      t = absxk / scale;
      y += t * t;
    }
  }

  return scale * std::sqrt(y);
}

real32_T rt_hypotf_snf(real32_T u0, real32_T u1)
{
  real32_T y;
  real32_T a;
  a = std::abs(u0);
  y = std::abs(u1);
  if (a < y) {
    a /= y;
    y *= std::sqrt(a * a + 1.0F);
  } else if (a > y) {
    y /= a;
    y = std::sqrt(y * y + 1.0F) * a;
  } else {
    if (!rtIsNaNF(y)) {
      y = a * 1.41421354F;
    }
  }

  return y;
}

// Function for MATLAB Function: '<S49>/MATLAB Function'
void MatlabControllerClass::xzlarf(int32_T m, int32_T n, int32_T iv0, real32_T
  tau, real32_T C_data[], int32_T ic0, real32_T work_data[])
{
  int32_T lastv;
  int32_T lastc;
  int32_T coltop;
  int32_T ix;
  real32_T c;
  int32_T iac;
  int32_T d;
  int32_T b_ia;
  int32_T jy;
  int32_T exitg1;
  boolean_T exitg2;
  if (tau != 0.0F) {
    lastv = m;
    lastc = iv0 + m;
    while ((lastv > 0) && (C_data[lastc - 2] == 0.0F)) {
      lastv--;
      lastc--;
    }

    lastc = n - 1;
    exitg2 = false;
    while ((!exitg2) && (lastc + 1 > 0)) {
      coltop = (lastc << 3) + ic0;
      jy = coltop;
      do {
        exitg1 = 0;
        if (jy <= (coltop + lastv) - 1) {
          if (C_data[jy - 1] != 0.0F) {
            exitg1 = 1;
          } else {
            jy++;
          }
        } else {
          lastc--;
          exitg1 = 2;
        }
      } while (exitg1 == 0);

      if (exitg1 == 1) {
        exitg2 = true;
      }
    }
  } else {
    lastv = 0;
    lastc = -1;
  }

  if (lastv > 0) {
    if (lastc + 1 != 0) {
      for (coltop = 0; coltop <= lastc; coltop++) {
        work_data[coltop] = 0.0F;
      }

      coltop = 0;
      jy = (lastc << 3) + ic0;
      for (iac = ic0; iac <= jy; iac += 8) {
        ix = iv0;
        c = 0.0F;
        d = (iac + lastv) - 1;
        for (b_ia = iac; b_ia <= d; b_ia++) {
          c += C_data[b_ia - 1] * C_data[ix - 1];
          ix++;
        }

        work_data[coltop] += c;
        coltop++;
      }
    }

    if (!(-tau == 0.0F)) {
      coltop = ic0 - 1;
      jy = 0;
      for (iac = 0; iac <= lastc; iac++) {
        if (work_data[jy] != 0.0F) {
          c = work_data[jy] * -tau;
          ix = iv0;
          d = lastv + coltop;
          for (b_ia = coltop; b_ia < d; b_ia++) {
            C_data[b_ia] += C_data[ix - 1] * c;
            ix++;
          }
        }

        jy++;
        coltop += 8;
      }
    }
  }
}

// Function for MATLAB Function: '<S49>/MATLAB Function'
void MatlabControllerClass::qrsolve(const real32_T A_data[], const int32_T
  A_size[2], const real32_T B_1[8], real32_T Y_data[], int32_T *Y_size)
{
  real32_T b_A_data[32];
  real32_T tau_data[4];
  int32_T jpvt_data[4];
  int32_T n;
  real32_T work_data[4];
  real32_T vn1_data[4];
  real32_T vn2_data[4];
  int32_T nmi;
  int32_T b_n;
  int32_T yk;
  int32_T idxmax;
  int32_T ix;
  real32_T smax;
  real32_T s;
  int32_T b_ix;
  int32_T iy;
  real32_T absxk;
  real32_T t;
  real32_T B_2[8];
  int32_T b_A_size[2];
  int8_T c_idx_0;
  b_A_size[0] = 8;
  b_A_size[1] = A_size[1];
  b_n = A_size[0] * A_size[1] - 1;
  if (0 <= b_n) {
    memcpy(&b_A_data[0], &A_data[0], (b_n + 1) * sizeof(real32_T));
  }

  n = A_size[1];
  if (A_size[1] < 1) {
    b_n = 0;
  } else {
    b_n = A_size[1];
  }

  if (b_n > 0) {
    jpvt_data[0] = 1;
    yk = 1;
    for (nmi = 2; nmi <= b_n; nmi++) {
      yk++;
      jpvt_data[nmi - 1] = yk;
    }
  }

  if (A_size[1] != 0) {
    c_idx_0 = (int8_T)A_size[1];
    if (0 <= c_idx_0 - 1) {
      memset(&work_data[0], 0, c_idx_0 * sizeof(real32_T));
    }

    b_n = 1;
    for (yk = 0; yk < n; yk++) {
      smax = 0.0F;
      s = 1.29246971E-26F;
      for (nmi = b_n; nmi <= b_n + 7; nmi++) {
        absxk = std::abs(A_data[nmi - 1]);
        if (absxk > s) {
          t = s / absxk;
          smax = smax * t * t + 1.0F;
          s = absxk;
        } else {
          t = absxk / s;
          smax += t * t;
        }
      }

      vn1_data[yk] = s * std::sqrt(smax);
      vn2_data[yk] = vn1_data[yk];
      b_n += 8;
    }

    for (b_n = 0; b_n < n; b_n++) {
      iy = b_n << 3;
      yk = iy + b_n;
      nmi = n - b_n;
      if (nmi < 1) {
        idxmax = 0;
      } else {
        idxmax = 1;
        if (nmi > 1) {
          ix = b_n;
          smax = std::abs(vn1_data[b_n]);
          for (b_ix = 2; b_ix <= nmi; b_ix++) {
            ix++;
            s = std::abs(vn1_data[ix]);
            if (s > smax) {
              idxmax = b_ix;
              smax = s;
            }
          }
        }
      }

      ix = (b_n + idxmax) - 1;
      if (ix + 1 != b_n + 1) {
        b_ix = ix << 3;
        for (idxmax = 0; idxmax < 8; idxmax++) {
          smax = b_A_data[b_ix];
          b_A_data[b_ix] = b_A_data[iy];
          b_A_data[iy] = smax;
          b_ix++;
          iy++;
        }

        b_ix = jpvt_data[ix];
        jpvt_data[ix] = jpvt_data[b_n];
        jpvt_data[b_n] = b_ix;
        vn1_data[ix] = vn1_data[b_n];
        vn2_data[ix] = vn2_data[b_n];
      }

      smax = b_A_data[yk];
      tau_data[b_n] = 0.0F;
      s = xnrm2(7 - b_n, b_A_data, yk + 2);
      if (s != 0.0F) {
        s = rt_hypotf_snf(b_A_data[yk], s);
        if (b_A_data[yk] >= 0.0F) {
          s = -s;
        }

        if (std::abs(s) < 9.86076132E-32F) {
          ix = -1;
          b_ix = (yk - b_n) + 8;
          do {
            ix++;
            for (iy = yk + 1; iy < b_ix; iy++) {
              b_A_data[iy] *= 1.01412048E+31F;
            }

            s *= 1.01412048E+31F;
            smax *= 1.01412048E+31F;
          } while (!(std::abs(s) >= 9.86076132E-32F));

          s = rt_hypotf_snf(smax, xnrm2(7 - b_n, b_A_data, yk + 2));
          if (smax >= 0.0F) {
            s = -s;
          }

          tau_data[b_n] = (s - smax) / s;
          smax = 1.0F / (smax - s);
          b_ix = (yk - b_n) + 8;
          for (iy = yk + 1; iy < b_ix; iy++) {
            b_A_data[iy] *= smax;
          }

          for (idxmax = 0; idxmax <= ix; idxmax++) {
            s *= 9.86076132E-32F;
          }

          smax = s;
        } else {
          tau_data[b_n] = (s - b_A_data[yk]) / s;
          smax = 1.0F / (b_A_data[yk] - s);
          ix = (yk - b_n) + 8;
          for (b_ix = yk + 1; b_ix < ix; b_ix++) {
            b_A_data[b_ix] *= smax;
          }

          smax = s;
        }
      }

      b_A_data[yk] = smax;
      if (b_n + 1 < n) {
        smax = b_A_data[yk];
        b_A_data[yk] = 1.0F;
        xzlarf(8 - b_n, nmi - 1, yk + 1, tau_data[b_n], b_A_data, (b_n + ((b_n +
                  1) << 3)) + 1, work_data);
        b_A_data[yk] = smax;
      }

      for (yk = b_n + 1; yk < n; yk++) {
        if (vn1_data[yk] != 0.0F) {
          nmi = (yk << 3) + b_n;
          smax = std::abs(b_A_data[nmi]) / vn1_data[yk];
          smax = 1.0F - smax * smax;
          if (smax < 0.0F) {
            smax = 0.0F;
          }

          s = vn1_data[yk] / vn2_data[yk];
          s = s * s * smax;
          if (s <= 0.000345266977F) {
            vn1_data[yk] = xnrm2(7 - b_n, b_A_data, nmi + 2);
            vn2_data[yk] = vn1_data[yk];
          } else {
            vn1_data[yk] *= std::sqrt(smax);
          }
        }
      }
    }
  }

  n = 0;
  if (b_A_size[1] > 0) {
    while ((n < b_A_size[1]) && (!(std::abs(b_A_data[(n << 3) + n]) <=
             9.53674316E-6F * std::abs(b_A_data[0])))) {
      n++;
    }
  }

  for (b_ix = 0; b_ix < 8; b_ix++) {
    B_2[b_ix] = B_1[b_ix];
  }

  LSQFromQR(b_A_data, b_A_size, tau_data, jpvt_data, B_2, n, Y_data, Y_size);
}

// Function for MATLAB Function: '<S49>/MATLAB Function'
void MatlabControllerClass::mldivide(const real32_T A_data[], const int32_T
  A_size[2], const real32_T B_0[8], real32_T Y_data[], int32_T *Y_size)
{
  if (A_size[1] == 0) {
    *Y_size = 0;
  } else {
    qrsolve(A_data, A_size, B_0, Y_data, Y_size);
  }
}

// Function for MATLAB Function: '<S49>/MATLAB Function'
boolean_T MatlabControllerClass::any(const boolean_T x_data[], const int32_T
  *x_size)
{
  boolean_T y;
  int32_T ix;
  boolean_T exitg1;
  y = false;
  ix = 1;
  exitg1 = false;
  while ((!exitg1) && (ix <= *x_size)) {
    if (!x_data[ix - 1]) {
      ix++;
    } else {
      y = true;
      exitg1 = true;
    }
  }

  return y;
}

// Function for MATLAB Function: '<S49>/MATLAB Function'
boolean_T MatlabControllerClass::ifWhileCond(const boolean_T x[4])
{
  boolean_T y;
  int32_T k;
  boolean_T exitg1;
  y = true;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 4)) {
    if (!x[k]) {
      y = false;
      exitg1 = true;
    } else {
      k++;
    }
  }

  return y;
}

// Function for MATLAB Function: '<S104>/trajFromWaypoints'
void MatlabControllerClass::trajSectionGetPos(const real_T traj_section_pos_x[6],
  const real_T traj_section_pos_y[6], const real_T traj_section_pos_z[6], real_T
  varargin_1, real_T pos[3])
{
  real_T px;
  real_T py;
  real_T pz;
  int32_T k;
  px = traj_section_pos_x[0];
  py = traj_section_pos_y[0];
  pz = traj_section_pos_z[0];
  for (k = 0; k < 5; k++) {
    px = varargin_1 * px + traj_section_pos_x[k + 1];
    py = varargin_1 * py + traj_section_pos_y[k + 1];
    pz = varargin_1 * pz + traj_section_pos_z[k + 1];
  }

  pos[0] = px;
  pos[1] = py;
  pos[2] = pz;
}

// Function for MATLAB Function: '<S104>/trajFromWaypoints'
void MatlabControllerClass::split_i(real_T x[650], int32_T nx, int32_T *nxnew,
  real_T *pathlen)
{
  int32_T ridx;
  real_T delta;
  int32_T j;
  int32_T n_idx_0;
  *pathlen = x[1] - x[0];
  if (*pathlen > 0.0) {
    n_idx_0 = (int32_T)(std::ceil(std::abs(*pathlen) * (10.0 / *pathlen)) - 1.0);
    *nxnew = 2 + n_idx_0;
    if (2 + n_idx_0 > 2) {
      x[n_idx_0 + 1] = x[1];
      ridx = n_idx_0;
      delta = (x[1] - x[0]) / (real_T)(n_idx_0 + 1);
      for (j = n_idx_0; j > 0; j--) {
        x[ridx] = (real_T)j * delta + x[0];
        ridx--;
      }
    }

    nx = 2 + n_idx_0;
  } else {
    *nxnew = 2;
  }

  ridx = 0;
  for (j = 1; j < nx; j++) {
    if (std::abs(x[j] - x[ridx]) > 0.0) {
      ridx++;
      x[ridx] = x[j];
    } else {
      (*nxnew)--;
    }
  }

  if (*nxnew < 2) {
    x[1] = x[nx - 1];
    *nxnew = 2;
  }
}

// Function for MATLAB Function: '<S104>/trajFromWaypoints'
void MatlabControllerClass::transfun_b(const b_cell_wrap_1 f_tunableEnvironment
  [3], const real_T t_data[], const int32_T t_size[2], boolean_T firstcall,
  real_T y_data[], int32_T y_size[2], boolean_T *tooclose)
{
  real_T tk2;
  int32_T k;
  int32_T loop_ub;
  int32_T x_size[2];
  int16_T b_idx_1;
  x_size[0] = t_size[0];
  x_size[1] = t_size[1];
  for (k = 0; k < t_size[1]; k++) {
    tk2 = t_data[k] * t_data[k];
    rtDW.x_data_bs[k] = 0.25 * t_data[k] * (3.0 - tk2) + 0.5;
    rtDW.xt_data_l[k] = (1.0 - tk2) * 0.75;
  }

  *tooclose = ((!firstcall) && mesh_has_collapsed(rtDW.x_data_bs, x_size));
  if (*tooclose) {
    b_idx_1 = (int16_T)t_size[1];
    y_size[0] = 1;
    y_size[1] = b_idx_1;
    loop_ub = b_idx_1 - 1;
    if (0 <= loop_ub) {
      memset(&y_data[0], 0, (loop_ub + 1) * sizeof(real_T));
    }
  } else {
    __anon_fcn(f_tunableEnvironment[0].f1.data, f_tunableEnvironment[0].f1.size,
               f_tunableEnvironment[1].f1.data, f_tunableEnvironment[1].f1.size,
               f_tunableEnvironment[2].f1.data, f_tunableEnvironment[2].f1.size,
               rtDW.x_data_bs, x_size, y_data, y_size);
    k = y_size[0] * y_size[1];
    y_size[0] = 1;
    loop_ub = k - 1;
    for (k = 0; k <= loop_ub; k++) {
      y_data[k] *= rtDW.xt_data_l[k];
    }
  }
}

// Function for MATLAB Function: '<S104>/trajFromWaypoints'
real_T MatlabControllerClass::midpArea_o(const b_cell_wrap_1
  f_tunableEnvironment[3])
{
  real_T q;
  real_T b_a;
  real_T c_a;
  real_T d_a;
  b_a = polyval_i(f_tunableEnvironment[0].f1.data, f_tunableEnvironment[0].
                  f1.size, 0.5);
  c_a = polyval_i(f_tunableEnvironment[1].f1.data, f_tunableEnvironment[1].
                  f1.size, 0.5);
  d_a = polyval_i(f_tunableEnvironment[2].f1.data, f_tunableEnvironment[2].
                  f1.size, 0.5);
  q = std::sqrt((b_a * b_a + c_a * c_a) + d_a * d_a);
  return q;
}

// Function for MATLAB Function: '<S104>/trajFromWaypoints'
real_T MatlabControllerClass::quadgk_g(const b_cell_wrap_1
  fun_tunableEnvironment[3])
{
  real_T q;
  int8_T problem_type;
  int32_T nsubs;
  real_T q_ok;
  real_T err_ok;
  boolean_T first_iteration;
  int32_T ix;
  real_T qsubs;
  real_T tol;
  real_T errsub1norm;
  real_T abserrsubk;
  real_T pathlen;
  boolean_T tooclose;
  int32_T b_k;
  static const real_T NODES[15] = { -0.99145537112081261, -0.94910791234275849,
    -0.8648644233597691, -0.74153118559939435, -0.58608723546769115,
    -0.40584515137739718, -0.20778495500789851, 0.0, 0.20778495500789851,
    0.40584515137739718, 0.58608723546769115, 0.74153118559939435,
    0.8648644233597691, 0.94910791234275849, 0.99145537112081261 };

  static const real_T d[15] = { 0.022935322010529221, 0.063092092629978544,
    0.1047900103222502, 0.14065325971552589, 0.16900472663926791,
    0.19035057806478539, 0.20443294007529891, 0.20948214108472779,
    0.20443294007529891, 0.19035057806478539, 0.16900472663926791,
    0.14065325971552589, 0.1047900103222502, 0.063092092629978544,
    0.022935322010529221 };

  static const real_T e[15] = { 0.022935322010529221, -0.066392873538891159,
    0.1047900103222502, -0.13905213177375081, 0.16900472663926791,
    -0.19147947244033353, 0.20443294007529891, -0.20847704258874161,
    0.20443294007529891, -0.19147947244033353, 0.16900472663926791,
    -0.13905213177375081, 0.1047900103222502, -0.066392873538891159,
    0.022935322010529221 };

  int32_T x_size[2];
  int32_T fx_size[2];
  int32_T subs_tmp;
  int32_T subs_tmp_0;
  int32_T subs_tmp_tmp;
  int32_T exitg1;
  rtDW.interval_d[0] = -1.0;
  rtDW.interval_d[1] = 1.0;
  problem_type = 1;
  memset(&rtDW.interval_d[2], 0, 648U * sizeof(real_T));
  q = 0.0;
  split_i(rtDW.interval_d, 2, &ix, &pathlen);
  if (!(pathlen > 0.0)) {
    problem_type = 0;
  }

  if (problem_type == 0) {
    q = midpArea_o(fun_tunableEnvironment);
  } else {
    nsubs = ix - 1;
    for (b_k = 0; b_k <= ix - 2; b_k++) {
      subs_tmp = b_k << 1;
      rtDW.subs_d[subs_tmp] = rtDW.interval_d[b_k];
      rtDW.subs_d[1 + subs_tmp] = rtDW.interval_d[b_k + 1];
    }

    q_ok = 0.0;
    err_ok = 0.0;
    first_iteration = true;
    do {
      exitg1 = 0;
      x_size[0] = 1;
      x_size[1] = 15 * nsubs;
      ix = -1;
      for (b_k = 0; b_k < nsubs; b_k++) {
        subs_tmp = b_k << 1;
        tol = rtDW.subs_d[subs_tmp + 1];
        qsubs = (tol + rtDW.subs_d[subs_tmp]) / 2.0;
        tol = (tol - rtDW.subs_d[b_k << 1]) / 2.0;
        for (subs_tmp = 0; subs_tmp < 15; subs_tmp++) {
          ix++;
          rtDW.x_data_b[ix] = NODES[subs_tmp] * tol + qsubs;
        }
      }

      transfun_b(fun_tunableEnvironment, rtDW.x_data_b, x_size, first_iteration,
                 rtDW.fx_data_n, fx_size, &tooclose);
      if (tooclose) {
        exitg1 = 1;
      } else {
        qsubs = 0.0;
        ix = -1;
        for (b_k = 0; b_k < nsubs; b_k++) {
          rtDW.qsub_l[b_k] = 0.0;
          rtDW.errsub_o[b_k] = 0.0;
          for (subs_tmp = 0; subs_tmp < 15; subs_tmp++) {
            ix++;
            rtDW.qsub_l[b_k] += d[subs_tmp] * rtDW.fx_data_n[ix];
            rtDW.errsub_o[b_k] += e[subs_tmp] * rtDW.fx_data_n[ix];
          }

          subs_tmp = b_k << 1;
          tol = (rtDW.subs_d[subs_tmp + 1] - rtDW.subs_d[subs_tmp]) / 2.0;
          rtDW.qsub_l[b_k] *= tol;
          qsubs += rtDW.qsub_l[b_k];
          rtDW.errsub_o[b_k] *= tol;
        }

        q = qsubs + q_ok;
        tol = 1.0E-6 * std::abs(q);
        if ((1.0E-10 > tol) || rtIsNaN(tol)) {
          tol = 1.0E-10;
        }

        qsubs = 2.0 * tol / pathlen;
        errsub1norm = 0.0;
        ix = 0;
        for (b_k = 0; b_k < nsubs; b_k++) {
          abserrsubk = std::abs(rtDW.errsub_o[b_k]);
          subs_tmp = b_k << 1;
          subs_tmp_tmp = subs_tmp + 1;
          if (abserrsubk <= (rtDW.subs_d[subs_tmp_tmp] - rtDW.subs_d[subs_tmp]) /
              2.0 * qsubs) {
            err_ok += rtDW.errsub_o[b_k];
            q_ok += rtDW.qsub_l[b_k];
          } else {
            errsub1norm += abserrsubk;
            ix++;
            subs_tmp = (ix - 1) << 1;
            rtDW.subs_d[subs_tmp] = rtDW.subs_d[b_k << 1];
            rtDW.subs_d[1 + subs_tmp] = rtDW.subs_d[subs_tmp_tmp];
          }
        }

        qsubs = std::abs(err_ok) + errsub1norm;
        if ((!isfinite(q)) || (!isfinite(qsubs)) || (ix == 0) || (qsubs <= tol))
        {
          exitg1 = 1;
        } else {
          nsubs = ix << 1;
          if (nsubs > 650) {
            exitg1 = 1;
          } else {
            while (ix > 0) {
              b_k = (ix - 1) << 1;
              subs_tmp = b_k + 1;
              subs_tmp_tmp = ix << 1;
              subs_tmp_0 = (subs_tmp_tmp - 1) << 1;
              rtDW.subs_d[1 + subs_tmp_0] = rtDW.subs_d[subs_tmp];
              rtDW.subs_d[subs_tmp_0] = (rtDW.subs_d[subs_tmp] + rtDW.subs_d[b_k])
                / 2.0;
              subs_tmp = (subs_tmp_tmp - 2) << 1;
              rtDW.subs_d[1 + subs_tmp] = rtDW.subs_d[subs_tmp_0];
              rtDW.subs_d[subs_tmp] = rtDW.subs_d[b_k];
              ix--;
            }

            first_iteration = false;
          }
        }
      }
    } while (exitg1 == 0);
  }

  return q;
}

// Model step function
void MatlabControllerClass::step()
{
  int32_T i;
  int32_T dim;
  real32_T d[16];
  int32_T j;
  real32_T q1_q1;
  real32_T q0_q1;
  real32_T q0_q2;
  real32_T q0_q3;
  real32_T q1_q2;
  real32_T q1_q3;
  real32_T q2_q3;
  real32_T scale;
  real32_T absxk;
  real32_T t;
  real_T B_b[3];
  real_T N[3];
  real_T dist_remaining;
  real_T b_index;
  real_T dist_available;
  real_T arc_len_start_diff;
  real_T px;
  real32_T sin_Psi;
  real32_T cos_Psi;
  real32_T x;
  real32_T M_bg[9];
  real32_T q_yaw[4];
  real32_T unusedU0[4];
  real32_T A[32];
  real32_T d_0[8];
  boolean_T i_free[4];
  real32_T A_free_data[32];
  real32_T p_free_data[4];
  real_T p[4];
  real_T dist[4];
  real_T c_data[4];
  int8_T g_data[4];
  int8_T h_data[4];
  int8_T i_data[4];
  int8_T j_data[4];
  int32_T aoffset;
  int32_T b_aoffset;
  emxArray_real_T *coeffs_x;
  emxArray_real_T *coeffs_y;
  emxArray_real_T *coeffs_z;
  real_T b;
  real_T total_arc_length;
  real_T total_distance;
  real_T absxk_0;
  b_cell_wrap_1 tunableEnvironment[3];
  real32_T rtb_M_bg[9];
  real32_T rtb_G2[16];
  trajectoryBus rtb_Switch;
  real32_T rtb_y_m2[4];
  real32_T rtb_y_dt[3];
  real32_T rtb_y_i[3];
  real32_T rtb_Diff[3];
  real32_T rtb_u[4];
  real32_T rtb_vel[3];
  real32_T rtb_n_b_d_dt[3];
  real32_T rtb_n_b_d[3];
  real32_T rtb_y_i5[3];
  real32_T rtb_y_m[3];
  real32_T rtb_delta_k;
  real32_T rtb_phi_n;
  real_T rtb_Add_kh[5];
  real32_T rtb_Delta_ny_i[3];
  real32_T rtb_Diff_i[3];
  real32_T rtb_Add1[2];
  real_T rtb_Assignment1[5];
  real32_T rtb_Sum2_e[6];
  real32_T rtb_xy_g_dt2[2];
  real32_T rtb_TmpSignalConversionAtSFunct[3];
  real_T rtb_Assignment2[5];
  section rtb_active_section;
  real32_T rtb_Pad[18];
  real32_T rtb_uDLookupTable2[4];
  boolean_T rtb_Compare;
  real32_T rtb_q_red[4];
  real32_T rtb_Gain_j;
  real_T rtb_pos_o[3];
  boolean_T rtb_Compare_l;
  int32_T Pad_outAdd[2];
  int32_T Pad_inAdd[2];
  boolean_T rtb_FixPtRelationalOperator;
  real32_T A_tmp[16];
  boolean_T rtb_u_data[4];
  real32_T rtb_Pad_0[6];
  real_T tmp[5];
  real_T tmp_0[5];
  real32_T tmp_1[2];
  real32_T tmp_2[16];
  real32_T rtb_y_o[4];
  real32_T rtb_u_0[4];
  real32_T A_tmp_0[8];
  boolean_T rtb_u_1[4];
  real_T tmp_3[5];
  int32_T i_0;
  int32_T A_free_size[2];
  int32_T p_free_size;
  real32_T cmd_V_NED;
  boolean_T f;
  real32_T rtb_Compare_b;
  real32_T rtb_y_idx_3;
  real32_T rtb_y_idx_2;
  real32_T rtb_y_idx_1;
  real32_T rtb_y_idx_0;
  real32_T rtb_y_d_idx_1;
  real32_T rtb_y_d_idx_0;
  real32_T cmd_V_NED_idx_1;
  real32_T cmd_V_NED_idx_0;
  real32_T rtb_y_dt_b_idx_1;
  real32_T rtb_y_dt_b_idx_0;
  boolean_T e_idx_3;
  boolean_T e_idx_2;
  boolean_T e_idx_1;
  boolean_T e_idx_0;
  real32_T rtb_TSamp_idx_1;
  real32_T rtb_TSamp_idx_0;
  boolean_T f_idx_2;
  boolean_T f_idx_1;
  boolean_T f_idx_0;
  real_T rtb_M_tg_idx_0;
  int32_T g_size_idx_0;
  real32_T u0;
  real32_T u0_0;
  real32_T u0_1;
  real32_T rtb_M_bg_tmp;
  int32_T exitg1;
  boolean_T exitg2;
  boolean_T exitg3;

  // S-Function (sdsppad): '<S11>/Pad' incorporates:
  //   Inport: '<Root>/cmd'

  // Length of input columns to copy. Start with output column length and adjust.
  // Compute initial column start addresses in both input and output arrays.
  Pad_inAdd[1] = 0;
  Pad_outAdd[1] = 0;

  // Copy all needed input columns to the output array.
  for (i = 0; i < 6; i++) {
    // Compute starting address of next column to copy.
    dim = Pad_inAdd[1] << 2;
    aoffset = Pad_outAdd[1] * 3;

    // Copy a column from input to output array.
    for (j = 0; j < 3; j++) {
      rtb_Pad[aoffset + j] = rtU.cmd.waypoints[dim + j];
    }

    // Increment the column starting address.
    Pad_inAdd[1]++;
    Pad_outAdd[1]++;
    if ((rtConstP.Pad_outDims[1] - rtConstP.Pad_padAfter[1] == Pad_inAdd[1]) ||
        (Pad_outAdd[1] == rtConstP.Pad_outDims[1])) {
      // Reset index of this dim and continue to the next dim.
      Pad_inAdd[1] = 0;
      Pad_outAdd[1] = 0;
    } else {
      // Done incrementing address for this pass.
    }
  }

  // End of S-Function (sdsppad): '<S11>/Pad'

  // DiscreteIntegrator: '<S85>/Discrete-Time Integrator'
  // end CopyInToOut.
  rtb_y_idx_0 = rtDW.DiscreteTimeIntegrator_DSTATE_a[0];
  rtb_y_idx_1 = rtDW.DiscreteTimeIntegrator_DSTATE_a[1];
  rtb_y_idx_2 = rtDW.DiscreteTimeIntegrator_DSTATE_a[2];
  rtb_y_idx_3 = rtDW.DiscreteTimeIntegrator_DSTATE_a[3];

  // MATLAB Function: '<S48>/MATLAB Function' incorporates:
  //   Constant: '<S48>/Constant4'
  //   Constant: '<S48>/ny_du_dt'
  for (i = 0; i < 16; i++) {
    rtb_G2[i] = rtConstP.ny_du_dt_Value[i] / 0.0025F;
    d[i] = 0.0F;
  }

  // MATLAB Function: '<Root>/Quaternions to Rotation Matrix'
  scale = 1.29246971E-26F;

  // MinMax: '<S48>/Max' incorporates:
  //   DiscreteIntegrator: '<S85>/Discrete-Time Integrator'
  if (rtDW.DiscreteTimeIntegrator_DSTATE_a[0] > 0.1F) {
    t = rtDW.DiscreteTimeIntegrator_DSTATE_a[0];
  } else {
    t = 0.1F;
  }

  // MATLAB Function: '<S48>/MATLAB Function'
  d[0] = t / 0.283F;

  // MATLAB Function: '<Root>/Quaternions to Rotation Matrix' incorporates:
  //   Inport: '<Root>/measure'
  absxk = std::abs(rtU.measure.q_bg[0]);
  if (absxk > 1.29246971E-26F) {
    q1_q1 = 1.0F;
    scale = absxk;
  } else {
    t = absxk / 1.29246971E-26F;
    q1_q1 = t * t;
  }

  // MinMax: '<S48>/Max' incorporates:
  //   DiscreteIntegrator: '<S85>/Discrete-Time Integrator'
  if (rtDW.DiscreteTimeIntegrator_DSTATE_a[1] > 0.1F) {
    t = rtDW.DiscreteTimeIntegrator_DSTATE_a[1];
  } else {
    t = 0.1F;
  }

  // MATLAB Function: '<S48>/MATLAB Function'
  d[5] = t / 0.283F;

  // MATLAB Function: '<Root>/Quaternions to Rotation Matrix' incorporates:
  //   Inport: '<Root>/measure'
  absxk = std::abs(rtU.measure.q_bg[1]);
  if (absxk > scale) {
    t = scale / absxk;
    q1_q1 = q1_q1 * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    q1_q1 += t * t;
  }

  // MinMax: '<S48>/Max' incorporates:
  //   DiscreteIntegrator: '<S85>/Discrete-Time Integrator'
  if (rtDW.DiscreteTimeIntegrator_DSTATE_a[2] > 0.1F) {
    t = rtDW.DiscreteTimeIntegrator_DSTATE_a[2];
  } else {
    t = 0.1F;
  }

  // MATLAB Function: '<S48>/MATLAB Function'
  d[10] = t / 0.283F;

  // MATLAB Function: '<Root>/Quaternions to Rotation Matrix' incorporates:
  //   Inport: '<Root>/measure'
  absxk = std::abs(rtU.measure.q_bg[2]);
  if (absxk > scale) {
    t = scale / absxk;
    q1_q1 = q1_q1 * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    q1_q1 += t * t;
  }

  // MinMax: '<S48>/Max' incorporates:
  //   DiscreteIntegrator: '<S85>/Discrete-Time Integrator'
  if (rtDW.DiscreteTimeIntegrator_DSTATE_a[3] > 0.1F) {
    t = rtDW.DiscreteTimeIntegrator_DSTATE_a[3];
  } else {
    t = 0.1F;
  }

  // MATLAB Function: '<S48>/MATLAB Function'
  d[15] = t / 0.283F;

  // MATLAB Function: '<Root>/Quaternions to Rotation Matrix' incorporates:
  //   Inport: '<Root>/measure'
  absxk = std::abs(rtU.measure.q_bg[3]);
  if (absxk > scale) {
    t = scale / absxk;
    q1_q1 = q1_q1 * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    q1_q1 += t * t;
  }

  q1_q1 = scale * std::sqrt(q1_q1);
  if (rtIsNaNF(q1_q1) || (!(2.22044605E-16F < q1_q1))) {
    q1_q1 = 2.22044605E-16F;
  }

  q_yaw[0] = rtU.measure.q_bg[0] / q1_q1;
  q_yaw[1] = rtU.measure.q_bg[1] / q1_q1;
  q_yaw[2] = rtU.measure.q_bg[2] / q1_q1;
  q_yaw[3] = rtU.measure.q_bg[3] / q1_q1;
  scale = q_yaw[0] * q_yaw[0];
  q1_q1 = q_yaw[1] * q_yaw[1];
  absxk = q_yaw[2] * q_yaw[2];
  t = q_yaw[3] * q_yaw[3];
  q0_q1 = q_yaw[0] * q_yaw[1];
  q0_q2 = q_yaw[0] * q_yaw[2];
  q0_q3 = q_yaw[0] * q_yaw[3];
  q1_q2 = q_yaw[1] * q_yaw[2];
  q1_q3 = q_yaw[1] * q_yaw[3];
  q2_q3 = q_yaw[2] * q_yaw[3];
  rtb_M_bg[0] = ((scale + q1_q1) - absxk) - t;
  rtb_M_bg[3] = (q1_q2 + q0_q3) * 2.0F;
  rtb_M_bg[6] = (q1_q3 - q0_q2) * 2.0F;
  rtb_M_bg[1] = (q1_q2 - q0_q3) * 2.0F;
  rtb_M_bg_tmp = scale - q1_q1;
  rtb_M_bg[4] = (rtb_M_bg_tmp + absxk) - t;
  rtb_M_bg[7] = (q2_q3 + q0_q1) * 2.0F;
  rtb_M_bg[2] = (q1_q3 + q0_q2) * 2.0F;
  rtb_M_bg[5] = (q2_q3 - q0_q1) * 2.0F;
  rtb_M_bg[8] = (rtb_M_bg_tmp - absxk) + t;

  // RelationalOperator: '<S105>/Compare' incorporates:
  //   Constant: '<S105>/Constant'
  //   Inport: '<Root>/cmd'
  rtb_Compare = (rtU.cmd.mission_change > 0);

  // RelationalOperator: '<S103>/FixPt Relational Operator' incorporates:
  //   UnitDelay: '<S103>/Delay Input1'
  //
  //  Block description for '<S103>/Delay Input1':
  //
  //   Store in Global RAM
  rtb_FixPtRelationalOperator = ((int32_T)rtb_Compare > (int32_T)
    rtDW.DelayInput1_DSTATE);

  // Outputs for Enabled SubSystem: '<S98>/Subsystem2' incorporates:
  //   EnablePort: '<S104>/Enable'
  if (rtb_FixPtRelationalOperator) {
    // MATLAB Function: '<S104>/trajFromWaypoints' incorporates:
    //   Constant: '<S91>/Constant15'
    rtDW.traj = rtConstP.Constant15_Value;
    px = rtDW.traj.polynomial_degree;
    if ((!rtIsInf(rtDW.traj.polynomial_degree)) && (!rtIsNaN
         (rtDW.traj.polynomial_degree))) {
      if (rtDW.traj.polynomial_degree == 0.0) {
        b = 0.0;
      } else {
        b = std::fmod(rtDW.traj.polynomial_degree, 2.0);
        if (b == 0.0) {
          b = 0.0;
        } else {
          if (rtDW.traj.polynomial_degree < 0.0) {
            b += 2.0;
          }
        }
      }
    } else {
      b = (rtNaN);
    }

    if (!(b != 0.0)) {
      px = rtDW.traj.polynomial_degree + 1.0;
    }

    for (i_0 = 0; i_0 < 6; i_0++) {
      rtb_Pad_0[i_0] = rtb_Pad[3 * i_0];
    }

    emxInit_real_T(&coeffs_x, 2);

    // MATLAB Function: '<S104>/trajFromWaypoints' incorporates:
    //   Constant: '<S91>/Constant'
    polyInterpolation(rtb_Pad_0, px, true, coeffs_x, &b);
    for (i_0 = 0; i_0 < 6; i_0++) {
      rtb_Pad_0[i_0] = rtb_Pad[3 * i_0 + 1];
    }

    emxInit_real_T(&coeffs_y, 2);

    // MATLAB Function: '<S104>/trajFromWaypoints' incorporates:
    //   Constant: '<S91>/Constant'
    polyInterpolation(rtb_Pad_0, px, true, coeffs_y, &total_arc_length);
    for (i_0 = 0; i_0 < 6; i_0++) {
      rtb_Pad_0[i_0] = rtb_Pad[3 * i_0 + 2];
    }

    emxInit_real_T(&coeffs_z, 2);

    // MATLAB Function: '<S104>/trajFromWaypoints' incorporates:
    //   Constant: '<S91>/Constant'
    polyInterpolation(rtb_Pad_0, px, true, coeffs_z, &total_distance);
    rtDW.traj.num_sections_set = b;
    rtDW.traj.is_repeated_course = true;
    rtDW.traj.polynomial_degree = px;
    for (i = 0; i < (int32_T)b; i++) {
      total_arc_length = ((1.0 + (real_T)i) - 1.0) * (px + 1.0) + 1.0;
      for (i_0 = 0; i_0 < 6; i_0++) {
        aoffset = (int32_T)(total_arc_length + (real_T)i_0) - 1;
        rtDW.traj.sections[i].pos_x[i_0] = coeffs_x->data[aoffset];
        rtDW.traj.sections[i].pos_y[i_0] = coeffs_y->data[aoffset];
        rtDW.traj.sections[i].pos_z[i_0] = coeffs_z->data[aoffset];
      }
    }

    emxFree_real_T(&coeffs_z);
    emxFree_real_T(&coeffs_y);
    emxFree_real_T(&coeffs_x);

    // MATLAB Function: '<S104>/trajFromWaypoints'
    total_arc_length = 0.0;
    total_distance = 0.0;
    for (b_aoffset = 0; b_aoffset < (int32_T)rtDW.traj.num_sections_set;
         b_aoffset++) {
      b = 1.0 + (real_T)b_aoffset;
      if (1.0 + (real_T)b_aoffset > rtDW.traj.num_sections_set) {
        b = 1.0;
      }

      trajSectionGetPos(rtDW.traj.sections[(int32_T)b - 1].pos_x,
                        rtDW.traj.sections[(int32_T)b - 1].pos_y,
                        rtDW.traj.sections[(int32_T)b - 1].pos_z, 1.0, rtb_pos_o);
      trajSectionGetPos(rtDW.traj.sections[(int32_T)b - 1].pos_x,
                        rtDW.traj.sections[(int32_T)b - 1].pos_y,
                        rtDW.traj.sections[(int32_T)b - 1].pos_z, 0.0, B_b);
      b_index = 3.3121686421112381E-170;
      absxk_0 = std::abs(rtb_pos_o[0] - B_b[0]);
      if (absxk_0 > 3.3121686421112381E-170) {
        dist_remaining = 1.0;
        b_index = absxk_0;
      } else {
        px = absxk_0 / 3.3121686421112381E-170;
        dist_remaining = px * px;
      }

      absxk_0 = std::abs(rtb_pos_o[1] - B_b[1]);
      if (absxk_0 > b_index) {
        px = b_index / absxk_0;
        dist_remaining = dist_remaining * px * px + 1.0;
        b_index = absxk_0;
      } else {
        px = absxk_0 / b_index;
        dist_remaining += px * px;
      }

      absxk_0 = std::abs(rtb_pos_o[2] - B_b[2]);
      if (absxk_0 > b_index) {
        px = b_index / absxk_0;
        dist_remaining = dist_remaining * px * px + 1.0;
        b_index = absxk_0;
      } else {
        px = absxk_0 / b_index;
        dist_remaining += px * px;
      }

      dist_remaining = b_index * std::sqrt(dist_remaining);
      polyder_a(rtDW.traj.sections[(int32_T)b - 1].pos_x, tunableEnvironment[0].
                f1.data, tunableEnvironment[0].f1.size);
      polyder_a(rtDW.traj.sections[(int32_T)b - 1].pos_y, tunableEnvironment[1].
                f1.data, tunableEnvironment[1].f1.size);
      polyder_a(rtDW.traj.sections[(int32_T)b - 1].pos_z, tunableEnvironment[2].
                f1.data, tunableEnvironment[2].f1.size);
      px = quadgk_g(tunableEnvironment);
      rtDW.traj.sections[b_aoffset].arc_length = px;
      rtDW.traj.sections[b_aoffset].distance = dist_remaining;
      total_arc_length += px;
      total_distance += dist_remaining;
    }

    rtDW.traj.distance = total_distance;
    rtDW.traj.arc_length = total_arc_length;
  }

  // End of Outputs for SubSystem: '<S98>/Subsystem2'

  // Logic: '<S98>/OR' incorporates:
  //   Delay: '<S98>/Delay'
  rtb_FixPtRelationalOperator = (rtDW.Delay_DSTATE_f ||
    rtb_FixPtRelationalOperator);

  // Switch: '<S98>/Switch' incorporates:
  //   Constant: '<S91>/Constant15'
  if (rtb_FixPtRelationalOperator) {
    rtb_Switch = rtDW.traj;
  } else {
    rtb_Switch = rtConstP.Constant15_Value;
  }

  // End of Switch: '<S98>/Switch'

  // MATLAB Function: '<S91>/trajGetMatch' incorporates:
  //   Constant: '<S91>/Constant2'
  //   DataTypeConversion: '<S91>/Cast To Double'
  //   DotProduct: '<S91>/Dot Product'
  //   Inport: '<Root>/measure'
  //   Sqrt: '<S91>/Sqrt'
  trajGetMatch(&rtb_Switch, rtU.measure.s_Kg, &b, &total_arc_length, &px);
  total_arc_length = b;
  if (b > rtb_Switch.num_sections_set) {
    total_arc_length = 1.0;
  }

  rtb_active_section = rtb_Switch.sections[(int32_T)total_arc_length - 1];
  rtb_active_section.t = px;
  trajSectionGetFrenetSerretWithG(&rtb_active_section, (real_T)std::sqrt
    ((rtU.measure.V_Kg[0] * rtU.measure.V_Kg[0] + rtU.measure.V_Kg[1] *
      rtU.measure.V_Kg[1]) + rtU.measure.V_Kg[2] * rtU.measure.V_Kg[2]), 9.81,
    rtb_pos_o, B_b, N, &total_arc_length, &total_distance);

  // RelationalOperator: '<S2>/Compare' incorporates:
  //   Constant: '<S2>/Constant'
  //   Inport: '<Root>/cmd'
  rtb_Compare_l = (rtU.cmd.RC_pwm[6] >= 1500.0F);

  // MATLAB Function: '<S91>/trajGetMatch'
  rtb_M_tg_idx_0 = rtb_pos_o[0];

  // SampleTimeMath: '<S38>/TSamp' incorporates:
  //   Inport: '<Root>/measure'
  //
  //  About '<S38>/TSamp':
  //   y = u * K where K = 1 / ( w * Ts )
  rtb_M_bg_tmp = rtU.measure.omega_Kb[0] * 400.0F;

  // Sum: '<S38>/Diff' incorporates:
  //   UnitDelay: '<S38>/UD'
  //
  //  Block description for '<S38>/Diff':
  //
  //   Add in CPU
  //
  //  Block description for '<S38>/UD':
  //
  //   Store in Global RAM
  rtb_Diff[0] = rtb_M_bg_tmp - rtDW.UD_DSTATE[0];

  // DiscreteIntegrator: '<S73>/Discrete-Time Integrator y'
  rtb_y_i[0] = rtDW.DiscreteTimeIntegratory_DSTATE[0];

  // SampleTimeMath: '<S38>/TSamp' incorporates:
  //   Inport: '<Root>/measure'
  //
  //  About '<S38>/TSamp':
  //   y = u * K where K = 1 / ( w * Ts )
  rtb_TSamp_idx_0 = rtb_M_bg_tmp;
  rtb_M_bg_tmp = rtU.measure.omega_Kb[1] * 400.0F;

  // Sum: '<S38>/Diff' incorporates:
  //   UnitDelay: '<S38>/UD'
  //
  //  Block description for '<S38>/Diff':
  //
  //   Add in CPU
  //
  //  Block description for '<S38>/UD':
  //
  //   Store in Global RAM
  rtb_Diff[1] = rtb_M_bg_tmp - rtDW.UD_DSTATE[1];

  // DiscreteIntegrator: '<S73>/Discrete-Time Integrator y'
  rtb_y_i[1] = rtDW.DiscreteTimeIntegratory_DSTATE[1];

  // SampleTimeMath: '<S38>/TSamp' incorporates:
  //   Inport: '<Root>/measure'
  //
  //  About '<S38>/TSamp':
  //   y = u * K where K = 1 / ( w * Ts )
  rtb_TSamp_idx_1 = rtb_M_bg_tmp;
  rtb_M_bg_tmp = rtU.measure.omega_Kb[2] * 400.0F;

  // Sum: '<S38>/Diff' incorporates:
  //   UnitDelay: '<S38>/UD'
  //
  //  Block description for '<S38>/Diff':
  //
  //   Add in CPU
  //
  //  Block description for '<S38>/UD':
  //
  //   Store in Global RAM
  rtb_Diff[2] = rtb_M_bg_tmp - rtDW.UD_DSTATE[2];

  // DiscreteIntegrator: '<S73>/Discrete-Time Integrator y'
  rtb_y_i[2] = rtDW.DiscreteTimeIntegratory_DSTATE[2];

  // DiscreteIntegrator: '<S74>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S73>/Discrete-Time Integrator y'
  if (rtDW.DiscreteTimeIntegratory_IC_LOAD != 0) {
    rtDW.DiscreteTimeIntegratory_DSTAT_n[0] =
      rtDW.DiscreteTimeIntegratory_DSTATE[0];
    rtDW.DiscreteTimeIntegratory_DSTAT_n[1] =
      rtDW.DiscreteTimeIntegratory_DSTATE[1];
    rtDW.DiscreteTimeIntegratory_DSTAT_n[2] =
      rtDW.DiscreteTimeIntegratory_DSTATE[2];
  }

  rtb_y_m[0] = rtDW.DiscreteTimeIntegratory_DSTAT_n[0];

  // DiscreteIntegrator: '<S73>/Discrete-Time Integrator y_dt'
  rtb_y_dt[0] = rtDW.DiscreteTimeIntegratory_dt_DSTA[0];

  // DiscreteIntegrator: '<S74>/Discrete-Time Integrator y'
  rtb_y_m[1] = rtDW.DiscreteTimeIntegratory_DSTAT_n[1];

  // DiscreteIntegrator: '<S73>/Discrete-Time Integrator y_dt'
  rtb_y_dt[1] = rtDW.DiscreteTimeIntegratory_dt_DSTA[1];

  // DiscreteIntegrator: '<S74>/Discrete-Time Integrator y'
  rtb_y_m[2] = rtDW.DiscreteTimeIntegratory_DSTAT_n[2];

  // DiscreteIntegrator: '<S73>/Discrete-Time Integrator y_dt'
  rtb_y_dt[2] = rtDW.DiscreteTimeIntegratory_dt_DSTA[2];

  // DiscreteIntegrator: '<S75>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S73>/Discrete-Time Integrator y_dt'
  if (rtDW.DiscreteTimeIntegratory_IC_LO_g != 0) {
    rtDW.DiscreteTimeIntegratory_DSTAT_f[0] =
      rtDW.DiscreteTimeIntegratory_dt_DSTA[0];
    rtDW.DiscreteTimeIntegratory_DSTAT_f[1] =
      rtDW.DiscreteTimeIntegratory_dt_DSTA[1];
    rtDW.DiscreteTimeIntegratory_DSTAT_f[2] =
      rtDW.DiscreteTimeIntegratory_dt_DSTA[2];
  }

  rtb_y_i5[0] = rtDW.DiscreteTimeIntegratory_DSTAT_f[0];
  rtb_y_i5[1] = rtDW.DiscreteTimeIntegratory_DSTAT_f[1];
  rtb_y_i5[2] = rtDW.DiscreteTimeIntegratory_DSTAT_f[2];

  // MATLAB Function: '<S91>/MATLAB Function3' incorporates:
  //   Constant: '<S91>/target vel'
  total_arc_length = -1.0;
  total_distance = -1.0;
  dist_remaining = 10.0 / rtb_Switch.arc_length;
  if (dist_remaining < 0.0) {
    dist_remaining = std::ceil(dist_remaining);
  } else {
    dist_remaining = std::floor(dist_remaining);
  }

  dist_remaining = std::abs(10.0 - dist_remaining * rtb_Switch.arc_length);
  b_aoffset = 0;
  do {
    exitg1 = 0;
    i_0 = (int32_T)(rtb_Switch.num_sections_set + 1.0) - 1;
    if (b_aoffset <= i_0) {
      b_index = b + (real_T)b_aoffset;
      if (b_index > rtb_Switch.num_sections_set) {
        b_index -= rtb_Switch.num_sections_set;
      }

      trajSectionGetArcLength(rtb_Switch.sections[(int32_T)b_index - 1].pos_x,
        rtb_Switch.sections[(int32_T)b_index - 1].pos_y, rtb_Switch.sections
        [(int32_T)b_index - 1].pos_z, px, &absxk_0, &arc_len_start_diff);
      dist_available = rtb_Switch.sections[(int32_T)b_index - 1].arc_length -
        absxk_0;
      if (dist_remaining <= dist_available) {
        total_distance = b_index;
        total_arc_length = px - ((absxk_0 - dist_remaining) - absxk_0) /
          arc_len_start_diff;
        trajSectionGetArcLength(rtb_Switch.sections[(int32_T)b_index - 1].pos_x,
          rtb_Switch.sections[(int32_T)b_index - 1].pos_y, rtb_Switch.sections
          [(int32_T)b_index - 1].pos_z, total_arc_length, &b,
          &arc_len_start_diff);
        total_arc_length -= ((b - dist_remaining) - absxk_0) /
          arc_len_start_diff;
        trajSectionGetArcLength(rtb_Switch.sections[(int32_T)b_index - 1].pos_x,
          rtb_Switch.sections[(int32_T)b_index - 1].pos_y, rtb_Switch.sections
          [(int32_T)b_index - 1].pos_z, total_arc_length, &b,
          &arc_len_start_diff);
        total_arc_length -= ((b - dist_remaining) - absxk_0) /
          arc_len_start_diff;
        exitg1 = 1;
      } else {
        dist_remaining -= dist_available;
        px = 0.0;
        b_aoffset++;
      }
    } else {
      exitg1 = 1;
    }
  } while (exitg1 == 0);

  // MATLAB Function: '<S91>/MATLAB Function6' incorporates:
  //   MATLAB Function: '<S91>/MATLAB Function3'
  b = total_distance;
  if ((total_distance > rtb_Switch.num_sections_set) || (total_distance < 1.0))
  {
    b = 1.0;
  }

  px = rtb_Switch.sections[(int32_T)b - 1].pos_x[0];
  dist_remaining = rtb_Switch.sections[(int32_T)b - 1].pos_y[0];
  b_index = rtb_Switch.sections[(int32_T)b - 1].pos_z[0];
  for (b_aoffset = 0; b_aoffset < 5; b_aoffset++) {
    px = total_arc_length * px + rtb_Switch.sections[(int32_T)b - 1]
      .pos_x[b_aoffset + 1];
    dist_remaining = total_arc_length * dist_remaining + rtb_Switch.sections
      [(int32_T)b - 1].pos_y[b_aoffset + 1];
    b_index = total_arc_length * b_index + rtb_Switch.sections[(int32_T)b - 1].
      pos_z[b_aoffset + 1];

    // Assignment: '<S101>/Assignment2'
    rtb_Assignment2[b_aoffset] = 0.0;
  }

  rtb_pos_o[0] = px;

  // Assignment: '<S101>/Assignment2' incorporates:
  //   MATLAB Function: '<S91>/MATLAB Function6'
  rtb_Assignment2[0] = px;

  // DiscreteIntegrator: '<S101>/Discrete-Time Integrator'
  if (rtDW.DiscreteTimeIntegrator_IC_LOADI != 0) {
    for (i = 0; i < 5; i++) {
      rtDW.DiscreteTimeIntegrator_DSTATE[i] = rtb_Assignment2[i];
    }
  }

  for (i = 0; i < 5; i++) {
    // Gain: '<S101>/Gain2' incorporates:
    //   DiscreteIntegrator: '<S101>/Discrete-Time Integrator'
    tmp[i] = 0.0;
    for (aoffset = 0; aoffset < 5; aoffset++) {
      tmp[i] += rtConstP.pooled5[5 * aoffset + i] *
        rtDW.DiscreteTimeIntegrator_DSTATE[aoffset];
    }

    // Assignment: '<S100>/Assignment2'
    rtb_Assignment2[i] = 0.0;
  }

  // Assignment: '<S100>/Assignment2' incorporates:
  //   MATLAB Function: '<S91>/MATLAB Function6'
  rtb_Assignment2[0] = dist_remaining;

  // DiscreteIntegrator: '<S100>/Discrete-Time Integrator1'
  if (rtDW.DiscreteTimeIntegrator1_IC_LOAD != 0) {
    for (i = 0; i < 5; i++) {
      rtDW.DiscreteTimeIntegrator1_DSTATE[i] = rtb_Assignment2[i];
    }
  }

  for (i = 0; i < 5; i++) {
    // Gain: '<S100>/Gain2' incorporates:
    //   DiscreteIntegrator: '<S100>/Discrete-Time Integrator1'
    tmp_0[i] = 0.0;
    for (aoffset = 0; aoffset < 5; aoffset++) {
      tmp_0[i] += rtConstP.pooled5[5 * aoffset + i] *
        rtDW.DiscreteTimeIntegrator1_DSTATE[aoffset];
    }

    // Assignment: '<S102>/Assignment2'
    rtb_Assignment2[i] = 0.0;
  }

  // Assignment: '<S102>/Assignment2' incorporates:
  //   MATLAB Function: '<S91>/MATLAB Function6'
  rtb_Assignment2[0] = b_index;

  // DiscreteIntegrator: '<S102>/Discrete-Time Integrator'
  if (rtDW.DiscreteTimeIntegrator_IC_LOA_d != 0) {
    for (i = 0; i < 5; i++) {
      rtDW.DiscreteTimeIntegrator_DSTATE_o[i] = rtb_Assignment2[i];
    }
  }

  // Gain: '<S102>/Gain2' incorporates:
  //   DiscreteIntegrator: '<S102>/Discrete-Time Integrator'
  for (aoffset = 0; aoffset < 5; aoffset++) {
    rtb_Assignment1[aoffset] = 0.0;
    for (dim = 0; dim < 5; dim++) {
      rtb_Assignment1[aoffset] += rtConstP.pooled5[5 * dim + aoffset] *
        rtDW.DiscreteTimeIntegrator_DSTATE_o[dim];
    }
  }

  // Gain: '<S24>/Gain3' incorporates:
  //   Gain: '<Root>/Gain'
  //   Inport: '<Root>/cmd'
  //   MATLAB Function: '<S12>/MATLAB Function'
  //   MATLAB Function: '<S24>/MATLAB Function1'
  q1_q1 = std::sqrt(rtU.cmd.roll * rtU.cmd.roll + -rtU.cmd.pitch *
                    -rtU.cmd.pitch) * 15.0F;

  // Gain: '<S36>/1//T' incorporates:
  //   DiscreteIntegrator: '<S36>/Discrete-Time Integrator'
  //   Gain: '<Root>/Gain'
  //   Inport: '<Root>/cmd'
  //   Product: '<S24>/Product'
  //   Sum: '<S36>/Sum2'
  rtb_y_dt_b_idx_0 = (-rtU.cmd.pitch * q1_q1 -
                      rtDW.DiscreteTimeIntegrator_DSTATE_b[0]) * 1.25F;

  // Saturate: '<S36>/Saturation'
  if (rtb_y_dt_b_idx_0 > 12.0F) {
    rtb_y_dt_b_idx_0 = 12.0F;
  } else {
    if (rtb_y_dt_b_idx_0 < -12.0F) {
      rtb_y_dt_b_idx_0 = -12.0F;
    }
  }

  // Gain: '<S36>/1//T' incorporates:
  //   DiscreteIntegrator: '<S36>/Discrete-Time Integrator'
  //   Inport: '<Root>/cmd'
  //   MATLAB Function: '<S12>/MATLAB Function'
  //   Product: '<S24>/Product'
  //   Sum: '<S36>/Sum2'
  rtb_y_dt_b_idx_1 = (rtU.cmd.roll * q1_q1 -
                      rtDW.DiscreteTimeIntegrator_DSTATE_b[1]) * 1.25F;

  // Saturate: '<S36>/Saturation'
  if (rtb_y_dt_b_idx_1 > 12.0F) {
    rtb_y_dt_b_idx_1 = 12.0F;
  } else {
    if (rtb_y_dt_b_idx_1 < -12.0F) {
      rtb_y_dt_b_idx_1 = -12.0F;
    }
  }

  // DiscreteIntegrator: '<S36>/Discrete-Time Integrator'
  rtb_y_d_idx_0 = rtDW.DiscreteTimeIntegrator_DSTATE_b[0];

  // Switch: '<S9>/Switch' incorporates:
  //   DataTypeConversion: '<S91>/Cast To Single1'
  //   Gain: '<S100>/Gain2'
  //   Gain: '<S101>/Gain2'
  //   SignalConversion: '<S97>/ConcatBufferAtVector ConcatenateIn1'
  if (rtb_Compare_l) {
    rtb_xy_g_dt2[0] = rtb_y_dt_b_idx_0;
    rtb_xy_g_dt2[1] = rtb_y_dt_b_idx_1;
  } else {
    rtb_xy_g_dt2[0] = (real32_T)tmp[2];
    rtb_xy_g_dt2[1] = (real32_T)tmp_0[2];
  }

  // DiscreteIntegrator: '<S36>/Discrete-Time Integrator'
  rtb_y_d_idx_1 = rtDW.DiscreteTimeIntegrator_DSTATE_b[1];

  // SampleTimeMath: '<S8>/TSamp' incorporates:
  //   Inport: '<Root>/measure'
  //
  //  About '<S8>/TSamp':
  //   y = u * K where K = 1 / ( w * Ts )
  cmd_V_NED = rtU.measure.V_Kg[0] * 400.0F;

  // Sum: '<S8>/Diff' incorporates:
  //   UnitDelay: '<S8>/UD'
  //
  //  Block description for '<S8>/Diff':
  //
  //   Add in CPU
  //
  //  Block description for '<S8>/UD':
  //
  //   Store in Global RAM
  rtb_Diff_i[0] = cmd_V_NED - rtDW.UD_DSTATE_c[0];

  // SampleTimeMath: '<S8>/TSamp' incorporates:
  //   Inport: '<Root>/measure'
  //
  //  About '<S8>/TSamp':
  //   y = u * K where K = 1 / ( w * Ts )
  cmd_V_NED_idx_0 = cmd_V_NED;
  cmd_V_NED = rtU.measure.V_Kg[1] * 400.0F;

  // Sum: '<S8>/Diff' incorporates:
  //   UnitDelay: '<S8>/UD'
  //
  //  Block description for '<S8>/Diff':
  //
  //   Add in CPU
  //
  //  Block description for '<S8>/UD':
  //
  //   Store in Global RAM
  rtb_Diff_i[1] = cmd_V_NED - rtDW.UD_DSTATE_c[1];

  // SampleTimeMath: '<S8>/TSamp' incorporates:
  //   Inport: '<Root>/measure'
  //
  //  About '<S8>/TSamp':
  //   y = u * K where K = 1 / ( w * Ts )
  cmd_V_NED_idx_1 = cmd_V_NED;
  cmd_V_NED = rtU.measure.V_Kg[2] * 400.0F;

  // Sum: '<S8>/Diff' incorporates:
  //   UnitDelay: '<S8>/UD'
  //
  //  Block description for '<S8>/Diff':
  //
  //   Add in CPU
  //
  //  Block description for '<S8>/UD':
  //
  //   Store in Global RAM
  rtb_Diff_i[2] = cmd_V_NED - rtDW.UD_DSTATE_c[2];

  // Sum: '<S26>/Add' incorporates:
  //   DiscreteIntegrator: '<S37>/Discrete-Time Integrator y'
  //   Inport: '<Root>/measure'
  rtb_Sum2_e[0] = rtDW.DiscreteTimeIntegratory_DSTAT_l[0] - rtU.measure.s_Kg[0];
  rtb_Sum2_e[2] = rtDW.DiscreteTimeIntegratory_DSTAT_l[2] - rtU.measure.V_Kg[0];
  rtb_Sum2_e[4] = rtDW.DiscreteTimeIntegratory_DSTAT_l[4] - rtb_Diff_i[0];
  rtb_Sum2_e[1] = rtDW.DiscreteTimeIntegratory_DSTAT_l[1] - rtU.measure.s_Kg[1];
  rtb_Sum2_e[3] = rtDW.DiscreteTimeIntegratory_DSTAT_l[3] - rtU.measure.V_Kg[1];
  rtb_Sum2_e[5] = rtDW.DiscreteTimeIntegratory_DSTAT_l[5] - rtb_Diff_i[1];

  // Gain: '<S26>/Gain'
  for (aoffset = 0; aoffset < 6; aoffset++) {
    // Saturate: '<S26>/Saturation1'
    if (rtb_Sum2_e[aoffset] > rtConstP.Saturation1_UpperSat[aoffset]) {
      rtb_Pad_0[aoffset] = rtConstP.Saturation1_UpperSat[aoffset];
    } else if (rtb_Sum2_e[aoffset] < rtConstP.Saturation1_LowerSat[aoffset]) {
      rtb_Pad_0[aoffset] = rtConstP.Saturation1_LowerSat[aoffset];
    } else {
      rtb_Pad_0[aoffset] = rtb_Sum2_e[aoffset];
    }

    // End of Saturate: '<S26>/Saturation1'
  }

  // DiscreteIntegrator: '<S84>/Discrete-Time Integrator'
  sin_Psi = rtDW.DiscreteTimeIntegrator_DSTAT_ad;

  // Gain: '<S84>/1//T' incorporates:
  //   DiscreteIntegrator: '<S84>/Discrete-Time Integrator'
  //   Inport: '<Root>/cmd'
  //   Lookup_n-D: '<S82>/1-D Lookup Table'
  //   Sum: '<S84>/Sum2'
  cos_Psi = (look1_iflf_binlx(rtU.cmd.thr, rtConstP.uDLookupTable_bp01Data,
              rtConstP.uDLookupTable_tableData, 2U) -
             rtDW.DiscreteTimeIntegrator_DSTAT_ad) * 3.33333325F;

  // Switch: '<S44>/Switch' incorporates:
  //   DataTypeConversion: '<S91>/Cast To Single1'
  //   Gain: '<S102>/Gain2'
  //   SignalConversion: '<S97>/ConcatBufferAtVector ConcatenateIn3'
  if (rtb_Compare_l) {
    q1_q3 = cos_Psi;
  } else {
    q1_q3 = (real32_T)rtb_Assignment1[2];
  }

  for (i = 0; i < 2; i++) {
    // Gain: '<S26>/Gain'
    tmp_1[i] = 0.0F;
    for (aoffset = 0; aoffset < 6; aoffset++) {
      tmp_1[i] += rtConstP.Gain_Gain_h[(aoffset << 1) + i] * rtb_Pad_0[aoffset];
    }

    // Saturate: '<S26>/Saturation' incorporates:
    //   Gain: '<S26>/Gain'
    //   Sum: '<S9>/Add1'
    if (tmp_1[i] > 9.0F) {
      t = 9.0F;
    } else if (tmp_1[i] < -9.0F) {
      t = -9.0F;
    } else {
      t = tmp_1[i];
    }

    // End of Saturate: '<S26>/Saturation'

    // Sum: '<S9>/Add1'
    rtb_delta_k = rtb_xy_g_dt2[i] + t;

    // SignalConversion: '<S27>/TmpSignal ConversionAt SFunction Inport1' incorporates:
    //   MATLAB Function: '<S23>/MATLAB Function1'
    rtb_TmpSignalConversionAtSFunct[i] = rtb_delta_k;

    // Sum: '<S9>/Add1'
    rtb_Add1[i] = rtb_delta_k;
  }

  // SignalConversion: '<S27>/TmpSignal ConversionAt SFunction Inport1' incorporates:
  //   MATLAB Function: '<S23>/MATLAB Function1'
  rtb_TmpSignalConversionAtSFunct[2] = q1_q3;

  // MATLAB Function: '<S23>/MATLAB Function1' incorporates:
  //   MATLAB Function: '<S23>/MATLAB Function4'
  //   SignalConversion: '<S27>/TmpSignal ConversionAt SFunction Inport1'
  scale = 1.29246971E-26F;
  absxk = std::abs(rtb_TmpSignalConversionAtSFunct[0]);
  if (absxk > 1.29246971E-26F) {
    q1_q1 = 1.0F;
    scale = absxk;
  } else {
    t = absxk / 1.29246971E-26F;
    q1_q1 = t * t;
  }

  rtb_vel[0] = rtb_Diff_i[0];
  absxk = std::abs(rtb_TmpSignalConversionAtSFunct[1]);
  if (absxk > scale) {
    t = scale / absxk;
    q1_q1 = q1_q1 * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    q1_q1 += t * t;
  }

  rtb_vel[1] = rtb_Diff_i[1];
  absxk = std::abs(q1_q3 - 9.81F);
  if (absxk > scale) {
    t = scale / absxk;
    q1_q1 = q1_q1 * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    q1_q1 += t * t;
  }

  rtb_vel[2] = rtb_Diff_i[2] - 9.81F;
  q1_q1 = scale * std::sqrt(q1_q1);

  // MATLAB Function: '<S23>/MATLAB Function4'
  scale = 1.29246971E-26F;
  absxk = std::abs(rtb_Diff_i[0]);
  if (absxk > 1.29246971E-26F) {
    rtb_delta_k = 1.0F;
    scale = absxk;
  } else {
    t = absxk / 1.29246971E-26F;
    rtb_delta_k = t * t;
  }

  absxk = std::abs(rtb_Diff_i[1]);
  if (absxk > scale) {
    t = scale / absxk;
    rtb_delta_k = rtb_delta_k * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    rtb_delta_k += t * t;
  }

  absxk = std::abs(rtb_Diff_i[2] - 9.81F);
  if (absxk > scale) {
    t = scale / absxk;
    rtb_delta_k = rtb_delta_k * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    rtb_delta_k += t * t;
  }

  rtb_delta_k = scale * std::sqrt(rtb_delta_k);

  // Sum: '<S14>/Add' incorporates:
  //   MATLAB Function: '<S14>/MATLAB Function1'
  //   MATLAB Function: '<S23>/MATLAB Function1'
  //   MATLAB Function: '<S23>/MATLAB Function4'
  //   Sum: '<S23>/Add2'
  for (aoffset = 0; aoffset < 3; aoffset++) {
    rtb_Delta_ny_i[aoffset] = ((rtb_M_bg[3 * aoffset + 1] * 0.0F + rtb_M_bg[3 *
      aoffset] * 0.0F) + -rtb_M_bg[3 * aoffset + 2]) +
      ((rtb_TmpSignalConversionAtSFunct[aoffset] - rtConstB.Gain2[aoffset]) /
       q1_q1 - rtb_vel[aoffset] / rtb_delta_k);
  }

  // End of Sum: '<S14>/Add'

  // MATLAB Function: '<S14>/MATLAB Function'
  scale = 1.29246971E-26F;
  absxk = std::abs(rtb_Delta_ny_i[0]);
  if (absxk > 1.29246971E-26F) {
    q1_q1 = 1.0F;
    scale = absxk;
  } else {
    t = absxk / 1.29246971E-26F;
    q1_q1 = t * t;
  }

  absxk = std::abs(rtb_Delta_ny_i[1]);
  if (absxk > scale) {
    t = scale / absxk;
    q1_q1 = q1_q1 * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    q1_q1 += t * t;
  }

  absxk = std::abs(rtb_Delta_ny_i[2]);
  if (absxk > scale) {
    t = scale / absxk;
    q1_q1 = q1_q1 * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    q1_q1 += t * t;
  }

  q1_q1 = scale * std::sqrt(q1_q1);
  rtb_Delta_ny_i[0] /= q1_q1;
  rtb_Delta_ny_i[1] /= q1_q1;
  rtb_Delta_ny_i[2] /= q1_q1;

  // End of MATLAB Function: '<S14>/MATLAB Function'

  // MATLAB Function: '<S14>/MATLAB Function4'
  MATLABFunction4(rtb_Delta_ny_i, &rtb_phi_n, &rtb_delta_k);

  // Saturate: '<S7>/Saturation'
  if (rtb_phi_n > 1.04719758F) {
    rtb_phi_n = 1.04719758F;
  } else {
    if (rtb_phi_n < 0.0F) {
      rtb_phi_n = 0.0F;
    }
  }

  // End of Saturate: '<S7>/Saturation'

  // Gain: '<S7>/Gain'
  rtb_Gain_j = 0.95492965F * rtb_phi_n;

  // MATLAB Function: '<S7>/MATLAB Function1'
  rtb_phi_n = rtb_Gain_j * std::sin(rtb_delta_k);
  rtb_delta_k = -rtb_Gain_j * std::cos(rtb_delta_k);

  // Gain: '<S58>/lean_angle_max' incorporates:
  //   MATLAB Function: '<S10>/MATLAB Function1'
  rtb_Gain_j = std::sqrt(rtb_phi_n * rtb_phi_n + rtb_delta_k * rtb_delta_k) *
    1.04719758F;
  rtb_phi_n = rt_atan2f_snf(rtb_phi_n, -rtb_delta_k);

  // MATLAB Function: '<S58>/lean angles 2 lean vector'
  rtb_delta_k = std::sin(rtb_Gain_j);

  // Gain: '<S73>/omega^2' incorporates:
  //   DiscreteIntegrator: '<S73>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S73>/Discrete-Time Integrator y_dt'
  //   Gain: '<S73>/2*d//omega'
  //   MATLAB Function: '<S58>/lean angles 2 lean vector'
  //   Sum: '<S73>/Sum2'
  //   Sum: '<S73>/Sum3'
  rtb_TmpSignalConversionAtSFunct[0] = (rtb_delta_k * std::cos(rtb_phi_n) -
    (0.166666672F * rtDW.DiscreteTimeIntegratory_dt_DSTA[0] +
     rtDW.DiscreteTimeIntegratory_DSTATE[0])) * 144.0F;
  rtb_TmpSignalConversionAtSFunct[1] = (rtb_delta_k * std::sin(rtb_phi_n) -
    (0.166666672F * rtDW.DiscreteTimeIntegratory_dt_DSTA[1] +
     rtDW.DiscreteTimeIntegratory_DSTATE[1])) * 144.0F;
  rtb_TmpSignalConversionAtSFunct[2] = (-std::cos(rtb_Gain_j) - (0.166666672F *
    rtDW.DiscreteTimeIntegratory_dt_DSTA[2] +
    rtDW.DiscreteTimeIntegratory_DSTATE[2])) * 144.0F;

  // DiscreteIntegrator: '<S76>/Discrete-Time Integrator y'
  if (rtDW.DiscreteTimeIntegratory_IC_LO_n != 0) {
    rtDW.DiscreteTimeIntegratory_DSTA_ll[0] = rtb_TmpSignalConversionAtSFunct[0];
    rtDW.DiscreteTimeIntegratory_DSTA_ll[1] = rtb_TmpSignalConversionAtSFunct[1];
    rtDW.DiscreteTimeIntegratory_DSTA_ll[2] = rtb_TmpSignalConversionAtSFunct[2];
  }

  rtb_Delta_ny_i[0] = rtDW.DiscreteTimeIntegratory_DSTA_ll[0];
  rtb_Delta_ny_i[1] = rtDW.DiscreteTimeIntegratory_DSTA_ll[1];
  rtb_Delta_ny_i[2] = rtDW.DiscreteTimeIntegratory_DSTA_ll[2];

  // MATLAB Function: '<S58>/MATLAB Function1' incorporates:
  //   Inport: '<Root>/measure'
  MATLABFunction(rtU.measure.omega_Kb, rtb_Diff, rtb_M_bg, rtb_y_m, rtb_y_i5,
                 rtb_Delta_ny_i, rtb_n_b_d, rtb_n_b_d_dt, rtb_vel);

  // MATLAB Function: '<S91>/MATLAB Function1' incorporates:
  //   MATLAB Function: '<S91>/trajGetMatch'
  px = std::cos(rt_atan2d_snf(-rtb_pos_o[2], std::sqrt(rtb_M_tg_idx_0 *
    rtb_M_tg_idx_0 + rtb_pos_o[1] * rtb_pos_o[1])));
  px = rt_atan2d_snf(rtb_pos_o[1] / px, rtb_M_tg_idx_0 / px);

  // DiscreteIntegrator: '<S58>/Discrete-Time Integrator2' incorporates:
  //   MATLAB Function: '<Root>/Rotations matrix to Euler angles'
  if (rtDW.DiscreteTimeIntegrator2_IC_LOAD != 0) {
    rtDW.DiscreteTimeIntegrator2_DSTATE = rt_atan2f_snf(rtb_M_bg[3], rtb_M_bg[0]);
  }

  // Switch: '<S43>/Switch' incorporates:
  //   DataTypeConversion: '<S91>/Cast To Single3'
  //   DiscreteIntegrator: '<S58>/Discrete-Time Integrator2'
  if (rtb_Compare_l) {
    rtb_delta_k = rtDW.DiscreteTimeIntegrator2_DSTATE;
  } else {
    rtb_delta_k = (real32_T)px;
  }

  // MATLAB Function: '<S53>/wrap angle'
  x = std::abs(rtb_delta_k);
  rtb_phi_n = x - std::floor(x / 6.28318548F) * 6.28318548F;

  // MATLAB Function: '<S54>/DCM to quaternions'
  DCMtoquaternions(rtb_M_bg, rtb_u);

  // MATLAB Function: '<S54>/MATLAB Function1'
  quatNormalize(rtb_u, q_yaw);
  M_bg[2] = (q_yaw[1] * q_yaw[3] + q_yaw[0] * q_yaw[2]) * 2.0F;
  M_bg[8] = ((q_yaw[0] * q_yaw[0] - q_yaw[1] * q_yaw[1]) - q_yaw[2] * q_yaw[2])
    + q_yaw[3] * q_yaw[3];
  if ((!rtIsNaNF(M_bg[8])) && (1.0F > M_bg[8])) {
    q0_q1 = M_bg[8];
  } else {
    q0_q1 = 1.0F;
  }

  if (!(-1.0F < q0_q1)) {
    q0_q1 = -1.0F;
  }

  rtb_Gain_j = std::acos(q0_q1);
  x = std::sin(rtb_Gain_j);
  x = x * x - M_bg[2] * M_bg[2];
  if ((q_yaw[2] * q_yaw[3] - q_yaw[0] * q_yaw[1]) * 2.0F >= 0.0F) {
    aoffset = -1;
  } else {
    aoffset = 1;
  }

  if (rtIsNaNF(x) || (!(0.0F < x))) {
    x = 0.0F;
  }

  x = rt_atan2f_snf((real32_T)aoffset * std::sqrt(x), -M_bg[2]);
  scale = std::sin(rtb_Gain_j / 2.0F);
  rtb_q_red[0] = std::cos(rtb_Gain_j / 2.0F);
  rtb_q_red[1] = std::sin(x) * scale;
  rtb_q_red[2] = -std::cos(x) * scale;
  rtb_q_red[3] = 0.0F * scale;
  rtb_Gain_j = ((rtb_q_red[0] * rtb_q_red[0] + rtb_q_red[1] * rtb_q_red[1]) +
                rtb_q_red[2] * rtb_q_red[2]) + rtb_q_red[3] * rtb_q_red[3];
  if (rtIsNaNF(rtb_Gain_j) || (!(2.22044605E-16F < rtb_Gain_j))) {
    rtb_Gain_j = 2.22044605E-16F;
  }

  q_yaw[0] = rtb_q_red[0] / rtb_Gain_j;
  q_yaw[1] = -rtb_q_red[1] / rtb_Gain_j;
  q_yaw[2] = -rtb_q_red[2] / rtb_Gain_j;
  q_yaw[3] = -rtb_q_red[3] / rtb_Gain_j;
  rtb_q_red[0] = ((q_yaw[0] * rtb_u[0] - q_yaw[1] * rtb_u[1]) - q_yaw[2] *
                  rtb_u[2]) - q_yaw[3] * rtb_u[3];
  rtb_q_red[1] = (q_yaw[0] * rtb_u[1] + rtb_u[0] * q_yaw[1]) + (q_yaw[2] *
    rtb_u[3] - q_yaw[3] * rtb_u[2]);
  rtb_q_red[2] = (q_yaw[0] * rtb_u[2] + rtb_u[0] * q_yaw[2]) + (q_yaw[3] *
    rtb_u[1] - q_yaw[1] * rtb_u[3]);
  rtb_q_red[3] = (q_yaw[0] * rtb_u[3] + rtb_u[0] * q_yaw[3]) + (q_yaw[1] *
    rtb_u[2] - q_yaw[2] * rtb_u[1]);
  quatNormalize(rtb_q_red, q_yaw);
  if (q_yaw[3] < 0.0F) {
    q2_q3 = -1.0F;
  } else if (q_yaw[3] > 0.0F) {
    q2_q3 = 1.0F;
  } else if (q_yaw[3] == 0.0F) {
    q2_q3 = 0.0F;
  } else {
    q2_q3 = (rtNaNF);
  }

  rtb_Gain_j = q2_q3 * q_yaw[0];
  if (rtb_Gain_j > 1.0F) {
    rtb_Gain_j = 1.0F;
  }

  if (rtb_Gain_j < -1.0F) {
    rtb_Gain_j = -1.0F;
  }

  rtb_Gain_j = 2.0F * std::acos(rtb_Gain_j);

  // End of MATLAB Function: '<S54>/MATLAB Function1'

  // MATLAB Function: '<S53>/wrap angle1'
  x = std::abs(rtb_Gain_j);
  x -= std::floor(x / 6.28318548F) * 6.28318548F;

  // MATLAB Function: '<S53>/wrap angle'
  if (!(rtb_delta_k >= 0.0F)) {
    rtb_phi_n = 6.28318548F - rtb_phi_n;
  }

  // MATLAB Function: '<S53>/wrap angle1'
  if (!(rtb_Gain_j >= 0.0F)) {
    x = 6.28318548F - x;
  }

  // MATLAB Function: '<S53>/angle error'
  q0_q3 = rtb_phi_n - x;
  if (q0_q3 > 3.1415926535897931) {
    q0_q3 -= 6.28318548F;
  } else {
    if (q0_q3 < -3.1415926535897931) {
      q0_q3 += 6.28318548F;
    }
  }

  // End of MATLAB Function: '<S53>/angle error'

  // SampleTimeMath: '<S89>/TSamp' incorporates:
  //   DataTypeConversion: '<S91>/Cast To Single3'
  //
  //  About '<S89>/TSamp':
  //   y = u * K where K = 1 / ( w * Ts )
  rtb_delta_k = (real32_T)px * 400.0F;

  // Sum: '<S89>/Diff' incorporates:
  //   UnitDelay: '<S89>/UD'
  //
  //  Block description for '<S89>/Diff':
  //
  //   Add in CPU
  //
  //  Block description for '<S89>/UD':
  //
  //   Store in Global RAM
  q1_q1 = rtb_delta_k - rtDW.UD_DSTATE_h;

  // Switch: '<S43>/Switch' incorporates:
  //   DiscreteIntegrator: '<S72>/Discrete-Time Integrator'
  if (rtb_Compare_l) {
    rtb_Compare_b = rtDW.DiscreteTimeIntegrator_DSTATE_g;
  } else {
    rtb_Compare_b = q1_q1;
  }

  // Gain: '<S72>/1//T' incorporates:
  //   DiscreteIntegrator: '<S72>/Discrete-Time Integrator'
  //   Gain: '<S58>/r_max'
  //   Inport: '<Root>/cmd'
  //   Sum: '<S72>/Sum2'
  rtb_phi_n = (5.23598766F * rtU.cmd.yaw - rtDW.DiscreteTimeIntegrator_DSTATE_g)
    * 10.0F;

  // SampleTimeMath: '<S90>/TSamp'
  //
  //  About '<S90>/TSamp':
  //   y = u * K where K = 1 / ( w * Ts )
  rtb_Gain_j = q1_q1 * 400.0F;

  // Switch: '<S43>/Switch' incorporates:
  //   Sum: '<S90>/Diff'
  //   UnitDelay: '<S90>/UD'
  //
  //  Block description for '<S90>/Diff':
  //
  //   Add in CPU
  //
  //  Block description for '<S90>/UD':
  //
  //   Store in Global RAM
  if (rtb_Compare_l) {
    scale = rtb_phi_n;
  } else {
    scale = rtb_Gain_j - rtDW.UD_DSTATE_m;
  }

  // Sum: '<S53>/error1 9'
  q1_q1 = scale - rtb_Diff[2];

  // SignalConversion: '<S55>/TmpSignal ConversionAtGainInport1' incorporates:
  //   Inport: '<Root>/measure'
  //   Sum: '<S53>/error1 5'
  //   Sum: '<S53>/error1 6'
  //   Sum: '<S53>/error1 8'
  //   Switch: '<S43>/Switch'
  M_bg[2] = q0_q3;
  M_bg[5] = rtb_Compare_b - rtU.measure.omega_Kb[2];
  M_bg[0] = rtb_n_b_d[0];
  M_bg[3] = rtb_n_b_d_dt[0];
  M_bg[6] = rtb_vel[0];
  M_bg[1] = rtb_n_b_d[1];
  M_bg[4] = rtb_n_b_d_dt[1];
  M_bg[7] = rtb_vel[1];
  M_bg[8] = q1_q1;

  // DiscreteIntegrator: '<S82>/Discrete-Time Integrator3' incorporates:
  //   Inport: '<Root>/measure'
  if (rtDW.DiscreteTimeIntegrator3_IC_LOAD != 0) {
    rtDW.DiscreteTimeIntegrator3_DSTATE = rtU.measure.s_Kg[2];
  }

  if (rtb_Compare_l && (rtDW.DiscreteTimeIntegrator3_PrevRes <= 0)) {
    rtDW.DiscreteTimeIntegrator3_DSTATE = rtU.measure.s_Kg[2];
  }

  // Switch: '<S44>/Switch' incorporates:
  //   DataTypeConversion: '<S91>/Cast To Single'
  //   DiscreteIntegrator: '<S82>/Discrete-Time Integrator3'
  //   Gain: '<S102>/Gain2'
  //   SignalConversion: '<S97>/ConcatBufferAtVector ConcatenateIn3'
  if (rtb_Compare_l) {
    rtb_Compare_b = rtDW.DiscreteTimeIntegrator3_DSTATE;
  } else {
    rtb_Compare_b = (real32_T)rtb_Assignment1[0];
  }

  // Sum: '<S77>/error1 3' incorporates:
  //   Inport: '<Root>/measure'
  q0_q1 = rtb_Compare_b - rtU.measure.s_Kg[2];

  // Switch: '<S44>/Switch' incorporates:
  //   DataTypeConversion: '<S91>/Cast To Single2'
  //   DiscreteIntegrator: '<S84>/Discrete-Time Integrator'
  //   Gain: '<S102>/Gain2'
  //   SignalConversion: '<S97>/ConcatBufferAtVector ConcatenateIn3'
  if (rtb_Compare_l) {
    rtb_Compare_b = rtDW.DiscreteTimeIntegrator_DSTAT_ad;
  } else {
    rtb_Compare_b = (real32_T)rtb_Assignment1[1];
  }

  // Sum: '<S77>/error1 1' incorporates:
  //   Inport: '<Root>/measure'
  q0_q2 = rtb_Compare_b - rtU.measure.V_Kg[2];

  // SampleTimeMath: '<S39>/TSamp' incorporates:
  //   Inport: '<Root>/measure'
  //
  //  About '<S39>/TSamp':
  //   y = u * K where K = 1 / ( w * Ts )
  x = rtU.measure.V_Kg[2] * 400.0F;

  // Sum: '<S39>/Diff' incorporates:
  //   UnitDelay: '<S39>/UD'
  //
  //  Block description for '<S39>/Diff':
  //
  //   Add in CPU
  //
  //  Block description for '<S39>/UD':
  //
  //   Store in Global RAM
  absxk = x - rtDW.UD_DSTATE_mw;

  // Sum: '<S77>/error1 2'
  t = q1_q3 - absxk;

  // Outport: '<Root>/logs' incorporates:
  //   Switch: '<S43>/Switch'
  rtY.logs[0] = rtb_n_b_d[0];
  rtY.logs[1] = rtb_n_b_d[1];
  rtY.logs[2] = q1_q1;
  rtY.logs[3] = q0_q3;
  rtY.logs[4] = q0_q1;
  rtY.logs[5] = q0_q2;
  rtY.logs[6] = t;
  for (i = 0; i < 6; i++) {
    rtY.logs[i + 7] = rtb_Sum2_e[i];
  }

  rtY.logs[13] = 0.0F;
  rtY.logs[14] = 0.0F;

  // End of Outport: '<Root>/logs'

  // DiscreteIntegrator: '<S24>/Discrete-Time Integrator' incorporates:
  //   Inport: '<Root>/measure'
  if (rtDW.DiscreteTimeIntegrator_IC_LOA_f != 0) {
    rtDW.DiscreteTimeIntegrator_DSTATE_h[0] = rtU.measure.s_Kg[0];
    rtDW.DiscreteTimeIntegrator_DSTATE_h[1] = rtU.measure.s_Kg[1];
  }

  if (rtb_Compare_l && (rtDW.DiscreteTimeIntegrator_PrevRese <= 0)) {
    rtDW.DiscreteTimeIntegrator_DSTATE_h[0] = rtU.measure.s_Kg[0];
    rtDW.DiscreteTimeIntegrator_DSTATE_h[1] = rtU.measure.s_Kg[1];
  }

  // Switch: '<S9>/Switch' incorporates:
  //   DataTypeConversion: '<S91>/Cast To Single'
  //   DiscreteIntegrator: '<S24>/Discrete-Time Integrator'
  //   Gain: '<S101>/Gain2'
  //   SignalConversion: '<S97>/ConcatBufferAtVector ConcatenateIn1'
  if (rtb_Compare_l) {
    rtb_Compare_b = rtDW.DiscreteTimeIntegrator_DSTATE_h[0];
  } else {
    rtb_Compare_b = (real32_T)tmp[0];
  }

  // Sum: '<S37>/Sum2' incorporates:
  //   DiscreteIntegrator: '<S37>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S37>/Discrete-Time Integrator y_dt'
  //   Gain: '<S37>/2*d//omega'
  //   Sum: '<S37>/Sum3'
  rtb_Sum2_e[0] = rtb_Compare_b - (0.198645547F *
    rtDW.DiscreteTimeIntegratory_dt_DS_m[0] +
    rtDW.DiscreteTimeIntegratory_DSTAT_l[0]);

  // Switch: '<S9>/Switch' incorporates:
  //   DataTypeConversion: '<S91>/Cast To Single2'
  //   DiscreteIntegrator: '<S36>/Discrete-Time Integrator'
  //   Gain: '<S101>/Gain2'
  //   SignalConversion: '<S97>/ConcatBufferAtVector ConcatenateIn1'
  if (rtb_Compare_l) {
    rtb_Compare_b = rtDW.DiscreteTimeIntegrator_DSTATE_b[0];
  } else {
    rtb_Compare_b = (real32_T)tmp[1];
  }

  // Sum: '<S37>/Sum2' incorporates:
  //   DiscreteIntegrator: '<S37>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S37>/Discrete-Time Integrator y_dt'
  //   Gain: '<S37>/2*d//omega'
  //   Sum: '<S37>/Sum3'
  rtb_Sum2_e[2] = rtb_Compare_b - (0.198645547F *
    rtDW.DiscreteTimeIntegratory_dt_DS_m[2] +
    rtDW.DiscreteTimeIntegratory_DSTAT_l[2]);
  rtb_Sum2_e[4] = rtb_xy_g_dt2[0] - (0.198645547F *
    rtDW.DiscreteTimeIntegratory_dt_DS_m[4] +
    rtDW.DiscreteTimeIntegratory_DSTAT_l[4]);

  // Switch: '<S9>/Switch' incorporates:
  //   DataTypeConversion: '<S91>/Cast To Single'
  //   DiscreteIntegrator: '<S24>/Discrete-Time Integrator'
  //   Gain: '<S100>/Gain2'
  //   SignalConversion: '<S97>/ConcatBufferAtVector ConcatenateIn2'
  if (rtb_Compare_l) {
    rtb_Compare_b = rtDW.DiscreteTimeIntegrator_DSTATE_h[1];
  } else {
    rtb_Compare_b = (real32_T)tmp_0[0];
  }

  // Sum: '<S37>/Sum2' incorporates:
  //   DiscreteIntegrator: '<S37>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S37>/Discrete-Time Integrator y_dt'
  //   Gain: '<S37>/2*d//omega'
  //   Sum: '<S37>/Sum3'
  rtb_Sum2_e[1] = rtb_Compare_b - (0.198645547F *
    rtDW.DiscreteTimeIntegratory_dt_DS_m[1] +
    rtDW.DiscreteTimeIntegratory_DSTAT_l[1]);

  // Switch: '<S9>/Switch' incorporates:
  //   DataTypeConversion: '<S91>/Cast To Single2'
  //   DiscreteIntegrator: '<S36>/Discrete-Time Integrator'
  //   Gain: '<S100>/Gain2'
  //   SignalConversion: '<S97>/ConcatBufferAtVector ConcatenateIn2'
  if (rtb_Compare_l) {
    rtb_Compare_b = rtDW.DiscreteTimeIntegrator_DSTATE_b[1];
  } else {
    rtb_Compare_b = (real32_T)tmp_0[1];
  }

  // Sum: '<S37>/Sum2' incorporates:
  //   DiscreteIntegrator: '<S37>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S37>/Discrete-Time Integrator y_dt'
  //   Gain: '<S37>/2*d//omega'
  //   Sum: '<S37>/Sum3'
  rtb_Sum2_e[3] = rtb_Compare_b - (0.198645547F *
    rtDW.DiscreteTimeIntegratory_dt_DS_m[3] +
    rtDW.DiscreteTimeIntegratory_DSTAT_l[3]);
  rtb_Sum2_e[5] = rtb_xy_g_dt2[1] - (0.198645547F *
    rtDW.DiscreteTimeIntegratory_dt_DS_m[5] +
    rtDW.DiscreteTimeIntegratory_DSTAT_l[5]);

  // SignalConversion: '<S102>/TmpHiddenBufferAtAssignment1Inport1'
  for (i = 0; i < 5; i++) {
    rtb_Assignment2[i] = 0.0;
    rtb_Assignment1[i] = 0.0;
  }

  // Assignment: '<S102>/Assignment1' incorporates:
  //   MATLAB Function: '<S91>/MATLAB Function6'
  rtb_Assignment2[4] = b_index;

  // Assignment: '<S100>/Assignment1' incorporates:
  //   MATLAB Function: '<S91>/MATLAB Function6'
  rtb_Assignment1[4] = dist_remaining;
  for (aoffset = 0; aoffset < 5; aoffset++) {
    // Gain: '<S100>/Gain1'
    tmp[aoffset] = 0.0;

    // Gain: '<S100>/Gain'
    tmp_0[aoffset] = 0.0;
    for (dim = 0; dim < 5; dim++) {
      // Gain: '<S100>/Gain1' incorporates:
      //   Gain: '<S100>/Gain'
      i = 5 * dim + aoffset;
      tmp[aoffset] += rtConstP.pooled7[i] * rtb_Assignment1[dim];

      // Gain: '<S100>/Gain' incorporates:
      //   DiscreteIntegrator: '<S100>/Discrete-Time Integrator1'
      tmp_0[aoffset] += rtConstP.pooled6[i] *
        rtDW.DiscreteTimeIntegrator1_DSTATE[dim];
    }

    // Sum: '<S100>/Add'
    rtb_Add_kh[aoffset] = tmp[aoffset] + tmp_0[aoffset];
  }

  // SignalConversion: '<S101>/TmpHiddenBufferAtAssignment1Inport1'
  for (i = 0; i < 5; i++) {
    rtb_Assignment1[i] = 0.0;
  }

  // Assignment: '<S101>/Assignment1'
  rtb_Assignment1[4] = rtb_pos_o[0];

  // MATLAB Function: '<S14>/DCM to quaternions'
  DCMtoquaternions(rtb_M_bg, rtb_q_red);

  // MATLAB Function: '<S23>/MATLAB Function2' incorporates:
  //   Constant: '<S23>/Constant1'
  //   Sum: '<S23>/Add'
  rtb_y_i5[2] = q1_q3 - -9.81F;
  if (std::abs(q1_q3 - -9.81F) < 1.0F) {
    if (q1_q3 - -9.81F > 0.0F) {
      rtb_y_i5[2] = 1.0F;
    } else {
      rtb_y_i5[2] = -1.0F;
    }
  }

  // MATLAB Function: '<S23>/MATLAB Function8' incorporates:
  //   MATLAB Function: '<S23>/MATLAB Function2'
  //   SignalConversion: '<S28>/TmpSignal ConversionAt SFunction Inport1'
  MATLABFunction3(rt_atan2f_snf(std::sqrt(rtb_Add1[0] * rtb_Add1[0] + rtb_Add1[1]
    * rtb_Add1[1]), -rtb_y_i5[2]), rt_atan2f_snf(rtb_Add1[1], rtb_Add1[0]),
                  rtb_u);

  // SignalConversion: '<S33>/TmpSignal ConversionAt SFunction Inport1' incorporates:
  //   Constant: '<S23>/Constant1'
  //   MATLAB Function: '<S23>/MATLAB Function9'
  //   Sum: '<S23>/Add3'
  rtb_y_m[0] = rtb_Diff_i[0];
  rtb_y_m[1] = rtb_Diff_i[1];
  rtb_y_m[2] = rtb_Diff_i[2] - -9.81F;

  // MATLAB Function: '<S23>/MATLAB Function9' incorporates:
  //   Constant: '<S23>/Constant1'
  //   SignalConversion: '<S33>/TmpSignal ConversionAt SFunction Inport1'
  //   Sum: '<S23>/Add3'
  rtb_y_i5[0] = rtb_Diff_i[0];
  rtb_y_i5[1] = rtb_Diff_i[1];
  rtb_y_i5[2] = rtb_Diff_i[2] - -9.81F;
  if (std::abs(rtb_Diff_i[2] - -9.81F) < 1.0F) {
    if (rtb_Diff_i[2] - -9.81F > 0.0F) {
      rtb_y_i5[2] = 1.0F;
    } else {
      rtb_y_i5[2] = -1.0F;
    }
  }

  // MATLAB Function: '<S23>/MATLAB Function3' incorporates:
  //   MATLAB Function: '<S23>/MATLAB Function9'
  //   SignalConversion: '<S33>/TmpSignal ConversionAt SFunction Inport1'
  MATLABFunction3(rt_atan2f_snf(std::sqrt(rtb_Diff_i[0] * rtb_Diff_i[0] +
    rtb_Diff_i[1] * rtb_Diff_i[1]), -rtb_y_i5[2]), rt_atan2f_snf(rtb_Diff_i[1],
    rtb_Diff_i[0]), rtb_y_m2);

  // MATLAB Function: '<S23>/MATLAB Function5'
  q1_q1 = ((rtb_y_m2[0] * rtb_y_m2[0] + rtb_y_m2[1] * rtb_y_m2[1]) + rtb_y_m2[2]
           * rtb_y_m2[2]) + rtb_y_m2[3] * rtb_y_m2[3];
  if (rtIsNaNF(q1_q1) || (!(2.22044605E-16F < q1_q1))) {
    q1_q1 = 2.22044605E-16F;
  }

  q_yaw[0] = rtb_y_m2[0] / q1_q1;
  q_yaw[1] = -rtb_y_m2[1] / q1_q1;
  q_yaw[2] = -rtb_y_m2[2] / q1_q1;
  q_yaw[3] = -rtb_y_m2[3] / q1_q1;
  rtb_y_m2[0] = ((q_yaw[0] * rtb_u[0] - q_yaw[1] * rtb_u[1]) - q_yaw[2] * rtb_u
                 [2]) - q_yaw[3] * rtb_u[3];
  rtb_y_m2[1] = (q_yaw[0] * rtb_u[1] + rtb_u[0] * q_yaw[1]) + (q_yaw[2] * rtb_u
    [3] - q_yaw[3] * rtb_u[2]);
  rtb_y_m2[2] = (q_yaw[0] * rtb_u[2] + rtb_u[0] * q_yaw[2]) + (q_yaw[3] * rtb_u
    [1] - q_yaw[1] * rtb_u[3]);
  rtb_y_m2[3] = (q_yaw[0] * rtb_u[3] + rtb_u[0] * q_yaw[3]) + (q_yaw[1] * rtb_u
    [2] - q_yaw[2] * rtb_u[1]);

  // End of MATLAB Function: '<S23>/MATLAB Function5'

  // MATLAB Function: '<S14>/total reduced attitude command (quaternion)'
  q_yaw[0] = ((rtb_q_red[0] * rtb_y_m2[0] - rtb_q_red[1] * rtb_y_m2[1]) -
              rtb_q_red[2] * rtb_y_m2[2]) - rtb_q_red[3] * rtb_y_m2[3];
  q_yaw[1] = (rtb_q_red[0] * rtb_y_m2[1] + rtb_y_m2[0] * rtb_q_red[1]) +
    (rtb_q_red[2] * rtb_y_m2[3] - rtb_q_red[3] * rtb_y_m2[2]);
  q_yaw[2] = (rtb_q_red[0] * rtb_y_m2[2] + rtb_y_m2[0] * rtb_q_red[2]) +
    (rtb_q_red[3] * rtb_y_m2[1] - rtb_q_red[1] * rtb_y_m2[3]);
  q_yaw[3] = (rtb_q_red[0] * rtb_y_m2[3] + rtb_y_m2[0] * rtb_q_red[3]) +
    (rtb_q_red[1] * rtb_y_m2[2] - rtb_q_red[2] * rtb_y_m2[1]);

  // MATLAB Function: '<S58>/MATLAB Function' incorporates:
  //   Inport: '<Root>/measure'
  MATLABFunction(rtU.measure.omega_Kb, rtb_Diff, rtb_M_bg, rtb_y_i, rtb_y_dt,
                 rtb_TmpSignalConversionAtSFunct, rtb_Diff_i, rtb_y_i5, rtb_y_m);

  // Gain: '<S79>/Gain' incorporates:
  //   SignalConversion: '<S79>/TmpSignal ConversionAtGainInport1'
  t = (50.0F * q0_q1 + 28.1578F * q0_q2) + 0.6786F * t;

  // MATLAB Function: '<S47>/dcm2Lean'
  if ((!rtIsNaNF(rtb_M_bg[8])) && (1.0F > rtb_M_bg[8])) {
    q0_q1 = rtb_M_bg[8];
  } else {
    q0_q1 = 1.0F;
  }

  if (!(-1.0F < q0_q1)) {
    q0_q1 = -1.0F;
  }

  // MATLAB Function: '<S47>/MATLAB Function' incorporates:
  //   Gain: '<S79>/Gain'
  //   MATLAB Function: '<S47>/dcm2Lean'
  //   Sum: '<S44>/Add1'
  q0_q1 = std::cos(std::acos(q0_q1));
  if (q0_q1 > 0.1) {
    q1_q3 = ((q1_q3 + t) - absxk) / q0_q1;
  } else {
    q1_q3 = (q1_q3 + t) - absxk;
  }

  // End of MATLAB Function: '<S47>/MATLAB Function'

  // MATLAB Function: '<S49>/MATLAB Function'
  unusedU0[0] = 0.0F;

  // Gain: '<S40>/Gain' incorporates:
  //   DiscreteIntegrator: '<S85>/Discrete-Time Integrator'
  //   MATLAB Function: '<S49>/MATLAB Function1'
  rtb_y_m2[0] = 1.0F - rtDW.DiscreteTimeIntegrator_DSTATE_a[0];

  // Lookup_n-D: '<S40>/1-D Lookup Table2' incorporates:
  //   DiscreteIntegrator: '<S85>/Discrete-Time Integrator'
  //   MATLAB Function: '<S49>/MATLAB Function1'
  rtb_uDLookupTable2[0] = ((0.1F - rtDW.DiscreteTimeIntegrator_DSTATE_a[0]) +
    (1.0F - rtDW.DiscreteTimeIntegrator_DSTATE_a[0])) * 0.5F;

  // MATLAB Function: '<S49>/MATLAB Function1' incorporates:
  //   DiscreteIntegrator: '<S85>/Discrete-Time Integrator'
  rtb_q_red[0] = 0.1F - rtDW.DiscreteTimeIntegrator_DSTATE_a[0];

  // MATLAB Function: '<S49>/MATLAB Function'
  unusedU0[1] = 0.0F;

  // Gain: '<S40>/Gain' incorporates:
  //   DiscreteIntegrator: '<S85>/Discrete-Time Integrator'
  //   MATLAB Function: '<S49>/MATLAB Function1'
  rtb_y_m2[1] = 1.0F - rtDW.DiscreteTimeIntegrator_DSTATE_a[1];

  // Lookup_n-D: '<S40>/1-D Lookup Table2' incorporates:
  //   DiscreteIntegrator: '<S85>/Discrete-Time Integrator'
  //   MATLAB Function: '<S49>/MATLAB Function1'
  rtb_uDLookupTable2[1] = ((0.1F - rtDW.DiscreteTimeIntegrator_DSTATE_a[1]) +
    (1.0F - rtDW.DiscreteTimeIntegrator_DSTATE_a[1])) * 0.5F;

  // MATLAB Function: '<S49>/MATLAB Function1' incorporates:
  //   DiscreteIntegrator: '<S85>/Discrete-Time Integrator'
  rtb_q_red[1] = 0.1F - rtDW.DiscreteTimeIntegrator_DSTATE_a[1];

  // MATLAB Function: '<S49>/MATLAB Function'
  unusedU0[2] = 0.0F;

  // Gain: '<S40>/Gain' incorporates:
  //   DiscreteIntegrator: '<S85>/Discrete-Time Integrator'
  //   MATLAB Function: '<S49>/MATLAB Function1'
  rtb_y_m2[2] = 1.0F - rtDW.DiscreteTimeIntegrator_DSTATE_a[2];

  // Lookup_n-D: '<S40>/1-D Lookup Table2' incorporates:
  //   DiscreteIntegrator: '<S85>/Discrete-Time Integrator'
  //   MATLAB Function: '<S49>/MATLAB Function1'
  rtb_uDLookupTable2[2] = ((0.1F - rtDW.DiscreteTimeIntegrator_DSTATE_a[2]) +
    (1.0F - rtDW.DiscreteTimeIntegrator_DSTATE_a[2])) * 0.5F;

  // MATLAB Function: '<S49>/MATLAB Function1' incorporates:
  //   DiscreteIntegrator: '<S85>/Discrete-Time Integrator'
  rtb_q_red[2] = 0.1F - rtDW.DiscreteTimeIntegrator_DSTATE_a[2];
  absxk = ((0.1F - rtDW.DiscreteTimeIntegrator_DSTATE_a[3]) + (1.0F -
            rtDW.DiscreteTimeIntegrator_DSTATE_a[3])) * 0.5F;

  // MATLAB Function: '<S49>/MATLAB Function'
  unusedU0[3] = 0.0F;

  // Gain: '<S40>/Gain' incorporates:
  //   DiscreteIntegrator: '<S85>/Discrete-Time Integrator'
  //   MATLAB Function: '<S49>/MATLAB Function1'
  rtb_y_m2[3] = 1.0F - rtDW.DiscreteTimeIntegrator_DSTATE_a[3];

  // Lookup_n-D: '<S40>/1-D Lookup Table2'
  rtb_uDLookupTable2[3] = absxk;

  // MATLAB Function: '<S49>/MATLAB Function1' incorporates:
  //   DiscreteIntegrator: '<S85>/Discrete-Time Integrator'
  rtb_q_red[3] = 0.1F - rtDW.DiscreteTimeIntegrator_DSTATE_a[3];

  // MATLAB Function: '<S49>/MATLAB Function' incorporates:
  //   Constant: '<S49>/Constant1'
  //   Constant: '<S49>/Constant5'
  //   Inport: '<Root>/cmd'
  //   Lookup_n-D: '<S40>/1-D Lookup Table1'
  //   MATLAB Function: '<S10>/MATLAB Function'
  //   Sum: '<S49>/Add1'
  q1_q1 = std::sqrt(look1_iflf_binlx(0.5F * rtU.cmd.thr + 0.5F,
    rtConstP.uDLookupTable1_bp01Data, rtConstP.uDLookupTable1_tableData, 3U) +
                    1000.0F);
  for (aoffset = 0; aoffset < 16; aoffset++) {
    A_tmp[aoffset] = q1_q1 * rtConstP.Constant5_Value[aoffset];
  }

  // MATLAB Function: '<S48>/MATLAB Function' incorporates:
  //   Constant: '<S48>/ny_du_red_trim'
  for (aoffset = 0; aoffset < 4; aoffset++) {
    for (dim = 0; dim < 4; dim++) {
      i = aoffset << 2;
      tmp_2[dim + i] = (((d[i + 1] * rtConstP.ny_du_red_trim_Value[dim + 4] +
                          d[i] * rtConstP.ny_du_red_trim_Value[dim]) + d[i + 2] *
                         rtConstP.ny_du_red_trim_Value[dim + 8]) + d[i + 3] *
                        rtConstP.ny_du_red_trim_Value[dim + 12]) + rtb_G2[i +
        dim];
    }
  }

  // MATLAB Function: '<S49>/MATLAB Function' incorporates:
  //   Constant: '<S49>/Constant6'
  for (aoffset = 0; aoffset < 4; aoffset++) {
    for (dim = 0; dim < 4; dim++) {
      i = aoffset << 2;
      j = dim + i;
      d[j] = 0.0F;
      b_aoffset = i + dim;
      d[j] = d[b_aoffset] + tmp_2[i] * A_tmp[dim];
      d[j] = tmp_2[i + 1] * A_tmp[dim + 4] + d[b_aoffset];
      d[j] = tmp_2[i + 2] * A_tmp[dim + 8] + d[b_aoffset];
      d[j] = tmp_2[i + 3] * A_tmp[dim + 12] + d[b_aoffset];
    }
  }

  for (aoffset = 0; aoffset < 4; aoffset++) {
    i = aoffset << 2;
    j = aoffset << 3;
    A[j] = d[i];
    A[4 + j] = rtConstP.Constant6_Value[i];
    dim = i + 1;
    A[1 + j] = d[dim];
    A[5 + j] = rtConstP.Constant6_Value[dim];
    dim = i + 2;
    A[2 + j] = d[dim];
    A[6 + j] = rtConstP.Constant6_Value[dim];
    i += 3;
    A[3 + j] = d[i];
    A[7 + j] = rtConstP.Constant6_Value[i];
  }

  // Gain: '<S55>/Gain'
  for (aoffset = 0; aoffset < 3; aoffset++) {
    rtb_y_i[aoffset] = 0.0F;
    for (dim = 0; dim < 9; dim++) {
      rtb_y_i[aoffset] += rtConstP.Gain_Gain_b[3 * dim + aoffset] * M_bg[dim];
    }
  }

  // End of Gain: '<S55>/Gain'

  // Sum: '<S40>/Add2' incorporates:
  //   Product: '<S40>/MatrixMultiply2'
  //   Sum: '<S43>/Add1'
  //   Switch: '<S43>/Switch'
  //   UnitDelay: '<S40>/Unit Delay1'
  p_free_data[0] = rtb_y_m[0] + rtb_y_i[0];
  p_free_data[1] = rtb_y_m[1] + rtb_y_i[1];
  p_free_data[2] = (scale + rtb_y_i[2]) - rtb_Diff[2];
  p_free_data[3] = q1_q3;
  for (aoffset = 0; aoffset < 4; aoffset++) {
    rtb_y_o[aoffset] = (((rtb_G2[aoffset + 4] * rtDW.UnitDelay1_DSTATE[1] +
                          rtb_G2[aoffset] * rtDW.UnitDelay1_DSTATE[0]) +
                         rtb_G2[aoffset + 8] * rtDW.UnitDelay1_DSTATE[2]) +
                        rtb_G2[aoffset + 12] * rtDW.UnitDelay1_DSTATE[3]) +
      p_free_data[aoffset];

    // MATLAB Function: '<S49>/MATLAB Function1' incorporates:
    //   DiscreteIntegrator: '<S85>/Discrete-Time Integrator'
    //   Product: '<S40>/MatrixMultiply2'
    //   Sum: '<S49>/Add'
    //   UnitDelay: '<S40>/Unit Delay1'
    rtb_u_0[aoffset] = 0.1F - rtDW.DiscreteTimeIntegrator_DSTATE_a[aoffset];
  }

  // End of Sum: '<S40>/Add2'

  // MATLAB Function: '<S49>/MATLAB Function' incorporates:
  //   Constant: '<S49>/Constant6'
  //   MATLAB Function: '<S49>/MATLAB Function1'
  for (aoffset = 0; aoffset < 4; aoffset++) {
    q0_q1 = A_tmp[aoffset + 12] * rtb_y_o[3] + (A_tmp[aoffset + 8] * rtb_y_o[2]
      + (A_tmp[aoffset + 4] * rtb_y_o[1] + A_tmp[aoffset] * rtb_y_o[0]));
    t = rtConstP.Constant6_Value[aoffset + 12] * rtb_u_0[3] +
      (rtConstP.Constant6_Value[aoffset + 8] * rtb_u_0[2] +
       (rtConstP.Constant6_Value[aoffset + 4] * rtb_u_0[1] +
        rtConstP.Constant6_Value[aoffset] * rtb_u_0[0]));
    A_tmp_0[aoffset] = q0_q1;
    A_tmp_0[aoffset + 4] = t;
  }

  for (aoffset = 0; aoffset < 8; aoffset++) {
    q0_q1 = A[aoffset + 24] * absxk + (A[aoffset + 16] * rtb_uDLookupTable2[2] +
      (A[aoffset + 8] * rtb_uDLookupTable2[1] + A[aoffset] * rtb_uDLookupTable2
       [0]));
    d_0[aoffset] = A_tmp_0[aoffset] - q0_q1;
  }

  i_free[0] = true;
  i_free[1] = true;
  i_free[2] = true;
  i_free[3] = true;
  i = 0;
  exitg2 = false;
  while ((!exitg2) && (i <= 99)) {
    b_aoffset = 0;
    if (i_free[0]) {
      b_aoffset = 1;
    }

    if (i_free[1]) {
      b_aoffset++;
    }

    if (i_free[2]) {
      b_aoffset++;
    }

    if (i_free[3]) {
      b_aoffset++;
    }

    g_size_idx_0 = b_aoffset;
    j = 0;
    if (i_free[0]) {
      g_data[0] = 1;
      j = 1;
    }

    if (i_free[1]) {
      g_data[j] = 2;
      j++;
    }

    if (i_free[2]) {
      g_data[j] = 3;
      j++;
    }

    if (i_free[3]) {
      g_data[j] = 4;
    }

    A_free_size[0] = 8;
    A_free_size[1] = b_aoffset;
    for (aoffset = 0; aoffset < b_aoffset; aoffset++) {
      for (dim = 0; dim < 8; dim++) {
        A_free_data[dim + (aoffset << 3)] = A[((g_data[aoffset] - 1) << 3) + dim];
      }
    }

    mldivide(A_free_data, A_free_size, d_0, p_free_data, &p_free_size);
    p[0] = 0.0;
    p[1] = 0.0;
    p[2] = 0.0;
    p[3] = 0.0;
    j = 0;
    if (i_free[0]) {
      p[0] = p_free_data[0];
      j = 1;
    }

    if (i_free[1]) {
      p[1] = p_free_data[j];
      j++;
    }

    if (i_free[2]) {
      p[2] = p_free_data[j];
      j++;
    }

    if (i_free[3]) {
      p[3] = p_free_data[j];
    }

    rtb_u[0] = rtb_uDLookupTable2[0] + (real32_T)p[0];
    rtb_u[1] = rtb_uDLookupTable2[1] + (real32_T)p[1];
    rtb_u[2] = rtb_uDLookupTable2[2] + (real32_T)p[2];
    rtb_u[3] = rtb_uDLookupTable2[3] + (real32_T)p[3];
    j = 0;
    if (i_free[0]) {
      j = 1;
    }

    if (i_free[1]) {
      j++;
    }

    if (i_free[2]) {
      j++;
    }

    if (i_free[3]) {
      j++;
    }

    dim = j;
    j = 0;
    if (i_free[0]) {
      h_data[0] = 1;
      j = 1;
    }

    if (i_free[1]) {
      h_data[j] = 2;
      j++;
    }

    if (i_free[2]) {
      h_data[j] = 3;
      j++;
    }

    if (i_free[3]) {
      h_data[j] = 4;
    }

    rtb_u_1[0] = ((rtb_u[0] < rtb_q_red[0]) || (rtb_u[0] > rtb_y_m2[0]));
    rtb_u_1[1] = ((rtb_u[1] < rtb_q_red[1]) || (rtb_u[1] > rtb_y_m2[1]));
    rtb_u_1[2] = ((rtb_u[2] < rtb_q_red[2]) || (rtb_u[2] > rtb_y_m2[2]));
    rtb_u_1[3] = ((rtb_u[3] < 0.1F - rtDW.DiscreteTimeIntegrator_DSTATE_a[3]) ||
                  (rtb_u[3] > 1.0F - rtDW.DiscreteTimeIntegrator_DSTATE_a[3]));
    for (aoffset = 0; aoffset < dim; aoffset++) {
      rtb_u_data[aoffset] = rtb_u_1[h_data[aoffset] - 1];
    }

    if (!any(rtb_u_data, &dim)) {
      rtb_uDLookupTable2[0] = rtb_u[0];
      rtb_uDLookupTable2[1] = rtb_u[1];
      rtb_uDLookupTable2[2] = rtb_u[2];
      rtb_uDLookupTable2[3] = rtb_u[3];
      if (b_aoffset == 1) {
        for (aoffset = 0; aoffset < 8; aoffset++) {
          A_tmp_0[aoffset] = 0.0F;
          for (dim = 0; dim < b_aoffset; dim++) {
            A_tmp_0[aoffset] += A_free_data[(dim << 3) + aoffset] *
              p_free_data[dim];
          }
        }
      } else if (p_free_size == 1) {
        for (aoffset = 0; aoffset < 8; aoffset++) {
          A_tmp_0[aoffset] = 0.0F;
          for (dim = 0; dim < b_aoffset; dim++) {
            A_tmp_0[aoffset] += A_free_data[(dim << 3) + aoffset] *
              p_free_data[dim];
          }
        }
      } else {
        for (j = 0; j < 8; j++) {
          A_tmp_0[j] = 0.0F;
        }

        for (j = 0; j < g_size_idx_0; j++) {
          b_aoffset = j << 3;
          for (dim = 0; dim < 8; dim++) {
            aoffset = b_aoffset + dim;
            A_tmp_0[dim] += A[((g_data[aoffset / 8] - 1) << 3) + aoffset % 8] *
              p_free_data[j];
          }
        }
      }

      for (aoffset = 0; aoffset < 8; aoffset++) {
        d_0[aoffset] -= A_tmp_0[aoffset];
      }

      for (aoffset = 0; aoffset < 4; aoffset++) {
        p_free_data[aoffset] = 0.0F;
        for (dim = 0; dim < 8; dim++) {
          p_free_data[aoffset] += A[(aoffset << 3) + dim] * d_0[dim];
        }

        absxk = unusedU0[aoffset] * p_free_data[aoffset];
        rtb_u_1[aoffset] = (absxk >= -2.22044605E-16F);
        rtb_u[aoffset] = absxk;
      }

      if (ifWhileCond(rtb_u_1)) {
        exitg2 = true;
      } else {
        if (!rtIsNaNF(rtb_u[0])) {
          j = 0;
        } else {
          j = -1;
          b_aoffset = 2;
          exitg3 = false;
          while ((!exitg3) && (b_aoffset < 5)) {
            if (!rtIsNaNF(rtb_u[b_aoffset - 1])) {
              j = b_aoffset - 1;
              exitg3 = true;
            } else {
              b_aoffset++;
            }
          }
        }

        if (j + 1 == 0) {
          j = 0;
        } else {
          q1_q3 = rtb_u[j];
          for (b_aoffset = j + 1; b_aoffset + 1 < 5; b_aoffset++) {
            if (q1_q3 > rtb_u[b_aoffset]) {
              q1_q3 = rtb_u[b_aoffset];
              j = b_aoffset;
            }
          }
        }

        unusedU0[j] = 0.0F;
        i_free[j] = true;
        i++;
      }
    } else {
      j = 0;
      dist[0] = 1.0;
      f = (p[0] < 0.0);
      e_idx_0 = (p[0] > 0.0);
      if (i_free[0] && f) {
        j = 1;
      }

      f_idx_0 = f;
      dist[1] = 1.0;
      f = (p[1] < 0.0);
      e_idx_1 = (p[1] > 0.0);
      if (i_free[1] && f) {
        j++;
      }

      f_idx_1 = f;
      dist[2] = 1.0;
      f = (p[2] < 0.0);
      e_idx_2 = (p[2] > 0.0);
      if (i_free[2] && f) {
        j++;
      }

      f_idx_2 = f;
      dist[3] = 1.0;
      f = (p[3] < 0.0);
      e_idx_3 = (p[3] > 0.0);
      if (i_free[3] && f) {
        j++;
      }

      dim = j;
      j = 0;
      if (i_free[0] && f_idx_0) {
        i_data[0] = 1;
        j = 1;
      }

      if (i_free[1] && f_idx_1) {
        i_data[j] = 2;
        j++;
      }

      if (i_free[2] && f_idx_2) {
        i_data[j] = 3;
        j++;
      }

      if (i_free[3] && f) {
        i_data[j] = 4;
      }

      for (aoffset = 0; aoffset < dim; aoffset++) {
        j = i_data[aoffset] - 1;
        c_data[aoffset] = (rtb_q_red[j] - rtb_uDLookupTable2[j]) / (real32_T)p[j];
      }

      j = 0;
      if (i_free[0] && f_idx_0) {
        dist[0] = c_data[0];
        j = 1;
      }

      if (i_free[1] && f_idx_1) {
        dist[1] = c_data[j];
        j++;
      }

      if (i_free[2] && f_idx_2) {
        dist[2] = c_data[j];
        j++;
      }

      if (i_free[3] && f) {
        dist[3] = c_data[j];
      }

      j = 0;
      if (i_free[0] && e_idx_0) {
        j = 1;
      }

      if (i_free[1] && e_idx_1) {
        j++;
      }

      if (i_free[2] && e_idx_2) {
        j++;
      }

      if (i_free[3] && e_idx_3) {
        j++;
      }

      dim = j;
      j = 0;
      if (i_free[0] && e_idx_0) {
        j_data[0] = 1;
        j = 1;
      }

      if (i_free[1] && e_idx_1) {
        j_data[j] = 2;
        j++;
      }

      if (i_free[2] && e_idx_2) {
        j_data[j] = 3;
        j++;
      }

      if (i_free[3] && e_idx_3) {
        j_data[j] = 4;
      }

      for (aoffset = 0; aoffset < dim; aoffset++) {
        j = j_data[aoffset] - 1;
        c_data[aoffset] = (rtb_y_m2[j] - rtb_uDLookupTable2[j]) / (real32_T)p[j];
      }

      j = 0;
      if (i_free[0] && e_idx_0) {
        dist[0] = c_data[0];
        j = 1;
      }

      if (i_free[1] && e_idx_1) {
        dist[1] = c_data[j];
        j++;
      }

      if (i_free[2] && e_idx_2) {
        dist[2] = c_data[j];
        j++;
      }

      if (i_free[3] && e_idx_3) {
        dist[3] = c_data[j];
      }

      if (!rtIsNaN(dist[0])) {
        j = 0;
      } else {
        j = -1;
        b_aoffset = 2;
        exitg3 = false;
        while ((!exitg3) && (b_aoffset < 5)) {
          if (!rtIsNaN(dist[b_aoffset - 1])) {
            j = b_aoffset - 1;
            exitg3 = true;
          } else {
            b_aoffset++;
          }
        }
      }

      if (j + 1 == 0) {
        px = dist[0];
        j = 0;
      } else {
        px = dist[j];
        for (dim = j + 1; dim + 1 < 5; dim++) {
          if (px > dist[dim]) {
            px = dist[dim];
            j = dim;
          }
        }
      }

      rtb_uDLookupTable2[0] += (real32_T)(px * p[0]);
      rtb_uDLookupTable2[1] += (real32_T)(px * p[1]);
      rtb_uDLookupTable2[2] += (real32_T)(px * p[2]);
      rtb_uDLookupTable2[3] += (real32_T)(px * p[3]);
      b_aoffset = (A_free_size[1] << 3) - 1;
      for (aoffset = 0; aoffset <= b_aoffset; aoffset++) {
        A_free_data[aoffset] *= (real32_T)px;
      }

      if (A_free_size[1] == 1) {
        for (aoffset = 0; aoffset < 8; aoffset++) {
          A_tmp_0[aoffset] = 0.0F;
          for (dim = 0; dim < 1; dim++) {
            A_tmp_0[aoffset] += A_free_data[aoffset] * p_free_data[0];
          }
        }
      } else if (p_free_size == 1) {
        b_aoffset = A_free_size[1];
        for (aoffset = 0; aoffset < 8; aoffset++) {
          A_tmp_0[aoffset] = 0.0F;
          for (dim = 0; dim < b_aoffset; dim++) {
            A_tmp_0[aoffset] += A_free_data[(dim << 3) + aoffset] *
              p_free_data[dim];
          }
        }
      } else {
        for (b_aoffset = 0; b_aoffset < 8; b_aoffset++) {
          A_tmp_0[b_aoffset] = 0.0F;
        }

        for (b_aoffset = 0; b_aoffset < A_free_size[1]; b_aoffset++) {
          aoffset = b_aoffset << 3;
          for (dim = 0; dim < 8; dim++) {
            A_tmp_0[dim] += A_free_data[aoffset + dim] * p_free_data[b_aoffset];
          }
        }
      }

      for (aoffset = 0; aoffset < 8; aoffset++) {
        d_0[aoffset] -= A_tmp_0[aoffset];
      }

      if (p[j] < 0.0) {
        unusedU0[j] = -1.0F;
      } else if (p[j] > 0.0) {
        unusedU0[j] = 1.0F;
      } else if (p[j] == 0.0) {
        unusedU0[j] = 0.0F;
      } else {
        unusedU0[j] = (rtNaNF);
      }

      i_free[j] = false;
      i++;
    }
  }

  // Sum: '<S10>/Add6' incorporates:
  //   DiscreteIntegrator: '<S46>/Discrete-Time Integrator y'
  rtb_Compare_b = rtb_uDLookupTable2[0] + rtDW.DiscreteTimeIntegratory_DSTAT_c[0];

  // Saturate: '<S10>/Saturation3'
  if (rtb_Compare_b > 1.0F) {
    rtb_Compare_b = 1.0F;
  } else {
    if (rtb_Compare_b < 0.1F) {
      rtb_Compare_b = 0.1F;
    }
  }

  // Sum: '<S46>/Sum2' incorporates:
  //   Delay: '<S10>/Delay'
  //   DiscreteIntegrator: '<S46>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S46>/Discrete-Time Integrator y_dt'
  //   Gain: '<S46>/2*d//omega'
  //   Sum: '<S46>/Sum3'
  rtb_q_red[0] = rtDW.Delay_DSTATE[0] - (0.0039788736F *
    rtDW.DiscreteTimeIntegratory_dt_DS_b[0] +
    rtDW.DiscreteTimeIntegratory_DSTAT_c[0]);

  // Sum: '<S10>/Add6' incorporates:
  //   DiscreteIntegrator: '<S46>/Discrete-Time Integrator y'
  u0_1 = rtb_uDLookupTable2[1] + rtDW.DiscreteTimeIntegratory_DSTAT_c[1];

  // Saturate: '<S10>/Saturation3'
  if (u0_1 > 1.0F) {
    u0_1 = 1.0F;
  } else {
    if (u0_1 < 0.1F) {
      u0_1 = 0.1F;
    }
  }

  // Sum: '<S46>/Sum2' incorporates:
  //   Delay: '<S10>/Delay'
  //   DiscreteIntegrator: '<S46>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S46>/Discrete-Time Integrator y_dt'
  //   Gain: '<S46>/2*d//omega'
  //   Sum: '<S46>/Sum3'
  rtb_q_red[1] = rtDW.Delay_DSTATE[1] - (0.0039788736F *
    rtDW.DiscreteTimeIntegratory_dt_DS_b[1] +
    rtDW.DiscreteTimeIntegratory_DSTAT_c[1]);

  // Sum: '<S10>/Add6' incorporates:
  //   DiscreteIntegrator: '<S46>/Discrete-Time Integrator y'
  u0_0 = rtb_uDLookupTable2[2] + rtDW.DiscreteTimeIntegratory_DSTAT_c[2];

  // Saturate: '<S10>/Saturation3'
  if (u0_0 > 1.0F) {
    u0_0 = 1.0F;
  } else {
    if (u0_0 < 0.1F) {
      u0_0 = 0.1F;
    }
  }

  // Sum: '<S46>/Sum2' incorporates:
  //   Delay: '<S10>/Delay'
  //   DiscreteIntegrator: '<S46>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S46>/Discrete-Time Integrator y_dt'
  //   Gain: '<S46>/2*d//omega'
  //   Sum: '<S46>/Sum3'
  rtb_q_red[2] = rtDW.Delay_DSTATE[2] - (0.0039788736F *
    rtDW.DiscreteTimeIntegratory_dt_DS_b[2] +
    rtDW.DiscreteTimeIntegratory_DSTAT_c[2]);

  // Sum: '<S10>/Add6' incorporates:
  //   DiscreteIntegrator: '<S46>/Discrete-Time Integrator y'
  u0 = rtb_uDLookupTable2[3] + rtDW.DiscreteTimeIntegratory_DSTAT_c[3];

  // Saturate: '<S10>/Saturation3'
  if (u0 > 1.0F) {
    u0 = 1.0F;
  } else {
    if (u0 < 0.1F) {
      u0 = 0.1F;
    }
  }

  // Sum: '<S46>/Sum2' incorporates:
  //   Delay: '<S10>/Delay'
  //   DiscreteIntegrator: '<S46>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S46>/Discrete-Time Integrator y_dt'
  //   Gain: '<S46>/2*d//omega'
  //   Sum: '<S46>/Sum3'
  rtb_q_red[3] = rtDW.Delay_DSTATE[3] - (0.0039788736F *
    rtDW.DiscreteTimeIntegratory_dt_DS_b[3] +
    rtDW.DiscreteTimeIntegratory_DSTAT_c[3]);

  // Outport: '<Root>/u' incorporates:
  //   Gain: '<Root>/Gain1'
  //   Gain: '<Root>/Gain2'
  //   Gain: '<Root>/Gain3'
  //   Gain: '<Root>/Gain4'
  //   Saturate: '<S10>/Saturation3'
  rtY.u[0] = u0_1;
  rtY.u[1] = u0;
  rtY.u[2] = rtb_Compare_b;
  rtY.u[3] = u0_0;
  rtY.u[4] = 0.0F * u0;
  rtY.u[5] = 0.0F * u0;
  rtY.u[6] = 0.0F * u0;
  rtY.u[7] = 0.0F * u0;

  // MATLAB Function: '<S19>/quaternion to  reduced attitude unit vector'
  scale = 1.29246971E-26F;
  absxk = std::abs(q_yaw[0]);
  if (absxk > 1.29246971E-26F) {
    q1_q1 = 1.0F;
    scale = absxk;
  } else {
    t = absxk / 1.29246971E-26F;
    q1_q1 = t * t;
  }

  absxk = std::abs(q_yaw[1]);
  if (absxk > scale) {
    t = scale / absxk;
    q1_q1 = q1_q1 * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    q1_q1 += t * t;
  }

  absxk = std::abs(q_yaw[2]);
  if (absxk > scale) {
    t = scale / absxk;
    q1_q1 = q1_q1 * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    q1_q1 += t * t;
  }

  absxk = std::abs(q_yaw[3]);
  if (absxk > scale) {
    t = scale / absxk;
    q1_q1 = q1_q1 * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    q1_q1 += t * t;
  }

  q1_q1 = scale * std::sqrt(q1_q1);
  if (rtIsNaNF(q1_q1) || (!(2.22044605E-16F < q1_q1))) {
    q1_q1 = 2.22044605E-16F;
  }

  q_yaw[0] /= q1_q1;
  q_yaw[1] /= q1_q1;
  q_yaw[2] /= q1_q1;
  q2_q3 = q_yaw[3] / q1_q1;
  scale = q_yaw[0] * q_yaw[0];
  q1_q1 = q_yaw[1] * q_yaw[1];
  absxk = q_yaw[2] * q_yaw[2];
  t = q2_q3 * q2_q3;
  q0_q1 = q_yaw[0] * q_yaw[1];
  q0_q2 = q_yaw[0] * q_yaw[2];
  q0_q3 = q_yaw[0] * q2_q3;
  q1_q2 = q_yaw[1] * q_yaw[2];
  q1_q3 = q_yaw[1] * q2_q3;
  q2_q3 *= q_yaw[2];
  rtb_M_bg[0] = ((scale + q1_q1) - absxk) - t;
  rtb_M_bg[1] = (q1_q2 + q0_q3) * 2.0F;
  rtb_M_bg[2] = (q1_q3 - q0_q2) * 2.0F;
  rtb_M_bg[3] = (q1_q2 - q0_q3) * 2.0F;
  scale -= q1_q1;
  rtb_M_bg[4] = (scale + absxk) - t;
  rtb_M_bg[5] = (q2_q3 + q0_q1) * 2.0F;
  rtb_M_bg[6] = (q1_q3 + q0_q2) * 2.0F;
  rtb_M_bg[7] = (q2_q3 - q0_q1) * 2.0F;
  rtb_M_bg[8] = (scale - absxk) + t;
  for (i = 0; i < 3; i++) {
    // Sum: '<S76>/Sum2' incorporates:
    //   DiscreteIntegrator: '<S76>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S76>/Discrete-Time Integrator y_dt'
    //   Gain: '<S76>/2*d//omega'
    //   Sum: '<S76>/Sum3'
    rtb_y_i[i] = rtb_TmpSignalConversionAtSFunct[i] - (0.0319788754F *
      rtDW.DiscreteTimeIntegratory_dt_DS_o[i] +
      rtDW.DiscreteTimeIntegratory_DSTA_ll[i]);

    // Sum: '<S75>/Sum2' incorporates:
    //   DiscreteIntegrator: '<S73>/Discrete-Time Integrator y_dt'
    //   DiscreteIntegrator: '<S75>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S75>/Discrete-Time Integrator y_dt'
    //   Gain: '<S75>/2*d//omega'
    //   Sum: '<S75>/Sum3'
    rtb_y_dt[i] = rtDW.DiscreteTimeIntegratory_dt_DSTA[i] - (0.0319788754F *
      rtDW.DiscreteTimeIntegratory_dt_DS_d[i] +
      rtDW.DiscreteTimeIntegratory_DSTAT_f[i]);

    // Sum: '<S74>/Sum2' incorporates:
    //   DiscreteIntegrator: '<S73>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S74>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S74>/Discrete-Time Integrator y_dt'
    //   Gain: '<S74>/2*d//omega'
    //   Sum: '<S74>/Sum3'
    rtb_Diff_i[i] = rtDW.DiscreteTimeIntegratory_DSTATE[i] - (0.0319788754F *
      rtDW.DiscreteTimeIntegratory_dt_DS_i[i] +
      rtDW.DiscreteTimeIntegratory_DSTAT_n[i]);
    rtb_Diff[i] = -rtb_M_bg[i + 6] + (rtb_M_bg[i + 3] * 0.0F + rtb_M_bg[i] *
      0.0F);
  }

  // End of MATLAB Function: '<S19>/quaternion to  reduced attitude unit vector'

  // MATLAB Function: '<S91>/MATLAB Function2'
  dist_remaining = 1.7000000000000002 / rtb_Switch.arc_length;
  if (dist_remaining < 0.0) {
    dist_remaining = std::ceil(dist_remaining);
  } else {
    dist_remaining = std::floor(dist_remaining);
  }

  dist_remaining = std::abs(1.7000000000000002 - dist_remaining *
    rtb_Switch.arc_length);
  b_aoffset = 0;
  exitg2 = false;
  while ((!exitg2) && (b_aoffset <= i_0)) {
    b_index = total_distance + (real_T)b_aoffset;
    if (b_index > rtb_Switch.num_sections_set) {
      b_index -= rtb_Switch.num_sections_set;
    }

    trajSectionGetArcLength(rtb_Switch.sections[(int32_T)b_index - 1].pos_x,
      rtb_Switch.sections[(int32_T)b_index - 1].pos_y, rtb_Switch.sections
      [(int32_T)b_index - 1].pos_z, total_arc_length, &absxk_0,
      &arc_len_start_diff);
    dist_available = rtb_Switch.sections[(int32_T)b_index - 1].arc_length -
      absxk_0;
    if (dist_remaining <= dist_available) {
      px = total_arc_length - ((absxk_0 - dist_remaining) - absxk_0) /
        arc_len_start_diff;
      trajSectionGetArcLength(rtb_Switch.sections[(int32_T)b_index - 1].pos_x,
        rtb_Switch.sections[(int32_T)b_index - 1].pos_y, rtb_Switch.sections
        [(int32_T)b_index - 1].pos_z, px, &b, &arc_len_start_diff);
      px -= ((b - dist_remaining) - absxk_0) / arc_len_start_diff;
      trajSectionGetArcLength(rtb_Switch.sections[(int32_T)b_index - 1].pos_x,
        rtb_Switch.sections[(int32_T)b_index - 1].pos_y, rtb_Switch.sections
        [(int32_T)b_index - 1].pos_z, px, &b, &arc_len_start_diff);
      exitg2 = true;
    } else {
      dist_remaining -= dist_available;
      total_arc_length = 0.0;
      b_aoffset++;
    }
  }

  // End of MATLAB Function: '<S91>/MATLAB Function2'

  // MATLAB Function: '<S19>/MATLAB Function4'
  MATLABFunction4(rtb_Diff, &q1_q3, &scale);

  // Update for DiscreteIntegrator: '<S85>/Discrete-Time Integrator' incorporates:
  //   Gain: '<S85>/1//T'
  //   Saturate: '<S10>/Saturation3'
  //   Sum: '<S85>/Sum2'
  rtDW.DiscreteTimeIntegrator_DSTATE_a[0] += (rtb_Compare_b -
    rtDW.DiscreteTimeIntegrator_DSTATE_a[0]) * 35.7142868F * 0.0025F;
  rtDW.DiscreteTimeIntegrator_DSTATE_a[1] += (u0_1 -
    rtDW.DiscreteTimeIntegrator_DSTATE_a[1]) * 35.7142868F * 0.0025F;
  rtDW.DiscreteTimeIntegrator_DSTATE_a[2] += (u0_0 -
    rtDW.DiscreteTimeIntegrator_DSTATE_a[2]) * 35.7142868F * 0.0025F;
  rtDW.DiscreteTimeIntegrator_DSTATE_a[3] += (u0 -
    rtDW.DiscreteTimeIntegrator_DSTATE_a[3]) * 35.7142868F * 0.0025F;

  // Update for UnitDelay: '<S103>/Delay Input1'
  //
  //  Block description for '<S103>/Delay Input1':
  //
  //   Store in Global RAM
  rtDW.DelayInput1_DSTATE = rtb_Compare;

  // Update for Delay: '<S98>/Delay'
  rtDW.Delay_DSTATE_f = rtb_FixPtRelationalOperator;

  // Update for DiscreteIntegrator: '<S74>/Discrete-Time Integrator y'
  rtDW.DiscreteTimeIntegratory_IC_LOAD = 0U;

  // Update for DiscreteIntegrator: '<S75>/Discrete-Time Integrator y'
  rtDW.DiscreteTimeIntegratory_IC_LO_g = 0U;

  // Update for UnitDelay: '<S38>/UD'
  //
  //  Block description for '<S38>/UD':
  //
  //   Store in Global RAM
  rtDW.UD_DSTATE[0] = rtb_TSamp_idx_0;

  // Update for DiscreteIntegrator: '<S73>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S73>/Discrete-Time Integrator y_dt'
  rtDW.DiscreteTimeIntegratory_DSTATE[0] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DSTA[0];

  // Update for DiscreteIntegrator: '<S74>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S74>/Discrete-Time Integrator y_dt'
  rtDW.DiscreteTimeIntegratory_DSTAT_n[0] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_i[0];

  // Update for DiscreteIntegrator: '<S73>/Discrete-Time Integrator y_dt'
  rtDW.DiscreteTimeIntegratory_dt_DSTA[0] += 0.0025F *
    rtb_TmpSignalConversionAtSFunct[0];

  // Update for DiscreteIntegrator: '<S75>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S75>/Discrete-Time Integrator y_dt'
  rtDW.DiscreteTimeIntegratory_DSTAT_f[0] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_d[0];

  // Update for UnitDelay: '<S38>/UD'
  //
  //  Block description for '<S38>/UD':
  //
  //   Store in Global RAM
  rtDW.UD_DSTATE[1] = rtb_TSamp_idx_1;

  // Update for DiscreteIntegrator: '<S73>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S73>/Discrete-Time Integrator y_dt'
  rtDW.DiscreteTimeIntegratory_DSTATE[1] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DSTA[1];

  // Update for DiscreteIntegrator: '<S74>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S74>/Discrete-Time Integrator y_dt'
  rtDW.DiscreteTimeIntegratory_DSTAT_n[1] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_i[1];

  // Update for DiscreteIntegrator: '<S73>/Discrete-Time Integrator y_dt'
  rtDW.DiscreteTimeIntegratory_dt_DSTA[1] += 0.0025F *
    rtb_TmpSignalConversionAtSFunct[1];

  // Update for DiscreteIntegrator: '<S75>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S75>/Discrete-Time Integrator y_dt'
  rtDW.DiscreteTimeIntegratory_DSTAT_f[1] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_d[1];

  // Update for UnitDelay: '<S38>/UD'
  //
  //  Block description for '<S38>/UD':
  //
  //   Store in Global RAM
  rtDW.UD_DSTATE[2] = rtb_M_bg_tmp;

  // Update for DiscreteIntegrator: '<S73>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S73>/Discrete-Time Integrator y_dt'
  rtDW.DiscreteTimeIntegratory_DSTATE[2] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DSTA[2];

  // Update for DiscreteIntegrator: '<S74>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S74>/Discrete-Time Integrator y_dt'
  rtDW.DiscreteTimeIntegratory_DSTAT_n[2] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_i[2];

  // Update for DiscreteIntegrator: '<S73>/Discrete-Time Integrator y_dt'
  rtDW.DiscreteTimeIntegratory_dt_DSTA[2] += 0.0025F *
    rtb_TmpSignalConversionAtSFunct[2];

  // Update for DiscreteIntegrator: '<S75>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S75>/Discrete-Time Integrator y_dt'
  rtDW.DiscreteTimeIntegratory_DSTAT_f[2] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_d[2];

  // Update for DiscreteIntegrator: '<S101>/Discrete-Time Integrator'
  rtDW.DiscreteTimeIntegrator_IC_LOADI = 0U;
  for (i_0 = 0; i_0 < 5; i_0++) {
    // Gain: '<S101>/Gain1'
    tmp[i_0] = 0.0;

    // Gain: '<S101>/Gain'
    tmp_0[i_0] = 0.0;
    for (aoffset = 0; aoffset < 5; aoffset++) {
      // Gain: '<S101>/Gain1' incorporates:
      //   Gain: '<S101>/Gain'
      dim = 5 * aoffset + i_0;
      tmp[i_0] += rtConstP.pooled7[dim] * rtb_Assignment1[aoffset];

      // Gain: '<S101>/Gain' incorporates:
      //   DiscreteIntegrator: '<S101>/Discrete-Time Integrator'
      tmp_0[i_0] += rtConstP.pooled6[dim] *
        rtDW.DiscreteTimeIntegrator_DSTATE[aoffset];
    }
  }

  // Update for DiscreteIntegrator: '<S100>/Discrete-Time Integrator1'
  rtDW.DiscreteTimeIntegrator1_IC_LOAD = 0U;

  // Update for DiscreteIntegrator: '<S102>/Discrete-Time Integrator'
  rtDW.DiscreteTimeIntegrator_IC_LOA_d = 0U;
  for (i_0 = 0; i_0 < 5; i_0++) {
    // Update for DiscreteIntegrator: '<S101>/Discrete-Time Integrator' incorporates:
    //   Sum: '<S101>/Add'
    rtDW.DiscreteTimeIntegrator_DSTATE[i_0] += (tmp[i_0] + tmp_0[i_0]) * 0.0025;

    // Update for DiscreteIntegrator: '<S100>/Discrete-Time Integrator1'
    rtDW.DiscreteTimeIntegrator1_DSTATE[i_0] += 0.0025 * rtb_Add_kh[i_0];

    // Gain: '<S102>/Gain1'
    rtb_Assignment1[i_0] = 0.0;

    // Gain: '<S102>/Gain'
    tmp_3[i_0] = 0.0;
    for (aoffset = 0; aoffset < 5; aoffset++) {
      // Gain: '<S102>/Gain1' incorporates:
      //   Gain: '<S102>/Gain'
      dim = 5 * aoffset + i_0;
      rtb_Assignment1[i_0] += rtConstP.pooled7[dim] * rtb_Assignment2[aoffset];

      // Gain: '<S102>/Gain' incorporates:
      //   DiscreteIntegrator: '<S102>/Discrete-Time Integrator'
      tmp_3[i_0] += rtConstP.pooled6[dim] *
        rtDW.DiscreteTimeIntegrator_DSTATE_o[aoffset];
    }
  }

  // Update for DiscreteIntegrator: '<S102>/Discrete-Time Integrator' incorporates:
  //   Sum: '<S102>/Add'
  for (i_0 = 0; i_0 < 5; i_0++) {
    rtDW.DiscreteTimeIntegrator_DSTATE_o[i_0] += (rtb_Assignment1[i_0] +
      tmp_3[i_0]) * 0.0025;
  }

  // Update for DiscreteIntegrator: '<S36>/Discrete-Time Integrator'
  rtDW.DiscreteTimeIntegrator_DSTATE_b[0] += 0.0025F * rtb_y_dt_b_idx_0;
  rtDW.DiscreteTimeIntegrator_DSTATE_b[1] += 0.0025F * rtb_y_dt_b_idx_1;

  // Update for UnitDelay: '<S8>/UD'
  //
  //  Block description for '<S8>/UD':
  //
  //   Store in Global RAM
  rtDW.UD_DSTATE_c[0] = cmd_V_NED_idx_0;
  rtDW.UD_DSTATE_c[1] = cmd_V_NED_idx_1;
  rtDW.UD_DSTATE_c[2] = cmd_V_NED;

  // Update for DiscreteIntegrator: '<S37>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S37>/Discrete-Time Integrator y_dt'
  for (i_0 = 0; i_0 < 6; i_0++) {
    rtDW.DiscreteTimeIntegratory_DSTAT_l[i_0] += 0.0025F *
      rtDW.DiscreteTimeIntegratory_dt_DS_m[i_0];
  }

  // End of Update for DiscreteIntegrator: '<S37>/Discrete-Time Integrator y'

  // Update for DiscreteIntegrator: '<S84>/Discrete-Time Integrator'
  rtDW.DiscreteTimeIntegrator_DSTAT_ad += 0.0025F * cos_Psi;
  if (rtDW.DiscreteTimeIntegrator_DSTAT_ad >= 25.0F) {
    rtDW.DiscreteTimeIntegrator_DSTAT_ad = 25.0F;
  } else {
    if (rtDW.DiscreteTimeIntegrator_DSTAT_ad <= -4.0F) {
      rtDW.DiscreteTimeIntegrator_DSTAT_ad = -4.0F;
    }
  }

  // End of Update for DiscreteIntegrator: '<S84>/Discrete-Time Integrator'

  // Update for DiscreteIntegrator: '<S76>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S76>/Discrete-Time Integrator y_dt'
  rtDW.DiscreteTimeIntegratory_IC_LO_n = 0U;
  rtDW.DiscreteTimeIntegratory_DSTA_ll[0] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_o[0];
  rtDW.DiscreteTimeIntegratory_DSTA_ll[1] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_o[1];
  rtDW.DiscreteTimeIntegratory_DSTA_ll[2] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_o[2];

  // Update for DiscreteIntegrator: '<S58>/Discrete-Time Integrator2' incorporates:
  //   DiscreteIntegrator: '<S72>/Discrete-Time Integrator'
  rtDW.DiscreteTimeIntegrator2_IC_LOAD = 0U;
  rtDW.DiscreteTimeIntegrator2_DSTATE += 0.0025F *
    rtDW.DiscreteTimeIntegrator_DSTATE_g;

  // Update for DiscreteIntegrator: '<S72>/Discrete-Time Integrator'
  rtDW.DiscreteTimeIntegrator_DSTATE_g += 0.0025F * rtb_phi_n;

  // Update for UnitDelay: '<S89>/UD'
  //
  //  Block description for '<S89>/UD':
  //
  //   Store in Global RAM
  rtDW.UD_DSTATE_h = rtb_delta_k;

  // Update for UnitDelay: '<S90>/UD'
  //
  //  Block description for '<S90>/UD':
  //
  //   Store in Global RAM
  rtDW.UD_DSTATE_m = rtb_Gain_j;

  // Update for DiscreteIntegrator: '<S82>/Discrete-Time Integrator3'
  rtDW.DiscreteTimeIntegrator3_IC_LOAD = 0U;
  rtDW.DiscreteTimeIntegrator3_DSTATE += 0.0025F * sin_Psi;
  rtDW.DiscreteTimeIntegrator3_PrevRes = (int8_T)rtb_Compare_l;

  // Update for UnitDelay: '<S39>/UD'
  //
  //  Block description for '<S39>/UD':
  //
  //   Store in Global RAM
  rtDW.UD_DSTATE_mw = x;

  // Update for DiscreteIntegrator: '<S37>/Discrete-Time Integrator y_dt' incorporates:
  //   Gain: '<S37>/omega^2'
  for (i_0 = 0; i_0 < 6; i_0++) {
    rtDW.DiscreteTimeIntegratory_dt_DS_m[i_0] += 101.368347F * rtb_Sum2_e[i_0] *
      0.0025F;
  }

  // End of Update for DiscreteIntegrator: '<S37>/Discrete-Time Integrator y_dt'

  // Update for DiscreteIntegrator: '<S24>/Discrete-Time Integrator'
  rtDW.DiscreteTimeIntegrator_IC_LOA_f = 0U;
  rtDW.DiscreteTimeIntegrator_DSTATE_h[0] += 0.0025F * rtb_y_d_idx_0;
  rtDW.DiscreteTimeIntegrator_DSTATE_h[1] += 0.0025F * rtb_y_d_idx_1;
  rtDW.DiscreteTimeIntegrator_PrevRese = (int8_T)rtb_Compare_l;

  // Update for UnitDelay: '<S40>/Unit Delay1'
  rtDW.UnitDelay1_DSTATE[0] = rtb_uDLookupTable2[0];

  // Update for DiscreteIntegrator: '<S46>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S46>/Discrete-Time Integrator y_dt'
  rtDW.DiscreteTimeIntegratory_DSTAT_c[0] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_b[0];

  // Update for DiscreteIntegrator: '<S46>/Discrete-Time Integrator y_dt' incorporates:
  //   Gain: '<S46>/omega^2'
  rtDW.DiscreteTimeIntegratory_dt_DS_b[0] += 252661.875F * rtb_q_red[0] *
    0.0025F;

  // Update for Delay: '<S10>/Delay'
  rtDW.Delay_DSTATE[0] = rtb_y_idx_0;

  // Update for UnitDelay: '<S40>/Unit Delay1'
  rtDW.UnitDelay1_DSTATE[1] = rtb_uDLookupTable2[1];

  // Update for DiscreteIntegrator: '<S46>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S46>/Discrete-Time Integrator y_dt'
  rtDW.DiscreteTimeIntegratory_DSTAT_c[1] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_b[1];

  // Update for DiscreteIntegrator: '<S46>/Discrete-Time Integrator y_dt' incorporates:
  //   Gain: '<S46>/omega^2'
  rtDW.DiscreteTimeIntegratory_dt_DS_b[1] += 252661.875F * rtb_q_red[1] *
    0.0025F;

  // Update for Delay: '<S10>/Delay'
  rtDW.Delay_DSTATE[1] = rtb_y_idx_1;

  // Update for UnitDelay: '<S40>/Unit Delay1'
  rtDW.UnitDelay1_DSTATE[2] = rtb_uDLookupTable2[2];

  // Update for DiscreteIntegrator: '<S46>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S46>/Discrete-Time Integrator y_dt'
  rtDW.DiscreteTimeIntegratory_DSTAT_c[2] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_b[2];

  // Update for DiscreteIntegrator: '<S46>/Discrete-Time Integrator y_dt' incorporates:
  //   Gain: '<S46>/omega^2'
  rtDW.DiscreteTimeIntegratory_dt_DS_b[2] += 252661.875F * rtb_q_red[2] *
    0.0025F;

  // Update for Delay: '<S10>/Delay'
  rtDW.Delay_DSTATE[2] = rtb_y_idx_2;

  // Update for UnitDelay: '<S40>/Unit Delay1'
  rtDW.UnitDelay1_DSTATE[3] = rtb_uDLookupTable2[3];

  // Update for DiscreteIntegrator: '<S46>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S46>/Discrete-Time Integrator y_dt'
  rtDW.DiscreteTimeIntegratory_DSTAT_c[3] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_b[3];

  // Update for DiscreteIntegrator: '<S46>/Discrete-Time Integrator y_dt' incorporates:
  //   Gain: '<S46>/omega^2'
  rtDW.DiscreteTimeIntegratory_dt_DS_b[3] += 252661.875F * rtb_q_red[3] *
    0.0025F;

  // Update for Delay: '<S10>/Delay'
  rtDW.Delay_DSTATE[3] = rtb_y_idx_3;

  // Update for DiscreteIntegrator: '<S76>/Discrete-Time Integrator y_dt' incorporates:
  //   Gain: '<S76>/omega^2'
  rtDW.DiscreteTimeIntegratory_dt_DS_o[0] += 3911.41284F * rtb_y_i[0] * 0.0025F;

  // Update for DiscreteIntegrator: '<S75>/Discrete-Time Integrator y_dt' incorporates:
  //   Gain: '<S75>/omega^2'
  rtDW.DiscreteTimeIntegratory_dt_DS_d[0] += 3911.41284F * rtb_y_dt[0] * 0.0025F;

  // Update for DiscreteIntegrator: '<S74>/Discrete-Time Integrator y_dt' incorporates:
  //   Gain: '<S74>/omega^2'
  rtDW.DiscreteTimeIntegratory_dt_DS_i[0] += 3911.41284F * rtb_Diff_i[0] *
    0.0025F;

  // Update for DiscreteIntegrator: '<S76>/Discrete-Time Integrator y_dt' incorporates:
  //   Gain: '<S76>/omega^2'
  rtDW.DiscreteTimeIntegratory_dt_DS_o[1] += 3911.41284F * rtb_y_i[1] * 0.0025F;

  // Update for DiscreteIntegrator: '<S75>/Discrete-Time Integrator y_dt' incorporates:
  //   Gain: '<S75>/omega^2'
  rtDW.DiscreteTimeIntegratory_dt_DS_d[1] += 3911.41284F * rtb_y_dt[1] * 0.0025F;

  // Update for DiscreteIntegrator: '<S74>/Discrete-Time Integrator y_dt' incorporates:
  //   Gain: '<S74>/omega^2'
  rtDW.DiscreteTimeIntegratory_dt_DS_i[1] += 3911.41284F * rtb_Diff_i[1] *
    0.0025F;

  // Update for DiscreteIntegrator: '<S76>/Discrete-Time Integrator y_dt' incorporates:
  //   Gain: '<S76>/omega^2'
  rtDW.DiscreteTimeIntegratory_dt_DS_o[2] += 3911.41284F * rtb_y_i[2] * 0.0025F;

  // Update for DiscreteIntegrator: '<S75>/Discrete-Time Integrator y_dt' incorporates:
  //   Gain: '<S75>/omega^2'
  rtDW.DiscreteTimeIntegratory_dt_DS_d[2] += 3911.41284F * rtb_y_dt[2] * 0.0025F;

  // Update for DiscreteIntegrator: '<S74>/Discrete-Time Integrator y_dt' incorporates:
  //   Gain: '<S74>/omega^2'
  rtDW.DiscreteTimeIntegratory_dt_DS_i[2] += 3911.41284F * rtb_Diff_i[2] *
    0.0025F;
}

// Model initialize function
void MatlabControllerClass::initialize()
{
  // Registration code

  // initialize non-finites
  rt_InitInfAndNaN(sizeof(real_T));

  // InitializeConditions for DiscreteIntegrator: '<S73>/Discrete-Time Integrator y'
  rtDW.DiscreteTimeIntegratory_DSTATE[0] = 0.0F;
  rtDW.DiscreteTimeIntegratory_DSTATE[1] = 0.0F;
  rtDW.DiscreteTimeIntegratory_DSTATE[2] = -1.0F;

  // InitializeConditions for DiscreteIntegrator: '<S74>/Discrete-Time Integrator y'
  rtDW.DiscreteTimeIntegratory_IC_LOAD = 1U;

  // InitializeConditions for DiscreteIntegrator: '<S75>/Discrete-Time Integrator y'
  rtDW.DiscreteTimeIntegratory_IC_LO_g = 1U;

  // InitializeConditions for DiscreteIntegrator: '<S101>/Discrete-Time Integrator'
  rtDW.DiscreteTimeIntegrator_IC_LOADI = 1U;

  // InitializeConditions for DiscreteIntegrator: '<S100>/Discrete-Time Integrator1'
  rtDW.DiscreteTimeIntegrator1_IC_LOAD = 1U;

  // InitializeConditions for DiscreteIntegrator: '<S102>/Discrete-Time Integrator'
  rtDW.DiscreteTimeIntegrator_IC_LOA_d = 1U;

  // InitializeConditions for DiscreteIntegrator: '<S76>/Discrete-Time Integrator y'
  rtDW.DiscreteTimeIntegratory_IC_LO_n = 1U;

  // InitializeConditions for DiscreteIntegrator: '<S58>/Discrete-Time Integrator2'
  rtDW.DiscreteTimeIntegrator2_IC_LOAD = 1U;

  // InitializeConditions for DiscreteIntegrator: '<S82>/Discrete-Time Integrator3'
  rtDW.DiscreteTimeIntegrator3_IC_LOAD = 1U;
  rtDW.DiscreteTimeIntegrator3_PrevRes = 2;

  // InitializeConditions for DiscreteIntegrator: '<S24>/Discrete-Time Integrator'
  rtDW.DiscreteTimeIntegrator_IC_LOA_f = 1U;
  rtDW.DiscreteTimeIntegrator_PrevRese = 2;
}

// Constructor
MatlabControllerClass::MatlabControllerClass()
{
  // Currently there is no constructor body generated.
}

// Destructor
MatlabControllerClass::~MatlabControllerClass()
{
  // Currently there is no destructor body generated.
}

//
// File trailer for generated code.
//
// [EOF]
//
