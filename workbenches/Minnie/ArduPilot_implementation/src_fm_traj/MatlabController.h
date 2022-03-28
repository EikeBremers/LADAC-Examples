//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: MatlabController.h
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
#ifndef RTW_HEADER_MatlabController_h_
#define RTW_HEADER_MatlabController_h_
#include "rtwtypes.h"
#include <stddef.h>
#include <cmath>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#ifndef ArduCopter_MinnieTrajController_COMMON_INCLUDES_
# define ArduCopter_MinnieTrajController_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 // ArduCopter_MinnieTrajController_COMMON_INCLUDES_ 

// Macros for accessing real-time model data structure
#ifndef DEFINED_TYPEDEF_FOR_cmdBus_
#define DEFINED_TYPEDEF_FOR_cmdBus_

typedef struct {
  real32_T roll;
  real32_T pitch;
  real32_T yaw;
  real32_T thr;
  real32_T s_Kg_init[3];
  real32_T yaw_init;
  uint16_T mission_change;
  real32_T waypoints[40];
  uint16_T num_waypoints;
  real32_T RC_pwm[16];
} cmdBus;

#endif

#ifndef DEFINED_TYPEDEF_FOR_measureBus_
#define DEFINED_TYPEDEF_FOR_measureBus_

typedef struct {
  real32_T omega_Kb[3];
  real32_T EulerAngles[3];
  real32_T q_bg[4];
  real32_T a_Kg[3];
  real32_T V_Kg[3];
  real32_T s_Kg[3];
  real32_T lla[3];
  real32_T rangefinder[6];
} measureBus;

#endif

#ifndef DEFINED_TYPEDEF_FOR_sections_
#define DEFINED_TYPEDEF_FOR_sections_

typedef struct {
  real_T pos_x[6];
  real_T pos_y[6];
  real_T pos_z[6];
  real_T vel[6];
  real_T t;
  real_T arc_length;
  real_T distance;
  real_T polynomial_degree;
} section;

#endif

#ifndef DEFINED_TYPEDEF_FOR_trajectoryBus_
#define DEFINED_TYPEDEF_FOR_trajectoryBus_

typedef struct {
  real_T num_sections_max;
  real_T num_sections_set;
  section sections[6];
  real_T active_section;
  real_T current_time;
  real_T arc_length;
  real_T distance;
  boolean_T is_repeated_course;
  real_T polynomial_degree;
} trajectoryBus;

#endif

// Custom Type definition for MATLAB Function: '<S14>/DCM to quaternions'
#ifndef struct_tag_skA4KFEZ4HPkJJBOYCrevdH
#define struct_tag_skA4KFEZ4HPkJJBOYCrevdH

struct tag_skA4KFEZ4HPkJJBOYCrevdH
{
  uint32_T SafeEq;
  uint32_T Absolute;
  uint32_T NaNBias;
  uint32_T NaNWithFinite;
  uint32_T FiniteWithNaN;
  uint32_T NaNWithNaN;
};

#endif                                 //struct_tag_skA4KFEZ4HPkJJBOYCrevdH

#ifndef typedef_skA4KFEZ4HPkJJBOYCrevdH
#define typedef_skA4KFEZ4HPkJJBOYCrevdH

typedef struct tag_skA4KFEZ4HPkJJBOYCrevdH skA4KFEZ4HPkJJBOYCrevdH;

#endif                                 //typedef_skA4KFEZ4HPkJJBOYCrevdH

#ifndef struct_tag_sJCxfmxS8gBOONUZjbjUd9E
#define struct_tag_sJCxfmxS8gBOONUZjbjUd9E

struct tag_sJCxfmxS8gBOONUZjbjUd9E
{
  boolean_T CaseSensitivity;
  boolean_T StructExpand;
  char_T PartialMatching[6];
  boolean_T IgnoreNulls;
};

#endif                                 //struct_tag_sJCxfmxS8gBOONUZjbjUd9E

#ifndef typedef_sJCxfmxS8gBOONUZjbjUd9E
#define typedef_sJCxfmxS8gBOONUZjbjUd9E

typedef struct tag_sJCxfmxS8gBOONUZjbjUd9E sJCxfmxS8gBOONUZjbjUd9E;

#endif                                 //typedef_sJCxfmxS8gBOONUZjbjUd9E

// Custom Type definition for MATLAB Function: '<S104>/trajFromWaypoints'
#ifndef struct_tag_swcXJGizFBRyMp3hXuXt1z
#define struct_tag_swcXJGizFBRyMp3hXuXt1z

struct tag_swcXJGizFBRyMp3hXuXt1z
{
  uint32_T abstol;
  uint32_T reltol;
  uint32_T waypoints;
  uint32_T maxintervalcount;
};

#endif                                 //struct_tag_swcXJGizFBRyMp3hXuXt1z

#ifndef typedef_swcXJGizFBRyMp3hXuXt1z
#define typedef_swcXJGizFBRyMp3hXuXt1z

typedef struct tag_swcXJGizFBRyMp3hXuXt1z swcXJGizFBRyMp3hXuXt1z;

#endif                                 //typedef_swcXJGizFBRyMp3hXuXt1z

#ifndef struct_tag_s9s8BC13iTohZXRbLMSIDHE
#define struct_tag_s9s8BC13iTohZXRbLMSIDHE

struct tag_s9s8BC13iTohZXRbLMSIDHE
{
  boolean_T CaseSensitivity;
  boolean_T StructExpand;
  boolean_T PartialMatching;
};

#endif                                 //struct_tag_s9s8BC13iTohZXRbLMSIDHE

#ifndef typedef_s9s8BC13iTohZXRbLMSIDHE
#define typedef_s9s8BC13iTohZXRbLMSIDHE

typedef struct tag_s9s8BC13iTohZXRbLMSIDHE s9s8BC13iTohZXRbLMSIDHE;

#endif                                 //typedef_s9s8BC13iTohZXRbLMSIDHE

#ifndef struct_emxArray_real_T
#define struct_emxArray_real_T

struct emxArray_real_T
{
  real_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};

#endif                                 //struct_emxArray_real_T

#ifndef typedef_emxArray_real_T
#define typedef_emxArray_real_T

typedef struct emxArray_real_T emxArray_real_T;

#endif                                 //typedef_emxArray_real_T

#ifndef struct_emxArray_real_T_1x5
#define struct_emxArray_real_T_1x5

struct emxArray_real_T_1x5
{
  real_T data[5];
  int32_T size[2];
};

#endif                                 //struct_emxArray_real_T_1x5

#ifndef typedef_emxArray_real_T_1x5
#define typedef_emxArray_real_T_1x5

typedef struct emxArray_real_T_1x5 emxArray_real_T_1x5;

#endif                                 //typedef_emxArray_real_T_1x5

#ifndef struct_sIxk2k5yWjI6JGhnnTa6DfG_tag
#define struct_sIxk2k5yWjI6JGhnnTa6DfG_tag

struct sIxk2k5yWjI6JGhnnTa6DfG_tag
{
  emxArray_real_T_1x5 f1;
};

#endif                                 //struct_sIxk2k5yWjI6JGhnnTa6DfG_tag

#ifndef typedef_b_cell_wrap_1
#define typedef_b_cell_wrap_1

typedef struct sIxk2k5yWjI6JGhnnTa6DfG_tag b_cell_wrap_1;

#endif                                 //typedef_b_cell_wrap_1

#ifndef struct_emxArray_int32_T
#define struct_emxArray_int32_T

struct emxArray_int32_T
{
  int32_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};

#endif                                 //struct_emxArray_int32_T

#ifndef typedef_emxArray_int32_T
#define typedef_emxArray_int32_T

typedef struct emxArray_int32_T emxArray_int32_T;

#endif                                 //typedef_emxArray_int32_T

// Block signals and states (default storage) for system '<Root>'
typedef struct {
  trajectoryBus traj;                  // '<S104>/trajFromWaypoints'
  creal_T a_data[289];
  creal_T T_data[289];
  creal_T A_data[289];
  creal_T At_data[289];
  creal_T b_A_data[289];
  creal_T b_h_data[289];
  creal_T b_A_data_m[289];
  real_T DiscreteTimeIntegrator_DSTATE[5];// '<S101>/Discrete-Time Integrator'
  real_T DiscreteTimeIntegrator1_DSTATE[5];// '<S100>/Discrete-Time Integrator1' 
  real_T DiscreteTimeIntegrator_DSTATE_o[5];// '<S102>/Discrete-Time Integrator' 
  real_T interval[650];
  real_T subs[1298];
  real_T qsub[649];
  real_T errsub[649];
  real_T x_data[9735];
  real_T fx_data[9735];
  real_T x_data_c[9735];
  real_T xt_data[9735];
  real_T tmp_data[9735];
  real_T tmp_data_k[9735];
  real_T tmp_data_c[9735];
  real_T tmp_data_b[9735];
  real_T z1_data[9735];
  real_T interval_p[650];
  real_T subs_c[1298];
  real_T qsub_f[649];
  real_T errsub_g[649];
  real_T x_data_g[9735];
  real_T fx_data_m[9735];
  real_T x_data_n[9735];
  real_T xt_data_p[9735];
  real_T tmp_data_l[9735];
  real_T tmp_data_j[9735];
  real_T tmp_data_d[9735];
  real_T tmp_data_g[9735];
  real_T z1_data_l[9735];
  real_T interval_d[650];
  real_T subs_d[1298];
  real_T qsub_l[649];
  real_T errsub_o[649];
  real_T x_data_b[9735];
  real_T fx_data_n[9735];
  real_T x_data_bs[9735];
  real_T xt_data_l[9735];
  real_T tmp_data_h[9735];
  real_T tmp_data_bn[9735];
  real_T tmp_data_da[9735];
  real_T tmp_data_e[9735];
  real_T z1_data_b[9735];
  real32_T DiscreteTimeIntegrator_DSTATE_a[4];// '<S85>/Discrete-Time Integrator' 
  real32_T UD_DSTATE[3];               // '<S38>/UD'
  real32_T DiscreteTimeIntegratory_DSTATE[3];// '<S73>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegratory_DSTAT_n[3];// '<S74>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegratory_dt_DSTA[3];// '<S73>/Discrete-Time Integrator y_dt' 
  real32_T DiscreteTimeIntegratory_DSTAT_f[3];// '<S75>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegrator_DSTATE_b[2];// '<S36>/Discrete-Time Integrator' 
  real32_T UD_DSTATE_c[3];             // '<S8>/UD'
  real32_T DiscreteTimeIntegratory_DSTAT_l[6];// '<S37>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegratory_DSTA_ll[3];// '<S76>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegratory_dt_DS_m[6];// '<S37>/Discrete-Time Integrator y_dt' 
  real32_T DiscreteTimeIntegrator_DSTATE_h[2];// '<S24>/Discrete-Time Integrator' 
  real32_T UnitDelay1_DSTATE[4];       // '<S40>/Unit Delay1'
  real32_T DiscreteTimeIntegratory_DSTAT_c[4];// '<S46>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegratory_dt_DS_b[4];// '<S46>/Discrete-Time Integrator y_dt' 
  real32_T Delay_DSTATE[4];            // '<S10>/Delay'
  real32_T DiscreteTimeIntegratory_dt_DS_o[3];// '<S76>/Discrete-Time Integrator y_dt' 
  real32_T DiscreteTimeIntegratory_dt_DS_d[3];// '<S75>/Discrete-Time Integrator y_dt' 
  real32_T DiscreteTimeIntegratory_dt_DS_i[3];// '<S74>/Discrete-Time Integrator y_dt' 
  real32_T DiscreteTimeIntegrator_DSTAT_ad;// '<S84>/Discrete-Time Integrator'
  real32_T DiscreteTimeIntegrator2_DSTATE;// '<S58>/Discrete-Time Integrator2'
  real32_T DiscreteTimeIntegrator_DSTATE_g;// '<S72>/Discrete-Time Integrator'
  real32_T UD_DSTATE_h;                // '<S89>/UD'
  real32_T UD_DSTATE_m;                // '<S90>/UD'
  real32_T DiscreteTimeIntegrator3_DSTATE;// '<S82>/Discrete-Time Integrator3'
  real32_T UD_DSTATE_mw;               // '<S39>/UD'
  int8_T DiscreteTimeIntegrator3_PrevRes;// '<S82>/Discrete-Time Integrator3'
  int8_T DiscreteTimeIntegrator_PrevRese;// '<S24>/Discrete-Time Integrator'
  uint8_T DiscreteTimeIntegratory_IC_LOAD;// '<S74>/Discrete-Time Integrator y'
  uint8_T DiscreteTimeIntegratory_IC_LO_g;// '<S75>/Discrete-Time Integrator y'
  uint8_T DiscreteTimeIntegrator_IC_LOADI;// '<S101>/Discrete-Time Integrator'
  uint8_T DiscreteTimeIntegrator1_IC_LOAD;// '<S100>/Discrete-Time Integrator1'
  uint8_T DiscreteTimeIntegrator_IC_LOA_d;// '<S102>/Discrete-Time Integrator'
  uint8_T DiscreteTimeIntegratory_IC_LO_n;// '<S76>/Discrete-Time Integrator y'
  uint8_T DiscreteTimeIntegrator2_IC_LOAD;// '<S58>/Discrete-Time Integrator2'
  uint8_T DiscreteTimeIntegrator3_IC_LOAD;// '<S82>/Discrete-Time Integrator3'
  uint8_T DiscreteTimeIntegrator_IC_LOA_f;// '<S24>/Discrete-Time Integrator'
  boolean_T DelayInput1_DSTATE;        // '<S103>/Delay Input1'
  boolean_T Delay_DSTATE_f;            // '<S98>/Delay'
} DW;

// Invariant block signals (default storage)
typedef const struct tag_ConstB {
  real32_T Gain2[3];                   // '<S23>/Gain2'
} ConstB;

// Constant parameters (default storage)
typedef struct {
  // Expression: traj_empty
  //  Referenced by: '<S91>/Constant15'

  trajectoryBus Constant15_Value;

  // Pooled Parameter (Expression: ss_model.C)
  //  Referenced by:
  //    '<S100>/Gain2'
  //    '<S101>/Gain2'
  //    '<S102>/Gain2'

  real_T pooled5[25];

  // Pooled Parameter (Expression: ss_model.A)
  //  Referenced by:
  //    '<S100>/Gain'
  //    '<S101>/Gain'
  //    '<S102>/Gain'

  real_T pooled6[25];

  // Pooled Parameter (Expression: ss_model.B)
  //  Referenced by:
  //    '<S100>/Gain1'
  //    '<S101>/Gain1'
  //    '<S102>/Gain1'

  real_T pooled7[25];

  // Computed Parameter: Pad_outDims
  //  Referenced by: '<S11>/Pad'

  int32_T Pad_outDims[2];

  // Computed Parameter: Pad_padAfter
  //  Referenced by: '<S11>/Pad'

  int32_T Pad_padAfter[2];

  // Computed Parameter: ny_du_red_trim_Value
  //  Referenced by: '<S48>/ny_du_red_trim'

  real32_T ny_du_red_trim_Value[16];

  // Computed Parameter: ny_du_dt_Value
  //  Referenced by: '<S48>/ny_du_dt'

  real32_T ny_du_dt_Value[16];

  // Computed Parameter: Constant5_Value
  //  Referenced by: '<S49>/Constant5'

  real32_T Constant5_Value[16];

  // Computed Parameter: Constant6_Value
  //  Referenced by: '<S49>/Constant6'

  real32_T Constant6_Value[16];

  // Expression: [horiz_pos_cntrl.limits.e_xy_max;1000;1000;1000;1000]
  //  Referenced by: '<S26>/Saturation1'

  real32_T Saturation1_UpperSat[6];

  // Expression: -[horiz_pos_cntrl.limits.e_xy_max;1000;1000;1000;1000]
  //  Referenced by: '<S26>/Saturation1'

  real32_T Saturation1_LowerSat[6];

  // Expression: horiz_pos_cntrl.K
  //  Referenced by: '<S26>/Gain'

  real32_T Gain_Gain_h[12];

  // Computed Parameter: uDLookupTable_tableData
  //  Referenced by: '<S82>/1-D Lookup Table'

  real32_T uDLookupTable_tableData[3];

  // Computed Parameter: uDLookupTable_bp01Data
  //  Referenced by: '<S82>/1-D Lookup Table'

  real32_T uDLookupTable_bp01Data[3];

  // Computed Parameter: uDLookupTable1_tableData
  //  Referenced by: '<S40>/1-D Lookup Table1'

  real32_T uDLookupTable1_tableData[4];

  // Computed Parameter: uDLookupTable1_bp01Data
  //  Referenced by: '<S40>/1-D Lookup Table1'

  real32_T uDLookupTable1_bp01Data[4];

  // Expression: atti_cntrl.K
  //  Referenced by: '<S55>/Gain'

  real32_T Gain_Gain_b[27];
} ConstP;

// External inputs (root inport signals with default storage)
typedef struct {
  cmdBus cmd;                          // '<Root>/cmd'
  measureBus measure;                  // '<Root>/measure'
} ExtU;

// External outputs (root outports fed by signals with default storage)
typedef struct {
  real32_T u[8];                       // '<Root>/u'
  real32_T logs[15];                   // '<Root>/logs'
} ExtY;

extern const ConstB rtConstB;          // constant block i/o

// Constant parameters (default storage)
extern const ConstP rtConstP;

// Class declaration for model ArduCopter_MinnieTrajController
class MatlabControllerClass {
  // public data and function members
 public:
  // External inputs
  ExtU rtU;

  // External outputs
  ExtY rtY;

  // model initialize function
  void initialize();

  // model step function
  void step();

  // Constructor
  MatlabControllerClass();

  // Destructor
  ~MatlabControllerClass();

  // private data and function members
 private:
  // Block signals and states
  DW rtDW;

  // private member function(s) for subsystem '<Root>'
  void emxInit_real_T(emxArray_real_T **pEmxArray, int32_T numDimensions);
  void emxEnsureCapacity_real_T(emxArray_real_T *emxArray, int32_T oldNumel);
  void emxFree_real_T(emxArray_real_T **pEmxArray);
  void emxInit_int32_T(emxArray_int32_T **pEmxArray, int32_T numDimensions);
  void emxEnsureCapacity_int32_T(emxArray_int32_T *emxArray, int32_T oldNumel);
  real_T xnrm2_i(int32_T n, const emxArray_real_T *x, int32_T ix0);
  void xscal(int32_T n, real_T a, emxArray_real_T *x, int32_T ix0);
  void xgeqp3(emxArray_real_T *A, emxArray_real_T *tau, emxArray_int32_T *jpvt);
  void emxFree_int32_T(emxArray_int32_T **pEmxArray);
  void lusolve(const emxArray_real_T *A, emxArray_real_T *B_9);
  void mldivide_i(const emxArray_real_T *A, const emxArray_real_T *B_8,
                  emxArray_real_T *Y);
  void polyder_c(const emxArray_real_T *u, emxArray_real_T *a);
  void polyInterpolation(const real32_T points[6], real_T degree, boolean_T
    cycle, emxArray_real_T *coeffs, real_T *num_of_splines);
  boolean_T anyNonFinite(const creal_T x_data[], const int32_T x_size[2]);
  boolean_T ishermitian(const creal_T A_data[], const int32_T A_size[2]);
  real_T xzlangeM(const creal_T x_data[], const int32_T x_size[2]);
  boolean_T isfinite(real_T x);
  void xzlascl(real_T cfrom, real_T cto, creal_T A_data[], int32_T A_size[2]);
  void xzggbal(creal_T A_data[], int32_T A_size[2], int32_T *ilo, int32_T *ihi,
               int32_T rscale_data[], int32_T *rscale_size);
  void xzlartg(const creal_T f, const creal_T g, real_T *cs, creal_T *sn,
               creal_T *r);
  void xzgghrd(int32_T ilo, int32_T ihi, creal_T A_data[], int32_T A_size[2]);
  real_T xzlanhs(const creal_T A_data[], const int32_T A_size[2], int32_T ilo,
                 int32_T ihi);
  int32_T mod(int32_T x);
  void sqrt_j(creal_T *x);
  void xzlartg_n(const creal_T f, const creal_T g, real_T *cs, creal_T *sn);
  void xzhgeqz(const creal_T A_data[], const int32_T A_size[2], int32_T ilo,
               int32_T ihi, int32_T *info, creal_T alpha1_data[], int32_T
               *alpha1_size, creal_T beta1_data[], int32_T *beta1_size);
  void xzlascl_n(real_T cfrom, real_T cto, creal_T A_data[], int32_T *A_size);
  void xzgeev(const creal_T A_data[], const int32_T A_size[2], int32_T *info,
              creal_T alpha1_data[], int32_T *alpha1_size, creal_T beta1_data[],
              int32_T *beta1_size);
  real_T xnrm2_g(int32_T n, const creal_T x_data[], int32_T ix0);
  real_T xdlapy3(real_T x1, real_T x2, real_T x3);
  creal_T recip(const creal_T y);
  creal_T xzlarfg(int32_T n, creal_T *alpha1, creal_T x_data[], int32_T ix0);
  int32_T ilazlr(int32_T m, int32_T n, const creal_T A_data[], int32_T ia0,
                 int32_T lda);
  void xgemv(int32_T m, int32_T n, const creal_T A_data[], int32_T ia0, int32_T
             lda, const creal_T x_data[], int32_T ix0, creal_T y_data[]);
  void xgerc(int32_T m, int32_T n, const creal_T alpha1, const creal_T x_data[],
             int32_T iy0, creal_T A_data[], int32_T ia0, int32_T lda);
  void xzlarf_f(int32_T m, int32_T n, int32_T iv0, const creal_T tau, creal_T
                C_data[], int32_T ic0, int32_T ldc, creal_T work_data[]);
  int32_T ilazlc(int32_T m, int32_T n, const creal_T A_data[], int32_T ia0,
                 int32_T lda);
  void xgemv_c(int32_T m, int32_T n, const creal_T A_data[], int32_T ia0,
               int32_T lda, const creal_T x_data[], int32_T ix0, creal_T y_data[]);
  void xgerc_c(int32_T m, int32_T n, const creal_T alpha1, int32_T ix0, const
               creal_T y_data[], creal_T A_data[], int32_T ia0, int32_T lda);
  void xzlarf_fy(int32_T m, int32_T n, int32_T iv0, const creal_T tau, creal_T
                 C_data[], int32_T ic0, int32_T ldc, creal_T work_data[]);
  void xgehrd(creal_T a_data[], int32_T a_size[2]);
  void xscal_jo(int32_T n, const creal_T a, creal_T x_data[], int32_T ix0,
                int32_T incx);
  void xscal_j(int32_T n, const creal_T a, creal_T x_data[], int32_T ix0);
  creal_T xzlarfg_e(creal_T *alpha1, creal_T *x);
  int32_T eml_zlahqr(creal_T h_data[], int32_T h_size[2]);
  int32_T xhseqr(creal_T h_data[], int32_T h_size[2]);
  void triu(creal_T x_data[], int32_T x_size[2]);
  void schur(creal_T A_data[], int32_T A_size[2], creal_T V_data[], int32_T
             V_size[2]);
  void mainDiagZeroImag(const creal_T D_data[], const int32_T D_size[2], creal_T
                        d_data[], int32_T *d_size);
  void eig(const creal_T A_data[], const int32_T A_size[2], creal_T V_data[],
           int32_T *V_size);
  void roots(const real_T c[18], creal_T r_data[], int32_T *r_size);
  void polyval_k(const real_T p[6], const real_T x_data[], const int32_T x_size
                 [2], real_T y_data[], int32_T y_size[2]);
  void trajGetMatch(const trajectoryBus *traj, const real32_T position[3],
                    real_T *section_idx, real_T *error, real_T *t);
  void polyder_a(const real_T u[6], real_T a_data[], int32_T a_size[2]);
  real_T polyval_i(const real_T p_data[], const int32_T p_size[2], real_T x);
  void polyder_ge(const real_T u_data[], const int32_T u_size[2], real_T a_data[],
                  int32_T a_size[2]);
  real_T norm_o(const real_T x[3]);
  void trajSectionGetFrenetSerretWithG(const section *traj_section, real_T vel,
    real_T g, real_T T[3], real_T B_a[3], real_T N[3], real_T *kappa, real_T
    *tau);
  void split(real_T x[650], int32_T nx, int32_T *nxnew, real_T *pathlen);
  boolean_T mesh_has_collapsed(const real_T x_data[], const int32_T x_size[2]);
  void polyval_ik(const real_T p_data[], const int32_T p_size[2], const real_T
                  x_data[], const int32_T x_size[2], real_T y_data[], int32_T
                  y_size[2]);
  void power(const real_T a_data[], const int32_T a_size[2], real_T y_data[],
             int32_T y_size[2]);
  void sqrt_p(real_T x_data[], int32_T x_size[2]);
  void __anon_fcn(const real_T dx_data[], const int32_T dx_size[2], const real_T
                  dy_data[], const int32_T dy_size[2], const real_T dz_data[],
                  const int32_T dz_size[2], const real_T ts_data[], const
                  int32_T ts_size[2], real_T varargout_1_data[], int32_T
                  varargout_1_size[2]);
  void transfun(int8_T problem_type, const b_cell_wrap_1 f_tunableEnvironment[3],
                const real_T t_data[], const int32_T t_size[2], real_T A, real_T
                B_5, real_T yZERO, boolean_T firstcall, real_T y_data[], int32_T
                y_size[2], boolean_T *tooclose);
  real_T midpArea(const b_cell_wrap_1 f_tunableEnvironment[3], real_T a, real_T
                  b);
  real_T quadgk(const b_cell_wrap_1 fun_tunableEnvironment[3], real_T b);
  void trajSectionGetArcLength(const real_T traj_section_pos_x[6], const real_T
    traj_section_pos_y[6], const real_T traj_section_pos_z[6], real_T varargin_1,
    real_T *arc_length, real_T *arc_length_dt);
  void quatNormalize(const real32_T q[4], real32_T q_out[4]);
  void LSQFromQR(const real32_T A_data[], const int32_T A_size[2], const
                 real32_T tau_data[], const int32_T jpvt_data[], real32_T B_3[8],
                 int32_T rankA, real32_T Y_data[], int32_T *Y_size);
  real32_T xnrm2(int32_T n, const real32_T x_data[], int32_T ix0);
  void xzlarf(int32_T m, int32_T n, int32_T iv0, real32_T tau, real32_T C_data[],
              int32_T ic0, real32_T work_data[]);
  void qrsolve(const real32_T A_data[], const int32_T A_size[2], const real32_T
               B_1[8], real32_T Y_data[], int32_T *Y_size);
  void mldivide(const real32_T A_data[], const int32_T A_size[2], const real32_T
                B_0[8], real32_T Y_data[], int32_T *Y_size);
  boolean_T any(const boolean_T x_data[], const int32_T *x_size);
  boolean_T ifWhileCond(const boolean_T x[4]);
  void trajSectionGetPos(const real_T traj_section_pos_x[6], const real_T
    traj_section_pos_y[6], const real_T traj_section_pos_z[6], real_T varargin_1,
    real_T pos[3]);
  void split_i(real_T x[650], int32_T nx, int32_T *nxnew, real_T *pathlen);
  void transfun_b(const b_cell_wrap_1 f_tunableEnvironment[3], const real_T
                  t_data[], const int32_T t_size[2], boolean_T firstcall, real_T
                  y_data[], int32_T y_size[2], boolean_T *tooclose);
  real_T midpArea_o(const b_cell_wrap_1 f_tunableEnvironment[3]);
  real_T quadgk_g(const b_cell_wrap_1 fun_tunableEnvironment[3]);
};

//-
//  These blocks were eliminated from the model due to optimizations:
//
//  Block '<Root>/Constant' : Unused code path elimination
//  Block '<S3>/Constant' : Unused code path elimination
//  Block '<S14>/Scope' : Unused code path elimination
//  Block '<S8>/Data Type Duplicate' : Unused code path elimination
//  Block '<S9>/Add4' : Unused code path elimination
//  Block '<S38>/Data Type Duplicate' : Unused code path elimination
//  Block '<S39>/Data Type Duplicate' : Unused code path elimination
//  Block '<S58>/Gain1' : Unused code path elimination
//  Block '<S58>/Gain2' : Unused code path elimination
//  Block '<S58>/Gain4' : Unused code path elimination
//  Block '<S58>/Gain5' : Unused code path elimination
//  Block '<S58>/Switch' : Unused code path elimination
//  Block '<S89>/Data Type Duplicate' : Unused code path elimination
//  Block '<S90>/Data Type Duplicate' : Unused code path elimination
//  Block '<S11>/Scope' : Unused code path elimination
//  Block '<S91>/Cast To Single4' : Unused code path elimination
//  Block '<S91>/Scope' : Unused code path elimination
//  Block '<S91>/Sum' : Unused code path elimination
//  Block '<S1>/Data Type Conversion' : Eliminate redundant data type conversion
//  Block '<S23>/Gain' : Eliminated nontunable gain of 1
//  Block '<S23>/Gain1' : Eliminated nontunable gain of 1
//  Block '<S24>/Gain' : Eliminated nontunable gain of 1
//  Block '<S24>/Gain1' : Eliminated nontunable gain of 1
//  Block '<S37>/Saturation' : Eliminated Saturate block
//  Block '<S49>/Reshape' : Reshape block reduction
//  Block '<S49>/Reshape1' : Reshape block reduction
//  Block '<S49>/Reshape2' : Reshape block reduction
//  Block '<S49>/Reshape4' : Reshape block reduction
//  Block '<S49>/Reshape5' : Reshape block reduction
//  Block '<S43>/Reshape' : Reshape block reduction
//  Block '<S43>/Reshape1' : Reshape block reduction
//  Block '<S43>/Reshape2' : Reshape block reduction
//  Block '<S53>/Data Type Conversion' : Eliminate redundant data type conversion
//  Block '<S58>/Data Type Conversion' : Eliminate redundant data type conversion
//  Block '<S58>/Data Type Conversion1' : Eliminate redundant data type conversion
//  Block '<S58>/Data Type Conversion2' : Eliminate redundant data type conversion
//  Block '<S58>/Data Type Conversion3' : Eliminate redundant data type conversion
//  Block '<S58>/Gain' : Eliminated nontunable gain of 1
//  Block '<S58>/Gain3' : Eliminated nontunable gain of 1
//  Block '<S72>/Saturation' : Eliminated Saturate block
//  Block '<S73>/Saturation' : Eliminated Saturate block
//  Block '<S58>/Reshape' : Reshape block reduction
//  Block '<S74>/Saturation' : Eliminated Saturate block
//  Block '<S75>/Saturation' : Eliminated Saturate block
//  Block '<S76>/Saturation' : Eliminated Saturate block
//  Block '<S82>/Gain1' : Eliminated nontunable gain of 1
//  Block '<S82>/Gain2' : Eliminated nontunable gain of 1
//  Block '<S84>/Saturation' : Eliminated Saturate block
//  Block '<S85>/Saturation' : Eliminated Saturate block
//  Block '<S46>/Saturation' : Eliminated Saturate block
//  Block '<S100>/Gain3' : Eliminated nontunable gain of 1
//  Block '<S101>/Gain3' : Eliminated nontunable gain of 1
//  Block '<S102>/Gain3' : Eliminated nontunable gain of 1


//-
//  The generated code includes comments that allow you to trace directly
//  back to the appropriate location in the model.  The basic format
//  is <system>/block_name, where system is the system number (uniquely
//  assigned by Simulink) and block_name is the name of the block.
//
//  Use the MATLAB hilite_system command to trace the generated code back
//  to the model.  For example,
//
//  hilite_system('<S3>')    - opens system 3
//  hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'ArduCopter_MinnieTrajController'
//  '<S1>'   : 'ArduCopter_MinnieTrajController/Actuator muxer'
//  '<S2>'   : 'ArduCopter_MinnieTrajController/Compare To Constant'
//  '<S3>'   : 'ArduCopter_MinnieTrajController/Multicopter INDI Loiter Flight Mode'
//  '<S4>'   : 'ArduCopter_MinnieTrajController/Quaternions to Rotation Matrix'
//  '<S5>'   : 'ArduCopter_MinnieTrajController/Rotations matrix to Euler angles'
//  '<S6>'   : 'ArduCopter_MinnieTrajController/log muxer'
//  '<S7>'   : 'ArduCopter_MinnieTrajController/Multicopter INDI Loiter Flight Mode/Delta attitude to stick commands'
//  '<S8>'   : 'ArduCopter_MinnieTrajController/Multicopter INDI Loiter Flight Mode/Discrete Derivative'
//  '<S9>'   : 'ArduCopter_MinnieTrajController/Multicopter INDI Loiter Flight Mode/Horizontal NDI position (NE) controller cascade with reduced attitude (angle) command'
//  '<S10>'  : 'ArduCopter_MinnieTrajController/Multicopter INDI Loiter Flight Mode/Multicopter Altitude Hold Flight Mode with INDI and reduced attitude control'
//  '<S11>'  : 'ArduCopter_MinnieTrajController/Multicopter INDI Loiter Flight Mode/Waypoint Reference'
//  '<S12>'  : 'ArduCopter_MinnieTrajController/Multicopter INDI Loiter Flight Mode/command direction'
//  '<S13>'  : 'ArduCopter_MinnieTrajController/Multicopter INDI Loiter Flight Mode/Delta attitude to stick commands/MATLAB Function1'
//  '<S14>'  : 'ArduCopter_MinnieTrajController/Multicopter INDI Loiter Flight Mode/Delta attitude to stick commands/conversion to total attitude command'
//  '<S15>'  : 'ArduCopter_MinnieTrajController/Multicopter INDI Loiter Flight Mode/Delta attitude to stick commands/conversion to total attitude command/DCM to quaternions'
//  '<S16>'  : 'ArduCopter_MinnieTrajController/Multicopter INDI Loiter Flight Mode/Delta attitude to stick commands/conversion to total attitude command/MATLAB Function'
//  '<S17>'  : 'ArduCopter_MinnieTrajController/Multicopter INDI Loiter Flight Mode/Delta attitude to stick commands/conversion to total attitude command/MATLAB Function1'
//  '<S18>'  : 'ArduCopter_MinnieTrajController/Multicopter INDI Loiter Flight Mode/Delta attitude to stick commands/conversion to total attitude command/MATLAB Function4'
//  '<S19>'  : 'ArduCopter_MinnieTrajController/Multicopter INDI Loiter Flight Mode/Delta attitude to stick commands/conversion to total attitude command/quaternion to lean angles'
//  '<S20>'  : 'ArduCopter_MinnieTrajController/Multicopter INDI Loiter Flight Mode/Delta attitude to stick commands/conversion to total attitude command/total reduced attitude command (quaternion)'
//  '<S21>'  : 'ArduCopter_MinnieTrajController/Multicopter INDI Loiter Flight Mode/Delta attitude to stick commands/conversion to total attitude command/quaternion to lean angles/MATLAB Function4'
//  '<S22>'  : 'ArduCopter_MinnieTrajController/Multicopter INDI Loiter Flight Mode/Delta attitude to stick commands/conversion to total attitude command/quaternion to lean angles/quaternion to  reduced attitude unit vector'
//  '<S23>'  : 'ArduCopter_MinnieTrajController/Multicopter INDI Loiter Flight Mode/Horizontal NDI position (NE) controller cascade with reduced attitude (angle) command/Convert to reduced attitude  pseudo control'
//  '<S24>'  : 'ArduCopter_MinnieTrajController/Multicopter INDI Loiter Flight Mode/Horizontal NDI position (NE) controller cascade with reduced attitude (angle) command/filtered command'
//  '<S25>'  : 'ArduCopter_MinnieTrajController/Multicopter INDI Loiter Flight Mode/Horizontal NDI position (NE) controller cascade with reduced attitude (angle) command/measures'
//  '<S26>'  : 'ArduCopter_MinnieTrajController/Multicopter INDI Loiter Flight Mode/Horizontal NDI position (NE) controller cascade with reduced attitude (angle) command/position controller'
//  '<S27>'  : 'ArduCopter_MinnieTrajController/Multicopter INDI Loiter Flight Mode/Horizontal NDI position (NE) controller cascade with reduced attitude (angle) command/Convert to reduced attitude  pseudo control/MATLAB Function1'
//  '<S28>'  : 'ArduCopter_MinnieTrajController/Multicopter INDI Loiter Flight Mode/Horizontal NDI position (NE) controller cascade with reduced attitude (angle) command/Convert to reduced attitude  pseudo control/MATLAB Function2'
//  '<S29>'  : 'ArduCopter_MinnieTrajController/Multicopter INDI Loiter Flight Mode/Horizontal NDI position (NE) controller cascade with reduced attitude (angle) command/Convert to reduced attitude  pseudo control/MATLAB Function3'
//  '<S30>'  : 'ArduCopter_MinnieTrajController/Multicopter INDI Loiter Flight Mode/Horizontal NDI position (NE) controller cascade with reduced attitude (angle) command/Convert to reduced attitude  pseudo control/MATLAB Function4'
//  '<S31>'  : 'ArduCopter_MinnieTrajController/Multicopter INDI Loiter Flight Mode/Horizontal NDI position (NE) controller cascade with reduced attitude (angle) command/Convert to reduced attitude  pseudo control/MATLAB Function5'
//  '<S32>'  : 'ArduCopter_MinnieTrajController/Multicopter INDI Loiter Flight Mode/Horizontal NDI position (NE) controller cascade with reduced attitude (angle) command/Convert to reduced attitude  pseudo control/MATLAB Function8'
//  '<S33>'  : 'ArduCopter_MinnieTrajController/Multicopter INDI Loiter Flight Mode/Horizontal NDI position (NE) controller cascade with reduced attitude (angle) command/Convert to reduced attitude  pseudo control/MATLAB Function9'
//  '<S34>'  : 'ArduCopter_MinnieTrajController/Multicopter INDI Loiter Flight Mode/Horizontal NDI position (NE) controller cascade with reduced attitude (angle) command/filtered command/MATLAB Function1'
//  '<S35>'  : 'ArduCopter_MinnieTrajController/Multicopter INDI Loiter Flight Mode/Horizontal NDI position (NE) controller cascade with reduced attitude (angle) command/filtered command/PT1 discrete reference model'
//  '<S36>'  : 'ArduCopter_MinnieTrajController/Multicopter INDI Loiter Flight Mode/Horizontal NDI position (NE) controller cascade with reduced attitude (angle) command/filtered command/PT1 discrete reference model/PT1 discrete with saturations'
//  '<S37>'  : 'ArduCopter_MinnieTrajController/Multicopter INDI Loiter Flight Mode/Horizontal NDI position (NE) controller cascade with reduced attitude (angle) command/position controller/PT2 discrete with saturation'
//  '<S38>'  : 'ArduCopter_MinnieTrajController/Multicopter INDI Loiter Flight Mode/Multicopter Altitude Hold Flight Mode with INDI and reduced attitude control/Discrete Derivative'
//  '<S39>'  : 'ArduCopter_MinnieTrajController/Multicopter INDI Loiter Flight Mode/Multicopter Altitude Hold Flight Mode with INDI and reduced attitude control/Discrete Derivative1'
//  '<S40>'  : 'ArduCopter_MinnieTrajController/Multicopter INDI Loiter Flight Mode/Multicopter Altitude Hold Flight Mode with INDI and reduced attitude control/INDI high level wls control allocation'
//  '<S41>'  : 'ArduCopter_MinnieTrajController/Multicopter INDI Loiter Flight Mode/Multicopter Altitude Hold Flight Mode with INDI and reduced attitude control/MATLAB Function'
//  '<S42>'  : 'ArduCopter_MinnieTrajController/Multicopter INDI Loiter Flight Mode/Multicopter Altitude Hold Flight Mode with INDI and reduced attitude control/MATLAB Function1'
//  '<S43>'  : 'ArduCopter_MinnieTrajController/Multicopter INDI Loiter Flight Mode/Multicopter Altitude Hold Flight Mode with INDI and reduced attitude control/Multicopter (Reduced) Attitude INDI Controller'
//  '<S44>'  : 'ArduCopter_MinnieTrajController/Multicopter INDI Loiter Flight Mode/Multicopter Altitude Hold Flight Mode with INDI and reduced attitude control/Multicopter Altitude INDI Controller1'
//  '<S45>'  : 'ArduCopter_MinnieTrajController/Multicopter INDI Loiter Flight Mode/Multicopter Altitude Hold Flight Mode with INDI and reduced attitude control/PT1 discrete reference model'
//  '<S46>'  : 'ArduCopter_MinnieTrajController/Multicopter INDI Loiter Flight Mode/Multicopter Altitude Hold Flight Mode with INDI and reduced attitude control/PT2 discrete with saturation1'
//  '<S47>'  : 'ArduCopter_MinnieTrajController/Multicopter INDI Loiter Flight Mode/Multicopter Altitude Hold Flight Mode with INDI and reduced attitude control/acceleration g to b frame'
//  '<S48>'  : 'ArduCopter_MinnieTrajController/Multicopter INDI Loiter Flight Mode/Multicopter Altitude Hold Flight Mode with INDI and reduced attitude control/control effectiveness'
//  '<S49>'  : 'ArduCopter_MinnieTrajController/Multicopter INDI Loiter Flight Mode/Multicopter Altitude Hold Flight Mode with INDI and reduced attitude control/INDI high level wls control allocation/INDI control allocation'
//  '<S50>'  : 'ArduCopter_MinnieTrajController/Multicopter INDI Loiter Flight Mode/Multicopter Altitude Hold Flight Mode with INDI and reduced attitude control/INDI high level wls control allocation/MATLAB Function'
//  '<S51>'  : 'ArduCopter_MinnieTrajController/Multicopter INDI Loiter Flight Mode/Multicopter Altitude Hold Flight Mode with INDI and reduced attitude control/INDI high level wls control allocation/INDI control allocation/MATLAB Function'
//  '<S52>'  : 'ArduCopter_MinnieTrajController/Multicopter INDI Loiter Flight Mode/Multicopter Altitude Hold Flight Mode with INDI and reduced attitude control/INDI high level wls control allocation/INDI control allocation/MATLAB Function1'
//  '<S53>'  : 'ArduCopter_MinnieTrajController/Multicopter INDI Loiter Flight Mode/Multicopter Altitude Hold Flight Mode with INDI and reduced attitude control/Multicopter (Reduced) Attitude INDI Controller/error computation'
//  '<S54>'  : 'ArduCopter_MinnieTrajController/Multicopter INDI Loiter Flight Mode/Multicopter Altitude Hold Flight Mode with INDI and reduced attitude control/Multicopter (Reduced) Attitude INDI Controller/measure'
//  '<S55>'  : 'ArduCopter_MinnieTrajController/Multicopter INDI Loiter Flight Mode/Multicopter Altitude Hold Flight Mode with INDI and reduced attitude control/Multicopter (Reduced) Attitude INDI Controller/ny control'
//  '<S56>'  : 'ArduCopter_MinnieTrajController/Multicopter INDI Loiter Flight Mode/Multicopter Altitude Hold Flight Mode with INDI and reduced attitude control/Multicopter (Reduced) Attitude INDI Controller/ny from reference'
//  '<S57>'  : 'ArduCopter_MinnieTrajController/Multicopter INDI Loiter Flight Mode/Multicopter Altitude Hold Flight Mode with INDI and reduced attitude control/Multicopter (Reduced) Attitude INDI Controller/ny measured'
//  '<S58>'  : 'ArduCopter_MinnieTrajController/Multicopter INDI Loiter Flight Mode/Multicopter Altitude Hold Flight Mode with INDI and reduced attitude control/Multicopter (Reduced) Attitude INDI Controller/reference model'
//  '<S59>'  : 'ArduCopter_MinnieTrajController/Multicopter INDI Loiter Flight Mode/Multicopter Altitude Hold Flight Mode with INDI and reduced attitude control/Multicopter (Reduced) Attitude INDI Controller/error computation/angle error'
//  '<S60>'  : 'ArduCopter_MinnieTrajController/Multicopter INDI Loiter Flight Mode/Multicopter Altitude Hold Flight Mode with INDI and reduced attitude control/Multicopter (Reduced) Attitude INDI Controller/error computation/wrap angle'
//  '<S61>'  : 'ArduCopter_MinnieTrajController/Multicopter INDI Loiter Flight Mode/Multicopter Altitude Hold Flight Mode with INDI and reduced attitude control/Multicopter (Reduced) Attitude INDI Controller/error computation/wrap angle1'
//  '<S62>'  : 'ArduCopter_MinnieTrajController/Multicopter INDI Loiter Flight Mode/Multicopter Altitude Hold Flight Mode with INDI and reduced attitude control/Multicopter (Reduced) Attitude INDI Controller/measure/DCM to quaternions'
//  '<S63>'  : 'ArduCopter_MinnieTrajController/Multicopter INDI Loiter Flight Mode/Multicopter Altitude Hold Flight Mode with INDI and reduced attitude control/Multicopter (Reduced) Attitude INDI Controller/measure/MATLAB Function1'
//  '<S64>'  : 'ArduCopter_MinnieTrajController/Multicopter INDI Loiter Flight Mode/Multicopter Altitude Hold Flight Mode with INDI and reduced attitude control/Multicopter (Reduced) Attitude INDI Controller/reference model/MATLAB Function'
//  '<S65>'  : 'ArduCopter_MinnieTrajController/Multicopter INDI Loiter Flight Mode/Multicopter Altitude Hold Flight Mode with INDI and reduced attitude control/Multicopter (Reduced) Attitude INDI Controller/reference model/MATLAB Function1'
//  '<S66>'  : 'ArduCopter_MinnieTrajController/Multicopter INDI Loiter Flight Mode/Multicopter Altitude Hold Flight Mode with INDI and reduced attitude control/Multicopter (Reduced) Attitude INDI Controller/reference model/PT1 discrete reference model2'
//  '<S67>'  : 'ArduCopter_MinnieTrajController/Multicopter INDI Loiter Flight Mode/Multicopter Altitude Hold Flight Mode with INDI and reduced attitude control/Multicopter (Reduced) Attitude INDI Controller/reference model/PT2 discrete reference model'
//  '<S68>'  : 'ArduCopter_MinnieTrajController/Multicopter INDI Loiter Flight Mode/Multicopter Altitude Hold Flight Mode with INDI and reduced attitude control/Multicopter (Reduced) Attitude INDI Controller/reference model/Subsystem'
//  '<S69>'  : 'ArduCopter_MinnieTrajController/Multicopter INDI Loiter Flight Mode/Multicopter Altitude Hold Flight Mode with INDI and reduced attitude control/Multicopter (Reduced) Attitude INDI Controller/reference model/Subsystem1'
//  '<S70>'  : 'ArduCopter_MinnieTrajController/Multicopter INDI Loiter Flight Mode/Multicopter Altitude Hold Flight Mode with INDI and reduced attitude control/Multicopter (Reduced) Attitude INDI Controller/reference model/Subsystem2'
//  '<S71>'  : 'ArduCopter_MinnieTrajController/Multicopter INDI Loiter Flight Mode/Multicopter Altitude Hold Flight Mode with INDI and reduced attitude control/Multicopter (Reduced) Attitude INDI Controller/reference model/lean angles 2 lean vector'
//  '<S72>'  : 'ArduCopter_MinnieTrajController/Multicopter INDI Loiter Flight Mode/Multicopter Altitude Hold Flight Mode with INDI and reduced attitude control/Multicopter (Reduced) Attitude INDI Controller/reference model/PT1 discrete reference model2/PT1 discrete with saturations'
//  '<S73>'  : 'ArduCopter_MinnieTrajController/Multicopter INDI Loiter Flight Mode/Multicopter Altitude Hold Flight Mode with INDI and reduced attitude control/Multicopter (Reduced) Attitude INDI Controller/reference model/PT2 discrete reference model/PT2 discrete with saturation'
//  '<S74>'  : 'ArduCopter_MinnieTrajController/Multicopter INDI Loiter Flight Mode/Multicopter Altitude Hold Flight Mode with INDI and reduced attitude control/Multicopter (Reduced) Attitude INDI Controller/reference model/Subsystem/PT2 discrete with saturation'
//  '<S75>'  : 'ArduCopter_MinnieTrajController/Multicopter INDI Loiter Flight Mode/Multicopter Altitude Hold Flight Mode with INDI and reduced attitude control/Multicopter (Reduced) Attitude INDI Controller/reference model/Subsystem1/PT2 discrete with saturation'
//  '<S76>'  : 'ArduCopter_MinnieTrajController/Multicopter INDI Loiter Flight Mode/Multicopter Altitude Hold Flight Mode with INDI and reduced attitude control/Multicopter (Reduced) Attitude INDI Controller/reference model/Subsystem2/PT2 discrete with saturation'
//  '<S77>'  : 'ArduCopter_MinnieTrajController/Multicopter INDI Loiter Flight Mode/Multicopter Altitude Hold Flight Mode with INDI and reduced attitude control/Multicopter Altitude INDI Controller1/error computation'
//  '<S78>'  : 'ArduCopter_MinnieTrajController/Multicopter INDI Loiter Flight Mode/Multicopter Altitude Hold Flight Mode with INDI and reduced attitude control/Multicopter Altitude INDI Controller1/measure'
//  '<S79>'  : 'ArduCopter_MinnieTrajController/Multicopter INDI Loiter Flight Mode/Multicopter Altitude Hold Flight Mode with INDI and reduced attitude control/Multicopter Altitude INDI Controller1/ny control'
//  '<S80>'  : 'ArduCopter_MinnieTrajController/Multicopter INDI Loiter Flight Mode/Multicopter Altitude Hold Flight Mode with INDI and reduced attitude control/Multicopter Altitude INDI Controller1/ny from reference'
//  '<S81>'  : 'ArduCopter_MinnieTrajController/Multicopter INDI Loiter Flight Mode/Multicopter Altitude Hold Flight Mode with INDI and reduced attitude control/Multicopter Altitude INDI Controller1/ny measured'
//  '<S82>'  : 'ArduCopter_MinnieTrajController/Multicopter INDI Loiter Flight Mode/Multicopter Altitude Hold Flight Mode with INDI and reduced attitude control/Multicopter Altitude INDI Controller1/reference model'
//  '<S83>'  : 'ArduCopter_MinnieTrajController/Multicopter INDI Loiter Flight Mode/Multicopter Altitude Hold Flight Mode with INDI and reduced attitude control/Multicopter Altitude INDI Controller1/reference model/PT1 discrete reference model'
//  '<S84>'  : 'ArduCopter_MinnieTrajController/Multicopter INDI Loiter Flight Mode/Multicopter Altitude Hold Flight Mode with INDI and reduced attitude control/Multicopter Altitude INDI Controller1/reference model/PT1 discrete reference model/PT1 discrete with saturations'
//  '<S85>'  : 'ArduCopter_MinnieTrajController/Multicopter INDI Loiter Flight Mode/Multicopter Altitude Hold Flight Mode with INDI and reduced attitude control/PT1 discrete reference model/PT1 discrete with saturations'
//  '<S86>'  : 'ArduCopter_MinnieTrajController/Multicopter INDI Loiter Flight Mode/Multicopter Altitude Hold Flight Mode with INDI and reduced attitude control/acceleration g to b frame/MATLAB Function'
//  '<S87>'  : 'ArduCopter_MinnieTrajController/Multicopter INDI Loiter Flight Mode/Multicopter Altitude Hold Flight Mode with INDI and reduced attitude control/acceleration g to b frame/dcm2Lean'
//  '<S88>'  : 'ArduCopter_MinnieTrajController/Multicopter INDI Loiter Flight Mode/Multicopter Altitude Hold Flight Mode with INDI and reduced attitude control/control effectiveness/MATLAB Function'
//  '<S89>'  : 'ArduCopter_MinnieTrajController/Multicopter INDI Loiter Flight Mode/Waypoint Reference/Discrete Derivative'
//  '<S90>'  : 'ArduCopter_MinnieTrajController/Multicopter INDI Loiter Flight Mode/Waypoint Reference/Discrete Derivative1'
//  '<S91>'  : 'ArduCopter_MinnieTrajController/Multicopter INDI Loiter Flight Mode/Waypoint Reference/waypoint Reference System'
//  '<S92>'  : 'ArduCopter_MinnieTrajController/Multicopter INDI Loiter Flight Mode/Waypoint Reference/waypoint Reference System/MATLAB Function'
//  '<S93>'  : 'ArduCopter_MinnieTrajController/Multicopter INDI Loiter Flight Mode/Waypoint Reference/waypoint Reference System/MATLAB Function1'
//  '<S94>'  : 'ArduCopter_MinnieTrajController/Multicopter INDI Loiter Flight Mode/Waypoint Reference/waypoint Reference System/MATLAB Function2'
//  '<S95>'  : 'ArduCopter_MinnieTrajController/Multicopter INDI Loiter Flight Mode/Waypoint Reference/waypoint Reference System/MATLAB Function3'
//  '<S96>'  : 'ArduCopter_MinnieTrajController/Multicopter INDI Loiter Flight Mode/Waypoint Reference/waypoint Reference System/MATLAB Function6'
//  '<S97>'  : 'ArduCopter_MinnieTrajController/Multicopter INDI Loiter Flight Mode/Waypoint Reference/waypoint Reference System/Subsystem2'
//  '<S98>'  : 'ArduCopter_MinnieTrajController/Multicopter INDI Loiter Flight Mode/Waypoint Reference/waypoint Reference System/Trajectory from Waypoints'
//  '<S99>'  : 'ArduCopter_MinnieTrajController/Multicopter INDI Loiter Flight Mode/Waypoint Reference/waypoint Reference System/trajGetMatch'
//  '<S100>' : 'ArduCopter_MinnieTrajController/Multicopter INDI Loiter Flight Mode/Waypoint Reference/waypoint Reference System/Subsystem2/n-th Order Filter1'
//  '<S101>' : 'ArduCopter_MinnieTrajController/Multicopter INDI Loiter Flight Mode/Waypoint Reference/waypoint Reference System/Subsystem2/n-th Order Filter2'
//  '<S102>' : 'ArduCopter_MinnieTrajController/Multicopter INDI Loiter Flight Mode/Waypoint Reference/waypoint Reference System/Subsystem2/n-th Order Filter3'
//  '<S103>' : 'ArduCopter_MinnieTrajController/Multicopter INDI Loiter Flight Mode/Waypoint Reference/waypoint Reference System/Trajectory from Waypoints/Detect Rise Positive'
//  '<S104>' : 'ArduCopter_MinnieTrajController/Multicopter INDI Loiter Flight Mode/Waypoint Reference/waypoint Reference System/Trajectory from Waypoints/Subsystem2'
//  '<S105>' : 'ArduCopter_MinnieTrajController/Multicopter INDI Loiter Flight Mode/Waypoint Reference/waypoint Reference System/Trajectory from Waypoints/Detect Rise Positive/Positive'
//  '<S106>' : 'ArduCopter_MinnieTrajController/Multicopter INDI Loiter Flight Mode/Waypoint Reference/waypoint Reference System/Trajectory from Waypoints/Subsystem2/trajFromWaypoints'
//  '<S107>' : 'ArduCopter_MinnieTrajController/Multicopter INDI Loiter Flight Mode/command direction/MATLAB Function'

#endif                                 // RTW_HEADER_MatlabController_h_

//
// File trailer for generated code.
//
// [EOF]
//
