#include <stdint.h>
#define YES 0x1
#define NO 0x0
#define Pole 4
#define Zero1 649540.0
#define Zero2 544998.0
#define PeriodMax 7200
#define PeriodMid 1800
#define CUR_PU 20.0
#define SPD_PU 1000.0
#define DIR_SPD -1.0
#define Pos_Zero 0.0
#define Line  10000//推杆电机 4096  旋变电机 16384  线数乘4 
typedef struct {  float  Alpha;       // Input: stationary d-axis stator variable
                  float  Beta;        // Input: stationary q-axis stator variable
                  float  Angle;       // Input: rotating angle (pu)
                  float  Ds;          // Output: rotating d-axis stator variable
                  float  Qs;          // Output: rotating q-axis stator variable
                  float  Sine;
                  float  Cosine;
                } PARK;
#define PARK_DEFAULTS {   0, \
                          0, \
                          0, \
                          0, \
                          0, \
                          0, \
                          0, \
                          }
typedef struct {  float  Alpha;       // Output: stationary d-axis stator variable
float  Beta;        // Output: stationary q-axis stator variable
float  Angle;       // Input: rotating angle (pu)
float  Ds;          // Input: rotating d-axis stator variable
float  Qs;          // Input: rotating q-axis stator variable
float  Sine;        // Input: Sine term
float  Cosine;      // Input: Cosine term
                } IPARK;
#define IPARK_DEFAULTS {  0, \
                          0, \
                          0, \
                          0, \
                          0, \
                          0, \
                          0, \
                       }
typedef struct {  float  Ref;             // Input: reference set-point
float  Fbk;             // Input: feedback
float  Out;             // Output: controller output
float  Kp;              // Parameter: proportional loop gain
float  Ki;              // Parameter: integral gain
float  Umax;            // Parameter: upper saturation limit
float  Umin;            // Parameter: lower saturation limit
float  up;              // Data: proportional term
float  ui;              // Data: integral term
float  v1;              // Data: pre-saturated controller output
float  i1;              // Data: integrator storage: ui(k-1)
float  w1;              // Data: saturation record: [u(k-1) - v(k-1)]
                        } PI_CONTROLLER;

#define PI_CONTROLLER_DEFAULTS {        \
                                 0,           \
                                 0,           \
                                 0,           \
                                 1.0,    \
                                 0.0,    \
                                 1.0,    \
                                 -1.0,   \
                                 0.0,    \
                                 0.0,    \
                                 0.0,    \
                                 0.0,    \
                                 1.0,    \
                               }
typedef struct {  float  As;          // Input: phase-a stator variable
                  float  Bs;          // Input: phase-b stator variable
                  float  Cs;          // Input: phase-c stator variable
                  float  Alpha;       // Output: stationary d-axis stator variable
                  float  Beta;        // Output: stationary q-axis stator variable
               } CLARKE;

#define CLARKE_DEFAULTS { 0, \
                          0, \
                          0, \
                          0, \
                          0, \
                         }
typedef struct  { float  Ualpha;          // Input: reference alpha-axis phase voltage
                  float  Ubeta;           // Input: reference beta-axis phase voltage
                  float  Ta;              // Output: reference phase-a switching function
                  float  Tb;              // Output: reference phase-b switching function
                  float  Tc;              // Output: reference phase-c switching function
                  float  tmp1;            // Variable: temp variable
                  float  tmp2;            // Variable: temp variable
                  float  tmp3;            // Variable: temp variable
                  uint16_t VecSector;     // Space vector sector
                   } SVGEN;
#define SVGEN_DEFAULTS { 0,0,0,0,0,0,0,0,0 }


 /*------------------------------------------------------------------------------
PI_GRANDO Macro Definition
------------------------------------------------------------------------------*/
 #define PI_MACRO(v)                                             \
                                                                                   \
/* proportional term */                                     \
 v.up = v.Ref - v.Fbk;                                       \
                                                                                   \
/* integral term */                                         \
v.ui =  (fabs(v.Out-v.v1)<0.000001)?(v.Ki * v.up+ v.i1) : v.i1;   \
v.i1 = v.ui;                                                \
                                                                                   \
/* control output */                                        \
v.v1 = v.Kp * (v.up  + v.ui); \
if(v.v1 > v.Umax) \
    v.Out =  v.Umax; \
else if(v.v1 < v.Umin) \
    v.Out =  v.Umin; \
else \
    v.Out = v.v1; \


#define CLARKE_MACRO(v)                                         \
                                                                \
v.Alpha = v.As;                                                 \
v.Beta = (v.As +2 * v.Bs ) * 0.57735026918963;   \

#define PARK_MACRO(v)                                           \
                                                                \
    v.Sine =  sinf(v.Angle*2*3.1415926);           \
    v.Cosine =  cosf(v.Angle*2*3.1415926);           \
    v.Ds = v.Alpha * v.Cosine + v.Beta * v.Sine;    \
    v.Qs = v.Beta * v.Cosine -v.Alpha * v.Sine; \

#define IPARK_MACRO(v)                                      \
                                                            \
v.Alpha = v.Ds * v.Cosine - v.Qs * v.Sine;      \
v.Beta  = v.Qs * v.Cosine + v.Ds * v.Sine;
#define SVGENDQ_MACRO(v)                                                        \
    v.tmp1= v.Ubeta;                                                            \
    v.tmp2= v.Ubeta * 0.5 + 0.866 * v.Ualpha;                                   \
    v.tmp3= v.tmp2 - v.tmp1;                                                    \
                                                                                \
    v.VecSector=3;                                                              \
    v.VecSector=(v.tmp2> 0)?( v.VecSector-1):v.VecSector;                       \
    v.VecSector=(v.tmp3> 0)?( v.VecSector-1):v.VecSector;                       \
    v.VecSector=(v.tmp1< 0)?(7-v.VecSector) :v.VecSector;                       \
                                                                                \
    if     (v.VecSector==1 || v.VecSector==4)                                   \
      {     v.Ta= v.tmp2;                                                       \
            v.Tb= v.tmp1-v.tmp3;                                                \
            v.Tc=-v.tmp2;                                                       \
      }                                                                         \
                                                                                \
    else if(v.VecSector==2 || v.VecSector==5)                                   \
      {     v.Ta= v.tmp3+v.tmp2;                                                \
            v.Tb= v.tmp1;                                                       \
            v.Tc=-v.tmp1;                                                       \
      }                                                                         \
                                                                                \
    else                                                                        \
      {     v.Ta= v.tmp3;                                                       \
            v.Tb=-v.tmp3;                                                       \
            v.Tc=-(v.tmp1+v.tmp2);                                              \
      }
 typedef struct {
                float MfuncC1;        // Input: EPWM1 A&B Duty cycle ratio (-1,1)
                float MfuncC2;        // Input: EPWM2 A&B Duty cycle ratio (-1,1)
                float MfuncC3;        // Input: EPWM3 A&B Duty cycle ratio (-1,1)
								uint16_t DutyA;
								uint16_t DutyB;
								uint16_t DutyC;
                } PWMGEN ;
#define PWMGEN_DEFAULTS   { \
                             0.0,    \
                             0.0,    \
                             0.0,    \
	                           0,    \
                             0,    \
                             0,    \
                             }
