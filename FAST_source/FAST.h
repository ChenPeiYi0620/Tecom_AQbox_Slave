/*
 * FAST.h
 *
 *  Created on: 2021/12/01
 *
 */
#ifndef FAST_H_
#define FAST_H_
#include "../motorVars.h"

typedef struct {  float  d_out;       // Input: stationary d-axis stator variable
                  float  q_out;        // Input: stationary q-axis stator variable
                  float  d_outlast;       // Input: rotating angle (pu)
                  float  q_outlast;          // Output: rotating d-axis stator variable
                  float  LamdaCur_d;          // Output: rotating q-axis stator variable
                  float  LamdaCur_q;
                  float  LamdaCur_dlast;
                  float  LamdaCur_qlast;
                  float  LamdaVolt_d;
                  float  LamdaVolt_q;
                  float  Lamda_df;
                  float  Lamda_qf;
                } Comflux;

/*-----------------------------------------------------------------------------
Default initalizer for the PARK object.
-----------------------------------------------------------------------------*/
#define Comflux_DEFAULTS {   0, \
                          0, \
                          0, \
                          0, \
                          0, \
                          0, \
                          0, \
                          0, \
                          0, \
                          0, \
                          0, \
                          0, \
                          }

typedef struct {  float  Alpha;       // Input: stationary d-axis stator variable
                  float  Beta;        // Input: stationary q-axis stator variable
                  float  Alpha_last;       // Input: rotating angle (pu)
                  float  Beta_last;          // Output: rotating d-axis stator variable
                  float  Alpha_LPF;          // Output: rotating q-axis stator variable
                  float  Beta_LPF;
                  float  Alpha_LPF_last;
                  float  Beta_LPF_last;
                } EMF;

/*-----------------------------------------------------------------------------
Default initalizer for the PARK object.
-----------------------------------------------------------------------------*/
#define EMF_DEFAULTS {   0, \
                          0, \
                          0, \
                          0, \
                          0, \
                          0, \
                          0, \
                          0, \
                          }

//typedef struct
//{
//    Uint32 CAP1;
//    Uint32 CAP2;
//    Uint32 CAP3;
//    Uint32 CAP4;
//    Uint32 last_CAP1;
//    Uint32 last_CAP2;
//    Uint32 last_CAP3;
//    Uint32 last_CAP4;
//} PWM_pack;

// Vsensing variables
//----------------------------------------
//float ram1,ram2,ram3,ram4,ram5,ram6,ram7,ram8,ram9,ram10;
extern float ref_volt1,ref_past1,ref_volt1_dir;
extern float ref_volt2,ref_past2,ref_volt2_dir;
extern float ref_volt3,ref_past3,ref_volt3_dir;
extern float phase_volt1,phase_volt1_past,phase_volt1_dir;
extern float phase_volt2,phase_volt2_past,phase_volt2_dir;
extern float phase_volt3,phase_volt3_past,phase_volt3_dir;
extern float phase_volt3,phase_volt3_dir;
extern float ref1_cap2_last,ref1_cap4_last,ref2_cap2_last,ref2_cap4_last,ref3_cap2_last,ref3_cap4_last;
extern float phase1_cap2_last,phase1_cap4_last,phase2_cap2_last,phase2_cap4_last,phase3_cap2_last,phase3_cap4_last;
extern float PWM_length;
extern int integration_mode, inv_disable;
extern float ref_theta;
extern int16 inv1,inv2,inv3,inv1_past,inv2_past,inv3_past,trig1,trig2,trig3;
extern float vol_filter_thres;

//V pole clib parameters
//Vpol_feature(V1_feature,ref_elms1,ref_volt1,ref_volt1_last,corr_count, corr_count_limit);
extern float ref_count,ref_freq_hz,ref_freq_hz_last,buff_count;
extern Uint16 ref_load,trig1_last;
extern float V1_feature[2],ref_elms1[20],V2_feature[2],ref_elms2[10],descend_ratio,coef_d;
extern Uint16 corr_count1,corr_count2,corr_count_limit;
extern float ref_volt1_last,ref_volt2_last;
extern float phase_volt1_last,phase_volt2_last,phase_volt2_las1,phase_volt2_las2,descend_trig1,descend_trig2;


float lastPWM_duty(float *ref1_cap2_last, float *ref1_cap4_last, float CAP2,
                   float CAP4);
void LinetoPhase(_iq Line_A, _iq Line_B, CLARKE *phase_volt);
void LinetoPhase_shift(_iq Vab, _iq Vbc, _iq Vca, CLARKE *phase_volt);
void EMFtoLPF(float samplingTime, float LPF_radius, EMF *EMF);
void Flux_comp(_iq omega_e, _iq LPF_radius, _iq *mag_comp, _iq *phase_comp);
void Flux_CurModel(_iq Ld, _iq Lq, _iq Lamda_m, _iq idr, _iq iqr,
                   IPARK *Flux_Cur);
void Com_fluxobserver(Comflux *comflux, float samplingTime);
void Vpol_feature(_iq *featureval, _iq *ref_elms, _iq NewVal, _iq *lastVal,
                  Uint16 *count, Uint16 n);
void Vpol_calib(_iq ref_volt, _iq *phase_volt, _iq *phase_volt_last,
                _iq featureval[], _iq *inverse, _iq *trig, _iq *descend_trig,
                _iq *in_trig, _iq descend_ratio);
void Vpol_calib_origin(_iq ref, _iq *phase, _iq wehz, _iq capturecount1,
                       _iq capturecount2, _iq capturecountthreshold);
void inverse(_iq *input);
void SFF(_iq SFFA, _iq SFFB, _iq SFFC, _iq *SFF_in, _iq *SSF_out);
void SFF_FAST(_iq wtr_theta, _iq *SFF_in, _iq *SFF_out, _iq epsilon, _iq T);
void voltage_filtering(float last_v, float *current_v, float filter_thres);


#endif
