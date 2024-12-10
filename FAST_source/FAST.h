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


float lastPWM_duty(float *ref1_cap2_last, float *ref1_cap4_last, float CAP2,
                   float CAP4);
void LinetoPhase(_iq Line_A, _iq Line_B, CLARKE *phase_volt);
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


#endif
