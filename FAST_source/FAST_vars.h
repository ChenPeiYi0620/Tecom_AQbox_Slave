/*
 * FAST_vars.h
 *
 *  Created on: 2024¦~10¤ë8¤é
 *      Author: MotorTech
 */

#ifndef FAST_VARS_H_
#define FAST_VARS_H_

#include "f2838x_device.h"
#include "../motorVars.h"
#include "sci_tx.h"
#include "FAST.h"
#include "diag.h"


// Current offset
extern float Cur_OffsetA ;
extern float Cur_OffsetB ;
extern float set ;
extern float offsetA,offsetA_sum,offsetA_avg,offsetB,offsetB_sum,offsetB_avg;
extern int cur_offset_complete;
extern float Va,Vb,Vc;

// Input Parameters
//------------------------------------
extern float Vdc;//47.5 DCbus;
extern float CURRENT ; //67.031 is measured by probe with 5 turns, 71.7391 is theoretical value
extern float Flux_Rs ;//0.4
extern float Ls;//float(0.009615)
extern float Lq;//0.0144234
extern float Ld;//0.0144234
extern float Lamda_m ;//0.045
extern float32 ECAP_FREQ;
extern float32 PWM_SAMPLING ;
extern float32 ECAP_PU;
//float32 Tsim = 0.0001;
extern float32 Tsim;
extern float POLE_PAIRS;
extern float cFangle ;//;-0.0410000011;



extern Uint32 last_count,corr_count;
//Uint32 capturecount1,capturecount2,capturecount3,capturecount4,capturecount5,capturecount6,capturecount7,capturecount8;
extern float32 capturecountthreshold3,capturecountthreshold4;
extern float modify1,modify2;
extern CLARKE Current;
extern CLARKE fluxClarke;
extern CLARKE Vabc;
extern PARK  Park_Flux;
extern IPARK ipark_Flux;
extern IPARK Flux_Cur;
extern float Flux_A_est_active[4],Flux_B_est_active[4];
extern int VT_enable;
extern float ScopeFlag;
extern float i,j;
extern float Last_current_As,Last_current_Bs;
// FAST variables
//---------------------------------------
//float ClarkeV_Alpha,ClarkeV_Beta;
extern float clarke_Alpha_lpf,clarke_Beta_lpf;
extern float clarke_Alphafloat,clarke_Betafloat;
extern float V_A,V_B,I_A,I_B;
extern float Emf_A_lpf2,Emf_B_lpf2,Emf_A_last,Emf_B_last,Emf_A_lpf_last,Emf_B_lpf_last;
extern float Emf_A_lpf_f,Emf_B_lpf_f,Emf_A_lpf_last_f,Emf_B_lpf_last_f,Emf_A_last_f,Emf_B_last_f,Emf_A_lpf,Emf_B_lpf;
extern EMF EMF1, Current_lpf;
extern IPARK Emf_ab_lpf,Emf_ab_lpf2;
extern float we,intgrBW;
extern float32 we_f,intgrBW_f,intgrBW_T_f,wehz;
extern float32 coef  , CrossFreq ;
extern float32 fast_wc,current_lpf_BW;
extern int cur_LPF_en;
extern float32 Emf_lpf_coef_one,Emf_lpf_coef_two;
extern float mag_compfloat,demag_mag_compfloat;
extern float phasefloat,demag_phasefloat;
extern float phase,mag_comp2,phase_comp,sinPU_test,test_atan,test_phase;
extern float mag_comp,demag_comp;
extern float idr,iqr,idr2_last,iqr2_last,idr1_last,iqr1_last;
extern float Vdr,Vqr,Vdr_last,Vqr_last,Vdr2_last,Vqr2_last;
extern float Vds,Vqs,Vds_last,Vqs_last;
extern float ids,iqs,ids_last,iqs_last;
extern float idr_avg,iqr_avg,Vdr_avg,Vqr_avg;
extern float idr_sum,iqr_sum,Vdr_sum,Vqr_sum;
extern float countIV ,countIVpoint;
extern float Torq_est,Torq_est_cur_mone;
extern int16 ld_floor;
extern IPARK Emf_lpf2;
extern SPEED_MEAS_QEP Speed_F,Speed_V;
extern IPARK Emf_ab_lpf;
extern IPARK Flux_mag;
extern PARK FAST_Cur,Flux_est;
extern float Torq_est,torq_i,Torq_est_sum,Torq_est_avg;
extern float efficiency;
extern float Torq_est_cur_mone,Torq_est_cur_mone_sum,Torq_est_cur_mone_avg,Torq_est_cur_mtwo,Torq_est_cur_mtwo_sum,Torq_est_cur_mtwo_avg;
extern float Torque_DB,Torque_DB_sum,Torque_DB_avg;
extern float torq_count,torq_i;
extern float currentAs,currentBs;
extern float Speed_est,Speed_est_sum,Speed_est_avg,Cur_measure,Cur_measure_sum,Cur_measure_avg;
extern float Power1,Power1_avg,Power1_avg_last;
extern float Speed_est_normal,we_est,we_mech;
extern int avg_load2;

// Low pass filter paramter
//---------------------------------------------
extern float wc5,k5;


//----------------------------------------------
// Recursive Least square
//----------------------------------------------
extern float forgetting_coef ;//0.999
extern float forgetting_coefinv;//float(1.001001001)
extern float est_enable ;
extern int enable_err;
extern int estimate_c;
extern float F_angle_sum,qep_sum,angle_err;

//----------------------------------------------
// Affine projection Algorithms
//----------------------------------------------
extern float u ;
extern float ita ;
// first time variable
//----------------------------------------------
extern float aT1_1,bT1_1,cT1_1,dT1_1;
extern float adbcT1;
extern float qT1_11,qT1_12,qT1_21,qT1_22;
extern float aT1_2,bT1_2,cT1_2,dT1_2;
//----------------------------------------------
// second time variable
//----------------------------------------------
extern float aT2_1,bT2_1,cT2_1,dT2_1;
extern float adbcT2;
extern float qT2_11,qT2_12,qT2_21,qT2_22;
extern float aT2_2,bT2_2,cT2_2,dT2_2;
extern myfloat myf_Rs, myf_Ls;






#endif /* FAST_VARS_H_ */
