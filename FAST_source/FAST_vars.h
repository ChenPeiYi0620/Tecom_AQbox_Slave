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

// Vsensing variables
//----------------------------------------
//float ram1,ram2,ram3,ram4,ram5,ram6,ram7,ram8,ram9,ram10;
extern float ref_volt1,ref_past1,ref_volt1_dir;
extern float ref_volt2,ref_past2,ref_volt2_dir;
extern float ref_volt3,ref_past3,ref_volt3_dir;
extern float phase_volt1,phase_volt1_past,phase_volt1_dir;
extern float phase_volt2,phase_volt2_past,phase_volt2_dir;
extern float phase_volt3,phase_volt3_dir;
extern float ref1_cap2_last,ref1_cap4_last,ref2_cap2_last,ref2_cap4_last,ref3_cap2_last,ref3_cap4_last;
extern float phase1_cap2_last,phase1_cap4_last,phase2_cap2_last,phase2_cap4_last,phase3_cap2_last,phase3_cap4_last;
#define PWM_length 20000;
extern float ref_theta;
extern float inv1,inv2,inv3,inv1_past,inv2_past,inv3_past;
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
extern float Emf_A,Emf_B,Emf_A_lpf2,Emf_B_lpf2,Emf_A_last,Emf_B_last,Emf_A_lpf_last,Emf_B_lpf_last;
extern float Emf_A_lpf_f,Emf_B_lpf_f,Emf_A_lpf_last_f,Emf_B_lpf_last_f,Emf_A_last_f,Emf_B_last_f,Emf_A_lpf,Emf_B_lpf;
extern float Emf_mag;
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
extern float Power1,Power2,Power3;
extern float Speed_est_normal,we_est,we_mech;
extern int avg_load2;

// Low pass filter paramter
//---------------------------------------------
extern float wc5,k5;

// demagnetization parametric
extern Uint16 avg_counter,avg_limit;
extern float fs_demag_avg_A,fs_demag_avg_B,local_demag_avg_A,local_demag_avg_B;
extern float local_demag_avg_last_A,local_demag_avg_last_B,fs_demag_avg_last_A,fs_demag_avg_last_B;
extern float lumda_h,lumda_offset;
extern float local_demag,fs_demag,HI_LD,HI_UD;
extern float scale1;//50
extern float scale;
extern Uint16 UD_flag,LD_flag;
extern float omega_r,omega_r_last,omega_r2_last;
extern int CountT ;
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
//V pole clib parameters
//Vpol_feature(V1_feature,ref_elms1,ref_volt1,ref_volt1_last,corr_count, corr_count_limit);
extern float ref_count,ref_freq_hz,ref_freq_hz_last,buff_count;
extern Uint16 ref_load,trig1_last;
extern float V1_feature[2],ref_elms1[20],V2_feature[2],ref_elms2[10],descend_ratio,coef_d;
extern Uint16 corr_count1,corr_count2,corr_count_limit;
extern float ref_volt1_last,ref_volt2_last;
extern float phase_volt1_last,phase_volt2_last,phase_volt2_las1,phase_volt2_las2,inverse1,inverse2,trig1,trig2,descend_trig1,descend_trig2,in_trig1,in_trig2;
// parameters for demag diagnosis
extern float LD_magcomp,LD_phase_comp,LD_harm;
extern float UD_feature[2],LD_feature[2],UD_feature_avg[2],LD_feature_avg[2],UD_feature_avg_last[2],LD_feature_avg_last[2],HI_demag[4];
extern float flux_energy,flux_energy_last;
extern Uint16 avg_count1,avg_limit1,avg_load1;
extern float avg_test,avg_test_last;
extern float UD_mag,LD_mag;
extern IPARK mag_flux;
extern float F_angle,V_angle;
//----------------------------------------------
// SCI paras
//----------------------------------------------
extern unsigned int test_ftoint;
extern myfloat myfloat_test;
extern myuint myuint_test;
extern int send_en,FAST_enable,send_delay;
extern Uint16 send_datalength;
extern Uint32 sci_count,overtime_count,send_count;
extern Uint16 ReceivedChar[16],ReceivedChar2,device_number;
extern Uint16 crc1,crc2;
extern float temp_test;
extern Uint16 temp_data1,temp_data2;
extern Uint16 update_FAST_en;
extern Uint16 data_transmit_test;
extern Uint16 Rs_U16,Ls_U16,H_8bits,L_8bits ;
extern Uint16 test_count,cmd_rcv_time;



#endif /* FAST_VARS_H_ */
