/*
 * FAST_var.c
 *
 *  Created on: 2024¦~10¤ë8¤é
 *      Author: MotorTech
 */

#include "FAST_vars.h"

// Current offset
float Cur_OffsetA =0.600112915;
float Cur_OffsetB =0.595352173;
float set = 0;
float offsetA,offsetA_sum,offsetA_avg,offsetB,offsetB_sum,offsetB_avg;
int cur_offset_complete=0;
float Va,Vb,Vc;

// Input Parameters
//------------------------------------
float Vdc = (300);//220VAC to DC bus;
float CURRENT = (15.389); //15.389 for TM101-OCS
float Flux_Rs = (12.1);//0.4
float Ls = (0.009615);//(0.009615)
float Lq = (0.0092);//0.0144234
float Ld = (0.0092);//0.0144234
float Lamda_m = (0.043);//0.045
float32 ECAP_FREQ = 200E6;
float32 PWM_SAMPLING = 10E3;
float32 ECAP_PU;
//float32 Tsim = 0.0001;
float32 Tsim = 0.0001;
float POLE_PAIRS = (4);
float cFangle = 0;//;-0.0410000011;

// Vsensing variables
//----------------------------------------
//float ram1,ram2,ram3,ram4,ram5,ram6,ram7,ram8,ram9,ram10;
float ref_volt1,ref_past1,ref_volt1_dir;
float ref_volt2,ref_past2,ref_volt2_dir;
float ref_volt3,ref_past3,ref_volt3_dir;
float phase_volt1,phase_volt1_past,phase_volt1_dir;
float phase_volt2,phase_volt2_past,phase_volt2_dir;
float phase_volt3,phase_volt3_dir;
float ref1_cap2_last,ref1_cap4_last,ref2_cap2_last,ref2_cap4_last,ref3_cap2_last,ref3_cap4_last;
float phase1_cap2_last,phase1_cap4_last,phase2_cap2_last,phase2_cap4_last,phase3_cap2_last,phase3_cap4_last;

float ref_theta;
float inv1,inv2,inv3,inv1_past,inv2_past,inv3_past;
Uint32 last_count,corr_count;
//Uint32 capturecount1,capturecount2,capturecount3,capturecount4,capturecount5,capturecount6,capturecount7,capturecount8;
float32 capturecountthreshold3 = 10,capturecountthreshold4 = 10;
float modify1 = (0.1),modify2 = (0.1);
CLARKE Current;
CLARKE fluxClarke;
CLARKE Vabc;
PARK  Park_Flux;
IPARK ipark_Flux;
IPARK Flux_Cur;
float Flux_A_est_active[4],Flux_B_est_active[4];
int VT_enable;
float ScopeFlag = (1);
float i=0,j=0;
float Last_current_As,Last_current_Bs;
// FAST variables
//---------------------------------------
//float ClarkeV_Alpha,ClarkeV_Beta;
float clarke_Alpha_lpf,clarke_Beta_lpf;
float clarke_Alphafloat,clarke_Betafloat;
float V_A,V_B,I_A,I_B;
float Emf_A,Emf_B,Emf_A_lpf2,Emf_B_lpf2,Emf_A_last,Emf_B_last,Emf_A_lpf_last,Emf_B_lpf_last;
float Emf_A_lpf_f,Emf_B_lpf_f,Emf_A_lpf_last_f,Emf_B_lpf_last_f,Emf_A_last_f,Emf_B_last_f,Emf_A_lpf,Emf_B_lpf;
float Emf_mag;
EMF EMF1, Current_lpf;
IPARK Emf_ab_lpf,Emf_ab_lpf2;
float we,intgrBW;
float32 we_f,intgrBW_f,intgrBW_T_f,wehz;
float32 coef = 0.2 , CrossFreq = 15;
float32 fast_wc,current_lpf_BW=5*200*2*PI;
int cur_LPF_en=0;
float32 Emf_lpf_coef_one,Emf_lpf_coef_two;
float mag_compfloat,demag_mag_compfloat;
float phasefloat,demag_phasefloat;
float phase,mag_comp2,phase_comp,sinPU_test,test_atan,test_phase;
float mag_comp,demag_comp;
float idr,iqr,idr2_last,iqr2_last,idr1_last,iqr1_last;
float Vdr,Vqr,Vdr_last,Vqr_last,Vdr2_last,Vqr2_last;
float Vds,Vqs,Vds_last,Vqs_last;
float ids,iqs,ids_last,iqs_last;
float idr_avg,iqr_avg,Vdr_avg,Vqr_avg;
float idr_sum,iqr_sum,Vdr_sum,Vqr_sum;
float countIV = (0),countIVpoint = (5);
float Torq_est,Torq_est_cur_mone;
int16 ld_floor;
IPARK Emf_lpf2;
SPEED_MEAS_QEP Speed_F,Speed_V;
IPARK Emf_ab_lpf;
IPARK Flux_mag;
PARK FAST_Cur,Flux_est;
float Torq_est,torq_i,Torq_est_sum,Torq_est_avg;
float efficiency;
float Torq_est_cur_mone,Torq_est_cur_mone_sum,Torq_est_cur_mone_avg,Torq_est_cur_mtwo,Torq_est_cur_mtwo_sum,Torq_est_cur_mtwo_avg;
float Torque_DB,Torque_DB_sum,Torque_DB_avg;
float torq_count=(1000),torq_i;
float currentAs,currentBs;
float Speed_est,Speed_est_sum,Speed_est_avg,Cur_measure,Cur_measure_sum,Cur_measure_avg;
float Power1,Power2,Power3;
float Speed_est_normal,we_est,we_mech;
int avg_load2;

// Low pass filter paramter
//---------------------------------------------
float wc5=(0.99),k5=(1);

// demagnetization parametric
Uint16 avg_counter,avg_limit=10000;
float fs_demag_avg_A,fs_demag_avg_B,local_demag_avg_A,local_demag_avg_B;
float local_demag_avg_last_A,local_demag_avg_last_B,fs_demag_avg_last_A,fs_demag_avg_last_B;
float lumda_h=0.00577,lumda_offset=0.0000362;
float local_demag,fs_demag,HI_LD,HI_UD;
float scale1=1;//50
float scale=5;
Uint16 UD_flag,LD_flag;
float omega_r,omega_r_last,omega_r2_last;
int CountT = 20;
//----------------------------------------------
// Recursive Least square
//----------------------------------------------
float forgetting_coef = (0.99);//0.999
float forgetting_coefinv = (1.01010101);//(1.001001001)
float est_enable = (0);
int enable_err=0;
int estimate_c=0;
float F_angle_sum=0,qep_sum=0,angle_err=0;

//----------------------------------------------
// Affine projection Algorithms
//----------------------------------------------
float u = (0.15);
float ita = (1.5);
// first time variable
//----------------------------------------------
float aT1_1,bT1_1,cT1_1,dT1_1;
float adbcT1;
float qT1_11,qT1_12,qT1_21,qT1_22;
float aT1_2,bT1_2,cT1_2,dT1_2;
//----------------------------------------------
// second time variable
//----------------------------------------------
float aT2_1,bT2_1,cT2_1,dT2_1;
float adbcT2;
float qT2_11,qT2_12,qT2_21,qT2_22;
float aT2_2,bT2_2,cT2_2,dT2_2;
//V pole clib parameters
//Vpol_feature(V1_feature,ref_elms1,ref_volt1,ref_volt1_last,corr_count, corr_count_limit);
float ref_count,ref_freq_hz,ref_freq_hz_last,buff_count;
Uint16 ref_load,trig1_last;
float V1_feature[2],ref_elms1[20],V2_feature[2],ref_elms2[10],descend_ratio,coef_d=0.7;
Uint16 corr_count1,corr_count2,corr_count_limit=10;
float ref_volt1_last,ref_volt2_last;
float phase_volt1_last,phase_volt2_last,phase_volt2_las1,phase_volt2_las2,inverse1,inverse2,trig1,trig2,descend_trig1,descend_trig2,in_trig1,in_trig2;
// parameters for demag diagnosis
float LD_magcomp,LD_phase_comp,LD_harm=-2;
float UD_feature[2],LD_feature[2],UD_feature_avg[2],LD_feature_avg[2],UD_feature_avg_last[2],LD_feature_avg_last[2],HI_demag[4];
float flux_energy,flux_energy_last;
Uint16 avg_count1,avg_limit1=10000,avg_load1;
float avg_test,avg_test_last;
float UD_mag,LD_mag;
IPARK mag_flux;
float F_angle,V_angle;
//----------------------------------------------
// SCI paras
//----------------------------------------------
unsigned int test_ftoint;
myfloat myfloat_test;
myuint myuint_test;
int send_en=0,FAST_enable=1,send_delay=-1;
Uint16 send_datalength=2000;
Uint32 sci_count,overtime_count,send_count;
Uint16 ReceivedChar[16],ReceivedChar2=0,device_number=100;
Uint16 crc1,crc2;
float temp_test;
Uint16 temp_data1,temp_data2;
Uint16 update_FAST_en=1;
Uint16 data_transmit_test=0;
Uint16 Rs_U16,Ls_U16,H_8bits,L_8bits ;
Uint16 test_count,cmd_rcv_time;




