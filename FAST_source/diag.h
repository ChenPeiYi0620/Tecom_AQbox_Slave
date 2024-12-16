//#define   MATH_TYPE      1
//#include "IQmathLib.h"
//#include "F28x_Project.h"
//#include "motorVars.h"
//#include "config.h"


#ifndef DIAG_H_
#define DIAG_H_
#include "../motorVars.h"

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
// parameters for demag diagnosis
extern float LD_magcomp,LD_phase_comp,LD_harm;
extern float UD_feature[2],LD_feature[2],UD_feature_avg[2],LD_feature_avg[2],UD_feature_avg_last[2],LD_feature_avg_last[2],HI_demag[4];
extern float flux_energy,flux_energy_last;
extern Uint16 avg_count1,avg_limit1,avg_load1;
extern float avg_test,avg_test_last,rms_test,rms_last;
extern float UD_mag,LD_mag;
extern IPARK mag_flux;
extern float F_angle,V_angle;

// winding fault parameters
//----------------------------------------------
extern PARK flux_cn,park_cn;
extern float fluxDs_cn_lpf;    // fault signal only pass low pass filter
extern float fluxQs_cn_lpf;
extern float parkDs_cn_lpf;    // fault signal only pass low pass filter
extern float parkQs_cn_lpf;
extern float wc, wc1, wc2;     // low pass filter frequency parameter
extern float k, k1, k2;        // low pass filter gain
extern float fluxDs_cn_sum;    // fault signal summation from phase current
extern float fluxQs_cn_sum;
extern float fluxDs_cn_avg;    // fault signal average form phase current
extern float fluxQs_cn_avg;
extern float fluxdq_cn_avg, fluxdq_cn_sum;
extern float parkDs_cn_sum;    // fault signal summation from phase current
extern float parkQs_cn_sum;
extern float parkDs_cn_avg;    // fault signal average form phase current
extern float parkQs_cn_avg;
extern float parkdq_cn_avg, parkdq_cn_sum;
extern float avg_count_cn, avg_limit_cn, avg_load1_cn;
extern float cn_err_code, cn_valve, current_mag_avg, current_mag_sum;


void LD_2nd_comp(_iq omega_e, _iq LPF_radius, _iq *mag_comp, _iq *phase_comp, _iq LD_harm);
_iq angle_est(_iq tan_a, _iq tan_b, _iq cFangle);
IPARK MAG_FLUX(_iq total_flux_alpha, _iq total_flux_beta, _iq iq, _iq id, _iq Lq, _iq Ld, _iq Fangle);
void average(_iq *curr_avg, _iq *last_avg, _iq newVar, Uint16 avg_load, _iq numerator);
void rms(_iq *curr_rms, _iq *last_rms, _iq newVar,Uint16 avg_load,_iq numerator);
void DEMAG_Feature(_iq *UD_feature, _iq *LD_feature, IPARK mag_flux, _iq LD_harm, _iq Fangle, _iq comp_phase);
void HI_DEMAG(_iq *HI_Demag, _iq lumda_h, _iq UDmag, _iq LDmag, _iq noise);
void CN_Fault(_iq *cn_err_code, _iq cn_valve, _iq current_mag, _iq cn_D,_iq cn_Q,_iq cn_mag );

#endif
