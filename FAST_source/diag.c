/*
 * diag.c
 *
 *  Created on: 2024¦~10¤ë9¤é
 *      Author: MotorTech
 */

#include "diag.h"

#define deg60 PI/3 // for winding fault diagnosis

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
// parameters for demag diagnosis
float LD_magcomp,LD_phase_comp,LD_harm=-2;
float UD_feature[2],LD_feature[2],UD_feature_avg[2],LD_feature_avg[2],UD_feature_avg_last[2],LD_feature_avg_last[2],HI_demag[4];
float flux_energy,flux_energy_last;
Uint16 avg_count1,avg_limit1=10000,avg_load1;
float avg_test,avg_test_last,rms_test,rms_last;
float UD_mag,LD_mag;
IPARK mag_flux;
float F_angle,V_angle;

// winding fault parameters
//----------------------------------------------
PARK flux_cn,park_cn;
float fluxDs_cn_lpf = 0;   // fault signal only pass low pass filter
float fluxQs_cn_lpf = 0;
float parkDs_cn_lpf = 0;   // fault signal only pass low pass filter
float parkQs_cn_lpf = 0;
float wc=1, wc1 = 0.01, wc2 = 0.01;   //low pass filter frequency parameter
float k=1 , k1 = 1 , k2 = 1;          //low pass filter gain
float fluxDs_cn_sum = 0;  // fault signal summation from phase current
float fluxQs_cn_sum = 0;
float fluxDs_cn_avg = 0;  // fault signal average form phase current
float fluxQs_cn_avg = 0;
float fluxdq_cn_avg = _IQ(0),fluxdq_cn_sum = _IQ(0);
float parkDs_cn_sum = 0;  // fault signal summation from phase current
float parkQs_cn_sum = 0;
float parkDs_cn_avg = 0;  // fault signal average form phase current
float parkQs_cn_avg = 0;
float parkdq_cn_avg = _IQ(0),parkdq_cn_sum = _IQ(0);
float avg_count_cn = 0,avg_limit_cn=10000,avg_load1_cn=0;
float cn_err_code=0, cn_valve=0.05, current_mag_avg=0,current_mag_sum=0;


void LD_2nd_comp(_iq omega_e, _iq LPF_radius, _iq *mag_comp,_iq *phase_comp,_iq LD_harm)
{
    if(_IQabs(omega_e)<_IQ(1)) *mag_comp = _IQ(1);
    else *mag_comp=_IQdiv(_IQmag(LD_harm*omega_e,LPF_radius),_IQmpy(_IQabs(LD_harm),_IQmag(omega_e,LPF_radius)));

    *phase_comp = _IQdiv(_IQ(LPF_radius),_IQmpy(LD_harm,omega_e));
    *phase_comp = -_IQmpy(_IQ(57.29578),_IQatan2(*phase_comp,_IQ(1)));//57.29578=180/pi
    *phase_comp = _IQdiv(*phase_comp,_IQ(360)); //degree to radius and change 0~1 phase*PI/180/2PI
}

_iq  angle_est(_iq tan_a, _iq tan_b, _iq cFangle){
    _iq  F_angle=0;
    F_angle = _IQdiv(_IQatan2(tan_b,tan_a),two_PI)+cFangle;
    if(F_angle<_IQ(0.0)) F_angle+=_IQ(1.0);
    if(F_angle>_IQ(1.0)) F_angle-=_IQ(1.0);
    return F_angle;}

    IPARK MAG_FLUX(_iq total_flux_alpha, _iq total_flux_beta, _iq iq, _iq id, _iq Lq, _iq Ld ,_iq Fangle){
        // transform total flux to dq
        PARK total_flux;
        total_flux.Alpha=total_flux_alpha;
        total_flux.Beta=total_flux_beta;
        total_flux.Angle=Fangle;
        total_flux.Sine   = __sinpuf32(total_flux.Angle);
        total_flux.Cosine = __cospuf32(total_flux.Angle);
        PARK_MACRO(total_flux)
        // decouple magnet flux to dq
        IPARK mag_flux;
        mag_flux.Ds = total_flux.Ds-_IQmpy(Ld,id);//-_IQmpy(0,idr_diff);
        mag_flux.Qs = total_flux.Qs-_IQmpy(Lq,iq);//-_IQmpy(0,iqr_diff);
        mag_flux.Angle = Fangle;
        mag_flux.Sine   = __sinpuf32(mag_flux.Angle);
        mag_flux.Cosine = __cospuf32(mag_flux.Angle);
        IPARK_MACRO(mag_flux)
    return mag_flux ;}

void average(_iq *curr_avg, _iq *last_avg, _iq newVar,Uint16 avg_load,_iq numerator){
    if (avg_load==0)
    {*curr_avg=*curr_avg+ newVar;}///(float)count_limit
    else {*last_avg=*curr_avg/numerator;
    *curr_avg=_IQ(0);
    }}

void rms(_iq *curr_rms, _iq *last_rms, _iq newVar,Uint16 avg_load,_iq numerator){
    if (avg_load==0)
    {*curr_rms=*curr_rms+ newVar*newVar;}///(float)count_limit
    else {*last_rms=sqrtf(*curr_rms/numerator);
    *curr_rms=_IQ(0);
    }}

void DEMAG_Feature(_iq *UD_feature,_iq *LD_feature, IPARK mag_flux,_iq LD_harm, _iq Fangle, _iq comp_phase){
    //UD_feature[0][1] is main frequency alpha&beta
    //LD_feature[0][1] is LD frequency alpha&beta
    *UD_feature=_IQmpy( mag_flux.Alpha,__cospuf32(-Fangle))-_IQmpy(mag_flux.Beta, __sinpuf32(-Fangle));
    *(UD_feature+1)=_IQmpy( mag_flux.Beta,__cospuf32(-Fangle)) + _IQmpy(mag_flux.Alpha, __sinpuf32(-Fangle));

    _iq ld_angle=-LD_harm*Fangle+comp_phase;
    *LD_feature=_IQmpy( mag_flux.Alpha,__cospuf32(ld_angle)) -_IQmpy(mag_flux.Beta, __sinpuf32(ld_angle));
    *(LD_feature+1)=_IQmpy( mag_flux.Beta,__cospuf32(ld_angle)) +_IQmpy(mag_flux.Alpha, __sinpuf32(ld_angle));
}

void HI_DEMAG(_iq *HI_Demag, _iq lumda_h, _iq UDmag , _iq LDmag, _iq noise){

// for HI construction
//    HI[0] is UD index
//    HI[1] is UD flag
//    HI[2] is LD index
//    HI[3] is LD flag

*HI_Demag=UDmag/lumda_h;
    if (*HI_Demag >=0.9) {
        *HI_Demag=1;
        *(HI_Demag+1)=0; }
    else if (*HI_Demag<0){
        *HI_Demag=1;
        *(HI_Demag+1)=1;}
    else *(HI_Demag+1)=1;

    *(HI_Demag+2)=(LDmag-noise)/(lumda_h-noise);
    if (*(HI_Demag+2) <=0.03) *(HI_Demag+3)=0;
    else *(HI_Demag+3)=1;
}
void CN_Fault(_iq *cn_err_code, _iq cn_valve, _iq current_mag, _iq cn_D,_iq cn_Q,_iq cn_mag ){
    if (cn_mag/current_mag < cn_valve){
        // if cn_mag is small, no short fault alarm
        cn_err_code=0;  }
    else {
    float  neg_freq_phase=_IQatan2(cn_Q,cn_D);
        if (fabs(neg_freq_phase)<deg60)
            *cn_err_code=1;//phase A fault if phase between -60 to 60 deg
        else if (neg_freq_phase>deg60)
            *cn_err_code=3;//phase C fault if phase between 60 to 180 deg
        else
            *cn_err_code=2;//phase B fault if phase between 180 to -60 deg
    }
}
