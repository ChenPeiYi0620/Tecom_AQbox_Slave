/*
 * diag.c
 *
 *  Created on: 2024¦~10¤ë9¤é
 *      Author: MotorTech
 */

#include "diag.h"

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

