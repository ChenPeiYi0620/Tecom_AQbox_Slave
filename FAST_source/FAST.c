/*
 * FAST.c
 *
 *  Created on: 2024¦~10¤ë9¤é
 *      Author: MotorTech
 */
#include "FAST.h"


float lastPWM_duty(float *ref1_cap2_last,float *ref1_cap4_last,float CAP2,float CAP4){
    float PWM_duty;
    float err1=_IQabs(CAP2-CAP4);
    float err2=_IQabs(*ref1_cap2_last-CAP4);
    float err3=_IQabs(CAP2-*ref1_cap4_last);
    if (err1 <= err2 && err1 <= err3) PWM_duty= CAP2+CAP4;
    if (err2 <= err1 && err2 <= err3) PWM_duty= *ref1_cap2_last+CAP4;
    if (err3 <= err1 && err3 <= err2) PWM_duty= CAP2+*ref1_cap4_last;
    *ref1_cap2_last=CAP2;
    *ref1_cap4_last=CAP4;
    return PWM_duty;
}

void LinetoPhase(_iq Line_A, _iq Line_B, CLARKE *phase_volt)
{
     phase_volt->As= _IQmpy(_IQ(0.333333),(_IQmpy(_IQ(2),Line_A)+Line_B));
     phase_volt->Bs = _IQmpy(_IQ(0.333333),(-Line_A+Line_B));

}

void EMFtoLPF(float samplingTime, float LPF_radius, EMF *EMF)
{
    float LPF_radius_T;
    float EMF_coef1,EMF_coef2;
    LPF_radius_T = LPF_radius * samplingTime;
    EMF_coef1 = samplingTime/(LPF_radius_T+2);
    EMF_coef2 = (LPF_radius_T-2)/(LPF_radius_T+2);

    EMF->Alpha_LPF = EMF_coef1*(EMF->Alpha_last + EMF->Alpha) - EMF_coef2 * EMF->Alpha_LPF_last;
    EMF->Beta_LPF  = EMF_coef1*(EMF->Beta_last + EMF->Beta) - EMF_coef2 * EMF->Beta_LPF_last;

    EMF->Alpha_last = EMF->Alpha;
    EMF->Beta_last  = EMF->Beta;

    EMF->Alpha_LPF_last = EMF->Alpha_LPF;
    EMF->Beta_LPF_last = EMF->Beta_LPF;
}

void Flux_comp(_iq omega_e, _iq LPF_radius, _iq *mag_comp, _iq *phase_comp)
{
    if(_IQabs(omega_e)<_IQ(1)) *mag_comp = _IQ(1);
    else *mag_comp = _IQdiv(_IQmag(omega_e,_IQ(LPF_radius)),_IQmag(omega_e,0));

    *phase_comp = -_IQmpy(_IQ(57.29578),_IQatan2(_IQ(LPF_radius),omega_e));
    *phase_comp = _IQdiv(*phase_comp,_IQ(360)); //degree to radius and change 0~1 phase*PI/180/2PI
}

void Flux_CurModel(_iq Ld, _iq Lq, _iq Lamda_m, _iq idr, _iq iqr, IPARK *Flux_Cur)
{
    Flux_Cur->Ds = _IQmpy(Ld,idr) + Lamda_m;
    Flux_Cur->Qs = _IQmpy(Lq,iqr);
}

void Com_fluxobserver(Comflux *comflux, float samplingTime)
{
    comflux->LamdaCur_d = _IQmpy(comflux->d_outlast,samplingTime) + comflux->LamdaCur_dlast;
    comflux->LamdaCur_q = _IQmpy(comflux->q_outlast,samplingTime) + comflux->LamdaCur_qlast;

    comflux->Lamda_df = comflux->LamdaCur_d + comflux->LamdaVolt_d;
    comflux->Lamda_qf = comflux->LamdaCur_q + comflux->LamdaVolt_q;

    comflux->d_outlast = comflux->d_out;
    comflux->q_outlast = comflux->q_out;

    comflux->LamdaCur_dlast = comflux->LamdaCur_d;
    comflux->LamdaCur_qlast = comflux->LamdaCur_q;
}


void Vpol_feature(_iq *featureval,_iq *ref_elms, _iq NewVal,_iq *lastVal, Uint16 *count, Uint16 n){
// record n last value of ref
        if (*count<n) *count=*count+1;
        else {
            *count=0;
            *featureval=*lastVal;
            *lastVal=NewVal;   }
// find local maximum from a set of ref_volt
    _iq max=-10000;
        int i;
//    int elm_length=sizeof(ref_elms)/2;
    int midindex=n/2;
    // Update ref matrix for finding maximum
    for (i=0;i<n-1;i++){
        if (*(ref_elms+i)>max) max=*(ref_elms+i);
        *(ref_elms+i)=*(ref_elms+i+1);
    }
    *(ref_elms+n-1)=NewVal;
// find local maximum
    if (*(ref_elms+n-1)> *ref_elms && *(ref_elms+midindex)> *(ref_elms+n-1)) *(featureval+1)=max;
}

void Vpol_calib(_iq ref_volt, _iq *phase_volt,_iq *phase_volt_last,_iq featureval[], _iq *inverse,_iq *trig, _iq *descend_trig,_iq *in_trig,_iq descend_ratio)
{
//    featureval[0] is the n step last value
//    featureval[1] is the local max value
//    compute adequate ratio of local max for descendent detection
       if (ref_volt==featureval[0]) *trig=0;

       if (_IQabs(ref_volt-featureval[0])>_IQ(0.001) & *trig==0){
              *trig=1;
              *descend_trig=0;
              *in_trig=1;       }

       if (ref_volt+featureval[1]*descend_ratio<featureval[0] && *descend_trig==0) *descend_trig=1;

       if (*descend_trig==1 && *phase_volt>*phase_volt_last+_IQ(0.0001) && *in_trig==1) *in_trig=0;

       if (*in_trig==0) *inverse=-1;
       else  *inverse=1;

       *phase_volt_last=*phase_volt;
       *phase_volt=*phase_volt**inverse;
}


// this is te old pole calibration data

void Vpol_calib_origin(_iq ref,_iq *phase,_iq wehz,_iq capturecount1,_iq capturecount2,_iq capturecountthreshold ){
    _iq modify1;
//    _iq coef;
    if(wehz<50){
        modify1 = _IQ(0.02);
//        coef = 0.2;
        }
    else if(wehz>=50 && wehz<150){
        modify1 = _IQ(0.05);
//        coef = 0.7;
        }
    else{
        modify1 = _IQ(0.08);
//        coef = 0.9;
        }

//    polar correction
    if(capturecount1>capturecount2+capturecountthreshold && ref<modify1) *phase = -*phase;

//        else phase = -phase;
//    if(capturecount6>capturecount5+capturecountthreshold3 && ref_volt1<modify1) phase_volt3 = -phase_volt3;
//    else phase_volt3=ref_volt3;
//    if(capturecount8>capturecount7+capturecountthreshold4 && ref_volt2<modify2) phase_volt4 = -phase_volt4;
//    else phase_volt4=ref_volt4;
    }
void inverse(_iq *input){*input=*input*-1;}


void SFF(_iq SFFA, _iq SFFB,_iq SFFC, _iq *SFF_in, _iq *SSF_out){
//    bilinear band pass filter
//    SSF[0] ->current data, SFF[1]->last data, SFF[2] last 2 step data
    _iq denum=SFFA+SFFB+SFFC;
    *SSF_out=SFFB/(denum)*(*SFF_in-*(SFF_in+2))-2*(SFFC-SFFA)/(denum)*(*(SSF_out+1))-(SFFA-SFFB+SFFC)/(denum)*(*(SSF_out+2));
//update data
    *(SFF_in+2)= *(SFF_in+1);*(SFF_in+1)= *(SFF_in);
    *(SSF_out+2)= *(SSF_out+1);*(SSF_out+1)= *(SSF_out);
}
//void SFF_FAST(_iq wtr_theta,_iq *SFF_in, _iq *SFF_out, _iq epsilon, _iq T){
//    _iq e1out,e2out;
//    // use SFF_in[0] to store raw input
//    // use SFF_in[1] to store e1_in
//    // use SFF_out[0] to store u_out
//    // use SFF_out[1] to store e2_in
//    *(SFF_in+1)=(*SFF_in-*SFF_out)*epsilon*_IQsin(wtr_theta);//e1
//    *(SFF_out+1)=(*SFF_in-*SFF_out)*epsilon*_IQcos(wtr_theta);//e1
//    // calculate uout
//    e1out=T*(*(SFF_in+2))+*(SFF_in+3);
//    e2out=T*(*(SFF_out+2))+*(SFF_out+3);
//    // e1_out=(Tsz^-1/1-z^-1)e1_out
//    // u_out= e1_out*sin(wrt)+e2_out*cos(wrt)
//    *SFF_out=e1out*_IQsin(wtr_theta)+e2out*_IQcos(wtr_theta);
//    // Update data
//    // use SFF_in[2] to store e1_last
//    // use SFF_in[3] to store e1_out_last
//    *(SFF_in+2)=*(SFF_in+1);
//    *(SFF_in+3)=e1out;
//    // use SFF_out[2] to store e2_last
//    // use SFF_out[3] to store e2_out_last
//    *(SFF_out+2)=*(SFF_out+1);
//    *(SFF_out+3)=e2out;
//
//}



