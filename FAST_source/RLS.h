/*
 * FAST.h
 *
 *  Created on: 2021/12/01
 *
 */
typedef struct {  _iq  y;       // Input: stationary d-axis stator variable
                  _iq  phi;        // Input: stationary q-axis stator variable
                  _iq  err;       // Input: rotating angle (pu)
                  _iq  M1;          // Output: rotating d-axis stator variable
                  _iq  M2;          // Output: rotating q-axis stator variable
                  _iq  K;
                  _iq  Ra;
                  _iq  Pl;
                  _iq  Pl_last;
                  _iq  forgetting_coef;
                  _iq  forgetting_coefinv;
                  _iq  parameter;
                  _iq  parameter_last;
                } RLS;

/*-----------------------------------------------------------------------------
Default initalizer for the PARK object.
-----------------------------------------------------------------------------*/
#define RLS_DEFAULTS {    0, \
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
                          0, \
                          }

void RLS_cal(RLS* Parameter)
{
    Parameter->err = Parameter->y - _IQmpy(Parameter->phi,Parameter->parameter_last);
    Parameter->M1 = _IQmpy(Parameter->Pl_last,Parameter->phi);
    Parameter->M2 = _IQdiv(_IQ(1), Parameter->forgetting_coef + _IQmpy(Parameter->phi,_IQmpy(Parameter->Pl_last,Parameter->phi)));
    Parameter->K = _IQmpy(Parameter->M1,Parameter->M2);
    Parameter->Ra = _IQmpy(_IQ(1) - _IQmpy(Parameter->K,Parameter->phi),Parameter->Pl_last);
    Parameter->Pl = _IQmpy(Parameter->forgetting_coefinv,Parameter->Ra);
//    Parameter->parameter = Parameter->parameter_last + _IQmpy(Parameter->K,Parameter->err);
//    Parameter->Pl_last = Parameter->Pl;
//    Parameter->parameter_last = Parameter->parameter;
}
