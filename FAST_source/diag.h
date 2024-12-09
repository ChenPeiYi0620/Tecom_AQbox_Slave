//#define   MATH_TYPE      1
//#include "IQmathLib.h"
//#include "F28x_Project.h"
//#include "motorVars.h"
//#include "config.h"


#ifndef DIAG_H_
#define DIAG_H_
#include "../motorVars.h"

void LD_2nd_comp(_iq omega_e, _iq LPF_radius, _iq *mag_comp, _iq *phase_comp, _iq LD_harm);
_iq angle_est(_iq tan_a, _iq tan_b, _iq cFangle);
IPARK MAG_FLUX(_iq total_flux_alpha, _iq total_flux_beta, _iq iq, _iq id, _iq Lq, _iq Ld, _iq Fangle);
void average(_iq *curr_avg, _iq *last_avg, _iq newVar, Uint16 avg_load, _iq numerator);
void DEMAG_Feature(_iq *UD_feature, _iq *LD_feature, IPARK mag_flux, _iq LD_harm, _iq Fangle, _iq comp_phase);
void HI_DEMAG(_iq *HI_Demag, _iq lumda_h, _iq UDmag, _iq LDmag, _iq noise);

#endif
