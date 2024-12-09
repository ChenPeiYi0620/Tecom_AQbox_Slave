/*
 * Emif_asram.h
 *
 *  Created on: 2024¦~10¤ë9¤é
 *      Author: MotorTech
 */

#ifndef EMIF_ASRAM_H_
#define EMIF_ASRAM_H_
#include "motorVars.h"

#define MAX_EMIF_length 10000

struct ASRAM_data{
    Uint16 data1[MAX_EMIF_length];
    Uint16 data2[MAX_EMIF_length];
    Uint16 data3[MAX_EMIF_length];
    Uint16 data4[MAX_EMIF_length];
    Uint16 data5[MAX_EMIF_length];
    Uint16 data6[MAX_EMIF_length];
};
extern int head1,head2,head3,head4,head5,head6;

extern volatile struct ASRAM_data AsramData;



#endif /* EMIF_ASRAM_H_ */
