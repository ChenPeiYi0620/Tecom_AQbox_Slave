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
#define FAST_EMIF_length 2000

struct ASRAM_data{
    Uint16 data1[MAX_EMIF_length];
    Uint16 data2[MAX_EMIF_length];
    Uint16 data3[MAX_EMIF_length];
    Uint16 data4[MAX_EMIF_length];
    Uint16 data5[FAST_EMIF_length]; // FAST data storage
    Uint16 data6[FAST_EMIF_length]; // FAST data storage

//    // double buffer to stabalize the  sci  transmit
//    Uint16 data1_sci[MAX_EMIF_length];
//    Uint16 data2_sci[MAX_EMIF_length];
//    Uint16 data3_sci[MAX_EMIF_length];
//    Uint16 data4_sci[MAX_EMIF_length];
//    Uint16 data5_sci[FAST_EMIF_length]; // FAST data storage
//    Uint16 data6_sci[FAST_EMIF_length]; // FAST data storage


};
extern int head1,head2,head3,head4,head5,head6;

extern volatile struct ASRAM_data AsramData;



#endif /* EMIF_ASRAM_H_ */
