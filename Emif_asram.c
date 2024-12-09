/*
 * Emif_asram.c
 *
 *  Created on: 2024¦~10¤ë9¤é
 *      Author: MotorTech
 */

#include "Emif_asram.h"

#ifdef __cplusplus
#pragma DATA_SECTION("Asaram_data_reg")
#else
#pragma DATA_SECTION(AsramData,"Asaram_data_reg");
#endif

int head1,head2,head3,head4,head5,head6;

volatile struct ASRAM_data AsramData;


