/*
 * sci_tx.h
 *
 *  Created on: 2022/12/01
 *
 */

#ifndef SCI_TX_H_
#define SCI_TX_H_

#include "F2837xS_IO_assignment.h"
#include "../motorVars.h"
#include "../Emif_asram.h"


typedef union {
    float f;
    struct
    {
        /*Uint32 mantissa : 23;
        Uint32 exponent : 8;
        Uint16 sign : 1;*/
        Uint16 last : 8;
        Uint16 third : 8;
        Uint16 second : 8;
        Uint16 first : 8;
    } raw;
} myfloat;


typedef union {
    unsigned int f;
    struct
    {
        Uint16 second : 8;
        Uint16 first : 8;
    } raw;
} myuint;


//----------------------------------------------
// SCI paras
//----------------------------------------------

extern Uint16 first_and_last_data_block[8]; //first and last data package of rs485
extern Uint16 FAST_data[4]; //Fast data for transmission (only used for version1 with EMIF wrong Layout)
extern int cb_index; // pointer for circular buffer transmission

extern unsigned int test_ftoint;
extern myfloat myfloat_test;
extern myuint myuint_test;
extern int send_en,FAST_enable,send_delay;
extern Uint16 send_datalength;
extern Uint32 sci_count,send_count,Master_timeout_count;
extern Uint32 Master_timeout, Master_timeout_prescaler,Master_timeout_prescaler_cnt;
extern Uint16 ReceivedChar[16],ReceivedChar2,device_number;
extern Uint16 crc1,crc2;
extern float temp_test;
extern Uint16 temp_data1,temp_data2;
extern Uint16 update_FAST_en;
extern Uint16 data_transmit_test;
extern Uint16 Rs_U16,Ls_U16,H_8bits,L_8bits ;
extern Uint16 test_count,cmd_rcv_time;
extern int sci_send_test;
extern Uint16 sci_test_array[5];

void InitsciGpio(void);
void scia_loopback_init(void);
void scib_loopback_init(void);
void scic_loopback_init(void);
void scia_xmit(int a);
void scia_fifo_init(void);
void scib_xmit(int a);
void scib_fifo_init(void);
void scib_float(float fl);
void scic_xmit(int a);
void scic_fifo_init(void);
void scic_float(float fl);
void scia_uint(unsigned int fl);
void scib_uint(unsigned int fl);
void scic_uint(unsigned int fl);
int scib_send_FAST(Uint16 send_datalength, Uint32 *sci_count, int send_delay,Uint16 first_and_last_data_block[],Uint16 FAST_data[],Uint16 device_number);
void updateCircularBuffer(volatile Uint16 arr[], int size, int *head, Uint16 newValue);
#endif
