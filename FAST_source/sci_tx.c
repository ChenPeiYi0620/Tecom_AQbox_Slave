/*
 * sci_tx.c
 *
 *  Created on: 2024¦~10¤ë9¤é
 *      Author: MotorTech
 */

#include "sci_tx.h"

Uint16 first_and_last_data_block[8]; //first and last data package of rs485
Uint16 FAST_data[4]; //Fast data for transmission (only used for version1 with EMIF wrong Layout)
int cb_index; // pointer for circular buffer transmission

void InitsciGpio(void)
{
    //28388 scib setting
            //SCIB_TX GPIO setup
            GPIO_SetupPinMux(10, GPIO_MUX_CPU1, 6);
            GPIO_SetupPinOptions(10, GPIO_OUTPUT, GPIO_ASYNC);
            //SCIB_RX GPIO setup
            GPIO_SetupPinMux(11, GPIO_MUX_CPU1, 6);
            GPIO_SetupPinOptions(11, GPIO_INPUT, GPIO_PUSHPULL);
            // setup tx_en
            GPIO_SetupPinMux(TX_EN_GPIO, GPIO_MUX_CPU1, TX_EN_MUX);
            GPIO_SetupPinOptions(TX_EN_GPIO, GPIO_OUTPUT, GPIO_PUSHPULL);
            // Initially disable TX
            GPIO_WritePin(TX_EN_GPIO,0);
}
void scia_loopback_init(void)
{
    //
    // Note: Clocks were turned on to the SCIA peripheral
    // in the InitSysCtrl() function
    //

    SciaRegs.SCICCR.all = 0x0007;  // 1 stop bit,  No loopback
                                   // No parity,8 char bits,
                                   // async mode, idle-line protocol
    SciaRegs.SCICTL1.all = 0x0003; // enable TX, RX, internal SCICLK,
                                   // Disable RX ERR, SLEEP, TXWAKE
    SciaRegs.SCICTL2.all = 0x0003;
    SciaRegs.SCICTL2.bit.TXINTENA = 1;
    SciaRegs.SCICTL2.bit.RXBKINTENA = 1;
    // @LSPCLK = 25 MHz (200 MHz SYSCLK) HBAUD = 0x01 and LBAUD = 0x45.
    // @LSPCLK = 50 MHz (200 MHz SYSCLK) HBAUD = 0x02 and LBAUD = 0x8B.
    // @LSPCLK = 100 MHz (200 MHz SYSCLK) HBAUD = 0x05 and LBAUD = 0x15.
    SciaRegs.SCIHBAUD.all = 0x0000;
    SciaRegs.SCILBAUD.all = 0x0006;
    SciaRegs.SCICCR.bit.LOOPBKENA = 0; // Disable loop back
    SciaRegs.SCICTL1.all = 0x0023;     // Relinquish SCI from Reset
}
void scib_loopback_init(void)
{
    //
    // Note: Clocks were turned on to the SCIA peripheral
    // in the InitSysCtrl() function
    //

    ScibRegs.SCICCR.all = 0x0007;  // 1 stop bit,  No loopback
                                   // No parity,8 char bits,
                                   // async mode, idle-line protocol

    ScibRegs.SCICCR.bit.STOPBITS =0x0; // 1 stop bit
    ScibRegs.SCICCR.bit.PARITYENA=0x0; // disable parity
    ScibRegs.SCICCR.bit.PARITY=0x1; // even parity

    ScibRegs.SCICTL1.all = 0x0003; // enable TX, RX, internal SCICLK,
                                   // Disable RX ERR, SLEEP, TXWAKE
    ScibRegs.SCICTL2.bit.TXINTENA = 0;
    ScibRegs.SCICTL2.bit.RXBKINTENA = 0;

    // @LSPCLK = 25 MHz (200 MHz SYSCLK) HBAUD = 0x01 and LBAUD = 0x45.
    // @LSPCLK = 50 MHz (200 MHz SYSCLK) HBAUD = 0x02 and LBAUD = 0x8B.
    // @LSPCLK = 100 MHz (200 MHz SYSCLK) HBAUD = 0x05 and LBAUD = 0x15.
    ScibRegs.SCIHBAUD.all = 0x0000;
    ScibRegs.SCILBAUD.all = 0x000D;//~921600 bps @100MHz
//    ScibRegs.SCILBAUD.all = 0x0013;//61000 bps @100MHz
//    ScibRegs.SCILBAUD.all = 0x000A;//576000 bps @100MHz
//
//    ScibRegs.SCIHBAUD.all = 0x0002;
//    ScibRegs.SCILBAUD.all = 0x008A;//9600bps

    ScibRegs.SCICCR.bit.LOOPBKENA = 0; // Disable  loop back

}
void scic_loopback_init(void)
{
    //
    // Note: Clocks were turned on to the SCIA peripheral
    // in the InitSysCtrl() function
    //

    ScicRegs.SCICCR.all = 0x0007;  // 1 stop bit,  No loopback
                                   // No parity,8 char bits,
                                   // async mode, idle-line protocol

    ScicRegs.SCICCR.bit.STOPBITS =0x0; // 1 stop bit for communication stability
//    ScicRegs.SCICCR.bit.STOPBITS =0x1; // 2 stop bit for communication stability

    ScicRegs.SCICTL1.all = 0x0003; // enable TX, RX, internal SCICLK,
                                   // Disable RX ERR, SLEEP, TXWAKE
    ScicRegs.SCICTL2.all = 0x0003;
    ScicRegs.SCICTL2.bit.TXINTENA = 0; //disable TX interrupt
    ScicRegs.SCICTL2.bit.RXBKINTENA = 1;
    // @LSPCLK = 25 MHz (200 MHz SYSCLK) HBAUD = 0x01 and LBAUD = 0x45.
    // @LSPCLK = 50 MHz (200 MHz SYSCLK) HBAUD = 0x02 and LBAUD = 0x8B.
    // @LSPCLK = 100 MHz (200 MHz SYSCLK) HBAUD = 0x05 and LBAUD = 0x15.
    ScicRegs.SCIHBAUD.all = 0x0000;
    ScicRegs.SCILBAUD.all = 0x0013; // 616000 baud @100MHz
    ScicRegs.SCICCR.bit.LOOPBKENA = 0; // Enable loop back
    ScicRegs.SCICTL1.all = 0x0023;     // Relinquish SCI from Reset
}

//
// scia_xmit - Transmit a character from the SCI
//
void scia_xmit(int a)
{
    while (SciaRegs.SCIFFTX.bit.TXFFST != 0) {}
    SciaRegs.SCITXBUF.all=a;
}

//
// scia_fifo_init - Initialize the SCI FIFO
//
void scia_fifo_init(void)
{
    SciaRegs.SCIFFTX.all = 0xE040;
    SciaRegs.SCIFFRX.all = 0x2044;
    SciaRegs.SCIFFCT.all = 0x0;
}
//
// scic_xmit - Transmit a character from the SCIc
//
void scib_xmit(int a)
{
    while (ScibRegs.SCIFFTX.bit.TXFFST != 0) {}
//    ScibRegs.SCICTL1.bit.TXWAKE=1;
    ScibRegs.SCITXBUF.all=a;
}

//
// scia_fifo_init - Initialize the SCI FIFO
//
void scib_fifo_init(void)
{
    ScibRegs.SCIFFTX.all = 0xE040; // tx interrupt disable
    ScibRegs.SCIFFRX.all = 0x0021; // 4-word FIFO triggering
    ScibRegs.SCIFFCT.all = 0x0;
    ScibRegs.SCIFFTX.bit.TXFIFORESET = 1;
    ScibRegs.SCIFFRX.bit.RXFIFORESET = 1;
    ScibRegs.SCICTL1.all = 0x0023;     // Relinquish SCI from Reset
}

void scib_float(float fl)
{
    myfloat v;
    v.f=fl;
    scib_xmit( v.raw.last);
    scib_xmit( v.raw.third);
    scib_xmit( v.raw.second);
    scib_xmit( v.raw.first);
}

//
// scic_xmit - Transmit a character from the SCIc
//
void scic_xmit(int a)
{
    while (ScicRegs.SCIFFTX.bit.TXFFST != 0) {}
    ScicRegs.SCICTL1.bit.TXWAKE=1;
    ScicRegs.SCITXBUF.all=a;
}

//
// scia_fifo_init - Initialize the SCI FIFO
//
void scic_fifo_init(void)
{
    ScicRegs.SCIFFTX.all = 0xE040;
    ScicRegs.SCIFFRX.all = 0x2044;
    ScicRegs.SCIFFCT.all = 0x0;
}
void scic_float(float fl)
{
    myfloat v;
    v.f=fl;
    scia_xmit( v.raw.last);
    scia_xmit( v.raw.third);
    scia_xmit( v.raw.second);
    scia_xmit( v.raw.first);
}

void scia_uint(unsigned int fl)
{
    myuint v;
    v.f=fl;
    scia_xmit( v.raw.second);
    scia_xmit( v.raw.first);


}
void scib_uint(unsigned int fl)
{
    myuint v;
    v.f=fl;
    scib_xmit( v.raw.second);
    scib_xmit( v.raw.first);


}

void scic_uint(unsigned int fl)
{
    myuint v;
    v.f=fl;

    scic_xmit( v.raw.second);
    scic_xmit( v.raw.first);


}

int scib_send_FAST(
        Uint16 send_datalength,Uint32 *sci_count, int send_delay, Uint16 first_and_last_data_block[],Uint16 FAST_data[],Uint16 device_number){

    int send_count=0; // for debug use
    int send_en=1;
    int asaram_head=0;


    // first package, send the motor status
    if (*sci_count==0){
        scib_xmit(2);                               // cmd code: from slave
        scib_xmit(device_number);                   // cmd code: device number to avoid conflict
        scib_uint(first_and_last_data_block[0]);
        scib_uint(first_and_last_data_block[1]);    //Torque, Torq_est_avg*32768+32768
        scib_uint(first_and_last_data_block[2]);    //Power1, Power1*32768+32768
        scib_uint(first_and_last_data_block[3]);    //efficiency, efficiency*32768+32768
        while(ScibRegs.SCIFFTX.bit.TXFFST != 0) {}  // wait until TXFF buffer clear
        while(ScibRegs.SCICTL2.bit.TXEMPTY != 1) {} // wait until transmit complete
        *sci_count=*sci_count+1;
        send_count+=4;
    }

    // Second package, send the diagnosis result
    else if (*sci_count==1){
        // send the data
        scib_xmit(2);                               // cmd code: from slave
        scib_xmit(device_number);                   // cmd code: device number to avoid conflict
        scib_uint(first_and_last_data_block[4]); // PM flux magnitude
        scib_uint(first_and_last_data_block[5]); // short circuit index_X
        scib_uint(first_and_last_data_block[6]); // short circuit index_Y
        scib_uint(0);                            // Idle data
        while(ScibRegs.SCIFFTX.bit.TXFFST != 0) {}  // wait until TXFF buffer clear
        while(ScibRegs.SCICTL2.bit.TXEMPTY != 1) {} // wait until transmit complete
        *sci_count=*sci_count+1;
        send_count+=3;
    }
    // signal packages
    else if (*sci_count>=2 && *sci_count< send_datalength+2){
        scib_xmit(2);                               // cmd code: from slave
        scib_xmit(device_number);                   // cmd code: device number to avoid conflict
        // (continue) send acquisition data
        //        scib_uint(AsramData.data1[asaram_head]);
        //        scib_uint(AsramData.data1[asaram_head]);    //Torque, Torq_est_avg*32768+32768
        //        scib_uint(AsramData.data1[asaram_head]);    //Power1, Power1*32768+32768
        //        scib_uint(AsramData.data1[asaram_head]);    //efficiency, efficiency*32768+32768
        scib_uint(FAST_data[0]);
        scib_uint(FAST_data[1]);    //Torque, Torq_est_avg*32768+32768
        scib_uint(FAST_data[2]);    //Power1, Power1*32768+32768
        scib_uint(FAST_data[3]);    //efficiency, efficiency*32768+32768
        while(ScibRegs.SCIFFTX.bit.TXFFST != 0) {}  // wait until TXFF buffer clear
        while(ScibRegs.SCICTL2.bit.TXEMPTY != 1) {} // wait until transmit complete

        asaram_head+=2;
        *sci_count=*sci_count+1;
        send_count+=4;
    }

    else  {
        scib_xmit(2);                               // cmd code: from slave
        scib_xmit(device_number);                   // cmd code: device number to avoid conflict
        // send crc
        scib_uint(first_and_last_data_block[7]);
        scib_uint(0); //idle data
        scib_uint(0); //idle data
        scib_uint(0); //idle data
        while(ScibRegs.SCIFFTX.bit.TXFFST != 0) {}  // wait until TXFF buffer clear
        while(ScibRegs.SCICTL2.bit.TXEMPTY != 1) {} // wait until transmit complete
        // data send complete, disable sending
        send_en=0;*sci_count=0;send_count+=1;
        // Disable TX
        GPIO_WritePin(TX_EN_GPIO,0);
    }
    return send_en;
}

void updateCircularBuffer(volatile Uint16 arr[], int size, int *head, Uint16 newValue) {
    arr[*head] = newValue;  //update data at head address
//    *head = (*head + 1) % size;  // update head address, circular the buffer
    *head = (*head + 2) % size;  // for EMIF wrong layout only
}


