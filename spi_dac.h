

//#define SINE_WAVE_VERTICAL_RESOLUTION 63  // (2^N-1)
//unsigned int DAC_RESOLUTION_in_LSB = 4096;			// (2^N)
//unsigned int SINE_DATA[SINE_WAVE_VERTICAL_RESOLUTION];
//
//float multiper;
//
//void SINE_WAVE_INIT(void)
//{
//	int i;
//
//	for(i=1;i<=SINE_WAVE_VERTICAL_RESOLUTION;i++)
//		SINE_DATA[i-1] = (sin(i/SINE_WAVE_VERTICAL_RESOLUTION)+1)*(DAC_RESOLUTION_in_LSB/2);
//
//}




// SPIDAC
void SPIDAC_setup()
{

  #define SPIDAC_SET_LDAC		GpioDataRegs.GPBSET.bit.GPIO61   = 1
  #define SPIDAC_CLEAR_LDAC		GpioDataRegs.GPBCLEAR.bit.GPIO61 = 0

//   SysCtrlRegs.PCLKCR0.bit.SPIAENCLK = 1;	// SPI-A
   CpuSysRegs.PCLKCR8.all |= 0x01;
/*
   EALLOW;
   GpioCtrlRegs.GPAMUX2.bit.GPIO16 = 1;	// 0=GPIO,  1=SPISIMO-A,  2=Resv,  3=TZ2
   GpioCtrlRegs.GPADIR.bit.GPIO16  = 1;	// 1=OUTput,  0=INput
   GpioCtrlRegs.GPAMUX2.bit.GPIO17 = 1;	// 0=GPIO,  1=SPISOMI-A,  2=Resv,  3=TZ3
   GpioCtrlRegs.GPADIR.bit.GPIO17  = 0;	// 1=OUTput,  0=INput
   GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 1;	// 0=GPIO,  1=SPICLK-A,  2=LINTX-A,  3=XCLKOUT
   GpioCtrlRegs.GPADIR.bit.GPIO18  = 1;	// 1=OUTput,  0=INput
   GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 1;	// 0=GPIO,  1=SPISTE-A,  2=LINRX-A,  3=ECAP1
   GpioCtrlRegs.GPADIR.bit.GPIO19  = 1;	// 1=OUTput,  0=INput
   EDIS;
*/

  // Setup the SPI CCR register
  SpiaRegs.SPICCR.bit.SPISWRESET = 0;
  SpiaRegs.SPICCR.bit.CLKPOLARITY = 1;
  SpiaRegs.SPICCR.bit.SPILBK = 0;
  SpiaRegs.SPICCR.bit.SPICHAR = 0xf;


  // Setup the SPI CTL register
  SpiaRegs.SPICTL.bit.OVERRUNINTENA = 0;
  SpiaRegs.SPICTL.bit.CLK_PHASE = 1;
  SpiaRegs.SPICTL.bit.MASTER_SLAVE = 1;
  SpiaRegs.SPICTL.bit.TALK = 1;
  SpiaRegs.SPICTL.bit.SPIINTENA = 0;
 // Initialize the SPI BRR (Baud Rate Register) to 1 Mb
  SpiaRegs.SPIBRR.all = 3;//5;  //  reduce -> increase clock


  // Setup the SPI FIFO TX path
    SpiaRegs.SPIFFTX.bit.SPIRST = 0;
    SpiaRegs.SPIFFTX.bit.SPIFFENA = 1;
    SpiaRegs.SPIFFTX.bit.TXFIFO = 1;
    SpiaRegs.SPIFFTX.bit.TXFFIENA = 0;
    SpiaRegs.SPIFFTX.bit.SPIRST = 1;

    // Setup the SPI FIFO RX path
    SpiaRegs.SPIFFRX.bit.RXFIFORESET = 0;
    SpiaRegs.SPIFFRX.bit.RXFFIENA = 0;
    SpiaRegs.SPIFFRX.bit.RXFIFORESET = 1;

    // Setup the SPI FIFO CTL
    SpiaRegs.SPIFFCT.bit.TXDLY = 1;  // reduce for more channel

  // Take the SPI port out of reset
  SpiaRegs.SPICCR.bit.SPISWRESET = 1;

  // Setup the SPI priority register
  SpiaRegs.SPIPRI.bit.FREE = 1;
  //SpiaRegs.SPIPRI.bit.STEINV = 0;
  //SpiaRegs.SPIPRI.bit.TRIWIRE = 0;

  // Set the LDAC pin high

  SPIDAC_SET_LDAC;

}




void SPIDAC_xmit_u16(unsigned int data)
{
	unsigned int volatile v;
//	int volatile i, j;

	do
	  {	v = SpiaRegs.SPIFFTX.bit.TXFFST; }	while (v == 4);

	SpiaRegs.SPITXBUF = data;

	do	// wait for the transmit FIFO to clear
	  {	v = SpiaRegs.SPIFFTX.bit.TXFFST; }	while (v > 0);


	do	// wait for the transmit buffer to clear
	  {	v = SpiaRegs.SPISTS.bit.BUFFULL_FLAG; }	while (v);

//	for (i=0; i<10; i++)
//		j+=i;
}




void SPIDAC_write_dac_channel(int ch, unsigned int data)
{
	unsigned int v;

	data &= 0xfff; ch &= 0x3; v=data|(ch<<14)|(1<<12);

	SPIDAC_xmit_u16(v);
}

void SPIDAC_update_all()
{
	SPIDAC_CLEAR_LDAC;
	SPIDAC_SET_LDAC;
}
