//----------------------------------------------------------------------------------
//	FILE:			config.c
//
//	Description:	Contains all peripheral init / config functions
//
//	Version: 		1.0
//
//  Target:  		TMS320F28377S
//
//----------------------------------------------------------------------------------
//  Copyright Texas Instruments ��蕭 2004-2015
//----------------------------------------------------------------------------------
//  Revision History:
//----------------------------------------------------------------------------------
//  Date	  | Description / Status
//----------------------------------------------------------------------------------
// 4 Nov 2015 - Peripheral config modules
//----------------------------------------------------------------------------------


/*-------------------------------------------------------------------------------
Include project specific include files.
-------------------------------------------------------------------------------*/
#include "F28x_Project.h"
#include "F2837xs_IO_assignment.h"
#include "MonoMtrServo-Settings.h"

// ****************************************************************************
// ****************************************************************************
//TODO EPWMDAC GPIO Configuration
// ****************************************************************************
// ****************************************************************************
void InitMotor1EPwmDACGpio(void)
{
    // set up EPWM7AB
    GPIO_SetupPinOptions(MOTOR1_EPWMDAC_A_GPIO, GPIO_OUTPUT, GPIO_PULLUP);
//    GPIO_SetupPinMux(MOTOR1_EPWMDAC_A_GPIO, 0, MOTOR1_EPWMDAC_A_MUX);

    GPIO_SetupPinOptions(MOTOR1_EPWMDAC_Ab_GPIO, GPIO_OUTPUT, GPIO_PULLUP);
//    GPIO_SetupPinMux(MOTOR1_EPWMDAC_Ab_GPIO, 0, MOTOR1_EPWMDAC_Ab_MUX);

    // set up EPWM8AB
    GPIO_SetupPinOptions(MOTOR1_EPWMDAC_B_GPIO, GPIO_OUTPUT, GPIO_PULLUP);
//    GPIO_SetupPinMux(MOTOR1_EPWMDAC_B_GPIO, 0, MOTOR1_EPWMDAC_B_MUX);

    GPIO_SetupPinOptions(MOTOR1_EPWMDAC_Bb_GPIO, GPIO_OUTPUT, GPIO_PULLUP);
//    GPIO_SetupPinMux(MOTOR1_EPWMDAC_Bb_GPIO, 0, MOTOR1_EPWMDAC_Bb_MUX);

    return;
}

// ****************************************************************************
// ****************************************************************************
//TODO EPWM GPIO Configuration
// ****************************************************************************
// ****************************************************************************
void InitMotor1EPwmGpio(void)
{
	// set up EPWM-A
	GPIO_SetupPinOptions(MOTOR1_EPWM_A_GPIO, GPIO_OUTPUT, GPIO_PULLUP);
	GPIO_SetupPinMux(MOTOR1_EPWM_A_GPIO, 0, MOTOR1_EPWM_A_MUX);

	GPIO_SetupPinOptions(MOTOR1_EPWM_Ab_GPIO, GPIO_OUTPUT, GPIO_PULLUP);
    GPIO_SetupPinMux(MOTOR1_EPWM_Ab_GPIO, 0, MOTOR1_EPWM_Ab_MUX);

	// set up EPWM-B
	GPIO_SetupPinOptions(MOTOR1_EPWM_B_GPIO, GPIO_OUTPUT, GPIO_PULLUP);
	GPIO_SetupPinMux(MOTOR1_EPWM_B_GPIO, 0, MOTOR1_EPWM_B_MUX);

	GPIO_SetupPinOptions(MOTOR1_EPWM_Bb_GPIO, GPIO_OUTPUT, GPIO_PULLUP);
    GPIO_SetupPinMux(MOTOR1_EPWM_Bb_GPIO, 0, MOTOR1_EPWM_Bb_MUX);

	// set up EPWM-C
	GPIO_SetupPinOptions(MOTOR1_EPWM_C_GPIO, GPIO_OUTPUT, GPIO_PULLUP);
	GPIO_SetupPinMux(MOTOR1_EPWM_C_GPIO, 0, MOTOR1_EPWM_C_MUX);

	GPIO_SetupPinOptions(MOTOR1_EPWM_Cb_GPIO, GPIO_OUTPUT, GPIO_PULLUP);
    GPIO_SetupPinMux(MOTOR1_EPWM_Cb_GPIO, 0, MOTOR1_EPWM_Cb_MUX);

	return;
}

// ****************************************************************************
// ****************************************************************************
//TODO EQEP GPIO Configuration
// ****************************************************************************
// ****************************************************************************
void InitMotor1EQepGpio(void)
{
	// Set up QEP-A
	GPIO_SetupPinOptions(MOTOR1_QEPA_GPIO, GPIO_INPUT, GPIO_SYNC);
	GPIO_SetupPinMux(MOTOR1_QEPA_GPIO,0,MOTOR1_QEPA_MUX);

	// Set up QEP-B
	GPIO_SetupPinOptions(MOTOR1_QEPB_GPIO, GPIO_INPUT, GPIO_SYNC);
	GPIO_SetupPinMux(MOTOR1_QEPB_GPIO,0,MOTOR1_QEPB_MUX);

	// Set up QEP-I
	GPIO_SetupPinOptions(MOTOR1_QEPI_GPIO, GPIO_INPUT, GPIO_SYNC);
	GPIO_SetupPinMux(MOTOR1_QEPI_GPIO,0,MOTOR1_QEPI_MUX);

	return;
}

// ****************************************************************************
// ****************************************************************************
//TODO SPI GPIO Configuration
// ****************************************************************************
// ****************************************************************************
void InitMotor1SpiGpio(void)
{
	// set up SDI pin
	GPIO_SetupPinOptions(MOTOR1_SDI_GPIO, GPIO_OUTPUT, GPIO_ASYNC);
	GPIO_SetupPinMux(MOTOR1_SDI_GPIO,0,MOTOR1_SDI_MUX);

	// set up SDO pin
	GPIO_SetupPinOptions(MOTOR1_SDO_GPIO, GPIO_OUTPUT, GPIO_ASYNC);
	GPIO_SetupPinMux(MOTOR1_SDO_GPIO,0,MOTOR1_SDO_MUX);

	// set up CLK pin
	GPIO_SetupPinOptions(MOTOR1_CLK_GPIO, GPIO_OUTPUT, GPIO_ASYNC);
	GPIO_SetupPinMux(MOTOR1_CLK_GPIO,0,MOTOR1_CLK_MUX);

	// set up SCS pin
	GPIO_SetupPinOptions(MOTOR1_STE_GPIO, GPIO_OUTPUT, GPIO_ASYNC);
	GPIO_SetupPinMux(MOTOR1_STE_GPIO,0,MOTOR1_STE_MUX);

	return;
}
void InitEcapGpio(void)
{
       GPIO_SetupPinOptions(Ecap1_GPIO, GPIO_INPUT, GPIO_ASYNC);
       GPIO_SetupPinMux(Ecap1_GPIO, 0, Ecap1_MUX);

       GPIO_SetupPinOptions(Ecap2_GPIO, GPIO_INPUT, GPIO_ASYNC);
       GPIO_SetupPinMux(Ecap2_GPIO, 0, Ecap2_MUX);

       GPIO_SetupPinOptions(Ecap3_GPIO, GPIO_INPUT, GPIO_ASYNC);
       GPIO_SetupPinMux(Ecap3_GPIO, 0, Ecap3_MUX);

       GPIO_SetupPinOptions(Ecap4_GPIO, GPIO_INPUT, GPIO_ASYNC);
       GPIO_SetupPinMux(Ecap4_GPIO, 0, Ecap4_MUX);

       GPIO_SetupPinOptions(Ecap5_GPIO, GPIO_INPUT, GPIO_ASYNC);
       GPIO_SetupPinMux(Ecap5_GPIO, 0, Ecap5_MUX);

       GPIO_SetupPinOptions(Ecap6_GPIO, GPIO_INPUT, GPIO_ASYNC);// GPIO_QUAL3
       GPIO_SetupPinMux(Ecap6_GPIO, 0, Ecap6_MUX);

       EALLOW;
//       GpioCtrlRegs.GPACTRL.bit.QUALPRD0=0x0A; // 13*1/100M s= 130ns clock base, thus the AQ window is 130*3=390ns
       GpioCtrlRegs.GPACTRL.bit.QUALPRD0=0x01; // 13*1/100M s= 130ns clock base, thus the AQ window is 20*3=60ns

       EDIS;

       return;
}

void ConfigInput_XBAR(void)
{
    EALLOW;

    InputXbarRegs.INPUT7SELECT   = 0x00;
    InputXbarRegs.INPUT8SELECT   = 0x01;
    InputXbarRegs.INPUT9SELECT   = 0x02;
    InputXbarRegs.INPUT10SELECT  = 0x03;
    InputXbarRegs.INPUT11SELECT   =0x04;
    InputXbarRegs.INPUT12SELECT  = 0x05;

    EDIS;

    return;
}

// ****************************************************************************
// ****************************************************************************
//TODO DRV83xx GPIO Configuration
// ****************************************************************************
// ****************************************************************************
void InitMotor1_DRV_Gpio(void)
{
	// Configure as Output : Motor 1 - EN-GATE
	GPIO_WritePin(MOTOR1_EN_GATE_GPIO, 0);  // Disable EN_GATE
	GPIO_SetupPinOptions(MOTOR1_EN_GATE_GPIO, GPIO_OUTPUT, GPIO_ASYNC);
	GPIO_SetupPinMux(MOTOR1_EN_GATE_GPIO, 0, MOTOR1_EN_GATE_MUX);

#if (DRV_MOTOR1 == DRV8301)
	// Configure as Output : Motor 1 - DC-CAL for DRV8301
	GPIO_WritePin(MOTOR1_DC_CAL_GPIO, 1);  // Set DC-CAL to 1 to disable amplifiers
	GPIO_SetupPinOptions(MOTOR1_DC_CAL_GPIO, GPIO_OUTPUT, GPIO_ASYNC);
	GPIO_SetupPinMux(MOTOR1_DC_CAL_GPIO, 0, MOTOR1_DC_CAL_MUX);
#else
	// Configure as Output : Motor 1 - WAKE for DRV8305
	GPIO_WritePin(MOTOR1_WAKE_GPIO, 0);  // Set WAKE to 0 (default state)
	GPIO_SetupPinOptions(MOTOR1_WAKE_GPIO, GPIO_OUTPUT, GPIO_ASYNC);
	GPIO_SetupPinMux(MOTOR1_WAKE_GPIO, 0, MOTOR1_WAKE_MUX);
#endif
}

// ****************************************************************************
// ****************************************************************************
//TODO CPU SYSTEM CONTROL Configuration
// ****************************************************************************
// ****************************************************************************
#pragma CODE_SECTION(InitFlash_Bank0, "ramfuncs");
#pragma CODE_SECTION(InitFlash_Bank1, "ramfuncs");

void InitSysCtrl1(void)
{
    // Disable the watchdog
    DisableDog();

#ifdef _FLASH
// Copy time critical code and Flash setup code to RAM
// This includes the following functions:  InitFlash();
// The  RamfuncsLoadStart, RamfuncsLoadSize, and RamfuncsRunStart
// symbols are created by the linker. Refer to the device .cmd file.
    memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (size_t)&RamfuncsLoadSize);

// Call Flash Initialization to setup flash waitstates
// This function must reside in RAM
//    InitFlash_Bank0();
    InitFlash();
#endif

    // *IMPORTANT*
    // The Device_cal function, which copies the ADC & oscillator calibration values
    // from TI reserved OTP into the appropriate trim registers, occurs automatically
    // in the Boot ROM. If the boot ROM code is bypassed during the debug process, the
    // following function MUST be called for the ADC and oscillators to function according
    // to specification. The clocks to the ADC MUST be enabled before calling this
    // function.
    // See the device data manual and/or the ADC Reference
    // Manual for more information.

    EALLOW;

    //enable pull-ups on unbonded IOs as soon as possible to reduce power consumption.
    GPIO_EnableUnbondedIOPullups();

	CpuSysRegs.PCLKCR13.bit.ADC_A = 1;
	CpuSysRegs.PCLKCR13.bit.ADC_B = 1;
	CpuSysRegs.PCLKCR13.bit.ADC_C = 1;
	CpuSysRegs.PCLKCR13.bit.ADC_D = 1;

    //check if device is trimmed
    if(*((Uint16 *)0x5D1B6) == 0x0000){
        //device is not trimmed, apply static calibration values
        AnalogSubsysRegs.ANAREFTRIMA.all = 31709;
        AnalogSubsysRegs.ANAREFTRIMB.all = 31709;
        AnalogSubsysRegs.ANAREFTRIMC.all = 31709;
        AnalogSubsysRegs.ANAREFTRIMD.all = 31709;
    }

	CpuSysRegs.PCLKCR13.bit.ADC_A = 0;
	CpuSysRegs.PCLKCR13.bit.ADC_B = 0;
	CpuSysRegs.PCLKCR13.bit.ADC_C = 0;
	CpuSysRegs.PCLKCR13.bit.ADC_D = 0;


    EDIS;

    // Defined options to be passed as arguments to this function are defined
    // in f2838x_examples.h.
    //
    // Note: The internal oscillator CANNOT be used as the PLL source if the
    // PLLSYSCLK is configured to frequencies above 194 MHz.
    //
    //  PLLSYSCLK = (XTAL_OSC) * (IMULT) /(REFDIV) * (ODIV) * (PLLSYSCLKDIV)
    //  PLLSYSCLK = 25*32/(1+1)/(1+1)/1 = 200MHz
    InitSysPll(XTAL_OSC, IMULT_32, REFDIV_2, ODIV_2, PLLCLK_BY_1, SYSCTL_DCC_BASE0); 		//PLLSYSCLK = 10Mhz(OSCCLK) * 40 (IMULT) * 1 (FMULT) /  2 (PLLCLK_BY_2)

    //Turn on all peripherals
	InitPeripheralClocks();

}

// ****************************************************************************
// ****************************************************************************
//TODO ADC Configuration
// ****************************************************************************
// ****************************************************************************
//Write ADC configurations and power up the ADC for both ADC A and ADC B
void ConfigureADC(void)
{
	EALLOW;

	//write configurations
	AdcaRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
	AdcbRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
	AdccRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcdRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4

//    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);//ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE
//    AdcSetMode(ADC_ADCB, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);//ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE

    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_16BIT, ADC_SIGNALMODE_SINGLE);//ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE
    AdcSetMode(ADC_ADCB, ADC_RESOLUTION_16BIT, ADC_SIGNALMODE_SINGLE);//ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE
    AdcSetMode(ADC_ADCC, ADC_RESOLUTION_16BIT, ADC_SIGNALMODE_SINGLE);//ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE
    AdcSetMode(ADC_ADCD, ADC_RESOLUTION_16BIT, ADC_SIGNALMODE_SINGLE);//ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE

	//Set pulse positions to late
	AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;
	AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1;
	AdccRegs.ADCCTL1.bit.INTPULSEPOS = 1;
	AdcdRegs.ADCCTL1.bit.INTPULSEPOS = 1;

	//power up the ADC
	AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;
	AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;
	AdccRegs.ADCCTL1.bit.ADCPWDNZ = 1;
	AdcdRegs.ADCCTL1.bit.ADCPWDNZ = 1;

	//delay for 1ms to allow ADC time to power up
	DELAY_US(1000);

	EDIS;
}

// ****************************************************************************
// ****************************************************************************
//TODO PWM Configuration
// ****************************************************************************
// ****************************************************************************
void PWM_1ch_UpDwnCnt_CNF(volatile struct EPWM_REGS * ePWM, Uint16 period, int16 db)
{
	EALLOW;
	// Time Base SubModule Registers
	ePWM->TBCTL.bit.PRDLD = TB_IMMEDIATE; // set Immediate load
	ePWM->TBPRD = period / 2; // PWM frequency = 1 / period
	ePWM->TBPHS.bit.TBPHS = 0;
	ePWM->TBCTR = 0;
	ePWM->TBCTL.bit.CTRMODE   = TB_COUNT_UPDOWN;
	ePWM->TBCTL.bit.HSPCLKDIV = TB_DIV1;
	ePWM->TBCTL.bit.CLKDIV    = TB_DIV1;

	ePWM->TBCTL.bit.PHSEN    = TB_DISABLE;
//	ePWM->TBCTL.bit.SYNCOSEL = TB_CTR_ZERO; // sync "down-stream"

	// Counter Compare Submodule Registers
	ePWM->CMPA.bit.CMPA = 0; // set duty 0% initially
	ePWM->CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	ePWM->CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;

	// Action Qualifier SubModule Registers
	ePWM->AQCTLA.bit.CAU = AQ_CLEAR;
	ePWM->AQCTLA.bit.CAD = AQ_SET;

	// Active high complementary PWMs - Set up the deadband
	ePWM->DBCTL.bit.IN_MODE  = DBA_ALL;
	ePWM->DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
	ePWM->DBCTL.bit.POLSEL   = DB_ACTV_HIC;
	ePWM->DBRED.all = db;
	ePWM->DBFED.all = db;
	EDIS;
}

// ****************************************************************************
// ****************************************************************************
//TODO PWM Configuration
// ****************************************************************************
// ****************************************************************************
void PWM_1ch_UpDwnCnt_CNF_noDB(volatile struct EPWM_REGS * ePWM, Uint16 period)
{
	EALLOW;
	// Time Base SubModule Registers
	ePWM->TBCTL.bit.PRDLD = TB_IMMEDIATE; // set Immediate load
	ePWM->TBPRD = period / 2; // PWM frequency = 1 / period
	ePWM->TBPHS.bit.TBPHS = 0;
	ePWM->TBCTR = 0;
	ePWM->TBCTL.bit.CTRMODE   = TB_COUNT_UPDOWN;
	ePWM->TBCTL.bit.HSPCLKDIV = TB_DIV1;
	ePWM->TBCTL.bit.CLKDIV    = TB_DIV1;

	ePWM->TBCTL.bit.PHSEN    = TB_DISABLE;
//	ePWM->TBCTL.bit.SYNCOSEL = TB_CTR_ZERO; // sync "down-stream"

	// Counter Compare Submodule Registers
	ePWM->CMPA.bit.CMPA = 0; // set duty 0% initially
	ePWM->CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	ePWM->CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;

	// Action Qualifier SubModule Registers
	ePWM->AQCTLA.bit.CAU = AQ_CLEAR;
	ePWM->AQCTLA.bit.CAD = AQ_SET;

	// Active high complementary PWMs - Disable deadband
	ePWM->DBCTL.bit.OUT_MODE = DB_DISABLE;
	EDIS;
}

void SetEcapReg(void)
{
    EALLOW;

    ECap1Regs.ECEINT.all = 0x0000; // Disable all capture interrupts
    ECap1Regs.ECCLR.all = 0xFFFF; // Clear all CAP interrupt flags
    ECap1Regs.ECCTL1.bit.CAPLDEN = 0; // Disable CAP1-CAP4 register loads
    ECap1Regs.ECCTL2.bit.TSCTRSTOP = 0; // Make sure the counter is stopped
    ECap1Regs.ECCTL1.bit.CAP1POL = 0;//CAP1POL_FALLING_EDGE;
    ECap1Regs.ECCTL1.bit.CAP2POL = 1;//CAP1POL_FALLING_EDGE;
    ECap1Regs.ECCTL1.bit.CAP3POL = 0;//CAP1POL_FALLING_EDGE;
    ECap1Regs.ECCTL1.bit.CAP4POL = 1;//CAP1POL_FALLING_EDGE;
    ECap1Regs.ECCTL1.bit.CTRRST1 = 1;//CTRRST1_DIFFERENCE_TS;
    ECap1Regs.ECCTL1.bit.CTRRST2 = 1;//CTRRST2_DIFFERENCE_TS;
    ECap1Regs.ECCTL1.bit.CTRRST3 = 1;//CTRRST3_DIFFERENCE_TS;
    ECap1Regs.ECCTL1.bit.CTRRST4 = 1;//CTRRST4_DIFFERENCE_TS;
    ECap1Regs.ECCTL1.bit.CAPLDEN = 1;//CAPLDEN_ENABLE;
    ECap1Regs.ECCTL1.bit.PRESCALE = 0;
    ECap1Regs.ECCTL2.bit.CAP_APWM = 0;//CAPTURE_MODE;
    ECap1Regs.ECCTL2.bit.SYNCO_SEL = 0;//SYNCO_DISABLE;
    ECap1Regs.ECCTL2.bit.SYNCI_EN = 0;//SYNCI_DISABLE;
    ECap1Regs.ECCTL2.bit.TSCTRSTOP = 1;//TSCNTSTP_FREE;

    ECap2Regs.ECEINT.all = 0x0000; // Disable all capture interrupts
    ECap2Regs.ECCLR.all = 0xFFFF; // Clear all CAP interrupt flags
    ECap2Regs.ECCTL1.bit.CAPLDEN = 0; // Disable CAP1-CAP4 register loads
    ECap2Regs.ECCTL2.bit.TSCTRSTOP = 0; // Make sure the counter is stopped
    ECap2Regs.ECCTL1.bit.CAP1POL = 0;//CAP1POL_FALLING_EDGE;
    ECap2Regs.ECCTL1.bit.CAP2POL = 1;//CAP1POL_FALLING_EDGE;
    ECap2Regs.ECCTL1.bit.CAP3POL = 0;//CAP1POL_FALLING_EDGE;
    ECap2Regs.ECCTL1.bit.CAP4POL = 1;//CAP1POL_FALLING_EDGE;
    ECap2Regs.ECCTL1.bit.CTRRST1 = 1;//CTRRST1_DIFFERENCE_TS;
    ECap2Regs.ECCTL1.bit.CTRRST2 = 1;//CTRRST2_DIFFERENCE_TS;
    ECap2Regs.ECCTL1.bit.CTRRST3 = 1;//CTRRST3_DIFFERENCE_TS;
    ECap2Regs.ECCTL1.bit.CTRRST4 = 1;//CTRRST4_DIFFERENCE_TS;
    ECap2Regs.ECCTL1.bit.CAPLDEN = 1;//CAPLDEN_ENABLE;
    ECap2Regs.ECCTL1.bit.PRESCALE = 0;
    ECap2Regs.ECCTL2.bit.CAP_APWM = 0;//CAPTURE_MODE;
    ECap2Regs.ECCTL2.bit.SYNCO_SEL = 0;//SYNCO_DISABLE;
    ECap2Regs.ECCTL2.bit.SYNCI_EN = 0;//SYNCI_DISABLE;
    ECap2Regs.ECCTL2.bit.TSCTRSTOP = 1;//TSCNTSTP_FREE;

    ECap3Regs.ECEINT.all = 0x0000; // Disable all capture interrupts
    ECap3Regs.ECCLR.all = 0xFFFF; // Clear all CAP interrupt flags
    ECap3Regs.ECCTL1.bit.CAPLDEN = 0; // Disable CAP1-CAP4 register loads
    ECap3Regs.ECCTL2.bit.TSCTRSTOP = 0; // Make sure the counter is stopped
    ECap3Regs.ECCTL1.bit.CAP1POL = 0;//CAP1POL_FALLING_EDGE;
    ECap3Regs.ECCTL1.bit.CAP2POL = 1;//CAP1POL_FALLING_EDGE;
    ECap3Regs.ECCTL1.bit.CAP3POL = 0;//CAP1POL_FALLING_EDGE;
    ECap3Regs.ECCTL1.bit.CAP4POL = 1;//CAP1POL_FALLING_EDGE;
    ECap3Regs.ECCTL1.bit.CTRRST1 = 1;//CTRRST1_DIFFERENCE_TS;
    ECap3Regs.ECCTL1.bit.CTRRST2 = 1;//CTRRST2_DIFFERENCE_TS;
    ECap3Regs.ECCTL1.bit.CTRRST3 = 1;//CTRRST3_DIFFERENCE_TS;
    ECap3Regs.ECCTL1.bit.CTRRST4 = 1;//CTRRST4_DIFFERENCE_TS;
    ECap3Regs.ECCTL1.bit.CAPLDEN = 1;//CAPLDEN_ENABLE;
    ECap3Regs.ECCTL1.bit.PRESCALE = 0;
    ECap3Regs.ECCTL2.bit.CAP_APWM = 0;//CAPTURE_MODE;
    ECap3Regs.ECCTL2.bit.SYNCO_SEL = 0;//SYNCO_DISABLE;
    ECap3Regs.ECCTL2.bit.SYNCI_EN = 0;//SYNCI_DISABLE;
    ECap3Regs.ECCTL2.bit.TSCTRSTOP = 1;//TSCNTSTP_FREE;

    ECap4Regs.ECEINT.all = 0x0000; // Disable all capture interrupts
    ECap4Regs.ECCLR.all = 0xFFFF; // Clear all CAP interrupt flags
    ECap4Regs.ECCTL1.bit.CAPLDEN = 0; // Disable CAP1-CAP4 register loads
    ECap4Regs.ECCTL2.bit.TSCTRSTOP = 0; // Make sure the counter is stopped
    ECap4Regs.ECCTL1.bit.CAP1POL = 0;//CAP1POL_FALLING_EDGE;
    ECap4Regs.ECCTL1.bit.CAP2POL = 1;//CAP1POL_FALLING_EDGE;
    ECap4Regs.ECCTL1.bit.CAP3POL = 0;//CAP1POL_FALLING_EDGE;
    ECap4Regs.ECCTL1.bit.CAP4POL = 1;//CAP1POL_FALLING_EDGE;
    ECap4Regs.ECCTL1.bit.CTRRST1 = 1;//CTRRST1_DIFFERENCE_TS;
    ECap4Regs.ECCTL1.bit.CTRRST2 = 1;//CTRRST2_DIFFERENCE_TS;
    ECap4Regs.ECCTL1.bit.CTRRST3 = 1;//CTRRST3_DIFFERENCE_TS;
    ECap4Regs.ECCTL1.bit.CTRRST4 = 1;//CTRRST4_DIFFERENCE_TS;
    ECap4Regs.ECCTL1.bit.CAPLDEN = 1;//CAPLDEN_ENABLE;
    ECap4Regs.ECCTL1.bit.PRESCALE = 0;
    ECap4Regs.ECCTL2.bit.CAP_APWM = 0;//CAPTURE_MODE;
    ECap4Regs.ECCTL2.bit.SYNCO_SEL = 0;//SYNCO_DISABLE;
    ECap4Regs.ECCTL2.bit.SYNCI_EN = 0;//SYNCI_DISABLE;
    ECap4Regs.ECCTL2.bit.TSCTRSTOP = 1;//TSCNTSTP_FREE;

    ECap5Regs.ECEINT.all = 0x0000; // Disable all capture interrupts
    ECap5Regs.ECCLR.all = 0xFFFF; // Clear all CAP interrupt flags
    ECap5Regs.ECCTL1.bit.CAPLDEN = 0; // Disable CAP1-CAP4 register loads
    ECap5Regs.ECCTL2.bit.TSCTRSTOP = 0; // Make sure the counter is stopped
    ECap5Regs.ECCTL1.bit.CAP1POL = 0;//CAP1POL_FALLING_EDGE;
    ECap5Regs.ECCTL1.bit.CAP2POL = 1;//CAP1POL_FALLING_EDGE;
    ECap5Regs.ECCTL1.bit.CAP3POL = 0;//CAP1POL_FALLING_EDGE;
    ECap5Regs.ECCTL1.bit.CAP4POL = 1;//CAP1POL_FALLING_EDGE;
    ECap5Regs.ECCTL1.bit.CTRRST1 = 1;//CTRRST1_DIFFERENCE_TS;
    ECap5Regs.ECCTL1.bit.CTRRST2 = 1;//CTRRST2_DIFFERENCE_TS;
    ECap5Regs.ECCTL1.bit.CTRRST3 = 1;//CTRRST3_DIFFERENCE_TS;
    ECap5Regs.ECCTL1.bit.CTRRST4 = 1;//CTRRST4_DIFFERENCE_TS;
    ECap5Regs.ECCTL1.bit.CAPLDEN = 1;//CAPLDEN_ENABLE;
    ECap5Regs.ECCTL1.bit.PRESCALE = 0;
    ECap5Regs.ECCTL2.bit.CAP_APWM = 0;//CAPTURE_MODE;
    ECap5Regs.ECCTL2.bit.SYNCO_SEL = 0;//SYNCO_DISABLE;
    ECap5Regs.ECCTL2.bit.SYNCI_EN = 0;//SYNCI_DISABLE;
    ECap5Regs.ECCTL2.bit.TSCTRSTOP = 1;//TSCNTSTP_FREE;

    ECap6Regs.ECEINT.all = 0x0000; // Disable all capture interrupts
    ECap6Regs.ECCLR.all = 0xFFFF; // Clear all CAP interrupt flags
    ECap6Regs.ECCTL1.bit.CAPLDEN = 0; // Disable CAP1-CAP4 register loads
    ECap6Regs.ECCTL2.bit.TSCTRSTOP = 0; // Make sure the counter is stopped
    ECap6Regs.ECCTL1.bit.CAP1POL = 0;//CAP1POL_FALLING_EDGE;
    ECap6Regs.ECCTL1.bit.CAP2POL = 1;//CAP1POL_FALLING_EDGE;
    ECap6Regs.ECCTL1.bit.CAP3POL = 0;//CAP1POL_FALLING_EDGE;
    ECap6Regs.ECCTL1.bit.CAP4POL = 1;//CAP1POL_FALLING_EDGE;
    ECap6Regs.ECCTL1.bit.CTRRST1 = 1;//CTRRST1_DIFFERENCE_TS;
    ECap6Regs.ECCTL1.bit.CTRRST2 = 1;//CTRRST2_DIFFERENCE_TS;
    ECap6Regs.ECCTL1.bit.CTRRST3 = 1;//CTRRST3_DIFFERENCE_TS;
    ECap6Regs.ECCTL1.bit.CTRRST4 = 1;//CTRRST4_DIFFERENCE_TS;
    ECap6Regs.ECCTL1.bit.CAPLDEN = 1;//CAPLDEN_ENABLE;
    ECap6Regs.ECCTL1.bit.PRESCALE = 0;
    ECap6Regs.ECCTL2.bit.CAP_APWM = 0;//CAPTURE_MODE;
    ECap6Regs.ECCTL2.bit.SYNCO_SEL = 0;//SYNCO_DISABLE;
    ECap6Regs.ECCTL2.bit.SYNCI_EN = 0;//SYNCI_DISABLE;
    ECap6Regs.ECCTL2.bit.TSCTRSTOP = 1;//TSCNTSTP_FREE;

    GpioCtrlRegs.GPACTRL.all |= 0x01;

    EDIS;
}

// ****************************************************************************
// ****************************************************************************
//TODO SPI Configuration
// ****************************************************************************
// ****************************************************************************
void InitSpiRegs_DRV830x(volatile struct SPI_REGS *s)
{
//	s->SPICCR.bit.SPISWRESET = 0; 		// Put SPI in reset state
//	s->SPICCR.bit.SPICHAR = 0xF;		// 16-bit character
//	s->SPICCR.bit.SPILBK = 0;     		// loopback off
//	s->SPICCR.bit.CLKPOLARITY = 0;  	// Rising edge without delay
//
//	s->SPICTL.bit.SPIINTENA = 0;		// disable SPI interrupt
//	s->SPICTL.bit.TALK = 1;				// enable transmission
//	s->SPICTL.bit.MASTER_SLAVE = 1;		// master
//	s->SPICTL.bit.CLK_PHASE = 0;    	// Rising edge without delay
//	s->SPICTL.bit.OVERRUNINTENA = 0;	// disable reciever overrun interrupt
//
//	s->SPIBRR.bit.SPI_BIT_RATE = 19;		// SPICLK = LSPCLK / 4 (max SPICLK)
//
//	s->SPICCR.bit.SPISWRESET=1;  		// Enable SPI
}


// ****************************************************************************
// ****************************************************************************
//TODO Device Number Gpio Configuration
// ****************************************************************************
// ****************************************************************************
void config_device_number_Gpio(void){
    GPIO_SetupPinMux(22,0,0);
    GPIO_SetupPinMux(23,0,0);
    GPIO_SetupPinMux(24,0,0);
    GPIO_SetupPinMux(25,0,0);

    GPIO_SetupPinOptions(22, GPIO_INPUT, GPIO_PUSHPULL);
    GPIO_SetupPinOptions(23, GPIO_INPUT, GPIO_PUSHPULL);
    GPIO_SetupPinOptions(24, GPIO_INPUT, GPIO_PUSHPULL);
    GPIO_SetupPinOptions(25, GPIO_INPUT, GPIO_PUSHPULL);

}

int set_device_number(void){
    int b0 = GpioDataRegs.GPADAT.bit.GPIO22;
    int b1 = GpioDataRegs.GPADAT.bit.GPIO23;
    int b2 = GpioDataRegs.GPADAT.bit.GPIO24;
    int b3 = GpioDataRegs.GPADAT.bit.GPIO25;

    int decimal = (b3 << 3) | (b2 << 2) | (b1 << 1) | b0;
    return decimal+1; // device number start from 1

}
/*****************************************************************************/
// EOF
/*****************************************************************************/
