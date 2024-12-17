/* ============================================================================
System Name:  	Mono Motor Servo Control using F28377s-XL and BOOSTXL-DRV830x

File Name:	  	MonoMtrServo.c

Target:			F28377s Launch Pad

Author:			C2000 Systems Lab, 30th September 2015

Description:	Motor ISR
				Coded within ADCB1INT ISR @ 10Khz,
				  --> triggered by ADCB SOC6,
				      --> set up by EPWM4_SOCA tied to EPWM4 PRD

//----------------------------------------------------------------------------------
//  Copyright Texas Instruments 嚙踝蕭謕 2004-2015
//----------------------------------------------------------------------------------
//  Revision History:Torque_DB
//----------------------------------------------------------------------------------
//  Date	  | Description / Status
//----------------------------------------------------------------------------------
// 4 Nov 2015 - Field Oriented Control of a PMSM with QEP feedback using F28377s-XL
 *              and BOOSTXL-DRV8301 or BOOSTXL-DRV8305EVM
//----------------------------------------------------------------------------------
 *
 *
Peripheral Assignments:
   MOTOR 1:
     - EPWMs ==>> EPWM4, EPWM5,  EPWM6  ---> A, B, C
     - QEP   ==>> EQep3
     - SPI   ==>> Spia

	 Analog signals - Motor 1
	 Vdc  ADC (B)14
	 Va   ADC B1
	 Vb   ADC B4
	 Vc   ADC B2
	 Ia   ADC A0
	 Ib   ADC B3 !!
	 Ic   ADC A1

	  DAC-C  ---> General purpose display (??)

===========================================================================  */


/* TECOM_sensor_asram_v2
 * This version has below enhancement
 * 1. The FAST data collection is wrapped as a function in  sci_tx.c (FAST_source)
 * 2. The EMIF memory access is established, but is only for this version since the chip layout is not correct (EMIF setting files )
 * 3. The collection should has two mode, one is transmitting in PWM, one is transmitting in receive ISR (main file )
 * 4. The VAC measurement is implement (config.c,  )
 * 5. This project is triggered by one byte
 * 6. Once the rx interrupt trigger, the buffer is forced to read a certain time
 * 7. IPC get sensor data one by one package
 * 8. The collection has three read mode 1. FAST  data 2. Flux data 3. RUL data
 * 9. According to 8., the ASRAM has to be able to save at least 6 arrays
 * 10. RX break detection, force the device to continuously receiving data
 * 11. Add communication mode, stop sampling when continuous sampling
 *
 */



// Include header files used in the main function
// define float maths and then include IQmath library

#include "MonoMtrServo.h"

#include "MonoMtrServo-Settings.h"

#include "spi_dac.h"
#include "sci_tx.h"
#include "EMIF_config.h"
#include "F2837xD_Dac_setting.h"
#include "FAST_vars.h"
#include "EMIF_config.h"
#include "Emif_asram.h"

// **********************************************************
// Prototypes for local functions within this file
// **********************************************************

// INTERRUPT FUNCTIONS
// ---------------------

#ifdef _FLASH
#pragma CODE_SECTION(MotorControlISR,".TI.ramfunc");
#pragma CODE_SECTION(scibRxFifoIsr,".TI.ramfunc");
#endif

#pragma INTERRUPT (MotorControlISR, HPI)
#pragma INTERRUPT (scibRxFifoIsr, HPI)

// Prototype statements for functions found within this file.
interrupt void MotorControlISR(void);
// interrupt for scib receiver
interrupt void scibRxFifoIsr(void);

// Core Motor Control Functions
// ------------------------------
#if BUILDLEVEL  != LEVEL1
inline void motorCurrentSense(void);
//inline void motorVACSense(float *Va,float *Vb,float *Vc );
//inline void motorACCSense(float *acc);
//inline void motorACCSense(float *temp);
//inline void set_communication_mode(Uint16 communication_mode_active);
inline void send_data_package(Uint16 device_num, Uint16 data1,Uint16 data2,Uint16 data3,Uint16 data4);
inline void posEncoder(MOTOR_VARS * motor);
#endif

void PwmTripConfig(volatile struct EPWM_REGS * PwmRegs, Uint16 TripNum);
void DMC1_Protection(void);
void DMC2_Protection(void);

// Miscellaneous functions
// -------------------------
_iq refPosGen(_iq out);
_iq ramper(_iq in, _iq out, _iq rampDelta);
_iq ramper_speed(_iq in, _iq out, _iq rampDelta);
void GPIO_TogglePin(Uint16 pin);


#ifdef _FLASH
#pragma DATA_SECTION(RX_err,"ramdata");
#endif


Uint16 RX_err=0;
//----------------------------------------------
// SFF paras
//----------------------------------------------
_iq SFFA=20000,SFFB,SFFC,Flux_A_est_active_SFF[4],Flux_B_est_active_SFF[4];
_iq epsilon=100;
int16 SFF_en=0;
//SFFA=2/T, SFFB=e*w0*2/T. SFFC=w0^2
//----------------------------------------------
// Torque estimation
//----------------------------------------------
_iq Flux_Cur_Alpha_last,Flux_Cur_Beta_last;
//----------------------------------------------
// Gopinath-style stator flux linkage observer
//----------------------------------------------
PI_CONTROLLER  Lamda_d;
PI_CONTROLLER  Lamda_q;
_iq vd_flux,vq_flux;
_iq id_flux,iq_flux;
_iq Lamda_dRef;
_iq Lamda_qRef;
_iq Lamda_df;
_iq Lamda_qf;
_iq Lamdac_d,Lamdac_q,Lamdac_d_last,Lamdac_q_last;
_iq vd_flux_last = _IQ(0);
_iq vq_flux_last = _IQ(0);
int autotune_enable = 0;
_iq smooth_data[5],smooth_data_temp[5],F_angle3;
_iq Flux_A_est,Flux_B_est,B_damp=0;
int sm_length=5,count_i;
int dac_out=3;

int sci_sts_count=0;
// external sensor
float acc_y,systemp,avg_temp,avg_temp_last;
volatile Uint16 communication_mode_active=0;

// State Machine function prototypes
//------------------------------------
// Alpha states
void A0(void);	//state A0
void B0(void);	//state B0
void C0(void);	//state C0

// A branch states
void A1(void);	//state A1
void A2(void);	//state A2
void A3(void);	//state A3

// B branch states
void B1(void);	//state B1
void B2(void);	//state B2
void B3(void);	//state B3

// C branch states
void C1(void);	//state C1
void C2(void);	//state C2
void C3(void);	//state C3

// Variable declarations
void (*Alpha_State_Ptr)(void);	// Base States pointer
void (*A_Task_Ptr)(void);		// State pointer A branch
void (*B_Task_Ptr)(void);		// State pointer B branch
void (*C_Task_Ptr)(void);		// State pointer C branch
// sci prototypes


// ****************************************************************************
// Variables for CPU control
// ****************************************************************************
// adc static cal
int *adc_cal;

int16	VTimer0[4];			// Virtual Timers slaved off CPU Timer 0 (A events)
int16	VTimer1[4]; 		// Virtual Timers slaved off CPU Timer 1 (B events)
int16	VTimer2[4]; 		// Virtual Timers slaved off CPU Timer 2 (C events)
int16	SerialCommsTimer;


// ****************************************************************************
// PWMDAC for control
// ****************************************************************************
int16 PwmDacCh1=0;
int16 PwmDacCh2=0;
int16 PwmDacCh3=0;
int16 PwmDacCh4=0;

// Instance a PWM DAC driver instance
PWMDAC pwmdac1 = PWMDAC_DEFAULTS;

//*********************** USER Variables *************************************


//****************************************************************************
// Global variables used in this system
//****************************************************************************

// ****************************************************************************
// Flag variables
// ****************************************************************************
//volatile Uint16 EnableFlag = FALSE;
volatile Uint16 EnableFlag = TRUE;

Uint32 IsrTicker = 0;

Uint16 BackTicker = 0;

int    LedCnt = 500;

int16 OffsetCalCounter;

_iq K1 = _IQ(0.998),		  // Offset filter coefficient K1: 0.05/(T+0.05);
        K2 = _IQ(0.001999);	      // Offset filter coefficient K2: T/(T+0.05);

MOTOR_VARS  motor1 = DRV830x_MOTOR_DEFAULTS;

// ****************************************************************************
// Miscellaneous Variables
// ****************************************************************************
_iq  IdRef_start = _IQ(0.1),
        IdRef_run   = _IQ(0.0);

// Variables for position reference generation and control
// =========================================================
_iq   posArray[8] = { _IQ(1.5), _IQ(-1.5), _IQ(2.5), _IQ(-2.5) },
        cntr1=0 ,
        posSlewRate = _IQ(0.001);

int16 ptrMax = 2,
        ptr1=0;

#if (BUILDLEVEL==LEVEL1)
Uint16 DRV_RESET = 1;
#else
Uint16 DRV_RESET = 0;
#endif

// ****************************************************************************
// Variables for Datalog module
// ****************************************************************************
float DBUFF_4CH1[4000],
DBUFF_4CH2[200],
DBUFF_4CH3[200],
DBUFF_4CH4[200],
DlogCh1,
DlogCh2,
DlogCh3,
DlogCh4;
int index1,index2,index3,index4;

// Create an instance of DATALOG Module
DLOG_4CH_F dlog_4ch1;

//*******************************************************************************

#if BUILDLEVEL != LEVEL1
// ******************************************************************************
// CURRENT SENSOR SUITE
// - Reads motor currents from inverter bottom leg SHUNTs
// ******************************************************************************
inline void motorCurrentSense()
{
    motor1.currentAs = (float) AdcaResultRegs.ADCRESULT0*ADC_PU_SCALE_FACTOR-Cur_OffsetA; //(float)IFB_A1_PPB* ADC_PU_PPB_SCALE_FACTOR;
    motor1.currentBs = (float) AdccResultRegs.ADCRESULT0*ADC_PU_SCALE_FACTOR-Cur_OffsetB; //(float)IFB_B1_PPB* ADC_PU_PPB_SCALE_FACTOR;
    motor1.currentCs = -motor1.currentAs - motor1.currentBs;

    return;
}

//inline void motorVACSense(float *Va,float *Vb,float *Vc )
//{
//    *Va = (float)AdccResultRegs.ADCRESULT4*ADC_PU_SCALE_FACTOR; //(float)IFB_A1_PPB* ADC_PU_PPB_SCALE_FACTOR;
//    *Va = (float)AdcaResultRegs.ADCRESULT4*ADC_PU_SCALE_FACTOR; //(float)IFB_A1_PPB* ADC_PU_PPB_SCALE_FACTOR;
//    *Va = (float)AdcdResultRegs.ADCRESULT4*ADC_PU_SCALE_FACTOR; //(float)IFB_A1_PPB* ADC_PU_PPB_SCALE_FACTOR;
//        return;
//}
//
//inline void motorACCSense(float *acc)
//{
//    *acc = (float)AdcaResultRegs.ADCRESULT1*ADC_PU_SCALE_FACTOR; //(float)IFB_A1_PPB* ADC_PU_PPB_SCALE_FACTOR;
//    return;
//}
//inline void motortempSense(float *acc)
//{
//    *acc = (float)AdcbResultRegs.ADCRESULT1*ADC_PU_SCALE_FACTOR; //(float)IFB_A1_PPB* ADC_PU_PPB_SCALE_FACTOR;
//    return;
//}




// ******************************************************************************
// POSITION ENCODER
// - Reads QEP
// - Angles are normalised to the the range 0 to 0.99999 (1.0)
// ******************************************************************************
inline void posEncoder(MOTOR_VARS * motor)
{
    //	Uint16 q = motor->eQEP;
    volatile struct EQEP_REGS * v = motor->QepRegs;

    // ----------------------------------
    // motor->lsw = 0 ---> Alignment Routine
    // ----------------------------------
    if (motor->lsw == 0)
    {
        // during alignment, assign the current shaft position as initial position
        v->QPOSCNT = 0;
        v->QCLR.bit.IEL = 1;  // Reset position cnt for QEP
    } // end if (motor->lsw=0)

    // ******************************************************************************
    //    Detect calibration angle and call the QEP module
    // ******************************************************************************
    // for once the QEP index pulse is found, go to lsw=2
    if(motor->lsw==1)
    {
        if (v->QFLG.bit.IEL == 1)			// Check the index occurrence
        {
            motor->qep.CalibratedAngle=v->QPOSILAT;
            //			v->QPOSINIT = v->QPOSILAT; //new
            //			v->QEPCTL.bit.IEI = IEI_RISING;   // new
            motor->lsw=2;
        }   // Keep the latched pos. at the first index
    }

    if (motor->lsw!=0)
    {
        QEP_MACRO(v,motor->qep);
    }

    // Reverse the sense of position if needed - comment / uncomment accordingly
    if (motor->PosSenseReverse)
    {
        // Position Sense Reversal
        motor->ElecTheta = 1.0 - motor->qep.ElecTheta;
        motor->MechTheta = 1.0 - motor->qep.MechTheta;
    }
    else
    {
        // Position Sense as is
        motor->ElecTheta = motor->qep.ElecTheta;
        motor->MechTheta = motor->qep.MechTheta;
    }

    return;
}
#endif


// ****************************************************************************
// ****************************************************************************
//TODO  DMC Protection Against Over Current Protection
// ****************************************************************************
// ****************************************************************************

#define  TRIP4   3

void PwmTripConfig(volatile struct EPWM_REGS * PwmRegs, Uint16 TripNum)
{
    EALLOW;

    PwmRegs->DCTRIPSEL.bit.DCAHCOMPSEL = TripNum; //TripNum is input to DCAHCOMPSEL
    PwmRegs->TZDCSEL.bit.DCAEVT1       = TZ_DCAH_HI;// TZ_EVT_DISABLE;
    PwmRegs->DCACTL.bit.EVT1SRCSEL     = DC_EVT1;
    PwmRegs->DCACTL.bit.EVT1FRCSYNCSEL = DC_EVT_ASYNC;
    PwmRegs->TZSEL.bit.DCAEVT1         = 1;
    PwmRegs->TZSEL.bit.CBC6    = 0x1;         // Emulator Stop
    PwmRegs->TZCTL.bit.TZA     = TZ_FORCE_LO; // TZA event force EPWMxA go low
    PwmRegs->TZCTL.bit.TZB     = TZ_FORCE_LO; // TZB event force EPWMxB go low
    PwmRegs->TZCLR.bit.DCAEVT1 = 1;           // Clear any spurious OV trip
    PwmRegs->TZCLR.bit.OST     = 1;           // clear any spurious OST set early

    EDIS;
}

//*****************************************************************************
void DMC1_Protection(void)
{
    // Configure GPIO used for Trip Mechanism

#if (MOTOR1_DRV == DRV8301)
    // Configure as Input - Motor 1 - OCTW - (active low), trip PWM based on this
    GPIO_SetupPinOptions(MOTOR1_OCTW_GPIO, GPIO_INPUT, GPIO_INVERT);
    GPIO_SetupPinMux(MOTOR1_OCTW_GPIO, 0, MOTOR1_OCTW_MUX);
#endif

    // Configure as Input - Motor 1 - FAULT - (active low), trip PWM based on this
    GPIO_SetupPinOptions(MOTOR1_FAULT_GPIO, GPIO_INPUT, GPIO_INVERT);
    GPIO_SetupPinMux(MOTOR1_FAULT_GPIO, 0, MOTOR1_FAULT_MUX);

    EALLOW;

    // Enable Mux 0  OR Mux 4 to generate TRIP4
    // Clear everything first
    EPwmXbarRegs.TRIP4MUX0TO15CFG.all  = 0x0000;    // clear all MUXes - MUX  0 to 15
    EPwmXbarRegs.TRIP4MUX16TO31CFG.all = 0x0000;    // clear all MUXes - MUX 16 to 31
    EPwmXbarRegs.TRIP4MUXENABLE.all    = 0x0000;    // Disable all the muxes first

    // Enable Muxes for ORed input of InputXbar1, InputXbar2
    InputXbarRegs.INPUT1SELECT = MOTOR1_FAULT_GPIO; // Select FAULT as INPUTXBAR1
    EPwmXbarRegs.TRIP4MUX0TO15CFG.bit.MUX1 = 1;     // Enable Mux for ored input of inputxbar1
    EPwmXbarRegs.TRIP4MUXENABLE.bit.MUX1   = 1;     // Enable MUX1 to generate TRIP4

#if (MOTOR1_DRV == DRV8301)
    InputXbarRegs.INPUT2SELECT = MOTOR1_OCTW_GPIO;  // Select OCTW as INPUTXBAR2
    EPwmXbarRegs.TRIP4MUX0TO15CFG.bit.MUX3 = 1;     // Enable Mux for ored input of inputxbar2
    EPwmXbarRegs.TRIP4MUXENABLE.bit.MUX3   = 1;     // Enable MUX3 to generate TRIP4
#endif

    EDIS;

    PwmTripConfig(motor1.PwmARegs, TRIP4);
    PwmTripConfig(motor1.PwmBRegs, TRIP4);
    PwmTripConfig(motor1.PwmCRegs, TRIP4);

    return;
}

// ****************************************************************************
// ****************************************************************************
// GENERAL PURPOSE UTILITY FUNCTIONS
// ****************************************************************************
// ****************************************************************************

// slew programmable ramper
_iq ramper(_iq in, _iq out, _iq rampDelta)
{
    _iq err;

    err = in - out;
    if (err > rampDelta)
        return(out + rampDelta);
    else if (err < -rampDelta)
        return(out - rampDelta);
    else
        return(in);
}

//*****************************************************************************
// Ramp Controller for speed reference (Not currently used)
_iq ramper_speed(_iq in, _iq out, _iq rampDelta)
{
    _iq err;

    err = in - out;
    if (err > rampDelta)
    {
        if((out+rampDelta)>1.0)
            return(1.0);
        else
            return (out+rampDelta);
    }
    else if (err < -rampDelta)
    {
        if(out-rampDelta<=0.0)
            return(0.0);
        else
            return(out - rampDelta);
    }
    else
        return(in);
}

//*****************************************************************************
// Reference Position Generator for position loop
_iq refPosGen(_iq out)
{
    _iq in = posArray[ptr1];

    out = ramper(in, out, posSlewRate);

    if (in == out)
        if (++cntr1 > 1000)
        {
            cntr1 = 0;
            if (++ptr1 >= ptrMax)
                ptr1 = 0;
        }
    return (out);
}

//*****************************************************************************
//Toggle a GPIO pin
void GPIO_TogglePin(Uint16 pin)
{
    volatile Uint32 *gpioDataReg;
    Uint32 pinMask;

    gpioDataReg = (volatile Uint32 *)&GpioDataRegs + (pin/32)*GPY_DATA_OFFSET;
    pinMask = 1UL << (pin % 32);

    gpioDataReg[GPYTOGGLE] = pinMask;

    return;
}

//*****************************************************************************
//*****************************************************************************
//*****************************************************************************
//*****************************************************************************

void main(void){

    // Initialize System Control:
    // PLL, WatchDog, enable Peripheral Clocks
    // This function derived from the one found in F2837x_SysCtrl.c file
    InitSysCtrl1();
    EALLOW;
    ClkCfgRegs.LOSPCP.bit.LSPCLKDIV=0x01; // set sci clock by SYSCLK/2 (100MHz)
    EDIS;

    configureDAC(DACA);
    configureDAC(DACB);
    configureDAC(DACC);

    // Waiting for enable flag set
    while (EnableFlag == FALSE)
    {
        BackTicker++;
    }

    // Clear all interrupts and initialize PIE vector table:

    // Disable CPU interrupts

    DINT;

    // Initialize the PIE control registers to their default state.
    // The default state is all PIE interrupts disabled and flags
    // are cleared.
    // This function is found in the F28M3Xx_PieCtrl.c file.
    InitPieCtrl();

    // Disable CPU interrupts and clear all CPU interrupt flags:
    IER = 0x0000;
    IFR = 0x0000;
    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    // This will populate the entire table, even if the interrupt
    // is not used in this example.  This is useful for debug purposes.
    // The shell ISR routines are found in F28M3Xx_DefaultIsr.c.
    // This function is found in F28M3Xx_PieVect.c.
    InitPieVectTable();

    // Configure a temp output pin for flagging (GPIO78)
    GPIO_SetupPinOptions(TEMP_GPIO, GPIO_OUTPUT, GPIO_ASYNC);
    GPIO_SetupPinMux(TEMP_GPIO, 0, TEMP_MUX);

    GPIO_SetupPinOptions(BLUE_LED_GPIO, GPIO_OUTPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(BLUE_LED_GPIO, GPIO_MUX_CPU1, BLUE_LED_MUX);

    // shilin servo on GPIO(16)
    GPIO_SetupPinOptions(SV_ON_GPIO, GPIO_OUTPUT, GPIO_PULLUP);
    GPIO_SetupPinMux(SV_ON_GPIO, GPIO_MUX_CPU1, SV_ON_MUX);
    // servo on
    GPIO_WritePin(SV_ON_GPIO,1);


    // Timing sync for background loops
    // Timer period definitions found in device specific PeripheralHeaderIncludes.h
    CpuTimer0Regs.PRD.all =  10000;     // A tasks
    CpuTimer1Regs.PRD.all =  20000;     // B tasks
    CpuTimer2Regs.PRD.all =  30000;     // C tasks

    // Tasks State-machine init
    Alpha_State_Ptr = &A0;
    A_Task_Ptr = &A1;
    B_Task_Ptr = &B1;
    C_Task_Ptr = &C1;

    // ****************************************************************************
    // ****************************************************************************
    // Set up peripheral assignments for motor control
    // ****************************************************************************
    // ****************************************************************************
    motor1.PwmARegs = &EPwm4Regs;    // set up EPWM for motor 1 phase A
    motor1.PwmBRegs = &EPwm5Regs;    // set up EPWM for motor 1 phase B
    motor1.PwmCRegs = &EPwm6Regs;    // set up EPWM for motor 1 phase C
    motor1.QepRegs  = &EQep2Regs;    // set up QEP # for motor 1
    motor1.SpiRegs  = &SpiaRegs;     // set up SPI for motor 1
    motor1.drvScsPin = MOTOR1_SCS_GPIO; // pin for SPI-drv1 chip select

    // ****************************************************************************
    // ****************************************************************************
    // Initialize EPWM modules for inverter PWM generation
    // ****************************************************************************
    // ****************************************************************************

    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;

    // *****************************************
    // Inverter PWM configuration for motor 1
    // ****************************************
    /* Deadband is set externally on DRV830x chip
     */
    PWM_1ch_UpDwnCnt_CNF(motor1.PwmARegs, INV_PWM_TICKS, INV_PWM_TICKS*0.008);
    PWM_1ch_UpDwnCnt_CNF(motor1.PwmBRegs, INV_PWM_TICKS, INV_PWM_TICKS*0.008);
    PWM_1ch_UpDwnCnt_CNF(motor1.PwmCRegs, INV_PWM_TICKS, INV_PWM_TICKS*0.008);

    // configure Epwms 5 and 6 as slaves
    //  (motor1.PwmBRegs)->TBCTL.bit.SYNCOSEL = TB_SYNC_IN;
    (motor1.PwmBRegs)->TBCTL.bit.PHSEN    = TB_ENABLE;
    (motor1.PwmBRegs)->TBPHS.bit.TBPHS    = 2;
    (motor1.PwmBRegs)->TBCTL.bit.PHSDIR   = TB_UP;

    //  (motor1.PwmCRegs)->TBCTL.bit.SYNCOSEL = TB_SYNC_IN;
    (motor1.PwmCRegs)->TBCTL.bit.PHSEN    = TB_ENABLE;
    (motor1.PwmCRegs)->TBPHS.bit.TBPHS    = 2;
    (motor1.PwmCRegs)->TBCTL.bit.PHSDIR   = TB_UP;

    InitMotor1EPwmGpio();  // Set up GPIOs for EPWMA of 4,5,6

    //---------------------------------------------------------------------------------------

    // Setting up link from EPWM to ADC (EPwm4 is chosen)
    EPwm4Regs.ETSEL.bit.SOCASEL = ET_CTR_PRD; // Select SOC from counter at ctr = PRD
    EPwm4Regs.ETPS.bit.SOCAPRD  = ET_1ST;     // Generate pulse on 1st even
    EPwm4Regs.ETSEL.bit.SOCAEN  = 1;          // Enable SOC on A group



    // ****************************************************************************
    // ****************************************************************************
    //
    // ****************************************************************************
    // ****************************************************************************
    //Configure the ADC and power it up
    ConfigureADC();

    //Select the channels to convert and end of conversion flag

    EALLOW;

    // Analog signals - Motor 1
    // Vdc  ADC (B)14
    // Va   ADC C4
    // Vb   ADC A4
    // Vc   ADC D4
    // Ia   ADC A2 (single) or A2A3 (differential)
    // Ib   ADC C2 (single) or C2C3 (differential)
    // Ic   ADC D2 (single) or D2D3 (differential)

    // On piccolo 133ns for ACQPS
    // hencce ACQPS on soprano is 133/5~30

    // Configure SOCx on ADCs A and B (C and D not used)

    // Motor 1: Temperature  adc @ A3
    // ********************************
//    AdcaRegs.ADCSOC1CTL.bit.CHSEL     =  3;                    // SOC0 will convert pin A3
//    AdcaRegs.ADCSOC1CTL.bit.ACQPS     = 15;                    // sample window in SYSCLK cycles
//    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL   = ADCTRIG11_EPWM4SOCA;   // trigger on ePWM7 SOCA/C
//    // Configure the post processing block (PPB) to eliminate subtraction related calculation
//    AdcaRegs.ADCPPB2CONFIG.bit.CONFIG = 1;                     // PPB is associated with SOC1
//    AdcaRegs.ADCPPB2OFFCAL.bit.OFFCAL = 0;                     // Write zero to this for now till offset ISR is run
//    // Motor 1: Vibration  adc @ B3
//    // ********************************
//    AdcbRegs.ADCSOC1CTL.bit.CHSEL     =  3;                    // SOC0 will convert pin B3
//    AdcbRegs.ADCSOC1CTL.bit.ACQPS     = 15;                    // sample window in SYSCLK cycles
//    AdcbRegs.ADCSOC1CTL.bit.TRIGSEL   = ADCTRIG11_EPWM4SOCA;   // trigger on ePWM7 SOCA/C
//    // Configure the post processing block (PPB) to eliminate subtraction related calculation
//    AdcbRegs.ADCPPB2CONFIG.bit.CONFIG = 1;                     // PPB is associated with SOC1
//    AdcbRegs.ADCPPB2OFFCAL.bit.OFFCAL = 0;                     // Write zero to this for now till offset ISR is run



    // Motor 1: Ia  @ A channel 2, SOC 0
    // ********************************
    AdcaRegs.ADCSOC0CTL.bit.CHSEL     =  2;                    // SOC0 will convert pin A0
    AdcaRegs.ADCSOC0CTL.bit.ACQPS     = 50;                    // sample window in SYSCLK cycles
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL   = ADCTRIG11_EPWM4SOCA;   // trigger on ePWM7 SOCA/C
    // Configure the post processing block (PPB) to eliminate subtraction related calculation
    AdcaRegs.ADCPPB1CONFIG.bit.CONFIG = 0;                     // PPB is associated with SOC0
    AdcaRegs.ADCPPB1OFFCAL.bit.OFFCAL = 0;                     // Write zero to this for now till offset ISR is run

    // Motor 1: Ib  @ C channel 2, SOC 0
    // ********************************
    AdccRegs.ADCSOC0CTL.bit.CHSEL     =  2;                    // SOC0 will convert pin B3
    AdccRegs.ADCSOC0CTL.bit.ACQPS     = 50;                    // sample window in SYSCLK cycles
    AdccRegs.ADCSOC0CTL.bit.TRIGSEL   = ADCTRIG11_EPWM4SOCA;   // trigger on ePWM2 SOCA/C
    // Configure the post processing block (PPB) to eliminate subtraction related calculation
    AdccRegs.ADCPPB1CONFIG.bit.CONFIG = 0;                     // PPB is associated with SOC0
    AdccRegs.ADCPPB1OFFCAL.bit.OFFCAL = 0;                     // Write zero to this for now till offset ISR is run

    //  // Motor 1: Ic  @ A1
    //  // ********************************
    //  AdcaRegs.ADCSOC2CTL.bit.CHSEL     =  3;                    // SOC2 will convert pin A1
    //  AdcaRegs.ADCSOC2CTL.bit.ACQPS     = 30;                    // sample window in SYSCLK cycles
    //  AdcaRegs.ADCSOC2CTL.bit.TRIGSEL   = ADCTRIG11_EPWM4SOCA;   // trigger on ePWM2 SOCA/C
    //  // Configure the post processing block (PPB) to eliminate subtraction related calculation
    //  AdcaRegs.ADCPPB3CONFIG.bit.CONFIG = 2;                     // PPB is associated with SOC2
    //  AdcaRegs.ADCPPB3OFFCAL.bit.OFFCAL = 0;                     // Write zero to this for now till offset ISR is run
    //
      // Motor 1: Va  @ B1
      // ********************************
      AdccRegs.ADCSOC4CTL.bit.CHSEL     =  4;                    // SOC4 will convert pin C4
      AdccRegs.ADCSOC4CTL.bit.ACQPS     = 30;                    // sample window in SYSCLK cycles
      AdccRegs.ADCSOC4CTL.bit.TRIGSEL   = ADCTRIG11_EPWM4SOCA;   // trigger on ePWM2 SOCA/C

      // Motor 1: Vb  @ B4
      // ********************************
      AdcaRegs.ADCSOC4CTL.bit.CHSEL     =  4;                    // SOC3 will convert pin A4
      AdcaRegs.ADCSOC4CTL.bit.ACQPS     = 30;                    // sample window in SYSCLK cycles
      AdcaRegs.ADCSOC4CTL.bit.TRIGSEL   = ADCTRIG11_EPWM4SOCA;   // trigger on ePWM2 SOCA/C

      // Motor 1: Vc  @ B2
      // ********************************
      AdcdRegs.ADCSOC4CTL.bit.CHSEL     =  4;                    // SOC0 will convert pin D4
      AdcdRegs.ADCSOC4CTL.bit.ACQPS     = 30;                    // sample window in SYSCLK cycles
      AdcdRegs.ADCSOC4CTL.bit.TRIGSEL   = ADCTRIG11_EPWM4SOCA;   // trigger on ePWM2 SOCA/C

      //      // Motor 1: Vdc  @ B14
      //      // ********************************
      //      AdcbRegs.ADCSOC6CTL.bit.CHSEL     = 14;                    // SOC6 will convert pin B15
      //      AdcbRegs.ADCSOC6CTL.bit.ACQPS     = 30;                    // sample window in SYSCLK cycles
      //      AdcbRegs.ADCSOC6CTL.bit.TRIGSEL   = ADCTRIG11_EPWM4SOCA;   // trigger on ePWM2 SOCA/C

    // ****************************************************************************
    // ****************************************************************************
    //TODO ISR Mapping
    // ****************************************************************************
    // ****************************************************************************
    // ADC A EOC of SOC1 is used to trigger Motor control Interrupt
    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL  = 0;
    AdcaRegs.ADCINTSEL1N2.bit.INT1CONT = 1;
    AdcaRegs.ADCINTSEL1N2.bit.INT1E    = 1;

    PieVectTable.ADCA1_INT         = &MotorControlISR;
    //  PieVectTable.TIMER0_INT = &cpu_timer0_isr;
    PieCtrlRegs.PIEIER1.bit.INTx1  = 1;  // Enable ADCA1INT in PIE group 1



    // SETUP DAC-C (DACs A, B and C are already used up)

    // PIE for scib
    PieVectTable.SCIB_RX_INT = &scibRxFifoIsr;
    PieCtrlRegs.PIEIER9.bit.INTx3 = 1;   // PIE Group 9, INT3 (scib_rx)
    ScibRegs.SCIFFRX.bit.RXFFINTCLR = 1;

    EDIS;
    //  ConfigCpuTimer(&CpuTimer0, 200, 100000);
    //  CpuTimer0Regs.TCR.all = 0x4000;
    // ****************************************************************************
    // ****************************************************************************
    // Initialize QEP module
    // ****************************************************************************
    // ****************************************************************************
    InitMotor1EQepGpio();               // Init motor 1 QEP GPIOs

    // ****************************************************************************
    // ****************************************************************************
    // Initialize PWMDAC module
    // ****************************************************************************
    // ****************************************************************************
    pwmdac1.PeriodMax = 500;   // @200Mhz: 4500->20kHz, 3000-> 30kHz, 1500->60kHz
    pwmdac1.PwmDacInPointer0 = &PwmDacCh1;
    pwmdac1.PwmDacInPointer1 = &PwmDacCh2;
    pwmdac1.PwmDacInPointer2 = &PwmDacCh3;
    pwmdac1.PwmDacInPointer3 = &PwmDacCh4;

    //  PWMDAC_INIT_MACRO(pwmdac1)
    InitMotor1EPwmDACGpio();  // Set up GPIOs for EPWMA of 7,8

    // ****************************************************************************
    // ****************************************************************************
    // Initialize SPI module for communication with DRV830x
    // ****************************************************************************
    // ****************************************************************************
    //  InitMotor1SpiGpio();                    // Init motor 1 SPI GPIOs
    //  InitSpiRegs_DRV830x(motor1.SpiRegs);    // Init SPI regs
    //    #if (MOTOR1_DRV == DRV8301)
    //      InitDRV8301Regs(&motor1);           // Init DRV regs' mirror variables
    //    #else
    //      InitDRV8305Regs(&motor1);
    //    #endif

    // ****************************************************************************
    // ****************************************************************************
    // Initialise DRV830x interface GPIOs
    // ****************************************************************************
    // ****************************************************************************
    InitMotor1_DRV_Gpio();                   // DRV init for motor 1


    //   InitMotor1SpiGpio();
//    SPIDAC_setup();
    InitEcapGpio();
    ConfigInput_XBAR();
    SetEcapReg();
    InitsciGpio();
    scib_loopback_init();               // Initialize SCI for digital loop back
    scib_fifo_init();                   // Initialize the SCI FIFO
    setup_emif1_pinmux_async_16bit(0);  //setup EMIF Gpio
    config_device_number_Gpio();        // set up device number GPIO
    device_number= set_device_number();



    bool EMIF_success = (setup_EMIF() == 0);

    ECAP_PU = ECAP_FREQ/PWM_SAMPLING;


    // ****************************************************************************
    // ****************************************************************************
    // Initialise DRV830x
    // ****************************************************************************
    // ****************************************************************************
    GPIO_WritePin(MOTOR1_EN_GATE_GPIO, 1);  // Enable DRV
    DELAY_US(50000);                        // delay to allow DRV830x supplies to ramp up

    //#if (MOTOR1_DRV == DRV8301)
    //  InitDRV8301(&motor1);
    //  while (motor1.drv8301.DRV_fault) ;  // hang on if drv init is faulty
    //#else
    //  InitDRV8305(&motor1);
    //  while (motor1.drv8305.DRV_fault) ;  // hang on if drv init is faulty
    //#endif

    // ****************************************************************************
    // ****************************************************************************
    // Paramaeter Initialisation
    // ****************************************************************************
    // ****************************************************************************

    // Init QEP parameters
    motor1.qep.LineEncoder = 1024; // these are the number of slots in the QEP encoder
    motor1.qep.MechScaler  = _IQ30(0.25/motor1.qep.LineEncoder);
    motor1.qep.PolePairs   = POLES/2;
    motor1.qep.CalibratedAngle = 0;
    QEP_INIT_MACRO(motor1.QepRegs,motor1.qep)
    (motor1.QepRegs)->QEPCTL.bit.IEI = 0;        // disable POSCNT=POSINIT @ Index

    // Initialize the Speed module for speed calculation from QEP
    motor1.speed.K1 = _IQ21(1/(BASE_FREQ*motor1.T));
    motor1.speed.K2 = _IQ(1/(1+motor1.T*2*PI*5));      // Low-pass cut-off frequency
    motor1.speed.K3 = _IQ(1)-motor1.speed.K2;
    motor1.speed.BaseRpm = 120*(BASE_FREQ/POLES);

    // Initialize the Speed module for speed calculation from Flux
    Speed_F.K1 = _IQ21(1/(BASE_FREQ*motor1.T));
    Speed_F.K2 = _IQ(1/(1+motor1.T*2*PI*5));      // Low-pass cut-off frequency
    Speed_F.K3 = _IQ(1)-Speed_F.K2;
    Speed_F.BaseRpm = 120*(BASE_FREQ/POLES);
    // Initialize the Speed module for speed calculation from Voltage
    Speed_V.K1 = _IQ21(1/(BASE_FREQ*motor1.T));
    Speed_V.K2 = _IQ(1/(1+motor1.T*2*PI*5));      // Low-pass cut-off frequency
    Speed_V.K3 = _IQ(1)-Speed_F.K2;
    Speed_V.BaseRpm = 120*(BASE_FREQ/POLES);

    // Initialize the RAMPGEN module
    motor1.rg.StepAngleMax = _IQ(BASE_FREQ*motor1.T);

    // Initialize the PI module for position
    motor1.pi_pos.Kp = _IQ(1.0);            //_IQ(10.0);
    motor1.pi_pos.Ki = _IQ(0.001);          //_IQ(motor1.T*SpeedLoopPrescaler/0.3);
    motor1.pi_pos.Umax = _IQ(1.0);
    motor1.pi_pos.Umin = _IQ(-1.0);

    //    // Initialize the PID module for position (alternative option for eval)
    //    motor1.pid_pos.Ref = 0;
    //    motor1.pid_pos.Fdb = 0;
    //    motor1.pid_pos.OutMin = _IQ(-0.5);
    //    motor1.pid_pos.OutMax = _IQ(0.5);
    //    motor1.pid_pos.Out = 0;
    //
    //    motor1.pid_pos.Kp = 1.0;
    //    motor1.pid_pos.Ki = 0;
    //    motor1.pid_pos.Kd = 0;
    //    motor1.pid_pos.Kc = 0.9;
    //
    //    motor1.pid_pos.Up1 = 0;
    //    motor1.pid_pos.Up  = 0;
    //    motor1.pid_pos.Ui  = 0;
    //    motor1.pid_pos.Ud  = 0;
    //    motor1.pid_pos.SatErr    = 0;
    //    motor1.pid_pos.OutPreSat = 0;

    // Initialize the PID module for speed
#if (BUILDLEVEL==LEVEL5)
    //  motor1.pid_spd.param.Kp=_IQ(2.5);
    //  motor1.pid_spd.param.Ki=_IQ(0.0001);
    //  motor1.pid_spd.param.Kd=_IQ(0.0);
    //  motor1.pid_spd.param.Kr=_IQ(1.0);
    //  motor1.pid_spd.param.Umax=_IQ(0.9);
    //  motor1.pid_spd.param.Umin=_IQ(-0.9);
    motor1.pid_spd.param.Kp=_IQ(1.0);
    motor1.pid_spd.param.Ki=_IQ(0.001);
    motor1.pid_spd.param.Kd=_IQ(0.0);
    motor1.pid_spd.param.Kr=_IQ(1.0);
    motor1.pid_spd.param.Umax=_IQ(0.95);
    motor1.pid_spd.param.Umin=_IQ(-0.95);

#else
    motor1.pid_spd.param.Kp   = _IQ(2.0);
    motor1.pid_spd.param.Ki   = _IQ(0.01);
    motor1.pid_spd.param.Kd   = _IQ(0.0);
    motor1.pid_spd.param.Kr   = _IQ(1.0);
    motor1.pid_spd.param.Umax = _IQ(0.95);
    motor1.pid_spd.param.Umin = _IQ(-0.95);
#endif

    // Init PI module for ID loop
    motor1.pi_id.Kp   = _IQ(1.0);//_IQ(3.0);
    motor1.pi_id.Ki   = _IQ(motor1.T/0.04);//0.0075);
    motor1.pi_id.Umax = _IQ(0.5);
    motor1.pi_id.Umin = _IQ(-0.5);

    // Init PI module for IQ loop
    motor1.pi_iq.Kp   = _IQ(1.0);//_IQ(4.0);
    motor1.pi_iq.Ki   = _IQ(motor1.T/0.04);//_IQ(0.015);
    motor1.pi_iq.Umax = _IQ(0.8);
    motor1.pi_iq.Umin = _IQ(-0.8);

    // Set mock REFERENCES for Speed and Iq loops
    motor1.SpeedRef = 0.05;

    motor1.IqRef = _IQ(0.0);

    // Init FLAGS
    motor1.RunMotor = 1;

    // Init PI module for ID loop
    Lamda_d.Kp   = _IQ(339.672894);//_IQ(3.0);//137.27965//339.672894
    Lamda_d.Ki   = _IQ(0.0029);//0.0075);//0.00113985//0.002856
    Lamda_d.Umax = _IQ(1000);
    Lamda_d.Umin = _IQ(-1000);

    // Init PI module for IQ loop
    Lamda_q.Kp   = _IQ(339.672894);//_IQ(4.0);
    Lamda_q.Ki   = _IQ(0.0029);//_IQ(0.015);
    Lamda_q.Umax = _IQ(1000);
    Lamda_q.Umin = _IQ(-1000);

    // Init RLS parameter
    //   RLS_Ls.forgetting_coef = _IQ(0.999);//0.999
    //   RLS_Ls.forgetting_coefinv = _IQ(1.001001001001001);//_IQ(1.001001001)
    //   RLS_Rs.forgetting_coef = _IQ(0.99);//0.999
    //   RLS_Rs.forgetting_coefinv = _IQ(1.0101010101);//_IQ(1.001001001)
    //   RLS_Lamda.forgetting_coef = _IQ(0.99);//0.999
    //   RLS_Lamda.forgetting_coefinv = _IQ(1.010101010101);//_IQ(1.001001001)

    //   RLS_Ls.Pl_last = _IQ(1);
    //   RLS_Rs.Pl_last = _IQ(1);
    //   RLS_Lamda.Pl_last = _IQ(1);
    //
    //   RLS_Ls.parameter_last = _IQ(0.001);
    //   RLS_Rs.parameter_last = _IQ(0.1);
    //   RLS_Lamda.parameter_last = _IQ(0.01);
    //
    //   RLS_Ls.parameter = _IQ(0.001);
    //   RLS_Rs.parameter = _IQ(0.1);
    //   RLS_Lamda.parameter = _IQ(0.01);



    //  Note that the vectorial sum of d-q PI outputs should be less than 1.0 which
    //  refers to maximum duty cycle for SVGEN. Another duty cycle limiting factor
    //  is current sense through shunt resistors which depends on hardware/software
    //  implementation. Depending on the application requirements 3,2 or a single
    //  shunt resistor can be used for current waveform reconstruction. The higher
    //  number of shunt resistors allow the higher duty cycle operation and better
    //  dc bus utilization. The users should adjust the PI saturation levels
    //  carefully during open loop tests (i.e pi_id.Umax, pi_iq.Umax and Umins) as
    //  in project manuals. Violation of this procedure yields distorted  current
    // waveforms and unstable closed loop operations which may damage the inverter.

    // ****************************************************
    // Initialize DATALOG module
    // ****************************************************
    DLOG_4CH_F_init(&dlog_4ch1);
    dlog_4ch1.input_ptr1 = &DlogCh1;    //data value
    dlog_4ch1.input_ptr2 = &DlogCh2;
    dlog_4ch1.input_ptr3 = &DlogCh3;
    dlog_4ch1.input_ptr4 = &DlogCh4;
    dlog_4ch1.output_ptr1 = &DBUFF_4CH1[0];
    dlog_4ch1.output_ptr2 = &DBUFF_4CH2[0];
    dlog_4ch1.output_ptr3 = &DBUFF_4CH3[0];
    dlog_4ch1.output_ptr4 = &DBUFF_4CH4[0];
    dlog_4ch1.size = 200;
    dlog_4ch1.pre_scalar = 1;
    dlog_4ch1.trig_value = 0.01;
    dlog_4ch1.status = 2;

    // ****************************************************************************
    // ****************************************************************************
    // Call DMC Protection function
    // ****************************************************************************
    // ****************************************************************************
    DMC1_Protection();


    // ****************************************************************************
    // ****************************************************************************
    // Feedbacks OFFSET Calibration Routine
    // ****************************************************************************
    // ****************************************************************************
#if (MOTOR1_DRV == DRV8301)
    GPIO_WritePin(MOTOR1_DC_CAL_GPIO,  0);  // Set DC-CAL to 0 to enable current amplifiers
#endif

    DELAY_US(5);                            // delay to allow DRV830x amplifiers to settle

    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;
    EDIS;

    motor1.offset_shntA = 0;
    motor1.offset_shntB = 0;
    motor1.offset_shntC = 0;

    for (OffsetCalCounter=0; OffsetCalCounter<20000; )
    {
        if(EPwm4Regs.ETFLG.bit.SOCA==1)
        {
            if(OffsetCalCounter>1000)
            {
                motor1.offset_shntA = K1*motor1.offset_shntA + K2*(IFB_A1)*ADC_PU_SCALE_FACTOR;     //Mtr1 : Phase A offset
                motor1.offset_shntB = K1*motor1.offset_shntB + K2*(IFB_B1)*ADC_PU_SCALE_FACTOR;     //Mtr1 : Phase B offset
                motor1.offset_shntC = K1*motor1.offset_shntC + K2*(IFB_C1)*ADC_PU_SCALE_FACTOR;     //Mtr1 : Phase C offset
            }
            EPwm4Regs.ETCLR.bit.SOCA=1;
            OffsetCalCounter++;
        }
    }

    // ********************************************
    // Init OFFSET regs with identified values
    // ********************************************
    EALLOW;

    AdcaRegs.ADCPPB1OFFREF = motor1.offset_shntA*4096.0;      // set shunt Iu1 offset
    AdcbRegs.ADCPPB1OFFREF = motor1.offset_shntB*4096.0;      // set shunt Iv1 offset
    AdcaRegs.ADCPPB2OFFREF = motor1.offset_shntA*4096.0;      // set shunt Iu1 offset
    AdcbRegs.ADCPPB2OFFREF = motor1.offset_shntB*4096.0;      // set shunt Iv1 offset
    AdcaRegs.ADCPPB3OFFREF = motor1.offset_shntC*4096.0;      // set shunt Iw1 offset

    EDIS;

    // ****************************************************************************
    // ****************************************************************************
    //TODO Enable Interrupts
    // ****************************************************************************
    // ****************************************************************************



    EALLOW;

    IER |= M_INT1; // Enable group 1 interrupts
    IER |= M_INT9; // Enable group 9 interrupts

    EINT;          // Enable Global interrupt INTM
    ERTM;          // Enable Global realtime interrupt DBGM
    EDIS;

    // ***************************************************************************
    //  Initialisations COMPLETE
    //  - IDLE loop. Just loop forever
    // ***************************************************************************
    for(;;)  //infinite loop
    {
        // State machine entry & exit point
        //===========================================================
        (*Alpha_State_Ptr)();   // jump to an Alpha state (A0,B0,...)
        //===========================================================
    }
} //END MAIN CODE
/******************************************************************************
 * ****************************************************************************
 * ****************************************************************************
 * ****************************************************************************
 */

//=================================================================================
//	STATE-MACHINE SEQUENCING AND SYNCRONIZATION FOR SLOW BACKGROUND TASKS
//=================================================================================

void A0(void)
{
    // loop rate synchronizer for A-tasks
    if(CpuTimer0Regs.TCR.bit.TIF == 1)
    {
        CpuTimer0Regs.TCR.bit.TIF = 1;	// clear flag

        //-----------------------------------------------------------
        (*A_Task_Ptr)();		// jump to an A Task (A1,A2,A3,...)
        //-----------------------------------------------------------

        VTimer0[0]++;			// virtual timer 0, instance 0 (spare)
        SerialCommsTimer++;
    }

    Alpha_State_Ptr = &B0;		// Comment out to allow only A tasks //jamjam??
}

void B0(void)
{
    // loop rate synchronizer for B-tasks
    if(CpuTimer1Regs.TCR.bit.TIF == 1)
    {
        CpuTimer1Regs.TCR.bit.TIF = 1;				// clear flag

        //-----------------------------------------------------------
        (*B_Task_Ptr)();		// jump to a B Task (B1,B2,B3,...)
        //-----------------------------------------------------------
        VTimer1[0]++;			// virtual timer 1, instance 0 (spare)
    }

    Alpha_State_Ptr = &C0;		// Allow C state tasks
}

void C0(void)
{
    // loop rate synchronizer for C-tasks
    if(CpuTimer2Regs.TCR.bit.TIF == 1)
    {
        CpuTimer2Regs.TCR.bit.TIF = 1;				// clear flag

        //-----------------------------------------------------------
        (*C_Task_Ptr)();		// jump to a C Task (C1,C2,C3,...)
        //-----------------------------------------------------------
        VTimer2[0]++;			//virtual timer 2, instance 0 (spare)
    }

    Alpha_State_Ptr = &A0;	// Back to State A0
}


//=================================================================================
//	A - TASKS (executed in every 50 usec)
//=================================================================================
//--------------------------------------------------------
void A1(void) // SPARE (not used)
//--------------------------------------------------------
{

    // *******************************************************
    // Motor 1 -- DRV830x protections
    // *******************************************************
    // Check for PWM trip due to over current

    if((motor1.PwmARegs)->TZFLG.bit.OST == 0x1)
    {
        motor1.TripFlagDMC = 1;                  // Trip on DMC (fault trip )
        GPIO_WritePin(MOTOR1_EN_GATE_GPIO, 0);   // de-assert the DRV830x EN_GATE pin
    }

    // If clear cmd received, reset PWM trip
    if (motor1.clearTripFlagDMC)
    {
        GPIO_WritePin(MOTOR1_EN_GATE_GPIO, 1);  // assert the DRV830x EN_GATE pin
        DELAY_US(50000);		                // DRV830x settling time

        motor1.TripFlagDMC = 0;
        motor1.clearTripFlagDMC = 0;

        // clear EPWM trip flags
        EALLOW;
        (motor1.PwmARegs)->TZCLR.bit.OST = 1;
        (motor1.PwmBRegs)->TZCLR.bit.OST = 1;
        (motor1.PwmCRegs)->TZCLR.bit.OST = 1;
        EDIS;
    }

    //-------------------
    //the next time CpuTimer0 'counter' reaches Period value go to A2
    A_Task_Ptr = &A2;
    //-------------------
}

//-----------------------------------------------------------------
void A2(void) // SPARE (not used)
//-----------------------------------------------------------------
{

    //-------------------
    //the next time CpuTimer0 'counter' reaches Period value go to A3
    A_Task_Ptr = &A3;
    //-------------------
}

//-----------------------------------------
void A3(void) // SPARE (not used)
//-----------------------------------------
{

    //-----------------
    //the next time CpuTimer0 'counter' reaches Period value go to A1
    A_Task_Ptr = &A1;
    //-----------------
}



//=================================================================================
//	B - TASKS (executed in every 100 usec)
//=================================================================================

//----------------------------------- USER ----------------------------------------

//----------------------------------------
void B1(void) // SPARE
//----------------------------------------
{

    //-----------------
    //the next time CpuTimer1 'counter' reaches Period value go to B2
    B_Task_Ptr = &B2;
    //-----------------
}

//----------------------------------------
void B2(void) //  SPARE
//----------------------------------------
{

    //-----------------
    //the next time CpuTimer1 'counter' reaches Period value go to B3
    B_Task_Ptr = &B3;
    //-----------------
}

//----------------------------------------
void B3(void) //  SPARE
//----------------------------------------
{

    //-----------------
    //the next time CpuTimer1 'counter' reaches Period value go to B1
    B_Task_Ptr = &B1;
    //-----------------
}


//=================================================================================
//	C - TASKS (executed in every 150 usec)
//=================================================================================

//--------------------------------- USER ------------------------------------------

//----------------------------------------
void C1(void)
//----------------------------------------
{
#if MOTOR1_DRV == DRV8305
    Uint16 tmp1, tmp2;
#endif

    GPIO_TogglePin(TEMP_GPIO);    // general purpose flag

    if (motor1.newCmdDRV)
    {
        //#if (MOTOR1_DRV == DRV8301)
        //		//write to DRV8301 control register 1, returns status register 1
        //		motor1.drv8301.stat_reg1.all = DRV8301_SPI_Write(&motor1, CNTRL_REG_1_ADDR);
        //
        //		//write to DRV8301 control register 2, returns status register 1
        //		motor1.drv8301.stat_reg1.all = DRV8301_SPI_Write(&motor1, CNTRL_REG_2_ADDR);
        //#else
        //		for (tmp1=5; tmp1<= 0xc; tmp1++)
        //		{
        //			if (tmp1 != 8)
        //				tmp2 = DRV8305_SPI_Write(&motor1, tmp1);                //write to DRV8305 control reg @ address 'tmp1';
        //		}
        //#endif
        motor1.newCmdDRV = 0;
    }

    //-----------------
    //the next time CpuTimer2 'counter' reaches Period value go to C2
    C_Task_Ptr = &C2;
    //-----------------

}

//----------------------------------------
void C2(void)
//----------------------------------------
{
    //#if (MOTOR1_DRV == DRV8301)
    //	DRV8301_diagnostics(&motor1);
    //#else
    //	DRV8305_diagnostics(&motor1);
    //#endif

    //-----------------
    //the next time CpuTimer2 'counter' reaches Period value go to C3
    C_Task_Ptr = &C3;
    //-----------------
}


//-----------------------------------------
void C3(void) //  SPARE
//-----------------------------------------
{
    static Uint16 i;

    // LED blinking code
    if (++i >= LedCnt)
    {
        i = 0;
        GPIO_TogglePin(BLUE_LED_GPIO);
    }

    //-----------------
    //the next time CpuTimer2 'counter' reaches Period value go to C1
    C_Task_Ptr = &C1;
    //-----------------
}

#if BUILDLEVEL == LEVEL1
// ****************************************************************************
// ****************************************************************************
//TODO Motor Control ISR - Build level 1
//	  Checks target independent modules, duty cycle waveforms and PWM update
//	  Keep the motors disconnected at this level
// ****************************************************************************
// ****************************************************************************
inline void BuildLevel1(MOTOR_VARS * motor)
{
    // ------------------------------------------------------------------------------
    //  Connect inputs of the RMP module and call the ramp control macro
    // ------------------------------------------------------------------------------
    motor->rc.TargetValue = motor->SpeedRef;
    RC_MACRO(motor->rc)

    // ------------------------------------------------------------------------------
    //  Connect inputs of the RAMP GEN module and call the ramp generator macro
    // ------------------------------------------------------------------------------
    motor->rg.Freq = motor->rc.SetpointValue;
    RG_MACRO(motor->rg)

    // ------------------------------------------------------------------------------
    //  Connect inputs of the INV_PARK module and call the inverse park trans. macro
    //	There are two option for trigonometric functions:
    //  IQ sin/cos look-up table provides 512 discrete sin and cos points in Q30 format
    //  IQsin/cos PU functions interpolate the data in the lookup table yielding higher resolution.
    // ------------------------------------------------------------------------------
    motor->park.Angle   = motor->rg.Out;
    motor->park.Sine    = __sinpuf32(motor->park.Angle);
    motor->park.Cosine  = __cospuf32(motor->park.Angle);

    motor->ipark.Ds     = motor->VdTesting;
    motor->ipark.Qs     = motor->VqTesting;
    motor->ipark.Sine   = motor->park.Sine;
    motor->ipark.Cosine = motor->park.Cosine;
    IPARK_MACRO(motor->ipark)

    // ------------------------------------------------------------------------------
    //  Connect inputs of the SVGEN_DQ module and call the space-vector gen. macro
    // ------------------------------------------------------------------------------
    motor->svgen.Ualpha = motor->ipark.Alpha;
    motor->svgen.Ubeta  = motor->ipark.Beta;
    SVGENDQ_MACRO(motor->svgen)

    // ------------------------------------------------------------------------------
    //  Computed Duty and Write to CMPA register
    // ------------------------------------------------------------------------------
    (motor->PwmARegs)->CMPA.bit.CMPA = (INV_PWM_HALF_TBPRD*motor->svgen.Ta)+INV_PWM_HALF_TBPRD;
    (motor->PwmBRegs)->CMPA.bit.CMPA = (INV_PWM_HALF_TBPRD*motor->svgen.Tb)+INV_PWM_HALF_TBPRD;
    (motor->PwmCRegs)->CMPA.bit.CMPA = (INV_PWM_HALF_TBPRD*motor->svgen.Tc)+INV_PWM_HALF_TBPRD;

    // ------------------------------------------------------------------------------
    //  Connect inputs of the DATALOG module
    // ------------------------------------------------------------------------------
    DlogCh1 = motor->rg.Out;
    DlogCh2 = motor->svgen.Ta;
    DlogCh3 = motor->svgen.Tb;
    DlogCh4 = motor->svgen.Tc;

    SPIDAC_write_dac_channel(0, ((motor1.rg.Out)*4096));//clarke1.As
    SPIDAC_write_dac_channel(1, ((motor1.svgen.Ta)*2047+2048));//clarke1.Bs
    SPIDAC_write_dac_channel(2, ((motor1.svgen.Tb)*2047+2048));//phase_volt[0]
    SPIDAC_write_dac_channel(3, ((motor1.svgen.Tc)*2047+2048));//phase_volt[1]
    SPIDAC_update_all();
    //
    //------------------------------------------------------------------------------
    // Variable display on DAC - not available
    //------------------------------------------------------------------------------

    return;
}
#endif


#if BUILDLEVEL == LEVEL2
// ****************************************************************************
// ****************************************************************************
//TODO Motor Control ISR - - Build level 2
//	  Level 2 verifies
//	     - current sense schems
//         - analog-to-digital conversion - shunt current
//       - clarke/park transformations (CLARKE/PARK)
//       - Position sensor interface
//         - speed estimation
// ****************************************************************************
// ****************************************************************************
inline void BuildLevel2(MOTOR_VARS * motor)
{

    if (communication_mode_active){
        return; // bypass the sampling during communication mode
    }

    //    GPIO_WritePin(66, 1);
    // ------------------------------------------------------------------------------
    // Alignment Routine: this routine aligns the motor to zero electrical angle
    // and in case of QEP also finds the index location and initializes the angle
    // w.r.t. the index location
    // ------------------------------------------------------------------------------
    if(!motor->RunMotor)
        motor->lsw = 0;
    else if (motor->lsw == 0)
    {
        // for restarting from (motor->RunMotor = 0)
        motor->rc.TargetValue =  motor->rc.SetpointValue = 0;

        motor->lsw = 1;   // for QEP, spin the motor to find the index pulse
    } // end else if (lsw=0)

    // ------------------------------------------------------------------------------
    //  Connect inputs of the RMP module and call the ramp control macro
    // ------------------------------------------------------------------------------
    if(motor->lsw==0)motor->rc.TargetValue = 0;
    else motor->rc.TargetValue = motor->SpeedRef;
    RC_MACRO(motor->rc)

    // ------------------------------------------------------------------------------
    //  Connect inputs of the RAMP GEN module and call the ramp generator macro
    // ------------------------------------------------------------------------------
    motor->rg.Freq = motor->rc.SetpointValue;
    RG_MACRO(motor->rg)

    // ------------------------------------------------------------------------------
    //  Measure phase currents, subtract the offset and normalize from (-0.5,+0.5) to (-1,+1).
    //	Connect inputs of the CLARKE module and call the clarke transformation macro
    // LPF the sampled current
    //---------------------------------------------------------------------------------

    if (cur_LPF_en==1){
        Current_lpf.Alpha=Last_current_As-offsetA_avg;
        Current_lpf.Beta =Last_current_Bs-offsetB_avg;
        // EMFtoLPF MACRO is equal to 1/s+wc
        EMFtoLPF(Tsim,current_lpf_BW,&Current_lpf);
        // and then multiply wc to become wc/s+wc
        motor->clarke.As = current_lpf_BW*Current_lpf.Alpha_LPF;
        motor->clarke.Bs = current_lpf_BW*Current_lpf.Beta_LPF;
    }
    else {
        motor->clarke.As = Last_current_As-offsetA_avg;//_IQmpy(motor->currentAs,_IQ(1)); // Phase A curr.
        motor->clarke.Bs = Last_current_Bs-offsetB_avg;//_IQmpy(motor->currentBs,_IQ(1)); // Phase B curr.
    }
    CLARKE_MACRO(motor->clarke)

    //-------------------------------------------------------------------------------
    //  Speed estimator
    //-------------------------------------------------------------------------------
    Speed_F.ElecTheta = F_angle;
    SPEED_FR_MACRO(Speed_F)


    // ------------------------------------------------------------------------------
    //  Connect inputs of the SPEED_FR module and call the speed calculation macro
    // ------------------------------------------------------------------------------
    motor->speed.ElecTheta = motor->ElecTheta;
    SPEED_FR_MACRO(motor->speed)

    // --------------------------------------------------------------------------------
    //   Ecap voltage sensing
    // --------------------------------------------------------------------------------
    //variable of polar correction
    ////    voltage reconstruction
    phase_volt1_past = phase_volt1; // Store the previous value of phase voltage 1
    phase_volt2_past = phase_volt2; // Store the previous value of phase voltage 2
    ref_past1 = ref_volt1;         // Store the previous value of reference voltage 1
    ref_past2 = ref_volt2;         // Store the previous value of reference voltage 2
    ref_past3 = ref_volt3;         // Store the previous value of reference voltage 3

    float duty_numerator = PWM_length;// (_iq)(ECap1Regs.CAP1 + ECap1Regs.CAP2 + ECap1Regs.CAP3 + ECap1Regs.CAP4);
    ref_volt1_dir = _IQdiv((_iq)(ECap1Regs.CAP2 + ECap1Regs.CAP4) , duty_numerator);
    ref_volt2_dir = _IQdiv((_iq)(ECap3Regs.CAP2 + ECap3Regs.CAP4) , duty_numerator);
    ref_volt3_dir = _IQdiv((_iq)(ECap5Regs.CAP2 + ECap5Regs.CAP4) , duty_numerator);
    phase_volt1_dir = _IQdiv((_iq)(ECap2Regs.CAP2 + ECap2Regs.CAP4) , duty_numerator);
    phase_volt2_dir = _IQdiv((_iq)(ECap4Regs.CAP2 + ECap4Regs.CAP4) , duty_numerator);
    phase_volt3_dir = _IQdiv((_iq)(ECap6Regs.CAP2 + ECap6Regs.CAP4) , duty_numerator);

    int integration_mode=1;
    ref_volt1= integration_mode ? ref_volt1_dir : lastPWM_duty(&ref1_cap2_last,&ref1_cap4_last,(_iq)ECap1Regs.CAP2,(_iq)ECap1Regs.CAP4)/PWM_length;
    ref_volt2= integration_mode ? ref_volt2_dir : lastPWM_duty(&ref2_cap2_last,&ref2_cap4_last,(_iq)ECap3Regs.CAP2,(_iq)ECap3Regs.CAP4)/PWM_length;
    ref_volt3= integration_mode ? ref_volt3_dir : lastPWM_duty(&ref3_cap2_last,&ref3_cap4_last,(_iq)ECap5Regs.CAP2,(_iq)ECap5Regs.CAP4)/PWM_length;
    phase_volt1= integration_mode ? phase_volt1_dir : lastPWM_duty(&phase1_cap2_last,&phase1_cap4_last,(_iq)ECap2Regs.CAP2,(_iq)ECap2Regs.CAP4)/PWM_length;
    phase_volt2= integration_mode ? phase_volt2_dir : lastPWM_duty(&phase2_cap2_last,&phase2_cap4_last,(_iq)ECap4Regs.CAP2,(_iq)ECap4Regs.CAP4)/PWM_length;
    phase_volt3= integration_mode ? phase_volt3_dir : lastPWM_duty(&phase3_cap2_last,&phase3_cap4_last,(_iq)ECap6Regs.CAP2,(_iq)ECap6Regs.CAP4)/PWM_length;

    //variable adjustment with speed
    wehz = we_f/2/PI;

    inv1_past = inv1; // Store the previous value of inv1
    inv2_past = inv2; // Store the previous value of inv2
    inv3_past = inv3; // Store the previous value of inv3

    // Check for stable conditions and adjust voltage inversion state for ref_volt1
    if (ref_past1 == ref_volt1 && ref_volt1 < modify1 && trig1 == 1) {
        inv1 = -1;
    } else {
        inv1 = 1;
        if (ref_past2 != ref_volt2) // Detect change in ref_volt2 to reset trigger for ref_volt1
            trig1 = 1;
        if (inv1_past != inv1) // Reset trigger when inversion state changes
            trig1 = 0;
    }

    // Check for stable conditions and adjust voltage inversion state for ref_volt2
    if (ref_past2 == ref_volt2 && ref_volt2 < modify1 && trig2 == 1) {
        inv2 = -1;
    } else {
        inv2 = 1;
        if (ref_past3 != ref_volt3) // Detect change in ref_volt3 to reset trigger for ref_volt2
            trig2 = 1;
        if (inv2_past != inv2) // Reset trigger when inversion state changes
            trig2 = 0;
    }

    // voltage filtering
    voltage_filtering( fabs(phase_volt1_past), &phase_volt1, vol_filter_thres);
    voltage_filtering( fabs(phase_volt1_past), &phase_volt1, vol_filter_thres);


    // Adjust phase voltage direction based on inversion state
    phase_volt1_dir=phase_volt1_dir*inv1;
    phase_volt2_dir=phase_volt2_dir*inv2;
    phase_volt1=phase_volt1*inv1; //V_ab
    phase_volt2=phase_volt2*inv2; //V_bc

    LinetoPhase(phase_volt1,phase_volt2,&Vabc);
    CLARKE_MACRO(Vabc);


    //---------------------------------------------------------------------------------
    // Parameters estimation of Affine Projection Algorithms
    //---------------------------------------------------------------------------------


    omega_r = _IQmpy(BASE_FREQ,_IQmpy(Speed_F.Speed,two_PI));

    if (FAST_enable==1)
    {
        //---------------------------------------------------------------------------------
        // FAST estimate
        //---------------------------------------------------------------------------------
        //    ClarkeV_Alpha = (ipark_Flux.Alpha);
        //    ClarkeV_Beta  = (ipark_Flux.Beta);

        //    ClarkeV_Alpha = (Vabc.Alpha);
        //    ClarkeV_Beta  = (Vabc.Beta);
        //

        // calculate sync speed from v (avoid unstable we )
        // for SFF use
        V_angle = _IQdiv(_IQatan2(Vabc.Beta,Vabc.Alpha),two_PI); // avoid  bad loops
        if(V_angle<_IQ(0.0)) V_angle+=_IQ(1.0);
        if(V_angle>_IQ(1.0)) V_angle-=_IQ(1.0);
        Speed_V.ElecTheta = V_angle;
        SPEED_FR_MACRO(Speed_V)

        // V-IR=EMF
        //---------------------------------------------------------------------------------
        V_A=_IQmpy(Vabc.Alpha,Vdc);
        V_B=_IQmpy(Vabc.Beta,Vdc);

        I_A=_IQmpy(motor->clarke.Alpha,CURRENT);
        I_B=_IQmpy(motor->clarke.Beta,CURRENT);

        // [1/s]*[s/(s+wc)]
        // calculate system speed and corner freq wc
        //---------------------------------------------------------------------------------
        //   we =_IQmpy(_IQmpy((Speed_F.Speed),BASE_FREQ),PI*2);
        we =_IQmpy(_IQmpy((Speed_V.Speed),BASE_FREQ),PI*2);
        we_f = we;
        intgrBW_f = we_f/(PI*2)*coef;
        if(intgrBW_f < CrossFreq) intgrBW_f = CrossFreq;

        intgrBW = intgrBW_f;
        fast_wc = intgrBW_f*(PI*2);

        // FLUX V MODEL   [  flux_v = HPF^-1(LPF( V - I*R ))  ]
        //---------------------------------------------------------------------------------

        EMF1.Alpha=V_A- _IQmpy(I_A,Flux_Rs);
        EMF1.Beta =V_B- _IQmpy(I_B,Flux_Rs);
        EMFtoLPF(Tsim,fast_wc,&EMF1);

        //-----------------------------------------------------------------------------------
        // [(s+wc)/s] compensation parameters
        //-----------------------------------------------------------------------------------

        Flux_comp(we,fast_wc,&mag_comp2,&phase_comp);
        LD_2nd_comp(we,fast_wc,&LD_magcomp,&LD_phase_comp,LD_harm);

        // limitation
        //-------------------------------------------------
        //    Emf_lpf2.Alpha  = _IQ15toIQ(Emf_A_lpf2);
        //    Emf_lpf2.Beta   = _IQ15toIQ(Emf_B_lpf2);

        // function IPARK by theta is equal to *exp(i*theta)
        //    Emf_lpf2.Ds  = _IQmpy((Emf_A_lpf2),mag_comp);//mag_comp
        //    Emf_lpf2.Qs   = _IQmpy((Emf_B_lpf2),mag_comp);//mag_comp
        //    Emf_lpf2.Angle  = theta_comp;
        //    Emf_lpf2.Sine   = __sinpuf32(Emf_lpf2.Angle);
        //    Emf_lpf2.Cosine = __cospuf32(Emf_lpf2.Angle);
        //    IPARK_MACRO(Emf_lpf2)

        //    Emf_ab_lpf.Ds = _IQmpy(Emf_lpf2.Ds,mag_comp);
        //    Emf_ab_lpf.Qs = _IQmpy(Emf_lpf2.Qs,mag_comp);

        //        Emf_D_lpf = _IQsat(Emf_D_lpf,mag_lim,-mag_lim) ;
        //        Emf_Q_lpf = _IQsat(Emf_Q_lpf,mag_lim,-mag_lim) ;

        // magnetic flux decoupling

        Emf_ab_lpf.Ds = _IQmpy(_IQ15toIQ(Emf_A_lpf2),mag_comp2);
        Emf_ab_lpf.Qs = _IQmpy(_IQ15toIQ(Emf_B_lpf2),mag_comp2);
        Emf_ab_lpf.Angle = LD_phase_comp;
        Emf_ab_lpf.Sine   = __sinpuf32(Emf_ab_lpf.Angle);
        Emf_ab_lpf.Cosine = __cospuf32(Emf_ab_lpf.Angle);
        IPARK_MACRO(Emf_ab_lpf)

        // function IPARK by theta is equal to *exp(i*theta)
        Emf_ab_lpf2.Ds = _IQmpy(EMF1.Alpha_LPF,mag_comp2);
        Emf_ab_lpf2.Qs = _IQmpy(EMF1.Beta_LPF,mag_comp2);
        Emf_ab_lpf2.Angle = phase_comp;
        Emf_ab_lpf2.Sine   = __sinpuf32(Emf_ab_lpf2.Angle);
        Emf_ab_lpf2.Cosine = __cospuf32(Emf_ab_lpf2.Angle);
        IPARK_MACRO(Emf_ab_lpf2)

        //-------------------------------------------------------------------------------
        //  Angle estimator
        //-------------------------------------------------------------------------------
        if (VT_enable==1){
            // Use i_beta/i_alpha as virtual encoder
            Flux_A_est_active[0] = motor1.clarke.Beta;
            Flux_B_est_active[0] = -motor1.clarke.Alpha;
        }
        else {
            Flux_A_est_active[0] = Emf_ab_lpf2.Alpha - _IQmpy(Ls,(I_A));
            Flux_B_est_active[0] = Emf_ab_lpf2.Beta - _IQmpy(Ls,(I_B));
        }

        //    SFFA= (2/T)^2, SFFB=epsilon*2/T, SFFC=w0^2
        SFFA=4/motor1.T/motor1.T;SFFB=epsilon*2/motor1.T;
        SFFC=(Speed_V.Speed*two_PI*BASE_FREQ)*(Speed_V.Speed*BASE_FREQ*two_PI);
        SFF(SFFA,SFFB,SFFC,Flux_A_est_active,Flux_A_est_active_SFF);
        SFF(SFFA,SFFB,SFFC,Flux_B_est_active,Flux_B_est_active_SFF);

        if (SFF_en==0)
            F_angle = _IQdiv(_IQatan2(Flux_B_est_active[0],Flux_A_est_active[0]),two_PI)+cFangle;
        else   F_angle =_IQdiv(_IQatan2(Flux_B_est_active_SFF[0],Flux_A_est_active_SFF[0]),two_PI)+cFangle;
        //  }
        if(F_angle<_IQ(0.0)) F_angle+=_IQ(1.0);
        if(F_angle>_IQ(1.0)) F_angle-=_IQ(1.0);
        //-------------------------------------------------------------------------------
        //  calculate the rotor frame components
        //-------------------------------------------------------------------------------
        FAST_Cur.Alpha  = motor->clarke.Alpha;
        FAST_Cur.Beta   = motor->clarke.Beta;
        FAST_Cur.Angle  = F_angle;
        FAST_Cur.Sine   = __sinpuf32(FAST_Cur.Angle);
        FAST_Cur.Cosine = __cospuf32(FAST_Cur.Angle);
        PARK_MACRO(FAST_Cur)

        idr = _IQmpy(FAST_Cur.Ds,CURRENT);
        iqr = _IQmpy(FAST_Cur.Qs,CURRENT);

        Vdr = _IQmpy(Park_Flux.Ds,Vdc);
        Vqr = _IQmpy(Park_Flux.Qs,Vdc);

        //-------------------------------------------------------------------------------
        //  magnet diagnosis start
        //-------------------------------------------------------------------------------
        // PM flux decoupling
        mag_flux= MAG_FLUX( Emf_ab_lpf2.Alpha, Emf_ab_lpf2.Beta,  iqr,  idr,  Lq,  Ld , F_angle);
        // feature extraction for demag signal
        DEMAG_Feature(UD_feature,LD_feature, mag_flux ,LD_harm, F_angle,LD_phase_comp);


        //  current model of flux observer
        //-------------------------------------------------------------------------------------
        // record last data for Gopinath observer
        Flux_Cur_Alpha_last = Flux_Cur.Alpha;
        Flux_Cur_Beta_last  = Flux_Cur.Beta;

        //flux of current model
        Flux_Cur.Ds = _IQmpy(Ld,idr)+Lamda_m;
        Flux_Cur.Qs = _IQmpy(Lq,iqr);
        //offset theta3
        F_angle3 = _IQdiv(_IQatan2(Flux_Cur.Qs ,Flux_Cur.Ds),two_PI);

        //transform to alpha,beta axis
        Flux_Cur.Angle = F_angle+F_angle3;
        Flux_Cur.Sine   = __sinpuf32(Flux_Cur.Angle);
        Flux_Cur.Cosine = __cospuf32(Flux_Cur.Angle);
        IPARK_MACRO(Flux_Cur)

        //---------------------------------------------------------------------------------
        // Gopinath-style stator flux Observer
        //---------------------------------------------------------------------------------
        Lamda_d.Ref = Flux_Cur_Alpha_last;//ref
        Lamda_d.Fbk = Lamda_df;//fbk
        PI_MACRO(Lamda_d)

        Lamda_q.Ref = Flux_Cur_Beta_last;//ref
        Lamda_q.Fbk = Lamda_qf;//fbk
        PI_MACRO(Lamda_q)
        //out
        vd_flux = Lamda_d.Out;
        vq_flux = Lamda_q.Out;
        // 1/s with ZOH, y(k)=Tx(k-1)+y(k-1)
        Lamdac_d = _IQmpy(vd_flux_last,Tsim) + Lamdac_d_last;
        Lamdac_q = _IQmpy(vq_flux_last,Tsim) + Lamdac_q_last;
        // combination of two model
        Lamda_df = Lamdac_d + Emf_ab_lpf2.Alpha;
        Lamda_qf = Lamdac_q + Emf_ab_lpf2.Beta;
        //record y(k-1)
        Lamdac_d_last =Lamdac_d;
        Lamdac_q_last =Lamdac_q;
        //record x(k-1)
        vd_flux_last = vd_flux;
        vq_flux_last = vq_flux;

        //--------------------------------------------------------------------------------------
        // Torque estimate
        //--------------------------------------------------------------------------------------
        // mechanical speed
        Speed_est =_IQtoIQ15(Speed_F.Speed);
        we_mech = _IQ15div(_IQ15mpy(_IQ15mpy(_IQ15mpy(Speed_est,BASE_FREQ),_IQ15(2)),PI),_IQtoIQ15(POLE_PAIRS));
        // voltage model
        Torq_est=1000*_IQmpy(_IQmpy(_IQ15(1.5),POLE_PAIRS),(_IQmpy(Emf_ab_lpf2.Alpha,I_B)-_IQmpy(Emf_ab_lpf2.Beta,I_A)))-_IQmpy(_IQ(B_damp),we_mech);

        // current model
        //    Torq_est_cur_mtwo = _IQmpy(_IQmpy(_IQ(1.5),POLE_PAIRS),(_IQmpy(Flux_Cur.Ds,iqr)-_IQmpy(Flux_Cur.Qs,idr)));
        Torq_est_cur_mtwo = 1000*_IQmpy(_IQmpy(_IQ(1.5),POLE_PAIRS),(_IQmpy(Flux_Cur.Alpha,I_B)-_IQmpy(Flux_Cur.Beta,I_A)));

        // combined model
        Torque_DB = 1000*_IQmpy(_IQmpy(_IQ(1.5),POLE_PAIRS),(_IQmpy(Lamda_df,I_B)-_IQmpy(Lamda_qf,I_A)))-_IQmpy(_IQ(B_damp),we_mech);

        // power estimate
//        Power1=(Vabc.Alpha*motor1.clarke.Alpha+Vabc.Beta*motor1.clarke.Beta)*Vdc*CURRENT;

        Power1=2*(V_A*I_A+V_B*I_B)+V_A*I_B+V_B*I_A;



        //-------------------------------------------------------------------------------
        //  averaging section
        //-------------------------------------------------------------------------------
        UD_mag=_IQmag(UD_feature[0],UD_feature[1]);

        if (avg_count1<avg_limit1){avg_count1+=1;
        avg_load1=0;}
        else {avg_load1=1;
        avg_count1=0;}

        average(&UD_feature_avg[0], &UD_feature_avg_last[0], UD_feature[0],avg_load1,avg_limit1);
        average(&UD_feature_avg[1], &UD_feature_avg_last[1], UD_feature[1],avg_load1,avg_limit1);
        average(&LD_feature_avg[0], &LD_feature_avg_last[0], LD_feature[0],avg_load1,avg_limit1);
        average(&LD_feature_avg[1], &LD_feature_avg_last[1], LD_feature[1],avg_load1,avg_limit1);
        average(&flux_energy, &flux_energy_last,_IQmag(mag_flux.Alpha,mag_flux.Beta), avg_load1,avg_limit1);
        average(&avg_test, &avg_test_last,UD_mag, avg_load1,avg_limit1);
        average(&avg_temp, &avg_temp_last,systemp, avg_load1,avg_limit1);// average the measured temperature
        average(&Power1_avg, &Power1_avg_last,Power1, avg_load1,avg_limit1);
        rms(&rms_test, &rms_last,phase_volt1*Vdc,avg_load1,avg_limit1);

        if (cur_offset_complete==0){
            average(&offsetA_sum, &offsetA_avg,motor1.currentAs, avg_load1,avg_limit1);
            average(&offsetB_sum, &offsetB_avg,motor1.currentBs, avg_load1,avg_limit1);
        }
        if(avg_load1==1)
            cur_offset_complete=1;// stop current offset average

        LD_mag=_IQmpy(LD_magcomp,_IQmag(LD_feature_avg_last[0],LD_feature_avg_last[1]));

        HI_DEMAG( HI_demag, lumda_h,  avg_test_last , LD_mag,_IQ(0));

        if (torq_i<torq_count){torq_i+=1;
        avg_load2=0;}
        else {avg_load2=1;
        torq_i=0;}
        average(&Torq_est_sum, &Torq_est_avg, Torq_est,avg_load2, torq_count);
        average(&Torq_est_cur_mtwo_sum, &Torq_est_cur_mtwo_avg, Torq_est_cur_mtwo,avg_load2,torq_count);
        average(&Torque_DB_sum, &Torque_DB_avg, Torque_DB,avg_load2,torq_count);

        //---------------------------------------------------------------------------------
        // winding fault detection
        //---------------------------------------------------------------------------------
         // winding fault detection by current harmonic
         park_cn.Alpha  = motor->clarke.Alpha;
         park_cn.Beta   = motor->clarke.Beta;
         //translation by -theta_e
         park_cn.Angle  = -F_angle;
         park_cn.Sine   = __sinpuf32(park_cn.Angle);
         park_cn.Cosine = __cospuf32(park_cn.Angle);
         PARK_MACRO(park_cn)
         // avg counter for cn detection

         if (avg_count_cn<avg_limit_cn){avg_count_cn+=1;
         avg_load1_cn=0;}
         else {avg_load1_cn=1;
         avg_count_cn=0;}
         // -1 harmonic projecting in x-axis
         average(&parkDs_cn_sum, &parkDs_cn_avg, park_cn.Ds,avg_load1_cn,avg_limit_cn);
         // -1 harmonic projecting in y-axis
         average(&parkQs_cn_sum, &parkQs_cn_avg, park_cn.Qs,avg_load1_cn,avg_limit_cn);
         // -1 harmonic magnitude
         parkdq_cn_avg=_IQmag(parkQs_cn_avg,parkDs_cn_avg);

         // calculating current magnitude
         average(&current_mag_sum, &current_mag_avg, _IQmag(park_cn.Alpha,park_cn.Beta),avg_load1_cn,avg_limit_cn);
         // cn fault diagnosis
         CN_Fault(&cn_err_code, cn_valve, current_mag_avg, parkDs_cn_avg, parkQs_cn_avg, parkdq_cn_avg );

        //-------------------------------------------------------------------------------
        //  data updating and transmission
        //-------------------------------------------------------------------------------
        if (update_FAST_en){

        // circularly update the data buffer
        int time_idx=IsrTicker%500;
        temp_test=(float)time_idx/500;
//        temp_test=_IQcosPU(temp_test);
        temp_data1= (Uint16) (_IQcosPU(temp_test)*32768+32768);
        temp_data2= (Uint16) (_IQsinPU(temp_test)*32768+32768);
        // data buffer 1-4 for RUL
        updateCircularBuffer(AsramData.data1, MAX_EMIF_length, &head1, (Uint16)(Vabc.Alpha*32768+32768) );
        updateCircularBuffer(AsramData.data2, MAX_EMIF_length, &head2, (Uint16)(Vabc.Beta*32768+32768));
        updateCircularBuffer(AsramData.data3, MAX_EMIF_length, &head3, (Uint16)(motor->clarke.Alpha*32768+32768));
        updateCircularBuffer(AsramData.data4, MAX_EMIF_length, &head4, (Uint16)(motor->clarke.Beta*32768+32768));
        //data buffer 5-6 for FAST
        updateCircularBuffer(AsramData.data5, MAX_EMIF_length, &head5, (Uint16)(mag_flux.Alpha*32768+32767));
        updateCircularBuffer(AsramData.data6, MAX_EMIF_length, &head6, (Uint16)(mag_flux.Beta*32768+32767));

        //Updating the rs485 package blocks

        // first block data (FAST result and crc1)
        crc1=(Speed_V.Speed)*32768+32768;// crc1 the first send data
        crc2=~(crc1 & 0xFFFF);// setup the crc2 code, crc1+crc2 should be 0xFFF
        first_and_last_data_block[0]=(Uint16)crc1;
        first_and_last_data_block[1]=(Uint16)1234;   //Torque, Torq_est_avg*32768+32768
        first_and_last_data_block[2]=(Uint16)5679;   //Power1, Power1*32768+32768
        first_and_last_data_block[3]=(Uint16)9101;   //efficiency, efficiency*32768+32768

        // last block data (Diagnosis result and crc2)
        first_and_last_data_block[4]=2468;   // PM flux magnitude
        first_and_last_data_block[5]=10121;  // short circuit index_X
        first_and_last_data_block[6]=16182;  // short circuit index_Y
        first_and_last_data_block[7]=crc2;

        }

        // ------------------------------------------------------------------------------
        //  Connect inputs of the PARK module and call the park trans. macro
        // ------------------------------------------------------------------------------
        motor->park.Alpha  = motor->clarke.Alpha;
        motor->park.Beta   = motor->clarke.Beta;
        motor->park.Angle  = motor->rg.Out;
        motor->park.Sine   = __sinpuf32(motor->park.Angle);
        motor->park.Cosine = __cospuf32(motor->park.Angle);
        PARK_MACRO(motor->park)


        // ------------------------------------------------------------------------------
        //	Connect inputs of the INV_PARK module and call the inverse park trans. macro
        // ------------------------------------------------------------------------------
        motor->ipark.Ds = motor->VdTesting;
        motor->ipark.Qs = motor->VqTesting;

        motor->ipark.Sine   = motor->park.Sine;
        motor->ipark.Cosine = motor->park.Cosine;
        IPARK_MACRO(motor->ipark)

        // ------------------------------------------------------------------------------
        //  Connect inputs of the SVGEN_DQ module and call the space-vector gen. macro
        // ------------------------------------------------------------------------------
        motor->svgen.Ualpha = motor->ipark.Alpha;
        motor->svgen.Ubeta  = motor->ipark.Beta;
        SVGENDQ_MACRO(motor->svgen)

        // ------------------------------------------------------------------------------
        //  Computed Duty and Write to CMPA register
        // ------------------------------------------------------------------------------
        (motor->PwmARegs)->CMPA.bit.CMPA = 3000;//(INV_PWM_HALF_TBPRD/2*motor->svgen.Ta)+INV_PWM_HALF_TBPRD/2;
        (motor->PwmBRegs)->CMPA.bit.CMPA = (INV_PWM_HALF_TBPRD/2*motor->svgen.Tb)+INV_PWM_HALF_TBPRD/2;
        (motor->PwmCRegs)->CMPA.bit.CMPA = (INV_PWM_HALF_TBPRD/2*motor->svgen.Tc)+INV_PWM_HALF_TBPRD/2;

        // ------------------------------------------------------------------------------
        //    Connect inputs of the DATALOG module
        // ------------------------------------------------------------------------------
        //	if(fst_count==1){
        //	count_d=dlog_4ch1.count;
        //	fst_count=0;}

        DlogCh1 = (float)acc_y;  //ClarkeV_Alpha+48
        DlogCh2 = (float)Vabc.Beta;   //  motor->svgen.Ta;      //ipark_Flux.Beta+48
        DlogCh3 = (float)I_A;
        DlogCh4 = (float)I_B;

        // ------------------------------------------------------------------------------
        //    Call the DATALOG update function.
        // ------------------------------------------------------------------------------
        DLOG_4CH_F_FUNC(&dlog_4ch1);

        //	index1=dlog_4ch1.count+200;
        //    index2=dlog_4ch1.count+400;

        SPIDAC_write_dac_channel(0, (phase_volt1_dir*scale*2047+2048));//Torq_est_avg*2.44140625+0.05)*4096 mag_flux.Alpha
        SPIDAC_write_dac_channel(1, (phase_volt1*scale*2047+2048));//Torq_est_cur_mone_avg*2.44140625+0.05)*4096
        SPIDAC_write_dac_channel(2, (F_angle*scale1*2047+2048));//local_demag_avg_last_A,descend_trig1
        SPIDAC_write_dac_channel(3, (Flux_Cur.Ds*scale1*2047+2048));// motor1.qep.ElecTheta Flux_mag.Alpha Vabc.Alpha
        SPIDAC_update_all();//motor1.clarke.As,phase_volt1,mag_flux.Beta
        // ------------------------------------------------------------------------------
        //    Updating current
        // ------------------------------------------------------------------------------
        // last step current
        Last_current_As=motor->currentAs;
        Last_current_Bs=motor->currentBs;
    }


    //------------------------------------------------------------------------------
    // Variable display on DAC - DAC-C not available
    //------------------------------------------------------------------------------

    switch (dac_out) {
        case 1:
            dacAval = (Uint16) _IQtoQ11(scale * motor1.currentAs + _IQ(1));
            dacBval = (Uint16) _IQtoQ11(scale * motor1.currentBs + _IQ(1));
            break;

        case 2:
            dacAval = (Uint16) _IQtoQ11(1 * phase_volt1 + _IQ(1));
            dacBval = (Uint16) _IQtoQ11(1 *scale * motor->clarke.Beta + _IQ(1));
            break;

        case 3:
            dacAval = (Uint16) _IQtoQ11(ref_volt1_dir + _IQ(1));
            dacBval = (Uint16) _IQtoQ11(phase_volt1 + _IQ(1));
            break;

        case 4:
            dacAval = (Uint16) _IQtoQ11(scale * mag_flux.Alpha + _IQ(1));
            dacBval = (Uint16) _IQtoQ11(scale * mag_flux.Beta + _IQ(1));
            break;

        case 5:
            dacAval = (Uint16) _IQtoQ11(scale * Emf_ab_lpf2.Alpha + _IQ(1));
            dacBval = (Uint16) _IQtoQ11(scale * I_B + _IQ(1));
            break;

        case 6:
            dacAval = (Uint16) _IQtoQ11(scale * EMF1.Alpha + _IQ(1));
            dacBval = (Uint16) _IQtoQ11(scale * EMF1.Beta + _IQ(1));
            break;

        case 7:
            dacAval = (Uint16) _IQtoQ11(2 * phase_volt1 + _IQ(1));
            dacBval = (Uint16) _IQtoQ11(_IQabs(0.001 * Torque_DB_avg));
            break;

        case 8:
            dacAval = (Uint16) _IQtoQ11(scale * park_cn.Alpha+ _IQ(1));
            dacBval = (Uint16) _IQtoQ11(scale * motor->clarke.Alpha+ _IQ(1));
            break;

        case 9:
            dacAval = (Uint16) _IQtoQ11(scale * park_cn.Qs+ _IQ(1));
            dacBval = (Uint16) _IQtoQ11(scale * parkQs_cn_avg+ _IQ(1));
            break;

        default:
            dacAval = (Uint16) _IQtoQ11(Vabc.Alpha + _IQ(1));
            dacBval = (Uint16) _IQtoQ11(F_angle + _IQ(1));
            break;
    }

    //------------------------------------------------------------------------------
    // Variable display on PWMDAC
    //------------------------------------------------------------------------------
    PwmDacCh1 = _IQtoQ15(10*motor->clarke.As);
    PwmDacCh2 = _IQtoQ15(10*motor->clarke.As);
    PwmDacCh3 = _IQtoQ15(10*motor->clarke.As);
    PwmDacCh4 = _IQtoQ15(10*motor->clarke.As);
    //------------------------------------------------------------------------------
    // Variable display on DAC - not available
    //------------------------------------------------------------------------------
    GPIO_TogglePin(66);
    return;
}
#endif


// ****************************************************************************
// ****************************************************************************
//TODO Motor Control ISR
// ****************************************************************************
// ****************************************************************************
interrupt void MotorControlISR(void)
{

    // Verifying the ISR
    IsrTicker++;

    //    GPIO_TogglePin(TEMP_GPIO, IsrTicker%2);

#if (BUILDLEVEL == LEVEL1)

    BuildLevel1(&motor1);

#else
    // ------------------------------------------------------------------------------
    //  Measure phase currents and obtain position encoder (QEP) feedback
    // ------------------------------------------------------------------------------
    motorCurrentSense();    //  Measure normalized phase currents (-1,+1)
//    motorVACSense(&Va,&Vb,&Vc);        //  Measure normalized phase voltages (-1,+1)
    posEncoder(&motor1);    //  Motor 1 Position encoder
//    acc_y=(float)AdcaResultRegs.ADCRESULT1*ADC_PU_SCALE_FACTOR;
//    systemp=(float)AdcbResultRegs.ADCRESULT1*ADC_PU_SCALE_FACTOR*3.3/0.01;
    //    motorACCSense(&acc_y);
    //    motortempSense(&systemp);

#if (BUILDLEVEL==LEVEL2)

    BuildLevel2(&motor1);

#elif (BUILDLEVEL==LEVEL3)

    BuildLevel3(&motor1);

#elif (BUILDLEVEL==LEVEL4)

    BuildLevel4(&motor1);

#elif (BUILDLEVEL==LEVEL5)

    BuildLevel5(&motor1);

#endif

#endif


    // ------------------------------------------------------------------------------
    //    Call the PWMDAC update macro.
    // ------------------------------------------------------------------------------
    PWMDAC_MACRO(pwmdac1)
    // ------------------------------------------------------------------------------
    //    Call DAC update.
    // ------------------------------------------------------------------------------
    DAC_PTR[DACA]->DACVALS.all = dacAval;
    DAC_PTR[DACB]->DACVALS.all = dacBval;
    //    DAC_PTR[DACC]->DACVALS.all = dacCval;

    //clear ADCINT1 INT and ack PIE INT
    AdcbRegs.ADCINTFLGCLR.bit.ADCINT1=1;
    PieCtrlRegs.PIEACK.all=PIEACK_GROUP1;

}// MainISR Ends Here

interrupt void scibRxFifoIsr(void)
{
    // wait receive buffer for 0.1 ms to ensure data received
    DELAY_US(200);
    sci_sts_count=0;
    while (ScibRegs.SCIFFRX.bit.RXFFST > 0) {   // check if FIFO has data
        ReceivedChar[sci_sts_count] = ScibRegs.SCIRXBUF.all; // read from FIFO
        sci_sts_count++;
    }


    // if BRKDT occur, reset the buffer
    if (ScibRegs.SCIRXST.bit.RXERROR==1){
        test_count++; // error times count
        if(ScibRegs.SCIRXST.bit.BRKDT==1){
            EALLOW;
            ScibRegs.SCICTL1.bit.SWRESET=0;//reset err flag
            ScibRegs.SCICTL1.bit.SWRESET=1;//re-enable
            EDIS;
        }
    }


    int cmd_rcv=0;//command receive
    if (ReceivedChar[0]==1 && ReceivedChar[1]==device_number){
        cmd_rcv=1;
        update_FAST_en=0; // stop FAST data buffer update
        cmd_rcv_time++;
    }


    if (cmd_rcv){
        int Master_cmd=ReceivedChar[2];
        switch (Master_cmd){

        case 1: //device status check
            GPIO_WritePin(TX_EN_GPIO,1);                // Receive complete, enable TX
            scib_xmit(2);                               // from slave
            scib_xmit(device_number);                   // device number to avoid conflict
            scib_xmit(~(device_number & 0xFFFF));       // slave response code
            scib_xmit(0);                               // Idle data
            while(ScibRegs.SCIFFTX.bit.TXFFST != 0) {}  // wait until TXFF buffer clear
            while(ScibRegs.SCICTL2.bit.TXEMPTY != 1) {} // wait until transmit complete
            GPIO_WritePin(TX_EN_GPIO,0);                // enable receiver
            break;

        case 2: // data collectfor motor monitoring (Torque, Efficiency, Short index )
            // for sending times check
            //circular buffer for FAST data  。
            cb_index=head1 +sci_count*2;
            cb_index= (cb_index>=MAX_EMIF_length) ? cb_index%MAX_EMIF_length : cb_index; // redirect pointer if overflow
            FAST_data[0]=data_transmit_test ? 5566 :  (Uint16)(Speed_F.Speed*32768)+32767;                   //Speed_F.Speed, pu
            FAST_data[1]=data_transmit_test ? update_FAST_en : (Uint16)(Torq_est_avg/1000*32768)+32767;                      //Torq_est, pu
            FAST_data[2]=data_transmit_test ? sci_count  : (Uint16)(Power1_avg_last/1000*32768)+32767;              //Power W, PU
            FAST_data[3]=data_transmit_test ? cb_index  : cb_index;                             //Idle
            send_data_package( device_number, FAST_data[0],FAST_data[1],FAST_data[2],FAST_data[3]);
            sci_count++;
            break;

        case 3: //device data collection for PM Flux (continuous)
            // entering communication mode for continuous transmit
            communication_mode_active=1;
            //circular buffer for FAST data  。
            cb_index=head1 +sci_count*2;
            cb_index= (cb_index>=MAX_EMIF_length) ? cb_index%MAX_EMIF_length : cb_index; // redirect pointer if overflow
            send_data_package( device_number, AsramData.data5[cb_index], AsramData.data6[cb_index],1145,1419);
            sci_count++;// for sending times check
            break;

        case 4: //device data collection for  RUL
            // entering communication mode for continuous transmit
            communication_mode_active=1;

            //circular buffer for FAST data  。
            cb_index=head1 +sci_count*2;
            cb_index= (cb_index>=MAX_EMIF_length) ? cb_index%MAX_EMIF_length : cb_index; // redirect pointer if overflow
            send_data_package( device_number,AsramData.data1[cb_index], AsramData.data2[cb_index],AsramData.data3[cb_index],AsramData.data4[cb_index]);
            sci_count++;// for sending times check
            break;

        case 5:// command =5: device data collection stop
            // leave communication mode
            communication_mode_active=0;
//            set_communication_mode(communication_mode_active);
            update_FAST_en=1; // enable fast update
            sci_count=0;
            cmd_rcv_time=0;
            test_count=0;
            // device response for master check
            GPIO_WritePin(TX_EN_GPIO,1);                // Receive complete, enable TX
            scib_xmit(2);                               // from slave
            scib_xmit(device_number);                   // device number to avoid conflict
            scib_xmit(~(device_number & 0xFFFF));       // slave response code
            scib_xmit(0);                               // Idle data
            while(ScibRegs.SCIFFTX.bit.TXFFST != 0) {}  // wait until TXFF buffer clear
            while(ScibRegs.SCICTL2.bit.TXEMPTY != 1) {} // wait until transmit complete
            GPIO_WritePin(TX_EN_GPIO,0);                // enable receiver
            break;

        case 6:// command =6: set the motor parameters
            // get the motor parameters from master
            myf_Rs.raw.last  = ReceivedChar[3];
            myf_Rs.raw.third  = ReceivedChar[4];
            myf_Rs.raw.second = ReceivedChar[5];
            myf_Rs.raw.first  = ReceivedChar[6];
            myf_Ls.raw.last   = ReceivedChar[7];
            myf_Ls.raw.third  = ReceivedChar[8];
            myf_Ls.raw.second = ReceivedChar[9];
            myf_Ls.raw.first  = ReceivedChar[10];
            POLE_PAIRS=ReceivedChar[11]/2;
            Flux_Rs=myf_Rs.f;
            Ls=myf_Ls.f;

//            POLE_PAIRS=ReceivedChar[7]/2;
//            H_8bits = ReceivedChar[3] & 0xFF;
//            L_8bits = ReceivedChar[4] & 0xFF;
//            Rs_U16 = ((Uint16)H_8bits << 8) | L_8bits;
//            H_8bits = ReceivedChar[5] & 0xFF;
//            L_8bits = ReceivedChar[6] & 0xFF;
//            Ls_U16 = ((Uint16)H_8bits << 8) | L_8bits;;
//            Flux_Rs=(float)Rs_U16/65535*20;
//            Ls=(float)Ls_U16/65535;
            // enable fast update
            update_FAST_en=1;
            sci_count=0;
            // send the eco back
            GPIO_WritePin(TX_EN_GPIO,1);                // Receive complete, enable TX
            scib_xmit(2);                               // cmd code: from slave
            scib_xmit(device_number);
            int i1=0;
            for (i1 = 0; i1 < 9; i1++){
                scib_xmit(ReceivedChar[i1+3]);
            }
            while(ScibRegs.SCIFFTX.bit.TXFFST != 0) {}  // wait until TXFF buffer clear
            while(ScibRegs.SCICTL2.bit.TXEMPTY != 1) {} // wait until transmit complete
            GPIO_WritePin(TX_EN_GPIO,0);                // enable receiver
            break;

        case 7: // data collectfor motor cn status  (Torque, Efficiency, Short index )
            // for sending times check
            sci_count++;
            //circular buffer for FAST data  。
            cb_index=head1 +sci_count*2;
            cb_index= (cb_index>=MAX_EMIF_length) ? cb_index%MAX_EMIF_length : cb_index; // redirect pointer if overflow
            FAST_data[0]=data_transmit_test ? 5566 :  (Uint16)(parkDs_cn_avg*32768)+32768;   //Icn_x
            FAST_data[1]=data_transmit_test ? 7788 :  (Uint16)(parkQs_cn_avg*32768)+32768;  //Icn_y
            FAST_data[2]=data_transmit_test ? sci_count  : (Uint16)(current_mag_avg*32768)+32768;                 //_rms
            FAST_data[3]=data_transmit_test ? cb_index  : cb_index;                   //Idle
            send_data_package( device_number, FAST_data[0],FAST_data[1],FAST_data[2],FAST_data[3]);
            break;

        default: // for undefine command check
            sci_count=0;
            break;

        }

    }

    // Force to clear FIFO receiver
    while (ScibRegs.SCIFFRX.bit.RXFFST!=0){
        int a =ScibRegs.SCIRXBUF.all;
    }
    GPIO_WritePin(TX_EN_GPIO,0);               // enable receiver

    ScibRegs.SCIFFRX.bit.RXFFOVRCLR=1;   // Clear Overflow flag
    ScibRegs.SCIFFRX.bit.RXFFINTCLR=1;   // Clear Interrupt flag
    ScibRegs.SCIFFTX.bit.TXFIFORESET = 1;// Reset FIFO
    ScibRegs.SCIFFRX.bit.RXFIFORESET= 0;// Reset the pointer of FIFO
    ScibRegs.SCIFFRX.bit.RXFIFORESET= 1;// Re-enable receive FIFO
    PieCtrlRegs.PIEACK.all|=PIEACK_GROUP9;// Write 1 to PIEACK for clear ACK9
}

inline void send_data_package(Uint16 device_num, Uint16 data1,Uint16 data2,Uint16 data3,Uint16 data4){
    GPIO_WritePin(TX_EN_GPIO,1);                // Receive complete, enable TX
    scib_xmit(2);                               // cmd code: from slave
    scib_xmit(device_num);                   // cmd code: device number to avoid conflict
    scib_uint(data1);
    scib_uint(data2);
    scib_uint(data3);
    scib_uint(data4);
    DELAY_US(100); // make sure data transmit complete
    while(ScibRegs.SCIFFTX.bit.TXFFST != 0) {}  // wait until TXFF buffer clear
    while(ScibRegs.SCICTL2.bit.TXEMPTY != 1) {} // wait until transmit complete
    GPIO_WritePin(TX_EN_GPIO,0);                // enable receiver
}

//inline void set_communication_mode(Uint16 communication_mode_active){
//    EALLOW;
//    if (communication_mode_active)
//        PieCtrlRegs.PIEIER1.bit.INTx2=0;// Stop ADC sampling
//    else
//        PieCtrlRegs.PIEIER1.bit.INTx2=1;// Start ADC sampling
//    EDIS;
//}

/****************************************************************************
 * End of Code *
 * ***************************************************************************
 */
