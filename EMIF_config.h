/*
 * EMIF_config.h
 *
 *  Created on: 2024¦~10¤ë8¤é
 *      Author: MotorTech
 */

#ifndef EMIF_CONFIG_H_
#define EMIF_CONFIG_H_

#define ASRAM_CS2_START_ADDR 0x100000
#define ASRAM_CS2_SIZE       0x8000

extern bool EMIF_success;

int setup_EMIF(void){
int ErrCount;
    //
    //Configure to run EMIF1 on full Rate (EMIF1CLK = CPU1SYSCLK)
    //
      EALLOW;
      ClkCfgRegs.PERCLKDIVSEL.bit.EMIF1CLKDIV = 0x0;
      EDIS;

      EALLOW;
    //
    // Grab EMIF1 For CPU1
    //
      Emif1ConfigRegs.EMIF1MSEL.all = 0x93A5CE71;
      if(Emif1ConfigRegs.EMIF1MSEL.all != 0x1)
      {
          ErrCount++;
      }

    //
    //Disable Access Protection (CPU_FETCH/CPU_WR/DMA_WR)
    //
      Emif1ConfigRegs.EMIF1ACCPROT0.all = 0x0;
      if(Emif1ConfigRegs.EMIF1ACCPROT0.all != 0x0)
      {
          ErrCount++;
      }

    //
    // Commit the configuration related to protection. Till this bit remains set
    // content of EMIF1ACCPROT0 register can't be changed.
    //
      Emif1ConfigRegs.EMIF1COMMIT.all = 0x1;
      if(Emif1ConfigRegs.EMIF1COMMIT.all != 0x1)
      {
         ErrCount++;
      }

    //
    // Lock the configuration so that EMIF1COMMIT register can't be
    // changed any more.
    //
      Emif1ConfigRegs.EMIF1LOCK.all = 0x1;
      if(Emif1ConfigRegs.EMIF1LOCK.all != 1)
      {
          ErrCount++;
      }

      EDIS;

      return ErrCount;
}

void setup_emif1_pinmux_async_16bit(Uint16 cpu_sel){
    Uint16 i;

        GPIO_SetupPinOptions(11, GPIO_INPUT, GPIO_PUSHPULL);
        // avoid short circuit of GPIO 33/34/35
        GPIO_SetupPinMux(33,cpu_sel,0);
        GPIO_SetupPinMux(35,cpu_sel,0);
        GPIO_SetupPinOptions(33, GPIO_INPUT, GPIO_PUSHPULL);
        GPIO_SetupPinOptions(35, GPIO_INPUT, GPIO_PUSHPULL);

        // EMIF Cs2/OE/WE setup
        GPIO_SetupPinMux(34, cpu_sel, 2); // CS2
        GPIO_SetupPinMux(31, cpu_sel, 2); // WE
        GPIO_SetupPinMux(37, cpu_sel, 2); // OE


        // EMIF address GPIO setup
        GPIO_SetupPinMux(38, cpu_sel, 2); // A0
        GPIO_SetupPinMux(39, cpu_sel, 2); // A1
        GPIO_SetupPinMux(40, cpu_sel, 2); // A2
        GPIO_SetupPinMux(41, cpu_sel, 2); // A3
        GPIO_SetupPinMux(44, cpu_sel, 2); // A4
        GPIO_SetupPinMux(45, cpu_sel, 2); // A5
        GPIO_SetupPinMux(46, cpu_sel, 2); // A6
        GPIO_SetupPinMux(47, cpu_sel, 2); // A7
        GPIO_SetupPinMux(48, cpu_sel, 2); // A8
        GPIO_SetupPinMux(49, cpu_sel, 2); // A9
        GPIO_SetupPinMux(50, cpu_sel, 2); // A10
        GPIO_SetupPinMux(51, cpu_sel, 2); // A11
        GPIO_SetupPinMux(52, cpu_sel, 2); // A12
        GPIO_SetupPinMux(86, cpu_sel, 2); // A13
        GPIO_SetupPinMux(87, cpu_sel, 2); // A14
        GPIO_SetupPinMux(88, cpu_sel, 2); // A15
        GPIO_SetupPinMux(89, cpu_sel, 2); // A16

        // EMIF Data GPIO setup
        GPIO_SetupPinMux(85, cpu_sel, 2); // D0
        GPIO_SetupPinMux(83, cpu_sel, 2); // D1
        GPIO_SetupPinMux(82, cpu_sel, 2); // D2
        GPIO_SetupPinMux(81, cpu_sel, 2); // D3
        GPIO_SetupPinMux(80, cpu_sel, 2); // D4
        GPIO_SetupPinMux(79, cpu_sel, 2); // D5
        GPIO_SetupPinMux(78, cpu_sel, 2); // D6
        GPIO_SetupPinMux(77, cpu_sel, 2); // D7
        GPIO_SetupPinMux(76, cpu_sel, 2); // D8
        GPIO_SetupPinMux(75, cpu_sel, 2); // D9
        GPIO_SetupPinMux(74, cpu_sel, 2); // D10
        GPIO_SetupPinMux(73, cpu_sel, 2); // D11
        GPIO_SetupPinMux(72, cpu_sel, 2); // D12
        GPIO_SetupPinMux(71, cpu_sel, 2); // D13
        GPIO_SetupPinMux(70, cpu_sel, 2); // D14
        GPIO_SetupPinMux(69, cpu_sel, 2); // D15




    //    for (i=28; i<=52; i++)
    //    {
    //        if ((i != 42) && (i != 43)&& (i != 33)&& (i != 35))
    //        {
    //            GPIO_SetupPinMux(i,cpu_sel,2);
    //        }
    //    }
    //    for (i=63; i<=87; i++)
    //    {
    //        if (i != 84)
    //        {
    //            GPIO_SetupPinMux(i,cpu_sel,2);
    //        }
    //    }

        GPIO_SetupPinMux(88,cpu_sel,3);
        GPIO_SetupPinMux(89,cpu_sel,3);
        GPIO_SetupPinMux(90,cpu_sel,3);
        GPIO_SetupPinMux(91,cpu_sel,3);
//        GPIO_SetupPinMux(92,cpu_sel,3); // for sci tx enable
        GPIO_SetupPinMux(93,cpu_sel,3);
        GPIO_SetupPinMux(94,cpu_sel,2);

        //
        // setup async mode and enable pull-ups for Data pins
        //
        for (i=69; i<=85; i++)
        {
            if (i != 84)
            {
                GPIO_SetupPinOptions(i,0,0x31); // GPIO_ASYNC||GPIO_PULLUP
            }
        }
}


void emif2_config(void){
    //
    //Configure the access timing for CS2 space
    //
      Emif1Regs.ASYNC_CS2_CR.all =  (EMIF_ASYNC_ASIZE_16    | // 16Bit Memory
                                                              // Interface
                                     EMIF_ASYNC_TA_1        | // Turn Around time
                                                              // of 2 Emif Clock
                                     EMIF_ASYNC_RHOLD_1     | // Read Hold time
                                                              // of 1 Emif Clock
                                     EMIF_ASYNC_RSTROBE_3   | // Read Strobe time
                                                              // of 4 Emif Clock
                                     EMIF_ASYNC_RSETUP_1    | // Read Setup time
                                                              // of 1 Emif Clock
                                     EMIF_ASYNC_WHOLD_1    | // Write Hold time
                                                              // of 1 Emif Clock
                                     EMIF_ASYNC_WSTROBE_3   | // Write Strobe time
                                                              // of 1 Emif Clock
                                     EMIF_ASYNC_WSETUP_1    | // Write Setup time
                                                              // of 1 Emif Clock
                                     EMIF_ASYNC_EW_DISABLE  | // Extended Wait
                                                              // Disable.
                                     EMIF_ASYNC_SS_DISABLE    // Strobe Select Mode
                                                              // Disable.
                                    );
}


#endif /* EMIF_CONFIG_H_ */
