/*
 * Copyright (c) 2015, Alex Taradov <alex@taradov.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

//-----------------------------------------------------------------------------
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "samd10.h"
#include "hal_gpio.h"

#define PERIOD_FAST     100
#define PERIOD_SLOW     500

HAL_GPIO_PIN(LED,      A, 5)
//HAL_GPIO_PIN(UART_TX,  A, 14)
//HAL_GPIO_PIN(UART_RX,  A, 15)

static void sys_init(void)
{
  SYSCTRL->OSC8M.bit.PRESC = 0;
  // Turning on OSC32K, refer 16.8.7 in datasheet
  NVMCTRL->CTRLB.bit.RWS = 1;

  SYSCTRL->OSC32K.bit.ONDEMAND = 0;
  SYSCTRL->OSC32K.bit.RUNSTDBY = 0;
  SYSCTRL->OSC32K.bit.EN1K = 0;
  SYSCTRL->OSC32K.bit.EN32K = 1;
  SYSCTRL->OSC32K.bit.WRTLOCK = 0;

  SYSCTRL->OSC32K.bit.STARTUP = 0x7;
  
  // Calibrate the clock
  SYSCTRL->OSC32K.bit.CALIB = FUSES_OSC32K(0);

  // Time to enable the clock!
  SYSCTRL->OSC32K.bit.ENABLE = 1;

  // Wait till we get a clock ready signal
  while(!SYSCTRL->PCLKSR.bit.OSC32KRDY);

  
  //Put OSC32K as source of Generic Clock Generator 1
  GCLK->GENDIV.bit.DIV = 1;
  GCLK->GENDIV.bit.ID = GCLK_GENDIV_ID(1);
  
  // Configure Generic Clock Generator 1 with OSC32K as source
  GCLK->GENCTRL.bit.RUNSTDBY = 0;   /* Generic Clock Generator is stopped in stdby */
  GCLK->GENCTRL.bit.DIVSEL =  0;    /* Use GENDIV.DIV value to divide the generator */
  GCLK->GENCTRL.bit.OE = 0;         /* Disable generator output to GCLK_IO[1] */
  GCLK->GENCTRL.bit.OOV = 0;        /* GCLK_IO[1] output value when generator is off */
  GCLK->GENCTRL.bit.IDC = 1;        /* Generator duty cycle is 50/50 */
  GCLK->GENCTRL.bit.GENEN = 1;      /* Enable the generator */
  GCLK->GENCTRL.bit.SRC = GCLK_GENCTRL_SRC(GCLK_GENCTRL_SRC_OSC32K_Val); /* Generator source: OSC32K output */
  GCLK->GENCTRL.bit.ID = GCLK_GENCTRL_ID(1);   /* Generator ID: 1 */

  // GENCTRL is Write-Synchronized...so wait for write to complete
  while(GCLK->STATUS.bit.SYNCBUSY);
  
  // 4) Put Generic Clock Generator 1 as source for Generic Clock Multiplexer 0 (DFLL48M reference)
  
  GCLK->CLKCTRL.bit.WRTLOCK = 0;		/* Generic Clock is not locked from subsequent writes */
  GCLK->CLKCTRL.bit.CLKEN = 1;			/* Enable the Generic Clock */
  GCLK->CLKCTRL.bit.GEN = GCLK_CLKCTRL_GEN(GCLK_CLKCTRL_GEN_GCLK1_Val); 	/* Generic Clock Generator 1 is the source */
  GCLK->CLKCTRL.bit.ID = GCLK_CLKCTRL_ID(GCLK_CLKCTRL_ID_DFLL48_Val);			/* Generic Clock Multiplexer 0 (DFLL48M Reference) */

  // 5) Enable DFLL48M clock
  
  // DFLL Configuration in Closed Loop mode, cf product data sheet chapter
  // 17.6.7.1 - Closed-Loop Operation
  
  // Enable the DFLL48M in open loop mode. Without this step, attempts to go into closed loop mode at 48 MHz will
  // result in Processor Reset (you'll be at the in the Reset_Handler in startup_samd21.c).
  // PCLKSR.DFLLRDY must be one before writing to the DFLL Control register
  // Note that the DFLLRDY bit represents status of register synchronization - NOT clock stability
  // (see Data Sheet 17.6.14 Synchronization for detail)
  while(!SYSCTRL->PCLKSR.bit.DFLLRDY);
  SYSCTRL->DFLLCTRL.reg = (uint16_t)(SYSCTRL_DFLLCTRL_ENABLE);
  while(!SYSCTRL->PCLKSR.bit.DFLLRDY);
  
  // Set up the Multiplier, Coarse and Fine steps
  SYSCTRL_DFLLMUL_Type sysctrl_dfllmul = {
    .bit.CSTEP = 31,		/* Coarse step - use half of the max value (63) */
    .bit.FSTEP = 511,		/* Fine step - use half of the max value (1023) */
    .bit.MUL = 1465			/* Multiplier = MAIN_CLK_FREQ (48MHz) / EXT_32K_CLK_FREQ (32768 Hz) */
  };
  // Write these settings
  SYSCTRL->DFLLMUL.reg = sysctrl_dfllmul.reg;
  // Wait for synchronization
  while(!SYSCTRL->PCLKSR.bit.DFLLRDY);
  
  // To reduce lock time, load factory calibrated values into DFLLVAL (cf. Data Sheet 17.6.7.1)
  // Location of value is defined in Data Sheet Table 10-5. NVM Software Calibration Area Mapping
  
  // Get factory calibrated value for "DFLL48M COARSE CAL" from NVM Software Calibration Area
  uint32_t* tempDFLL48CalibrationCoarse = (uint32_t*)FUSES_DFLL48M_COARSE_CAL_ADDR;
  *tempDFLL48CalibrationCoarse &= FUSES_DFLL48M_COARSE_CAL_Msk;
  *tempDFLL48CalibrationCoarse = *tempDFLL48CalibrationCoarse >> FUSES_DFLL48M_COARSE_CAL_Pos;
  // Write the coarse calibration value
  SYSCTRL->DFLLVAL.bit.COARSE = tempDFLL48CalibrationCoarse;
  // Switch DFLL48M to Closed Loop mode and enable WAITLOCK
  while(!SYSCTRL->PCLKSR.bit.DFLLRDY);
  SYSCTRL->DFLLCTRL.reg |= (uint16_t) (SYSCTRL_DFLLCTRL_MODE | SYSCTRL_DFLLCTRL_WAITLOCK);
  
  /* ----------------------------------------------------------------------------------------------
  * 6) Switch Generic Clock Generator 0 to DFLL48M. CPU will run at 48MHz.
  */
  
  // Now that DFLL48M is running, switch CLKGEN0 source to it to run the core at 48 MHz.
  // Enable output of Generic Clock Generator 0 (GCLK_MAIN) to the GCLK_IO[0] GPIO Pin
  GCLK_GENCTRL_Type gclk_genctrl0 = {
    .bit.RUNSTDBY = 0,		/* Generic Clock Generator is stopped in stdby */
    .bit.DIVSEL =  0,		/* Use GENDIV.DIV value to divide the generator */
    .bit.OE = 1,			/* Enable generator output to GCLK_IO[0] */
    .bit.OOV = 0,			/* GCLK_IO[0] output value when generator is off */
    .bit.IDC = 1,			/* Generator duty cycle is 50/50 */
    .bit.GENEN = 1,			/* Enable the generator */
    .bit.SRC = 0x07,		/* Generator source: DFLL48M output */
    .bit.ID = 0			/* Generator ID: 0 */
  };
  GCLK->GENCTRL.reg = gclk_genctrl0.reg;
  // GENCTRL is Write-Synchronized...so wait for write to complete
  while(GCLK->STATUS.bit.SYNCBUSY);
  
  // Direct the GCLK_IO[0] output to PA28
  PORT_WRCONFIG_Type port0_wrconfig = {
    .bit.HWSEL = 1,			/* Pin# (28) - falls in the upper half of the 32-pin PORT group */
    .bit.WRPINCFG = 1,		/* Update PINCFGy registers for all pins selected */
    .bit.WRPMUX = 1,		/* Update PMUXn registers for all pins selected */
    .bit.PMUX = 7,			/* Peripheral Function H selected (GCLK_IO[0]) */
    .bit.PMUXEN = 1,		/* Enable peripheral Multiplexer */
    .bit.PINMASK = (uint16_t)(1 << (28-16)) /* Select the pin(s) to be configured */
  };
  // Write these settings
  PORT->Group[0].WRCONFIG.reg = port0_wrconfig.reg;
  
  /* ----------------------------------------------------------------------------------------------
  * 7) Modify prescaler value of OSC8M to produce 8MHz output
  */

  SYSCTRL->OSC8M.bit.PRESC = 0;		/* Prescale by 1 */
  SYSCTRL->OSC8M.bit.ONDEMAND = 0 ;	/* Oscillator is always on if enabled */
  
  /* ----------------------------------------------------------------------------------------------
  * 8) Put OSC8M as source for Generic Clock Generator 3
  */
  
  // Set the Generic Clock Generator 3 output divider to 1
  // Configure GCLK->GENDIV settings
  GCLK_GENDIV_Type gclk3_gendiv = {
    .bit.DIV = 1,								/* Set output division factor = 1 */
    .bit.ID = GCLK_GENDIV_ID(3)		/* Apply division factor to Generator 3 */
  };
  // Write these settings
  GCLK->GENDIV.reg = gclk3_gendiv.reg;
  
  // Configure Generic Clock Generator 3 with OSC8M as source
  GCLK_GENCTRL_Type gclk3_genctrl = {
    .bit.RUNSTDBY = 0,		/* Generic Clock Generator is stopped in stdby */
    .bit.DIVSEL =  0,		/* Use GENDIV.DIV value to divide the generator */
    .bit.OE = 0,			/* Disable generator output to GCLK_IO[1] */
    .bit.OOV = 0,			/* GCLK_IO[2] output value when generator is off */
    .bit.IDC = 1,			/* Generator duty cycle is 50/50 */
    .bit.GENEN = 1,			/* Enable the generator */
    .bit.SRC = 0x06,		/* Generator source: OSC8M output */
    .bit.ID = GCLK_GENCTRL_SRC_OSC8M			/* Generator ID: 3 */
  };
  // Write these settings
  GCLK->GENCTRL.reg = gclk3_genctrl.reg;
  // GENCTRL is Write-Synchronized...so wait for write to complete
  while(GCLK->STATUS.bit.SYNCBUSY);
  
  /* ----------------------------------------------------------------------------------------------
  * 9) Set CPU and APBx BUS Clocks to 48MHz
  */
  PM->CPUSEL.reg  = PM_CPUSEL_CPUDIV_DIV1 ;
  PM->APBASEL.reg = PM_APBASEL_APBADIV_DIV1_Val ;
  PM->APBBSEL.reg = PM_APBBSEL_APBBDIV_DIV1_Val ;
  PM->APBCSEL.reg = PM_APBCSEL_APBCDIV_DIV1_Val ;
  
}

int main(void)
{
  sys_init();

  HAL_GPIO_LED_out();
  HAL_GPIO_LED_clr();

  // The infinite loop
  while(true)
  {
    HAL_GPIO_LED_toggle();
    HAL_GPIO_LED_toggle();
    HAL_GPIO_LED_toggle();
    HAL_GPIO_LED_toggle();
    HAL_GPIO_LED_toggle();
    HAL_GPIO_LED_toggle();
    HAL_GPIO_LED_toggle();
    HAL_GPIO_LED_toggle();
    HAL_GPIO_LED_toggle();
    HAL_GPIO_LED_toggle();
  }

  return 0;
}
