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
  // Set the OSC8M prescaler to 0 (divide-by-1) to achieve the maximum 8MHz frequency
  SYSCTRL->OSC8M.bit.PRESC = 0;

  // Enable the DFLL48M module
  SYSCTRL->DFLLCTRL.reg = SYSCTRL_DFLLCTRL_ENABLE;

  // Wait for the DFLL to stabilize
  while (!(SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_DFLLRDY));

  // Configure the DFLL to generate a 48MHz clock
  SYSCTRL->DFLLMUL.reg = SYSCTRL_DFLLMUL_MUL(0);  // This line makes no sense
  SYSCTRL->DFLLCTRL.reg |= SYSCTRL_DFLLCTRL_MODE | SYSCTRL_DFLLCTRL_CCDIS;
  SYSCTRL->DFLLCTRL.reg |= SYSCTRL_DFLLCTRL_USBCRM;
  SYSCTRL->DFLLCTRL.reg |= SYSCTRL_DFLLCTRL_ENABLE;

  // Wait for the DFLL to lock
  while (!(SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_DFLLLCKC));

  // Switch the main system clock to DFLL48M
  GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(0) | GCLK_GENCTRL_SRC_DFLL48M | GCLK_GENCTRL_GENEN;
  while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);

  // Set the division factor to 1 for the main system clock
  GCLK->GENDIV.reg = GCLK_GENDIV_ID(0) | GCLK_GENDIV_DIV(1);

  // Enable the generic clock generator
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(0) | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_CLKEN;
  while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);
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
  }

  return 0;
}
