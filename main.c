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

//-----------------------------------------------------------------------------
#define PERIOD_FAST     100
#define PERIOD_SLOW     500

HAL_GPIO_PIN(LED,      A, 5)
HAL_GPIO_PIN(UART_TX,  A, 8)
HAL_GPIO_PIN(UART_RX,  A, 9)

//-----------------------------------------------------------------------------

// Timer for LED blinky
static void timer1_set_period(uint16_t i)
{
  TC1->COUNT16.CC[0].reg = (F_CPU / 1000ul / 256) * i;
  TC1->COUNT16.COUNT.reg = 0;
}

//-----------------------------------------------------------------------------
void irq_handler_tc1(void)
{
  if (TC1->COUNT16.INTFLAG.reg & TC_INTFLAG_MC(1))
  {
    HAL_GPIO_LED_toggle();
    TC1->COUNT16.INTFLAG.reg = TC_INTFLAG_MC(1);
  }
}

//-----------------------------------------------------------------------------
static void timer1_init(void)
{
  PM->APBCMASK.reg |= PM_APBCMASK_TC1;

  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(TC1_GCLK_ID) |
      GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN(0);

  TC1->COUNT16.CTRLA.reg = TC_CTRLA_MODE_COUNT16 | TC_CTRLA_WAVEGEN_MFRQ |
      TC_CTRLA_PRESCALER_DIV256 | TC_CTRLA_PRESCSYNC_RESYNC;

  TC1->COUNT16.COUNT.reg = 0;

  timer1_set_period(PERIOD_SLOW);

  TC1->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;

  TC1->COUNT16.INTENSET.reg = TC_INTENSET_MC(1);
  NVIC_EnableIRQ(TC1_IRQn);
}

//-----------------------------------------------------------------------------
static void uart_init(uint32_t baud)
{
  uint64_t br = (uint64_t)65536 * (F_CPU - 16 * baud) / F_CPU;

  HAL_GPIO_UART_TX_out();
  HAL_GPIO_UART_TX_pmuxen(PORT_PMUX_PMUXE_C_Val);
  HAL_GPIO_UART_RX_in();
  HAL_GPIO_UART_RX_pmuxen(PORT_PMUX_PMUXE_C_Val);

  PM->APBCMASK.reg |= PM_APBCMASK_SERCOM1;

  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(SERCOM1_GCLK_ID_CORE) |
      GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN(0);

  SERCOM1->USART.CTRLA.reg =
      SERCOM_USART_CTRLA_DORD | SERCOM_USART_CTRLA_MODE_USART_INT_CLK |
      SERCOM_USART_CTRLA_RXPO(3/*PAD3*/) | SERCOM_USART_CTRLA_TXPO(1/*PAD2*/);

  SERCOM1->USART.CTRLB.reg = SERCOM_USART_CTRLB_RXEN | SERCOM_USART_CTRLB_TXEN |
      SERCOM_USART_CTRLB_CHSIZE(0/*8 bits*/);

  SERCOM1->USART.BAUD.reg = (uint16_t)br+1;

  SERCOM1->USART.CTRLA.reg |= SERCOM_USART_CTRLA_ENABLE;
}

//-----------------------------------------------------------------------------
static void uart_putc(char c)
{
  while (!(SERCOM1->USART.INTFLAG.reg & SERCOM_USART_INTFLAG_DRE));
  SERCOM1->USART.DATA.reg = c;
}

//-----------------------------------------------------------------------------
static void uart_puts(char *s)
{
  while (*s)
    uart_putc(*s++);
}

// Timer2 for UART UART logging
static void timer2_set_period(uint16_t i)
{
  TC2->COUNT16.CC[0].reg = (F_CPU / 1000ul / 256) * i;
  TC2->COUNT16.COUNT.reg = 0;
}

// Timer2 init
static void timer2_init(void)
{
  PM->APBCMASK.reg |= PM_APBCMASK_TC2;

  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(TC2_GCLK_ID) |
      GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN(0);

  TC2->COUNT16.CTRLA.reg = TC_CTRLA_MODE_COUNT16 | TC_CTRLA_WAVEGEN_MFRQ |
      TC_CTRLA_PRESCALER_DIV256 | TC_CTRLA_PRESCSYNC_RESYNC;

  TC2->COUNT16.COUNT.reg = 0;

  timer2_set_period(PERIOD_SLOW);

  TC2->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;

  TC2->COUNT16.INTENSET.reg = TC_INTENSET_MC(1);
  NVIC_EnableIRQ(TC2_IRQn);
}

// Function to run the timer 2 interrupt request
void irq_handler_tc2(void)
{
  if (TC2->COUNT16.INTFLAG.reg & TC_INTFLAG_MC(1))
  {
    uart_puts("\r\nNew Timer working perfectly\n\r");
    TC2->COUNT16.INTFLAG.reg = TC_INTFLAG_MC(1);
  }
}

//-----------------------------------------------------------------------------
static void sys_init(void)
{
  // Switch to 8MHz clock (disable prescaler)
  SYSCTRL->OSC8M.bit.PRESC = 0;
}

//-----------------------------------------------------------------------------
int main(void)
{
  sys_init();
  timer1_init();
  uart_init(115200);

  timer2_init();

  uart_puts("\r\nHello, world!\r\n");

  HAL_GPIO_LED_out();
  HAL_GPIO_LED_clr();

  // The infinite loop
  while(true);

  return 0;
}
