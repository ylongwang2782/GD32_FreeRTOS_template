/*!
    \file    systick.c
    \brief   the systick configuration file

    \version 2024-01-15, V3.2.0, firmware for GD32F4xx
*/

/*
    Copyright (c) 2024, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors
       may be used to endorse or promote products derived from this software without
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
OF SUCH DAMAGE.
*/

#include "gd32f4xx.h"
#include "systick.h"

#ifndef USE_OS
volatile static uint32_t delay;
volatile static uint32_t uwTick = 0;
#else
#include "FreeRTOS.h"
#include "task.h"
static uint8_t fac_us = 0;  //us延时倍乘数
static uint16_t fac_ms = 0; //ms延时倍乘数,在rtos下,代表每个节拍的ms数
extern void xPortSysTickHandler(void);

/*!
    \brief    delay a time in microseconds
    \param[in]  nus: count in microseconds
    \param[out] none
    \retval     none
*/
void delay_us(uint32_t nus)
{
    uint32_t ticks;
    uint32_t told, tnow, tcnt = 0;
    uint32_t reload = SysTick->LOAD;
    ticks = nus * fac_us;
    told = SysTick->VAL;
    while (1)
    {
        tnow = SysTick->VAL;
        if (tnow != told)
        {
            if (tnow < told)
                tcnt += told - tnow;
            else
                tcnt += reload - tnow + told;
            told = tnow;
            if (tcnt >= ticks)
                break;
        }
    };
}
#endif

/*!
    \brief    configure systick
    \param[in]  none
    \param[out] none
    \retval     none
*/
void systick_config(void)
{
#ifndef USE_OS
    /* setup systick timer for 1000Hz interrupts */
    if (SysTick_Config(SystemCoreClock / 1000U))
    {
        /* capture error */
        while (1)
        {
        }
    }
#else
    /* setup systick timer for 1000Hz interrupts */
    if (SysTick_Config(SystemCoreClock / configTICK_RATE_HZ))
    {
        /* capture error */
        while (1)
        {
        }
    }
    fac_us = SystemCoreClock / 1000000;
    fac_ms = 10000 / configTICK_RATE_HZ;
#endif
    /* configure the systick handler priority */
    NVIC_SetPriority(SysTick_IRQn, 0x00U);
}

/*!
    \brief    delay a time in milliseconds
    \param[in]  count: count in milliseconds
    \param[out] none
    \retval     none
*/
void delay_1ms(uint32_t nms)
{
#ifndef USE_OS
    delay = nms;
    while (0U != delay)
    {
    }
#else
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) //系统已经运行
    {
        if (nms >= fac_ms) //延时的时间大于OS的最少时间周期
        {
            vTaskDelay(10 * nms / fac_ms); //FreeRTOS延时
        }
        nms %= fac_ms; //OS已经无法提供这么小的延时了,采用普通方式延时
    }
    delay_us((uint32_t)(nms * 1000)); //普通方式延时
#endif
}

#ifndef USE_OS
/*!
    \brief    delay decrement
    \param[in]  none
    \param[out] none
    \retval     none
*/

void delay_decrement(void)
{
    uwTick++;
    if (0U != delay)
    {
        delay--;
    }
}

uint32_t Get_Tick()
{
    return uwTick;
}
#endif

void SysTick_Handler(void)
{
#ifndef USE_OS
    delay_decrement();
    
#else
#if (INCLUDE_xTaskGetSchedulerState == 1)
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
    {
#endif
        xPortSysTickHandler();
#if (INCLUDE_xTaskGetSchedulerState == 1)
    }
#endif

#endif
}
