/*
 * This file is part of the Nauteff Autopilot project.
 *
 * Copyright (C) 2022 Nauteff https://nauteff.com
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef RCC_H
#define RCC_H


#define RCC_BASE 0x40023800
#define RCC_CR          (*(vuint32_t*)(RCC_BASE + 0x00))
#define RCC_PLLCFGR     (*(vuint32_t*)(RCC_BASE + 0x04))
#define RCC_CFGR        (*(vuint32_t*)(RCC_BASE + 0x08))
#define RCC_CIR         (*(vuint32_t*)(RCC_BASE + 0x0C))
#define RCC_AHB1ENR     (*(vuint32_t*)(RCC_BASE + 0x30))
#define RCC_AHB2ENR     (*(vuint32_t*)(RCC_BASE + 0x34))
#define RCC_AHB3ENR     (*(vuint32_t*)(RCC_BASE + 0x38))
#define RCC_APB1ENR     (*(vuint32_t*)(RCC_BASE + 0x40))
#define RCC_APB2ENR     (*(vuint32_t*)(RCC_BASE + 0x44))
#define RCC_CSR         (*(vuint32_t*)(RCC_BASE + 0x74))
#define RCC_SSCGR       (*(vuint32_t*)(RCC_BASE + 0x80))
#define RCC_PLLI2SCFGR  (*(vuint32_t*)(RCC_BASE + 0x84))
#define RCC_PLLSAICFGR  (*(vuint32_t*)(RCC_BASE + 0x88))
#define RCC_DCKCFGR     (*(vuint32_t*)(RCC_BASE + 0x8C))
#define CKGATENR        (*(vuint32_t*)(RCC_BASE + 0x90))
#define RCC_DCKCFGR2    (*(vuint32_t*)(RCC_BASE + 0x94))






typedef struct
{
    unsigned SYSCLK_Freq_Mhz;
    unsigned AHB_2pwr_Prescaler;
    unsigned APB1_2pwr_Prescaler;
    unsigned APB2_2pwr_Prescaler;
} RCC_config_t;

#define AHB_PRESCALER_NO_DEVIDE       0x0
#define AHB_PRESCALER_DEVIDED_BY_2    0x8
#define AHB_PRESCALER_DEVIDED_BY_4    0x9
#define AHB_PRESCALER_DEVIDED_BY_8    0xA
#define AHB_PRESCALER_DEVIDED_BY_16   0xB
/* No value for syst. clock divided by 32 ! ? ! ? */
#define AHB_PRESCALER_DEVIDED_BY_64   0xC
#define AHB_PRESCALER_DEVIDED_BY_128  0xD
#define AHB_PRESCALER_DEVIDED_BY_256  0xE
#define AHB_PRESCALER_DEVIDED_BY_512  0xF

#define APBX_PRESCALER_NO_DEVIDE      0x0
#define APBX_PRESCALER_DEVIDED_BY_2   0x4
#define APBX_PRESCALER_DEVIDED_BY_4   0x5
#define APBX_PRESCALER_DEVIDED_BY_8   0x6
#define APBX_PRESCALER_DEVIDED_BY_16  0x7

int RCC_init_for_nauteff (RCC_config_t *cfg);

uint32_t RCC_get_APB1_Freq_Hz ();
uint32_t RCC_get_APB2_Freq_Hz ();
uint32_t RCC_get_SYSCLK_Freq_Hz ();
uint32_t RCC_get_APB1_Timer_Freq_Hz ();
uint32_t RCC_get_APB2_Timer_Freq_Hz ();

__attribute__((always_inline))
inline void rcc_start_GPIOA (void)
{
    RCC_AHB1ENR |= (0x1 << 0);
}
__attribute__((always_inline))
inline void rcc_start_GPIOB (void)
{
    RCC_AHB1ENR |= (0x1 << 1);
}
__attribute__((always_inline))
inline void rcc_start_GPIOC (void)
{
    RCC_AHB1ENR |= (0x1 << 2);
}
__attribute__((always_inline))
inline void rcc_start_GPIOD (void)
{
    RCC_AHB1ENR |= (0x1 << 3);
}
inline void rcc_start_GPIOE (void)
{
    RCC_AHB1ENR |= (0x1 << 4);
}
inline void rcc_start_GPIOF (void)
{
    RCC_AHB1ENR |= (0x1 << 5);
}
inline void rcc_start_GPIOG (void)
{
    RCC_AHB1ENR |= (0x1 << 6);
}
inline void rcc_start_GPIOH (void)
{
    RCC_AHB1ENR |= (0x1 << 7);
}
inline void rcc_start_DMA1 (void)
{
    RCC_AHB1ENR |= (0x1 << 21);
}
inline void rcc_start_DMA2 (void)
{
    RCC_AHB1ENR |= (0x1 << 22);
}

inline void rcc_start_TIM2 (void)
{
    RCC_APB1ENR |= (0x1 << 0);
}
inline void rcc_start_TIM3 (void)
{
    RCC_APB1ENR |= (0x1 << 1);
}
inline void rcc_start_TIM4 (void)
{
    RCC_APB1ENR |= (0x1 << 2);
}
inline void rcc_start_TIM5 (void)
{
    RCC_APB1ENR |= (0x1 << 3);
}
inline void rcc_start_TIM6 (void)
{
    RCC_APB1ENR |= (0x1 << 4);
}
inline void rcc_start_TIM7 (void)
{
    RCC_APB1ENR |= (0x1 << 5);
}
inline void rcc_start_TIM12 (void)
{
    RCC_APB1ENR |= (0x1 << 6);
}
inline void rcc_start_TIM13 (void)
{
    RCC_APB1ENR |= (0x1 << 7);
}
inline void rcc_start_TIM14 (void)
{
    RCC_APB1ENR |= (0x1 << 8);
}
inline void rcc_start_WWDG (void)
{
    RCC_APB1ENR |= (0x1 << 11);
}
inline void rcc_start_SPI2 (void)
{
    RCC_APB1ENR |= (0x1 << 14);
}
inline void rcc_start_SPI3 (void)
{
    RCC_APB1ENR |= (0x1 << 15);
}
inline void rcc_start_SPDIFRX (void)
{
    RCC_APB1ENR |= (0x1 << 16);
}
inline void rcc_start_USART2 (void)
{
    RCC_APB1ENR |= (0x1 << 17);
}
inline void rcc_start_USART3 (void)
{
    RCC_APB1ENR |= (0x1 << 18);
}
inline void rcc_start_USART4 (void)
{
    RCC_APB1ENR |= (0x1 << 19);
}
inline void rcc_start_USART5 (void)
{
    RCC_APB1ENR |= (0x1 << 20);
}
inline void rcc_start_I2C1 (void)
{
    RCC_APB1ENR |= (0x1 << 21);
}
inline void rcc_start_I2C2 (void)
{
    RCC_APB1ENR |= (0x1 << 22);
}
inline void rcc_start_I2C3 (void)
{
    RCC_APB1ENR |= (0x1 << 23);
}
inline void rcc_start_FMPI2C1 (void)
{
    RCC_APB1ENR |= (0x1 << 24);
}
inline void rcc_start_CAN1 (void)
{
    RCC_APB1ENR |= (0x1 << 25);
}
inline void rcc_start_CAN2 (void)
{
    RCC_APB1ENR |= (0x1 << 26);
}
inline void rcc_start_CEC (void)
{
    RCC_APB1ENR |= (0x1 << 27);
}
inline void rcc_start_PWR (void)
{
    RCC_APB1ENR |= (0x1 << 28);
}
inline void rcc_start_DAC (void)
{
    RCC_APB1ENR |= (0x1 << 29);
}

inline void rcc_start_TIM1 (void)
{
    RCC_APB2ENR |= (0x1 << 0);
}
inline void rcc_start_TIM8 (void)
{
    RCC_APB2ENR |= (0x1 << 1);
}
inline void rcc_start_USART1 (void)
{
    RCC_APB2ENR |= (0x1 << 4);
}
inline void rcc_start_USART6 (void)
{
    RCC_APB2ENR |= (0x1 << 5);
}
inline void rcc_start_ADC1 (void)
{
    RCC_APB2ENR |= (0x1 << 8);
}
inline void rcc_start_ADC2 (void)
{
    RCC_APB2ENR |= (0x1 << 9);
}
inline void rcc_start_ADC3 (void)
{
    RCC_APB2ENR |= (0x1 << 10);
}
inline void rcc_start_SDIO (void)
{
    RCC_APB2ENR |= (0x1 << 11);
}
inline void rcc_start_SPI1 (void)
{
    RCC_APB2ENR |= (0x1 << 12);
}
inline void rcc_start_SPI4 (void)
{
    RCC_APB2ENR |= (0x1 << 13);
}
__attribute__((always_inline))
inline void rcc_start_SYSCFG (void)
{
    RCC_APB2ENR |= (0x1 << 14);
}
inline void rcc_start_TIM9 (void)
{
    RCC_APB2ENR |= (0x1 << 16);
}
inline void rcc_start_TIM10 (void)
{
    RCC_APB2ENR |= (0x1 << 17);
}
inline void rcc_start_TIM11 (void)
{
    RCC_APB2ENR |= (0x1 << 18);
}
inline void rcc_start_SAI1 (void)
{
    RCC_APB2ENR |= (0x1 << 22);
}
inline void rcc_start_SAI2 (void)
{
    RCC_APB2ENR |= (0x1 << 23);
}

#if 0
int RCC_select_sys_clk_src (unsigned src);
int RCC_select_PLL_clk_src (unsigned src);
int RCC_config_PLL (unsigned freqCLK, uint32_t freqQ);

RCC_set_SYSCLK_freq_MHz(unsigned freqMHz);
RCC_set_AHB_2pwr_Prescaler ();
RCC_set_APB1_2pwr_Prescaler ();
RCC_set_APB2_2pwr_Prescaler ();
#endif

#endif
