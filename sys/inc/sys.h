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

#ifndef SYS_H_
#define SYS_H_

#include <stm32F446.h>

void sys_init();
void sys_init_clocks(void);
void sys_init_Data(void);
void sys_init_FPU(void);
void sys_init_GPIOs(void);
void sys_init_DMA(void);
void sys_init_USART1(void);
void sys_init_I2C(void);
void sys_init_ADC(void);
void sys_init_EXTI(void);

/*extern unsigned long _etext;*/
extern unsigned _sidata;
extern unsigned _edata;
extern unsigned _sbss;
extern unsigned _sdata;
extern unsigned _ebss;
extern unsigned _estack;
/*extern unsigned long _sccmdata;*/
/*extern unsigned long _eccmdata;*/
/*extern unsigned long _sccmtext;*/
/*extern unsigned long _eccmtext;*/

#if 0
inline void rcc_start_GPIOA(void) {RCC_AHB1ENR |= (0x1 << 0);}
inline void rcc_start_GPIOB(void) {RCC_AHB1ENR |= (0x1 << 1);}
inline void rcc_start_GPIOC(void) {RCC_AHB1ENR |= (0x1 << 2);}
inline void rcc_start_GPIOD(void) {RCC_AHB1ENR |= (0x1 << 3);}
inline void rcc_start_GPIOE(void) {RCC_AHB1ENR |= (0x1 << 4);}
inline void rcc_start_GPIOF(void) {RCC_AHB1ENR |= (0x1 << 5);}
inline void rcc_start_GPIOG(void) {RCC_AHB1ENR |= (0x1 << 6);}
inline void rcc_start_GPIOH(void) {RCC_AHB1ENR |= (0x1 << 7);}
inline void rcc_start_DMA1(void)  {RCC_AHB1ENR |= (0x1 << 21);}
inline void rcc_start_DMA2(void)  {RCC_AHB1ENR |= (0x1 << 22);}

inline void rcc_start_TIM2(void)    {RCC_APB1ENR |= (0x1 << 0);}
inline void rcc_start_TIM3(void)    {RCC_APB1ENR |= (0x1 << 1);}
inline void rcc_start_TIM4(void)    {RCC_APB1ENR |= (0x1 << 2);}
inline void rcc_start_TIM5(void)    {RCC_APB1ENR |= (0x1 << 3);}
inline void rcc_start_TIM6(void)    {RCC_APB1ENR |= (0x1 << 4);}
inline void rcc_start_TIM7(void)    {RCC_APB1ENR |= (0x1 << 5);}
inline void rcc_start_TIM12(void)   {RCC_APB1ENR |= (0x1 << 6);}
inline void rcc_start_TIM13(void)   {RCC_APB1ENR |= (0x1 << 7);}
inline void rcc_start_TIM14(void)   {RCC_APB1ENR |= (0x1 << 8);}
inline void rcc_start_WWDG(void)    {RCC_APB1ENR |= (0x1 << 11);}
inline void rcc_start_SPI2(void)    {RCC_APB1ENR |= (0x1 << 14);}
inline void rcc_start_SPI3(void)    {RCC_APB1ENR |= (0x1 << 15);}
inline void rcc_start_SPDIFRX(void) {RCC_APB1ENR |= (0x1 << 16);}
inline void rcc_start_USART2(void)  {RCC_APB1ENR |= (0x1 << 17);}
inline void rcc_start_USART3(void)  {RCC_APB1ENR |= (0x1 << 18);}
inline void rcc_start_USART4(void)  {RCC_APB1ENR |= (0x1 << 19);}
inline void rcc_start_USART5(void)  {RCC_APB1ENR |= (0x1 << 20);}
inline void rcc_start_I2C1(void)    {RCC_APB1ENR |= (0x1 << 21);}
inline void rcc_start_I2C2(void)    {RCC_APB1ENR |= (0x1 << 22);}
inline void rcc_start_I2C3(void)    {RCC_APB1ENR |= (0x1 << 23);}
inline void rcc_start_FMPI2C1(void) {RCC_APB1ENR |= (0x1 << 24);}
inline void rcc_start_CAN1(void)    {RCC_APB1ENR |= (0x1 << 25);}
inline void rcc_start_CAN2(void)    {RCC_APB1ENR |= (0x1 << 26);}
inline void rcc_start_CEC(void)     {RCC_APB1ENR |= (0x1 << 27);}
inline void rcc_start_PWR(void)     {RCC_APB1ENR |= (0x1 << 28);}
inline void rcc_start_DAC(void)     {RCC_APB1ENR |= (0x1 << 29);}

inline void rcc_start_TIM1(void)    {RCC_APB2ENR |= (0x1 << 0);}
inline void rcc_start_TIM8(void)    {RCC_APB2ENR |= (0x1 << 1);}
inline void rcc_start_USART1(void)  {RCC_APB2ENR |= (0x1 << 4);}
inline void rcc_start_USART6(void)  {RCC_APB2ENR |= (0x1 << 5);}
inline void rcc_start_ADC1(void)    {RCC_APB2ENR |= (0x1 << 8);}
inline void rcc_start_ADC2(void)    {RCC_APB2ENR |= (0x1 << 9);}
inline void rcc_start_ADC3(void)    {RCC_APB2ENR |= (0x1 << 10);}
inline void rcc_start_SDIO(void)    {RCC_APB2ENR |= (0x1 << 11);}
inline void rcc_start_SPI1(void  )  {RCC_APB2ENR |= (0x1 << 12);}
inline void rcc_start_SPI4(void)    {RCC_APB2ENR |= (0x1 << 13);}
inline void rcc_start_SYSCFG(void)  {RCC_APB2ENR |= (0x1 << 14);}
inline void rcc_start_TIM9(void)    {RCC_APB2ENR |= (0x1 << 16);}
inline void rcc_start_TIM10(void)   {RCC_APB2ENR |= (0x1 << 17);}
inline void rcc_start_TIM11(void)   {RCC_APB2ENR |= (0x1 << 18);}
inline void rcc_start_SAI1(void)    {RCC_APB2ENR |= (0x1 << 22);}
inline void rcc_start_SAI2(void)    {RCC_APB2ENR |= (0x1 << 23);}

uint32_t rcc_get_APB1_Freq ();
uint32_t rcc_get_APB2_Freq ();
uint32_t rcc_get_SYSCLK_Freq ();

#endif

#endif /* SYS_H_ */
