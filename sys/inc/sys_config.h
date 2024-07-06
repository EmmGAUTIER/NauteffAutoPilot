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

#ifndef SYSCFG_H_
#define SYSCFG_H_

#define SYS_USE_DMA1              1
#define SYS_USE_DMA2              1

#define SYS_USE_ADC1              1
#define SYS_USE_ADC2              1
#define SYS_USE_ADC3              0

#define SYS_USE_USART1            1
#define SYS_USE_USART2            0
#define SYS_USE_USART3            0
#define SYS_USE_USART4            0
#define SYS_USE_USART5            1
#define SYS_USE_USART6            0
#define SYS_USARTX_RX_BUFFSIZE  128
#define SYS_USARTX_TX_BUFFSIZE  128

#define SYS_USE_I2C1              1
#define SYS_USE_I2C2              0
#define SYS_USE_I2C3              1

#endif /* SYSCFG_H_ */
