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

#include "stdint.h"
#include "util.h"
#include "nassert.h"
#include "gpio.h"

void GPIO_init_pin_input(GPIO_TypeDef *port, unsigned pin, unsigned pullupdown)
{
	nassert(pin < 16U);
	nassert(pullupdown < 3U);

	port->MODER &= ~(0x03 << (2 * pin)); /* reset MODER for pin */
	port->MODER |= (GPIO_MODER_INPUT << (2 * pin)); /* set MODER for pin */

	port->PUPDR &= ~(0x3 << (2 * pin)); /* pull up down reset */
	port->PUPDR |= pullupdown << (2 * pin);

}

void GPIO_init_pin_output(GPIO_TypeDef *port, unsigned pin, unsigned ppod,
		unsigned speed, unsigned pupd)
{

	nassert (pin<16U);
	nassert (ppod<2U);
	nassert (speed<2U);
	nassert (pupd<3U);

	port->MODER &= ~(0x03 << (2 * pin)); /* reset MODER for pin */
	port->MODER |= (GPIO_MODER_OUTPUT << (2 * pin)); /* set MODER for pin */

	port->OTYPER &= ~(0x01 << (1 * pin)); /* reset OTYPER for pin */
	port->OTYPER |= ppod << (1 * pin);

	port->OSPEEDR &= ~(0x03 << (2 * pin)); /* reset OSPEEDR for pin */
	port->OSPEEDR |= speed << (2 * pin);

	port->PUPDR &= ~(0x03 << (2 * pin)); /* reset PUPDR for pin */
	port->PUPDR |= pupd << (2 * pin);

}

void GPIO_init_pin_altfct(GPIO_TypeDef *port, unsigned pin, unsigned ppod,
		unsigned speed, unsigned pupd, unsigned af)
{

	nassert (pin<16);
	nassert (ppod<2);
	nassert (speed<4);
	nassert (pupd<3U);
	nassert (af<16);

	port->MODER &= ~(0x03 << (2 * pin)); /* reset MODER for pin */
	port->MODER |= (GPIO_MODER_ALTFCT << (2 * pin)); /* set MODER for pin */

	port->OTYPER &= ~(0x01 << (1 * pin)); /* reset OTYPER for pin */
	port->OTYPER |= ppod << (1 * pin);

	port->OSPEEDR &= ~(0x03 << (2 * pin)); /* reset OSPEEDR for pin */
	port->OSPEEDR |= speed << (2 * pin);

	port->PUPDR &= ~(0x03 << (2 * pin)); /* pull up down reset */
	port->PUPDR |= pupd << (2 * pin);

	if (pin < 8)
	{
		port->AFRL &= ~(0x0F << (4 * pin));
		port->AFRL |= (af << (4 * pin));
	}
	else
	{
		port->AFRH &= ~(0x0F << (4 * (pin-8)));
		port->AFRH |= (af << (4 * (pin-8)));
	}
}

void GPIO_init_pin_adc(GPIO_TypeDef *port, unsigned pin)
{
	nassert (pin<16);

	/* Set MODER  */
	port->MODER &= ~(0x03 << (2 * pin)); /* reset MODER for pin */
	port->MODER |= (GPIO_MODER_ANALOG << (2 * pin)); /* set MODER for pin */

	port->OTYPER &= ~(0x01 << (1 * pin)); /* reset OTYPER for pin */

	port->OSPEEDR &= ~(0x03 << (2 * pin)); /* reset OSPEEDR for pin */

	port->PUPDR &= ~(0x03 << (2 * pin)); /* pull up down reset */

}

void GPIO_init_pin_unused(GPIO_TypeDef *port, unsigned pin)
{
	/* Unused pins are set in input mode and are pulled down */
	/* for noise reduction */
	port->MODER &= ~(0x03 << (2 * pin)); /* reset MODER for pin */
	port->MODER |= (GPIO_MODER_INPUT << (2 * pin)); /* set MODER for pin */

	port->PUPDR &= ~(0x3 << (2 * pin)); /* pull up down reset */
	port->PUPDR |= GPIO_PUPDR_PULLDOWN << (2 * pin);

}
