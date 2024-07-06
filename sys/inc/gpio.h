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

#ifndef GPIO_H
#define GPIO_H

typedef struct
{
    vuint32_t MODER;   /* Mode                      */
    vuint32_t OTYPER;  /* Port output type register */
    vuint32_t OSPEEDR; /* Output speed              */
    vuint32_t PUPDR;   /* Pull Up Pull Down         */
    vuint32_t IDR;     /* Input Data                */
    vuint32_t ODR;     /* Output Data               */
    vuint32_t BSRR;    /* Bit Set Reset             */
    vuint32_t LCKR;    /* Lock                      */
    vuint32_t AFRL;    /* Alternate Function Low    */
    vuint32_t AFRH;    /* Alternate Function High   */
} GPIO_TypeDef;

#define GPIO_BASE 0x40020000

#define GPIOA ((GPIO_TypeDef *)(GPIO_BASE + 0x0000))
#define GPIOB ((GPIO_TypeDef *)(GPIO_BASE + 0x0400))
#define GPIOC ((GPIO_TypeDef *)(GPIO_BASE + 0x0800))
#define GPIOD ((GPIO_TypeDef *)(GPIO_BASE + 0x0C00))
#define GPIOE ((GPIO_TypeDef *)(GPIO_BASE + 0x1000))
#define GPIOF ((GPIO_TypeDef *)(GPIO_BASE + 0x1400))
#define GPIOG ((GPIO_TypeDef *)(GPIO_BASE + 0x1800))
#define GPIOH ((GPIO_TypeDef *)(GPIO_BASE + 0x1C00))

#define GPIO_MODER_INPUT (0x0)
#define GPIO_MODER_OUTPUT (0x1)
#define GPIO_MODER_ALTFCT (0x2)
#define GPIO_MODER_ANALOG (0x3)

#define GPIO_OTYPER_PUSHPULL (0x0)
#define GPIO_OTYPER_OPENDRAIN (0x1)

#define GPIO_OSPEEDR_LOWSPEED (0x00)
#define GPIO_OSPEEDR_MEDIUMSPEED (0x01)
#define GPIO_OSPEEDR_FASTSPEED (0x02)
#define GPIO_OSPEEDR_HIGHSPEED (0x03)

#define GPIO_PUPDR_NONE (0x00)
#define GPIO_PUPDR_PULLUP (0x01)
#define GPIO_PUPDR_PULLDOWN (0x02)

/// @brief Initialise une broche
/// @param port
/// @param pin
/// @param pullupdown
void GPIO_init_pin_input(GPIO_TypeDef *port,
                         unsigned pin,
                         unsigned pullupdown);

/// @brief
/// @param port
/// @param pin
/// @param ppod
/// @param speed
/// @param pullupdown
/// @return void
void GPIO_init_pin_output(GPIO_TypeDef *port, unsigned pin, unsigned ppod,
                          unsigned speed, unsigned pullupdown);

void GPIO_init_pin_altfct(GPIO_TypeDef *port, unsigned pin, unsigned ppod,
                          unsigned speed, unsigned pullupdown, unsigned af);

void GPIO_init_pin_adc(GPIO_TypeDef *port, unsigned pin);

void GPIO_init_pin_unused(GPIO_TypeDef *port, unsigned pin);

__attribute__((always_inline)) inline void GPIO_pin_set(GPIO_TypeDef *port, unsigned pin)
{
    port->BSRR = (0x1 << pin);
}

__attribute__((always_inline)) inline void GPIO_pin_reset(GPIO_TypeDef *port, unsigned pin)
{
    port->BSRR = (0x1 << (pin + 16U));
}

/**
 *
 * @param[in] port IO port register structure
 * @param[in] mask
 * @return none, parameter checking with assert when debugging
 */
__attribute__((always_inline)) inline void GPIO_pins_set(GPIO_TypeDef *port, uint32_t maskset)
{
    port->BSRR = (maskset);
}

__attribute__((always_inline)) inline void GPIO_pins_reset(GPIO_TypeDef *port,
                                                           uint32_t maskreset)
{
    port->BSRR = (maskreset << 16U);
}

__attribute__((always_inline)) inline void GPIO_pins_setreset(GPIO_TypeDef *port,
                                                              uint32_t maskset,
                                                              uint32_t maskreset)
{
    port->BSRR = (maskset) & (maskreset << 16U);
}

/*
 * @param port
 * @param pin
 * @return value (0 or 1) of pin
 */

__attribute__((always_inline)) inline int GPIO_pin_get(GPIO_TypeDef *port, int pin)
{
    return (port->IDR & (0x1 << pin)) ? 1 : 0;
}

/*
 * @param port
 * @param mask mask of the bits to select
 * This function is inline.
 * @return bits of a port selected by mask
 */
__attribute__((always_inline)) inline uint32_t GPIO_pins_get(GPIO_TypeDef *port, uint32_t mask)
{
    return (uint32_t)(port->IDR & mask);
}

#endif /* #ifndef GPIO_H */
