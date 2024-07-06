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

/**
 * Main types declarations.
 */

#ifndef UTIL_H_
#define UTIL_H_

#include <stddef.h>
#include <stdint.h>

#define INLINE __attribute__((always_inline)) inline
#define bool _Bool

typedef volatile uint8_t vuint8_t;
typedef volatile uint16_t vuint16_t;
typedef volatile uint32_t vuint32_t;

typedef volatile int8_t vint8_t;
typedef volatile int16_t vint16_t;
typedef volatile int32_t vint32_t;

__attribute__((always_inline))
inline void setbits   (vuint32_t * i, const vuint32_t m) { *i |= m; }
__attribute__((always_inline))
inline void clearbits (vuint32_t * i, const vuint32_t m) { *i &= ~m; }
__attribute__((always_inline))
inline void setbit     (vuint32_t *i, const int bn)    { *i |= (0x01<<bn); }
__attribute__((always_inline))
inline void clearbit   (vuint32_t *i, const int bn)    { *i &= ~(0x1<<bn); }

//#define MAKE_LIST (...) __VA_ARGS__

#endif /* UTIL_H_ */
