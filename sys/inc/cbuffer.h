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

#include "util.h"

typedef struct {
	char *buff;
	size_t sz;
	vuint16_t idx_start;
	vuint16_t idx_end;
	vuint16_t state;
} CBuffer_t;

__attribute__((always_inline))
   inline CBuffer_t* cbuffer_create(size_t psz) {

	char *buff;

	CBuffer_t *cbuffer = malloc(sizeof(CBuffer_t));
	buff = malloc(psz);
	if ((cbuffer != NULL) && (buff != NULL)) {
		cbuffer->sz = psz;
		cbuffer->buff = buff;
		cbuffer->idx_start = 0U;
		cbuffer->idx_end = 0U;
		cbuffer->state = 0U;
	} else {
		free(cbuffer);
		free(buff);
		cbuffer = (CBuffer_t*)0U;
	}

	return cbuffer;
}

__attribute__((always_inline))
    inline size_t cbuffer_getSize(CBuffer_t *pcbuff) {
	return pcbuff->sz;
}

__attribute__((always_inline))
inline int cbuffer_putChar(CBuffer_t pcbuff, char pchar) {

	int res;

	if ((pcbuff->idx_start + 1) % pcbuff->sz != pcbuff->idx_end) {
		pcbuff->buff[pcbuff->idx_start] = pchar;
		pcbuff->idx_start = (pcbuff->idx_start + 1) % pcbuff->sz;
		res = (int) pchar;
	} else {/*Full Buffer*/
		res = -1;
	}

	return res;
}

__attribute__((always_inline))
    inline size_t cbuffer_putString(CBuffer_t pcbuff) {
	return pcbuff->sz;
}

__attribute__((always_inline))
    inline size_t cbuffer_putString(CBuffer_t pcbuff) {
	return pcbuff->sz;
}

__attribute__((always_inline))
    inline size_t cbuffer_getString(CBuffer_t pcbuff) {
	return pcbuff->sz;
}

__attribute__((always_inline))
    inline size_t cbuffer_getChar(CBuffer_t pcbuff) {
	return pcbuff->sz;
}

