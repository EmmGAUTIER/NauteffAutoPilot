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

#include <stdint.h>
#include <stdbool.h>
#include "util.h"
#include "stm32F446.h"
#include "dma.h"
#include "nassert.h"

void DMA_init_Stream(DMA_regs_t *dma, uint32_t stream)
{
	nassert(stream < 8);
	nassert((dma == &DMA1) || (dma == &DMA2));

	/*DMA_Stream_regs_t */

	dma->DMA_LIFCR = 0x0;
	dma->DMA_HIFCR = 0x0;

	dma->streamRegs[stream].DMA_SxCR =0x0;
	dma->streamRegs[stream].DMA_SxCR =0x0;

	return;
}
