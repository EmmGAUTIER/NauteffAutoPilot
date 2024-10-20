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

typedef struct {
	vuint32_t DMA_SxCR;
	vuint32_t DMA_SxNDTR;
	vuint32_t DMA_SxPAR;
	vuint32_t DMA_SxM0AR;
	vuint32_t DMA_SxM1AR;
	vuint32_t DMA_SxFCR;
}DMA_Stream_regs_t;

typedef struct {
    vuint32_t DMA_LISR;
    vuint32_t DMA_HISR;
    vuint32_t DMA_LIFCR;
    vuint32_t DMA_HIFCR;
    DMA_Stream_regs_t streamRegs[8];
}DMA_regs_t;

#define DMA1 (*(DMA_regs_t*)(0x40026000))
#define DMA2 (*(DMA_regs_t*)(0x40026400))

void DMA_init_Stream(DMA_regs_t* dma, uint32_t stream);
void DMA_set_chanel_select (DMA_regs_t* dma , uint32_t stream, unsigned cn);
void DMA_set_priority (DMA_regs_t* dma, uint32_t stream, unsigned pri);
void DMA_enable_memory_increment (DMA_regs_t* dma, uint32_t stream);
void DMA_disable_memory_increment (DMA_regs_t* dma, uint32_t stream);
void DMA_enable_circular_mode (DMA_regs_t* dma, uint32_t stream);
void DMA_disable_circular_mode (DMA_regs_t* dma, uint32_t stream);
void DMA_set_periph_to_mem_dir (DMA_regs_t* dma, uint32_t stream);
void DMA_set_mem_to_periph_dir (DMA_regs_t* dma, uint32_t stream);
void DMA_set_mem_to_mem_dir (DMA_regs_t* dma, uint32_t stream);
void DMA_enable_transfer_complete_intr (DMA_regs_t* dma, uint32_t stream);
void DMA_enable_half_transfer_intr (DMA_regs_t* dma, uint32_t stream);
void DMA_enable_transfer_error_intr (DMA_regs_t* dma, uint32_t stream);
void DMA_enable_dir_mode_error_intr (DMA_regs_t* dma, uint32_t stream);
void DMA_enable_stream (DMA_regs_t* dma, uint32_t stream);
void DMA_set_peripheral_add (DMA_regs_t*, uint32_t stream, void*);
void DMA_set_memory0_add (DMA_regs_t*, uint32_t stream, void*);
void DMA_set_memory1_add (DMA_regs_t*, uint32_t stream, void*);
void DMA_set_number_of_data (DMA_regs_t* dma , uint32_t stream, unsigned nd);
uint32_t DMA_get_intr_status_reg (DMA_regs_t* dma, uint32_t stream);
bool DMA_get_transfer_complete_intr_flag (DMA_regs_t* dma, uint32_t stream);
bool DMA_get_half_transfer_intr_flag (DMA_regs_t* dma, uint32_t stream);
bool DMA_get_transfer_error_intr_flag (DMA_regs_t* dma, uint32_t stream);
bool DMA_get_direct_mode_error_intr_flag (DMA_regs_t* dma, uint32_t stream);
/*
bool DMA_get_XXX_flag (DMA_regs_t* dma, uint32_t stream);
bool DMA_get_XXX_flag (DMA_regs_t* dma, uint32_t stream);
bool DMA_get_XXX_flag (DMA_regs_t* dma, uint32_t stream);
*/
