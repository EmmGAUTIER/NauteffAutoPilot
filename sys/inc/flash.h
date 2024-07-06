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

#ifndef INC_FLASH_H_
#define INC_FLASH_H_

#define FLASH_BASE 0x40023C00

#define FLASH_ACR       (*(vuint32_t*)(FLASH_BASE + 0x00))
#define FLASH_KEYR      (*(vuint32_t*)(FLASH_BASE + 0x04))
#define FLASH_OPTKEYR   (*(vuint32_t*)(FLASH_BASE + 0x08))
#define FLASH_SR        (*(vuint32_t*)(FLASH_BASE + 0x0C))
#define FLASH_CR        (*(vuint32_t*)(FLASH_BASE + 0x10))
#define FLASH_OPTCR     (*(vuint32_t*)(FLASH_BASE + 0x14))

#endif /* INC_FLASH_H_ */
