/*
 * Core driver for data_dma_direct IP
 *
 * (c) Copyright 2020 	OscillatorIMP Digital
 * Gwenhael Goavec-Merou <gwenhael.goavec-merou@trabucayre.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#ifndef DATA_DMA_DIRECT_CONFIG_H
#define DATA_DMA_DIRECT_CONFIG_H

#define  DATA_DMA_DIRECT_REG_ID		(0x00 << 2)
#define  DATA_DMA_DIRECT_REG_START  (0x01 << 2)
#define  DATA_DMA_DIRECT_REG_SIZE	(0x02 << 2)

/* ioctl */
#define DATA_DMA_DIRECT_GET_IP_BUFFER_BYTE _IOR('a', DATA_DMA_DIRECT_REG_SIZE, long)

#endif
