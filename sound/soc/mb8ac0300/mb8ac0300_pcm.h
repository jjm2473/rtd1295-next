/*
 * linux/sound/soc/mb8ac0300/mb8ac0300_pcm.h
 *
 * Copyright (C) 2011-2012 FUJITSU SEMICONDUCTOR LIMITED
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _MB8AC0300_PCM_H
#define _MB8AC0300_PCM_H

#define MB8AC0300_PCM_ST_RUNNING	(1)	/* running state */

/* dma parameters structure */
struct mb8ac0300_pcm_dma_params {
	dma_addr_t dma_addr;	/* address of I2S TX/RX register */
	int dmacb_tw;		/* TW of dmaca register */
	int dma_width;		/* DMA transfer width(actual bytes number) */
	int channel;		/* DMA channel */
	int dmaca_is;		/* IS of dmaca register */
};

/* runtime data of pcm driver */
struct mb8ac0300_pcm_runtime_data {
	spinlock_t lock;			/* lock */
	int state;				/* state of pcm */
	unsigned int dma_loaded;		/* loaded count of dma */
	unsigned int dma_period;		/* period bytes */
	dma_addr_t dma_start;			/* start pos of dma buffer */
	dma_addr_t dma_pos;			/* current pos of dma buffer */
	dma_addr_t dma_end;			/* end position of dma buffer */
	struct mb8ac0300_pcm_dma_params params;/* parameters of dma */
};

#endif/*_MB8AC0300_PCM_H*/
