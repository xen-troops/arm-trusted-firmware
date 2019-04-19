/*
 * Copyright (c) 2016,2017 ARM Limited and Contributors. All rights reserved.
 * Copyright (c) 2017 EPAM Systems Inc. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * Neither the name of ARM nor the names of its contributors may be used
 * to endorse or promote products derived from this software without specific
 * prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <debug.h>
#include <mmio.h>
#include <sys/errno.h>
#include "rcar_def.h"
#include "rcar_private.h"

#define GENMASK(hi, lo) (BIT(hi) - 1 - BIT(lo) + 1)

#define SCPI_OK		0
#define SCPI_E_PARAM	1
#define SCPI_E_ALIGN	2
#define SCPI_E_SIZE	3
#define SCPI_E_HANDLER	4
#define SCPI_E_ACCESS	5
#define SCPI_E_RANGE	6
#define SCPI_E_TIMEOUT	7
#define SCPI_E_NOMEM	8
#define SCPI_E_PWRSTATE	9
#define SCPI_E_SUPPORT	10
#define SCPI_E_DEVICE	11
#define SCPI_E_BUSY	12

#define SCP_CMD_CAPABILITY	0x02
#define SCP_CMD_DVFS_CAPABILITY	0x08
#define SCP_CMD_DVFS_GET_INFO	0x09
#define SCP_CMD_DVFS_SET_INDEX	0x0a
#define SCP_CMD_DVFS_GET_INDEX	0x0b
#define SCP_CMD_DVFS_GET_STAT	0x0c
#define SCP_CMD_CLOCKS_CAPS	0x0d
#define SCP_CMD_CLOCK_GET_INFO	0x0e
#define SCP_CMD_CLOCK_SET_RATE	0x0f
#define SCP_CMD_CLOCK_GET_RATE	0x10

#define SCP_CMDS_IMPLEMENTED						\
	GENMASK(SCP_CMD_DVFS_GET_INDEX, SCP_CMD_DVFS_CAPABILITY)	|	\
	GENMASK(SCP_CMD_CLOCK_GET_RATE, SCP_CMD_CLOCKS_CAPS)

#define RCAR_SCPI_SHMEM_BASE	DRAM1_NS_SCPI_BASE

static int write_clock_info(uintptr_t payload, int clocknr)
{
	const char *name, *s;
	int i;

	name = rcar_clock_get_name(clocknr);
	if (!name)
		return -SCPI_E_PARAM;

	mmio_write_32(payload + 0x0, (clocknr & 0xffff) | (0x03 << 16));
	mmio_write_32(payload + 0x4, rcar_clock_get_min_rate(clocknr));
	mmio_write_32(payload + 0x8, rcar_clock_get_max_rate(clocknr));
	for (i = 0, s = name; s[i] != 0; i++)
		mmio_write_8(payload + 12 + i, s[i]);
	mmio_write_8(payload + 12 + i, 0);

	return 12 + i;
}

static uint32_t scpi_handle_cmd(int cmd, uint8_t *payload_size,
				uintptr_t payload_in, uintptr_t payload_out)
{
	uint32_t par1 = mmio_read_32(payload_in);
	int ret;

	/*NOTICE("%s: The command is 0x%x (par1=0x%x)\n", __func__, cmd, par1);*/

	switch (cmd) {
	case SCP_CMD_CAPABILITY:
		mmio_write_32(payload_out + 0x00, (1U << 16) | (2U << 0));
		/*
		 * The SCPI spec says this field holds the payload sizes for
		 * the receive and transmit channel, but the Linux driver
		 * decodes an event version ID from it.
		 * Let's play nice with Linux for now and ignore the spec.
		 *
		 * mmio_write_32(payload_out + 0x04,
		 *		 ((256 - 1) << 16) | (256 - 1));
		 */
		mmio_write_32(payload_out + 0x04, 1U << 16);

		mmio_write_32(payload_out + 0x08, 1U << 24);
		mmio_write_32(payload_out + 0x0c, SCP_CMDS_IMPLEMENTED);
		mmio_write_32(payload_out + 0x10, 0x0);
		mmio_write_32(payload_out + 0x14, 0x0);
		mmio_write_32(payload_out + 0x18, 0x0);
		*payload_size = 0x1c;
		return SCPI_OK;
	case SCP_CMD_CLOCKS_CAPS:
		/* number of implemented clocks */
		mmio_write_32(payload_out, rcar_clock_nr_clocks());
		*payload_size = 0x4;
		return SCPI_OK;
	case SCP_CMD_CLOCK_GET_INFO:
		ret = write_clock_info(payload_out, par1 & 0xffff);
		if (ret < 0) {
			*payload_size = 0;
			return SCPI_E_PARAM;
		}

		*payload_size = ret;
		return SCPI_OK;
	case SCP_CMD_CLOCK_SET_RATE: {
		uint32_t freq = mmio_read_32(payload_in + 4);

		*payload_size = 0;
		ret = rcar_clock_set_rate(par1 & 0xffff, freq);
		if (ret < 0)
			return SCPI_E_RANGE;

		return SCPI_OK;
	}
	case SCP_CMD_CLOCK_GET_RATE:
		ret = rcar_clock_get_rate(par1 & 0xffff);
		if (ret < 0) {
			*payload_size = 0;
			return SCPI_E_RANGE;
		}

		mmio_write_32(payload_out, ret);
		*payload_size = 4;
		return SCPI_OK;
	case SCP_CMD_DVFS_CAPABILITY:
		/* number of implemented voltage domains: only one */
		mmio_write_32(payload_out, 1);
		*payload_size = 0x1;
		return SCPI_OK;
	case SCP_CMD_DVFS_GET_INFO: {
		int i, nr_opp = rcar_dvfs_get_nr_opp();

		mmio_write_32(payload_out, nr_opp << 8);
		for (i = 0; i < nr_opp; i++) {
			mmio_write_32(payload_out + 4 + 2 * i * 4,
					rcar_dvfs_get_get_opp_frequency(i));
			mmio_write_32(payload_out + 4 + 2 * i * 4 + 4,
					rcar_dvfs_get_get_opp_voltage(i));
		}
		*payload_size = 4 + 2 * nr_opp * 4;
		return SCPI_OK;
	}
	case SCP_CMD_DVFS_SET_INDEX:
		if ((par1 & 0xff) != 0)
			return SCPI_E_PARAM;

		if (rcar_dvfs_set_index((par1 >> 8) & 0xff))
			return SCPI_E_RANGE;
		return SCPI_OK;
	case SCP_CMD_DVFS_GET_INDEX:
		mmio_write_32(payload_out, rcar_dvfs_get_index());
		*payload_size = 0x1;
		return SCPI_OK;
	}

	return SCPI_E_SUPPORT;
}

uint32_t rcar_trigger_scpi(uint32_t x1, uint32_t x2, uint32_t x3, uint32_t x4)
{
	uint32_t ret;
	uint64_t scpi_header;
	uint8_t payload_size;

	scpi_header = *(uint64_t *)(RCAR_SCPI_SHMEM_BASE + 0x100);
	payload_size = (scpi_header >> 16) & 0xff;

	ret = scpi_handle_cmd(scpi_header & 0xff, &payload_size,
			RCAR_SCPI_SHMEM_BASE + 0x108,
			RCAR_SCPI_SHMEM_BASE + 0x8);

	if (ret)
		NOTICE("%s: Failed to process command (%u)\n", __func__, ret);

	mmio_write_32(RCAR_SCPI_SHMEM_BASE, (scpi_header & 0xffff) |
			(uint32_t)payload_size << 16);
	mmio_write_32(RCAR_SCPI_SHMEM_BASE + 4, ret);

	return ret;
}
