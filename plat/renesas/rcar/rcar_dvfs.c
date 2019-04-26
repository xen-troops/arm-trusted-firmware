/*
 * Copyright (c) 2016,2017 ARM Limited and Contributors. All rights reserved.
 * Copyright (c) 2015-2017 Renesas Electronics Corporation. All rights reserved.
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
#include "rcar_def.h"
#include "rcar_private.h"
#include "iic_dvfs.h"

/* TODO These should be taken from avs_driver.c */
#define EFUSE_AVS0			(0U)
#define EFUSE_AVS_NUM		(8U)
static uint32_t efuse_avs = EFUSE_AVS0;

struct op_points
{
	unsigned long freq;	/* Hz */
	unsigned long volt;	/* uV */
};

#define A57_DOMAIN	0
#define A53_DOMAIN	1

#define NR_H3_A57_OPP	5
#define NR_M3_A57_OPP	6

#define NR_H3_A53_OPP	3
#define NR_M3_A53_OPP	4

#define ARRAY_SIZE(a)	(sizeof(a) / sizeof((a)[0]))

/* Describe OPPs exactly how they are described in the device-tree */
static const struct op_points rcar_h3_a57_op_points[EFUSE_AVS_NUM][NR_H3_A57_OPP] = {
	{
		{ 500000000,  830000, },
		{ 1000000000, 830000, },
		{ 1500000000, 830000, },
		{ 1600000000, 900000, },
		{ 1700000000, 960000, },
	},
	{
		{ 500000000,  820000, },
		{ 1000000000, 820000, },
		{ 1500000000, 820000, },
		{ 1600000000, 890000, },
		{ 1700000000, 950000, },
	},
	{
		{ 500000000,  810000, },
		{ 1000000000, 810000, },
		{ 1500000000, 810000, },
		{ 1600000000, 880000, },
		{ 1700000000, 930000, },
	},
	{
		{ 500000000,  800000, },
		{ 1000000000, 800000, },
		{ 1500000000, 800000, },
		{ 1600000000, 870000, },
		{ 1700000000, 910000, },
	},
	{
		{ 500000000,  790000, },
		{ 1000000000, 790000, },
		{ 1500000000, 790000, },
		{ 1600000000, 860000, },
		{ 1700000000, 890000, },
	},
	{
		{ 500000000,  780000, },
		{ 1000000000, 780000, },
		{ 1500000000, 780000, },
		{ 1600000000, 850000, },
		{ 1700000000, 880000, },
	},
	{
		{ 500000000,  770000, },
		{ 1000000000, 770000, },
		{ 1500000000, 770000, },
		{ 1600000000, 840000, },
		{ 1700000000, 870000, },
	},
	{
		{ 500000000,  760000, },
		{ 1000000000, 760000, },
		{ 1500000000, 760000, },
		{ 1600000000, 830000, },
		{ 1700000000, 860000, },
	},
};

static const struct op_points rcar_h3_a53_op_points[NR_H3_A53_OPP] = {
	{ 800000000,  820000, },
	{ 1000000000, 820000, },
	{ 1200000000, 820000, },
};

static const struct op_points rcar_m3_a57_op_points[EFUSE_AVS_NUM][NR_M3_A57_OPP] = {
	{
		{ 500000000,  830000, },
		{ 1000000000, 830000, },
		{ 1500000000, 830000, },
		{ 1600000000, 900000, },
		{ 1700000000, 900000, },
		{ 1800000000, 960000, },
	},
	{
		{ 500000000,  820000, },
		{ 1000000000, 820000, },
		{ 1500000000, 820000, },
		{ 1600000000, 890000, },
		{ 1700000000, 890000, },
		{ 1800000000, 950000, },
	},
	{
		{ 500000000,  810000, },
		{ 1000000000, 810000, },
		{ 1500000000, 810000, },
		{ 1600000000, 880000, },
		{ 1700000000, 880000, },
		{ 1800000000, 930000, },
	},
	{
		{ 500000000,  800000, },
		{ 1000000000, 800000, },
		{ 1500000000, 800000, },
		{ 1600000000, 870000, },
		{ 1700000000, 870000, },
		{ 1800000000, 910000, },
	},

	{
		{ 500000000,  790000, },
		{ 1000000000, 790000, },
		{ 1500000000, 790000, },
		{ 1600000000, 860000, },
		{ 1700000000, 860000, },
		{ 1800000000, 890000, },
	},
	{
		{ 500000000,  780000, },
		{ 1000000000, 780000, },
		{ 1500000000, 780000, },
		{ 1600000000, 850000, },
		{ 1700000000, 850000, },
		{ 1800000000, 880000, },
	},
	{
		{ 500000000,  770000, },
		{ 1000000000, 770000, },
		{ 1500000000, 770000, },
		{ 1600000000, 840000, },
		{ 1700000000, 840000, },
		{ 1800000000, 870000, },
	},
	{
		{ 500000000,  760000, },
		{ 1000000000, 760000, },
		{ 1500000000, 760000, },
		{ 1600000000, 830000, },
		{ 1700000000, 830000, },
		{ 1800000000, 860000, },
	},
};

static const struct op_points rcar_m3_a53_op_points[NR_M3_A53_OPP] = {
	{ 800000000,  820000, },
	{ 1000000000, 820000, },
	{ 1200000000, 820000, },
	{ 1300000000, 820000, },
};

#define DIV_ROUND(n, d) (((n) + (d) / 2) / (d))
#define min(x, y)		(((x) < (y)) ? (x) : (y))
#define max(x, y)		(((x) > (y)) ? (x) : (y))

/* CPG base address */
#define	CPG_BASE		(0xE6150000U)

#define CPG_PLLECR		0x00D0
#define CPG_PLL0CR		0x00d8
#define CPG_PLL2CR		0x002c

/* Implementation for customized clocks (Z-clk, Z2-clk, PLL0-clk) for CPUFreq */
#define CPG_PLLECR_PLL0ST	BIT(8)
#define CPG_PLLECR_PLL2ST	BIT(10)

/* Define for PLL0 clk driver */
#define CPG_PLLCR_STC_MASK             0x7f000000
#define CPG_PLLCR_STC_SHIFT            24

/* Modify for Z-clock and Z2-clock
 *
 * Traits of this clock:
 * prepare - clk_prepare only ensures that parents are prepared
 * enable - clk_enable only ensures that parents are enabled
 * rate - rate is adjustable.  clk->rate = parent->rate * mult / 32
 * parent - fixed parent.  No clk_set_parent support
 */
#define CPG_FRQCRB			0x00000004
#define CPG_FRQCRB_KICK			BIT(31)
#define CPG_FRQCRC			0x000000e0
#define CPG_FRQCRC_ZFC_MASK		(0x1f << 8)
#define CPG_FRQCRC_ZFC_SHIFT		8
#define CPG_FRQCRC_Z2FC_MASK		0x1f

#define Z_CLK_MAX_THRESHOLD             1500000000U
#define Z2_CLK_MAX_THRESHOLD            1200000000U

static int current_a57_opp_index;
static int current_a57_opp_latency;
static int current_a57_opp_limit = 0;
static const struct op_points *current_a57_opp_table;

static int current_a53_opp_index;
static int current_a53_opp_latency;
static int current_a53_opp_limit = 0;
static const struct op_points *current_a53_opp_table;

static int dvfs_inited = 0;

uint32_t rcar_dvfs_get_opp_voltage(int domain, int oppnr)
{
	if (domain == A57_DOMAIN) {
		if (oppnr < 0 || oppnr >= current_a57_opp_limit)
			return ~0;

		/* Protocol requires voltage to be in mV */
		return current_a57_opp_table[oppnr].volt / 1000;
	} else if (domain == A53_DOMAIN) {
		if (oppnr < 0 || oppnr >= current_a53_opp_limit)
			return ~0;

		/* Protocol requires voltage to be in mV */
		return current_a53_opp_table[oppnr].volt / 1000;
	}

	return ~0;
}

uint32_t rcar_dvfs_get_opp_frequency(int domain, int oppnr)
{
	if (domain == A57_DOMAIN) {
		if (oppnr < 0 || oppnr >= current_a57_opp_limit)
			return ~0;

		/* Protocol requires frequency to be in Hz */
		return current_a57_opp_table[oppnr].freq;
	} else if (domain == A53_DOMAIN) {
		if (oppnr < 0 || oppnr >= current_a53_opp_limit)
			return ~0;

		/* Protocol requires frequency to be in Hz */
		return current_a53_opp_table[oppnr].freq;
	}

	return ~0;
}

static unsigned long pll_clk_parent_rate(void)
{
	static const unsigned long extal_freq[] = {
			16660000U,	/* MD14_MD13_TYPE_0 */
			20000000U,	/* MD14_MD13_TYPE_1 */
			25000000U,	/* MD14_MD13_TYPE_2 */
			33330000U,	/* MD14_MD13_TYPE_3 */
	};
	unsigned long rate;
	int idx;

	idx = (mmio_read_32(RCAR_MODEMR) & MODEMR_BOOT_PLL_MASK)
			>> MODEMR_BOOT_PLL_SHIFT;

	rate = extal_freq[idx];
	/* Divider setting of EXTAL input is 1/2 when MD14=1 MD13=1 */
	if (idx == MD14_MD13_TYPE_3)
		rate = DIV_ROUND(rate, 2);

	return rate;
}

static unsigned long pll0_clk_round_rate(unsigned long rate)
{
	unsigned long parent_rate = pll_clk_parent_rate();
	unsigned int mult;

	if (rate < Z_CLK_MAX_THRESHOLD)
		rate = Z_CLK_MAX_THRESHOLD; /* Set lowest value: 1.5GHz */

	mult = DIV_ROUND(rate, parent_rate);
	mult = max(mult, 90U); /* Lowest value is 1.5GHz (stc == 90) */
	mult = min(mult, 108U);

	rate = parent_rate * mult;
	/* Round to closest value at 100MHz unit */
	rate = 100000000 * DIV_ROUND(rate, 100000000);

	return rate;
}

static unsigned long pll0_clk_recalc_rate(void)
{
	unsigned long parent_rate = pll_clk_parent_rate();
	unsigned int val;
	unsigned long rate;

	val = (mmio_read_32(CPG_BASE + CPG_PLL0CR) & CPG_PLLCR_STC_MASK)
			>> CPG_PLLCR_STC_SHIFT;

	rate = parent_rate * (val + 1);
	/* Round to closest value at 100MHz unit */
	rate = 100000000 * DIV_ROUND(rate, 100000000);

	return rate;
}

static int pll0_clk_set_rate(unsigned long rate)
{
	unsigned long parent_rate = pll_clk_parent_rate();
	unsigned int stc_val;
	uint32_t val;
	int i;

	stc_val = DIV_ROUND(rate, parent_rate);
	stc_val = max(stc_val, 90U); /* Lowest value is 1.5GHz (stc == 90) */
	stc_val = min(stc_val, 108U);

	stc_val -= 1;
	val = mmio_read_32(CPG_BASE + CPG_PLL0CR);
	val &= ~CPG_PLLCR_STC_MASK;
	val |= stc_val << CPG_PLLCR_STC_SHIFT;
	mmio_write_32(CPG_BASE + CPG_PLL0CR, val);

	i = 0;
	while (!(mmio_read_32(CPG_BASE + CPG_PLLECR) & CPG_PLLECR_PLL0ST)) {
		/*cpu_relax();*/
		i++;
	}

	if (i > 1000)
		NOTICE("%s(): PLL0: long settled time: %d\n", __func__, i);

	return 0;
}

static unsigned long pll2_clk_round_rate(unsigned long rate)
{
	unsigned long parent_rate = pll_clk_parent_rate();
	unsigned int mult;

	if (rate < Z2_CLK_MAX_THRESHOLD)
		rate = Z2_CLK_MAX_THRESHOLD; /* Set lowest value: 1.2GHz */

	mult = DIV_ROUND(rate, parent_rate);
	mult = max(mult, 72U); /* Lowest value is 1.2GHz (stc == 72) */
	mult = min(mult, 78U);

	rate = parent_rate * mult;
	/* Round to closest value at 100MHz unit */
	rate = 100000000 * DIV_ROUND(rate, 100000000);

	return rate;
}

static unsigned long pll2_clk_recalc_rate(void)
{
	unsigned long parent_rate = pll_clk_parent_rate();
	unsigned int val;
	unsigned long rate;

	val = (mmio_read_32(CPG_BASE + CPG_PLL2CR) & CPG_PLLCR_STC_MASK)
			>> CPG_PLLCR_STC_SHIFT;

	rate = parent_rate * (val + 1);
	/* Round to closest value at 100MHz unit */
	rate = 100000000 * DIV_ROUND(rate, 100000000);

	return rate;
}

static int pll2_clk_set_rate(unsigned long rate)
{
	unsigned long parent_rate = pll_clk_parent_rate();
	unsigned int stc_val;
	uint32_t val;
	int i;

	stc_val = DIV_ROUND(rate, parent_rate);
	stc_val = max(stc_val, 72U); /* Lowest value is 1.2GHz (stc == 72) */
	stc_val = min(stc_val, 78U);

	stc_val -= 1;
	val = mmio_read_32(CPG_BASE + CPG_PLL2CR);
	val &= ~CPG_PLLCR_STC_MASK;
	val |= stc_val << CPG_PLLCR_STC_SHIFT;
	mmio_write_32(CPG_BASE + CPG_PLL2CR, val);

	i = 0;
	while (!(mmio_read_32(CPG_BASE + CPG_PLLECR) & CPG_PLLECR_PLL2ST)) {
		/*cpu_relax();*/
		i++;
	}

	if (i > 1000)
		NOTICE("%s(): PLL2: long settled time: %d\n", __func__, i);

	return 0;
}

static unsigned long z_clk_round_rate(unsigned long rate, unsigned long *parent_rate)
{
	unsigned long prate = *parent_rate;
	unsigned int mult;

	if (!prate)
		prate = 1;

	if (rate <= Z_CLK_MAX_THRESHOLD) { /* Focus on changing z-clock */
		prate = Z_CLK_MAX_THRESHOLD; /* Set parent to: 1.5GHz */
		mult = DIV_ROUND(rate * 32, prate);
	} else {
		/* Focus on changing parent. Fix z-clock divider is 32/32 */
		mult = 32;
	}
	mult = max(mult, 1U);
	mult = min(mult, 32U);

	/* Re-calculate the parent_rate to propagate new rate for it */
	prate = DIV_ROUND(rate * 32, mult);
	prate = 100000000 * DIV_ROUND(prate, 100000000);
	rate = 100000000 * DIV_ROUND(prate / 32 * mult, 100000000);
	*parent_rate = prate;

	return rate;
}

static unsigned long z_clk_recalc_rate(unsigned long parent_rate)
{
	unsigned int mult;
	unsigned int val;
	unsigned long rate;

	val = (mmio_read_32(CPG_BASE + CPG_FRQCRC) & CPG_FRQCRC_ZFC_MASK)
			>> CPG_FRQCRC_ZFC_SHIFT;
	mult = 32 - val;

	rate = DIV_ROUND(parent_rate * mult, 32);
	/* Round to closest value at 100MHz unit */
	rate = 100000000 * DIV_ROUND(rate, 100000000);

	return rate;
}

static int z_clk_set_rate(unsigned long rate, unsigned long parent_rate)
{
	unsigned int mult;
	uint32_t val, kick;
	unsigned int i;

	if (rate <= Z_CLK_MAX_THRESHOLD) { /* Focus on changing z-clock */
		parent_rate = Z_CLK_MAX_THRESHOLD; /* Set parent to: 1.5GHz */
		mult = DIV_ROUND(rate * 32, parent_rate);
	} else {
		mult = 32;
	}
	mult = max(mult, 1U);
	mult = min(mult, 32U);

	if (mmio_read_32(CPG_BASE + CPG_FRQCRB) & CPG_FRQCRB_KICK)
		return -1;

	val = mmio_read_32(CPG_BASE + CPG_FRQCRC);
	val &= ~CPG_FRQCRC_ZFC_MASK;
	val |= (32 - mult) << CPG_FRQCRC_ZFC_SHIFT;
	mmio_write_32(CPG_BASE + CPG_FRQCRC, val);

	/*
	 * Set KICK bit in FRQCRB to update hardware setting and wait for
	 * clock change completion.
	 */
	kick = mmio_read_32(CPG_BASE + CPG_FRQCRB);
	kick |= CPG_FRQCRB_KICK;
	mmio_write_32(CPG_BASE + CPG_FRQCRB, kick);

	/*
	 * Note: There is no HW information about the worst case latency.
	 *
	 * Using experimental measurements, it seems that no more than
	 * ~10 iterations are needed, independently of the CPU rate.
	 * Since this value might be dependent of external xtal rate, pll1
	 * rate or even the other emulation clocks rate, use 1000 as a
	 * "super" safe value.
	 */
	for (i = 1000; i; i--) {
		if (!(mmio_read_32(CPG_BASE + CPG_FRQCRB) & CPG_FRQCRB_KICK))
			return 0;

		/*cpu_relax();*/
	}

	return -1;
}

static unsigned long z2_clk_round_rate(unsigned long rate, unsigned long *parent_rate)
{
	unsigned long prate = *parent_rate;
	unsigned int mult;

	if (!prate)
		prate = 1;

	if (rate <= Z2_CLK_MAX_THRESHOLD) { /* Focus on changing z2-clock */
		prate = Z2_CLK_MAX_THRESHOLD; /* Set parent to: 1.2GHz */
		mult = DIV_ROUND(rate * 32, prate);
	} else {
		/* Focus on changing parent. Fix z2-clock divider is 32/32 */
		mult = 32;
	}
	mult = max(mult, 1U);
	mult = min(mult, 32U);

	/* Re-calculate the parent_rate to propagate new rate for it */
	prate = DIV_ROUND(rate * 32, mult);
	prate = 100000000 * DIV_ROUND(prate, 100000000);
	rate = 100000000 * DIV_ROUND(prate / 32 * mult, 100000000);
	*parent_rate = prate;

	return rate;
}

static unsigned long z2_clk_recalc_rate(unsigned long parent_rate)
{
	unsigned int mult;
	unsigned int val;
	unsigned long rate;

	val = mmio_read_32(CPG_BASE + CPG_FRQCRC) & CPG_FRQCRC_Z2FC_MASK;
	mult = 32 - val;

	rate = DIV_ROUND(parent_rate * mult, 32);
	/* Round to closest value at 100MHz unit */
	rate = 100000000 * DIV_ROUND(rate, 100000000);

	return rate;
}

static int z2_clk_set_rate(unsigned long rate, unsigned long parent_rate)
{
	unsigned int mult;
	uint32_t val, kick;
	unsigned int i;

	if (rate <= Z2_CLK_MAX_THRESHOLD) { /* Focus on changing z2-clock */
		parent_rate = Z2_CLK_MAX_THRESHOLD; /* Set parent to: 1.2GHz */
		mult = DIV_ROUND(rate * 32, parent_rate);
	} else {
		mult = 32;
	}
	mult = max(mult, 1U);
	mult = min(mult, 32U);

	if (mmio_read_32(CPG_BASE + CPG_FRQCRB) & CPG_FRQCRB_KICK)
		return -1;

	val = mmio_read_32(CPG_BASE + CPG_FRQCRC);
	val &= ~CPG_FRQCRC_Z2FC_MASK;
	val |= 32 - mult;
	mmio_write_32(CPG_BASE + CPG_FRQCRC, val);

	/*
	 * Set KICK bit in FRQCRB to update hardware setting and wait for
	 * clock change completion.
	 */
	kick = mmio_read_32(CPG_BASE + CPG_FRQCRB);
	kick |= CPG_FRQCRB_KICK;
	mmio_write_32(CPG_BASE + CPG_FRQCRB, kick);

	/*
	 * Note: There is no HW information about the worst case latency.
	 *
	 * Using experimental measurements, it seems that no more than
	 * ~10 iterations are needed, independently of the CPU rate.
	 * Since this value might be dependent of external xtal rate, pll1
	 * rate or even the other emulation clocks rate, use 1000 as a
	 * "super" safe value.
	 */
	for (i = 1000; i; i--) {
		if (!(mmio_read_32(CPG_BASE + CPG_FRQCRB) & CPG_FRQCRB_KICK))
			return 0;

		/*cpu_relax();*/
	}

	return -1;
}

/* Default limits measured in millivolts and milliamps */
#define BD9571MWV_MIN_MV		750
#define BD9571MWV_MAX_MV		1030
#define BD9571MWV_STEP_MV		10

/* Define Register */
#define REG_DATA_DVFS_SetVID_MASK	(0x7EU)

static int set_voltage(unsigned long volt)
{
	uint8_t val;
	int ret;

	if (volt < BD9571MWV_MIN_MV * 1000 || volt > BD9571MWV_MAX_MV * 1000)
		return -1;

	val = DIV_ROUND(volt, BD9571MWV_STEP_MV * 1000);
	val &= REG_DATA_DVFS_SetVID_MASK;

	ret = rcar_iic_dvfs_send(SLAVE_ADDR_PMIC, REG_ADDR_DVFS_SetVID, val);
	if (ret) {
		NOTICE("%s(): failed to write PMIC register: %d\n", __func__, ret);
		return ret;
	}

	return 0;
}

/*static unsigned long get_voltage(void)
{
	uint8_t val;
	unsigned long volt;
	int ret;

	ret = rcar_iic_dvfs_recieve(SLAVE_ADDR_PMIC, REG_ADDR_DVFS_SetVID, &val);
	if (ret) {
		NOTICE("%s(): failed to read PMIC register: %d\n", __func__, ret);
		return ret;
	}

	val &= REG_DATA_DVFS_SetVID_MASK;
	volt = (unsigned long)val * BD9571MWV_STEP_MV * 1000;

	return volt;
}*/

static const struct op_points *find_opp(int domain, unsigned long freq)
{
	int i;

	if (domain == A57_DOMAIN) {
		for (i = 0; i < current_a57_opp_limit; i++) {
			if (current_a57_opp_table[i].freq == freq)
				return &current_a57_opp_table[i];
		}
	} else if (domain == A53_DOMAIN) {
		for (i = 0; i < current_a53_opp_limit; i++) {
			if (current_a53_opp_table[i].freq == freq)
				return &current_a53_opp_table[i];
		}
	}

	return NULL;
}

static int set_a57_opp(unsigned long target_freq)
{
	unsigned long freq, old_freq, prate, old_prate;
	const struct op_points *opp, *old_opp;
	int ret;

	prate = 0;
	freq = z_clk_round_rate(target_freq, &prate);

	old_prate = pll0_clk_recalc_rate();
	old_freq = z_clk_recalc_rate(old_prate);

	/* Return early if nothing to do */
	if (old_freq == freq) {
		NOTICE("%s(): old/new frequencies (%lu Hz) are same, nothing to do\n",
			__func__, freq);
		return 0;
	}

	old_opp = find_opp(A57_DOMAIN, old_freq);
	if (!old_opp) {
		NOTICE("%s(): failed to find current OPP for freq %lu\n",
				__func__, old_freq);
	}

	opp = find_opp(A57_DOMAIN, freq);
	if (!opp) {
		NOTICE("%s(): failed to find new OPP for freq %lu\n",
				__func__, freq);
		return -1;
	}

	/* Scaling up? Scale voltage before frequency */
	if (freq > old_freq) {
		ret = set_voltage(opp->volt);
		if (ret) {
			NOTICE("%s(): failed to set voltage: %lu uV\n",
					__func__, opp->volt);
			goto restore_voltage;
		}
	}

	/*NOTICE("%s(): Switching CPU OPP: %lu Hz --> %lu Hz\n", __func__, old_freq, freq);*/

	prate = pll0_clk_round_rate(prate);
	if (old_prate != prate)
		pll0_clk_set_rate(prate);

	ret = z_clk_set_rate(freq, prate);
	if (ret) {
		NOTICE("%s(): failed to set clock rate: %d\n", __func__, ret);
		if (old_prate != prate)
			pll0_clk_set_rate(old_prate);
		goto restore_voltage;
	}

	/* Scaling down? Scale voltage after frequency */
	if (freq < old_freq) {
		ret = set_voltage(opp->volt);
		if (ret) {
			NOTICE("%s(): failed to set voltage: %lu uV\n",
					__func__, opp->volt);
			goto restore_freq;
		}
	}

	return 0;

restore_freq:
	if (old_prate != prate)
		pll0_clk_set_rate(old_prate);
	if (z_clk_set_rate(old_freq, old_prate))
		NOTICE("%s(): failed to restore old freq: %lu Hz\n",
				__func__, old_freq);

restore_voltage:
	/* This shouldn't harm even if the voltages weren't updated earlier */
	if (old_opp) {
		if (set_voltage(old_opp->volt))
			NOTICE("%s(): failed to restore old voltage: %lu uV\n",
					__func__, old_opp->volt);
	}

	return ret;
}

static int set_a53_opp(unsigned long target_freq)
{
	unsigned long freq, old_freq, prate, old_prate;
	const struct op_points *opp, *old_opp;
	int ret;

	prate = 0;
	freq = z2_clk_round_rate(target_freq, &prate);

	old_prate = pll2_clk_recalc_rate();
	old_freq = z2_clk_recalc_rate(old_prate);

	/* Return early if nothing to do */
	if (old_freq == freq) {
		NOTICE("%s(): old/new frequencies (%lu Hz) are same, nothing to do\n",
			__func__, freq);
		return 0;
	}

	old_opp = find_opp(A53_DOMAIN, old_freq);
	if (!old_opp) {
		NOTICE("%s(): failed to find current OPP for freq %lu\n",
				__func__, old_freq);
	}

	opp = find_opp(A53_DOMAIN, freq);
	if (!opp) {
		NOTICE("%s(): failed to find new OPP for freq %lu\n",
				__func__, freq);
		return -1;
	}

	/*NOTICE("%s(): Switching CPU OPP: %lu Hz --> %lu Hz\n", __func__, old_freq, freq);*/

	prate = pll2_clk_round_rate(prate);
	if (old_prate != prate)
		pll2_clk_set_rate(prate);

	ret = z2_clk_set_rate(freq, prate);
	if (ret) {
		NOTICE("%s(): failed to set clock rate: %d\n", __func__, ret);
	}

	return ret;
}

int rcar_dvfs_set_index(int domain, int index)
{
	int ret;

	if (domain == A57_DOMAIN) {
		if (index < 0 || index >= current_a57_opp_limit)
			return -1;

		ret = set_a57_opp(rcar_dvfs_get_opp_frequency(domain, index));
		if (ret)
			return ret;

		current_a57_opp_index = index;

		return 0;
	} else if (domain == A53_DOMAIN) {
		if (index < 0 || index >= current_a53_opp_limit)
			return -1;

		ret = set_a53_opp(rcar_dvfs_get_opp_frequency(domain, index));
		if (ret)
			return ret;

		current_a53_opp_index = index;

		return 0;
	}

	return -1;
}

int rcar_dvfs_get_index(int domain)
{
	if (domain == A57_DOMAIN)
		return current_a57_opp_index;
	else if (domain == A53_DOMAIN)
		return current_a53_opp_index;

	return 0;
}

int rcar_dvfs_get_nr_opp(int domain)
{
	if (domain == A57_DOMAIN)
		return current_a57_opp_limit;
	else if (domain == A53_DOMAIN)
		return current_a53_opp_limit;

	return 0;
}

int rcar_dvfs_get_latency(int domain)
{
	if (domain == A57_DOMAIN)
		return current_a57_opp_latency;
	else if (domain == A53_DOMAIN)
		return current_a53_opp_latency;

	return 0;
}

int rcar_dvfs_opp_init(void)
{
	uint32_t product;

	if (dvfs_inited)
		return 0;

	product = mmio_read_32(RCAR_PRR) & RCAR_PRODUCT_MASK;

	if (product == RCAR_PRODUCT_H3) {
		current_a57_opp_limit = ARRAY_SIZE(rcar_h3_a57_op_points[efuse_avs]);
		current_a57_opp_table = &rcar_h3_a57_op_points[efuse_avs][0];

		current_a53_opp_limit = ARRAY_SIZE(rcar_h3_a53_op_points);
		current_a53_opp_table = &rcar_h3_a53_op_points[0];
	} else if (product == RCAR_PRODUCT_M3) {
		current_a57_opp_limit = ARRAY_SIZE(rcar_m3_a57_op_points[efuse_avs]);
		current_a57_opp_table = &rcar_m3_a57_op_points[efuse_avs][0];

		current_a53_opp_limit = ARRAY_SIZE(rcar_m3_a53_op_points);
		current_a53_opp_table = &rcar_m3_a53_op_points[0];
	} else
		return -1;

	/* Guess it is 1500000000 Hz for now */
	current_a57_opp_index = 2;
	/* Latency is 300 uS for all OPPs */
	current_a57_opp_latency = 300;

	/* Guess it is 1200000000 Hz for now */
	current_a53_opp_index = 2;
	/* Latency is 300 uS for all OPPs */
	current_a53_opp_latency = 300;

	dvfs_inited = 1;

	return 0;
}
