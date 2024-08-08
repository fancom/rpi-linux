#define MODE_HACK
#define VERBOSE
//#define HARDCODED_REGS
//#define SN65DSI83_TEST_PATTERN

// SPDX-License-Identifier: GPL-2.0
/*
 * TI SN65DSI83,84,85 driver
 *
 * Currently supported:
 * - SN65DSI83
 *   = 1x Single-link DSI ~ 1x Single-link LVDS
 *   - Supported
 *   - Single-link LVDS mode tested
 * - SN65DSI84
 *   = 1x Single-link DSI ~ 2x Single-link or 1x Dual-link LVDS
 *   - Supported
 *   - Dual-link LVDS mode tested
 *   - 2x Single-link LVDS mode unsupported
 *     (should be easy to add by someone who has the HW)
 * - SN65DSI85
 *   = 2x Single-link or 1x Dual-link DSI ~ 2x Single-link or 1x Dual-link LVDS
 *   - Unsupported
 *     (should be easy to add by someone who has the HW)
 *
 * Copyright (C) 2021 Marek Vasut <marex@denx.de>
 *
 * Based on previous work of:
 * Valentin Raevsky <valentin@compulab.co.il>
 * Philippe Schenker <philippe.schenker@toradex.com>
 */

#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <linux/regmap.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_bridge.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_of.h>
#include <drm/drm_panel.h>
#include <drm/drm_print.h>
#include <drm/drm_probe_helper.h>

/* ID registers */
#define REG_ID(n)				(0x00 + (n))
/* Reset and clock registers */
#define REG_RC_RESET				0x09
#define  REG_RC_RESET_SOFT_RESET		BIT(0)
#define REG_RC_LVDS_PLL				0x0a
#define  REG_RC_LVDS_PLL_PLL_EN_STAT		BIT(7)
#define  REG_RC_LVDS_PLL_LVDS_CLK_RANGE(n)	(((n) & 0x7) << 1)
#define  REG_RC_LVDS_PLL_HS_CLK_SRC_DPHY	BIT(0)
#define REG_RC_DSI_CLK				0x0b
#define  REG_RC_DSI_CLK_DSI_CLK_DIVIDER(n)	(((n) & 0x1f) << 3)
#define  REG_RC_DSI_CLK_REFCLK_MULTIPLIER(n)	((n) & 0x3)
#define REG_RC_PLL_EN				0x0d
#define  REG_RC_PLL_EN_PLL_EN			BIT(0)
/* DSI registers */
#define REG_DSI_LANE				0x10
#define  REG_DSI_LANE_LEFT_RIGHT_PIXELS		BIT(7)	/* DSI85-only */
#define  REG_DSI_LANE_DSI_CHANNEL_MODE_DUAL	0	/* DSI85-only */
#define  REG_DSI_LANE_DSI_CHANNEL_MODE_2SINGLE	BIT(6)	/* DSI85-only */
#define  REG_DSI_LANE_DSI_CHANNEL_MODE_SINGLE	BIT(5)
#define  REG_DSI_LANE_CHA_DSI_LANES(n)		(((n) & 0x3) << 3)
#define  REG_DSI_LANE_CHB_DSI_LANES(n)		(((n) & 0x3) << 1)
#define  REG_DSI_LANE_SOT_ERR_TOL_DIS		BIT(0)
#define REG_DSI_EQ				0x11
#define  REG_DSI_EQ_CHA_DSI_DATA_EQ(n)		(((n) & 0x3) << 6)
#define  REG_DSI_EQ_CHA_DSI_CLK_EQ(n)		(((n) & 0x3) << 2)
#define REG_DSI_CLK				0x12
#define  REG_DSI_CLK_CHA_DSI_CLK_RANGE(n)	((n) & 0xff)
/* LVDS registers */
#define REG_LVDS_FMT				0x18
#define  REG_LVDS_FMT_DE_NEG_POLARITY		BIT(7)
#define  REG_LVDS_FMT_HS_NEG_POLARITY		BIT(6)
#define  REG_LVDS_FMT_VS_NEG_POLARITY		BIT(5)
#define  REG_LVDS_FMT_LVDS_LINK_CFG		BIT(4)	/* 0:AB 1:A-only */
#define  REG_LVDS_FMT_CHA_24BPP_MODE		BIT(3)
#define  REG_LVDS_FMT_CHB_24BPP_MODE		BIT(2)
#define  REG_LVDS_FMT_CHA_24BPP_FORMAT1		BIT(1)
#define  REG_LVDS_FMT_CHB_24BPP_FORMAT1		BIT(0)
#define REG_LVDS_VCOM				0x19
#define  REG_LVDS_VCOM_CHA_LVDS_VOCM		BIT(6)
#define  REG_LVDS_VCOM_CHB_LVDS_VOCM		BIT(4)
#define  REG_LVDS_VCOM_CHA_LVDS_VOD_SWING(n)	(((n) & 0x3) << 2)
#define  REG_LVDS_VCOM_CHB_LVDS_VOD_SWING(n)	((n) & 0x3)
#define REG_LVDS_LANE				0x1a
#define  REG_LVDS_LANE_EVEN_ODD_SWAP		BIT(6)
#define  REG_LVDS_LANE_CHA_REVERSE_LVDS		BIT(5)
#define  REG_LVDS_LANE_CHB_REVERSE_LVDS		BIT(4)
#define  REG_LVDS_LANE_CHA_LVDS_TERM		BIT(1)
#define  REG_LVDS_LANE_CHB_LVDS_TERM		BIT(0)
#define REG_LVDS_CM				0x1b
#define  REG_LVDS_CM_CHA_LVDS_CM_ADJUST(n)	(((n) & 0x3) << 4)
#define  REG_LVDS_CM_CHB_LVDS_CM_ADJUST(n)	((n) & 0x3)
/* Video registers */
#define REG_VID_CHA_ACTIVE_LINE_LENGTH_LOW	0x20
#define REG_VID_CHA_ACTIVE_LINE_LENGTH_HIGH	0x21
#define REG_VID_CHA_VERTICAL_DISPLAY_SIZE_LOW	0x24
#define REG_VID_CHA_VERTICAL_DISPLAY_SIZE_HIGH	0x25
#define REG_VID_CHA_SYNC_DELAY_LOW		0x28
#define REG_VID_CHA_SYNC_DELAY_HIGH		0x29
#define REG_VID_CHA_HSYNC_PULSE_WIDTH_LOW	0x2c
#define REG_VID_CHA_HSYNC_PULSE_WIDTH_HIGH	0x2d
#define REG_VID_CHA_VSYNC_PULSE_WIDTH_LOW	0x30
#define REG_VID_CHA_VSYNC_PULSE_WIDTH_HIGH	0x31
#define REG_VID_CHA_HORIZONTAL_BACK_PORCH	0x34
#define REG_VID_CHA_VERTICAL_BACK_PORCH		0x36
#define REG_VID_CHA_HORIZONTAL_FRONT_PORCH	0x38
#define REG_VID_CHA_VERTICAL_FRONT_PORCH	0x3a
#define REG_VID_CHA_TEST_PATTERN		0x3c
/* sn65dsi85 only */
#define REG_VID_RIGHT_CROP              0x3d
#define REG_VID_LEFT_CROP               0x3e

/* IRQ registers */
#define REG_IRQ_GLOBAL				0xe0
#define  REG_IRQ_GLOBAL_IRQ_EN			BIT(0)
#define REG_IRQ_EN				0xe1
#define  REG_IRQ_EN_CHA_SYNCH_ERR_EN		BIT(7)
#define  REG_IRQ_EN_CHA_CRC_ERR_EN		BIT(6)
#define  REG_IRQ_EN_CHA_UNC_ECC_ERR_EN		BIT(5)
#define  REG_IRQ_EN_CHA_COR_ECC_ERR_EN		BIT(4)
#define  REG_IRQ_EN_CHA_LLP_ERR_EN		BIT(3)
#define  REG_IRQ_EN_CHA_SOT_BIT_ERR_EN		BIT(2)
#define  REG_IRQ_EN_CHA_PLL_UNLOCK_EN		BIT(0)
#define REG_IRQ_STAT				0xe5
#define  REG_IRQ_STAT_CHA_SYNCH_ERR		BIT(7)
#define  REG_IRQ_STAT_CHA_CRC_ERR		BIT(6)
#define  REG_IRQ_STAT_CHA_UNC_ECC_ERR		BIT(5)
#define  REG_IRQ_STAT_CHA_COR_ECC_ERR		BIT(4)
#define  REG_IRQ_STAT_CHA_LLP_ERR		BIT(3)
#define  REG_IRQ_STAT_CHA_SOT_BIT_ERR		BIT(2)
#define  REG_IRQ_STAT_CHA_PLL_UNLOCK		BIT(0)

enum sn65dsi83_model {
	MODEL_SN65DSI83,
	MODEL_SN65DSI84,
};

struct sn65dsi83 {
	struct drm_bridge		bridge;
	struct drm_display_mode		mode;
	struct device			*dev;
	struct regmap			*regmap;
	struct device_node		*host_node;
	struct mipi_dsi_device		*dsi;
	struct drm_bridge		*panel_bridge;
	struct gpio_desc		*enable_gpio;
	int				dsi_lanes;
	bool				lvds_dual_link;
	bool				lvds_dual_link_even_odd_swap;
	bool				lvds_format_24bpp;
	bool				lvds_format_jeida;
};

static const struct regmap_range sn65dsi83_readable_ranges[] = {
	regmap_reg_range(REG_ID(0), REG_ID(8)),
	regmap_reg_range(REG_RC_LVDS_PLL, REG_RC_DSI_CLK),
	regmap_reg_range(REG_RC_PLL_EN, REG_RC_PLL_EN),
	regmap_reg_range(REG_DSI_LANE, REG_DSI_CLK),
	regmap_reg_range(REG_LVDS_FMT, REG_LVDS_CM),
	regmap_reg_range(REG_VID_CHA_ACTIVE_LINE_LENGTH_LOW,
			 REG_VID_CHA_ACTIVE_LINE_LENGTH_HIGH),
	regmap_reg_range(REG_VID_CHA_VERTICAL_DISPLAY_SIZE_LOW,
			 REG_VID_CHA_VERTICAL_DISPLAY_SIZE_HIGH),
	regmap_reg_range(REG_VID_CHA_SYNC_DELAY_LOW,
			 REG_VID_CHA_SYNC_DELAY_HIGH),
	regmap_reg_range(REG_VID_CHA_HSYNC_PULSE_WIDTH_LOW,
			 REG_VID_CHA_HSYNC_PULSE_WIDTH_HIGH),
	regmap_reg_range(REG_VID_CHA_VSYNC_PULSE_WIDTH_LOW,
			 REG_VID_CHA_VSYNC_PULSE_WIDTH_HIGH),
	regmap_reg_range(REG_VID_CHA_HORIZONTAL_BACK_PORCH,
			 REG_VID_CHA_HORIZONTAL_BACK_PORCH),
	regmap_reg_range(REG_VID_CHA_VERTICAL_BACK_PORCH,
			 REG_VID_CHA_VERTICAL_BACK_PORCH),
	regmap_reg_range(REG_VID_CHA_HORIZONTAL_FRONT_PORCH,
			 REG_VID_CHA_HORIZONTAL_FRONT_PORCH),
	regmap_reg_range(REG_VID_CHA_VERTICAL_FRONT_PORCH,
			 REG_VID_CHA_VERTICAL_FRONT_PORCH),
	regmap_reg_range(REG_VID_CHA_TEST_PATTERN, REG_VID_CHA_TEST_PATTERN),
	regmap_reg_range(REG_IRQ_GLOBAL, REG_IRQ_EN),
	regmap_reg_range(REG_IRQ_STAT, REG_IRQ_STAT),
};

static const struct regmap_access_table sn65dsi83_readable_table = {
	.yes_ranges = sn65dsi83_readable_ranges,
	.n_yes_ranges = ARRAY_SIZE(sn65dsi83_readable_ranges),
};

static const struct regmap_range sn65dsi83_writeable_ranges[] = {
	regmap_reg_range(REG_RC_RESET, REG_RC_DSI_CLK),
	regmap_reg_range(REG_RC_PLL_EN, REG_RC_PLL_EN),
	regmap_reg_range(REG_DSI_LANE, REG_DSI_CLK),
	regmap_reg_range(REG_LVDS_FMT, REG_LVDS_CM),
	regmap_reg_range(REG_VID_CHA_ACTIVE_LINE_LENGTH_LOW,
			 REG_VID_CHA_ACTIVE_LINE_LENGTH_HIGH),
	regmap_reg_range(REG_VID_CHA_VERTICAL_DISPLAY_SIZE_LOW,
			 REG_VID_CHA_VERTICAL_DISPLAY_SIZE_HIGH),
	regmap_reg_range(REG_VID_CHA_SYNC_DELAY_LOW,
			 REG_VID_CHA_SYNC_DELAY_HIGH),
	regmap_reg_range(REG_VID_CHA_HSYNC_PULSE_WIDTH_LOW,
			 REG_VID_CHA_HSYNC_PULSE_WIDTH_HIGH),
	regmap_reg_range(REG_VID_CHA_VSYNC_PULSE_WIDTH_LOW,
			 REG_VID_CHA_VSYNC_PULSE_WIDTH_HIGH),
	regmap_reg_range(REG_VID_CHA_HORIZONTAL_BACK_PORCH,
			 REG_VID_CHA_HORIZONTAL_BACK_PORCH),
	regmap_reg_range(REG_VID_CHA_VERTICAL_BACK_PORCH,
			 REG_VID_CHA_VERTICAL_BACK_PORCH),
	regmap_reg_range(REG_VID_CHA_HORIZONTAL_FRONT_PORCH,
			 REG_VID_CHA_HORIZONTAL_FRONT_PORCH),
	regmap_reg_range(REG_VID_CHA_VERTICAL_FRONT_PORCH,
			 REG_VID_CHA_VERTICAL_FRONT_PORCH),
	regmap_reg_range(REG_VID_CHA_TEST_PATTERN, REG_VID_CHA_TEST_PATTERN),
	regmap_reg_range(REG_IRQ_GLOBAL, REG_IRQ_EN),
	regmap_reg_range(REG_IRQ_STAT, REG_IRQ_STAT),
};

static const struct regmap_access_table sn65dsi83_writeable_table = {
	.yes_ranges = sn65dsi83_writeable_ranges,
	.n_yes_ranges = ARRAY_SIZE(sn65dsi83_writeable_ranges),
};

static const struct regmap_range sn65dsi83_volatile_ranges[] = {
	regmap_reg_range(REG_RC_RESET, REG_RC_RESET),
	regmap_reg_range(REG_RC_LVDS_PLL, REG_RC_LVDS_PLL),
	regmap_reg_range(REG_IRQ_STAT, REG_IRQ_STAT),
};

static const struct regmap_access_table sn65dsi83_volatile_table = {
	.yes_ranges = sn65dsi83_volatile_ranges,
	.n_yes_ranges = ARRAY_SIZE(sn65dsi83_volatile_ranges),
};

static const struct regmap_config sn65dsi83_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.rd_table = &sn65dsi83_readable_table,
	.wr_table = &sn65dsi83_writeable_table,
	.volatile_table = &sn65dsi83_volatile_table,
	.cache_type = REGCACHE_RBTREE,
	.max_register = REG_IRQ_STAT,
};

static const struct reg_default sn65dsi83_reg_defaults[] = {
	/* Reset */
	{0x09, 0x00},

	/* Core */
	{0x0A, 0x05},//ok
	{0x0B, 0x38},//ok
	{0x0D, 0x00},//ok
	{0x10, 0x28},//ok
	{0x11, 0x00},//ok
	{0x12, 0x64},//ok
	{0x18, 0x6C},//ok
	{0x19, 0x0F},//ok
	{0x1A, 0x20},//ok
	{0x1B, 0x00},//ok

	/* Channel A+B */
	{0x20, 0x80},//ok
	{0x21, 0x07},//ok
	{0x24, 0x38},//ok
	{0x25, 0x04},//ok
	{0x28, 0x21},//ok
	{0x29, 0x00},//ok
	{0x2C, 0x1E},//ok
	{0x2D, 0x00},//ok
	{0x30, 0x04},//ok
	{0x31, 0x00},//ok
	{0x34, 0x1E},//ok
	{0x36, 0x03},//ok
	{0x38, 0x1E},//ok
	{0x3A, 0x03},//ok

	/* interrupts */
	{0xE0, 0x00},
	{0xE1, 0x00},
	{0xE5, 0x00},
#ifdef SN65DSI83_TEST_PATTERN
	/* Test */
	{0x3C, 0x10},
#else
	{0x3C, 0x00},
#endif
};

/* fwd declaration */
static void sn65dsi83_mode_set(struct drm_bridge *bridge,
			       const struct drm_display_mode *mode,
			       const struct drm_display_mode *adj);

static struct sn65dsi83 *bridge_to_sn65dsi83(struct drm_bridge *bridge)
{
	return container_of(bridge, struct sn65dsi83, bridge);
}

static int sn65dsi83_attach(struct drm_bridge *bridge,
			    enum drm_bridge_attach_flags flags)
{
	struct sn65dsi83 *ctx = bridge_to_sn65dsi83(bridge);
	struct device *dev = ctx->dev;
	struct mipi_dsi_device *dsi;
	struct mipi_dsi_host *host;
	int ret = 0;

	const struct mipi_dsi_device_info info = {
		.type = "sn65dsi83",
		.channel = 0,
		.node = NULL,
	};

	host = of_find_mipi_dsi_host_by_node(ctx->host_node);
	if (!host) {
		dev_err(dev, "failed to find dsi host\n");
		return -EPROBE_DEFER;
	}

	dsi = mipi_dsi_device_register_full(host, &info);
	if (IS_ERR(dsi)) {
		return dev_err_probe(dev, PTR_ERR(dsi),
				     "failed to create dsi device\n");
	}

	ctx->dsi = dsi;

	dsi->lanes = ctx->dsi_lanes;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO /*| MIPI_DSI_MODE_VIDEO_BURST*/;

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		dev_err(dev, "failed to attach dsi to host\n");
		goto err_dsi_attach;
	}

	return drm_bridge_attach(bridge->encoder, ctx->panel_bridge,
				 &ctx->bridge, flags);

err_dsi_attach:
	mipi_dsi_device_unregister(dsi);
	return ret;
}

static void sn65dsi83_pre_enable(struct drm_bridge *bridge)
{
	struct sn65dsi83 *ctx = bridge_to_sn65dsi83(bridge);
#ifdef MODE_HACK
	struct drm_display_mode *adjusted_mode;
#endif
	/*
	 * Reset the chip, pull EN line low for t_reset=10ms,
	 * then high for t_en=1ms.
	 */
	regcache_mark_dirty(ctx->regmap);
	gpiod_set_value(ctx->enable_gpio, 0);
	usleep_range(10000, 11000);
	gpiod_set_value(ctx->enable_gpio, 1);
	usleep_range(1000, 1100);

	/* disable PLL, should already be the case after toggle of enable pin */
	regmap_write(ctx->regmap, REG_RC_PLL_EN, 0x00);
	regmap_write(ctx->regmap, REG_RC_RESET, 0x00);

#ifdef MODE_HACK
	/* TODO: hack until mode_set and mode_valid are called */
	adjusted_mode = &(bridge->encoder->crtc->state->adjusted_mode);
	sn65dsi83_mode_set(bridge, &ctx->mode, adjusted_mode);
	ctx->lvds_format_24bpp = true;
	ctx->lvds_format_jeida = false;
#endif
}

static u8 sn65dsi83_get_lvds_range(struct sn65dsi83 *ctx)
{
	/*
	 * The encoding of the LVDS_CLK_RANGE is as follows:
	 * 000 - 25 MHz <= LVDS_CLK < 37.5 MHz
	 * 001 - 37.5 MHz <= LVDS_CLK < 62.5 MHz
	 * 010 - 62.5 MHz <= LVDS_CLK < 87.5 MHz
	 * 011 - 87.5 MHz <= LVDS_CLK < 112.5 MHz
	 * 100 - 112.5 MHz <= LVDS_CLK < 137.5 MHz
	 * 101 - 137.5 MHz <= LVDS_CLK <= 154 MHz
	 * which is a range of 12.5MHz..162.5MHz in 50MHz steps, except that
	 * the ends of the ranges are clamped to the supported range. Since
	 * sn65dsi83_mode_valid() already filters the valid modes and limits
	 * the clock to 25..154 MHz, the range calculation can be simplified
	 * as follows:
	 */
	int mode_clock = ctx->mode.clock;

	if (ctx->lvds_dual_link)
		mode_clock /= 2;

	return (mode_clock - 12500) / 25000;
}

static u8 sn65dsi83_get_dsi_range(struct sn65dsi83 *ctx)
{
	/*
	 * The encoding of the CHA_DSI_CLK_RANGE is as follows:
	 * 0x00 through 0x07 - Reserved
	 * 0x08 - 40 <= DSI_CLK < 45 MHz
	 * 0x09 - 45 <= DSI_CLK < 50 MHz
	 * ...
	 * 0x63 - 495 <= DSI_CLK < 500 MHz
	 * 0x64 - 500 MHz
	 * 0x65 through 0xFF - Reserved
	 * which is DSI clock in 5 MHz steps, clamped to 40..500 MHz.
	 * The DSI clock are calculated as:
	 *  DSI_CLK = mode clock * bpp / dsi_data_lanes / 2
	 * the 2 is there because the bus is DDR.
	 */

	u8 dsiClk = DIV_ROUND_UP(clamp(((unsigned int)ctx->mode.clock *
			    mipi_dsi_pixel_format_to_bpp(ctx->dsi->format)) /
			    (ctx->dsi_lanes * 2), 40000U, 500000U), 5000U);
#ifdef VERBOSE
	printk(KERN_ERR "DSI_BRIDGE: %s: set dsiClk between %d and %d Mhz based on %ld, %d, %d\n",
	       __func__, dsiClk * 5, dsiClk * 6, ctx->mode.clock, ctx->dsi->format, ctx->dsi_lanes);
#endif
	return dsiClk;
}

static u8 sn65dsi83_get_dsi_div(struct sn65dsi83 *ctx)
{
	/* The divider is (DSI_CLK / LVDS_CLK) - 1, which really is: */
	unsigned int dsi_div = mipi_dsi_pixel_format_to_bpp(ctx->dsi->format);

	dsi_div /= ctx->dsi_lanes;

	if (!ctx->lvds_dual_link)
		dsi_div /= 2;

#ifdef VERBOSE
	printk(KERN_ERR "DSI_BRIDGE: %s: set dsiDiv to %d based on %ld, %d, %d\n",
	       __func__, dsi_div, ctx->dsi->format, ctx->dsi_lanes, ctx->lvds_dual_link);
#endif
	return dsi_div - 1;
}

#ifdef VERBOSE
static void dumpRegs(struct drm_bridge *bridge)
{
	struct sn65dsi83 *ctx = bridge_to_sn65dsi83(bridge);
	unsigned int val;
	int i;

	for (i = 0; i < ARRAY_SIZE(sn65dsi83_reg_defaults); i++) {
		struct reg_default conf = sn65dsi83_reg_defaults[i];
		regmap_read(ctx->regmap, conf.reg, &val);
		printk(KERN_ERR "DSI_BRIDGE: %s: reg 0x%02x val 0x%02x\n", __func__, conf.reg, val);
	}
}
#endif

static void sn65dsi83_enable(struct drm_bridge *bridge)
{
	struct sn65dsi83 *ctx = bridge_to_sn65dsi83(bridge);
	unsigned int pval;
	u16 val;
	int ret;
#ifdef HARDCODED_REGS
	int i;
#endif

	/* Reference clock derived from DSI link clock. */
	regmap_write(ctx->regmap, REG_RC_LVDS_PLL,
		REG_RC_LVDS_PLL_LVDS_CLK_RANGE(sn65dsi83_get_lvds_range(ctx)) |
		REG_RC_LVDS_PLL_HS_CLK_SRC_DPHY);
	regmap_write(ctx->regmap, REG_DSI_CLK,
		REG_DSI_CLK_CHA_DSI_CLK_RANGE(sn65dsi83_get_dsi_range(ctx)));
	regmap_write(ctx->regmap, REG_RC_DSI_CLK,
		REG_RC_DSI_CLK_DSI_CLK_DIVIDER(sn65dsi83_get_dsi_div(ctx)));

	/* Set number of DSI lanes and LVDS link config. */
	regmap_write(ctx->regmap, REG_DSI_LANE,
		REG_DSI_LANE_DSI_CHANNEL_MODE_SINGLE |
		REG_DSI_LANE_CHA_DSI_LANES(~(ctx->dsi_lanes - 1)) |
		/* CHB is DSI85-only, set to default on DSI83/DSI84 */
		REG_DSI_LANE_CHB_DSI_LANES(3));
	/* No equalization. */
	regmap_write(ctx->regmap, REG_DSI_EQ, 0x00);

	/* Set up sync signal polarity. */
	val = (ctx->mode.flags & DRM_MODE_FLAG_NHSYNC ?
	       REG_LVDS_FMT_HS_NEG_POLARITY : 0) |
	      (ctx->mode.flags & DRM_MODE_FLAG_NVSYNC ?
	       REG_LVDS_FMT_VS_NEG_POLARITY : 0);

	/* Set up bits-per-pixel, 18bpp or 24bpp. */
	if (ctx->lvds_format_24bpp) {
		val |= REG_LVDS_FMT_CHA_24BPP_MODE;
		if (ctx->lvds_dual_link)
			val |= REG_LVDS_FMT_CHB_24BPP_MODE;
	}

	/* Set up LVDS format, JEIDA/Format 1 or SPWG/Format 2 */
	if (ctx->lvds_format_jeida) {
		val |= REG_LVDS_FMT_CHA_24BPP_FORMAT1;
		if (ctx->lvds_dual_link)
			val |= REG_LVDS_FMT_CHB_24BPP_FORMAT1;
	}

	/* Set up LVDS output config (DSI84,DSI85) */
	if (!ctx->lvds_dual_link)
		val |= REG_LVDS_FMT_LVDS_LINK_CFG;

	regmap_write(ctx->regmap, REG_LVDS_FMT, val);
	// set correct voltage swing for InnoLux, TODO: configure through DTS
	//regmap_write(ctx->regmap, REG_LVDS_VCOM, 0x05);
	regmap_write(ctx->regmap, REG_LVDS_VCOM, 0x0F);
	// set 100ohm term for InnoLux, TODO: configure through DTS, using 200hm as default and optionally adding '100-ohm-termination;'
	// set reverse channel A LVDS, TODO: configure through DTS, using non-reverse as default and optionally adding 'reverse-channel-A;' and/or 'reverse-channel-B;'
	/*regmap_write(ctx->regmap, REG_LVDS_LANE,
		(ctx->lvds_dual_link_even_odd_swap ?
		 REG_LVDS_LANE_EVEN_ODD_SWAP : 0) |
		REG_LVDS_LANE_CHA_LVDS_TERM |
		REG_LVDS_LANE_CHB_LVDS_TERM);*/
	regmap_write(ctx->regmap, REG_LVDS_LANE,
		(ctx->lvds_dual_link_even_odd_swap ?
		 REG_LVDS_LANE_EVEN_ODD_SWAP : 0) |
		 REG_LVDS_LANE_CHA_REVERSE_LVDS);
	regmap_write(ctx->regmap, REG_LVDS_CM, 0x00);

	// only single channel DSI is supported for now
	val = ctx->mode.hdisplay;
	regmap_bulk_write(ctx->regmap, REG_VID_CHA_ACTIVE_LINE_LENGTH_LOW,
			&val, 2);
	regmap_bulk_write(ctx->regmap, REG_VID_CHA_VERTICAL_DISPLAY_SIZE_LOW,
			&ctx->mode.vdisplay, 2);

	val = 32 + 1;	/* 32 + 1 pixel clock to ensure proper operation */
	regmap_bulk_write(ctx->regmap, REG_VID_CHA_SYNC_DELAY_LOW, &val, 2);

#ifdef VERBOSE
	printk(KERN_ERR "DSI_BRIDGE: %s: hsync_end %d hsync_start %d\n", __func__, ctx->mode.hsync_end, ctx->mode.hsync_start);
#endif
	val = ctx->mode.hsync_end - ctx->mode.hsync_start;
	if (ctx->lvds_dual_link)
		val /= 2;
	regmap_bulk_write(ctx->regmap, REG_VID_CHA_HSYNC_PULSE_WIDTH_LOW,
			  &val, 2);
#ifdef VERBOSE
	printk(KERN_ERR "DSI_BRIDGE: %s: vsync_end %d vsync_start %d\n", __func__, ctx->mode.vsync_end, ctx->mode.vsync_start);
#endif
	val = ctx->mode.vsync_end - ctx->mode.vsync_start;
	regmap_bulk_write(ctx->regmap, REG_VID_CHA_VSYNC_PULSE_WIDTH_LOW,
			  &val, 2);

#ifdef VERBOSE
	printk(KERN_ERR "DSI_BRIDGE: %s: htotal %d hsync_end %d\n", __func__, ctx->mode.htotal, ctx->mode.hsync_end);
#endif
	val = ctx->mode.htotal - ctx->mode.hsync_end;
	if (ctx->lvds_dual_link)
		val /= 2;
	regmap_write(ctx->regmap, REG_VID_CHA_HORIZONTAL_BACK_PORCH,
		     val);

#ifdef VERBOSE
	printk(KERN_ERR "DSI_BRIDGE: %s: vtotal %d vsync_end %d\n", __func__, ctx->mode.vtotal, ctx->mode.vsync_end);
#endif
	val = ctx->mode.vtotal - ctx->mode.vsync_end;
	regmap_write(ctx->regmap, REG_VID_CHA_VERTICAL_BACK_PORCH,
		     val);

#ifdef VERBOSE
	printk(KERN_ERR "DSI_BRIDGE: %s: hsync_start %d hdisplay %d\n", __func__, ctx->mode.hsync_start, ctx->mode.hdisplay);
#endif
	val = ctx->mode.hsync_start - ctx->mode.hdisplay;
	if (ctx->lvds_dual_link)
		val /= 2;
	regmap_write(ctx->regmap, REG_VID_CHA_HORIZONTAL_FRONT_PORCH,
		     val);

#ifdef VERBOSE	
	printk(KERN_ERR "DSI_BRIDGE: %s: vsync_start %d vdisplay %d\n", __func__, ctx->mode.vsync_start, ctx->mode.vdisplay);
#endif
	val = ctx->mode.vsync_start - ctx->mode.vdisplay;
	regmap_write(ctx->regmap, REG_VID_CHA_VERTICAL_FRONT_PORCH,
		     val);
	//enable test pattern
#ifndef SN65DSI83_TEST_PATTERN
	regmap_write(ctx->regmap, REG_VID_CHA_TEST_PATTERN, 0x00);
#else
	regmap_write(ctx->regmap, REG_VID_CHA_TEST_PATTERN, 0x10);
#endif

#ifdef HARDCODED_REGS
#ifdef VERBOSE
	printk(KERN_ERR "DSI_BRIDGE: %s: sn65dsi83_reg_defaults\n", __func__);
#endif
	for (i = 0; i < ARRAY_SIZE(sn65dsi83_reg_defaults); i++) {
		struct reg_default conf = sn65dsi83_reg_defaults[i];
		regmap_write(ctx->regmap, conf.reg, conf.def);
	}
#ifdef VERBOSE
	printk(KERN_ERR "DSI_BRIDGE: %s: written %d sn65dsi83_reg_defaults\n",
	       __func__, i);
#endif
#endif

	/* Enable PLL */
	regmap_write(ctx->regmap, REG_RC_PLL_EN, REG_RC_PLL_EN_PLL_EN);
	usleep_range(3000, 4000);
	ret = regmap_read_poll_timeout(ctx->regmap, REG_RC_LVDS_PLL, pval,
					pval & REG_RC_LVDS_PLL_PLL_EN_STAT,
					1000, 100000);
	if (ret) {
		dev_err(ctx->dev, "failed to lock PLL, ret=%i\n", ret);
		/* On failure, disable PLL again and exit. */
		regmap_write(ctx->regmap, REG_RC_PLL_EN, 0x00);
		return;
	}

	/* Trigger reset after CSR register update. */
	regmap_write(ctx->regmap, REG_RC_RESET, REG_RC_RESET_SOFT_RESET);

	/* Clear all errors that got asserted during initialization. */
	regmap_read(ctx->regmap, REG_IRQ_STAT, &pval);
	regmap_write(ctx->regmap, REG_IRQ_STAT, pval);

#ifdef VERBOSE
	dumpRegs(bridge);
#endif
}

static void sn65dsi83_disable(struct drm_bridge *bridge)
{
	struct sn65dsi83 *ctx = bridge_to_sn65dsi83(bridge);

	/* Clear reset, disable PLL */
	regmap_write(ctx->regmap, REG_RC_RESET, 0x00);
	regmap_write(ctx->regmap, REG_RC_PLL_EN, 0x00);
}

static void sn65dsi83_post_disable(struct drm_bridge *bridge)
{
	struct sn65dsi83 *ctx = bridge_to_sn65dsi83(bridge);

	/* Put the chip in reset, pull EN line low. */
	gpiod_set_value(ctx->enable_gpio, 0);
}

static enum drm_mode_status
sn65dsi83_mode_valid(struct drm_bridge *bridge,
		     const struct drm_display_info *info,
		     const struct drm_display_mode *mode)
{
	struct sn65dsi83 *ctx = bridge_to_sn65dsi83(bridge);

	/* LVDS output clock range 25..154 MHz */
	if (mode->clock < 25000)
		return MODE_CLOCK_LOW;
	if (mode->clock > 154000)
		return MODE_CLOCK_HIGH;

	switch (info->bus_formats[0]) {
	case MEDIA_BUS_FMT_RGB666_1X7X3_SPWG:
		ctx->lvds_format_24bpp = false;
		ctx->lvds_format_jeida = false;
		break;
	case MEDIA_BUS_FMT_RGB888_1X7X4_JEIDA:
		ctx->lvds_format_24bpp = true;
		ctx->lvds_format_jeida = true;
		break;
	case MEDIA_BUS_FMT_RGB888_1X7X4_SPWG:
		ctx->lvds_format_24bpp = true;
		ctx->lvds_format_jeida = false;
		break;
	default:
		ctx->lvds_format_24bpp = true;
		ctx->lvds_format_jeida = false;
		/*
		 * Some bridges still don't set the correct LVDS bus pixel
		 * format, use SPWG24 default format until those are fixed.
		 */
		dev_warn(ctx->dev,
			"Unsupported LVDS bus format 0x%04x, please check output bridge driver. Falling back to SPWG24.\n",
			info->bus_formats[0]);
		break;
	}

#ifdef VERBOSE
	printk(KERN_ERR "DSI_BRIDGE: %s: mode %ux%u@%d is valid\n", __func__, mode->hdisplay, mode->vdisplay, mode->clock);
#endif

	return MODE_OK;
}

static void sn65dsi83_mode_set(struct drm_bridge *bridge,
			       const struct drm_display_mode *mode,
			       const struct drm_display_mode *adj)
{
	struct sn65dsi83 *ctx = bridge_to_sn65dsi83(bridge);
	u32 input_bus_format = MEDIA_BUS_FMT_RGB888_1X24;
	struct drm_encoder *encoder = bridge->encoder;
	struct drm_device *ddev = encoder->dev;
	struct drm_connector *connector;

	/* The DSI format is always RGB888_1X24 */
	list_for_each_entry(connector, &ddev->mode_config.connector_list, head) {
		if (connector->encoder != encoder)
			continue;

		drm_display_info_set_bus_formats(&connector->display_info,
						 &input_bus_format, 1);
	}

	ctx->mode = *adj;
}

static const struct drm_bridge_funcs sn65dsi83_funcs = {
	.attach		= sn65dsi83_attach,
	.pre_enable	= sn65dsi83_pre_enable,
	.enable		= sn65dsi83_enable,
	.disable	= sn65dsi83_disable,
	.post_disable	= sn65dsi83_post_disable,
	.mode_valid	= sn65dsi83_mode_valid,
	.mode_set	= sn65dsi83_mode_set,
};

static int sn65dsi83_parse_dt(struct sn65dsi83 *ctx, enum sn65dsi83_model model)
{
	struct drm_bridge *panel_bridge;
	struct device *dev = ctx->dev;
	struct device_node *endpoint;
	struct drm_panel *panel;
	int ret;

	endpoint = of_graph_get_endpoint_by_regs(dev->of_node, 0, 0);

#ifdef VERBOSE
	printk(KERN_ERR "DSI_BRIDGE: %s: found endpoint %s\n",
	       __func__, endpoint->full_name ? endpoint->full_name: endpoint->name);
#endif

	ctx->dsi_lanes = of_property_count_u32_elems(endpoint, "data-lanes");
	ctx->host_node = of_graph_get_remote_port_parent(endpoint);
	of_node_put(endpoint);

#ifdef VERBOSE
	printk(KERN_ERR "DSI_BRIDGE: %s: dsi_lanes count is %d\n", __func__, ctx->dsi_lanes);
#endif

	if (ctx->dsi_lanes < 0 || ctx->dsi_lanes > 4)
		return -EINVAL;
	if (!ctx->host_node)
		return -ENODEV;

	ctx->lvds_dual_link = false;
	ctx->lvds_dual_link_even_odd_swap = false;
	if (model != MODEL_SN65DSI83) {
		struct device_node *port2, *port3;
		int dual_link;

		port2 = of_graph_get_port_by_id(dev->of_node, 2);
		port3 = of_graph_get_port_by_id(dev->of_node, 3);
		dual_link = drm_of_lvds_get_dual_link_pixel_order(port2, port3);
		of_node_put(port2);
		of_node_put(port3);

		if (dual_link == DRM_LVDS_DUAL_LINK_ODD_EVEN_PIXELS) {
			ctx->lvds_dual_link = true;
			/* Odd pixels to LVDS Channel A, even pixels to B */
			ctx->lvds_dual_link_even_odd_swap = false;
		} else if (dual_link == DRM_LVDS_DUAL_LINK_EVEN_ODD_PIXELS) {
			ctx->lvds_dual_link = true;
			/* Even pixels to LVDS Channel A, odd pixels to B */
			ctx->lvds_dual_link_even_odd_swap = true;
		}
	}

	ret = drm_of_find_panel_or_bridge(dev->of_node, 2, 0, &panel, &panel_bridge);
	if (ret < 0)
		return ret;

	if (panel) {
#ifdef VERBOSE
		printk(KERN_ERR "DSI_BRIDGE: %s: panel found\n", __func__);
		if (panel->dev) {
			if (panel->dev->of_node) {
				printk(KERN_ERR "DSI_BRIDGE: %s: panel of_node %s\n",
				       __func__, panel->dev->of_node->full_name);
			}
			printk(KERN_ERR "DSI_BRIDGE: %s: panel dev %s\n",
			       __func__, panel->dev->init_name);
		} else {
			printk(KERN_ERR "DSI_BRIDGE: %s: panel invalid dev/of_node\n",
			       __func__);
		}
#endif
		panel_bridge = devm_drm_panel_bridge_add(dev, panel);
		if (IS_ERR(panel_bridge))
			return PTR_ERR(panel_bridge);
	}

	ctx->panel_bridge = panel_bridge;

	return 0;
}

static int sn65dsi83_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	enum sn65dsi83_model model;
	struct sn65dsi83 *ctx;
	int ret;

#ifdef VERBOSE
	printk(KERN_ERR "DSI_BRIDGE: %s: init\n", __func__);
#endif

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->dev = dev;

	if (dev->of_node)
		model = (enum sn65dsi83_model)of_device_get_match_data(dev);
	else
		model = id->driver_data;

	ctx->enable_gpio = devm_gpiod_get(ctx->dev, "enable", GPIOD_OUT_LOW);
	if (IS_ERR(ctx->enable_gpio))
		return PTR_ERR(ctx->enable_gpio);

	ret = sn65dsi83_parse_dt(ctx, model);
	if (ret)
		return ret;

	ctx->regmap = devm_regmap_init_i2c(client, &sn65dsi83_regmap_config);
	if (IS_ERR(ctx->regmap))
		return PTR_ERR(ctx->regmap);

	dev_set_drvdata(dev, ctx);
	i2c_set_clientdata(client, ctx);

	ctx->bridge.funcs = &sn65dsi83_funcs;
	ctx->bridge.of_node = dev->of_node;
	drm_bridge_add(&ctx->bridge);

	return 0;
}

static int sn65dsi83_remove(struct i2c_client *client)
{
	struct sn65dsi83 *ctx = i2c_get_clientdata(client);

	drm_bridge_remove(&ctx->bridge);

	return 0;
}

static struct i2c_device_id sn65dsi83_id[] = {
	{ "ti,sn65dsi83", MODEL_SN65DSI83 },
	{ "ti,sn65dsi84", MODEL_SN65DSI84 },
	{},
};
MODULE_DEVICE_TABLE(i2c, sn65dsi83_id);

static const struct of_device_id sn65dsi83_match_table[] = {
	{ .compatible = "ti,sn65dsi83", .data = (void *)MODEL_SN65DSI83 },
	{ .compatible = "ti,sn65dsi84", .data = (void *)MODEL_SN65DSI84 },
	{},
};
MODULE_DEVICE_TABLE(of, sn65dsi83_match_table);

static struct i2c_driver sn65dsi83_driver = {
	.probe = sn65dsi83_probe,
	.remove = sn65dsi83_remove,
	.id_table = sn65dsi83_id,
	.driver = {
		.name = "sn65dsi83",
		.of_match_table = sn65dsi83_match_table,
	},
};
module_i2c_driver(sn65dsi83_driver);

MODULE_AUTHOR("Marek Vasut <marex@denx.de>");
MODULE_DESCRIPTION("TI SN65DSI83 DSI to LVDS bridge driver");
MODULE_LICENSE("GPL v2");
