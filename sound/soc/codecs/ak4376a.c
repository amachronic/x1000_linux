// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (c) 2022 Aidan MacDonald <aidanmacdonald.0x0@gmail.com>
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>

#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/tlv.h>

#define AK4376A_PM1		0x00
#define AK4376A_PM2		0x01
#define AK4376A_PM3		0x02
#define AK4376A_PM4		0x03
#define AK4376A_OUTPUT_MODE	0x04
#define AK4376A_CLOCK_MODE	0x05
#define AK4376A_DIG_FILTER	0x06
#define AK4376A_MIXER		0x07
#define AK4376A_RESERVED1	0x08
#define AK4376A_RESERVED2	0x09
#define AK4376A_RESERVED3	0x0a
#define AK4376A_LCH_VOLUME	0x0b
#define AK4376A_RCH_VOLUME	0x0c
#define AK4376A_HP_VOLUME	0x0d
#define AK4376A_PLL_CLK_SRC	0x0e
#define AK4376A_PLL_REFCLKDIV1	0x0f
#define AK4376A_PLL_REFCLKDIV2	0x10
#define AK4376A_PLL_FBCLKDIV1	0x11
#define AK4376A_PLL_FBCLKDIV2	0x12
#define AK4376A_DAC_CLK_SRC	0x13
#define AK4376A_DAC_CLK_DIV	0x14
#define AK4376A_AUDIO_IF_FMT	0x15
#define AK4376A_CHIP_ID		0x21
#define AK4376A_MODE_CTRL	0x24
#define AK4376A_DAC_ADJUST1	0x26
#define AK4376A_DAC_ADJUST2	0x2a

#define AK4376A_PLL_CLK_SRC_PLS		GENMASK(1, 0)

#define AK4376A_AUDIO_IF_FMT_BCKO	BIT(3)

enum ak4376a_supply_id {
	AK4376A_SUPPLY_AVDD,
	AK4376A_SUPPLY_CVDD,
	AK4376A_SUPPLY_LVDD,
	AK4376A_SUPPLY_TVDD,
	AK4376A_NUM_SUPPLIES
};

static const char * const ak4376a_supply_names[AK4376A_NUM_SUPPLIES] = {
	"avdd",
	"cvdd",
	"lvdd",
	"tvdd",
};

struct ak4376a_priv {
	struct regmap *regmap;
	struct regulator_bulk_data supplies[AK4376A_NUM_SUPPLIES];
	struct gpio_desc *pdn_gpio;
};

static int ak4376a_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct snd_soc_component *component = dai->component;
}

static int ak4376a_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params,
			     struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;
}

static int ak4376a_calc_pll(unsigned int freq_in, unsigned int freq_out,
			    unsigned int *pll_d, unsigned int *pll_m, unsigned int *mdiv)
{
	unsigned int d, d_min, d_max;
	unsigned long m, div;
	u64 freq, err, best_err = U64_MAX;
	int ret = -EINVAL;

	if (freq_out == 0) {
		*pll_d = 0;
		*pll_m = 0;
		*mdiv = 0;
		return 0;
	}

	d_max = freq_in / 256000;
	d_min = freq_in / 3072000;
	if (d_min < 1)
		d_min = 1;
	
	for (d = d_min; d <= d_max; ++d) {
		rational_best_approximation(freq_in/d, freq_out, 0x10000, 0x100, &m, &div);
		if (m <= 1) {
			if (m == 0 || div > 0x80)
				continue;

			m++;
			div *= 2;
		}

		freq = div_u64((u64)(freq_in/d) * m, div);
		err = freq - (u64)freq_out;

		if (err <= best_err) {
			*pll_d = d - 1;
			*pll_m = m - 1;
			*mdiv = div;
			best_err = err;
			ret = 0;

			if (err == 0)
				break;
		}
	}

	return ret;
}

static int ak4376a_set_pll(struct snd_soc_dai *dai, int pll_id, int source,
			   unsigned int freq_in, unsigned int freq_out)
{
	struct snd_soc_component *component = dai->component;
	unsigned int pll_d, pll_m, mdiv;
	int ret;

	if (pll_id != 0)
		return -EINVAL;

	ret = ak4376a_calc_pll(freq_in, freq_out, &pll_d, &pll_m, &mdiv);
	if (ret)
		return ret;

	switch (source) {
	case AK4376A_PLL_SRC_MCKI:
	case AK4376A_PLL_SRC_BCLK:
	case AK4376A_PLL_SRC_XTAL:
		snd_soc_component_write_bits(component, AK4376A_PLL_CLK_SRC,
					     AK4376A_PLL_CLK_SRC_PLS, source);
		break;
	default:
		return -EINVAL;
	}

	snd_soc_component_write(component, AK4376A_PLL_REFCLKDIV1, pll_d >> 8);
	snd_soc_component_write(component, AK4376A_PLL_REFCLKDIV2, pll_d & 0xff);
	snd_soc_component_write(component, AK4376A_PLL_FBCLKDIV1, pll_m >> 8);
	snd_soc_component_write(component, AK4376A_PLL_FBCLKDIV2, pll_m & 0xff);
	snd_soc_component_write(component, AK4376A_DAC_CLK_DIV, mdiv);
	return 0;
}

static int ak4376a_set_sysclk(struct snd_soc_dai *cpu_dai, int clk_id,
			      unsigned int freq, int dir)
{
	struct snd_soc_component *component = dai->component;

	switch (clk_id) {
	}
}

static int ak4376a_set_bclk_ratio(struct snd_soc_dai *dai, unsigned int ratio)
{
	struct snd_soc_component *component = dai->component;

	switch (ratio) {
	case 32:
		snd_soc_component_write_bits(component, AK4376A_AUDIO_IF_FMT,
					      AK4376A_AUDIO_IF_FMT_BCKO, 1);
		break;
	case 64:
		snd_soc_component_write_bits(component, AK4376A_AUDIO_IF_FMT,
					      AK4376A_AUDIO_IF_FMT_BCKO, 0);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const struct snd_soc_dai_ops ak4376a_dai_ops = {
	.set_fmt = ak4376a_set_fmt,
	.hw_params = ak4376a_hw_params,
	.set_sysclk = ak4376a_set_sysclk,
	.set_bclk_ratio = ak4376a_set_bclk_ratio,
};

static struct snd_soc_dai_driver ak4376a_dai = {
	.name = "ak4376a",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_CONTINUOUS,
		.rate_min = 8000,
		.rate_max = 384000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE |
			   SNDRV_PCM_FMTBIT_S24_LE |
			   SNDRV_PCM_FMTBIT_S32_LE,
	},
	.ops = &es9218p_dai_ops,
};

static int ak4376a_power_on(struct snd_soc_component *component)
{
	struct ak4376a_priv *priv = snd_soc_component_get_drvdata(component);
	int ret;

	ret = regulator_bulk_enable(ARRAY_SIZE(priv->supplies), priv->supplies);
	if (ret)
		return ret;

	/* 1 ms delay from regulator enable to PDN high */
	usleep_range(1000, 1500);
	gpiod_set_value_cansleep(priv->pdn_gpio, 1);

	/* 1 ms delay from PDN high to I2C interface access */
	usleep_range(1000, 1500);

	/* set values recommended by datasheet */
	regmap_write(priv->regmap, AK4376A_DAC_ADJUST1, 0x20);
	regmap_write(priv->regmap, AK4376A_DAC_ADJUST2, 0x05);

	regcache_cache_only(priv->regmap, false);
	ret = regcache_sync(priv->regmap);
	if (ret) {
		regcache_cache_only(priv->regmap, true);
		gpiod_set_value_cansleep(priv->pdn_gpio, 0);
		regulator_bulk_disable(ARRAY_SIZE(priv->supplies), priv->supplies);
		return ret;
	}

	/* 10ms delay from PDN high to analog blocks powered */
	usleep_range(9000, 10000);

	/* MCLK, BCLK, and LRCLK must be supplied */
	/* sample rate must be set */

	return 0;
}

static int ak4376a_set_bias_level(struct snd_soc_component *component,
				  enum snd_soc_bias_level level)
{
	struct ak4376a_priv *priv = snd_soc_component_get_drvdata(component);
	int ret = 0;
	
	switch (level) {
	case SND_SOC_BIAS_ON:
		break;
		
	case SND_SOC_BIAS_PREPARE:
		break;
		
	case SND_SOC_BIAS_STANDBY:
		break;

	case SND_SOC_BIAS_OFF:
		break;
	}
	
	return ret;
}

static DECLARE_TLV_DB_SCALE(hp_vol_scale, -2000, 200, 1);
static DECLARE_TLV_DB_SCALE(digital_vol_scale, -1200, 50, 1);

static const char * const digfilter_texts[] = { "Sharp", "Slow", "Short Sharp", "Short Slow" };
static SOC_ENUM_SINGLE_DECL(digfilter_enum, AK4376A_DIG_FILTER, 6, digfilter_texts);

static const char * const channel_mix_texts[] = { "Mute", "Left", "Right", "Left+Right" };
static SOC_ENUM_DOUBLE_DECL(channel_mix_enum, AK4376A_MIXER, 0, 4, digfilter_texts);

static const char * const mixer_half_texts[] = { "Full", "Half" };
static SOC_ENUM_DOUBLE_DECL(mixer_half_enum, AK4376A_MIXER, 2, 6, mixer_half_texts);

static const struct snd_kcontrol_new ak4376a_controls[] = {
	SOC_SINGLE_TLV("Headphone Playback Volume",
		       AK4376A_HP_VOLUME, 0, 0xf, 0, hp_vol_scale),
	SOC_DOUBLE_R_TLV("Master Playback Volume",
			 AK4376A_LCH_VOLUME, AK4376A_RCH_VOLUME, 0, 0x1f, 0, digital_vol_scale),
	SOC_ENUM("Digital Filter Roll-Off", digfilter_enum),
	SOC_ENUM("Channel Mixer", channel_mix_enum),
	SOC_DOUBLE("Mixer Output Level", mixer_half_texts),
};

static const struct snd_soc_dapm_widget ak4376a_dapm_widgets[] = {
	SND_SOC_DAPM_DAC("DAC", "Playback", AK4376A_PM3, 0, 0),
	SND_SOC_DAPM_PGA("HP_Amp_L", AK4376A_PM4, 0, 0, NULL, 0),
	SND_SOC_DAPM_PGA("HP_Amp_R", AK4376A_PM4, 1, 0, NULL, 0),
	SND_SOC_DAPM_OUTPUT("HPL"),
	SND_SOC_DAPM_OUTPUT("HPR"),
};

static const struct snd_soc_dapm_route ak4376a_dapm_routes[] = {
	{"HP_Amp_L", NULL, "DAC"},
	{"HP_Amp_R", NULL, "DAC"},
	{"HPL", NULL, "HP_Amp_L"},
	{"HPR", NULL, "HP_Amp_R"},
};

static const struct snd_soc_component_driver ak4376a_component = {
	.set_bias_level		= ak4376a_set_bias_level,
	.controls		= ak4376a_controls,
	.num_controls		= ARRAY_SIZE(ak4376a_controls),
	.dapm_widgets		= ak4376a_dapm_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(ak4376a_dapm_widgets),
	.dapm_routes		= ak4376a_dapm_routes,
	.num_dapm_routes	= ARRAY_SIZE(ak4376a_dapm_routes),
	.non_legacy_dai_naming	= 1,
};

static bool ak4376a_reg_accessible(struct device *dev, unsigned int reg)
{
	if (reg >= AK4376A_PM1 && reg <= AK4376A_AUDIO_IF_FMT)
		return true;

	switch (reg) {
	case AK4376A_CHIP_ID:
	case AK4376A_MODE_CTRL:
	case AK4376A_DAC_ADJUST1:
	case AK4376A_DAC_ADJUST2:
		return true;

	default:
		return false;
	}
}

static const u8 ak4376a_reg_defaults[] = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x19, 0x19, 0x0b, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x40,
};

static const struct regmap_config ak4376a_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.readable_reg = ak4376a_reg_accessible,
	.writeable_reg = ak4376a_reg_accessible,
	.reg_defaults_raw = ak4376a_reg_defaults,
	.num_reg_defaults_raw = ARRAY_SIZE(ak4376a_reg_defaults),
	.max_register = AK4376A_DAC_ADJUST2,
	.cache_type = REGCACHE_RBTREE,
};

static int ak4376a_i2c_probe(struct i2c_client *i2c,
			     const struct i2c_device_id *id)
{
	struct device *dev = &i2s->dev;
	struct ak4376a_priv *priv;
	int ret, i;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->regmap = devm_regmap_init_i2c(i2s, &ak4376a_regmap);
	if (IS_ERR(priv->regmap))
		return PTR_ERR(priv->regmap);

	for (i = 0; i < AK4376A_NUM_SUPPLIES; i++)
		priv->supplies[i].supply = ak4376a_supply_names[i];

	ret = devm_regulator_bulk_get(dev, AK4376A_NUM_SUPPLIES, priv->supplies);
	if (ret)
		return PTR_ERR(priv->regmap);

	priv->pdn_gpio = devm_gpiod_get(dev, "pdn", GPIOD_OUT_HIGH);
	if (IS_ERR(priv->pdn_gpio))
		return PTR_ERR(priv->pdn_gpio);

	i2c_set_clientdata(i2c, priv);

	return devm_snd_soc_register_component(dev, &ak4376a_component, &ak4376a_dai, 1);
}

static const struct of_device_id ak4376a_of_match[] = {
	{ .compatible = "asahi-kasei,ak4376a" },
	{ }
};
MODULE_DEVICE_TABLE(of, ak4376a_of_match);

static const struct i2c_device_id ak4376a_i2c_id[] = {
	{ "ak4376a", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ak4376a_i2c_id);

static struct i2c_driver ak4376a_i2c_driver = {
	.driver = {
		.name = "ak4376a",
		.of_match_table = of_match_ptr(ak4376a_of_match),
	},
	.probe = ak4376a_i2c_probe,
	.id_table = ak4376a_i2c_id,
};
module_i2c_driver(ak4376a_i2c_driver);

MODULE_DESCRIPTION("Asahi Kasei AK4376A ALSA SoC Codec Driver");
MODULE_AUTHOR("Aidan MacDonald <aidanmacdonald.0x0@gmail.com>");
MODULE_LICENSE("GPL");
