// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * AXP192 pinctrl and GPIO driver
 *
 * Copyright (C) 2022 Aidan MacDonald <aidanmacdonald.0x0@gmail.com>
 */

#include <linux/bitops.h>
#include <linux/device.h>
#include <linux/gpio/driver.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/limits.h>
#include <linux/mfd/axp20x.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/slab.h>

enum {
	AXP192_FUNC_OUTPUT = 0,
	AXP192_FUNC_INPUT,
	AXP192_FUNC_LDO,
	AXP192_FUNC_PWM,
	AXP192_FUNC_ADC,
	AXP192_FUNC_LOW_OUTPUT,
	AXP192_FUNC_FLOATING,
	AXP192_FUNC_EXT_CHG_CTL,
	AXP192_FUNC_LDO_STATUS,
	AXP192_FUNCS_NB,
};

/**
 * struct axp192_pctl_function - describes a function that GPIOs may have
 *
 * @name: Function name
 * @muxvals: Mux values used for selecting this function, one per GPIO.
 *           The i'th element corresponds to the i'th GPIO and is written
 *           to the GPIO's control register field to select this function.
 *           U8_MAX indicates that the pin does not support this function.
 * @groups: Array of @ngroups groups listing pins supporting this function.
 * @ngroups: Number of pin groups.
 */
struct axp192_pctl_function {
	const char		*name;
	/* Mux value written to the control register to select the function (-1 if unsupported) */
	const u8		*muxvals;
	const char * const	*groups;
	unsigned int		ngroups;
};

/**
 * struct axp192_pctl_reg_info - describes a register and field
 *
 * @reg: Register address.
 * @mask: Bitmask that defines the field.
 */
struct axp192_pctl_reg_info {
	u8 reg;
	u8 mask;
};

/**
 * struct axp192_pctl_desc - describes a pin control device
 *
 * @npins: Number of pins.
 * @pins: Array of @npins pin descriptors.
 * @ctrl_regs: Describes the control register and field for selecting
 *	       pin function for each pin. Pin function mux values are
 *	       written to this field.
 * @out_regs: Describes the output state register and bit for setting
 *	      the output signal level when the pin is configured as a
 *	      GPIO output for each pin.
 * @in_regs: Describes the input state register and bit for polling
 *	     the input signal level when the pin is configured as a GPIO
 *	     input for each pin.
 * @pull_down_regs: Describes the register and bit used to enable the
 *		    pull down resistor for each pin. Use a zero mask if
 *		    the pin does not support a configurable pull down.
 *
 * @nfunctions: Number of selectable pinmux functions.
 * @functions: Array of @nfunctions function descriptors.
 */
struct axp192_pctl_desc {
	unsigned int				npins;
	const struct pinctrl_pin_desc		*pins;
	const struct axp192_pctl_reg_info	*ctrl_regs;
	const struct axp192_pctl_reg_info	*out_regs;
	const struct axp192_pctl_reg_info	*in_regs;
	const struct axp192_pctl_reg_info	*pull_down_regs;

	unsigned int				nfunctions;
	const struct axp192_pctl_function	*functions;
};

static const struct pinctrl_pin_desc axp192_pins[] = {
	PINCTRL_PIN(0, "GPIO0"),
	PINCTRL_PIN(1, "GPIO1"),
	PINCTRL_PIN(2, "GPIO2"),
	PINCTRL_PIN(3, "GPIO3"),
	PINCTRL_PIN(4, "GPIO4"),
	PINCTRL_PIN(5, "N_RSTO"),
};

static const char * const axp192_io_groups[] = { "GPIO0", "GPIO1", "GPIO2",
						 "GPIO3", "GPIO4", "N_RSTO" };
static const char * const axp192_ldo_groups[] = { "GPIO0" };
static const char * const axp192_pwm_groups[] = { "GPIO1", "GPIO2" };
static const char * const axp192_adc_groups[] = { "GPIO0", "GPIO1", "GPIO2", "GPIO3" };
static const char * const axp192_extended_io_groups[] = { "GPIO0", "GPIO1", "GPIO2" };
static const char * const axp192_ext_chg_ctl_groups[] = { "GPIO3", "GPIO4" };
static const char * const axp192_ldo_status_groups[] = { "N_RSTO" };

static const u8 axp192_output_muxvals[]		= {  0,  0,  0,  1,  1,  2 };
static const u8 axp192_input_muxvals[]		= {  1,  1,  1,  2,  2,  3 };
static const u8 axp192_ldo_muxvals[]		= {  2, -1, -1, -1, -1, -1 };
static const u8 axp192_pwm_muxvals[]		= { -1,  2,  2, -1, -1, -1 };
static const u8 axp192_adc_muxvals[]		= {  4,  4,  4,  3, -1, -1 };
static const u8 axp192_low_output_muxvals[]	= {  5,  5,  5, -1, -1, -1 };
static const u8 axp192_floating_muxvals[]	= {  6,  6,  6, -1, -1, -1 };
static const u8 axp192_ext_chg_ctl_muxvals[]	= { -1, -1, -1,  0,  0, -1 };
static const u8 axp192_ldo_status_muxvals[]	= { -1, -1, -1, -1, -1,  0 };

static const struct axp192_pctl_function axp192_functions[AXP192_FUNCS_NB] = {
	[AXP192_FUNC_OUTPUT] = {
		.name = "output",
		.muxvals = axp192_output_muxvals,
		.groups = axp192_io_groups,
		.ngroups = ARRAY_SIZE(axp192_io_groups),
	},
	[AXP192_FUNC_INPUT] = {
		.name = "input",
		.muxvals = axp192_input_muxvals,
		.groups = axp192_io_groups,
		.ngroups = ARRAY_SIZE(axp192_io_groups),
	},
	[AXP192_FUNC_LDO] = {
		.name = "ldo",
		.muxvals = axp192_ldo_muxvals,
		.groups = axp192_ldo_groups,
		.ngroups = ARRAY_SIZE(axp192_ldo_groups),
	},
	[AXP192_FUNC_PWM] = {
		.name = "pwm",
		.muxvals = axp192_pwm_muxvals,
		.groups = axp192_pwm_groups,
		.ngroups = ARRAY_SIZE(axp192_pwm_groups),
	},
	[AXP192_FUNC_ADC] = {
		.name = "adc",
		.muxvals = axp192_adc_muxvals,
		.groups = axp192_adc_groups,
		.ngroups = ARRAY_SIZE(axp192_adc_groups),
	},
	[AXP192_FUNC_LOW_OUTPUT] = {
		.name = "low_output",
		.muxvals = axp192_low_output_muxvals,
		.groups = axp192_extended_io_groups,
		.ngroups = ARRAY_SIZE(axp192_extended_io_groups),
	},
	[AXP192_FUNC_FLOATING] = {
		.name = "floating",
		.muxvals = axp192_floating_muxvals,
		.groups = axp192_extended_io_groups,
		.ngroups = ARRAY_SIZE(axp192_extended_io_groups),
	},
	[AXP192_FUNC_EXT_CHG_CTL] = {
		.name = "ext_chg_ctl",
		.muxvals = axp192_ext_chg_ctl_muxvals,
		.groups = axp192_ext_chg_ctl_groups,
		.ngroups = ARRAY_SIZE(axp192_ext_chg_ctl_groups),
	},
	[AXP192_FUNC_LDO_STATUS] = {
		.name = "ldo_status",
		.muxvals = axp192_ldo_status_muxvals,
		.groups = axp192_ldo_groups,
		.ngroups = ARRAY_SIZE(axp192_ldo_status_groups),
	},
};

static const struct axp192_pctl_reg_info axp192_pin_ctrl_regs[] = {
	{ .reg = AXP192_GPIO0_CTRL,   .mask = GENMASK(2, 0) },
	{ .reg = AXP192_GPIO1_CTRL,   .mask = GENMASK(2, 0) },
	{ .reg = AXP192_GPIO2_CTRL,   .mask = GENMASK(2, 0) },
	{ .reg = AXP192_GPIO4_3_CTRL, .mask = GENMASK(1, 0) },
	{ .reg = AXP192_GPIO4_3_CTRL, .mask = GENMASK(3, 2) },
	{ .reg = AXP192_N_RSTO_CTRL,  .mask = GENMASK(7, 6) },
};

static const struct axp192_pctl_reg_info axp192_pin_in_regs[] = {
	{ .reg = AXP192_GPIO2_0_STATE, .mask = BIT(4) },
	{ .reg = AXP192_GPIO2_0_STATE, .mask = BIT(5) },
	{ .reg = AXP192_GPIO2_0_STATE, .mask = BIT(6) },
	{ .reg = AXP192_GPIO4_3_STATE, .mask = BIT(4) },
	{ .reg = AXP192_GPIO4_3_STATE, .mask = BIT(5) },
	{ .reg = AXP192_N_RSTO_CTRL,   .mask = BIT(4) },
};

static const struct axp192_pctl_reg_info axp192_pin_out_regs[] = {
	{ .reg = AXP192_GPIO2_0_STATE, .mask = BIT(0) },
	{ .reg = AXP192_GPIO2_0_STATE, .mask = BIT(1) },
	{ .reg = AXP192_GPIO2_0_STATE, .mask = BIT(2) },
	{ .reg = AXP192_GPIO4_3_STATE, .mask = BIT(0) },
	{ .reg = AXP192_GPIO4_3_STATE, .mask = BIT(1) },
	{ .reg = AXP192_N_RSTO_CTRL,   .mask = BIT(5) },
};

static const struct axp192_pctl_reg_info axp192_pull_down_regs[] = {
	{ .reg = AXP192_GPIO2_0_PULL, .mask = BIT(0) },
	{ .reg = AXP192_GPIO2_0_PULL, .mask = BIT(1) },
	{ .reg = AXP192_GPIO2_0_PULL, .mask = BIT(2) },
	{ .reg = 0, .mask = 0 /* unsupported */ },
	{ .reg = 0, .mask = 0 /* unsupported */ },
	{ .reg = 0, .mask = 0 /* unsupported */ },
};

static const struct axp192_pctl_desc axp192_data = {
	.npins = ARRAY_SIZE(axp192_pins),
	.pins = axp192_pins,
	.ctrl_regs = axp192_pin_ctrl_regs,
	.out_regs = axp192_pin_out_regs,
	.in_regs = axp192_pin_in_regs,
	.pull_down_regs = axp192_pull_down_regs,

	.nfunctions = ARRAY_SIZE(axp192_functions),
	.functions = axp192_functions,
};

struct axp192_pctl {
	struct gpio_chip		chip;
	struct regmap			*regmap;
	struct regmap_irq_chip_data	*regmap_irqc;
	const struct axp192_pctl_desc	*desc;
	int				*irqs;
	struct pinctrl_desc		pctrl_desc;
};

static int axp192_gpio_get(struct gpio_chip *chip, unsigned int offset)
{
	struct axp192_pctl *pctl = gpiochip_get_data(chip);
	const struct axp192_pctl_reg_info *reginfo = &pctl->desc->in_regs[offset];
	unsigned int val;
	int ret;

	ret = regmap_read(pctl->regmap, reginfo->reg, &val);
	if (ret)
		return ret;

	return !!(val & reginfo->mask);
}

static int axp192_gpio_get_direction(struct gpio_chip *chip, unsigned int offset)
{
	struct axp192_pctl *pctl = gpiochip_get_data(chip);
	const struct axp192_pctl_reg_info *reginfo = &pctl->desc->ctrl_regs[offset];
	const u8 *input_muxvals = pctl->desc->functions[AXP192_FUNC_INPUT].muxvals;
	unsigned int val;
	int ret;

	ret = regmap_read(pctl->regmap, reginfo->reg, &val);
	if (ret)
		return ret;

	if ((val & reginfo->mask) == (input_muxvals[offset] << (ffs(reginfo->mask) - 1)))
		return GPIO_LINE_DIRECTION_IN;

	return GPIO_LINE_DIRECTION_OUT;
}

static void axp192_gpio_set(struct gpio_chip *chip, unsigned int offset, int value)
{
	struct axp192_pctl *pctl = gpiochip_get_data(chip);
	const struct axp192_pctl_reg_info *reginfo = &pctl->desc->out_regs[offset];

	regmap_update_bits(pctl->regmap, reginfo->reg, reginfo->mask, value ? reginfo->mask : 0);
}

static int axp192_gpio_direction_input(struct gpio_chip *chip, unsigned int offset)
{
	return pinctrl_gpio_direction_input(chip->base + offset);
}

static int axp192_gpio_direction_output(struct gpio_chip *chip, unsigned int offset, int value)
{
	chip->set(chip, offset, value);

	return pinctrl_gpio_direction_output(chip->base + offset);
}

static int axp192_gpio_to_irq(struct gpio_chip *chip, unsigned int offset)
{
	struct axp192_pctl *pctl = gpiochip_get_data(chip);

	/* GPIO IRQs are optional */
	if (!pctl->irqs[offset])
		return 0;

	return regmap_irq_get_virq(pctl->regmap_irqc, pctl->irqs[offset]);
}

static int axp192_pinconf_get_pull_down(struct pinctrl_dev *pctldev, unsigned int pin,
					bool *state)
{
	struct axp192_pctl *pctl = pinctrl_dev_get_drvdata(pctldev);
	const struct axp192_pctl_reg_info *reginfo = &pctl->desc->pull_down_regs[pin];
	unsigned int val;
	int ret;

	if (!reginfo->mask)
		return -ENOTSUPP;

	ret = regmap_read(pctl->regmap, reginfo->reg, &val);
	if (ret)
		return ret;

	*state = !!(val & reginfo->mask);
	return 0;
}

static int axp192_pinconf_set_pull_down(struct pinctrl_dev *pctldev, unsigned int pin, int value)
{
	struct axp192_pctl *pctl = pinctrl_dev_get_drvdata(pctldev);
	const struct axp192_pctl_reg_info *reginfo = &pctl->desc->pull_down_regs[pin];

	if (!reginfo->mask)
		return -ENOTSUPP;

	return regmap_update_bits(pctl->regmap, reginfo->reg, reginfo->mask,
				  value ? reginfo->mask : 0);
}

static int axp192_pinconf_get(struct pinctrl_dev *pctldev, unsigned int pin, unsigned long *config)
{
	enum pin_config_param param = pinconf_to_config_param(*config);
	unsigned int arg = 1;
	bool pull_down;
	int ret;

	switch (param) {
	case PIN_CONFIG_BIAS_DISABLE:
		ret = axp192_pinconf_get_pull_down(pctldev, pin, &pull_down);
		if (ret)
			return ret;
		if (pull_down)
			return -EINVAL;
		break;

	case PIN_CONFIG_BIAS_PULL_DOWN:
		ret = axp192_pinconf_get_pull_down(pctldev, pin, &pull_down);
		if (ret)
			return ret;
		if (!pull_down)
			return -EINVAL;
		break;

	case PIN_CONFIG_DRIVE_OPEN_DRAIN:
		/* GPIO outputs are always open-drain. */
		break;

	default:
		return -ENOTSUPP;
	}

	*config = pinconf_to_config_packed(param, arg);
	return 0;
}

static int axp192_pinconf_set(struct pinctrl_dev *pctldev, unsigned int pin,
			      unsigned long *configs, unsigned int num_configs)
{
	int ret;
	unsigned int cfg;

	for (cfg = 0; cfg < num_configs; cfg++) {
		switch (pinconf_to_config_param(configs[cfg])) {
		case PIN_CONFIG_BIAS_DISABLE:
			ret = axp192_pinconf_set_pull_down(pctldev, pin, 0);
			if (ret)
				return ret;
			break;

		case PIN_CONFIG_BIAS_PULL_DOWN:
			ret = axp192_pinconf_set_pull_down(pctldev, pin, 1);
			if (ret)
				return ret;
			break;

		case PIN_CONFIG_DRIVE_OPEN_DRAIN:
			/*
			 * GPIO outputs are always open-drain, so this is a no-op.
			 * Accept the configuration to prevent gpiolib from trying
			 * to emulate it by setting the pin to input state.
			 */
			break;

		default:
			return -ENOTSUPP;
		}
	}

	return 0;
}

static const struct pinconf_ops axp192_conf_ops = {
	.is_generic = true,
	.pin_config_get = axp192_pinconf_get,
	.pin_config_set = axp192_pinconf_set,
	.pin_config_group_get = axp192_pinconf_get,
	.pin_config_group_set = axp192_pinconf_set,
};

static int axp192_pmx_set(struct pinctrl_dev *pctldev, unsigned int offset, u8 config)
{
	struct axp192_pctl *pctl = pinctrl_dev_get_drvdata(pctldev);
	const struct axp192_pctl_reg_info *reginfo = &pctl->desc->ctrl_regs[offset];
	unsigned int regval = config << (ffs(reginfo->mask) - 1);

	return regmap_update_bits(pctl->regmap, reginfo->reg, reginfo->mask, regval);
}

static int axp192_pmx_func_cnt(struct pinctrl_dev *pctldev)
{
	struct axp192_pctl *pctl = pinctrl_dev_get_drvdata(pctldev);

	return pctl->desc->nfunctions;
}

static const char *axp192_pmx_func_name(struct pinctrl_dev *pctldev, unsigned int selector)
{
	struct axp192_pctl *pctl = pinctrl_dev_get_drvdata(pctldev);

	return pctl->desc->functions[selector].name;
}

static int axp192_pmx_func_groups(struct pinctrl_dev *pctldev, unsigned int selector,
				  const char * const **groups, unsigned int *num_groups)
{
	struct axp192_pctl *pctl = pinctrl_dev_get_drvdata(pctldev);

	*groups = pctl->desc->functions[selector].groups;
	*num_groups = pctl->desc->functions[selector].ngroups;

	return 0;
}

static int axp192_pmx_set_mux(struct pinctrl_dev *pctldev,
			      unsigned int function, unsigned int group)
{
	struct axp192_pctl *pctl = pinctrl_dev_get_drvdata(pctldev);
	const u8 *muxvals = pctl->desc->functions[function].muxvals;

	if (muxvals[group] == U8_MAX)
		return -EINVAL;

	/*
	 * Switching to LDO or PWM function will enable LDO/PWM output, so it's
	 * better to ignore these requests and let the regulator or PWM drivers
	 * handle muxing to avoid interfering with them.
	 */
	if (function == AXP192_FUNC_LDO || function == AXP192_FUNC_PWM)
		return 0;

	return axp192_pmx_set(pctldev, group, muxvals[group]);
}

static int axp192_pmx_gpio_set_direction(struct pinctrl_dev *pctldev,
					 struct pinctrl_gpio_range *range,
					 unsigned int offset, bool input)
{
	struct axp192_pctl *pctl = pinctrl_dev_get_drvdata(pctldev);
	const u8 *muxvals = input ? pctl->desc->functions[AXP192_FUNC_INPUT].muxvals
				  : pctl->desc->functions[AXP192_FUNC_OUTPUT].muxvals;

	return axp192_pmx_set(pctldev, offset, muxvals[offset]);
}

static const struct pinmux_ops axp192_pmx_ops = {
	.get_functions_count	= axp192_pmx_func_cnt,
	.get_function_name	= axp192_pmx_func_name,
	.get_function_groups	= axp192_pmx_func_groups,
	.set_mux		= axp192_pmx_set_mux,
	.gpio_set_direction	= axp192_pmx_gpio_set_direction,
	.strict			= true,
};

static int axp192_groups_cnt(struct pinctrl_dev *pctldev)
{
	struct axp192_pctl *pctl = pinctrl_dev_get_drvdata(pctldev);

	return pctl->desc->npins;
}

static const char *axp192_group_name(struct pinctrl_dev *pctldev, unsigned int selector)
{
	struct axp192_pctl *pctl = pinctrl_dev_get_drvdata(pctldev);

	return pctl->desc->pins[selector].name;
}

static int axp192_group_pins(struct pinctrl_dev *pctldev, unsigned int selector,
			     const unsigned int **pins, unsigned int *num_pins)
{
	struct axp192_pctl *pctl = pinctrl_dev_get_drvdata(pctldev);

	*pins = &pctl->desc->pins[selector].number;
	*num_pins = 1;

	return 0;
}

static const struct pinctrl_ops axp192_pctrl_ops = {
	.dt_node_to_map		= pinconf_generic_dt_node_to_map_group,
	.dt_free_map		= pinconf_generic_dt_free_map,
	.get_groups_count	= axp192_groups_cnt,
	.get_group_name		= axp192_group_name,
	.get_group_pins		= axp192_group_pins,
};

static int axp192_pctl_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct axp20x_dev *axp20x = dev_get_drvdata(dev->parent);
	struct axp192_pctl *pctl;
	struct pinctrl_dev *pctl_dev;
	int ret, i;

	pctl = devm_kzalloc(dev, sizeof(*pctl), GFP_KERNEL);
	if (!pctl)
		return -ENOMEM;

	pctl->desc = device_get_match_data(dev);
	pctl->regmap = axp20x->regmap;
	pctl->regmap_irqc = axp20x->regmap_irqc;

	pctl->chip.base			= -1;
	pctl->chip.can_sleep		= true;
	pctl->chip.request		= gpiochip_generic_request;
	pctl->chip.free			= gpiochip_generic_free;
	pctl->chip.parent		= dev;
	pctl->chip.label		= dev_name(dev);
	pctl->chip.owner		= THIS_MODULE;
	pctl->chip.get			= axp192_gpio_get;
	pctl->chip.get_direction	= axp192_gpio_get_direction;
	pctl->chip.set			= axp192_gpio_set;
	pctl->chip.direction_input	= axp192_gpio_direction_input;
	pctl->chip.direction_output	= axp192_gpio_direction_output;
	pctl->chip.to_irq		= axp192_gpio_to_irq;
	pctl->chip.ngpio		= pctl->desc->npins;

	pctl->irqs = devm_kcalloc(dev, pctl->desc->npins, sizeof(int), GFP_KERNEL);
	if (!pctl->irqs)
		return -ENOMEM;

	for (i = 0; i < pctl->desc->npins; i++) {
		ret = platform_get_irq_byname_optional(pdev, pctl->desc->pins[i].name);
		if (ret > 0)
			pctl->irqs[i] = ret;
	}

	platform_set_drvdata(pdev, pctl);

	pctl->pctrl_desc.name = dev_name(dev);
	pctl->pctrl_desc.owner = THIS_MODULE;
	pctl->pctrl_desc.pins = pctl->desc->pins;
	pctl->pctrl_desc.npins = pctl->desc->npins;
	pctl->pctrl_desc.pctlops = &axp192_pctrl_ops;
	pctl->pctrl_desc.pmxops = &axp192_pmx_ops;
	pctl->pctrl_desc.confops = &axp192_conf_ops;

	ret = devm_pinctrl_register_and_init(dev, &pctl->pctrl_desc, pctl, &pctl_dev);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to register pinctrl driver\n");

	ret = pinctrl_enable(pctl_dev);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to enable pinctrl driver\n");

	ret = devm_gpiochip_add_data(dev, &pctl->chip, pctl);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to register GPIO chip\n");

	return 0;
}

static const struct of_device_id axp192_pctl_match[] = {
	{ .compatible = "x-powers,axp192-gpio", .data = &axp192_data, },
	{ }
};
MODULE_DEVICE_TABLE(of, axp192_pctl_match);

static struct platform_driver axp192_pctl_driver = {
	.probe		= axp192_pctl_probe,
	.driver = {
		.name		= "axp192-gpio",
		.of_match_table	= axp192_pctl_match,
	},
};
module_platform_driver(axp192_pctl_driver);

MODULE_AUTHOR("Aidan MacDonald <aidanmacdonald.0x0@gmail.com>");
MODULE_DESCRIPTION("AXP192 PMIC pinctrl and GPIO driver");
MODULE_LICENSE("GPL");
