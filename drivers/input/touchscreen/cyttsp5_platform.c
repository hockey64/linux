/*
 * cyttsp5_platform.c
 * Parade TrueTouch(TM) Standard Product V5 Platform Module.
 * For use with Parade touchscreen controllers.
 * Supported parts include:
 * CYTMA5XX
 * CYTMA448
 * CYTMA445A
 * CYTT21XXX
 * CYTT31XXX
 *
 * Copyright (C) 2015 Parade Technologies
 * Copyright (C) 2013-2015 Cypress Semiconductor
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2, and only version 2, as published by the
 * Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Contact Parade Technologies at www.paradetech.com <ttdrivers@paradetech.com>
 *
 */

#include "cyttsp5_regs.h"
#include <linux/cyttsp5_platform.h>

static struct pt_core_commands *cmd;

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_PLATFORM_FW_UPGRADE
/* FW for Panel ID = 0x00 */
#include "cyttsp5_fw_pid00.h"
static struct cyttsp5_touch_firmware cyttsp5_firmware_pid00 = {
	.img = cyttsp4_img_pid00,
	.size = ARRAY_SIZE(cyttsp4_img_pid00),
	.ver = cyttsp4_ver_pid00,
	.vsize = ARRAY_SIZE(cyttsp4_ver_pid00),
	.panel_id = 0x00,
};

/* FW for Panel ID = 0x01 */
#include "cyttsp5_fw_pid01.h"
static struct cyttsp5_touch_firmware cyttsp5_firmware_pid01 = {
	.img = cyttsp4_img_pid01,
	.size = ARRAY_SIZE(cyttsp4_img_pid01),
	.ver = cyttsp4_ver_pid01,
	.vsize = ARRAY_SIZE(cyttsp4_ver_pid01),
	.panel_id = 0x01,
};

/* FW for Panel ID not enabled (legacy) */
#include "cyttsp5_fw.h"
static struct cyttsp5_touch_firmware cyttsp5_firmware = {
	.img = cyttsp4_img,
	.size = ARRAY_SIZE(cyttsp4_img),
	.ver = cyttsp4_ver,
	.vsize = ARRAY_SIZE(cyttsp4_ver),
};
#else
/* FW for Panel ID not enabled (legacy) */
static struct cyttsp5_touch_firmware cyttsp5_firmware = {
	.img = NULL,
	.size = 0,
	.ver = NULL,
	.vsize = 0,
};
#endif

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_PLATFORM_TTCONFIG_UPGRADE
/* TT Config for Panel ID = 0x00 */
#include "cyttsp5_params_pid00.h"
static struct touch_settings cyttsp5_sett_param_regs_pid00 = {
	.data = (uint8_t *)&cyttsp4_param_regs_pid00[0],
	.size = ARRAY_SIZE(cyttsp4_param_regs_pid00),
	.tag = 0,
};

static struct touch_settings cyttsp5_sett_param_size_pid00 = {
	.data = (uint8_t *)&cyttsp4_param_size_pid00[0],
	.size = ARRAY_SIZE(cyttsp4_param_size_pid00),
	.tag = 0,
};

static struct cyttsp5_touch_config cyttsp5_ttconfig_pid00 = {
	.param_regs = &cyttsp5_sett_param_regs_pid00,
	.param_size = &cyttsp5_sett_param_size_pid00,
	.fw_ver = ttconfig_fw_ver_pid00,
	.fw_vsize = ARRAY_SIZE(ttconfig_fw_ver_pid00),
	.panel_id = 0x00,
};

/* TT Config for Panel ID = 0x01 */
#include "cyttsp5_params_pid01.h"
static struct touch_settings cyttsp5_sett_param_regs_pid01 = {
	.data = (uint8_t *)&cyttsp4_param_regs_pid01[0],
	.size = ARRAY_SIZE(cyttsp4_param_regs_pid01),
	.tag = 0,
};

static struct touch_settings cyttsp5_sett_param_size_pid01 = {
	.data = (uint8_t *)&cyttsp4_param_size_pid01[0],
	.size = ARRAY_SIZE(cyttsp4_param_size_pid01),
	.tag = 0,
};

static struct cyttsp5_touch_config cyttsp5_ttconfig_pid01 = {
	.param_regs = &cyttsp5_sett_param_regs_pid01,
	.param_size = &cyttsp5_sett_param_size_pid01,
	.fw_ver = ttconfig_fw_ver_pid01,
	.fw_vsize = ARRAY_SIZE(ttconfig_fw_ver_pid01),
	.panel_id = 0x01,
};

/* TT Config for Panel ID not enabled (legacy)*/
#include "cyttsp5_params.h"
static struct touch_settings cyttsp5_sett_param_regs = {
	.data = (uint8_t *)&cyttsp4_param_regs[0],
	.size = ARRAY_SIZE(cyttsp4_param_regs),
	.tag = 0,
};

static struct touch_settings cyttsp5_sett_param_size = {
	.data = (uint8_t *)&cyttsp4_param_size[0],
	.size = ARRAY_SIZE(cyttsp4_param_size),
	.tag = 0,
};

static struct cyttsp5_touch_config cyttsp5_ttconfig = {
	.param_regs = &cyttsp5_sett_param_regs,
	.param_size = &cyttsp5_sett_param_size,
	.fw_ver = ttconfig_fw_ver,
	.fw_vsize = ARRAY_SIZE(ttconfig_fw_ver),
};
#else
/* TT Config for Panel ID not enabled (legacy)*/
static struct cyttsp5_touch_config cyttsp5_ttconfig = {
	.param_regs = NULL,
	.param_size = NULL,
	.fw_ver = NULL,
	.fw_vsize = 0,
};
#endif

static struct cyttsp5_touch_firmware *cyttsp5_firmwares[] = {
#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_PLATFORM_FW_UPGRADE
	&cyttsp5_firmware_pid00,
	&cyttsp5_firmware_pid01,
#endif
	NULL, /* Last item should always be NULL */
};

static struct cyttsp5_touch_config *cyttsp5_ttconfigs[] = {
#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_PLATFORM_TTCONFIG_UPGRADE
	&cyttsp5_ttconfig_pid00,
	&cyttsp5_ttconfig_pid01,
#endif
	NULL, /* Last item should always be NULL */
};

struct cyttsp5_loader_platform_data _cyttsp5_loader_platform_data = {
	.fw = &cyttsp5_firmware,
	.ttconfig = &cyttsp5_ttconfig,
	.fws = cyttsp5_firmwares,
	.ttconfigs = cyttsp5_ttconfigs,
	.flags = CY_LOADER_FLAG_NONE,
};

int cyttsp5_xres(struct cyttsp5_core_platform_data *pdata,
		struct device *dev)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);
	int rst_gpio     = pdata->rst_gpio;
#if defined(PT_TDDI_SUPPORT)
	int ddi_rst_gpio = pdata->ddi_rst_gpio;
#endif
	int rc = 0;
	int wd_enabled = cd->watchdog_enabled;

	cmd = pt_get_commands();
	if (cmd)
		cmd->request_stop_wd(dev);

	/* Toggling only TP_XRES as DDI_XRES resets the entire part */
	gpio_set_value(rst_gpio, 1);
#if defined(PT_TDDI_SUPPORT) || defined(PT_TT_DESCRETE_SUPPORT)
	gpio_set_value(ddi_rst_gpio, 1);
#endif
	msleep(20);
	gpio_set_value(rst_gpio, 0);
	msleep(50);
	gpio_set_value(rst_gpio, 1);
#if defined(PT_TDDI_SUPPORT) || defined(PT_TT_DESCRETE_SUPPORT)
	gpio_set_value(ddi_rst_gpio, 1);
#endif
	msleep(20);

	if (cmd && wd_enabled)
		cmd->request_start_wd(dev);

	pt_debug(dev, DL_WARN, "%s: 50ms HARD RESET on gpio=%d\n",
		__func__, pdata->rst_gpio);
	return rc;
}

int cyttsp5_init(struct cyttsp5_core_platform_data *pdata,
		int on, struct device *dev)
{
	int rst_gpio       = pdata->rst_gpio;
	int irq_gpio       = pdata->irq_gpio;
	int ddi_rst_gpio   = pdata->ddi_rst_gpio;
	int boot_mode_gpio = pdata->boot_mode_gpio;
	int rc = 0;

	if (on) {
		/* Configure RST GPIO */
		pt_debug(dev, DL_WARN, "%s: Request RST GPIO %d",
			__func__, rst_gpio);
		rc = gpio_request(rst_gpio, NULL);
		if (rc < 0) {
			gpio_free(rst_gpio);
			rc = gpio_request(rst_gpio, NULL);
		}
		if (rc < 0) {
			pt_debug(dev, DL_ERROR, "%s: Failed requesting RST GPIO %d\n",
				__func__, rst_gpio);
			goto fail_rst_gpio;
		} else {
			/*
			 * Set the GPIO direction and the starting level.
			 * The start level is hight because the DUT needs to
			 * stay in reset during power up.
			 */
			rc = gpio_direction_output(rst_gpio, 1);
			if (rc < 0) {
				pt_debug(dev, DL_ERROR, "%s: Failed to set RST GPIO %d as an output\n",
					__func__, rst_gpio);
				goto fail_rst_gpio;
			}
		}

		/* Configure IRQ GPIO */
		pt_debug(dev, DL_WARN, "%s: Request IRQ GPIO %d",
			__func__, irq_gpio);
		rc = gpio_request(irq_gpio, NULL);
		if (rc < 0) {
			gpio_free(irq_gpio);
			rc = gpio_request(irq_gpio, NULL);
		}
		if (rc < 0) {
			pt_debug(dev, DL_ERROR, "%s: Failed requesting IRQ GPIO %d\n",
				__func__, irq_gpio);
			goto fail_irq_gpio;
		} else {
			/* Set the GPIO direction */
			rc = gpio_direction_input(irq_gpio);
			if (rc < 0) {
				pt_debug(dev, DL_ERROR, "%s: Failed to set IRQ gpio %d as an input\n",
					__func__, irq_gpio);
				goto fail_irq_gpio;
			}
		}

		/* Configure DDI RST GPIO */
		pt_debug(dev, DL_WARN, "%s: Request DDI RST GPIO %d",
			__func__, ddi_rst_gpio);
		rc = gpio_request(ddi_rst_gpio, NULL);
		if (rc < 0) {
			gpio_free(ddi_rst_gpio);
			rc = gpio_request(ddi_rst_gpio, NULL);
		}
		if (rc < 0) {
			pt_debug(dev, DL_ERROR, "%s: Failed requesting DDI RST GPIO %d\n",
				__func__, ddi_rst_gpio);
			goto fail_ddi_rst_gpio;
		} else {
			/* Set the GPIO direction and the starting level */
			rc = gpio_direction_output(ddi_rst_gpio, 0);
			if (rc < 0) {
				pt_debug(dev, DL_ERROR, "%s: Failed to set RST GPIO %d as an output\n",
					__func__, ddi_rst_gpio);
				goto fail_ddi_rst_gpio;
			}
		}

		/* Configure BOOT MODE GPIO */
		pt_debug(dev, DL_WARN, "%s: Request BOOT MODE GPIO %d",
			__func__, boot_mode_gpio);
		rc = gpio_request(boot_mode_gpio, NULL);
		if (rc < 0) {
			gpio_free(boot_mode_gpio);
			rc = gpio_request(boot_mode_gpio, NULL);
		}
		if (rc < 0) {
			pt_debug(dev, DL_ERROR, "%s: Failed requesting BOOT MODE GPIO %d\n",
				__func__, boot_mode_gpio);
			goto fail_boot_mode_gpio;
		} else {
			/* Set the GPIO direction and the starting level */
			rc = gpio_direction_output(boot_mode_gpio, 1);
			if (rc < 0) {
				pt_debug(dev, DL_ERROR,
					"%s: Failed to set BOOT MODE GPIO %d as an output\n",
					__func__, boot_mode_gpio);
				goto fail_boot_mode_gpio;
			}
		}
		/* All GPIO's created successfully */
		goto success;
	} else {
		/* "on" not set, therefore free all gpio's */
		gpio_free(boot_mode_gpio);
		gpio_free(ddi_rst_gpio);
		gpio_free(irq_gpio);
		gpio_free(rst_gpio);
		goto success;
	}


fail_boot_mode_gpio:
	pt_debug(dev, DL_ERROR, "%s: ERROR - GPIO setup Failure, freeing Boot Mode GPIO %d\n",
		__func__, boot_mode_gpio);
	gpio_free(boot_mode_gpio);
fail_ddi_rst_gpio:
	pt_debug(dev, DL_ERROR, "%s: ERROR - GPIO setup Failure, freeing DDI_XRES GPIO %d\n",
		__func__, ddi_rst_gpio);
	gpio_free(ddi_rst_gpio);
fail_irq_gpio:
	pt_debug(dev, DL_ERROR, "%s: ERROR - GPIO setup Failure, freeing IRQ GPIO %d\n",
		__func__, irq_gpio);
	gpio_free(irq_gpio);
fail_rst_gpio:
	pt_debug(dev, DL_ERROR, "%s: ERROR - GPIO setup Failure, freeing TP_XRES GPIO %d\n",
		__func__, rst_gpio);
	gpio_free(rst_gpio);

success:
	pt_debug(dev, DL_INFO,
		"%s: SUCCESS - Configured Boot Mode GPIO %d, DDI_XRES GPIO %d, IRQ GPIO %d, TP_XRES GPIO %d\n",
		__func__, boot_mode_gpio, ddi_rst_gpio, irq_gpio, rst_gpio);
	return rc;
}

static int cyttsp5_wakeup(struct cyttsp5_core_platform_data *pdata,
		struct device *dev, atomic_t *ignore_irq)
{
	return 0;
}

static int cyttsp5_sleep(struct cyttsp5_core_platform_data *pdata,
		struct device *dev, atomic_t *ignore_irq)
{
	return 0;
}

int cyttsp5_power(struct cyttsp5_core_platform_data *pdata,
		int on, struct device *dev, atomic_t *ignore_irq)
{
	if (on)
		return cyttsp5_wakeup(pdata, dev, ignore_irq);

	return cyttsp5_sleep(pdata, dev, ignore_irq);
}

int cyttsp5_irq_stat(struct cyttsp5_core_platform_data *pdata,
		struct device *dev)
{
	return gpio_get_value(pdata->irq_gpio);
}

#ifdef CYTTSP5_DETECT_HW
int cyttsp5_detect(struct cyttsp5_core_platform_data *pdata,
		struct device *dev, cyttsp5_platform_read read)
{
	int retry = 3;
	int rc;
	char buf[1];

	while (retry--) {
		/* Perform reset, wait for 100 ms and perform read */
		pt_debug(dev, DL_WARN, "%s: Performing a reset\n",
			__func__);
		pdata->xres(pdata, dev);
		msleep(100);
		rc = read(dev, buf, 1);
		if (!rc)
			return 0;

		pt_debug(dev, DL_ERROR, "%s: Read unsuccessful, try=%d\n",
			__func__, 3 - retry);
	}

	return rc;
}
#endif
