/*
 * pt_platform.c
 * Parade TrueTouch(TM) Standard Product V5 Platform Module.
 * For use with Parade touchscreen controllers.
 * Supported parts include:
 * TMA5XX
 * TMA448
 * TMA445A
 * TT21XXX
 * TT31XXX
 * TT4XXXX
 * TT7XXX
 * TC3XXX
 *
 * Copyright (C) 2015-2018 Parade Technologies
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
 */

#include "pt_regs.h"
#include <linux/pt_platform.h>

#ifdef CONFIG_TOUCHSCREEN_PARADE_PT_PLATFORM_FW_UPGRADE
/* FW for Panel ID = 0x00 */
#include "pt_fw_pid00.h"
static struct pt_touch_firmware pt_firmware_pid00 = {
	.img = pt_img_pid00,
	.size = ARRAY_SIZE(pt_img_pid00),
	.ver = pt_ver_pid00,
	.vsize = ARRAY_SIZE(pt_ver_pid00),
	.panel_id = 0x00,
};

/* FW for Panel ID = 0x01 */
#include "pt_fw_pid01.h"
static struct pt_touch_firmware pt_firmware_pid01 = {
	.img = pt_img_pid01,
	.size = ARRAY_SIZE(pt_img_pid01),
	.ver = pt_ver_pid01,
	.vsize = ARRAY_SIZE(pt_ver_pid01),
	.panel_id = 0x01,
};

/* FW for Panel ID not enabled (legacy) */
#include "pt_fw.h"
static struct pt_touch_firmware pt_firmware = {
	.img = pt_img,
	.size = ARRAY_SIZE(pt_img),
	.ver = pt_ver,
	.vsize = ARRAY_SIZE(pt_ver),
};
#else
/* FW for Panel ID not enabled (legacy) */
static struct pt_touch_firmware pt_firmware = {
	.img = NULL,
	.size = 0,
	.ver = NULL,
	.vsize = 0,
};
#endif

#ifdef CONFIG_TOUCHSCREEN_PARADE_PT_PLATFORM_TTCONFIG_UPGRADE
/* TT Config for Panel ID = 0x00 */
#include "pt_params_pid00.h"
static struct touch_settings pt_sett_param_regs_pid00 = {
	.data = (uint8_t *)&pt_param_regs_pid00[0],
	.size = ARRAY_SIZE(pt_param_regs_pid00),
	.tag = 0,
};

static struct touch_settings pt_sett_param_size_pid00 = {
	.data = (uint8_t *)&pt_param_size_pid00[0],
	.size = ARRAY_SIZE(pt_param_size_pid00),
	.tag = 0,
};

static struct pt_touch_config pt_ttconfig_pid00 = {
	.param_regs = &pt_sett_param_regs_pid00,
	.param_size = &pt_sett_param_size_pid00,
	.fw_ver = ttconfig_fw_ver_pid00,
	.fw_vsize = ARRAY_SIZE(ttconfig_fw_ver_pid00),
	.panel_id = 0x00,
};

/* TT Config for Panel ID = 0x01 */
#include "pt_params_pid01.h"
static struct touch_settings pt_sett_param_regs_pid01 = {
	.data = (uint8_t *)&pt_param_regs_pid01[0],
	.size = ARRAY_SIZE(pt_param_regs_pid01),
	.tag = 0,
};

static struct touch_settings pt_sett_param_size_pid01 = {
	.data = (uint8_t *)&pt_param_size_pid01[0],
	.size = ARRAY_SIZE(pt_param_size_pid01),
	.tag = 0,
};

static struct pt_touch_config pt_ttconfig_pid01 = {
	.param_regs = &pt_sett_param_regs_pid01,
	.param_size = &pt_sett_param_size_pid01,
	.fw_ver = ttconfig_fw_ver_pid01,
	.fw_vsize = ARRAY_SIZE(ttconfig_fw_ver_pid01),
	.panel_id = 0x01,
};

/* TT Config for Panel ID not enabled (legacy)*/
#include "pt_params.h"
static struct touch_settings pt_sett_param_regs = {
	.data = (uint8_t *)&pt_param_regs[0],
	.size = ARRAY_SIZE(pt_param_regs),
	.tag = 0,
};

static struct touch_settings pt_sett_param_size = {
	.data = (uint8_t *)&pt_param_size[0],
	.size = ARRAY_SIZE(pt_param_size),
	.tag = 0,
};

static struct pt_touch_config pt_ttconfig = {
	.param_regs = &pt_sett_param_regs,
	.param_size = &pt_sett_param_size,
	.fw_ver = ttconfig_fw_ver,
	.fw_vsize = ARRAY_SIZE(ttconfig_fw_ver),
};
#else
/* TT Config for Panel ID not enabled (legacy)*/
static struct pt_touch_config pt_ttconfig = {
	.param_regs = NULL,
	.param_size = NULL,
	.fw_ver = NULL,
	.fw_vsize = 0,
};
#endif

static struct pt_touch_firmware *pt_firmwares[] = {
#ifdef CONFIG_TOUCHSCREEN_PARADE_PT_PLATFORM_FW_UPGRADE
	&pt_firmware_pid00,
	&pt_firmware_pid01,
#endif
	NULL, /* Last item should always be NULL */
};

static struct pt_touch_config *pt_ttconfigs[] = {
#ifdef CONFIG_TOUCHSCREEN_PARADE_PT_PLATFORM_TTCONFIG_UPGRADE
	&pt_ttconfig_pid00,
	&pt_ttconfig_pid01,
#endif
	NULL, /* Last item should always be NULL */
};

struct pt_loader_platform_data _pt_loader_platform_data = {
	.fw = &pt_firmware,
	.ttconfig = &pt_ttconfig,
	.fws = pt_firmwares,
	.ttconfigs = pt_ttconfigs,
	.flags = PT_LOADER_FLAG_NONE,
};

/*******************************************************************************
 * FUNCTION: pt_xres
 *
 * SUMMARY: Toggles the reset gpio (TP_XRES) to perform a HW reset
 *
 * RETURN:
 *	 0 = success
 *	!0 = fail
 *
 * PARAMETERS:
 *  *pdata - pointer to the platform data structure
 *  *dev   - pointer to Device structure
 ******************************************************************************/
int pt_xres(struct pt_core_platform_data *pdata,
		struct device *dev)
{
	int rst_gpio = pdata->rst_gpio;
	int rc = 0;
	int ddi_rst_gpio = pdata->ddi_rst_gpio;

	pt_debug(dev, DL_WARN, "%s: 20ms HARD RESET on gpio=%d\n",
		__func__, pdata->rst_gpio);

	/* Toggling only TP_XRES as DDI_XRES resets the entire part */
	gpio_set_value(rst_gpio, 1);
	if (ddi_rst_gpio)
		gpio_set_value(ddi_rst_gpio, 1);
	usleep_range(2000, 5000);
	gpio_set_value(rst_gpio, 0);
	msleep(20);
	gpio_set_value(rst_gpio, 1);
	if (ddi_rst_gpio)
		gpio_set_value(ddi_rst_gpio, 1);

	/* Sleep to allow the DUT to boot */
	usleep_range(3000, 5000);
	return rc;
}

/*******************************************************************************
 * FUNCTION: pt_init
 *
 * SUMMARY: Set up/free gpios for TP_RST, IRQ, DDI_RST, RUNFW.
 *
 * RETURN:
 *	 0 = success
 *	!0 = fail
 *
 * PARAMETERS:
 *  *pdata - pointer to the platform data structure
 *   on    - flag to set up or free gpios(0:free; !0:set up)
 *  *dev   - pointer to Device structure
 ******************************************************************************/
int pt_init(struct pt_core_platform_data *pdata,
		int on, struct device *dev)
{
	int rst_gpio     = pdata->rst_gpio;
	int irq_gpio     = pdata->irq_gpio;
	int ddi_rst_gpio = pdata->ddi_rst_gpio;
	int runfw_gpio   = pdata->runfw_gpio;
	int rc = 0;

	if (on && rst_gpio) {
		/* Configure RST GPIO */
		pt_debug(dev, DL_WARN, "%s: Request RST GPIO %d",
			__func__, rst_gpio);
		rc = gpio_request(rst_gpio, NULL);
		if (rc < 0) {
			gpio_free(rst_gpio);
			rc = gpio_request(rst_gpio, NULL);
		}
		if (rc < 0) {
			pt_debug(dev, DL_ERROR,
				"%s: Failed requesting RST GPIO %d\n",
				__func__, rst_gpio);
			goto fail_rst_gpio;
		} else {
			/*
			 * Set the GPIO direction and the starting level
			 * The start level is high because the DUT needs
			 * to stay in reset during power up.
			 */
			rc = gpio_direction_output(rst_gpio, 1);
			if (rc < 0) {
				pt_debug(dev, DL_ERROR,
					"%s: Failed to set RST GPIO %d as an output\n",
					__func__, rst_gpio);
				goto fail_rst_gpio;
			}
		}
	}

	if (on && irq_gpio) {
		/* Configure IRQ GPIO */
		pt_debug(dev, DL_WARN, "%s: Request IRQ GPIO %d",
			__func__, irq_gpio);
		rc = gpio_request(irq_gpio, NULL);
		if (rc < 0) {
			gpio_free(irq_gpio);
			rc = gpio_request(irq_gpio, NULL);
		}
		if (rc < 0) {
			pt_debug(dev, DL_ERROR,
				"%s: Failed requesting IRQ GPIO %d\n",
				__func__, irq_gpio);
			goto fail_irq_gpio;
		} else {
			/* Set the GPIO direction */
			rc = gpio_direction_input(irq_gpio);
			if (rc < 0) {
				pt_debug(dev, DL_ERROR,
					"%s: Failed to set IRQ gpio %d as an input\n",
					__func__, irq_gpio);
				goto fail_irq_gpio;
			}
		}
	}

	if (on && ddi_rst_gpio) {
		/* Configure DDI RST GPIO */
		pt_debug(dev, DL_WARN, "%s: Request DDI RST GPIO %d",
			__func__, ddi_rst_gpio);
		rc = gpio_request(ddi_rst_gpio, NULL);
		if (rc < 0) {
			gpio_free(ddi_rst_gpio);
			rc = gpio_request(ddi_rst_gpio, NULL);
		}
		if (rc < 0) {
			pt_debug(dev, DL_ERROR,
				"%s: Failed requesting DDI RST GPIO %d\n",
				__func__, ddi_rst_gpio);
			goto fail_ddi_rst_gpio;
		} else {
			/* Set the GPIO direction and the starting level */
			rc = gpio_direction_output(ddi_rst_gpio, 0);
			if (rc < 0) {
				pt_debug(dev, DL_ERROR,
					"%s: Failed to set RST GPIO %d as an output\n",
					__func__, ddi_rst_gpio);
				goto fail_ddi_rst_gpio;
			}
		}
	}

	if (on && runfw_gpio) {
		/* Configure BOOT MODE GPIO */
		pt_debug(dev, DL_WARN, "%s: Request BOOT MODE GPIO %d",
			__func__, runfw_gpio);
		rc = gpio_request(runfw_gpio, NULL);
		if (rc < 0) {
			gpio_free(runfw_gpio);
			rc = gpio_request(runfw_gpio, NULL);
		}
		if (rc < 0) {
			pt_debug(dev, DL_ERROR,
				"%s: Failed requesting BOOT MODE GPIO %d\n",
				__func__, runfw_gpio);
			goto fail_runfw_gpio;
		} else {
			/* Set the GPIO direction and the starting level */
			rc = gpio_direction_output(runfw_gpio, 1);
			if (rc < 0) {
				pt_debug(dev, DL_ERROR,
					"%s: Failed to set BOOT MODE GPIO %d as an output\n",
					__func__, runfw_gpio);
				goto fail_runfw_gpio;
			}
		}
	}

	if (!on) {
		/* "on" not set, therefore free all gpio's */
		if (runfw_gpio)
			gpio_free(runfw_gpio);
		if (ddi_rst_gpio)
			gpio_free(ddi_rst_gpio);
		if (irq_gpio)
			gpio_free(irq_gpio);
		if (rst_gpio)
			gpio_free(rst_gpio);
	}

	/* All GPIO's created successfully */
	goto success;


fail_runfw_gpio:
	pt_debug(dev, DL_ERROR,
		"%s: ERROR - GPIO setup Failure, freeing Boot Mode GPIO %d\n",
		__func__, runfw_gpio);
	gpio_free(runfw_gpio);
fail_ddi_rst_gpio:
	pt_debug(dev, DL_ERROR,
		"%s: ERROR - GPIO setup Failure, freeing DDI_XRES GPIO %d\n",
		__func__, ddi_rst_gpio);
	gpio_free(ddi_rst_gpio);
fail_irq_gpio:
	pt_debug(dev, DL_ERROR,
		"%s: ERROR - GPIO setup Failure, freeing IRQ GPIO %d\n",
		__func__, irq_gpio);
	gpio_free(irq_gpio);
fail_rst_gpio:
	pt_debug(dev, DL_ERROR,
		"%s: ERROR - GPIO setup Failure, freeing TP_XRES GPIO %d\n",
		__func__, rst_gpio);
	gpio_free(rst_gpio);

success:
	pt_debug(dev, DL_INFO,
		"%s: SUCCESS - Configured Boot Mode GPIO %d, DDI_XRES GPIO %d, IRQ GPIO %d, TP_XRES GPIO %d\n",
		__func__, runfw_gpio, ddi_rst_gpio, irq_gpio, rst_gpio);
	return rc;
}

/*******************************************************************************
 * FUNCTION: pt_wakeup
 *
 * SUMMARY: Resume power for "power on/off" sleep strategy which against to
 *  "deepsleep" strategy.
 *
 * RETURN:
 *	 0 = success
 *	!0 = fail
 *
 * PARAMETERS:
 *  *pdata       - pointer to the platform data structure
 *  *dev         - pointer to Device structure
 *  *ignore_irq  - pointer to atomic structure to allow the host ignoring false
 *                 IRQ during power up
 ******************************************************************************/
static int pt_wakeup(struct pt_core_platform_data *pdata,
		struct device *dev, atomic_t *ignore_irq)
{
	return 0;
}

/*******************************************************************************
 * FUNCTION: pt_sleep
 *
 * SUMMARY: Suspend power for "power on/off" sleep strategy which against to
 *  "deepsleep" strategy.
 *
 * RETURN:
 *	 0 = success
 *	!0 = fail
 *
 * PARAMETERS:
 *  *pdata       - pointer to the platform data structure
 *  *dev         - pointer to Device structure
 *  *ignore_irq  - pointer to atomic structure to allow the host ignoring false
 *                 IRQ during power down
 ******************************************************************************/
static int pt_sleep(struct pt_core_platform_data *pdata,
		struct device *dev, atomic_t *ignore_irq)
{
	return 0;
}

/*******************************************************************************
 * FUNCTION: pt_power
 *
 * SUMMARY: Wrapper function to resume/suspend power with function
 *  pt_wakeup()/pt_sleep().
 *
 * RETURN:
 *	 0 = success
 *	!0 = fail
 *
 * PARAMETERS:
 *  *pdata       - pointer to the platform data structure
 *   on          - flag to remsume/suspend power(0:resume; 1:suspend)
 *  *dev         - pointer to Device structure
 *  *ignore_irq  - pointer to atomic structure to allow the host ignoring false
 *                 IRQ during power up/down
 ******************************************************************************/
int pt_power(struct pt_core_platform_data *pdata,
		int on, struct device *dev, atomic_t *ignore_irq)
{
	if (on)
		return pt_wakeup(pdata, dev, ignore_irq);

	return pt_sleep(pdata, dev, ignore_irq);
}

/*******************************************************************************
 * FUNCTION: pt_irq_stat
 *
 * SUMMARY: Obtain the level state of IRQ gpio.
 *
 * RETURN:
 *	 level state of IRQ gpio
 *
 * PARAMETERS:
 *  *pdata       - pointer to the platform data structure
 *  *dev         - pointer to Device structure
 ******************************************************************************/
int pt_irq_stat(struct pt_core_platform_data *pdata,
		struct device *dev)
{
	return gpio_get_value(pdata->irq_gpio);
}

#ifdef PT_DETECT_HW
/*******************************************************************************
 * FUNCTION: pt_detect
 *
 * SUMMARY: Detect the I2C device by reading one byte(FW sentiel) after the
 *  reset operation.
 *
 * RETURN:
 *	 0 - detected
 *  !0 - undetected
 *
 * PARAMETERS:
 *  *pdata - pointer to the platform data structure
 *  *dev   - pointer to Device structure
 *   read  - pointer to the function to perform a read operation
 ******************************************************************************/
int pt_detect(struct pt_core_platform_data *pdata,
		struct device *dev, pt_platform_read read)
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

/*******************************************************************************
 * FUNCTION: pt_setup_power
 *
 * SUMMARY: Turn on/turn off voltage regulator
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*pdata - pointer to  core platform data
 *	on     - flag to decide power state,PT_MT_POWER_ON/PT_MT_POWER_OFF
 *	*dev   - pointer to device
 ******************************************************************************/
int pt_setup_power(struct pt_core_platform_data *pdata, int on,
		struct device *dev)
{
#ifdef PT_PTSBC_SUPPORT
	int en_vcc  = GPIOE(1);
	int en_vddi = GPIOE(5);
	int en_avdd = GPIOE(6);
	int en_avee = GPIOE(7);
#endif
	int rc = 0;

	/*
	 * For TDDI parts, force part into RESET by holding DDI XRES
	 * while powering it up
	 */
	if (pdata->ddi_rst_gpio)
		gpio_set_value(pdata->ddi_rst_gpio, 0);

	/*
	 * Force part into RESET by holding XRES#(TP_XRES)
	 * while powering it up
	 */
	if (pdata->rst_gpio)
		gpio_set_value(pdata->rst_gpio, 0);

	if (on == PT_MT_POWER_ON) {
#ifdef PT_PTSBC_SUPPORT
		/*
		 * ================ PtSBC Coconut Board Rev B+ ================
		 * Enable GPIOs to turn on voltage regulators to pwr up
		 * - TC device power up order: VDDI, AVDD, AVEE
		 * - TT device power up order: VDDI, VCC (AVDD, AVEE not
		 *   needed)
		 * NOTE: VDDI must be stable for >10ms before XRES is released
		 */
		pt_debug(dev, DL_INFO,
		"%s: Enable VDDI, VCC, AVDD, AVEE\n", __func__);

		/* Turn on VDDI [Digital Interface] (+1.8v) */
		rc = gpio_request(en_vddi, NULL);
		if (rc < 0) {
			gpio_free(en_vddi);
			rc = gpio_request(en_vddi, NULL);
		}
		if (rc < 0) {
			pr_err("%s: Failed requesting VDDI GPIO %d\n",
				__func__, en_vddi);
		}
		rc = sw_gpio_setcfg(en_vddi, 1);
		if (rc)
			pr_err("%s: setcfg for VDDI GPIO %d failed\n",
				__func__, en_vddi);
		rc = gpio_direction_output(en_vddi, 1);
		gpio_free(en_vddi);
		usleep_range(3000, 4000);

		/* Turn on VCC */
		rc = gpio_request(en_vcc, NULL);
		if (rc < 0) {
			gpio_free(en_vcc);
			rc = gpio_request(en_vcc, NULL);
		}
		if (rc < 0) {
			pr_err("%s: Failed requesting VCC GPIO %d\n",
				__func__, en_vcc);
		}
		rc = sw_gpio_setcfg(en_vcc, 1);
		if (rc)
			pr_err("%s: setcfg for VDDI GPIO %d failed\n",
				__func__, en_vcc);
		rc = gpio_direction_output(en_vcc, 1);
		gpio_free(en_vcc);
		usleep_range(3000, 4000);

		/* Turn on AVDD (+5.0v) */
		rc = gpio_request(en_avdd, NULL);
		if (rc < 0) {
			gpio_free(en_avdd);
			rc = gpio_request(en_avdd, NULL);
		}
		if (rc < 0) {
			pr_err("%s: Failed requesting AVDD GPIO %d\n",
				__func__, en_avdd);
		}
		rc = sw_gpio_setcfg(en_avdd, 1);
		if (rc)
			pr_err("%s: setcfg for AVDD GPIO %d failed\n",
				__func__, en_avdd);
		rc = gpio_direction_output(en_avdd, 1);
		gpio_free(en_avdd);
		usleep_range(3000, 4000);

		/* Turn on AVEE (-5.0v) */
		rc = gpio_request(en_avee, NULL);
		if (rc < 0) {
			gpio_free(en_avee);
			rc = gpio_request(en_avee, NULL);
		}
		if (rc < 0) {
			pr_err("%s: Failed requesting AVEE GPIO %d\n",
				__func__, en_avee);
		}
		rc = sw_gpio_setcfg(en_avee, 1);
		if (rc)
			pr_err("%s: setcfg for AVEE GPIO %d failed\n",
				__func__, en_avee);
		rc = gpio_direction_output(en_avee, 1);
		gpio_free(en_avee);
		usleep_range(3000, 4000);
#endif
	} else {
		/*
		 * Disable GPIOs to turn off voltage regulators to pwr down
		 * TC device The power down order is: AVEE, AVDD, VDDI
		 * TT device The power down order is: VCC, VDDI
		 *
		 * Note:Turn off some of regulators may effect display
		 * parts for TDDI chip
		 */
#ifdef PT_PTSBC_SUPPORT
		pt_debug(dev, DL_INFO,
		"%s: Turn off VCC, AVEE, AVDD, VDDI\n", __func__);

		/* Turn off VCC */
		rc = gpio_request(en_vcc, NULL);
		if (rc < 0) {
			gpio_free(en_vcc);
			rc = gpio_request(en_vcc, NULL);
		}
		if (rc < 0) {
			pr_err("%s: Failed requesting VCC GPIO %d\n",
				__func__, en_vcc);
		}
		rc = sw_gpio_setcfg(en_vcc, 1);
		if (rc)
			pr_err("%s: setcfg for VCC GPIO %d failed\n",
				__func__, en_vcc);
		rc = gpio_direction_output(en_vcc, 0);
		gpio_free(en_vcc);

		/* Turn off AVEE (-5.0v) */
		rc = gpio_request(en_avee, NULL);
		if (rc < 0) {
			gpio_free(en_avee);
			rc = gpio_request(en_avee, NULL);
		}
		if (rc < 0) {
			pr_err("%s: Failed requesting AVEE GPIO %d\n",
				__func__, en_avee);
		}
		rc = sw_gpio_setcfg(en_avee, 1);
		if (rc)
			pr_err("%s: setcfg for AVEE GPIO %d failed\n",
				__func__, en_avee);
		rc = gpio_direction_output(en_avee, 0);
		gpio_free(en_avee);

		/* Turn off AVDD (+5.0v) */
		rc = gpio_request(en_avdd, NULL);
		if (rc < 0) {
			gpio_free(en_avdd);
			rc = gpio_request(en_avdd, NULL);
		}
		if (rc < 0) {
			pr_err("%s: Failed requesting AVDD GPIO %d\n",
				__func__, en_avdd);
		}
		rc = sw_gpio_setcfg(en_avdd, 1);
		if (rc)
			pr_err("%s: setcfg for AVDD GPIO %d failed\n",
				__func__, en_avdd);
		rc = gpio_direction_output(en_avdd, 0);
		gpio_free(en_avdd);

		/* Turn off VDDI [Digital Interface] (+1.8v) */
		rc = gpio_request(en_vddi, NULL);
		if (rc < 0) {
			gpio_free(en_vddi);
			rc = gpio_request(en_vddi, NULL);
		}
		if (rc < 0) {
			pr_err("%s: Failed requesting VDDI GPIO %d\n",
				__func__, en_vddi);
		}
		rc = sw_gpio_setcfg(en_vddi, 1);
		if (rc)
			pr_err("%s: setcfg for VDDI GPIO %d failed\n",
				__func__, en_vddi);
		rc = gpio_direction_output(en_vddi, 0);
		gpio_free(en_vddi);
		usleep_range(10000, 12000);
#endif
	}

	/* Force part out of RESET by releasing XRES#(TP_XRES) */
	if (pdata->rst_gpio)
		gpio_set_value(pdata->rst_gpio, 1);

	/* Force part out of RESET by releasing DDI XRES */
	if (pdata->ddi_rst_gpio)
		gpio_set_value(pdata->ddi_rst_gpio, 1);

	return rc;
}

