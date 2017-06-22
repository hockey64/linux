/*
 * cyttsp5_loader.c
 * Parade TrueTouch(TM) Standard Product V5 FW Loader Module.
 * For use with Parade touchscreen controllers.
 * Supported parts include:
 * CYTMA5XX
 * CYTMA448
 * CYTMA445A
 * CYTT21XXX
 * CYTT31XXX
 *
 * Copyright (C) 2015 Parade Technologies
 * Copyright (C) 2012-2015 Cypress Semiconductor, Inc.
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
#include <linux/firmware.h>

#define CYTTSP5_LOADER_NAME "cyttsp5_loader"
#define CY_FW_MANUAL_UPGRADE_FILE_NAME "cyttsp5_fw_manual_upgrade"

/* Enable UPGRADE_FW_AND_CONFIG_IN_PROBE definition
 * to perform FW and config upgrade during probe
 * instead of scheduling a work for it
 */
/* #define UPGRADE_FW_AND_CONFIG_IN_PROBE */

#define CYTTSP5_AUTO_LOAD_FOR_CORRUPTED_FW 1
#define CYTTSP5_LOADER_FW_UPGRADE_RETRY_COUNT 3

#define CYTTSP5_FW_UPGRADE \
	(defined(CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_PLATFORM_FW_UPGRADE) \
	|| defined(CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_BINARY_FW_UPGRADE))

#define CYTTSP5_TTCONFIG_UPGRADE \
	(defined(CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_PLATFORM_TTCONFIG_UPGRADE) \
	|| defined(CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_MANUAL_TTCONFIG_UPGRADE))

static const u8 cyttsp5_security_key[] = {
	0xA5, 0x01, 0x02, 0x03, 0xFF, 0xFE, 0xFD, 0x5A
};

/* Timeout values in ms. */
#define CY_LDR_REQUEST_EXCLUSIVE_TIMEOUT		500
#define CY_LDR_SWITCH_TO_APP_MODE_TIMEOUT		300

#define CY_MAX_STATUS_SIZE				32

#define CY_DATA_MAX_ROW_SIZE				256
#define CY_DATA_ROW_SIZE				128

#define CY_ARRAY_ID_OFFSET				0
#define CY_ROW_NUM_OFFSET				1
#define CY_ROW_SIZE_OFFSET				3
#define CY_ROW_DATA_OFFSET				5

#define CY_POST_TT_CFG_CRC_MASK				0x2
static inline struct cyttsp5_loader_data *cyttsp5_get_loader_data(
		struct device *dev);

static struct pt_core_commands *cmd;

#define PIP2_LAUNCH_BL_DELAY		200
#define PIP2_LAUNCH_APP_DELAY		300

struct pip2_loader_data {
	struct device *dev;
	struct completion pip2_fw_upgrade_complete; /* mutex for loader */
	u8 pip2_file_handle;
};

struct cyttsp5_loader_data {
	struct device *dev;
	struct cyttsp5_sysinfo *si;
	u8 status_buf[CY_MAX_STATUS_SIZE];
	struct completion int_running;
	struct completion calibration_complete;
#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_BINARY_FW_UPGRADE
	struct completion builtin_bin_fw_complete;
	int builtin_bin_fw_status;
	bool is_manual_upgrade_enabled;
#endif
	struct work_struct fw_and_config_upgrade;
	struct work_struct calibration_work;
	struct cyttsp5_loader_platform_data *loader_pdata;
#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_MANUAL_TTCONFIG_UPGRADE
	struct mutex config_lock;
	u8 *config_data;
	int config_size;
	bool config_loading;
#endif
	struct pip2_loader_data *pip2_data;
	bool pip2_load_fw_to_ram;
	bool pip2_load_builtin;
	u8   pip2_bl_status;
	u8   pip2_erase_status;
};

/* PIP2 BL status codes. 1-99% are "active" */
enum PIP2_BL_STATUS {
	PIP2_BL_STATUS_IDLE              = 0,
	PIP2_BL_STATUS_ACTIVE_1          = 1,
	PIP2_BL_STATUS_ACTIVE_99         = 99,
	PIP2_BL_STATUS_COMPLETE          = 100,
	PIP2_BL_STATUS_GENERAL_ERROR     = 200,
	PIP2_BL_STATUS_PIP_VERSION_ERROR = 201,
	PIP2_BL_STATUS_FW_VERSION_ERROR  = 202,
	PIP2_BL_STATUS_ERASE_ERROR       = 203,
	PIP2_BL_STATUS_FILE_CLOSE_ERROR  = 204,
	PIP2_BL_STATUS_WRITE_ERROR       = 205,
	PIP2_BL_STATUS_EXECUTE_ERROR     = 206,
	PIP2_BL_STATUS_RESET_ERROR       = 207,
	PIP2_BL_STATUS_MODE_ERROR        = 208,
};

struct cyttsp5_dev_id {
	u32 silicon_id;
	u8 rev_id;
	u32 bl_ver;
};

struct cyttsp5_hex_image {
	u8 array_id;
	u16 row_num;
	u16 row_size;
	u8 row_data[CY_DATA_ROW_SIZE];
} __packed;

#if defined(PT_TDDI_SUPPORT) || defined(PT_TT_DISCRETE_SUPPORT)
static int pt_pip2_upgrade_firmware_from_builtin(struct device *dev);
#endif

static struct cyttsp5_module loader_module;

static inline struct cyttsp5_loader_data *cyttsp5_get_loader_data(
		struct device *dev)
{
	return cyttsp5_get_module_data(dev, &loader_module);
}

#if !defined(PT_TDDI_SUPPORT) && !defined(PT_TT_DISCRETE_SUPPORT)
#if CYTTSP5_FW_UPGRADE \
	|| defined(CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_PLATFORM_TTCONFIG_UPGRADE)
static u8 cyttsp5_get_panel_id(struct device *dev)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);

	return cd->panel_id;
}
#endif
#endif

#if defined(CYTTSP5_FW_UPGRADE) || defined(CYTTSP5_TTCONFIG_UPGRADE)
/*
 * return code:
 * -1: Do not upgrade firmware
 *  0: Version info same, let caller decide
 *  1: Do a firmware upgrade
 */
static int cyttsp5_check_firmware_version(struct device *dev,
		u32 fw_ver_new, u32 fw_revctrl_new)
{
	struct cyttsp5_loader_data *ld = cyttsp5_get_loader_data(dev);
	u32 fw_ver_img;
	u32 fw_revctrl_img;

	fw_ver_img = ld->si->cydata.fw_ver_major << 8;
	fw_ver_img += ld->si->cydata.fw_ver_minor;

	pt_debug(dev, DL_INFO,
		"%s: img vers:0x%04X new vers:0x%04X\n", __func__,
			fw_ver_img, fw_ver_new);

	if (fw_ver_new > fw_ver_img) {
		pt_debug(dev, DL_WARN,
			"%s: Image is newer, will upgrade\n", __func__);
		return 1;
	}

	if (fw_ver_new < fw_ver_img) {
		pt_debug(dev, DL_WARN,
			"%s: Image is older, will NOT upgrade\n", __func__);
		return -1;
	}

	fw_revctrl_img = ld->si->cydata.revctrl;

	pt_debug(dev, DL_INFO,
		"%s: img revctrl:0x%04X new revctrl:0x%04X\n",
		__func__, fw_revctrl_img, fw_revctrl_new);

	if (fw_revctrl_new > fw_revctrl_img) {
		pt_debug(dev, DL_WARN,
			"%s: Image is newer, will upgrade\n", __func__);
		return 1;
	}

	if (fw_revctrl_new < fw_revctrl_img) {
		pt_debug(dev, DL_WARN,
			"%s: Image is older, will NOT upgrade\n", __func__);
		return -1;
	}

	return 0;
}

static void cyttsp5_calibrate_idacs(struct work_struct *calibration_work)
{
	struct cyttsp5_loader_data *ld = container_of(calibration_work,
			struct cyttsp5_loader_data, calibration_work);
	struct device *dev = ld->dev;
	u8 mode;
	u8 status;
	int rc;

	rc = cmd->request_exclusive(dev, CY_LDR_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0)
		goto exit;

	rc = cmd->nonhid_cmd->suspend_scanning(dev, 0);
	if (rc < 0)
		goto release;

	for (mode = 0; mode < 3; mode++) {
		rc = cmd->nonhid_cmd->calibrate_idacs(dev, 0, mode, &status);
		if (rc < 0)
			goto release;
	}

	rc = cmd->nonhid_cmd->resume_scanning(dev, 0);
	if (rc < 0)
		goto release;

	pt_debug(dev, DL_INFO, "%s: Calibration Done\n", __func__);

release:
	cmd->release_exclusive(dev);
exit:
	complete(&ld->calibration_complete);
}

static int cyttsp5_calibration_attention(struct device *dev)
{
	struct cyttsp5_loader_data *ld = cyttsp5_get_loader_data(dev);
	int rc = 0;

	schedule_work(&ld->calibration_work);

	cmd->unsubscribe_attention(dev, CY_ATTEN_STARTUP, CYTTSP5_LOADER_NAME,
		cyttsp5_calibration_attention, 0);

	return rc;
}


#endif /* CYTTSP5_FW_UPGRADE || CYTTSP5_TTCONFIG_UPGRADE */

#if CYTTSP5_FW_UPGRADE
static u8 *cyttsp5_get_row_(struct device *dev, u8 *row_buf,
		u8 *image_buf, int size)
{
	memcpy(row_buf, image_buf, size);
	return image_buf + size;
}

static int cyttsp5_ldr_enter_(struct device *dev, struct cyttsp5_dev_id *dev_id)
{
	int rc;
	u8 return_data[8];
	u8 mode;

	dev_id->silicon_id = 0;
	dev_id->rev_id = 0;
	dev_id->bl_ver = 0;

	cmd->request_reset(dev);

	rc = cmd->request_get_mode(dev, 0, &mode);
	if (rc < 0)
		return rc;

	if (mode == CY_MODE_UNKNOWN)
		return -EINVAL;

	if (mode == CY_MODE_OPERATIONAL) {
		rc = cmd->nonhid_cmd->start_bl(dev, 0);
		if (rc < 0)
			return rc;
	}

	rc = cmd->nonhid_cmd->get_bl_info(dev, 0, return_data);
	if (rc < 0)
		return rc;

	dev_id->silicon_id = get_unaligned_le32(&return_data[0]);
	dev_id->rev_id = return_data[4];
	dev_id->bl_ver = return_data[5] + (return_data[6] << 8)
		+ (return_data[7] << 16);

	return 0;
}

static int cyttsp5_ldr_init_(struct device *dev,
		struct cyttsp5_hex_image *row_image)
{
	return cmd->nonhid_cmd->initiate_bl(dev, 0, 8,
			(u8 *)cyttsp5_security_key, row_image->row_size,
			row_image->row_data);
}

static int cyttsp5_ldr_parse_row_(struct device *dev, u8 *row_buf,
	struct cyttsp5_hex_image *row_image)
{
	int rc = 0;

	row_image->array_id = row_buf[CY_ARRAY_ID_OFFSET];
	row_image->row_num = get_unaligned_be16(&row_buf[CY_ROW_NUM_OFFSET]);
	row_image->row_size = get_unaligned_be16(&row_buf[CY_ROW_SIZE_OFFSET]);

	if (row_image->row_size > ARRAY_SIZE(row_image->row_data)) {
		pt_debug(dev, DL_ERROR,
			"%s: row data buffer overflow\n", __func__);
		rc = -EOVERFLOW;
		goto cyttsp5_ldr_parse_row_exit;
	}

	memcpy(row_image->row_data, &row_buf[CY_ROW_DATA_OFFSET],
	       row_image->row_size);
cyttsp5_ldr_parse_row_exit:
	return rc;
}

static int cyttsp5_ldr_prog_row_(struct device *dev,
				 struct cyttsp5_hex_image *row_image)
{
	u16 length = row_image->row_size + 3;
	u8 data[3 + row_image->row_size];
	u8 offset = 0;

	data[offset++] = row_image->array_id;
	data[offset++] = LOW_BYTE(row_image->row_num);
	data[offset++] = HI_BYTE(row_image->row_num);
	memcpy(data + 3, row_image->row_data, row_image->row_size);
	return cmd->nonhid_cmd->prog_and_verify(dev, 0, length, data);
}

static int cyttsp5_ldr_verify_chksum_(struct device *dev)
{
	u8 result;
	int rc;

	rc = cmd->nonhid_cmd->verify_app_integrity(dev, 0, &result);
	if (rc)
		return rc;

	/* fail */
	if (result == 0)
		return -EINVAL;

	return 0;
}

static int cyttsp5_ldr_exit_(struct device *dev)
{
	return cmd->nonhid_cmd->launch_app(dev, 0);
}

static int cyttsp5_load_app_(struct device *dev, const u8 *fw, int fw_size)
{
	struct cyttsp5_dev_id *dev_id;
	struct cyttsp5_hex_image *row_image;
	u8 *row_buf;
	size_t image_rec_size;
	size_t row_buf_size = CY_DATA_MAX_ROW_SIZE;
	int row_count = 0;
	u8 *p;
	u8 *last_row;
	int rc;
	int rc_tmp;

	image_rec_size = sizeof(struct cyttsp5_hex_image);
	if (fw_size % image_rec_size != 0) {
		pt_debug(dev, DL_ERROR,
			"%s: Firmware image is misaligned\n", __func__);
		rc = -EINVAL;
		goto _cyttsp5_load_app_error;
	}

	pt_debug(dev, DL_INFO, "%s: start load app\n", __func__);
#ifdef TTHE_TUNER_SUPPORT
	cmd->request_tthe_print(dev, NULL, 0, "start load app");
#endif

	row_buf = kzalloc(row_buf_size, GFP_KERNEL);
	row_image = kzalloc(sizeof(struct cyttsp5_hex_image), GFP_KERNEL);
	dev_id = kzalloc(sizeof(struct cyttsp5_dev_id), GFP_KERNEL);
	if (!row_buf || !row_image || !dev_id) {
		rc = -ENOMEM;
		goto _cyttsp5_load_app_exit;
	}

	cmd->request_stop_wd(dev);

	pt_debug(dev, DL_INFO, "%s: Send BL Loader Enter\n", __func__);
#ifdef TTHE_TUNER_SUPPORT
	cmd->request_tthe_print(dev, NULL, 0, "Send BL Loader Enter");
#endif
	rc = cyttsp5_ldr_enter_(dev, dev_id);
	if (rc) {
		pt_debug(dev, DL_ERROR, "%s: Error cannot start Loader (ret=%d)\n",
			__func__, rc);
		goto _cyttsp5_load_app_exit;
	}
	pt_debug(dev, DL_INFO,
		"%s: dev: silicon id=%08X rev=%02X bl=%08X\n", __func__,
		dev_id->silicon_id, dev_id->rev_id, dev_id->bl_ver);

	/* get last row */
	last_row = (u8 *)fw + fw_size - image_rec_size;
	cyttsp5_get_row_(dev, row_buf, last_row, image_rec_size);
	cyttsp5_ldr_parse_row_(dev, row_buf, row_image);

	/* initialise bootloader */
	rc = cyttsp5_ldr_init_(dev, row_image);
	if (rc) {
		pt_debug(dev, DL_ERROR, "%s: Error cannot init Loader (ret=%d)\n",
			__func__, rc);
		goto _cyttsp5_load_app_exit;
	}

	pt_debug(dev, DL_INFO, "%s: Send BL Loader Blocks\n", __func__);
#ifdef TTHE_TUNER_SUPPORT
	cmd->request_tthe_print(dev, NULL, 0, "Send BL Loader Blocks");
#endif
	p = (u8 *)fw;
	while (p < last_row) {
		/* Get row */
		pt_debug(dev, DL_INFO, "%s: read row=%d\n",
			__func__, ++row_count);
		memset(row_buf, 0, row_buf_size);
		p = cyttsp5_get_row_(dev, row_buf, p, image_rec_size);

		/* Parse row */
		pt_debug(dev, DL_INFO, "%s: p=%p buf=%p buf[0]=%02X\n",
			__func__, p, row_buf, row_buf[0]);
		rc = cyttsp5_ldr_parse_row_(dev, row_buf, row_image);
		pt_debug(dev, DL_INFO,
			"%s: array_id=%02X row_num=%04X(%d) row_size=%04X(%d)\n",
			__func__, row_image->array_id,
			row_image->row_num, row_image->row_num,
			row_image->row_size, row_image->row_size);
		if (rc) {
			pt_debug(dev, DL_ERROR, "%s: Parse Row Error (a=%d r=%d ret=%d\n",
				__func__, row_image->array_id,
				row_image->row_num, rc);
			goto _cyttsp5_load_app_exit;
		} else {
			pt_debug(dev, DL_INFO,
				"%s: Parse Row (a=%d r=%d ret=%d\n",
				__func__, row_image->array_id,
				row_image->row_num, rc);
		}

		/* program row */
		rc = cyttsp5_ldr_prog_row_(dev, row_image);
		if (rc) {
			pt_debug(dev, DL_ERROR, "%s: Program Row Error (array=%d row=%d ret=%d)\n",
				__func__, row_image->array_id,
				row_image->row_num, rc);
			goto _cyttsp5_load_app_exit;
		}

		pt_debug(dev, DL_INFO,
			"%s: array=%d row_cnt=%d row_num=%04X\n",
			__func__, row_image->array_id, row_count,
			row_image->row_num);
	}

	/* exit loader */
	pt_debug(dev, DL_INFO, "%s: Send BL Loader Terminate\n", __func__);
#ifdef TTHE_TUNER_SUPPORT
	cmd->request_tthe_print(dev, NULL, 0, "Send BL Loader Terminate");
#endif
	rc = cyttsp5_ldr_exit_(dev);
	if (rc) {
		pt_debug(dev, DL_ERROR, "%s: Error on exit Loader (ret=%d)\n",
			__func__, rc);

		/* verify app checksum */
		rc_tmp = cyttsp5_ldr_verify_chksum_(dev);
		if (rc_tmp)
			pt_debug(dev, DL_ERROR, "%s: ldr_verify_chksum fail r=%d\n",
				__func__, rc_tmp);
		else
			pt_debug(dev, DL_INFO,
				"%s: APP Checksum Verified\n", __func__);
	}

_cyttsp5_load_app_exit:
	kfree(row_buf);
	kfree(row_image);
	kfree(dev_id);
_cyttsp5_load_app_error:
	return rc;
}

static int cyttsp5_upgrade_firmware(struct device *dev, const u8 *fw_img,
		int fw_size)
{
	struct cyttsp5_loader_data *ld = cyttsp5_get_loader_data(dev);
	int retry = CYTTSP5_LOADER_FW_UPGRADE_RETRY_COUNT;
	bool wait_for_calibration_complete = false;
	int rc;

	pm_runtime_get_sync(dev);

	rc = cmd->request_exclusive(dev, CY_LDR_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0)
		goto exit;

	while (retry--) {
		rc = cyttsp5_load_app_(dev, fw_img, fw_size);
		if (rc < 0)
			pt_debug(dev, DL_ERROR,
				"%s: Firmware update failed rc=%d, retry:%d\n",
				__func__, rc, retry);
		else
			break;
		msleep(20);
	}
	if (rc < 0) {
		pt_debug(dev, DL_ERROR,
			"%s: Firmware update failed with error code %d\n",
			__func__, rc);
	} else if (ld->loader_pdata &&
			(ld->loader_pdata->flags
			 & CY_LOADER_FLAG_CALIBRATE_AFTER_FW_UPGRADE)) {
#if (KERNEL_VERSION(3, 13, 0) <= LINUX_VERSION_CODE)
		reinit_completion(&ld->calibration_complete);
#else
		INIT_COMPLETION(ld->calibration_complete);
#endif
		/* set up call back for startup */
		pt_debug(dev, DL_INFO,
			"%s: Adding callback for calibration\n", __func__);
		rc = cmd->subscribe_attention(dev, CY_ATTEN_STARTUP,
			CYTTSP5_LOADER_NAME, cyttsp5_calibration_attention, 0);
		if (rc) {
			pt_debug(dev, DL_ERROR,
				"%s: Failed adding callback for calibration\n",
				__func__);
			pt_debug(dev, DL_ERROR,
				"%s: No calibration will be performed\n",
				__func__);
			rc = 0;
		} else
			wait_for_calibration_complete = true;
	}

	cmd->release_exclusive(dev);

exit:
	if (!rc)
		cmd->request_restart(dev, true);

	pm_runtime_put_sync(dev);

	if (wait_for_calibration_complete)
		wait_for_completion(&ld->calibration_complete);

	return rc;
}

static int cyttsp5_loader_attention(struct device *dev)
{
	struct cyttsp5_loader_data *ld = cyttsp5_get_loader_data(dev);

	complete(&ld->int_running);
	return 0;
}
#endif /* CYTTSP5_FW_UPGRADE */

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_PLATFORM_FW_UPGRADE
static int cyttsp5_check_firmware_version_platform(struct device *dev,
		struct cyttsp5_touch_firmware *fw)
{
	struct cyttsp5_loader_data *ld = cyttsp5_get_loader_data(dev);
	u32 fw_ver_new;
	u32 fw_revctrl_new;
	int upgrade;

	if (!ld->si) {
		pt_debug(dev, DL_INFO,
			"%s: No firmware info found, DUT FW may be corrupted\n",
			__func__);
		return CYTTSP5_AUTO_LOAD_FOR_CORRUPTED_FW;
	}

	fw_ver_new = get_unaligned_be16(fw->ver + 2);
	/* 4 middle bytes are not used */
	fw_revctrl_new = get_unaligned_be32(fw->ver + 8);

	upgrade = cyttsp5_check_firmware_version(dev, fw_ver_new,
		fw_revctrl_new);

	if (upgrade > 0)
		return 1;

	return 0;
}

static struct cyttsp5_touch_firmware *cyttsp5_get_platform_firmware(
		struct device *dev)
{
	struct cyttsp5_loader_data *ld = cyttsp5_get_loader_data(dev);
	struct cyttsp5_touch_firmware **fws;
	struct cyttsp5_touch_firmware *fw;
	u8 panel_id;

	panel_id = cyttsp5_get_panel_id(dev);
	if (panel_id == PANEL_ID_NOT_ENABLED) {
		pt_debug(dev, DL_WARN,
			"%s: Panel ID not enabled, using legacy firmware\n",
			__func__);
		return ld->loader_pdata->fw;
	}

	fws = ld->loader_pdata->fws;
	if (!fws) {
		pt_debug(dev, DL_ERROR,
			"%s: No firmwares provided\n", __func__);
		return NULL;
	}

	/* Find FW according to the Panel ID */
	while ((fw = *fws++)) {
		if (fw->panel_id == panel_id) {
			pt_debug(dev, DL_WARN,
				"%s: Found matching fw:%p with Panel ID: 0x%02X\n",
				__func__, fw, fw->panel_id);
			return fw;
		}
		pt_debug(dev, DL_WARN,
			"%s: Found mismatching fw:%p with Panel ID: 0x%02X\n",
			__func__, fw, fw->panel_id);
	}

	return NULL;
}

static int upgrade_firmware_from_platform(struct device *dev,
		bool forced)
{
	struct cyttsp5_loader_data *ld = cyttsp5_get_loader_data(dev);
	struct cyttsp5_touch_firmware *fw;
	int rc = -ENODEV;
	int upgrade;

	if (!ld->loader_pdata) {
		pt_debug(dev, DL_ERROR,
			"%s: No loader platform data\n", __func__);
		return rc;
	}

	fw = cyttsp5_get_platform_firmware(dev);
	if (!fw || !fw->img || !fw->size) {
		pt_debug(dev, DL_ERROR,
			"%s: No platform firmware\n", __func__);
		return rc;
	}

	if (!fw->ver || !fw->vsize) {
		pt_debug(dev, DL_ERROR, "%s: No platform firmware version\n",
			__func__);
		return rc;
	}

	if (forced)
		upgrade = forced;
	else
		upgrade = cyttsp5_check_firmware_version_platform(dev, fw);

	if (upgrade)
		return cyttsp5_upgrade_firmware(dev, fw->img, fw->size);

	return rc;
}
#endif /* CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_PLATFORM_FW_UPGRADE */

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_BINARY_FW_UPGRADE
static void _cyttsp5_firmware_cont(const struct firmware *fw, void *context)
{
	struct device *dev = context;
	struct cyttsp5_loader_data *ld = cyttsp5_get_loader_data(dev);
	u8 header_size = 0;

	if (!fw)
		goto cyttsp5_firmware_cont_exit;

	if (!fw->data || !fw->size) {
		pt_debug(dev, DL_ERROR,
			"%s: No firmware received\n", __func__);
		goto cyttsp5_firmware_cont_release_exit;
	}

	header_size = fw->data[0];
	if (header_size >= (fw->size + 1)) {
		pt_debug(dev, DL_ERROR,
			"%s: Firmware format is invalid\n", __func__);
		goto cyttsp5_firmware_cont_release_exit;
	}

	cyttsp5_upgrade_firmware(dev, &(fw->data[header_size + 1]),
		fw->size - (header_size + 1));

cyttsp5_firmware_cont_release_exit:
	release_firmware(fw);

cyttsp5_firmware_cont_exit:
	ld->is_manual_upgrade_enabled = 0;
}

#if !defined(PT_TDDI_SUPPORT) && !defined(PT_TT_DISCRETE_SUPPORT)
static int cyttsp5_check_firmware_version_builtin(struct device *dev,
		const struct firmware *fw)
{
	struct cyttsp5_loader_data *ld = cyttsp5_get_loader_data(dev);
	u32 fw_ver_new;
	u32 fw_revctrl_new;
	int upgrade;

	if (!ld->si) {
		pt_debug(dev, DL_INFO,
			"%s: No firmware info found, DUT FW may be corrupted\n",
			__func__);
		return CYTTSP5_AUTO_LOAD_FOR_CORRUPTED_FW;
	}

	fw_ver_new = get_unaligned_be16(fw->data + 3);
	/* 4 middle bytes are not used */
	fw_revctrl_new = get_unaligned_be32(fw->data + 9);

	upgrade = cyttsp5_check_firmware_version(dev, fw_ver_new,
			fw_revctrl_new);

	if (upgrade > 0)
		return 1;

	return 0;
}

static void _cyttsp5_firmware_cont_builtin(const struct firmware *fw,
		void *context)
{
	struct device *dev = context;
	struct cyttsp5_loader_data *ld = cyttsp5_get_loader_data(dev);
	int upgrade;

	if (!fw) {
		pt_debug(dev, DL_INFO,
			"%s: No builtin firmware\n", __func__);
		goto _cyttsp5_firmware_cont_builtin_exit;
	}

	if (!fw->data || !fw->size) {
		pt_debug(dev, DL_ERROR,
			"%s: Invalid builtin firmware\n", __func__);
		goto _cyttsp5_firmware_cont_builtin_exit;
	}

	pt_debug(dev, DL_INFO, "%s: Found firmware\n", __func__);

	upgrade = cyttsp5_check_firmware_version_builtin(dev, fw);
	if (upgrade) {
		_cyttsp5_firmware_cont(fw, dev);
		ld->builtin_bin_fw_status = 0;
		complete(&ld->builtin_bin_fw_complete);
		return;
	}

_cyttsp5_firmware_cont_builtin_exit:
	release_firmware(fw);

	ld->builtin_bin_fw_status = -EINVAL;
	complete(&ld->builtin_bin_fw_complete);
}
#endif /* --- !(PT_TDDI_SUPPORT) && !(PT_TT_DISCRETE_SUPPORT) --- */

static int upgrade_firmware_from_class(struct device *dev)
{
	int retval;

	pt_debug(dev, DL_INFO,
		"%s: Enabling firmware class loader\n", __func__);

	retval = request_firmware_nowait(THIS_MODULE, FW_ACTION_NOHOTPLUG,
			CY_FW_MANUAL_UPGRADE_FILE_NAME, dev, GFP_KERNEL, dev,
			_cyttsp5_firmware_cont);
	if (retval < 0) {
		pt_debug(dev, DL_ERROR,
			"%s: Fail request firmware class file load\n",
			__func__);
		return retval;
	}

	return 0;
}

/* Parade TDDIs do not support panel ID */
#if !defined(PT_TDDI_SUPPORT)
/*
 * Generates binary FW filename as following:
 * - Panel ID not enabled: cyttsp5_fw.bin
 * - Panel ID enabled: cyttsp5_fw_pidXX.bin
 */
static char *generate_firmware_filename(struct device *dev)
{
	char *filename;
	u8 panel_id;

#define FILENAME_LEN_MAX 64
	filename = kzalloc(FILENAME_LEN_MAX, GFP_KERNEL);
	if (!filename)
		return NULL;

	panel_id = cyttsp5_get_panel_id(dev);
	if (panel_id == PANEL_ID_NOT_ENABLED)
		snprintf(filename, FILENAME_LEN_MAX, "%s", CY_FW_FILE_NAME);
	else
		snprintf(filename, FILENAME_LEN_MAX, "%s_pid%02X%s",
			CY_FW_FILE_PREFIX, panel_id, CY_FW_FILE_SUFFIX);

	pt_debug(dev, DL_INFO, "%s: Filename: %s\n",
		__func__, filename);

	return filename;
}

static int upgrade_firmware_from_builtin(struct device *dev)
{
	struct cyttsp5_loader_data *ld = cyttsp5_get_loader_data(dev);
	char *filename;
	int retval;

	pt_debug(dev, DL_INFO,
		"%s: Enabling firmware class loader built-in\n",
		__func__);

	filename = generate_firmware_filename(dev);
	if (!filename)
		return -ENOMEM;

	retval = request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
			filename, dev, GFP_KERNEL, dev,
			_cyttsp5_firmware_cont_builtin);
	if (retval < 0) {
		pt_debug(dev, DL_ERROR,
			"%s: Fail request firmware class file load\n",
			__func__);
		goto exit;
	}

	/* wait until FW binary upgrade finishes */
	wait_for_completion(&ld->builtin_bin_fw_complete);

	retval = ld->builtin_bin_fw_status;

exit:
	kfree(filename);

	return retval;
}
#endif /* --- !(PT_TDDI_SUPPORT) --- */
#endif /* CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_BINARY_FW_UPGRADE */

#if CYTTSP5_TTCONFIG_UPGRADE
static int cyttsp5_write_config_row_(struct device *dev, u8 ebid,
		u16 row_number, u16 row_size, u8 *data)
{
	int rc;
	u16 actual_write_len;

	rc = cmd->nonhid_cmd->write_conf_block(dev, 0, row_number,
			row_size, ebid, data, (u8 *)cyttsp5_security_key,
			&actual_write_len);
	if (rc) {
		pt_debug(dev, DL_ERROR,
			"%s: Fail Put EBID=%d row=%d cmd fail r=%d\n",
			__func__, ebid, row_number, rc);
		return rc;
	}

	if (actual_write_len != row_size) {
		pt_debug(dev, DL_ERROR,
			"%s: Fail Put EBID=%d row=%d wrong write size=%d\n",
			__func__, ebid, row_number, actual_write_len);
		rc = -EINVAL;
	}

	return rc;
}

static int cyttsp5_upgrade_ttconfig(struct device *dev,
		const u8 *ttconfig_data, int ttconfig_size)
{
	struct cyttsp5_loader_data *ld = cyttsp5_get_loader_data(dev);
	bool wait_for_calibration_complete = false;
	u8 ebid = CY_TCH_PARM_EBID;
	u16 row_size = CY_DATA_ROW_SIZE;
	u16 table_size;
	u16 row_count;
	u16 residue;
	u8 *row_buf;
	u8 verify_crc_status;
	u16 calculated_crc;
	u16 stored_crc;
	int rc = 0;
	int i;

	table_size = ttconfig_size;
	row_count = table_size / row_size;
	row_buf = (u8 *)ttconfig_data;
	pt_debug(dev, DL_INFO, "%s: size:%d row_size=%d row_count=%d\n",
		__func__, table_size, row_size, row_count);

	pm_runtime_get_sync(dev);

	rc = cmd->request_exclusive(dev, CY_LDR_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0)
		goto exit;

	rc = cmd->nonhid_cmd->suspend_scanning(dev, 0);
	if (rc < 0)
		goto release;

	for (i = 0; i < row_count; i++) {
		pt_debug(dev, DL_INFO, "%s: row=%d size=%d\n",
			__func__, i, row_size);
		rc = cyttsp5_write_config_row_(dev, ebid, i, row_size,
				row_buf);
		if (rc) {
			pt_debug(dev, DL_ERROR, "%s: Fail put row=%d r=%d\n",
				__func__, i, rc);
			break;
		}
		row_buf += row_size;
	}
	if (!rc) {
		residue = table_size % row_size;
		pt_debug(dev, DL_WARN, "%s: row=%d size=%d\n",
			__func__, i, residue);
		rc = cyttsp5_write_config_row_(dev, ebid, i, residue,
				row_buf);
		row_count++;
		if (rc)
			pt_debug(dev, DL_ERROR, "%s: Fail put row=%d r=%d\n",
				__func__, i, rc);
	}

	if (!rc)
		pt_debug(dev, DL_WARN,
			"%s: TT_CFG updated: rows:%d bytes:%d\n",
			__func__, row_count, table_size);

	rc = cmd->nonhid_cmd->verify_config_block_crc(dev, 0, ebid,
			&verify_crc_status, &calculated_crc, &stored_crc);
	if (rc || verify_crc_status)
		pt_debug(dev, DL_ERROR,
			"%s: CRC Failed, ebid=%d, status=%d, scrc=%X ccrc=%X\n",
			__func__, ebid, verify_crc_status,
			calculated_crc, stored_crc);
	else
		pt_debug(dev, DL_INFO,
			"%s: CRC PASS, ebid=%d, status=%d, scrc=%X ccrc=%X\n",
			__func__, ebid, verify_crc_status,
			calculated_crc, stored_crc);

	rc = cmd->nonhid_cmd->resume_scanning(dev, 0);
	if (rc < 0)
		goto release;

	if (ld->loader_pdata &&
			(ld->loader_pdata->flags
			 & CY_LOADER_FLAG_CALIBRATE_AFTER_TTCONFIG_UPGRADE)) {
#if (KERNEL_VERSION(3, 13, 0) <= LINUX_VERSION_CODE)
		reinit_completion(&ld->calibration_complete);
#else
		INIT_COMPLETION(ld->calibration_complete);
#endif
		/* set up call back for startup */
		pt_debug(dev, DL_INFO, "%s: Adding callback for calibration\n",
			__func__);
		rc = cmd->subscribe_attention(dev, CY_ATTEN_STARTUP,
			CYTTSP5_LOADER_NAME, cyttsp5_calibration_attention, 0);
		if (rc) {
			pt_debug(dev, DL_ERROR,
				"%s: Failed adding callback for calibration\n",
				__func__);
			pt_debug(dev, DL_ERROR,
				"%s: No calibration will be performed\n",
				__func__);
			rc = 0;
		} else
			wait_for_calibration_complete = true;
	}

release:
	cmd->release_exclusive(dev);

exit:
	if (!rc)
		cmd->request_restart(dev, true);

	pm_runtime_put_sync(dev);

	if (wait_for_calibration_complete)
		wait_for_completion(&ld->calibration_complete);

	return rc;
}
#endif /* CYTTSP5_TTCONFIG_UPGRADE */

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_PLATFORM_TTCONFIG_UPGRADE
static int cyttsp5_get_ttconfig_crc(struct device *dev,
		const u8 *ttconfig_data, int ttconfig_size, u16 *crc)
{
	u16 crc_loc;

	crc_loc = get_unaligned_le16(&ttconfig_data[2]);
	if (ttconfig_size < crc_loc + 2)
		return -EINVAL;

	*crc = get_unaligned_le16(&ttconfig_data[crc_loc]);

	return 0;
}

static int cyttsp5_get_ttconfig_version(struct device *dev,
		const u8 *ttconfig_data, int ttconfig_size, u16 *version)
{
	if (ttconfig_size < CY_TTCONFIG_VERSION_OFFSET
			+ CY_TTCONFIG_VERSION_SIZE)
		return -EINVAL;

	*version = get_unaligned_le16(
		&ttconfig_data[CY_TTCONFIG_VERSION_OFFSET]);

	return 0;
}

static int cyttsp5_check_ttconfig_version(struct device *dev,
		const u8 *ttconfig_data, int ttconfig_size)
{
	struct cyttsp5_loader_data *ld = cyttsp5_get_loader_data(dev);
	u16 cfg_crc_new;
	int rc;

	if (!ld->si)
		return 0;

	/* Check for config version */
	if (ld->loader_pdata->flags &
			CY_LOADER_FLAG_CHECK_TTCONFIG_VERSION) {
		u16 cfg_ver_new;

		rc = cyttsp5_get_ttconfig_version(dev, ttconfig_data,
				ttconfig_size, &cfg_ver_new);
		if (rc)
			return 0;

		pt_debug(dev, DL_INFO, "%s: img_ver:0x%04X new_ver:0x%04X\n",
			__func__, ld->si->cydata.fw_ver_conf, cfg_ver_new);

		/* Check if config version is newer */
		if (cfg_ver_new > ld->si->cydata.fw_ver_conf) {
			pt_debug(dev, DL_WARN,
			"%s: Config version newer, will upgrade\n", __func__);
			return 1;
		}

		pt_debug(dev, DL_WARN,
			"%s: Config version is identical or older, will NOT upgrade\n",
			__func__);
	/* Check for config CRC */
	} else {
		rc = cyttsp5_get_ttconfig_crc(dev, ttconfig_data,
				ttconfig_size, &cfg_crc_new);
		if (rc)
			return 0;

		pt_debug(dev, DL_INFO, "%s: img_crc:0x%04X new_crc:0x%04X\n",
			__func__, ld->si->ttconfig.crc, cfg_crc_new);

		if (cfg_crc_new != ld->si->ttconfig.crc) {
			pt_debug(dev, DL_WARN,
				"%s: Config CRC different, will upgrade\n",
				__func__);
			return 1;
		}

		pt_debug(dev, DL_WARN,
			"%s: Config CRC equal, will NOT upgrade\n", __func__);
	}

	return 0;
}

static int cyttsp5_check_ttconfig_version_platform(struct device *dev,
		struct cyttsp5_touch_config *ttconfig)
{
	struct cyttsp5_loader_data *ld = cyttsp5_get_loader_data(dev);
	u32 fw_ver_config;
	u32 fw_revctrl_config;

	if (!ld->si) {
		pt_debug(dev, DL_INFO,
			"%s: No firmware info found, DUT FW may be corrupted\n",
			__func__);
		return 0;
	}

	fw_ver_config = get_unaligned_be16(ttconfig->fw_ver + 2);
	/* 4 middle bytes are not used */
	fw_revctrl_config = get_unaligned_be32(ttconfig->fw_ver + 8);

	/* FW versions should match */
	if (cyttsp5_check_firmware_version(dev, fw_ver_config,
			fw_revctrl_config)) {
		pt_debug(dev, DL_ERROR,
			"%s: FW versions mismatch\n", __func__);
		return 0;
	}

	/* Check PowerOn Self Test, TT_CFG CRC bit */
	if ((ld->si->cydata.post_code & CY_POST_TT_CFG_CRC_MASK) == 0) {
		pt_debug(dev, DL_ERROR,
			"%s: POST, TT_CFG failed (%X), will upgrade\n",
			__func__, ld->si->cydata.post_code);
		return 1;
	}

	return cyttsp5_check_ttconfig_version(dev, ttconfig->param_regs->data,
			ttconfig->param_regs->size);
}

static struct cyttsp5_touch_config *cyttsp5_get_platform_ttconfig(
		struct device *dev)
{
	struct cyttsp5_loader_data *ld = cyttsp5_get_loader_data(dev);
	struct cyttsp5_touch_config **ttconfigs;
	struct cyttsp5_touch_config *ttconfig;
	u8 panel_id;

	panel_id = cyttsp5_get_panel_id(dev);
	if (panel_id == PANEL_ID_NOT_ENABLED) {
		/* TODO: Make debug message */
		pt_debug(dev, DL_INFO,
			"%s: Panel ID not enabled, using legacy ttconfig\n",
			__func__);
		return ld->loader_pdata->ttconfig;
	}

	ttconfigs = ld->loader_pdata->ttconfigs;
	if (!ttconfigs)
		return NULL;

	/* Find TT config according to the Panel ID */
	while ((ttconfig = *ttconfigs++)) {
		if (ttconfig->panel_id == panel_id) {
			/* TODO: Make debug message */
			pt_debug(dev, DL_INFO,
				"%s: Found matching ttconfig:%p with Panel ID: 0x%02X\n",
				__func__, ttconfig, ttconfig->panel_id);
			return ttconfig;
		}
		pt_debug(dev, DL_ERROR,
			"%s: Found mismatching ttconfig:%p with Panel ID: 0x%02X\n",
			__func__, ttconfig, ttconfig->panel_id);
	}

	return NULL;
}

static int upgrade_ttconfig_from_platform(struct device *dev)
{
	struct cyttsp5_loader_data *ld = cyttsp5_get_loader_data(dev);
	struct cyttsp5_touch_config *ttconfig;
	struct touch_settings *param_regs;
	struct cyttsp5_touch_fw;
	int rc = -ENODEV;
	int upgrade;

	if (!ld->loader_pdata) {
		pt_debug(dev, DL_ERROR,
			"%s: No loader platform data\n", __func__);
		return rc;
	}

	ttconfig = cyttsp5_get_platform_ttconfig(dev);
	if (!ttconfig) {
		pt_debug(dev, DL_ERROR, "%s: No ttconfig data\n", __func__);
		return rc;
	}

	param_regs = ttconfig->param_regs;
	if (!param_regs) {
		pt_debug(dev, DL_ERROR, "%s: No touch parameters\n",
			__func__);
		return rc;
	}

	if (!param_regs->data || !param_regs->size) {
		pt_debug(dev, DL_ERROR,
			"%s: Invalid touch parameters\n", __func__);
		return rc;
	}

	if (!ttconfig->fw_ver || !ttconfig->fw_vsize) {
		pt_debug(dev, DL_ERROR,
			"%s: Invalid FW version for touch parameters\n",
			__func__);
		return rc;
	}

	upgrade = cyttsp5_check_ttconfig_version_platform(dev, ttconfig);
	if (upgrade)
		return cyttsp5_upgrade_ttconfig(dev, param_regs->data,
				param_regs->size);

	return rc;
}
#endif /* CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_PLATFORM_TTCONFIG_UPGRADE */

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_MANUAL_TTCONFIG_UPGRADE
static ssize_t cyttsp5_config_data_write(struct file *filp,
		struct kobject *kobj, struct bin_attribute *bin_attr,
		char *buf, loff_t offset, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct cyttsp5_loader_data *data = cyttsp5_get_loader_data(dev);
	u8 *p;

	pt_debug(dev, DL_INFO, "%s: offset:%lld count:%zu\n",
		__func__, offset, count);

	mutex_lock(&data->config_lock);

	if (!data->config_loading) {
		mutex_unlock(&data->config_lock);
		return -ENODEV;
	}

	p = krealloc(data->config_data, offset + count, GFP_KERNEL);
	if (!p) {
		kfree(data->config_data);
		data->config_data = NULL;
		mutex_unlock(&data->config_lock);
		return -ENOMEM;
	}
	data->config_data = p;

	memcpy(&data->config_data[offset], buf, count);
	data->config_size += count;

	mutex_unlock(&data->config_lock);

	return count;
}

static struct bin_attribute bin_attr_config_data = {
	.attr = {
		.name = "config_data",
		.mode = S_IWUSR,
	},
	.size = 0,
	.write = cyttsp5_config_data_write,
};

static ssize_t cyttsp5_config_loading_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cyttsp5_loader_data *ld = cyttsp5_get_loader_data(dev);
	bool config_loading;

	mutex_lock(&ld->config_lock);
	config_loading = ld->config_loading;
	mutex_unlock(&ld->config_lock);

	return sprintf(buf, "%d\n", config_loading);
}

static int cyttsp5_verify_ttconfig_binary(struct device *dev,
		u8 *bin_config_data, int bin_config_size, u8 **start, int *len)
{
	struct cyttsp5_loader_data *ld = cyttsp5_get_loader_data(dev);
	int header_size;
	u16 config_size;
	u32 fw_ver_config;
	u32 fw_revctrl_config;

	if (!ld->si) {
		pt_debug(dev, DL_ERROR,
			"%s: No firmware info found, DUT FW may be corrupted\n",
			__func__);
		return -ENODEV;
	}

	/*
	 * We need 11 bytes for FW version control info and at
	 * least 6 bytes in config (Length + Max Length + CRC)
	 */
	header_size = bin_config_data[0] + 1;
	if (header_size < 11 || header_size >= bin_config_size - 6) {
		pt_debug(dev, DL_ERROR,
			"%s: Invalid header size %d\n", __func__,
			header_size);
		return -EINVAL;
	}

	fw_ver_config = get_unaligned_be16(&bin_config_data[1]);
	/* 4 middle bytes are not used */
	fw_revctrl_config = get_unaligned_be32(&bin_config_data[7]);

	/* FW versions should match */
	if (cyttsp5_check_firmware_version(dev, fw_ver_config,
			fw_revctrl_config)) {
		pt_debug(dev, DL_ERROR,
			"%s: FW versions mismatch\n", __func__);
		return -EINVAL;
	}

	config_size = get_unaligned_le16(&bin_config_data[header_size]);
	/* Perform a simple size check (2 bytes for CRC) */
	if (config_size != bin_config_size - header_size - 2) {
		pt_debug(dev, DL_ERROR,
			"%s: Config size invalid\n", __func__);
		return -EINVAL;
	}

	*start = &bin_config_data[header_size];
	*len = bin_config_size - header_size;

	return 0;
}

/*
 * 1: Start loading TT Config
 * 0: End loading TT Config and perform upgrade
 *-1: Exit loading
 */
static ssize_t cyttsp5_config_loading_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct cyttsp5_loader_data *ld = cyttsp5_get_loader_data(dev);
	long value;
	u8 *start;
	int length;
	int rc;

	rc = kstrtol(buf, 10, &value);
	if (rc < 0 || value < -1 || value > 1) {
		pt_debug(dev, DL_ERROR, "%s: Invalid value\n", __func__);
		return size;
	}

	mutex_lock(&ld->config_lock);

	if (value == 1)
		ld->config_loading = true;
	else if (value == -1)
		ld->config_loading = false;
	else if (value == 0 && ld->config_loading) {
		ld->config_loading = false;
		if (ld->config_size == 0) {
			pt_debug(dev, DL_ERROR,
				"%s: No config data\n", __func__);
			goto exit_free;
		}

		rc = cyttsp5_verify_ttconfig_binary(dev,
				ld->config_data, ld->config_size,
				&start, &length);
		if (rc)
			goto exit_free;

		rc = cyttsp5_upgrade_ttconfig(dev, start, length);
	}

exit_free:
	kfree(ld->config_data);
	ld->config_data = NULL;
	ld->config_size = 0;

	mutex_unlock(&ld->config_lock);

	if (rc)
		return rc;

	return size;
}

static DEVICE_ATTR(config_loading, S_IRUSR | S_IWUSR,
	cyttsp5_config_loading_show, cyttsp5_config_loading_store);
#endif /* CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_MANUAL_TTCONFIG_UPGRADE */

static void cyttsp5_fw_and_config_upgrade(
		struct work_struct *fw_and_config_upgrade)
{
	struct cyttsp5_loader_data *ld = container_of(fw_and_config_upgrade,
			struct cyttsp5_loader_data, fw_and_config_upgrade);
	struct device *dev = ld->dev;

	ld->si = cmd->request_sysinfo(dev);
	if (!ld->si)
		pt_debug(dev, DL_ERROR,
			"%s: Fail get sysinfo pointer from core\n",
			__func__);
#if !CYTTSP5_FW_UPGRADE
	pt_debug(dev, DL_INFO,
		"%s: No FW upgrade method selected!\n", __func__);
#endif

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_PLATFORM_FW_UPGRADE
	if (!upgrade_firmware_from_platform(dev, false))
		return;
#endif

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_BINARY_FW_UPGRADE
#if defined(PT_TDDI_SUPPORT) || defined(PT_TT_DISCRETE_SUPPORT)
	if (!pt_pip2_upgrade_firmware_from_builtin(dev))
		return;
#else
	if (!upgrade_firmware_from_builtin(dev))
		return;
#endif
#endif

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_PLATFORM_TTCONFIG_UPGRADE
	if (!upgrade_ttconfig_from_platform(dev))
		return;
#endif
}

#if CYTTSP5_FW_UPGRADE
static int cyttsp5_fw_upgrade_cb(struct device *dev)
{
#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_PLATFORM_FW_UPGRADE
	pt_debug(dev, DL_WARN, "%s: Upgrade Platform FW", __func__);
	if (!upgrade_firmware_from_platform(dev, false))
		return 1;
#endif

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_BINARY_FW_UPGRADE
	pt_debug(dev, DL_WARN, "%s: Upgrade Builtin FW", __func__);
#if defined(PT_TDDI_SUPPORT) || defined(PT_TT_DISCRETE_SUPPORT)
	if (!pt_pip2_upgrade_firmware_from_builtin(dev))
		return 1;
#else
	if (!upgrade_firmware_from_builtin(dev))
		return 1;
#endif
#endif
	return 0;
}
#endif

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_PLATFORM_FW_UPGRADE
static ssize_t cyttsp5_forced_upgrade_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int rc = upgrade_firmware_from_platform(dev, true);

	if (rc)
		return rc;
	return size;
}

static DEVICE_ATTR(forced_upgrade, S_IWUSR,
	NULL, cyttsp5_forced_upgrade_store);
#endif

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_BINARY_FW_UPGRADE
static ssize_t cyttsp5_manual_upgrade_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct cyttsp5_loader_data *ld = cyttsp5_get_loader_data(dev);
	int rc;

	if (ld->is_manual_upgrade_enabled)
		return -EBUSY;

	ld->is_manual_upgrade_enabled = 1;

	rc = upgrade_firmware_from_class(ld->dev);

	if (rc < 0)
		ld->is_manual_upgrade_enabled = 0;

	return size;
}

static DEVICE_ATTR(manual_upgrade, S_IWUSR,
	NULL, cyttsp5_manual_upgrade_store);
#endif


/*******************************************************************************
 * FUNCTION: pt_pip2_set_boot_mode_pin
 *
 * SUMMARY: Set the state of the host_mode gpio
 *
 * PARAMETERS:
 *	*dev   - pointer to device structure
 *	 value - <1|0> set value for GPIO
 ******************************************************************************/
static void pip2_set_boot_mode_pin(struct device *dev, int value)
{
	struct cyttsp5_platform_data *pdata = dev_get_platdata(dev);
	int boot_mode_gpio = pdata->core_pdata->boot_mode_gpio;
	int rc;


	gpio_free(boot_mode_gpio);
	rc = gpio_request(boot_mode_gpio, NULL);
	if (rc) {
		pt_debug(dev, DL_ERROR,
			"error requesting boot_mode_pin gpio %d\n",
			boot_mode_gpio);
		goto exit;
	} else {
		if (!value)
			gpio_direction_output(boot_mode_gpio, 0);
		else
			gpio_direction_input(boot_mode_gpio);
	}
	pt_debug(dev, DL_WARN, "Set Boot Mode GPIO %d to: %d\n",
		boot_mode_gpio, value);

exit:
	return;
}

/*******************************************************************************
 * FUNCTION: _pt_pip2_get_flash_info
 *
 * SUMMARY: Sends a FLASH_INFO command to the DUT logging the results to kmsg
 *
 * PARAMETERS:
 *	*dev - pointer to device structure
 *	*pip2_cmd - pointer to the PIP2 command structure
 *	*read_buf - pointer to the read buffer array to store the response
 *
 ******************************************************************************/
static void _pt_pip2_get_flash_info(struct device *dev,
	struct pip2_cmd_structure *pip2_cmd, u8 *read_buf)
{
	u16 actual_read_len;
	int ret;

	/* Get flash info for debugging information */
	ret = cmd->nonhid_cmd->pip2_send_cmd(dev, pip2_cmd,
		PIP2_CMD_ID_FLASH_INFO, NULL, 0, read_buf, &actual_read_len);
	if (!ret) {
		pt_debug(dev, DL_DEBUG,
			"%s --- FLASH Information ---\n", __func__);
		pt_debug(dev, DL_DEBUG,
			"%s Manufacturer ID: 0x%02x\n",
			__func__, read_buf[1]);
		pt_debug(dev, DL_DEBUG,
			"%s Memory Type    : 0x%02x\n",
			__func__, read_buf[2]);
		pt_debug(dev, DL_DEBUG,
			"%s Num Sectors    : 0x%02x%02x%02x%02x\n",
			__func__, read_buf[3], read_buf[4],
			read_buf[5], read_buf[6]);
		pt_debug(dev, DL_DEBUG,
			"%s Sectors Size   : 0x%02x%02x%02x%02x\n",
			__func__, read_buf[7], read_buf[8],
			read_buf[9], read_buf[10]);
		pt_debug(dev, DL_DEBUG,
			"%s Page Size      : 0x%02x%02x%02x%02x\n",
			__func__, read_buf[11], read_buf[12],
			read_buf[13], read_buf[14]);
	}
}

/*******************************************************************************
 * FUNCTION: _pt_pip2_log_last_error
 *
 * SUMMARY: Sends a STATUS command to the DUT logging the results until all
 *	errors are cleared.
 *
 * PARAMETERS:
 *	*dev - pointer to device structure
 *	*pip2_cmd - pointer to the PIP2 command structure
 *	*read_buf - pointer to the read buffer array to store the response
 *
 ******************************************************************************/
static void _pt_pip2_log_last_error(struct device *dev,
	struct pip2_cmd_structure *pip2_cmd, u8 *read_buf)
{
	u16 actual_read_len;
	u8 loop = 5;
	u8 error = 0xFF;
	int ret;

	/*
	 * Send the STATUS command until no errors are found.
	 * The BL will store an error code for each layer of the stack,
	 * and each read will return one error.
	 */
	while (loop > 0 && error) {
		ret = cmd->nonhid_cmd->pip2_send_cmd(dev, pip2_cmd,
			PIP2_CMD_ID_STATUS, NULL, 0, read_buf,
			&actual_read_len);

		if (!ret) {
			error  = (u8)read_buf[PIP2_RESPONSE_BODY_OFFSET];

			pt_debug(dev, DL_ERROR,
				"%s: STATUS=0x%02x BOOT=%d BUSY=%d INT=%d ERR_PHY=%d ERR_REG=%d BL_Status code=0x%02x",
				__func__,
				(u8)read_buf[PIP2_RESPONSE_STATUS_OFFSET],
				error & 0x01,
				(error & 0x02) >> 1,
				(error & 0x04) >> 2,
				(error & 0x18) >> 3,
				(error & 0xE0) >> 5,
				(u8)read_buf[PIP2_RESPONSE_BODY_OFFSET + 1]);
		}
		loop--;
	}
}


/*******************************************************************************
 * FUNCTION: _pt_pip2_firmware_cont
 *
 * SUMMARY: Bootload the DUT with a FW image using the PIP2 protocol. This
 *	includes getting the DUT into BL mode, writing the file to either SRAM
 *	or FLASH, and launching the application directly in SRAM or by resetting
 *	the DUT without the hostmode pin asserted.
 *
 *	NOTE: Special care must be taken to support a DUT communicating in
 *		PIP2.0 where the length field is defined differently.
 *
 * PARAMETERS:
 *	*fw      - pointer to the new FW image to load
 *	*context - pointer to the device
 ******************************************************************************/
static void _pt_pip2_firmware_cont(const struct firmware *fw,
		void *context)
{
	struct device *dev = context;
	struct pip2_cmd_structure pip2_cmd;
	u16 actual_read_len;
	int upgrade;
	u8 file_handle;
	u8 read_buf[256];
	u16 status;
	u8 *fw_img = 0;
	int fw_size = 0;
	u8 buf[256];
	u8 len_per_packet = 100;
	u8 write_len;
	int remain_bytes;
	int ret = 0;
	u8 data[20];
	u8 pip_version_major;
	u8 pip_version_minor;
	u8 app_version_lsb;
	u8 app_version_msb;
	u8 img_app_version_lsb;
	u8 img_app_version_msb;
	u8 mode;
	u8 retry_packet = 0;
	/* bool upgrade = true; */
	struct cyttsp5_loader_data *ld = cyttsp5_get_loader_data(dev);
	struct pip2_loader_data *pip2_data = ld->pip2_data;
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);

	pt_debug(dev, DL_DEBUG, "Entering function %s...\n", __func__);

	/* Loading status start at 1% complete */
	ld->pip2_bl_status = PIP2_BL_STATUS_ACTIVE_1;
	pt_debug(dev, DL_WARN, "%s: Begin BL\n", __func__);

	if (!fw) {
		if (ld->pip2_load_builtin) {
			pt_debug(dev, DL_ERROR,
				"%s: No builtin firmware\n", __func__);
			ld->builtin_bin_fw_status = -EINVAL;
			complete(&ld->builtin_bin_fw_complete);
		} else {
			pt_debug(dev, DL_ERROR,
				"%s: No firmware provided to load\n", __func__);
		}
		return;
	}

	if (!fw->data || !fw->size) {
		pt_debug(dev, DL_ERROR,
			"%s: Invalid builtin firmware\n", __func__);
		goto cyttsp5_firmware_cont_release_exit;
	}
	pt_debug(dev, DL_WARN,
		"%s: Found firmware of size:%d bytes\n",
		__func__, fw->size);

	/* Stop watchdog */
	ret = cmd->request_stop_wd(dev);
	if (ret)
		pt_debug(dev, DL_ERROR, "%s stop watchdog failed",
			__func__);

	/* Suspend Scanning */
	pt_debug(dev, DL_WARN, "%s Suspend Scanning\n", __func__);
	ret = cmd->nonhid_cmd->suspend_scanning(dev, 1);
	if (ret)
		pt_debug(dev, DL_ERROR, "%s Suspend Scan Failed\n",
			__func__);

	/* Wait for completion of FW upgrade thread before continuing */
	if (!ld->pip2_load_builtin)
		init_completion(&pip2_data->pip2_fw_upgrade_complete);

	/* Determine if in Application or BL mode */
	ret = cmd->request_pip2_get_mode(dev, &mode, false);
	pt_debug(dev, DL_WARN, "%s: Get Mode = %d", __func__, mode);
	if (mode == CY_MODE_OPERATIONAL) {
		/* Hold Boot Mode pin low to force BL to not launch FW */
		pt_debug(dev, DL_WARN,
			"%s Enabling host mode pin\n", __func__);
		pip2_set_boot_mode_pin(dev, 0);

		/* Reset device to enter the BL */
		if (!ld->pip2_load_fw_to_ram) {
			/*
			 * PATCH - Use "Enter BL" command to force slow clock
			 * when writing to FLASH, using an XRES only is not
			 * enough
			 */
			pt_debug(dev, DL_WARN,
				"%s Send start_bl cmd to slow clock\n",
				__func__);
			/* PIP cmd start_bl has no response - don't check ret */
			ret = cmd->nonhid_cmd->start_bl(dev, 1);
		} else {
			/* Reset the DUT by function or soft reset */
			if (cd->cpdata->xres)
				cd->cpdata->xres(cd->cpdata, cd->dev);
			else {
				/* If no XRES GPIO configured try soft reset */
				pt_debug(dev, DL_WARN,
					"%s sending soft reset cmd now...\n",
					__func__);
				cmd->request_stop_wd(dev);
				ret = cmd->nonhid_cmd->pip2_send_cmd(dev,
					&pip2_cmd, PIP2_CMD_ID_RESET, NULL, 0,
					read_buf, &actual_read_len);
			}
		}
	}
	ld->pip2_bl_status += 1;

	/* Sleep to alow the BL to come up, update BL % complete */
	msleep(PIP2_LAUNCH_BL_DELAY);
	ld->pip2_bl_status += 1;

	/* Release boot mode pin, this pin could be shared with the INT line */
	pt_debug(dev, DL_INFO, "%s Releasing host mode pin\n", __func__);
	pip2_set_boot_mode_pin(dev, 1);

	/* Ensure device is now in BL mode */
	ret = cmd->request_pip2_get_mode(dev, &mode, false);
	pt_debug(dev, DL_WARN, "%s: Get Mode = %d", __func__, mode);
	if (!ret && mode == CY_MODE_BOOTLOADER) {
		/* Clear startup status bitmask */
		cd->startup_status = STARTUP_STATUS_START;

		/* Get BL PIP version */
		cmd->request_active_pip_prot(dev,
			&(cd->bl_pip_version_major),
			&(cd->bl_pip_version_minor));
	} else {
		pt_debug(dev, DL_ERROR,
			"%s ERROR: Not in BL as expected", __func__);
		pt_debug(dev, DL_ERROR,
			"%s Enter BL error PIP ver = 0x%02x, Abort\n",
			__func__, pip_version_major);
		ld->pip2_bl_status = PIP2_BL_STATUS_PIP_VERSION_ERROR;
		goto exit;
	}
	pt_debug(dev, DL_WARN, "%s In bootloader mode now\n", __func__);

	/* Only compare FW versions when doing a built-in BL */
	if (ld->pip2_load_builtin) {
		ret = cmd->nonhid_cmd->pip2_send_cmd(dev, &pip2_cmd,
			PIP2_CMD_ID_VERSION, NULL, 0, read_buf,
			&actual_read_len);

		app_version_lsb = read_buf[PIP2_RESPONSE_BODY_OFFSET + 4];
		app_version_msb = read_buf[PIP2_RESPONSE_BODY_OFFSET + 5];
		img_app_version_lsb = fw->data[3];
		img_app_version_msb = fw->data[2];

		pt_debug(dev, DL_INFO,
			"%s Current FW Version: 0x%02x.0x%02x\n",
			__func__, app_version_msb, app_version_lsb);
		pt_debug(dev, DL_INFO,
			"%s BL Image Version:   0x%02x.0x%02x\n",
			__func__, app_version_msb, app_version_lsb);

		if ((256 * img_app_version_msb + img_app_version_lsb) <=
		    (256 * app_version_msb + app_version_lsb)) {
			pt_debug(dev, DL_WARN,
				"Image FW version lower, will not upgrade FW");
			ld->pip2_bl_status = PIP2_BL_STATUS_FW_VERSION_ERROR;
			goto exit;
		} else
			upgrade = 1;
	} else
		/* Always do a manual upgrade */
		upgrade = 1;

	/* Open correct file */
	if (ld->pip2_load_fw_to_ram) {
		data[0] = 0;	/* File 0 - SRAM loader file */
		pt_debug(dev, DL_INFO,
			"%s Send OPEN File for RAM write\n", __func__);
	} else {
		data[0] = 1;	/* File 1 - FLASH Boot image file */
		pt_debug(dev, DL_INFO,
			"%s Send OPEN File for FLASH write\n", __func__);
	}

	ret = cmd->nonhid_cmd->pip2_send_cmd(dev, &pip2_cmd,
		PIP2_CMD_ID_FILE_OPEN, data, 1, read_buf, &actual_read_len);
	status      = read_buf[PIP2_RESPONSE_STATUS_OFFSET];
	file_handle = read_buf[PIP2_RESPONSE_BODY_OFFSET];
	if (ret || ((status != 0x00) && (status != 0x03)) || file_handle > 1) {
		pt_debug(dev, DL_ERROR,
			"%s File open failure with file = %d\n",
			__func__, file_handle);
		ret = -1;
		goto exit;
	}
	pip2_data->pip2_file_handle = file_handle;
	pt_debug(dev, DL_INFO,
		"%s File handle = 0x%02x\n", __func__,
		pip2_data->pip2_file_handle);

	/* FLASH program only */
	if (!ld->pip2_load_fw_to_ram) {
		/* Log the Flash part info if debugging enabled */
		if (cd->debug_level >= DL_DEBUG)
			_pt_pip2_get_flash_info(dev, &pip2_cmd, read_buf);

		/* File ioctl, erase file before loading */
		pt_debug(dev, DL_WARN,
			"%s Send ERASE file command...\n", __func__);
		data[0] = file_handle;
		data[1] = PIP2_FILE_IOCTL_CODE_ERASE_FILE;
		ret = cmd->nonhid_cmd->pip2_send_cmd(dev, &pip2_cmd,
			PIP2_CMD_ID_FILE_IOCTL, data, 2, read_buf,
			&actual_read_len);
		status = read_buf[PIP2_RESPONSE_STATUS_OFFSET];
		if (status) {
			pt_debug(dev, DL_ERROR,
				"%s error: file erase failure\n", __func__);
			ret = -1;
			ld->pip2_bl_status = PIP2_BL_STATUS_ERASE_ERROR;
			goto exit;
		}
		pt_debug(dev, DL_INFO,
			"%s File erase successful\n", __func__);
	}

	if (upgrade) {
		u8 header_size = 0;

		if (!fw)
			release_firmware(fw);
		if (!fw->data || !fw->size) {
			pt_debug(dev, DL_ERROR,
				"%s: No firmware received\n", __func__);
		}
		header_size = fw->data[0];
		if (header_size >= (fw->size + 1)) {
			pt_debug(dev, DL_ERROR,
				"%s: Firmware format is invalid\n", __func__);
		}
		fw_img = (u8 *)&(fw->data[0]);
		fw_size = fw->size;
		remain_bytes = fw_size;
		buf[0] = file_handle;
		pt_debug(dev, DL_WARN,
			"%s: Writing %d bytes of firmware data now\n",
			__func__, fw_size);

		/*
		 * Continue writing while data remains.
		 * PATCH - If the last packet is exactly 58 bytes, break it up
		 *	TC3300 issue with 64byte packet.
		 */
		while (remain_bytes > len_per_packet ||
			remain_bytes == 58) {

			/*
			 * PATCH - 58 bytes will produce a 64 byte write which
			 * will fail with BL 1.0 so break it up.
			 */
			if (remain_bytes == 58)
				write_len = 32;
			else
				write_len = len_per_packet;

			/* Don't print debug msg on every pass */
			if (remain_bytes % 10000 < 100) {
				/* Calculate % complete for Pt_Tuner */
				ld->pip2_bl_status =
				(fw_size - remain_bytes) * 100 / fw_size;

				pt_debug(dev, DL_WARN,
					"Wrote %d of %d bytes to File 0x%02x\n",
					fw_size - remain_bytes - write_len,
					fw_size, file_handle);
			}
			if (retry_packet > 0) {
				cd->bl_retry_packet_count++;
				pt_debug(dev, DL_WARN,
					"%s: === Retry Packet #%d ===\n",
					__func__, retry_packet);
				/* Get and log the last error(s) */
				_pt_pip2_log_last_error(dev, &pip2_cmd,
					read_buf);
			}

			/* Send the file write cmd as a PIP2 command */
			pip2_data->pip2_file_handle = file_handle;
			memcpy(&buf[1], fw_img, write_len);
			ret = cmd->nonhid_cmd->pip2_send_cmd(dev, &pip2_cmd,
				PIP2_CMD_ID_FILE_WRITE, buf, write_len + 1,
				read_buf, &actual_read_len);
			status = read_buf[PIP2_RESPONSE_STATUS_OFFSET];

			/* Write cmd successful with a fail status */
			if (!ret && status) {
				/*
				 * The last time through the loop when
				 * remain_bytes = write_len, no partial
				 * payload will remain, the last successful
				 * write (when writing to RAM) will respond with
				 * EOF status, writing to FLASH will respond
				 * with a standard success status of 0x00
				 */
				if (ld->pip2_load_fw_to_ram &&
				    status == PIP2_RSP_ERR_END_OF_FILE &&
				    remain_bytes == write_len) {
					pt_debug(dev, DL_WARN,
						"%s Last write, ret = 0x%02x\n",
						__func__, status);
					/* Drop out of the while loop */
					break;
				}
				ld->pip2_bl_status = PIP2_BL_STATUS_WRITE_ERROR;
				pt_debug(dev, DL_ERROR,
					"%s file write failure, status = 0x%02x\n",
					__func__, status);
				if (retry_packet >= 3) {
					/* Tripple retry error - break */
					remain_bytes = 0;
					pt_debug(dev, DL_ERROR,
						"%s %d - Packet retry status error - Break\n",
						__func__, status);
				} else {
					retry_packet++;
				}
			} else if (ret) {
				/* Packet write failed - retry 3x */
				if (retry_packet >= 3) {
					remain_bytes = 0;
					pt_debug(dev, DL_ERROR,
						"%s %d - Packet Retry cmd error - Break\n",
						__func__, ret);
				}
				retry_packet++;
			} else
				/* Cmd succes and status success */
				retry_packet = 0;

			if (retry_packet == 0) {
				fw_img += write_len;
				remain_bytes -= write_len;
			}
		}
		/* Write the remaining bytes if any remain */
		if (remain_bytes > 0 && retry_packet == 0) {
			pt_debug(dev, DL_WARN,
				"%s: Write last %d bytes to File = 0x%02x\n",
				__func__, remain_bytes, file_handle);
			memcpy(&buf[1], fw_img, remain_bytes);
			ret = cmd->nonhid_cmd->pip2_send_cmd(dev, &pip2_cmd,
				PIP2_CMD_ID_FILE_WRITE, buf, remain_bytes + 1,
				read_buf, &actual_read_len);
			status = read_buf[PIP2_RESPONSE_STATUS_OFFSET];
			if (!ret || status) {
				/*
				 * Any non zero status when writing to FLASH is
				 * an error. Only when writing to RAM, the last
				 * packet must respond with an EOF status
				 */
				if (!ld->pip2_load_fw_to_ram ||
				     (ld->pip2_load_fw_to_ram &&
				     status != PIP2_RSP_ERR_END_OF_FILE)) {
					ld->pip2_bl_status =
						PIP2_BL_STATUS_WRITE_ERROR;
					pt_debug(dev, DL_ERROR,
						"%s write failure, status = 0x%02x\n",
						__func__, status);
				}
			}
		}
	}

	/* Close the file */
	pt_debug(dev, DL_WARN, "%s closing file_handle = 0x%02x",
		__func__, file_handle);
	data[0] = file_handle;
	ret = cmd->nonhid_cmd->pip2_send_cmd(dev, &pip2_cmd,
		PIP2_CMD_ID_FILE_CLOSE, data, 1, read_buf, &actual_read_len);
	status = read_buf[PIP2_RESPONSE_STATUS_OFFSET];
	if (ret || status) {
		pt_debug(dev, DL_ERROR,
			"%s file close failure\n", __func__);
		ld->pip2_bl_status = PIP2_BL_STATUS_FILE_CLOSE_ERROR;
		goto exit;
	}

	if (retry_packet >= 3) {
		/* A packet write failure occured 3x */
		pt_debug(dev, DL_ERROR,
			"%s: BIN file write terminated due to consecutive write errors\n",
			__func__);
		ld->pip2_bl_status = PIP2_BL_STATUS_WRITE_ERROR;
		goto exit;
	} else
		pt_debug(dev, DL_INFO,
			"%s: BIN file write finished successfully\n", __func__);

	ld->pip2_bl_status = PIP2_BL_STATUS_ACTIVE_99;
	if (ld->pip2_load_fw_to_ram) {
		/* When writing to RAM don't reset, just launch application */
		pt_debug(dev, DL_WARN,
			"%s Sending execute command now...\n", __func__);
		ret = cmd->nonhid_cmd->pip2_send_cmd(dev, &pip2_cmd,
			PIP2_CMD_ID_EXECUTE, NULL, 0, read_buf,
			&actual_read_len);
		status = read_buf[PIP2_RESPONSE_STATUS_OFFSET];
		if (ret || status) {
			pt_debug(dev, DL_ERROR,
				"%s Execute command failure\n", __func__);
			ld->pip2_bl_status = PIP2_BL_STATUS_EXECUTE_ERROR;
			goto exit;
		}
	} else {
		/* Do a reset if writing to FLASH */
		pt_debug(dev, DL_WARN,
			"%s Toggle TP_XRES now...\n", __func__);
		cmd->request_reset(dev);
	}

	/*
	 * Sleep to allow device to launch application. If this delay is too
	 * long, incoming touch reports could be read like a PIP2 report,
	 * reading the 2 bytes beyond the actual report length.
	 */
	msleep(PIP2_LAUNCH_APP_DELAY);

	/* After the reset or execute, determine if in Application mode */
	ret = cmd->request_pip2_get_mode(dev, &mode, false);
	pt_debug(dev, DL_WARN,
		"%s: Expecting App mode (2), Got Mode = %d", __func__, mode);
	if (mode != CY_MODE_OPERATIONAL) {
		cmd->request_active_pip_prot(dev,
			&pip_version_major, &pip_version_minor);

		pt_debug(dev, DL_ERROR,
			"%s ERROR: Not in App mode as expected, PIP%d.%d detected\n",
			__func__, pip_version_major, pip_version_minor);

		ld->pip2_bl_status = PIP2_BL_STATUS_MODE_ERROR;
		goto exit;
	}

	pt_debug(dev, DL_INFO,
		"%s === PIP2 FW upgrade finished ===\n", __func__);
	if (ld->pip2_load_builtin) {
		ld->builtin_bin_fw_status = 0;
		complete(&ld->builtin_bin_fw_complete);
	}

	/* Either RESET or LAUNCH APP has occurred so definately not in BL */
	ld->pip2_bl_status = PIP2_BL_STATUS_COMPLETE;

exit:
	/* Ensure the boot mode pin is released */
	pt_debug(dev, DL_WARN,
		"%s Disabling host mode pin\n", __func__);
	pip2_set_boot_mode_pin(dev, 1);
	/*cmd->release_exclusive(dev); ZZZ*/

cyttsp5_firmware_cont_release_exit:
	release_firmware(fw);

	if (ld->pip2_load_builtin)
		complete(&ld->builtin_bin_fw_complete);

	/* Issue a restart to get HID descriptor etc. */
	pt_debug(dev, DL_WARN, "%s Requesting RESTART\n", __func__);
	cmd->request_restart(dev, true);

	/* Start watchdog */
	pt_debug(dev, DL_WARN, "%s Starting watchdog\n", __func__);
	ret = cmd->request_start_wd(dev);
}

#if defined(PT_TDDI_SUPPORT) || defined(PT_TT_DISCRETE_SUPPORT)
/*******************************************************************************
 * FUNCTION: pt_pip2_upgrade_firmware_from_builtin
 *
 * SUMMARY: Bootload the DUT with a built in firmware binary image.
 *	Load either a SRAM image "cyttsp5_fw_RAM.bin" or a FLASH image
 *	"cyttsp5_fw.bin" with the priority being the SRAM image.
 *
 *	*dev - pointer to the device structure
 ******************************************************************************/
static int pt_pip2_upgrade_firmware_from_builtin(struct device *dev)
{
	struct cyttsp5_loader_data *ld = cyttsp5_get_loader_data(dev);
	int retval;

	pt_debug(dev, DL_INFO,
		"%s: Enabling firmware class loader built-in\n", __func__);
	ld->pip2_load_builtin = true;

	/* Try to load the RAM image first if it exists */
	retval = request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
			"cyttsp5_fw_RAM.bin", dev, GFP_KERNEL, dev,
			_pt_pip2_firmware_cont);
	if (retval < 0) {
		ld->pip2_load_fw_to_ram = false;
		retval = request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
				"cyttsp5_fw.bin", dev, GFP_KERNEL, dev,
				_pt_pip2_firmware_cont);
		if (retval < 0) {
			pt_debug(dev, DL_ERROR,
				"%s: Fail request firmware class file load\n",
				__func__);
			goto exit;
		}
	} else
		ld->pip2_load_fw_to_ram = true;

	/* wait until FW binary upgrade finishes */
	wait_for_completion(&ld->builtin_bin_fw_complete);
	retval = ld->builtin_bin_fw_status;
exit:
	return retval;
}
#endif

/*******************************************************************************
 * FUNCTION: pt_pip2_do_fw_upgrade
 *
 * SUMMARY: Create the firmware class but don't actually laod any FW to the
 *	DUT. This creats all the sysfs nodes needed for a user to bootload
 *	the DUT with their own bin file.
 *
 *	*pip2_data     - pointer to the PIP2 loader data structure
 *	 force_upgrade - Force the upgrade even if version in DUT is less
 ******************************************************************************/
static int pt_pip2_do_fw_upgrade(struct pip2_loader_data *pip2_data,
	bool force_upgrade)
{

	int ret = 0;
	struct device *dev = pip2_data->dev;
	struct cyttsp5_loader_data *ld = cyttsp5_get_loader_data(dev);

	/*
	 * The file name "pip2_firmware_upgrade" is an invalid bin file name
	 * used intentionally because request_firmware_nowait will not find the
	 * file which is what we want and then simply create the fw class nodes.
	 */
	ld->pip2_load_builtin = false;
	pt_debug(dev, DL_INFO, "%s: Request FW Class", __func__);
	ret = request_firmware_nowait(THIS_MODULE, FW_ACTION_NOHOTPLUG,
			"pip2_firmware_upgrade", dev, GFP_KERNEL, dev,
			_pt_pip2_firmware_cont);
	if (ret < 0) {
		pt_debug(dev, DL_ERROR,
			"%s: ERROR requesting firmware class\n", __func__);
		goto exit;
	}
exit:
	return 0;
}

/*******************************************************************************
 * FUNCTION: pt_pip2_manual_upgrade_store
 *
 * SUMMARY: Store method for the pip2_manual_upgrade sysfs node. Allows
 *	sysfs control of bootloading a new FW image to FLASH.
 *
 * PARAMETERS:
 *      *dev   - pointer to device structure
 *      *attr  - pointer to device attributes
 *      *buf   - pointer to output buffer
 *	size   - size of data in buffer
 ******************************************************************************/
static ssize_t pt_pip2_manual_upgrade_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct cyttsp5_loader_data *ld = cyttsp5_get_loader_data(dev);
	int rc;

	if (ld->is_manual_upgrade_enabled) {
		pt_debug(dev, DL_ERROR,
			"%s: ERROR - Manual upgrade busy\n", __func__);
		return -EBUSY;
	}
	ld->pip2_load_fw_to_ram = false;
	ld->is_manual_upgrade_enabled = 1;
	rc = pt_pip2_do_fw_upgrade(ld->pip2_data, true);
	ld->is_manual_upgrade_enabled = 0;
	if (rc < 0)
		pt_debug(dev, DL_ERROR,
			"%s: ERROR - FLASH Upgrade failed\n", __func__);
	return size;
}
static DEVICE_ATTR(pip2_manual_upgrade, S_IWUSR,
	NULL, pt_pip2_manual_upgrade_store);


/*******************************************************************************
 * FUNCTION: pt_pip2_manual_ram_upgrade_store
 *
 * SUMMARY: Store method for the pip2_manual_ram_upgrade sysfs node. Allows
 *	sysfs control of bootloading a new FW image to SRAM.
 *
 * PARAMETERS:
 *      *dev   - pointer to device structure
 *      *attr  - pointer to device attributes
 *      *buf   - pointer to output buffer
 *	size   - size of data in buffer
 ******************************************************************************/
static ssize_t pt_pip2_manual_ram_upgrade_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct cyttsp5_loader_data *ld = cyttsp5_get_loader_data(dev);
	int rc;

	if (ld->is_manual_upgrade_enabled) {
		pt_debug(dev, DL_ERROR,
			"%s: ERROR - Manual upgrade busy\n", __func__);
		return -EBUSY;
	}
	ld->pip2_load_fw_to_ram = true;
	ld->is_manual_upgrade_enabled = 1;
	rc = pt_pip2_do_fw_upgrade(ld->pip2_data, true);
	ld->is_manual_upgrade_enabled = 0;
	if (rc < 0)
		pt_debug(dev, DL_ERROR,
			"%s: ERROR - RAM Upgrade failed\n", __func__);
	return size;
}
static DEVICE_ATTR(pip2_manual_ram_upgrade, S_IWUSR,
	NULL, pt_pip2_manual_ram_upgrade_store);


/*******************************************************************************
 * FUNCTION: pt_pip2_boot_mode_pin_store
 *
 * SUMMARY: Store method for the pip2_boot_mode_pin sysfs node. Allows direct
 *	control of the GPIO state.
 *
 * PARAMETERS:
 *      *dev   - pointer to device structure
 *      *attr  - pointer to device attributes
 *      *buf   - pointer to output buffer
 *	size   - size of data in buffer
 ******************************************************************************/
static ssize_t pt_pip2_boot_mode_pin_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct cyttsp5_platform_data *pdata = dev_get_platdata(dev);
	int boot_mode_gpio = pdata->core_pdata->boot_mode_gpio;
	unsigned long value;
	int rc;

	rc = kstrtoul(buf, 10, &value);
	if (rc < 0) {
		pt_debug(dev, DL_ERROR,
			"%s: Invalid value\n", __func__);
		goto exit;
	}

	switch (value) {
	case 0: /* Set output value to 0 */
		pt_debug(dev, DL_INFO,
			"Host_Mode_Pin: Low\n");
		pip2_set_boot_mode_pin(dev, 0);
		break;
	case 1: /* External pull up to 2.5V is needed */
		pt_debug(dev, DL_INFO,
			"Host_Mode_Pin: input mode, float high\n");
		pip2_set_boot_mode_pin(dev, 1);
		break;
	case 11:
		pt_debug(dev, DL_WARN,
			"Host_Mode_Pin: Drive High\n");
		gpio_free(boot_mode_gpio);
		rc = gpio_request(boot_mode_gpio, NULL);
		if (rc)
			pt_debug(dev, DL_ERROR,
				"%s: Error: requesting boot mode gpio %d\n",
				__func__, boot_mode_gpio);
		gpio_direction_output(boot_mode_gpio, 1);
		break;
	default:
		pt_debug(dev, DL_ERROR, "error: Invalid value\n");
		break;
	}
exit:
	return size;
}

static DEVICE_ATTR(pip2_boot_mode_pin, S_IWUSR,
	NULL, pt_pip2_boot_mode_pin_store);

/*******************************************************************************
 * FUNCTION: pt_pip2_flash_erase_show
 *
 * SUMMARY: The show method for the "pip2_flash_erase" sysfs node.
 *	Prints current erase status to output buffer.
 *
 * PARAMETERS:
 *	*dev  - pointer to device structure
 *	*attr - pointer to device attributes structure
 *	*buf  - pointer to print output buffer
 ******************************************************************************/
static ssize_t pt_pip2_flash_erase_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cyttsp5_loader_data *ld = cyttsp5_get_loader_data(dev);

	return snprintf(buf, CY_MAX_PRBUF_SIZE,
		"ERASE Status: 0x%02x\n",
		ld->pip2_erase_status);
}

/*******************************************************************************
 * FUNCTION: pt_pip2_flash_erase_store
 *
 * SUMMARY: The store method for the "pip2_flash_erase" sysfs node. Allows the
 *	caller to provide the file number to erase in FLASH.
 *
 *	NOTE: The DUT must be in BL mode before calling this function.
 *
 * PARAMETERS:
 *	*dev  - pointer to device structure
 *	*attr - pointer to device attributes structure
 *	*buf  - pointer to print output buffer
 *	size  - size of buffer
 ******************************************************************************/
static ssize_t pt_pip2_flash_erase_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned long value;
	int rc;
	int ret;
	u8 file_handle;
	u16 actual_read_len;
	u8 read_buf[256];
	u8 data[20];
	u8 mode = CY_MODE_UNKNOWN;
	struct pip2_cmd_structure pip2_cmd;
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);
	struct cyttsp5_loader_data *ld = cyttsp5_get_loader_data(dev);

	rc = kstrtoul(buf, 10, &value);
	if (rc < 0) {
		/* Positive integers only */
		pt_debug(dev, DL_ERROR,
			"%s: Invalid value\n", __func__);
		goto exit;
	}

	/* Determine if in Application or BL mode */
	rc = cmd->request_pip2_get_mode(dev, &mode, false);
	pt_debug(dev, DL_WARN, "%s: Get Mode = %d", __func__, mode);
	if (mode == CY_MODE_OPERATIONAL) {
		/* Hold Boot Mode pin low to force BL to not launch FW */
		pip2_set_boot_mode_pin(dev, 0);

		/*
		 * PATCH - Use "Enter BL" command to force slow clock
		 * Using an XRES only is not enough
		 */
		rc = cmd->nonhid_cmd->start_bl(dev, 0);
		if (rc < 0)
			return rc;
	}

	/* PATCH - In BL mode now, get the active BL PIP protocol version */
	cmd->request_active_pip_prot(dev, &(cd->bl_pip_version_major),
		&(cd->bl_pip_version_minor));
	pt_debug(dev, DL_WARN, "%s: BL PIP Version %d.%d", __func__,
		cd->bl_pip_version_major, cd->bl_pip_version_minor);

	/* Sleep to allow BL to come up */
	msleep(PIP2_LAUNCH_BL_DELAY);

	/* Release Boot Mode pin */
	pip2_set_boot_mode_pin(dev, 1);

	/* Open the requested file */
	pt_debug(dev, DL_WARN, "%s: Preparing to erase file: %lu\n",
		__func__, value);

	/* Ensure the SRAM file is not being erased */
	if (value == 0) {
		ld->pip2_erase_status = PIP2_RSP_ERR_BAD_FILE;
		pt_debug(dev, DL_ERROR, "%s: ERROR - Invalid File\n",
			__func__);
		goto exit;
	}
	data[0] = value;
	ret = cmd->nonhid_cmd->pip2_send_cmd(dev, &pip2_cmd,
		PIP2_CMD_ID_FILE_OPEN, data, 1, read_buf, &actual_read_len);
	file_handle = read_buf[PIP2_RESPONSE_BODY_OFFSET];
	ld->pip2_erase_status = read_buf[PIP2_RESPONSE_STATUS_OFFSET];
	if (ld->pip2_erase_status) {
		pt_debug(dev, DL_ERROR,
			"%s ERROR: File open command failed, handle = %d status = %d\n",
			__func__, file_handle, ld->pip2_erase_status);
		goto exit;
	}

	/* File ioctl, erase file */
	pt_debug(dev, DL_INFO, "%s Erasing file...\n", __func__);
	data[0] = file_handle;
	data[1] = PIP2_FILE_IOCTL_CODE_ERASE_FILE;
	ret = cmd->nonhid_cmd->pip2_send_cmd(dev, &pip2_cmd,
		PIP2_CMD_ID_FILE_IOCTL, data, 2, read_buf, &actual_read_len);
	ld->pip2_erase_status = read_buf[PIP2_RESPONSE_STATUS_OFFSET];
	if (ld->pip2_erase_status) {
		pt_debug(dev, DL_ERROR,
			"%s ERROR: file erase failure\n", __func__);
		ret = -1;
		goto exit;
	}
	pt_debug(dev, DL_INFO,
		"%s File erase successful\n", __func__);
exit:
	return size;
}

static DEVICE_ATTR(pip2_flash_erase, S_IRUSR | S_IWUSR,
	pt_pip2_flash_erase_show, pt_pip2_flash_erase_store);

/*******************************************************************************
 * FUNCTION: pt_pip2_get_version_show
 *
 * SUMMARY: Sends a PIP2 VERSION command to the DUT and prints the
 *	contents of the response to the passed in output buffer.
 *
 * PARAMETERS:
 *	*dev  - pointer to device structure
 *	*attr - pointer to device attributes structure
 *	*buf  - pointer to print output buffer
 ******************************************************************************/
static ssize_t pt_pip2_get_version_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int rc;
	u16 actual_read_len;
	u8 read_buf[256];
	struct pip2_cmd_structure pip2_cmd;

	/*get version*/
	rc = cmd->nonhid_cmd->pip2_send_cmd(dev, &pip2_cmd,
		PIP2_CMD_ID_VERSION, NULL, 0, read_buf, &actual_read_len);

	if (rc == 0) {
		return snprintf(buf, CY_MAX_PRBUF_SIZE,
			"PIP VERSION  : %02x.%02x\n"
			"BL VERSION   : %02x.%02x\n"
			"FW VERSION   : %02x.%02x\n"
			"SILICON ID   : %02x.%02x.%02x.%02x\n",
			read_buf[PIP2_RESPONSE_BODY_OFFSET + 1],
			read_buf[PIP2_RESPONSE_BODY_OFFSET],
			read_buf[PIP2_RESPONSE_BODY_OFFSET + 3],
			read_buf[PIP2_RESPONSE_BODY_OFFSET + 2],
			read_buf[PIP2_RESPONSE_BODY_OFFSET + 5],
			read_buf[PIP2_RESPONSE_BODY_OFFSET + 4],
			read_buf[PIP2_RESPONSE_BODY_OFFSET + 6],
			read_buf[PIP2_RESPONSE_BODY_OFFSET + 7],
			read_buf[PIP2_RESPONSE_BODY_OFFSET + 8],
			read_buf[PIP2_RESPONSE_BODY_OFFSET + 9]);
	} else {
		return snprintf(buf, CY_MAX_PRBUF_SIZE,
			"PIP VERSION  : - DUT access error\n"
			"BL VERSION   : - DUT access error\n"
			"FW VERSION   : - DUT access error\n"
			"SILICON ID   : - DUT access error\n");
	}
}
static DEVICE_ATTR(pip2_get_version, S_IRUGO,
	pt_pip2_get_version_show, NULL);


/*******************************************************************************
 * FUNCTION: pt_pip2_bl_status_show
 *
 * SUMMARY: The show method for the pip2_bl_status sysfs node.
 *	Shows the percent completion of the current BL or an error message.
 *
 * PARAMETERS:
 *	*dev  - pointer to device structure
 *	*attr - pointer to device attributes structure
 *	*buf  - pointer to print output buffer
 ******************************************************************************/
static ssize_t pt_pip2_bl_status_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cyttsp5_loader_data *ld = cyttsp5_get_loader_data(dev);

	if (ld->pip2_bl_status <= PIP2_BL_STATUS_COMPLETE) {
		pt_debug(dev, DL_DEBUG,
			"%s BL_STATUS = %d\n", __func__, ld->pip2_bl_status);
		return sprintf(buf, "%d\n", ld->pip2_bl_status);
	}

	switch (ld->pip2_bl_status) {
	case PIP2_BL_STATUS_GENERAL_ERROR:
		sprintf(buf,
			"ERROR: %d - General programming error",
			ld->pip2_bl_status);
		break;
	case PIP2_BL_STATUS_PIP_VERSION_ERROR:
		sprintf(buf,
			"ERROR: %d - Wrong PIP version detected",
			ld->pip2_bl_status);
		break;
	case PIP2_BL_STATUS_FW_VERSION_ERROR:
		sprintf(buf,
			"ERROR: %d - FW vervion newer than bin file",
			ld->pip2_bl_status);
		break;
	case PIP2_BL_STATUS_ERASE_ERROR:
		sprintf(buf,
			"ERROR: %d - Erase FW file error",
			ld->pip2_bl_status);
		break;
	case PIP2_BL_STATUS_FILE_CLOSE_ERROR:
		sprintf(buf,
			"ERROR: %d - Close FW file error",
			ld->pip2_bl_status);
		break;
	case PIP2_BL_STATUS_WRITE_ERROR:
		sprintf(buf,
			"ERROR: %d - File write error",
			ld->pip2_bl_status);
		break;
	case PIP2_BL_STATUS_EXECUTE_ERROR:
		sprintf(buf,
			"ERROR: %d - Execute RAM image failure",
			ld->pip2_bl_status);
		break;
	case PIP2_BL_STATUS_RESET_ERROR:
		sprintf(buf,
			"ERROR: %d - Reset DUT error",
			ld->pip2_bl_status);
		break;
	case PIP2_BL_STATUS_MODE_ERROR:
		sprintf(buf,
			"ERROR: %d - Incorrect BL/App mode detected",
			ld->pip2_bl_status);
		break;
	default:
		sprintf(buf,
			"ERROR: %d - Unknown error",
			ld->pip2_bl_status);
	}
	return 0;
}
static DEVICE_ATTR(pip2_bl_status, S_IRUSR, pt_pip2_bl_status_show, NULL);

/*******************************************************************************
 * FUNCTION: pt_pip2_get_last_error_show
 *
 * SUMMARY: The show method for the pip2_get_last_error_show sysfs node.
 *	Shows the last BL error code.
 *
 *	NOTE: If function is called when DUT is already in BL mode, the DUT
 *	will remain in BL mode when funciton exits, however if DUT is in
 *	normal mode when funciton is called, the DUT will be forced into BL
 *	mode and then returned to normal mode when funciton exits.
 *
 * PARAMETERS:
 *	*dev  - pointer to device structure
 *	*attr - pointer to device attributes structure
 *	*buf  - pointer to print output buffer
 ******************************************************************************/
static ssize_t pt_pip2_get_last_error_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int rc;
	u16 actual_read_len;
	u8 read_buf[256];
	u8 mode;
	struct pip2_cmd_structure pip2_cmd;
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);

	/*
	 * First determine if we are already in BL mode
	 * The PIP2 STATUS command is only available in the BL.
	 */
	rc = cmd->request_get_mode(dev, 0, &mode);
	if (rc < 0 || mode == CY_MODE_BOOTLOADER)
		mode = CY_MODE_BOOTLOADER;

	if (mode != CY_MODE_BOOTLOADER) {
		cmd->request_stop_wd(dev);

		/* Hold the host mode pin to force DUT to stay in BL */
		pip2_set_boot_mode_pin(dev, 0);

		/* PATCH Send enter_bl command to ensure correct clock speed */
		pt_debug(dev, DL_WARN,
			"%s sending PIP start_bl command now...\n", __func__);
		rc = cmd->nonhid_cmd->start_bl(dev, 1);

		/* sleep to allow BL to come up */
		msleep(PIP2_LAUNCH_BL_DELAY);

		pip2_set_boot_mode_pin(dev, 1);
	}

	/* Determine Bootloader PIP version, 2.0 needs special handling */
	cmd->request_active_pip_prot(dev, &(cd->bl_pip_version_major),
		&(cd->bl_pip_version_minor));

	/* get last error information */
	rc = cmd->nonhid_cmd->pip2_send_cmd(dev, &pip2_cmd,
		PIP2_CMD_ID_GET_LAST_ERRNO, NULL, 0, read_buf,
		&actual_read_len);

	/* Reset device to get back to running FW if thats how we started */
	if (mode != CY_MODE_BOOTLOADER) {
		if (cd->cpdata->xres)
			cd->cpdata->xres(cd->cpdata, cd->dev);
		cmd->request_start_wd(dev);
	}

	return snprintf(buf, CY_MAX_PRBUF_SIZE,
		"PIP2_LAST_ERRORNO: %d\n",
		read_buf[PIP2_RESPONSE_BODY_OFFSET + 1]);

}
static DEVICE_ATTR(pip2_get_last_error, S_IRUSR,
	pt_pip2_get_last_error_show, NULL);

/*******************************************************************************
 * FUNCTION: pt_loader_probe
 *
 * SUMMARY: The probe function for the FW loader
 *
 * PARAMETERS:
 *	*dev   - pointer to device structure
 *	**data - double pointer to the loader data to be created here
 ******************************************************************************/
static int pt_loader_probe(struct device *dev, void **data)
{
	struct cyttsp5_loader_data *ld;
	struct pip2_loader_data *pip2_data;
	struct cyttsp5_platform_data *pdata = dev_get_platdata(dev);
	int rc;

	pt_debug(dev, DL_INFO,
		"%s: entering pt_loader_probe\n", __func__);

	if (!pdata || !pdata->loader_pdata) {
		pt_debug(dev, DL_ERROR,
			"%s: Missing platform data\n", __func__);
		rc = -ENODEV;
		goto error_no_pdata;
	}

	ld = kzalloc(sizeof(*ld), GFP_KERNEL);
	if (!ld) {
		rc = -ENOMEM;
		goto error_alloc_data_failed;
	}

	pip2_data = kzalloc(sizeof(*pip2_data), GFP_KERNEL);
	if (!pip2_data) {
		rc = -ENOMEM;
		goto error_alloc_data_failed;
	}
	pip2_data->dev = dev;
	ld->pip2_data = pip2_data;

	/* Initialize boot loader status */
	ld->pip2_bl_status = PIP2_BL_STATUS_IDLE;

	rc = device_create_file(dev, &dev_attr_pip2_boot_mode_pin);
	if (rc) {
		pt_debug(dev, DL_ERROR, "%s: Error creating pip2_boot_mode_pin sysfs\n",
				__func__);
		goto error_create_pip2_boot_mode_pin;
	}

	rc = device_create_file(dev, &dev_attr_pip2_manual_upgrade);
	if (rc) {
		pt_debug(dev, DL_ERROR, "%s: Error creating pip2_manual_upgrade sysfs\n",
				__func__);
		goto error_create_pip2_manual_upgrade;
	}

	rc = device_create_file(dev, &dev_attr_pip2_manual_ram_upgrade);
	if (rc) {
		pt_debug(dev, DL_ERROR, "%s: Error creating pip2_manual_ram_upgrade sysfs\n",
				__func__);
		goto error_create_pip2_manual_ram_upgrade;
	}

	rc = device_create_file(dev, &dev_attr_pip2_flash_erase);
	if (rc) {
		pt_debug(dev, DL_ERROR, "%s: Error creating pip2_flash_erase sysfs\n",
				__func__);
		goto error_create_pip2_flash_erase;
	}

	rc = device_create_file(dev, &dev_attr_pip2_get_version);
	if (rc) {
		pt_debug(dev, DL_ERROR, "%s: Error creating pip2_get_version sysfs\n",
				__func__);
		goto error_create_pip2_get_version;
	}

	rc = device_create_file(dev, &dev_attr_pip2_bl_status);
	if (rc) {
		pt_debug(dev, DL_ERROR, "%s: Error creating pip2_bl_status sysfs\n",
				__func__);
		goto error_create_pip2_bl_status;
	}

	rc = device_create_file(dev, &dev_attr_pip2_get_last_error);
	if (rc) {
		pt_debug(dev, DL_ERROR, "%s: Error creating pip2_get_last_error\n",
				__func__);
		goto error_create_pip2_get_last_error;
	}

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_PLATFORM_FW_UPGRADE
	rc = device_create_file(dev, &dev_attr_forced_upgrade);
	if (rc) {
		pt_debug(dev, DL_ERROR, "%s: Error creating forced_upgrade\n",
				__func__);
		goto error_create_forced_upgrade;
	}
#endif

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_BINARY_FW_UPGRADE
	rc = device_create_file(dev, &dev_attr_manual_upgrade);
	if (rc) {
		pt_debug(dev, DL_ERROR, "%s: Error creating manual_upgrade\n",
				__func__);
		goto error_create_manual_upgrade;
	}
#endif

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_MANUAL_TTCONFIG_UPGRADE
	rc = device_create_file(dev, &dev_attr_config_loading);
	if (rc) {
		pt_debug(dev, DL_ERROR, "%s: Error creating config_loading\n",
				__func__);
		goto error_create_config_loading;
	}

	rc = device_create_bin_file(dev, &bin_attr_config_data);
	if (rc) {
		pt_debug(dev, DL_ERROR, "%s: Error creating config_data\n",
				__func__);
		goto error_create_config_data;
	}
#endif

	ld->loader_pdata = pdata->loader_pdata;
	ld->dev = dev;
	*data = ld;

#if CYTTSP5_FW_UPGRADE
	init_completion(&ld->int_running);
#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_BINARY_FW_UPGRADE
	init_completion(&ld->builtin_bin_fw_complete);
#endif
	cmd->subscribe_attention(dev, CY_ATTEN_IRQ, CYTTSP5_LOADER_NAME,
		cyttsp5_loader_attention, CY_MODE_BOOTLOADER);

	cmd->subscribe_attention(dev, CY_ATTEN_LOADER, CYTTSP5_LOADER_NAME,
		cyttsp5_fw_upgrade_cb, CY_MODE_UNKNOWN);
#endif
#if CYTTSP5_FW_UPGRADE || CYTTSP5_TTCONFIG_UPGRADE
	init_completion(&ld->calibration_complete);
	INIT_WORK(&ld->calibration_work, cyttsp5_calibrate_idacs);
#endif
#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_MANUAL_TTCONFIG_UPGRADE
	mutex_init(&ld->config_lock);
#endif

#ifdef UPGRADE_FW_AND_CONFIG_IN_PROBE
	/* Call FW and config upgrade directly in probe */
	cyttsp5_fw_and_config_upgrade(&ld->fw_and_config_upgrade);
#else
	INIT_WORK(&ld->fw_and_config_upgrade, cyttsp5_fw_and_config_upgrade);
	schedule_work(&ld->fw_and_config_upgrade);
#endif

	pt_debug(dev, DL_INFO, "%s: Successful probe %s\n",
		__func__, dev_name(dev));
	return 0;

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_MANUAL_TTCONFIG_UPGRADE
error_create_config_data:
	device_remove_file(dev, &dev_attr_config_loading);
error_create_config_loading:
#endif
#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_BINARY_FW_UPGRADE
	device_remove_file(dev, &dev_attr_manual_upgrade);
error_create_manual_upgrade:
#endif
#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_PLATFORM_FW_UPGRADE
	device_remove_file(dev, &dev_attr_forced_upgrade);
error_create_forced_upgrade:
#endif

error_create_pip2_boot_mode_pin:
error_create_pip2_manual_upgrade:
error_create_pip2_manual_ram_upgrade:
error_create_pip2_flash_erase:
error_create_pip2_get_version:
error_create_pip2_bl_status:
error_create_pip2_get_last_error:

	kfree(ld);
error_alloc_data_failed:
error_no_pdata:
	pt_debug(dev, DL_ERROR, "%s failed.\n", __func__);
	return rc;
}

static void cyttsp5_loader_release(struct device *dev, void *data)
{
	struct cyttsp5_loader_data *ld = (struct cyttsp5_loader_data *)data;

#if CYTTSP5_FW_UPGRADE
	cmd->unsubscribe_attention(dev, CY_ATTEN_IRQ, CYTTSP5_LOADER_NAME,
		cyttsp5_loader_attention, CY_MODE_BOOTLOADER);

	cmd->unsubscribe_attention(dev, CY_ATTEN_LOADER, CYTTSP5_LOADER_NAME,
		cyttsp5_fw_upgrade_cb, CY_MODE_UNKNOWN);
#endif
#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_MANUAL_TTCONFIG_UPGRADE
	device_remove_bin_file(dev, &bin_attr_config_data);
	device_remove_file(dev, &dev_attr_config_loading);
	kfree(ld->config_data);
#endif
#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_BINARY_FW_UPGRADE
	device_remove_file(dev, &dev_attr_manual_upgrade);
#endif
#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_PLATFORM_FW_UPGRADE
	device_remove_file(dev, &dev_attr_forced_upgrade);
#endif
	kfree(ld);
}

static struct cyttsp5_module loader_module = {
	.name = CYTTSP5_LOADER_NAME,
	.probe = pt_loader_probe,
	.release = cyttsp5_loader_release,
};

static int __init cyttsp5_loader_init(void)
{
	int rc;

	cmd = pt_get_commands();
	if (!cmd)
		return -EINVAL;

	rc = cyttsp5_register_module(&loader_module);
	if (rc < 0) {
		pr_err("%s: Error, failed registering module\n",
			__func__);
			return rc;
	}

	pr_info("%s: Parade TTSP FW Loader Driver (Built %s) rc=%d\n",
		 __func__, CY_DRIVER_VERSION, rc);
	return 0;
}
module_init(cyttsp5_loader_init);

static void __exit cyttsp5_loader_exit(void)
{
	cyttsp5_unregister_module(&loader_module);
}
module_exit(cyttsp5_loader_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Parade TrueTouch(R) Standard Product FW Loader Driver");
MODULE_AUTHOR("Parade Technologies <ttdrivers@paradetech.com>");
