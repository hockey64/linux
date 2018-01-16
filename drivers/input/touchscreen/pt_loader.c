/*
 * pt_loader.c
 * Parade TrueTouch(TM) Standard Product V5 FW Loader Module.
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
#include <linux/firmware.h>

#define PT_LOADER_NAME "pt_loader"
#define PT_FW_MANUAL_UPGRADE_FILE_NAME "pt_fw_manual_upgrade"

/* Enable UPGRADE_FW_AND_CONFIG_IN_PROBE definition
 * to perform FW and config upgrade during probe
 * instead of scheduling a work for it
 */
/* #define UPGRADE_FW_AND_CONFIG_IN_PROBE */

#define PT_AUTO_LOAD_FOR_CORRUPTED_FW 1
#define PT_LOADER_FW_UPGRADE_RETRY_COUNT 3

#define PT_FW_UPGRADE \
	(defined(CONFIG_TOUCHSCREEN_PARADE_PLATFORM_FW_UPGRADE) \
	|| defined(CONFIG_TOUCHSCREEN_PARADE_BINARY_FW_UPGRADE))

#define PT_TTCONFIG_UPGRADE \
	(defined(CONFIG_TOUCHSCREEN_PARADE_PLATFORM_TTCONFIG_UPGRADE) \
	|| defined(CONFIG_TOUCHSCREEN_PARADE_MANUAL_TTCONFIG_UPGRADE))

static const u8 pt_security_key[] = {
	0xA5, 0x01, 0x02, 0x03, 0xFF, 0xFE, 0xFD, 0x5A
};

/* Timeout values in ms. */
#define PT_LDR_REQUEST_EXCLUSIVE_TIMEOUT		500
#define PT_LDR_SWITCH_TO_APP_MODE_TIMEOUT		300

#define PT_MAX_STATUS_SIZE				32

#define PT_DATA_MAX_ROW_SIZE				256
#define PT_DATA_ROW_SIZE				128

#define PT_ARRAY_ID_OFFSET				0
#define PT_ROW_NUM_OFFSET				1
#define PT_ROW_SIZE_OFFSET				3
#define PT_ROW_DATA_OFFSET				5

#define PT_POST_TT_CFG_CRC_MASK				0x2
static inline struct pt_loader_data *pt_get_loader_data(
		struct device *dev);

static struct pt_core_commands *cmd;

#define PIP2_LAUNCH_APP_DELAY	400

struct pip2_loader_data {
	struct device *dev;
	struct completion pip2_fw_upgrade_complete; /* mutex for loader */
	u8 pip2_file_handle;
};

struct pt_loader_data {
	struct device *dev;
	struct pt_sysinfo *si;
	u8 status_buf[PT_MAX_STATUS_SIZE];
	struct completion int_running;
	struct completion calibration_complete;
#ifdef CONFIG_TOUCHSCREEN_PARADE_BINARY_FW_UPGRADE
	int builtin_bin_fw_status;
	bool is_manual_upgrade_enabled;
#endif
	struct work_struct fw_and_config_upgrade;
	struct work_struct calibration_work;
	struct pt_loader_platform_data *loader_pdata;
#ifdef CONFIG_TOUCHSCREEN_PARADE_MANUAL_TTCONFIG_UPGRADE
	struct mutex config_lock;
	u8 *config_data;
	int config_size;
	bool config_loading;
#endif
	struct pip2_loader_data *pip2_data;
	bool pip2_load_fw_to_ram;
	bool pip2_load_builtin;
};

static u8   pip2_bl_status;
static u8   pip2_erase_status;

/* PIP2 BL status codes. 1-99% are "active" */
enum PIP2_BL_STATUS {
	PIP2_BL_STATUS_IDLE                 = 0,
	PIP2_BL_STATUS_ACTIVE_1             = 1,
	PIP2_BL_STATUS_ACTIVE_99            = 99,
	PIP2_BL_STATUS_COMPLETE             = 100,
	PIP2_BL_STATUS_GENERAL_ERROR        = 200,
	PIP2_BL_STATUS_PIP_VERSION_ERROR    = 201,
	PIP2_BL_STATUS_FW_VERSION_ERROR     = 202,
	PIP2_BL_STATUS_ERASE_ERROR          = 203,
	PIP2_BL_STATUS_FILE_CLOSE_ERROR     = 204,
	PIP2_BL_STATUS_WRITE_ERROR          = 205,
	PIP2_BL_STATUS_EXECUTE_ERROR        = 206,
	PIP2_BL_STATUS_RESET_ERROR          = 207,
	PIP2_BL_STATUS_MODE_ERROR           = 208,
	PIP2_BL_STATUS_ENTER_BL_ERROR       = 209,
	PIP2_BL_STATUS_FILE_OPEN_ERROR      = 210,
	PIP2_BL_STATUS_FW_SENTINEL_NOT_SEEN = 211,
};

struct pt_dev_id {
	u32 silicon_id;
	u8 rev_id;
	u32 bl_ver;
};

struct pt_hex_image {
	u8 array_id;
	u16 row_num;
	u16 row_size;
	u8 row_data[PT_DATA_ROW_SIZE];
} __packed;

#ifdef CONFIG_TOUCHSCREEN_PARADE_BINARY_FW_UPGRADE
static int pt_pip2_upgrade_firmware_from_builtin(struct device *dev);
#endif

static struct pt_module loader_module;

/*******************************************************************************
 * FUNCTION: pt_get_loader_data
 *
 * SUMMARY: Inline function to get pt_loader_data pointer from loader module.
 *
 * RETURN:
 *  pointer to pt_loader_data structure
 *
 * PARAMETERS:
 *  *dev  - pointer to device structure
 ******************************************************************************/
static inline struct pt_loader_data *pt_get_loader_data(
		struct device *dev)
{
	return pt_get_module_data(dev, &loader_module);
}

#if PT_FW_UPGRADE || PT_TTCONFIG_UPGRADE
/*******************************************************************************
 * FUNCTION: pt_calibrate_idacs
 *
 * SUMMARY: Calibrate idac for mutual-cap,buttons,self-cap by called functions.
 * It needs stop panel scan during calibration.
 *
 * PARAMETERS:
 *	*calibration_work - pointer to work_struct structure
 ******************************************************************************/
static void pt_calibrate_idacs(struct work_struct *calibration_work)
{
	struct pt_loader_data *ld = container_of(calibration_work,
			struct pt_loader_data, calibration_work);
	struct device *dev = ld->dev;
	u8 mode;
	u8 status;
	int rc;

	pt_debug(dev, DL_INFO, "Entering %s\n", __func__);
	rc = cmd->request_exclusive(dev, PT_LDR_REQUEST_EXCLUSIVE_TIMEOUT);
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

/*******************************************************************************
 * FUNCTION: pt_calibration_attention
 *
 * SUMMARY: Wrapper function to schedule calibration work used to subscribe into
 * TTDL attention list.Once called will unsubscribe from attention list.
 *
 * RETURN:
 *   0 = success
 *  !0 = failure
 *
 * PARAMETERS:
 *  *dev - pointer to device structure
 ******************************************************************************/
static int pt_calibration_attention(struct device *dev)
{
	struct pt_loader_data *ld = pt_get_loader_data(dev);
	int rc = 0;

	schedule_work(&ld->calibration_work);

	cmd->unsubscribe_attention(dev, PT_ATTEN_STARTUP, PT_LOADER_NAME,
		pt_calibration_attention, 0);

	return rc;
}
#endif /* PT_FW_UPGRADE || PT_TTCONFIG_UPGRADE */

#if PT_FW_UPGRADE \
	|| defined(CONFIG_TOUCHSCREEN_PARADE_PLATFORM_TTCONFIG_UPGRADE)

/*******************************************************************************
 * FUNCTION: pt_get_panel_id
 *
 * SUMMARY: Get panel id from core data.
 *
 * RETURN:
 *	 panel id
 *
 * PARAMETERS:
 *	*dev  - pointer to device structure
 ******************************************************************************/
static u8 pt_get_panel_id(struct device *dev)
{
	struct pt_core_data *cd = dev_get_drvdata(dev);

	return cd->panel_id;
}
#endif


#if (PT_FW_UPGRADE || PT_TTCONFIG_UPGRADE)
/*******************************************************************************
 * FUNCTION: pt_check_firmware_version
 *
 * SUMMARY: Compare fw's version and revision control number with fw image's.
 *
 * RETURN:
 *  -1: Do not upgrade firmware
 *   0: Version info same, let caller decide
 *   1: Do a firmware upgrade
 *
 * PARAMETERS:
 *  *dev             - pointer to device structure
 *   fw_ver_new      - firmware version
 *   fw_revctrl_new  - firmware revision control number
 ******************************************************************************/
static int pt_check_firmware_version(struct device *dev,
		u32 fw_ver_new, u32 fw_revctrl_new)
{
	struct pt_loader_data *ld = pt_get_loader_data(dev);
	u32 fw_ver_img;
	u32 fw_revctrl_img;

	fw_ver_img = ld->si->ttdata.fw_ver_major << 8;
	fw_ver_img += ld->si->ttdata.fw_ver_minor;

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

	fw_revctrl_img = ld->si->ttdata.revctrl;

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

#endif /* PT_FW_UPGRADE || PT_TTCONFIG_UPGRADE */

#if PT_FW_UPGRADE
/*******************************************************************************
 * FUNCTION: pt_get_row_
 *
 * SUMMARY: Copy the image data to the "row_buf".
 *
 * RETURN:
 *	 pointer to image buffer plus copy size
 *
 * PARAMETERS:
 *  *dev        - pointer to device structure
 *  *row_buf    - pointer to written buffer to program chip
 *  *image_buf  - pointer to image buffer
 *   size       - size of written data
 ******************************************************************************/
static u8 *pt_get_row_(struct device *dev, u8 *row_buf,
		u8 *image_buf, int size)
{
	memcpy(row_buf, image_buf, size);
	return image_buf + size;
}

/*******************************************************************************
 * FUNCTION: pt_ldr_enter_
 *
 * SUMMARY: Enter bootloader state and update device id(silicon id, rev id,
 *  bootloader version).
 *
 * RETURN:
 *   0 = success
 *  !0 = failure
 *
 * PARAMETERS:
 *  *dev     - pointer to device structure
 *  *dev_id  - pointer to device id
 ******************************************************************************/
static int pt_ldr_enter_(struct device *dev, struct pt_dev_id *dev_id)
{
	int rc;
	u8 return_data[8];
	u8 mode;

	dev_id->silicon_id = 0;
	dev_id->rev_id = 0;
	dev_id->bl_ver = 0;

	cmd->request_reset(dev, PT_CORE_CMD_UNPROTECTED);

	rc = cmd->request_get_mode(dev, 0, &mode);
	if (rc < 0)
		return rc;

	if (mode == PT_MODE_UNKNOWN)
		return -EINVAL;

	if (mode == PT_MODE_OPERATIONAL) {
		rc = cmd->nonhid_cmd->start_bl(dev, PT_CORE_CMD_UNPROTECTED);
		if (rc < 0)
			return rc;
	}

	rc = cmd->nonhid_cmd->get_bl_info(dev,
		PT_CORE_CMD_UNPROTECTED, return_data);
	if (rc < 0)
		return rc;

	dev_id->silicon_id = get_unaligned_le32(&return_data[0]);
	dev_id->rev_id = return_data[4];
	dev_id->bl_ver = return_data[5] + (return_data[6] << 8)
		+ (return_data[7] << 16);

	return 0;
}

/*******************************************************************************
 * FUNCTION: pt_ldr_init_
 *
 * SUMMARY: Erase the entire TrueTouch application, Configuration Data block,
 *  and Design Data block in flash and enables the host to execute the Program
 *  and Verify Row command to bootload the application image and data.
 *
 * RETURN:
 *   0 = success
 *  !0 = failure
 *
 * PARAMETERS:
 *  *dev           - pointer to device structure
 *  *pt_hex_image  - pointer to hex image structure
 ******************************************************************************/
static int pt_ldr_init_(struct device *dev,
		struct pt_hex_image *row_image)
{
	return cmd->nonhid_cmd->initiate_bl(dev, 0, 8,
			(u8 *)pt_security_key, row_image->row_size,
			row_image->row_data);
}

/*******************************************************************************
 * FUNCTION: pt_ldr_parse_row_
 *
 * SUMMARY: Parse and copy the row buffer data to hex image structure.
 *
 * RETURN:
 *   0 = success
 *  !0 = failure
 *
 * PARAMETERS:
 *  *dev        - pointer to device structure
 *  *row_buf    - pointer to row buffer
 *  *row_image  - pointer to hex image structure
 ******************************************************************************/
static int pt_ldr_parse_row_(struct device *dev, u8 *row_buf,
	struct pt_hex_image *row_image)
{
	int rc = 0;

	row_image->array_id = row_buf[PT_ARRAY_ID_OFFSET];
	row_image->row_num = get_unaligned_be16(&row_buf[PT_ROW_NUM_OFFSET]);
	row_image->row_size = get_unaligned_be16(&row_buf[PT_ROW_SIZE_OFFSET]);

	if (row_image->row_size > ARRAY_SIZE(row_image->row_data)) {
		pt_debug(dev, DL_ERROR,
			"%s: row data buffer overflow\n", __func__);
		rc = -EOVERFLOW;
		goto pt_ldr_parse_row_exit;
	}

	memcpy(row_image->row_data, &row_buf[PT_ROW_DATA_OFFSET],
	       row_image->row_size);
pt_ldr_parse_row_exit:
	return rc;
}

/*******************************************************************************
 * FUNCTION: pt_ldr_prog_row_
 *
 * SUMMARY: Program one row that the hex image structure data to the chip.
 *
 * RETURN:
 *   0 = success
 *  !0 = failure
 *
 * PARAMETERS:
 *  *dev        - pointer to device structure
 *  *row_image  - pointer to hex image structure
 ******************************************************************************/
static int pt_ldr_prog_row_(struct device *dev,
				 struct pt_hex_image *row_image)
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

/*******************************************************************************
 * FUNCTION: pt_ldr_verify_chksum_
 *
 * SUMMARY: Perform a full verification of the application integrity by
 *  calculating the CRC of the TrueTouch application image in flash and
 *  comparing it to the expected CRC stored in the TrueTouch application CRC
 *  value stored in the Metadata row.
 *
 * RETURN:
 *   0 = success
 *  !0 = failure
 *
 * PARAMETERS:
 *  *dev  - pointer to device structure
 ******************************************************************************/
static int pt_ldr_verify_chksum_(struct device *dev)
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

/*******************************************************************************
 * FUNCTION: pt_ldr_exit_
 *
 * SUMMARY: Launch the application from bootloader.
 *
 * RETURN:
 *   0 = success
 *  !0 = failure
 *
 * PARAMETERS:
 *  *dev  - pointer to device structure
 ******************************************************************************/
static int pt_ldr_exit_(struct device *dev)
{
	return cmd->nonhid_cmd->launch_app(dev, 0);
}

/*******************************************************************************
 * FUNCTION: pt_load_app_
 *
 * SUMMARY: Program the firmware image to the chip.
 *
 * RETURN:
 *   0 = success
 *  !0 = failure
 *
 * PARAMETERS:
 *  *dev      - pointer to device structure
 *  *fw       - pointer to the firmware
 *   fw_size  - size of firmware
 ******************************************************************************/
static int pt_load_app_(struct device *dev, const u8 *fw, int fw_size)
{
	struct pt_dev_id *dev_id;
	struct pt_hex_image *row_image;
	u8 *row_buf;
	size_t image_rec_size;
	size_t row_buf_size = PT_DATA_MAX_ROW_SIZE;
	int row_count = 0;
	u8 *p;
	u8 *last_row;
	int rc;
	int rc_tmp;

	image_rec_size = sizeof(struct pt_hex_image);
	if (fw_size % image_rec_size != 0) {
		pt_debug(dev, DL_ERROR,
			"%s: Firmware image is misaligned\n", __func__);
		rc = -EINVAL;
		goto _pt_load_app_error;
	}

	pt_debug(dev, DL_INFO, "%s: start load app\n", __func__);
#ifdef TTHE_TUNER_SUPPORT
	cmd->request_tthe_print(dev, NULL, 0, "start load app");
#endif

	row_buf = kzalloc(row_buf_size, GFP_KERNEL);
	row_image = kzalloc(sizeof(struct pt_hex_image), GFP_KERNEL);
	dev_id = kzalloc(sizeof(struct pt_dev_id), GFP_KERNEL);
	if (!row_buf || !row_image || !dev_id) {
		rc = -ENOMEM;
		goto _pt_load_app_exit;
	}

	cmd->request_stop_wd(dev);

	pt_debug(dev, DL_INFO, "%s: Send BL Loader Enter\n", __func__);
#ifdef TTHE_TUNER_SUPPORT
	cmd->request_tthe_print(dev, NULL, 0, "Send BL Loader Enter");
#endif
	rc = pt_ldr_enter_(dev, dev_id);
	if (rc) {
		pt_debug(dev, DL_ERROR, "%s: Error cannot start Loader (ret=%d)\n",
			__func__, rc);
		goto _pt_load_app_exit;
	}
	pt_debug(dev, DL_INFO,
		"%s: dev: silicon id=%08X rev=%02X bl=%08X\n", __func__,
		dev_id->silicon_id, dev_id->rev_id, dev_id->bl_ver);

	/* get last row */
	last_row = (u8 *)fw + fw_size - image_rec_size;
	pt_get_row_(dev, row_buf, last_row, image_rec_size);
	pt_ldr_parse_row_(dev, row_buf, row_image);

	/* initialise bootloader */
	rc = pt_ldr_init_(dev, row_image);
	if (rc) {
		pt_debug(dev, DL_ERROR, "%s: Error cannot init Loader (ret=%d)\n",
			__func__, rc);
		goto _pt_load_app_exit;
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
		p = pt_get_row_(dev, row_buf, p, image_rec_size);

		/* Parse row */
		pt_debug(dev, DL_INFO, "%s: p=%p buf=%p buf[0]=%02X\n",
			__func__, p, row_buf, row_buf[0]);
		rc = pt_ldr_parse_row_(dev, row_buf, row_image);
		pt_debug(dev, DL_INFO,
			"%s: array_id=%02X row_num=%04X(%d) row_size=%04X(%d)\n",
			__func__, row_image->array_id,
			row_image->row_num, row_image->row_num,
			row_image->row_size, row_image->row_size);
		if (rc) {
			pt_debug(dev, DL_ERROR, "%s: Parse Row Error (a=%d r=%d ret=%d\n",
				__func__, row_image->array_id,
				row_image->row_num, rc);
			goto _pt_load_app_exit;
		} else {
			pt_debug(dev, DL_INFO,
				"%s: Parse Row (a=%d r=%d ret=%d\n",
				__func__, row_image->array_id,
				row_image->row_num, rc);
		}

		/* program row */
		rc = pt_ldr_prog_row_(dev, row_image);
		if (rc) {
			pt_debug(dev, DL_ERROR, "%s: Program Row Error (array=%d row=%d ret=%d)\n",
				__func__, row_image->array_id,
				row_image->row_num, rc);
			goto _pt_load_app_exit;
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
	rc = pt_ldr_exit_(dev);
	if (rc) {
		pt_debug(dev, DL_ERROR, "%s: Error on exit Loader (ret=%d)\n",
			__func__, rc);

		/* verify app checksum */
		rc_tmp = pt_ldr_verify_chksum_(dev);
		if (rc_tmp)
			pt_debug(dev, DL_ERROR, "%s: ldr_verify_chksum fail r=%d\n",
				__func__, rc_tmp);
		else
			pt_debug(dev, DL_INFO,
				"%s: APP Checksum Verified\n", __func__);
	}

_pt_load_app_exit:
	kfree(row_buf);
	kfree(row_image);
	kfree(dev_id);
_pt_load_app_error:
	return rc;
}

/*******************************************************************************
 * FUNCTION: pt_upgrade_firmware
 *
 * SUMMARY: Program the firmware image and set call back for start up.
 *
 * RETURN:
 *   0 = success
 *  !0 = failure
 *
 * PARAMETERS:
 *  *dev      - pointer to device structure
 *  *fw_img   - pointer to the firmware
 *   fw_size  - size of firmware
 ******************************************************************************/
static int pt_upgrade_firmware(struct device *dev, const u8 *fw_img,
		int fw_size)
{
	struct pt_loader_data *ld = pt_get_loader_data(dev);
	int retry = PT_LOADER_FW_UPGRADE_RETRY_COUNT;
	bool wait_for_calibration_complete = false;
	int rc;

	pm_runtime_get_sync(dev);

	rc = cmd->request_exclusive(dev, PT_LDR_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0)
		goto exit;

	while (retry--) {
		rc = pt_load_app_(dev, fw_img, fw_size);
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
			 & PT_LOADER_FLAG_CALIBRATE_AFTER_FW_UPGRADE)) {
#if (KERNEL_VERSION(3, 13, 0) <= LINUX_VERSION_CODE)
		reinit_completion(&ld->calibration_complete);
#else
		INIT_COMPLETION(ld->calibration_complete);
#endif
		/* set up call back for startup */
		pt_debug(dev, DL_INFO,
			"%s: Adding callback for calibration\n", __func__);
		rc = cmd->subscribe_attention(dev, PT_ATTEN_STARTUP,
			PT_LOADER_NAME, pt_calibration_attention, 0);
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

#endif /* PT_FW_UPGRADE */

#ifdef CONFIG_TOUCHSCREEN_PARADE_PLATFORM_FW_UPGRADE
/*******************************************************************************
 * FUNCTION: pt_check_firmware_version_platform
 *
 * SUMMARY: The caller of function pt_check_firmware_version() to determine
 *  whether to load firmware from touch_firmware structure.
 *
 * RETURN:
 *   0: Don't upgrade
 *   1: Upgrade
 *
 * PARAMETERS:
 *  *dev  - pointer to device structure
 *  *fw   - pointer to the touch_firmware structure
 ******************************************************************************/
static int pt_check_firmware_version_platform(struct device *dev,
		struct pt_touch_firmware *fw)
{
	struct pt_loader_data *ld = pt_get_loader_data(dev);
	u32 fw_ver_new;
	u32 fw_revctrl_new;
	int upgrade;

	if (!ld->si) {
		pt_debug(dev, DL_INFO,
			"%s: No firmware info found, DUT FW may be corrupted\n",
			__func__);
		return PT_AUTO_LOAD_FOR_CORRUPTED_FW;
	}

	fw_ver_new = get_unaligned_be16(fw->ver + 2);
	/* 4 middle bytes are not used */
	fw_revctrl_new = get_unaligned_be32(fw->ver + 8);

	upgrade = pt_check_firmware_version(dev, fw_ver_new,
		fw_revctrl_new);

	if (upgrade > 0)
		return 1;

	return 0;
}

/*******************************************************************************
 * FUNCTION: pt_get_platform_firmware
 *
 * SUMMARY: To get the pointer of right touch_firmware structure by panel id.
 *
 * RETURN:
 *   pointer to touch_firmware structure or null pointer if fail
 *
 * PARAMETERS:
 *  *dev  - pointer to device structure
 ******************************************************************************/
static struct pt_touch_firmware *pt_get_platform_firmware(
		struct device *dev)
{
	struct pt_loader_data *ld = pt_get_loader_data(dev);
	struct pt_touch_firmware **fws;
	struct pt_touch_firmware *fw;
	u8 panel_id;

	panel_id = pt_get_panel_id(dev);
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

/*******************************************************************************
 * FUNCTION: upgrade_firmware_from_platform
 *
 * SUMMARY: Get touch_firmware structure and perform upgrade if pass the
 *  firmware version check.
 *
 * RETURN:
 *   0 = success
 *  !0 = failure
 *
 * PARAMETERS:
 *  *dev     - pointer to device structure
 *   forced  - flag to force upgrade(1:force to upgrade)
 ******************************************************************************/
static int upgrade_firmware_from_platform(struct device *dev,
		bool forced)
{
	struct pt_loader_data *ld = pt_get_loader_data(dev);
	struct pt_touch_firmware *fw;
	int rc = -ENODEV;
	int upgrade;

	if (!ld->loader_pdata) {
		pt_debug(dev, DL_ERROR,
			"%s: No loader platform data\n", __func__);
		return rc;
	}

	fw = pt_get_platform_firmware(dev);
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
		upgrade = pt_check_firmware_version_platform(dev, fw);

	if (upgrade)
		return pt_upgrade_firmware(dev, fw->img, fw->size);

	return rc;
}
#endif /* CONFIG_TOUCHSCREEN_PARADE_PLATFORM_FW_UPGRADE */

#ifdef CONFIG_TOUCHSCREEN_PARADE_BINARY_FW_UPGRADE
/*******************************************************************************
 * FUNCTION: _pt_firmware_cont
 *
 * SUMMARY: Firmware upgrade continue function that verifies the firmware size
 *  in the firmware class and then upgrades the firmware.
 *
 * PARAMETERS:
 *  *fw      - pointer to firmware structure
 *   forced  - flag to force upgrade(1:force to upgrade)
 ******************************************************************************/
static void _pt_firmware_cont(const struct firmware *fw, void *context)
{
	struct device *dev = context;
	struct pt_loader_data *ld = pt_get_loader_data(dev);
	u8 header_size = 0;

	if (!fw)
		goto pt_firmware_cont_exit;

	if (!fw->data || !fw->size) {
		pt_debug(dev, DL_ERROR,
			"%s: No firmware received\n", __func__);
		goto pt_firmware_cont_release_exit;
	}

	header_size = fw->data[0];
	if (header_size >= (fw->size + 1)) {
		pt_debug(dev, DL_ERROR,
			"%s: Firmware format is invalid\n", __func__);
		goto pt_firmware_cont_release_exit;
	}

	pt_upgrade_firmware(dev, &(fw->data[header_size + 1]),
		fw->size - (header_size + 1));

pt_firmware_cont_release_exit:
	if (fw)
		release_firmware(fw);

pt_firmware_cont_exit:
	ld->is_manual_upgrade_enabled = 0;
}

/*******************************************************************************
 * FUNCTION: pt_check_firmware_version_builtin
 *
 * SUMMARY: The caller of function pt_check_firmware_version() to determine
 *  whether to load built-in firmware.
 *
 * RETURN:
 *   0: Don't upgrade
 *   1: Upgrade
 *
 * PARAMETERS:
 *  *dev  - pointer to device structure
 *  *fw   - pointer to the firmware structure
 ******************************************************************************/
static int pt_check_firmware_version_builtin(struct device *dev,
		const struct firmware *fw)
{
	struct pt_loader_data *ld = pt_get_loader_data(dev);
	u32 fw_ver_new;
	u32 fw_revctrl_new;
	int upgrade;

	if (!ld->si) {
		pt_debug(dev, DL_INFO,
			"%s: No firmware info found, DUT FW may be corrupted\n",
			__func__);
		return PT_AUTO_LOAD_FOR_CORRUPTED_FW;
	}

	fw_ver_new = get_unaligned_be16(fw->data + 3);
	/* 4 middle bytes are not used */
	fw_revctrl_new = get_unaligned_be32(fw->data + 9);

	upgrade = pt_check_firmware_version(dev, fw_ver_new,
			fw_revctrl_new);

	if (upgrade > 0)
		return 1;

	return 0;
}

/*******************************************************************************
 * FUNCTION: _pt_firmware_cont_builtin
 *
 * SUMMARY: Perform upgrade if pass the firmware version check.
 *
 * PARAMETERS:
 *  *dev     - pointer to device structure
 *   forced  - flag to force upgrade(1:force to upgrade)
 ******************************************************************************/
static void _pt_firmware_cont_builtin(const struct firmware *fw,
		void *context)
{
	struct device *dev = context;
	struct pt_loader_data *ld = pt_get_loader_data(dev);
	int upgrade;

	if (!fw) {
		pt_debug(dev, DL_INFO,
			"%s: No builtin firmware\n", __func__);
		goto _pt_firmware_cont_builtin_exit;
	}

	if (!fw->data || !fw->size) {
		pt_debug(dev, DL_ERROR,
			"%s: Invalid builtin firmware\n", __func__);
		goto _pt_firmware_cont_builtin_exit;
	}

	pt_debug(dev, DL_INFO, "%s: Found firmware\n", __func__);

	upgrade = pt_check_firmware_version_builtin(dev, fw);
	if (upgrade) {
		_pt_firmware_cont(fw, dev);
		ld->builtin_bin_fw_status = 0;
		return;
	}

_pt_firmware_cont_builtin_exit:
	if (fw)
		release_firmware(fw);

	ld->builtin_bin_fw_status = -EINVAL;
}

/*******************************************************************************
 * FUNCTION: upgrade_firmware_from_class
 *
 * SUMMARY: Create the firmware class but don't actually load any FW to the
 *	DUT. This creates all the sysfs nodes needed for a user to bootload
 *	the DUT with their own bin file.
 *
 * RETURN:
 *   0 = success
 *
 * PARAMETERS:
 *  *dev     - pointer to device structure
 ******************************************************************************/
static int upgrade_firmware_from_class(struct device *dev)
{
	int retval;

	pt_debug(dev, DL_INFO,
		"%s: Enabling firmware class loader\n", __func__);

	retval = request_firmware_nowait(THIS_MODULE, FW_ACTION_NOHOTPLUG,
			PT_FW_MANUAL_UPGRADE_FILE_NAME, dev, GFP_KERNEL, dev,
			_pt_firmware_cont);
	if (retval < 0) {
		pt_debug(dev, DL_ERROR,
			"%s: Fail request firmware class file load\n",
			__func__);
		return retval;
	}

	return 0;
}

/*******************************************************************************
 * FUNCTION: generate_firmware_filename
 *
 * SUMMARY: Generate firmware file name by panel id. Generates binary FW
 *  filename as following:
 *  - Panel ID not enabled: tt_fw.bin
 *  - Panel ID enabled: tt_fw_pidXX.bin
 *
 * RETURN:
 *   pointer to file name or null pointer if fail
 *
 * PARAMETERS:
 *  *dev  - pointer to device structure
 ******************************************************************************/
static char *generate_firmware_filename(struct device *dev)
{
	char *filename;
	u8 panel_id;

#define FILENAME_LEN_MAX 64
	filename = kzalloc(FILENAME_LEN_MAX, GFP_KERNEL);
	if (!filename)
		return NULL;

	panel_id = pt_get_panel_id(dev);
	if (panel_id == PANEL_ID_NOT_ENABLED)
		snprintf(filename, FILENAME_LEN_MAX, "%s", PT_FW_FILE_NAME);
	else
		snprintf(filename, FILENAME_LEN_MAX, "%s_pid%02X%s",
			PT_FW_FILE_PREFIX, panel_id, PT_FW_FILE_SUFFIX);

	pt_debug(dev, DL_INFO, "%s: Filename: %s\n",
		__func__, filename);

	return filename;
}

/*******************************************************************************
 * FUNCTION: upgrade_firmware_from_builtin
 *
 * SUMMARY: Create the firmware class load FW by searching the name of built-in
 *	file. Then perform upgrade after getting the file.
 *
 * RETURN:
 *   0 = success
 *  !0 = failure
 *
 * PARAMETERS:
 *  *dev     - pointer to device structure
 ******************************************************************************/
static int upgrade_firmware_from_builtin(struct device *dev)
{
	struct pt_loader_data *ld = pt_get_loader_data(dev);
	struct pt_core_data *cd = dev_get_drvdata(dev);
	char *filename;
	int retval;
	const struct firmware *fw_entry = NULL;

	pt_debug(dev, DL_INFO,
		"%s: Enabling firmware class loader built-in\n",
		__func__);

	filename = generate_firmware_filename(dev);
	if (!filename) {
		pt_debug(dev, DL_ERROR,
			"%s: ERROR - could not generate FW filename\n",
			__func__);
		return -ENOMEM;
	}

	mutex_lock(&cd->firmware_class_lock);
	retval = request_firmware(&fw_entry, filename, dev);
	if (retval < 0) {
		mutex_unlock(&cd->firmware_class_lock);
		pt_debug(dev, DL_ERROR,
			"%s: Fail request firmware class file load\n",
			__func__);
		goto exit;
	}
	_pt_firmware_cont_builtin(fw_entry, dev);

	mutex_unlock(&cd->firmware_class_lock);

	retval = ld->builtin_bin_fw_status;

exit:
	kfree(filename);

	return retval;
}
#endif /* CONFIG_TOUCHSCREEN_PARADE_BINARY_FW_UPGRADE */

#if PT_TTCONFIG_UPGRADE
/*******************************************************************************
 * FUNCTION: pt_write_config_row_
 *
 * SUMMARY: Alow to program the data block area that includes configuration
 *  data, manufacturing data, design data.
 *
 * RETURN:
 *   0 = success
 *  !0 = failure
 *
 * PARAMETERS:
 *  *dev         - pointer to device structure
 *   ebid        - block id to determine the block name(configuration,etc)
 *   row_number  - row number if written block
 *   row_size    - row size of written data
 *  *data        - pointer to the data to write
 ******************************************************************************/
static int pt_write_config_row_(struct device *dev, u8 ebid,
		u16 row_number, u16 row_size, u8 *data)
{
	int rc;
	u16 actual_write_len;

	rc = cmd->nonhid_cmd->write_conf_block(dev, 0, row_number,
			row_size, ebid, data, (u8 *)pt_security_key,
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

/*******************************************************************************
 * FUNCTION: pt_upgrade_ttconfig
 *
 * SUMMARY: Program ttconfig_data with following steps:
 *  1) Suspend scanning
 *  2) Write data to the data block
 *  3) Verify the crc for data block
 *  4) Resume scanning
 *  5) Set up call back for calibration if required
 *
 * RETURN:
 *   0 = success
 *  !0 = failure
 *
 * PARAMETERS:
 *  *dev            - pointer to device structure
 *  *ttconfig_data  - pointer to the config data to write to data block
 *   ttconfig_size  - size of config data to write
 ******************************************************************************/
static int pt_upgrade_ttconfig(struct device *dev,
		const u8 *ttconfig_data, int ttconfig_size)
{
	struct pt_loader_data *ld = pt_get_loader_data(dev);
	bool wait_for_calibration_complete = false;
	u8 ebid = PT_TCH_PARM_EBID;
	u16 row_size = PT_DATA_ROW_SIZE;
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

	rc = cmd->request_exclusive(dev, PT_LDR_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0)
		goto exit;

	rc = cmd->nonhid_cmd->suspend_scanning(dev, 0);
	if (rc < 0)
		goto release;

	for (i = 0; i < row_count; i++) {
		pt_debug(dev, DL_INFO, "%s: row=%d size=%d\n",
			__func__, i, row_size);
		rc = pt_write_config_row_(dev, ebid, i, row_size,
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
		rc = pt_write_config_row_(dev, ebid, i, residue,
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

	rc = cmd->nonhid_cmd->verify_cfg_block_crc(dev, 0, ebid,
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
			 & PT_LOADER_FLAG_CALIBRATE_AFTER_TTCONFIG_UPGRADE)) {
#if (KERNEL_VERSION(3, 13, 0) <= LINUX_VERSION_CODE)
		reinit_completion(&ld->calibration_complete);
#else
		INIT_COMPLETION(ld->calibration_complete);
#endif
		/* set up call back for startup */
		pt_debug(dev, DL_INFO, "%s: Adding callback for calibration\n",
			__func__);
		rc = cmd->subscribe_attention(dev, PT_ATTEN_STARTUP,
			PT_LOADER_NAME, pt_calibration_attention, 0);
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
#endif /* PT_TTCONFIG_UPGRADE */

#ifdef CONFIG_TOUCHSCREEN_PARADE_PLATFORM_TTCONFIG_UPGRADE
/*******************************************************************************
 * FUNCTION: pt_get_ttconfig_crc
 *
 * SUMMARY: Get crc from ttconfig data.
 *
 * RETURN:
 *   0 = success
 *  !0 = failure
 *
 * PARAMETERS:
 *  *dev            - pointer to device structure
 *  *ttconfig_data  - pointer to the config data
 *   ttconfig_size  - size of config data
 *  *crc            - pointer to the crc of configure to be stored
 ******************************************************************************/
static int pt_get_ttconfig_crc(struct device *dev,
		const u8 *ttconfig_data, int ttconfig_size, u16 *crc)
{
	u16 crc_loc;

	crc_loc = get_unaligned_le16(&ttconfig_data[2]);
	if (ttconfig_size < crc_loc + 2)
		return -EINVAL;

	*crc = get_unaligned_le16(&ttconfig_data[crc_loc]);

	return 0;
}

/*******************************************************************************
 * FUNCTION: pt_get_ttconfig_version
 *
 * SUMMARY: Get version number from ttconfig data.
 *
 * RETURN:
 *   0 = success
 *  !0 = failure
 *
 * PARAMETERS:
 *  *dev            - pointer to device structure
 *  *ttconfig_data  - pointer to the config data
 *   ttconfig_size  - size of config data
 *  *version        - pointer to the version of configure to be stored
 ******************************************************************************/
static int pt_get_ttconfig_version(struct device *dev,
		const u8 *ttconfig_data, int ttconfig_size, u16 *version)
{
	if (ttconfig_size < PT_TTCONFIG_VERSION_OFFSET
			+ PT_TTCONFIG_VERSION_SIZE)
		return -EINVAL;

	*version = get_unaligned_le16(
		&ttconfig_data[PT_TTCONFIG_VERSION_OFFSET]);

	return 0;
}

/*******************************************************************************
 * FUNCTION: pt_check_ttconfig_version
 *
 * SUMMARY: Check the configure version and crc value to determine whether to
 *  upgrade,the upgrade conditions as followings:
 *      1) To upgrade if the config version is newer than current config, but
 *         this check is based on the flag in loader plarform data.
 *      2) To upgrade if config CRC is different.
 *      3) Don't upgrade when can't match any of above conditions.
 *
 * RETURN:
 *   0: Don't upgrade
 *   1: Upgrade
 *
 * PARAMETERS:
 *  *dev            - pointer to device structure
 *  *ttconfig_data  - pointer to the config data
 *   ttconfig_size  - size of config data
 ******************************************************************************/
static int pt_check_ttconfig_version(struct device *dev,
		const u8 *ttconfig_data, int ttconfig_size)
{
	struct pt_loader_data *ld = pt_get_loader_data(dev);
	u16 cfg_crc_new;
	int rc;

	if (!ld->si)
		return 0;

	/* Check for config version */
	if (ld->loader_pdata->flags &
			PT_LOADER_FLAG_CHECK_TTCONFIG_VERSION) {
		u16 cfg_ver_new;

		rc = pt_get_ttconfig_version(dev, ttconfig_data,
				ttconfig_size, &cfg_ver_new);
		if (rc)
			return 0;

		pt_debug(dev, DL_INFO, "%s: img_ver:0x%04X new_ver:0x%04X\n",
			__func__, ld->si->ttdata.fw_ver_conf, cfg_ver_new);

		/* Check if config version is newer */
		if (cfg_ver_new > ld->si->ttdata.fw_ver_conf) {
			pt_debug(dev, DL_WARN,
			"%s: Config version newer, will upgrade\n", __func__);
			return 1;
		}

		pt_debug(dev, DL_WARN,
			"%s: Config version is identical or older, will NOT upgrade\n",
			__func__);
	/* Check for config CRC */
	} else {
		rc = pt_get_ttconfig_crc(dev, ttconfig_data,
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

/*******************************************************************************
 * FUNCTION: pt_check_ttconfig_version_platform
 *
 * SUMMARY: To call the function pt_check_ttconfig_version() to determine
 *  whether to load config if the firmware version match with current firmware.
 *
 * RETURN:
 *   0: Don't upgrade
 *   1: Upgrade
 *
 * PARAMETERS:
 *  *dev       - pointer to device structure
 *  *ttconfig  - pointer to touch_config structure
 ******************************************************************************/
static int pt_check_ttconfig_version_platform(struct device *dev,
		struct pt_touch_config *ttconfig)
{
	struct pt_loader_data *ld = pt_get_loader_data(dev);
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
	if (pt_check_firmware_version(dev, fw_ver_config,
			fw_revctrl_config)) {
		pt_debug(dev, DL_ERROR,
			"%s: FW versions mismatch\n", __func__);
		return 0;
	}

	/* Check PowerOn Self Test, TT_CFG CRC bit */
	if ((ld->si->ttdata.post_code & PT_POST_TT_CFG_CRC_MASK) == 0) {
		pt_debug(dev, DL_ERROR,
			"%s: POST, TT_CFG failed (%X), will upgrade\n",
			__func__, ld->si->ttdata.post_code);
		return 1;
	}

	return pt_check_ttconfig_version(dev, ttconfig->param_regs->data,
			ttconfig->param_regs->size);
}

/*******************************************************************************
 * FUNCTION: pt_get_platform_ttconfig
 *
 * SUMMARY: To get the pointer of right touch_config structure by panel id.
 *
 * RETURN:
 *   pointer to touch_config structure or null pointer if fail
 *
 * PARAMETERS:
 *  *dev  - pointer to device structure
 ******************************************************************************/
static struct pt_touch_config *pt_get_platform_ttconfig(
		struct device *dev)
{
	struct pt_loader_data *ld = pt_get_loader_data(dev);
	struct pt_touch_config **ttconfigs;
	struct pt_touch_config *ttconfig;
	u8 panel_id;

	panel_id = pt_get_panel_id(dev);
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

/*******************************************************************************
 * FUNCTION: upgrade_ttconfig_from_platform
 *
 * SUMMARY: Get touch_firmware structure and perform upgrade if pass the
 *  firmware version check.
 *
 * RETURN:
 *   0 = success
 *  !0 = failure
 *
 * PARAMETERS:
 *  *dev     - pointer to device structure
 *   forced  - flag to force upgrade(1:force to upgrade)
 ******************************************************************************/
static int upgrade_ttconfig_from_platform(struct device *dev)
{
	struct pt_loader_data *ld = pt_get_loader_data(dev);
	struct pt_touch_config *ttconfig;
	struct touch_settings *param_regs;

	int rc = -ENODEV;
	int upgrade;

	if (!ld->loader_pdata) {
		pt_debug(dev, DL_ERROR,
			"%s: No loader platform data\n", __func__);
		return rc;
	}

	ttconfig = pt_get_platform_ttconfig(dev);
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

	upgrade = pt_check_ttconfig_version_platform(dev, ttconfig);
	if (upgrade)
		return pt_upgrade_ttconfig(dev, param_regs->data,
				param_regs->size);

	return rc;
}
#endif /* CONFIG_TOUCHSCREEN_PARADE_PLATFORM_TTCONFIG_UPGRADE */

#ifdef CONFIG_TOUCHSCREEN_PARADE_MANUAL_TTCONFIG_UPGRADE
/*******************************************************************************
 * FUNCTION: pt_config_data_write
 *
 * SUMMARY: The write method for the config_data sysfs node. The passed
 *  in data (config file) is written to the config_data buffer.
 *
 * RETURN: Size of passed in buffer is success
 *
 * PARAMETERS:
 *  *filp     - pointer to file structure
 *  *kobj     - pointer to kobject structure
 *  *bin_attr - pointer to bin_attribute structure
 *   buf      - pointer to cmd input buffer
 *   offset   - offset index to store input buffer
 *   count    - size of data in buffer
 ******************************************************************************/
static ssize_t pt_config_data_write(struct file *filp,
		struct kobject *kobj, struct bin_attribute *bin_attr,
		char *buf, loff_t offset, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct pt_loader_data *data = pt_get_loader_data(dev);
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
	.write = pt_config_data_write,
};

/*******************************************************************************
 * FUNCTION: pt_config_loading_show
 *
 * SUMMARY: The show method for config_loading sysfs node. This node
 *  displays the stored value that was written to this node.
 *
 * PARAMETERS:
 *      *dev  - pointer to Device structure
 *      *attr - pointer to the device attribute structure
 *      *buf  - pointer to buffer to print
 ******************************************************************************/
static ssize_t pt_config_loading_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct pt_loader_data *ld = pt_get_loader_data(dev);
	bool config_loading;

	mutex_lock(&ld->config_lock);
	config_loading = ld->config_loading;
	mutex_unlock(&ld->config_lock);

	return sprintf(buf, "%d\n", config_loading);
}

/*******************************************************************************
 * FUNCTION: pt_verify_ttconfig_binary
 *
 * SUMMARY: Perform a simple size check if the firmware version match.And
 *  calculate the start pointer of config data to write and the size to write.
 *
 * RETURN:
 *   0 = success
 *  !0 = failure
 *
 * PARAMETERS:
 *   *dev              - pointer to device structure
 *   *bin_config_data  - pointer to binary config data
 *    bin_config_size  - size of binary config data
 *  **start            - double pointer to config data where to be written
 *   *len              - pointer to the size of config data to store
 ******************************************************************************/
static int pt_verify_ttconfig_binary(struct device *dev,
		u8 *bin_config_data, int bin_config_size, u8 **start, int *len)
{
	struct pt_loader_data *ld = pt_get_loader_data(dev);
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
	if (pt_check_firmware_version(dev, fw_ver_config,
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

/*******************************************************************************
 * FUNCTION: pt_config_loading_store
 *
 * SUMMARY: The store method for the config_loading sysfs node. The
 *  passed in value controls if config loading is performed.
 *
 * RETURN: Size of passed in buffer is success
 *
 * PARAMETERS:
 *  *dev  - pointer to device structure
 *  *attr - pointer to device attributes
 *  *buf  - pointer to buffer that hold the command parameters
 *   size - size of buf
 ******************************************************************************/
static ssize_t pt_config_loading_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct pt_loader_data *ld = pt_get_loader_data(dev);
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

		rc = pt_verify_ttconfig_binary(dev,
				ld->config_data, ld->config_size,
				&start, &length);
		if (rc)
			goto exit_free;

		rc = pt_upgrade_ttconfig(dev, start, length);
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

static DEVICE_ATTR(config_loading, S_IRUGO | S_IWUSR,
	pt_config_loading_show, pt_config_loading_store);
#endif /* CONFIG_TOUCHSCREEN_PARADE_MANUAL_TTCONFIG_UPGRADE */

#ifdef CONFIG_TOUCHSCREEN_PARADE_PLATFORM_FW_UPGRADE
/*******************************************************************************
 * FUNCTION: pt_forced_upgrade_store
 *
 * SUMMARY: The store method for the forced_upgrade sysfs node. The firmware
 *  loading is forced to performed with platform upgrade strategy.
 *
 * RETURN: Size of passed in buffer is success
 *
 * PARAMETERS:
 *  *dev  - pointer to device structure
 *  *attr - pointer to device attributes
 *  *buf  - pointer to buffer that hold the command parameters
 *   size - size of buf
 ******************************************************************************/
static ssize_t pt_forced_upgrade_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int rc = upgrade_firmware_from_platform(dev, true);

	if (rc)
		return rc;
	return size;
}

static DEVICE_ATTR(forced_upgrade, S_IWUSR,
	NULL, pt_forced_upgrade_store);
#endif

#ifdef CONFIG_TOUCHSCREEN_PARADE_BINARY_FW_UPGRADE
/*******************************************************************************
 * FUNCTION: pt_manual_upgrade_store
 *
 * SUMMARY: The store method for the forced_upgrade sysfs node that it is
 *  caller for function upgrade_firmware_from_class() to allow upgrade firmware
 *  manually.
 *
 * RETURN: Size of passed in buffer is success
 *
 * PARAMETERS:
 *  *dev  - pointer to device structure
 *  *attr - pointer to device attributes
 *  *buf  - pointer to buffer that hold the command parameters
 *   size - size of buf
 ******************************************************************************/
static ssize_t pt_manual_upgrade_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct pt_loader_data *ld = pt_get_loader_data(dev);
	int rc;

	if (ld->is_manual_upgrade_enabled)
		return -EBUSY;

	ld->is_manual_upgrade_enabled = 1;

	rc = upgrade_firmware_from_class(ld->dev);

	if (rc < 0)
		ld->is_manual_upgrade_enabled = 0;

	return size;
}

static DEVICE_ATTR(manual_upgrade, S_IWUSR, NULL, pt_manual_upgrade_store);
#endif
/*******************************************************************************
 * FUNCTION: pt_fw_and_config_upgrade
 *
 * SUMMARY: Perform all methods for firmware upgrade and config upgrade
 *  according to the definition of macro.
 *
 * PARAMETERS:
 *  *work_struct  - pointer to work_struct structure
 ******************************************************************************/
static void pt_fw_and_config_upgrade(
		struct work_struct *fw_and_config_upgrade)
{
	struct pt_loader_data *ld = container_of(fw_and_config_upgrade,
			struct pt_loader_data, fw_and_config_upgrade);
	struct device *dev = ld->dev;
#if (PT_FW_UPGRADE || PT_TTCONFIG_UPGRADE)
	u8 dut_gen = cmd->request_dut_generation(dev);
#endif
	ld->si = cmd->request_sysinfo(dev);
	if (!ld->si)
		pt_debug(dev, DL_ERROR,
			"%s: Fail get sysinfo pointer from core\n",
			__func__);
#if !PT_FW_UPGRADE
	pt_debug(dev, DL_INFO,
		"%s: No FW upgrade method selected!\n", __func__);
#endif

#ifdef CONFIG_TOUCHSCREEN_PARADE_PLATFORM_FW_UPGRADE
	if (dut_gen == DUT_PIP1_ONLY) {
		if (!upgrade_firmware_from_platform(dev, false))
			return;
	}
#endif

#ifdef CONFIG_TOUCHSCREEN_PARADE_BINARY_FW_UPGRADE
	if (dut_gen == DUT_PIP2_CAPABLE) {
		if (!pt_pip2_upgrade_firmware_from_builtin(dev)) {
			pt_debug(dev, DL_WARN, "%s: Builtin FW upgrade failed",
				__func__);
			return;
		}
	} else {
		if (!upgrade_firmware_from_builtin(dev))
			return;
	}
#endif

#ifdef CONFIG_TOUCHSCREEN_PARADE_PLATFORM_TTCONFIG_UPGRADE
	if (dut_get == DUT_PIP1_ONLY) {
		if (!upgrade_ttconfig_from_platform(dev))
			return;
	}
#endif
}

#ifdef CONFIG_TOUCHSCREEN_PARADE_BINARY_FW_UPGRADE
/*******************************************************************************
 * FUNCTION: _pt_pip2_get_flash_info
 *
 * SUMMARY: Sends a FLASH_INFO command to the DUT logging the results to kmsg
 *
 * PARAMETERS:
 *	*dev - pointer to device structure
 *	*pip2_cmd - pointer to the PIP2 command structure
 *	*read_buf - pointer to the read buffer array to store the response
 ******************************************************************************/
static void _pt_pip2_get_flash_info(struct device *dev,
	struct pip2_cmd_structure *pip2_cmd, u8 *read_buf)
{
	u16 actual_read_len;
	int ret;

	/* Get flash info for debugging information */
	ret = cmd->nonhid_cmd->pip2_send_cmd(dev,
		PT_CORE_CMD_UNPROTECTED, pip2_cmd,
		PIP2_CMD_ID_FLASH_INFO, NULL, 0, read_buf, &actual_read_len);
	if (!ret) {
		pt_debug(dev, DL_DEBUG,
			"%s --- FLASH Information ---\n", __func__);
		pt_debug(dev, DL_DEBUG,
			"%s Manufacturer ID: 0x%02x\n",
			__func__, read_buf[PIP2_RESPONSE_BODY_OFFSET]);
		pt_debug(dev, DL_DEBUG,
			"%s Memory Type    : 0x%02x\n",
			__func__, read_buf[PIP2_RESPONSE_BODY_OFFSET + 1]);
		pt_debug(dev, DL_DEBUG,
			"%s Num Sectors    : 0x%02x%02x%02x%02x\n",
			__func__, read_buf[PIP2_RESPONSE_BODY_OFFSET + 2],
			read_buf[PIP2_RESPONSE_BODY_OFFSET + 3],
			read_buf[PIP2_RESPONSE_BODY_OFFSET + 4],
			read_buf[PIP2_RESPONSE_BODY_OFFSET + 5]);
		pt_debug(dev, DL_DEBUG,
			"%s Sectors Size   : 0x%02x%02x%02x%02x\n",
			__func__, read_buf[PIP2_RESPONSE_BODY_OFFSET + 6],
			read_buf[PIP2_RESPONSE_BODY_OFFSET + 7],
			read_buf[PIP2_RESPONSE_BODY_OFFSET + 8],
			read_buf[PIP2_RESPONSE_BODY_OFFSET + 9]);
		pt_debug(dev, DL_DEBUG,
			"%s Page Size      : 0x%02x%02x%02x%02x\n",
			__func__, read_buf[PIP2_RESPONSE_BODY_OFFSET + 10],
			read_buf[PIP2_RESPONSE_BODY_OFFSET + 11],
			read_buf[PIP2_RESPONSE_BODY_OFFSET + 12],
			read_buf[PIP2_RESPONSE_BODY_OFFSET + 13]);
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
 ******************************************************************************/
static void _pt_pip2_log_last_error(struct device *dev,
	struct pip2_cmd_structure *pip2_cmd, u8 *read_buf)
{
	u16 actual_read_len;
	u8 loop = 5;
	u8 info = 0xFF;
	u8 error = 0xFF;
	int ret;

	/* Send the GET_LAST_ERROR command to get the last BL startup error */
	ret = cmd->nonhid_cmd->pip2_send_cmd(dev,
		PT_CORE_CMD_UNPROTECTED, pip2_cmd,
		PIP2_CMD_ID_GET_LAST_ERRNO, NULL, 0, read_buf,
		&actual_read_len);
	if (!ret) {
		pt_debug(dev, DL_ERROR,
			"%s: GET_LAST_ERR: Status=0x%02X ERRNO=0x%02X BOOTMODE=%d\n",
			__func__,
			(u8)read_buf[PIP2_RESPONSE_STATUS_OFFSET],
			(u8)read_buf[PIP2_RESPONSE_BODY_OFFSET],
			(u8)read_buf[PIP2_RESPONSE_BODY_OFFSET + 1]);
	}

	/*
	 * Send the STATUS command until no errors are found.
	 * The BL will store an error code for each layer of the stack,
	 * and each read will return one error.
	 */
	while (loop > 0 && error) {

		ret = cmd->nonhid_cmd->pip2_send_cmd(dev,
			PT_CORE_CMD_UNPROTECTED, pip2_cmd,
			PIP2_CMD_ID_STATUS, NULL, 0, read_buf,
			&actual_read_len);

		if (!ret) {
			info  = (u8)read_buf[PIP2_RESPONSE_BODY_OFFSET];
			error = (u8)read_buf[PIP2_RESPONSE_BODY_OFFSET + 1];

			pt_debug(dev, DL_ERROR,
				"%s: STATUS: Status=0x%02X BOOT=%d BUSY=%d INT=%d ERR_PHY=%d ERR_REG=%d ERROR=0x%02X",
				__func__,
				(u8)read_buf[PIP2_RESPONSE_STATUS_OFFSET],
				info & 0x01,
				(info & 0x02) >> 1,
				(info & 0x04) >> 2,
				(info & 0x18) >> 3,
				(info & 0xE0) >> 5,
				error);
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
 *	NOTE: The len_per_packet is set so that the overall packet size is
 *		less than 255. The overhead is 9 bytes: 2 byte address (0101),
 *		4 byte header, 1 byte file no. 2 byte CRC
 *
 * PARAMETERS:
 *	*fw      - pointer to the new FW image to load
 *	*context - pointer to the device
 ******************************************************************************/
static void _pt_pip2_firmware_cont(const struct firmware *fw,
		void *context)
{
	u8 file_handle;
	u8 read_buf[255];
	u8 *fw_img = 0;
	u8 buf[255];
	u8 len_per_packet = 245;
	u8 write_len;
	u8 data[20];
	u8 img_app_major_ver;
	u8 img_app_minor_ver;
	u32 img_app_rev_ctrl;
	u8 mode;
	u8 retry_packet = 0;
	u8 attempt = 0;
	u16 actual_read_len;
	u16 status;
	int fw_size = 0;
	int remain_bytes;
	int ret = 0;
	int upgrade;
	int percent_cmplt;
	bool wait_for_calibration_complete = false;
	struct device *dev = context;
	struct pip2_cmd_structure pip2_cmd;
	struct pt_loader_data *ld = pt_get_loader_data(dev);
	struct pip2_loader_data *pip2_data = ld->pip2_data;
	struct pt_core_data *cd = dev_get_drvdata(dev);
	struct pt_bin_file_hdr hdr;

	pt_debug(dev, DL_DEBUG, "Entering function %s...\n", __func__);

	/* Loading status start at 1% complete */
	pip2_bl_status = PIP2_BL_STATUS_ACTIVE_1;
	pt_debug(dev, DL_WARN, "%s: Begin BL\n", __func__);

	if (!fw) {
		if (ld->pip2_load_builtin) {
			pt_debug(dev, DL_ERROR,
				"%s: No builtin firmware\n", __func__);
			ld->builtin_bin_fw_status = -EINVAL;
		} else {
			pt_debug(dev, DL_ERROR,
				"%s: No firmware provided to load\n", __func__);
		}
		pt_debug(dev, DL_ERROR, "%s: Exit BL\n", __func__);
		goto pt_firmware_cont_release_exit;
	}

	if (!fw->data || !fw->size) {
		pt_debug(dev, DL_ERROR,
			"%s: Invalid builtin firmware\n", __func__);
		goto pt_firmware_cont_release_exit;
	}

	pip2_bl_status += 1;
	pt_debug(dev, DL_WARN,
		"%s: Found firmware of size:%d bytes\n",
		__func__, (int)fw->size);

	pm_runtime_get_sync(dev);

	ret = cmd->request_exclusive(dev, PT_LDR_REQUEST_EXCLUSIVE_TIMEOUT);
	if (ret < 0)
		goto pt_firmware_cont_release_exit;

	/* Wait for completion of FW upgrade thread before continuing */
	if (!ld->pip2_load_builtin)
		init_completion(&pip2_data->pip2_fw_upgrade_complete);

	/* Force DUT to enter the BL */
	pip2_bl_status += 1;
	ret = cmd->request_pip2_enter_bl(dev, &mode);
	if (ret) {
		pt_debug(dev, DL_ERROR, "%s: Failed to enter BL\n", __func__);
		pip2_bl_status = PIP2_BL_STATUS_ENTER_BL_ERROR;
		goto exit;
	}
	pip2_bl_status += 1;

	/* Only compare FW versions when doing a built-in BL */
	if (ld->pip2_load_builtin) {
		img_app_major_ver = fw->data[3];
		img_app_minor_ver = fw->data[4];
		img_app_rev_ctrl  = fw->data[9]<<24 | fw->data[10]<<16 |
				    fw->data[11]<<8 | fw->data[12];
		pt_debug(dev, DL_WARN,
			"%s BL Image Version:   %02x.%02x.%d\n",
			__func__, img_app_major_ver, img_app_minor_ver,
			img_app_rev_ctrl);

		ret = cmd->request_pip2_bin_hdr(dev, &hdr);
		if (ret != 0) {
			pt_debug(dev, DL_WARN,
				"App ver info not available, will upgrade FW");
			upgrade = 1;
			goto prepare_upgrade;
		}
		pt_debug(dev, DL_WARN,
			"%s Current FW Version: %02X.%02X.%d\n",
			__func__, hdr.fw_major, hdr.fw_minor, hdr.fw_rev_ctrl);
		pt_debug(dev, DL_WARN,
			"%s BL Image Version:   %02x.%02x.%d\n",
			__func__, img_app_major_ver, img_app_minor_ver,
			img_app_rev_ctrl);

		if ((256 * img_app_major_ver + img_app_minor_ver) <
		    (256 * hdr.fw_major + hdr.fw_minor)) {
			pt_debug(dev, DL_WARN,
				"bin file version <=, will not upgrade FW");
			pip2_bl_status = PIP2_BL_STATUS_FW_VERSION_ERROR;
			goto exit;
		} else if (img_app_rev_ctrl <= hdr.fw_rev_ctrl) {
			pt_debug(dev, DL_WARN,
				"bin file rev ctrl <=, will not upgrade FW");
			pip2_bl_status = PIP2_BL_STATUS_FW_VERSION_ERROR;
			goto exit;
		} else
			upgrade = 1;
	} else
		/* Always do a manual upgrade */
		upgrade = 1;

prepare_upgrade:
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

	ret = cmd->nonhid_cmd->pip2_send_cmd(dev,
		PT_CORE_CMD_UNPROTECTED, &pip2_cmd,
		PIP2_CMD_ID_FILE_OPEN, data, 1, read_buf, &actual_read_len);
	status      = read_buf[PIP2_RESPONSE_STATUS_OFFSET];
	file_handle = read_buf[PIP2_RESPONSE_BODY_OFFSET];
	if (ret || ((status != 0x00) && (status != 0x03)) || file_handle > 1) {
		pt_debug(dev, DL_ERROR,
			"%s File open failure with file = %d\n",
			__func__, file_handle);
		ret = -1;
		pip2_bl_status = PIP2_BL_STATUS_FILE_OPEN_ERROR;
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
		ret = cmd->nonhid_cmd->pip2_send_cmd(dev,
			PT_CORE_CMD_UNPROTECTED, &pip2_cmd,
			PIP2_CMD_ID_FILE_IOCTL, data, 2, read_buf,
			&actual_read_len);
		status = read_buf[PIP2_RESPONSE_STATUS_OFFSET];
		if (ret || status) {
			pt_debug(dev, DL_ERROR,
				"%s: File erase failure rc=%d status=%d\n",
				__func__, ret, status);
			ret = -1;
			pip2_bl_status = PIP2_BL_STATUS_ERASE_ERROR;
			goto exit;
		}
	pt_debug(dev, DL_INFO,
			"%s File erase successful\n", __func__);
	}

	if (upgrade) {
		u8 header_size = 0;

		if (!fw)
			goto pt_firmware_cont_release_exit;
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

		/* Continue writing while data remains */
		while (remain_bytes > len_per_packet) {
			write_len = len_per_packet;

			/* Don't print debug msg on every pass */
			if (remain_bytes % 1000 < len_per_packet) {
				/* Calculate % complete for bl_status sysfs */
				percent_cmplt = (fw_size - remain_bytes) *
						100 / fw_size;
				pip2_bl_status = (percent_cmplt >
					pip2_bl_status) ? percent_cmplt :
					pip2_bl_status;

				pt_debug(dev, DL_WARN,
					"Wrote %d of %d bytes to File 0x%02x\n",
					fw_size - remain_bytes - write_len,
					fw_size, file_handle);
			}
			if (retry_packet > 0) {
#ifdef TTDL_DIAGNOSTICS
				cd->bl_retry_packet_count++;
				pt_debug(dev, DL_WARN,
					"%s: === Retry Packet #%d ===\n",
					__func__, retry_packet);
#endif
				/* Get and log the last error(s) */
				_pt_pip2_log_last_error(dev, &pip2_cmd,
					read_buf);
			}

			/* Send the file write cmd as a PIP2 command */
			pip2_data->pip2_file_handle = file_handle;
			memcpy(&buf[1], fw_img, write_len);
			ret = cmd->nonhid_cmd->pip2_send_cmd(dev,
				PT_CORE_CMD_UNPROTECTED, &pip2_cmd,
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
				pip2_bl_status = PIP2_BL_STATUS_WRITE_ERROR;
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
			ret = cmd->nonhid_cmd->pip2_send_cmd(dev,
				PT_CORE_CMD_UNPROTECTED, &pip2_cmd,
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
					pip2_bl_status =
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
	ret = cmd->nonhid_cmd->pip2_send_cmd(dev,
		PT_CORE_CMD_UNPROTECTED, &pip2_cmd,
		PIP2_CMD_ID_FILE_CLOSE, data, 1, read_buf, &actual_read_len);
	status = read_buf[PIP2_RESPONSE_STATUS_OFFSET];
	if (ret || status) {
		pt_debug(dev, DL_ERROR,
			"%s file close failure\n", __func__);
		pip2_bl_status = PIP2_BL_STATUS_FILE_CLOSE_ERROR;
		goto exit;
	}

	if (retry_packet >= 3) {
		/* A packet write failure occured 3x */
		pt_debug(dev, DL_ERROR,
			"%s: BIN file write terminated due to consecutive write errors\n",
			__func__);
		pip2_bl_status = PIP2_BL_STATUS_WRITE_ERROR;
		goto exit;
	} else
		pt_debug(dev, DL_INFO,
			"%s: BIN file write finished successfully\n", __func__);

	pip2_bl_status = PIP2_BL_STATUS_ACTIVE_99;
	if (ld->pip2_load_fw_to_ram) {
		/* When writing to RAM don't reset, just launch application */
		pt_debug(dev, DL_WARN,
			"%s Sending execute command now...\n", __func__);
		ret = cmd->nonhid_cmd->pip2_send_cmd(dev,
			PT_CORE_CMD_UNPROTECTED, &pip2_cmd,
			PIP2_CMD_ID_EXECUTE, NULL, 0, read_buf,
			&actual_read_len);
		status = read_buf[PIP2_RESPONSE_STATUS_OFFSET];
		if (ret || status) {
			pt_debug(dev, DL_ERROR,
				"%s Execute command failure\n", __func__);
			pip2_bl_status = PIP2_BL_STATUS_EXECUTE_ERROR;
			goto exit;
		}
	} else {
		/* Do a reset if writing to FLASH */
		pt_debug(dev, DL_WARN,
			"%s Toggle TP_XRES now...\n", __func__);
		cmd->request_reset(dev, PT_CORE_CMD_UNPROTECTED);
	}

	/* Wait to see the FW reset sentinel for up to 500ms */
	while (attempt++ < 25) {
		msleep(20);
		pt_debug(dev, DL_WARN,
			"%s: 0x%04X Waiting for FW sentinel for %dms",
			__func__, cd->startup_status, attempt*20);
		if (cd->startup_status & STARTUP_STATUS_FW_RESET_SENTINEL)
			break;
	}

	/* Double verify DUT is alive and well in Application mode */
	if (cd->startup_status & STARTUP_STATUS_FW_RESET_SENTINEL) {
		ret = cmd->request_pip2_get_mode(dev, PT_CORE_CMD_UNPROTECTED,
			&mode);
		pt_debug(dev, DL_WARN,
			"%s: Expecting App mode (2), Got Mode = %d",
			__func__, mode);
		if (mode != PT_MODE_OPERATIONAL) {
			pt_debug(dev, DL_ERROR,
				"%s ERROR: Not in App mode as expected\n",
				__func__);
			pip2_bl_status = PIP2_BL_STATUS_MODE_ERROR;
			goto exit;
		}
	} else {
		pt_debug(dev, DL_ERROR,
			"%s FW sentinel not seen\n", __func__);
		/* Get and log the last error(s) */
		_pt_pip2_log_last_error(dev, &pip2_cmd, read_buf);
		pip2_bl_status = PIP2_BL_STATUS_FW_SENTINEL_NOT_SEEN;
		goto exit;
	}

	pt_debug(dev, DL_INFO,
		"%s === PIP2 FW upgrade finished ===\n", __func__);

	/* Either RESET or LAUNCH APP has occurred so definately not in BL */
	pip2_bl_status = PIP2_BL_STATUS_COMPLETE;

	/* Subscribe calibration task if calibration flag is set */
	if (ld->loader_pdata &&	(ld->loader_pdata->flags
			 & PT_LOADER_FLAG_CALIBRATE_AFTER_FW_UPGRADE)) {
#if (KERNEL_VERSION(3, 13, 0) <= LINUX_VERSION_CODE)
		reinit_completion(&ld->calibration_complete);
#else
		INIT_COMPLETION(ld->calibration_complete);
#endif
		/* set up call back for startup */
		pt_debug(dev, DL_INFO, "%s: Adding callback for calibration\n",
			__func__);
		ret = cmd->subscribe_attention(dev, PT_ATTEN_STARTUP,
			PT_LOADER_NAME, pt_calibration_attention, 0);
		if (ret) {
			pt_debug(dev, DL_ERROR,
				"%s: Failed adding callback for calibration\n",
				__func__);
			pt_debug(dev, DL_ERROR,
				"%s: No calibration will be performed\n",
				__func__);
			ret = 0;
		} else
			wait_for_calibration_complete = true;
	}

exit:
	/* Ensure the boot mode pin is released */
	pt_debug(dev, DL_WARN,
		"%s Disabling host mode pin\n", __func__);
	cmd->request_set_runfw_pin(dev, 1);
	cmd->release_exclusive(dev);

pt_firmware_cont_release_exit:
	pm_runtime_put_sync(dev);
	if (fw)
		release_firmware(fw);
	if (ld->pip2_load_builtin) {
		if (pip2_bl_status == PIP2_BL_STATUS_COMPLETE)
			ld->builtin_bin_fw_status = 0;
		else
			ld->builtin_bin_fw_status = -EINVAL;
	}

	/* Issue a restart if a load took place to get HID descriptor etc. */
	if (!ld->builtin_bin_fw_status) {
		pt_debug(dev, DL_WARN, "%s Requesting RESTART\n", __func__);
		cmd->request_restart(dev, true);
	}

	if (wait_for_calibration_complete)
		wait_for_completion(&ld->calibration_complete);

	/*
	 * Start watchdog, temporarily override the WD interval to 500ms to
	 * force the WD to fire sooner and therefor re-enumerating the DUT
	 * with TTDL quicker. The WD work function ensure the WD interval
	 * is returned back to it's original value.
	 */
	cd->watchdog_interval = 500;
	pt_debug(dev, DL_WARN, "%s Starting watchdog\n", __func__);
	ret = cmd->request_start_wd(dev);
}

/*******************************************************************************
 * FUNCTION: pt_pip2_upgrade_firmware_from_builtin
 *
 * SUMMARY: Bootload the DUT with a built in firmware binary image.
 *	Load either a SRAM image "ttdl_fw_RAM.bin" or a FLASH image
 *	"ttdl_fw.bin" with the priority being the SRAM image.
 *
 * PARAMETERS:
 *	*dev - pointer to the device structure
 ******************************************************************************/
static int pt_pip2_upgrade_firmware_from_builtin(struct device *dev)
{
	struct pt_loader_data *ld = pt_get_loader_data(dev);
	struct pt_core_data *cd = dev_get_drvdata(dev);
	const struct firmware *fw_entry = NULL;
	int retval;

	pt_debug(dev, DL_INFO,
		"%s: Enabling firmware class loader built-in\n", __func__);
	ld->pip2_load_builtin = true;

	mutex_lock(&cd->firmware_class_lock);

	/* Try to load the RAM image first if it exists */
	retval = request_firmware(&fw_entry, PT_FW_RAM_FILE_NAME, dev);
	if (retval < 0) {
		ld->pip2_load_fw_to_ram = false;
		retval = request_firmware(&fw_entry, PT_FW_FILE_NAME, dev);
		if (retval < 0) {
			mutex_unlock(&cd->firmware_class_lock);
			pt_debug(dev, DL_ERROR,
				"%s: Fail request firmware class file load\n",
				__func__);
			goto exit;
		}
	} else
		ld->pip2_load_fw_to_ram = true;

	_pt_pip2_firmware_cont(fw_entry, dev);

	mutex_unlock(&cd->firmware_class_lock);
	retval = ld->builtin_bin_fw_status;
exit:
	return retval;
}

/*******************************************************************************
 * FUNCTION: pt_pip2_do_fw_upgrade
 *
 * SUMMARY: Create the firmware class but don't actually laod any FW to the
 *	DUT. This creates all the sysfs nodes needed for a user to bootload
 *	the DUT with their own bin file.
 *
 * PARAMETERS:
 *	*pip2_data     - pointer to the PIP2 loader data structure
 *	 force_upgrade - Force the upgrade even if version in DUT is less
 ******************************************************************************/
static int pt_pip2_do_fw_upgrade(struct pip2_loader_data *pip2_data,
	bool force_upgrade)
{

	int ret = 0;
	struct device *dev = pip2_data->dev;
	struct pt_loader_data *ld = pt_get_loader_data(dev);

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
 *       size   - size of data in buffer
 ******************************************************************************/
static ssize_t pt_pip2_manual_upgrade_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct pt_loader_data *ld = pt_get_loader_data(dev);
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
 * FUNCTION: pt_update_fw_store
 *
 * SUMMARY: Store method for the update_fw sysfs node. This node is required
 *	by ChromeOS to first determine if loading is available and then perform
 *	the loading if required. This function is simply a wrapper to call:
 *		pt_pip2_manual_upgrade_store - for the TC3XXX or TT7XXX parts
 *		pt_manual_upgrade_store - for the legacy Gen5/6 devices.
 *
 * PARAMETERS:
 *      *dev   - pointer to device structure
 *      *attr  - pointer to device attributes
 *      *buf   - pointer to output buffer
 *       size   - size of data in buffer
 ******************************************************************************/
static ssize_t pt_update_fw_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	u8 dut_gen = cmd->request_dut_generation(dev);

	if (dut_gen == DUT_PIP2_CAPABLE)
		size = pt_pip2_manual_upgrade_store(dev, attr, buf, size);
	else if (dut_gen == DUT_PIP1_ONLY)
		size = pt_manual_upgrade_store(dev, attr, buf, size);

	return size;
}
static DEVICE_ATTR(update_fw, S_IWUSR, NULL, pt_update_fw_store);

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
 *       size   - size of data in buffer
 ******************************************************************************/
static ssize_t pt_pip2_manual_ram_upgrade_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct pt_loader_data *ld = pt_get_loader_data(dev);
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
#endif /* CONFIG_TOUCHSCREEN_PARADE_BINARY_FW_UPGRADE */

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
	return snprintf(buf, PT_MAX_PRBUF_SIZE,
		"ERASE Status: 0x%02x\n",
		pip2_erase_status);
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
 *	 size - size of buffer
 ******************************************************************************/
static ssize_t pt_pip2_flash_erase_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned long value;
	int rc;
	u8 file_handle;
	u8 read_buf[256];
	u8 data[20];
	u8 mode;
	u16 actual_read_len;
	struct pip2_cmd_structure pip2_cmd;

	rc = kstrtoul(buf, 10, &value);
	if (rc < 0) {
		/* Positive integers only */
		pt_debug(dev, DL_ERROR,
			"%s: Invalid value\n", __func__);
		goto exit_erase;
	}

	pm_runtime_get_sync(dev);

	rc = cmd->request_exclusive(dev, PT_LDR_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0)
		goto error;

	rc = cmd->request_pip2_enter_bl(dev, &mode);
	if (rc) {
		pt_debug(dev, DL_ERROR, "%s: Failed to enter BL\n", __func__);
		goto exit_erase;
	}

	/* Open the requested file */
	pt_debug(dev, DL_WARN, "%s: Preparing to erase file: %lu\n",
		__func__, value);

	/* Ensure the SRAM file is not being erased */
	if (value == 0) {
		pip2_erase_status = PIP2_RSP_ERR_BAD_FILE;
		pt_debug(dev, DL_ERROR, "%s: ERROR - Invalid File\n",
			__func__);
		goto exit_erase;
	}
	data[0] = value;
	rc = cmd->nonhid_cmd->pip2_send_cmd(dev,
		PT_CORE_CMD_UNPROTECTED, &pip2_cmd,
		PIP2_CMD_ID_FILE_OPEN, data, 1, read_buf, &actual_read_len);
	file_handle = read_buf[PIP2_RESPONSE_BODY_OFFSET];
	pip2_erase_status = read_buf[PIP2_RESPONSE_STATUS_OFFSET];
	if (pip2_erase_status) {
		pt_debug(dev, DL_ERROR,
			"%s ERROR: File open command failed, handle = %d status = %d\n",
			__func__, file_handle, pip2_erase_status);
		goto exit_erase;
	}

	/* File ioctl, erase file */
	pt_debug(dev, DL_INFO, "%s Erasing file...\n", __func__);
	data[0] = file_handle;
	data[1] = PIP2_FILE_IOCTL_CODE_ERASE_FILE;
	rc = cmd->nonhid_cmd->pip2_send_cmd(dev,
		PT_CORE_CMD_UNPROTECTED, &pip2_cmd,
		PIP2_CMD_ID_FILE_IOCTL, data, 2, read_buf, &actual_read_len);
	pip2_erase_status = read_buf[PIP2_RESPONSE_STATUS_OFFSET];
	if (pip2_erase_status) {
		pt_debug(dev, DL_ERROR,
			"%s ERROR: file erase failure\n", __func__);
		goto exit_erase;
	}
	pt_debug(dev, DL_INFO,
		"%s File erase successful\n", __func__);

	/*
	 * If the FW was not the file erased and the DUT was in operational
	 * mode before the erase function then queue a restart to get back
	 * to operational mode.
	 */
	if (file_handle != 1 && mode == PT_MODE_OPERATIONAL) {
		pt_debug(dev, DL_WARN, "%s Requesting RESTART\n", __func__);
		cmd->request_reset(dev, PT_CORE_CMD_UNPROTECTED);
		cmd->request_restart(dev, true);
	}
	cmd->request_start_wd(dev);

exit_erase:
	pm_runtime_put_sync(dev);
	cmd->release_exclusive(dev);
error:
	return size;
}

static DEVICE_ATTR(pip2_flash_erase, S_IRUGO | S_IWUSR,
	pt_pip2_flash_erase_show, pt_pip2_flash_erase_store);

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
	ssize_t ret;

	if (pip2_bl_status <= PIP2_BL_STATUS_COMPLETE) {
		pt_debug(dev, DL_DEBUG,
			"%s BL_STATUS = %d\n", __func__, pip2_bl_status);
		return sprintf(buf, "%d\n", pip2_bl_status);
	}

	switch (pip2_bl_status) {
	case PIP2_BL_STATUS_GENERAL_ERROR:
		ret = sprintf(buf,
			"ERROR: %d - General programming error\n",
			pip2_bl_status);
		break;
	case PIP2_BL_STATUS_PIP_VERSION_ERROR:
		ret = sprintf(buf,
			"ERROR: %d - Wrong PIP version detected\n",
			pip2_bl_status);
		break;
	case PIP2_BL_STATUS_FW_VERSION_ERROR:
		ret = sprintf(buf,
			"ERROR: %d - FW vervion newer than bin file\n",
			pip2_bl_status);
		break;
	case PIP2_BL_STATUS_ERASE_ERROR:
		ret = sprintf(buf,
			"ERROR: %d - Erase FW file error\n",
			pip2_bl_status);
		break;
	case PIP2_BL_STATUS_FILE_CLOSE_ERROR:
		ret = sprintf(buf,
			"ERROR: %d - Close FW file error\n",
			pip2_bl_status);
		break;
	case PIP2_BL_STATUS_WRITE_ERROR:
		ret = sprintf(buf,
			"ERROR: %d - File write error\n",
			pip2_bl_status);
		break;
	case PIP2_BL_STATUS_EXECUTE_ERROR:
		ret = sprintf(buf,
			"ERROR: %d - Execute RAM image failure\n",
			pip2_bl_status);
		break;
	case PIP2_BL_STATUS_RESET_ERROR:
		ret = sprintf(buf,
			"ERROR: %d - Reset DUT error\n",
			pip2_bl_status);
		break;
	case PIP2_BL_STATUS_MODE_ERROR:
		ret = sprintf(buf,
			"ERROR: %d - Program complete, Incorrect BL/App mode detected after sentinel\n",
			pip2_bl_status);
		break;
	case PIP2_BL_STATUS_ENTER_BL_ERROR:
		ret = sprintf(buf,
			"ERROR: %d - Could not enter the BL\n",
			pip2_bl_status);
		break;
	case PIP2_BL_STATUS_FILE_OPEN_ERROR:
		ret = sprintf(buf,
			"ERROR: %d - Error opening the file in FLASH\n",
			pip2_bl_status);
		break;
	case PIP2_BL_STATUS_FW_SENTINEL_NOT_SEEN:
		ret = sprintf(buf,
			"ERROR: %d - FW Reset Sentinel not seen after XRES\n",
			pip2_bl_status);
		break;
	default:
		ret = sprintf(buf,
			"ERROR: %d - Unknown error\n",
			pip2_bl_status);
	}
	return ret;
}
static DEVICE_ATTR(pip2_bl_status, S_IRUGO, pt_pip2_bl_status_show, NULL);

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
	u8 initial_mode;
	struct pip2_cmd_structure pip2_cmd;
	struct pt_core_data *cd = dev_get_drvdata(dev);

	pm_runtime_get_sync(dev);

	rc = cmd->request_exclusive(dev, PT_LDR_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0)
		goto error;

	/* Force the DUT to enter the BL */
	rc = cmd->request_pip2_enter_bl(dev, &initial_mode);
	if (rc) {
		pt_debug(dev, DL_ERROR, "%s: Failed to enter BL\n", __func__);
		goto exit_error_show;
	}

	/* Determine Bootloader PIP version, 2.0 needs special handling */
	cmd->request_active_pip_prot(dev, PT_CORE_CMD_UNPROTECTED,
		&(cd->bl_pip_version_major), &(cd->bl_pip_version_minor));

	/* get last error information */
	rc = cmd->nonhid_cmd->pip2_send_cmd(dev,
		PT_CORE_CMD_UNPROTECTED, &pip2_cmd,
		PIP2_CMD_ID_GET_LAST_ERRNO, NULL, 0, read_buf,
		&actual_read_len);

	/* Reset device to get back to running FW if thats how we started */
	if (initial_mode != PT_MODE_BOOTLOADER) {
		pt_debug(dev, DL_WARN, "%s Requesting RESTART\n", __func__);
		cmd->request_reset(dev, PT_CORE_CMD_UNPROTECTED);
		cmd->request_restart(dev, true);
		cmd->request_start_wd(dev);
	}

exit_error_show:
	pm_runtime_put_sync(dev);
	cmd->release_exclusive(dev);
error:
	return snprintf(buf, PT_MAX_PRBUF_SIZE,
		"PIP2_LAST_ERRORNO: %d\n",
		read_buf[PIP2_RESPONSE_BODY_OFFSET + 1]);

}
static DEVICE_ATTR(pip2_get_last_error, S_IRUGO,
	pt_pip2_get_last_error_show, NULL);

#if PT_FW_UPGRADE
/*******************************************************************************
 * FUNCTION: pt_loader_attention
 *
 * SUMMARY: Funtion to be registered to TTDL attention list to set up
 *  int_running semaphore.
 *
 * RETURN:
 *   0 = success
 *
 * PARAMETERS:
 *  *dev  - pointer to device structure
 ******************************************************************************/
static int pt_loader_attention(struct device *dev)
{
	struct pt_loader_data *ld = pt_get_loader_data(dev);

	complete(&ld->int_running);
	return 0;
}

/*******************************************************************************
 * FUNCTION: pt_fw_upgrade_cb
 *
 * SUMMARY: Function to be registered to TTDL attention list to allow upgrade
 *  if host cannot get response from firmware with ping command.
 * RETURN:
 *   0 = success
 *
 * PARAMETERS:
 *  *dev  - pointer to device structure
 ******************************************************************************/
static int pt_fw_upgrade_cb(struct device *dev)
{
	u8 dut_gen = cmd->request_dut_generation(dev);

#ifdef CONFIG_TOUCHSCREEN_PARADE_PLATFORM_FW_UPGRADE
	if (dut_gen == DUT_PIP1_ONLY) {
		pt_debug(dev, DL_WARN, "%s: Upgrade Platform FW", __func__);
		if (!upgrade_firmware_from_platform(dev, false))
			return 1;
	}
#endif

#ifdef CONFIG_TOUCHSCREEN_PARADE_BINARY_FW_UPGRADE
	pt_debug(dev, DL_WARN, "%s: Upgrade Builtin FW", __func__);
	if (dut_gen == DUT_PIP2_CAPABLE) {
		if (!pt_pip2_upgrade_firmware_from_builtin(dev)) {
			pt_debug(dev, DL_WARN, "%s: Builtin FW upgrade failed",
				__func__);
			return 1;
		}
	} else {
		if (!upgrade_firmware_from_builtin(dev))
			return 1;
	}
#endif
	return 0;
}
#endif /* PT_FW_UPGRADE */

/*******************************************************************************
 * FUNCTION: pt_loader_probe
 *
 * SUMMARY: The probe function for the FW loader.
 *
 * PARAMETERS:
 *   *dev   - pointer to device structure
 *  **data  - double pointer to the loader data to be created here
 ******************************************************************************/
static int pt_loader_probe(struct device *dev, void **data)
{
	struct pt_loader_data *ld;
	struct pip2_loader_data *pip2_data;
	struct pt_platform_data *pdata = dev_get_platdata(dev);
	int rc;
	u8 dut_gen = cmd->request_dut_generation(dev);

	pt_debug(dev, DL_WARN,
		"%s: entering pt_loader_probe\n", __func__);
#ifdef PT_FW_UPGRADE
	pt_debug(dev, DL_WARN,
		"%s: PT_FW_UPGRADE is set\n", __func__);
#endif

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

	if (dut_gen == DUT_PIP2_CAPABLE) {
		pip2_data = kzalloc(sizeof(*pip2_data), GFP_KERNEL);
		if (!pip2_data) {
			rc = -ENOMEM;
			goto error_alloc_data_failed;
		}
		pip2_data->dev = dev;
		ld->pip2_data = pip2_data;

		/* Initialize boot loader status */
		pip2_bl_status = PIP2_BL_STATUS_IDLE;

#ifdef CONFIG_TOUCHSCREEN_PARADE_BINARY_FW_UPGRADE
		rc = device_create_file(dev, &dev_attr_update_fw);
		if (rc) {
			pt_debug(dev, DL_ERROR,
				"%s: Error creating update_fw sysfs\n",
				__func__);
			goto error_create_update_fw;
		}

		rc = device_create_file(dev, &dev_attr_pip2_manual_upgrade);
		if (rc) {
			pt_debug(dev, DL_ERROR,
				"%s: Error creating pip2_manual_upgrade sysfs\n",
				__func__);
			goto error_create_pip2_manual_upgrade;
		}

		rc = device_create_file(dev, &dev_attr_pip2_manual_ram_upgrade);
		if (rc) {
			pt_debug(dev, DL_ERROR,
				"%s: Error creating pip2_manual_ram_upgrade sysfs\n",
				__func__);
			goto error_create_pip2_manual_ram_upgrade;
		}
#endif

		rc = device_create_file(dev, &dev_attr_pip2_flash_erase);
		if (rc) {
			pt_debug(dev, DL_ERROR,
				"%s: Error creating pip2_flash_erase sysfs\n",
				__func__);
			goto error_create_pip2_flash_erase;
		}

		rc = device_create_file(dev, &dev_attr_pip2_bl_status);
		if (rc) {
			pt_debug(dev, DL_ERROR,
				"%s: Error creating pip2_bl_status sysfs\n",
				__func__);
			goto error_create_pip2_bl_status;
		}

		rc = device_create_file(dev, &dev_attr_pip2_get_last_error);
		if (rc) {
			pt_debug(dev, DL_ERROR,
				"%s: Error creating pip2_get_last_error\n",
				__func__);
			goto error_create_pip2_get_last_error;
		}
	} else {
#ifdef CONFIG_TOUCHSCREEN_PARADE_PLATFORM_FW_UPGRADE
		rc = device_create_file(dev, &dev_attr_forced_upgrade);
		if (rc) {
			pt_debug(dev, DL_ERROR,
				"%s: Error creating forced_upgrade\n",
				__func__);
			goto error_create_forced_upgrade;
		}
#endif

#ifdef CONFIG_TOUCHSCREEN_PARADE_BINARY_FW_UPGRADE
		rc = device_create_file(dev, &dev_attr_update_fw);
		if (rc) {
			pt_debug(dev, DL_ERROR,
				"%s: Error creating update_fw sysfs\n",
				__func__);
			goto error_create_update_fw_legacy;
		}

		rc = device_create_file(dev, &dev_attr_manual_upgrade);
		if (rc) {
			pt_debug(dev, DL_ERROR,
				"%s: Error creating manual_upgrade\n",
				__func__);
			goto error_create_manual_upgrade;
		}
#endif

#ifdef CONFIG_TOUCHSCREEN_PARADE_MANUAL_TTCONFIG_UPGRADE
		rc = device_create_file(dev, &dev_attr_config_loading);
		if (rc) {
			pt_debug(dev, DL_ERROR,
				"%s: Error creating config_loading\n",
				__func__);
			goto error_create_config_loading;
		}

		rc = device_create_bin_file(dev, &bin_attr_config_data);
		if (rc) {
			pt_debug(dev, DL_ERROR,
				"%s: Error creating config_data\n",
				__func__);
			goto error_create_config_data;
		}
#endif
	}

	ld->loader_pdata = pdata->loader_pdata;
	ld->dev = dev;
	*data = ld;

#if PT_FW_UPGRADE
	init_completion(&ld->int_running);

	cmd->subscribe_attention(dev, PT_ATTEN_IRQ, PT_LOADER_NAME,
		pt_loader_attention, PT_MODE_BOOTLOADER);

	cmd->subscribe_attention(dev, PT_ATTEN_LOADER, PT_LOADER_NAME,
		pt_fw_upgrade_cb, PT_MODE_UNKNOWN);
#endif
#if PT_FW_UPGRADE || PT_TTCONFIG_UPGRADE
	pt_debug(dev, DL_WARN, "%s: INIT_WORK pt_calibrate_idacs\n",
		__func__);
	init_completion(&ld->calibration_complete);
	INIT_WORK(&ld->calibration_work, pt_calibrate_idacs);
#endif

#ifdef CONFIG_TOUCHSCREEN_PARADE_MANUAL_TTCONFIG_UPGRADE
	if (dut_gen == DUT_PIP1_ONLY)
		mutex_init(&ld->config_lock);
#endif

#ifdef UPGRADE_FW_AND_CONFIG_IN_PROBE
	/* Call FW and config upgrade directly in probe */
	pt_fw_and_config_upgrade(&ld->fw_and_config_upgrade);
#else
	pt_debug(dev, DL_INFO, "%s: Schedule FW upgrade work\n", __func__);
	INIT_WORK(&ld->fw_and_config_upgrade, pt_fw_and_config_upgrade);
	schedule_work(&ld->fw_and_config_upgrade);
#endif

	pt_debug(dev, DL_INFO, "%s: Successful probe %s\n",
		__func__, dev_name(dev));
	return 0;

#ifdef CONFIG_TOUCHSCREEN_PARADE_MANUAL_TTCONFIG_UPGRADE
error_create_config_data:
	device_remove_file(dev, &dev_attr_config_loading);
error_create_config_loading:
#endif
#ifdef CONFIG_TOUCHSCREEN_PARADE_BINARY_FW_UPGRADE
	device_remove_file(dev, &dev_attr_manual_upgrade);
error_create_manual_upgrade:
error_create_update_fw_legacy:
#endif
#ifdef CONFIG_TOUCHSCREEN_PARADE_PLATFORM_FW_UPGRADE
	device_remove_file(dev, &dev_attr_forced_upgrade);
error_create_forced_upgrade:
#endif
error_create_pip2_get_last_error:
	device_remove_file(dev, &dev_attr_pip2_bl_status);
error_create_pip2_bl_status:
	device_remove_file(dev, &dev_attr_pip2_flash_erase);
error_create_pip2_flash_erase:
#ifdef CONFIG_TOUCHSCREEN_PARADE_BINARY_FW_UPGRADE
	device_remove_file(dev, &dev_attr_pip2_manual_ram_upgrade);
error_create_pip2_manual_ram_upgrade:
	device_remove_file(dev, &dev_attr_pip2_manual_upgrade);
error_create_pip2_manual_upgrade:
	device_remove_file(dev, &dev_attr_update_fw);
error_create_update_fw:
#endif
	kfree(ld->pip2_data);

	kfree(ld);
error_alloc_data_failed:
error_no_pdata:
	pt_debug(dev, DL_ERROR, "%s failed.\n", __func__);
	return rc;
}

/*******************************************************************************
 * FUNCTION: pt_loader_release
 *
 * SUMMARY: Remove function for loader module that does following cleanup:
 *  - Unsubscibe all registered attention tasks
 *  - Removes all created sysfs nodes
 *  - Frees all pointers
 *
 * PARAMETERS:
 *  *dev   - pointer to device structure
 *  *data  - pointer to the loader data
 ******************************************************************************/
static void pt_loader_release(struct device *dev, void *data)
{
	struct pt_loader_data *ld = (struct pt_loader_data *)data;
	u8 dut_gen = cmd->request_dut_generation(dev);

#if PT_FW_UPGRADE
	cmd->unsubscribe_attention(dev, PT_ATTEN_IRQ, PT_LOADER_NAME,
		pt_loader_attention, PT_MODE_BOOTLOADER);

	cmd->unsubscribe_attention(dev, PT_ATTEN_LOADER, PT_LOADER_NAME,
		pt_fw_upgrade_cb, PT_MODE_UNKNOWN);
#endif
	if (dut_gen == DUT_PIP1_ONLY) {
#ifdef CONFIG_TOUCHSCREEN_PARADE_MANUAL_TTCONFIG_UPGRADE
		device_remove_bin_file(dev, &bin_attr_config_data);
		device_remove_file(dev, &dev_attr_config_loading);
		if (!ld->config_data)
			kfree(ld->config_data);
#endif
#ifdef CONFIG_TOUCHSCREEN_PARADE_BINARY_FW_UPGRADE
		device_remove_file(dev, &dev_attr_update_fw);
		device_remove_file(dev, &dev_attr_manual_upgrade);
#endif
#ifdef CONFIG_TOUCHSCREEN_PARADE_PLATFORM_FW_UPGRADE
		device_remove_file(dev, &dev_attr_forced_upgrade);
#endif
	} else {
		device_remove_file(dev, &dev_attr_pip2_get_last_error);
		device_remove_file(dev, &dev_attr_pip2_bl_status);
		device_remove_file(dev, &dev_attr_pip2_flash_erase);
#ifdef CONFIG_TOUCHSCREEN_PARADE_BINARY_FW_UPGRADE
		device_remove_file(dev, &dev_attr_pip2_manual_ram_upgrade);
		device_remove_file(dev, &dev_attr_pip2_manual_upgrade);
		device_remove_file(dev, &dev_attr_update_fw);
#endif
		kfree(ld->pip2_data);
	}
	kfree(ld);
}

static struct pt_module loader_module = {
	.name = PT_LOADER_NAME,
	.probe = pt_loader_probe,
	.release = pt_loader_release,
};

/*******************************************************************************
 * FUNCTION: pt_loader_init
 *
 * SUMMARY: Initialize function for loader module which to register
 * loader_module into TTDL module list.
 *
 * RETURN:
 *   0 = success
 *  !0 = failure
 *
 * PARAMETERS:
 *  *dev   - pointer to device structure
 *  *data  - pointer to the loader data
 ******************************************************************************/
static int __init pt_loader_init(void)
{
	int rc;

	cmd = pt_get_commands();
	if (!cmd)
		return -EINVAL;

	rc = pt_register_module(&loader_module);
	if (rc < 0) {
		pr_err("%s: Error, failed registering module\n",
			__func__);
			return rc;
	}

	pr_info("%s: Parade FW Loader Driver (Version %s) rc=%d\n",
		 __func__, PT_DRIVER_VERSION, rc);
	return 0;
}
module_init(pt_loader_init);

/*******************************************************************************
 * FUNCTION: pt_loader_exit
 *
 * SUMMARY: Exit function for loader module which to unregister loader_module
 * from TTDL module list.
 *
 ******************************************************************************/
static void __exit pt_loader_exit(void)
{
	pt_unregister_module(&loader_module);
}
module_exit(pt_loader_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Parade TrueTouch(R) Standard Product FW Loader Driver");
MODULE_AUTHOR("Parade Technologies <ttdrivers@paradetech.com>");
