/*
 * cyttsp5_regs.h
 * Parade TrueTouch(TM) Standard Product V5 Registers.
 * For use with Parade touchscreen controllers.
 * Supported parts include:
 * CYTMA5XX
 * CYTMA448
 * CYTMA445A
 * CYTT21XXX
 * CYTT31XXX
 *
 * Copyright (C) 2015 Parade Technologies
 * Copyright (C) 2012-2015 Cypress Semiconductor
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

#ifndef _CYTTSP5_REGS_H
#define _CYTTSP5_REGS_H

#define DEBUG
#include <linux/device.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#elif defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#endif

#include <asm/unaligned.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/hid.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/limits.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of.h>
#include <linux/pm_runtime.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/suspend.h>
#include <linux/stringify.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#include <linux/version.h>
#include <linux/cyttsp5_core.h>

#include <linux/timer.h>
#include <linux/timex.h>
#include <linux/rtc.h>


/* #define EASYWAKE_TSG6 */

#define CY_FW_FILE_PREFIX	"cyttsp5_fw"
#define CY_FW_FILE_SUFFIX	".bin"
#define CY_FW_FILE_NAME		"cyttsp5_fw.bin"

#define TTHE_TUNER_SUPPORT
#ifdef TTHE_TUNER_SUPPORT
#define CYTTSP5_TTHE_TUNER_FILE_NAME "tthe_tuner"
#endif

#define CY_MAX_PRBUF_SIZE           PIPE_BUF
#define CY_PR_TRUNCATED             " truncated..."

#define CY_DEFAULT_CORE_ID          "cyttsp5_core0"
#define CY_MAX_NUM_CORE_DEVS        5
#define CY_IRQ_ASSERTED_VALUE	0

/* -----------------------------------------------------------------------------
 * Define one or more of the following for Parade Technology DUT Support. For
 * systems that include an older PIP based DUT (TSG5 or TSG6) none of these
 * should be defined unless building for the PtSBC development platform.
 *
 * PT_TDDI_SUPPORT        - Enable for all TDDI based parts
 *                          e.g. TC33xx parts
 * PT_TT_DISCRETE_SUPPORT - Enable for all new Parade discrete parts, this
 *                          should not be enabled with PT_TDDI_SUPPORT
 *                          e.g. TT7xxx parts
 * PT_PTSBC_SUPPORT       - Enable when building TTDL for the Parade PtSBC
 *                          development platform
 *
 * The following lines have been added as examples and can be used by automated
 * build systems using the 'sed' command to insert the correct #defines.
 */
/* #define PT_TDDI_SUPPORT         1 */
/* #define PT_TT_DISCRETE_SUPPORT  1 */
/* #define PT_PTSBC_SUPPORT        1 */

/*
 * Enable TTDL diagnostic features, using this will add diagnostic counters
 * thoroughout the driver and expands several sysfs nodes to report these
 * counters
 */
#define TTDL_DIAGNOSTICS 1

#ifdef CY_ENABLE_MAX_ELEN
#define CY_MAX_ELEN 100
#endif

/* Enum for debug reporting levels */
enum PT_DEBUG_LEVEL {
	DL_QUIET	= 0,
	DL_ERROR	= 1,
	DL_WARN		= 2,
	DL_INFO		= 3,
	DL_DEBUG	= 4,
	DL_MAX
};

/* Startup DUT enum status bitmask */
enum PT_STARTUP_STATUS {
	STARTUP_STATUS_START			= 0,
	STARTUP_STATUS_BL_RESET_SENTINEL	= 0x001,
	STARTUP_STATUS_FW_RESET_SENTINEL	= 0x002,
	STARTUP_STATUS_GET_DESC			= 0x004,
	STARTUP_STATUS_GET_MODE			= 0x008,
	STARTUP_STATUS_GET_RPT_DESC		= 0x010,
	STARTUP_STATUS_GET_SYS_INFO		= 0x020,
	STARTUP_STATUS_GET_CFG_CRC		= 0x040,
	STARTUP_STATUS_RESTORE_PARM		= 0x080,
	STARTUP_STATUS_COMPLETE			= 0x100,
	STARTUP_STATUS_FULL			= 0x1FF
};

#define CY_INITIAL_DEBUG_LEVEL DL_WARN
#define CY_INITIAL_SHOW_TIME_STAMP 0

/*
 * Print out all debug prints that are less then or equal to set level.
 */
#define pt_debug(dev, dlevel, format, arg...)	 \
	do { \
		struct cyttsp5_core_data *cd_tmp = dev_get_drvdata(dev);\
		if (cd_tmp->debug_level >= dlevel) {\
			dev_dbg(dev, "%d-[%u]"format, dlevel,\
			jiffies_to_msecs(jiffies), ##arg);\
		} \
	} while (0)

/* HID */
#define HID_CYVENDOR					0xff010000
#define CY_HID_VENDOR_ID				0x04B4
#define CY_HID_BL_PRODUCT_ID				0xC100
#define CY_HID_APP_PRODUCT_ID				0xC101
#define CY_HID_VERSION					0x0100
#define CY_HID_APP_REPORT_ID				0xF7
#define CY_HID_BL_REPORT_ID				0xFF

#define HID_INVALID_REPORT_ID				0x0
#define HID_TOUCH_REPORT_ID				0x1
#define HID_BTN_REPORT_ID				0x3
#define HID_WAKEUP_REPORT_ID				0x4
#define HID_NOISE_METRIC_REPORT_ID			0x5
#define HID_TRACKING_HEATMAP_REPOR_ID			0xE
#define HID_SENSOR_DATA_REPORT_ID			0xF
#define HID_APP_RESPONSE_REPORT_ID			0x1F
#define HID_APP_OUTPUT_REPORT_ID			0x2F
#define HID_BL_RESPONSE_REPORT_ID			0x30
#define HID_BL_OUTPUT_REPORT_ID				0x40
#define HID_RESPONSE_REPORT_ID				0xF0

#define HID_OUTPUT_RESPONSE_REPORT_OFFSET		2
#define HID_OUTPUT_RESPONSE_CMD_OFFSET			4
#define HID_OUTPUT_RESPONSE_CMD_MASK			0x7F
#define HID_OUTPUT_CMD_OFFSET				6
#define HID_OUTPUT_CMD_MASK				0x7F

#define HID_SYSINFO_CYDATA_OFFSET			5
#define HID_SYSINFO_SENSING_OFFSET			33
#define HID_SYSINFO_BTN_OFFSET				48
#define HID_SYSINFO_BTN_MASK				0xFF
#define HID_SYSINFO_MAX_BTN				8

#define HID_POWER_ON					0x0
#define HID_POWER_SLEEP					0x1
#define HID_LENGTH_BYTES				2
#define HID_LENGTH_AND_REPORT_ID_BYTES			3

/*  Timeout in ms */
#define CY_REQUEST_EXCLUSIVE_TIMEOUT			8000
#define CY_WATCHDOG_TIMEOUT				2000
#define CY_PTSBC_INIT_WATCHDOG_TIMEOUT			20000
#define CY_HID_RESET_TIMEOUT				5000
#define CY_HID_AUTO_CALI_CPLT_TIMEOUT			2500
/* HID_DESCRIPTOR_TIMEOUT value based on FW spec (CAL_OS) */
#define CY_HID_GET_HID_DESCRIPTOR_TIMEOUT		4000
#define CY_HID_GET_REPORT_DESCRIPTOR_TIMEOUT		500
#define CY_HID_SET_POWER_TIMEOUT			500
#ifdef VERBOSE_DEBUG
#define CY_HID_OUTPUT_TIMEOUT				2000
#else
#define CY_HID_OUTPUT_TIMEOUT				500
#endif
#define CY_HID_OUTPUT_START_BOOTLOADER_TIMEOUT		2000
#define CY_HID_OUTPUT_USER_TIMEOUT			8000
#define CY_HID_OUTPUT_GET_SYSINFO_TIMEOUT		3000
#define CY_HID_OUTPUT_CALIBRATE_IDAC_TIMEOUT		5000
#define CY_HID_OUTPUT_WRITE_CONF_BLOCK_TIMEOUT		400
#define CY_HID_OUTPUT_RUN_SELF_TEST_TIMEOUT		10000
#define CY_HID_OUTPUT_BL_INITIATE_BL_TIMEOUT		20000
#define CY_HID_OUTPUT_BL_PROGRAM_AND_VERIFY_TIMEOUT	400
#define PT_REQUEST_STARTUP_TIMEOUT                      4000

#define CY_WATCHDOG_RETRY_COUNT			30
#define PT_I2C_READ_INPUT_RETRY_COUNT	1

/* maximum number of concurrent tracks */
#define TOUCH_REPORT_SIZE           10
#define TOUCH_INPUT_HEADER_SIZE     7
#define TOUCH_COUNT_BYTE_OFFSET     5
#define BTN_REPORT_SIZE             9
#define BTN_INPUT_HEADER_SIZE       5
#define SENSOR_REPORT_SIZE          150
#define SENSOR_HEADER_SIZE          4

/* helpers */
#define GET_NUM_TOUCHES(x)          ((x) & 0x1F)
#define IS_LARGE_AREA(x)            ((x) & 0x20)
#define IS_BAD_PKT(x)               ((x) & 0x20)
#define IS_TMO(t)                   ((t) == 0)
#define HI_BYTE(x)                  (u8)(((x) >> 8) & 0xFF)
#define LOW_BYTE(x)                 (u8)((x) & 0xFF)
#define SET_CMD_LOW(byte, bits)	\
	((byte) = (((byte) & 0xF0) | ((bits) & 0x0F)))
#define SET_CMD_HIGH(byte, bits)\
	((byte) = (((byte) & 0x0F) | ((bits) & 0xF0)))

#define GET_MASK(length) \
	((1 << length) - 1)
#define GET_FIELD(name, length, shift) \
	((name >> shift) & GET_MASK(length))

#define HID_ITEM_SIZE_MASK	0x03
#define HID_ITEM_TYPE_MASK	0x0C
#define HID_ITEM_TAG_MASK	0xF0

#define HID_ITEM_SIZE_SHIFT	0
#define HID_ITEM_TYPE_SHIFT	2
#define HID_ITEM_TAG_SHIFT	4

#define HID_GET_ITEM_SIZE(x)  \
	((x & HID_ITEM_SIZE_MASK) >> HID_ITEM_SIZE_SHIFT)
#define HID_GET_ITEM_TYPE(x) \
	((x & HID_ITEM_TYPE_MASK) >> HID_ITEM_TYPE_SHIFT)
#define HID_GET_ITEM_TAG(x) \
	((x & HID_ITEM_TAG_MASK) >> HID_ITEM_TAG_SHIFT)

#define IS_DEEP_SLEEP_CONFIGURED(x) \
		((x) == 0 || (x) == 0xFF)

#define IS_PIP_VER_GE(p, maj, min) \
		((p)->cydata.pip_ver_major < (maj) ? \
			0 : \
			((p)->cydata.pip_ver_minor < (min) ? \
				0 : \
				1))
#define IS_PIP_VER_EQ(p, maj, min) \
		((p)->cydata.pip_ver_major == (maj) ? \
			((p)->cydata.pip_ver_minor == (min) ? \
			1 : \
				0 : \
				0))

/* DUT Debug commands (dut_debug sysfs) */
#define PT_DUT_DBG_HID_RESET             50
#define PT_DUT_DBG_HID_GET_REPORT        51
#define PT_DUT_DBG_HID_SET_REPORT        52
#define PT_DUT_DBG_HID_SET_POWER_ON      53
#define PT_DUT_DBG_HID_SET_POWER_SLEEP   54
#define PT_DUT_DBG_SOFT_RESET            97
#define PT_DUT_DBG_RESET                 98
#define PT_DUT_DBG_HID_NULL              100
#define PT_DUT_DBG_HID_ENTER_BL          101
#define PT_DUT_DBG_HID_SYSINFO           102
#define PT_DUT_DBG_HID_SUSPEND_SCAN      103
#define PT_DUT_DBG_HID_RESUME_SCAN       104
#define	PT_DUT_DBG_HID_DESC              109

/* Driver Debug commands (drv_debug sysfs) */
#define PT_DRV_DBG_SUSPEND               4
#define PT_DRV_DBG_RESUME                5
#define PT_DRV_DBG_STOP_WD               105
#define PT_DRV_DBG_START_WD              106
#define PT_DRV_DBG_TTHE_TUNER_EXIT       107
#define	PT_DRV_DBG_TTHE_BUF_CLEAN        108
#define	PT_DRV_DBG_CLEAR_PARM_LIST       110
/*
 * Commands that require additional parameters
 * will be in the 200 range. Commands that do not
 * require additional parameters remain below 200.
 */
#define PT_DRV_DBG_REPORT_LEVEL		 200
#define PT_DRV_DBG_WATCHDOG_INTERVAL	 201
#define PT_DRV_DBG_SHOW_TIMESTAMP	 202
#define PT_DRV_DBG_SET_PIP_2P0_FLAG	 203
#define	PT_DRV_DBG_FLUSH_I2C_BUS	 204
#define	PT_DRV_DBG_TC3300_LEN_FLAG	 205
#define PT_DRV_DBG_WD_CORRECTIVE_ACTION  298
#define PT_DRV_DBG_VIRTUAL_I2C_DUT       299

/* Recognized usages */
/* undef them first for possible redefinition in Linux */
#undef HID_DI_PRESSURE
#undef HID_DI_TIP
#undef HID_DI_CONTACTID
#undef HID_DI_CONTACTCOUNT
#undef HID_DI_SCANTIME
#define HID_DI_PRESSURE		0x000d0030
#define HID_DI_TIP		0x000d0042
#define HID_DI_CONTACTID	0x000d0051
#define HID_DI_CONTACTCOUNT	0x000d0054
#define HID_DI_SCANTIME		0x000d0056

/* Parade vendor specific usages */
#define HID_CY_UNDEFINED	0xff010000
#define HID_CY_BOOTLOADER	0xff010001
#define HID_CY_TOUCHAPPLICATION	0xff010002
#define HID_CY_BUTTONS		0xff010020
#define HID_CY_GENERICITEM	0xff010030
#define HID_CY_LARGEOBJECT	0xff010040
#define HID_CY_NOISEEFFECTS	0xff010041
#define HID_CY_REPORTCOUNTER	0xff010042
#define HID_CY_TOUCHTYPE	0xff010060
#define HID_CY_EVENTID		0xff010061
#define HID_CY_MAJORAXISLENGTH	0xff010062
#define HID_CY_MINORAXISLENGTH	0xff010063
#define HID_CY_ORIENTATION	0xff010064
#define HID_CY_BUTTONSIGNAL	0xff010065
#define HID_CY_MAJOR_CONTACT_AXIS_LENGTH	0xff010066
#define HID_CY_MINOR_CONTACT_AXIS_LENGTH	0xff010067
#define HID_CY_TCH_COL_USAGE_PG 0x000D0022
#define HID_CY_BTN_COL_USAGE_PG 0xFF010020

#define PANEL_ID_NOT_ENABLED	0xFF

#ifdef EASYWAKE_TSG6
#define GESTURE_DOUBLE_TAP         (1)
#define GESTURE_TWO_FINGERS_SLIDE  (2)
#define GESTURE_TOUCH_DETECTED     (3)
#define GESTURE_PUSH_BUTTON        (4)
#define GESTURE_SINGLE_SLIDE_DE_TX (5)
#define GESTURE_SINGLE_SLIDE_IN_TX (6)
#define GESTURE_SINGLE_SLIDE_DE_RX (7)
#define GESTURE_SINGLE_SLIDE_IN_RX (8)
#endif

/* FW RAM parameters */
#define CY_RAM_ID_TOUCHMODE_ENABLED	0x02
#define CY_RAM_ID_PROXIMITY_ENABLE	0x20
#define CY_RAM_ID_TOUCHMODE_ENABLED_SIZE	1
#define CY_RAM_ID_PROXIMITY_ENABLE_SIZE	1

/* abs signal capabilities offsets in the frameworks array */
enum cyttsp5_sig_caps {
	CY_SIGNAL_OST,
	CY_MIN_OST,
	CY_MAX_OST,
	CY_FUZZ_OST,
	CY_FLAT_OST,
	CY_NUM_ABS_SET	/* number of signal capability fields */
};

/* helpers */
#define NUM_SIGNALS(frmwrk)		((frmwrk)->size / CY_NUM_ABS_SET)
#define PARAM(frmwrk, sig_ost, cap_ost) \
		((frmwrk)->abs[((sig_ost) * CY_NUM_ABS_SET) + (cap_ost)])

#define PARAM_SIGNAL(frmwrk, sig_ost)	PARAM(frmwrk, sig_ost, CY_SIGNAL_OST)
#define PARAM_MIN(frmwrk, sig_ost)	PARAM(frmwrk, sig_ost, CY_MIN_OST)
#define PARAM_MAX(frmwrk, sig_ost)	PARAM(frmwrk, sig_ost, CY_MAX_OST)
#define PARAM_FUZZ(frmwrk, sig_ost)	PARAM(frmwrk, sig_ost, CY_FUZZ_OST)
#define PARAM_FLAT(frmwrk, sig_ost)	PARAM(frmwrk, sig_ost, CY_FLAT_OST)

/* abs axis signal offsets in the framworks array  */
enum cyttsp5_sig_ost {
	CY_ABS_X_OST,
	CY_ABS_Y_OST,
	CY_ABS_P_OST,
	CY_ABS_W_OST,
	CY_ABS_ID_OST,
	CY_ABS_MAJ_OST,
	CY_ABS_MIN_OST,
	CY_ABS_OR_OST,
	CY_ABS_TOOL_OST,
	CY_ABS_D_OST,
	CY_NUM_ABS_OST	/* number of abs signals */
};

enum hid_command {
	HID_CMD_RESERVED,
	HID_CMD_RESET,
	HID_CMD_GET_REPORT,
	HID_CMD_SET_REPORT,
	HID_CMD_GET_IDLE,
	HID_CMD_SET_IDLE,
	HID_CMD_GET_PROTOCOL,
	HID_CMD_SET_PROTOCOL,
	HID_CMD_SET_POWER,
	HID_CMD_VENDOR = 0xE,
};

enum hid_output_cmd_type {
	HID_OUTPUT_CMD_APP,
	HID_OUTPUT_CMD_BL,
};

enum hid_output {
	HID_OUTPUT_NULL,
	HID_OUTPUT_START_BOOTLOADER,
	HID_OUTPUT_GET_SYSINFO,
	HID_OUTPUT_SUSPEND_SCANNING,
	HID_OUTPUT_RESUME_SCANNING,
	HID_OUTPUT_GET_PARAM,
	HID_OUTPUT_SET_PARAM,
	HID_OUTPUT_GET_NOISE_METRICS,
	HID_OUTPUT_RESERVED,
	HID_OUTPUT_ENTER_EASYWAKE_STATE,
	HID_OUTPUT_VERIFY_CONFIG_BLOCK_CRC = 0x20,
	HID_OUTPUT_GET_CONFIG_ROW_SIZE,
	HID_OUTPUT_READ_CONF_BLOCK,
	HID_OUTPUT_WRITE_CONF_BLOCK,
	HID_OUTPUT_GET_DATA_STRUCTURE,
	HID_OUTPUT_LOAD_SELF_TEST_PARAM,
	HID_OUTPUT_RUN_SELF_TEST,
	HID_OUTPUT_GET_SELF_TEST_RESULT,
	HID_OUTPUT_CALIBRATE_IDACS,
	HID_OUTPUT_INITIALIZE_BASELINES,
	HID_OUTPUT_EXEC_PANEL_SCAN,
	HID_OUTPUT_RETRIEVE_PANEL_SCAN,
	HID_OUTPUT_START_SENSOR_DATA_MODE,
	HID_OUTPUT_STOP_SENSOR_DATA_MODE,
	HID_OUTPUT_START_TRACKING_HEATMAP_MODE,
	HID_OUTPUT_INT_PIN_OVERRIDE = 0x40,
	HID_OUTPUT_STORE_PANEL_SCAN = 0x60,
	HID_OUTPUT_PROCESS_PANEL_SCAN,
	HID_OUTPUT_DISCARD_INPUT_REPORT,
	HID_OUTPUT_LAST,
	HID_OUTPUT_USER_CMD,
};

/* PIP2 Command/Response data and structures */
enum PIP2_CMD_ID {
	PIP2_CMD_ID_PING,
	PIP2_CMD_ID_STATUS,
	PIP2_CMD_ID_CTRL,
	PIP2_CMD_ID_CONFIG,
	PIP2_CMD_ID_RESERVE,
	PIP2_CMD_ID_CLEAR,
	PIP2_CMD_ID_RESET,
	PIP2_CMD_ID_VERSION,
	PIP2_CMD_ID_FILE_OPEN		= 0x10,
	PIP2_CMD_ID_FILE_CLOSE,
	PIP2_CMD_ID_FILE_READ,
	PIP2_CMD_ID_FILE_WRITE,
	PIP2_CMD_ID_FILE_IOCTL,
	PIP2_CMD_ID_FLASH_INFO,
	PIP2_CMD_ID_EXECUTE,
	PIP2_CMD_ID_GET_LAST_ERRNO,
};

enum PIP2_RSP_ERR {
	PIP2_RSP_ERR_NONE               = 0x00,
	PIP2_RSP_ERR_BUSY               = 0x01,
	PIP2_RSP_ERR_INIT_FAILURE       = 0x02,
	PIP2_RSP_ERR_ALREADY_OPEN       = 0x03,
	PIP2_RSP_ERR_NOT_OPEN           = 0x04,
	PIP2_RSP_ERR_IO_FAILURE         = 0x05,
	PIP2_RSP_ERR_UNKNOWN_IOCTL      = 0x06,
	PIP2_RSP_ERR_BAD_ADDRESS        = 0x07,
	PIP2_RSP_ERR_BAD_FILE           = 0x08,
	PIP2_RSP_ERR_END_OF_FILE        = 0x09,
	PIP2_RSP_ERR_TOO_MANY_FILES     = 0x0A,
	PIP2_RSP_ERR_TIMEOUT            = 0x0B,
	PIP2_RSP_ERR_ABORTED            = 0x0C,
	PIP2_RSP_ERR_CRC                = 0x0D,
	PIP2_RSP_ERR_UNKNOWN_REC_TYPE   = 0x0E,
	PIP2_RSP_ERR_BAD_FRAME          = 0x0F,
	PIP2_RSP_ERR_NO_PERMISSION      = 0x10,
	PIP2_RSP_ERR_UNKNOWN_COMMAND    = 0x11,
	PIP2_RSP_ERR_INVALID_PARAM      = 0x12,
	PIP2_RSP_ERR_IO_ALREADY_ACTIVE  = 0x13,
	PIP2_RSP_ERR_SHUTDOWN           = 0x14,
	PIP2_RSP_ERR_INVALID_IMAGE      = 0x15,
};

/*
 * Extra bytes for PIP2 include:
 * 5 byte header - (len_lsb, len_msb, report ID, Tag, Sequence, Status)
 * 2 byte footer - (crc_lsb, crc_msb)
 *
 * Extra bytes = header + footer = 7
 */
#define PIP2_HID_OUTPUT_RESP_CMD_OFFSET		5
#define PIP2_HID_INPUT_RESP_CMD_OFFSET		3
#define PIP2_EXTRA_BYTES_NUM			7
#define PIP2_RESPONSE_SEQ_OFFSET		2
#define PIP2_RESPONSE_ID_OFFSET			3
#define PIP2_RESPONSE_STATUS_OFFSET		4
#define PIP2_RESPONSE_BODY_OFFSET		5
#define PIP2_CRC_SIZE				2
#define PIP2_LEN_FIELD_SIZE			2

/* File IOCTL commands */
#define PIP2_FILE_IOCTL_CODE_ERASE_FILE		0
#define PIP2_FILE_IOCTL_CODE_SEEK_POINTER	1
#define PIP2_FILE_IOCTL_CODE_AES_CONTROL	2

struct pip2_cmd_structure {
	u8 reg[2];
	u16 len;
	u8 id;
	u8 seq;
	u8 *data;
	u8 crc[2]; /* MSB:crc[0], LSB:crc[1] */
};

struct pip2_cmd_response_structure {
	u8 id;
	u16 response_len;
};

/* PIP BL cmd IDs and input for dut_debug sysfs */
enum hid_output_bl {
	HID_OUTPUT_BL_VERIFY_APP_INTEGRITY = 0x31,
	HID_OUTPUT_BL_GET_INFO             = 0x38,
	HID_OUTPUT_BL_PROGRAM_AND_VERIFY   = 0x39,
	HID_OUTPUT_BL_LAUNCH_APP           = 0x3B,
	HID_OUTPUT_BL_GET_PANEL_ID         = 0x3E,
	HID_OUTPUT_BL_INITIATE_BL          = 0x48,
	HID_OUTPUT_BL_LAST,
};

#define HID_OUTPUT_BL_SOP	0x1
#define HID_OUTPUT_BL_EOP	0x17

enum hid_output_bl_status {
	ERROR_SUCCESS,
	ERROR_KEY,
	ERROR_VERIFICATION,
	ERROR_LENGTH,
	ERROR_DATA,
	ERROR_COMMAND,
	ERROR_CRC = 8,
	ERROR_FLASH_ARRAY,
	ERROR_FLASH_ROW,
	ERROR_FLASH_PROTECTION,
	ERROR_UKNOWN = 15,
	ERROR_INVALID,
};

enum cyttsp5_mode {
	CY_MODE_UNKNOWN,
	CY_MODE_BOOTLOADER,
	CY_MODE_OPERATIONAL,
};

enum cyttsp5_cmd_status {
	CY_CMD_STATUS_SUCCESS,
	CY_CMD_STATUS_FAILURE,
};

enum {
	CY_IC_GRPNUM_RESERVED,
	CY_IC_GRPNUM_CMD_REGS,
	CY_IC_GRPNUM_TCH_REP,
	CY_IC_GRPNUM_DATA_REC,
	CY_IC_GRPNUM_TEST_REC,
	CY_IC_GRPNUM_PCFG_REC,
	CY_IC_GRPNUM_TCH_PARM_VAL,
	CY_IC_GRPNUM_TCH_PARM_SIZE,
	CY_IC_GRPNUM_RESERVED1,
	CY_IC_GRPNUM_RESERVED2,
	CY_IC_GRPNUM_OPCFG_REC,
	CY_IC_GRPNUM_DDATA_REC,
	CY_IC_GRPNUM_MDATA_REC,
	CY_IC_GRPNUM_TEST_REGS,
	CY_IC_GRPNUM_BTN_KEYS,
	CY_IC_GRPNUM_TTHE_REGS,
	CY_IC_GRPNUM_SENSING_CONF,
	CY_IC_GRPNUM_NUM,
};

enum cyttsp5_event_id {
	CY_EV_NO_EVENT,
	CY_EV_TOUCHDOWN,
	CY_EV_MOVE,		/* significant displacement (> act dist) */
	CY_EV_LIFTOFF,		/* record reports last position */
};

enum cyttsp5_object_id {
	CY_OBJ_STANDARD_FINGER,
	CY_OBJ_PROXIMITY,
	CY_OBJ_STYLUS,
	CY_OBJ_HOVER,
	CY_OBJ_GLOVE,
};

enum cyttsp5_self_test_result {
	CY_ST_RESULT_PASS,
	CY_ST_RESULT_FAIL,
	CY_ST_RESULT_HOST_MUST_INTERPRET = 0xFF,
};

enum cyttsp5_self_test_id {
	CY_ST_ID_NULL       = 0,
	CY_ST_ID_BIST       = 1,
	CY_ST_ID_SHORTS     = 2,
	CY_ST_ID_OPENS      = 3,
	CY_ST_ID_AUTOSHORTS = 4,
	CY_ST_ID_CM_PANEL   = 5,
	CY_ST_ID_CP_PANEL   = 6,
	CY_ST_ID_CM_BUTTON  = 7,
	CY_ST_ID_CP_BUTTON  = 8,
	CY_ST_ID_FORCE      = 9,
};

#define CY_NUM_MFGID                9

/* System Information interface definitions */
struct cyttsp5_cydata_dev {
	u8 pip_ver_major;
	u8 pip_ver_minor;
	__le16 fw_pid;
	u8 fw_ver_major;
	u8 fw_ver_minor;
	__le32 revctrl;
	__le16 fw_ver_conf;
	u8 bl_ver_major;
	u8 bl_ver_minor;
	__le16 jtag_si_id_l;
	__le16 jtag_si_id_h;
	u8 mfg_id[CY_NUM_MFGID];
	__le16 post_code;
} __packed;

struct cyttsp5_sensing_conf_data_dev {
	u8 electrodes_x;
	u8 electrodes_y;
	__le16 len_x;
	__le16 len_y;
	__le16 res_x;
	__le16 res_y;
	__le16 max_z;
	u8 origin_x;
	u8 origin_y;
	u8 panel_id;
	u8 btn;
	u8 scan_mode;
	u8 max_num_of_tch_per_refresh_cycle;
} __packed;

struct cyttsp5_cydata {
	u8 pip_ver_major;
	u8 pip_ver_minor;
	u8 bl_ver_major;
	u8 bl_ver_minor;
	u8 fw_ver_major;
	u8 fw_ver_minor;
	u16 fw_pid;
	u16 fw_ver_conf;
	u16 post_code;
	u32 revctrl;
	u16 jtag_id_l;
	u16 jtag_id_h;
	u8 mfg_id[CY_NUM_MFGID];
};

struct cyttsp5_sensing_conf_data {
	u16 res_x;
	u16 res_y;
	u16 max_z;
	u16 len_x;
	u16 len_y;
	u8 electrodes_x;
	u8 electrodes_y;
	u8 origin_x;
	u8 origin_y;
	u8 panel_id;
	u8 btn;
	u8 scan_mode;
	u8 max_tch;
	u8 rx_num;
	u8 tx_num;
};

enum cyttsp5_tch_abs {	/* for ordering within the extracted touch data array */
	CY_TCH_X,	/* X */
	CY_TCH_Y,	/* Y */
	CY_TCH_P,	/* P (Z) */
	CY_TCH_T,	/* TOUCH ID */
	CY_TCH_E,	/* EVENT ID */
	CY_TCH_O,	/* OBJECT ID */
	CY_TCH_TIP,	/* OBJECT ID */
	CY_TCH_MAJ,	/* TOUCH_MAJOR */
	CY_TCH_MIN,	/* TOUCH_MINOR */
	CY_TCH_OR,	/* ORIENTATION */
	CY_TCH_NUM_ABS,
};

enum cyttsp5_tch_hdr {
	CY_TCH_TIME,	/* SCAN TIME */
	CY_TCH_NUM,	/* NUMBER OF RECORDS */
	CY_TCH_LO,	/* LARGE OBJECT */
	CY_TCH_NOISE,	/* NOISE EFFECT */
	CY_TCH_COUNTER,	/* REPORT_COUNTER */
	CY_TCH_NUM_HDR,
};

static const char * const cyttsp5_tch_abs_string[] = {
	[CY_TCH_X]	= "X",
	[CY_TCH_Y]	= "Y",
	[CY_TCH_P]	= "P",
	[CY_TCH_T]	= "T",
	[CY_TCH_E]	= "E",
	[CY_TCH_O]	= "O",
	[CY_TCH_TIP]	= "TIP",
	[CY_TCH_MAJ]	= "MAJ",
	[CY_TCH_MIN]	= "MIN",
	[CY_TCH_OR]	= "OR",
	[CY_TCH_NUM_ABS] = "INVALID",
};

static const char * const cyttsp5_tch_hdr_string[] = {
	[CY_TCH_TIME]	= "SCAN TIME",
	[CY_TCH_NUM]	= "NUMBER OF RECORDS",
	[CY_TCH_LO]	= "LARGE OBJECT",
	[CY_TCH_NOISE]	= "NOISE EFFECT",
	[CY_TCH_COUNTER] = "REPORT_COUNTER",
	[CY_TCH_NUM_HDR] = "INVALID",
};

static const int cyttsp5_tch_abs_field_map[] = {
	[CY_TCH_X]	= 0x00010030 /* HID_GD_X */,
	[CY_TCH_Y]	= 0x00010031 /* HID_GD_Y */,
	[CY_TCH_P]	= HID_DI_PRESSURE,
	[CY_TCH_T]	= HID_DI_CONTACTID,
	[CY_TCH_E]	= HID_CY_EVENTID,
	[CY_TCH_O]	= HID_CY_TOUCHTYPE,
	[CY_TCH_TIP]	= HID_DI_TIP,
	[CY_TCH_MAJ]	= HID_CY_MAJORAXISLENGTH,
	[CY_TCH_MIN]	= HID_CY_MINORAXISLENGTH,
	[CY_TCH_OR]	= HID_CY_ORIENTATION,
	[CY_TCH_NUM_ABS] = 0,
};

static const int cyttsp5_tch_hdr_field_map[] = {
	[CY_TCH_TIME]	= HID_DI_SCANTIME,
	[CY_TCH_NUM]	= HID_DI_CONTACTCOUNT,
	[CY_TCH_LO]	= HID_CY_LARGEOBJECT,
	[CY_TCH_NOISE]	= HID_CY_NOISEEFFECTS,
	[CY_TCH_COUNTER] = HID_CY_REPORTCOUNTER,
	[CY_TCH_NUM_HDR] = 0,
};

#define CY_NUM_EXT_TCH_FIELDS   3

struct cyttsp5_tch_abs_params {
	size_t ofs;	/* abs byte offset */
	size_t size;	/* size in bits */
	size_t min;	/* min value */
	size_t max;	/* max value */
	size_t bofs;	/* bit offset */
	u8 report;
};

struct cyttsp5_touch {
	int hdr[CY_TCH_NUM_HDR];
	int abs[CY_TCH_NUM_ABS];
};

/* button to keycode support */
#define CY_BITS_PER_BTN		1
#define CY_NUM_BTN_EVENT_ID	((1 << CY_BITS_PER_BTN) - 1)

enum cyttsp5_btn_state {
	CY_BTN_RELEASED = 0,
	CY_BTN_PRESSED = 1,
	CY_BTN_NUM_STATE
};

struct cyttsp5_btn {
	bool enabled;
	int state;	/* CY_BTN_PRESSED, CY_BTN_RELEASED */
	int key_code;
};

enum cyttsp5_ic_ebid {
	CY_TCH_PARM_EBID,
	CY_MDATA_EBID,
	CY_DDATA_EBID,
};

/* ttconfig block */
#define CY_TTCONFIG_VERSION_OFFSET	8
#define CY_TTCONFIG_VERSION_SIZE	2
#define CY_TTCONFIG_VERSION_ROW		0

struct cyttsp5_ttconfig {
	u16 version;
	u16 crc;
};

struct cyttsp5_report_desc_data {
	u16 tch_report_id;
	u16 tch_record_size;
	u16 tch_header_size;
	u16 btn_report_id;
};

struct cyttsp5_sysinfo {
	bool ready;
	struct cyttsp5_cydata cydata;
	struct cyttsp5_sensing_conf_data sensing_conf_data;
	struct cyttsp5_report_desc_data desc;
	int num_btns;
	struct cyttsp5_btn *btn;
	struct cyttsp5_ttconfig ttconfig;
	struct cyttsp5_tch_abs_params tch_hdr[CY_TCH_NUM_HDR];
	struct cyttsp5_tch_abs_params tch_abs[CY_TCH_NUM_ABS];
	u8 *xy_mode;
	u8 *xy_data;
};

enum cyttsp5_atten_type {
	CY_ATTEN_IRQ,
	CY_ATTEN_STARTUP,
	CY_ATTEN_EXCLUSIVE,
	CY_ATTEN_WAKE,
	CY_ATTEN_LOADER,
	CY_ATTEN_SUSPEND,
	CY_ATTEN_RESUME,
	CY_ATTEN_NUM_ATTEN,
};

enum cyttsp5_sleep_state {
	SS_SLEEP_OFF,
	SS_SLEEP_ON,
	SS_SLEEPING,
	SS_WAKING,
};

enum cyttsp5_fb_state {
	FB_ON,
	FB_OFF,
};

enum cyttsp5_startup_state {
	STARTUP_NONE,
	STARTUP_QUEUED,
	STARTUP_RUNNING,
	STARTUP_ILLEGAL,
};

struct cyttsp5_hid_desc {
	__le16 hid_desc_len;
	u8 packet_id;
	u8 reserved_byte;
	__le16 bcd_version;
	__le16 report_desc_len;
	__le16 report_desc_register;
	__le16 input_register;
	__le16 max_input_len;
	__le16 output_register;
	__le16 max_output_len;
	__le16 command_register;
	__le16 data_register;
	__le16 vendor_id;
	__le16 product_id;
	__le16 version_id;
	u8 reserved[4];
} __packed;

struct cyttsp5_hid_core {
	u16 hid_vendor_id;
	u16 hid_product_id;
	__le16 hid_desc_register;
	u16 hid_report_desc_len;
	u16 hid_max_input_len;
	u16 hid_max_output_len;
};

#define CY_HID_MAX_REPORTS		8
#define CY_HID_MAX_FIELDS		128
#define CY_HID_MAX_COLLECTIONS		3
#define CY_HID_MAX_NESTED_COLLECTIONS	CY_HID_MAX_COLLECTIONS

#define CY_MAX_INPUT		512
#define CY_PIP_1P7_EMPTY_BUF    0xFF00

enum cyttsp5_module_id {
	CY_MODULE_MT,
	CY_MODULE_BTN,
	CY_MODULE_PROX,
	CY_MODULE_LAST,
};

struct cyttsp5_mt_data;
struct cyttsp5_mt_function {
	int (*mt_release)(struct device *dev);
	int (*mt_probe)(struct device *dev, struct cyttsp5_mt_data *md);
	void (*report_slot_liftoff)(struct cyttsp5_mt_data *md, int max_slots);
	void (*input_sync)(struct input_dev *input);
	void (*input_report)(struct input_dev *input, int sig, int t, int type);
	void (*final_sync)(struct input_dev *input, int max_slots,
			int mt_sync_count, unsigned long *ids);
	int (*input_register_device)(struct input_dev *input, int max_slots);
};

struct cyttsp5_mt_data {
	struct device *dev;
	struct cyttsp5_mt_platform_data *pdata;
	struct cyttsp5_sysinfo *si;
	struct input_dev *input;
	struct cyttsp5_mt_function mt_function;
	struct mutex mt_lock;
	bool is_suspended;
	bool input_device_registered;
	char phys[NAME_MAX];
	int num_prv_rec;
	int or_min;
	int or_max;
	int t_min;
	int t_max;
};

struct cyttsp5_btn_data {
	struct device *dev;
	struct cyttsp5_btn_platform_data *pdata;
	struct cyttsp5_sysinfo *si;
	struct input_dev *input;
	struct mutex btn_lock;
	bool is_suspended;
	bool input_device_registered;
	char phys[NAME_MAX];
};

struct cyttsp5_proximity_data {
	struct device *dev;
	struct cyttsp5_proximity_platform_data *pdata;
	struct cyttsp5_sysinfo *si;
	struct input_dev *input;
	struct mutex prox_lock;
	struct mutex sysfs_lock;
	int enable_count;
	bool input_device_registered;
	char phys[NAME_MAX];
};

enum cyttsp5_calibrate_idacs_sensing_mode {
	CY_CI_SM_MUTCAP_FINE,
	CY_CI_SM_MUTCAP_BUTTON,
	CY_CI_SM_SELFCAP,
};

enum cyttsp5_initialize_baselines_sensing_mode {
	CY_IB_SM_MUTCAP = 1,
	CY_IB_SM_BUTTON = 2,
	CY_IB_SM_SELFCAP = 4,
	CY_IB_SM_BALANCED = 8,
};

struct cyttsp5_core_nonhid_cmd {
	int (*start_bl)(struct device *dev, int protect);
	int (*suspend_scanning)(struct device *dev, int protect);
	int (*resume_scanning)(struct device *dev, int protect);
	int (*get_param)(struct device *dev, int protect, u8 param_id,
			u32 *value);
	int (*set_param)(struct device *dev, int protect, u8 param_id,
			u32 value, u8 size);
	int (*verify_config_block_crc)(struct device *dev, int protect,
			u8 ebid, u8 *status, u16 *calculated_crc,
			u16 *stored_crc);
	int (*get_config_row_size)(struct device *dev, int protect,
			u16 *row_size);
	int (*get_data_structure)(struct device *dev, int protect,
			u16 read_offset, u16 read_length, u8 data_id,
			u8 *status, u8 *data_format, u16 *actual_read_len,
			u8 *data);
	int (*run_selftest)(struct device *dev, int protect, u8 test_id,
		u8 write_idacs_to_flash, u8 *status, u8 *summary_result,
		u8 *results_available);
	int (*get_selftest_result)(struct device *dev, int protect,
		u16 read_offset, u16 read_length, u8 test_id, u8 *status,
		u16 *actual_read_len, u8 *data);
	int (*calibrate_idacs)(struct device *dev, int protect, u8 mode,
			u8 *status);
	int (*initialize_baselines)(struct device *dev, int protect,
			u8 test_id, u8 *status);
	int (*exec_panel_scan)(struct device *dev, int protect);
	int (*retrieve_panel_scan)(struct device *dev, int protect,
			u16 read_offset, u16 read_count, u8 data_id,
			u8 *response, u8 *config, u16 *actual_read_len,
			u8 *read_buf);
	int (*write_conf_block)(struct device *dev, int protect,
			u16 row_number, u16 write_length, u8 ebid,
			u8 *write_buf, u8 *security_key, u16 *actual_write_len);
	int (*user_cmd)(struct device *dev, int protect, u16 read_len,
			u8 *read_buf, u16 write_len, u8 *write_buf,
			u16 *actual_read_len);
	int (*get_bl_info)(struct device *dev, int protect, u8 *return_data);
	int (*initiate_bl)(struct device *dev, int protect, u16 key_size,
			u8 *key_buf, u16 row_size, u8 *metadata_row_buf);
	int (*launch_app)(struct device *dev, int protect);
	int (*prog_and_verify)(struct device *dev, int protect, u16 data_len,
			u8 *data_buf);
	int (*verify_app_integrity)(struct device *dev, int protect,
			u8 *result);
	int (*get_panel_id)(struct device *dev, int protect, u8 *panel_id);
	int (*pip2_send_cmd)(struct device *dev,
			struct pip2_cmd_structure *pip2_cmd, u8 id, u8 *data,
			u8 report_body_len, u8 *read_buf, u16 *actual_read_len);
	int (*get_bl_pip2_version)(struct device *dev);
};

typedef int (*cyttsp5_atten_func) (struct device *);

struct pt_core_commands {
	int (*subscribe_attention)(struct device *dev,
			enum cyttsp5_atten_type type, char *id,
			cyttsp5_atten_func func, int flags);
	int (*unsubscribe_attention)(struct device *dev,
			enum cyttsp5_atten_type type, char *id,
			cyttsp5_atten_func func, int flags);
	int (*request_exclusive)(struct device *dev, int timeout_ms);
	int (*release_exclusive)(struct device *dev);
	int (*request_reset)(struct device *dev);
	int (*request_restart)(struct device *dev, bool wait);
	struct cyttsp5_sysinfo * (*request_sysinfo)(struct device *dev);
	struct cyttsp5_loader_platform_data
		*(*request_loader_pdata)(struct device *dev);
	int (*request_stop_wd)(struct device *dev);
	int (*request_start_wd)(struct device *dev);
	int (*request_get_hid_desc)(struct device *dev, int protect);
	int (*request_get_mode)(struct device *dev, int protect, u8 *mode);
	int (*request_pip2_get_mode)(struct device *dev, u8 *mode,
		bool resume_scan);
	int (*request_active_pip_prot)(struct device *dev,
		u8 *pip_version_major, u8 *pip_version_minor);
	int (*request_enable_scan_type)(struct device *dev, u8 scan_type);
	int (*request_disable_scan_type)(struct device *dev, u8 scan_type);
	struct cyttsp5_core_nonhid_cmd *nonhid_cmd;
#ifdef TTHE_TUNER_SUPPORT
	int (*request_tthe_print)(struct device *dev, u8 *buf, int buf_len,
			const u8 *data_name);
#endif
};

struct cyttsp5_features {
	uint8_t easywake;
	uint8_t noise_metric;
	uint8_t tracking_heatmap;
	uint8_t sensor_data;
};

#define NEED_SUSPEND_NOTIFIER \
	((LINUX_VERSION_CODE < KERNEL_VERSION(3, 3, 0)) \
	&& defined(CONFIG_PM_SLEEP) && defined(CONFIG_PM_RUNTIME))

struct cyttsp5_module {
	struct list_head node;
	char *name;
	int (*probe)(struct device *dev, void **data);
	void (*release)(struct device *dev, void *data);
};

struct cyttsp5_core_data {
	struct list_head node;
	struct list_head module_list; /* List of probed modules */
	char core_id[20];
	struct device *dev;
	struct list_head atten_list[CY_ATTEN_NUM_ATTEN];
	struct list_head param_list;
	struct mutex module_list_lock;
	struct mutex system_lock;
	struct mutex adap_lock;
	struct mutex hid_report_lock;
	enum cyttsp5_mode mode;
	spinlock_t spinlock;
	struct cyttsp5_mt_data md;
	struct cyttsp5_btn_data bd;
	struct cyttsp5_proximity_data pd;
	int phys_num;
	int number_of_open_input_device;
	int pm_runtime_usage_count;
	void *cyttsp5_dynamic_data[CY_MODULE_LAST];
	struct cyttsp5_platform_data *pdata;
	struct cyttsp5_core_platform_data *cpdata;
	const struct cyttsp5_bus_ops *bus_ops;
	wait_queue_head_t wait_q;
	enum cyttsp5_sleep_state sleep_state;
	enum cyttsp5_startup_state startup_state;
	int irq;
	bool irq_enabled;
	bool irq_wake;
	bool irq_disabled;
	bool hw_detected;
	bool sysfs_nodes_created;
	u8 easy_wakeup_gesture;
#ifdef EASYWAKE_TSG6
	u8 gesture_id;
	u8 gesture_data_length;
	u8 gesture_data[80];
#endif
	bool wake_initiated_by_device;
	bool wait_until_wake;
	u8 panel_id;
#if NEED_SUSPEND_NOTIFIER
	/*
	 * This notifier is used to receive suspend prepare events
	 * When device is PM runtime suspended, pm_generic_suspend()
	 * does not call our PM suspend callback for kernels with
	 * version less than 3.3.0.
	 */
	struct notifier_block pm_notifier;
#endif
	struct work_struct startup_work;
	struct cyttsp5_sysinfo sysinfo;
	void *exclusive_dev;
	int exclusive_waits;
	struct work_struct watchdog_work;
	struct timer_list watchdog_timer;
#ifdef PT_PTSBC_SUPPORT
	struct work_struct irq_work;
	struct work_struct probe_work;
	struct timer_list probe_timer;
#endif
	u16 startup_retry_count;
	struct cyttsp5_hid_core hid_core;
	int hid_cmd_state;
	int hid_reset_cmd_state; /* reset can happen any time */
	struct cyttsp5_hid_desc hid_desc;
	struct cyttsp5_hid_report *hid_reports[CY_HID_MAX_REPORTS];
	int num_hid_reports;
	struct cyttsp5_features features;
#define CYTTSP5_PREALLOCATED_CMD_BUFFER 32
	u8 cmd_buf[CYTTSP5_PREALLOCATED_CMD_BUFFER];
	u8 input_buf[CY_MAX_INPUT];
	u8 response_buf[CY_MAX_INPUT];
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend es;
#elif defined(CONFIG_FB)
	struct notifier_block fb_notifier;
	enum cyttsp5_fb_state fb_state;
#endif
#ifdef TTHE_TUNER_SUPPORT
	struct dentry *tthe_debugfs;
	u8 *tthe_buf;
	u32 tthe_buf_len;
	struct mutex tthe_lock;
	u8 tthe_exit;
#endif
#ifdef VERBOSE_DEBUG
	/* TODO - clean up use of pr_buf */
	u8 pr_buf[CY_MAX_PRBUF_SIZE];
#endif
	u8 debug_level;
	u8 watchdog_enabled;
	u32 watchdog_interval;
	u8 show_timestamp;
	u32 startup_status;
	u8 pip2_cmd_tag_seq;
	u8 use_tc3300_pip20_len_field; /* PIP2.0 (TC3300) Len field */
	u8 pip_2p0_prot_active;
	u8 pip2_prot_active;
	u8 route_i2c_virt_dut;
	u8 get_param_id;
	u8 bl_pip_version_major;
	u8 bl_pip_version_minor;
	u8 wd_corrective_action;
#ifdef TTDL_DIAGNOSTICS
	u8 t_refresh_active;
	u16 i2c_crc_error_count;
	u16 t_refresh_count;
	u16 t_refresh_total;
	u16 hid_reset_count;
	u32 watchdog_count;
	u32 watchdog_irq_stuck_count;
	u32 watchdog_failed_access_count;
	u32 i2c_transmission_error_count;
	u32 irq_count;
	u32 bl_retry_packet_count;
	unsigned long t_refresh_time;
#endif
};

struct gd_sensor {
	int32_t cm_min;
	int32_t cm_max;
	int32_t cm_ave;
	int32_t cm_min_exclude_edge;
	int32_t cm_max_exclude_edge;
	int32_t cm_ave_exclude_edge;
	int32_t gradient_val;
};

#ifdef TTHE_TUNER_SUPPORT
#define CY_CMD_RET_PANEL_IN_DATA_OFFSET	0
#define CY_CMD_RET_PANEL_ELMNT_SZ_MASK	0x07
#define CY_CMD_RET_PANEL_HDR		0x0A
#define CY_CMD_RET_PANEL_ELMNT_SZ_MAX	0x2

enum scan_data_type_list {
	CY_MUT_RAW,
	CY_MUT_BASE,
	CY_MUT_DIFF,
	CY_SELF_RAW,
	CY_SELF_BASE,
	CY_SELF_DIFF,
	CY_BAL_RAW,
	CY_BAL_BASE,
	CY_BAL_DIFF,
};
#endif

struct cyttsp5_bus_ops {
	u16 bustype;

	int (*read_default)(struct device *dev, void *buf, int size);
	int (*read_default_nosize)(struct device *dev, u8 *buf, u32 max);
	int (*write_read_specific)(struct device *dev, u8 write_len,
			u8 *write_buf, u8 *read_buf);
};

static inline int cyttsp5_adap_read_default(struct cyttsp5_core_data *cd,
		void *buf, int size)
{
	return cd->bus_ops->read_default(cd->dev, buf, size);
}

static inline int cyttsp5_adap_read_default_nosize(struct cyttsp5_core_data *cd,
		void *buf, int max)
{
	return cd->bus_ops->read_default_nosize(cd->dev, buf, max);
}

static inline int cyttsp5_adap_write_read_specific(struct cyttsp5_core_data *cd,
		u8 write_len, u8 *write_buf, u8 *read_buf)
{
	return cd->bus_ops->write_read_specific(cd->dev, write_len, write_buf,
			read_buf);
}

static inline void *cyttsp5_get_dynamic_data(struct device *dev, int id)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);

	return cd->cyttsp5_dynamic_data[id];
}

int request_exclusive(struct cyttsp5_core_data *cd, void *ownptr,
		int timeout_ms);
int release_exclusive(struct cyttsp5_core_data *cd, void *ownptr);
int _cyttsp5_request_hid_output_get_param(struct device *dev,
		int protect, u8 param_id, u32 *value);
int _cyttsp5_request_hid_output_set_param(struct device *dev,
		int protect, u8 param_id, u32 value, u8 size);

static inline int cyttsp5_request_exclusive(struct device *dev, int timeout_ms)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);

	return request_exclusive(cd, dev, timeout_ms);
}

static inline int cyttsp5_release_exclusive(struct device *dev)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);

	return release_exclusive(cd, dev);
}

static inline int cyttsp5_request_nonhid_get_param(struct device *dev,
		int protect, u8 param_id, u32 *value)
{
	return _cyttsp5_request_hid_output_get_param(dev, protect, param_id,
			value);
}

static inline int cyttsp5_request_nonhid_set_param(struct device *dev,
		int protect, u8 param_id, u32 value, u8 size)
{
	return _cyttsp5_request_hid_output_set_param(dev, protect, param_id,
			value, size);
}

void pt_pr_buf(struct device *dev, u8 debug_level, u8 *buf,
	u16 buf_len, const char *data_name);

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_DEVICETREE_SUPPORT
int cyttsp5_devtree_create_and_get_pdata(struct device *adap_dev);
int cyttsp5_devtree_clean_pdata(struct device *adap_dev);
#else
static inline int cyttsp5_devtree_create_and_get_pdata(struct device *adap_dev)
{
	return 0;
}

static inline int cyttsp5_devtree_clean_pdata(struct device *adap_dev)
{
	return 0;
}
#endif

int pt_probe(const struct cyttsp5_bus_ops *ops, struct device *dev,
		u16 irq, size_t xfer_buf_size);
int cyttsp5_release(struct cyttsp5_core_data *cd);

struct pt_core_commands *pt_get_commands(void);
struct cyttsp5_core_data *cyttsp5_get_core_data(char *id);

int cyttsp5_mt_release(struct device *dev);
int cyttsp5_mt_probe(struct device *dev);

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_BUTTON
int cyttsp5_btn_probe(struct device *dev);
int cyttsp5_btn_release(struct device *dev);
#else
static inline int cyttsp5_btn_probe(struct device *dev) { return 0; }
static inline int cyttsp5_btn_release(struct device *dev) { return 0; }
#endif

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_PROXIMITY
int cyttsp5_proximity_probe(struct device *dev);
int cyttsp5_proximity_release(struct device *dev);
#else
static inline int cyttsp5_proximity_probe(struct device *dev) { return 0; }
static inline int cyttsp5_proximity_release(struct device *dev) { return 0; }
#endif

void cyttsp5_init_function_ptrs(struct cyttsp5_mt_data *md);
int _pt_subscribe_attention(struct device *dev,
	enum cyttsp5_atten_type type, char *id, int (*func)(struct device *),
	int mode);
int _pt_unsubscribe_attention(struct device *dev,
	enum cyttsp5_atten_type type, char *id, int (*func)(struct device *),
	int mode);
struct cyttsp5_sysinfo *_pt_request_sysinfo(struct device *dev);

extern const struct dev_pm_ops cyttsp5_pm_ops;

int cyttsp5_register_module(struct cyttsp5_module *module);
void cyttsp5_unregister_module(struct cyttsp5_module *module);

void *cyttsp5_get_module_data(struct device *dev,
		struct cyttsp5_module *module);

#endif /* _CYTTSP5_REGS_H */
