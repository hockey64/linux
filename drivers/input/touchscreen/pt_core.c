/*
 * pt_core.c
 * Parade TrueTouch(TM) Standard Product V5 Core Module.
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
#include <linux/kthread.h>

/* This section is needed for the PtSBC in how it handles I2C and IRQs */
#ifdef PT_PTSBC_SUPPORT
#include <mach/irqs.h>
#include <mach/system.h>
#include <mach/hardware.h>
#include <mach/sys_config.h>
#include <mach/gpio.h>
#include <linux/init-input.h>
#include <mach/i2c.h>
#define PT_CORE_PROBE_STARTUP_DELAY_MS		500
static struct workqueue_struct *parade_wq;
#else  /* --- PT_PTSBC_SUPPORT --- */
#include <linux/i2c.h>
#include <linux/gpio.h>
#endif /* --- PT_PTSBC_SUPPORT --- */

#define PT_CORE_STARTUP_RETRY_COUNT		3

MODULE_FIRMWARE(PT_FW_FILE_NAME);

static const char *pt_driver_core_name = PT_CORE_NAME;
static const char *pt_driver_core_version = PT_DRIVER_VERSION;
static const char *pt_driver_core_date = PT_DRIVER_DATE;
static const struct pt_bus_ops *pt_bus_ops_save;

struct pt_hid_field {
	int report_count;
	int report_size;
	int size; /* report_count * report_size */
	int offset;
	int data_type;
	int logical_min;
	int logical_max;
	/* Usage Page (Hi 16 bit) + Usage (Lo 16 bit) */
	u32 usage_page;
	u32 collection_usage_pages[PT_HID_MAX_COLLECTIONS];
	struct pt_hid_report *report;
	bool record_field;
};

struct pt_hid_report {
	u8 id;
	u8 type;
	int size;
	struct pt_hid_field *fields[PT_HID_MAX_FIELDS];
	int num_fields;
	int record_field_index;
	int header_size;
	int record_size;
	u32 usage_page;
};

struct atten_node {
	struct list_head node;
	char *id;
	struct device *dev;

	int (*func)(struct device *);
	int mode;
};

struct param_node {
	struct list_head node;
	u8 id;
	u32 value;
	u8 size;
};

struct module_node {
	struct list_head node;
	struct pt_module *module;
	void *data;
};

struct pt_hid_cmd {
	u8 opcode;
	u8 report_type;
	union {
		u8 report_id;
		u8 power_state;
	};
	u8 has_data_register;
	size_t write_length;
	u8 *write_buf;
	u8 *read_buf;
	u8 wait_interrupt;
	u8 reset_cmd;
	u16 timeout_ms;
};

struct pt_hid_output {
	u8 cmd_type;
	u16 length;
	u8 command_code;
	size_t write_length;
	u8 *write_buf;
	u8 novalidate;
	u8 reset_expected;
	u16 timeout_ms;
};

#define SET_CMD_OPCODE(byte, opcode) SET_CMD_LOW(byte, opcode)
#define SET_CMD_REPORT_TYPE(byte, type) SET_CMD_HIGH(byte, ((type) << 4))
#define SET_CMD_REPORT_ID(byte, id) SET_CMD_LOW(byte, id)

#define HID_OUTPUT_APP_COMMAND(command) \
	.cmd_type = HID_OUTPUT_CMD_APP, \
	.command_code = command

#define HID_OUTPUT_BL_COMMAND(command) \
	.cmd_type = HID_OUTPUT_CMD_BL, \
	.command_code = command

/*******************************************************************************
 * FUNCTION: pt_pr_buf
 *
 * SUMMARY: Print out the contents of a buffer to kmsg based on the debug level
 *
 * RETURN: Void
 *
 * PARAMETERS:
 *	*dev         - pointer to Device structure
 *	 debug_level - requested debug level to print at
 *	*buf         - pointer to buffer to print
 *	 buf_len     - size of buf
 *	*data_name   - Descriptive name of data prefixed to data
 ******************************************************************************/
void pt_pr_buf(struct device *dev, u8 debug_level, u8 *buf,
	u16 buf_len, const char *data_name)
{
	struct pt_core_data *cd = dev_get_drvdata(dev);
	int i;
	int pr_buf_index = 0;
	int max_size;

	/* only proceed if valid debug level and there is data to print */
	if (debug_level <= cd->debug_level && buf_len > 0) {
		char pr_buf[1018];

		/*
		 * Subtract size of name, 11 bytes for " [0..xxx]: " and
		 * one space in between each data byte (buf_len)
		 */
		max_size = sizeof(pr_buf) - sizeof(data_name) - 11 - buf_len;

		/* Ensure pr_buf_index stays within the 1024 size */
		pr_buf_index += sprintf(pr_buf, "%s [0..%d]: ",
			data_name, buf_len);
		for (i = 0; i < buf_len && pr_buf_index < max_size; i++)
			pr_buf_index += sprintf(pr_buf + pr_buf_index,
				"%02X ", buf[i]);

		pt_debug(dev, debug_level, "%s\n", pr_buf);
	}
}
EXPORT_SYMBOL_GPL(pt_pr_buf);

#ifdef TTHE_TUNER_SUPPORT
/*******************************************************************************
 * FUNCTION: tthe_print
 *
 * SUMMARY: Format data name and time stamp as the header and format the
 *  content of input buffer with hex base to "tthe_buf". And then wait up event
 *  semaphore for tthe debugfs node.
 *
 * RETURN:
 *   0 = success
 *  !0 = failure
 *
 * PARAMETERS:
 *  *cd         - pointer to core data
 *  *buf        - pointer to input buffer
 *   buf_len    - size of input buffer
 *  *data_name  - pointer to data name
 ******************************************************************************/
static int tthe_print(struct pt_core_data *cd, u8 *buf, int buf_len,
		const u8 *data_name)
{
	int len = strlen(data_name);
	int i, n;
	u8 *p;
	int remain;
	u8 data_name_with_time_stamp[100];

	/* Prepend timestamp, if requested, to data_name */
	if (cd->show_timestamp) {
		sprintf(data_name_with_time_stamp, "[%u] %s",
			jiffies_to_msecs(jiffies), data_name);
		data_name = data_name_with_time_stamp;
		len = strlen(data_name);
	}

	mutex_lock(&cd->tthe_lock);
	if (!cd->tthe_buf)
		goto exit;

	if (cd->tthe_buf_len + (len + buf_len) > PT_MAX_PRBUF_SIZE)
		goto exit;

	if (len + buf_len == 0)
		goto exit;

	remain = PT_MAX_PRBUF_SIZE - cd->tthe_buf_len;
	if (remain < len)
		len = remain;

	p = cd->tthe_buf + cd->tthe_buf_len;
	memcpy(p, data_name, len);
	cd->tthe_buf_len += len;
	p += len;
	remain -= len;

	*p = 0;
	for (i = 0; i < buf_len; i++) {
		n = scnprintf(p, remain, "%02X ", buf[i]);
		if (!n)
			break;
		p += n;
		remain -= n;
		cd->tthe_buf_len += n;
	}

	n = scnprintf(p, remain, "\n");
	if (!n)
		cd->tthe_buf[cd->tthe_buf_len] = 0;
	cd->tthe_buf_len += n;
	wake_up(&cd->wait_q);
exit:
	mutex_unlock(&cd->tthe_lock);
	return 0;
}

/*******************************************************************************
 * FUNCTION: _pt_request_tthe_print
 *
 * SUMMARY: Function pointer included in core_cmds to allow other modules
 *	to request to print data to the "tthe_buffer".
 *
 * RETURN:
 *   0 = success
 *  !0 = failure
 *
 * PARAMETERS:
 *  *dev - pointer to device structure
 ******************************************************************************/
static int _pt_request_tthe_print(struct device *dev, u8 *buf,
		int buf_len, const u8 *data_name)
{
	struct pt_core_data *cd = dev_get_drvdata(dev);

	return tthe_print(cd, buf, buf_len, data_name);
}
#endif

/*******************************************************************************
 * FUNCTION: pt_platform_detect_read
 *
 * SUMMARY: To be passed to platform dectect function to perform a read
 *  operation.
 *
 * RETURN:
 *   0 = success
 *  !0 = failure
 *
 * PARAMETERS:
 *  *dev  - pointer to Device structure
 *  *buf  - pointer to buffer where the data read will be stored
 *   size - size to be read
 ******************************************************************************/
static int pt_platform_detect_read(struct device *dev, void *buf, int size)
{
	struct pt_core_data *cd = dev_get_drvdata(dev);

	return pt_adap_read_default(cd, buf, size);
}

/*******************************************************************************
 * FUNCTION: pt_add_parameter
 *
 * SUMMARY: Adds a parameter that has been altered to the parameter linked list.
 *	On every reset of the DUT this linked list is traversed and all
 *	parameters in it are restored to the DUT.
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*cd          - pointer to core data
 *	 param_id    - parameter ID to add
 *	 param_value - Value corresponding to the ID
 *	 param_size  - Size of param_value
 ******************************************************************************/
static int pt_add_parameter(struct pt_core_data *cd,
		u8 param_id, u32 param_value, u8 param_size)
{
	struct param_node *param, *param_new;

	/* Check if parameter already exists in the list */
	spin_lock(&cd->spinlock);
	list_for_each_entry(param, &cd->param_list, node) {
		if (param->id == param_id) {
			/* Update parameter */
			param->value = param_value;
			pt_debug(cd->dev, DL_INFO,
				"%s: Update parameter id:%d value:%d size:%d\n",
				 __func__, param_id, param_value, param_size);
			goto exit_unlock;
		}
	}
	spin_unlock(&cd->spinlock);

	param_new = kzalloc(sizeof(*param_new), GFP_KERNEL);
	if (!param_new)
		return -ENOMEM;

	param_new->id = param_id;
	param_new->value = param_value;
	param_new->size = param_size;

	pt_debug(cd->dev, DL_INFO,
		"%s: Add parameter id:%d value:%d size:%d\n",
		__func__, param_id, param_value, param_size);

	spin_lock(&cd->spinlock);
	list_add(&param_new->node, &cd->param_list);
exit_unlock:
	spin_unlock(&cd->spinlock);

	return 0;
}

/*******************************************************************************
 * FUNCTION: pt_erase_parameter_list
 *
 * SUMMARY: Empty out the entire parameter linked list of all parameter/value
 *	pairs. In some test cases this functionality is needed to ensure DUT
 *	returns to a virgin state after a reset and no parameters are restored.
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*cd - pointer to core data
 ******************************************************************************/
static int pt_erase_parameter_list(struct pt_core_data *cd)
{
	struct param_node *pos, *temp;

	spin_lock(&cd->spinlock);
	list_for_each_entry_safe(pos, temp, &cd->param_list, node) {
		pt_debug(cd->dev, DL_WARN,
			"%s: Parameter Restore List - remove 0x%02x\n",
			__func__, pos->id);
		list_del(&pos->node);
		kfree(pos);
	}
	spin_unlock(&cd->spinlock);

	return 0;
}

/*******************************************************************************
 * FUNCTION: pt_count_parameter_list
 *
 * SUMMARY: Count the items in the RAM parameter restor list
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*cd - pointer to core data
 ******************************************************************************/
static int pt_count_parameter_list(struct pt_core_data *cd)
{
	struct param_node *pos, *temp;
	int entries = 0;

	spin_lock(&cd->spinlock);
	list_for_each_entry_safe(pos, temp, &cd->param_list, node)
		entries++;
	spin_unlock(&cd->spinlock);

	return entries;
}


/*******************************************************************************
 * FUNCTION: request_exclusive
 *
 * SUMMARY: Request exclusive access to the DUT
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*cd         - pointer to core data
 *	*ownptr     - pointer to device
 *	 timeout_ms - Timeout value
 ******************************************************************************/
int request_exclusive(struct pt_core_data *cd, void *ownptr,
		int timeout_ms)
{
	int t = msecs_to_jiffies(timeout_ms);
	bool with_timeout = (timeout_ms != 0);

	mutex_lock(&cd->system_lock);
	if (!cd->exclusive_dev && cd->exclusive_waits == 0) {
		cd->exclusive_dev = ownptr;
		goto exit;
	}

	cd->exclusive_waits++;
wait:
	mutex_unlock(&cd->system_lock);
	if (with_timeout) {
		t = wait_event_timeout(cd->wait_q, !cd->exclusive_dev, t);
		if (IS_TMO(t)) {
			pt_debug(cd->dev, DL_ERROR,
				"%s: tmo waiting exclusive access\n", __func__);
			return -ETIME;
		}
	} else {
		wait_event(cd->wait_q, !cd->exclusive_dev);
	}
	mutex_lock(&cd->system_lock);
	if (cd->exclusive_dev)
		goto wait;
	cd->exclusive_dev = ownptr;
	cd->exclusive_waits--;
exit:
	mutex_unlock(&cd->system_lock);
	pt_debug(cd->dev, DL_DEBUG, "%s: request_exclusive ok=%p\n",
		__func__, ownptr);

	return 0;
}

/*******************************************************************************
 * FUNCTION: release_exclusive_
 *
 * SUMMARY: Release exclusive access to the DUT
 *
 * RETURN:
 *	 0 = success
 *
 * PARAMETERS:
 *	*cd         - pointer to core data
 *	*ownptr     - pointer to device
 ******************************************************************************/
static int release_exclusive_(struct pt_core_data *cd, void *ownptr)
{
	if (cd->exclusive_dev != ownptr)
		return -EINVAL;

	pt_debug(cd->dev, DL_DEBUG, "%s: exclusive_dev %p freed\n",
		__func__, cd->exclusive_dev);
	cd->exclusive_dev = NULL;
	wake_up(&cd->wait_q);
	return 0;
}

/*******************************************************************************
 * FUNCTION: release_exclusive
 *
 * SUMMARY: Protected wrapper to release_exclusive_()
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*cd         - pointer to core data
 *	*ownptr     - pointer to device
 ******************************************************************************/
int release_exclusive(struct pt_core_data *cd, void *ownptr)
{
	int rc;

	mutex_lock(&cd->system_lock);
	rc = release_exclusive_(cd, ownptr);
	mutex_unlock(&cd->system_lock);

	return rc;
}

/*******************************************************************************
 * FUNCTION: pt_hid_exec_cmd_
 *
 * SUMMARY: Send the HID command to the DUT
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*cd      - pointer to core data
 *	*hid_cmd - pointer to the HID command to send
 ******************************************************************************/
static int pt_hid_exec_cmd_(struct pt_core_data *cd,
		struct pt_hid_cmd *hid_cmd)
{
	int rc;
	u8 *cmd;
	u8 cmd_length;
	u8 cmd_offset = 0;

	cmd_length = 2 /* command register */
		+ 2    /* command */
		+ (hid_cmd->report_id >= 0XF ? 1 : 0)   /* Report ID */
		+ (hid_cmd->has_data_register ? 2 : 0)	/* Data register */
		+ hid_cmd->write_length;                /* Data length */

	cmd = kzalloc(cmd_length, GFP_KERNEL);
	if (!cmd)
		return -ENOMEM;

	/* Set Command register */
	memcpy(&cmd[cmd_offset], &cd->hid_desc.command_register,
			sizeof(cd->hid_desc.command_register));
	cmd_offset += sizeof(cd->hid_desc.command_register);

	/* Set Command */
	SET_CMD_REPORT_TYPE(cmd[cmd_offset], hid_cmd->report_type);

	if (hid_cmd->report_id >= 0XF)
		SET_CMD_REPORT_ID(cmd[cmd_offset], 0xF);
	else
		SET_CMD_REPORT_ID(cmd[cmd_offset], hid_cmd->report_id);
	cmd_offset++;

	SET_CMD_OPCODE(cmd[cmd_offset], hid_cmd->opcode);
	cmd_offset++;

	if (hid_cmd->report_id >= 0XF) {
		cmd[cmd_offset] = hid_cmd->report_id;
		cmd_offset++;
	}

	/* Set Data register */
	if (hid_cmd->has_data_register) {
		memcpy(&cmd[cmd_offset], &cd->hid_desc.data_register,
				sizeof(cd->hid_desc.data_register));
		cmd_offset += sizeof(cd->hid_desc.data_register);
	}

	/* Set Data */
	if (hid_cmd->write_length && hid_cmd->write_buf) {
		memcpy(&cmd[cmd_offset], hid_cmd->write_buf,
				hid_cmd->write_length);
		cmd_offset += hid_cmd->write_length;
	}

	pt_debug(cd->dev, DL_INFO,
		">>> %s: Write Buffer Size[%d] Cmd[0x%02X]\n",
		__func__, cmd_length, hid_cmd->report_id);
	pt_pr_buf(cd->dev, DL_DEBUG, cmd, cmd_length, ">>> Cmd");
	rc = pt_adap_write_read_specific(cd, cmd_length, cmd,
			hid_cmd->read_buf);
	if (rc)
		pt_debug(cd->dev, DL_ERROR,
		"%s: Fail pt_adap_transfer\n", __func__);

	kfree(cmd);
	return rc;
}
#ifdef TTDL_DIAGNOSTICS
/*******************************************************************************
 * FUNCTION: pt_toggle_err_gpio
 *
 * SUMMARY: Toggles the pre-defined error GPIO
 *
 * RETURN: n/a
 *
 * PARAMETERS:
 *	*cd - pointer to core data
 ******************************************************************************/
void pt_toggle_err_gpio(struct pt_core_data *cd)
{
	if (cd->err_gpio) {
		pt_debug(cd->dev, DL_ERROR, "%s: Toggle ERR GPIO\n", __func__);
		gpio_direction_output(cd->err_gpio,
			!gpio_get_value(cd->err_gpio));
	}
}
#endif /* TTDL_DIAGNOSTICS */

/*******************************************************************************
 * FUNCTION: pt_hid_exec_cmd_and_wait_
 *
 * SUMMARY: Send the HID command to the DUT and wait for the response
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*cd      - pointer to core data
 *	*hid_cmd - pointer to the HID command to send
 ******************************************************************************/
static int pt_hid_exec_cmd_and_wait_(struct pt_core_data *cd,
		struct pt_hid_cmd *hid_cmd)
{
	int rc;
	int t;
	u16 timeout_ms;
	int *cmd_state;

	if (hid_cmd->reset_cmd)
		cmd_state = &cd->hid_reset_cmd_state;
	else
		cmd_state = &cd->hid_cmd_state;

	if (hid_cmd->wait_interrupt) {
		mutex_lock(&cd->system_lock);
		*cmd_state = 1;
		mutex_unlock(&cd->system_lock);
	}

	rc = pt_hid_exec_cmd_(cd, hid_cmd);
	if (rc) {
		if (hid_cmd->wait_interrupt)
			goto error;

		goto exit;
	}

	if (!hid_cmd->wait_interrupt)
		goto exit;

	if (hid_cmd->timeout_ms)
		timeout_ms = hid_cmd->timeout_ms;
	else
		timeout_ms = PT_HID_RESET_TIMEOUT;

	t = wait_event_timeout(cd->wait_q, (*cmd_state == 0),
			msecs_to_jiffies(timeout_ms));
	if (IS_TMO(t)) {
#ifdef TTDL_DIAGNOSTICS
		cd->i2c_transmission_error_count++;
		pt_toggle_err_gpio(cd);
#endif /* TTDL_DIAGNOSTICS */
		pt_debug(cd->dev, DL_ERROR,
			"%s: HID output cmd execution timed out\n",
			__func__);
		rc = -ETIME;
		goto error;
	}

	goto exit;

error:
	mutex_lock(&cd->system_lock);
	*cmd_state = 0;
	mutex_unlock(&cd->system_lock);

exit:
	return rc;
}

/*******************************************************************************
 * FUNCTION: pt_hid_cmd_reset_
 *
 * SUMMARY: Send the HID RESET command to the DUT
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*cd      - pointer to core data
 ******************************************************************************/
static int pt_hid_cmd_reset_(struct pt_core_data *cd)
{
	struct pt_hid_cmd hid_cmd = {
		.opcode = HID_CMD_RESET,
		.wait_interrupt = 1,
		.reset_cmd = 1,
		.timeout_ms = PT_HID_RESET_TIMEOUT,
	};

	return pt_hid_exec_cmd_and_wait_(cd, &hid_cmd);
}

/*******************************************************************************
 * FUNCTION: pt_hid_cmd_reset
 *
 * SUMMARY: Wrapper function for pt_hid_cmd_reset_ that guarantees exclusive
 *	access.
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*cd      - pointer to core data
 ******************************************************************************/
static int pt_hid_cmd_reset(struct pt_core_data *cd)
{
	int rc;

	rc = request_exclusive(cd, cd->dev, PT_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		pt_debug(cd->dev, DL_ERROR,
			"%s: fail get exclusive ex=%p own=%p\n",
			__func__, cd->exclusive_dev, cd->dev);
		return rc;
	}

	pt_debug(cd->dev, DL_INFO, "%s: Send HID Reset command\n", __func__);
	rc = pt_hid_cmd_reset_(cd);

	if (release_exclusive(cd, cd->dev) < 0)
		pt_debug(cd->dev, DL_ERROR,
			"%s: fail to release exclusive\n", __func__);

	return rc;
}

/*******************************************************************************
 * FUNCTION: pt_hid_cmd_set_power_
 *
 * SUMMARY: Send hid cmd to set power state for the DUT and wait for response
 *
 * RETURN:
 *   0 = success
 *  !0 = failure
 *
 * PARAMETERS:
 *  *cd           - pointer to core data
 *   power_state  - power state to set(HID_POWER_ON/HID_POWER_SLEEP)
 ******************************************************************************/
static int pt_hid_cmd_set_power_(struct pt_core_data *cd,
		u8 power_state)
{
	int rc;
	struct pt_hid_cmd hid_cmd = {
		.opcode = HID_CMD_SET_POWER,
		.wait_interrupt = 1,
		.timeout_ms = PT_HID_SET_POWER_TIMEOUT,
	};
	hid_cmd.power_state = power_state;

	rc =  pt_hid_exec_cmd_and_wait_(cd, &hid_cmd);
	if (rc) {
		pt_debug(cd->dev, DL_ERROR,
			"%s: Failed to set power to state:%d\n",
			__func__, power_state);
	       return rc;
	}

	/* validate */
	if ((cd->response_buf[2] != HID_RESPONSE_REPORT_ID)
			|| ((cd->response_buf[3] & 0x3) != power_state)
			|| ((cd->response_buf[4] & 0xF) != HID_CMD_SET_POWER))
		rc = -EINVAL;

	return rc;
}

/*******************************************************************************
 * FUNCTION: pt_hid_cmd_set_power
 *
 * SUMMARY: Wrapper function for pt_hid_cmd_set_power_ that guarantees
 *  exclusive access.
 *
 * RETURN:
 *   0 = success
 *  !0 = failure
 *
 * PARAMETERS:
 *  *cd           - pointer to core data
 *   power_state  - power state to set(HID_POWER_ON/HID_POWER_SLEEP)
 ******************************************************************************/
static int pt_hid_cmd_set_power(struct pt_core_data *cd,
		u8 power_state)
{
	int rc;

	rc = request_exclusive(cd, cd->dev, PT_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		pt_debug(cd->dev, DL_ERROR,
			"%s: fail get exclusive ex=%p own=%p\n",
			__func__, cd->exclusive_dev, cd->dev);
		return rc;
	}

	rc = pt_hid_cmd_set_power_(cd, power_state);

	if (release_exclusive(cd, cd->dev) < 0)
		pt_debug(cd->dev, DL_ERROR,
			"%s: fail to release exclusive\n", __func__);

	return rc;
}

static const u16 crc_table[16] = {
	0x0000, 0x1021, 0x2042, 0x3063,
	0x4084, 0x50a5, 0x60c6, 0x70e7,
	0x8108, 0x9129, 0xa14a, 0xb16b,
	0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
};

/*******************************************************************************
 * FUNCTION: _pt_compute_crc
 *
 * SUMMARY: Calculate CRC by CRC table.
 *
 * RETURN:
 *   CRC calculation result
 *
 * PARAMETERS:
 *  *buf   - pointer to the data array to be calculated
 *   size  - size of data array
 ******************************************************************************/
static u16 _pt_compute_crc(u8 *buf, u32 size)
{
	u16 remainder = 0xFFFF;
	u16 xor_mask = 0x0000;
	u32 index;
	u32 byte_value;
	u32 table_index;
	u32 crc_bit_width = sizeof(u16) * 8;

	/* Divide the message by polynomial, via the table. */
	for (index = 0; index < size; index++) {
		byte_value = buf[index];
		table_index = ((byte_value >> 4) & 0x0F)
			^ (remainder >> (crc_bit_width - 4));
		remainder = crc_table[table_index] ^ (remainder << 4);
		table_index = (byte_value & 0x0F)
			^ (remainder >> (crc_bit_width - 4));
		remainder = crc_table[table_index] ^ (remainder << 4);
	}

	/* Perform the final remainder CRC. */
	return remainder ^ xor_mask;
}

u16 ccitt_Table[] = {
	0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
	0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
	0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
	0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
	0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
	0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
	0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
	0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
	0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
	0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
	0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
	0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
	0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
	0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
	0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
	0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
	0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
	0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
	0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
	0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
	0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
	0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
	0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
	0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
	0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
	0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
	0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
	0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
	0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
	0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
	0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
	0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0,
};

/*******************************************************************************
 * FUNCTION: crc_ccitt_calculate
 *
 * SUMMARY: Calculate CRC with ccitt standard by CRC table.
 *
 * RETURN:
 *   CRC calculation result
 *
 * PARAMETERS:
 *  *q    - pointer to the data array to be calculated
 *   len  - size of data array
 ******************************************************************************/
static unsigned short crc_ccitt_calculate(unsigned char *q, int len)
{
	unsigned short crc = 0xffff;

	while (len-- > 0)
		crc = ccitt_Table[(crc >> 8 ^ *q++) & 0xff] ^ (crc << 8);
	return crc;
}

/*******************************************************************************
 * FUNCTION: pt_pip2_cmd_calculate_crc
 *
 * SUMMARY: Calculate the CRC of a command packet
 *
 * RETURN: void
 *
 * PARAMETERS:
 *	*cmd         - pointer to command data
 *	 extra_bytes - Extra bytes included in command length
 ******************************************************************************/
static void pt_pip2_cmd_calculate_crc(struct pip2_cmd_structure *cmd,
	u8 extra_bytes)
{
	u8 buf[256];
	unsigned short crc;

	memset(buf, 0, sizeof(buf));
	buf[0] = cmd->len & 0xff;
	buf[1] = (cmd->len & 0xff00) >> 8;
	buf[2] = cmd->seq;
	buf[3] = cmd->id;
	memcpy(&buf[4], cmd->data, cmd->len - extra_bytes);
	/* Calculate the CRC for the first 4 bytes above and the data payload */
	crc = crc_ccitt_calculate(buf, 4 + (cmd->len - extra_bytes));
	cmd->crc[0] = (crc & 0xff00) >> 8;
	cmd->crc[1] = (crc & 0xff);
}

/*******************************************************************************
 * FUNCTION: pt_pip2_get_next_cmd_seq
 *
 * SUMMARY: Gets the next sequence number for a PIP2 command. The sequence
 *	number is a 3 bit value (bits [0-2]) but because TTDL will always have
 *	the TAG bit set (bit 3), the counter starts at 0x08 and goes to 0x0F
 *
 * RETURN: Next command sequence number [0x08-0x0F]
 *
 * PARAMETERS:
 *	*cd - pointer to core data
 ******************************************************************************/
static u8 pt_pip2_get_next_cmd_seq(struct pt_core_data *cd)
{
	cd->pip2_cmd_tag_seq++;
	if (cd->pip2_cmd_tag_seq > 0x0F)
		cd->pip2_cmd_tag_seq = 0x08;
	return cd->pip2_cmd_tag_seq;
}


/*
 * response_len = -1 means the response length is variable
 */
static struct pip2_cmd_response_structure pip2_cmd_response[] = {
	{.id = PIP2_CMD_ID_PING,
		.response_len = -1},
	{.id = PIP2_CMD_ID_STATUS,
		.response_len = PIP2_EXTRA_BYTES_NUM + 4},
	{.id = PIP2_CMD_ID_CTRL,
		.response_len = PIP2_EXTRA_BYTES_NUM + 1},
	{.id = PIP2_CMD_ID_CONFIG,
		.response_len = PIP2_EXTRA_BYTES_NUM + 1},
	{.id = PIP2_CMD_ID_CLEAR,
		.response_len = PIP2_EXTRA_BYTES_NUM + 0},
	{.id = PIP2_CMD_ID_RESET,
		.response_len = PIP2_EXTRA_BYTES_NUM + 0},
	{.id = PIP2_CMD_ID_VERSION,
		.response_len = PIP2_EXTRA_BYTES_NUM + 22},
	{.id = PIP2_CMD_ID_FILE_OPEN,
		.response_len = PIP2_EXTRA_BYTES_NUM + 2},
	{.id = PIP2_CMD_ID_FILE_CLOSE,
		.response_len = PIP2_EXTRA_BYTES_NUM + 1},
	{.id = PIP2_CMD_ID_FILE_READ,
		.response_len = -1},
	{.id = PIP2_CMD_ID_FILE_WRITE,
		.response_len = PIP2_EXTRA_BYTES_NUM + 1},
	{.id = PIP2_CMD_ID_FILE_IOCTL,
		.response_len = -1},
	{.id = PIP2_CMD_ID_FLASH_INFO,
		.response_len = PIP2_EXTRA_BYTES_NUM + 15},
	{.id = PIP2_CMD_ID_EXECUTE,
		.response_len = PIP2_EXTRA_BYTES_NUM + 1},
	{.id = PIP2_CMD_ID_GET_LAST_ERRNO,
		.response_len = PIP2_EXTRA_BYTES_NUM + 3},
};

/*******************************************************************************
 * FUNCTION: pt_pip2_get_cmd_response_len
 *
 * SUMMARY: Gets the expected response length based on the command ID
 *
 * RETURN: Expected response length
 *
 * PARAMETERS:
 *	id - Command ID
 * TODO: Ensure while loop breaks if ID not found and return default len
 ******************************************************************************/
static int pt_pip2_get_cmd_response_len(u8 id)
{
	struct pip2_cmd_response_structure *p = pip2_cmd_response;

	while (p->id != id)
		p++;
	return p->response_len;
}

/*******************************************************************************
 * FUNCTION: pt_hid_output_validate_bl_response
 *
 * SUMMARY: Validate the response of bootloader command.
 *
 * RETURN:
 *   0 = success
 *  !0 = failure
 *
 * PARAMETERS:
 *  *cd          - pointer to core data
 *  *hid_output  - pointer to hid output data structure
 ******************************************************************************/
static int pt_hid_output_validate_bl_response(
		struct pt_core_data *cd,
		struct pt_hid_output *hid_output)
{
	u16 size;
	u16 crc;
	u8 status;

	size = get_unaligned_le16(&cd->response_buf[0]);

	if (hid_output->reset_expected && !size)
		return 0;

	if (cd->response_buf[HID_OUTPUT_RESPONSE_REPORT_OFFSET]
			!= HID_BL_RESPONSE_REPORT_ID) {
		pt_debug(cd->dev, DL_ERROR,
			"%s: BL output response, wrong report_id\n", __func__);
		return -EPROTO;
	}

	if (cd->response_buf[4] != HID_OUTPUT_BL_SOP) {
		pt_debug(cd->dev, DL_ERROR,
			"%s: BL output response, wrong SOP\n", __func__);
		return -EPROTO;
	}

	if (cd->response_buf[size - 1] != HID_OUTPUT_BL_EOP) {
		pt_debug(cd->dev, DL_ERROR,
			"%s: BL output response, wrong EOP\n", __func__);
		return -EPROTO;
	}

	crc = _pt_compute_crc(&cd->response_buf[4], size - 7);
	if (cd->response_buf[size - 3] != LOW_BYTE(crc)
			|| cd->response_buf[size - 2] != HI_BYTE(crc)) {
		pt_debug(cd->dev, DL_ERROR,
			"%s: BL output response, wrong CRC 0x%X\n",
			__func__, crc);
		return -EPROTO;
	}

	status = cd->response_buf[5];
	if (status) {
		pt_debug(cd->dev, DL_ERROR,
			"%s: BL output response, ERROR:%d\n",
			__func__, status);
		return -EPROTO;
	}

	return 0;
}

/*******************************************************************************
 * FUNCTION: pt_hid_output_validate_app_response
 *
 * SUMMARY: Validate the response of application command.
 *
 * RETURN:
 *   0 = success
 *  !0 = failure
 *
 * PARAMETERS:
 *  *cd          - pointer to core data
 *  *hid_output  - pointer to hid output data structure
 ******************************************************************************/
static int pt_hid_output_validate_app_response(
		struct pt_core_data *cd,
		struct pt_hid_output *hid_output)
{
	int command_code;
	u16 size;

	size = get_unaligned_le16(&cd->response_buf[0]);

	if (hid_output->reset_expected && !size)
		return 0;

	if (cd->response_buf[HID_OUTPUT_RESPONSE_REPORT_OFFSET]
			!= HID_APP_RESPONSE_REPORT_ID) {
		pt_debug(cd->dev, DL_ERROR,
			"%s: APP output response, wrong report_id\n", __func__);
		return -EPROTO;
	}

	command_code = cd->response_buf[HID_OUTPUT_RESPONSE_CMD_OFFSET]
		& HID_OUTPUT_RESPONSE_CMD_MASK;
	if (command_code != hid_output->command_code) {
		pt_debug(cd->dev, DL_ERROR,
			"%s: APP output response, wrong command_code:%X\n",
			__func__, command_code);
		return -EPROTO;
	}

	return 0;
}

/*******************************************************************************
 * FUNCTION: pt_check_set_parameter
 *
 * SUMMARY: Check command input and response for Set Parameter command.And
 *  store the parameter to the list for resume work if pass the check.
 *
 * PARAMETERS:
 *  *cd          - pointer to core data
 *  *hid_output  - pointer to hid output data structure
 *   raw         - flag to show if output cmd is user cmd(1:user cmd)
 ******************************************************************************/
static void pt_check_set_parameter(struct pt_core_data *cd,
		struct pt_hid_output *hid_output, bool raw)
{
	u8 *param_buf;
	u32 param_value = 0;
	u8 param_size;
	u8 param_id;
	int i = 0;

	if (!(cd->cpdata->flags & PT_CORE_FLAG_RESTORE_PARAMETERS))
		return;

	/* Check command input for Set Parameter command */
	if (raw && hid_output->length >= 10 && hid_output->length <= 13
			&& !memcmp(&hid_output->write_buf[0],
					&cd->hid_desc.output_register,
					sizeof(cd->hid_desc.output_register))
			&& hid_output->write_buf[4] ==
					HID_APP_OUTPUT_REPORT_ID
			&& hid_output->write_buf[6] ==
					HID_OUTPUT_SET_PARAM)
		param_buf = &hid_output->write_buf[7];
	else if (!raw && hid_output->cmd_type == HID_OUTPUT_CMD_APP
			&& hid_output->command_code == HID_OUTPUT_SET_PARAM
			&& hid_output->write_length >= 3
			&& hid_output->write_length <= 6)
		param_buf = &hid_output->write_buf[0];
	else
		return;

	/* Get parameter ID, size and value */
	param_id = param_buf[0];
	param_size = param_buf[1];
	if (param_size > 4) {
		pt_debug(cd->dev, DL_ERROR,
			"%s: Invalid parameter size\n", __func__);
		return;
	}

	param_buf = &param_buf[2];
	while (i < param_size)
		param_value += *(param_buf++) << (8 * i++);

	/* Check command response for Set Parameter command */
	if (cd->response_buf[2] != HID_APP_RESPONSE_REPORT_ID
			|| (cd->response_buf[4] & HID_OUTPUT_CMD_MASK) !=
				HID_OUTPUT_SET_PARAM
			|| cd->response_buf[5] != param_id
			|| cd->response_buf[6] != param_size) {
		pt_debug(cd->dev, DL_ERROR,
			"%s: Set Parameter command not successful\n",
			__func__);
		return;
	}

	pt_add_parameter(cd, param_id, param_value, param_size);
}

/*******************************************************************************
 * FUNCTION: pt_check_command
 *
 * SUMMARY: Check the ouput command. The function pt_check_set_parameter() is
 *  called here to check output command and store parameter to the list.
 *
 * PARAMETERS:
 *  *cd          - pointer to core data
 *  *hid_output  - pointer to hid output data structure
 *   raw         - flag to show if output cmd is user cmd(1:user cmd)
 ******************************************************************************/
static void pt_check_command(struct pt_core_data *cd,
		struct pt_hid_output *hid_output, bool raw)
{
	pt_check_set_parameter(cd, hid_output, raw);
}

/*******************************************************************************
 * FUNCTION: pt_hid_output_validate_response
 *
 * SUMMARY: Validate the response of application or bootloader command.
 *
 * RETURN:
 *   0 = success
 *  !0 = failure
 *
 * PARAMETERS:
 *  *cd          - pointer to core data
 *  *hid_output  - pointer to hid output data structure
 ******************************************************************************/
static int pt_hid_output_validate_response(struct pt_core_data *cd,
		struct pt_hid_output *hid_output)
{
	if (hid_output->cmd_type == HID_OUTPUT_CMD_BL)
		return pt_hid_output_validate_bl_response(cd, hid_output);

	return pt_hid_output_validate_app_response(cd, hid_output);

}

/*******************************************************************************
 * FUNCTION: pt_hid_send_output_user_
 *
 * SUMMARY: Blindly send user data to the DUT.
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*cd         - pointer to core data
 *	*hid_output - pointer to the command to send
 ******************************************************************************/
static int pt_hid_send_output_user_(struct pt_core_data *cd,
		struct pt_hid_output *hid_output)
{
	int rc;
	int cmd;

	if (!hid_output->length || !hid_output->write_buf)
		return -EINVAL;

	if (cd->pip2_prot_active) {
		cmd = hid_output->write_buf[PIP2_HID_OUTPUT_RESP_CMD_OFFSET];
		cmd &= HID_OUTPUT_CMD_MASK;
	} else
		cmd = hid_output->write_buf[HID_OUTPUT_CMD_OFFSET];

	pt_debug(cd->dev, DL_INFO,
		">>> %s: Write Buffer Size[%d] Cmd[0x%02X]\n",
		__func__, hid_output->length, cmd);
	pt_pr_buf(cd->dev, DL_DEBUG, hid_output->write_buf,
		hid_output->length, ">>> User Cmd");
	rc = pt_adap_write_read_specific(cd, hid_output->length,
			hid_output->write_buf, NULL);
	if (rc)
		pt_debug(cd->dev, DL_ERROR,
			"%s: Fail pt_adap_transfer\n", __func__);

	return rc;
}

/*******************************************************************************
 * FUNCTION: pt_hid_send_output_user_and_wait_
 *
 * SUMMARY: Blindly send user data to the DUT and wait for the response. Count
 *	transmission errors if built with TTDL_DIAGNOSTICS enabled.
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*cd         - pointer to core data
 *	*hid_output - pointer to the command to send
 ******************************************************************************/
static int pt_hid_send_output_user_and_wait_(struct pt_core_data *cd,
		struct pt_hid_output *hid_output)
{
	int rc;
	int t;

	mutex_lock(&cd->system_lock);
	cd->hid_cmd_state = HID_OUTPUT_USER_CMD + 1;
	mutex_unlock(&cd->system_lock);

	rc = pt_hid_send_output_user_(cd, hid_output);
	if (rc)
		goto error;
	/*
	 * PATCH - If it is PIP2 STATUS or VERSION command, set timeout to 20ms.
	 * These PIP2 commands used to test device PIP version and BL/APP mode.
	 */
	if ((hid_output->write_buf[0] == 0x01) &&
	    (hid_output->write_buf[1] == 0x01) &&
	    ((hid_output->write_buf[5] == 0x01) ||
	     (hid_output->write_buf[5] == 0x07))) {
		t = wait_event_timeout(cd->wait_q, (cd->hid_cmd_state == 0),
			msecs_to_jiffies(200));
	} else
		t = wait_event_timeout(cd->wait_q, (cd->hid_cmd_state == 0),
			msecs_to_jiffies(PT_HID_OUTPUT_USER_TIMEOUT));

	if (IS_TMO(t)) {
#ifdef TTDL_DIAGNOSTICS
		cd->i2c_transmission_error_count++;
		pt_toggle_err_gpio(cd);
#endif /* TTDL_DIAGNOSTICS */
		pt_debug(cd->dev, DL_ERROR,
			"%s: HID output cmd execution timed out\n",
			__func__);
		rc = -ETIME;
		goto error;
	}

	pt_check_command(cd, hid_output, true);

	goto exit;

error:
	mutex_lock(&cd->system_lock);
	cd->hid_cmd_state = 0;
	mutex_unlock(&cd->system_lock);

exit:
	return rc;
}

/*******************************************************************************
 * FUNCTION: pt_flush_i2c
 *
 * SUMMARY: Force flushing the I2C bus by reading len bytes or forced 255 bytes
 *	Used if IRQ is found to be stuck low
 *
 * RETURN: Length of bytes read from bus
 *
 * PARAMETERS:
 *      *cd   - pointer to core data
 *      force - <0> Read len first and flush len bytes
 *              <1> Force a read of 255 bytes
 ******************************************************************************/
static ssize_t pt_flush_i2c(struct pt_core_data *cd, u8 force)
{
	u8 buf[256];
	u8 len;
	u8 read_len;

	if (force == 0) {
		i2c_master_recv(to_i2c_client(cd->dev), buf, 2);
		len = 2 + get_unaligned_le16(&buf[0]);
		pt_debug(cd->dev, DL_INFO,
			"%s: Forced read of %d bytes...\n", __func__, len);
		while (len > 0) {
			if (len > 255)
				read_len = 255;
			else
				read_len = len;
			i2c_master_recv(to_i2c_client(cd->dev), buf, read_len);
			len -= read_len;
		}
	} else {
		pt_debug(cd->dev, DL_INFO,
			"%s: Forced read of max 255 bytes...\n", __func__);
		i2c_master_recv(to_i2c_client(cd->dev), buf, 255);
		len =  255;
	}
	return len;
}

/*******************************************************************************
 * FUNCTION: pt_hid_send_output_
 *
 * SUMMARY: Send valid data to the DUT
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*cd         - pointer to core data
 *	*hid_output - pointer to the command to send
 ******************************************************************************/
static int pt_hid_send_output_(struct pt_core_data *cd,
		struct pt_hid_output *hid_output)
{
	int rc;
	u8 *cmd;
	u16 length;
	u8 report_id;
	u8 cmd_offset = 0;
	u16 crc;
	u8 cmd_allocated = 0;

#ifdef FUTURE
	/*
	 * *** TODO - Determine side effects of adding this safty net ***
	 * If IRQ is already asserted due to a pending report, it must be
	 * cleared before sending command.
	 */
	if (cd->cpdata->irq_stat &&
	    cd->cpdata->irq_stat(cd->cpdata, cd->dev) ==
	    PT_IRQ_ASSERTED_VALUE) {
		pt_flush_i2c(cd, 0);
		pt_debug(cd->dev, DL_INFO,
			"%s: IRQ asserted before write, forced clear\n",
			__func__);
	}
#endif

	switch (hid_output->cmd_type) {
	case HID_OUTPUT_CMD_APP:
		report_id = HID_APP_OUTPUT_REPORT_ID;
		length = 5;
		break;
	case HID_OUTPUT_CMD_BL:
		report_id = HID_BL_OUTPUT_REPORT_ID;
		length = 11 /* 5 + SOP + LEN(2) + CRC(2) + EOP */;
		break;
	default:
		return -EINVAL;
	}

	length += hid_output->write_length;

	if (length + 2 > PT_PREALLOCATED_CMD_BUFFER) {
		cmd = kzalloc(length + 2, GFP_KERNEL);
		if (!cmd)
			return -ENOMEM;
		cmd_allocated = 1;
	} else {
		cmd = cd->cmd_buf;
	}

	/* Set Output register */
	memcpy(&cmd[cmd_offset], &cd->hid_desc.output_register,
			sizeof(cd->hid_desc.output_register));
	cmd_offset += sizeof(cd->hid_desc.output_register);

	cmd[cmd_offset++] = LOW_BYTE(length);
	cmd[cmd_offset++] = HI_BYTE(length);
	cmd[cmd_offset++] = report_id;
	cmd[cmd_offset++] = 0x0; /* reserved */
	if (hid_output->cmd_type == HID_OUTPUT_CMD_BL)
		cmd[cmd_offset++] = HID_OUTPUT_BL_SOP;
	cmd[cmd_offset++] = hid_output->command_code;

	/* Set Data Length for bootloader */
	if (hid_output->cmd_type == HID_OUTPUT_CMD_BL) {
		cmd[cmd_offset++] = LOW_BYTE(hid_output->write_length);
		cmd[cmd_offset++] = HI_BYTE(hid_output->write_length);
	}
	/* Set Data */
	if (hid_output->write_length && hid_output->write_buf) {
		memcpy(&cmd[cmd_offset], hid_output->write_buf,
				hid_output->write_length);
		cmd_offset += hid_output->write_length;
	}
	if (hid_output->cmd_type == HID_OUTPUT_CMD_BL) {
		crc = _pt_compute_crc(&cmd[6],
				hid_output->write_length + 4);
		cmd[cmd_offset++] = LOW_BYTE(crc);
		cmd[cmd_offset++] = HI_BYTE(crc);
		cmd[cmd_offset++] = HID_OUTPUT_BL_EOP;
	}

	pt_debug(cd->dev, DL_INFO,
		">>> %s: Write Buffer Size[%d] Cmd[0x%02X]\n",
		__func__, length + 2, hid_output->command_code);
	pt_pr_buf(cd->dev, DL_DEBUG, cmd, length + 2, ">>> Cmd");
	rc = pt_adap_write_read_specific(cd, length + 2, cmd, NULL);
	if (rc)
		pt_debug(cd->dev, DL_ERROR,
			"%s: Fail pt_adap_transfer\n", __func__);

	if (cmd_allocated)
		kfree(cmd);
	return rc;
}

/*******************************************************************************
 * FUNCTION: pt_hid_send_output_and_wait_
 *
 * SUMMARY: Send valid data to the DUT and wait for the response. Count
 *	transmission errors if built with TTDL_DIAGNOSTICS enabled.
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*cd         - pointer to core data
 *	*hid_output - pointer to the command to send
 ******************************************************************************/
static int pt_hid_send_output_and_wait_(struct pt_core_data *cd,
		struct pt_hid_output *hid_output)
{
	int rc;
	int t;
	u16 timeout_ms;

	mutex_lock(&cd->system_lock);
	cd->hid_cmd_state = hid_output->command_code + 1;
	mutex_unlock(&cd->system_lock);

	if (hid_output->timeout_ms)
		timeout_ms = hid_output->timeout_ms;
	else
		timeout_ms = PT_HID_OUTPUT_TIMEOUT;

	rc = pt_hid_send_output_(cd, hid_output);
	if (rc)
		goto error;


	t = wait_event_timeout(cd->wait_q, (cd->hid_cmd_state == 0),
			msecs_to_jiffies(timeout_ms));
	if (IS_TMO(t)) {
#ifdef TTDL_DIAGNOSTICS
		cd->i2c_transmission_error_count++;
		pt_toggle_err_gpio(cd);
#endif /* TTDL_DIAGNOSTICS */
		pt_debug(cd->dev, DL_ERROR,
			"%s: HID output cmd execution timed out\n",
			__func__);
		rc = -ETIME;
		goto error;
	}

	if (!hid_output->novalidate)
		rc = pt_hid_output_validate_response(cd, hid_output);

	pt_check_command(cd, hid_output, false);
	goto exit;

error:
	mutex_lock(&cd->system_lock);
	cd->hid_cmd_state = 0;
	mutex_unlock(&cd->system_lock);
exit:
	return rc;
}

/*******************************************************************************
 * FUNCTION: pt_pip_null_
 *
 * SUMMARY: Send the PIP "ping"(0x00) command to the DUT and wait for response.
 *  This function is used by watchdog to check if the fw corrupts.
 *
 * RETURN:
 *   0 = success
 *  !0 = failure
 *
 * PARAMETERS:
 *  *cd  - pointer to core data
 ******************************************************************************/
static int pt_pip_null_(struct pt_core_data *cd)
{
	struct pt_hid_output hid_output = {
		HID_OUTPUT_APP_COMMAND(HID_OUTPUT_NULL),
	};

	return pt_hid_send_output_and_wait_(cd, &hid_output);
}

/*******************************************************************************
 * FUNCTION: pt_hid_cmd_set_power
 *
 * SUMMARY: Wrapper function for pt_pip_null_ that guarantees exclusive access.
 *
 * RETURN:
 *   0 = success
 *  !0 = failure
 *
 * PARAMETERS:
 *  *cd  - pointer to core data
 ******************************************************************************/
static int pt_pip_null(struct pt_core_data *cd)
{
	int rc;

	rc = request_exclusive(cd, cd->dev, PT_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		pt_debug(cd->dev, DL_ERROR,
				"%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->dev);
		return rc;
	}

	rc = pt_pip_null_(cd);

	if (release_exclusive(cd, cd->dev) < 0)
		pt_debug(cd->dev, DL_ERROR,
				"%s: fail to release exclusive\n", __func__);

	return rc;
}

static void pt_stop_wd_timer(struct pt_core_data *cd);

/*******************************************************************************
 * FUNCTION: pt_pip_start_bootloader_
 *
 * SUMMARY: Sends the HID command start_bootloader [PIP cmd 0x01] to the DUT
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*cd - pointer to core data
 ******************************************************************************/
static int pt_pip_start_bootloader_(struct pt_core_data *cd)
{
	int rc;

	struct pt_hid_output hid_output = {
		HID_OUTPUT_APP_COMMAND(HID_OUTPUT_START_BOOTLOADER),
		.timeout_ms = PT_HID_OUTPUT_START_BOOTLOADER_TIMEOUT,
		.reset_expected = 1,
	};

	/* Entering the BL the WD must be stopped as the BL will not respond */
	pt_stop_wd_timer(cd);

	/* Reset startup status after entering BL, new DUT enum required */
	cd->startup_status = STARTUP_STATUS_START;
	rc = pt_hid_send_output_and_wait_(cd, &hid_output);
	if (rc) {
		pt_debug(cd->dev, DL_ERROR,
			"%s: Start BL PIP cmd failed. rc = %d\n",
			__func__, rc);
	}
	return rc;
}

/*******************************************************************************
 * FUNCTION: pt_pip_start_bootloader
 *
 * SUMMARY: Protected function to force DUT to enter the BL
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *      *cd - pointer to core data structure
 ******************************************************************************/
static int pt_pip_start_bootloader(struct pt_core_data *cd)
{
	int rc;

	rc = request_exclusive(cd, cd->dev, PT_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		pt_debug(cd->dev, DL_ERROR,
			"%s: fail get exclusive ex=%p own=%p\n",
			__func__, cd->exclusive_dev, cd->dev);
		return rc;
	}

	rc = pt_pip_start_bootloader_(cd);

	if (release_exclusive(cd, cd->dev) < 0)
		pt_debug(cd->dev, DL_ERROR,
				"%s: fail to release exclusive\n", __func__);

	return rc;
}

/*******************************************************************************
 * FUNCTION: _pt_request_pip_start_bl
 *
 * SUMMARY: Function pointer included in core_nonhid_cmds to allow other
 *	modules to request the DUT to enter the BL
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *      *dev     - pointer to device structure
 *	 protect - flag to run in protected mode
 ******************************************************************************/
static int _pt_request_pip_start_bl(struct device *dev, int protect)
{
	struct pt_core_data *cd = dev_get_drvdata(dev);

	if (protect)
		return pt_pip_start_bootloader(cd);

	return pt_pip_start_bootloader_(cd);
}

/*******************************************************************************
 * FUNCTION: pt_si_get_ttdata
 *
 * SUMMARY: Function to load the version information from the system information
 *	PIP command into the core data struct.
 *
 * RETURN: n/a
 *
 * PARAMETERS:
 *      *cd - pointer to core data structure
 ******************************************************************************/
static void pt_si_get_ttdata(struct pt_core_data *cd)
{
	struct pt_ttdata *ttdata = &cd->sysinfo.ttdata;
	struct pt_ttdata_dev *ttdata_dev =
		(struct pt_ttdata_dev *)
		&cd->response_buf[HID_SYSINFO_TTDATA_OFFSET];

	ttdata->pip_ver_major = ttdata_dev->pip_ver_major;
	ttdata->pip_ver_minor = ttdata_dev->pip_ver_minor;
	ttdata->bl_ver_major = ttdata_dev->bl_ver_major;
	ttdata->bl_ver_minor = ttdata_dev->bl_ver_minor;
	ttdata->fw_ver_major = ttdata_dev->fw_ver_major;
	ttdata->fw_ver_minor = ttdata_dev->fw_ver_minor;

	ttdata->fw_pid = get_unaligned_le16(&ttdata_dev->fw_pid);
	ttdata->fw_ver_conf = get_unaligned_le16(&ttdata_dev->fw_ver_conf);
	ttdata->post_code = get_unaligned_le16(&ttdata_dev->post_code);
	ttdata->revctrl = get_unaligned_le32(&ttdata_dev->revctrl);
	ttdata->jtag_id_l = get_unaligned_le16(&ttdata_dev->jtag_si_id_l);
	ttdata->jtag_id_h = get_unaligned_le16(&ttdata_dev->jtag_si_id_h);

	memcpy(ttdata->mfg_id, ttdata_dev->mfg_id, PT_NUM_MFGID);

	pt_pr_buf(cd->dev, DL_INFO, (u8 *)ttdata_dev,
		sizeof(struct pt_ttdata_dev), "sysinfo_ttdata");
}

/*******************************************************************************
 * FUNCTION: pt_si_get_sensing_conf_data
 *
 * SUMMARY: Function to load the sensing information from the system information
 *	PIP command into the core data struct.
 *
 * RETURN: n/a
 *
 * PARAMETERS:
 *      *cd - pointer to core data structure
 ******************************************************************************/
static void pt_si_get_sensing_conf_data(struct pt_core_data *cd)
{
	struct pt_sensing_conf_data *scd = &cd->sysinfo.sensing_conf_data;
	struct pt_sensing_conf_data_dev *scd_dev =
		(struct pt_sensing_conf_data_dev *)
		&cd->response_buf[HID_SYSINFO_SENSING_OFFSET];

	scd->electrodes_x = scd_dev->electrodes_x;
	scd->electrodes_y = scd_dev->electrodes_y;
	scd->origin_x = scd_dev->origin_x;
	scd->origin_y = scd_dev->origin_y;

	/* PIP 1.4 (001-82649 *Q) add X_IS_TX bit in X_ORG */
	if (scd->origin_x & 0x02) {
		scd->tx_num = scd->electrodes_x;
		scd->rx_num = scd->electrodes_y;
	} else {
		scd->tx_num = scd->electrodes_y;
		scd->rx_num = scd->electrodes_x;
	}

	scd->panel_id = scd_dev->panel_id;
	scd->btn = scd_dev->btn;
	scd->scan_mode = scd_dev->scan_mode;
	scd->max_tch = scd_dev->max_num_of_tch_per_refresh_cycle;

	scd->res_x = get_unaligned_le16(&scd_dev->res_x);
	scd->res_y = get_unaligned_le16(&scd_dev->res_y);
	scd->max_z = get_unaligned_le16(&scd_dev->max_z);
	scd->len_x = get_unaligned_le16(&scd_dev->len_x);
	scd->len_y = get_unaligned_le16(&scd_dev->len_y);

	pt_pr_buf(cd->dev, DL_INFO, (u8 *)scd_dev,
		sizeof(struct pt_sensing_conf_data_dev),
		"sensing_conf_data");
}

/*******************************************************************************
 * FUNCTION: pt_si_setup
 *
 * SUMMARY: Setup the xy_data and xy_mode by allocating the needed memory
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *      *cd - pointer to core data structure
 ******************************************************************************/
static int pt_si_setup(struct pt_core_data *cd)
{
	struct pt_sysinfo *si = &cd->sysinfo;
	int max_tch = si->sensing_conf_data.max_tch;

	if (!si->xy_data)
		si->xy_data = kzalloc(max_tch * si->desc.tch_record_size,
				GFP_KERNEL);
	if (!si->xy_data)
		return -ENOMEM;

	if (!si->xy_mode)
		si->xy_mode = kzalloc(si->desc.tch_header_size, GFP_KERNEL);
	if (!si->xy_mode) {
		kfree(si->xy_data);
		return -ENOMEM;
	}

	return 0;
}

/*******************************************************************************
 * FUNCTION: pt_si_get_btn_data
 *
 * SUMMARY: Setup the core data button information based on the response of the
 *	System Information PIP command.
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *      *cd - pointer to core data structure
 ******************************************************************************/
static int pt_si_get_btn_data(struct pt_core_data *cd)
{
	struct pt_sysinfo *si = &cd->sysinfo;
	int num_btns = 0;
	int num_defined_keys;
	u16 *key_table;
	int btn;
	int i;
	int rc = 0;
	unsigned int btns = cd->response_buf[HID_SYSINFO_BTN_OFFSET]
		& HID_SYSINFO_BTN_MASK;
	size_t btn_keys_size;

	pt_debug(cd->dev, DL_INFO, "%s: get btn data\n", __func__);

	for (i = 0; i < HID_SYSINFO_MAX_BTN; i++) {
		if (btns & (1 << i))
			num_btns++;
	}
	si->num_btns = num_btns;

	if (num_btns) {
		btn_keys_size = num_btns * sizeof(struct pt_btn);
		if (!si->btn)
			si->btn = kzalloc(btn_keys_size, GFP_KERNEL);
		if (!si->btn)
			return -ENOMEM;

		if (cd->cpdata->sett[PT_IC_GRPNUM_BTN_KEYS] == NULL)
			num_defined_keys = 0;
		else if (cd->cpdata->sett[PT_IC_GRPNUM_BTN_KEYS]->data == NULL)
			num_defined_keys = 0;
		else
			num_defined_keys = cd->cpdata->sett
				[PT_IC_GRPNUM_BTN_KEYS]->size;

		for (btn = 0; btn < num_btns && btn < num_defined_keys; btn++) {
			key_table = (u16 *)cd->cpdata->sett
				[PT_IC_GRPNUM_BTN_KEYS]->data;
			si->btn[btn].key_code = key_table[btn];
			si->btn[btn].enabled = true;
		}
		for (; btn < num_btns; btn++) {
			si->btn[btn].key_code = KEY_RESERVED;
			si->btn[btn].enabled = true;
		}

		return rc;
	}

	kfree(si->btn);
	si->btn = NULL;
	return rc;
}

/*******************************************************************************
 * FUNCTION: pt_si_put_log_data
 *
 * SUMMARY: Prints all sys info data to kmsg log
 *
 * RETURN: n/a
 *
 * PARAMETERS:
 *	*cd - pointer to core data structure
 ******************************************************************************/
static void pt_si_put_log_data(struct pt_core_data *cd)
{
	struct pt_sysinfo *si = &cd->sysinfo;
	struct pt_ttdata *ttdata = &si->ttdata;
	struct pt_sensing_conf_data *scd = &si->sensing_conf_data;
	int i;

	pt_debug(cd->dev, DL_DEBUG, "%s: pip_ver_major =0x%02X (%d)\n",
		__func__, ttdata->pip_ver_major, ttdata->pip_ver_major);
	pt_debug(cd->dev, DL_DEBUG, "%s: pip_ver_minor =0x%02X (%d)\n",
		__func__, ttdata->pip_ver_minor, ttdata->pip_ver_minor);
	pt_debug(cd->dev, DL_DEBUG, "%s: fw_pid =0x%04X (%d)\n",
		__func__, ttdata->fw_pid, ttdata->fw_pid);
	pt_debug(cd->dev, DL_DEBUG, "%s: fw_ver_major =0x%02X (%d)\n",
		__func__, ttdata->fw_ver_major, ttdata->fw_ver_major);
	pt_debug(cd->dev, DL_DEBUG, "%s: fw_ver_minor =0x%02X (%d)\n",
		__func__, ttdata->fw_ver_minor, ttdata->fw_ver_minor);
	pt_debug(cd->dev, DL_DEBUG, "%s: revctrl =0x%08X (%d)\n",
		__func__, ttdata->revctrl, ttdata->revctrl);
	pt_debug(cd->dev, DL_DEBUG, "%s: fw_ver_conf =0x%04X (%d)\n",
		__func__, ttdata->fw_ver_conf, ttdata->fw_ver_conf);
	pt_debug(cd->dev, DL_DEBUG, "%s: bl_ver_major =0x%02X (%d)\n",
		__func__, ttdata->bl_ver_major, ttdata->bl_ver_major);
	pt_debug(cd->dev, DL_DEBUG, "%s: bl_ver_minor =0x%02X (%d)\n",
		__func__, ttdata->bl_ver_minor, ttdata->bl_ver_minor);
	pt_debug(cd->dev, DL_DEBUG, "%s: jtag_id_h =0x%04X (%d)\n",
		__func__, ttdata->jtag_id_h, ttdata->jtag_id_h);
	pt_debug(cd->dev, DL_DEBUG, "%s: jtag_id_l =0x%04X (%d)\n",
		__func__, ttdata->jtag_id_l, ttdata->jtag_id_l);

	for (i = 0; i < PT_NUM_MFGID; i++)
		pt_debug(cd->dev, DL_DEBUG,
			"%s: mfg_id[%d] =0x%02X (%d)\n",
			__func__, i, ttdata->mfg_id[i],
			ttdata->mfg_id[i]);

	pt_debug(cd->dev, DL_DEBUG, "%s: post_code =0x%04X (%d)\n",
		__func__, ttdata->post_code, ttdata->post_code);
	pt_debug(cd->dev, DL_DEBUG, "%s: electrodes_x =0x%02X (%d)\n",
		__func__, scd->electrodes_x, scd->electrodes_x);
	pt_debug(cd->dev, DL_DEBUG, "%s: electrodes_y =0x%02X (%d)\n",
		__func__, scd->electrodes_y, scd->electrodes_y);
	pt_debug(cd->dev, DL_DEBUG, "%s: len_x =0x%04X (%d)\n",
		__func__, scd->len_x, scd->len_x);
	pt_debug(cd->dev, DL_DEBUG, "%s: len_y =0x%04X (%d)\n",
		__func__, scd->len_y, scd->len_y);
	pt_debug(cd->dev, DL_DEBUG, "%s: res_x =0x%04X (%d)\n",
		__func__, scd->res_x, scd->res_x);
	pt_debug(cd->dev, DL_DEBUG, "%s: res_y =0x%04X (%d)\n",
		__func__, scd->res_y, scd->res_y);
	pt_debug(cd->dev, DL_DEBUG, "%s: max_z =0x%04X (%d)\n",
		__func__, scd->max_z, scd->max_z);
	pt_debug(cd->dev, DL_DEBUG, "%s: origin_x =0x%02X (%d)\n",
		__func__, scd->origin_x, scd->origin_x);
	pt_debug(cd->dev, DL_DEBUG, "%s: origin_y =0x%02X (%d)\n",
		__func__, scd->origin_y, scd->origin_y);
	pt_debug(cd->dev, DL_DEBUG, "%s: panel_id =0x%02X (%d)\n",
		__func__, scd->panel_id, scd->panel_id);
	pt_debug(cd->dev, DL_DEBUG, "%s: btn =0x%02X (%d)\n",
		__func__, scd->btn, scd->btn);
	pt_debug(cd->dev, DL_DEBUG, "%s: scan_mode =0x%02X (%d)\n",
		__func__, scd->scan_mode, scd->scan_mode);
	pt_debug(cd->dev, DL_DEBUG,
		"%s: max_num_of_tch_per_refresh_cycle =0x%02X (%d)\n",
		__func__, scd->max_tch, scd->max_tch);
	pt_debug(cd->dev, DL_DEBUG, "%s: xy_mode =%p\n",
		__func__, si->xy_mode);
	pt_debug(cd->dev, DL_DEBUG, "%s: xy_data =%p\n",
		__func__, si->xy_data);
}

/*******************************************************************************
 * FUNCTION: pt_get_sysinfo_regs
 *
 * SUMMARY: Setup all the core data System information based on the response
 *	of the System Information PIP command.
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *      *cd - pointer to core data structure
 ******************************************************************************/
static int pt_get_sysinfo_regs(struct pt_core_data *cd)
{
	struct pt_sysinfo *si = &cd->sysinfo;
	int rc;

	rc = pt_si_get_btn_data(cd);
	if (rc < 0)
		return rc;

	pt_si_get_ttdata(cd);

	pt_si_get_sensing_conf_data(cd);

	pt_si_setup(cd);

	pt_si_put_log_data(cd);

	si->ready = true;
	return rc;
}

/*******************************************************************************
 * FUNCTION: pt_free_si_ptrs
 *
 * SUMMARY: Frees all memory associated with the System Infomation within
 *	core data
 *
 * RETURN: n/a
 *
 * PARAMETERS:
 *      *cd - pointer to core data structure
 ******************************************************************************/
static void pt_free_si_ptrs(struct pt_core_data *cd)
{
	struct pt_sysinfo *si = &cd->sysinfo;

	kfree(si->btn);
	kfree(si->xy_mode);
	kfree(si->xy_data);
}

/*******************************************************************************
 * FUNCTION: pt_hid_output_get_sysinfo_
 *
 * SUMMARY: Sends the PIP Get SYS INFO command to the DUT and waits for the
 *	response.
 *
 * RETURN::
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*cd - pointer to core data structure
 ******************************************************************************/
static int pt_hid_output_get_sysinfo_(struct pt_core_data *cd)
{
	int rc;
	struct pt_hid_output hid_output = {
		HID_OUTPUT_APP_COMMAND(HID_OUTPUT_GET_SYSINFO),
		.timeout_ms = PT_HID_OUTPUT_GET_SYSINFO_TIMEOUT,
	};

	rc = pt_hid_send_output_and_wait_(cd, &hid_output);
	if (rc)
		return rc;

	rc = pt_get_sysinfo_regs(cd);
	if (rc)
		pt_free_si_ptrs(cd);

	return rc;
}

/*******************************************************************************
 * FUNCTION: pt_hid_output_get_sysinfo
 *
 * SUMMARY: Protected call to pt_hid_output_get_sysinfo_
 *
 * RETURN::
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*cd - pointer to core data structure
 ******************************************************************************/
static int pt_hid_output_get_sysinfo(struct pt_core_data *cd)
{
	int rc;

	rc = request_exclusive(cd, cd->dev, PT_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		pt_debug(cd->dev, DL_ERROR,
				"%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->dev);
		return rc;
	}

	rc = pt_hid_output_get_sysinfo_(cd);

	if (release_exclusive(cd, cd->dev) < 0)
		pt_debug(cd->dev, DL_ERROR,
				"%s: fail to release exclusive\n", __func__);

	return rc;
}

/*******************************************************************************
 * FUNCTION: pt_pip_suspend_scanning_
 *
 * SUMMARY: Sends the PIP Suspend Scanning command to the DUT
 *
 * RETURN::
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*cd - pointer to core data structure
 ******************************************************************************/
static int pt_pip_suspend_scanning_(struct pt_core_data *cd)
{
	int rc;

	struct pt_hid_output hid_output = {
		HID_OUTPUT_APP_COMMAND(HID_OUTPUT_SUSPEND_SCANNING),
	};

	rc = pt_hid_send_output_and_wait_(cd, &hid_output);
	if (rc) {
		pt_debug(cd->dev, DL_ERROR,
			"%s: Suspend Scan PIP cmd failed. rc = %d\n",
			__func__, rc);
	}
	return rc;
}

/*******************************************************************************
 * FUNCTION: pt_pip_suspend_scanning
 *
 * SUMMARY: Protected wrapper for calling pt_hid_output_suspend_scanning_
 *
 * RETURN::
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*cd - pointer to core data structure
 ******************************************************************************/
static int pt_pip_suspend_scanning(struct pt_core_data *cd)
{
	int rc;

	rc = request_exclusive(cd, cd->dev, PT_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		pt_debug(cd->dev, DL_ERROR,
				"%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->dev);
		return rc;
	}

	rc = pt_pip_suspend_scanning_(cd);

	if (release_exclusive(cd, cd->dev) < 0)
		pt_debug(cd->dev, DL_ERROR,
				"%s: fail to release exclusive\n", __func__);

	return rc;
}

/*******************************************************************************
 * FUNCTION: _pt_request_pip_suspend_scanning
 *
 * SUMMARY: Function pointer included in core_nonhid_cmd struct for external
 *	calls to the protected or unprotected call to pt_pip_suspend_scanning
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*dev - pointer to device structure
 *	 protect - 0 = call non-protected function
 *		   1 = call protected function
 ******************************************************************************/
static int _pt_request_pip_suspend_scanning(struct device *dev,
		int protect)
{
	struct pt_core_data *cd = dev_get_drvdata(dev);

	if (protect)
		return pt_pip_suspend_scanning(cd);

	return pt_pip_suspend_scanning_(cd);
}

/*******************************************************************************
 * FUNCTION: pt_pip_resume_scanning_
 *
 * SUMMARY: Sends the PIP Resume Scanning command to the DUT
 *
 * RETURN::
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*cd - pointer to core data structure
 ******************************************************************************/
static int pt_pip_resume_scanning_(struct pt_core_data *cd)
{
	int rc;

	struct pt_hid_output hid_output = {
		HID_OUTPUT_APP_COMMAND(HID_OUTPUT_RESUME_SCANNING),
	};

	rc = pt_hid_send_output_and_wait_(cd, &hid_output);
	if (rc) {
		pt_debug(cd->dev, DL_ERROR,
			"%s: Resume Scan PIP cmd failed. rc = %d\n",
			__func__, rc);
	}
	return rc;
}

/*******************************************************************************
 * FUNCTION: pt_pip_resume_scanning
 *
 * SUMMARY: Protected wrapper for calling pt_pip_resume_scanning_
 *
 * RETURN::
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*cd - pointer to core data structure
 ******************************************************************************/
static int pt_pip_resume_scanning(struct pt_core_data *cd)
{
	int rc;

	rc = request_exclusive(cd, cd->dev, PT_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		pt_debug(cd->dev, DL_ERROR,
			"%s: fail get exclusive ex=%p own=%p\n",
			__func__, cd->exclusive_dev, cd->dev);
		return rc;
	}

	rc = pt_pip_resume_scanning_(cd);

	if (release_exclusive(cd, cd->dev) < 0)
		pt_debug(cd->dev, DL_ERROR,
			"%s: fail to release exclusive\n", __func__);

	return rc;
}

/*******************************************************************************
 * FUNCTION: _pt_request_pip_resume_scanning
 *
 * SUMMARY: Function pointer included in core_nonhid_cmd struct for external
 *	calls to the protected or unprotected call to pt_pip_resume_scanning
 *
 * RETURN::
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*dev - pointer to device structure
 *	 protect - 0 = call non-protected function
 *		   1 = call protected function
 ******************************************************************************/
static int _pt_request_pip_resume_scanning(struct device *dev,
		int protect)
{
	struct pt_core_data *cd = dev_get_drvdata(dev);

	if (protect)
		return pt_pip_resume_scanning(cd);

	return pt_pip_resume_scanning_(cd);
}

/*******************************************************************************
 * FUNCTION: pt_pip_get_param_
 *
 * SUMMARY: Sends a PIP command 0x05 Get Parameter to the DUT and returns
 *	the 32bit parameter value
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*cd       - pointer to core data
 *	 param_id - parameter ID to retrieve
 *	*value    - value of DUT parameter
 ******************************************************************************/
static int pt_pip_get_param_(struct pt_core_data *cd,
		u8 param_id, u32 *value)
{
	int write_length = 1;
	u8 param[1] = { param_id };
	u8 read_param_id;
	int param_size;
	u8 *ptr;
	int rc;
	int i;
	struct pt_hid_output hid_output = {
		HID_OUTPUT_APP_COMMAND(HID_OUTPUT_GET_PARAM),
		.write_length = write_length,
		.write_buf = param,
	};

	rc = pt_hid_send_output_and_wait_(cd, &hid_output);
	if (rc)
		return rc;

	read_param_id = cd->response_buf[5];
	if (read_param_id != param_id)
		return -EPROTO;

	param_size = cd->response_buf[6];
	ptr = &cd->response_buf[7];
	*value = 0;
	for (i = 0; i < param_size; i++)
		*value += ptr[i] << (i * 8);
	return 0;
}

/*******************************************************************************
 * FUNCTION: pt_pip_get_param
 *
 * SUMMARY: Protected call to pt_hid_output_get_param_ by a request exclusive
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*cd       - pointer to core data
 *	 param_id - parameter ID to retrieve
 *	*value    - value of DUT parameter
 ******************************************************************************/
static int pt_pip_get_param(struct pt_core_data *cd,
		u8 param_id, u32 *value)
{
	int rc;

	rc = request_exclusive(cd, cd->dev, PT_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		pt_debug(cd->dev, DL_ERROR,
				"%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->dev);
		return rc;
	}

	rc = pt_pip_get_param_(cd, param_id, value);

	if (release_exclusive(cd, cd->dev) < 0)
		pt_debug(cd->dev, DL_ERROR,
				"%s: fail to release exclusive\n", __func__);

	return rc;
}

/*******************************************************************************
 * FUNCTION: _pt_request_pip_get_param
 *
 * SUMMARY: Function pointer included in core_nonhid_cmd struct for external
 *	calls to the protected or unprotected call to pt_pip_get_param
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*dev      - pointer to device structure
 *	 protect  - flag to call protected or non protected function
 *	 param_id - parameter ID to retrieve
 *	*value    - value of DUT parameter
 ******************************************************************************/
int _pt_request_pip_get_param(struct device *dev,
		int protect, u8 param_id, u32 *value)
{
	struct pt_core_data *cd = dev_get_drvdata(dev);

	if (protect)
		return pt_pip_get_param(cd, param_id, value);

	return pt_pip_get_param_(cd, param_id, value);
}

/*******************************************************************************
 * FUNCTION: pt_pip_set_param_
 *
 * SUMMARY: Sends a PIP command 0x06 Set Parameter to the DUT writing the
 *	passed in value to flash
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*cd       - pointer to core data
 *	 param_id - parameter ID to set
 *	 value    - value to write
 *	 size     - size to write
 ******************************************************************************/
static int pt_pip_set_param_(struct pt_core_data *cd,
		u8 param_id, u32 value, u8 size)
{
	u8 write_buf[6];
	u8 *ptr = &write_buf[2];
	int rc;
	int i;
	struct pt_hid_output hid_output = {
		HID_OUTPUT_APP_COMMAND(HID_OUTPUT_SET_PARAM),
		.write_buf = write_buf,
	};

	write_buf[0] = param_id;
	write_buf[1] = size;
	for (i = 0; i < size; i++) {
		ptr[i] = value & 0xFF;
		value = value >> 8;
	}

	hid_output.write_length = 2 + size;

	rc = pt_hid_send_output_and_wait_(cd, &hid_output);
	if (rc)
		return rc;

	if (param_id != cd->response_buf[5] || size != cd->response_buf[6])
		return -EPROTO;

	return 0;
}

/*******************************************************************************
 * FUNCTION: pt_pip_set_param
 *
 * SUMMARY: Protected call to pt_hid_output_set_param_ by a request exclusive
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*cd       - pointer to core data
 *	 param_id - parameter ID to set
 *	 value    - value to write
 *	 size     - size to write
 ******************************************************************************/
static int pt_pip_set_param(struct pt_core_data *cd,
		u8 param_id, u32 value, u8 size)
{
	int rc;

	rc = request_exclusive(cd, cd->dev, PT_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		pt_debug(cd->dev, DL_ERROR,
				"%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->dev);
		return rc;
	}

	rc = pt_pip_set_param_(cd, param_id, value, size);

	if (release_exclusive(cd, cd->dev) < 0)
		pt_debug(cd->dev, DL_ERROR,
				"%s: fail to release exclusive\n", __func__);

	return rc;
}

/*******************************************************************************
 * FUNCTION: _pt_request_pip_set_param
 *
 * SUMMARY: Function pointer included in core_nonhid_cmd struct for external
 *	calls to the protected or unprotected call to pt_pip_set_param
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*dev      - pointer to device structure
 *	 protect  - flag to call protected or non-protected
 *	 param_id - parameter ID to set
 *	 value    - value to write
 *	 size     - size to write
 ******************************************************************************/
int _pt_request_pip_set_param(struct device *dev, int protect,
	u8 param_id, u32 value, u8  size)
{
	struct pt_core_data *cd = dev_get_drvdata(dev);

	if (protect)
		return pt_pip_set_param(cd, param_id, value, size);

	return pt_pip_set_param_(cd, param_id, value, size);
}

/*******************************************************************************
 * FUNCTION: _pt_pip_enter_easywake_state_
 *
 * SUMMARY: Sends a PIP command 0x09 Enter EasyWake State to the DUT
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*dev         - pointer to device structure
 *	 data        - easywake guesture (Only used for PIP1.6 and earlier)
 *	*return_data - return status if easywake was entered
 ******************************************************************************/
static int pt_hid_output_enter_easywake_state_(
		struct pt_core_data *cd, u8 data, u8 *return_data)
{
	int write_length = 1;
	u8 param[1] = { data };
	int rc;
	struct pt_hid_output hid_output = {
		HID_OUTPUT_APP_COMMAND(HID_OUTPUT_ENTER_EASYWAKE_STATE),
		.write_length = write_length,
		.write_buf = param,
	};

	rc = pt_hid_send_output_and_wait_(cd, &hid_output);
	if (rc)
		return rc;

	*return_data = cd->response_buf[5];
	return rc;
}

/*******************************************************************************
 * FUNCTION: pt_pip_verify_config_block_crc_
 *
 * SUMMARY: Sends the PIP "Verify Data Block CRC" (0x20) command to the DUT
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*cd             - pointer the core data structure
 *	 ebid           - enumerated block ID
 *	*status         - PIP command status
 *	 calculated_crc - calculated CRC
 *	 stored_crc     - stored CRC in config area
 ******************************************************************************/
static int pt_pip_verify_config_block_crc_(
		struct pt_core_data *cd, u8 ebid, u8 *status,
		u16 *calculated_crc, u16 *stored_crc)
{
	int write_length = 1;
	u8 param[1] = { ebid };
	u8 *ptr;
	int rc;
	struct pt_hid_output hid_output = {
		HID_OUTPUT_APP_COMMAND(HID_OUTPUT_VERIFY_CONFIG_BLOCK_CRC),
		.write_length = write_length,
		.write_buf = param,
	};

	rc = pt_hid_send_output_and_wait_(cd, &hid_output);
	if (rc)
		return rc;

	ptr = &cd->response_buf[5];
	*status = ptr[0];
	*calculated_crc = get_unaligned_le16(&ptr[1]);
	*stored_crc = get_unaligned_le16(&ptr[3]);
	return 0;
}

/*******************************************************************************
 * FUNCTION: pt_pip_verify_config_block_crc
 *
 * SUMMARY: Protected call to pt_hid_output_verify_config_block_crc_() within
 *	an exclusive access to the DUT.
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*cd             - pointer the core data structure
 *	 ebid           - enumerated block ID
 *	*status         - PIP command status
 *	 calculated_crc - calculated CRC
 *	 stored_crc     - stored CRC in config area
 ******************************************************************************/
static int pt_pip_verify_config_block_crc(
		struct pt_core_data *cd, u8 ebid, u8 *status,
		u16 *calculated_crc, u16 *stored_crc)
{
	int rc;

	rc = request_exclusive(cd, cd->dev, PT_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		pt_debug(cd->dev, DL_ERROR,
				"%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->dev);
		return rc;
	}

	rc = pt_pip_verify_config_block_crc_(cd, ebid, status,
			calculated_crc, stored_crc);

	if (release_exclusive(cd, cd->dev) < 0)
		pt_debug(cd->dev, DL_ERROR,
				"%s: fail to release exclusive\n", __func__);

	return rc;
}

/*******************************************************************************
 * FUNCTION: _pt_request_pip_verify_config_block_crc
 *
 * SUMMARY: Function pointer included in core_nonhid_cmd struct for external
 *	calls to the protected or unprotected call to
 *	pt_pip_verify_config_block_crc_
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*dev            - pointer to device structure
 *	 protect        - flag to call protected or non-protected
 *	 ebid           - enumerated block ID
 *	*status         - PIP command status
 *	 calculated_crc - calculated CRC
 *	 stored_crc     - stored CRC in config area
 ******************************************************************************/
static int _pt_request_pip_verify_config_block_crc(
		struct device *dev, int protect, u8 ebid, u8 *status,
		u16 *calculated_crc, u16 *stored_crc)
{
	struct pt_core_data *cd = dev_get_drvdata(dev);

	if (protect)
		return pt_pip_verify_config_block_crc(cd, ebid,
				status, calculated_crc, stored_crc);

	return pt_pip_verify_config_block_crc_(cd, ebid,
			status, calculated_crc, stored_crc);
}

/*******************************************************************************
 * FUNCTION: pt_pip_get_config_row_size_
 *
 * SUMMARY: Sends the PIP "Get Data Row Size" (0x21) command to the DUT
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*cd       - pointer to core data
 *	 protect  - flag to call protected or non-protected
 *	*row_size - pointer to store the retrieved row size
 ******************************************************************************/
static int pt_pip_get_config_row_size_(struct pt_core_data *cd,
		u16 *row_size)
{
	int rc;
	struct pt_hid_output hid_output = {
		HID_OUTPUT_APP_COMMAND(HID_OUTPUT_GET_CONFIG_ROW_SIZE),
	};

	rc = pt_hid_send_output_and_wait_(cd, &hid_output);
	if (rc)
		return rc;

	*row_size = get_unaligned_le16(&cd->response_buf[5]);
	return 0;
}

/*******************************************************************************
 * FUNCTION: pt_pip_get_config_row_size
 *
 * SUMMARY: Protected call to pt_hid_output_get_config_row_size_ within
 *	an exclusive access to the DUT.
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*cd       - pointer to core data
 *	 protect  - flag to call protected or non-protected
 *	*row_size - pointer to store the retrieved row size
 ******************************************************************************/
static int pt_pip_get_config_row_size(struct pt_core_data *cd,
		u16 *row_size)
{
	int rc;

	rc = request_exclusive(cd, cd->dev, PT_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		pt_debug(cd->dev, DL_ERROR,
				"%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->dev);
		return rc;
	}

	rc = pt_pip_get_config_row_size_(cd, row_size);

	if (release_exclusive(cd, cd->dev) < 0)
		pt_debug(cd->dev, DL_ERROR,
				"%s: fail to release exclusive\n", __func__);

	return rc;
}

/*******************************************************************************
 * FUNCTION: _pt_request_pip_get_config_row_size
 *
 * SUMMARY: Function pointer included in core_nonhid_cmd struct for external
 *	calls to the protected or unprotected call to
 *	pt_pip_get_config_row_size_
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*dev      - pointer to device structure
 *	 protect  - flag to call protected or non-protected
 *	*row_size - pointer to store the retrieved row size
 ******************************************************************************/
static int _pt_request_pip_get_config_row_size(struct device *dev,
		int protect, u16 *row_size)
{
	struct pt_core_data *cd = dev_get_drvdata(dev);

	if (protect)
		return pt_pip_get_config_row_size(cd, row_size);

	return pt_pip_get_config_row_size_(cd, row_size);
}

/*******************************************************************************
 * FUNCTION: pt_hid_output_read_conf_block_
 *
 * SUMMARY: Sends the PIP "Read Data Block" (0x22) command to the DUT and print
 *  ouput data to the "read_buf" and update "crc".
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *  *cd          - pointer to core data
 *   row_number  - row number
 *   length      - length of data to read
 *   ebid        - block id
 *  *read_buf    - pointer to the buffer to store read data
 *  *crc         - pointer to store CRC of row data
 ******************************************************************************/
static int pt_hid_output_read_conf_block_(struct pt_core_data *cd,
		u16 row_number, u16 length, u8 ebid, u8 *read_buf, u16 *crc)
{
	int read_ebid;
	int read_length;
	int status;
	int rc;
	int write_length = 5;
	u8 write_buf[5];
	u8 cmd_offset = 0;
	struct pt_hid_output hid_output = {
		HID_OUTPUT_APP_COMMAND(HID_OUTPUT_READ_CONF_BLOCK),
		.write_length = write_length,
		.write_buf = write_buf,
	};

	write_buf[cmd_offset++] = LOW_BYTE(row_number);
	write_buf[cmd_offset++] = HI_BYTE(row_number);
	write_buf[cmd_offset++] = LOW_BYTE(length);
	write_buf[cmd_offset++] = HI_BYTE(length);
	write_buf[cmd_offset++] = ebid;

	rc = pt_hid_send_output_and_wait_(cd, &hid_output);
	if (rc)
		return rc;

	status = cd->response_buf[5];
	if (status)
		return -EINVAL;

	read_ebid = cd->response_buf[6];
	if ((read_ebid != ebid) || (cd->response_buf[9] != 0))
		return -EPROTO;

	read_length = get_unaligned_le16(&cd->response_buf[7]);
	if (length < read_length)
		length = read_length;

	memcpy(read_buf, &cd->response_buf[10], length);
	*crc = get_unaligned_le16(&cd->response_buf[read_length + 10]);

	return 0;
}

/*******************************************************************************
 * FUNCTION: pt_hid_output_read_conf_ver_
 *
 * SUMMARY: Read the configuration block from the DUT and parse out the config
 *	version
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*cd         - pointer to core data
 *	*config_ver - pointer to where to store the config version
 ******************************************************************************/
static int pt_hid_output_read_conf_ver_(struct pt_core_data *cd,
		u16 *config_ver)
{
	int rc;
	u8 read_buf[PT_TTCONFIG_VERSION_OFFSET + PT_TTCONFIG_VERSION_SIZE];
	u16 crc;

	rc = pt_hid_output_read_conf_block_(cd, PT_TTCONFIG_VERSION_ROW,
			PT_TTCONFIG_VERSION_OFFSET + PT_TTCONFIG_VERSION_SIZE,
			PT_TCH_PARM_EBID, read_buf, &crc);
	if (rc)
		return rc;

	*config_ver = get_unaligned_le16(
				&read_buf[PT_TTCONFIG_VERSION_OFFSET]);

	return 0;
}

/*******************************************************************************
 * FUNCTION: pt_hid_output_write_conf_block_
 *
 * SUMMARY: Sends the PIP "Write Data Block" (0x23) command to the DUT and
 *  write data to the data block.
 *
 * RETURN:
 *   0 = success
 *  !0 = failure
 *
 * PARAMETERS:
 *  *dev              - pointer to device structure
 *   row_number       - row in config block to write to
 *   write_length     - length of data to write
 *   ebid             - enumerated block ID
 *  *write_buf        - pointer to buffer to write
 *  *security_key     - pointer to security key to allow write
 *  *actual_write_len - pointer to store data length actually written
 ******************************************************************************/
static int pt_hid_output_write_conf_block_(struct pt_core_data *cd,
		u16 row_number, u16 write_length, u8 ebid, u8 *write_buf,
		u8 *security_key, u16 *actual_write_len)
{
	/* row_number + write_len + ebid + security_key + crc */
	int full_write_length = 2 + 2 + 1 + write_length + 8 + 2;
	u8 *full_write_buf;
	u8 cmd_offset = 0;
	u16 crc;
	int status;
	int rc;
	int read_ebid;
	u8 *data;
	struct pt_hid_output hid_output = {
		HID_OUTPUT_APP_COMMAND(HID_OUTPUT_WRITE_CONF_BLOCK),
		.write_length = full_write_length,
		.timeout_ms = PT_HID_OUTPUT_WRITE_CONF_BLOCK_TIMEOUT,
	};

	full_write_buf = kzalloc(full_write_length, GFP_KERNEL);
	if (!full_write_buf)
		return -ENOMEM;

	hid_output.write_buf = full_write_buf;
	full_write_buf[cmd_offset++] = LOW_BYTE(row_number);
	full_write_buf[cmd_offset++] = HI_BYTE(row_number);
	full_write_buf[cmd_offset++] = LOW_BYTE(write_length);
	full_write_buf[cmd_offset++] = HI_BYTE(write_length);
	full_write_buf[cmd_offset++] = ebid;
	data = &full_write_buf[cmd_offset];
	memcpy(data, write_buf, write_length);
	cmd_offset += write_length;
	memcpy(&full_write_buf[cmd_offset], security_key, 8);
	cmd_offset += 8;
	crc = _pt_compute_crc(data, write_length);
	full_write_buf[cmd_offset++] = LOW_BYTE(crc);
	full_write_buf[cmd_offset++] = HI_BYTE(crc);

	rc = pt_hid_send_output_and_wait_(cd, &hid_output);
	if (rc)
		goto exit;

	status = cd->response_buf[5];
	if (status) {
		rc = -EINVAL;
		goto exit;
	}

	read_ebid = cd->response_buf[6];
	if (read_ebid != ebid) {
		rc = -EPROTO;
		goto exit;
	}

	*actual_write_len = get_unaligned_le16(&cd->response_buf[7]);

exit:
	kfree(full_write_buf);
	return rc;
}

/*******************************************************************************
 * FUNCTION: pt_hid_output_write_conf_block
 *
 * SUMMARY: Protected call to pt_hid_output_write_conf_block_ within an
 *  exclusive access to the DUT.
 *
 * RETURN:
 *   0 = success
 *  !0 = failure
 *
 * PARAMETERS:
 *  *dev              - pointer to device structure
 *   row_number       - row in config block to write to
 *   write_length     - length of data to write
 *   ebid             - enumerated block ID
 *  *write_buf        - pointer to buffer to write
 *  *security_key     - pointer to security key to allow write
 *  *actual_write_len - pointer to store data length actually written
 ******************************************************************************/
static int pt_hid_output_write_conf_block(struct pt_core_data *cd,
		u16 row_number, u16 write_length, u8 ebid, u8 *write_buf,
		u8 *security_key, u16 *actual_write_len)
{
	int rc;

	rc = request_exclusive(cd, cd->dev, PT_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		pt_debug(cd->dev, DL_ERROR,
				"%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->dev);
		return rc;
	}

	rc = pt_hid_output_write_conf_block_(cd, row_number, write_length,
			ebid, write_buf, security_key, actual_write_len);

	if (release_exclusive(cd, cd->dev) < 0)
		pt_debug(cd->dev, DL_ERROR,
				"%s: fail to release exclusive\n", __func__);

	return rc;
}

/*******************************************************************************
 * FUNCTION: _pt_request_pip_write_conf_block
 *
 * SUMMARY: Function pointer included in core_nonhid_cmd struct for external
 *	calls to the protected or unprotected call to
 *	pt_hid_output_write_conf_block
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*dev              - pointer to device structure
 *	 protect          - flag to call protected or non-protected
 *	 row_number       - row in config block to write to
 *	 write_length     - length of data to write
 *	 ebid             - enumerated block ID
 *	*write_buf        - pointer to buffer to write
 *	*security_key     - pointer to security key to allow write
 *	*actual_write_len - pointer to store data length actually written
 ******************************************************************************/
static int _pt_request_pip_write_conf_block(struct device *dev,
		int protect, u16 row_number, u16 write_length, u8 ebid,
		u8 *write_buf, u8 *security_key, u16 *actual_write_len)
{
	struct pt_core_data *cd = dev_get_drvdata(dev);

	if (protect)
		return pt_hid_output_write_conf_block(cd, row_number,
				write_length, ebid, write_buf, security_key,
				actual_write_len);

	return pt_hid_output_write_conf_block_(cd, row_number,
			write_length, ebid, write_buf, security_key,
			actual_write_len);
}

/*******************************************************************************
 * FUNCTION: pt_pip_get_data_structure_
 *
 * SUMMARY: Sends the PIP "Retrieve Data Structure" (0x24) command to the DUT
 *	returning a structure of data defined by data_id
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*cd              - pointer to core data
 *	 read_offset     - read pointer offset
 *	 read_length     - length of data to read
 *	 data_id         - data ID to read
 *	*status          - pointer to store the read response status
 *	*data_format     - pointer to store format of data read
 *	*actual_read_len - pointer to store data length actually read
 *	*data            - pointer to store data read
 ******************************************************************************/
static int pt_pip_get_data_structure_(
		struct pt_core_data *cd, u16 read_offset, u16 read_length,
		u8 data_id, u8 *status, u8 *data_format, u16 *actual_read_len,
		u8 *data)
{
	int rc;
	u16 total_read_len = 0;
	u16 read_len;
	u16 off_buf = 0;
	u8 write_buf[5];
	u8 read_data_id;
	struct pt_hid_output hid_output = {
		HID_OUTPUT_APP_COMMAND(HID_OUTPUT_GET_DATA_STRUCTURE),
		.write_length = 5,
		.write_buf = write_buf,
	};

again:
	write_buf[0] = LOW_BYTE(read_offset);
	write_buf[1] = HI_BYTE(read_offset);
	write_buf[2] = LOW_BYTE(read_length);
	write_buf[3] = HI_BYTE(read_length);
	write_buf[4] = data_id;

	rc = pt_hid_send_output_and_wait_(cd, &hid_output);
	if (rc)
		return rc;

	if (cd->response_buf[5] != PT_CMD_STATUS_SUCCESS)
		goto set_status;

	read_data_id = cd->response_buf[6];
	if (read_data_id != data_id)
		return -EPROTO;

	read_len = get_unaligned_le16(&cd->response_buf[7]);
	if (read_len && data) {
		memcpy(&data[off_buf], &cd->response_buf[10], read_len);

		total_read_len += read_len;

		if (read_len < read_length) {
			read_offset += read_len;
			off_buf += read_len;
			read_length -= read_len;
			goto again;
		}
	}

	if (data_format)
		*data_format = cd->response_buf[9];
	if (actual_read_len)
		*actual_read_len = total_read_len;
set_status:
	if (status)
		*status = cd->response_buf[5];

	return rc;
}

/*******************************************************************************
 * FUNCTION: pt_pip_get_data_structure
 *
 * SUMMARY: Protected call to pt_hid_output_get_data_structure within
 *	an exclusive access to the DUT.
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*cd              - pointer to core data
 *	 read_offset     - read pointer offset
 *	 read_length     - length of data to read
 *	 data_id         - data ID to read
 *	*status          - pointer to store the read response status
 *	*data_format     - pointer to store format of data read
 *	*actual_read_len - pointer to store data length actually read
 *	*data            - pointer to store data read
 ******************************************************************************/
static int pt_pip_get_data_structure(
		struct pt_core_data *cd, u16 read_offset, u16 read_length,
		u8 data_id, u8 *status, u8 *data_format, u16 *actual_read_len,
		u8 *data)
{
	int rc;

	rc = request_exclusive(cd, cd->dev, PT_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		pt_debug(cd->dev, DL_ERROR,
				"%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->dev);
		return rc;
	}

	rc = pt_pip_get_data_structure_(cd, read_offset,
			read_length, data_id, status, data_format,
			actual_read_len, data);

	if (release_exclusive(cd, cd->dev) < 0)
		pt_debug(cd->dev, DL_ERROR,
			"%s: fail to release exclusive\n", __func__);

	return rc;
}

/*******************************************************************************
 * FUNCTION: _pt_request_pip_get_data_structure
 *
 * SUMMARY: Function pointer included in core_nonhid_cmd struct for external
 *	calls to the protected or unprotected call to
 *	pt_pip_get_data_structure
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*dev             - pointer to device structure
 *	 protect         - flag to call protected or non-protected
 *	 read_offset     - read pointer offset
 *	 read_length     - length of data to read
 *	 data_id         - data ID to read
 *	*status          - pointer to store the read response status
 *	*data_format     - pointer to store format of data read
 *	*actual_read_len - pointer to store data length actually read
 *	*data            - pointer to store data read
 ******************************************************************************/
static int _pt_request_pip_get_data_structure(struct device *dev,
		int protect, u16 read_offset, u16 read_length, u8 data_id,
		u8 *status, u8 *data_format, u16 *actual_read_len, u8 *data)
{
	struct pt_core_data *cd = dev_get_drvdata(dev);

	if (protect)
		return pt_pip_get_data_structure(cd,
				read_offset, read_length, data_id, status,
				data_format, actual_read_len, data);

	return pt_pip_get_data_structure_(cd,
			read_offset, read_length, data_id, status,
			data_format, actual_read_len, data);
}

/*******************************************************************************
 * FUNCTION: pt_pip_run_selftest_
 *
 * SUMMARY: Sends the PIP "Run Self Test" (0x26) command to the DUT
 *	to execute a FW built in self test
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*cd                   - pointer to core data
 *	 test_id              - enumerated test ID to run
 *	 write_idacs_to_flash - flag whether to write new IDACS to flash
 *	*status               - pointer to store the read response status
 *	*summary_results      - pointer to store the results summary
 *	*results_available    - pointer to store if results are available
 *****************************************************************************/
static int pt_pip_run_selftest_(
		struct pt_core_data *cd, u8 test_id,
		u8 write_idacs_to_flash, u8 *status, u8 *summary_result,
		u8 *results_available)
{
	int rc;
	u8 write_buf[2];
	struct pt_hid_output hid_output = {
		HID_OUTPUT_APP_COMMAND(HID_OUTPUT_RUN_SELF_TEST),
		.write_length = 2,
		.write_buf = write_buf,
		.timeout_ms = PT_HID_OUTPUT_RUN_SELF_TEST_TIMEOUT,
	};

	write_buf[0] = test_id;
	write_buf[1] = write_idacs_to_flash;

	rc = pt_hid_send_output_and_wait_(cd, &hid_output);
	if (rc)
		return rc;

	if (status)
		*status = cd->response_buf[5];
	if (summary_result)
		*summary_result = cd->response_buf[6];
	if (results_available)
		*results_available = cd->response_buf[7];

	return rc;
}

/*******************************************************************************
 * FUNCTION: pt_pip_run_selftest
 *
 * SUMMARY: Protected call to pt_hid_output_run_selftest within
 *	an exclusive access to the DUT.
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*cd                   - pointer to core data
 *	 test_id              - enumerated test ID to run
 *	 write_idacs_to_flash - flag whether to write new IDACS to flash
 *	*status               - pointer to store the read response status
 *	*summary_results      - pointer to store the results summary
 *	*results_available    - pointer to store if results are available
 ******************************************************************************/
static int pt_pip_run_selftest(
		struct pt_core_data *cd, u8 test_id,
		u8 write_idacs_to_flash, u8 *status, u8 *summary_result,
		u8 *results_available)
{
	int rc;

	rc = request_exclusive(cd, cd->dev, PT_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		pt_debug(cd->dev, DL_ERROR,
				"%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->dev);
		return rc;
	}

	rc = pt_pip_run_selftest_(cd, test_id,
			write_idacs_to_flash, status, summary_result,
			results_available);

	if (release_exclusive(cd, cd->dev) < 0)
		pt_debug(cd->dev, DL_ERROR,
				"%s: fail to release exclusive\n", __func__);

	return rc;
}

/*******************************************************************************
 * FUNCTION: _pt_request_pip_run_selftest
 *
 * SUMMARY: Function pointer included in core_nonhid_cmd struct for external
 *	calls to the protected or unprotected call to pt_pip_run_selftest
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*dev                  - pointer to device structure
 *	 protect              - flag to call protected or non-protected
 *	 test_id              - enumerated test ID to run
 *	 write_idacs_to_flash - flag whether to write new IDACS to flash
 *	*status               - pointer to store the read response status
 *	*summary_results      - pointer to store the results summary
 *	*results_available    - pointer to store if results are available
 ******************************************************************************/
static int _pt_request_pip_run_selftest(struct device *dev,
		int protect, u8 test_id, u8 write_idacs_to_flash, u8 *status,
		u8 *summary_result, u8 *results_available)
{
	struct pt_core_data *cd = dev_get_drvdata(dev);

	if (protect)
		return pt_pip_run_selftest(cd, test_id,
				write_idacs_to_flash, status, summary_result,
				results_available);

	return pt_pip_run_selftest_(cd, test_id,
			write_idacs_to_flash, status, summary_result,
			results_available);
}

/*******************************************************************************
 * FUNCTION: _pt_pip_get_selftest_result_
 *
 * SUMMARY: Sends the PIP "Get Self Test Results" (0x27) command to the DUT
 *	to retrieve the self test results from the self test already executed
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*cd              - pointer to core data
 *	 read_offset     - read pointer offset
 *	 read_length     - length of data to read
 *	 test_id         - enumerated test ID to read selftest results from
 *	*status          - pointer to store the read response status
 *	*actual_read_len - pointer to store data length actually read
 *	*status          - pointer to where the cmd response statas is stored
 ******************************************************************************/
static int pt_pip_get_selftest_result_(
		struct pt_core_data *cd, u16 read_offset, u16 read_length,
		u8 test_id, u8 *status, u16 *actual_read_len, u8 *data)
{
	int rc;
	u16 total_read_len = 0;
	u16 read_len;
	u16 off_buf = 0;
	u8 write_buf[5];
	u8 read_test_id;
	bool repeat;
	struct pt_hid_output hid_output = {
		HID_OUTPUT_APP_COMMAND(HID_OUTPUT_GET_SELF_TEST_RESULT),
		.write_length = 5,
		.write_buf = write_buf,
	};

	/*
	 * Do not repeat reading for Auto Shorts test
	 * when PIP version < 1.3
	 */
	repeat = IS_PIP_VER_GE(&cd->sysinfo, 1, 3)
			|| test_id != PT_ST_ID_AUTOSHORTS;

again:
	write_buf[0] = LOW_BYTE(read_offset);
	write_buf[1] = HI_BYTE(read_offset);
	write_buf[2] = LOW_BYTE(read_length);
	write_buf[3] = HI_BYTE(read_length);
	write_buf[4] = test_id;

	rc = pt_hid_send_output_and_wait_(cd, &hid_output);
	if (rc)
		return rc;

	if (cd->response_buf[5] != PT_CMD_STATUS_SUCCESS)
		goto set_status;

	read_test_id = cd->response_buf[6];
	if (read_test_id != test_id)
		return -EPROTO;

	read_len = get_unaligned_le16(&cd->response_buf[7]);
	if (read_len && data) {
		memcpy(&data[off_buf], &cd->response_buf[10], read_len);

		total_read_len += read_len;

		if (repeat && read_len < read_length) {
			read_offset += read_len;
			off_buf += read_len;
			read_length -= read_len;
			goto again;
		}
	}

	if (actual_read_len)
		*actual_read_len = total_read_len;
set_status:
	if (status)
		*status = cd->response_buf[5];

	return rc;
}

/*******************************************************************************
 * FUNCTION: _pt_pip_get_selftest_result
 *
 * SUMMARY: Protected call to pt_hid_output_get_selftest_result by exclusive
 *	access to the DUT
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*cd              - pointer to core data
 *	 read_offset     - read pointer offset
 *	 read_length     - length of data to read
 *	 test_id         - enumerated test ID to read selftest results from
 *	*status          - pointer to store the read response status
 *	*actual_read_len - pointer to store data length actually read
 *	*status          - pointer to where the cmd response statas is stored
 ******************************************************************************/
static int pt_pip_get_selftest_result(
		struct pt_core_data *cd, u16 read_offset, u16 read_length,
		u8 test_id, u8 *status, u16 *actual_read_len, u8 *data)
{
	int rc;

	rc = request_exclusive(cd, cd->dev, PT_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		pt_debug(cd->dev, DL_ERROR,
				"%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->dev);
		return rc;
	}

	rc = pt_pip_get_selftest_result_(cd, read_offset,
		read_length, test_id, status, actual_read_len, data);

	if (release_exclusive(cd, cd->dev) < 0)
		pt_debug(cd->dev, DL_ERROR,
				"%s: fail to release exclusive\n", __func__);

	return rc;
}

/*******************************************************************************
 * FUNCTION: _pt_request_pip_get_selftest_result
 *
 * SUMMARY: Function pointer included in core_nonhid_cmd struct for external
 *	calls to the protected or unprotected call to pt_pip_get_selftest_result
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*dev             - pointer to device structure
 *	 protect         - flag to call protected or non-protected
 *	 read_offset     - read pointer offset
 *	 read_length     - length of data to read
 *	 test_id         - enumerated test ID to read selftest results from
 *	*status          - pointer to store the read response status
 *	*actual_read_len - pointer to store data length actually read
 *	*data            - pointer to where the data read is stored
 ******************************************************************************/
static int _pt_request_pip_get_selftest_result(struct device *dev,
		int protect, u16 read_offset, u16 read_length, u8 test_id,
		u8 *status, u16 *actual_read_len, u8 *data)
{
	struct pt_core_data *cd = dev_get_drvdata(dev);

	if (protect)
		return pt_pip_get_selftest_result(cd, read_offset,
				read_length, test_id, status, actual_read_len,
				data);

	return pt_pip_get_selftest_result_(cd, read_offset,
			read_length, test_id, status, actual_read_len,
			data);
}

/*******************************************************************************
 * FUNCTION: pt_pip_calibrate_idacs_
 *
 * SUMMARY: Send the PIP Calibrate IDACs command (0x28) to the DUT waiting
 *	for the response
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*cd     - pointer to core data
 *	 mode   - sense mode to calibrate (0-5)
 *	*status - pointer to where the command response statas is stored
 ******************************************************************************/
static int pt_pip_calibrate_idacs_(struct pt_core_data *cd,
		u8 mode, u8 *status)
{
	int rc;
	int write_length = 1;
	u8 write_buf[1];
	u8 cmd_offset = 0;
	struct pt_hid_output hid_output = {
		HID_OUTPUT_APP_COMMAND(HID_OUTPUT_CALIBRATE_IDACS),
		.write_length = write_length,
		.write_buf = write_buf,
		.timeout_ms = PT_HID_OUTPUT_CALIBRATE_IDAC_TIMEOUT,
	};

	write_buf[cmd_offset++] = mode;
	rc =  pt_hid_send_output_and_wait_(cd, &hid_output);
	if (rc)
		return rc;

	*status = cd->response_buf[5];
	if (*status)
		return -EINVAL;

	return 0;
}

/*******************************************************************************
 * FUNCTION: pt_pip_calibrate_idacs
 *
 * SUMMARY: Protected call to pt_hid_output_calibrate_idacs_ by exclusive
 *	access to the DUT
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*cd     - pointer to core data
 *	 mode   - sense mode to calibrate (0-5)
 *	*status - pointer to where the command response statas is stored
 ******************************************************************************/
static int pt_pip_calibrate_idacs(struct pt_core_data *cd,
		u8 mode, u8 *status)
{
	int rc;

	rc = request_exclusive(cd, cd->dev, PT_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		pt_debug(cd->dev, DL_ERROR,
				"%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->dev);
		return rc;
	}

	rc = pt_pip_calibrate_idacs_(cd, mode, status);

	if (release_exclusive(cd, cd->dev) < 0)
		pt_debug(cd->dev, DL_ERROR,
				"%s: fail to release exclusive\n", __func__);

	return rc;
}

/*******************************************************************************
 * FUNCTION: _pt_request_pip_calibrate_idacs
 *
 * SUMMARY: Function pointer included in core_nonhid_cmd struct for external
 *	calls to the protected or unprotected call to pt_pip_calibrate_idacs
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*dev     - pointer to device structure
 *	 protect - flag to call protected or non-protected
 *	 mode    - sense mode to calibrate (0-5)
 *	*status  - pointer to where the command response statas is stored
 ******************************************************************************/
static int _pt_request_pip_calibrate_idacs(struct device *dev,
		int protect, u8 mode, u8 *status)
{
	struct pt_core_data *cd = dev_get_drvdata(dev);

	if (protect)
		return pt_pip_calibrate_idacs(cd, mode, status);

	return pt_pip_calibrate_idacs_(cd, mode, status);
}

/*******************************************************************************
 * FUNCTION: pt_hid_output_initialize_baselines_
 *
 * SUMMARY: Send the PIP "Initialize Baselines" command (0x29) to the DUT
 *  waiting for the response.
 *
 * RETURN:
 *   0 = success
 *  !0 = failure
 *
 * PARAMETERS:
 *  *cd       - pointer to core data
 *   test_id  - bit type flag to allow initialize baseline MUT,BTN,SELG
 *              each or together with a single command.
 *  *status   - pointer to where the command response statas is stored
 ******************************************************************************/
static int pt_hid_output_initialize_baselines_(
		struct pt_core_data *cd, u8 test_id, u8 *status)
{
	int rc;
	int write_length = 1;
	u8 write_buf[1];
	u8 cmd_offset = 0;
	struct pt_hid_output hid_output = {
		HID_OUTPUT_APP_COMMAND(HID_OUTPUT_INITIALIZE_BASELINES),
		.write_length = write_length,
		.write_buf = write_buf,
	};

	write_buf[cmd_offset++] = test_id;

	rc = pt_hid_send_output_and_wait_(cd, &hid_output);
	if (rc)
		return rc;

	*status = cd->response_buf[5];
	if (*status)
		return -EINVAL;

	return rc;
}

/*******************************************************************************
 * FUNCTION: pt_hid_output_initialize_baselines
 *
 * SUMMARY: Protected call to pt_hid_output_initialize_baselines_ by exclusive
 *  access to the DUT
 *
 * RETURN:
 *   0 = success
 *  !0 = failure
 *
 * PARAMETERS:
 *  *cd       - pointer to core data
 *   test_id  - enumerated ID against which to initialize the baseline
 *  *status   - pointer to where the command response statas is stored
 ******************************************************************************/
static int pt_hid_output_initialize_baselines(struct pt_core_data *cd,
		u8 test_id, u8 *status)
{
	int rc;

	rc = request_exclusive(cd, cd->dev, PT_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		pt_debug(cd->dev, DL_ERROR,
				"%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->dev);
		return rc;
	}

	rc = pt_hid_output_initialize_baselines_(cd, test_id, status);

	if (release_exclusive(cd, cd->dev) < 0)
		pt_debug(cd->dev, DL_ERROR,
				"%s: fail to release exclusive\n", __func__);

	return rc;
}

/*******************************************************************************
 * FUNCTION: _pt_request_pip_initialize_baselines
 *
 * SUMMARY: Function pointer included in core_nonhid_cmd struct for external
 *	calls to the protected or unprotected call to
 *	pt_pip_initialize_baselines
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*dev     - pointer to device structure
 *	 protect - flag to call protected or non-protected
 *	 test_id - enumerated ID against which to initialize the baseline
 *	*status  - pointer to where the command response statas is stored
 ******************************************************************************/
static int _pt_request_pip_initialize_baselines(struct device *dev,
		int protect, u8 test_id, u8 *status)
{
	struct pt_core_data *cd = dev_get_drvdata(dev);

	if (protect)
		return pt_hid_output_initialize_baselines(cd, test_id,
				status);

	return pt_hid_output_initialize_baselines_(cd, test_id, status);
}

/*******************************************************************************
 * FUNCTION: pt_hid_output_exec_panel_scan_
 *
 * SUMMARY: Send the PIP "Execute Panel Scan" command (0x2A) to the DUT waiting
 *  for the response
 *
 * RETURN:
 *   0 = success
 *  !0 = failure
 *
 * PARAMETERS:
 *  *cd  - pointer to core data
 ******************************************************************************/
static int pt_hid_output_exec_panel_scan_(struct pt_core_data *cd)
{
	struct pt_hid_output hid_output = {
		HID_OUTPUT_APP_COMMAND(HID_OUTPUT_EXEC_PANEL_SCAN),
	};

	return pt_hid_send_output_and_wait_(cd, &hid_output);
}

/*******************************************************************************
 * FUNCTION: pt_hid_output_exec_panel_scan
 *
 * SUMMARY: Protected call to pt_hid_output_exec_panel_scan_ by exclusive
 *  access to the DUT.
 *
 * RETURN:
 *   0 = success
 *  !0 = failure
 *
 * PARAMETERS:
 *  *cd  - pointer to core data
 ******************************************************************************/
static int pt_hid_output_exec_panel_scan(struct pt_core_data *cd)
{
	int rc;

	rc = request_exclusive(cd, cd->dev, PT_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		pt_debug(cd->dev, DL_ERROR,
				"%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->dev);
		return rc;
	}

	rc = pt_hid_output_exec_panel_scan_(cd);

	if (release_exclusive(cd, cd->dev) < 0)
		pt_debug(cd->dev, DL_ERROR,
				"%s: fail to release exclusive\n", __func__);

	return rc;
}

/*******************************************************************************
 * FUNCTION: _pt_request_pip_exec_panel_scan
 *
 * SUMMARY: Function pointer included in core_nonhid_cmd struct for external
 *	calls to the protected or unprotected call to
 *	pt_hid_output_exec_panel_scan
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*dev     - pointer to device structure
 *	 protect - flag to call protected or non-protected
 ******************************************************************************/
static int _pt_request_pip_exec_panel_scan(struct device *dev,
		int protect)
{
	struct pt_core_data *cd = dev_get_drvdata(dev);

	if (protect)
		return pt_hid_output_exec_panel_scan(cd);

	return pt_hid_output_exec_panel_scan_(cd);
}

/*******************************************************************************
 * FUNCTION: pt_hid_output_retrieve_panel_scan_
 *
 * SUMMARY: Sends the PIP "Retrieve Panel Scan" (0x2B) command to the DUT
 *  to retrieve the specified data type for a the last successful Execute
 *  Panel Scan command.
 *
 * RETURN:
 *    0 = success
 *   !0 = failure
 *
 * PARAMETERS:
 *  *dev             - pointer to device structure
 *   protect         - flag to call protected or non-protected
 *   read_offset     - read pointer offset
 *   read_count      - length of data to read
 *   data_id         - enumerated test ID to read selftest results from
 *  *response        - pointer to store the read response status
 *  *config          - pointer to store config data
 *  *actual_read_len - pointer to store data length actually read
 *  *read_buf        - pointer to the read buffer
 ******************************************************************************/
static int pt_hid_output_retrieve_panel_scan_(
		struct pt_core_data *cd, u16 read_offset, u16 read_count,
		u8 data_id, u8 *response, u8 *config, u16 *actual_read_len,
		u8 *read_buf)
{
	int status;
	u8 read_data_id;
	int rc;
	int write_length = 5;
	u8 write_buf[5];
	u8 cmd_offset = 0;
	u8 data_elem_size;
	int size;
	int data_size;
	struct pt_hid_output hid_output = {
		HID_OUTPUT_APP_COMMAND(HID_OUTPUT_RETRIEVE_PANEL_SCAN),
		.write_length = write_length,
		.write_buf = write_buf,
	};

	write_buf[cmd_offset++] = LOW_BYTE(read_offset);
	write_buf[cmd_offset++] = HI_BYTE(read_offset);
	write_buf[cmd_offset++] = LOW_BYTE(read_count);
	write_buf[cmd_offset++] = HI_BYTE(read_count);
	write_buf[cmd_offset++] = data_id;

	rc = pt_hid_send_output_and_wait_(cd, &hid_output);
	if (rc)
		return rc;

	status = cd->response_buf[5];
	if (status)
		return -EINVAL;

	read_data_id = cd->response_buf[6];
	if (read_data_id != data_id)
		return -EPROTO;

	size = get_unaligned_le16(&cd->response_buf[0]);
	*actual_read_len = get_unaligned_le16(&cd->response_buf[7]);
	*config = cd->response_buf[9];

	data_elem_size = *config & 0x07;
	data_size = *actual_read_len * data_elem_size;

	if (read_buf)
		memcpy(read_buf, &cd->response_buf[10], data_size);
	if (response)
		memcpy(response, cd->response_buf, size);
	return rc;
}

/*******************************************************************************
 * FUNCTION: pt_hid_output_retrieve_panel_scan
 *
 * SUMMARY: Protected call to pt_hid_output_retrieve_panel_scan_ by exclusive
 *  access to the DUT.
 *
 * RETURN:
 *    0 = success
 *   !0 = failure
 *
 * PARAMETERS:
 *  *dev             - pointer to device structure
 *   protect         - flag to call protected or non-protected
 *   read_offset     - read pointer offset
 *   read_count      - length of data to read
 *   data_id         - enumerated test ID to read selftest results from
 *  *response        - pointer to store the read response status
 *  *config          - pointer to store config data
 *  *actual_read_len - pointer to store data length actually read
 *  *read_buf        - pointer to the read buffer
 ******************************************************************************/
static int pt_hid_output_retrieve_panel_scan(
		struct pt_core_data *cd, u16 read_offset, u16 read_count,
		u8 data_id, u8 *response, u8 *config, u16 *actual_read_len,
		u8 *read_buf)
{
	int rc;

	rc = request_exclusive(cd, cd->dev, PT_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		pt_debug(cd->dev, DL_ERROR,
				"%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->dev);
		return rc;
	}

	rc = pt_hid_output_retrieve_panel_scan_(cd, read_offset,
			read_count, data_id, response, config,
			actual_read_len, read_buf);

	if (release_exclusive(cd, cd->dev) < 0)
		pt_debug(cd->dev, DL_ERROR,
				"%s: fail to release exclusive\n", __func__);

	return rc;
}

/*******************************************************************************
 * FUNCTION: _pt_request_pip_retrieve_panel_scan
 *
 * SUMMARY: Function pointer included in core_nonhid_cmd struct for external
 *	calls to the protected or unprotected call to
 *	pt_hid_output_retrieve_panel_scan
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*dev             - pointer to device structure
 *	 protect         - flag to call protected or non-protected
 *	 read_offset     - read pointer offset
 *	 read_count      - length of data to read
 *	 data_id         - enumerated test ID to read selftest results from
 *	*response        - pointer to store the read response status
 *	*config          - pointer to store config data
 *	*actual_read_len - pointer to store data length actually read
 *	*read_buf        - pointer to the read buffer
 ******************************************************************************/
static int _pt_request_pip_retrieve_panel_scan(struct device *dev,
		int protect, u16 read_offset, u16 read_count, u8 data_id,
		u8 *response, u8 *config, u16 *actual_read_len, u8 *read_buf)
{
	struct pt_core_data *cd = dev_get_drvdata(dev);

	if (protect)
		return pt_hid_output_retrieve_panel_scan(cd,
				read_offset, read_count, data_id, response,
				config, actual_read_len, read_buf);

	return pt_hid_output_retrieve_panel_scan_(cd,
			read_offset, read_count, data_id, response,
			config, actual_read_len, read_buf);
}

/*******************************************************************************
 * FUNCTION: pt_hid_output_user_cmd_
 *
 * SUMMARY: Load the write buffer into a HID structure and send it as a HID cmd
 *	to the DUT waiting for the response and loading it into the read buffer
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*cd              - pointer to core data
 *	 read_len        - expected read length of the response
 *	*read_buf        - pointer to where the response will be loaded
 *	 write_len       - length of the write buffer
 *	*write_buf       - pointer to the write buffer
 *	*actual_read_len - pointer to the actual amount of data read back
 ******************************************************************************/
static int pt_hid_output_user_cmd_(struct pt_core_data *cd,
		u16 read_len, u8 *read_buf, u16 write_len, u8 *write_buf,
		u16 *actual_read_len)
{
	int rc;
	u16 size;
#ifdef TTHE_TUNER_SUPPORT
	int command_code = 0;
	int len;
#endif
	struct pt_hid_output hid_output = {
		.length = write_len,
		.write_buf = write_buf,
	};

	rc = pt_hid_send_output_user_and_wait_(cd, &hid_output);
	if (rc)
		return rc;

	/* Get the response size from the first 2 bytes in the response */
	size = get_unaligned_le16(&cd->response_buf[0]);

	/* Ensure size is not greater than max buffer size */
	if (size > PT_MAX_PRBUF_SIZE - PIP2_LEN_FIELD_SIZE)
		size = PT_MAX_PRBUF_SIZE - PIP2_LEN_FIELD_SIZE;

	/* Minimum size to read is the 2 byte len field */
	if (size == 0)
		size = 2;

	if (size > read_len) {
		*actual_read_len = 0;
		return -EINVAL;
	}

	memcpy(read_buf, cd->response_buf, size);
	*actual_read_len = size;

#ifdef TTHE_TUNER_SUPPORT
	/* print up to cmd code */
	len = HID_OUTPUT_CMD_OFFSET + 1;
	if (write_len < len)
		len = write_len;
	else
		command_code = write_buf[HID_OUTPUT_CMD_OFFSET]
			& HID_OUTPUT_CMD_MASK;

	/* Do not print for EXEC_PANEL_SCAN & RETRIEVE_PANEL_SCAN commands */
	if (command_code != HID_OUTPUT_EXEC_PANEL_SCAN
			&& command_code != HID_OUTPUT_RETRIEVE_PANEL_SCAN)
		tthe_print(cd, write_buf, len, "CMD=");
#endif

	return 0;
}

/*******************************************************************************
 * FUNCTION: _pt_get_scan_state
 *
 * SUMMARY: Determine if the FW is actively scanning or not. There is no PIP
 * command that will directly provide this information but any PIP command
 * above 0x1F requires scanning to be disabled before it will be operational.
 * If scanning was not disabled before sending these PIP commands the FW will
 * respond with a 6 byte error response. So to safely determine the scanning
 * state, a PIP message that will not affect the operation of the FW was chosen:
 * "Verify Data Block CRC (ID 0x20)" is sent and if a 6 byte error code is
 * received scanning is active.
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *      *cd    - pointer to core data
 *      *state  - pointer to scan state
 ******************************************************************************/
static int _pt_get_scan_state(struct pt_core_data *cd, u8 *state)
{
	int write_length = 1;
	int report_length;
	u8 param[1] = { PT_TCH_PARM_EBID };
	struct pt_hid_output hid_output = {
		HID_OUTPUT_APP_COMMAND(HID_OUTPUT_VERIFY_CONFIG_BLOCK_CRC),
		.write_length = write_length,
		.write_buf = param,
		.novalidate = true,
	};
	int rc = 0;

	*state = PT_SCAN_STATE_UNKNOWN;
	if (cd->mode != PT_MODE_OPERATIONAL) {
		*state = PT_SCAN_STATE_UNKNOWN;
		return rc;
	}

	rc = pt_hid_send_output_and_wait_(cd, &hid_output);
	if (rc) {
		*state = PT_SCAN_STATE_UNKNOWN;
		return rc;
	}

	report_length = (cd->response_buf[1] << 8) | (cd->response_buf[0]);
	if ((report_length == 0x06) &&
		((cd->response_buf[4] & 0x7F) == 0x00) &&
		(cd->response_buf[5] == HID_OUTPUT_VERIFY_CONFIG_BLOCK_CRC)) {
		*state = PT_SCAN_STATE_ACTIVE;
	} else if ((report_length == 0x0A) &&
				((cd->response_buf[4]&0x7F) ==
				HID_OUTPUT_VERIFY_CONFIG_BLOCK_CRC)) {
		*state = PT_SCAN_STATE_INACTIVE;
	}

	pt_debug(cd->dev, DL_INFO, "%s: State = %d\n", __func__, *state);
	return rc;
}

/*******************************************************************************
 * FUNCTION: pt_get_scan_state
 *
 * SUMMARY: Protected call to _pt_get_scan_state() to determine if FW scanning
 * is active or not.
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *      *cd     - pointer to core data
 *      *state  - pointer to scan state
 ******************************************************************************/
static int pt_get_scan_state(struct pt_core_data *cd, u8 *state)
{
	int rc;

	rc = request_exclusive(cd, cd->dev, PT_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		pt_debug(cd->dev, DL_ERROR,
				"%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->dev);
		return rc;
	}

	rc = _pt_get_scan_state(cd, state);

	if (release_exclusive(cd, cd->dev) < 0)
		pt_debug(cd->dev, DL_ERROR,
				"%s: fail to release exclusive\n", __func__);

	return rc;
}

/*******************************************************************************
 * FUNCTION: _pt_request_get_scan_state
 *
 * SUMMARY: Function pointer included in core_cmds to allow other modules
 *	to requst to get scan state
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *      *dev      - pointer to device structure
 *	 protect  - flag to call protected or non-protected
 *      *state    - pointer to scan state
 ******************************************************************************/
static int _pt_request_get_scan_state(struct device *dev, int protect,
		u8 *state)
{
	struct pt_core_data *cd = dev_get_drvdata(dev);

	if (protect)
		return pt_get_scan_state(cd, state);

	return _pt_get_scan_state(cd, state);
}

/*******************************************************************************
 * FUNCTION: pt_get_config_ver_
 *
 * SUMMARY: Retrieve the config version from the DUT
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*cd - pointer to core data
 ******************************************************************************/
static int pt_get_config_ver_(struct pt_core_data *cd)
{
	struct pt_sysinfo *si = &cd->sysinfo;
	int rc;
	u16 config_ver = 0;
	u8 scan_state = PT_SCAN_STATE_UNKNOWN;
	u8 retry = 3;

	rc = pt_pip_suspend_scanning_(cd);
	if (rc)
		goto error;

	rc = pt_hid_output_read_conf_ver_(cd, &config_ver);
	if (rc)
		goto exit;

	si->ttdata.fw_ver_conf = config_ver;

exit:
	pt_pip_resume_scanning_(cd);
error:
	/* Check scan state,try to fix if it is not right */
	if (retry--) {
		_pt_get_scan_state(cd, &scan_state);
		if (scan_state != PT_SCAN_STATE_ACTIVE) {
			pt_debug(cd->dev, DL_ERROR,
				"%s: scan state: %d, retry: %d, rc = %d\n",
				__func__, scan_state, retry, rc);
			goto exit;
		}
	}
	pt_debug(cd->dev, DL_ERROR, "%s: CONFIG_VER:%04X\n",
		__func__, config_ver);
	return rc;
}

/*******************************************************************************
 * FUNCTION: pt_hid_output_user_cmd
 *
 * SUMMARY: Protected call to pt_hid_output_user_cmd_ by exclusive access to
 *  the DUT.
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*cd              - pointer to core data
 *	 read_len        - length of data to read
 *	*read_buf        - pointer to store read data
 *	 write_len       - length of data to write
 *	*write_buf       - pointer to buffer to write
 *	*actual_read_len - pointer to store data length actually read
 ******************************************************************************/
static int pt_hid_output_user_cmd(struct pt_core_data *cd,
		u16 read_len, u8 *read_buf, u16 write_len, u8 *write_buf,
		u16 *actual_read_len)
{
	int rc;

	rc = request_exclusive(cd, cd->dev, PT_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		pt_debug(cd->dev, DL_ERROR,
			"%s: fail get exclusive ex=%p own=%p\n",
			__func__, cd->exclusive_dev, cd->dev);
		return rc;
	}

	rc = pt_hid_output_user_cmd_(cd, read_len, read_buf,
			write_len, write_buf, actual_read_len);

	if (release_exclusive(cd, cd->dev) < 0)
		pt_debug(cd->dev, DL_ERROR,
			"%s: fail to release exclusive\n", __func__);

	return rc;
}

/*******************************************************************************
 * FUNCTION: _pt_request_pip_user_cmd
 *
 * SUMMARY: Function pointer included in core_nonhid_cmd struct for external
 *	calls to the protected or unprotected call to
 *	pt_hid_output_user_cmd
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*dev             - pointer to device structure
 *	 protect         - flag to call protected or non-protected
 *	 read_len        - length of data to read
 *	*read_buf        - pointer to store read data
 *	 write_len       - length of data to write
 *	*write_buf       - pointer to buffer to write
 *	*actual_read_len - pointer to store data length actually read
 ******************************************************************************/
static int _pt_request_pip_user_cmd(struct device *dev,
		int protect, u16 read_len, u8 *read_buf, u16 write_len,
		u8 *write_buf, u16 *actual_read_len)
{
	struct pt_core_data *cd = dev_get_drvdata(dev);

	if (protect)
		return pt_hid_output_user_cmd(cd, read_len, read_buf,
				write_len, write_buf, actual_read_len);

	return pt_hid_output_user_cmd_(cd, read_len, read_buf,
			write_len, write_buf, actual_read_len);
}

/*******************************************************************************
 * FUNCTION: pt_hid_output_bl_get_information_
 *
 * SUMMARY: Sends the PIP "Get Bootloader Information" (0x38) command to the
 *  DUT to retrieve bootloader version and chip identification information.
 *
 * RETURN:
 *   0 = success
 *  !0 = failure
 *
 * PARAMETERS:
 *  *cd           - pointer to core data
 *  *return_data  - pointer to store the return data
 *****************************************************************************/
static int pt_hid_output_bl_get_information_(struct pt_core_data *cd,
		u8 *return_data)
{
	int rc;
	int data_len;
	struct pt_hid_output hid_output = {
		HID_OUTPUT_BL_COMMAND(HID_OUTPUT_BL_GET_INFO),
	};

	rc = pt_hid_send_output_and_wait_(cd, &hid_output);
	if (rc)
		return rc;

	data_len = get_unaligned_le16(&cd->input_buf[6]);
	if (!data_len)
		return -EPROTO;

	memcpy(return_data, &cd->response_buf[8], data_len);

	return 0;
}

/*******************************************************************************
 * FUNCTION: pt_hid_output_bl_get_information
 *
 * SUMMARY: Protected call to pt_hid_output_bl_get_information_ by exclusive
 *  access to the DUT.
 *
 * RETURN:
 *   0 = success
 *  !0 = failure
 *
 * PARAMETERS:
 *  *cd           - pointer to core data
 *  *return_data  - pointer to store the return data
 ******************************************************************************/
static int pt_hid_output_bl_get_information(struct pt_core_data *cd,
		u8 *return_data)
{
	int rc;

	rc = request_exclusive(cd, cd->dev, PT_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		pt_debug(cd->dev, DL_ERROR,
			"%s: fail get exclusive ex=%p own=%p\n",
			__func__, cd->exclusive_dev, cd->dev);
		return rc;
	}

	rc = pt_hid_output_bl_get_information_(cd, return_data);

	if (release_exclusive(cd, cd->dev) < 0)
		pt_debug(cd->dev, DL_ERROR,
			"%s: fail to release exclusive\n", __func__);

	return rc;
}

/*******************************************************************************
 * FUNCTION: _pt_request_pip_bl_get_information
 *
 * SUMMARY: Function pointer included in core_nonhid_cmd struct for external
 *	calls to the protected or unprotected call to
 *	pt_hid_output_bl_get_information
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*dev         - pointer to device structure
 *	 protect     - flag to call protected or non-protected
 *	*return_data - pointer to store bl data
 ******************************************************************************/
static int _pt_request_pip_bl_get_information(struct device *dev,
		int protect, u8 *return_data)
{
	struct pt_core_data *cd = dev_get_drvdata(dev);

	if (protect)
		return pt_hid_output_bl_get_information(cd, return_data);

	return pt_hid_output_bl_get_information_(cd, return_data);
}

/*******************************************************************************
 * FUNCTION: pt_hid_output_bl_initiate_bl_
 *
 * SUMMARY: Sends the PIP "Get Bootloader Information" (0x48) command to the
 *  DUT to erases the entire TrueTouch application, Configuration Data block,
 *  and Design Data block in flash and enables the host to execute the Program
 *  and Verify Row command to bootload the application image and data.
 *
 * RETURN:
 *   0 = success
 *  !0 = failure
 *
 * PARAMETERS:
 *  *cd               - pointer to core data
 *   protect          - flag to call protected or non-protected
 *   key_size         - size of key
 *  *key_buf          - pointer to key data to allow operation
 *   row_size         - size of the meta data row
 *  *metadata_row_buf - pointer to meta data to write
 ******************************************************************************/
static int pt_hid_output_bl_initiate_bl_(struct pt_core_data *cd,
		u16 key_size, u8 *key_buf, u16 row_size, u8 *metadata_row_buf)
{
	u16 write_length = key_size + row_size;
	u8 *write_buf;
	int rc;
	struct pt_hid_output hid_output = {
		HID_OUTPUT_BL_COMMAND(HID_OUTPUT_BL_INITIATE_BL),
		.write_length = write_length,
		.timeout_ms = PT_HID_OUTPUT_BL_INITIATE_BL_TIMEOUT,
	};

	write_buf = kzalloc(write_length, GFP_KERNEL);
	if (!write_buf)
		return -ENOMEM;

	hid_output.write_buf = write_buf;

	if (key_size)
		memcpy(write_buf, key_buf, key_size);

	if (row_size)
		memcpy(&write_buf[key_size], metadata_row_buf, row_size);

	rc =  pt_hid_send_output_and_wait_(cd, &hid_output);

	kfree(write_buf);
	return rc;
}

/*******************************************************************************
 * FUNCTION: pt_hid_output_bl_initiate_bl
 *
 * SUMMARY: Protected call to pt_hid_output_bl_initiate_bl_ by exclusive
 *  access to the DUT.
 *
 * RETURN:
 *   0 = success
 *  !0 = failure
 *
 * PARAMETERS:
 *  *cd               - pointer to core data
 *   key_size         - size of key
 *  *key_buf          - pointer to key data to allow operation
 *   row_size         - size of the meta data row
 *  *metadata_row_buf - pointer to meta data to write
 ******************************************************************************/
static int pt_hid_output_bl_initiate_bl(struct pt_core_data *cd,
		u16 key_size, u8 *key_buf, u16 row_size, u8 *metadata_row_buf)
{
	int rc;

	rc = request_exclusive(cd, cd->dev, PT_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		pt_debug(cd->dev, DL_ERROR,
			"%s: fail get exclusive ex=%p own=%p\n",
			__func__, cd->exclusive_dev, cd->dev);
		return rc;
	}

	rc = pt_hid_output_bl_initiate_bl_(cd, key_size, key_buf,
			row_size, metadata_row_buf);

	if (release_exclusive(cd, cd->dev) < 0)
		pt_debug(cd->dev, DL_ERROR,
			"%s: fail to release exclusive\n", __func__);

	return rc;
}

/*******************************************************************************
 * FUNCTION: _pt_request_pip_bl_initiate_bl
 *
 * SUMMARY: Function pointer included in core_nonhid_cmd struct for external
 *	calls to the protected or unprotected call to
 *	pt_hid_output_bl_initiate_bl
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*dev              - pointer to device structure
 *	 protect          - flag to call protected or non-protected
 *	 key_size         - size of key
 *	*key_buf          - pointer to key data to allow operation
 *	 row_size         - size of the meta data row
 *	*metadata_row_buf - pointer to meta data to write
 ******************************************************************************/
static int _pt_request_pip_bl_initiate_bl(struct device *dev,
		int protect, u16 key_size, u8 *key_buf, u16 row_size,
		u8 *metadata_row_buf)
{
	struct pt_core_data *cd = dev_get_drvdata(dev);

	if (protect)
		return pt_hid_output_bl_initiate_bl(cd, key_size, key_buf,
				row_size, metadata_row_buf);

	return pt_hid_output_bl_initiate_bl_(cd, key_size, key_buf,
			row_size, metadata_row_buf);
}

/*******************************************************************************
 * FUNCTION: pt_hid_output_bl_program_and_verify_
 *
 * SUMMARY: Sends the PIP "Get Bootloader Information" (0x39) command to upload
 *  and program a 128-byte row into the flash, and then verifies written data.
 *
 * RETURN:
 *   0 = success
 *  !0 = failure
 *
 * PARAMETERS:
 *  *cd       - pointer to core data
 *   data_len - length of data_buf
 *  *data_buf - firmware image to program
 ******************************************************************************/
static int pt_hid_output_bl_program_and_verify_(
		struct pt_core_data *cd, u16 data_len, u8 *data_buf)
{
	struct pt_hid_output hid_output = {
		HID_OUTPUT_BL_COMMAND(HID_OUTPUT_BL_PROGRAM_AND_VERIFY),
		.write_length = data_len,
		.write_buf = data_buf,
		.timeout_ms = PT_HID_OUTPUT_BL_PROGRAM_AND_VERIFY_TIMEOUT,
	};

	return pt_hid_send_output_and_wait_(cd, &hid_output);
}

/*******************************************************************************
 * FUNCTION: pt_hid_output_bl_program_and_verify
 *
 * SUMMARY: Protected call to pt_hid_output_bl_program_and_verify_ by exclusive
 *  access to the DUT.
 *
 * RETURN:
 *   0 = success
 *  !0 = failure
 *
 * PARAMETERS:
 *  *cd       - pointer to core data
 *   data_len - length of data_buf
 *  *data_buf - firmware image to program
 ******************************************************************************/
static int pt_hid_output_bl_program_and_verify(
		struct pt_core_data *cd, u16 data_len, u8 *data_buf)
{
	int rc;

	rc = request_exclusive(cd, cd->dev, PT_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		pt_debug(cd->dev, DL_ERROR,
			"%s: fail get exclusive ex=%p own=%p\n",
			__func__, cd->exclusive_dev, cd->dev);
		return rc;
	}

	rc = pt_hid_output_bl_program_and_verify_(cd, data_len, data_buf);

	if (release_exclusive(cd, cd->dev) < 0)
		pt_debug(cd->dev, DL_ERROR,
			"%s: fail to release exclusive\n", __func__);

	return rc;
}

/*******************************************************************************
 * FUNCTION: _pt_request_pip_bl_program_and_verify
 *
 * SUMMARY: Function pointer included in core_nonhid_cmds to allow other modules
 *	to request to have the BL program and verify a FW image
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *      *dev      - pointer to device structure
 *	 protect  - boolean to determine to call the protected function
 *	 data_len - length of data_buf
 *	*data_buf - firmware image to program
 ******************************************************************************/
static int _pt_request_pip_bl_program_and_verify(
		struct device *dev, int protect, u16 data_len, u8 *data_buf)
{
	struct pt_core_data *cd = dev_get_drvdata(dev);

	if (protect)
		return pt_hid_output_bl_program_and_verify(cd, data_len,
				data_buf);

	return pt_hid_output_bl_program_and_verify_(cd, data_len,
			data_buf);
}

/*******************************************************************************
 * FUNCTION: pt_hid_output_bl_verify_app_integrity_
 *
 * SUMMARY: Sends the PIP "Get Bootloader Information" (0x31) command to
 *  perform a full verification of the application integrity by calculating the
 *  CRC of the image in flash and compare it to the expected CRC stored in the
 *  Metadata row.
 *
 * RETURN:
 *   0 = success
 *  !0 = failure
 *
 * PARAMETERS:
 *  *cd      - pointer to core data
 *  *result  - pointer to store result
 ******************************************************************************/
static int pt_hid_output_bl_verify_app_integrity_(
		struct pt_core_data *cd, u8 *result)
{
	int rc;
	struct pt_hid_output hid_output = {
		HID_OUTPUT_BL_COMMAND(HID_OUTPUT_BL_VERIFY_APP_INTEGRITY),
	};

	rc = pt_hid_send_output_and_wait_(cd, &hid_output);
	if (rc) {
		*result = 0;
		return rc;
	}

	*result = cd->response_buf[8];
	return 0;
}

/*******************************************************************************
 * FUNCTION: pt_hid_output_bl_verify_app_integrity
 *
 * SUMMARY: Protected call to pt_hid_output_bl_verify_app_integrity_ by
 *  exclusive access to the DUT.
 *
 * RETURN:
 *   0 = success
 *  !0 = failure
 *
 * PARAMETERS:
 *  *cd      - pointer to core data
 *  *result  - pointer to store result
 ******************************************************************************/
static int pt_hid_output_bl_verify_app_integrity(
		struct pt_core_data *cd, u8 *result)
{
	int rc;

	rc = request_exclusive(cd, cd->dev, PT_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		pt_debug(cd->dev, DL_ERROR,
			"%s: fail get exclusive ex=%p own=%p\n",
			__func__, cd->exclusive_dev, cd->dev);
		return rc;
	}

	rc = pt_hid_output_bl_verify_app_integrity_(cd, result);

	if (release_exclusive(cd, cd->dev) < 0)
		pt_debug(cd->dev, DL_ERROR,
			"%s: fail to release exclusive\n", __func__);

	return rc;
}

/*******************************************************************************
 * FUNCTION: _pt_request_pip_bl_verify_app_integrity
 *
 * SUMMARY: Function pointer included in core_nonhid_cmds to allow other modules
 *	to request to have the BL verify the application integrity (PIP1.x only)
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *      *dev     - pointer to device structure
 *	 protect - boolean to determine to call the protected function
 *	*result  - pointer to store result
 ******************************************************************************/
static int _pt_request_pip_bl_verify_app_integrity(
		struct device *dev, int protect, u8 *result)
{
	struct pt_core_data *cd = dev_get_drvdata(dev);

	if (protect)
		return pt_hid_output_bl_verify_app_integrity(cd, result);

	return pt_hid_output_bl_verify_app_integrity_(cd, result);
}

/*******************************************************************************
 * FUNCTION: pt_hid_output_bl_launch_app_
 *
 * SUMMARY: Sends the PIP "Launch Application" (0x3B) command to launch the
 *  application from bootloader (PIP1.x only).
 *
 * RETURN:
 *   0 = success
 *  !0 = failure
 *
 * PARAMETERS:
 *  *cd  - pointer to core data
 ******************************************************************************/
static int pt_hid_output_bl_launch_app_(struct pt_core_data *cd)
{
	struct pt_hid_output hid_output = {
		HID_OUTPUT_BL_COMMAND(HID_OUTPUT_BL_LAUNCH_APP),
		.reset_expected = 1,
	};

	return pt_hid_send_output_and_wait_(cd, &hid_output);
}

/*******************************************************************************
 * FUNCTION: pt_hid_output_bl_launch_app
 *
 * SUMMARY: Protected call to pt_hid_output_bl_launch_app_ by exclusive access
 *  to the DUT.
 *
 * RETURN:
 *   0 = success
 *  !0 = failure
 *
 * PARAMETERS:
 *  *cd  - pointer to core data
 ******************************************************************************/
static int pt_hid_output_bl_launch_app(struct pt_core_data *cd)
{
	int rc;

	rc = request_exclusive(cd, cd->dev, PT_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		pt_debug(cd->dev, DL_ERROR,
			"%s: fail get exclusive ex=%p own=%p\n",
			__func__, cd->exclusive_dev, cd->dev);
		return rc;
	}

	rc = pt_hid_output_bl_launch_app_(cd);

	if (release_exclusive(cd, cd->dev) < 0)
		pt_debug(cd->dev, DL_ERROR,
			"%s: fail to release exclusive\n", __func__);

	return rc;
}

/*******************************************************************************
 * FUNCTION: _pt_request_pip_launch_app
 *
 * SUMMARY: Function pointer included in core_nonhid_cmds to allow other modules
 *	to requst to have the BL launch the application. (PIP1.x only)
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *      *dev      - pointer to device structure
 *	 protect  - boolean to determine to call the protected function
 ******************************************************************************/
static int _pt_request_pip_launch_app(struct device *dev,
		int protect)
{
	struct pt_core_data *cd = dev_get_drvdata(dev);

	if (protect)
		return pt_hid_output_bl_launch_app(cd);

	return pt_hid_output_bl_launch_app_(cd);
}

/*******************************************************************************
 * FUNCTION: pt_hid_output_bl_get_panel_id_
 *
 * SUMMARY: Sends the PIP "Get Panel ID" (0x3E) command to return the Panel ID
 *  value store in the System Information.
 *
 * RETURN:
 *   0 = success
 *  !0 = failure
 *
 * PARAMETERS:
 *  *cd       - pointer to core data
 *  *panel_id - pointer to where the panel ID will be stored
 ******************************************************************************/
static int pt_hid_output_bl_get_panel_id_(
		struct pt_core_data *cd, u8 *panel_id)
{
	int rc;
	struct pt_hid_output hid_output = {
		HID_OUTPUT_BL_COMMAND(HID_OUTPUT_BL_GET_PANEL_ID),
	};

	rc = pt_hid_send_output_and_wait_(cd, &hid_output);
	if (rc == -EPROTO && cd->response_buf[5] == ERROR_COMMAND) {
		pt_debug(cd->dev, DL_ERROR,
			"%s: Get Panel ID command not supported\n",
			__func__);
		*panel_id = PANEL_ID_NOT_ENABLED;
		return 0;
	} else if (rc < 0) {
		pt_debug(cd->dev, DL_ERROR,
			"%s: Error on Get Panel ID command\n", __func__);
		return rc;
	}

	*panel_id = cd->response_buf[8];
	return 0;
}

/*******************************************************************************
 * FUNCTION: pt_hid_output_bl_get_panel_id
 *
 * SUMMARY: Protected call to pt_hid_output_bl_get_panel_id_ by exclusive access
 *  to the DUT.
 *
 * RETURN:
 *   0 = success
 *  !0 = failure
 *
 * PARAMETERS:
 *  *cd       - pointer to core data
 *  *panel_id - pointer to where the panel ID will be stored
 ******************************************************************************/
static int pt_hid_output_bl_get_panel_id(
		struct pt_core_data *cd, u8 *panel_id)
{
	int rc;

	rc = request_exclusive(cd, cd->dev, PT_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		pt_debug(cd->dev, DL_ERROR,
			"%s: fail get exclusive ex=%p own=%p\n",
			__func__, cd->exclusive_dev, cd->dev);
		return rc;
	}

	rc = pt_hid_output_bl_get_panel_id_(cd, panel_id);

	if (release_exclusive(cd, cd->dev) < 0)
		pt_debug(cd->dev, DL_ERROR,
			"%s: fail to release exclusive\n", __func__);

	return rc;
}

/*******************************************************************************
 * FUNCTION: _pt_request_pip2_get_panel_id
 *
 * SUMMARY: Function pointer included in core_nonhid_cmds to allow other modules
 *	to have the BL retrieve the panel ID
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*dev      - pointer to device structure
 *	 protect  - flag to run in protected mode
 *	*panel_id - pointer to where the panel ID will be stored
 ******************************************************************************/
static int _pt_request_pip_bl_get_panel_id(
		struct device *dev, int protect, u8 *panel_id)
{
	struct pt_core_data *cd = dev_get_drvdata(dev);

	if (protect)
		return pt_hid_output_bl_get_panel_id(cd, panel_id);

	return pt_hid_output_bl_get_panel_id_(cd, panel_id);
}

/*******************************************************************************
 * FUNCTION: _pt_request_pip2_send_cmd
 *
 * SUMMARY: Writes a PIP2 command packet to DUT, then waits for the
 *	interrupt and reads response data to read_buf
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*dev             - pointer to device structure
 *	 protect         - flag to run in protected mode
 *	*pip2_cmd        - pointer to PIP2 command to send
 *	 id              - ID of PIP command
 *	*data            - pointer to PIP data payload
 *	 report_body_len - report length
 *	*read_buf        - pointer to response buffer
 *	*actual_read_len - pointer to response buffer length
 ******************************************************************************/
static int _pt_request_pip2_send_cmd(struct device *dev,
	int protect, struct pip2_cmd_structure *pip2_cmd, u8 id, u8 *data,
	u8 report_body_len, u8 *read_buf, u16 *actual_read_len)
{
	struct pt_core_data *cd = dev_get_drvdata(dev);
	int rc = 0;
	int i = 0;
	int j = 0;
	u16 write_len;
	u8 *write_buf = NULL;
	u16 read_len;
	u8 response_seq;
	u8 extra_bytes;

	memset(pip2_cmd, 0, sizeof(struct pip2_cmd_structure));

	/*
	 * All PIP2 commands come through this function.
	 * Set flag for PIP2.x interface to allow response parsing to know
	 * how to decode the protocol header.
	 */
	cd->pip2_prot_active = 1;

	/* Hard coded register for PIP2.x */
	pip2_cmd->reg[0] = 0x01;
	pip2_cmd->reg[1] = 0x01;

	/*
	 * For PIP2.1+ the length field value includes itself:
	 * ADD 6: 2 (LEN) + 1 (SEQ) + 1 (REPORT ID) + 2 (CRC)
	 *
	 * The overall write length must include only the register:
	 * ADD 2: 2 (Register)
	 */
	extra_bytes = 6;
	write_len = 2;

	pip2_cmd->len  = report_body_len + extra_bytes;
	pip2_cmd->id   = id;
	pip2_cmd->seq  = pt_pip2_get_next_cmd_seq(cd);
	pip2_cmd->data = data;
	pt_pip2_cmd_calculate_crc(pip2_cmd, extra_bytes);

	/* Add the command length to the extra bytes based on PIP version */
	write_len += pip2_cmd->len;

	pt_debug(dev, DL_INFO, "%s Length Field: %d, Write Len: %d",
		__func__, pip2_cmd->len, write_len);

	write_buf = kzalloc(write_len, GFP_KERNEL);
	if (write_buf == NULL) {
		rc = -ENOMEM;
		goto exit;
	}
	if (pip2_cmd == NULL) {
		pt_debug(dev, DL_ERROR, "%s cmd is NULL\n", __func__);
		rc = -EINVAL;
		goto exit;
	}
	write_buf[i++] = pip2_cmd->reg[0];
	write_buf[i++] = pip2_cmd->reg[1];
	write_buf[i++] = pip2_cmd->len & 0xff;
	write_buf[i++] = (pip2_cmd->len & 0xff00) >> 8;
	write_buf[i++] = pip2_cmd->seq;
	write_buf[i++] = pip2_cmd->id;

	for (j = i; j < i + pip2_cmd->len - extra_bytes; j++)
		write_buf[j] = pip2_cmd->data[j-i];
	write_buf[j++] = pip2_cmd->crc[0];
	write_buf[j++] = pip2_cmd->crc[1];

	read_len = pt_pip2_get_cmd_response_len(pip2_cmd->id);
	if (read_len < 0)
		read_len = 255;
	pt_debug(dev, DL_INFO,
		"%s cmd_id[0x%02X] expected response length:%d ",
		__func__, pip2_cmd->id, read_len);

	if (protect == PT_CORE_CMD_PROTECTED)
		rc = pt_hid_output_user_cmd(cd, read_len, read_buf,
		write_len, write_buf, actual_read_len);
	else
		rc = pt_hid_output_user_cmd_(cd, read_len, read_buf,
		write_len, write_buf, actual_read_len);
	if (rc) {
		pt_debug(dev, DL_ERROR,
			"%s: nonhid_cmd->user_cmd() Error = %d\n",
			__func__, rc);
		goto exit;
	}

	/* Verify the SEQ number matches from command to response */
	response_seq = read_buf[PIP2_RESPONSE_SEQ_OFFSET];
	if ((pip2_cmd->seq & 0x07) != (response_seq & 0x07)) {
		pt_debug(dev, DL_ERROR,
			"%s cmd_id[0x%02x] send_seq = 0x%02x, response_seq = 0x%02x\n",
			__func__, pip2_cmd->id, pip2_cmd->seq, response_seq);
		rc = -EINVAL;
	}

exit:
	kfree(write_buf);
	cd->pip2_prot_active = 0;
	return rc;
}

/*******************************************************************************
 * FUNCTION: pt_get_hid_descriptor_
 *
 * SUMMARY: Send the get HID descriptor command to the DUT and load the response
 *	into the HID descriptor structure
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*cd   - pointer to core data
 *	*desc - pointer to the HID descriptor data read back from DUT
 ******************************************************************************/
static int pt_get_hid_descriptor_(struct pt_core_data *cd,
		struct pt_hid_desc *desc)
{
	struct device *dev = cd->dev;
	int rc;
	int t;
	u8 cmd[2];

	if (cd->dut_generation == DUT_PIP2_CAPABLE) {
		/*
		 * PT/TT devices no longer support the retrieval of the HID
		 * descriptor, so the values are hard coded.
		 */
		cd->hid_desc.packet_id            = 0xF7;
		cd->hid_desc.report_desc_len      = 230;
		cd->hid_desc.report_desc_register = 0x0002;
		cd->hid_desc.input_register       = 0x0003;
		cd->hid_desc.max_input_len        = 0x0100;
		cd->hid_desc.output_register      = 0x0004;
		cd->hid_desc.max_output_len       = 0x00FE;
		cd->hid_desc.command_register     = 0x0005;
		cd->hid_desc.data_register        = 0x0006;
		cd->hid_desc.vendor_id            = 0x04B4;
		cd->hid_desc.product_id           = 0xC101;
		rc = 0;
		goto exit;
	}

	/*
	 * During startup the HID descriptor is required for all future
	 * processing. If IRQ is already asserted due to an early touch report
	 * the report must be cleared before sending command.
	 */
	if (cd->cpdata->irq_stat &&
	    cd->cpdata->irq_stat(cd->cpdata, cd->dev) ==
	    PT_IRQ_ASSERTED_VALUE) {
		pt_flush_i2c(cd, 0);
		pt_debug(cd->dev, DL_INFO,
			"%s: IRQ asserted before write, forced clear\n",
			__func__);
	}

	/* Read HID descriptor length and version */
	mutex_lock(&cd->system_lock);
	cd->hid_cmd_state = 1;
	mutex_unlock(&cd->system_lock);

	/* Set HID descriptor register */
	memcpy(cmd, &cd->hid_core.hid_desc_register,
		sizeof(cd->hid_core.hid_desc_register));

	pt_debug(cd->dev, DL_INFO, ">>> %s: Write Buffer [%zu]",
		__func__, sizeof(cmd));
	pt_pr_buf(cd->dev, DL_DEBUG, cmd, sizeof(cmd), ">>> Get HID Desc");
	rc = pt_adap_write_read_specific(cd, 2, cmd, NULL);
	if (rc) {
		pt_debug(dev, DL_ERROR,
			"%s: failed to get HID descriptor, rc=%d\n",
			__func__, rc);
		goto error;
	}

	t = wait_event_timeout(cd->wait_q, (cd->hid_cmd_state == 0),
		msecs_to_jiffies(PT_HID_GET_HID_DESCRIPTOR_TIMEOUT));

	if (IS_TMO(t)) {
#ifdef TTDL_DIAGNOSTICS
		cd->i2c_transmission_error_count++;
		pt_toggle_err_gpio(cd);
#endif /* TTDL_DIAGNOSTICS */
		pt_debug(cd->dev, DL_ERROR,
			"%s: HID get descriptor timed out\n", __func__);
		rc = -ETIME;
		goto error;
	} else {
		cd->hw_detected = true;
	}

	/* Load the HID descriptor including all registers */
	memcpy((u8 *)desc, cd->response_buf, sizeof(struct pt_hid_desc));

	/* Check HID descriptor length and version */
	pt_debug(dev, DL_INFO, "%s: HID len:%X HID ver:%X\n", __func__,
		le16_to_cpu(desc->hid_desc_len),
		le16_to_cpu(desc->bcd_version));

	if (le16_to_cpu(desc->hid_desc_len) != sizeof(*desc) ||
		le16_to_cpu(desc->bcd_version) != PT_HID_VERSION) {
		pt_debug(dev, DL_ERROR, "%s: Unsupported HID version\n",
			__func__);
		return -ENODEV;
	}

	goto exit;

error:
	mutex_lock(&cd->system_lock);
	cd->hid_cmd_state = 0;
	mutex_unlock(&cd->system_lock);
exit:
	return rc;
}

/*******************************************************************************
 * FUNCTION: pt_get_hid_descriptor
 *
 * SUMMARY: Protected call to pt_get_hid_descriptor_()
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*cd   - pointer to core data
 *	*desc - pointer to the HID descriptor data read back from DUT
 ******************************************************************************/
static int pt_get_hid_descriptor(struct pt_core_data *cd,
		struct pt_hid_desc *desc)
{
	int rc;

	rc = request_exclusive(cd, cd->dev, PT_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		pt_debug(cd->dev, DL_ERROR,
			"%s: fail get exclusive ex=%p own=%p\n",
			__func__, cd->exclusive_dev, cd->dev);
		return rc;
	}

	rc = pt_get_hid_descriptor_(cd, desc);

	if (release_exclusive(cd, cd->dev) < 0)
		pt_debug(cd->dev, DL_ERROR,
			"%s: fail to release exclusive\n", __func__);

	return rc;
}

/*******************************************************************************
 * FUNCTION: _pt_request_get_hid_desc
 *
 * SUMMARY: Function pointer included in core_cmds to allow other modules
 *	to requst to get the HID descriptor
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *      *dev - pointer to device structure
 ******************************************************************************/
static int _pt_request_get_hid_desc(struct device *dev, int protect)
{
	struct pt_core_data *cd = dev_get_drvdata(dev);

	if (protect)
		return pt_get_hid_descriptor(cd, &cd->hid_desc);

	return pt_get_hid_descriptor_(cd, &cd->hid_desc);
}

/*******************************************************************************
 * FUNCTION: _pt_request_active_pip_protocol
 *
 * SUMMARY: Get active PIP protocol version using the PIP2 version command.
 *	Function will return PIP version of BL or application based on
 *	when it's called.
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*dev               - pointer to device structure
 *	 protect           - flag to run in protected mode
 *	*pip_version_major - pointer to store PIP major version
 *	*pip_version_minor - pointer to store PIP minor version
 ******************************************************************************/
int _pt_request_active_pip_protocol(struct device *dev, int protect,
	u8 *pip_version_major, u8 *pip_version_minor)
{
	u16 actual_read_len;
	u8 read_buf[256];
	struct pip2_cmd_structure pip2_cmd;
	struct pt_core_data *cd = dev_get_drvdata(dev);
	int rc;
	struct pt_hid_output sys_info = {
		HID_OUTPUT_APP_COMMAND(HID_OUTPUT_GET_SYSINFO),
		.timeout_ms = PT_HID_OUTPUT_GET_SYSINFO_TIMEOUT,
	};

	/* PATCH - sleep required as FW is not ready to handle PIP2 cmd */
	msleep(200);

	rc = _pt_request_pip2_send_cmd(dev, protect, &pip2_cmd,
		PIP2_CMD_ID_VERSION, NULL, 0, read_buf, &actual_read_len);
	if (!rc) {
		if ((read_buf[3] & 0x7F) == PIP2_CMD_ID_VERSION) {
			*pip_version_minor =
				read_buf[PIP2_RESPONSE_BODY_OFFSET];
			*pip_version_major =
				read_buf[PIP2_RESPONSE_BODY_OFFSET + 1];

			pt_debug(dev, DL_INFO,
				"%s: pip_version = 0x%02x.%02x\n", __func__,
				*pip_version_major, *pip_version_minor);
		} else
			pt_debug(dev, DL_ERROR,
				"%s: PIP2 Version cmd/resp mismatch, rc = %d\n",
				__func__, rc);
	} else {
		/*
		 * Legacy products do not support the pip2 protocol to get
		 * pip version. However, they do support the "get sysinfo"
		 * command to get pip version from FW, but the bootloader
		 * does not support it. This function will try "get sysinfo"
		 * command if the pip2 command failed but this cmd could also
		 * fail if DUT is stuck in bootloader mode.
		 */
		pt_debug(dev, DL_WARN,
			"%s: PIP2 no response rc = %d, try legacy cmd\n",
			__func__, rc);

		rc = pt_hid_send_output_and_wait_(cd, &sys_info);
		if (!rc) {
			*pip_version_minor =
				cd->response_buf[HID_SYSINFO_TTDATA_OFFSET+1];
			*pip_version_major =
				cd->response_buf[HID_SYSINFO_TTDATA_OFFSET];

			pt_debug(dev, DL_INFO,
				"%s: pip_version = 0x%02X.%02X\n", __func__,
				*pip_version_major, *pip_version_minor);
		} else {
			pt_debug(dev, DL_ERROR,
				"%s: pip_version Not Detected\n", __func__);
		}
	}

	return rc;
}
EXPORT_SYMBOL_GPL(_pt_request_active_pip_protocol);

/*******************************************************************************
 * FUNCTION: _pt_request_dut_generation
 *
 * SUMMARY: Determine the generation of device that we are communicating with:
 *		DUT_PIP1_ONLY (Gen5 or Gen6)
 *		DUT_PIP2_CAPABLE (TC33xx or TT7xxx)
 *	The HID_DESC command is supported in Gen5/6 BL and FW as well as
 *	TT/TC FW. The packet ID in the descriptor, however, is unique when
 *	coming form the BL or the FW:
 *		Packet_ID in BL = 0xFF
 *		Packet_ID in FW = 0xF7
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *      *dev     - pointer to device structure
 ******************************************************************************/
static int _pt_request_dut_generation(struct device *dev)
{
	int rc;
	u8 gen = DUT_UNKNOWN;
	u8 mode = PT_MODE_UNKNOWN;
	u8 read_buf[256];
	u16 actual_read_len;
	struct pt_core_data *cd = dev_get_drvdata(dev);
	struct pip2_cmd_structure pip2_cmd;

	mutex_lock(&cd->system_lock);
	cd->dut_generation = DUT_UNKNOWN;
	mutex_unlock(&cd->system_lock);

	rc = pt_get_hid_descriptor_(cd, &cd->hid_desc);
	if (!rc && cd->hid_desc.packet_id == 0xFF) {
		gen = DUT_PIP1_ONLY; /* Gen5/6 BL */
		mode = PT_MODE_BOOTLOADER;
		/* Special case - FW Sentinel also sent in BL */
		if (cd->startup_status & STARTUP_STATUS_FW_RESET_SENTINEL) {
			mutex_lock(&cd->system_lock);
			cd->startup_status = STARTUP_STATUS_BL_RESET_SENTINEL;
			mutex_unlock(&cd->system_lock);
		}
	} else if (!rc && cd->hid_desc.packet_id == 0xF7) {
		rc = _pt_request_pip2_send_cmd(dev, PT_CORE_CMD_UNPROTECTED,
			&pip2_cmd, PIP2_CMD_ID_VERSION, NULL, 0, read_buf,
			&actual_read_len);
		if (!rc) {
			gen = DUT_PIP2_CAPABLE; /* TT/TC FW */
			mode = PT_MODE_OPERATIONAL;
		} else {
			gen = DUT_PIP1_ONLY; /* Gen5/6 FW */
			mode = PT_MODE_OPERATIONAL;
		}
	} else if (rc) {
		rc = _pt_request_pip2_send_cmd(dev, PT_CORE_CMD_UNPROTECTED,
			&pip2_cmd, PIP2_CMD_ID_VERSION, NULL, 0, read_buf,
			&actual_read_len);
		if (!rc) {
			gen = DUT_PIP2_CAPABLE; /* TT/TC BL */
			mode = PT_MODE_BOOTLOADER;
		}
	}

	mutex_lock(&cd->system_lock);
	cd->dut_generation = gen;
	cd->mode = mode;
	mutex_unlock(&cd->system_lock);

#ifdef TTDL_DIAGNOSTICS
	pt_debug(cd->dev, DL_INFO, "%s: Generation=%d Mode=%d\n",
		__func__, cd->dut_generation, cd->mode);
#endif /* TTDL_DIAGNOSTICS */

	return cd->dut_generation;
}

/*******************************************************************************
 * FUNCTION: pt_start_wd_timer
 *
 * SUMMARY: Starts the TTDL watchdog timer if the timer interval is > 0
 *
 * RETURN: void
 *
 * PARAMETERS:
 *      *cd - pointer to core data
 ******************************************************************************/
static void pt_start_wd_timer(struct pt_core_data *cd)
{
	if (cd->watchdog_interval < 100) {
		pt_debug(cd->dev, DL_ERROR,
			"%s: WARNING: Invalid watchdog interval: %d\n",
			__func__, cd->watchdog_interval);
		return;
	}

	if (cd->watchdog_force_stop) {
		pt_debug(cd->dev, DL_INFO,
			"%s: TTDL WD Forced stop\n", __func__);
		return;
	}

	mod_timer(&cd->watchdog_timer, jiffies +
		msecs_to_jiffies(cd->watchdog_interval));
	cd->watchdog_enabled = 1;
	pt_debug(cd->dev, DL_INFO, "%s: TTDL WD Started\n", __func__);
}

/*******************************************************************************
 * FUNCTION: pt_stop_wd_timer
 *
 * SUMMARY: Stops the TTDL watchdog timer if the timer interval is > 0
 *
 * RETURN: void
 *
 * PARAMETERS:
 *      *cd - pointer to core data
 ******************************************************************************/
static void pt_stop_wd_timer(struct pt_core_data *cd)
{
	if (!cd->watchdog_interval)
		return;

	/*
	 * Ensure we wait until the watchdog timer
	 * running on a different CPU finishes
	 */
	del_timer_sync(&cd->watchdog_timer);
	cancel_work_sync(&cd->watchdog_work);
	del_timer_sync(&cd->watchdog_timer);
	cd->watchdog_enabled = 0;
	pt_debug(cd->dev, DL_INFO, "%s: TTDL WD Stopped\n", __func__);
}

/*******************************************************************************
 * FUNCTION: pt_hw_soft_reset
 *
 * SUMMARY: Sends a PIP reset command to the DUT. Disable/re-enable the
 *	TTDL watchdog around the reset to ensure the WD doesn't happen to
 *	schedule a restart if it fires when the DUT is being reset.
 *	This can cause a double reset.
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*cd      - pointer to core data struct
 *	 protect - flag to call protected or non-protected
 ******************************************************************************/
static int pt_hw_soft_reset(struct pt_core_data *cd, int protect)
{
	int rc;
	/* Don't stop watchdog if it already stopped */
	int wd_state = cd->watchdog_enabled;

	if (wd_state)
		pt_stop_wd_timer(cd);
	if (cd->hid_desc.hid_desc_len == 0) {
		rc = pt_get_hid_descriptor_(cd, &cd->hid_desc);
		if (rc < 0)
			return rc;
	}

	cd->startup_status = STARTUP_STATUS_START;
	if (protect)
		rc = pt_hid_cmd_reset(cd);
	else
		rc = pt_hid_cmd_reset_(cd);
	if (rc < 0) {
		pt_debug(cd->dev, DL_ERROR,
			"%s: FAILED to execute SOFT reset\n", __func__);
		return rc;
	}
	pt_debug(cd->dev, DL_WARN, "%s: SOFT reset successful\n",
		__func__);
	if (wd_state)
		pt_start_wd_timer(cd);
	return 0;
}

/*******************************************************************************
 * FUNCTION: pt_hw_hard_reset
 *
 * SUMMARY: Calls the platform xres function if it exists to perform a hard
 *	reset on the DUT by toggling the XRES gpio. Disable/re-enable the
 *	TTDL watchdog around the reset to ensure the WD doesn't happen to
 *	schedule a restart if it fires when the DUT is being reset.
 *	This can cause a double reset.
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*cd - pointer to core data struct
 ******************************************************************************/
static int pt_hw_hard_reset(struct pt_core_data *cd)
{
	/* Don't stop watchdog if it already stopped */
	int wd_state = cd->watchdog_enabled;

	if (wd_state)
		pt_stop_wd_timer(cd);
	if (cd->cpdata->xres) {
		pt_debug(cd->dev, DL_WARN,
			"%s: Reset Startup Status bitmask\n", __func__);
		cd->startup_status = STARTUP_STATUS_START;
		cd->cpdata->xres(cd->cpdata, cd->dev);
		pt_debug(cd->dev, DL_WARN, "%s: executed HARD reset\n",
			__func__);
		return 0;
	}
	pt_debug(cd->dev, DL_ERROR,
		"%s: FAILED to execute HARD reset\n", __func__);
	if (wd_state)
		pt_start_wd_timer(cd);
	return -ENODEV;
}

/*******************************************************************************
 * FUNCTION: pt_dut_reset
 *
 * SUMMARY: Attempts to reset the DUT by a hard reset and if that fails a
 *	soft reset.
 *
 * NOTE: "protect" flag is only used for soft reset.
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*cd      - pointer to core data structure
 *	 protect - flag to call protected or non-protected
 ******************************************************************************/
static int pt_dut_reset(struct pt_core_data *cd, int protect)
{
	int rc;

	pt_debug(cd->dev, DL_WARN, "%s: reset hw...\n", __func__);
	mutex_lock(&cd->system_lock);
	cd->hid_reset_cmd_state = 1;
	rc = pt_hw_hard_reset(cd);
	mutex_unlock(&cd->system_lock);

	if (rc == -ENODEV) {
		mutex_lock(&cd->system_lock);
		cd->hid_reset_cmd_state = 0;
		mutex_unlock(&cd->system_lock);
		pt_debug(cd->dev, DL_ERROR,
			"%s: Hard reset failed, try soft reset\n", __func__);
		rc = pt_hw_soft_reset(cd, protect);
	}

	if (rc)
		pt_debug(cd->dev, DL_ERROR, "%s: %s dev='%s' r=%d\n",
			__func__, "Fail hw reset", dev_name(cd->dev), rc);
	return rc;
}

/*******************************************************************************
 * FUNCTION: pt_dut_reset_and_wait
 *
 * SUMMARY: Wrapper function for pt_dut_reset that waits for the reset to
 *	complete
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *      *cd - pointer to core data structure
 ******************************************************************************/
static int pt_dut_reset_and_wait(struct pt_core_data *cd)
{
	int rc = 0;
	int t;

	if (cd->dut_generation == DUT_PIP1_ONLY) {
		rc = pt_dut_reset(cd, PT_CORE_CMD_UNPROTECTED);
		if (rc < 0)
			goto exit;

		t = wait_event_timeout(cd->wait_q,
			(cd->hid_reset_cmd_state == 0),
			msecs_to_jiffies(PT_HID_RESET_TIMEOUT));
		if (IS_TMO(t)) {
#ifdef TTDL_DIAGNOSTICS
			cd->i2c_transmission_error_count++;
			pt_toggle_err_gpio(cd);
#endif /* TTDL_DIAGNOSTICS */
			pt_debug(cd->dev, DL_ERROR, "%s: reset timed out\n",
				__func__);
			rc = -ETIME;
			goto exit;
		}
	}

exit:
	return rc;
}

/*
 * touch deafult parameters (from report descriptor) to resolve protocol for
 * touch report
 */
const struct pt_tch_abs_params tch_hdr_default[PT_TCH_NUM_HDR] = {
	/* byte offset, size, min, max, bit offset, report */
	{0x00, 0x02, 0x00, 0x10000, 0x00, 0x01},	/* SCAN TIME */
	{0x02, 0x01, 0x00, 0x20,    0x00, 0x01},	/* NUMBER OF RECORDS */
	{0x02, 0x01, 0x00, 0x02,    0x05, 0x01},	/* LARGE OBJECT */
	{0x03, 0x01, 0x00, 0x08,    0x00, 0x01},	/* NOISE EFFECT */
	{0x03, 0x01, 0x00, 0x04,    0x06, 0x01},	/* REPORT_COUNTER */
};

/*
 * button deafult parameters (from report descriptor) to resolve protocol for
 * button report
 */
const struct pt_tch_abs_params tch_abs_default[PT_TCH_NUM_ABS] = {
	/* byte offset, size, min, max, bit offset, report */
	{0x02, 0x02, 0x00, 0x10000, 0x00, 0x01},	/* X */
	{0x04, 0x02, 0x00, 0x10000, 0x00, 0x01},	/* Y */
	{0x06, 0x01, 0x00, 0x100,   0x00, 0x01},	/* P (Z) */
	{0x01, 0x01, 0x00, 0x20,    0x00, 0x01},	/* TOUCH ID */
	{0x01, 0x01, 0x00, 0x04,    0x05, 0x01},	/* EVENT ID */
	{0x00, 0x01, 0x00, 0x08,    0x00, 0x01},	/* OBJECT ID */
	{0x01, 0x01, 0x00, 0x02,    0x07, 0x01},	/* LIFTOFF */
	{0x07, 0x01, 0x00, 0x100,   0x00, 0x01},	/* TOUCH_MAJOR */
	{0x08, 0x01, 0x00, 0x100,   0x00, 0x01},	/* TOUCH_MINOR */
	{0x09, 0x01, 0x00, 0x100,   0x00, 0x01},	/* ORIENTATION */
};

/*******************************************************************************
 * FUNCTION: pt_init_pip_report_fields
 *
 * SUMMARY: Setup default values for touch/button report parsing.
 *
 * PARAMETERS:
 *  *cd  - pointer to core data structure
 ******************************************************************************/
static void pt_init_pip_report_fields(struct pt_core_data *cd)
{
	struct pt_sysinfo *si = &cd->sysinfo;

	memcpy(si->tch_hdr, tch_hdr_default, sizeof(tch_hdr_default));
	memcpy(si->tch_abs, tch_abs_default, sizeof(tch_abs_default));

	si->desc.tch_report_id = HID_TOUCH_REPORT_ID;
	si->desc.tch_record_size = TOUCH_REPORT_SIZE;
	si->desc.tch_header_size = TOUCH_INPUT_HEADER_SIZE;
	si->desc.btn_report_id = HID_BTN_REPORT_ID;

	cd->features.easywake = 1;
	cd->features.noise_metric = 1;
	cd->features.tracking_heatmap = 1;
	cd->features.sensor_data = 1;
}

/*******************************************************************************
 * FUNCTION: pt_get_mode
 *
 * SUMMARY: Determine the current mode from the contents of a HID descriptor
 *	message
 *
 * RETURN: Enum of the current mode
 *
 * PARAMETERS:
 *      *cd      - pointer to the Core Data structure
 *       protect - run command in protected mode
 *      *mode    - pointer to store the retrieved mode
 ******************************************************************************/
static int pt_get_mode(struct pt_core_data *cd,
		struct pt_hid_desc *desc)
{
	if (desc->packet_id == PT_HID_APP_REPORT_ID)
		return PT_MODE_OPERATIONAL;
	else if (desc->packet_id == PT_HID_BL_REPORT_ID)
		return PT_MODE_BOOTLOADER;

	return PT_MODE_UNKNOWN;
}

/*******************************************************************************
 * FUNCTION: _pt_request_get_mode
 *
 * SUMMARY: Function pointer included in core_cmds to allow other modules
 *	to determine the current mode of the DUT by use of the Get HID
 *	Descriptor command.
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *      *dev     - pointer to device structure
 *       protect - run command in protected mode
 *      *mode    - pointer to store the retrieved mode
 ******************************************************************************/
static int _pt_request_get_mode(struct device *dev, int protect, u8 *mode)
{
	struct pt_core_data *cd = dev_get_drvdata(dev);
	int rc;

	if (protect)
		rc = pt_get_hid_descriptor(cd, &cd->hid_desc);
	else
		rc = pt_get_hid_descriptor_(cd, &cd->hid_desc);

	if (rc)
		*mode = PT_MODE_UNKNOWN;
	else
		*mode = pt_get_mode(cd, &cd->hid_desc);

	return rc;
}

/*******************************************************************************
 * FUNCTION: pt_queue_startup_
 *
 * SUMMARY: Queues a TTDL startup by scheduling work with the startup_work()
 *	function.
 *
 * RETURN: void
 *
 * PARAMETERS:
 *      *cd - pointer to core data
 ******************************************************************************/
static void pt_queue_startup_(struct pt_core_data *cd)
{
	if (cd->startup_state == STARTUP_NONE) {
		cd->startup_state = STARTUP_QUEUED;

		schedule_work(&cd->startup_work);
		pt_debug(cd->dev, DL_INFO,
			"%s: pt_startup queued\n", __func__);
	} else {
		pt_debug(cd->dev, DL_INFO, "%s: startup_state = %d\n",
			__func__, cd->startup_state);
	}
}

/*******************************************************************************
 * FUNCTION: pt_queue_startup
 *
 * SUMMARY: Queues a TTDL starup within a mutex lock
 *
 * RETURN: void
 *
 * PARAMETERS:
 *      *cd - pointer to core data
 ******************************************************************************/
static void pt_queue_startup(struct pt_core_data *cd)
{
	mutex_lock(&cd->system_lock);
	pt_queue_startup_(cd);
	mutex_unlock(&cd->system_lock);
}

/*******************************************************************************
 * FUNCTION: call_atten_cb
 *
 * SUMMARY: Iterate over attention list call the function that registered.
 *
 * RETURN: void
 *
 * PARAMETERS:
 *  *cd    - pointer to core data
 *   type  - type of attention list
 *   mode  - condition for execution
 ******************************************************************************/
static void call_atten_cb(struct pt_core_data *cd,
		enum pt_atten_type type, int mode)
{
	struct atten_node *atten, *atten_n;

	pt_debug(cd->dev, DL_DEBUG, "%s: check list type=%d mode=%d\n",
		__func__, type, mode);
	spin_lock(&cd->spinlock);
	list_for_each_entry_safe(atten, atten_n,
			&cd->atten_list[type], node) {
		if (!mode || atten->mode & mode) {
			spin_unlock(&cd->spinlock);
			pt_debug(cd->dev, DL_DEBUG,
				"%s: attention for '%s'",
				__func__, dev_name(atten->dev));
			atten->func(atten->dev);
			spin_lock(&cd->spinlock);
		}
	}
	spin_unlock(&cd->spinlock);
}

/*******************************************************************************
 * FUNCTION: start_fw_upgrade
 *
 * SUMMARY: Calling "PT_ATTEN_LOADER" attention list that loader registered to
 *  start firmware upgrade.
 *
 * RETURN:
 *   0 = success
 *
 * PARAMETERS:
 *  *data  - pointer to core data
 ******************************************************************************/
static int start_fw_upgrade(void *data)
{
	struct pt_core_data *cd = (struct pt_core_data *)data;

	call_atten_cb(cd, PT_ATTEN_LOADER, 0);

	return 0;
}

/*******************************************************************************
 * FUNCTION: pt_put_device_into_easy_wakeup_
 *
 * SUMMARY: Call the enter_easywake_state function and set the device into easy
 *  wake up state.
 *
 * RETURN:
 *   0 = success
 *  !0 = failure
 *
 * PARAMETERS:
 *  *cd  - pointer to core data
 ******************************************************************************/
static int pt_put_device_into_easy_wakeup_(struct pt_core_data *cd)
{
	int rc;
	u8 status = 0;

	mutex_lock(&cd->system_lock);
	cd->wait_until_wake = 0;
	mutex_unlock(&cd->system_lock);

	rc = pt_hid_output_enter_easywake_state_(cd,
		cd->easy_wakeup_gesture, &status);
	if (rc || status == 0)
		return -EBUSY;

	return rc;
}

/*******************************************************************************
 * FUNCTION: pt_put_device_into_deep_sleep_
 *
 * SUMMARY: Call the set_power function and set the DUT to deep sleep
 *
 * RETURN:
 *	 0 = success
 *	!0 = error
 *
 * PARAMETERS:
 *      *cd - pointer to core data
 ******************************************************************************/
static int pt_put_device_into_deep_sleep_(struct pt_core_data *cd)
{
	int rc;

	rc = pt_hid_cmd_set_power_(cd, HID_POWER_SLEEP);
	if (rc)
		rc = -EBUSY;
	return rc;
}

/*******************************************************************************
 * FUNCTION: pt_put_device_into_sleep_
 *
 * SUMMARY: Wapper function to set device into deep sleep or easy wake up based
 *  on the configuration of easy_wakeup_gesture in the core data structure.
 *
 * RETURN:
 *   0 = success
 *  !0 = failure
 *
 * PARAMETERS:
 *  *cd  - pointer to core data
 ******************************************************************************/
static int pt_put_device_into_sleep_(struct pt_core_data *cd)
{
	int rc;

	if (IS_DEEP_SLEEP_CONFIGURED(cd->easy_wakeup_gesture))
		rc = pt_put_device_into_deep_sleep_(cd);
	else
		rc = pt_put_device_into_easy_wakeup_(cd);

	return rc;
}

/*******************************************************************************
 * FUNCTION: pt_core_poweroff_device_
 *
 * SUMMARY: Disable IRQ and HW power down the device.
 *
 * RETURN:
 *   0 = success
 *  !0 = failure
 *
 * PARAMETERS:
 *  *cd  - pointer to core data
 ******************************************************************************/
static int pt_core_poweroff_device_(struct pt_core_data *cd)
{
	int rc;

	if (cd->irq_enabled) {
		cd->irq_enabled = false;
		disable_irq_nosync(cd->irq);
	}

	rc = cd->cpdata->power(cd->cpdata, 0, cd->dev, 0);
	if (rc < 0)
		pt_debug(cd->dev, DL_ERROR, "%s: HW Power down fails r=%d\n",
			__func__, rc);
	return rc;
}

/*******************************************************************************
 * FUNCTION: pt_core_sleep_
 *
 * SUMMARY: Suspend the device with power off or deep sleep based on the
 *  configuration in the core platform data structure.
 *
 * RETURN:
 *   0 = success
 *  !0 = failure
 *
 * PARAMETERS:
 *  *cd  - pointer to core data
 ******************************************************************************/
static int pt_core_sleep_(struct pt_core_data *cd)
{
	int rc;

	mutex_lock(&cd->system_lock);
	if (cd->sleep_state == SS_SLEEP_OFF) {
		cd->sleep_state = SS_SLEEPING;
	} else {
		mutex_unlock(&cd->system_lock);
		return 1;
	}
	mutex_unlock(&cd->system_lock);

	/* Ensure watchdog and startup works stopped */
	pt_stop_wd_timer(cd);
	cancel_work_sync(&cd->startup_work);
	pt_stop_wd_timer(cd);

	if (cd->cpdata->flags & PT_CORE_FLAG_POWEROFF_ON_SLEEP)
		rc = pt_core_poweroff_device_(cd);
	else
		rc = pt_put_device_into_sleep_(cd);

	mutex_lock(&cd->system_lock);
	cd->sleep_state = SS_SLEEP_ON;
	mutex_unlock(&cd->system_lock);

	return rc;
}

/*******************************************************************************
 * FUNCTION: pt_core_sleep
 *
 * SUMMARY: Protected call to pt_core_sleep_ by exclusive access to the DUT.
 *
 * RETURN:
 *   0 = success
 *  !0 = failure
 *
 * PARAMETERS:
 *  *cd  - pointer to core data
 ******************************************************************************/
static int pt_core_sleep(struct pt_core_data *cd)
{
	int rc;

	rc = request_exclusive(cd, cd->dev, PT_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		pt_debug(cd->dev, DL_ERROR,
			"%s: fail get exclusive ex=%p own=%p\n",
			__func__, cd->exclusive_dev, cd->dev);
		return rc;
	}

	rc = pt_core_sleep_(cd);

	if (release_exclusive(cd, cd->dev) < 0)
		pt_debug(cd->dev, DL_ERROR,
			"%s: fail to release exclusive\n", __func__);
	else
		pt_debug(cd->dev, DL_DEBUG, "%s: pass release exclusive\n",
			__func__);

	return rc;
}

/*******************************************************************************
 * FUNCTION: pt_wakeup_host
 *
 * SUMMARY: Check wake up report and call the PT_ATTEN_WAKE attention list.
 *
 * NOTE: TSG5 EasyWake and TSG6 EasyWake use different protocol.
 *
 * RETURN:
 *   0 = success
 *  !0 = failure
 *
 * PARAMETERS:
 *  *cd  - pointer to core data
 ******************************************************************************/
static int pt_wakeup_host(struct pt_core_data *cd)
{
#ifndef EASYWAKE_TSG6
	/* TSG5 EasyWake */
	int rc = 0;
	int event_id;
	int size = get_unaligned_le16(&cd->input_buf[0]);

	/* Validate report */
	if (size != 4 || cd->input_buf[2] != 4)
		rc = -EINVAL;

	cd->wake_initiated_by_device = 1;
	event_id = cd->input_buf[3];

	pt_debug(cd->dev, DL_INFO, "%s: e=%d, rc=%d\n",
		__func__, event_id, rc);

	if (rc) {
		pt_core_sleep_(cd);
		goto exit;
	}

	/* attention WAKE */
	call_atten_cb(cd, PT_ATTEN_WAKE, 0);
exit:
	return rc;
#else
	/* TSG6 FW1.3 EasyWake */
	int rc = 0;
	int i = 0;
	int report_length;

	/* Validate report */
	if (cd->input_buf[2] != 4)
		rc = -EINVAL;

	cd->wake_initiated_by_device = 1;

	pt_debug(cd->dev, DL_INFO, "%s: rc=%d\n", __func__, rc);

	if (rc) {
		pt_core_sleep_(cd);
		goto exit;
	}

	/* Get gesture id and gesture data length */
	cd->gesture_id = cd->input_buf[3];
	report_length = (cd->input_buf[1] << 8) | (cd->input_buf[0]);
	cd->gesture_data_length = report_length - 4;

	pt_debug(cd->dev, DL_INFO,
		"%s: gesture_id = %d, gesture_data_length = %d\n",
		__func__, cd->gesture_id, cd->gesture_data_length);

	for (i = 0; i < cd->gesture_data_length; i++)
		cd->gesture_data[i] = cd->input_buf[4 + i];

	/* attention WAKE */
	call_atten_cb(cd, PT_ATTEN_WAKE, 0);
exit:
	return rc;
#endif
}

/*******************************************************************************
 * FUNCTION: pt_get_touch_axis
 *
 * SUMMARY: Function to calculate touch axis
 *
 * PARAMETERS:
 *     *cd      - pointer to core data structure
 *     *axis    - pointer to axis calculation result
 *      size    - size in bytes
 *      max     - max value of result
 *     *xy_data - pointer to input data to be parsed
 *      bofs    - bit offset
 ******************************************************************************/
static void pt_get_touch_axis(struct pt_core_data *cd,
	int *axis, int size, int max, u8 *data, int bofs)
{
	int nbyte;
	int next;

	for (nbyte = 0, *axis = 0, next = 0; nbyte < size; nbyte++) {
		*axis = *axis + ((data[next] >> bofs) << (nbyte * 8));
		next++;
	}

	*axis &= max - 1;
}

/*******************************************************************************
 * FUNCTION: move_tracking_heatmap_data
 *
 * SUMMARY: Move the valid tracking heatmap data from the input buffer into the
 *	system information structure, xy_mode and xy_data.
 *	- If TTHE_TUNER_SUPPORT is defined print the raw sensor data into
 *	the tthe_tuner sysfs node under the label "THM"
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*cd - pointer to core data
 *	*si - pointer to the system information structure
 ******************************************************************************/
static int move_tracking_heatmap_data(struct pt_core_data *cd,
	struct pt_sysinfo *si)
{
#ifdef TTHE_TUNER_SUPPORT
	int size = get_unaligned_le16(&cd->input_buf[0]);

	if (size)
		tthe_print(cd, cd->input_buf, size, "THM=");
#endif
	memcpy(si->xy_mode, cd->input_buf, SENSOR_HEADER_SIZE);
	return 0;
}

/*******************************************************************************
 * FUNCTION: move_sensor_data
 *
 * SUMMARY: Move the valid sensor data from the input buffer into the system
 *	system information structure, xy_mode and xy_data.
 *	- If TTHE_TUNER_SUPPORT is defined print the raw sensor data into
 *	the tthe_tuner sysfs node under the label "sensor_monitor"
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*cd - pointer to core data
 *	*si - pointer to the system information structure
 ******************************************************************************/
static int move_sensor_data(struct pt_core_data *cd,
	struct pt_sysinfo *si)
{
#ifdef TTHE_TUNER_SUPPORT
	int size = get_unaligned_le16(&cd->input_buf[0]);

	if (size)
		tthe_print(cd, cd->input_buf, size, "sensor_monitor=");
#endif
	memcpy(si->xy_mode, cd->input_buf, SENSOR_HEADER_SIZE);
	return 0;
}

/*******************************************************************************
 * FUNCTION: move_button_data
 *
 * SUMMARY: Move the valid button data from the input buffer into the system
 *	system information structure, xy_mode and xy_data.
 *	- If TTHE_TUNER_SUPPORT is defined print the raw button data into
 *	the tthe_tuner sysfs node under the label "OpModeData"
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*cd - pointer to core data
 *	*si - pointer to the system information structure
 ******************************************************************************/
static int move_button_data(struct pt_core_data *cd,
	struct pt_sysinfo *si)
{
#ifdef TTHE_TUNER_SUPPORT
	int size = get_unaligned_le16(&cd->input_buf[0]);

	if (size)
		tthe_print(cd, cd->input_buf, size, "OpModeData=");
#endif
	memcpy(si->xy_mode, cd->input_buf, BTN_INPUT_HEADER_SIZE);
	pt_pr_buf(cd->dev, DL_INFO, (u8 *)si->xy_mode, BTN_INPUT_HEADER_SIZE,
		"xy_mode");

	memcpy(si->xy_data, &cd->input_buf[BTN_INPUT_HEADER_SIZE],
		BTN_REPORT_SIZE);
	pt_pr_buf(cd->dev, DL_INFO, (u8 *)si->xy_data, BTN_REPORT_SIZE,
		"xy_data");
	return 0;
}

/*******************************************************************************
 * FUNCTION: move_touch_data
 *
 * SUMMARY: Move the valid touch data from the input buffer into the system
 *	system information structure, xy_mode and xy_data.
 *	- If TTHE_TUNER_SUPPORT is defined print the raw touch data into
 *	the tthe_tuner sysfs node under the label "OpModeData"
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*cd - pointer to core data
 *	*si - pointer to the system information structure
 ******************************************************************************/
static int move_touch_data(struct pt_core_data *cd,
	struct pt_sysinfo *si)
{
	int max_tch = si->sensing_conf_data.max_tch;
	int num_cur_tch;
	int length;
	struct pt_tch_abs_params *tch = &si->tch_hdr[PT_TCH_NUM];
#ifdef TTHE_TUNER_SUPPORT
	int size = get_unaligned_le16(&cd->input_buf[0]);

	if (size)
		tthe_print(cd, cd->input_buf, size, "OpModeData=");
#endif

	memcpy(si->xy_mode, cd->input_buf, si->desc.tch_header_size);
	pt_pr_buf(cd->dev, DL_INFO, (u8 *)si->xy_mode,
		si->desc.tch_header_size, "xy_mode");

	pt_get_touch_axis(cd, &num_cur_tch, tch->size,
			tch->max, si->xy_mode + 3 + tch->ofs, tch->bofs);
	if (unlikely(num_cur_tch > max_tch))
		num_cur_tch = max_tch;

	length = num_cur_tch * si->desc.tch_record_size;

	memcpy(si->xy_data, &cd->input_buf[si->desc.tch_header_size], length);
	pt_pr_buf(cd->dev, DL_INFO, (u8 *)si->xy_data, length, "xy_data");
	return 0;
}

/*******************************************************************************
 * FUNCTION: parse_touch_input
 *
 * SUMMARY: Parse the touch report and take action based on the touch
 *	report_id.
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*cd   - pointer to core data
 *	 size - size of touch record
 ******************************************************************************/
static int parse_touch_input(struct pt_core_data *cd, int size)
{
	struct pt_sysinfo *si = &cd->sysinfo;
	int report_id = cd->input_buf[2];
	int rc = -EINVAL;

	pt_debug(cd->dev, DL_DEBUG, "%s: Received touch report\n",
		__func__);
	if (!si->ready) {
		pt_debug(cd->dev, DL_ERROR,
			"%s: Need system information to parse touches\n",
			__func__);
		return 0;
	}

	if (!si->xy_mode || !si->xy_data)
		return rc;

	if (report_id == si->desc.tch_report_id)
		rc = move_touch_data(cd, si);
	else if (report_id == si->desc.btn_report_id)
		rc = move_button_data(cd, si);
	else if (report_id == HID_SENSOR_DATA_REPORT_ID)
		rc = move_sensor_data(cd, si);
	else if (report_id == HID_TRACKING_HEATMAP_REPOR_ID)
		rc = move_tracking_heatmap_data(cd, si);

	if (rc)
		return rc;

	/* attention IRQ */
	call_atten_cb(cd, PT_ATTEN_IRQ, cd->mode);

	return 0;
}

/*******************************************************************************
 * FUNCTION: parse_command_input
 *
 * SUMMARY: Move the response data from the input buffer to the response buffer
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*cd   - pointer to core data
 *	 size - size of response data
 ******************************************************************************/
static int parse_command_input(struct pt_core_data *cd, int size)
{
	pt_debug(cd->dev, DL_DEBUG, "%s: Received cmd interrupt\n",
		__func__);

	memcpy(cd->response_buf, cd->input_buf, size);

	mutex_lock(&cd->system_lock);
	cd->hid_cmd_state = 0;
	mutex_unlock(&cd->system_lock);
	wake_up(&cd->wait_q);

	return 0;
}

/*******************************************************************************
 * FUNCTION: pt_parse_input
 *
 * SUMMARY: Parse the input data read from DUT due to IRQ. Handle data based
 *	on if its a response to a command or asynchronous touch data / reset
 *	sentinel. PIP2.x messages have addional error checking that is
 *	parsed (SEQ match from cmd to rsp, CRC valid).
 *	Look for special packets based on unique lengths:
 *		0 bytes  - APP(FW) reset sentinel or Gen5/6 BL sentinel
 *		2 bytes  - Empty buffer (PIP 1.7 and earlier)
 *		11 bytes - possible PIP2.x reset sentinel (TAG and SEQ must = 0)
 *		0xFFXX   - Empty buffer (PIP 1.7+)
 *	Queue a startup after any asynchronous FW reset sentinel is seen, unless
 *	the inital probe has not yet been done.
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*cd - pointer to core data
 ******************************************************************************/
static int pt_parse_input(struct pt_core_data *cd)
{
	int report_id;
	int cmd_id;
	int is_command = 0;
	int size;
	int print_size;
	bool touch_report = true;
	unsigned short calc_crc;
	unsigned short resp_crc;

	size = get_unaligned_le16(&cd->input_buf[0]);
	print_size = size;
	pt_debug(cd->dev, DL_INFO, "<<< %s: IRQ Triggered, read len [%d]\n",
			__func__, print_size);
	pt_pr_buf(cd->dev, DL_DEBUG, cd->input_buf, print_size, "<<< Read buf");

	if (size == 0 ||
	   (size == 11 &&
	   (cd->input_buf[PIP2_RESPONSE_SEQ_OFFSET] & 0x0F) == 0 &&
	   (cd->input_buf[PIP2_RESPONSE_ID_OFFSET] & 0x7F) ==
	   PIP2_CMD_ID_STATUS)) {
		touch_report = false;
		cd->hw_detected = true;
		if (size == 0) {
			cd->startup_status = STARTUP_STATUS_FW_RESET_SENTINEL;
			cd->pip2_prot_active = false;
			pt_debug(cd->dev, DL_WARN,
				"%s: FW Sentinel - %s(%d) %s(%d)\n", __func__,
				"hid_cmd_state", cd->hid_cmd_state,
				"hid_reset_cmd_state", cd->hid_reset_cmd_state);

			if ((!cd->hid_reset_cmd_state) &&
			    (cd->core_probe_complete) &&
			    (cd->hid_cmd_state !=
			    HID_OUTPUT_START_BOOTLOADER + 1) &&
				(cd->hid_cmd_state !=
				HID_OUTPUT_BL_LAUNCH_APP + 1)) {
				pt_debug(cd->dev, DL_WARN,
					"%s: Queue Startup\n", __func__);
				pt_queue_startup(cd);
			}

		} else {
			/* Sentinel must be from TT/TC BL */
			cd->pip2_prot_active = 1;
			cd->startup_status = STARTUP_STATUS_BL_RESET_SENTINEL;
			pt_debug(cd->dev, DL_WARN,
				"%s: BL Reset sentinel recieved\n", __func__);
		}
		memcpy(cd->response_buf, cd->input_buf, 2);
		mutex_lock(&cd->system_lock);
		if (!cd->hid_reset_cmd_state && !cd->hid_cmd_state) {
			mutex_unlock(&cd->system_lock);
			pt_debug(cd->dev, DL_WARN,
				"%s: Device Initiated Reset\n", __func__);
			return 0;
		}

		cd->hid_reset_cmd_state = 0;
		if (cd->hid_cmd_state == HID_OUTPUT_START_BOOTLOADER + 1 ||
		    cd->hid_cmd_state == HID_OUTPUT_BL_LAUNCH_APP + 1 ||
		    cd->hid_cmd_state == HID_OUTPUT_USER_CMD + 1)
			cd->hid_cmd_state = 0;
		wake_up(&cd->wait_q);
		mutex_unlock(&cd->system_lock);
		return 0;
	} else if (size == 2 || size >= PT_PIP_1P7_EMPTY_BUF) {
		/*
		 * This debug message below is used by PBATS to calculate the
		 * time from the last lift off IRQ to when FW exits LFT mode.
		 */
		touch_report = false;
		pt_debug(cd->dev, DL_WARN,
			"%s: DUT - Empty buffer detected\n", __func__);
		return 0;
	} else if (size > PT_MAX_INPUT) {
		pt_debug(cd->dev, DL_ERROR,
			"%s: DUT - Unexpected length field in i2c data!\n",
			__func__);
		return -EINVAL;
	}

	if (cd->pip2_prot_active) {
		pt_debug(cd->dev, DL_DEBUG,
			"%s: Decode PIP2.x Response\n", __func__);
		if (cd->pip2_cmd_tag_seq != (cd->input_buf[2] & 0x0F)) {
			pt_debug(cd->dev, DL_WARN,
				"%s: ###PIP2 SEQ ERR: cmd=0x%02X rsp=0x%02X\n",
				__func__, cd->pip2_cmd_tag_seq,
				(cd->input_buf[2] & 0x0F));
		}
		/* PIP2 does not have a report id, hard code it */
		report_id = 0x00;
		cmd_id = cd->input_buf[PIP2_HID_INPUT_RESP_CMD_OFFSET];

		calc_crc = crc_ccitt_calculate(cd->input_buf, size - 2);
		resp_crc  = cd->input_buf[size - 2] << 8;
		resp_crc |= cd->input_buf[size - 1];
		if (resp_crc != calc_crc) {
			pt_debug(cd->dev, DL_ERROR,
				"%s: ###PIP2 CRC ERR: rsp=0x%04X calc=0x%04X\n",
				__func__, resp_crc, calc_crc);
#ifdef TTDL_DIAGNOSTICS
			cd->i2c_crc_error_count++;
			/* TODO - Add retry read */
#endif /* TTDL_DIAGNOSTICS */
		}
	} else {
		report_id = cd->input_buf[HID_OUTPUT_RESPONSE_REPORT_OFFSET];
		cmd_id = cd->input_buf[HID_OUTPUT_RESPONSE_CMD_OFFSET];
	}

#ifdef TTDL_DIAGNOSTICS
	pt_debug(cd->dev, DL_INFO,
		"%s: pip2 = %d report_id: 0x%02X, cmd_code: 0x%02X\n",
		__func__, cd->pip2_prot_active, report_id, (cmd_id & 0x7F));
#endif /* TTDL_DIAGNOSTICS */

	if (report_id == HID_WAKEUP_REPORT_ID) {
		pt_wakeup_host(cd);
		return 0;
	}

	mod_timer_pending(&cd->watchdog_timer, jiffies +
			msecs_to_jiffies(cd->watchdog_interval));

	if (cd->pip2_prot_active ||
		(report_id != cd->sysinfo.desc.tch_report_id &&
		 report_id != cd->sysinfo.desc.btn_report_id &&
		 report_id != HID_SENSOR_DATA_REPORT_ID &&
		 report_id != HID_TRACKING_HEATMAP_REPOR_ID)) {
		is_command = 1;
		touch_report = false;
	}

	if (unlikely(is_command)) {
		parse_command_input(cd, size);
		return 0;
	}

	if (touch_report)
		parse_touch_input(cd, size);

	return 0;
}

/*******************************************************************************
 * FUNCTION: pt_read_input
 *
 * SUMMARY: Reads incoming data off of the I2C bus
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *      *cd - pointer to core data
 ******************************************************************************/
static int pt_read_input(struct pt_core_data *cd)
{
	struct device *dev = cd->dev;
	int rc = 0;
	int t;
	int retry = PT_I2C_READ_INPUT_RETRY_COUNT;

	/*
	 * Workaround for easywake failure
	 * Interrupt for easywake, wait for bus controller to wake
	 */
	mutex_lock(&cd->system_lock);
	if (!IS_DEEP_SLEEP_CONFIGURED(cd->easy_wakeup_gesture)) {
		if (cd->sleep_state == SS_SLEEP_ON) {
			mutex_unlock(&cd->system_lock);
			if (!dev->power.is_suspended)
				goto read;
			t = wait_event_timeout(cd->wait_q,
				(cd->wait_until_wake == 1),
				msecs_to_jiffies(2000));
			if (IS_TMO(t)) {
#ifdef TTDL_DIAGNOSTICS
				cd->i2c_transmission_error_count++;
				pt_toggle_err_gpio(cd);
				pt_debug(dev, DL_ERROR,
					"%s: !!!I2C Transmission Error %d\n",
					__func__,
					cd->i2c_transmission_error_count);
#endif /* TTDL_DIAGNOSTICS */
				pt_queue_startup(cd);
			}
			goto read;
		}
	}
	mutex_unlock(&cd->system_lock);

read:
	/* Try reading up to 'retry' times */
	while (retry-- != 0) {
		rc = pt_adap_read_default_nosize(cd, cd->input_buf,
			PT_MAX_INPUT);
		if (!rc) {
			pt_debug(dev, DL_DEBUG,
				"%s: Read input successfully\n", __func__);
			goto read_exit;
		}
		msleep(20);
	}
	pt_debug(dev, DL_ERROR,
		"%s: Error getting report, rc=%d\n", __func__, rc);

read_exit:
	return rc;
}

/*******************************************************************************
 * FUNCTION: pt_check_irq_asserted
 *
 * SUMMARY: Checks if the IRQ GPIO is asserted or not
 *
 * RETURN:
 *	true  = IRQ asserted
 *	false = IRQ not asserted
 *
 * PARAMETERS:
 *      *cd - pointer to core data
 ******************************************************************************/
static  bool pt_check_irq_asserted(struct pt_core_data *cd)
{
#ifdef ENABLE_WORKAROUND_FOR_GLITCH_AFTER_BL_LAUNCH_APP
	/*
	 * Workaround for FW defect, CDT165308
	 * bl_launch app creates a glitch in IRQ line
	 */
	if (cd->hid_cmd_state == HID_OUTPUT_BL_LAUNCH_APP + 1
			&& cd->cpdata->irq_stat){
		/*
		 * in X1S panel and GC1546 panel, the width for the INT
		 * glitch is about 4us,the normal INT width of response
		 * will last more than 200us, so use 10us delay
		 * for distinguish the glitch the normal INT is enough.
		 */
		udelay(10);
		if (cd->cpdata->irq_stat &&
		    cd->cpdata->irq_stat(cd->cpdata, cd->dev)
			!= PT_IRQ_ASSERTED_VALUE)
			return false;
	}
#else
	if (cd->cpdata->irq_stat &&
	    cd->cpdata->irq_stat(cd->cpdata, cd->dev)
	    != PT_IRQ_ASSERTED_VALUE)
		return false;
#endif
	return true;
}

/*******************************************************************************
 * FUNCTION: _pt_irq
 *
 * SUMMARY: Process all detected interrupts
 *
 * RETURN:
 *	IRQ_HANDLED - Finished processing the interrupt
 *
 * PARAMETERS:
 *       irq    - IRQ number
 *	*handle - pointer to core data struct
 ******************************************************************************/
static irqreturn_t pt_irq(int irq, void *handle)
{
	struct pt_core_data *cd = handle;
	int rc;

	if (!pt_check_irq_asserted(cd))
		return IRQ_HANDLED;

	rc = pt_read_input(cd);
#ifdef TTDL_DIAGNOSTICS
	cd->irq_count++;

	/* Used to calculate T-Refresh */
	if (cd->t_refresh_active) {
		if (cd->t_refresh_count == 0) {
			cd->t_refresh_time = jiffies;
			cd->t_refresh_count++;
		} else if (cd->t_refresh_count < cd->t_refresh_total) {
			cd->t_refresh_count++;
		} else {
			cd->t_refresh_active = 0;
			cd->t_refresh_time =
			jiffies_to_msecs(jiffies - cd->t_refresh_time);
		}
	}
#endif /* TTDL_DIAGNOSTICS */

	if (!rc)
		pt_parse_input(cd);

	return IRQ_HANDLED;
}

/*******************************************************************************
 * FUNCTION: _pt_subscribe_attention
 *
 * SUMMARY: Function pointer included in core_cmds to allow other modules
 *	to subscribe themselves into the TTDL attention list
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *      *dev  - pointer to device structure
 *	 type - attention type enum
 *	*id   - ID of the module calling this function
 *	*func - callback function pointer to be called when notified
 *	 mode - attention mode
 ******************************************************************************/
int _pt_subscribe_attention(struct device *dev,
	enum pt_atten_type type, char *id, int (*func)(struct device *),
	int mode)
{
	struct pt_core_data *cd = dev_get_drvdata(dev);
	struct atten_node *atten, *atten_new;

	atten_new = kzalloc(sizeof(*atten_new), GFP_KERNEL);
	if (!atten_new)
		return -ENOMEM;

	pt_debug(cd->dev, DL_INFO, "%s from '%s'\n", __func__,
		dev_name(cd->dev));

	spin_lock(&cd->spinlock);
	list_for_each_entry(atten, &cd->atten_list[type], node) {
		if (atten->id == id && atten->mode == mode) {
			spin_unlock(&cd->spinlock);
			kfree(atten_new);
			pt_debug(cd->dev, DL_INFO, "%s: %s=%p %s=%d\n",
				 __func__,
				 "already subscribed attention",
				 dev, "mode", mode);

			return 0;
		}
	}

	atten_new->id = id;
	atten_new->dev = dev;
	atten_new->mode = mode;
	atten_new->func = func;

	list_add(&atten_new->node, &cd->atten_list[type]);
	spin_unlock(&cd->spinlock);

	return 0;
}

/*******************************************************************************
 * FUNCTION: _pt_unsubscribe_attention
 *
 * SUMMARY: Function pointer included in core_cmds to allow other modules
 *	to unsubscribe themselves from the TTDL attention list
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *      *dev  - pointer to device structure
 *	 type - attention type enum
 *	*id   - ID of the module calling this function
 *	*func - function pointer
 *	 mode - attention mode
 ******************************************************************************/
int _pt_unsubscribe_attention(struct device *dev,
	enum pt_atten_type type, char *id, int (*func)(struct device *),
	int mode)
{
	struct pt_core_data *cd = dev_get_drvdata(dev);
	struct atten_node *atten, *atten_n;

	spin_lock(&cd->spinlock);
	list_for_each_entry_safe(atten, atten_n, &cd->atten_list[type], node) {
		if (atten->id == id && atten->mode == mode) {
			list_del(&atten->node);
			spin_unlock(&cd->spinlock);
			kfree(atten);
			pt_debug(cd->dev, DL_DEBUG, "%s: %s=%p %s=%d\n",
				__func__,
				"unsub for atten->dev", atten->dev,
				"atten->mode", atten->mode);
			return 0;
		}
	}
	spin_unlock(&cd->spinlock);

	return -ENODEV;
}

/*******************************************************************************
 * FUNCTION: _pt_request_exclusive
 *
 * SUMMARY: Function pointer included in core_cmds to allow other modules
 *	to request exclusive access
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *      *dev        - pointer to device structure
 *	 timeout_ms - timeout to wait for exclusive access
 ******************************************************************************/
static int _pt_request_exclusive(struct device *dev,
		int timeout_ms)
{
	struct pt_core_data *cd = dev_get_drvdata(dev);

	return request_exclusive(cd, (void *)dev, timeout_ms);
}

/*******************************************************************************
 * FUNCTION: _pt_release_exclusive
 *
 * SUMMARY: Function pointer included in core_cmds to allow other modules
 *	to release exclusive access
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *      *dev - pointer to device structure
 ******************************************************************************/
static int _pt_release_exclusive(struct device *dev)
{
	struct pt_core_data *cd = dev_get_drvdata(dev);

	return release_exclusive(cd, (void *)dev);
}


/*******************************************************************************
 * FUNCTION: _pt_request_reset
 *
 * SUMMARY: Function pointer included in core_cmds to allow other modules
 *	to requst the DUT to be reset. Function returns err if refused or
 *	timeout occurs (Note: core uses fixed timeout period).
 *
 * NOTE: Function blocks until ISR occurs.
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *      *dev - pointer to device structure
 *	 protect - flag to call protected or non-protected
 ******************************************************************************/
static int _pt_request_reset(struct device *dev, int protect)
{
	struct pt_core_data *cd = dev_get_drvdata(dev);
	int rc;

	rc = pt_dut_reset(cd, protect);
	if (rc < 0) {
		pt_debug(cd->dev, DL_ERROR, "%s: Error on h/w reset r=%d\n",
			__func__, rc);
	}

	return rc;
}

/*******************************************************************************
 * FUNCTION: _pt_request_restart
 *
 * SUMMARY: Function pointer included in core_cmds to allow other modules
 *	to requst TTDL to queue a startup. This function will return err
 *	if refused; if no error then restart has completed and system is in
 *	normal operation mode.
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *      *dev - pointer to device structure
 *	 wait - boolean to determine if to wait for startup event
 ******************************************************************************/
static int _pt_request_restart(struct device *dev, bool wait)
{
	struct pt_core_data *cd = dev_get_drvdata(dev);

	pt_queue_startup(cd);

	if (wait)
		wait_event_timeout(cd->wait_q,
			cd->startup_state == STARTUP_NONE,
			msecs_to_jiffies(PT_REQUEST_STARTUP_TIMEOUT));

	return 0;
}

/*******************************************************************************
 * FUNCTION: _pt_request_sysinfo
 *
 * SUMMARY: Function pointer included in core_cmds to allow other modules
 *	to requst the pointer to the system information structure. This
 *	function will return NULL if sysinfo has not been aquired from the
 *	DUT yet.
 *
 * RETURN: Pointer to the system information struct
 *
 * PARAMETERS:
 *      *dev - pointer to device structure
 ******************************************************************************/
struct pt_sysinfo *_pt_request_sysinfo(struct device *dev)
{
	struct pt_core_data *cd = dev_get_drvdata(dev);

	if (cd->sysinfo.ready)
		return &cd->sysinfo;

	return NULL;
}

/*******************************************************************************
 * FUNCTION: _pt_request_loader_pdata
 *
 * SUMMARY: Function pointer included in core_cmds to allow other modules
 *	to requst the pointer to the loader platform data
 *
 * RETURN: Pointer to the loader platform data struct
 *
 * PARAMETERS:
 *      *dev - pointer to device structure
 ******************************************************************************/
static struct pt_loader_platform_data *_pt_request_loader_pdata(
		struct device *dev)
{
	struct pt_core_data *cd = dev_get_drvdata(dev);

	return cd->pdata->loader_pdata;
}

/*******************************************************************************
 * FUNCTION: _pt_request_start_wd
 *
 * SUMMARY: Function pointer included in core_cmds to allow other modules
 *	to requst to start the TTDL watchdog
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *      *dev - pointer to device structure
 ******************************************************************************/
static int _pt_request_start_wd(struct device *dev)
{
	struct pt_core_data *cd = dev_get_drvdata(dev);

	pt_start_wd_timer(cd);
	return 0;
}

/*******************************************************************************
 * FUNCTION: _pt_request_stop_wd
 *
 * SUMMARY: Function pointer included in core_cmds to allow other modules
 *	to requst to stop the TTDL watchdog
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *      *dev - pointer to device structure
 ******************************************************************************/
static int _pt_request_stop_wd(struct device *dev)
{
	struct pt_core_data *cd = dev_get_drvdata(dev);

	pt_stop_wd_timer(cd);
	return 0;
}

/*******************************************************************************
 * FUNCTION: _pt_request_pip2_get_mode
 *
 * SUMMARY: Determine the current mode of the DUT by use of the PIP2 STATUS
 *	command.
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*dev         - pointer to device structure
 *	 protect     - flag to run in protected mode
 *	*mode        - pointer to store the retrieved mode
 ******************************************************************************/
static int _pt_request_pip2_get_mode(struct device *dev, int protect,
	u8 *mode)
{
	struct pt_core_data *cd = dev_get_drvdata(dev);
	struct pip2_cmd_structure pip2_cmd;
	u16 actual_read_len;
	u8 read_buf[12];
	u8 status, boot;
	int rc;

	/* Get PIP2 Status to determine mode */
	rc = _pt_request_pip2_send_cmd(dev, protect, &pip2_cmd,
		PIP2_CMD_ID_STATUS, NULL, 0, read_buf, &actual_read_len);

	pt_debug(dev, DL_WARN, "%s: PIP2 STATUS command rc = %d\n",
		__func__, rc);

	if (rc) {
		/*
		 * FW supporting PIP1.x does not support this command
		 * so try old method to get mode
		 */
		rc = _pt_request_get_mode(dev, protect, mode);
		if (*mode == PT_MODE_UNKNOWN) {
			/* Flush i2c as a precaution */
			pt_debug(dev, DL_WARN,
				"%s: Flush I2C bus after failed cmd\n",
				__func__);
			pt_flush_i2c(cd, 0);
		}
	} else {
		status = read_buf[PIP2_RESPONSE_STATUS_OFFSET];
		boot = read_buf[PIP2_RESPONSE_BODY_OFFSET] & 0x01;
		pt_pr_buf(dev, DL_WARN, read_buf, actual_read_len, "Mode");

		if (status == PIP2_RSP_ERR_NONE && boot == 0x00)
			*mode = PT_MODE_BOOTLOADER;
		else if (status == PIP2_RSP_ERR_NONE && boot == 0x01)
			*mode = PT_MODE_OPERATIONAL;
		else
			*mode = PT_MODE_UNKNOWN;
	}

	return rc;
}

/*******************************************************************************
 * FUNCTION: pt_core_wake_device_from_deep_sleep_
 *
 * SUMMARY: Call the set_power function and set the DUT to wake up from
 *  deep sleep.
 *
 * RETURN:
 *	 0 = success
 *	!0 = error
 *
 * PARAMETERS:
 *      *cd - pointer to core data
 ******************************************************************************/
static int pt_core_wake_device_from_deep_sleep_(
		struct pt_core_data *cd)
{
	int rc;

	rc = pt_hid_cmd_set_power_(cd, HID_POWER_ON);
	if (rc)
		rc =  -EAGAIN;

	/* Prevent failure on sequential wake/sleep requests from OS */
	msleep(20);

	return rc;
}

/*******************************************************************************
 * FUNCTION: pt_core_wake_device_
 *
 * SUMMARY: Wake up device from deep sleep state or skip wake up operation based
 *  on the configuration of easy_wakeup_gesture in core data structure.
 *
 * RETURN:
 *   0 = success
 *  !0 = failure
 *
 * PARAMETERS:
 *  *cd  - pointer to core data
 ******************************************************************************/
static int pt_core_wake_device_(struct pt_core_data *cd)
{
	if (!IS_DEEP_SLEEP_CONFIGURED(cd->easy_wakeup_gesture)) {
		mutex_lock(&cd->system_lock);
		cd->wait_until_wake = 1;
		mutex_unlock(&cd->system_lock);
		wake_up(&cd->wait_q);
		msleep(20);

		if (cd->wake_initiated_by_device) {
			cd->wake_initiated_by_device = 0;
			return 0;
		}
	}

	return pt_core_wake_device_from_deep_sleep_(cd);
}

/*******************************************************************************
 * FUNCTION: pt_restore_parameters_
 *
 * SUMMARY: This function sends all RAM parameters stored in the linked list
 *	back to the DUT
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*cd   - pointer the core data structure
 ******************************************************************************/
static int pt_restore_parameters_(struct pt_core_data *cd)
{
	struct param_node *param;
	int rc = 0;

	if (!(cd->cpdata->flags & PT_CORE_FLAG_RESTORE_PARAMETERS))
		goto exit;

	spin_lock(&cd->spinlock);
	list_for_each_entry(param, &cd->param_list, node) {
		spin_unlock(&cd->spinlock);
		pt_debug(cd->dev, DL_INFO, "%s: Parameter id:%d value:%d\n",
			 __func__, param->id, param->value);
		rc = pt_pip_set_param_(cd, param->id,
			param->value, param->size);
		if (rc)
			goto exit;
		spin_lock(&cd->spinlock);
	}
	spin_unlock(&cd->spinlock);
exit:
	return rc;
}

/*******************************************************************************
 * FUNCTION: _fast_startup
 *
 * SUMMARY: Perform fast startup after resume device by power on/off stratergy.
 *
 * RETURN:
 *   0 = success
 *  !0 = failure
 *
 * PARAMETERS:
 *  *cd   - pointer the core data structure
 ******************************************************************************/
static int _fast_startup(struct pt_core_data *cd)
{
	int retry = PT_CORE_STARTUP_RETRY_COUNT;
	int rc;

reset:
	if (retry != PT_CORE_STARTUP_RETRY_COUNT)
		pt_debug(cd->dev, DL_WARN, "%s: Retry %d\n",
			__func__, PT_CORE_STARTUP_RETRY_COUNT - retry);

	rc = pt_get_hid_descriptor_(cd, &cd->hid_desc);
	if (rc < 0) {
		pt_debug(cd->dev, DL_ERROR,
			"%s: Error on getting HID descriptor r=%d\n",
			__func__, rc);
		if (retry--)
			goto reset;
		goto exit;
	}
	cd->mode = pt_get_mode(cd, &cd->hid_desc);

	if (cd->mode == PT_MODE_BOOTLOADER) {
		pt_debug(cd->dev, DL_INFO, "%s: Bootloader mode\n",
			__func__);
		rc = pt_hid_output_bl_launch_app_(cd);
		if (rc < 0) {
			pt_debug(cd->dev, DL_ERROR,
				"%s: Error on launch app r=%d\n", __func__, rc);
			if (retry--)
				goto reset;
			goto exit;
		}
		rc = pt_get_hid_descriptor_(cd, &cd->hid_desc);
		if (rc < 0) {
			pt_debug(cd->dev, DL_ERROR,
				"%s: Error on getting HID descriptor r=%d\n",
				__func__, rc);
			if (retry--)
				goto reset;
			goto exit;
		}
		cd->mode = pt_get_mode(cd, &cd->hid_desc);
		if (cd->mode == PT_MODE_BOOTLOADER) {
			if (retry--)
				goto reset;
			goto exit;
		}
	}

	rc = pt_restore_parameters_(cd);
	if (rc)
		pt_debug(cd->dev, DL_ERROR,
			"%s: failed to restore parameters rc=%d\n",
			__func__, rc);

exit:
	return rc;
}

/*******************************************************************************
 * FUNCTION: pt_core_poweron_device_
 *
 * SUMMARY: Power on device, enable IRQ, and then perform a fast startup.
 *
 * RETURN:
 *   0 = success
 *  !0 = failure
 *
 * PARAMETERS:
 *  *cd  - pointer to core data
 ******************************************************************************/
static int pt_core_poweron_device_(struct pt_core_data *cd)
{
	struct device *dev = cd->dev;
	int rc;

	rc = cd->cpdata->power(cd->cpdata, 1, dev, 0);
	if (rc < 0) {
		pt_debug(dev, DL_ERROR, "%s: HW Power up fails r=%d\n",
			__func__, rc);
		goto exit;
	}

	if (!cd->irq_enabled) {
		cd->irq_enabled = true;
		enable_irq(cd->irq);
	}

	rc = _fast_startup(cd);
exit:
	return rc;
}

/*******************************************************************************
 * FUNCTION: pt_core_wake_
 *
 * SUMMARY: Resume the device with a power on or wake from deep sleep based on
 *  the configuration in the core platform data structure.
 *
 * RETURN:
 *   0 = success
 *  !0 = failure
 *
 * PARAMETERS:
 *  *cd  - pointer to core data
 ******************************************************************************/
static int pt_core_wake_(struct pt_core_data *cd)
{
	int rc;

	mutex_lock(&cd->system_lock);
	if (cd->sleep_state == SS_SLEEP_ON) {
		cd->sleep_state = SS_WAKING;
	} else {
		mutex_unlock(&cd->system_lock);
		return 1;
	}
	mutex_unlock(&cd->system_lock);

	if (cd->cpdata->flags & PT_CORE_FLAG_POWEROFF_ON_SLEEP)
		rc = pt_core_poweron_device_(cd);
	else
		rc = pt_core_wake_device_(cd);

	mutex_lock(&cd->system_lock);
	cd->sleep_state = SS_SLEEP_OFF;
	mutex_unlock(&cd->system_lock);

	pt_start_wd_timer(cd);
	return rc;
}

/*******************************************************************************
 * FUNCTION: pt_core_wake_
 *
 * SUMMARY: Protected call to pt_core_wake_ by exclusive access to the DUT.
 *
 * RETURN:
 *   0 = success
 *  !0 = failure
 *
 * PARAMETERS:
 *  *cd  - pointer to core data
 ******************************************************************************/
static int pt_core_wake(struct pt_core_data *cd)
{
	int rc;

	rc = request_exclusive(cd, cd->dev, PT_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		pt_debug(cd->dev, DL_ERROR,
				"%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->dev);
		return rc;
	}

	rc = pt_core_wake_(cd);

	if (release_exclusive(cd, cd->dev) < 0)
		pt_debug(cd->dev, DL_ERROR, "%s: fail to release exclusive\n",
			__func__);
	else
		pt_debug(cd->dev, DL_DEBUG, "%s: pass release exclusive\n",
			__func__);

	return rc;
}

/*******************************************************************************
 * FUNCTION: pt_get_ic_crc_
 *
 * SUMMARY: This function retrieves the config block CRC
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*cd   - pointer the core data structure
 *	 ebid - enumerated block ID
 ******************************************************************************/
static int pt_get_ic_crc_(struct pt_core_data *cd, u8 ebid)
{
	struct pt_sysinfo *si = &cd->sysinfo;
	int rc;
	u8 status;
	u16 calculated_crc = 0;
	u16 stored_crc = 0;
	u8 scan_state = PT_SCAN_STATE_UNKNOWN;
	u8 retry = 3;

	rc = pt_pip_suspend_scanning_(cd);
	if (rc)
		goto error;

	rc = pt_pip_verify_config_block_crc_(cd, ebid, &status,
			&calculated_crc, &stored_crc);
	if (rc)
		goto error_resume_scan;

	if (status) {
		rc = -EINVAL;
		goto error_resume_scan;
	}

	si->ttconfig.crc = stored_crc;
	pt_debug(cd->dev, DL_INFO,
		"%s: CRC: calculated: 0x%04X, stored: 0x%04X\n",
		__func__, calculated_crc, stored_crc);
	pt_pip_resume_scanning_(cd);
	goto exit;

error_resume_scan:
	pt_pip_resume_scanning_(cd);
error:
	pt_debug(cd->dev, DL_ERROR, "%s: CRC: ebid:%d, crc:0x%04X, RC=%d\n",
		__func__, ebid, si->ttconfig.crc, rc);
exit:
	/* Check scan state,try to fix if it is not right */
	if (retry--) {
		_pt_get_scan_state(cd, &scan_state);
		if (scan_state != PT_SCAN_STATE_ACTIVE) {
			pt_debug(cd->dev, DL_ERROR,
					"%s: scan state: %d, retry: %d, rc = %d\n",
					__func__, scan_state, retry, rc);
			goto error_resume_scan;
		}
	}
	return rc;
}

/*******************************************************************************
 * FUNCTION: pt_startup_
 *
 * SUMMARY: This function does the full enumeration of the DUT with TTDL.
 *	The core data (cd) startup_status will store, as a bitmask, each
 *	state of the enumeration process. The startup will be attempted
 *	PT_CORE_STARTUP_RETRY_COUNT times before giving up.
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*cd    - pointer the core data structure
 *	 reset - Flag to reset the DUT before attempting to enumerate
 ******************************************************************************/
static int pt_startup_(struct pt_core_data *cd, bool reset)
{
	int try = 1;
	int rc;
	int count = 0;
	int bytes_read;
	bool detected = false;
	u8 mode = PT_MODE_UNKNOWN;


#ifdef TTHE_TUNER_SUPPORT
	tthe_print(cd, NULL, 0, "enter startup");
#endif
	pt_stop_wd_timer(cd);

reset:
	pt_debug(cd->dev, DL_WARN, "%s: DUT Enum Attempt %d\n", __func__, try);

	while (pt_check_irq_asserted(cd) && count < 5) {
		count++;
		bytes_read = pt_flush_i2c(cd, 0);
		if (bytes_read) {
			pt_debug(cd->dev, DL_WARN,
				"%s: Cleared %d bytes off I2C bus\n",
				__func__, bytes_read);
		}
	}

	_pt_request_dut_generation(cd->dev);
	if (cd->dut_generation == DUT_PIP1_ONLY) {
		if (reset || try != PT_CORE_STARTUP_RETRY_COUNT) {
			/*
			 * Reset hardware only for Legacy parts. Skip for TT/TC
			 * parts because if the FW image was loaded directly
			 * to SRAM issueing a reset ill wipe out what was just
			 * loaded.
			 */
			rc = pt_dut_reset_and_wait(cd);
			if (rc < 0) {
				pt_debug(cd->dev, DL_ERROR,
					"%s: Error on h/w reset r=%d\n",
					__func__, rc);
				if (try++ < PT_CORE_STARTUP_RETRY_COUNT)
					goto reset;
				goto exit;
			}
		}

		rc = pt_get_hid_descriptor_(cd, &cd->hid_desc);
		if (rc < 0) {
			pt_debug(cd->dev, DL_ERROR,
				"%s: Error on getting HID descriptor r=%d\n",
				__func__, rc);
			if (try++ < PT_CORE_STARTUP_RETRY_COUNT)
				goto reset;
			goto exit;
		}

		/* Don't set DESC flag if in Gen5/6 BL */
		if (!cd->hid_desc.packet_id == 0xFF)
			cd->startup_status |= STARTUP_STATUS_GET_DESC;
		detected = true;

		cd->mode = pt_get_mode(cd, &cd->hid_desc);

		/* Only valid for legacy DUTs that support panel ID */
		/* Must be in bootloader mode to get Panel ID */
		if (cd->mode == PT_MODE_OPERATIONAL) {
			rc = pt_pip_start_bootloader_(cd);
			if (rc < 0) {
				pt_debug(cd->dev, DL_ERROR,
					"%s: Error on start bootloader r=%d\n",
					__func__, rc);
				if (try++ < PT_CORE_STARTUP_RETRY_COUNT)
					goto reset;
				goto exit;
			}
			pt_debug(cd->dev, DL_INFO, "%s: Bootloader mode\n",
				__func__);
		}

		pt_hid_output_bl_get_panel_id_(cd, &cd->panel_id);
		pt_debug(cd->dev, DL_INFO, "%s: Panel ID: 0x%02X\n",
			__func__, cd->panel_id);

		rc = pt_hid_output_bl_launch_app_(cd);
		if (rc < 0) {
			pt_debug(cd->dev, DL_ERROR,
				"%s: Error on launch app r=%d\n",
				__func__, rc);
			if (try++ < PT_CORE_STARTUP_RETRY_COUNT)
				goto reset;
			goto exit;
		}

		cd->mode = pt_get_mode(cd, &cd->hid_desc);
		if (cd->mode == PT_MODE_BOOTLOADER) {
			if (try++ < PT_CORE_STARTUP_RETRY_COUNT)
				goto reset;
			goto exit;
		}
		/* Read and store the descriptor lengths */
		mutex_lock(&cd->system_lock);
		cd->hid_core.hid_report_desc_len =
			le16_to_cpu(cd->hid_desc.report_desc_len);
		cd->hid_core.hid_max_input_len =
			le16_to_cpu(cd->hid_desc.max_input_len);
		cd->hid_core.hid_max_output_len =
			le16_to_cpu(cd->hid_desc.max_output_len);
		mutex_unlock(&cd->system_lock);

		cd->mode = pt_get_mode(cd, &cd->hid_desc);
		if (cd->mode == PT_MODE_OPERATIONAL)
			pt_debug(cd->dev, DL_INFO,
				"%s: Operational mode\n", __func__);
		else if (cd->mode == PT_MODE_BOOTLOADER)
			pt_debug(cd->dev, DL_INFO,
				"%s: Bootloader mode\n", __func__);
		else if (cd->mode == PT_MODE_UNKNOWN) {
			pt_debug(cd->dev, DL_ERROR,
				"%s: Unknown mode\n", __func__);
			rc = -ENODEV;
			if (try++ < PT_CORE_STARTUP_RETRY_COUNT)
				goto reset;
			goto exit;
		}
		cd->startup_status |= STARTUP_STATUS_GET_MODE;
	} else {
		/* Generation is PIP2 Capable */
		_pt_request_pip2_get_mode(cd->dev, PT_CORE_CMD_UNPROTECTED,
					&mode);
		cd->mode = mode;
		if (cd->mode == PT_MODE_OPERATIONAL)
			pt_debug(cd->dev, DL_INFO,
				"%s: Operational mode\n", __func__);
		/* TODO: Maybe need exit bootloader command*/
		else if (cd->mode == PT_MODE_BOOTLOADER)
			pt_debug(cd->dev, DL_INFO,
				"%s: Bootloader mode\n", __func__);
		else if (cd->mode == PT_MODE_UNKNOWN) {
			pt_debug(cd->dev, DL_ERROR,
				"%s: Unknown mode\n", __func__);
			rc = -ENODEV;
			if (try++ < PT_CORE_STARTUP_RETRY_COUNT)
				goto reset;
			goto exit;
		}
		cd->startup_status |= STARTUP_STATUS_GET_DESC;
		cd->startup_status |= STARTUP_STATUS_GET_MODE;
		detected = true;
	}

	pt_init_pip_report_fields(cd);
	cd->startup_status |= STARTUP_STATUS_GET_RPT_DESC;
	if (!cd->features.easywake)
		cd->easy_wakeup_gesture = PT_CORE_EWG_NONE;

	pt_debug(cd->dev, DL_INFO, "%s: Reading sysinfo\n", __func__);
	rc = pt_hid_output_get_sysinfo_(cd);
	if (rc) {
		pt_debug(cd->dev, DL_ERROR,
			"%s: Error on getting sysinfo r=%d\n", __func__, rc);
		if (try++ < PT_CORE_STARTUP_RETRY_COUNT)
			goto reset;
		goto exit;
	}
	cd->startup_status |= STARTUP_STATUS_GET_SYS_INFO;

	pt_debug(cd->dev, DL_INFO, "%s pt Prot Version: %d.%d\n",
			__func__,
			cd->sysinfo.ttdata.pip_ver_major,
			cd->sysinfo.ttdata.pip_ver_minor);

	/* Read config version directly if PIP version < 1.2 */
	if (!IS_PIP_VER_GE(&cd->sysinfo, 1, 2)) {
		rc = pt_get_config_ver_(cd);
		if (rc) {
			pt_debug(cd->dev, DL_ERROR,
				"%s: failed to read config version rc=%d\n",
				__func__, rc);
		}
	}

	rc = pt_get_ic_crc_(cd, PT_TCH_PARM_EBID);
	if (rc) {
		pt_debug(cd->dev, DL_ERROR,
			"%s: DUT Config block CRC failure rc=%d\n",
			__func__, rc);
		if (try++ < PT_CORE_STARTUP_RETRY_COUNT)
			goto reset;
		goto exit;
	} else
		cd->startup_status |= STARTUP_STATUS_GET_CFG_CRC;

	rc = pt_restore_parameters_(cd);
	if (rc) {
		pt_debug(cd->dev, DL_ERROR,
			"%s: Failed to restore parameters rc=%d\n",
			__func__, rc);
	} else
		cd->startup_status |= STARTUP_STATUS_RESTORE_PARM;

	call_atten_cb(cd, PT_ATTEN_STARTUP, 0);
	cd->startup_status |= STARTUP_STATUS_COMPLETE;
	cd->watchdog_interval = PT_WATCHDOG_TIMEOUT;
	cd->startup_retry_count = 0;

exit:
	pt_start_wd_timer(cd);

	if (!detected)
		rc = -ENODEV;

#ifdef TTHE_TUNER_SUPPORT
	tthe_print(cd, NULL, 0, "exit startup");
#endif

	return rc;
}

/*******************************************************************************
 * FUNCTION: pt_startup
 *
 * SUMMARY: This is the safe function wrapper for pt_startup_() by requesting
 *	exclusive access around the call.
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*cd    - pointer the core data structure
 *	 reset - Flag to reset the DUT before attempting to enumerate
 ******************************************************************************/
static int pt_startup(struct pt_core_data *cd, bool reset)
{
	int rc;

	mutex_lock(&cd->system_lock);
	cd->startup_state = STARTUP_RUNNING;
	mutex_unlock(&cd->system_lock);

	rc = request_exclusive(cd, cd->dev, PT_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		pt_debug(cd->dev, DL_ERROR,
			"%s: fail get exclusive ex=%p own=%p\n",
			__func__, cd->exclusive_dev, cd->dev);
		goto exit;
	}

	rc = pt_startup_(cd, reset);

	if (release_exclusive(cd, cd->dev) < 0)
		/* Don't return fail code, mode is already changed. */
		pt_debug(cd->dev, DL_ERROR, "%s: fail to release exclusive\n",
			__func__);
	else
		pt_debug(cd->dev, DL_DEBUG, "%s: pass release exclusive\n",
			__func__);

exit:
	/* Set the startup state for any tasks waiting for startup completion */
	mutex_lock(&cd->system_lock);
	cd->startup_state = STARTUP_NONE;
	mutex_unlock(&cd->system_lock);

	/* Wake the waiters for end of startup */
	wake_up(&cd->wait_q);

	return rc;
}

static int add_sysfs_interfaces(struct device *dev);
/*******************************************************************************
 * FUNCTION: _pt_ttdl_restart
 *
 * SUMMARY: Restarts TTDL enumeration with the DUT and re-probes all modules
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *      *dev  - pointer to core device
 ******************************************************************************/
static int _pt_ttdl_restart(struct device *dev)
{
	int rc;
	struct pt_core_data *cd = dev_get_drvdata(dev);
#ifdef CONFIG_TOUCHSCREEN_PARADE_I2C
	struct i2c_client *client =
		(struct i2c_client *)container_of(dev, struct i2c_client, dev);
#endif

	/*
	 * Make sure the device is awake, pt_mt_release function will
	 * cause pm sleep function and lead to deadlock.
	 */
	pm_runtime_get_sync(dev);
	/* Use ttdl_restart_lock to avoid reentry */
	mutex_lock(&cd->ttdl_restart_lock);
	pt_btn_release(dev);
	pt_mt_release(dev);

#ifdef CONFIG_TOUCHSCREEN_PARADE_I2C
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pt_debug(dev, DL_ERROR,
			"%s I2C functionality not Supported\n", __func__);
		rc = -EIO;
		goto ttdl_no_error;
	}
#endif

#ifdef CONFIG_TOUCHSCREEN_PARADE_DEVICETREE_SUPPORT
	rc = pt_devtree_create_and_get_pdata(dev);
	if (rc < 0)
		goto ttdl_no_error;
#endif

	pt_debug(dev, DL_INFO, "%s: Call pt_startup\n", __func__);
	rc = pt_startup(cd, true);
	if (rc < 0)
		goto ttdl_error_startup;

	rc = pt_mt_probe(dev);
	if (rc < 0) {
		pt_debug(dev, DL_ERROR,
			"%s: Error, fail mt probe\n", __func__);
		goto ttdl_error_startup;
	}

	rc = pt_btn_probe(dev);
	if (rc < 0) {
		pt_debug(dev, DL_ERROR,
			"%s: Error, fail btn probe\n", __func__);
		goto ttdl_error_startup_mt;
	}

	if (!cd->sysfs_nodes_created && cd->hw_detected) {
		pt_debug(cd->dev, DL_INFO,
			"%s: Adding sysfs interfaces\n", __func__);
		rc = add_sysfs_interfaces(cd->dev);
		if (rc < 0)
			pt_debug(cd->dev, DL_ERROR,
				"%s: Error, fail sysfs init\n",
				__func__);
		else
			pt_start_wd_timer(cd);
	}

	pt_debug(cd->dev, DL_WARN,
			"%s: Well Done! TTDL Restart Successfull\n", __func__);
	rc = 0;
	goto ttdl_no_error;

ttdl_error_startup_mt:
	pr_err("%s PARADE error_startup_mt\n", __func__);
	pt_mt_release(dev);
ttdl_error_startup:
	pr_err("%s PARADE error_startup\n", __func__);
	pm_runtime_disable(dev);
ttdl_no_error:
	mutex_unlock(&cd->ttdl_restart_lock);
	pm_runtime_put(dev);
	return rc;
}

/*******************************************************************************
 * FUNCTION: pt_startup_work_function
 *
 * SUMMARY: This is the wrapper function placed in a work queue to call
 *	pt_startup()
 *
 * RETURN: none
 *
 * PARAMETERS:
 *	*work - pointer to the work_struct
 ******************************************************************************/
static void pt_startup_work_function(struct work_struct *work)
{
	struct pt_core_data *cd =  container_of(work,
		struct pt_core_data, startup_work);
	int rc;

	rc = _pt_ttdl_restart(cd->dev);
	if (rc < 0)
		pt_debug(cd->dev, DL_ERROR, "%s: Fail queued startup r=%d\n",
			__func__, rc);
}

/*
 * CONFIG_PM_RUNTIME option is removed in 3.19.0.
 */
#if defined(CONFIG_PM_RUNTIME) || \
		(KERNEL_VERSION(3, 19, 0) <= LINUX_VERSION_CODE)

/*******************************************************************************
 * FUNCTION: pt_core_rt_suspend
 *
 * SUMMARY: Wrapper function with PM Runtime stratergy to call pt_core_sleep.
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *      *dev  - pointer to core device
 ******************************************************************************/
static int pt_core_rt_suspend(struct device *dev)
{
	struct pt_core_data *cd = dev_get_drvdata(dev);
	int rc;

	rc = pt_core_sleep(cd);
	if (rc < 0) {
		pt_debug(dev, DL_ERROR, "%s: Error on sleep\n", __func__);
		return -EAGAIN;
	}
	return 0;
}

/*******************************************************************************
 * FUNCTION: pt_core_rt_resume
 *
 * SUMMARY: Wrapper function with PM Runtime stratergy to call pt_core_wake.
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *      *dev  - pointer to core device
 ******************************************************************************/
static int pt_core_rt_resume(struct device *dev)
{
	struct pt_core_data *cd = dev_get_drvdata(dev);
	int rc;

	rc = pt_core_wake(cd);
	if (rc < 0) {
		pt_debug(dev, DL_ERROR, "%s: Error on wake\n", __func__);
		return -EAGAIN;
	}

	return 0;
}
#endif

#if defined(CONFIG_PM_SLEEP)
/*******************************************************************************
 * FUNCTION: pt_core_suspend
 *
 * SUMMARY: Wrapper function with device suspend/resume stratergy to call
 *  pt_core_sleep. This function may disable IRQ during sleep state.
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *      *dev  - pointer to core device
 ******************************************************************************/
static int pt_core_suspend(struct device *dev)
{
	struct pt_core_data *cd = dev_get_drvdata(dev);

	pt_core_sleep(cd);

	if (IS_DEEP_SLEEP_CONFIGURED(cd->easy_wakeup_gesture))
		return 0;

	/* Required to prevent interrupts before i2c awake */
	disable_irq(cd->irq);
	cd->irq_disabled = 1;

	if (device_may_wakeup(dev)) {
		pt_debug(dev, DL_WARN, "%s Device MAY wakeup\n",
			__func__);
		if (!enable_irq_wake(cd->irq))
			cd->irq_wake = 1;
	} else {
		pt_debug(dev, DL_WARN, "%s Device MAY NOT wakeup\n",
			__func__);
	}

	return 0;
}

/*******************************************************************************
 * FUNCTION: pt_core_resume
 *
 * SUMMARY: Wrapper function with device suspend/resume stratergy to call
 *  pt_core_wake. This function may enable IRQ before wake up.
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *      *dev  - pointer to core device
 ******************************************************************************/
static int pt_core_resume(struct device *dev)
{
	struct pt_core_data *cd = dev_get_drvdata(dev);

	if (IS_DEEP_SLEEP_CONFIGURED(cd->easy_wakeup_gesture))
		goto exit;

	/*
	 * I2C bus pm does not call suspend if device runtime suspended
	 * This flag is covers that case
	 */
	if (cd->irq_disabled) {
		enable_irq(cd->irq);
		cd->irq_disabled = 0;
	}

	if (device_may_wakeup(dev)) {
		pt_debug(dev, DL_WARN, "%s Device MAY wakeup\n",
			__func__);
		if (cd->irq_wake) {
			disable_irq_wake(cd->irq);
			cd->irq_wake = 0;
		}
	} else {
		pt_debug(dev, DL_WARN, "%s Device MAY NOT wakeup\n",
			__func__);
	}

exit:
	pt_core_wake(cd);

	return 0;
}
#endif

#if NEED_SUSPEND_NOTIFIER
/*******************************************************************************
 * FUNCTION: pt_pm_notifier
 *
 * SUMMARY: This function is registered to notifier chain and will perform
 *  suspend operation if match event PM_SUSPEND_PREPARE.
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *  *nb     - pointer to notifier_block structure
 *   action - notifier event type
 *  *data   - void pointer
 ******************************************************************************/
static int pt_pm_notifier(struct notifier_block *nb,
		unsigned long action, void *data)
{
	struct pt_core_data *cd = container_of(nb,
			struct pt_core_data, pm_notifier);

	if (action == PM_SUSPEND_PREPARE) {
		pt_debug(cd->dev, DL_INFO, "%s: Suspend prepare\n",
			__func__);

		/*
		 * If PM runtime is not suspended, either call runtime
		 * PM suspend callback or wait until it finishes
		 */
		if (!pm_runtime_suspended(cd->dev))
			pm_runtime_suspend(cd->dev);

		(void) pt_core_suspend(cd->dev);
	}

	return NOTIFY_DONE;
}
#endif

const struct dev_pm_ops pt_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(pt_core_suspend, pt_core_resume)
	SET_RUNTIME_PM_OPS(pt_core_rt_suspend, pt_core_rt_resume,
			NULL)
};
EXPORT_SYMBOL_GPL(pt_pm_ops);


/*******************************************************************************
 * FUNCTION: _pt_request_set_runfw_pin
 *
 * SUMMARY: Set the state of the run_fw gpio.
 *	0 = Set GPIO as output and force it low
 *	1 = Set GPIO as input and let it float high
 *
 * PARAMETERS:
 *      *dev   - pointer to device structure
 *       value - <1|0> set value for GPIO
 ******************************************************************************/
static void _pt_request_set_runfw_pin(struct device *dev, int value)
{
	struct pt_platform_data *pdata = dev_get_platdata(dev);
	int runfw_gpio = pdata->core_pdata->runfw_gpio;
	int rc;

	if (runfw_gpio) {
		gpio_free(runfw_gpio);
		rc = gpio_request(runfw_gpio, NULL);
		if (rc) {
			pt_debug(dev, DL_ERROR,
				"error requesting runfw_gpio gpio %d\n",
				runfw_gpio);
			goto exit;
		} else {
			if (!value)
				gpio_direction_output(runfw_gpio, 0);
			else
				gpio_direction_input(runfw_gpio);
		}
		pt_debug(dev, DL_WARN,
			"%s: Set RunFW GPIO %d to: %d; read %d\n", __func__,
			runfw_gpio, value, gpio_get_value(runfw_gpio));
	} else {
		pt_debug(dev, DL_WARN, "%s: RunFW GPIO not defined\n",
			__func__);
	}
exit:
	return;
}

/*******************************************************************************
 * FUNCTION: _pt_request_pip2_enter_bl
 *
 * SUMMARY: Force the DUT to enter the BL by use of the RunFW pin and reseting
 *	the DUT by use of the XRES pin or a soft reset.
 *
 *	NOTE: The WD will be stopped and not restarted as it is not valid when
 *	      the DUT is in the BL
 *
 * RETURNS:
 *	PT_ENTER_BL_PASS              (0)
 *	PT_ENTER_BL_ERROR             (1)
 *	PT_ENTER_BL_RESET_FAIL        (2)
 *	PT_ENTER_BL_HID_START_BL_FAIL (3)
 *	PT_ENTER_BL_CONFIRM_FAIL      (4)
 *
 * PARAMETERS:
 *	*dev        - pointer to device structure
 *	*start_mode - pointer to the mode the DUT was in when this function
 *	              starts
 ******************************************************************************/
int _pt_request_pip2_enter_bl(struct device *dev, u8 *start_mode)
{
	int rc;
	int result = PT_ENTER_BL_ERROR;
	u8 mode = PT_MODE_UNKNOWN;
	struct pt_core_data *cd = dev_get_drvdata(dev);
	u8 host_mode_cmd[4] = {0xA5, 0xA5, 0xA5, 0xA5};

	pt_stop_wd_timer(cd);
	cd->startup_status = STARTUP_STATUS_START;

	rc = _pt_request_pip2_get_mode(dev, PT_CORE_CMD_UNPROTECTED,
		&mode);
	if (rc) {
		pt_debug(dev, DL_ERROR,
			"%s: Get mode failed, mode unknown\n", __func__);
	}
	*start_mode = mode;
	pt_debug(dev, DL_WARN, "%s: Get Mode = %d", __func__, mode);

	switch (mode) {
	case PT_MODE_UNKNOWN:
		/*
		 * When the mode could not be determined the DUT could be
		 * in App mode running corrupted FW or FW that is not
		 * responding to the mode request, assume no communication
		 * and do a hard reset
		 */
		pt_debug(dev, DL_WARN,
			"%s Set RunFW pin low\n", __func__);
		_pt_request_set_runfw_pin(dev, 0);
		rc = pt_dut_reset(cd, PT_CORE_CMD_UNPROTECTED);
		if (rc)
			result = PT_ENTER_BL_RESET_FAIL;
		break;

	case PT_MODE_OPERATIONAL:
		pt_debug(dev, DL_WARN, "%s: Suspend Scanning\n", __func__);
		rc = pt_pip_suspend_scanning_(cd);
		if (rc) {
			/*
			 * Print to log but don't exit, the FW could be
			 * running but be hung or fail to respond to this
			 * request
			 */
			pt_debug(dev, DL_ERROR, "%s Suspend Scan Failed\n",
				__func__);
		}
		/* sleep to allow the suspend scanning to be processed */
		msleep(PT_PIP2_LAUNCH_BL_DELAY);

		pt_debug(dev, DL_WARN, "%s Set RunFW pin low\n", __func__);
		_pt_request_set_runfw_pin(dev, 0);

		/* Reset device to enter the BL */
		rc = pt_dut_reset(cd, PT_CORE_CMD_UNPROTECTED);
		if (rc)
			result = PT_ENTER_BL_RESET_FAIL;
		break;

	case PT_MODE_BOOTLOADER:
		/* Do nothing as we are already in the BL */
		result = PT_ENTER_BL_PASS;
		break;

	default:
		/* Should NEVER get here */
		result = PT_ENTER_BL_ERROR;
		pt_debug(dev, DL_ERROR, "%s: Unknow mode code\n", __func__);
		break;
	}

	if (mode == PT_MODE_UNKNOWN || mode == PT_MODE_OPERATIONAL) {
		/*
		 * DUTs that support PIP2.04 or greater may not have a
		 * RunFW pin, so sending the special "Host Mode" command
		 * will instruct the BL to not execute the FW it has loaded
		 * into RAM. The command must be sent within a 40ms window
		 * from releasing the XRES pin.
		 */
		rc = pt_adap_write_read_specific(cd, 4,
			host_mode_cmd, NULL);

		/* Sleep to alow the BL to come up */
		msleep(PT_PIP2_LAUNCH_BL_DELAY);

	}

	pt_debug(dev, DL_INFO, "%s: Set RunFW pin high\n", __func__);
	_pt_request_set_runfw_pin(dev, 1);

	/* Ensure device is now in BL mode */
	rc = _pt_request_pip2_get_mode(dev, PT_CORE_CMD_UNPROTECTED,
		&mode);
	pt_debug(dev, DL_WARN, "%s: Get Mode = %d", __func__, mode);
	if (!rc && mode == PT_MODE_BOOTLOADER) {
		rc = _pt_request_active_pip_protocol(dev,
			PT_CORE_CMD_UNPROTECTED,
			&(cd->bl_pip_version_major),
			&(cd->bl_pip_version_minor));
		pt_debug(dev, DL_WARN, "%s In bootloader mode now\n", __func__);
		cd->pip2_prot_active = 1;
		result = PT_ENTER_BL_PASS;
	} else {
		result = PT_ENTER_BL_CONFIRM_FAIL;
		pt_debug(dev, DL_ERROR,
			"%s ERROR: Not in BL as expected", __func__);
		pt_debug(dev, DL_ERROR,
			"%s Enter BL error PIP ver = 0x%02x, Abort\n",
			__func__, cd->bl_pip_version_major);
	}

	pt_debug(dev, DL_INFO, "%s: Ensure RunFW pin high\n", __func__);
	_pt_request_set_runfw_pin(dev, 1);
	return result;
}

/*******************************************************************************
 * FUNCTION: _pt_pip2_file_open
 *
 * SUMMARY: Using the BL PIP2 commands open a file and return the file handle
 *
 *	NOTE: The DUT must be in BL mode for this command to work
 *
 * RETURNS:
 *	<0 = Error
 *	>0 = file handle opened
 *
 * PARAMETERS:
 *	*dev     - pointer to device structure
 *	 file_no - PIP2 file number to open
 ******************************************************************************/
int _pt_pip2_file_open(struct device *dev, u8 file_no)
{
	int ret = 0;
	u16 status;
	u16 actual_read_len;
	u8  file_handle;
	u8  data[2];
	u8  read_buf[10];
	struct pip2_cmd_structure pip2_cmd;

	data[0] = file_no;
	ret = _pt_request_pip2_send_cmd(dev,
		PT_CORE_CMD_UNPROTECTED, &pip2_cmd,
		PIP2_CMD_ID_FILE_OPEN, data, 1, read_buf, &actual_read_len);
	status = read_buf[PIP2_RESPONSE_STATUS_OFFSET];
	file_handle = read_buf[PIP2_RESPONSE_BODY_OFFSET];
	if (ret || ((status != 0x00) && (status != 0x03)) ||
	    (file_handle != file_no)) {
		pt_debug(dev, DL_ERROR,
			"%s File open failure with file = %d\n",
			__func__, file_handle);
		return -1;
	}
	return file_handle;
}

/*******************************************************************************
 * FUNCTION: _pt_pip2_file_close
 *
 * SUMMARY: Using the BL PIP2 commands close a file
 *
 *	NOTE: The DUT must be in BL mode for this command to work
 *
 * RETURNS:
 *	<0 = Error
 *	>0 = file handle closed
 *
 * PARAMETERS:
 *	*dev         - pointer to device structure
 *	 file_handle - handle to the file to be closed
 ******************************************************************************/
int _pt_pip2_file_close(struct device *dev, u8 file_handle)
{
	int ret = 0;
	u16 status;
	u16 actual_read_len;
	u8  data[2];
	u8  read_buf[10];
	struct pip2_cmd_structure pip2_cmd;

	data[0] = file_handle;
	ret = _pt_request_pip2_send_cmd(dev,
		PT_CORE_CMD_UNPROTECTED, &pip2_cmd,
		PIP2_CMD_ID_FILE_CLOSE, data, 1, read_buf, &actual_read_len);
	status = read_buf[PIP2_RESPONSE_STATUS_OFFSET];
	if (ret || (status != 0x00)) {
		pt_debug(dev, DL_ERROR,
			"%s File close failure with file = %d\n",
			__func__, file_handle);
		return -1;
	}
	return file_handle;
}

/*******************************************************************************
 * FUNCTION: _pt_pip2_file_read
 *
 * SUMMARY: Using the BL PIP2 commands read n bytes from a already opened file
 *
 *	NOTE: The DUT must be in BL mode for this command to work
 *
 * RETURNS:
 *	<0 = Error
 *	>0 = number of bytes read
 *
 * PARAMETERS:
 *	*dev         - pointer to device structure
 *	 file_handle - File handle to read from
 *	 num_bytes   - number of bytes to read
 ******************************************************************************/
int _pt_pip2_file_read(struct device *dev, u8 file_handle, u16 num_bytes,
	u8 *read_buf)
{
	int ret = 0;
	u16 status;
	u16 actual_read_len;
	u8  data[3];
	struct pip2_cmd_structure pip2_cmd;

	data[0] = file_handle;
	data[1] = (num_bytes & 0x00FF);
	data[2] = (num_bytes & 0xFF00) >> 8;
	ret = _pt_request_pip2_send_cmd(dev,
		PT_CORE_CMD_UNPROTECTED, &pip2_cmd,
		PIP2_CMD_ID_FILE_READ, data, 3, read_buf,
		&actual_read_len);
	status = read_buf[PIP2_RESPONSE_STATUS_OFFSET];
	if (ret || ((status != 0x00) && (status != 0x03))) {
		pt_debug(dev, DL_ERROR,
			"%s File open failure with error code = %d\n",
			__func__, status);
		return -1;
	}
	ret = num_bytes;

	return ret;
}

/*******************************************************************************
 * FUNCTION: _pt_request_pip2_bin_hdr
 *
 * SUMMARY: Read the stored bin file header from Flash and parse the contents
 *
 * RETURNS:
 *	 0 = Success
 *	!0 = Error condition
 *
 * PARAMETERS:
 *	*dev - pointer to device structure
 ******************************************************************************/
int _pt_request_pip2_bin_hdr(struct device *dev, struct pt_bin_file_hdr *hdr)
{
	struct pt_core_data *cd = dev_get_drvdata(dev);
	u8  file_handle;
	u8  read_buf[32];
	u8  mode;
	int bytes_read;
	int read_size;
	int ret = -1;

	/* Enter BL */
	pt_debug(dev, DL_INFO, "%s Enter BL\n", __func__);
	if (_pt_request_pip2_enter_bl(dev, &mode) != 0) {
		pt_debug(dev, DL_ERROR,
			"%s Failed to enter BL\n", __func__);
		goto exit;
	}

	/* Open the bin file in Flash */
	pt_debug(dev, DL_INFO, "%s Open File 1\n", __func__);
	file_handle = _pt_pip2_file_open(dev, 1);
	if (file_handle != 1) {
		pt_debug(dev, DL_ERROR,
			"%s Failed to open bin file\n", __func__);
		goto exit;
	}

	/* Read the 19 byte header from the file */
	pt_debug(dev, DL_INFO, "%s Read 19 byte header 1\n", __func__);
	read_size = 19;
	bytes_read = _pt_pip2_file_read(dev, file_handle, read_size, read_buf);
	if (bytes_read != read_size) {
		pt_debug(dev, DL_ERROR,
			"%s Failed to read bin file\n", __func__);
		goto exit;
	}

	hdr->length      = (read_buf[5]);
	hdr->ttpid       = (read_buf[6] << 8) | read_buf[7];
	hdr->fw_major    = (read_buf[8]);
	hdr->fw_minor    = (read_buf[9]);
	hdr->fw_rev_ctrl = (read_buf[14] << 24) | (read_buf[15] << 16) |
			   (read_buf[16] << 8)  | (read_buf[17]);
	hdr->si_rev      = (read_buf[19] << 8)  | (read_buf[18]);
	hdr->si_id       = (read_buf[21] << 8)  | (read_buf[20]);
	hdr->config_ver  = (read_buf[22] << 8)  | (read_buf[23]);

	/* Close the file */
	if (file_handle != _pt_pip2_file_close(dev, file_handle)) {
		pt_debug(dev, DL_ERROR,
			"%s Failed to close bin file\n", __func__);
		goto exit;
	}
	if (hdr->length < PT_BIN_FILE_MIN_HDR_LENGTH &&
	    hdr->length > PT_BIN_FILE_MAX_HDR_LENGTH)
		goto exit;

	ret = 0;
exit:

	/* If DUT was in app mode before this function, return to app mode */
	if (mode == PT_MODE_OPERATIONAL) {
		ret = pt_hw_hard_reset(cd);
		_pt_request_start_wd(dev);
	}

	return ret;
}

#define PT_MAX_CONFIG_BYTES_DEC  256
#define PT_INPUT_ELEM_SZ_DEC      10
#define PT_MAX_CONFIG_BYTES_HEX  256
#define PT_INPUT_ELEM_SZ_HEX     256

/*******************************************************************************
 * FUNCTION: pt_ic_parse_input_dec
 *
 * SUMMARY: Parse a char data array as space delimited decimal values into
 *	an int array.
 *
 * RETURN: Length of parsed data
 *
 * PARAMETERS:
 *	*dev         - pointer to device structure
 *	*buf         - pointer to buffer that holds the input array to parse
 *	 buf_size    - size of buf
 *	*ic_buf      - pointer to array to store parsed data
 *	 ic_buf_size - max size of ic_buf
 ******************************************************************************/
static int pt_ic_parse_input_dec(struct device *dev, const char *buf,
		size_t buf_size, u32 *ic_buf, size_t ic_buf_size)
{
	const char *pbuf = buf;
	unsigned long value;
	char scan_buf[PT_INPUT_ELEM_SZ_DEC];
	u32 i = 0;
	u32 j;
	int last = 0;
	int ret;

	pt_debug(dev, DL_DEBUG,
		"%s: pbuf=%p buf=%p size=%zu %s=%zu buf=%s\n",
		__func__, pbuf, buf, buf_size, "scan buf size",
		(size_t)PT_INPUT_ELEM_SZ_DEC, buf);

	while (pbuf <= (buf + buf_size)) {
		if (i >= PT_MAX_CONFIG_BYTES_DEC) {
			pt_debug(dev, DL_ERROR, "%s: %s size=%d max=%d\n",
					__func__,
					"Max cmd size exceeded", i,
					PT_MAX_CONFIG_BYTES_DEC);
			return -EINVAL;
		}
		if (i >= ic_buf_size) {
			pt_debug(dev, DL_ERROR, "%s: %s size=%d buf_size=%zu\n",
					__func__,
					"Buffer size exceeded", i, ic_buf_size);
			return -EINVAL;
		}
		while (((*pbuf == ' ') || (*pbuf == ','))
				&& (pbuf < (buf + buf_size))) {
			last = *pbuf;
			pbuf++;
		}

		if (pbuf >= (buf + buf_size))
			break;

		memset(scan_buf, 0, PT_INPUT_ELEM_SZ_DEC);
		if ((last == ',') && (*pbuf == ',')) {
			pt_debug(dev, DL_ERROR, "%s: %s \",,\" not allowed.\n",
					__func__, "Invalid data format.");
			return -EINVAL;
		}
		for (j = 0; j < (PT_INPUT_ELEM_SZ_DEC - 1)
				&& (pbuf < (buf + buf_size))
				&& (*pbuf != ' ')
				&& (*pbuf != ','); j++) {
			last = *pbuf;
			scan_buf[j] = *pbuf++;
		}
		ret = kstrtoul(scan_buf, 10, &value);
		if (ret < 0) {
			pt_debug(dev, DL_ERROR,
					"%s: Invalid data format.\n", __func__);
			return ret;
		}

		ic_buf[i] = value;
		i++;
	}

	return i;
}

/*******************************************************************************
 * FUNCTION: pt_ic_parse_input_hex
 *
 * SUMMARY: Parse a char data array as space delimited hex values into
 *	an int array.
 *
 * RETURN: Length of parsed data
 *
 * PARAMETERS:
 *	*dev         - pointer to device structure
 *	*buf         - pointer to buffer that holds the input array to parse
 *	 buf_size    - size of buf
 *	*ic_buf      - pointer to array to store parsed data
 *	 ic_buf_size - max size of ic_buf
 ******************************************************************************/
static int pt_ic_parse_input_hex(struct device *dev, const char *buf,
	size_t buf_size, u8 *ic_buf, size_t ic_buf_size)
{
	const char *pbuf = buf;
	unsigned long value;
	char scan_buf[PT_INPUT_ELEM_SZ_HEX];
	u32 i = 0;
	u32 j;
	int last = 0;
	int ret;

	pt_debug(dev, DL_DEBUG,
		"%s: pbuf=%p buf=%p size=%zu %s=%d buf=%s\n",
		__func__, pbuf, buf, buf_size, "scan buf size",
		PT_INPUT_ELEM_SZ_HEX, buf);

	while (pbuf <= (buf + buf_size)) {
		if (i >= PT_MAX_CONFIG_BYTES_HEX) {
			pt_debug(dev, DL_ERROR, "%s: %s size=%d max=%d\n",
				__func__, "Max cmd size exceeded", i,
				PT_MAX_CONFIG_BYTES_HEX);
			return -EINVAL;
		}
		if (i >= ic_buf_size) {
			pt_debug(dev, DL_ERROR, "%s: %s size=%d buf_size=%zu\n",
				__func__, "Buffer size exceeded", i,
				ic_buf_size);
			return -EINVAL;
		}
		while (((*pbuf == ' ') || (*pbuf == ','))
			&& (pbuf < (buf + buf_size))) {
			last = *pbuf;
			pbuf++;
		}

		if (pbuf >= (buf + buf_size))
			break;

		memset(scan_buf, 0, PT_INPUT_ELEM_SZ_HEX);
		if ((last == ',') && (*pbuf == ',')) {
			pt_debug(dev, DL_ERROR, "%s: %s \",,\" not allowed.\n",
				__func__, "Invalid data format.");
			return -EINVAL;
		}
		for (j = 0; j < (PT_INPUT_ELEM_SZ_HEX - 1)
		     && (pbuf < (buf + buf_size))
		     && (*pbuf != ' ')
		     && (*pbuf != ','); j++) {
			last = *pbuf;
			scan_buf[j] = *pbuf++;
		}
		ret = kstrtoul(scan_buf, 16, &value);
		if (ret < 0) {
			pt_debug(dev, DL_ERROR,
				"%s: %s '%s' %s%s i=%d r=%d\n", __func__,
				"Invalid data format. ", scan_buf,
				"Use \"0xHH,...,0xHH\"", " instead.",
				i, ret);
			return ret;
		}

		ic_buf[i] = value;
		pt_debug(dev, DL_DEBUG, "%s: item = %d, value = 0x%02lx",
			__func__, i, value);
		i++;
	}

	return i;
}

#ifdef TTHE_TUNER_SUPPORT
/*******************************************************************************
 * FUNCTION: tthe_debugfs_open
 *
 * SUMMARY: Open method for tthe_tuner debugfs node.
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *      *inode - file inode number
 *      *filp  - file pointer to debugfs file
 ******************************************************************************/
static int tthe_debugfs_open(struct inode *inode, struct file *filp)
{
	struct pt_core_data *cd = inode->i_private;

	filp->private_data = inode->i_private;

	if (cd->tthe_buf)
		return -EBUSY;

	cd->tthe_buf = kzalloc(PT_MAX_PRBUF_SIZE, GFP_KERNEL);
	if (!cd->tthe_buf)
		return -ENOMEM;

	return 0;
}

/*******************************************************************************
 * FUNCTION: tthe_debugfs_close
 *
 * SUMMARY: Close method for tthe_tuner debugfs node.
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *      *inode - file inode number
 *      *filp  - file pointer to debugfs file
 ******************************************************************************/
static int tthe_debugfs_close(struct inode *inode, struct file *filp)
{
	struct pt_core_data *cd = filp->private_data;

	filp->private_data = NULL;

	kfree(cd->tthe_buf);
	cd->tthe_buf = NULL;

	return 0;
}

/*******************************************************************************
 * FUNCTION: tthe_debugfs_read
 *
 * SUMMARY: Read method for tthe_tuner debugfs node. This function prints
 *	tthe_buf to user buffer.
 *
 * RETURN: Size of debugfs data print
 *
 * PARAMETERS:
 *      *filp   - file pointer to debugfs file
 *      *buf    - the user space buffer to read to
 *       count  - the maximum number of bytes to read
 *      *ppos   - the current position in the buffer
 ******************************************************************************/
static ssize_t tthe_debugfs_read(struct file *filp, char __user *buf,
		size_t count, loff_t *ppos)
{
	struct pt_core_data *cd = filp->private_data;
	int size;
	int ret;

	wait_event_interruptible(cd->wait_q,
			cd->tthe_buf_len != 0 || cd->tthe_exit);
	mutex_lock(&cd->tthe_lock);
	if (cd->tthe_exit) {
		mutex_unlock(&cd->tthe_lock);
		return 0;
	}
	if (count > cd->tthe_buf_len)
		size = cd->tthe_buf_len;
	else
		size = count;
	if (!size) {
		mutex_unlock(&cd->tthe_lock);
		return 0;
	}

	ret = copy_to_user(buf, cd->tthe_buf, cd->tthe_buf_len);
	if (ret == size)
		return -EFAULT;
	size -= ret;
	cd->tthe_buf_len -= size;
	mutex_unlock(&cd->tthe_lock);
	*ppos += size;
	return size;
}

static const struct file_operations tthe_debugfs_fops = {
	.open = tthe_debugfs_open,
	.release = tthe_debugfs_close,
	.read = tthe_debugfs_read,
};
#endif

static struct pt_core_nonhid_cmd _pt_core_nonhid_cmd = {
	.start_bl             = _pt_request_pip_start_bl,
	.suspend_scanning     = _pt_request_pip_suspend_scanning,
	.resume_scanning      = _pt_request_pip_resume_scanning,
	.get_param            = _pt_request_pip_get_param,
	.set_param            = _pt_request_pip_set_param,
	.verify_cfg_block_crc = _pt_request_pip_verify_config_block_crc,
	.get_config_row_size  = _pt_request_pip_get_config_row_size,
	.get_data_structure   = _pt_request_pip_get_data_structure,
	.run_selftest         = _pt_request_pip_run_selftest,
	.get_selftest_result  = _pt_request_pip_get_selftest_result,
	.calibrate_idacs      = _pt_request_pip_calibrate_idacs,
	.initialize_baselines = _pt_request_pip_initialize_baselines,
	.exec_panel_scan      = _pt_request_pip_exec_panel_scan,
	.retrieve_panel_scan  = _pt_request_pip_retrieve_panel_scan,
	.write_conf_block     = _pt_request_pip_write_conf_block,
	.user_cmd             = _pt_request_pip_user_cmd,
	.get_bl_info          = _pt_request_pip_bl_get_information,
	.initiate_bl          = _pt_request_pip_bl_initiate_bl,
	.launch_app           = _pt_request_pip_launch_app,
	.prog_and_verify      = _pt_request_pip_bl_program_and_verify,
	.verify_app_integrity = _pt_request_pip_bl_verify_app_integrity,
	.get_panel_id         = _pt_request_pip_bl_get_panel_id,
	.pip2_send_cmd        = _pt_request_pip2_send_cmd,
};

static struct pt_core_commands _pt_core_commands = {
	.subscribe_attention     = _pt_subscribe_attention,
	.unsubscribe_attention   = _pt_unsubscribe_attention,
	.request_exclusive       = _pt_request_exclusive,
	.release_exclusive       = _pt_release_exclusive,
	.request_reset           = _pt_request_reset,
	.request_restart         = _pt_request_restart,
	.request_sysinfo         = _pt_request_sysinfo,
	.request_loader_pdata    = _pt_request_loader_pdata,
	.request_stop_wd         = _pt_request_stop_wd,
	.request_start_wd        = _pt_request_start_wd,
	.request_get_hid_desc    = _pt_request_get_hid_desc,
	.request_get_mode        = _pt_request_get_mode,
	.request_active_pip_prot = _pt_request_active_pip_protocol,
	.request_pip2_get_mode   = _pt_request_pip2_get_mode,
	.request_pip2_enter_bl   = _pt_request_pip2_enter_bl,
	.request_pip2_bin_hdr    = _pt_request_pip2_bin_hdr,
	.request_set_runfw_pin   = _pt_request_set_runfw_pin,
	.request_dut_generation  = _pt_request_dut_generation,
#ifdef TTHE_TUNER_SUPPORT
	.request_tthe_print      = _pt_request_tthe_print,
#endif
	.nonhid_cmd              = &_pt_core_nonhid_cmd,
	.request_get_scan_state  = _pt_request_get_scan_state,
};

struct pt_core_commands *pt_get_commands(void)
{
	return &_pt_core_commands;
}
EXPORT_SYMBOL_GPL(pt_get_commands);

static DEFINE_MUTEX(core_list_lock);
static LIST_HEAD(core_list);
static DEFINE_MUTEX(module_list_lock);
static LIST_HEAD(module_list);
static int core_number;

/*******************************************************************************
 * FUNCTION: pt_probe_module
 *
 * SUMMARY: Add the module pointer to module_node and call the probe pointer.
 *
 * RETURN:
 *   0 = success
 *  !0 = failure
 *
 * PARAMETERS:
 *  *cd      - pointer to core data
 *  *module  - pointer to module structure
 ******************************************************************************/
static int pt_probe_module(struct pt_core_data *cd,
		struct pt_module *module)
{
	struct module_node *module_node;
	int rc = 0;

	module_node = kzalloc(sizeof(*module_node), GFP_KERNEL);
	if (!module_node)
		return -ENOMEM;

	module_node->module = module;

	mutex_lock(&cd->module_list_lock);
	list_add(&module_node->node, &cd->module_list);
	mutex_unlock(&cd->module_list_lock);

	rc = module->probe(cd->dev, &module_node->data);
	if (rc) {
		/*
		 * Remove from the list when probe fails
		 * in order not to call release
		 */
		mutex_lock(&cd->module_list_lock);
		list_del(&module_node->node);
		mutex_unlock(&cd->module_list_lock);
		kfree(module_node);
		goto exit;
	}

exit:
	return rc;
}

/*******************************************************************************
 * FUNCTION: pt_release_module
 *
 * SUMMARY: Call the release pointer and remove the module pointer from
 *  module_list.
 *
 * PARAMETERS:
 *  *cd      - pointer to core data
 *  *module  - pointer to module structure
 ******************************************************************************/
static void pt_release_module(struct pt_core_data *cd,
		struct pt_module *module)
{
	struct module_node *m, *m_n;

	mutex_lock(&cd->module_list_lock);
	list_for_each_entry_safe(m, m_n, &cd->module_list, node)
		if (m->module == module) {
			module->release(cd->dev, m->data);
			list_del(&m->node);
			kfree(m);
			break;
		}
	mutex_unlock(&cd->module_list_lock);
}

/*******************************************************************************
 * FUNCTION: pt_probe_modules
 *
 * SUMMARY: Iterate module_list and probe each module.
 *
 * PARAMETERS:
 *  *cd  - pointer to core data
 ******************************************************************************/
static void pt_probe_modules(struct pt_core_data *cd)
{
	struct pt_module *m;
	int rc = 0;

	mutex_lock(&module_list_lock);
	list_for_each_entry(m, &module_list, node) {
		pt_debug(cd->dev, DL_ERROR, "%s: Probe module %s\n",
			__func__, m->name);
		rc = pt_probe_module(cd, m);
		if (rc)
			pt_debug(cd->dev, DL_ERROR,
				"%s: Probe fails for module %s\n",
				__func__, m->name);
	}
	mutex_unlock(&module_list_lock);
}

/*******************************************************************************
 * FUNCTION: pt_release_modules
 *
 * SUMMARY: Iterate module_list and remove each module.
 *
 * PARAMETERS:
 *  *cd  - pointer to core data
 ******************************************************************************/
static void pt_release_modules(struct pt_core_data *cd)
{
	struct pt_module *m;

	mutex_lock(&module_list_lock);
	list_for_each_entry(m, &module_list, node)
		pt_release_module(cd, m);
	mutex_unlock(&module_list_lock);
}

/*******************************************************************************
 * FUNCTION: pt_get_core_data
 *
 * SUMMARY: Iterate core_list and get core data.
 *
 * RETURN:
 *   pointer to core data or null pointer if fail
 *
 * PARAMETERS:
 *  *id  - pointer to core id
 ******************************************************************************/
struct pt_core_data *pt_get_core_data(char *id)
{
	struct pt_core_data *d;

	list_for_each_entry(d, &core_list, node)
		if (!strncmp(d->core_id, id, 20))
			return d;
	return NULL;
}
EXPORT_SYMBOL_GPL(pt_get_core_data);

/*******************************************************************************
 * FUNCTION: pt_add_core
 *
 * SUMMARY: Add core data to the core_list.
 *
 * PARAMETERS:
 *  *dev  - pointer to device structure
 ******************************************************************************/
static void pt_add_core(struct device *dev)
{
	struct pt_core_data *d;
	struct pt_core_data *cd = dev_get_drvdata(dev);

	mutex_lock(&core_list_lock);
	list_for_each_entry(d, &core_list, node)
		if (d->dev == dev)
			goto unlock;

	list_add(&cd->node, &core_list);
unlock:
	mutex_unlock(&core_list_lock);
}

/*******************************************************************************
 * FUNCTION: pt_del_core
 *
 * SUMMARY: Remove core data from the core_list.
 *
 * PARAMETERS:
 *  *dev  - pointer to device structure
 ******************************************************************************/
static void pt_del_core(struct device *dev)
{
	struct pt_core_data *d, *d_n;

	mutex_lock(&core_list_lock);
	list_for_each_entry_safe(d, d_n, &core_list, node)
		if (d->dev == dev) {
			list_del(&d->node);
			goto unlock;
		}
unlock:
	mutex_unlock(&core_list_lock);
}

/*******************************************************************************
 * FUNCTION: pt_register_module
 *
 * SUMMARY: Register the module to module_list and probe the module for each
 *  core.
 *
 * RETURN:
 *   0 = success
 *  !0 = failure
 *
 * PARAMETERS:
 *  *module  - pointer to module structure
 ******************************************************************************/
int pt_register_module(struct pt_module *module)
{
	struct pt_module *m;
	struct pt_core_data *cd;

	int rc = 0;

	if (!module || !module->probe || !module->release)
		return -EINVAL;

	mutex_lock(&module_list_lock);
	list_for_each_entry(m, &module_list, node)
		if (m == module) {
			rc = -EEXIST;
			goto unlock;
		}

	list_add(&module->node, &module_list);

	/* Probe the module for each core */
	mutex_lock(&core_list_lock);
	list_for_each_entry(cd, &core_list, node)
		pt_probe_module(cd, module);
	mutex_unlock(&core_list_lock);

unlock:
	mutex_unlock(&module_list_lock);
	return rc;
}
EXPORT_SYMBOL_GPL(pt_register_module);

/*******************************************************************************
 * FUNCTION: pt_unregister_module
 *
 * SUMMARY: Release the module for each core and remove the module from
 *  module_list.
 *
 * RETURN:
 *   0 = success
 *  !0 = failure
 *
 * PARAMETERS:
 *  *module  - pointer to module structure
 ******************************************************************************/
void pt_unregister_module(struct pt_module *module)
{
	struct pt_module *m, *m_n;
	struct pt_core_data *cd;

	if (!module)
		return;

	mutex_lock(&module_list_lock);

	/* Release the module for each core */
	mutex_lock(&core_list_lock);
	list_for_each_entry(cd, &core_list, node)
		pt_release_module(cd, module);
	mutex_unlock(&core_list_lock);

	list_for_each_entry_safe(m, m_n, &module_list, node)
		if (m == module) {
			list_del(&m->node);
			break;
		}

	mutex_unlock(&module_list_lock);
}
EXPORT_SYMBOL_GPL(pt_unregister_module);

/*******************************************************************************
 * FUNCTION: pt_get_module_data
 *
 * SUMMARY: Get module data from module_node by module_list.
 *
 * RETURN:
 *   pointer to module data
 *
 * PARAMETERS:
 *  *dev     - pointer to device structure
 *  *module  - pointer to module structure
 ******************************************************************************/
void *pt_get_module_data(struct device *dev, struct pt_module *module)
{
	struct pt_core_data *cd = dev_get_drvdata(dev);
	struct module_node *m;
	void *data = NULL;

	mutex_lock(&cd->module_list_lock);
	list_for_each_entry(m, &cd->module_list, node)
		if (m->module == module) {
			data = m->data;
			break;
		}
	mutex_unlock(&cd->module_list_lock);

	return data;
}
EXPORT_SYMBOL(pt_get_module_data);

#ifdef CONFIG_HAS_EARLYSUSPEND
/*******************************************************************************
 * FUNCTION: pt_early_suspend
 *
 * SUMMARY: Android PM architecture function that will call "PT_ATTEN_SUSPEND"
 *  attention list.
 *
 * PARAMETERS:
 *  *h  - pointer to early_suspend structure
 ******************************************************************************/
static void pt_early_suspend(struct early_suspend *h)
{
	struct pt_core_data *cd =
		container_of(h, struct pt_core_data, es);

	call_atten_cb(cd, PT_ATTEN_SUSPEND, 0);
}

/*******************************************************************************
 * FUNCTION: pt_late_resume
 *
 * SUMMARY: Android PM architecture function that will call "PT_ATTEN_RESUME"
 *  attention list.
 *
 * PARAMETERS:
 *  *h  - pointer to early_suspend structure
 ******************************************************************************/
static void pt_late_resume(struct early_suspend *h)
{
	struct pt_core_data *cd =
		container_of(h, struct pt_core_data, es);

	call_atten_cb(cd, PT_ATTEN_RESUME, 0);
}

/*******************************************************************************
 * FUNCTION: pt_setup_early_suspend
 *
 * SUMMARY: Register early/suspend funtion to the system.
 *
 * PARAMETERS:
 *  *cd  - pointer to core data
 ******************************************************************************/
static void pt_setup_early_suspend(struct pt_core_data *cd)
{
	cd->es.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	cd->es.suspend = pt_early_suspend;
	cd->es.resume = pt_late_resume;

	register_early_suspend(&cd->es);
}
#elif defined(CONFIG_FB)
/*******************************************************************************
 * FUNCTION: fb_notifier_callback
 *
 * SUMMARY: Call back function for fb notifier to allow to call resume/suspend
 *  attention list.
 *
 * RETURN:
 *   0 = success
 *
 * PARAMETERS:
 *  *self   - pointer to notifier_block structure
 *   event  - event type of fb notifier
 *  *data   - pointer to fb_event structure
 ******************************************************************************/
static int fb_notifier_callback(struct notifier_block *self,
		unsigned long event, void *data)
{
	struct pt_core_data *cd =
		container_of(self, struct pt_core_data, fb_notifier);
	struct fb_event *evdata = data;
	int *blank;

	if (event != FB_EVENT_BLANK || !evdata)
		goto exit;

	blank = evdata->data;
	if (*blank == FB_BLANK_UNBLANK) {
		pt_debug(cd->dev, DL_INFO, "%s: UNBLANK!\n", __func__);
		if (cd->fb_state != FB_ON) {
			call_atten_cb(cd, PT_ATTEN_RESUME, 0);
			cd->fb_state = FB_ON;
		}
	} else if (*blank == FB_BLANK_POWERDOWN) {
		pt_debug(cd->dev, DL_INFO, "%s: POWERDOWN!\n", __func__);
		if (cd->fb_state != FB_OFF) {
			call_atten_cb(cd, PT_ATTEN_SUSPEND, 0);
			cd->fb_state = FB_OFF;
		}
	}

exit:
	return 0;
}

/*******************************************************************************
 * FUNCTION: pt_setup_fb_notifier
 *
 * SUMMARY: Set up call back function into fb notifier.
 *
 * PARAMETERS:
 *  *cd   - pointer to core data
 ******************************************************************************/
static void pt_setup_fb_notifier(struct pt_core_data *cd)
{
	int rc;

	cd->fb_state = FB_ON;

	cd->fb_notifier.notifier_call = fb_notifier_callback;

	rc = fb_register_client(&cd->fb_notifier);
	if (rc)
		pt_debug(cd->dev, DL_ERROR,
			"Unable to register fb_notifier: %d\n", rc);
}
#endif

/*
 * These functions are only needed to support the Parade Technologies
 * Development platform.
 */
#ifdef PT_PTSBC_SUPPORT
/*******************************************************************************
 * FUNCTION: pt_irq_work_function
 *
 * SUMMARY: Work function for queued IRQ activity
 *
 * RETURN: Void
 *
 * PARAMETERS:
 *	*work - pointer to work structure
 ******************************************************************************/
static void pt_irq_work_function(struct work_struct *work)
{
	struct pt_core_data *cd = container_of(work,
			struct pt_core_data, irq_work);

	pt_irq(cd->irq, (void *)cd);
}

/*******************************************************************************
 * FUNCTION: pt_irq_wrapper
 *
 * SUMMARY: Wrapper function for IRQ to queue the irq_work function
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*handle - void pointer to contain the core_data pointer
 ******************************************************************************/
peint_handle *pt_irq_wrapper(void *handle)
{
	struct pt_core_data *cd = (struct pt_core_data *)handle;

	queue_work(parade_wq, &cd->irq_work);
	return 0;
}
#endif /* --- End PT_PTSBC_SUPPORT --- */


/*******************************************************************************
 * FUNCTION: pt_setup_irq_gpio
 *
 * SUMMARY: Configure the IRQ GPIO used by the TT DUT
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *      *cd - pointer to core data
 ******************************************************************************/
static int pt_setup_irq_gpio(struct pt_core_data *cd)
{
	struct device *dev = cd->dev;
	unsigned long irq_flags;
	int rc = 0;
#ifdef PT_PTSBC_SUPPORT
	u32 int_handle;
#endif

	/*
	 * When TTDL has direct access to the GPIO the irq_stat function will
	 * be defined and the gpio_to_irq conversion must be performed.
	 * e.g. For CHROMEOS this is not the case, the irq is passed in directly
	 */
	if (cd->cpdata->irq_stat) {
		/* Initialize IRQ */
		dev_vdbg(dev, "%s: Value Passed to gpio_to_irq =%d\n", __func__,
			cd->cpdata->irq_gpio);
		cd->irq = gpio_to_irq(cd->cpdata->irq_gpio);
		dev_vdbg(dev, "%s: Value Returned from gpio_to_irq =%d\n",
			__func__, cd->irq);
	}
	if (cd->irq < 0)
		return -EINVAL;

	cd->irq_enabled = true;

	pt_debug(dev, DL_INFO, "%s: initialize threaded irq=%d\n",
		__func__, cd->irq);

	if (cd->cpdata->level_irq_udelay > 0)
#ifdef PT_PTSBC_SUPPORT
		/* TODO Move irq_flag retrival to platform.c */
		/* use level triggered interrupts */
		irq_flags = TRIG_LEVL_LOW;
	else
		/* use edge triggered interrupts */
		irq_flags = TRIG_EDGE_NEGATIVE;
#else
		/* use level triggered interrupts */
		irq_flags = IRQF_TRIGGER_LOW | IRQF_ONESHOT;
	else
		/* use edge triggered interrupts */
		irq_flags = IRQF_TRIGGER_FALLING | IRQF_ONESHOT;
#endif

#ifdef PT_PTSBC_SUPPORT
	parade_wq = create_singlethread_workqueue("parade_wq");
	if (!parade_wq)
		pt_debug(dev, DL_ERROR, "%s Create workqueue failed.\n",
			__func__);

	int_handle = sw_gpio_irq_request(cd->cpdata->irq_gpio, irq_flags,
			(peint_handle)pt_irq_wrapper, cd);
	if (!int_handle) {
		pt_debug(dev, DL_ERROR, "%s: PARADE could not request irq\n",
			__func__);
		rc = -1;
	} else {
		rc = 0;
		ctp_set_int_port_rate(cd->cpdata->irq_gpio, 1);
		/* Debounce INT Line */
		ctp_set_int_port_deb(cd->cpdata->irq_gpio, 0x07);
		pt_debug(cd->dev, DL_INFO,
			"%s: Parade sw_gpio_irq_request SUCCESS\n", __func__);
	}
#else
	rc = request_threaded_irq(cd->irq, NULL, pt_irq, irq_flags,
		dev_name(dev), cd);
	if (rc < 0)
		pt_debug(dev, DL_ERROR, "%s: Error, could not request irq\n",
			__func__);
#endif

	return rc;
}

/*******************************************************************************
 * FUNCTION: pt_watchdog_work
 *
 * SUMMARY: This is where the watchdog work is done except if the DUT is
 *	sleeping then this function simply returns. If the DUT is awake the
 *	first thing is to ensure the IRQ is not stuck asserted meaning that
 *	somehow a response is waiting on the DUT that has not been read. If
 *	this occurs the message is simply consumed. If or once the IRQ is
 *	cleared, a PIP PING message is sent to the DUT and if the response
 *	is recieved the watchdog succeeds and exits, if no response is seen
 *	a startup is queued unless the maximum number of startups have already
 *	been attempted, in that case a BL is attempted.
 *
 *	NOTE: If the TTDL_DIAGNOSTICS compile flag is enabled, several IRQ
 *	      based counters are active.
 *	NOTE: pt_stop_wd_timer() cannot be called within the context of this
 *	      work thread
 *
 * RETURN: void
 *
 * PARAMETERS:
 *      *work - pointer to a work structure for the watchdog work queue
 ******************************************************************************/
static void pt_watchdog_work(struct work_struct *work)
{
	int rc;
	struct pt_core_data *cd = container_of(work,
				struct pt_core_data, watchdog_work);

	/*
	 * if found the current sleep_state is SS_SLEEPING
	 * then no need to request_exclusive, directly return
	 */
	if (cd->sleep_state == SS_SLEEPING)
		return;

#ifdef TTDL_DIAGNOSTICS
	cd->watchdog_count++;
#endif /* TTDL_DIAGNOSTICS */

	/*
	 * The first WD interval was extended to allow DDI to come up.
	 * If the WD interval is not the default then adjust timer to the
	 * current setting. The user can override value with drv_debug sysfs.
	 */
	if (cd->watchdog_interval != PT_WATCHDOG_TIMEOUT) {
		mod_timer(&cd->watchdog_timer, jiffies +
			msecs_to_jiffies(cd->watchdog_interval));
	}

	if (cd->cpdata->irq_stat &&
	    cd->cpdata->irq_stat(cd->cpdata, cd->dev) ==
		PT_IRQ_ASSERTED_VALUE) {
		pt_debug(cd->dev, DL_WARN,
			"%s: TTDL WD found IRQ asserted, attempt to clear\n",
			__func__);
#ifdef TTDL_DIAGNOSTICS
		cd->watchdog_irq_stuck_count++;
		pt_toggle_err_gpio(cd);
#endif /* TTDL_DIAGNOSTICS */
		if (cd->wd_corrective_action == 1) {
			/* Do a flush based on the 2 byte read length */
			pt_flush_i2c(cd, 0);
		}
	}

	rc = request_exclusive(cd, cd->dev, PT_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		pt_debug(cd->dev, DL_ERROR,
				"%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->dev);
		goto queue_startup;
	}

	rc = pt_pip_null_(cd);

	if (release_exclusive(cd, cd->dev) < 0)
		pt_debug(cd->dev, DL_ERROR,
				"%s: fail to release exclusive\n", __func__);

queue_startup:
	if (rc) {
#ifdef TTDL_DIAGNOSTICS
		cd->watchdog_failed_access_count++;
		pt_toggle_err_gpio(cd);
#endif /* TTDL_DIAGNOSTICS */
		pt_debug(cd->dev, DL_ERROR,
			"%s: failed to access device in WD, retry count=%d\n",
			__func__, cd->startup_retry_count);

		/* Already tried FW upgrade because of watchdog but failed */
		if (cd->startup_retry_count > PT_WATCHDOG_RETRY_COUNT)
			return;

		if (cd->startup_retry_count++ < PT_WATCHDOG_RETRY_COUNT) {
			/*
			 * Do a direct xres to avoid any wrapper function
			 * that trys to disable the WD which would cancel this
			 * worker thread
			 */
			if (cd->wd_corrective_action && cd->cpdata->xres) {
				cd->hw_detected = false;
				cd->startup_status = STARTUP_STATUS_START;
#ifdef TTDL_DIAGNOSTICS
				cd->wd_xres_count++;
				pt_debug(cd->dev, DL_WARN,
					"%s: Comm Failed - DUT reset [#%d]\n",
					__func__, cd->wd_xres_count);
#endif /* TTDL_DIAGNOSTICS */
				cd->cpdata->xres(cd->cpdata, cd->dev);
			}
		} else {
			/*
			 * After trying PT_WATCHDOG_RETRY_COUNT times to
			 * reset the part to regain communications, try to BL
			 */
			pt_debug(cd->dev, DL_WARN,
				"%s: WD DUT access failure, Start FW Upgrade\n",
				__func__);
#ifdef TTDL_DIAGNOSTICS
			/*
			 * When diagnostics is enabled allow TTDL to keep
			 * trying to find the DUT. This allows the DUT to be
			 * hot swap-able while the host stays running. In
			 * production this may not be wanted as a customer
			 * may have several touch drivers and any driver
			 * that doesn't match the current DUT should give
			 * up trying and give up using the bus.
			 */
			pt_debug(cd->dev, DL_WARN,
				"%s: Resetting startup_retry_count\n",
				__func__);
			cd->startup_retry_count = 0;
#endif /* TTDL_DIAGNOSTICS */
			/*
			 * Since fw may be broken,reset sysinfo ready flag
			 * to let upgrade function work.
			 */
			mutex_lock(&cd->system_lock);
			cd->sysinfo.ready = false;
			mutex_unlock(&cd->system_lock);
			kthread_run(start_fw_upgrade, cd, "pt_loader");
		}
	} else {
		cd->hw_detected = true;
		if (cd->startup_status <= (STARTUP_STATUS_FW_RESET_SENTINEL |
		    STARTUP_STATUS_BL_RESET_SENTINEL)) {
			pt_debug(cd->dev, DL_WARN,
				"%s: HW detected but not enumerated\n",
				__func__);
			pt_queue_startup(cd);
		}
	}
	pt_start_wd_timer(cd);
}

/*******************************************************************************
 * FUNCTION: pt_watchdog_timer
 *
 * SUMMARY: The function that is called when the WD timer expires. If the
 *	watchdog work is not already busy schedule the watchdog work queue.
 *
 * RETURN: void
 *
 * PARAMETERS:
 *      handle - Handle to the watchdog timer
 ******************************************************************************/
static void pt_watchdog_timer(unsigned long handle)
{
	struct pt_core_data *cd = (struct pt_core_data *)handle;

	if (!cd)
		return;

	pt_debug(cd->dev, DL_DEBUG, "%s: Watchdog timer triggered\n",
		__func__);

	if (!work_pending(&cd->watchdog_work))
		schedule_work(&cd->watchdog_work);
}

#ifdef PT_PTSBC_SUPPORT
/* Required to support the Parade Techologies Development Platform */
static int pt_probe_complete(struct pt_core_data *cd);

/*******************************************************************************
 * FUNCTION: pt_probe_work
 *
 * SUMMARY: For the PtSBC the probe functionality is split into two functions;
 *	pt_probe() and pt_probe_complete() which is called from here.
 *	This function is scheduled as a "work" task in order to launch after
 *	I2C is up.
 *
 * RETURN: Void
 *
 * PARAMETERS:
 *	*work - pointer to work structure
 ******************************************************************************/
static void pt_probe_work(struct work_struct *work)
{
	struct pt_core_data *cd =
			container_of(work, struct pt_core_data,
					probe_work);
	int rc;

	rc = pt_probe_complete(cd);

	if (rc < 0)
		pr_err("%s: Probe_complete returns rc=%d\n", __func__, rc);
	else
		pt_debug(cd->dev, DL_INFO,
			"%s: Probe_complete returns rc=%d\n", __func__, rc);
}

/*******************************************************************************
 * FUNCTION: pt_probe_timer
 *
 * SUMMARY: For the PtSBC the probe functionality is split into two functions;
 *	pt_probe() and pt_probe_complete(). This timer shedules the
 *	probe_work function.
 *
 * RETURN: Void
 *
 * PARAMETERS:
 *	handle - pointer to the core data
 ******************************************************************************/
static void pt_probe_timer(unsigned long handle)
{
	struct pt_core_data *cd = (struct pt_core_data *)handle;

	if (!cd)
		return;

	pt_debug(cd->dev, DL_INFO, "%s: Watchdog timer triggered\n",
		__func__);

	if (!work_pending(&cd->probe_work))
		schedule_work(&cd->probe_work);
}
#endif /* --- End PT_PTSBC_SUPPORT --- */



/*******************************************************************************
 * Core sysfs show and store functions
 ******************************************************************************/

#ifdef TTDL_DIAGNOSTICS
/*******************************************************************************
 * FUNCTION: pt_err_gpio_show
 *
 * SUMMARY: Show method for the err_gpio sysfs node that will show if
 *	setting up the gpio was successful
 *
 * RETURN: Char buffer with printed GPIO creation state
 *
 * PARAMETERS:
 *	*dev  - pointer to device structure
 *	*attr - pointer to device attributes
 *	*buf  - pointer to output buffer
 ******************************************************************************/
static ssize_t pt_err_gpio_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct pt_core_data *cd = dev_get_drvdata(dev);

	return sprintf(buf, "Err GPIO %d: %s\n", cd->err_gpio,
		(cd->err_gpio ? (gpio_get_value(cd->err_gpio) ?
		"HIGH" : "low") : "not defined"));
}

/*******************************************************************************
 * FUNCTION: pt_err_gpio_store
 *
 * SUMMARY: The store method for the err_gpio sysfs node that allows any
 *	available host GPIO to be used to trigger when TTDL detects a PIP
 *	command/response timeout.
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*dev  - pointer to device structure
 *	*attr - pointer to device attributes
 *	*buf  - pointer to buffer that hold the command parameters
 *	 size - size of buf
 ******************************************************************************/
static ssize_t pt_err_gpio_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct pt_core_data *cd = dev_get_drvdata(dev);
	unsigned long value;
	int rc = 0;

	rc = kstrtoul(buf, 10, &value);
	if (rc < 0) {
		pt_debug(dev, DL_ERROR, "%s: Invalid value\n", __func__);
		goto exit;
	}

	mutex_lock(&cd->system_lock);
	gpio_free(value);
	rc = gpio_request(value, NULL);
	if (rc) {
		pt_debug(dev, DL_ERROR, "error requesting gpio %lu\n", value);
	} else {
		cd->err_gpio = value;
		gpio_direction_output(value, 0);
	}
	mutex_unlock(&cd->system_lock);

exit:
	return size;
}
static DEVICE_ATTR(err_gpio, S_IRUGO | S_IWUSR, pt_err_gpio_show,
	pt_err_gpio_store);
#endif /* TTDL_DIAGNOSTICS */

/*******************************************************************************
 * FUNCTION: pt_ic_ver_show
 *
 * SUMMARY: Show method for the ic_ver sysfs node that will show the
 *	firmware, bootloader and PIP version information
 *
 * RETURN: Size of printed buffer
 *
 * PARAMETERS:
 *	*dev  - pointer to device structure
 *	*attr - pointer to device attributes
 *	*buf  - pointer to output buffer
 ******************************************************************************/
static ssize_t pt_ic_ver_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct pt_core_data *cd = dev_get_drvdata(dev);
	struct pt_ttdata *ttdata = &cd->sysinfo.ttdata;

	return sprintf(buf,
		"%s: 0x%02X\n"
		"%s: 0x%02X\n"
		"%s: %d\n"
		"%s: 0x%04X\n"
		"%s: 0x%02X\n"
		"%s: 0x%02X\n"
		"%s: 0x%02X\n"
		"%s: 0x%02X\n",
		"Firmware Major Version        ", ttdata->fw_ver_major,
		"Firmware Minor Version        ", ttdata->fw_ver_minor,
		"Revision Control Number       ", ttdata->revctrl,
		"Firmware Configuration Version", ttdata->fw_ver_conf,
		"Bootloader Major Version      ", ttdata->bl_ver_major,
		"Bootloader Minor Version      ", ttdata->bl_ver_minor,
		"Protocol Major Version        ", ttdata->pip_ver_major,
		"Protocol Minor Version        ", ttdata->pip_ver_minor);
}

/*******************************************************************************
 * FUNCTION: pt_drv_ver_show
 *
 * SUMMARY: Show method for the drv_ver sysfs node that will show the
 *	TTDL version information
 *
 * RETURN: Char buffer with printed TTDL version information
 *
 * PARAMETERS:
 *	*dev  - pointer to device structure
 *	*attr - pointer to device attributes
 *	*buf  - pointer to output buffer
 ******************************************************************************/
static ssize_t pt_drv_ver_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PT_MAX_PRBUF_SIZE,
		"Driver: %s\nVersion: %s\nDate: %s\n",
		pt_driver_core_name, pt_driver_core_version,
		pt_driver_core_date);
}
static DEVICE_ATTR(drv_ver, S_IRUGO, pt_drv_ver_show, NULL);

/*******************************************************************************
 * FUNCTION: pt_hw_reset_store
 *
 * SUMMARY: The store method for the hw_reset sysfs node that does a hw reset
 *	by toggling the XRES line and then calls the startup function to
 *	allow TTDL to re-enumerate the DUT.
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*dev  - pointer to device structure
 *	*attr - pointer to device attributes
 *	*buf  - pointer to buffer that hold the command parameters
 *	 size - size of buf
 ******************************************************************************/
static ssize_t pt_hw_reset_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct pt_core_data *cd = dev_get_drvdata(dev);
	int rc;

	rc = pt_dut_reset(cd, PT_CORE_CMD_PROTECTED);
	if (rc < 0)
		goto error_reset;

	msleep(300);

	rc = pt_startup(cd, true);
	if (rc < 0)
		goto error_reset;

	pt_start_wd_timer(cd);
	return size;
error_reset:
	pt_debug(cd->dev, DL_ERROR,
		"%s: HW reset failed rc = %d\n", __func__, rc);
	return -1;
}

#ifdef TTDL_DIAGNOSTICS
/*******************************************************************************
 * FUNCTION: pt_hw_irq_stat_show
 *
 * SUMMARY: Show method for the hw_irq_stat sysfs node that will show the
 *	IRQ GPIO pin state
 *
 * RETURN: Size of printed buffer
 *
 * PARAMETERS:
 *	*dev  - pointer to device structure
 *	*attr - pointer to device attributes
 *	*buf  - pointer to output buffer
 ******************************************************************************/
static ssize_t pt_hw_irq_stat_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct pt_core_data *cd = dev_get_drvdata(dev);
	int retval;

	if (cd->cpdata->irq_stat) {
		retval = cd->cpdata->irq_stat(cd->cpdata, dev);
		switch (retval) {
		case 0:
			return snprintf(buf, PT_MAX_PRBUF_SIZE,
				"Interrupt line is LOW.\n");
		case 1:
			return snprintf(buf, PT_MAX_PRBUF_SIZE,
				"Interrupt line is HIGH.\n");
		default:
			return snprintf(buf, PT_MAX_PRBUF_SIZE,
				"Function irq_stat() returned %d.\n", retval);
		}
	}

	return snprintf(buf, PT_MAX_PRBUF_SIZE,
		"Function irq_stat() undefined.\n");
}

/*******************************************************************************
 * FUNCTION: pt_drv_irq_show
 *
 * SUMMARY: Show method for the drv_irq sysfs node that will show if the
 *	TTDL interrupt is enabled/disabled
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*dev  - pointer to device structure
 *	*attr - pointer to device attributes
 *	*buf  - pointer to output buffer
 ******************************************************************************/
static ssize_t pt_drv_irq_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct pt_core_data *cd = dev_get_drvdata(dev);
	ssize_t ret;

	mutex_lock(&cd->system_lock);
	if (cd->irq_enabled)
		ret = snprintf(buf, PT_MAX_PRBUF_SIZE,
			"Driver interrupt is ENABLED\n");
	else
		ret = snprintf(buf, PT_MAX_PRBUF_SIZE,
			"Driver interrupt is DISABLED\n");
	mutex_unlock(&cd->system_lock);

	return ret;
}

/*******************************************************************************
 * FUNCTION: pt_drv_irq_store
 *
 * SUMMARY: The store method for the drv_irq sysfs node that allows the TTDL
 *	IRQ to be enabled/disabled.
 *
 * RETURN: Size of passed in buffer
 *
 * PARAMETERS:
 *	*dev  - pointer to device structure
 *	*attr - pointer to device attributes
 *	*buf  - pointer to buffer that hold the command parameters
 *	 size - size of buf
 ******************************************************************************/
static ssize_t pt_drv_irq_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct pt_core_data *cd = dev_get_drvdata(dev);
	unsigned long value;
	int retval = 0;

	retval = kstrtoul(buf, 10, &value);
	if (retval < 0) {
		pt_debug(dev, DL_ERROR, "%s: Invalid value\n", __func__);
		goto pt_drv_irq_store_error_exit;
	}

	mutex_lock(&cd->system_lock);
	switch (value) {
	case 0:
		if (cd->irq_enabled) {
			cd->irq_enabled = false;
			/* Disable IRQ */
			disable_irq_nosync(cd->irq);
			pt_debug(dev, DL_INFO,
				"%s: Driver IRQ now disabled\n",
				__func__);
		} else
			pt_debug(dev, DL_INFO,
				"%s: Driver IRQ already disabled\n",
				__func__);
		break;

	case 1:
		if (cd->irq_enabled == false) {
			cd->irq_enabled = true;
			/* Enable IRQ */
			enable_irq(cd->irq);
			pt_debug(dev, DL_INFO,
				"%s: Driver IRQ now enabled\n",
				__func__);
		} else
			pt_debug(dev, DL_INFO,
				"%s: Driver IRQ already enabled\n",
				__func__);
		break;

	default:
		pt_debug(dev, DL_ERROR, "%s: Invalid value\n", __func__);
	}
	mutex_unlock(&(cd->system_lock));

pt_drv_irq_store_error_exit:

	return size;
}
#endif /* TTDL_DIAGNOSTICS */

/*******************************************************************************
 * FUNCTION: pt_command_store
 *
 * SUMMARY: This is the store method for the raw PIP command sysfs node. Any
 *	raw PIP command echo'd to this node will be sent directly to the DUT.
 *	TTDL will not parse the command.
 *
 * RETURN: Size of passed in buffer
 *
 * PARAMETERS:
 *	*dev  - pointer to device structure
 *	*attr - pointer to device attributes
 *	*buf  - pointer to buffer that hold the command parameters
 *	 size - size of buf
 ******************************************************************************/
static ssize_t pt_command_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct pt_core_data *cd = dev_get_drvdata(dev);
	u16 actual_read_len;
	u8 input_data[256];
	int length;
	int rc;

	mutex_lock(&cd->sysfs_lock);
	cd->raw_cmd_status   = 0;
	cd->response_buf_len = 0;
	mutex_unlock(&cd->sysfs_lock);

	length = pt_ic_parse_input_hex(dev, buf, size,
			input_data, PT_MAX_PRBUF_SIZE);
	if (length <= 0) {
		pt_debug(dev, DL_ERROR, "%s: %s failed\n", __func__,
				"pt_ic_parse_input_dec");
		goto pt_command_store_exit;
	}

	/* write PIP command to log */
	pt_pr_buf(dev, DL_INFO, input_data, length, "command_buf");

	pm_runtime_get_sync(dev);
	rc = pt_hid_output_user_cmd(cd, PT_MAX_INPUT, cd->response_buf,
		length, input_data, &actual_read_len);
	pm_runtime_put(dev);

	mutex_lock(&cd->sysfs_lock);
	if (rc) {
		cd->response_buf_len = 0;
		pt_debug(dev, DL_ERROR, "%s: Failed to send command\n",
			__func__);
	} else {
		cd->response_buf_len = actual_read_len;
		cd->raw_cmd_status = 1;
	}
	mutex_unlock(&cd->sysfs_lock);

pt_command_store_exit:
	return size;
}
static DEVICE_ATTR(command, S_IWUSR, NULL, pt_command_store);


/*******************************************************************************
 * FUNCTION: pt_response_show
 *
 * SUMMARY: The show method for the raw PIP response sysfs node. Any PIP
 *	response generated after using the pt_command_store sysfs node, are
 *	available to be read here.
 *
 * PARAMETERS:
 *      *dev  - pointer to Device structure
 *      *attr - pointer to the device attribute structure
 *      *buf  - pointer to buffer to print
 ******************************************************************************/
static ssize_t pt_response_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct pt_core_data *cd = dev_get_drvdata(dev);
	int i;
	ssize_t num_read;
	int index;

	mutex_lock(&cd->sysfs_lock);
	index = scnprintf(buf, PT_MAX_PRBUF_SIZE,
		"Status %d\n", cd->raw_cmd_status);
	if (!cd->raw_cmd_status)
		goto error;

	num_read = cd->response_buf_len;
	for (i = 0; i < num_read; i++)
		index += scnprintf(buf + index, PT_MAX_PRBUF_SIZE - index,
			"0x%02X\n", cd->response_buf[i]);

	index += scnprintf(buf + index, PT_MAX_PRBUF_SIZE - index,
		"(%zd bytes)\n", num_read);

error:
	mutex_unlock(&cd->sysfs_lock);
	return index;
}
static DEVICE_ATTR(response, S_IRUGO, pt_response_show, NULL);

/*******************************************************************************
 * FUNCTION: pt_status_show
 *
 * SUMMARY: The show method for the raw PIP status sysfs node. This node
 *	displays the status of any PIP response being ready in the response
 *	sysfs node
 *
 * PARAMETERS:
 *      *dev  - pointer to Device structure
 *      *attr - pointer to the device attribute structure
 *      *buf  - pointer to buffer to print
 ******************************************************************************/
static ssize_t pt_status_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct pt_core_data *cd = dev_get_drvdata(dev);
	u8 val;

	mutex_lock(&cd->sysfs_lock);
	val = cd->raw_cmd_status;
	mutex_unlock(&cd->sysfs_lock);

	return scnprintf(buf, PT_MAX_PRBUF_SIZE, "%d\n", val);
}
static DEVICE_ATTR(status, S_IRUGO, pt_status_show, NULL);

/*******************************************************************************
 * FUNCTION: pt_drv_debug_store
 *
 * SUMMARY: Currently the store method for both sysfs nodes: drv_debug and
 *	dut_debug. Drv_debug will contain all functionality that can be run
 *	without a DUT preset and is available anytime TTDL is running.
 *	Dut_debug requires a DUT to be available and will only be created after
 *	a DUT has been detected.
 *	This funciton will eventually be split into two but until the overlap
 *	has been depricated this funciton contains all commands that can be
 *	used for TTDL/DUT debugging status and control.
 *	All commands require at least one value to be passed in *buf with some
 *	requiring two.
 *
 * RETURN: Size of passed in buffer
 *
 * PARAMETERS:
 *	*dev  - pointer to device structure
 *	*attr - pointer to device attributes
 *	*buf  - pointer to buffer that hold the command parameters
 *	 size - size of buf
 ******************************************************************************/
static ssize_t pt_drv_debug_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct pt_core_data *cd = dev_get_drvdata(dev);
	unsigned long value;
	int rc;
	u8 return_data[8];
	static u8 wd_disabled;
	u32 input_data[2];
	int length;

	input_data[0] = 0;
	input_data[1] = 0;

	/* Maximmum input is two elements */
	length = pt_ic_parse_input_dec(dev, buf, size,
			input_data, 3);
	if (length <= 0) {
		pt_debug(dev, DL_ERROR, "%s: %s failed\n", __func__,
				"pt_ic_parse_input_dec");
		goto pt_drv_debug_store_exit;
	}
	value = input_data[0];

	if (length == 1) {
		pt_debug(dev, DL_DEBUG,
			"%s: Debug Cmd Recieved (id=%d)\n",
			__func__, input_data[0]);
	} else if (length == 2) {
		pt_debug(dev, DL_DEBUG,
			"%s: Debug Cmd Recieved (id=%d, data=%d)\n",
			__func__, input_data[0], input_data[1]);
	} else {
		pt_debug(dev, DL_DEBUG,
			"%s: Invalid arguments recieved\n", __func__);
		goto pt_drv_debug_store_exit;
	}

	/* Start watchdog timer command */
	if (value == PT_DRV_DBG_START_WD) {
		pt_debug(dev, DL_INFO, "%s: Cmd: Start Watchdog\n", __func__);
		wd_disabled = 0;
		cd->watchdog_force_stop = false;
		pt_start_wd_timer(cd);
		goto pt_drv_debug_store_exit;
	}

	/* Stop watchdog timer temporarily */
	pt_stop_wd_timer(cd);

	if (value == PT_DRV_DBG_STOP_WD) {
		pt_debug(dev, DL_INFO, "%s: Cmd: Stop Watchdog\n", __func__);
		wd_disabled = 1;
		cd->watchdog_force_stop = true;
		goto pt_drv_debug_store_exit;
	}

	switch (value) {
	case PT_DUT_DBG_HID_RESET:
		pt_debug(dev, DL_INFO, "%s: Cmd: hid_reset\n", __func__);
		pt_hid_cmd_reset(cd);
		break;
	case PT_DUT_DBG_HID_SET_POWER_ON:
		pt_debug(dev, DL_INFO, "%s: Cmd: hid_set_power_on\n", __func__);
		pt_hid_cmd_set_power(cd, HID_POWER_ON);
		wd_disabled = 0;
		break;
	case PT_DUT_DBG_HID_SET_POWER_SLEEP:
		pt_debug(dev, DL_INFO, "%s: Cmd: hid_set_power_off\n",
			 __func__);
		wd_disabled = 1;
		pt_hid_cmd_set_power(cd, HID_POWER_SLEEP);
		break;
	case PT_DUT_DBG_SOFT_RESET:
		pt_debug(dev, DL_INFO, "%s: Cmd: Soft Reset\n", __func__);
		rc = pt_hw_soft_reset(cd, PT_CORE_CMD_PROTECTED);
		break;
	case PT_DUT_DBG_RESET:
		pt_debug(dev, DL_INFO, "%s: Cmd: Hard Reset\n", __func__);
		rc = pt_hw_hard_reset(cd);
		break;
	case PT_DUT_DBG_HID_NULL:
		pt_debug(dev, DL_INFO, "%s: Cmd: Ping (null)\n", __func__);
		pt_pip_null(cd);
		break;
	case PT_DUT_DBG_HID_ENTER_BL:
		pt_debug(dev, DL_INFO, "%s: Cmd: start_bootloader\n", __func__);
		pt_pip_start_bootloader(cd);
		break;
	case PT_DUT_DBG_HID_SYSINFO:
		pt_debug(dev, DL_INFO, "%s: Cmd: get_sysinfo\n", __func__);
		pt_hid_output_get_sysinfo(cd);
		break;
	case PT_DUT_DBG_HID_SUSPEND_SCAN:
		pt_debug(dev, DL_INFO, "%s: Cmd: suspend_scanning\n", __func__);
		pt_pip_suspend_scanning(cd);
		break;
	case PT_DUT_DBG_HID_RESUME_SCAN:
		pt_debug(dev, DL_INFO, "%s: Cmd: resume_scanning\n", __func__);
		pt_pip_resume_scanning(cd);
		break;
	case PT_DUT_DBG_HID_DESC:
		pt_debug(dev, DL_INFO, "%s: Cmd: get_hid_desc\n", __func__);
		pt_get_hid_descriptor(cd, &cd->hid_desc);
		break;

	case HID_OUTPUT_BL_VERIFY_APP_INTEGRITY:
		pt_debug(dev, DL_INFO, "%s: Cmd: verify app integ\n", __func__);
		pt_hid_output_bl_verify_app_integrity(cd, &return_data[0]);
		break;
	case HID_OUTPUT_BL_GET_INFO:
		pt_debug(dev, DL_INFO, "%s: Cmd: bl get info\n", __func__);
		pt_hid_output_bl_get_information(cd, return_data);
		break;
	case HID_OUTPUT_BL_PROGRAM_AND_VERIFY:
		pt_debug(dev, DL_INFO, "%s: Cmd: program and verify\n",
			__func__);
		pt_hid_output_bl_program_and_verify(cd, 0, NULL);
		break;
	case HID_OUTPUT_BL_LAUNCH_APP:
		pt_debug(dev, DL_INFO, "%s: Cmd: launch app\n", __func__);
		pt_hid_output_bl_launch_app(cd);
		break;
	case HID_OUTPUT_BL_INITIATE_BL:
		pt_debug(dev, DL_INFO, "%s: Cmd: initiate bl\n", __func__);
		pt_hid_output_bl_initiate_bl(cd, 0, NULL, 0, NULL);
		break;

#ifdef TTHE_TUNER_SUPPORT
	case PT_DRV_DBG_TTHE_TUNER_EXIT:
		cd->tthe_exit = 1;
		wake_up(&cd->wait_q);
		kfree(cd->tthe_buf);
		cd->tthe_buf = NULL;
		cd->tthe_exit = 0;
		break;
	case PT_DRV_DBG_TTHE_BUF_CLEAN:
		if (cd->tthe_buf)
			memset(cd->tthe_buf, 0, PT_MAX_PRBUF_SIZE);
		else
			pt_debug(dev, DL_INFO, "%s : tthe_buf not existed\n",
				__func__);
		break;
#endif
	case PT_DRV_DBG_SUSPEND:
		pt_debug(dev, DL_INFO, "%s: TTDL: Core Sleep\n", __func__);
		rc = pt_core_sleep(cd);
		if (rc)
			pt_debug(dev, DL_ERROR, "%s: Suspend failed rc=%d\n",
				__func__, rc);
		else
			pt_debug(dev, DL_INFO, "%s: Suspend succeeded\n",
				__func__);
		break;

	case PT_DRV_DBG_RESUME:
		pt_debug(dev, DL_INFO, "%s: TTDL: Wake\n", __func__);
		rc = pt_core_wake(cd);
		if (rc)
			pt_debug(dev, DL_ERROR, "%s: Resume failed rc=%d\n",
				__func__, rc);
		else
			pt_debug(dev, DL_INFO, "%s: Resume succeeded\n",
				__func__);
		break;
	case PT_DRV_DBG_REPORT_LEVEL:
		mutex_lock(&cd->system_lock);
		if (input_data[1] < DL_MAX) {
			cd->debug_level = input_data[1];
			pt_debug(dev, DL_INFO, "%s: Set debug_level: %d\n",
				__func__, cd->debug_level);
		} else
			pt_debug(dev, DL_ERROR, "%s: Invalid debug_level: %d\n",
				__func__, input_data[1]);
		mutex_unlock(&(cd->system_lock));
		break;
	case PT_DRV_DBG_WATCHDOG_INTERVAL:
		mutex_lock(&cd->system_lock);
		if (input_data[1] < 100) {
			pt_debug(dev, DL_ERROR,
				"%s: WARNING: Invalid watchdog interval: %d\n",
				__func__, input_data[1]);
		} else {
			cd->watchdog_interval = input_data[1];
			pt_start_wd_timer(cd);
		}
		mutex_unlock(&(cd->system_lock));
		pt_debug(dev, DL_INFO, "%s: Watchdog interval Set to: %d\n",
			__func__, cd->watchdog_interval);
		break;
#ifdef TTDL_DIAGNOSTICS
	case PT_DRV_DBG_CLEAR_PARM_LIST:
		pt_debug(dev, DL_INFO, "%s: TTDL: Clear Parameter List\n",
			__func__);
		pt_erase_parameter_list(cd);
		break;
	case PT_DRV_DBG_SHOW_TIMESTAMP:
		mutex_lock(&cd->system_lock);
		cd->show_timestamp = input_data[1];
		pt_debug(dev, DL_INFO, "%s: TTDL: Set show_timestamp: %d\n",
			__func__, cd->show_timestamp);
		mutex_unlock(&(cd->system_lock));
		break;
	case PT_DRV_DBG_WD_CORRECTIVE_ACTION:
		mutex_lock(&cd->system_lock);
		cd->wd_corrective_action = input_data[1] == 0 ? 0 : 1;
		pt_debug(dev, DL_WARN,
			"%s: TTDL: wd_corrective_action: %d\n",
			__func__, cd->wd_corrective_action);
		mutex_unlock(&(cd->system_lock));
		break;
	case PT_DRV_DBG_VIRTUAL_I2C_DUT:
		mutex_lock(&cd->system_lock);
		cd->route_i2c_virt_dut = input_data[1] == 0 ? 0 : 1;
		pt_debug(dev, DL_WARN,
			"%s: TTDL: route_i2c_virt_dut: %d\n",
			__func__, cd->route_i2c_virt_dut);
		mutex_unlock(&(cd->system_lock));
		break;
	case PT_DRV_DBG_FLUSH_I2C_BUS:
		mutex_lock(&cd->system_lock);
		pt_debug(dev, DL_INFO, "%s: TTDL: Flush I2C Bus\n", __func__);
		if (input_data[1] == 0)
			pt_flush_i2c(cd, 0);
		else
			pt_flush_i2c(cd, 1);
		mutex_unlock(&(cd->system_lock));
		break;
#endif /* TTDL_DIAGNOSTICS */
	default:
		pt_debug(dev, DL_ERROR, "%s: Invalid value\n", __func__);
	}

	/* Enable watchdog timer */
	if (!wd_disabled)
		pt_start_wd_timer(cd);

pt_drv_debug_store_exit:
	return size;
}
static DEVICE_ATTR(drv_debug,  S_IWUSR, NULL, pt_drv_debug_store);

#ifdef TTDL_DIAGNOSTICS
/*******************************************************************************
 * FUNCTION: pt_pip2_enter_bl_show
 *
 * SUMMARY: Show method for the pip2_enter_bl sysfs node that will force
 *	the DUT into the BL and show the success or failure of entering the BL
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*dev  - pointer to device structure
 *	*attr - pointer to device attributes
 *	*buf  - pointer to output buffer
 ******************************************************************************/
static ssize_t pt_pip2_enter_bl_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	int rc;
	u8 mode;
	u8 mode_str[10];

	rc = _pt_request_pip2_enter_bl(dev, &mode);
	switch (rc) {
	case PT_ENTER_BL_PASS:
		if (mode == PT_MODE_OPERATIONAL)
			strcpy(mode_str, "APP");
		else if (mode == PT_MODE_BOOTLOADER)
			strcpy(mode_str, "BL");
		else
			strcpy(mode_str, "unknown");

		ret = snprintf(buf, PT_MAX_PRBUF_SIZE,
			"STATUS: %d\nSuccess, entered BL from %s mode\n",
			PT_ENTER_BL_PASS, mode_str);
		break;
	case PT_ENTER_BL_ERROR:
		ret = snprintf(buf, PT_MAX_PRBUF_SIZE, "STATUS: %d\n%s\n",
			PT_ENTER_BL_ERROR,
			"  Unknown Error");
		break;
	case PT_ENTER_BL_RESET_FAIL:
		ret = snprintf(buf, PT_MAX_PRBUF_SIZE, "STATUS: %d\n%s\n",
			PT_ENTER_BL_RESET_FAIL,
			"  Soft Reset Failed");
		break;
	case PT_ENTER_BL_HID_START_BL_FAIL:
		ret = snprintf(buf, PT_MAX_PRBUF_SIZE, "STATUS: %d\n%s\n",
			PT_ENTER_BL_HID_START_BL_FAIL,
			"  PIP Start BL Cmd Failed");
		break;
	case PT_ENTER_BL_CONFIRM_FAIL:
		ret = snprintf(buf, PT_MAX_PRBUF_SIZE, "STATUS: %d\n%s\n",
			PT_ENTER_BL_CONFIRM_FAIL,
			"  Error confirming DUT entered BL");
		break;
	default:
		ret = snprintf(buf, PT_MAX_PRBUF_SIZE, "STATUS: %d\n%s\n",
			rc, "  Unknown Error");
		break;
	};
	return ret;
}
#endif /* TTDL_DIAGNOSTICS */

/*******************************************************************************
 * FUNCTION: pt_pip2_bin_hdr_show
 *
 * SUMMARY: Show method for the pip2_bin_hdr sysfs node that will read
 *	the bin file header from flash and show each field
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*dev  - pointer to device structure
 *	*attr - pointer to device attributes
 *	*buf  - pointer to output buffer
 ******************************************************************************/
static ssize_t pt_pip2_bin_hdr_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	struct pt_bin_file_hdr hdr;
	int rc;

	rc = _pt_request_pip2_bin_hdr(dev, &hdr);
	ret = sprintf(buf,
		"%s %d\n"
		"%s: %d\n"
		"%s: 0x%04X\n"
		"%s: 0x%02X\n"
		"%s: 0x%02X\n"
		"%s: %d\n"
		"%s: 0x%04X\n"
		"%s: 0x%04X\n"
		"%s: 0x%04X\n",
		"status", rc,
		"Header Length  ", rc ? 0 : hdr.length,
		"TTPID          ", rc ? 0 : hdr.ttpid,
		"FW Major Ver   ", rc ? 0 : hdr.fw_major,
		"FW Minor Ver   ", rc ? 0 : hdr.fw_minor,
		"FW Rev Control ", rc ? 0 : hdr.fw_rev_ctrl,
		"Silicon Rev    ", rc ? 0 : hdr.si_rev,
		"Silicon ID     ", rc ? 0 : hdr.si_id,
		"Config Ver     ", rc ? 0 : hdr.config_ver
	);
	return ret;
}
static DEVICE_ATTR(pip2_bin_hdr,  S_IRUGO, pt_pip2_bin_hdr_show, NULL);

/*******************************************************************************
 * FUNCTION: pt_sleep_status_show
 *
 * SUMMARY: Show method for the sleep_status sysfs node that will show the
 *	sleep status as either ON or OFF
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*dev  - pointer to device structure
 *	*attr - pointer to device attributes
 *	*buf  - pointer to output buffer
 ******************************************************************************/
static ssize_t pt_sleep_status_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct pt_core_data *cd = dev_get_drvdata(dev);
	ssize_t ret;

	mutex_lock(&cd->system_lock);
	if (cd->sleep_state == SS_SLEEP_ON)
		ret = snprintf(buf, PT_MAX_PRBUF_SIZE, "off\n");
	else
		ret = snprintf(buf, PT_MAX_PRBUF_SIZE, "on\n");
	mutex_unlock(&cd->system_lock);

	return ret;
}

/*******************************************************************************
 * FUNCTION: pt_easy_wakeup_gesture_show
 *
 * SUMMARY: Show method for the easy_wakeup_gesture sysfs node that will show
 *	current easy wakeup gesture
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*dev  - pointer to device structure
 *	*attr - pointer to device attributes
 *	*buf  - pointer to output buffer
 ******************************************************************************/
static ssize_t pt_easy_wakeup_gesture_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct pt_core_data *cd = dev_get_drvdata(dev);
	ssize_t ret;

	mutex_lock(&cd->system_lock);
	ret = snprintf(buf, PT_MAX_PRBUF_SIZE, "0x%02X\n",
			cd->easy_wakeup_gesture);
	mutex_unlock(&cd->system_lock);
	return ret;
}

/*******************************************************************************
 * FUNCTION: pt_easy_wakeup_gesture_store
 *
 * SUMMARY: The store method for the easy_wakeup_gesture sysfs node that
 *	allows the wake gesture to be set to a custom value.
 *
 * RETURN: Size of passed in buffer is success
 *
 * PARAMETERS:
 *	*dev  - pointer to device structure
 *	*attr - pointer to device attributes
 *	*buf  - pointer to buffer that hold the command parameters
 *	 size - size of buf
 ******************************************************************************/
static ssize_t pt_easy_wakeup_gesture_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct pt_core_data *cd = dev_get_drvdata(dev);
	unsigned long value;
	int ret;

	pt_debug(dev, DL_INFO, "%s: features.easywake = 0x%02X\n",
		__func__, cd->features.easywake);

	if (!cd->features.easywake)
		return -EINVAL;

	ret = kstrtoul(buf, 10, &value);
	if (ret < 0)
		return ret;

	if (value > 0xFF)
		return -EINVAL;

	pm_runtime_get_sync(dev);

	mutex_lock(&cd->system_lock);
	if (cd->sysinfo.ready && IS_PIP_VER_GE(&cd->sysinfo, 1, 2)) {
		cd->easy_wakeup_gesture = (u8)value;
		pt_debug(dev, DL_INFO,
			"%s: Updated easy_wakeup_gesture = 0x%02X\n",
			__func__, cd->easy_wakeup_gesture);
	} else
		ret = -ENODEV;
	mutex_unlock(&cd->system_lock);

	pm_runtime_put(dev);

	if (ret)
		return ret;

	return size;
}

#ifdef EASYWAKE_TSG6
/*******************************************************************************
 * FUNCTION: pt_easy_wakeup_gesture_id_show
 *
 * SUMMARY: Show method for the easy_wakeup_gesture_id sysfs node that will
 *	show the TSG6 easywake gesture ID
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*dev  - pointer to device structure
 *	*attr - pointer to device attributes
 *	*buf  - pointer to output buffer
 ******************************************************************************/
static ssize_t pt_easy_wakeup_gesture_id_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct pt_core_data *cd = dev_get_drvdata(dev);
	ssize_t ret;

	mutex_lock(&cd->system_lock);
	ret = snprintf(buf, PT_MAX_PRBUF_SIZE, "0x%02X\n",
			cd->gesture_id);
	mutex_unlock(&cd->system_lock);
	return ret;
}

/*******************************************************************************
 * FUNCTION: pt_easy_wakeup_gesture_data_show
 *
 * SUMMARY: Show method for the easy_wakeup_gesture_data sysfs node that will
 *	show the TSG6 easywake gesture data in the following format:
 *	x1(LSB),x1(MSB), y1(LSB),y1(MSB), x2(LSB),x2(MSB), y2(LSB),y2(MSB),...
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*dev  - pointer to device structure
 *	*attr - pointer to device attributes
 *	*buf  - pointer to output buffer
 ******************************************************************************/
static ssize_t pt_easy_wakeup_gesture_data_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct pt_core_data *cd = dev_get_drvdata(dev);
	ssize_t ret = 0;
	int i;

	mutex_lock(&cd->system_lock);

	for (i = 0; i < cd->gesture_data_length; i++)
		ret += snprintf(buf + ret, PT_MAX_PRBUF_SIZE - ret,
				"0x%02X\n", cd->gesture_data[i]);

	ret += snprintf(buf + ret, PT_MAX_PRBUF_SIZE - ret,
			"(%d bytes)\n", cd->gesture_data_length);

	mutex_unlock(&cd->system_lock);
	return ret;
}
#endif /* EASYWAKE_TSG6 */

/*******************************************************************************
 * FUNCTION: pt_panel_id_show
 *
 * SUMMARY: Show method for the panel_id sysfs node that will show the
 *	detected panel ID from the DUT
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*dev  - pointer to device structure
 *	*attr - pointer to device attributes
 *	*buf  - pointer to output buffer
 ******************************************************************************/
static ssize_t pt_panel_id_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct pt_core_data *cd = dev_get_drvdata(dev);
	ssize_t ret;

	ret = snprintf(buf, PT_MAX_PRBUF_SIZE, "0x%02X\n",
			cd->panel_id);
	return ret;
}

/*******************************************************************************
 * FUNCTION: pt_platform_data_show
 *
 * SUMMARY: Show method for the platform_data sysfs node that will show the
 *	active platform data including: GPIOs, Vendor and Product IDs,
 *	Virtual Key coordinates, Core/MT/Loader flags, Level trigger delay,
 *	HID registers, and Easy wake gesture
 *
 * RETURN: Size of printed data
 *
 * PARAMETERS:
 *	*dev  - pointer to device structure
 *	*attr - pointer to device attributes
 *	*buf  - pointer to output buffer
 ******************************************************************************/
static ssize_t pt_platform_data_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct pt_platform_data *pdata = dev_get_platdata(dev);
	struct pt_core_data *cd = dev_get_drvdata(dev);
	ssize_t ret;

	ret = sprintf(buf,
		"%s: %d\n"
		"%s: %d\n"
		"%s: %d\n"
		"%s: %d\n"
		"%s: %d\n"
		"%s: %d\n"
		"%s: %d\n"
		"%s: %d\n"
		"%s: %d\n"
		"%s: %d\n"
		"%s: %d\n"
		"%s: %d\n"
		"%s: %d\n"
		"%s: %d\n"
		"%s: %d\n",
		"Interrupt GPIO           ", pdata->core_pdata->irq_gpio,
		"Reset GPIO               ", pdata->core_pdata->rst_gpio,
		"DDI Reset GPIO           ", pdata->core_pdata->ddi_rst_gpio,
		"Run FW GPIO              ", pdata->core_pdata->runfw_gpio,
		"Vendor ID                ", pdata->core_pdata->vendor_id,
		"Product ID               ", pdata->core_pdata->product_id,
		"Vkeys x                  ", pdata->mt_pdata->vkeys_x,
		"Vkeys y                  ", pdata->mt_pdata->vkeys_y,
		"Core data flags          ", pdata->core_pdata->flags,
		"MT data flags            ", pdata->mt_pdata->flags,
		"Loader data flags        ", pdata->loader_pdata->flags,
		"Level trigger delay (us) ",
			pdata->core_pdata->level_irq_udelay,
		"HID Descriptor Register  ",
			pdata->core_pdata->hid_desc_register,
		"HID Command Register     ",
			cd->hid_desc.command_register,
		"Easy wakeup gesture      ",
			pdata->core_pdata->easy_wakeup_gesture);
	return ret;
}

/*******************************************************************************
 * FUNCTION: pt_get_param_store
 *
 * SUMMARY: Store method for the get_param sysfs node. Stores what parameter
 *	ID to retrieve with the show method.
 *
 * NOTE: This sysfs node is only available after a DUT has been enumerated
 *
 * RETURN: Size of passed in buffer if successful
 *
 * PARAMETERS:
 *	*dev  - pointer to device structure
 *	*attr - pointer to device attributes
 *	*buf  - pointer to output buffer
 *	 size - size of buf
 ******************************************************************************/
static ssize_t pt_get_param_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct pt_core_data *cd = dev_get_drvdata(dev);
	u32 input_data[1];
	int length;

	/* Maximum input of one value */
	length = pt_ic_parse_input_dec(dev, buf, size, input_data, 2);
	if (length <= 0 || length > 1) {
		pt_debug(dev, DL_ERROR, "%s: %s failed, parm count error\n",
			__func__, "pt_ic_parse_input_dec");
	}

	mutex_lock(&cd->system_lock);
	cd->get_param_id = input_data[0];
	mutex_unlock(&(cd->system_lock));

	return size;
}

/*******************************************************************************
 * FUNCTION: pt_get_param_show
 *
 * SUMMARY: Show method for the get_param sysfs node. Retrieves the
 *	parameter data from the DUT based on the ID stored in the core
 *	data variable "get_param_id". If the ID is invalid, the DUT cannot
 *	communicate or some other error occures, an error status is returned
 *	with no value following.
 *	Output is in the form:
 *	   status x
 *	   yyyyyyyy
 *	The 32bit data will only follow the status code if the status == 0
 *
 * NOTE: This sysfs node is only available after a DUT has been enumerated
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*dev  - pointer to device structure
 *	*attr - pointer to device attributes
 *	*buf  - pointer to output buffer
 ******************************************************************************/
static ssize_t pt_get_param_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct  pt_core_data *cd = dev_get_drvdata(dev);
	ssize_t ret = 0;
	u8      status;
	u32     value = 0;

	status = pt_pip_get_param(cd, cd->get_param_id, &value);
	if (status) {
		pt_debug(dev, DL_ERROR, "%s: %s Failed, status = %d\n",
			__func__, "pt_get_param", status);
		ret = sprintf(buf,
			"%s %d\n",
			"status", status);
	} else {
		pt_debug(dev, DL_DEBUG, "%s: Param [%d] = 0x%04X\n",
			__func__, cd->get_param_id, value);
		ret = sprintf(buf,
			"%s %d\n"
			"%04X\n",
			"status", status,
			value);
	}
	return ret;
}

#ifdef TTDL_DIAGNOSTICS
/*******************************************************************************
 * FUNCTION: pt_t_refresh_store
 *
 * SUMMARY: Store method for the t-refresh sysfs node that will takes a passed
 *	in integer as the number of interrupts to count. A timer is started to
 *	calculate the total time it takes to see that number of interrupts.
 *
 * RETURN: Size of passed in buffer if successful
 *
 * PARAMETERS:
 *	*dev  - pointer to device structure
 *	*attr - pointer to device attributes
 *	*buf  - pointer to buffer that hold the command parameters
 *	 size - size of buf
 ******************************************************************************/
static ssize_t pt_t_refresh_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct pt_core_data *cd = dev_get_drvdata(dev);
	unsigned long value;
	int retval = 0;

	mutex_lock(&cd->system_lock);

	retval = kstrtoul(buf, 10, &value);
	if (retval >= 0) {
		pt_debug(dev, DL_WARN, "%s: Input value: %lu\n",
			__func__, value);
		if (value <= 1000) {
			cd->t_refresh_total = value;
		} else {
			cd->t_refresh_total = 1000;
			pt_debug(dev, DL_ERROR,
				"%s: Invalid value, default to 1000\n",
				__func__);
		}
		cd->t_refresh_count  = 0;
		cd->t_refresh_active = 1;
	} else {
		pt_debug(dev, DL_ERROR, "%s: Invalid value\n", __func__);
	}
	mutex_unlock(&cd->system_lock);
	return size;
}
#endif /* TTDL_DIAGNOSTICS */

/*******************************************************************************
 * FUNCTION: pt_ttdl_restart_store
 *
 * SUMMARY: Store method for ttdl_restart sysfs node. This node releases all
 *	probed modules, calls startup() and then re-probes modules.
 *
 * RETURN: Size of passed in buffer if successful
 *
 * PARAMETERS:
 *      *dev  - pointer to core device
 *	*attr - pointer to attributes
 *	*buf  - pointer to cmd input buffer
 *	 size - size of buf
 ******************************************************************************/
static ssize_t pt_ttdl_restart_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	u32 input_data[2];
	int length;

	/* maximum input of one value */
	length = pt_ic_parse_input_dec(dev, buf, size, input_data, 2);
	if (length <= 0) {
		pt_debug(dev, DL_ERROR, "%s: %s failed\n", __func__,
				"pt_ic_parse_input_dec");
	}
	if (_pt_ttdl_restart(dev) != 0)
		pt_debug(dev, DL_ERROR, "%s: %s failed\n", __func__,
			"_pt_ttdl_restart()");

	return size;
}
static DEVICE_ATTR(ttdl_restart, S_IWUSR, NULL, pt_ttdl_restart_store);

#ifdef TTDL_DIAGNOSTICS
/*******************************************************************************
 * FUNCTION: pt_ttdl_bist_show
 *
 * SUMMARY: Show method for the ttdl_bist sysfs node. This built in self test
 *	will test that the TP_XRES, IRQ and RUN_FW pins are operational.
 *
 *	NOTE: This function will reset the DUT and the startup_status bit
 *	mask. A full ttdl_restart() will be called after completion.
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*dev  - pointer to device structure
 *	*attr - pointer to device attributes
 *	*buf  - pointer to output buffer
 ******************************************************************************/
static ssize_t pt_ttdl_bist_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct pt_core_data *cd = dev_get_drvdata(dev);
	struct pip2_cmd_structure pip2_cmd;
	ssize_t ret;
	u16 actual_read_len;
	u8 read_buf[256];
	u8 irq_err_str[64];
	u8 xres_err_str[64];
	u8 runfw_err_str[64];
	bool bl_detected_runfw_set = false;
	int t;
	int rc;
	int count         = 0;
	int status        = 1;    /* 0 = Pass, !0 = fail */
	int bytes_read    = 0;
	u8 irq_toggled    = 0x0F; /* default to untested */
	u8 xres_toggled   = 0x0F; /* default to untested */
	u8 runfw_toggled  = 0x0F; /* default to untested */

	/*
	 * ----- Initialize TTDL to begin BIST -----
	 */

	memset(irq_err_str, 0, sizeof(irq_err_str));
	memset(xres_err_str, 0, sizeof(xres_err_str));
	memset(runfw_err_str, 0, sizeof(runfw_err_str));

	/* Turn off the TTDL WD during the test */
	pt_stop_wd_timer(cd);

	/* Clear the startup bit mask, reset and restart will re-populate it */
	cd->startup_status = STARTUP_STATUS_START;

	/*
	 * --------------- IRQ BIST TEST ---------------
	 * This test will ensure there is a good connection between the host
	 * and the DUT on the irq pin. First determine if the IRQ is stuck
	 * asserted and if so keep reading messages off of the bus until
	 * it de-asserts. Possible outcomes:
	 *  - IRQ was already de-asserted: Send a PIP command and if an
	 *	interrupt is generated the test passes.
	 *  - IRQ was asserted: Reading off the bus de-assertes the IRQ,
	 *	test passes.
	 *  - IRQ stays asserted: After reading the bus multiple times
	 *	the IRQ stays asserted. Likely open or shorted to GND
	 */
	pt_debug(dev, DL_INFO, "%s: ----- Start IRQ BIST -----", __func__);

	/* Clear IRQ triggered count, and re-evaluate at the end of test */
	cd->irq_count = 0;

	/*
	 * Check if IRQ is stuck asserted, if so try a non forced flush of
	 * the I2C bus based on the 2 byte initial length read. Try up to 5x.
	 */
	while (pt_check_irq_asserted(cd) && count < 5) {
		count++;
		bytes_read += pt_flush_i2c(cd, 0);
	}
	if (count > 1 && bytes_read > 0) {
		/*
		 * IRQ was stuck but data was successfully read from the I2C
		 * bus which means the IRQ line must have triggered.
		 */
		pt_debug(dev, DL_INFO, "%s: count=%d bytes_read=%d\n",
			__func__, count, bytes_read);
		irq_toggled = 1;
		goto bist_tp_xres;
	}
	if (count == 5 && bytes_read == 0) {
		/*
		 * Looped 5x and read nothing off the bus yet the IRQ is still
		 * asserted - IRQ could be open circuit or shorted to GND, no
		 * point going any further as the other tests require the IRQ
		 */
		strcpy(irq_err_str, "- likely shorted to GND.");
		pt_debug(dev, DL_ERROR, "%s: %s, count=%d bytes_read=%d\n",
			__func__, irq_err_str, count, bytes_read);
		irq_toggled = 0;
		goto bist_tp_xres;
	}

	/* Try sending a PIP command to see if we get a response */
	rc = _pt_request_pip2_send_cmd(dev, PT_CORE_CMD_PROTECTED, &pip2_cmd,
		PIP2_CMD_ID_VERSION, NULL, 0, read_buf, &actual_read_len);
	if (rc) {
		/*
		 * Potential IRQ issue, but could also be bad FW not
		 * responding. Save the error string but the XRES test below
		 * may show the IRQ is actually working.
		 */
		strcpy(irq_err_str, "- likely open or shorted to VDDI.");
		pt_debug(dev, DL_ERROR, "%s: request_active_pip_prot failed\n",
			__func__);
		irq_toggled = 0;
		goto bist_tp_xres;
	}
	if (cd->irq_count > 0) {
		pt_debug(dev, DL_INFO, "%s: irq_count=%d\n",
			__func__, cd->irq_count);
		irq_toggled = 1;
	}

bist_tp_xres:
	/*
	 * --------------- TP_XRES BIST TEST ---------------
	 * This test will ensure there is a good connection between the host
	 * and the DUT on the tp_xres pin. The pin will be toggled to
	 * generate a TP reset which will cause the DUT to output a reset
	 * sentinel. If the reset sentinel is seen the test passes. If it is
	 * not seen the test will attempt to send a soft reset to simply gain
	 * some additonal information on the failure:
	 *  - soft reset fails to send: XRES and IRQ likely open
	 *  - soft reset passes: XRES likely open or stuck de-asserted
	 *  - soft reset fails: XRES likely stuck asserted
	 */
	pt_debug(dev, DL_INFO, "%s: ----- Start TP_XRES BIST -----", __func__);

	/* Ensure we have nothing pending on the i2c bus */
	while (pt_check_irq_asserted(cd) && count < 5) {
		count++;
		bytes_read += pt_flush_i2c(cd, 0);
	}

	/* Perform a hard XRES toggle and wait for reset sentinel */
	mutex_lock(&cd->system_lock);
	cd->startup_status = STARTUP_STATUS_START;
	cd->hid_reset_cmd_state = 1;
	mutex_unlock(&cd->system_lock);
	pt_debug(dev, DL_INFO, "%s: Perform a hard reset\n", __func__);
	rc = pt_hw_hard_reset(cd);

	t = wait_event_timeout(cd->wait_q,
		(cd->startup_status != STARTUP_STATUS_START),
		msecs_to_jiffies(500));
	if (IS_TMO(t)) {
		pt_debug(cd->dev, DL_ERROR,
			"%s: TMO waiting for DUT enumeration\n", __func__);
		strcpy(xres_err_str, "- likely open. (No Reset Sentinel)");
		goto print_results;
	}

	pt_debug(dev, DL_INFO, "%s: TP_XRES BIST hard reset rc=%d",
		__func__, rc);
	cd->hid_reset_cmd_state = 0;

	/* Look for BL or FW reset sentinels */
	if (!rc && ((cd->startup_status & STARTUP_STATUS_BL_RESET_SENTINEL) ||
	    (cd->startup_status & STARTUP_STATUS_FW_RESET_SENTINEL))) {
		pt_debug(dev, DL_INFO, "%s: hard XRES pass\n", __func__);
		xres_toggled = 1;

		/*
		 * There are cases where the IRQ test above failed because of
		 * bad FW in the DUT. In this case the IRQ net could actually
		 * be functioning and once the DUT is in the BL it toggles when
		 * the reset sentinal is received. If this is the case, set the
		 * pin as 'pass' and clear any error string.
		 */
		irq_toggled = 1;
		memset(irq_err_str, 0, sizeof(irq_err_str));
	} else {
		/*
		 * Hard reset failed, however some additional information
		 * could be determined. Try a soft reset to see if DUT resets
		 * with the possible outcomes:
		 * - if it resets the line is not stuck asserted
		 * - if it does not reset the line could be stuck asserted
		 */
		xres_toggled = 0;
		rc = pt_hw_soft_reset(cd, PT_CORE_CMD_PROTECTED);
		msleep(30);
		pt_debug(dev, DL_INFO, "%s: TP_XRES BIST soft reset rc=%d",
			__func__, rc);
		if (rc) {
			strcpy(xres_err_str, "- likely open.");
			pt_debug(dev, DL_ERROR,
				"%s: Hard reset failed, soft reset failed %s\n",
				__func__, xres_err_str);
			goto print_results;
		}
		if (cd->startup_status & STARTUP_STATUS_BL_RESET_SENTINEL ||
		    cd->startup_status & STARTUP_STATUS_FW_RESET_SENTINEL) {
			strcpy(xres_err_str,
				"- likely open or stuck high, soft reset OK");
			pt_debug(dev, DL_ERROR,
				"%s: Hard reset failed, soft reset passed-%s\n",
				__func__, xres_err_str);
		} else if (cd->startup_status == 0) {
			strcpy(xres_err_str, "- likely stuck high.");
			pt_debug(dev, DL_ERROR,
				"%s: Hard reset failed, soft reset failed-%s\n",
				__func__, xres_err_str);
		} else {
			strcpy(xres_err_str, "- open or stuck.");
			pt_debug(dev, DL_ERROR,
				"%s: Hard and Soft reset failed - %s\n",
				__func__, xres_err_str);
		}
	}

	/*
	 * --------------- RUNFW BIST TEST ---------------
	 * This test will ensure there is a good connection between the host
	 * and the DUT on the RunFW pin. The pin will be asserted and
	 * then a reset will be triggered. If the connection is good the DUT
	 * will remain in the BL mode and the PIP2 command GET_LAST_ERRNO will
	 * respond that the BL saw the correct state of the RunFW pin. The
	 * pin will then be de-asserted and the DUT will be reset again. Now
	 * if there is no valid FW in FLASH the DUT will remain in the BL and
	 * sending the GET_LAST_ERRNO again should show the RunFW pin is in
	 * the de-asserted state, however if there is valid FW in FLASH the DUT
	 * will boot into flash and if we see the FW Reset Sentinel we also
	 * know the pin correctly toggled.
	 */
	pt_debug(dev, DL_INFO, "%s: ----- Start RUN_FW BIST ----", __func__);

	/* Ensure we have nothing pending on the i2c bus */
	while (pt_check_irq_asserted(cd) && count < 5) {
		count++;
		bytes_read += pt_flush_i2c(cd, 0);
	}

	/* Assert the RunFW pin to force BL to stay in the BL ROM */
	_pt_request_set_runfw_pin(dev, 0);

	/* Reset the DUT and then verify we stay in the BL */
	mutex_lock(&cd->system_lock);
	cd->startup_status = STARTUP_STATUS_START;
	mutex_unlock(&cd->system_lock);
	rc = pt_dut_reset(cd, PT_CORE_CMD_UNPROTECTED);
	t = wait_event_timeout(cd->wait_q,
		(cd->startup_status != STARTUP_STATUS_START),
		msecs_to_jiffies(500));
	if (IS_TMO(t)) {
		pt_debug(cd->dev, DL_ERROR,
			"%s: TMO waiting for DUT enumeration\n", __func__);
		strcpy(xres_err_str, "- likely open. (No Reset Sentinel)");
		goto print_results;
	}

	pt_debug(dev, DL_INFO, "%s: RUN_FW BIST reset rc=%d", __func__, rc);
	pt_debug(dev, DL_INFO, "%s: Startup_status = 0x%04X\n", __func__,
		cd->startup_status);

	/*
	 * If either of the BL or FW sentinels are seen after the reset, the
	 * IRQ net must be working. In some rare cases the IRQ test above
	 * could have failed but this test shows it is actually working.
	 * This does not prove the XRES is working as the reset could have
	 * occured do to a soft reset.
	 */
	if (cd->startup_status & STARTUP_STATUS_FW_RESET_SENTINEL ||
	    cd->startup_status & STARTUP_STATUS_BL_RESET_SENTINEL) {
		irq_toggled  = 1;
		memset(irq_err_str, 0, sizeof(irq_err_str));
	}

	/*
	 * Should be in the BL now, however, test if the FW reset sentinel
	 * was seen instead of the BL reset sentinel.
	 */
	if (cd->startup_status & STARTUP_STATUS_FW_RESET_SENTINEL) {
		strcpy(runfw_err_str, "- likely open or stuck high.");
		pt_debug(dev, DL_ERROR,
			"%s: Not in BL when expected to be, Run_FW %s\n",
			__func__, runfw_err_str);
		runfw_toggled = 0;
		goto exit_bl;
	} else if (cd->startup_status & STARTUP_STATUS_BL_RESET_SENTINEL) {
		/*
		 * Confirmed we are in the BL but need to ensure it is because
		 * the BL saw the RunFW pin asserted. Send the PIP2
		 * GET_LAST_ERRNO command and parse the response to verify
		 * the BL saw the RunFW pin low.
		 */
		rc = _pt_request_pip2_send_cmd(dev,
			PT_CORE_CMD_PROTECTED, &pip2_cmd,
			PIP2_CMD_ID_GET_LAST_ERRNO, NULL, 0, read_buf,
			&actual_read_len);
		if (rc) {
			/*
			 * Shouldn't get here - Command failed to send.
			 * Somehow we are either not in the BL or the IRQ is
			 * not working when the RunFW pin is low.
			 */
			strcpy(runfw_err_str,
				"- likely open or stuck high.");
			pt_debug(dev, DL_ERROR,
				"%s: BL cmd failed to send, Run_FW %s\n",
				__func__, runfw_err_str);
			runfw_toggled = 0;

			/* No need to exit BL, goto print results directly */
			goto exit_bl;
		}
		pt_debug(dev, DL_INFO, "%s: BootMode pin state = %d\n",
			__func__, read_buf[6]);

		/*
		 * The GET_LAST_ERRNO command produced a response. Test to
		 * see if a valid response with the bootmode byte set to 0
		 * showing the RunFW pin was seen asserted
		 */
		if (read_buf[4] == 0 && read_buf[6] == 0x00) {
			bl_detected_runfw_set = true;
			pt_debug(dev, DL_WARN,
				"%s: BL detected Run_FW aserted\n",
				__func__);
		} else {
			strcpy(runfw_err_str,
				"- likely open or stuck high.");
			pt_debug(dev, DL_ERROR,
				"%s: Bootmode bit not low when BL entered, RUN_FW %s\n",
				__func__, runfw_err_str);
			runfw_toggled = 0;
			goto exit_bl;
		}
	} else {
		pt_debug(dev, DL_ERROR,
			"%s: No BL or FW sentinel seen after XRES\n", __func__);
		runfw_toggled = 0;
		/*
		 * Neither the BL or FW sentinel was seen after the XRES. This
		 * state should not be seen unless the XRES or IRQ nets are
		 * not working.
		 * In rare cases the RunFW and IRQ pins could be shorted
		 * causing the IRQ to stay low while the RunFW pin was low.
		 * This results in no response from the DUT being read as the
		 * IRQ never releases. To test for this case the following
		 * extra test is performed:
		 * 1) Test to see the IRQ is low (maybe due to RunRW being low)
		 * 2) Force a I2C flush to clear any pending response
		 * 3) Test to see if the IRQ is now high, if so no short
		 * 4) Release the RunFW pin
		 * 5) Force a I2C flush to clear any pending response
		 * 6) Test to see the IRQ is now high
		 */
		if (pt_check_irq_asserted(cd)) {
			pt_debug(dev, DL_ERROR,
				"%s: IRQ found low when RunFW low\n",
				__func__);
			pt_flush_i2c(cd, 1);
			if (!pt_check_irq_asserted(cd)) {
				pt_debug(dev, DL_WARN,
					"%s: IRQ released after flush\n",
					__func__);
				goto exit_bl;
			}
			_pt_request_set_runfw_pin(dev, 0);
			pt_flush_i2c(cd, 1);
			if (pt_check_irq_asserted(cd)) {
				strcpy(runfw_err_str,
					"- likely shorted to IRQ.");
				pt_debug(dev, DL_ERROR,
					"%s: IRQ released with RunFW %s\n",
					__func__, runfw_err_str);
			} else {
				strcpy(runfw_err_str,
					"- likely open or shorted to IRQ.");
				pt_debug(dev, DL_ERROR,
					"%s: IRQ did not release with RunFW\n",
					__func__);
			}
		} else {
			strcpy(runfw_err_str,
				"- likely open or shorted to IRQ.");
			pt_debug(dev, DL_ERROR,
				"%s: IRQ found high when RunFW low\n",
				__func__);
		}
		goto exit_bl;
	}

	/* De-assert the RunFW pin to allow the BL to try and load FW */
	_pt_request_set_runfw_pin(dev, 1);
	mutex_lock(&cd->system_lock);
	cd->startup_status = STARTUP_STATUS_START;
	mutex_unlock(&cd->system_lock);

	/* The DUT is in the BL so send a BL RESET command to reset the DUT */
	rc = _pt_request_pip2_send_cmd(dev, PT_CORE_CMD_PROTECTED,
		&pip2_cmd, PIP2_CMD_ID_RESET, NULL, 0, read_buf,
		&actual_read_len);
	if (rc) {
		pt_debug(dev, DL_ERROR, "%s: Sending BL RESET cmd failed\n",
			__func__);
		/* RESET cmd failed so as a last ditch effort reset the part */
		rc = pt_dut_reset(cd, PT_CORE_CMD_UNPROTECTED);
	}
	t = wait_event_timeout(cd->wait_q,
		(cd->startup_status != STARTUP_STATUS_START),
		msecs_to_jiffies(500));
	if (IS_TMO(t)) {
		pt_debug(cd->dev, DL_ERROR,
			"%s: TMO waiting for DUT enumeration\n", __func__);
		strcpy(xres_err_str, "- likely open. (No Reset Sentinel)");
		goto print_results;
	}


	/*
	 * Four possible outcomes:
	 * 1) There is no FW in the FLASH we will be back in the BL
	 * 2) There is valid FW we will be running FW
	 * 3) There is FW in the flash that loads but does not run
	 * 4) The RunFW and IRQ nets are shorted which should have been
	 *    caught above.
	 */

	pt_debug(dev, DL_INFO, "%s: Startup_status = 0x%04X\n", __func__,
		cd->startup_status);

	/*
	 * Test if we are back running FW. Only the FW sentinel will arrive
	 * as the BL sentinel is only produced if the BL doesn not execute FW
	 */
	if (cd->startup_status & STARTUP_STATUS_FW_RESET_SENTINEL &&
	    bl_detected_runfw_set) {
		/* In the FW so the RunFW pin must have been de-asserted */
		runfw_toggled = 1;
		/* Do a clean exit to have TTDL re-enum with the DUT */
		goto exit_bl;
	}

	/* Test if we are back in the boot loader */
	if (cd->startup_status & STARTUP_STATUS_BL_RESET_SENTINEL) {
		/* Send GET_LAST_ERRNO cmd to the BL and parse response */
		rc = _pt_request_pip2_send_cmd(dev,
			PT_CORE_CMD_PROTECTED, &pip2_cmd,
			PIP2_CMD_ID_GET_LAST_ERRNO, NULL, 0, read_buf,
			&actual_read_len);
		if (rc) {
			/*
			 * Command failed to send - should not get here.
			 * If we saw the BL sentinel the GET_LAST_ERRNO cmd
			 * should be available and execute.
			 */
			strcpy(runfw_err_str, "- likely stuck low.");
			pt_debug(dev, DL_ERROR,
				"%s: BL cmd failed to send, Run_FW %s\n",
				__func__, runfw_err_str);
			runfw_toggled = 0;
			goto exit_bl;
		}

		pt_debug(dev, DL_INFO, "%s: BootMode pin state = %d\n",
			__func__, read_buf[6]);
		/*
		 * Must see a valid response with the bootmode byte set to 1
		 * showing the host mode pin was not asserted
		 */
		if (read_buf[4] == 0 && read_buf[6] == 0x01 &&
		    bl_detected_runfw_set) {
			runfw_toggled = 1;
			goto exit_bl;
		} else {
			strcpy(runfw_err_str, "- likely stuck high.");
			pt_debug(dev, DL_ERROR,
				"%s: Bootmode state error when BL entered, RUN_FW %s\n",
				__func__, runfw_err_str);
			runfw_toggled = 0;
		}
	}

exit_bl:
	/*
	 * We're done! - De-assert RunFW pin and perform a hard XRES
	 * toggle, allowing BL to load FW if there is any in Flash
	 */
	pt_debug(dev, DL_WARN,
		"%s: TTDL BIST Complete - Final reset\n", __func__);
	_pt_request_set_runfw_pin(dev, 1);
	mutex_lock(&cd->system_lock);
	cd->startup_status = STARTUP_STATUS_START;
	mutex_unlock(&cd->system_lock);

	rc = pt_hw_hard_reset(cd);
	t = wait_event_timeout(cd->wait_q,
		(cd->startup_status != STARTUP_STATUS_START),
		msecs_to_jiffies(500));
	if (IS_TMO(t)) {
		pt_debug(cd->dev, DL_ERROR,
			"%s: TMO waiting for DUT enumeration\n", __func__);
	}

print_results:
	/*
	 * --------------- PRINT OUT BIST RESULTS ---------------
	 * Ensure the RunFW pin is de-asserted before we are done. In the
	 * case where the RunFW and the IRQ pins are shorted the RunFW will
	 * not release as the DUT may have a response waiting and is holding
	 * the IRQ low. In this case force a read off of the bus to release
	 * the IRQ and in turn the RunFW pin as well.
	 */
	_pt_request_set_runfw_pin(dev, 1);
	while (pt_check_irq_asserted(cd) && count < 5) {
		count++;
		bytes_read += pt_flush_i2c(cd, 0);
	}

	status = irq_toggled + xres_toggled + runfw_toggled;
	pt_debug(dev, DL_INFO, "%s: status = %d (%d, %d, %d)\n",
		__func__, status, irq_toggled, xres_toggled, runfw_toggled);
	ret = sprintf(buf,
		"status %d\n"
		"IRQ connection:      %s %s\n"
		"TP_XRES connection:  %s %s\n"
		"RUN_FW connection:   %s %s\n",
		status == 3 ? 0 : 1,
		irq_toggled == 0x0F ? "[UNTEST]" :
			irq_toggled == 1 ? "[  OK  ]" : "[FAILED]",
		irq_err_str,
		xres_toggled == 0x0F ? "[UNTEST]" :
			xres_toggled == 1 ? "[  OK  ]" : "[FAILED]",
		xres_err_str,
		runfw_toggled == 0x0F ? "[UNTEST]" :
			runfw_toggled == 1 ? "[  OK  ]" : "[FAILED]",
		runfw_err_str);

	/* Put TTDL back into a known state, issue a ttdl restart if needed */
	pt_debug(dev, DL_INFO, "%s: Startup_status = 0x%04X\n",
		__func__, cd->startup_status);
	pt_start_wd_timer(cd);

	return ret;
}
static DEVICE_ATTR(ttdl_bist,  S_IRUGO, pt_ttdl_bist_show, NULL);
#endif /* TTDL_DIAGNOSTICS */

/*******************************************************************************
 * FUNCTION: pt_ttdl_status_show
 *
 * SUMMARY: Show method for the ttdl_status sysfs node. Displays TTDL internal
 *	variable states and GPIO levels. Additional information printed when
 *	TTDL_DIAGOSTICS is enabled.
 *
 *	NOTE: All counters will be reset to 0 when this function is called.
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*dev  - pointer to device structure
 *	*attr - pointer to device attributes
 *	*buf  - pointer to output buffer
 ******************************************************************************/
static ssize_t pt_ttdl_status_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct pt_core_data *cd = dev_get_drvdata(dev);
	struct pt_platform_data *pdata = dev_get_platdata(dev);
	ssize_t ret;

	ret = sprintf(buf,
		"%s: 0x%04X\n"
		"%s: %d\n"
		"%s: %d\n"
		"%s: %s\n"
		"%s: %d\n"
		"%s: %s\n"
		"%s: %s\n"
		"%s: %s\n"
		"%s: %d\n"
		"%s: %d\n"
		"%s: %d\n"
		"%s: %d\n"
		"%s: %d\n"
#ifdef TTDL_DIAGNOSTICS
		"%s: %d\n"
		"%s: %d\n"
		"%s: %d\n"
		"%s: %d\n"
		"%s: %d\n"
		"%s: %d\n"
		"%s: %d\n"
		"%s: %d\n"
#endif /* TTDL_DIAGNOSTICS */
		,
		"Startup Status            ", cd->startup_status,
		"TTDL Debug Level          ", cd->debug_level,
		"Mode                      ", cd->mode,
		"DUT Generation Detected   ",
			cd->dut_generation ?
			(cd->dut_generation == DUT_PIP2_CAPABLE ?
			"PT TC/TT" : "Gen5/6") : "Unknown",
		"HW Detected               ", cd->hw_detected,
		"GPIO state - IRQ          ",
			cd->cpdata->irq_stat ?
			(cd->cpdata->irq_stat(cd->cpdata, dev) ?
			"HIGH" : "low") : "not defined",
		"GPIO state - Host_Mode    ",
			pdata->core_pdata->runfw_gpio ?
			(gpio_get_value(pdata->core_pdata->runfw_gpio) ?
			"HIGH" : "low") : "not defined",
		"GPIO state - TP_XRES      ",
			pdata->core_pdata->rst_gpio ?
			(gpio_get_value(pdata->core_pdata->rst_gpio) ?
			"HIGH" : "low") : "not defined",
		"RAM Parm restore list     ", pt_count_parameter_list(cd),
		"Startup Retry Count       ", cd->startup_retry_count,
		"WD - Manual Force Stop    ", cd->watchdog_force_stop,
		"WD - Enabled              ", cd->watchdog_enabled,
		"WD - Interval (ms)        ", cd->watchdog_interval
#ifdef TTDL_DIAGNOSTICS
		, "WD - Triggered Count      ", cd->watchdog_count,
		"WD - IRQ Stuck low count  ", cd->watchdog_irq_stuck_count,
		"WD - Device Access Errors ", cd->watchdog_failed_access_count,
		"WD - XRES Count           ", cd->wd_xres_count,
		"IRQ Triggered Count       ", cd->irq_count,
		"BL Packet Retry Count     ", cd->bl_retry_packet_count,
		"I2C CRC Error Count       ", cd->i2c_crc_error_count,
		"I2C Transmission Errors   ", cd->i2c_transmission_error_count
#endif /* TTDL_DIAGNOSTICS */
	);

#ifdef TTDL_DIAGNOSTICS
	/* Reset all diagnostic counters */
	cd->watchdog_count               = 0;
	cd->watchdog_irq_stuck_count     = 0;
	cd->watchdog_failed_access_count = 0;
	cd->wd_xres_count                = 0;
	cd->irq_count                    = 0;
	cd->bl_retry_packet_count        = 0;
	cd->i2c_crc_error_count          = 0;
	cd->i2c_transmission_error_count = 0;
#endif /* TTDL_DIAGNOSTICS */

	return ret;
}
static DEVICE_ATTR(ttdl_status,  S_IRUGO, pt_ttdl_status_show, NULL);

#ifdef TTDL_DIAGNOSTICS
/*******************************************************************************
 * FUNCTION: pt_flush_i2c_store
 *
 * SUMMARY: Store method for the flush_i2c sysfs node.
 *
 * RETURN: Size of passed in buffer if successful
 *
 * PARAMETERS:
 *	*dev  - pointer to device structure
 *	*attr - pointer to device attributes
 *	*buf  - pointer to command buffer
 *	 size - size of buf
 ******************************************************************************/
static ssize_t pt_flush_i2c_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct pt_core_data *cd = dev_get_drvdata(dev);
	u32 input_data[1];
	int length;

	/* Maximum input of one value */
	length = pt_ic_parse_input_dec(dev, buf, size, input_data, 2);
	if (length <= 0) {
		pt_debug(dev, DL_ERROR, "%s: %s failed\n", __func__,
			"pt_ic_parse_input_dec");
	}

	mutex_lock(&cd->system_lock);
	if (input_data[0] == 0)
		pt_flush_i2c(cd, 0);
	else
		pt_flush_i2c(cd, 1);
	mutex_unlock(&(cd->system_lock));

	return size;
}
static DEVICE_ATTR(flush_i2c,  S_IWUSR, NULL, pt_flush_i2c_store);

/*******************************************************************************
 * FUNCTION: pt_t_refresh_show
 *
 * SUMMARY: Show method for the t-refresh sysfs node that will show the results
 *	of the T-Refresh timer counting the time it takes to see a user defined
 *	number of interrupts.
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*dev  - pointer to device structure
 *	*attr - pointer to device attributes
 *	*buf  - pointer to output buffer
 ******************************************************************************/
static ssize_t pt_t_refresh_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	struct pt_core_data *cd = dev_get_drvdata(dev);
	u32 whole;
	u16 fraction;

	mutex_lock(&cd->system_lock);

	/* Check if we have counted the number requested */
	if (cd->t_refresh_count != cd->t_refresh_total) {
		ret = sprintf(buf, "%s: %d\n",
			"Still counting... current IRQ count",
			cd->t_refresh_count);
	} else {
		/* Ensure T-Refresh is de-activated */
		cd->t_refresh_active = 0;

		whole = cd->t_refresh_time / cd->t_refresh_count;
		fraction = cd->t_refresh_time % cd->t_refresh_count;
		fraction = fraction * 1000 / cd->t_refresh_count;

		ret = sprintf(buf,
			"%s: %d\n"
			"%s: %d\n"
			"%s: %d\n"
			"%s: %d.%02d\n",
			"Requested IRQ Count     ", cd->t_refresh_total,
			"IRQ Counted             ", cd->t_refresh_count,
			"Total Time Elapsed (ms) ", (int)cd->t_refresh_time,
			"Average T-Refresh  (ms) ", whole, fraction);
	}
	mutex_unlock(&cd->system_lock);
	return ret;
}

/*******************************************************************************
 * FUNCTION: pt_dut_status_show
 *
 * SUMMARY: Show method for DUT status sysfs node. Display DUT's scan state, and
 * more items such as operation mode,easywake state are added in the future.
 *
 * RETURN: Char buffer with printed scan status information
 *
 * PARAMETERS:
 *	*dev  - pointer to device structure
 *	*attr - pointer to device attributes
 *	*buf  - pointer to output buffer
 ******************************************************************************/
static ssize_t pt_dut_status_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	u8 state = 0;
	char *outputstring[3] = {"UNKNOWN", "ACTIVE", "INACTIVE"};
	struct pt_core_data *cd = dev_get_drvdata(dev);
	ssize_t ret;
	u16 calculated_crc = 0;
	u16 stored_crc = 0;
	u8 status;
	int rc;

	pt_get_scan_state(cd, &state);

	if (state == PT_SCAN_STATE_ACTIVE) {
		rc = pt_pip_suspend_scanning_(cd);
		if (rc)
			goto print_results;
	}

	rc = pt_pip_verify_config_block_crc_(cd, PT_TCH_PARM_EBID,
			&status, &calculated_crc, &stored_crc);
	if (rc) {
		calculated_crc = 0;
		stored_crc = 0;
	}

	if (state == PT_SCAN_STATE_ACTIVE)
		pt_pip_resume_scanning_(cd);

print_results:
	ret = snprintf(buf, PT_MAX_PRBUF_SIZE,
		"%s: %d\n"
		"%s: %s\n"
		"%s: 0x%04X\n"
		"%s: 0x%04X\n",
		"status ", rc,
		"Scan Status    ", outputstring[state],
		"Stored CRC     ", stored_crc,
		"Calculated CRC ", calculated_crc);

	 return ret;
}

/*******************************************************************************
 * FUNCTION: pt_set_runfw_pin_store
 *
 * SUMMARY: Store method for the pip2_boot_mode_pin sysfs node. Allows direct
 *	control of the GPIO state.
 *
 * PARAMETERS:
 *      *dev   - pointer to device structure
 *      *attr  - pointer to device attributes
 *      *buf   - pointer to output buffer
 *	 size  - size of data in buffer
 ******************************************************************************/
static ssize_t pt_set_runfw_pin_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
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
		_pt_request_set_runfw_pin(dev, 0);
		break;
	case 1: /* External pull up to 2.5V is needed */
		pt_debug(dev, DL_INFO,
			"Host_Mode_Pin: input mode, float high\n");
		_pt_request_set_runfw_pin(dev, 1);
		break;
	default:
		pt_debug(dev, DL_ERROR, "error: Invalid value\n");
		break;
	}
exit:
	return size;
}
static DEVICE_ATTR(set_runfw_pin, S_IWUSR, NULL, pt_set_runfw_pin_store);
#endif /* TTDL_DIAGNOSTICS */

/*******************************************************************************
 * FUNCTION: pt_pip2_get_version_show
 *
 * SUMMARY: Sends a PIP2 VERSION command to the DUT and prints the
 *	contents of the response to the passed in output buffer.
 *
 * RETURN: size of data written to sysfs node
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
	u16 pip_ver;
	u8 read_buf[256];
	u8 status;
	struct pip2_cmd_structure pip2_cmd;
	u8 index = PIP2_RESPONSE_STATUS_OFFSET;

	rc = _pt_request_pip2_send_cmd(dev,
		PT_CORE_CMD_UNPROTECTED, &pip2_cmd,
		PIP2_CMD_ID_VERSION, NULL, 0, read_buf, &actual_read_len);

	if (rc) {
		pt_debug(dev, DL_ERROR,
			"%s: Failed to send PIP2 VERSION cmd\n", __func__);
		goto ver_err;
	}

	status = read_buf[index];
	if (status == 0) {
		pip_ver = 256 * read_buf[index + 2] + read_buf[index + 1];

		/*
		 * BL PIP 2.02 and greater supports additional fields
		 * and the BL version and FW version are swapped. Sub Lot bytes
		 * 16 and 17 are reserved.
		 */
		if (pip_ver >= 0x0202) {
			return snprintf(buf, PT_MAX_PRBUF_SIZE,
				"PIP VERSION   : %02X.%02X\n"
				"BL VERSION    : %02X.%02X\n"
				"FW VERSION    : %02X.%02X\n"
				"SILICON ID    : %02X.%02X.%02X.%02X\n"
				"WAFER LOT     : %c%c%c%c%c\n"
				"WAFER NUMBER  : %d\n"
				"X-COORD       : %d\n"
				"Y-COORD       : %d\n",
				read_buf[index + 2], read_buf[index + 1],
				read_buf[index + 6], read_buf[index + 5],
				read_buf[index + 4], read_buf[index + 3],

				read_buf[index + 10], read_buf[index + 9],
				read_buf[index + 8], read_buf[index + 7],

				read_buf[index + 15], read_buf[index + 14],
				read_buf[index + 13], read_buf[index + 12],
				read_buf[index + 11],

				read_buf[index + 18],

				read_buf[index + 20] << 8 |
				read_buf[index + 19],

				read_buf[index + 22] << 8 |
				read_buf[index + 21]);
		} else {
			return snprintf(buf, PT_MAX_PRBUF_SIZE,
				"PIP VERSION   : %02X.%02X\n"
				"BL VERSION    : %02X.%02X\n"
				"FW VERSION    : %02X.%02X\n"
				"SILICON ID    : %02X.%02X.%02X.%02X\n"
				"WAFER LOT     : n/a\n"
				"WAFER SUB-LOT : n/a\n"
				"WAFER NUMBER  : n/a\n"
				"X-COORD       : n/a\n"
				"Y-COORD       : n/a\n",
				read_buf[index + 2], read_buf[index + 1],
				read_buf[index + 4], read_buf[index + 3],
				read_buf[index + 6], read_buf[index + 5],

				read_buf[index + 8], read_buf[index + 7],
				read_buf[index + 10], read_buf[index + 9]);
		}
	} else {
		return snprintf(buf, PT_MAX_PRBUF_SIZE,
			"status %d\n", status);
	}

ver_err:
	return snprintf(buf, PT_MAX_PRBUF_SIZE,
		"PIP VERSION  : - DUT access error\n"
		"BL VERSION   : - DUT access error\n"
		"FW VERSION   : - DUT access error\n"
		"SILICON ID   : - DUT access error\n");
}
static DEVICE_ATTR(pip2_get_version, S_IRUGO, pt_pip2_get_version_show, NULL);


/*******************************************************************************
 * FUNCTION: pt_hw_version_show
 *
 * SUMMARY: Gets the HW version for either PIP1.x or PIP2.x DUTS
 *	Output data format: [SiliconID].[RevID FamilyID].[PanelID]
 *
 * RETURN: size of data written to sysfs node
 *
 * PARAMETERS:
 *	*dev  - pointer to device structure
 *	*attr - pointer to device attributes structure
 *	*buf  - pointer to print output buffer
 ******************************************************************************/
static ssize_t pt_hw_version_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int rc;
	u16 actual_read_len;
	u16 pip_ver;
	u8 rd_buf[256];
	u8 status;
	u8 index = PIP2_RESPONSE_STATUS_OFFSET;
	u8 return_data[8];
	u8 panel_id;
	struct pip2_cmd_structure pip2_cmd;
	struct pt_core_data *cd = dev_get_drvdata(dev);
	struct pt_ttdata *ttdata = &cd->sysinfo.ttdata;

	/* For Parade TC or TT parts */
	if (cd->dut_generation == DUT_PIP2_CAPABLE) {
		rc = _pt_request_pip2_send_cmd(dev,
			PT_CORE_CMD_UNPROTECTED, &pip2_cmd,
			PIP2_CMD_ID_VERSION, NULL, 0, rd_buf,
			&actual_read_len);

		if (rc) {
			pt_debug(dev, DL_ERROR,
				"%s: Failed to send PIP2 VERSION cmd\n",
				__func__);
			return snprintf(buf, PT_MAX_PRBUF_SIZE, "...");
		}

		status = rd_buf[index];
		if (status == 0) {
			pip_ver = 256 * rd_buf[index + 2] + rd_buf[index + 1];

			/*
			 * BL PIP 2.02 and greater the version fields are
			 * swapped
			 */
			if (pip_ver >= 0x0202) {
				return snprintf(buf, PT_MAX_PRBUF_SIZE,
					"%02X%02X.%02X%02X.FF\n",
					rd_buf[index + 10], rd_buf[index + 9],
					rd_buf[index + 8], rd_buf[index + 7]);
			} else {
				return snprintf(buf, PT_MAX_PRBUF_SIZE,
					"%02X%02X.%02X%02X.FF\n",
					rd_buf[index + 8], rd_buf[index + 7],
					rd_buf[index + 10], rd_buf[index + 9]);
			}
		} else {
			return snprintf(buf, PT_MAX_PRBUF_SIZE, "...");
		}
	} else if (cd->dut_generation == DUT_PIP1_ONLY) {
		/*
		 * For Parade/Cypress legacy parts the RevID and FamilyID are
		 * hard coded to FFFF
		 */
		if (cd->hw_detected) {
			if (cd->mode == PT_MODE_OPERATIONAL) {
				/* In FW - simply retrieve from ttdata struct */
				return snprintf(buf, PT_MAX_PRBUF_SIZE,
					"%04X.FFFF.%02X\n",
					ttdata->jtag_id_h,
					cd->panel_id);
			} else {
				/* Send BL command */
				rc = pt_hid_output_bl_get_information(cd,
					return_data);
				if (!rc)
					rc = pt_hid_output_bl_get_panel_id(cd,
						&panel_id);
				if (!rc) {
					return snprintf(buf, PT_MAX_PRBUF_SIZE,
						"%02X%02X.FFFF.%02X\n",
						return_data[3], return_data[2],
						panel_id);
				} else {
					return snprintf(buf, PT_MAX_PRBUF_SIZE,
						"FFFF.FFFF.FF");
				}
			}
		} else {
			/* No HW detected */
			return snprintf(buf, PT_MAX_PRBUF_SIZE,
				"FFFF.FFFF.FF");
		}
	} else {
		/* Unkown HW detected */
		return snprintf(buf, PT_MAX_PRBUF_SIZE,
			"FFFF.FFFF.FF");
	}
}
static DEVICE_ATTR(hw_version, S_IRUGO, pt_hw_version_show, NULL);


/*******************************************************************************
 * Structure of sysfs attributes for all DUT dependent sysfs node
 ******************************************************************************/
static struct device_attribute attributes[] = {
	__ATTR(ic_ver, S_IRUGO, pt_ic_ver_show, NULL),
	__ATTR(hw_reset, S_IWUSR, NULL, pt_hw_reset_store),
	__ATTR(dut_debug, S_IWUSR, NULL, pt_drv_debug_store),
	__ATTR(sleep_status, S_IRUGO, pt_sleep_status_show, NULL),
	__ATTR(easy_wakeup_gesture, S_IRUGO | S_IWUSR,
		pt_easy_wakeup_gesture_show,
		pt_easy_wakeup_gesture_store),
#ifdef EASYWAKE_TSG6
	__ATTR(easy_wakeup_gesture_id, S_IRUGO,
		pt_easy_wakeup_gesture_id_show, NULL),
	__ATTR(easy_wakeup_gesture_data, S_IRUGO,
		pt_easy_wakeup_gesture_data_show, NULL),
#endif
	__ATTR(panel_id, S_IRUGO, pt_panel_id_show, NULL),
	__ATTR(platform_data, S_IRUGO, pt_platform_data_show, NULL),
	__ATTR(get_param, S_IRUGO | S_IWUSR,
		pt_get_param_show, pt_get_param_store),
#ifdef TTDL_DIAGNOSTICS
	__ATTR(pip2_enter_bl, S_IRUGO, pt_pip2_enter_bl_show, NULL),
	__ATTR(drv_irq, S_IRUGO | S_IWUSR, pt_drv_irq_show, pt_drv_irq_store),
	__ATTR(hw_irq_stat, S_IRUGO, pt_hw_irq_stat_show, NULL),
	__ATTR(dut_status, S_IRUGO, pt_dut_status_show, NULL),
	__ATTR(t_refresh, S_IRUGO | S_IWUSR,
		pt_t_refresh_show, pt_t_refresh_store),
#endif /* TTDL_DIAGNOSTICS */
};

/*******************************************************************************
 * FUNCTION: add_sysfs_interfaces
 *
 * SUMMARY: Creates all DUT dependent sysfs nodes owned by the core
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*dev  - pointer to device structure
 ******************************************************************************/
static int add_sysfs_interfaces(struct device *dev)
{
	int i;
	struct pt_core_data *cd = dev_get_drvdata(dev);

	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		if (device_create_file(dev, attributes + i))
			goto undo;
	cd->sysfs_nodes_created = true;

	return 0;
undo:
	for (i--; i >= 0; i--)
		device_remove_file(dev, attributes + i);
	cd->sysfs_nodes_created = false;
	pt_debug(dev, DL_ERROR, "%s: failed to create sysfs interface\n",
		__func__);
	return -ENODEV;
}

/*******************************************************************************
 * FUNCTION: remove_sysfs_interfaces
 *
 * SUMMARY: Removes all DUT dependent sysfs nodes owned by the core
 *
 * RETURN: void
 *
 * PARAMETERS:
 *	*dev  - pointer to device structure
 ******************************************************************************/
static void remove_sysfs_interfaces(struct device *dev)
{
	u32 i;
	struct pt_core_data *cd = dev_get_drvdata(dev);

	cd->sysfs_nodes_created = false;
	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		device_remove_file(dev, attributes + i);
}


/*******************************************************************************
 *******************************************************************************
 * FUNCTION: pt_probe
 *
 * SUMMARY: Probe of the core module.
 *
 *	NOTE: For the Parade Technologies development platform (PtSBC) the
 *	probe functionality is split into two functions; pt_probe() and
 *	pt_probe_complete(). the initial setup is done in this function which
 *	then creates a WORK task which runs after the probe timer expires. This
 *	ensures the I2C is up on the PtSBC in time for TTDL.
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*ops           - pointer to the bus
 *	*dev           - pointer to the device structure
 *	 irq           - IRQ
 *	 xfer_buf_size - size of the buffer
 ******************************************************************************/
int pt_probe(const struct pt_bus_ops *ops, struct device *dev,
		u16 irq, size_t xfer_buf_size)
{
	struct pt_core_data *cd;
	struct pt_platform_data *pdata = dev_get_platdata(dev);
	enum pt_atten_type type;
	int rc = 0;
#ifndef PT_PTSBC_SUPPORT
	u8 current_dl = PT_INITIAL_DEBUG_LEVEL;
	u8 buf[256];
	u8 pip_ver_major;
	u8 pip_ver_minor;
#endif

	/* Set default static values */
	pt_bus_ops_save = NULL;

	if (!pdata || !pdata->core_pdata || !pdata->mt_pdata) {
		pt_debug(dev, DL_ERROR, "%s: Missing platform data\n",
				__func__);
		rc = -ENODEV;
		goto error_no_pdata;
	}

	if (pdata->core_pdata->flags & PT_CORE_FLAG_POWEROFF_ON_SLEEP) {
		if (!pdata->core_pdata->power) {
			pt_debug(dev, DL_ERROR,
					"%s: Missing platform data function\n",
					__func__);
			rc = -ENODEV;
			goto error_no_pdata;
		}
	}

	/* get context and debug print buffers */
	cd = kzalloc(sizeof(*cd), GFP_KERNEL);
	if (!cd) {
		rc = -ENOMEM;
		goto error_alloc_data;
	}

	/* Initialize device info */
	cd->dev                        = dev;
	cd->pdata                      = pdata;
	cd->cpdata                     = pdata->core_pdata;
	cd->bus_ops                    = ops;
	cd->debug_level                = PT_INITIAL_DEBUG_LEVEL;
	cd->show_timestamp             = PT_INITIAL_SHOW_TIME_STAMP;
	scnprintf(cd->core_id, 20, "%s%d", PT_CORE_NAME, core_number++);
	cd->hw_detected                = false;
	cd->sysfs_nodes_created        = false;
	cd->pip2_prot_active           = 0;
	cd->bl_pip_version_major       = 0;
	cd->bl_pip_version_minor       = 0;
	cd->pip2_cmd_tag_seq           = 0x08; /* PIP2 TAG=1 and 3 bit SEQ=0 */
	cd->get_param_id               = 0;
	cd->watchdog_enabled           = 0;
	cd->dut_generation             = DUT_UNKNOWN;
#ifdef TTDL_DIAGNOSTICS
	cd->wd_xres_count              = 0;
#endif /* TTDL_DIAGNOSTICS */
	cd->wd_corrective_action       = 1;
	cd->startup_retry_count        = 0;
#ifdef PT_PTSBC_SUPPORT
	/*
	 * Force watchdog stop for the need of Touch debug, but for
	 * user release, watchdog should be enabled.
	 */
	cd->watchdog_force_stop        = true;
#else
	cd->watchdog_force_stop	       = false;
#endif

	cd->core_probe_complete        = 0;

#ifdef PT_PTSBC_SUPPORT
	/*
	 * Extend first WD to allow DDI to be configured before a
	 * WD initiated startup(). Without the DDI configured the FW will
	 * not respond.
	 */
	cd->watchdog_interval          = PT_PTSBC_INIT_WATCHDOG_TIMEOUT;
#else
	cd->watchdog_interval          = PT_WATCHDOG_TIMEOUT;
#endif

	/*
	 * Initialize the HID descriptor based on standard values for the rare
	 * case when a report arrives before the HID descriptor is read.
	 * Reading the HID descriptor during the probe will overwrite these
	 * values with what is sent by the DUT.
	 */
	cd->hid_desc.report_desc_register = 2;
	cd->hid_desc.input_register       = 3;
	cd->hid_desc.max_input_len        = 256;
	cd->hid_desc.output_register      = 4;
	cd->hid_desc.max_output_len       = 254;
	cd->hid_desc.command_register     = 5;
	cd->hid_desc.data_register        = 6;
	cd->hid_cmd_state                 = 1;

#ifdef TTDL_DIAGNOSTICS
	cd->t_refresh_active              = 0;
	cd->t_refresh_count               = 0;
	cd->i2c_crc_error_count           = 0;
	cd->bl_retry_packet_count         = 0;
	cd->route_i2c_virt_dut            = 0;
#endif /* TTDL_DIAGNOSTICS */

	/* Initialize mutexes and spinlocks */
	mutex_init(&cd->module_list_lock);
	mutex_init(&cd->system_lock);
	mutex_init(&cd->adap_lock);
	mutex_init(&cd->hid_report_lock);
	mutex_init(&cd->sysfs_lock);
	mutex_init(&cd->ttdl_restart_lock);
	mutex_init(&cd->firmware_class_lock);
	spin_lock_init(&cd->spinlock);

	/* Initialize module list */
	INIT_LIST_HEAD(&cd->module_list);

	/* Initialize attention lists */
	for (type = 0; type < PT_ATTEN_NUM_ATTEN; type++)
		INIT_LIST_HEAD(&cd->atten_list[type]);

	/* Initialize parameter list */
	INIT_LIST_HEAD(&cd->param_list);

	/* Initialize wait queue */
	init_waitqueue_head(&cd->wait_q);

	/* Initialize works */
	INIT_WORK(&cd->startup_work, pt_startup_work_function);
	INIT_WORK(&cd->watchdog_work, pt_watchdog_work);
#ifdef PT_PTSBC_SUPPORT
	/* Adding new work queue to cd struct */
	INIT_WORK(&cd->irq_work, pt_irq_work_function);
#endif

	/* Initialize HID specific data */
	cd->hid_core.hid_vendor_id = (cd->cpdata->vendor_id) ?
		cd->cpdata->vendor_id : PT_HID_VENDOR_ID;
	cd->hid_core.hid_product_id = (cd->cpdata->product_id) ?
		cd->cpdata->product_id : PT_HID_APP_PRODUCT_ID;
	cd->hid_core.hid_desc_register =
		cpu_to_le16(cd->cpdata->hid_desc_register);

	/* Set platform easywake value */
	cd->easy_wakeup_gesture = cd->cpdata->easy_wakeup_gesture;

	/* Set Panel ID to Not Enabled */
	cd->panel_id = PANEL_ID_NOT_ENABLED;

	dev_set_drvdata(dev, cd);
#ifndef PT_PTSBC_SUPPORT
	/* PtSBC builds will call this function in pt_probe_complete() */
	pt_add_core(dev);
#endif

	/* Create sysfs nodes not dependent on the DUT being accessible */
	device_create_file(dev, &dev_attr_drv_ver);
	device_create_file(dev, &dev_attr_ttdl_restart);
	device_create_file(dev, &dev_attr_ttdl_status);
	device_create_file(dev, &dev_attr_drv_debug);
	device_create_file(dev, &dev_attr_command);
	device_create_file(dev, &dev_attr_response);
	device_create_file(dev, &dev_attr_status);
	device_create_file(dev, &dev_attr_hw_version);
#ifdef TTDL_DIAGNOSTICS
	device_create_file(dev, &dev_attr_ttdl_bist);
	device_create_file(dev, &dev_attr_set_runfw_pin);
	device_create_file(dev, &dev_attr_flush_i2c);
	device_create_file(dev, &dev_attr_err_gpio);
#endif /* TTDL_DIAGNOSTICS */

	/*
	 * Save the pointer to a global value, which will be used
	 * in ttdl_restart function
	 */
	pt_bus_ops_save = ops;

	/*
	 * When the IRQ GPIO is not direclty accessable and no function is
	 * defined to get the IRQ status, the IRQ passed in must be assigned
	 * directly as the gpio_to_irq will not work. e.g. CHROMEOS
	 */
	if (!cd->cpdata->irq_stat) {
		cd->irq = irq;
		pt_debug(cd->dev, DL_WARN, "%s:Set cd->irq = %d\n",
			__func__, cd->irq);
	}

#ifdef PT_PTSBC_SUPPORT
	/*
	 * For the PtSBC, on the first bring up, I2C will not be ready in
	 * time so complete probe with pt_probe_complete() after work
	 * probe timer expires.
	 */
	INIT_WORK(&cd->probe_work, pt_probe_work);
	setup_timer(&cd->probe_timer, pt_probe_timer,
			(unsigned long)cd);

	/* Some host i2c busses start late and then run too slow */
	pt_debug(cd->dev, DL_INFO, "%s:start wait for probe timer\n",
		__func__);
	mod_timer(&cd->probe_timer, jiffies +
			msecs_to_jiffies(PT_CORE_PROBE_STARTUP_DELAY_MS));
	return rc;

error_alloc_data:
error_no_pdata:
	pt_debug(dev, DL_ERROR, "%s failed.\n", __func__);
	return rc;
}

/*******************************************************************************
 * FUNCTION: pt_probe_complete
 *
 * SUMMARY: This funciton is only needed when PT_PTSBC_SUPPORT is enabled.
 *	For the PtSBC, the probe functionality is split into two functions;
 *	pt_probe() and pt_probe_complete(). The initial setup is done
 *	in pt_probe() and the rest is done here after I2C is up. This
 *	function also configures all voltage regulators for the PtSBC.
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*cd - pointer to the core data structure
 ******************************************************************************/
static int pt_probe_complete(struct pt_core_data *cd)
{
	int rc = -1;
	u8 buf[256];
	struct device *dev = cd->dev;
	u8 current_dl = cd->debug_level;
	u8 pip_ver_major;
	u8 pip_ver_minor;

	/* Override the debug level to at least DL_WARN, during probe */
	if (cd->debug_level < DL_WARN)
		cd->debug_level = DL_WARN;

	pt_debug(cd->dev, DL_DEBUG,
		"%s: PARADE Entering Probe complete function\n", __func__);
	pt_add_core(cd->dev);

	/* Call platform init function before setting up the GPIO's */
	if (cd->cpdata->init) {
		pt_debug(cd->dev, DL_INFO, "%s: Init HW\n", __func__);
		rc = cd->cpdata->init(cd->cpdata, PT_MT_POWER_ON, cd->dev);
	} else {
		pt_debug(cd->dev, DL_WARN,
			"%s: No HW INIT function\n", __func__);
		rc = 0;
	}
	if (rc < 0) {
		pt_debug(cd->dev, DL_ERROR,
			"%s: HW Init fail r=%d\n", __func__, rc);
	}

	/* Power on the regulator(s) needed by the PtSBC */
	if (cd->cpdata->setup_power) {
		pt_debug(cd->dev, DL_INFO, "%s: Device power on!\n", __func__);
		rc = cd->cpdata->setup_power(cd->cpdata,
			PT_MT_POWER_ON, cd->dev);
	} else {
		pt_debug(cd->dev, DL_WARN, "%s: No setup power function\n",
			__func__);
		rc = 0;
	}
	if (rc < 0)
		pt_debug(cd->dev, DL_ERROR, "%s: Setup power on fail r=%d\n",
			__func__, rc);
	/* TODO - determine how long we have to detect PIP2 BL after release */

#else
	if (cd->cpdata->init) {
		pt_debug(cd->dev, DL_INFO, "%s: Init HW\n", __func__);
		rc = cd->cpdata->init(cd->cpdata, PT_MT_POWER_ON, cd->dev);
	} else {
		pt_debug(cd->dev, DL_WARN, "%s: No HW INIT function\n",
			__func__);
		rc = 0;
	}
	if (rc < 0)
		pt_debug(cd->dev, DL_ERROR, "%s: HW Init fail r=%d\n",
			__func__, rc);

	if (cd->cpdata->setup_power) {
		pt_debug(cd->dev, DL_INFO, "%s: Device power on!\n", __func__);
		rc = cd->cpdata->setup_power(cd->cpdata,
			PT_MT_POWER_ON, cd->dev);
	} else {
		pt_debug(cd->dev, DL_WARN, "%s: No setup power function\n",
			__func__);
		rc = 0;
	}
	if (rc < 0)
		pt_debug(cd->dev, DL_ERROR, "%s: Setup power on fail r=%d\n",
			__func__, rc);
#endif /* --- End PT_PTSBC_SUPPORT --- */

#ifdef TTDL_DIAGNOSTICS
	cd->watchdog_irq_stuck_count     = 0;
	cd->i2c_transmission_error_count = 0;
#endif /* TTDL_DIAGNOSTICS */

	if (cd->cpdata->detect) {
		pt_debug(cd->dev, DL_INFO, "%s: Detect HW\n", __func__);
		rc = cd->cpdata->detect(cd->cpdata, cd->dev,
				pt_platform_detect_read);
		if (!rc) {
			cd->hw_detected = true;
			pt_debug(cd->dev, DL_INFO, "%s: HW detected\n",
					__func__);
		} else {
			cd->hw_detected = false;
			pt_debug(cd->dev, DL_INFO, "%s: No HW detected\n",
					__func__);
			rc = -ENODEV;
			goto error_detect;
		}
	} else
		pt_debug(dev, DL_ERROR,
				"%s: PARADE No HW detect function pointer\n",
				__func__);

	rc = pt_setup_irq_gpio(cd);
	if (rc < 0) {
		pt_debug(dev, DL_ERROR, "%s: Error, could not setup IRQ\n",
				__func__);
		goto error_setup_irq;
	}

	setup_timer(&cd->watchdog_timer, pt_watchdog_timer,
			(unsigned long)cd);
	pt_stop_wd_timer(cd);

#ifdef TTHE_TUNER_SUPPORT
	mutex_init(&cd->tthe_lock);
	cd->tthe_debugfs = debugfs_create_file(PT_TTHE_TUNER_FILE_NAME,
			0644, NULL, cd, &tthe_debugfs_fops);
#endif
	rc = device_init_wakeup(dev, 1);
	if (rc < 0)
		pt_debug(dev, DL_ERROR, "%s: Error, device_init_wakeup rc:%d\n",
				__func__, rc);

	pm_runtime_get_noresume(dev);
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	msleep(100);

	/* If IRQ asserted, read out all from buffer to release INT pin */
	if (cd->cpdata->irq_stat &&
	    cd->cpdata->irq_stat(cd->cpdata, cd->dev)
	     == PT_IRQ_ASSERTED_VALUE) {
		pt_debug(dev, DL_WARN, "%s: IRQ asserted, clear buffer...\n",
			__func__);
		i2c_master_recv(to_i2c_client(cd->dev), buf, 2);
		i2c_master_recv(to_i2c_client(cd->dev), buf,
		2 + get_unaligned_le16(&buf[0]));
		pt_debug(dev, DL_INFO,
			"%s: Buffer cleared, IRQ deasserted\n", __func__);
	} else {
		/*
		 * With no IRQ Stat, force a read in case the reset sentinel
		 * already arrieved.
		 */
		rc = pt_read_input(cd);
		if (!rc)
			pt_parse_input(cd);
	}

	/* Detect what PIP interface version the DUT is communicating in */
	pt_debug(dev, DL_INFO, "%s: Detect PIP Interface Version...\n",
		__func__);

	_pt_request_active_pip_protocol(cd->dev, PT_CORE_CMD_PROTECTED,
		&pip_ver_major, &pip_ver_minor);
	if (pip_ver_major == 2) {
		cd->bl_pip_version_major = pip_ver_major;
		cd->bl_pip_version_minor = pip_ver_minor;
		dev_err(dev, " ======== PIP2.%02X Detected ========\n",
			pip_ver_minor);

#ifdef TTDL_DIAGNOSTICS
		/*
		 * PM functions should be in pairs in any calling path
		 * to ensure PM "get" and "put" calling is complete.
		 * Here it calls pm_runtime_get_noresume() at the beginning
		 * and it need a "put" function to allow device can enter
		 * sleep mode for this calling path.
		 */
#endif /* TTDL_DIAGNOSTICS */
		pm_runtime_put_sync(dev);

#ifdef TTDL_DIAGNOSTICS
		/*
		 * If the TC3300 comes up with a PIP2.x protocol this
		 * means either there was no valid FW in the FLASH part, or
		 * this probe occurred before the BL started executing FW.
		 * In both cases the DUT is not ready for pt_startup()
		 * so delay to see if FW comes up.
		 */
#endif /* TTDL_DIAGNOSTICS */
		goto create_sysfs_nodes;
	} else if (pip_ver_major == 1) {
		cd->bl_pip_version_major = pip_ver_major;
		cd->bl_pip_version_minor = pip_ver_minor;
		dev_err(dev, " ======== PIP1.%02X Detected ========\n",
			pip_ver_minor);
	} else {
		dev_err(dev, " ======== PIP Version Not Retrieved  ========\n");
		pm_runtime_put_sync(dev);
		/* goto create_sysfs_nodes; */
	}

	/*
	 * Call startup directly to ensure that the device is tested
	 * before leaving the probe
	 */
	pt_debug(dev, DL_WARN, "%s: call startup\n", __func__);
	pt_debug(dev, DL_INFO,
		"%s: === call Startup Directly, hw_detected = %d ===\n",
		__func__, cd->hw_detected);
	rc = pt_startup(cd, false);

	pm_runtime_put_sync(dev);

	if (rc == -ENODEV) {
		pt_debug(cd->dev, DL_ERROR,
			"%s: Fail initial startup r=%d\n", __func__, rc);
#ifndef PT_PTSBC_SUPPORT
		/* For PtSBC don't error out, allow TTDL to stay up */
		goto error_after_startup;
#endif
	}

create_sysfs_nodes:
	_pt_request_dut_generation(cd->dev);
	if (cd->dut_generation == DUT_PIP2_CAPABLE) {
		device_create_file(dev, &dev_attr_pip2_bin_hdr);
		device_create_file(dev, &dev_attr_pip2_get_version);
	}
	if (cd->hw_detected) {
		pt_debug(dev, DL_INFO, "%s: Add sysfs interfaces\n",
			__func__);
		rc = add_sysfs_interfaces(dev);
		if (rc < 0) {
			pt_debug(dev, DL_ERROR,
				"%s: Error, fail sysfs init\n", __func__);
			goto error_after_startup;
		}
	}

	pt_debug(dev, DL_INFO, "%s: Probe: MT, BTN\n", __func__);

#ifndef PT_PTSBC_SUPPORT
	/*
	 * PtSBC will trigger it's own ttdl_restart after DDI init is
	 * complete. Probing MT and BTN now is of no value because
	 * the HID descriptor has not yet been read out.
	 */
	rc = pt_mt_probe(dev);
	if (rc < 0) {
		pt_debug(dev, DL_ERROR, "%s: Error, fail mt probe\n",
			__func__);
		goto error_after_sysfs_create;
	}

	rc = pt_btn_probe(dev);
	if (rc < 0) {
		pt_debug(dev, DL_ERROR, "%s: Error, fail btn probe\n",
			__func__);
		goto error_after_startup_mt;
	}
	pt_start_wd_timer(cd);
#endif
	pt_probe_modules(cd);

#ifdef CONFIG_HAS_EARLYSUSPEND
	pt_setup_early_suspend(cd);
#elif defined(CONFIG_FB)
	pt_setup_fb_notifier(cd);
#endif

#if NEED_SUSPEND_NOTIFIER
	cd->pm_notifier.notifier_call = pt_pm_notifier;
	register_pm_notifier(&cd->pm_notifier);
#endif
	cd->debug_level = current_dl;
	cd->core_probe_complete = 1;
	return 0;


#ifndef PT_PTSBC_SUPPORT
error_after_startup_mt:
	pr_err("%s PARADE error_after_startup_mt\n", __func__);
	pt_mt_release(dev);
error_after_sysfs_create:
	pr_err("%s PARADE error_after_sysfs_create\n", __func__);
	pm_runtime_disable(dev);
#if (KERNEL_VERSION(3, 16, 0) > LINUX_VERSION_CODE)
	device_wakeup_disable(dev);
#endif
	device_init_wakeup(dev, 0);
	cancel_work_sync(&cd->startup_work);
	pt_stop_wd_timer(cd);
	pt_free_si_ptrs(cd);
	remove_sysfs_interfaces(dev);
#endif
error_after_startup:
	pr_err("%s PARADE error_after_startup\n", __func__);
	disable_irq_nosync(cd->irq);
	free_irq(cd->irq, cd);
	del_timer(&cd->watchdog_timer);
error_setup_irq:
error_detect:
#ifdef PT_PTSBC_SUPPORT
	cancel_work_sync(&cd->irq_work);
	destroy_workqueue(parade_wq);
	del_timer(&cd->probe_timer);
#endif
	if (cd->cpdata->init)
		cd->cpdata->init(cd->cpdata, PT_MT_POWER_OFF, dev);
	if (cd->cpdata->setup_power)
		cd->cpdata->setup_power(cd->cpdata, PT_MT_POWER_OFF, dev);
	pt_del_core(dev);
	dev_set_drvdata(dev, NULL);
	kfree(cd);
#ifndef PT_PTSBC_SUPPORT
error_alloc_data:
error_no_pdata:
#endif
	pr_err("%s failed.\n", __func__);
	return rc;
}
EXPORT_SYMBOL_GPL(pt_probe);

/*******************************************************************************
 * FUNCTION: pt_release
 *
 * SUMMARY: This fuction does the following cleanup:
 *	- Releases all probed modules
 *	- Stops the watchdog
 *	- Cancels all pending work tasks
 *	- Removes all created sysfs nodes
 *	- Removes all created debugfs nodes
 *	- Frees the IRQ
 *	- Deletes the core
 *	- Frees all pointers and HID reports
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*cd - pointer to the core data structure
 ******************************************************************************/
int pt_release(struct pt_core_data *cd)
{
	struct device *dev = cd->dev;

	pt_release_modules(cd);
	pt_proximity_release(dev);
	pt_btn_release(dev);
	pt_mt_release(dev);

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&cd->es);
#elif defined(CONFIG_FB)
	fb_unregister_client(&cd->fb_notifier);
#endif

#if NEED_SUSPEND_NOTIFIER
	unregister_pm_notifier(&cd->pm_notifier);
#endif

	/*
	 * Suspend the device before freeing the startup_work and stopping
	 * the watchdog since sleep function restarts watchdog on failure
	 */
	pm_runtime_suspend(dev);
	pm_runtime_disable(dev);

	cancel_work_sync(&cd->startup_work);

	pt_stop_wd_timer(cd);

#if (KERNEL_VERSION(3, 16, 0) > LINUX_VERSION_CODE)
	device_wakeup_disable(dev);
#endif
	device_init_wakeup(dev, 0);

#ifdef TTHE_TUNER_SUPPORT
	mutex_lock(&cd->tthe_lock);
	cd->tthe_exit = 1;
	wake_up(&cd->wait_q);
	mutex_unlock(&cd->tthe_lock);
	debugfs_remove(cd->tthe_debugfs);
#endif
	/* Remove sysfs nodes not dependent on the DUT being accessible */
	device_remove_file(dev, &dev_attr_drv_ver);
	device_remove_file(dev, &dev_attr_ttdl_restart);
	device_remove_file(dev, &dev_attr_ttdl_status);
	device_remove_file(dev, &dev_attr_drv_debug);
	device_remove_file(dev, &dev_attr_command);
	device_remove_file(dev, &dev_attr_response);
	device_remove_file(dev, &dev_attr_status);
	device_remove_file(dev, &dev_attr_hw_version);
#ifdef TTDL_DIAGNOSTICS
	device_remove_file(dev, &dev_attr_ttdl_bist);
	device_remove_file(dev, &dev_attr_set_runfw_pin);
	device_remove_file(dev, &dev_attr_flush_i2c);
	device_remove_file(dev, &dev_attr_err_gpio);
#endif /* TTDL_DIAGNOSTICS */

	remove_sysfs_interfaces(dev);
	free_irq(cd->irq, cd);
	if (cd->cpdata->init)
		cd->cpdata->init(cd->cpdata, PT_MT_POWER_OFF, dev);
	if (cd->cpdata->setup_power)
		cd->cpdata->setup_power(cd->cpdata, PT_MT_POWER_OFF, dev);
	dev_set_drvdata(dev, NULL);
	pt_del_core(dev);
	pt_free_si_ptrs(cd);
	kfree(cd);
	return 0;
}
EXPORT_SYMBOL_GPL(pt_release);


MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Parade TrueTouch(R) Standard Product Core Driver");
MODULE_AUTHOR("Parade Technologies <ttdrivers@paradetech.com>");
