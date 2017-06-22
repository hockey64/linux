/*
 * cyttsp5_core.c
 * Parade TrueTouch(TM) Standard Product V5 Core Module.
 * For use with Parade touchscreen controllers.
 * Supported parts include:
 * CYTMA5XX
 * CYTMA448
 * CYTMA445A
 * CYTT21XXX
 * CYTT31XXX
 *
 * Copyright (C) 2016 Parade Technologies
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

#include "cyttsp5_regs.h"
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
#define CY_CORE_PROBE_STARTUP_DELAY_MS		500
static struct workqueue_struct *parade_wq;
#endif /* --- PT_PTSBC_SUPPORT --- */

#define CY_CORE_STARTUP_RETRY_COUNT		3

MODULE_FIRMWARE(CY_FW_FILE_NAME);

static const char *cy_driver_core_name = CYTTSP5_CORE_NAME;
static const char *cy_driver_core_version = CY_DRIVER_VERSION;
static const char *cy_driver_core_date = CY_DRIVER_DATE;
static const struct cyttsp5_bus_ops *cyttsp5_bus_ops_save;

struct cyttsp5_hid_field {
	int report_count;
	int report_size;
	int size; /* report_count * report_size */
	int offset;
	int data_type;
	int logical_min;
	int logical_max;
	/* Usage Page (Hi 16 bit) + Usage (Lo 16 bit) */
	u32 usage_page;
	u32 collection_usage_pages[CY_HID_MAX_COLLECTIONS];
	struct cyttsp5_hid_report *report;
	bool record_field;
};

struct cyttsp5_hid_report {
	u8 id;
	u8 type;
	int size;
	struct cyttsp5_hid_field *fields[CY_HID_MAX_FIELDS];
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
	struct cyttsp5_module *module;
	void *data;
};

struct cyttsp5_hid_cmd {
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

struct cyttsp5_hid_output {
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
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);
	int i;
	int pr_buf_index = 0;
	int max_size;

	/* only proceed if valid debug level and there is data to print */
	if (debug_level <= cd->debug_level && buf_len > 0) {
		char pr_buf[1024];

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
static int tthe_print(struct cyttsp5_core_data *cd, u8 *buf, int buf_len,
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

	if (cd->tthe_buf_len + (len + buf_len) > CY_MAX_PRBUF_SIZE)
		goto exit;

	if (len + buf_len == 0)
		goto exit;

	remain = CY_MAX_PRBUF_SIZE - cd->tthe_buf_len;
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

static int _cyttsp5_request_tthe_print(struct device *dev, u8 *buf,
		int buf_len, const u8 *data_name)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);

	return tthe_print(cd, buf, buf_len, data_name);
}
#endif

/*
 * cyttsp5_platform_detect_read()
 *
 * This function is passed to platform detect
 * function to perform a read operation
 */
static int cyttsp5_platform_detect_read(struct device *dev, void *buf, int size)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);

	return cyttsp5_adap_read_default(cd, buf, size);
}

/* Must be called with cd->hid_report_lock acquired */
static struct cyttsp5_hid_report *cyttsp5_get_hid_report_(
		struct cyttsp5_core_data *cd, u8 report_type, u8 report_id,
		bool create)
{
	struct cyttsp5_hid_report *report = NULL;
	int i;

	/* Look for created reports */
	for (i = 0; i < cd->num_hid_reports; i++) {
		if (cd->hid_reports[i]->type == report_type
				&& cd->hid_reports[i]->id == report_id) {
			return cd->hid_reports[i];
		}
	}

	/* Create a new report */
	if (create && cd->num_hid_reports < CY_HID_MAX_REPORTS) {
		report = kzalloc(sizeof(struct cyttsp5_hid_report),
				GFP_KERNEL);
		if (!report)
			return NULL;

		report->type = report_type;
		report->id = report_id;
		cd->hid_reports[cd->num_hid_reports++] = report;
	}

	return report;
}

/* Must be called with cd->hid_report_lock acquired */
static void cyttsp5_free_hid_reports_(struct cyttsp5_core_data *cd)
{
	struct cyttsp5_hid_report *report;
	int i, j;

	for (i = 0; i < cd->num_hid_reports; i++) {
		report = cd->hid_reports[i];
		for (j = 0; j < report->num_fields; j++)
			kfree(report->fields[j]);
		kfree(report);
		cd->hid_reports[i] = NULL;
	}

	cd->num_hid_reports = 0;
}

static void cyttsp5_free_hid_reports(struct cyttsp5_core_data *cd)
{
	mutex_lock(&cd->hid_report_lock);
	cyttsp5_free_hid_reports_(cd);
	mutex_unlock(&cd->hid_report_lock);
}

/* Must be called with cd->hid_report_lock acquired */
static struct cyttsp5_hid_field *cyttsp5_create_hid_field_(
		struct cyttsp5_hid_report *report)
{
	struct cyttsp5_hid_field *field;

	if (!report)
		return NULL;

	if (report->num_fields == CY_HID_MAX_FIELDS)
		return NULL;

	field = kzalloc(sizeof(struct cyttsp5_hid_field), GFP_KERNEL);
	if (!field)
		return NULL;

	field->report = report;

	report->fields[report->num_fields++] = field;

	return field;
}

/*******************************************************************************
 * FUNCTION: cyttsp5_add_parameter
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
static int cyttsp5_add_parameter(struct cyttsp5_core_data *cd,
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
static int pt_erase_parameter_list(struct cyttsp5_core_data *cd)
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
static int pt_count_parameter_list(struct cyttsp5_core_data *cd)
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
int request_exclusive(struct cyttsp5_core_data *cd, void *ownptr,
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

static int release_exclusive_(struct cyttsp5_core_data *cd, void *ownptr)
{
	if (cd->exclusive_dev != ownptr)
		return -EINVAL;

	pt_debug(cd->dev, DL_DEBUG, "%s: exclusive_dev %p freed\n",
		__func__, cd->exclusive_dev);
	cd->exclusive_dev = NULL;
	wake_up(&cd->wait_q);
	return 0;
}

/*
 * returns error if was not owned
 */
int release_exclusive(struct cyttsp5_core_data *cd, void *ownptr)
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
static int pt_hid_exec_cmd_(struct cyttsp5_core_data *cd,
		struct cyttsp5_hid_cmd *hid_cmd)
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
	rc = cyttsp5_adap_write_read_specific(cd, cmd_length, cmd,
			hid_cmd->read_buf);
	if (rc)
		pt_debug(cd->dev, DL_ERROR,
		"%s: Fail cyttsp5_adap_transfer\n", __func__);

	kfree(cmd);
	return rc;
}

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
static int pt_hid_exec_cmd_and_wait_(struct cyttsp5_core_data *cd,
		struct cyttsp5_hid_cmd *hid_cmd)
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
		timeout_ms = CY_HID_RESET_TIMEOUT;

	t = wait_event_timeout(cd->wait_q, (*cmd_state == 0),
			msecs_to_jiffies(timeout_ms));
	if (IS_TMO(t)) {
#ifdef TTDL_DIAGNOSTICS
		cd->i2c_transmission_error_count++;
#endif
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

static int cyttsp5_hid_cmd_reset_(struct cyttsp5_core_data *cd)
{
	struct cyttsp5_hid_cmd hid_cmd = {
		.opcode = HID_CMD_RESET,
		.wait_interrupt = 1,
		.reset_cmd = 1,
		.timeout_ms = CY_HID_RESET_TIMEOUT,
	};

	return pt_hid_exec_cmd_and_wait_(cd, &hid_cmd);
}

static int cyttsp5_hid_cmd_reset(struct cyttsp5_core_data *cd)
{
	int rc;

	rc = request_exclusive(cd, cd->dev, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		pt_debug(cd->dev, DL_ERROR,
			"%s: fail get exclusive ex=%p own=%p\n",
			__func__, cd->exclusive_dev, cd->dev);
		return rc;
	}

	rc = cyttsp5_hid_cmd_reset_(cd);

	if (release_exclusive(cd, cd->dev) < 0)
		pt_debug(cd->dev, DL_ERROR,
			"%s: fail to release exclusive\n", __func__);

	return rc;
}

static int cyttsp5_hid_cmd_set_power_(struct cyttsp5_core_data *cd,
		u8 power_state)
{
	int rc;
	struct cyttsp5_hid_cmd hid_cmd = {
		.opcode = HID_CMD_SET_POWER,
		.wait_interrupt = 1,
		.timeout_ms = CY_HID_SET_POWER_TIMEOUT,
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

static int cyttsp5_hid_cmd_set_power(struct cyttsp5_core_data *cd,
		u8 power_state)
{
	int rc;

	rc = request_exclusive(cd, cd->dev, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		pt_debug(cd->dev, DL_ERROR,
			"%s: fail get exclusive ex=%p own=%p\n",
			__func__, cd->exclusive_dev, cd->dev);
		return rc;
	}

	rc = cyttsp5_hid_cmd_set_power_(cd, power_state);

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

static u16 _cyttsp5_compute_crc(u8 *buf, u32 size)
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
static u8 pt_pip2_get_next_cmd_seq(struct cyttsp5_core_data *cd)
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
		.response_len = PIP2_EXTRA_BYTES_NUM + 10},
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

static int cyttsp5_hid_output_validate_bl_response(
		struct cyttsp5_core_data *cd,
		struct cyttsp5_hid_output *hid_output)
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
			"%s: HID output response, wrong report_id\n",
			__func__);
		return -EPROTO;
	}

	if (cd->response_buf[4] != HID_OUTPUT_BL_SOP) {
		pt_debug(cd->dev, DL_ERROR,
			"%s: HID output response, wrong SOP\n",
			__func__);
		return -EPROTO;
	}

	if (cd->response_buf[size - 1] != HID_OUTPUT_BL_EOP) {
		pt_debug(cd->dev, DL_ERROR,
			"%s: HID output response, wrong EOP\n",
			__func__);
		return -EPROTO;
	}

	crc = _cyttsp5_compute_crc(&cd->response_buf[4], size - 7);
	if (cd->response_buf[size - 3] != LOW_BYTE(crc)
			|| cd->response_buf[size - 2] != HI_BYTE(crc)) {
		pt_debug(cd->dev, DL_ERROR,
			"%s: HID output response, wrong CRC 0x%X\n",
			__func__, crc);
		return -EPROTO;
	}

	status = cd->response_buf[5];
	if (status) {
		pt_debug(cd->dev, DL_ERROR,
			"%s: HID output response, ERROR:%d\n",
			__func__, status);
		return -EPROTO;
	}

	return 0;
}

static int cyttsp5_hid_output_validate_app_response(
		struct cyttsp5_core_data *cd,
		struct cyttsp5_hid_output *hid_output)
{
	int command_code;
	u16 size;

	size = get_unaligned_le16(&cd->response_buf[0]);

	if (hid_output->reset_expected && !size)
		return 0;

	if (cd->response_buf[HID_OUTPUT_RESPONSE_REPORT_OFFSET]
			!= HID_APP_RESPONSE_REPORT_ID) {
		pt_debug(cd->dev, DL_ERROR,
			"%s: HID output response, wrong report_id\n",
			__func__);
		return -EPROTO;
	}

	command_code = cd->response_buf[HID_OUTPUT_RESPONSE_CMD_OFFSET]
		& HID_OUTPUT_RESPONSE_CMD_MASK;
	if (command_code != hid_output->command_code) {
		pt_debug(cd->dev, DL_ERROR,
			"%s: HID output response, wrong command_code:%X\n",
			__func__, command_code);
		return -EPROTO;
	}

	return 0;
}

static void cyttsp5_check_set_parameter(struct cyttsp5_core_data *cd,
		struct cyttsp5_hid_output *hid_output, bool raw)
{
	u8 *param_buf;
	u32 param_value = 0;
	u8 param_size;
	u8 param_id;
	int i = 0;

	if (!(cd->cpdata->flags & CY_CORE_FLAG_RESTORE_PARAMETERS))
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

	cyttsp5_add_parameter(cd, param_id, param_value, param_size);
}

static void cyttsp5_check_command(struct cyttsp5_core_data *cd,
		struct cyttsp5_hid_output *hid_output, bool raw)
{
	cyttsp5_check_set_parameter(cd, hid_output, raw);
}

static int cyttsp5_hid_output_validate_response(struct cyttsp5_core_data *cd,
		struct cyttsp5_hid_output *hid_output)
{
	if (hid_output->cmd_type == HID_OUTPUT_CMD_BL)
		return cyttsp5_hid_output_validate_bl_response(cd, hid_output);

	return cyttsp5_hid_output_validate_app_response(cd, hid_output);

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
static int pt_hid_send_output_user_(struct cyttsp5_core_data *cd,
		struct cyttsp5_hid_output *hid_output)
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
	rc = cyttsp5_adap_write_read_specific(cd, hid_output->length,
			hid_output->write_buf, NULL);
	if (rc)
		pt_debug(cd->dev, DL_ERROR,
			"%s: Fail cyttsp5_adap_transfer\n", __func__);

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
static int pt_hid_send_output_user_and_wait_(struct cyttsp5_core_data *cd,
		struct cyttsp5_hid_output *hid_output)
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
			msecs_to_jiffies(CY_HID_OUTPUT_USER_TIMEOUT));

	if (IS_TMO(t)) {
#ifdef TTDL_DIAGNOSTICS
		cd->i2c_transmission_error_count++;
#endif
		pt_debug(cd->dev, DL_ERROR,
			"%s: HID output cmd execution timed out\n",
			__func__);
		rc = -ETIME;
		goto error;
	}

	cyttsp5_check_command(cd, hid_output, true);

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
static ssize_t pt_flush_i2c(struct cyttsp5_core_data *cd, u8 force)
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
static int pt_hid_send_output_(struct cyttsp5_core_data *cd,
		struct cyttsp5_hid_output *hid_output)
{
	int rc;
	u8 *cmd;
	u16 length;
	u8 report_id;
	u8 cmd_offset = 0;
	u16 crc;
	u8 cmd_allocated = 0;

#if 0
	/*
	 * *** TODO - Determine side effects of adding this safty net ***
	 * If IRQ is already asserted due to a pending report, it must be
	 * cleared before sending command.
	 */
	if (cd->cpdata->irq_stat &&
	    cd->cpdata->irq_stat(cd->cpdata, cd->dev) ==
	    CY_IRQ_ASSERTED_VALUE) {
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

	if (length + 2 > CYTTSP5_PREALLOCATED_CMD_BUFFER) {
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
		crc = _cyttsp5_compute_crc(&cmd[6],
				hid_output->write_length + 4);
		cmd[cmd_offset++] = LOW_BYTE(crc);
		cmd[cmd_offset++] = HI_BYTE(crc);
		cmd[cmd_offset++] = HID_OUTPUT_BL_EOP;
	}

	pt_debug(cd->dev, DL_INFO,
		">>> %s: Write Buffer Size[%d] Cmd[0x%02X]\n",
		__func__, length + 2, hid_output->command_code);
	pt_pr_buf(cd->dev, DL_DEBUG, cmd, length + 2, ">>> Cmd");
	rc = cyttsp5_adap_write_read_specific(cd, length + 2, cmd, NULL);
	if (rc)
		pt_debug(cd->dev, DL_ERROR,
			"%s: Fail cyttsp5_adap_transfer\n", __func__);

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
static int pt_hid_send_output_and_wait_(struct cyttsp5_core_data *cd,
		struct cyttsp5_hid_output *hid_output)
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
		timeout_ms = CY_HID_OUTPUT_TIMEOUT;

	rc = pt_hid_send_output_(cd, hid_output);
	if (rc)
		goto error;


	t = wait_event_timeout(cd->wait_q, (cd->hid_cmd_state == 0),
			msecs_to_jiffies(timeout_ms));
	if (IS_TMO(t)) {
#ifdef TTDL_DIAGNOSTICS
		cd->i2c_transmission_error_count++;
#endif
		pt_debug(cd->dev, DL_ERROR,
			"%s: HID output cmd execution timed out\n",
			__func__);
		rc = -ETIME;
		goto error;
	}

	if (!hid_output->novalidate)
		rc = cyttsp5_hid_output_validate_response(cd, hid_output);

	cyttsp5_check_command(cd, hid_output, false);
	goto exit;

error:
	mutex_lock(&cd->system_lock);
	cd->hid_cmd_state = 0;
	mutex_unlock(&cd->system_lock);
exit:
	return rc;
}

static int cyttsp5_hid_output_null_(struct cyttsp5_core_data *cd)
{
	struct cyttsp5_hid_output hid_output = {
		HID_OUTPUT_APP_COMMAND(HID_OUTPUT_NULL),
	};

	return pt_hid_send_output_and_wait_(cd, &hid_output);
}

static int cyttsp5_hid_output_null(struct cyttsp5_core_data *cd)
{
	int rc;

	rc = request_exclusive(cd, cd->dev, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		pt_debug(cd->dev, DL_ERROR,
				"%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->dev);
		return rc;
	}

	rc = cyttsp5_hid_output_null_(cd);

	if (release_exclusive(cd, cd->dev) < 0)
		pt_debug(cd->dev, DL_ERROR,
				"%s: fail to release exclusive\n", __func__);

	return rc;
}

static void pt_stop_wd_timer(struct cyttsp5_core_data *cd);

/*******************************************************************************
 * FUNCTION: pt_hid_output_start_bootloader_
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
static int pt_hid_output_start_bootloader_(struct cyttsp5_core_data *cd)
{
	struct cyttsp5_hid_output hid_output = {
		HID_OUTPUT_APP_COMMAND(HID_OUTPUT_START_BOOTLOADER),
		.timeout_ms = CY_HID_OUTPUT_START_BOOTLOADER_TIMEOUT,
		.reset_expected = 1,
	};

	/* Entering the BL the WD must be stopped as the BL will not respond */
	pt_stop_wd_timer(cd);

	/* Reset startup status after entering BL, new DUT enum required */
	cd->startup_status = STARTUP_STATUS_START;
	return pt_hid_send_output_and_wait_(cd, &hid_output);
}

/*******************************************************************************
 * FUNCTION: pt_hid_output_start_bootloader
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
static int pt_hid_output_start_bootloader(struct cyttsp5_core_data *cd)
{
	int rc;

	rc = request_exclusive(cd, cd->dev, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		pt_debug(cd->dev, DL_ERROR,
				"%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->dev);
		return rc;
	}

	rc = pt_hid_output_start_bootloader_(cd);

	if (release_exclusive(cd, cd->dev) < 0)
		pt_debug(cd->dev, DL_ERROR,
				"%s: fail to release exclusive\n", __func__);

	return rc;
}

/*******************************************************************************
 * FUNCTION: _pt_request_hid_output_start_bl
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
static int _pt_request_hid_output_start_bl(struct device *dev, int protect)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);

	if (protect)
		return pt_hid_output_start_bootloader(cd);

	return pt_hid_output_start_bootloader_(cd);
}

static void cyttsp5_si_get_cydata(struct cyttsp5_core_data *cd)
{
	struct cyttsp5_cydata *cydata = &cd->sysinfo.cydata;
	struct cyttsp5_cydata_dev *cydata_dev =
		(struct cyttsp5_cydata_dev *)
		&cd->response_buf[HID_SYSINFO_CYDATA_OFFSET];

	cydata->pip_ver_major = cydata_dev->pip_ver_major;
	cydata->pip_ver_minor = cydata_dev->pip_ver_minor;
	cydata->bl_ver_major = cydata_dev->bl_ver_major;
	cydata->bl_ver_minor = cydata_dev->bl_ver_minor;
	cydata->fw_ver_major = cydata_dev->fw_ver_major;
	cydata->fw_ver_minor = cydata_dev->fw_ver_minor;

	cydata->fw_pid = get_unaligned_le16(&cydata_dev->fw_pid);
	cydata->fw_ver_conf = get_unaligned_le16(&cydata_dev->fw_ver_conf);
	cydata->post_code = get_unaligned_le16(&cydata_dev->post_code);
	cydata->revctrl = get_unaligned_le32(&cydata_dev->revctrl);
	cydata->jtag_id_l = get_unaligned_le16(&cydata_dev->jtag_si_id_l);
	cydata->jtag_id_h = get_unaligned_le16(&cydata_dev->jtag_si_id_h);

	memcpy(cydata->mfg_id, cydata_dev->mfg_id, CY_NUM_MFGID);

	pt_pr_buf(cd->dev, DL_INFO, (u8 *)cydata_dev,
		sizeof(struct cyttsp5_cydata_dev), "sysinfo_cydata");
}

static void cyttsp5_si_get_sensing_conf_data(struct cyttsp5_core_data *cd)
{
	struct cyttsp5_sensing_conf_data *scd = &cd->sysinfo.sensing_conf_data;
	struct cyttsp5_sensing_conf_data_dev *scd_dev =
		(struct cyttsp5_sensing_conf_data_dev *)
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
		sizeof(struct cyttsp5_sensing_conf_data_dev),
		"sensing_conf_data");
}

static int cyttsp5_si_setup(struct cyttsp5_core_data *cd)
{
	struct cyttsp5_sysinfo *si = &cd->sysinfo;
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

static int cyttsp5_si_get_btn_data(struct cyttsp5_core_data *cd)
{
	struct cyttsp5_sysinfo *si = &cd->sysinfo;
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
		btn_keys_size = num_btns * sizeof(struct cyttsp5_btn);
		if (!si->btn)
			si->btn = kzalloc(btn_keys_size, GFP_KERNEL);
		if (!si->btn)
			return -ENOMEM;

		if (cd->cpdata->sett[CY_IC_GRPNUM_BTN_KEYS] == NULL)
			num_defined_keys = 0;
		else if (cd->cpdata->sett[CY_IC_GRPNUM_BTN_KEYS]->data == NULL)
			num_defined_keys = 0;
		else
			num_defined_keys = cd->cpdata->sett
				[CY_IC_GRPNUM_BTN_KEYS]->size;

		for (btn = 0; btn < num_btns && btn < num_defined_keys; btn++) {
			key_table = (u16 *)cd->cpdata->sett
				[CY_IC_GRPNUM_BTN_KEYS]->data;
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

static void cyttsp5_si_put_log_data(struct cyttsp5_core_data *cd)
{
	struct cyttsp5_sysinfo *si = &cd->sysinfo;
	struct cyttsp5_cydata *cydata = &si->cydata;
	struct cyttsp5_sensing_conf_data *scd = &si->sensing_conf_data;
	int i;

	pt_debug(cd->dev, DL_DEBUG, "%s: pip_ver_major =0x%02X (%d)\n",
		__func__, cydata->pip_ver_major, cydata->pip_ver_major);
	pt_debug(cd->dev, DL_DEBUG, "%s: pip_ver_minor =0x%02X (%d)\n",
		__func__, cydata->pip_ver_minor, cydata->pip_ver_minor);
	pt_debug(cd->dev, DL_DEBUG, "%s: fw_pid =0x%04X (%d)\n",
		__func__, cydata->fw_pid, cydata->fw_pid);
	pt_debug(cd->dev, DL_DEBUG, "%s: fw_ver_major =0x%02X (%d)\n",
		__func__, cydata->fw_ver_major, cydata->fw_ver_major);
	pt_debug(cd->dev, DL_DEBUG, "%s: fw_ver_minor =0x%02X (%d)\n",
		__func__, cydata->fw_ver_minor, cydata->fw_ver_minor);
	pt_debug(cd->dev, DL_DEBUG, "%s: revctrl =0x%08X (%d)\n",
		__func__, cydata->revctrl, cydata->revctrl);
	pt_debug(cd->dev, DL_DEBUG, "%s: fw_ver_conf =0x%04X (%d)\n",
		__func__, cydata->fw_ver_conf, cydata->fw_ver_conf);
	pt_debug(cd->dev, DL_DEBUG, "%s: bl_ver_major =0x%02X (%d)\n",
		__func__, cydata->bl_ver_major, cydata->bl_ver_major);
	pt_debug(cd->dev, DL_DEBUG, "%s: bl_ver_minor =0x%02X (%d)\n",
		__func__, cydata->bl_ver_minor, cydata->bl_ver_minor);
	pt_debug(cd->dev, DL_DEBUG, "%s: jtag_id_h =0x%04X (%d)\n",
		__func__, cydata->jtag_id_h, cydata->jtag_id_h);
	pt_debug(cd->dev, DL_DEBUG, "%s: jtag_id_l =0x%04X (%d)\n",
		__func__, cydata->jtag_id_l, cydata->jtag_id_l);

	for (i = 0; i < CY_NUM_MFGID; i++)
		pt_debug(cd->dev, DL_DEBUG,
			"%s: mfg_id[%d] =0x%02X (%d)\n",
			__func__, i, cydata->mfg_id[i],
			cydata->mfg_id[i]);

	pt_debug(cd->dev, DL_DEBUG, "%s: post_code =0x%04X (%d)\n",
		__func__, cydata->post_code, cydata->post_code);
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

static int cyttsp5_get_sysinfo_regs(struct cyttsp5_core_data *cd)
{
	struct cyttsp5_sysinfo *si = &cd->sysinfo;
	int rc;

	rc = cyttsp5_si_get_btn_data(cd);
	if (rc < 0)
		return rc;

	cyttsp5_si_get_cydata(cd);

	cyttsp5_si_get_sensing_conf_data(cd);

	cyttsp5_si_setup(cd);

	cyttsp5_si_put_log_data(cd);

	si->ready = true;
	return rc;
}

static void cyttsp5_free_si_ptrs(struct cyttsp5_core_data *cd)
{
	struct cyttsp5_sysinfo *si = &cd->sysinfo;

	kfree(si->btn);
	kfree(si->xy_mode);
	kfree(si->xy_data);
}

static int cyttsp5_hid_output_get_sysinfo_(struct cyttsp5_core_data *cd)
{
	int rc;
	struct cyttsp5_hid_output hid_output = {
		HID_OUTPUT_APP_COMMAND(HID_OUTPUT_GET_SYSINFO),
		.timeout_ms = CY_HID_OUTPUT_GET_SYSINFO_TIMEOUT,
	};

	rc = pt_hid_send_output_and_wait_(cd, &hid_output);
	if (rc)
		return rc;

	rc = cyttsp5_get_sysinfo_regs(cd);
	if (rc)
		cyttsp5_free_si_ptrs(cd);

	return rc;
}

static int cyttsp5_hid_output_get_sysinfo(struct cyttsp5_core_data *cd)
{
	int rc;

	rc = request_exclusive(cd, cd->dev, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		pt_debug(cd->dev, DL_ERROR,
				"%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->dev);
		return rc;
	}

	rc = cyttsp5_hid_output_get_sysinfo_(cd);

	if (release_exclusive(cd, cd->dev) < 0)
		pt_debug(cd->dev, DL_ERROR,
				"%s: fail to release exclusive\n", __func__);

	return rc;
}

/*******************************************************************************
 * FUNCTION: pt_hid_output_suspend_scanning_
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
static int pt_hid_output_suspend_scanning_(struct cyttsp5_core_data *cd)
{
	struct cyttsp5_hid_output hid_output = {
		HID_OUTPUT_APP_COMMAND(HID_OUTPUT_SUSPEND_SCANNING),
	};

	return pt_hid_send_output_and_wait_(cd, &hid_output);
}

/*******************************************************************************
 * FUNCTION: pt_hid_output_suspend_scanning
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
static int pt_hid_output_suspend_scanning(struct cyttsp5_core_data *cd)
{
	int rc;

	rc = request_exclusive(cd, cd->dev, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		pt_debug(cd->dev, DL_ERROR,
				"%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->dev);
		return rc;
	}

	rc = pt_hid_output_suspend_scanning_(cd);

	if (release_exclusive(cd, cd->dev) < 0)
		pt_debug(cd->dev, DL_ERROR,
				"%s: fail to release exclusive\n", __func__);

	return rc;
}

/*******************************************************************************
 * FUNCTION: _pt_request_hid_output_suspend_scanning
 *
 * SUMMARY: Wrapper for calling either the protected or non protected function
 *	to suspend scanning
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
static int _pt_request_hid_output_suspend_scanning(struct device *dev,
		int protect)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);

	if (protect)
		return pt_hid_output_suspend_scanning(cd);

	return pt_hid_output_suspend_scanning_(cd);
}

/*******************************************************************************
 * FUNCTION: pt_hid_output_resume_scanning_
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
static int pt_hid_output_resume_scanning_(struct cyttsp5_core_data *cd)
{
	struct cyttsp5_hid_output hid_output = {
		HID_OUTPUT_APP_COMMAND(HID_OUTPUT_RESUME_SCANNING),
	};

	return pt_hid_send_output_and_wait_(cd, &hid_output);
}

/*******************************************************************************
 * FUNCTION: pt_hid_output_resume_scanning
 *
 * SUMMARY: Protected wrapper for calling pt_hid_output_resume_scanning_
 *
 * RETURN::
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*cd - pointer to core data structure
 ******************************************************************************/
static int pt_hid_output_resume_scanning(struct cyttsp5_core_data *cd)
{
	int rc;

	rc = request_exclusive(cd, cd->dev, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		pt_debug(cd->dev, DL_ERROR,
				"%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->dev);
		return rc;
	}

	rc = pt_hid_output_resume_scanning_(cd);

	if (release_exclusive(cd, cd->dev) < 0)
		pt_debug(cd->dev, DL_ERROR,
				"%s: fail to release exclusive\n", __func__);

	return rc;
}

/*******************************************************************************
 * FUNCTION: _pt_request_hid_output_resume_scanning
 *
 * SUMMARY: Wrapper for calling either the protected or non protected function
 *	to resume scanning
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
static int _pt_request_hid_output_resume_scanning(struct device *dev,
		int protect)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);

	if (protect)
		return pt_hid_output_resume_scanning(cd);

	return pt_hid_output_resume_scanning_(cd);
}

/*******************************************************************************
 * FUNCTION: pt_hid_output_get_param_
 *
 * SUMMARY: Sends a HID command 0x05 Get Parameter to the DUT and returns
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
static int pt_hid_output_get_param_(struct cyttsp5_core_data *cd,
		u8 param_id, u32 *value)
{
	int write_length = 1;
	u8 param[1] = { param_id };
	u8 read_param_id;
	int param_size;
	u8 *ptr;
	int rc;
	int i;
	struct cyttsp5_hid_output hid_output = {
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
 * FUNCTION: pt_hid_output_get_param
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
static int pt_hid_output_get_param(struct cyttsp5_core_data *cd,
		u8 param_id, u32 *value)
{
	int rc;

	rc = request_exclusive(cd, cd->dev, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		pt_debug(cd->dev, DL_ERROR,
				"%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->dev);
		return rc;
	}

	rc = pt_hid_output_get_param_(cd, param_id, value);

	if (release_exclusive(cd, cd->dev) < 0)
		pt_debug(cd->dev, DL_ERROR,
				"%s: fail to release exclusive\n", __func__);

	return rc;
}

/*******************************************************************************
 * FUNCTION: _pt_request_hid_output_get_param
 *
 * SUMMARY: Function pointer included in core_nonhid_cmd struct for external
 *	calls to the protected or unprotected call to pt_hid_output_get_param
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*cd       - pointer to core data
 *	 protect  - flag to call protected or non protected function
 *	 param_id - parameter ID to retrieve
 *	*value    - value of DUT parameter
 ******************************************************************************/
int _pt_request_hid_output_get_param(struct device *dev,
		int protect, u8 param_id, u32 *value)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);

	if (protect)
		return pt_hid_output_get_param(cd, param_id, value);

	return pt_hid_output_get_param_(cd, param_id, value);
}

static int cyttsp5_hid_output_set_param_(struct cyttsp5_core_data *cd,
		u8 param_id, u32 value, u8 size)
{
	u8 write_buf[6];
	u8 *ptr = &write_buf[2];
	int rc;
	int i;
	struct cyttsp5_hid_output hid_output = {
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

static int cyttsp5_hid_output_set_param(struct cyttsp5_core_data *cd,
		u8 param_id, u32 value, u8 size)
{
	int rc;

	rc = request_exclusive(cd, cd->dev, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		pt_debug(cd->dev, DL_ERROR,
				"%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->dev);
		return rc;
	}

	rc = cyttsp5_hid_output_set_param_(cd, param_id, value, size);

	if (release_exclusive(cd, cd->dev) < 0)
		pt_debug(cd->dev, DL_ERROR,
				"%s: fail to release exclusive\n", __func__);

	return rc;
}

int _cyttsp5_request_hid_output_set_param(struct device *dev,
		int protect, u8 param_id, u32 value, u8  size)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);

	if (protect)
		return cyttsp5_hid_output_set_param(cd, param_id, value, size);

	return cyttsp5_hid_output_set_param_(cd, param_id, value, size);
}

static int cyttsp5_hid_output_enter_easywake_state_(
		struct cyttsp5_core_data *cd, u8 data, u8 *return_data)
{
	int write_length = 1;
	u8 param[1] = { data };
	int rc;
	struct cyttsp5_hid_output hid_output = {
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

static int cyttsp5_hid_output_verify_config_block_crc_(
		struct cyttsp5_core_data *cd, u8 ebid, u8 *status,
		u16 *calculated_crc, u16 *stored_crc)
{
	int write_length = 1;
	u8 param[1] = { ebid };
	u8 *ptr;
	int rc;
	struct cyttsp5_hid_output hid_output = {
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

static int cyttsp5_hid_output_verify_config_block_crc(
		struct cyttsp5_core_data *cd, u8 ebid, u8 *status,
		u16 *calculated_crc, u16 *stored_crc)
{
	int rc;

	rc = request_exclusive(cd, cd->dev, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		pt_debug(cd->dev, DL_ERROR,
				"%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->dev);
		return rc;
	}

	rc = cyttsp5_hid_output_verify_config_block_crc_(cd, ebid, status,
			calculated_crc, stored_crc);

	if (release_exclusive(cd, cd->dev) < 0)
		pt_debug(cd->dev, DL_ERROR,
				"%s: fail to release exclusive\n", __func__);

	return rc;
}

static int _cyttsp5_request_hid_output_verify_config_block_crc(
		struct device *dev, int protect, u8 ebid, u8 *status,
		u16 *calculated_crc, u16 *stored_crc)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);

	if (protect)
		return cyttsp5_hid_output_verify_config_block_crc(cd, ebid,
				status, calculated_crc, stored_crc);

	return cyttsp5_hid_output_verify_config_block_crc_(cd, ebid,
			status, calculated_crc, stored_crc);
}

static int cyttsp5_hid_output_get_config_row_size_(struct cyttsp5_core_data *cd,
		u16 *row_size)
{
	int rc;
	struct cyttsp5_hid_output hid_output = {
		HID_OUTPUT_APP_COMMAND(HID_OUTPUT_GET_CONFIG_ROW_SIZE),
	};

	rc = pt_hid_send_output_and_wait_(cd, &hid_output);
	if (rc)
		return rc;

	*row_size = get_unaligned_le16(&cd->response_buf[5]);
	return 0;
}

static int cyttsp5_hid_output_get_config_row_size(struct cyttsp5_core_data *cd,
		u16 *row_size)
{
	int rc;

	rc = request_exclusive(cd, cd->dev, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		pt_debug(cd->dev, DL_ERROR,
				"%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->dev);
		return rc;
	}

	rc = cyttsp5_hid_output_get_config_row_size_(cd, row_size);

	if (release_exclusive(cd, cd->dev) < 0)
		pt_debug(cd->dev, DL_ERROR,
				"%s: fail to release exclusive\n", __func__);

	return rc;
}

static int _cyttsp5_request_hid_output_get_config_row_size(struct device *dev,
		int protect, u16 *row_size)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);

	if (protect)
		return cyttsp5_hid_output_get_config_row_size(cd, row_size);

	return cyttsp5_hid_output_get_config_row_size_(cd, row_size);
}

static int cyttsp5_hid_output_read_conf_block_(struct cyttsp5_core_data *cd,
		u16 row_number, u16 length, u8 ebid, u8 *read_buf, u16 *crc)
{
	int read_ebid;
	int read_length;
	int status;
	int rc;
	int write_length = 5;
	u8 write_buf[5];
	u8 cmd_offset = 0;
	struct cyttsp5_hid_output hid_output = {
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

static int cyttsp5_hid_output_read_conf_ver_(struct cyttsp5_core_data *cd,
		u16 *config_ver)
{
	int rc;
	u8 read_buf[CY_TTCONFIG_VERSION_OFFSET + CY_TTCONFIG_VERSION_SIZE];
	u16 crc;

	rc = cyttsp5_hid_output_read_conf_block_(cd, CY_TTCONFIG_VERSION_ROW,
			CY_TTCONFIG_VERSION_OFFSET + CY_TTCONFIG_VERSION_SIZE,
			CY_TCH_PARM_EBID, read_buf, &crc);
	if (rc)
		return rc;

	*config_ver = get_unaligned_le16(
				&read_buf[CY_TTCONFIG_VERSION_OFFSET]);

	return 0;
}

static int cyttsp5_hid_output_write_conf_block_(struct cyttsp5_core_data *cd,
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
	struct cyttsp5_hid_output hid_output = {
		HID_OUTPUT_APP_COMMAND(HID_OUTPUT_WRITE_CONF_BLOCK),
		.write_length = full_write_length,
		.timeout_ms = CY_HID_OUTPUT_WRITE_CONF_BLOCK_TIMEOUT,
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
	crc = _cyttsp5_compute_crc(data, write_length);
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

static int cyttsp5_hid_output_write_conf_block(struct cyttsp5_core_data *cd,
		u16 row_number, u16 write_length, u8 ebid, u8 *write_buf,
		u8 *security_key, u16 *actual_write_len)
{
	int rc;

	rc = request_exclusive(cd, cd->dev, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		pt_debug(cd->dev, DL_ERROR,
				"%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->dev);
		return rc;
	}

	rc = cyttsp5_hid_output_write_conf_block_(cd, row_number, write_length,
			ebid, write_buf, security_key, actual_write_len);

	if (release_exclusive(cd, cd->dev) < 0)
		pt_debug(cd->dev, DL_ERROR,
				"%s: fail to release exclusive\n", __func__);

	return rc;
}

static int _cyttsp5_request_hid_output_write_conf_block(struct device *dev,
		int protect, u16 row_number, u16 write_length, u8 ebid,
		u8 *write_buf, u8 *security_key, u16 *actual_write_len)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);

	if (protect)
		return cyttsp5_hid_output_write_conf_block(cd, row_number,
				write_length, ebid, write_buf, security_key,
				actual_write_len);

	return cyttsp5_hid_output_write_conf_block_(cd, row_number,
			write_length, ebid, write_buf, security_key,
			actual_write_len);
}

static int cyttsp5_hid_output_get_data_structure_(
		struct cyttsp5_core_data *cd, u16 read_offset, u16 read_length,
		u8 data_id, u8 *status, u8 *data_format, u16 *actual_read_len,
		u8 *data)
{
	int rc;
	u16 total_read_len = 0;
	u16 read_len;
	u16 off_buf = 0;
	u8 write_buf[5];
	u8 read_data_id;
	struct cyttsp5_hid_output hid_output = {
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

	if (cd->response_buf[5] != CY_CMD_STATUS_SUCCESS)
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

static int cyttsp5_hid_output_get_data_structure(
		struct cyttsp5_core_data *cd, u16 read_offset, u16 read_length,
		u8 data_id, u8 *status, u8 *data_format, u16 *actual_read_len,
		u8 *data)
{
	int rc;

	rc = request_exclusive(cd, cd->dev, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		pt_debug(cd->dev, DL_ERROR,
				"%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->dev);
		return rc;
	}

	rc = cyttsp5_hid_output_get_data_structure_(cd, read_offset,
			read_length, data_id, status, data_format,
			actual_read_len, data);

	if (release_exclusive(cd, cd->dev) < 0)
		pt_debug(cd->dev, DL_ERROR,
			"%s: fail to release exclusive\n", __func__);

	return rc;
}

static int _cyttsp5_request_hid_output_get_data_structure(struct device *dev,
		int protect, u16 read_offset, u16 read_length, u8 data_id,
		u8 *status, u8 *data_format, u16 *actual_read_len, u8 *data)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);

	if (protect)
		return cyttsp5_hid_output_get_data_structure(cd,
				read_offset, read_length, data_id, status,
				data_format, actual_read_len, data);

	return cyttsp5_hid_output_get_data_structure_(cd,
			read_offset, read_length, data_id, status,
			data_format, actual_read_len, data);
}

static int cyttsp5_hid_output_run_selftest_(
		struct cyttsp5_core_data *cd, u8 test_id,
		u8 write_idacs_to_flash, u8 *status, u8 *summary_result,
		u8 *results_available)
{
	int rc;
	u8 write_buf[2];
	struct cyttsp5_hid_output hid_output = {
		HID_OUTPUT_APP_COMMAND(HID_OUTPUT_RUN_SELF_TEST),
		.write_length = 2,
		.write_buf = write_buf,
		.timeout_ms = CY_HID_OUTPUT_RUN_SELF_TEST_TIMEOUT,
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

static int cyttsp5_hid_output_run_selftest(
		struct cyttsp5_core_data *cd, u8 test_id,
		u8 write_idacs_to_flash, u8 *status, u8 *summary_result,
		u8 *results_available)
{
	int rc;

	rc = request_exclusive(cd, cd->dev, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		pt_debug(cd->dev, DL_ERROR,
				"%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->dev);
		return rc;
	}

	rc = cyttsp5_hid_output_run_selftest_(cd, test_id,
			write_idacs_to_flash, status, summary_result,
			results_available);

	if (release_exclusive(cd, cd->dev) < 0)
		pt_debug(cd->dev, DL_ERROR,
				"%s: fail to release exclusive\n", __func__);

	return rc;
}

static int _cyttsp5_request_hid_output_run_selftest(struct device *dev,
		int protect, u8 test_id, u8 write_idacs_to_flash, u8 *status,
		u8 *summary_result, u8 *results_available)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);

	if (protect)
		return cyttsp5_hid_output_run_selftest(cd, test_id,
				write_idacs_to_flash, status, summary_result,
				results_available);

	return cyttsp5_hid_output_run_selftest_(cd, test_id,
			write_idacs_to_flash, status, summary_result,
			results_available);
}

static int cyttsp5_hid_output_get_selftest_result_(
		struct cyttsp5_core_data *cd, u16 read_offset, u16 read_length,
		u8 test_id, u8 *status, u16 *actual_read_len, u8 *data)
{
	int rc;
	u16 total_read_len = 0;
	u16 read_len;
	u16 off_buf = 0;
	u8 write_buf[5];
	u8 read_test_id;
	bool repeat;
	struct cyttsp5_hid_output hid_output = {
		HID_OUTPUT_APP_COMMAND(HID_OUTPUT_GET_SELF_TEST_RESULT),
		.write_length = 5,
		.write_buf = write_buf,
	};

	/*
	 * Do not repeat reading for Auto Shorts test
	 * when PIP version < 1.3
	 */
	repeat = IS_PIP_VER_GE(&cd->sysinfo, 1, 3)
			|| test_id != CY_ST_ID_AUTOSHORTS;

again:
	write_buf[0] = LOW_BYTE(read_offset);
	write_buf[1] = HI_BYTE(read_offset);
	write_buf[2] = LOW_BYTE(read_length);
	write_buf[3] = HI_BYTE(read_length);
	write_buf[4] = test_id;

	rc = pt_hid_send_output_and_wait_(cd, &hid_output);
	if (rc)
		return rc;

	if (cd->response_buf[5] != CY_CMD_STATUS_SUCCESS)
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

static int cyttsp5_hid_output_get_selftest_result(
		struct cyttsp5_core_data *cd, u16 read_offset, u16 read_length,
		u8 test_id, u8 *status, u16 *actual_read_len, u8 *data)
{
	int rc;

	rc = request_exclusive(cd, cd->dev, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		pt_debug(cd->dev, DL_ERROR,
				"%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->dev);
		return rc;
	}

	rc = cyttsp5_hid_output_get_selftest_result_(cd, read_offset,
			read_length, test_id, status, actual_read_len, data);

	if (release_exclusive(cd, cd->dev) < 0)
		pt_debug(cd->dev, DL_ERROR,
				"%s: fail to release exclusive\n", __func__);

	return rc;
}

static int _cyttsp5_request_hid_output_get_selftest_result(struct device *dev,
		int protect, u16 read_offset, u16 read_length, u8 test_id,
		u8 *status, u16 *actual_read_len, u8 *data)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);

	if (protect)
		return cyttsp5_hid_output_get_selftest_result(cd, read_offset,
				read_length, test_id, status, actual_read_len,
				data);

	return cyttsp5_hid_output_get_selftest_result_(cd, read_offset,
			read_length, test_id, status, actual_read_len,
			data);
}

static int cyttsp5_hid_output_calibrate_idacs_(struct cyttsp5_core_data *cd,
		u8 mode, u8 *status)
{
	int rc;
	int write_length = 1;
	u8 write_buf[1];
	u8 cmd_offset = 0;
	struct cyttsp5_hid_output hid_output = {
		HID_OUTPUT_APP_COMMAND(HID_OUTPUT_CALIBRATE_IDACS),
		.write_length = write_length,
		.write_buf = write_buf,
		.timeout_ms = CY_HID_OUTPUT_CALIBRATE_IDAC_TIMEOUT,
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

static int cyttsp5_hid_output_calibrate_idacs(struct cyttsp5_core_data *cd,
		u8 mode, u8 *status)
{
	int rc;

	rc = request_exclusive(cd, cd->dev, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		pt_debug(cd->dev, DL_ERROR,
				"%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->dev);
		return rc;
	}

	rc = cyttsp5_hid_output_calibrate_idacs_(cd, mode, status);

	if (release_exclusive(cd, cd->dev) < 0)
		pt_debug(cd->dev, DL_ERROR,
				"%s: fail to release exclusive\n", __func__);

	return rc;
}

static int _cyttsp5_request_hid_output_calibrate_idacs(struct device *dev,
		int protect, u8 mode, u8 *status)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);

	if (protect)
		return cyttsp5_hid_output_calibrate_idacs(cd, mode, status);

	return cyttsp5_hid_output_calibrate_idacs_(cd, mode, status);
}

static int cyttsp5_hid_output_initialize_baselines_(
		struct cyttsp5_core_data *cd, u8 test_id, u8 *status)
{
	int rc;
	int write_length = 1;
	u8 write_buf[1];
	u8 cmd_offset = 0;
	struct cyttsp5_hid_output hid_output = {
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

static int cyttsp5_hid_output_initialize_baselines(struct cyttsp5_core_data *cd,
		u8 test_id, u8 *status)
{
	int rc;

	rc = request_exclusive(cd, cd->dev, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		pt_debug(cd->dev, DL_ERROR,
				"%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->dev);
		return rc;
	}

	rc = cyttsp5_hid_output_initialize_baselines_(cd, test_id, status);

	if (release_exclusive(cd, cd->dev) < 0)
		pt_debug(cd->dev, DL_ERROR,
				"%s: fail to release exclusive\n", __func__);

	return rc;
}

static int _cyttsp5_request_hid_output_initialize_baselines(struct device *dev,
		int protect, u8 test_id, u8 *status)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);

	if (protect)
		return cyttsp5_hid_output_initialize_baselines(cd, test_id,
				status);

	return cyttsp5_hid_output_initialize_baselines_(cd, test_id, status);
}

static int cyttsp5_hid_output_exec_panel_scan_(struct cyttsp5_core_data *cd)
{
	struct cyttsp5_hid_output hid_output = {
		HID_OUTPUT_APP_COMMAND(HID_OUTPUT_EXEC_PANEL_SCAN),
	};

	return pt_hid_send_output_and_wait_(cd, &hid_output);
}

static int cyttsp5_hid_output_exec_panel_scan(struct cyttsp5_core_data *cd)
{
	int rc;

	rc = request_exclusive(cd, cd->dev, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		pt_debug(cd->dev, DL_ERROR,
				"%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->dev);
		return rc;
	}

	rc = cyttsp5_hid_output_exec_panel_scan_(cd);

	if (release_exclusive(cd, cd->dev) < 0)
		pt_debug(cd->dev, DL_ERROR,
				"%s: fail to release exclusive\n", __func__);

	return rc;
}

static int _cyttsp5_request_hid_output_exec_panel_scan(struct device *dev,
		int protect)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);

	if (protect)
		return cyttsp5_hid_output_exec_panel_scan(cd);

	return cyttsp5_hid_output_exec_panel_scan_(cd);
}

/* @response: set none NULL only if all response required including header */
static int cyttsp5_hid_output_retrieve_panel_scan_(
		struct cyttsp5_core_data *cd, u16 read_offset, u16 read_count,
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
	struct cyttsp5_hid_output hid_output = {
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

static int cyttsp5_hid_output_retrieve_panel_scan(
		struct cyttsp5_core_data *cd, u16 read_offset, u16 read_count,
		u8 data_id, u8 *response, u8 *config, u16 *actual_read_len,
		u8 *read_buf)
{
	int rc;

	rc = request_exclusive(cd, cd->dev, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		pt_debug(cd->dev, DL_ERROR,
				"%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->dev);
		return rc;
	}

	rc = cyttsp5_hid_output_retrieve_panel_scan_(cd, read_offset,
			read_count, data_id, response, config,
			actual_read_len, read_buf);

	if (release_exclusive(cd, cd->dev) < 0)
		pt_debug(cd->dev, DL_ERROR,
				"%s: fail to release exclusive\n", __func__);

	return rc;
}

static int _cyttsp5_request_hid_output_retrieve_panel_scan(struct device *dev,
		int protect, u16 read_offset, u16 read_count, u8 data_id,
		u8 *response, u8 *config, u16 *actual_read_len, u8 *read_buf)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);

	if (protect)
		return cyttsp5_hid_output_retrieve_panel_scan(cd,
				read_offset, read_count, data_id, response,
				config, actual_read_len, read_buf);

	return cyttsp5_hid_output_retrieve_panel_scan_(cd,
			read_offset, read_count, data_id, response,
			config, actual_read_len, read_buf);
}

/*******************************************************************************
 * FUNCTION: cyttsp5_hid_output_user_cmd_
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
static int cyttsp5_hid_output_user_cmd_(struct cyttsp5_core_data *cd,
		u16 read_len, u8 *read_buf, u16 write_len, u8 *write_buf,
		u16 *actual_read_len)
{
	int rc;
	u16 size;
#ifdef TTHE_TUNER_SUPPORT
	int command_code = 0;
	int len;
#endif
	struct cyttsp5_hid_output hid_output = {
		.length = write_len,
		.write_buf = write_buf,
	};

	rc = pt_hid_send_output_user_and_wait_(cd, &hid_output);
	if (rc)
		return rc;

	/* Get the response size from the first 2 bytes in the response */
	size = get_unaligned_le16(&cd->response_buf[0]);

	/* Ensure size is not greater than max buffer size */
	if (size > CY_MAX_PRBUF_SIZE - PIP2_LEN_FIELD_SIZE)
		size = CY_MAX_PRBUF_SIZE - PIP2_LEN_FIELD_SIZE;

	/* PIP2.0 packet length doesn't include the size of the len field */
	if (cd->pip_2p0_prot_active)
		size = size + PIP2_LEN_FIELD_SIZE;

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

static int cyttsp5_get_config_ver_(struct cyttsp5_core_data *cd)
{
	struct cyttsp5_sysinfo *si = &cd->sysinfo;
	int rc;
	u16 config_ver = 0;

	rc = pt_hid_output_suspend_scanning_(cd);
	if (rc)
		goto error;

	rc = cyttsp5_hid_output_read_conf_ver_(cd, &config_ver);
	if (rc)
		goto exit;

	si->cydata.fw_ver_conf = config_ver;

exit:
	pt_hid_output_resume_scanning_(cd);
error:
	pt_debug(cd->dev, DL_ERROR, "%s: CONFIG_VER:%04X\n",
		__func__, config_ver);
	return rc;
}

static int cyttsp5_hid_output_user_cmd(struct cyttsp5_core_data *cd,
		u16 read_len, u8 *read_buf, u16 write_len, u8 *write_buf,
		u16 *actual_read_len)
{
	int rc;

	rc = request_exclusive(cd, cd->dev, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		pt_debug(cd->dev, DL_ERROR,
				"%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->dev);
		return rc;
	}

	rc = cyttsp5_hid_output_user_cmd_(cd, read_len, read_buf,
			write_len, write_buf, actual_read_len);

	if (release_exclusive(cd, cd->dev) < 0)
		pt_debug(cd->dev, DL_ERROR,
				"%s: fail to release exclusive\n", __func__);

	return rc;
}

static int _cyttsp5_request_hid_output_user_cmd(struct device *dev,
		int protect, u16 read_len, u8 *read_buf, u16 write_len,
		u8 *write_buf, u16 *actual_read_len)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);

	if (protect)
		return cyttsp5_hid_output_user_cmd(cd, read_len, read_buf,
				write_len, write_buf, actual_read_len);

	return cyttsp5_hid_output_user_cmd_(cd, read_len, read_buf,
			write_len, write_buf, actual_read_len);
}

static int cyttsp5_hid_output_bl_get_information_(struct cyttsp5_core_data *cd,
		u8 *return_data)
{
	int rc;
	int data_len;
	struct cyttsp5_hid_output hid_output = {
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

static int cyttsp5_hid_output_bl_get_information(struct cyttsp5_core_data *cd,
		u8 *return_data)
{
	int rc;

	rc = request_exclusive(cd, cd->dev, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		pt_debug(cd->dev, DL_ERROR,
				"%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->dev);
		return rc;
	}

	rc = cyttsp5_hid_output_bl_get_information_(cd, return_data);

	if (release_exclusive(cd, cd->dev) < 0)
		pt_debug(cd->dev, DL_ERROR,
				"%s: fail to release exclusive\n", __func__);

	return rc;
}

static int _cyttsp5_request_hid_output_bl_get_information(struct device *dev,
		int protect, u8 *return_data)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);

	if (protect)
		return cyttsp5_hid_output_bl_get_information(cd, return_data);

	return cyttsp5_hid_output_bl_get_information_(cd, return_data);
}

static int cyttsp5_hid_output_bl_initiate_bl_(struct cyttsp5_core_data *cd,
		u16 key_size, u8 *key_buf, u16 row_size, u8 *metadata_row_buf)
{
	u16 write_length = key_size + row_size;
	u8 *write_buf;
	int rc;
	struct cyttsp5_hid_output hid_output = {
		HID_OUTPUT_BL_COMMAND(HID_OUTPUT_BL_INITIATE_BL),
		.write_length = write_length,
		.timeout_ms = CY_HID_OUTPUT_BL_INITIATE_BL_TIMEOUT,
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

static int cyttsp5_hid_output_bl_initiate_bl(struct cyttsp5_core_data *cd,
		u16 key_size, u8 *key_buf, u16 row_size, u8 *metadata_row_buf)
{
	int rc;

	rc = request_exclusive(cd, cd->dev, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		pt_debug(cd->dev, DL_ERROR,
				"%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->dev);
		return rc;
	}

	rc = cyttsp5_hid_output_bl_initiate_bl_(cd, key_size, key_buf,
			row_size, metadata_row_buf);

	if (release_exclusive(cd, cd->dev) < 0)
		pt_debug(cd->dev, DL_ERROR,
				"%s: fail to release exclusive\n", __func__);

	return rc;
}

static int _cyttsp5_request_hid_output_bl_initiate_bl(struct device *dev,
		int protect, u16 key_size, u8 *key_buf, u16 row_size,
		u8 *metadata_row_buf)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);

	if (protect)
		return cyttsp5_hid_output_bl_initiate_bl(cd, key_size, key_buf,
				row_size, metadata_row_buf);

	return cyttsp5_hid_output_bl_initiate_bl_(cd, key_size, key_buf,
			row_size, metadata_row_buf);
}

static int cyttsp5_hid_output_bl_program_and_verify_(
		struct cyttsp5_core_data *cd, u16 data_len, u8 *data_buf)
{
	struct cyttsp5_hid_output hid_output = {
		HID_OUTPUT_BL_COMMAND(HID_OUTPUT_BL_PROGRAM_AND_VERIFY),
		.write_length = data_len,
		.write_buf = data_buf,
		.timeout_ms = CY_HID_OUTPUT_BL_PROGRAM_AND_VERIFY_TIMEOUT,
	};

	return pt_hid_send_output_and_wait_(cd, &hid_output);
}

static int cyttsp5_hid_output_bl_program_and_verify(
		struct cyttsp5_core_data *cd, u16 data_len, u8 *data_buf)
{
	int rc;

	rc = request_exclusive(cd, cd->dev, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		pt_debug(cd->dev, DL_ERROR,
				"%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->dev);
		return rc;
	}

	rc = cyttsp5_hid_output_bl_program_and_verify_(cd, data_len, data_buf);

	if (release_exclusive(cd, cd->dev) < 0)
		pt_debug(cd->dev, DL_ERROR,
				"%s: fail to release exclusive\n", __func__);

	return rc;
}

/*******************************************************************************
 * FUNCTION: _pt_request_hid_output_bl_program_and_verify
 *
 * SUMMARY: Function pointer included in core_nonhid_cmds to allow other modules
 *	to requst to have the BL program and verify a FW image
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
static int _pt_request_hid_output_bl_program_and_verify(
		struct device *dev, int protect, u16 data_len, u8 *data_buf)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);

	if (protect)
		return cyttsp5_hid_output_bl_program_and_verify(cd, data_len,
				data_buf);

	return cyttsp5_hid_output_bl_program_and_verify_(cd, data_len,
			data_buf);
}

static int cyttsp5_hid_output_bl_verify_app_integrity_(
		struct cyttsp5_core_data *cd, u8 *result)
{
	int rc;
	struct cyttsp5_hid_output hid_output = {
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

static int cyttsp5_hid_output_bl_verify_app_integrity(
		struct cyttsp5_core_data *cd, u8 *result)
{
	int rc;

	rc = request_exclusive(cd, cd->dev, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		pt_debug(cd->dev, DL_ERROR,
				"%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->dev);
		return rc;
	}

	rc = cyttsp5_hid_output_bl_verify_app_integrity_(cd, result);

	if (release_exclusive(cd, cd->dev) < 0)
		pt_debug(cd->dev, DL_ERROR,
				"%s: fail to release exclusive\n", __func__);

	return rc;
}

static int _cyttsp5_request_hid_output_bl_verify_app_integrity(
		struct device *dev, int protect, u8 *result)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);

	if (protect)
		return cyttsp5_hid_output_bl_verify_app_integrity(cd, result);

	return cyttsp5_hid_output_bl_verify_app_integrity_(cd, result);
}

static int cyttsp5_hid_output_bl_launch_app_(struct cyttsp5_core_data *cd)
{
	struct cyttsp5_hid_output hid_output = {
		HID_OUTPUT_BL_COMMAND(HID_OUTPUT_BL_LAUNCH_APP),
		.reset_expected = 1,
	};

	return pt_hid_send_output_and_wait_(cd, &hid_output);
}

static int cyttsp5_hid_output_bl_launch_app(struct cyttsp5_core_data *cd)
{
	int rc;

	rc = request_exclusive(cd, cd->dev, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		pt_debug(cd->dev, DL_ERROR,
				"%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->dev);
		return rc;
	}

	rc = cyttsp5_hid_output_bl_launch_app_(cd);

	if (release_exclusive(cd, cd->dev) < 0)
		pt_debug(cd->dev, DL_ERROR,
				"%s: fail to release exclusive\n", __func__);

	return rc;
}

static int _cyttsp5_request_hid_output_launch_app(struct device *dev,
		int protect)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);

	if (protect)
		return cyttsp5_hid_output_bl_launch_app(cd);

	return cyttsp5_hid_output_bl_launch_app_(cd);
}

static int cyttsp5_hid_output_bl_get_panel_id_(
		struct cyttsp5_core_data *cd, u8 *panel_id)
{
	int rc;
	struct cyttsp5_hid_output hid_output = {
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
		pt_debug(cd->dev, DL_ERROR, "%s: Error on Get Panel ID command\n",
			__func__);
		return rc;
	}

	*panel_id = cd->response_buf[8];
	return 0;
}

static int cyttsp5_hid_output_bl_get_panel_id(
		struct cyttsp5_core_data *cd, u8 *panel_id)
{
	int rc;

	rc = request_exclusive(cd, cd->dev, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		pt_debug(cd->dev, DL_ERROR,
				"%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->dev);
		return rc;
	}

	rc = cyttsp5_hid_output_bl_get_panel_id_(cd, panel_id);

	if (release_exclusive(cd, cd->dev) < 0)
		pt_debug(cd->dev, DL_ERROR,
				"%s: fail to release exclusive\n", __func__);

	return rc;
}

static int _cyttsp5_request_hid_output_bl_get_panel_id(
		struct device *dev, int protect, u8 *panel_id)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);

	if (protect)
		return cyttsp5_hid_output_bl_get_panel_id(cd, panel_id);

	return cyttsp5_hid_output_bl_get_panel_id_(cd, panel_id);
}

/*******************************************************************************
 * FUNCTION: _pt_request_hid_output_pip2_send_cmd
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
 *	*pip2_cmd        - pointer to PIP2 command to send
 *	 id              - ID of PIP command
 *	*data            - pointer to PIP data payload
 *	 report_body_len - report length
 *	*read_buf        - pointer to response buffer
 *	*actual_read_len - pointer to response buffer length
 ******************************************************************************/
static int _pt_request_hid_output_pip2_send_cmd(struct device *dev,
	struct pip2_cmd_structure *pip2_cmd, u8 id, u8 *data,
	u8 report_body_len, u8 *read_buf, u16 *actual_read_len)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);
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

	/* PATCH - Handle PIP2.0 (TC3300) in a special way */
	if (cd->use_tc3300_pip20_len_field) {
		/*
		 * For PIP2.0 the length in the report DOES NOT include the
		 * length field:
		 * ADD 4: 1 (SEQ) + 1 (REPORT ID) + 2 (CRC)
		 *
		 * The overall write length must include the length field and
		 * the register:
		 * ADD 4: 2 (Register) + 2 (LEN)
		 */
		cd->pip_2p0_prot_active = 1;
		extra_bytes = 4;
		write_len = 4;
	} else {
		/*
		 * For PIP2.1+ the length in the report DOES include the
		 * length field:
		 * ADD 6: 2 (LEN) + 1 (SEQ) + 1 (REPORT ID) + 2 (CRC)
		 *
		 * The overall write length must include only the register:
		 * ADD 2: 2 (Register)
		 */
		extra_bytes = 6;
		write_len = 2;
	}
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

	rc = cyttsp5_hid_output_user_cmd(cd, read_len, read_buf, write_len,
			write_buf, actual_read_len);
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
	cd->pip2_prot_active    = 0;
	cd->pip_2p0_prot_active = 0;
	return rc;
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
 *	*pip_version_major - pointer to store PIP major version
 *	*pip_version_minor - pointer to store PIP minor version
 ******************************************************************************/
int _pt_request_active_pip_protocol(struct device *dev, u8 *pip_version_major,
	u8 *pip_version_minor)
{
	u16 actual_read_len;
	u8 read_buf[256];
	struct pip2_cmd_structure pip2_cmd;
	int rc;

	rc = _pt_request_hid_output_pip2_send_cmd(dev, &pip2_cmd,
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
	} else
		pt_debug(dev, DL_ERROR,
			"%s: PIP communication FAILED, no response. rc = %d\n",
			__func__, rc);
	return rc;
}
EXPORT_SYMBOL_GPL(_pt_request_active_pip_protocol);

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
static int pt_get_hid_descriptor_(struct cyttsp5_core_data *cd,
		struct cyttsp5_hid_desc *desc)
{
	struct device *dev = cd->dev;
	int rc;
	int t;
	u8 cmd[2];

	/*
	 * During startup the HID descriptor is required for all future
	 * processing. If IRQ is already asserted due to an early touch report
	 * the report must be cleared before sending command.
	 */
	if (cd->cpdata->irq_stat &&
	    cd->cpdata->irq_stat(cd->cpdata, cd->dev) ==
	    CY_IRQ_ASSERTED_VALUE) {
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

	pt_debug(cd->dev, DL_INFO, ">>> %s: Write Buffer [%d]",
		__func__, sizeof(cmd));
	pt_pr_buf(cd->dev, DL_DEBUG, cmd, sizeof(cmd), ">>> Get HID Desc");
	rc = cyttsp5_adap_write_read_specific(cd, 2, cmd, NULL);
	if (rc) {
		pt_debug(dev, DL_ERROR,
			"%s: failed to get HID descriptor, rc=%d\n",
			__func__, rc);
		goto error;
	}
	t = wait_event_timeout(cd->wait_q, (cd->hid_cmd_state == 0),
			msecs_to_jiffies(CY_HID_GET_HID_DESCRIPTOR_TIMEOUT));

	if (IS_TMO(t)) {
#ifdef TTDL_DIAGNOSTICS
		cd->i2c_transmission_error_count++;
#endif
		pt_debug(cd->dev, DL_ERROR,
			"%s: HID get descriptor timed out\n", __func__);
		cd->hw_detected = false;
		rc = -ETIME;
		goto error;
	} else
		cd->hw_detected = true;

	/* Load the HID descriptor including all registers */
	memcpy((u8 *)desc, cd->response_buf, sizeof(struct cyttsp5_hid_desc));

	/* Check HID descriptor length and version */
	pt_debug(dev, DL_INFO, "%s: HID len:%X HID ver:%X\n", __func__,
		le16_to_cpu(desc->hid_desc_len),
		le16_to_cpu(desc->bcd_version));

	if (le16_to_cpu(desc->hid_desc_len) != sizeof(*desc) ||
		le16_to_cpu(desc->bcd_version) != CY_HID_VERSION) {
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

static int cyttsp5_get_hid_descriptor(struct cyttsp5_core_data *cd,
		struct cyttsp5_hid_desc *desc)
{
	int rc;

	rc = request_exclusive(cd, cd->dev, CY_REQUEST_EXCLUSIVE_TIMEOUT);
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
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);

	if (protect)
		return cyttsp5_get_hid_descriptor(cd, &cd->hid_desc);

	return pt_get_hid_descriptor_(cd, &cd->hid_desc);
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
static void pt_start_wd_timer(struct cyttsp5_core_data *cd)
{
	if (!cd->watchdog_interval)
		return;

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
static void pt_stop_wd_timer(struct cyttsp5_core_data *cd)
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
 *	*cd - pointer to core data struct
 ******************************************************************************/
static int cyttsp5_hw_soft_reset(struct cyttsp5_core_data *cd)
{
	int rc;

	pt_stop_wd_timer(cd);
	if (cd->hid_desc.hid_desc_len == 0) {
		rc = pt_get_hid_descriptor_(cd, &cd->hid_desc);
		if (rc < 0)
			return rc;
	}

	rc = cyttsp5_hid_cmd_reset_(cd);
	if (rc < 0) {
		pt_debug(cd->dev, DL_ERROR,
			"%s: FAILED to execute SOFT reset\n", __func__);
		return rc;
	}
	pt_debug(cd->dev, DL_WARN, "%s: execute SOFT reset\n",
		__func__);
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
static int pt_hw_hard_reset(struct cyttsp5_core_data *cd)
{
	pt_stop_wd_timer(cd);
	if (cd->cpdata->xres) {
		pt_debug(cd->dev, DL_WARN,
			"%s: Reset Startup Status bitmask\n", __func__);
		cd->startup_status = STARTUP_STATUS_START;
		cd->cpdata->xres(cd->cpdata, cd->dev);
		pt_debug(cd->dev, DL_WARN, "%s: execute HARD reset\n",
			__func__);
		return 0;
	}
	pt_debug(cd->dev, DL_ERROR,
			"%s: FAILED to execute HARD reset\n", __func__);
	pt_start_wd_timer(cd);
	return -ENODEV;
}

/*******************************************************************************
 * FUNCTION: pt_hw_reset
 *
 * SUMMARY: Safe function to perform a DUT reset. A hard reset is attempted
 *	first and if it fails a soft reset is tried
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *      *cd - pointer to core data structure
 ******************************************************************************/
static int pt_hw_reset(struct cyttsp5_core_data *cd)
{
	int rc;

	mutex_lock(&cd->system_lock);
	rc = pt_hw_hard_reset(cd);
	mutex_unlock(&cd->system_lock);
	if (rc == -ENODEV)
		rc = cyttsp5_hw_soft_reset(cd);
	return rc;
}

static inline int get_hid_item_data(u8 *data, int item_size)
{
	if (item_size == 1)
		return (int)*data;
	else if (item_size == 2)
		return (int)get_unaligned_le16(data);
	else if (item_size == 4)
		return (int)get_unaligned_le32(data);
	return 0;
}

static int parse_report_descriptor(struct cyttsp5_core_data *cd,
		u8 *report_desc, size_t len)
{
	struct cyttsp5_hid_report *report;
	struct cyttsp5_hid_field *field;
	u8 *buf = report_desc;
	u8 *end = buf + len;
	int rc = 0;
	int offset = 0;
	int i;
	u8 report_type;
	u32 up_usage;
	/* Global items */
	u8 report_id = 0;
	u16 usage_page = 0;
	int report_count = 0;
	int report_size = 0;
	int logical_min = 0;
	int logical_max = 0;
	/* Local items */
	u16 usage = 0;
	/* Main items - Collection stack */
	u32 collection_usages[CY_HID_MAX_NESTED_COLLECTIONS] = {0};
	u8 collection_types[CY_HID_MAX_NESTED_COLLECTIONS] = {0};
	/* First collection for header, second for report */
	int logical_collection_count = 0;
	int collection_nest = 0;

	pt_debug(cd->dev, DL_INFO, "%s: Report descriptor length: %u\n",
		__func__, (u32)len);

	mutex_lock(&cd->hid_report_lock);
	cyttsp5_free_hid_reports_(cd);

	while (buf < end) {
		int item_type;
		int item_size;
		int item_tag;
		u8 *data;

		/* Get Item */
		item_size = HID_GET_ITEM_SIZE(buf[0]);
		if (item_size == 3)
			item_size = 4;
		item_type = HID_GET_ITEM_TYPE(buf[0]);
		item_tag = HID_GET_ITEM_TAG(buf[0]);

		data = ++buf;
		buf += item_size;

		/* Process current item */
		switch (item_type) {
		case HID_ITEM_TYPE_GLOBAL:
			switch (item_tag) {
			case HID_GLOBAL_ITEM_TAG_REPORT_ID:
				if (item_size != 1) {
					rc = -EINVAL;
					goto exit;
				}
				report_id = get_hid_item_data(data, item_size);
				offset = 0;
				logical_collection_count = 0;
				break;
			case HID_GLOBAL_ITEM_TAG_USAGE_PAGE:
				if (item_size == 0 || item_size == 4) {
					rc = -EINVAL;
					goto exit;
				}
				usage_page = (u16)get_hid_item_data(data,
						item_size);
				break;
			case HID_GLOBAL_ITEM_TAG_LOGICAL_MINIMUM:
				if (item_size == 0) {
					rc = -EINVAL;
					goto exit;
				}
				logical_min = get_hid_item_data(data,
						item_size);
				break;
			case HID_GLOBAL_ITEM_TAG_LOGICAL_MAXIMUM:
				if (item_size == 0) {
					rc = -EINVAL;
					goto exit;
				}
				logical_max = get_hid_item_data(data,
						item_size);
				break;
			case HID_GLOBAL_ITEM_TAG_REPORT_COUNT:
				if (item_size == 0) {
					rc = -EINVAL;
					goto exit;
				}
				report_count = get_hid_item_data(data,
						item_size);
				break;
			case HID_GLOBAL_ITEM_TAG_REPORT_SIZE:
				if (item_size == 0) {
					rc = -EINVAL;
					goto exit;
				}
				report_size = get_hid_item_data(data,
						item_size);
				break;
			default:
				pt_debug(cd->dev, DL_INFO,
					"%s: Unrecognized Global tag %d\n",
					__func__, item_tag);
			}
			break;
		case HID_ITEM_TYPE_LOCAL:
			switch (item_tag) {
			case HID_LOCAL_ITEM_TAG_USAGE:
				if (item_size == 0 || item_size == 4) {
					rc = -EINVAL;
					goto exit;
				}
				usage = (u16)get_hid_item_data(data,
						item_size);
				break;
			case HID_LOCAL_ITEM_TAG_USAGE_MINIMUM:
				if (item_size == 0) {
					rc = -EINVAL;
					goto exit;
				}
				/* usage_min = */
				get_hid_item_data(data, item_size);
				break;
			case HID_LOCAL_ITEM_TAG_USAGE_MAXIMUM:
				if (item_size == 0) {
					rc = -EINVAL;
					goto exit;
				}
				/* usage_max = */
				get_hid_item_data(data, item_size);
				break;
			default:
				pt_debug(cd->dev, DL_INFO,
					"%s: Unrecognized Local tag %d\n",
					__func__, item_tag);
			}
			break;
		case HID_ITEM_TYPE_MAIN:
			switch (item_tag) {
			case HID_MAIN_ITEM_TAG_BEGIN_COLLECTION:
				if (item_size != 1) {
					rc = -EINVAL;
					goto exit;
				}
				if (CY_HID_MAX_NESTED_COLLECTIONS ==
						collection_nest) {
					rc = -EINVAL;
					goto exit;
				}

				up_usage = usage_page << 16 | usage;

				/* Update collection stack */
				collection_usages[collection_nest] = up_usage;
				collection_types[collection_nest] =
					get_hid_item_data(data, item_size);

				if (collection_types[collection_nest] ==
						HID_COLLECTION_LOGICAL)
					logical_collection_count++;

				collection_nest++;
				break;
			case HID_MAIN_ITEM_TAG_END_COLLECTION:
				if (item_size != 0) {
					rc = -EINVAL;
					goto exit;
				}
				if (--collection_nest < 0) {
					rc = -EINVAL;
					goto exit;
				}
				break;
			case HID_MAIN_ITEM_TAG_INPUT:
				report_type = HID_INPUT_REPORT;
				goto continue_main_item;
			case HID_MAIN_ITEM_TAG_OUTPUT:
				report_type = HID_OUTPUT_REPORT;
				goto continue_main_item;
			case HID_MAIN_ITEM_TAG_FEATURE:
				report_type = HID_FEATURE_REPORT;
continue_main_item:
				if (item_size != 1) {
					rc = -EINVAL;
					goto exit;
				}

				up_usage = usage_page << 16 | usage;

				/* Get or create report */
				report = cyttsp5_get_hid_report_(cd,
						report_type, report_id, true);
				if (!report) {
					rc = -ENOMEM;
					goto exit;
				}
				if (!report->usage_page && collection_nest > 0)
					report->usage_page =
						collection_usages
							[collection_nest - 1];

				/* Create field */
				field = cyttsp5_create_hid_field_(report);
				if (!field) {
					rc = -ENOMEM;
					goto exit;
				}

				field->report_count = report_count;
				field->report_size = report_size;
				field->size = report_count * report_size;
				field->offset = offset;
				field->data_type =
					get_hid_item_data(data, item_size);
				field->logical_min = logical_min;
				field->logical_max = logical_max;
				field->usage_page = up_usage;

				for (i = 0; i < collection_nest; i++) {
					field->collection_usage_pages
							[collection_types[i]] =
						collection_usages[i];
				}

				/* Update report's header or record size */
				if (logical_collection_count == 1) {
					report->header_size += field->size;
				} else if (logical_collection_count == 2) {
					field->record_field = true;
					field->offset -= report->header_size;
					/* Set record field index */
					if (report->record_field_index == 0)
						report->record_field_index =
							report->num_fields - 1;
					report->record_size += field->size;
				}

				report->size += field->size;

				offset += field->size;
				break;
			default:
				pt_debug(cd->dev, DL_INFO, "%s: Unrecognized Main tag %d\n",
					__func__, item_tag);
			}

			/* Reset all local items */
			usage = 0;
			break;
		}
	}

	if (buf != end) {
		pt_debug(cd->dev, DL_ERROR, "%s: Report descriptor length invalid\n",
			__func__);
		rc = -EINVAL;
		goto exit;
	}

	if (collection_nest) {
		pt_debug(cd->dev, DL_ERROR, "%s: Unbalanced collection items (%d)\n",
			__func__, collection_nest);
		rc = -EINVAL;
		goto exit;
	}

exit:
	if (rc)
		cyttsp5_free_hid_reports_(cd);
	mutex_unlock(&cd->hid_report_lock);
	return rc;
}

static struct cyttsp5_hid_field *find_report_desc_field(
		struct cyttsp5_core_data *cd, u32 usage_page,
		u32 collection_usage_page)
{
	struct cyttsp5_hid_report *report = NULL;
	struct cyttsp5_hid_field *field = NULL;
	int i;
	int j;
	u32 field_cup;
	u32 field_up;

	for (i = 0; i < cd->num_hid_reports; i++) {
		report = cd->hid_reports[i];
		for (j = 0; j < report->num_fields; j++) {
			field = report->fields[j];
			field_cup = field->collection_usage_pages
				[HID_COLLECTION_LOGICAL];
			field_up = field->usage_page;
			if (field_cup == collection_usage_page
					&& field_up == usage_page) {
				return field;
			}
		}
	}

	return NULL;
}

static int fill_tch_abs(struct cyttsp5_tch_abs_params *tch_abs,
		struct cyttsp5_hid_field *field)
{
	tch_abs->ofs = field->offset / 8;
	tch_abs->size = field->report_size / 8;
	if (field->report_size % 8)
		tch_abs->size += 1;
	tch_abs->min = 0;
	tch_abs->max = 1 << field->report_size;
	tch_abs->bofs = field->offset - (tch_abs->ofs << 3);

	return 0;
}

static struct cyttsp5_hid_report *find_report_desc(struct cyttsp5_core_data *cd,
		u32 usage_page)
{
	struct cyttsp5_hid_report *report = NULL;
	int i;

	for (i = 0; i < cd->num_hid_reports; i++) {
		if (cd->hid_reports[i]->usage_page == usage_page) {
			report = cd->hid_reports[i];
			break;
		}
	}

	return report;
}

static int setup_report_descriptor(struct cyttsp5_core_data *cd)
{
	struct cyttsp5_sysinfo *si = &cd->sysinfo;
	struct cyttsp5_hid_report *report;
	struct cyttsp5_hid_field *field;
	int i;
	u32 tch_collection_usage_page = HID_CY_TCH_COL_USAGE_PG;
	u32 btn_collection_usage_page = HID_CY_BTN_COL_USAGE_PG;

	for (i = CY_TCH_X; i < CY_TCH_NUM_ABS; i++) {
		field = find_report_desc_field(cd,
				cyttsp5_tch_abs_field_map[i],
				tch_collection_usage_page);
		if (field) {
			pt_debug(cd->dev, DL_DEBUG,
				" Field %p: rep_cnt:%d rep_sz:%d off:%d data:%02X min:%d max:%d usage_page:%08X\n",
				field, field->report_count, field->report_size,
				field->offset, field->data_type,
				field->logical_min, field->logical_max,
				field->usage_page);
			fill_tch_abs(&si->tch_abs[i], field);
			si->tch_abs[i].report = 1;
			pt_debug(cd->dev, DL_DEBUG,
				"%s: ofs:%u size:%u min:%u max:%u bofs:%u report:%d",
				cyttsp5_tch_abs_string[i],
				(u32)si->tch_abs[i].ofs,
				(u32)si->tch_abs[i].size,
				(u32)si->tch_abs[i].min,
				(u32)si->tch_abs[i].max,
				(u32)si->tch_abs[i].bofs,
				si->tch_abs[i].report);

		} else {
			si->tch_abs[i].report = 0;
		}
	}
	for (i = CY_TCH_TIME; i < CY_TCH_NUM_HDR; i++) {
		field = find_report_desc_field(cd,
				cyttsp5_tch_hdr_field_map[i],
				tch_collection_usage_page);
		if (field) {
			pt_debug(cd->dev, DL_DEBUG,
				" Field %p: rep_cnt:%d rep_sz:%d off:%d data:%02X min:%d max:%d usage_page:%08X\n",
				field, field->report_count, field->report_size,
				field->offset, field->data_type,
				field->logical_min, field->logical_max,
				field->usage_page);
			fill_tch_abs(&si->tch_hdr[i], field);
			si->tch_hdr[i].report = 1;
			pt_debug(cd->dev, DL_DEBUG,
				"%s: ofs:%u size:%u min:%u max:%u bofs:%u report:%d",
				cyttsp5_tch_hdr_string[i],
				(u32)si->tch_hdr[i].ofs,
				(u32)si->tch_hdr[i].size,
				(u32)si->tch_hdr[i].min,
				(u32)si->tch_hdr[i].max,
				(u32)si->tch_hdr[i].bofs,
				si->tch_hdr[i].report);

		} else {
			si->tch_hdr[i].report = 0;
		}
	}

	report = find_report_desc(cd, tch_collection_usage_page);
	if (report) {
		si->desc.tch_report_id = report->id;
		si->desc.tch_record_size = report->record_size / 8;
		si->desc.tch_header_size = (report->header_size / 8) + 3;
	} else {
		si->desc.tch_report_id = HID_TOUCH_REPORT_ID;
		si->desc.tch_record_size = TOUCH_REPORT_SIZE;
		si->desc.tch_header_size = TOUCH_INPUT_HEADER_SIZE;
	}

	report = find_report_desc(cd, btn_collection_usage_page);
	if (report)
		si->desc.btn_report_id = report->id;
	else
		si->desc.btn_report_id = HID_BTN_REPORT_ID;

	for (i = 0; i < cd->num_hid_reports; i++) {
		struct cyttsp5_hid_report *report = cd->hid_reports[i];

		switch (report->id) {
		case HID_WAKEUP_REPORT_ID:
			cd->features.easywake = 1;
			break;
		case HID_NOISE_METRIC_REPORT_ID:
			cd->features.noise_metric = 1;
			break;
		case HID_TRACKING_HEATMAP_REPOR_ID:
			cd->features.tracking_heatmap = 1;
			break;
		case HID_SENSOR_DATA_REPORT_ID:
			cd->features.sensor_data = 1;
			break;
		default:
			break;
		}
	}

	pt_debug(cd->dev, DL_INFO, "Features: easywake:%d noise_metric:%d tracking_heatmap:%d sensor_data:%d\n",
		cd->features.easywake, cd->features.noise_metric,
		cd->features.tracking_heatmap,
		cd->features.sensor_data);

	return 0;
}

/*******************************************************************************
 * FUNCTION: cyttsp5_get_report_descriptor_
 *
 * SUMMARY: Send the get report descriptor command to the DUT and load the
 *	response into the Report descriptor structure
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*cd - pointer to core data
 ******************************************************************************/
static int cyttsp5_get_report_descriptor_(struct cyttsp5_core_data *cd)
{
	struct device *dev = cd->dev;
	u8 cmd[2];
	int rc;
	int t;

	/* Read report descriptor length and version */
	mutex_lock(&cd->system_lock);
	cd->hid_cmd_state = 1;
	mutex_unlock(&cd->system_lock);

	/* Set report descriptor register */
	memcpy(cmd, &cd->hid_desc.report_desc_register,
		sizeof(cd->hid_desc.report_desc_register));

	pt_debug(cd->dev, DL_INFO, ">>> %s: Write Buffer Size[%d] Cmd[%d]",
		__func__, sizeof(cmd), cd->hid_desc.report_desc_register);
	pt_pr_buf(cd->dev, DL_DEBUG, cmd, sizeof(cmd), ">>> Get Report Desc");
	rc = cyttsp5_adap_write_read_specific(cd, 2, cmd, NULL);
	if (rc) {
		pt_debug(dev, DL_ERROR,
			"%s: failed to get Report descriptor, rc=%d\n",
			__func__, rc);
		goto error;
	}

	t = wait_event_timeout(cd->wait_q, (cd->hid_cmd_state == 0),
		msecs_to_jiffies(CY_HID_GET_REPORT_DESCRIPTOR_TIMEOUT));
	if (IS_TMO(t)) {
#ifdef TTDL_DIAGNOSTICS
		cd->i2c_transmission_error_count++;
#endif
		pt_debug(cd->dev, DL_ERROR,
			"%s: Get Report descriptor timed out\n", __func__);
		rc = -ETIME;
		goto error;
	}

	pt_pr_buf(cd->dev, DL_INFO, cd->response_buf,
		cd->hid_core.hid_report_desc_len, "Report Desc");

	rc = parse_report_descriptor(cd, cd->response_buf + 3,
		get_unaligned_le16(&cd->response_buf[0]) - 3);
	if (rc) {
		pt_debug(cd->dev, DL_ERROR,
			"%s: Error parsing report descriptor r=%d\n",
			__func__, rc);
	}

	pt_debug(cd->dev, DL_INFO, "%s: %d reports found in descriptor\n",
		__func__, cd->num_hid_reports);

	for (t = 0; t < cd->num_hid_reports; t++) {
		struct cyttsp5_hid_report *report = cd->hid_reports[t];
		int j;

		pt_debug(cd->dev, DL_INFO,
			"Report %d: type:%d id:%02X size:%d fields:%d rec_fld_index:%d hdr_sz:%d rec_sz:%d usage_page:%08X\n",
			t, report->type, report->id,
			report->size, report->num_fields,
			report->record_field_index, report->header_size,
			report->record_size, report->usage_page);

		for (j = 0; j < report->num_fields; j++) {
			struct cyttsp5_hid_field *field = report->fields[j];

			pt_debug(cd->dev, DL_DEBUG,
				" Field %d: rep_cnt:%d rep_sz:%d off:%d data:%02X min:%d max:%d usage_page:%08X\n",
				j, field->report_count, field->report_size,
				field->offset, field->data_type,
				field->logical_min, field->logical_max,
				field->usage_page);

			pt_debug(cd->dev, DL_DEBUG,
				" Collections Phys:%08X App:%08X Log:%08X\n",
				field->collection_usage_pages
					[HID_COLLECTION_PHYSICAL],
				field->collection_usage_pages
					[HID_COLLECTION_APPLICATION],
				field->collection_usage_pages
					[HID_COLLECTION_LOGICAL]);
		}
	}

	rc = setup_report_descriptor(cd);

	/* Free it for now */
	cyttsp5_free_hid_reports_(cd);

	pt_debug(cd->dev, DL_INFO, "%s: %d reports found in descriptor\n",
		__func__, cd->num_hid_reports);

	goto exit;

error:
	mutex_lock(&cd->system_lock);
	cd->hid_cmd_state = 0;
	mutex_unlock(&cd->system_lock);
exit:
	return rc;
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
static int pt_get_mode(struct cyttsp5_core_data *cd,
		struct cyttsp5_hid_desc *desc)
{
	if (desc->packet_id == CY_HID_APP_REPORT_ID)
		return CY_MODE_OPERATIONAL;
	else if (desc->packet_id == CY_HID_BL_REPORT_ID)
		return CY_MODE_BOOTLOADER;

	return CY_MODE_UNKNOWN;
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
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);
	int rc;

	if (protect)
		rc = cyttsp5_get_hid_descriptor(cd, &cd->hid_desc);
	else
		rc = pt_get_hid_descriptor_(cd, &cd->hid_desc);

	if (rc)
		*mode = CY_MODE_UNKNOWN;
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
static void pt_queue_startup_(struct cyttsp5_core_data *cd)
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
static void pt_queue_startup(struct cyttsp5_core_data *cd)
{
	mutex_lock(&cd->system_lock);
	pt_queue_startup_(cd);
	mutex_unlock(&cd->system_lock);
}

static void call_atten_cb(struct cyttsp5_core_data *cd,
		enum cyttsp5_atten_type type, int mode)
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

static int start_fw_upgrade(void *data)
{
	struct cyttsp5_core_data *cd = (struct cyttsp5_core_data *)data;

	call_atten_cb(cd, CY_ATTEN_LOADER, 0);

	return 0;
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
 *
 * RETURN: void
 *
 * PARAMETERS:
 *      *work - pointer to a work structure for the watchdog work queue
 ******************************************************************************/
static void pt_watchdog_work(struct work_struct *work)
{
	int rc;
	u8 buf[256];

	struct cyttsp5_core_data *cd = container_of(work,
				struct cyttsp5_core_data, watchdog_work);
	/*
	 * fix CDT207254
	 * if found the current sleep_state is SS_SLEEPING
	 * then no need to request_exclusive, directly return
	 */
	if (cd->sleep_state == SS_SLEEPING)
		return;

#ifdef TTDL_DIAGNOSTICS
	/* count the number of times the watchdog has fired */
	cd->watchdog_count++;
#endif

	/* Check if IRQ stuck asserted */
	if (cd->cpdata->irq_stat &&
	    cd->cpdata->irq_stat(cd->cpdata, cd->dev) ==
		CY_IRQ_ASSERTED_VALUE) {
		pt_debug(cd->dev, DL_WARN,
			"%s: TTDL WD found IRQ asserted, attempt to clear\n",
			__func__);
#ifdef TTDL_DIAGNOSTICS
		cd->watchdog_irq_stuck_count++;
#endif
		if (cd->wd_corrective_action == 1) {
			i2c_master_recv(to_i2c_client(cd->dev), buf, 2);
			i2c_master_recv(to_i2c_client(cd->dev), buf,
				2 + get_unaligned_le16(&buf[0]));
		}
	}

	rc = request_exclusive(cd, cd->dev, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		pt_debug(cd->dev, DL_ERROR,
				"%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->dev);
		goto queue_startup;
	}

	/* Send HID PING command */
	rc = cyttsp5_hid_output_null_(cd);

	if (release_exclusive(cd, cd->dev) < 0)
		pt_debug(cd->dev, DL_ERROR,
				"%s: fail to release exclusive\n", __func__);

queue_startup:
	if (rc) {
#ifdef TTDL_DIAGNOSTICS
		cd->watchdog_failed_access_count++;
#endif
		/* WD failed to communicate with DUT */
		cd->hw_detected = 0;
		pt_debug(cd->dev, DL_ERROR,
			"%s: failed to access device in watchdog timer r=%d\n",
			__func__, rc);

		/* Already tried FW upgrade because of watchdog but failed */
		if (cd->startup_retry_count > CY_WATCHDOG_RETRY_COUNT)
			return;

		if (cd->startup_retry_count++ < CY_WATCHDOG_RETRY_COUNT) {
#ifdef TTDL_DIAGNOSTICS
			cd->hid_reset_count++;
#endif
			/*
			 * Send a HID reset to reset the DUT COMM block and
			 * then queue a ttdl startup to re-enum with the DUT
			 */
			if (cd->wd_corrective_action == 1) {
				cyttsp5_hid_cmd_reset(cd);
				pt_queue_startup(cd);
			}
		} else {
			pt_debug(cd->dev, DL_WARN,
				"%s: WD DUT access failure, Start FW Upgrade\n",
				__func__);
#if defined(PT_TDDI_SUPPORT) || defined(PT_TT_DISCRETE_SUPPORT)
			/* TODO - update to call PIP2 loader */
			kthread_run(start_fw_upgrade, cd, "cyttp5_loader");
#else
			kthread_run(start_fw_upgrade, cd, "cyttp5_loader");
#endif
		}

		return;
	}

	/* WD was able to communicate with DUT, HW must be there */
	cd->hw_detected = 1;
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
	struct cyttsp5_core_data *cd = (struct cyttsp5_core_data *)handle;

	if (!cd)
		return;

	pt_debug(cd->dev, DL_DEBUG, "%s: Watchdog timer triggered\n",
		__func__);

	if (!work_pending(&cd->watchdog_work))
		schedule_work(&cd->watchdog_work);
}

static int cyttsp5_put_device_into_easy_wakeup_(struct cyttsp5_core_data *cd)
{
	int rc;
	u8 status = 0;

	mutex_lock(&cd->system_lock);
	cd->wait_until_wake = 0;
	mutex_unlock(&cd->system_lock);

	rc = cyttsp5_hid_output_enter_easywake_state_(cd,
			cd->easy_wakeup_gesture, &status);
	if (rc || status == 0)
		return -EBUSY;

	return rc;
}

static int cyttsp5_put_device_into_deep_sleep_(struct cyttsp5_core_data *cd)
{
	int rc;

	rc = cyttsp5_hid_cmd_set_power_(cd, HID_POWER_SLEEP);
	if (rc)
		rc = -EBUSY;
	return rc;
}

static int cyttsp5_put_device_into_sleep_(struct cyttsp5_core_data *cd)
{
	int rc;

	if (IS_DEEP_SLEEP_CONFIGURED(cd->easy_wakeup_gesture))
		rc = cyttsp5_put_device_into_deep_sleep_(cd);
	else
		rc = cyttsp5_put_device_into_easy_wakeup_(cd);

	return rc;
}

static int cyttsp5_core_poweroff_device_(struct cyttsp5_core_data *cd)
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

static int cyttsp5_core_sleep_(struct cyttsp5_core_data *cd)
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

	if (cd->cpdata->flags & CY_CORE_FLAG_POWEROFF_ON_SLEEP)
		rc = cyttsp5_core_poweroff_device_(cd);
	else
		rc = cyttsp5_put_device_into_sleep_(cd);

	mutex_lock(&cd->system_lock);
	cd->sleep_state = SS_SLEEP_ON;
	mutex_unlock(&cd->system_lock);

	return rc;
}

static int cyttsp5_core_sleep(struct cyttsp5_core_data *cd)
{
	int rc;

	rc = request_exclusive(cd, cd->dev, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		pt_debug(cd->dev, DL_ERROR, "%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->dev);
		return rc;
	}

	rc = cyttsp5_core_sleep_(cd);

	if (release_exclusive(cd, cd->dev) < 0)
		pt_debug(cd->dev, DL_ERROR,
				"%s: fail to release exclusive\n", __func__);
	else
		pt_debug(cd->dev, DL_DEBUG, "%s: pass release exclusive\n",
			__func__);

	return rc;
}

static int cyttsp5_wakeup_host(struct cyttsp5_core_data *cd)
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
		cyttsp5_core_sleep_(cd);
		goto exit;
	}

	/* attention WAKE */
	call_atten_cb(cd, CY_ATTEN_WAKE, 0);
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
		cyttsp5_core_sleep_(cd);
		goto exit;
	}

	/* Get gesture id and gesture data length */
	cd->gesture_id = cd->input_buf[3];
	report_length = (cd->input_buf[1] << 8) | (cd->input_buf[0]);
	cd->gesture_data_length = report_length - 4;

	pt_debug(cd->dev, DL_INFO, "%s: gesture_id = %d, gesture_data_length = %d\n",
		__func__, cd->gesture_id, cd->gesture_data_length);

	for (i = 0; i < cd->gesture_data_length; i++)
		cd->gesture_data[i] = cd->input_buf[4 + i];

	/* attention WAKE */
	call_atten_cb(cd, CY_ATTEN_WAKE, 0);
exit:
	return rc;
#endif
}

static void cyttsp5_get_touch_axis(struct cyttsp5_core_data *cd,
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
static int move_tracking_heatmap_data(struct cyttsp5_core_data *cd,
	struct cyttsp5_sysinfo *si)
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
static int move_sensor_data(struct cyttsp5_core_data *cd,
	struct cyttsp5_sysinfo *si)
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
static int move_button_data(struct cyttsp5_core_data *cd,
	struct cyttsp5_sysinfo *si)
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
static int move_touch_data(struct cyttsp5_core_data *cd,
	struct cyttsp5_sysinfo *si)
{
	int max_tch = si->sensing_conf_data.max_tch;
	int num_cur_tch;
	int length;
	struct cyttsp5_tch_abs_params *tch = &si->tch_hdr[CY_TCH_NUM];
#ifdef TTHE_TUNER_SUPPORT
	int size = get_unaligned_le16(&cd->input_buf[0]);

	if (size)
		tthe_print(cd, cd->input_buf, size, "OpModeData=");
#endif

	memcpy(si->xy_mode, cd->input_buf, si->desc.tch_header_size);
	pt_pr_buf(cd->dev, DL_INFO, (u8 *)si->xy_mode,
		si->desc.tch_header_size, "xy_mode");

	cyttsp5_get_touch_axis(cd, &num_cur_tch, tch->size,
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
static int parse_touch_input(struct cyttsp5_core_data *cd, int size)
{
	struct cyttsp5_sysinfo *si = &cd->sysinfo;
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
	call_atten_cb(cd, CY_ATTEN_IRQ, cd->mode);

	return 0;
}

/*******************************************************************************
 * FUNCTION: parse_command_input
 *
 * SUMMARY: Move the response data from the intput buffer to the response buffer
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*cd   - pointer to core data
 *	 size - size of response data
 ******************************************************************************/
static int parse_command_input(struct cyttsp5_core_data *cd, int size)
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
 * FUNCTION: cyttsp5_parse_input
 *
 * SUMMARY: Parse the input data read from DUT due to IRQ having been triggered.
 *	Handle data based on if its a response to a command or asynchronous
 *	touch data. Also look for special packets based on unique sizes
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*cd - pointer to core data
 ******************************************************************************/
static int cyttsp5_parse_input(struct cyttsp5_core_data *cd)
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

	/* PIP2.0 length field does not include itself so add 2 to see CRC */
	if (cd->pip_2p0_prot_active)
		print_size = size + 2;
	else
		print_size = size;
	pt_debug(cd->dev, DL_INFO, "<<< %s: IRQ Triggered, read len [%d]\n",
		__func__, print_size);
	pt_pr_buf(cd->dev, DL_DEBUG, cd->input_buf, print_size, "<<< Read buf");

	/*
	 * Check for PIP1.x reset sentinel where size = 0, or PIP2.1+ reset
	 * sentinel which is a STATUS response with TAG and SEQ set to 0
	 *
	 * NOTE: a user initiated STATUS cmd via the command sysfs node could
	 * be interpreted as a reset condition when TAG is not set and BL sends
	 * response with SEQ = 0.
	 */
	if (size == 0 ||
	   (size == 11 &&
	    (cd->input_buf[PIP2_RESPONSE_SEQ_OFFSET] & 0x0F) == 0 &&
	    (cd->input_buf[PIP2_RESPONSE_ID_OFFSET] & 0x7F) ==
	     PIP2_CMD_ID_STATUS)) {
		touch_report = false;

		/* Reset startup status after a DUT reset */
		if (size == 0) {
			cd->startup_status |= STARTUP_STATUS_FW_RESET_SENTINEL;
			pt_debug(cd->dev, DL_WARN,
				"%s: FW Reset sentinel recieved\n", __func__);
		} else {
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
		if (cd->hid_cmd_state == HID_OUTPUT_START_BOOTLOADER + 1
				|| cd->hid_cmd_state ==
					HID_OUTPUT_BL_LAUNCH_APP + 1
				|| cd->hid_cmd_state ==
					HID_OUTPUT_USER_CMD + 1)
			cd->hid_cmd_state = 0;
		wake_up(&cd->wait_q);
		mutex_unlock(&cd->system_lock);
		return 0;
	} else if (size == 2 || size >= CY_PIP_1P7_EMPTY_BUF) {
		/*
		 * Special lengths that indicate the DUT has no more data
		 * Before PIP 1.7 - empty buffer is 0x0002;
		 * PIP 1.7+       - empty buffer is 0xFFXX
		 *
		 * This debug message below is used by PBATS to calculate the
		 * time from the last lift off IRQ to when FW exits LFT mode.
		 */
		touch_report = false;
		pt_debug(cd->dev, DL_WARN,
			"%s: DUT - Empty buffer detected\n", __func__);
		return 0;
	}

	/* Decode a PIP2.x header differently */
	if (cd->pip2_prot_active) {
		pt_debug(cd->dev, DL_DEBUG,
			"%s: Decode PIP2.x Response\n", __func__);
		/* Verify the SEQ matches from cmd to resp */
		if (cd->pip2_cmd_tag_seq != (cd->input_buf[2] & 0x0F))
			pt_debug(cd->dev, DL_ERROR,
				"%s: !!! PIP2 SEQ MISSMATCH: cmd = 0x%02X rsp = 0x%02X\n",
				__func__, cd->pip2_cmd_tag_seq,
				(cd->input_buf[2] & 0x0F));

		/* PIP2 does not have a report id */
		report_id = 0x00;
		cmd_id = cd->input_buf[PIP2_HID_INPUT_RESP_CMD_OFFSET];

		/* PIP2.0 length field does not include itself so add 2 */
		if (cd->pip_2p0_prot_active)
			size = size + 2;

		/* Verify the Resp CRC is valid */
		calc_crc = crc_ccitt_calculate(cd->input_buf, size - 2);
		resp_crc  = cd->input_buf[size - 2] << 8;
		resp_crc |= cd->input_buf[size - 1];
		if (resp_crc != calc_crc) {
			pt_debug(cd->dev, DL_ERROR,
				"%s: ### PIP2 CRC ERROR: resp = 0x%04X calc = 0x%04X\n",
				__func__, resp_crc, calc_crc);
			cd->i2c_crc_error_count++;
			/* TODO - Add retry read */
		}

		/* PATCH - Handle PIP2.0 messages in a special way */
		if (cd->pip_2p0_prot_active) {
			memcpy(cd->response_buf, cd->input_buf, size);
			mutex_lock(&cd->system_lock);
			cd->hid_cmd_state = 0;
			mutex_unlock(&cd->system_lock);
			wake_up(&cd->wait_q);
			/* We are done here as PIP2.0 is only for BL cmds */
			goto exit;
		}
	} else {
		report_id = cd->input_buf[HID_OUTPUT_RESPONSE_REPORT_OFFSET];
		cmd_id = cd->input_buf[HID_OUTPUT_RESPONSE_CMD_OFFSET];
	}

	pt_debug(cd->dev, DL_INFO, "%s: report_id: 0x%02X, cmd_code: 0x%02X\n",
		__func__, report_id, (cmd_id & 0x7F));

	/* Check wake-up report */
	if (report_id == HID_WAKEUP_REPORT_ID) {
		cyttsp5_wakeup_host(cd);
		return 0;
	}

	/* update watchdog expire time */
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
exit:
	return 0;
}

static int cyttsp5_read_input(struct cyttsp5_core_data *cd)
{
	struct device *dev = cd->dev;
	int rc = 0;
	int t;
	int retry = PT_I2C_READ_INPUT_RETRY_COUNT;

	/* added as workaround to CDT170960: easywake failure */
	/* Interrupt for easywake, wait for bus controller to wake */
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
#endif
				pt_queue_startup(cd);
			}
			goto read;
		}
	}
	mutex_unlock(&cd->system_lock);

read:
	/* Try reading up to 'retry' times */
	while (retry-- != 0) {
		rc = cyttsp5_adap_read_default_nosize(cd, cd->input_buf,
			CY_MAX_INPUT);
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
static  bool pt_check_irq_asserted(struct cyttsp5_core_data *cd)
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
			!= CY_IRQ_ASSERTED_VALUE)
			return false;
	}
#else
	if (cd->cpdata->irq_stat &&
	    cd->cpdata->irq_stat(cd->cpdata, cd->dev)
	    != CY_IRQ_ASSERTED_VALUE)
		return false;
#endif
	return true;
}


static irqreturn_t cyttsp5_irq(int irq, void *handle)
{
	struct cyttsp5_core_data *cd = handle;
	int rc;

	if (!pt_check_irq_asserted(cd))
		return IRQ_HANDLED;

	rc = cyttsp5_read_input(cd);
#ifdef TTDL_DIAGNOSTICS
	cd->irq_count++;

	/* Used to calculate T-Refresh */
	if (cd->t_refresh_active) {
		if (cd->t_refresh_count == 0) {
			cd->t_refresh_time = jiffies;
/*			cd->t_refresh_time = jiffies_to_msecs(jiffies);*/
			cd->t_refresh_count++;
		} else if (cd->t_refresh_count < cd->t_refresh_total) {
			cd->t_refresh_count++;
		} else {
			cd->t_refresh_active = 0;
			cd->t_refresh_time =
			jiffies_to_msecs(jiffies - cd->t_refresh_time);
/*			jiffies_to_msecs(jiffies) - cd->t_refresh_time; */
		}
	}
#endif

	if (!rc)
		cyttsp5_parse_input(cd);

/*	pr_info("%s: !!!! ---- IRQ Handled ---- !!!!\n", __func__);*/
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
	enum cyttsp5_atten_type type, char *id, int (*func)(struct device *),
	int mode)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);
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
	enum cyttsp5_atten_type type, char *id, int (*func)(struct device *),
	int mode)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);
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
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);

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
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);

	return release_exclusive(cd, (void *)dev);
}

static int pt_dut_reset(struct cyttsp5_core_data *cd)
{
	int rc;

	/* reset hardware */
	pt_debug(cd->dev, DL_WARN, "%s: reset hw...\n", __func__);
	rc = pt_hw_reset(cd);
	if (rc < 0)
		pt_debug(cd->dev, DL_ERROR, "%s: %s dev='%s' r=%d\n",
			__func__, "Fail hw reset", dev_name(cd->dev), rc);
	return rc;
}

/*
 * --- PATCH ---
 * This function is temporarily being removed for TC3300 due to a SPI issue
 * where if the chip is reset with the system clock staying at 108MHz the
 * BL can not access the FLASH device over SPI. When support for TC3300 is no
 * longer required this can be removed.
 */
#ifndef PT_TDDI_SUPPORT
static int pt_dut_reset_and_wait(struct cyttsp5_core_data *cd)
{
	int rc;
	int t;

	mutex_lock(&cd->system_lock);
	cd->hid_reset_cmd_state = 1;
	mutex_unlock(&cd->system_lock);

	rc = pt_dut_reset(cd);
	if (rc < 0)
		goto error;

	t = wait_event_timeout(cd->wait_q, (cd->hid_reset_cmd_state == 0),
			msecs_to_jiffies(CY_HID_RESET_TIMEOUT));
	if (IS_TMO(t)) {
#ifdef TTDL_DIAGNOSTICS
		cd->i2c_transmission_error_count++;
#endif
		pt_debug(cd->dev, DL_ERROR, "%s: reset timed out\n",
			__func__);
		rc = -ETIME;
		goto error;
	}

	goto exit;

error:
	mutex_lock(&cd->system_lock);
	cd->hid_reset_cmd_state = 0;
	mutex_unlock(&cd->system_lock);
exit:
	return rc;
}
#endif

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
 ******************************************************************************/
static int _pt_request_reset(struct device *dev)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);
	int rc;

	mutex_lock(&cd->system_lock);
	cd->hid_reset_cmd_state = 1;
	mutex_unlock(&cd->system_lock);

	rc = pt_dut_reset(cd);
	if (rc < 0) {
		pt_debug(cd->dev, DL_ERROR, "%s: Error on h/w reset r=%d\n",
			__func__, rc);
		mutex_lock(&cd->system_lock);
		cd->hid_reset_cmd_state = 0;
		mutex_unlock(&cd->system_lock);
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
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);

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
struct cyttsp5_sysinfo *_pt_request_sysinfo(struct device *dev)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);

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
static struct cyttsp5_loader_platform_data *_pt_request_loader_pdata(
		struct device *dev)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);

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
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);

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
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);

	pt_stop_wd_timer(cd);
	return 0;
}

static int cyttsp5_core_wake_device_from_deep_sleep_(
		struct cyttsp5_core_data *cd)
{
	int rc;

	rc = cyttsp5_hid_cmd_set_power_(cd, HID_POWER_ON);
	if (rc)
		rc =  -EAGAIN;

	/* Prevent failure on sequential wake/sleep requests from OS */
	msleep(20);

	return rc;
}

static int cyttsp5_core_wake_device_(struct cyttsp5_core_data *cd)
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

	return cyttsp5_core_wake_device_from_deep_sleep_(cd);
}

static int cyttsp5_restore_parameters_(struct cyttsp5_core_data *cd)
{
	struct param_node *param;
	int rc = 0;

	if (!(cd->cpdata->flags & CY_CORE_FLAG_RESTORE_PARAMETERS))
		goto exit;

	spin_lock(&cd->spinlock);
	list_for_each_entry(param, &cd->param_list, node) {
		spin_unlock(&cd->spinlock);
		pt_debug(cd->dev, DL_INFO, "%s: Parameter id:%d value:%d\n",
			 __func__, param->id, param->value);
		rc = cyttsp5_hid_output_set_param_(cd, param->id,
				param->value, param->size);
		if (rc)
			goto exit;
		spin_lock(&cd->spinlock);
	}
	spin_unlock(&cd->spinlock);
exit:
	return rc;
}

static int _fast_startup(struct cyttsp5_core_data *cd)
{
	int retry = CY_CORE_STARTUP_RETRY_COUNT;
	int rc;

reset:
	if (retry != CY_CORE_STARTUP_RETRY_COUNT)
		pt_debug(cd->dev, DL_WARN, "%s: Retry %d\n",
			__func__, CY_CORE_STARTUP_RETRY_COUNT - retry);

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

	if (cd->mode == CY_MODE_BOOTLOADER) {
		pt_debug(cd->dev, DL_INFO, "%s: Bootloader mode\n",
			__func__);
		rc = cyttsp5_hid_output_bl_launch_app_(cd);
		if (rc < 0) {
			pt_debug(cd->dev, DL_ERROR, "%s: Error on launch app r=%d\n",
				__func__, rc);
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
		if (cd->mode == CY_MODE_BOOTLOADER) {
			if (retry--)
				goto reset;
			goto exit;
		}
	}

	rc = cyttsp5_restore_parameters_(cd);
	if (rc)
		pt_debug(cd->dev, DL_ERROR, "%s: failed to restore parameters rc=%d\n",
			__func__, rc);

exit:
	return rc;
}

static int cyttsp5_core_poweron_device_(struct cyttsp5_core_data *cd)
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

static int cyttsp5_core_wake_(struct cyttsp5_core_data *cd)
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

	if (cd->cpdata->flags & CY_CORE_FLAG_POWEROFF_ON_SLEEP)
		rc = cyttsp5_core_poweron_device_(cd);
	else
		rc = cyttsp5_core_wake_device_(cd);

	mutex_lock(&cd->system_lock);
	cd->sleep_state = SS_SLEEP_OFF;
	mutex_unlock(&cd->system_lock);

	pt_start_wd_timer(cd);
	return rc;
}

static int cyttsp5_core_wake(struct cyttsp5_core_data *cd)
{
	int rc;

	rc = request_exclusive(cd, cd->dev, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		pt_debug(cd->dev, DL_ERROR,
				"%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->dev);
		return rc;
	}

	rc = cyttsp5_core_wake_(cd);

	if (release_exclusive(cd, cd->dev) < 0)
		pt_debug(cd->dev, DL_ERROR, "%s: fail to release exclusive\n",
			__func__);
	else
		pt_debug(cd->dev, DL_DEBUG, "%s: pass release exclusive\n",
			__func__);

	return rc;
}

static int cyttsp5_get_ic_crc_(struct cyttsp5_core_data *cd, u8 ebid)
{
	struct cyttsp5_sysinfo *si = &cd->sysinfo;
	int rc;
	u8 status;
	u16 calculated_crc = 0;
	u16 stored_crc = 0;

	rc = pt_hid_output_suspend_scanning_(cd);
	if (rc)
		goto error;

	rc = cyttsp5_hid_output_verify_config_block_crc_(cd, ebid, &status,
			&calculated_crc, &stored_crc);
	if (rc)
		goto exit;

	if (status) {
		rc = -EINVAL;
		goto exit;
	}

	si->ttconfig.crc = stored_crc;

exit:
	pt_hid_output_resume_scanning_(cd);
error:
	pt_debug(cd->dev, DL_ERROR, "%s: CRC: ebid:%d, crc:0x%04X, RC=%d\n",
		__func__, ebid, si->ttconfig.crc, rc);
	return rc;
}

/*******************************************************************************
 * FUNCTION: pt_check_and_deassert_int
 *
 * SUMMARY: Checks if IRQ is asserted and if so will read the first two bytes
 *	to get the length to read and then read the rest to clear the DUT output
 *	buffer and allow the DUT to de-assert the IRQ line.
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *      *cd - pointer to core data
 ******************************************************************************/
static int pt_check_and_deassert_int(struct cyttsp5_core_data *cd)
{
	u16 size;
	u8 buf[2];
	u8 *p;
	u8 retry = 3;
	int rc;

	do {
		rc = cyttsp5_adap_read_default(cd, buf, 2);
		if (rc < 0)
			return rc;
		size = get_unaligned_le16(&buf[0]);
		dev_vdbg(cd->dev, "%s: First read size = %d\n", __func__, size);

		if (size == 2 || size == 0 || size >= CY_PIP_1P7_EMPTY_BUF)
			return 0;

		p = kzalloc(size, GFP_KERNEL);
		if (!p)
			return -ENOMEM;

		rc = cyttsp5_adap_read_default(cd, p, size);
		pt_debug(cd->dev, DL_WARN,
			"%s: adap_read_default rc=%d\n", __func__, rc);
		kfree(p);
		if (rc < 0)
			return rc;
	} while (retry--);

	return -EINVAL;
}

static int add_sysfs_interfaces(struct device *dev);

/*******************************************************************************
 * FUNCTION: pt_startup_
 *
 * SUMMARY: This function does the full enumeration of the DUT with TTDL.
 *	The core data (cd) startup_status will store, as a bitmask, each
 *	state of the enumeration process. The startup will be attempted
 *	CY_CORE_STARTUP_RETRY_COUNT times before giving up.
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*cd    - pointer the core data structure
 *	 reset - Flag to reset the DUT before attempting to enumerate
 ******************************************************************************/
static int pt_startup_(struct cyttsp5_core_data *cd, bool reset)
{
	int retry = CY_CORE_STARTUP_RETRY_COUNT;
	int rc;
	bool detected = false;


#ifdef TTHE_TUNER_SUPPORT
	tthe_print(cd, NULL, 0, "enter startup");
#endif
	pt_stop_wd_timer(cd);

	/* Decrement before looping back to "reset" label which increments */
	cd->startup_retry_count--;
reset:
	cd->startup_retry_count++;
	pt_debug(cd->dev, DL_WARN, "%s: Retry %d\n", __func__,
		CY_CORE_STARTUP_RETRY_COUNT - retry);

	rc = pt_check_and_deassert_int(cd);

#if !defined(PT_TDDI_SUPPORT) && !defined(PT_TT_DISCRETE_SUPPORT)
	/*
	 * PATCH - Excluding this reset for TDDIs and TT_DISCRETE parts as a
	 * temporary patch due to if the FW image was directly loaded to SRAM
	 * issuing a reset will wipe out what was just loaded.
	 */
	if (reset || retry != CY_CORE_STARTUP_RETRY_COUNT) {
		/* reset hardware */
		rc = pt_dut_reset_and_wait(cd);
		if (rc < 0) {
			pt_debug(cd->dev, DL_ERROR,
				"%s: Error on h/w reset r=%d\n", __func__, rc);
			if (retry--)
				goto reset;
			goto exit;
		}
	}
#endif

	rc = pt_get_hid_descriptor_(cd, &cd->hid_desc);
	if (rc < 0) {
		pt_debug(cd->dev, DL_ERROR,
			"%s: Error on getting HID descriptor r=%d\n",
			__func__, rc);
		if (retry--)
			goto reset;
		goto exit;
	}

	/* HID descriptor retrieved, HW detected */
	cd->startup_status |= STARTUP_STATUS_GET_DESC;
	detected = true;

	cd->mode = pt_get_mode(cd, &cd->hid_desc);

/* Remove for TDDIs which do not support panel ID */
#ifndef PT_TDDI_SUPPORT
	/* Switch to bootloader mode to get Panel ID */
	if (cd->mode == CY_MODE_OPERATIONAL) {
		rc = pt_hid_output_start_bootloader_(cd);
		if (rc < 0) {
			pt_debug(cd->dev, DL_ERROR,
				"%s: Error on start bootloader r=%d\n",
				__func__, rc);
			if (retry--)
				goto reset;
			goto exit;
		}
		pt_debug(cd->dev, DL_INFO, "%s: Bootloader mode\n",
			__func__);
	}

	cyttsp5_hid_output_bl_get_panel_id_(cd, &cd->panel_id);
	pt_debug(cd->dev, DL_INFO, "%s: Panel ID: 0x%02X\n",
		__func__, cd->panel_id);

	rc = cyttsp5_hid_output_bl_launch_app_(cd);
	if (rc < 0) {
		pt_debug(cd->dev, DL_ERROR, "%s: Error on launch app r=%d\n",
			__func__, rc);
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
	cd->startup_status |= STARTUP_STATUS_GET_DESC;

	cd->mode = pt_get_mode(cd, &cd->hid_desc);
	if (cd->mode == CY_MODE_BOOTLOADER) {
		if (retry--)
			goto reset;
		goto exit;
	}
#endif

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
	if (cd->mode == CY_MODE_OPERATIONAL)
		pt_debug(cd->dev, DL_INFO, "%s: Operational mode\n", __func__);
	else if (cd->mode == CY_MODE_BOOTLOADER)
		pt_debug(cd->dev, DL_INFO, "%s: Bootloader mode\n", __func__);
	else if (cd->mode == CY_MODE_UNKNOWN) {
		pt_debug(cd->dev, DL_ERROR, "%s: Unknown mode\n", __func__);
		rc = -ENODEV;
		if (retry--)
			goto reset;
		goto exit;
	}
	cd->startup_status |= STARTUP_STATUS_GET_MODE;

	pt_debug(cd->dev, DL_INFO, "%s: Reading report descriptor\n",
		__func__);
	rc = cyttsp5_get_report_descriptor_(cd);
	if (rc < 0) {
		pt_debug(cd->dev, DL_ERROR,
			"%s: Error on getting report descriptor r=%d\n",
			__func__, rc);
		if (retry--)
			goto reset;
		goto exit;
	}
	cd->startup_status |= STARTUP_STATUS_GET_RPT_DESC;

	if (!cd->features.easywake)
		cd->easy_wakeup_gesture = CY_CORE_EWG_NONE;

	pt_debug(cd->dev, DL_INFO, "%s: Reading sysinfo\n", __func__);
	rc = cyttsp5_hid_output_get_sysinfo_(cd);
	if (rc) {
		pt_debug(cd->dev, DL_ERROR,
			"%s: Error on getting sysinfo r=%d\n", __func__, rc);
		if (retry--)
			goto reset;
		goto exit;
	}
	cd->startup_status |= STARTUP_STATUS_GET_SYS_INFO;

	pt_debug(cd->dev, DL_INFO, "cyttsp5 Prot Version: %d.%d\n",
			cd->sysinfo.cydata.pip_ver_major,
			cd->sysinfo.cydata.pip_ver_minor);

	/* Read config version directly if PIP version < 1.2 */
	if (!IS_PIP_VER_GE(&cd->sysinfo, 1, 2)) {
		rc = cyttsp5_get_config_ver_(cd);
		if (rc) {
			pt_debug(cd->dev, DL_ERROR,
				"%s: failed to read config version rc=%d\n",
				__func__, rc);
		}
	}

	rc = cyttsp5_get_ic_crc_(cd, CY_TCH_PARM_EBID);
	if (rc) {
		pt_debug(cd->dev, DL_ERROR,
			"%s: DUT Config block CRC failure rc=%d\n",
			__func__, rc);
		if (retry--)
			goto reset;
		goto exit;
	} else
		cd->startup_status |= STARTUP_STATUS_GET_CFG_CRC;

	rc = cyttsp5_restore_parameters_(cd);
	if (rc) {
		pt_debug(cd->dev, DL_ERROR,
			"%s: Failed to restore parameters rc=%d\n",
			__func__, rc);
	} else
		cd->startup_status |= STARTUP_STATUS_RESTORE_PARM;

	/* attention startup */
	call_atten_cb(cd, CY_ATTEN_STARTUP, 0);
	cd->startup_status |= STARTUP_STATUS_COMPLETE;

	/* After successful enumeration, set WD to user setting */
	cd->watchdog_interval = CY_WATCHDOG_TIMEOUT;

	/* Reset retry count if we successfully enumerated with DUT */
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
static int pt_startup(struct cyttsp5_core_data *cd, bool reset)
{
	int rc;

	mutex_lock(&cd->system_lock);
	cd->startup_state = STARTUP_RUNNING;
	mutex_unlock(&cd->system_lock);

	rc = request_exclusive(cd, cd->dev, CY_REQUEST_EXCLUSIVE_TIMEOUT);
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
	struct cyttsp5_core_data *cd =  container_of(work,
		struct cyttsp5_core_data, startup_work);
	int rc;

	rc = pt_startup(cd, true);
	if (rc < 0)
		pt_debug(cd->dev, DL_ERROR, "%s: Fail queued startup r=%d\n",
			__func__, rc);
}

/*
 * CONFIG_PM_RUNTIME option is removed in 3.19.0.
 */
#if defined(CONFIG_PM_RUNTIME) || \
		(KERNEL_VERSION(3, 19, 0) <= LINUX_VERSION_CODE)
static int cyttsp5_core_rt_suspend(struct device *dev)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);
	int rc;

	rc = cyttsp5_core_sleep(cd);
	if (rc < 0) {
		pt_debug(dev, DL_ERROR, "%s: Error on sleep\n", __func__);
		return -EAGAIN;
	}
	return 0;
}

static int cyttsp5_core_rt_resume(struct device *dev)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);
	int rc;

	rc = cyttsp5_core_wake(cd);
	if (rc < 0) {
		pt_debug(dev, DL_ERROR, "%s: Error on wake\n", __func__);
		return -EAGAIN;
	}

	return 0;
}
#endif

#if defined(CONFIG_PM_SLEEP)
static int cyttsp5_core_suspend(struct device *dev)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);

	cyttsp5_core_sleep(cd);

	if (IS_DEEP_SLEEP_CONFIGURED(cd->easy_wakeup_gesture))
		return 0;

	/*
	 * This will not prevent resume
	 * Required to prevent interrupts before i2c awake
	 */
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

static int cyttsp5_core_resume(struct device *dev)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);

	if (IS_DEEP_SLEEP_CONFIGURED(cd->easy_wakeup_gesture))
		goto exit;

	/*
	 * I2C bus pm does not call suspend if device runtime suspended
	 * This flag is cover that case
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
	cyttsp5_core_wake(cd);

	return 0;
}
#endif

#if NEED_SUSPEND_NOTIFIER
static int cyttsp5_pm_notifier(struct notifier_block *nb,
		unsigned long action, void *data)
{
	struct cyttsp5_core_data *cd = container_of(nb,
			struct cyttsp5_core_data, pm_notifier);

	if (action == PM_SUSPEND_PREPARE) {
		pt_debug(cd->dev, DL_INFO, "%s: Suspend prepare\n",
			__func__);

		/*
		 * If not runtime PM suspended, either call runtime
		 * PM suspend callback or wait until it finishes
		 */
		if (!pm_runtime_suspended(cd->dev))
			pm_runtime_suspend(cd->dev);

		(void) cyttsp5_core_suspend(cd->dev);
	}

	return NOTIFY_DONE;
}
#endif

const struct dev_pm_ops cyttsp5_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(cyttsp5_core_suspend, cyttsp5_core_resume)
	SET_RUNTIME_PM_OPS(cyttsp5_core_rt_suspend, cyttsp5_core_rt_resume,
			NULL)
};
EXPORT_SYMBOL_GPL(cyttsp5_pm_ops);

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
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);
	struct cyttsp5_cydata *cydata = &cd->sysinfo.cydata;

	return sprintf(buf,
		"%s: 0x%02X\n"
		"%s: 0x%02X\n"
		"%s: 0x%08X\n"
		"%s: 0x%04X\n"
		"%s: 0x%02X\n"
		"%s: 0x%02X\n"
		"%s: 0x%02X\n"
		"%s: 0x%02X\n",
		"Firmware Major Version", cydata->fw_ver_major,
		"Firmware Minor Version", cydata->fw_ver_minor,
		"Revision Control Number", cydata->revctrl,
		"Firmware Configuration Version", cydata->fw_ver_conf,
		"Bootloader Major Version", cydata->bl_ver_major,
		"Bootloader Minor Version", cydata->bl_ver_minor,
		"Protocol Major Version", cydata->pip_ver_major,
		"Protocol Minor Version", cydata->pip_ver_minor);
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
	return snprintf(buf, CY_MAX_PRBUF_SIZE,
		"Driver: %s\nVersion: %s\nDate: %s\n",
		cy_driver_core_name, cy_driver_core_version,
		cy_driver_core_date);
}

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
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);
	int rc;

	/* PATCH - TC3300 has to slow clk before a reset */
	if (cd->use_tc3300_pip20_len_field)
		_pt_request_hid_output_start_bl(dev, 1);

	rc = pt_dut_reset(cd);
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
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);
	int retval;

	if (cd->cpdata->irq_stat) {
		retval = cd->cpdata->irq_stat(cd->cpdata, dev);
		switch (retval) {
		case 0:
			return snprintf(buf, CY_MAX_PRBUF_SIZE,
				"Interrupt line is LOW.\n");
		case 1:
			return snprintf(buf, CY_MAX_PRBUF_SIZE,
				"Interrupt line is HIGH.\n");
		default:
			return snprintf(buf, CY_MAX_PRBUF_SIZE,
				"Function irq_stat() returned %d.\n", retval);
		}
	}

	return snprintf(buf, CY_MAX_PRBUF_SIZE,
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
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);
	ssize_t ret;

	mutex_lock(&cd->system_lock);
	if (cd->irq_enabled)
		ret = snprintf(buf, CY_MAX_PRBUF_SIZE,
			"Driver interrupt is ENABLED\n");
	else
		ret = snprintf(buf, CY_MAX_PRBUF_SIZE,
			"Driver interrupt is DISABLED\n");
	mutex_unlock(&cd->system_lock);

	return ret;
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
 *      *dev         - pointer to device structure
 *      *mode        - pointer to store the retrieved mode
 *	 resume_scan - Resume scanning after function completes
 ******************************************************************************/
static int _pt_request_pip2_get_mode(struct device *dev, u8 *mode,
	bool resume_scan)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);
	struct pip2_cmd_structure pip2_cmd;
	u16 actual_read_len;
	u8 read_buf[12];
	u8 status, boot;
	int rc;

	/* Stop scanning to avoid unwanted touch reports */
	rc = pt_hid_output_suspend_scanning_(cd);
	if (rc)
		pt_debug(dev, DL_ERROR, "%s Suspend Scan Failed\n",
		__func__);

	/* Get PIP2 Status to determine mode */
	rc = _pt_request_hid_output_pip2_send_cmd(dev, &pip2_cmd,
		PIP2_CMD_ID_STATUS, NULL, 0, read_buf, &actual_read_len);

	pt_debug(dev, DL_WARN, "%s: PIP2 STATUS command rc = %d\n",
		__func__, rc);

	if (rc) {
		/*
		 * FW supporting PIP1.x does not support this command
		 * so assume success
		 */
		*mode = CY_MODE_OPERATIONAL;
		/* Flush i2c as a precaution */
		pt_debug(dev, DL_WARN,
			"%s: Flush I2C bus after failed cmd\n", __func__);
		pt_flush_i2c(cd, 0);
	} else {
		status = read_buf[PIP2_RESPONSE_STATUS_OFFSET];
		boot = read_buf[PIP2_RESPONSE_BODY_OFFSET] & 0x01;
		pt_pr_buf(dev, DL_WARN, read_buf, actual_read_len, "Mode");

		if (status == PIP2_RSP_ERR_NONE && boot == 0x00)
			*mode = CY_MODE_BOOTLOADER;
		else if (status == PIP2_RSP_ERR_NONE && boot == 0x01)
			*mode = CY_MODE_OPERATIONAL;
		else
			*mode = CY_MODE_UNKNOWN;
	}

	if (resume_scan) {
		rc = pt_hid_output_resume_scanning_(cd);
		if (rc)
			pt_debug(dev, DL_ERROR, "%s Resume Scan Failed\n",
			__func__);
	}

	return rc;
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
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);
	unsigned long value;
	int retval = 0;

	retval = kstrtoul(buf, 10, &value);
	if (retval < 0) {
		pt_debug(dev, DL_ERROR, "%s: Invalid value\n", __func__);
		goto cyttsp5_drv_irq_store_error_exit;
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

cyttsp5_drv_irq_store_error_exit:

	return size;
}

/*
 * Gets user input from sysfs and parse it
 * return size of parsed output buffer
 */

#define CY_MAX_CONFIG_BYTES_DEC    256
#define CYTTSP5_INPUT_ELEM_SZ_DEC   10

static int cyttsp5_ic_parse_input_dec(struct device *dev, const char *buf,
		size_t buf_size, u32 *ic_buf, size_t ic_buf_size)
{
	const char *pbuf = buf;
	unsigned long value;
	char scan_buf[CYTTSP5_INPUT_ELEM_SZ_DEC];
	u32 i = 0;
	u32 j;
	int last = 0;
	int ret;

	pt_debug(dev, DL_DEBUG,
		"%s: pbuf=%p buf=%p size=%zu %s=%zu buf=%s\n",
		__func__, pbuf, buf, buf_size, "scan buf size",
		(size_t)CYTTSP5_INPUT_ELEM_SZ_DEC, buf);

	while (pbuf <= (buf + buf_size)) {
		if (i >= CY_MAX_CONFIG_BYTES_DEC) {
			pt_debug(dev, DL_ERROR, "%s: %s size=%d max=%d\n",
					__func__,
					"Max cmd size exceeded", i,
					CY_MAX_CONFIG_BYTES_DEC);
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

		memset(scan_buf, 0, CYTTSP5_INPUT_ELEM_SZ_DEC);
		if ((last == ',') && (*pbuf == ',')) {
			pt_debug(dev, DL_ERROR, "%s: %s \",,\" not allowed.\n",
					__func__, "Invalid data format.");
			return -EINVAL;
		}
		for (j = 0; j < (CYTTSP5_INPUT_ELEM_SZ_DEC - 1)
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
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);
	unsigned long value;
	int rc;
	u8 return_data[8];
	static u8 wd_disabled;
	u32 input_data[2];
	int length;

	/*maximal input two data*/
	length = cyttsp5_ic_parse_input_dec(dev, buf, size,
			input_data, 3);
	if (length <= 0) {
		pt_debug(dev, DL_ERROR, "%s: %s failed\n", __func__,
				"cyttsp5_ic_parse_input_dec");
		goto pt_drv_debug_store_exit;
	}
	value = input_data[0];

	pt_debug(dev, DL_DEBUG, "%s: Debug Cmd Recieved (cd=%p)\n",
		__func__, cd);

	/* Start watchdog timer command */
	if (value == PT_DRV_DBG_START_WD) {
		pt_debug(dev, DL_INFO, "%s: Cmd: Start Watchdog\n", __func__);
		wd_disabled = 0;
		pt_start_wd_timer(cd);
		goto pt_drv_debug_store_exit;
	}

	/* Stop watchdog timer temporarily */
	pt_stop_wd_timer(cd);

	if (value == PT_DRV_DBG_STOP_WD) {
		pt_debug(dev, DL_INFO, "%s: Cmd: Stop Watchdog\n", __func__);
		wd_disabled = 1;
		goto pt_drv_debug_store_exit;
	}

	switch (value) {
	case PT_DUT_DBG_HID_RESET:
		pt_debug(dev, DL_INFO, "%s: Cmd: hid_reset\n", __func__);
		cyttsp5_hid_cmd_reset(cd);
		break;
	case PT_DUT_DBG_HID_SET_POWER_ON:
		pt_debug(dev, DL_INFO, "%s: Cmd: hid_set_power_on\n", __func__);
		cyttsp5_hid_cmd_set_power(cd, HID_POWER_ON);
		wd_disabled = 0;
		break;
	case PT_DUT_DBG_HID_SET_POWER_SLEEP:
		pt_debug(dev, DL_INFO, "%s: Cmd: hid_set_power_off\n",
			 __func__);
		wd_disabled = 1;
		cyttsp5_hid_cmd_set_power(cd, HID_POWER_SLEEP);
		break;
	case PT_DUT_DBG_SOFT_RESET:
		pt_debug(dev, DL_INFO, "%s: Cmd: Soft Reset\n", __func__);
		rc = cyttsp5_hw_soft_reset(cd);
		break;
	case PT_DUT_DBG_RESET:
		/* PATCH - TC3300 has to slow clk before a reset */
		if (cd->use_tc3300_pip20_len_field)
			_pt_request_hid_output_start_bl(dev, 1);

		pt_debug(dev, DL_INFO, "%s: Cmd: Hard Reset\n", __func__);
		rc = pt_hw_hard_reset(cd);
		break;
	case PT_DUT_DBG_HID_NULL:
		pt_debug(dev, DL_INFO, "%s: Cmd: Ping (null)\n", __func__);
		cyttsp5_hid_output_null(cd);
		break;
	case PT_DUT_DBG_HID_ENTER_BL:
		pt_debug(dev, DL_INFO, "%s: Cmd: start_bootloader\n", __func__);
		pt_hid_output_start_bootloader(cd);
		break;
	case PT_DUT_DBG_HID_SYSINFO:
		pt_debug(dev, DL_INFO, "%s: Cmd: get_sysinfo\n", __func__);
		cyttsp5_hid_output_get_sysinfo(cd);
		break;
	case PT_DUT_DBG_HID_SUSPEND_SCAN:
		pt_debug(dev, DL_INFO, "%s: Cmd: suspend_scanning\n", __func__);
		pt_hid_output_suspend_scanning(cd);
		break;
	case PT_DUT_DBG_HID_RESUME_SCAN:
		pt_debug(dev, DL_INFO, "%s: Cmd: resume_scanning\n", __func__);
		pt_hid_output_resume_scanning(cd);
		break;
	case PT_DUT_DBG_HID_DESC:
		pt_debug(dev, DL_INFO, "%s: Cmd: get_hid_desc\n", __func__);
		cyttsp5_get_hid_descriptor(cd, &cd->hid_desc);
		break;

	case HID_OUTPUT_BL_VERIFY_APP_INTEGRITY:
		pt_debug(dev, DL_INFO, "%s: Cmd: verify app integ\n", __func__);
		cyttsp5_hid_output_bl_verify_app_integrity(cd, &return_data[0]);
		break;
	case HID_OUTPUT_BL_GET_INFO:
		pt_debug(dev, DL_INFO, "%s: Cmd: bl get info\n", __func__);
		cyttsp5_hid_output_bl_get_information(cd, return_data);
		break;
	case HID_OUTPUT_BL_PROGRAM_AND_VERIFY:
		pt_debug(dev, DL_INFO, "%s: Cmd: program and verify\n",
			__func__);
		cyttsp5_hid_output_bl_program_and_verify(cd, 0, NULL);
		break;
	case HID_OUTPUT_BL_LAUNCH_APP:
		pt_debug(dev, DL_INFO, "%s: Cmd: launch app\n", __func__);
		cyttsp5_hid_output_bl_launch_app(cd);
		break;
	case HID_OUTPUT_BL_INITIATE_BL:
		pt_debug(dev, DL_INFO, "%s: Cmd: initiate bl\n", __func__);
		cyttsp5_hid_output_bl_initiate_bl(cd, 0, NULL, 0, NULL);
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
			memset(cd->tthe_buf, 0, CY_MAX_PRBUF_SIZE);
		else
			pt_debug(dev, DL_INFO, "%s : tthe_buf not existed\n",
				__func__);
		break;
#endif
	case PT_DRV_DBG_SUSPEND:
		pt_debug(dev, DL_INFO, "%s: TTDL: Core Sleep\n", __func__);
		rc = cyttsp5_core_sleep(cd);
		if (rc)
			pt_debug(dev, DL_ERROR, "%s: Suspend failed rc=%d\n",
				__func__, rc);
		else
			pt_debug(dev, DL_INFO, "%s: Suspend succeeded\n",
				__func__);
		break;

	case PT_DRV_DBG_RESUME:
		pt_debug(dev, DL_INFO, "%s: TTDL: Wake\n", __func__);
		rc = cyttsp5_core_wake(cd);
		if (rc)
			pt_debug(dev, DL_ERROR, "%s: Resume failed rc=%d\n",
				__func__, rc);
		else
			pt_debug(dev, DL_INFO, "%s: Resume succeeded\n",
				__func__);
		break;
	case PT_DRV_DBG_CLEAR_PARM_LIST:
		pt_debug(dev, DL_INFO, "%s: TTDL: Clear Parameter List\n",
			__func__);
		pt_erase_parameter_list(cd);
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
		if (input_data[1] == 0) {
			cd->watchdog_interval = input_data[1];
			pt_stop_wd_timer(cd);
		} else if (input_data[1] > 0 && input_data[1] < 200) {
			pt_debug(dev, DL_ERROR,
				"%s: ERROR: Invalid watchdog interval: %d\n",
				__func__, input_data[1]);
		} else {
			cd->watchdog_interval = input_data[1];
			pt_start_wd_timer(cd);
		}
		pt_debug(dev, DL_INFO, "%s: Watchdog interval Set to: %d\n",
			__func__, cd->watchdog_interval);
		mutex_unlock(&(cd->system_lock));
		break;
	case PT_DRV_DBG_SHOW_TIMESTAMP:
		mutex_lock(&cd->system_lock);
		cd->show_timestamp = input_data[1];
		pt_debug(dev, DL_INFO, "%s: TTDL: Set show_timestamp: %d\n",
			__func__, cd->show_timestamp);
		mutex_unlock(&(cd->system_lock));
		break;
/*
 * --- PATCH ---
 * This conditional and all code can be removed when TC3300 support is no
 * longer needed.
 */
#if defined(PT_TDDI_SUPPORT) || defined(PT_TT_DISCRETE_SUPPORT)
	case PT_DRV_DBG_SET_PIP_2P0_FLAG:
		mutex_lock(&cd->system_lock);
		cd->pip_2p0_prot_active = input_data[1];
		pt_debug(dev, DL_WARN,
			"%s: TTDL: Force pip_2p0_prot_active: %d\n",
			__func__, cd->pip_2p0_prot_active);
		mutex_unlock(&(cd->system_lock));
		break;

	case PT_DRV_DBG_TC3300_LEN_FLAG:
		mutex_lock(&cd->system_lock);
		cd->use_tc3300_pip20_len_field = input_data[1];
		pt_debug(dev, DL_WARN,
			"%s: TTDL: Force use_tc3300_pip20_len_field: %d\n",
			__func__, cd->use_tc3300_pip20_len_field);
		mutex_unlock(&(cd->system_lock));
		break;
#endif
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
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);
	ssize_t ret;

	mutex_lock(&cd->system_lock);
	if (cd->sleep_state == SS_SLEEP_ON)
		ret = snprintf(buf, CY_MAX_PRBUF_SIZE, "off\n");
	else
		ret = snprintf(buf, CY_MAX_PRBUF_SIZE, "on\n");
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
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);
	ssize_t ret;

	mutex_lock(&cd->system_lock);
	ret = snprintf(buf, CY_MAX_PRBUF_SIZE, "0x%02X\n",
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
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);
	unsigned long value;
	int ret;

	if (!cd->features.easywake)
		return -EINVAL;

	ret = kstrtoul(buf, 10, &value);
	if (ret < 0)
		return ret;

	if (value > 0xFF)
		return -EINVAL;

	pm_runtime_get_sync(dev);

	mutex_lock(&cd->system_lock);
	if (cd->sysinfo.ready && IS_PIP_VER_GE(&cd->sysinfo, 1, 2))
		cd->easy_wakeup_gesture = (u8)value;
	else
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
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);
	ssize_t ret;

	mutex_lock(&cd->system_lock);
	ret = snprintf(buf, CY_MAX_PRBUF_SIZE, "0x%02X\n",
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
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);
	ssize_t ret = 0;
	int i;

	mutex_lock(&cd->system_lock);

	for (i = 0; i < cd->gesture_data_length; i++)
		ret += snprintf(buf + ret, CY_MAX_PRBUF_SIZE - ret,
				"0x%02X\n", cd->gesture_data[i]);

	ret += snprintf(buf + ret, CY_MAX_PRBUF_SIZE - ret,
			"(%d bytes)\n", cd->gesture_data_length);

	mutex_unlock(&cd->system_lock);
	return ret;
}
#endif

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
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);
	ssize_t ret;

	ret = snprintf(buf, CY_MAX_PRBUF_SIZE, "0x%02X\n",
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
	struct cyttsp5_platform_data *pdata = dev_get_platdata(dev);
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);
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
		"Boot Mode GPIO           ", pdata->core_pdata->boot_mode_gpio,
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
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);
	u32 input_data[1];
	int length;

	/* Maximum input of one value */
	length = cyttsp5_ic_parse_input_dec(dev, buf, size, input_data, 2);
	if (length <= 0 || length > 1) {
		pt_debug(dev, DL_ERROR, "%s: %s failed, parm count error\n",
			__func__, "cyttsp5_ic_parse_input_dec");
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
	struct  cyttsp5_core_data *cd = dev_get_drvdata(dev);
	ssize_t ret = 0;
	u8      status;
	u32     value = 0;

	status = pt_hid_output_get_param(cd, cd->get_param_id, &value);
	if (status) {
		pt_debug(dev, DL_ERROR, "%s: %s Failed, status = %d\n",
			__func__, "pt_hid_output_get_param", status);
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
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);
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
#endif

#ifdef TTDL_DIAGNOSTICS
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
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);
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
#endif

static struct device_attribute attributes[] = {
	__ATTR(ic_ver, S_IRUGO, pt_ic_ver_show, NULL),
	__ATTR(drv_ver, S_IRUGO, pt_drv_ver_show, NULL),
	__ATTR(hw_reset, S_IWUSR, NULL, pt_hw_reset_store),
	__ATTR(hw_irq_stat, S_IRUSR, pt_hw_irq_stat_show, NULL),
	__ATTR(drv_irq, S_IRUSR | S_IWUSR, pt_drv_irq_show,
		pt_drv_irq_store),
	__ATTR(dut_debug, S_IWUSR, NULL, pt_drv_debug_store),
	__ATTR(sleep_status, S_IRUSR, pt_sleep_status_show, NULL),
	__ATTR(easy_wakeup_gesture, S_IRUSR | S_IWUSR,
		pt_easy_wakeup_gesture_show,
		pt_easy_wakeup_gesture_store),
#ifdef EASYWAKE_TSG6
	__ATTR(easy_wakeup_gesture_id, S_IRUSR,
		pt_easy_wakeup_gesture_id_show, NULL),
	__ATTR(easy_wakeup_gesture_data, S_IRUSR,
		pt_easy_wakeup_gesture_data_show, NULL),
#endif
	__ATTR(panel_id, S_IRUGO, pt_panel_id_show, NULL),
	__ATTR(platform_data, S_IRUGO, pt_platform_data_show, NULL),
	__ATTR(get_param, S_IRUSR | S_IWUSR,
		pt_get_param_show, pt_get_param_store),
#ifdef TTDL_DIAGNOSTICS
	__ATTR(t_refresh, S_IRUSR | S_IWUSR,
		pt_t_refresh_show, pt_t_refresh_store),
#endif
};

static int add_sysfs_interfaces(struct device *dev)
{
	int i;
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);

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

static void remove_sysfs_interfaces(struct device *dev)
{
	u32 i;
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);

	cd->sysfs_nodes_created = false;
	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		device_remove_file(dev, attributes + i);
}

#ifdef TTHE_TUNER_SUPPORT
static int tthe_debugfs_open(struct inode *inode, struct file *filp)
{
	struct cyttsp5_core_data *cd = inode->i_private;

	filp->private_data = inode->i_private;

	if (cd->tthe_buf)
		return -EBUSY;

	cd->tthe_buf = kzalloc(CY_MAX_PRBUF_SIZE, GFP_KERNEL);
	if (!cd->tthe_buf)
		return -ENOMEM;

	return 0;
}

static int tthe_debugfs_close(struct inode *inode, struct file *filp)
{
	struct cyttsp5_core_data *cd = filp->private_data;

	filp->private_data = NULL;

	kfree(cd->tthe_buf);
	cd->tthe_buf = NULL;

	return 0;
}

static ssize_t tthe_debugfs_read(struct file *filp, char __user *buf,
		size_t count, loff_t *ppos)
{
	struct cyttsp5_core_data *cd = filp->private_data;
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

static struct cyttsp5_core_nonhid_cmd _cyttsp5_core_nonhid_cmd = {
	.start_bl             = _pt_request_hid_output_start_bl,
	.suspend_scanning     = _pt_request_hid_output_suspend_scanning,
	.resume_scanning      = _pt_request_hid_output_resume_scanning,
	.get_param            = _pt_request_hid_output_get_param,
	.set_param            = _cyttsp5_request_hid_output_set_param,
	.verify_config_block_crc =
		_cyttsp5_request_hid_output_verify_config_block_crc,
	.get_config_row_size  = _cyttsp5_request_hid_output_get_config_row_size,
	.get_data_structure   = _cyttsp5_request_hid_output_get_data_structure,
	.run_selftest         = _cyttsp5_request_hid_output_run_selftest,
	.get_selftest_result  = _cyttsp5_request_hid_output_get_selftest_result,
	.calibrate_idacs      = _cyttsp5_request_hid_output_calibrate_idacs,
	.initialize_baselines =
		_cyttsp5_request_hid_output_initialize_baselines,
	.exec_panel_scan      = _cyttsp5_request_hid_output_exec_panel_scan,
	.retrieve_panel_scan  = _cyttsp5_request_hid_output_retrieve_panel_scan,
	.write_conf_block     = _cyttsp5_request_hid_output_write_conf_block,
	.user_cmd             = _cyttsp5_request_hid_output_user_cmd,
	.get_bl_info          = _cyttsp5_request_hid_output_bl_get_information,
	.initiate_bl          = _cyttsp5_request_hid_output_bl_initiate_bl,
	.launch_app           = _cyttsp5_request_hid_output_launch_app,
	.prog_and_verify      = _pt_request_hid_output_bl_program_and_verify,
	.verify_app_integrity =
		_cyttsp5_request_hid_output_bl_verify_app_integrity,
	.get_panel_id         = _cyttsp5_request_hid_output_bl_get_panel_id,
	.pip2_send_cmd        = _pt_request_hid_output_pip2_send_cmd,
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
	.request_pip2_get_mode   = _pt_request_pip2_get_mode,
	.request_active_pip_prot = _pt_request_active_pip_protocol,
#ifdef TTHE_TUNER_SUPPORT
	.request_tthe_print      = _cyttsp5_request_tthe_print,
#endif
	.nonhid_cmd              = &_cyttsp5_core_nonhid_cmd,
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

static int cyttsp5_probe_module(struct cyttsp5_core_data *cd,
		struct cyttsp5_module *module)
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

static void cyttsp5_release_module(struct cyttsp5_core_data *cd,
		struct cyttsp5_module *module)
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

static void cyttsp5_probe_modules(struct cyttsp5_core_data *cd)
{
	struct cyttsp5_module *m;
	int rc = 0;

	mutex_lock(&module_list_lock);
	list_for_each_entry(m, &module_list, node) {
		pt_debug(cd->dev, DL_ERROR, "%s: Probe module %s\n",
			__func__, m->name);
		rc = cyttsp5_probe_module(cd, m);
		if (rc)
			pt_debug(cd->dev, DL_ERROR,
				"%s: Probe fails for module %s\n",
				__func__, m->name);
	}
	mutex_unlock(&module_list_lock);
}

static void cyttsp5_release_modules(struct cyttsp5_core_data *cd)
{
	struct cyttsp5_module *m;

	mutex_lock(&module_list_lock);
	list_for_each_entry(m, &module_list, node)
		cyttsp5_release_module(cd, m);
	mutex_unlock(&module_list_lock);
}

struct cyttsp5_core_data *cyttsp5_get_core_data(char *id)
{
	struct cyttsp5_core_data *d;

	list_for_each_entry(d, &core_list, node)
		if (!strncmp(d->core_id, id, 20))
			return d;
	return NULL;
}
EXPORT_SYMBOL_GPL(cyttsp5_get_core_data);

static void cyttsp5_add_core(struct device *dev)
{
	struct cyttsp5_core_data *d;
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);

	mutex_lock(&core_list_lock);
	list_for_each_entry(d, &core_list, node)
		if (d->dev == dev)
			goto unlock;

	list_add(&cd->node, &core_list);
unlock:
	mutex_unlock(&core_list_lock);
}

static void cyttsp5_del_core(struct device *dev)
{
	struct cyttsp5_core_data *d, *d_n;

	mutex_lock(&core_list_lock);
	list_for_each_entry_safe(d, d_n, &core_list, node)
		if (d->dev == dev) {
			list_del(&d->node);
			goto unlock;
		}
unlock:
	mutex_unlock(&core_list_lock);
}

int cyttsp5_register_module(struct cyttsp5_module *module)
{
	struct cyttsp5_module *m;
	struct cyttsp5_core_data *cd;

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
		cyttsp5_probe_module(cd, module);
	mutex_unlock(&core_list_lock);

unlock:
	mutex_unlock(&module_list_lock);
	return rc;
}
EXPORT_SYMBOL_GPL(cyttsp5_register_module);

void cyttsp5_unregister_module(struct cyttsp5_module *module)
{
	struct cyttsp5_module *m, *m_n;
	struct cyttsp5_core_data *cd;

	if (!module)
		return;

	mutex_lock(&module_list_lock);

	/* Release the module for each core */
	mutex_lock(&core_list_lock);
	list_for_each_entry(cd, &core_list, node)
		cyttsp5_release_module(cd, module);
	mutex_unlock(&core_list_lock);

	list_for_each_entry_safe(m, m_n, &module_list, node)
		if (m == module) {
			list_del(&m->node);
			break;
		}

	mutex_unlock(&module_list_lock);
}
EXPORT_SYMBOL_GPL(cyttsp5_unregister_module);

void *cyttsp5_get_module_data(struct device *dev, struct cyttsp5_module *module)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);
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
EXPORT_SYMBOL(cyttsp5_get_module_data);

#ifdef CONFIG_HAS_EARLYSUSPEND
static void cyttsp5_early_suspend(struct early_suspend *h)
{
	struct cyttsp5_core_data *cd =
		container_of(h, struct cyttsp5_core_data, es);

	call_atten_cb(cd, CY_ATTEN_SUSPEND, 0);
}

static void cyttsp5_late_resume(struct early_suspend *h)
{
	struct cyttsp5_core_data *cd =
		container_of(h, struct cyttsp5_core_data, es);

	call_atten_cb(cd, CY_ATTEN_RESUME, 0);
}

static void cyttsp5_setup_early_suspend(struct cyttsp5_core_data *cd)
{
	cd->es.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	cd->es.suspend = cyttsp5_early_suspend;
	cd->es.resume = cyttsp5_late_resume;

	register_early_suspend(&cd->es);
}
#elif defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self,
		unsigned long event, void *data)
{
	struct cyttsp5_core_data *cd =
		container_of(self, struct cyttsp5_core_data, fb_notifier);
	struct fb_event *evdata = data;
	int *blank;

	if (event != FB_EVENT_BLANK || !evdata)
		goto exit;

	blank = evdata->data;
	if (*blank == FB_BLANK_UNBLANK) {
		pt_debug(cd->dev, DL_INFO, "%s: UNBLANK!\n", __func__);
		if (cd->fb_state != FB_ON) {
			call_atten_cb(cd, CY_ATTEN_RESUME, 0);
			cd->fb_state = FB_ON;
		}
	} else if (*blank == FB_BLANK_POWERDOWN) {
		pt_debug(cd->dev, DL_INFO, "%s: POWERDOWN!\n", __func__);
		if (cd->fb_state != FB_OFF) {
			call_atten_cb(cd, CY_ATTEN_SUSPEND, 0);
			cd->fb_state = FB_OFF;
		}
	}

exit:
	return 0;
}

static void cyttsp5_setup_fb_notifier(struct cyttsp5_core_data *cd)
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
	struct cyttsp5_core_data *cd = container_of(work,
			struct cyttsp5_core_data, irq_work);

	cyttsp5_irq(cd->irq, (void *)cd);
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
	struct cyttsp5_core_data *cd = (struct cyttsp5_core_data *)handle;

	queue_work(parade_wq, &cd->irq_work);
	return 0;
}
#endif /* --- End PT_PTSBC_SUPPORT --- */

/*******************************************************************************
 * FUNCTION: cyttsp5_setup_irq_gpio
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
static int cyttsp5_setup_irq_gpio(struct cyttsp5_core_data *cd)
{
	struct device *dev = cd->dev;
	unsigned long irq_flags;
	int rc = 0;
#ifdef PT_PTSBC_SUPPORT
	u32 int_handle;
#endif

	/* Initialize IRQ */
	dev_vdbg(dev, "%s: Value Passed to gpio_to_irq =%d\n", __func__,
			cd->cpdata->irq_gpio);
	cd->irq = gpio_to_irq(cd->cpdata->irq_gpio);
	dev_vdbg(dev, "%s: Value Returned from gpio_to_irq =%d\n", __func__,
			cd->irq);
	if (cd->irq < 0)
		return -EINVAL;

	cd->irq_enabled = true;

	pt_debug(dev, DL_INFO, "%s: initialize threaded irq=%d\n",
		__func__, cd->irq);
	if (cd->cpdata->level_irq_udelay > 0)
#ifdef PT_PTSBC_SUPPORT
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
	rc = request_threaded_irq(cd->irq, NULL, cyttsp5_irq, irq_flags,
		dev_name(dev), cd);
	if (rc < 0)
		pt_debug(dev, DL_ERROR, "%s: Error, could not request irq\n",
			__func__);
#endif

	return rc;
}

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
static int _pt_ttdl_restart(struct device *dev, int force)
{
	int rc;
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);
	struct i2c_client *client =
		(struct i2c_client *)container_of(dev, struct i2c_client, dev);

	/* Before restarting, release modules before reprobing below */
	cyttsp5_btn_release(dev);
	cyttsp5_mt_release(dev);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pt_debug(dev, DL_ERROR,
			"%s I2C functionality not Supported\n", __func__);
		return -EIO;
	}

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_DEVICETREE_SUPPORT
	rc = cyttsp5_devtree_create_and_get_pdata(dev);
	if (rc < 0)
		return rc;
#endif

	pt_debug(dev, DL_INFO, "%s: Call pt_startup\n", __func__);
	rc = pt_startup(cd, true);
	if (rc < 0)
		/* Startup Failed - We are done here */
		goto ttdl_error_startup;

	/* Startup passed, probe required modules next */
	rc = cyttsp5_mt_probe(dev);
	if (rc < 0) {
		pt_debug(dev, DL_ERROR,
			"%s: Error, fail mt probe\n", __func__);
		goto ttdl_error_startup;
	}

	rc = cyttsp5_btn_probe(dev);
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

	/* No issues */
	pt_debug(cd->dev, DL_WARN,
			"%s: Well Done! TTDL Restart Successfull\n", __func__);
	rc = 0;
	goto ttdl_no_error;

ttdl_error_startup_mt:
	pr_err("%s PARADE error_startup_mt\n", __func__);
	cyttsp5_mt_release(dev);
ttdl_error_startup:
	pr_err("%s PARADE error_startup\n", __func__);
	pm_runtime_disable(dev);
ttdl_no_error:
	return rc;
}

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
	unsigned long value;
	int length;

	/* maximum input of one value */
	length = cyttsp5_ic_parse_input_dec(dev, buf, size, input_data, 2);
	if (length <= 0) {
		pt_debug(dev, DL_ERROR, "%s: %s failed\n", __func__,
				"cyttsp5_ic_parse_input_dec");
	}
	value = input_data[0];
	if (_pt_ttdl_restart(dev, value) != 0)
		pt_debug(dev, DL_ERROR, "%s: %s failed\n", __func__,
			"_pt_ttdl_restart()");

	return size;
}
static DEVICE_ATTR(ttdl_restart, S_IWUSR, NULL, pt_ttdl_restart_store);

/*******************************************************************************
 * FUNCTION: pt_pip2_set_boot_mode_pin
 *
 * SUMMARY: Set the state of the host_mode gpio
 *
 * PARAMETERS:
 *      *dev   - pointer to device structure
 *       value - <1|0> set value for GPIO
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
 * FUNCTION: pt_ttdl_bist_show
 *
 * SUMMARY: Show method for the ttdl_bist sysfs node. This built in self test
 *	will test that the TP_XRES, IRQ and HOST_MODE pins are operational.
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
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);
	struct pip2_cmd_structure pip2_cmd;
	ssize_t ret;
	u16 actual_read_len;
	u8 read_buf[256];
	u8 irq_err_str[64];
	u8 xres_err_str[64];
	u8 hm_err_str[64];
	bool bl_detected_host_mode_set = false;
	int rc;
	int count            = 0;
	int status           = 1;    /* 0 = Pass, !0 = fail */
	int bytes_read       = 0;
	u8 pip_ver_minor     = 0;
	u8 pip_ver_major     = 0;
	u8 irq_toggled       = 0x0F; /* default to untested */
	u8 xres_toggled      = 0x0F; /* default to untested */
	u8 host_mode_toggled = 0x0F; /* default to untested */

	/*
	 * ----- Initialize TTDL to begin BIST -----
	 */

	memset(irq_err_str, 0, sizeof(irq_err_str));
	memset(xres_err_str, 0, sizeof(xres_err_str));
	memset(hm_err_str, 0, sizeof(hm_err_str));

	/* Turn off the TTDL WD during the test */
	pt_stop_wd_timer(cd);

	/* Suspend Scanning */
	rc = pt_hid_output_suspend_scanning(cd);
	if (rc)
		pt_debug(dev, DL_ERROR, "%s Suspend Scan Failed\n",
			__func__);

	/* Clear the startup bit mask, reset and restart will re-populate it */
	cd->startup_status = STARTUP_STATUS_START;

	/*
	 * ----- IRQ BIST TEST -----
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
		goto print_results;
	}

	/* Try sending a PIP command to see if we get a response */
	rc = _pt_request_active_pip_protocol(dev, &pip_ver_major,
		&pip_ver_minor);
	if (rc) {
		strcpy(irq_err_str, "- likely open or shorted to VDDI.");
		pt_debug(dev, DL_ERROR, "%s: request_active_pip_prot failed\n",
			__func__);
		irq_toggled = 0;
		goto print_results;
	}
	if (cd->irq_count > 0) {
		pt_debug(dev, DL_INFO, "%s: irq_count=%d\n",
			__func__, cd->irq_count);
		irq_toggled = 1;
	}

bist_tp_xres:
	/*
	 * ----- TP_XRES BIST TEST -----
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

	/* Perform a hard 50ms XRES toggle and check for reset sentinel */
	if (cd->use_tc3300_pip20_len_field) {
		/* -PATCH- For TC3300 start_bl will slow DUT clock */
		rc = _pt_request_hid_output_start_bl(dev, 1);
	} else
		rc = pt_hw_hard_reset(cd);

	msleep(150);
	pt_debug(dev, DL_INFO, "%s: TP_XRES BIST hard reset rc=%d",
		__func__, rc);

	/* Look for BL or FW reset sentinels */
	if (!rc && ((cd->startup_status & STARTUP_STATUS_BL_RESET_SENTINEL) ||
	    (cd->startup_status & STARTUP_STATUS_FW_RESET_SENTINEL))) {
		pt_debug(dev, DL_INFO, "%s: hard XRES pass\n", __func__);
		xres_toggled = 1;
	} else {
		/*
		 * Hard reset failed, however some additional information
		 * could be determined. Try a soft reset to see if DUT resets
		 * with the possible outcomes:
		 * - if it resets the line is not stuck asserted
		 * - if it does not reset the line could be stuck asserted
		 */
		xres_toggled = 0;
		rc = cyttsp5_hw_soft_reset(cd);
		msleep(30);
		pt_debug(dev, DL_INFO, "%s: TP_XRES BIST soft reset rc=%d",
			__func__, rc);
		if (rc) {
			strcpy(xres_err_str, "- likely open.");
			pt_debug(dev, DL_ERROR,
				"%s: Hard reset failed, soft reset failed to send %s\n",
				__func__, xres_err_str);
			goto print_results;
		}
		if (cd->startup_status & STARTUP_STATUS_BL_RESET_SENTINEL ||
		    cd->startup_status & STARTUP_STATUS_FW_RESET_SENTINEL) {
			strcpy(xres_err_str,
				"- likely open or stuck de-asserted.");
			pt_debug(dev, DL_ERROR,
				"%s: Hard reset failed, soft reset passed-%s\n",
				__func__, xres_err_str);
		} else if (cd->startup_status == 0) {
			strcpy(xres_err_str, "- likely stuck asserted.");
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
	 * ----- HOST_MODE BIST TEST -----
	 * This test will ensure there is a good connection between the host
	 * and the DUT on the host_mode pin. The pin will be asserted and
	 * then a reset will be triggered. If the connection is good the DUT
	 * will remain in the BL mode and the PIP2 command GET_LAST_ERRNO will
	 * respond with the correct state of the host_mode pin. The host_mode
	 * pin will then be de-asserted and the DUT will be reset again. Now
	 * if there is no valid FW in FLASH the DUT will remain in the BL and
	 * seding the GET_LAST_ERRNO again should show the host_mode pin is in
	 * the de-asserted state, however if there is valid FW in FLASH the DUT
	 * will boot into flash and if we see the FW Reset Sentinel we also
	 * know the pin correctly toggled.
	 */
	pt_debug(dev, DL_INFO, "%s: ----- Start HOST_MODE BIST ----", __func__);

	/* Ensure we have nothing pending on the i2c bus */
	while (pt_check_irq_asserted(cd) && count < 5) {
		count++;
		bytes_read += pt_flush_i2c(cd, 0);
	}

	/* Assert the boot mode pin to force BL to stay in the BL ROM */
	pip2_set_boot_mode_pin(dev, 0);

	/* Perform a hard 50ms XRES toggle and verify we stay in the BL */
	if (cd->use_tc3300_pip20_len_field) {
		/* -PATCH- For TC3300 start_bl will slow DUT clock */
		rc = _pt_request_hid_output_start_bl(dev, 1);
		/* -PATCH- Fake seeing the BL sentinel for TC3300 */
		cd->startup_status |= STARTUP_STATUS_BL_RESET_SENTINEL;
	} else
		rc = pt_hw_hard_reset(cd);
	msleep(150);
	pt_debug(dev, DL_INFO, "%s: HOST_MODE BIST hard reset rc=%d",
		__func__, rc);

	/*
	 * Should be in the BL now, if we did not see a FW reset sentinel
	 * send the PIP GET_LAST_ERRNO cmd to the BL and parse the response
	 * seeing BL saw host mode pin asserted
	 */
	pt_debug(dev, DL_INFO, "%s: Startup_status = 0x%04X\n",
		__func__, cd->startup_status);
	if (cd->startup_status & STARTUP_STATUS_FW_RESET_SENTINEL) {
		strcpy(hm_err_str, "- likely open or stuck de-asserted.");
		pt_debug(dev, DL_ERROR,
			"%s: Not in BL when expected to be, host_mode %s\n",
			__func__, hm_err_str);
		host_mode_toggled = 0;
		/* No need to exit BL, goto print results directly */
		goto print_results;
	} else {
		rc = _pt_request_hid_output_pip2_send_cmd(dev, &pip2_cmd,
			PIP2_CMD_ID_GET_LAST_ERRNO, NULL, 0, read_buf,
			&actual_read_len);
		if (rc) {
			/*
			 * Shouldn't get here - Command failed to send.
			 * Somehow we are not in the BL as this cmd is only
			 * valid in the BL, so the host_mode pin must be
			 * stuck asserted
			 */
			strcpy(hm_err_str, "- likely open or stuck asserted.");
			pt_debug(dev, DL_ERROR,
				"%s: BL cmd failed to send, host_mode %s\n",
				__func__, hm_err_str);
			host_mode_toggled = 0;
			/* No need to exit BL, goto print results directly */
			goto print_results;
		}
		pt_debug(dev, DL_INFO, "%s: BootMode pin state = %d\n",
			__func__, read_buf[6]);

		/*
		 * Must see a BL reset sentinel and a valid response with the
		 * bootmode byte set to 0 showing the host mode pin was asserted
		 */
		if (cd->startup_status & STARTUP_STATUS_BL_RESET_SENTINEL &&
			read_buf[4] == 0 && read_buf[6] == 0x00) {
			bl_detected_host_mode_set = true;
			pt_debug(dev, DL_WARN,
				"%s: BL detected host_mode aserted\n",
				__func__);
		} else {
			strcpy(hm_err_str,
				"- likely open or stuck de-asserted.");
			pt_debug(dev, DL_ERROR,
				"%s: Bootmode bit not low when BL entered, HOST_MODE %s\n",
				__func__, hm_err_str);
			host_mode_toggled = 0;
			goto exit_bl;
		}
	}

	/* De-assert the boot mode pin to force BL to try and load FW */
	pip2_set_boot_mode_pin(dev, 1);

	/* In the BL so send a BL RESET command to reset the DUT */
	rc = _pt_request_hid_output_pip2_send_cmd(dev, &pip2_cmd,
		PIP2_CMD_ID_RESET, NULL, 0, read_buf,
		&actual_read_len);
	if (rc)
		pt_debug(dev, DL_ERROR, "%s: Sending BL RESET cmd failed\n",
			__func__);
	msleep(150);

	/*
	 * Two possible outcomes:
	 * 1) There is no FW in the FLASH we will be back in the BL
	 * 2) There is valid FW we will be running FW
	 */

	/* -PATCH- TC3300 will not produce a BL reset sentinel */
	if (cd->use_tc3300_pip20_len_field)
		cd->startup_status |= STARTUP_STATUS_BL_RESET_SENTINEL;

	pt_debug(dev, DL_INFO, "%s: Startup_status = 0x%04X\n",
		__func__, cd->startup_status);

	/* Test if we are back running FW */
	if (cd->startup_status & STARTUP_STATUS_BL_RESET_SENTINEL &&
	    cd->startup_status & STARTUP_STATUS_FW_RESET_SENTINEL &&
	    bl_detected_host_mode_set) {
		/* In the FW so the host mode pin must have been de-asserted */
		host_mode_toggled = 1;
		goto print_results;
	}

	/* Test if we are back in the boot loader */
	if (cd->startup_status & STARTUP_STATUS_BL_RESET_SENTINEL) {
		/* Send GET_LAST_ERRNO cmd to the BL and parse response */
		rc = _pt_request_hid_output_pip2_send_cmd(dev, &pip2_cmd,
			PIP2_CMD_ID_GET_LAST_ERRNO, NULL, 0, read_buf,
			&actual_read_len);
		if (rc) {
			/* Command failed to send - if host_mode pin is stuck
			 * asserted, and FW is available, the BL will load and
			 * run FW and this command is not valid in FW
			 */
			strcpy(hm_err_str, "- likely stuck asserted.");
			pt_debug(dev, DL_ERROR,
				"%s: BL cmd failed to send, host_mode %s\n",
				__func__, hm_err_str);
			host_mode_toggled = 0;
			goto exit_bl;
		}

		pt_debug(dev, DL_INFO, "%s: BootMode pin state = %d\n",
			__func__, read_buf[6]);
		/*
		 * Must see a valid response with the bootmode byte set to 1
		 * showing the host mode pin was not asserted
		 */
		if (read_buf[4] == 0 && read_buf[6] == 0x01 &&
		    bl_detected_host_mode_set) {
			host_mode_toggled = 1;
			goto print_results;
		} else {
			strcpy(hm_err_str, "- likely stuck de-asserted.");
			pt_debug(dev, DL_ERROR,
				"%s: Bootmode state error when BL entered, HOST_MODE %s\n",
				__func__, hm_err_str);
			host_mode_toggled = 0;
		}
	}

exit_bl:
	/* Send a BL RESET command to get back to running FW */
	rc = _pt_request_hid_output_pip2_send_cmd(dev, &pip2_cmd,
		PIP2_CMD_ID_RESET, NULL, 0, read_buf,
		&actual_read_len);
	if (rc)
		pt_debug(dev, DL_ERROR, "%s: Sending BL RESET cmd failed\n",
			__func__);
	msleep(30);

print_results:
	/*
	 * ----- PRINT OUT BIST RESULTS -----
	 */
	/* Ensure host_mode is de-asserted before we are done */
	pip2_set_boot_mode_pin(dev, 1);

	status = irq_toggled + xres_toggled + host_mode_toggled;
	pt_debug(dev, DL_INFO, "%s: status = %d (%d, %d, %d)\n",
		__func__, status, irq_toggled, xres_toggled, host_mode_toggled);
	ret = sprintf(buf,
		"status %d\n"
		"IRQ connection:       %s %s\n"
		"TP_XRES connection:   %s %s\n"
		"HOST_MODE connection: %s %s\n",
		status == 3 ? 0 : 1,
		irq_toggled == 0x0F ? "[UNTEST]" :
			irq_toggled == 1 ? "[  OK  ]" : "[FAILED]",
		irq_err_str,
		xres_toggled == 0x0F ? "[UNTEST]" :
			xres_toggled == 1 ? "[  OK  ]" : "[FAILED]",
		xres_err_str,
		host_mode_toggled == 0x0F ? "[UNTEST]" :
			host_mode_toggled == 1 ? "[  OK  ]" : "[FAILED]",
		hm_err_str);

	/* Put TTDL back into a known state, issue a ttdl restart if needed */
	pt_debug(dev, DL_INFO, "%s: Startup_status = 0x%04X\n",
		__func__, cd->startup_status);
	pt_start_wd_timer(cd);

	return ret;
}
static DEVICE_ATTR(ttdl_bist,  S_IRUGO, pt_ttdl_bist_show, NULL);

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
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);
	struct cyttsp5_platform_data *pdata = dev_get_platdata(dev);
	ssize_t ret;

	ret = sprintf(buf,
		"%s: 0x%04X\n"
		"%s: %d\n"
		"%s: %d\n"
		"%s: %d\n"
		"%s: %d\n"
		"%s: %s\n"
		"%s: %s\n"
		"%s: %s\n"
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
#endif
		,
		"Startup Status            ", cd->startup_status,
		"TTDL Debug Level          ", cd->debug_level,
		"Use PIP2.0 when in BL     ", cd->use_tc3300_pip20_len_field,
		"PIP2.0 Protocol Active    ", cd->pip_2p0_prot_active,
		"HW Detected               ", cd->hw_detected,
		"GPIO state - IRQ          ",
			cd->cpdata->irq_stat(cd->cpdata, dev) ?
			"HIGH" : "low",
		"GPIO state - Host_Mode    ",
			gpio_get_value(pdata->core_pdata->boot_mode_gpio) ?
			"HIGH" : "low",
		"GPIO state - TP_XRES      ",
			gpio_get_value(pdata->core_pdata->rst_gpio) ?
			"HIGH" : "low",
		"RAM Parm restore list     ", pt_count_parameter_list(cd),
		"Startup Retry Count       ", cd->startup_retry_count,
		"WD - Enabled              ", cd->watchdog_enabled,
		"WD - Interval (ms)        ", cd->watchdog_interval
#ifdef TTDL_DIAGNOSTICS
		, "WD - Triggered Count      ", cd->watchdog_count,
		"WD - IRQ Stuck low count  ", cd->watchdog_irq_stuck_count,
		"WD - Device Access Errors ", cd->watchdog_failed_access_count,
		"IRQ Triggered Count       ", cd->irq_count,
		"HID (comm) Reset Count    ", cd->hid_reset_count,
		"BL Packet Retry Count     ", cd->bl_retry_packet_count,
		"I2C CRC Error Count       ", cd->i2c_crc_error_count,
		"I2C Transmission Errors   ", cd->i2c_transmission_error_count
#endif
	);

#ifdef TTDL_DIAGNOSTICS
	/* Reset all diagnostic counters */
	cd->watchdog_count               = 0;
	cd->irq_count                    = 0;
	cd->watchdog_irq_stuck_count     = 0;
	cd->watchdog_failed_access_count = 0;
	cd->hid_reset_count              = 0;
	cd->bl_retry_packet_count        = 0;
	cd->i2c_crc_error_count          = 0;
	cd->i2c_transmission_error_count = 0;
#endif

	return ret;
}
static DEVICE_ATTR(ttdl_status,  S_IRUGO, pt_ttdl_status_show, NULL);

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
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);
	u32 input_data[1];
	int length;

	/* Maximum input of one value */
	length = cyttsp5_ic_parse_input_dec(dev, buf, size, input_data, 2);
	if (length <= 0) {
		pt_debug(dev, DL_ERROR, "%s: %s failed\n", __func__,
			"cyttsp5_ic_parse_input_dec");
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

/* Required to support the Parade Techologies Development Platform */
#ifdef PT_PTSBC_SUPPORT
static int pt_probe_complete(struct cyttsp5_core_data *cd);

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
	struct cyttsp5_core_data *cd =
			container_of(work, struct cyttsp5_core_data,
					probe_work);
	int rc;

	rc = pt_probe_complete(cd);
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
	struct cyttsp5_core_data *cd = (struct cyttsp5_core_data *)handle;

	if (!cd)
		return;

	pt_debug(cd->dev, DL_INFO, "%s: Watchdog timer triggered\n",
		__func__);

	if (!work_pending(&cd->probe_work))
		schedule_work(&cd->probe_work);
}
#endif /* --- End PT_PTSBC_SUPPORT --- */

/*******************************************************************************
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
int pt_probe(const struct cyttsp5_bus_ops *ops, struct device *dev,
		u16 irq, size_t xfer_buf_size)
{
	struct cyttsp5_core_data *cd;
	struct cyttsp5_platform_data *pdata = dev_get_platdata(dev);
	enum cyttsp5_atten_type type;
	int rc = 0;
#ifndef PT_PTSBC_SUPPORT
	u8 current_dl = CY_INITIAL_DEBUG_LEVEL;
	u8 buf[256];
	u8 pip_ver_major;
	u8 pip_ver_minor;
#endif

	/* Set default static values */
	cyttsp5_bus_ops_save = NULL;

	if (!pdata || !pdata->core_pdata || !pdata->mt_pdata) {
		pt_debug(dev, DL_ERROR, "%s: Missing platform data\n",
				__func__);
		rc = -ENODEV;
		goto error_no_pdata;
	}

	if (pdata->core_pdata->flags & CY_CORE_FLAG_POWEROFF_ON_SLEEP) {
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
	cd->debug_level                = CY_INITIAL_DEBUG_LEVEL;
	cd->show_timestamp             = CY_INITIAL_SHOW_TIME_STAMP;
	scnprintf(cd->core_id, 20, "%s%d", CYTTSP5_CORE_NAME, core_number++);
	cd->hw_detected                = false;
	cd->sysfs_nodes_created        = false;
	/* PATCH - default to TC3300 until TC3310 available */
	cd->use_tc3300_pip20_len_field = 1;
	cd->pip_2p0_prot_active        = 0;
	cd->pip2_prot_active           = 0;
	cd->bl_pip_version_major       = 0;
	cd->bl_pip_version_minor       = 0;
	cd->pip2_cmd_tag_seq           = 0x08; /* PIP2 TAG=1 and 3 bit SEQ=0 */
	cd->get_param_id               = 0;
	cd->watchdog_enabled           = 0;
	cd->hid_reset_count            = 0;
	cd->wd_corrective_action       = 1;

#ifdef PT_PTSBC_SUPPORT
	/*
	 * Extend first WD to allow DDI to be configured before a
	 * WD initiated startup(). Without the DDI configured the FW will
	 * not respond.
	 */
	cd->watchdog_interval          = CY_PTSBC_INIT_WATCHDOG_TIMEOUT;
#else
	cd->watchdog_interval          = CY_WATCHDOG_TIMEOUT;
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
#endif

	/* Initialize mutexes and spinlocks */
	mutex_init(&cd->module_list_lock);
	mutex_init(&cd->system_lock);
	mutex_init(&cd->adap_lock);
	mutex_init(&cd->hid_report_lock);
	spin_lock_init(&cd->spinlock);

	/* Initialize module list */
	INIT_LIST_HEAD(&cd->module_list);

	/* Initialize attention lists */
	for (type = 0; type < CY_ATTEN_NUM_ATTEN; type++)
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
		cd->cpdata->vendor_id : CY_HID_VENDOR_ID;
	cd->hid_core.hid_product_id = (cd->cpdata->product_id) ?
		cd->cpdata->product_id : CY_HID_APP_PRODUCT_ID;
	cd->hid_core.hid_desc_register =
		cpu_to_le16(cd->cpdata->hid_desc_register);

	/* Set platform easywake value */
	cd->easy_wakeup_gesture = cd->cpdata->easy_wakeup_gesture;

	/* Set Panel ID to Not Enabled */
	cd->panel_id = PANEL_ID_NOT_ENABLED;

	dev_set_drvdata(dev, cd);
#ifndef PT_PTSBC_SUPPORT
	/* PtSBC builds will call this function in pt_probe_complete() */
	cyttsp5_add_core(dev);
#endif

	/* Create sysfs nodes not dependent on the DUT being accessible */
	device_create_file(dev, &dev_attr_ttdl_restart);
	device_create_file(dev, &dev_attr_ttdl_status);
	device_create_file(dev, &dev_attr_ttdl_bist);
	device_create_file(dev, &dev_attr_flush_i2c);
	device_create_file(dev, &dev_attr_drv_debug);

	/*
	 * Save the pointer to a global value, which will be used
	 * in ttdl_restart function
	 */
	cyttsp5_bus_ops_save = ops;

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
			msecs_to_jiffies(CY_CORE_PROBE_STARTUP_DELAY_MS));
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
static int pt_probe_complete(struct cyttsp5_core_data *cd)
{
	int rc = 0;
	s32 ret;
	u8 buf[256];
	struct device *dev = cd->dev;
	struct cyttsp5_platform_data *pdata = dev_get_platdata(dev);
	int en_vddi = GPIOE(5);
	int en_avdd = GPIOE(6);
	int en_avee = GPIOE(7);
	u8 current_dl = cd->debug_level;
	u8 pip_ver_major;
	u8 pip_ver_minor;

	/* Override the debug level to at least DL_WARN, during probe */
	if (cd->debug_level < DL_WARN)
		cd->debug_level = DL_WARN;

	pt_debug(cd->dev, DL_DEBUG,
		"%s: PARADE Entering Probe complete function\n", __func__);
	cyttsp5_add_core(cd->dev);

	/*
	 * ================ PtSBC Coconut Board Rev B+ ================
	 * Enable GPIOs to turn on voltage regulators to pwr up TT device
	 * The power up order is: VDDI, AVDD, AVEE
	 */
	/* Call platform init function before setting up the GPIO's */
	if (cd->cpdata->init) {
		pt_debug(cd->dev, DL_INFO, "%s: Init HW\n", __func__);
		rc = cd->cpdata->init(cd->cpdata, 1, cd->dev);
	} else {
		pt_debug(cd->dev, DL_WARN,
			"%s: No HW INIT function\n", __func__);
		rc = 0;
	}
	if (rc < 0) {
		pt_debug(cd->dev, DL_ERROR,
			"%s: HW Init fail r=%d\n", __func__, rc);
	}

	pt_debug(cd->dev, DL_INFO,
		"%s: Enable VDDI, AVDD, AVEE\n", __func__);

	/* Force part into RESET by holding DDI XRES while powering it up */
	gpio_set_value(pdata->core_pdata->ddi_rst_gpio, 0);

	/* Turn on VDDI [Digital Interface] (+1.8v) */
	ret = sw_gpio_setcfg(en_vddi, 1);
	if (ret)
		pr_err("%s: setcfg for VDDI GPIO %d failed\n",
			__func__, en_vddi);
	ret = gpio_direction_output(en_vddi, 1);
	gpio_free(en_vddi);
	msleep(20);

	/* Turn on AVDD (+5.0v) */
	ret = sw_gpio_setcfg(en_avdd, 1);
	if (ret)
		pr_err("%s: setcfg for AVDD GPIO %d failed\n",
			__func__, en_avdd);
	ret = gpio_direction_output(en_avdd, 1);
	gpio_free(en_avdd);
	msleep(20);

	/* Turn on AVEE (-5.0v) */
	ret = sw_gpio_setcfg(en_avee, 1);
	if (ret)
		pr_err("%s: setcfg for AVEE GPIO %d failed\n",
			__func__, en_avee);
	ret = gpio_direction_output(en_avee, 1);
	gpio_free(en_avee);
	msleep(20);

	/* Force part out of RESET by releasing DDI XRES */
	gpio_set_value(pdata->core_pdata->ddi_rst_gpio, 1);
	/* TODO - determine how long we have to detect PIP2 BL after release */

#else
	/* Call platform init function */
	if (cd->cpdata->init) {
		pt_debug(cd->dev, DL_INFO, "%s: Init HW\n", __func__);
		rc = cd->cpdata->init(cd->cpdata, 1, cd->dev);
	} else {
		pt_debug(cd->dev, DL_INFO, "%s: No HW INIT function\n",
			__func__);
		rc = 0;
	}
	if (rc < 0)
		pt_debug(cd->dev, DL_ERROR, "%s: HW Init fail r=%d\n",
			__func__, rc);
#endif /* --- End PT_PTSBC_SUPPORT --- */

#ifdef TTDL_DIAGNOSTICS
	/* Initialize error counters */
	cd->watchdog_irq_stuck_count     = 0;
	cd->i2c_transmission_error_count = 0;
#endif

	/* Call platform detect function */
	if (cd->cpdata->detect) {
		pt_debug(cd->dev, DL_INFO, "%s: Detect HW\n", __func__);
		rc = cd->cpdata->detect(cd->cpdata, cd->dev,
				cyttsp5_platform_detect_read);
		if (rc) {
			pt_debug(cd->dev, DL_INFO, "%s: No HW detected\n",
					__func__);
			rc = -ENODEV;
			goto error_detect;
		}
	} else
		pt_debug(dev, DL_ERROR,
				"%s: PARADE No HW detect function pointer\n",
				__func__);

	rc = cyttsp5_setup_irq_gpio(cd);
	if (rc < 0) {
		pt_debug(dev, DL_ERROR, "%s: Error, could not setup IRQ\n",
				__func__);
		goto error_setup_irq;
	}

	/* Setup watchdog timer */
	setup_timer(&cd->watchdog_timer, pt_watchdog_timer,
			(unsigned long)cd);
	pt_stop_wd_timer(cd);

#ifdef TTHE_TUNER_SUPPORT
	mutex_init(&cd->tthe_lock);
	cd->tthe_debugfs = debugfs_create_file(CYTTSP5_TTHE_TUNER_FILE_NAME,
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
	     == CY_IRQ_ASSERTED_VALUE) {
		pt_debug(dev, DL_WARN, "%s: IRQ asserted, clear buffer...\n",
			__func__);
		i2c_master_recv(to_i2c_client(cd->dev), buf, 2);
		i2c_master_recv(to_i2c_client(cd->dev), buf,
		2 + get_unaligned_le16(&buf[0]));
		pt_debug(dev, DL_INFO,
			"%s: Buffer cleared, IRQ deasserted\n", __func__);
	}

	/* Detect what PIP interface version the DUT is communicating in */
	pt_debug(dev, DL_INFO, "%s: Detect PIP Interface Version...\n",
		__func__);

	_pt_request_active_pip_protocol(cd->dev, &pip_ver_major,
		&pip_ver_minor);
	if (pip_ver_major == 2) {
		cd->hw_detected = true;
		cd->bl_pip_version_major = pip_ver_major;
		cd->bl_pip_version_minor = pip_ver_minor;
		/* use dev_err so this will print to serial console at pwr up */
		dev_err(dev, " ==================================\n");
		dev_err(dev, " ======== PIP2.%02X Detected ========\n",
			pip_ver_minor);
		dev_err(dev, " ==================================\n");
		/*
		 * If the TC3300 comes up with a PIP2.x protocol this
		 * means either there was no valid FW in the FLASH part, or
		 * this probe occurred before the BL started executing FW.
		 * In both cases the DUT is not ready for pt_startup()
		 * so delay to see if FW comes up.
		 */
		goto create_sysfs_nodes;
	} else if (pip_ver_major == 1) {
		cd->hw_detected = true;
		cd->bl_pip_version_major = pip_ver_major;
		cd->bl_pip_version_minor = pip_ver_minor;
		dev_err(dev, " ======== PIP%d.%02X Detected ========\n",
			pip_ver_major, pip_ver_minor);
	} else {
		cd->hw_detected = false;
		dev_err(dev, " ======== DUT Not Detected  ========\n");
		goto create_sysfs_nodes;
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
	/*
	 * Only create the DUT dependent sysfs nodes if HW was detected
	 * (flag set in get HID desc)
	 */
	if (cd->hw_detected || cd->pip_2p0_prot_active) {
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
	rc = cyttsp5_mt_probe(dev);
	if (rc < 0) {
		pt_debug(dev, DL_ERROR, "%s: Error, fail mt probe\n",
			__func__);
		goto error_after_sysfs_create;
	}

	rc = cyttsp5_btn_probe(dev);
	if (rc < 0) {
		pt_debug(dev, DL_ERROR, "%s: Error, fail btn probe\n",
			__func__);
		goto error_after_startup_mt;
	}
	pt_start_wd_timer(cd);
#endif
	/* Probe registered modules */
	cyttsp5_probe_modules(cd);

#ifdef CONFIG_HAS_EARLYSUSPEND
	cyttsp5_setup_early_suspend(cd);
#elif defined(CONFIG_FB)
	cyttsp5_setup_fb_notifier(cd);
#endif

#if NEED_SUSPEND_NOTIFIER
	cd->pm_notifier.notifier_call = cyttsp5_pm_notifier;
	register_pm_notifier(&cd->pm_notifier);
#endif
	cd->debug_level = current_dl;
	return 0;


#ifndef PT_PTSBC_SUPPORT
error_after_startup_mt:
	pr_err("%s PARADE error_after_startup_mt\n", __func__);
	cyttsp5_mt_release(dev);
error_after_sysfs_create:
	pr_err("%s PARADE error_after_sysfs_create\n", __func__);
	pm_runtime_disable(dev);
#if (KERNEL_VERSION(3, 16, 0) > LINUX_VERSION_CODE)
	device_wakeup_disable(dev);
#endif
	device_init_wakeup(dev, 0);
	cancel_work_sync(&cd->startup_work);
	pt_stop_wd_timer(cd);
	cyttsp5_free_si_ptrs(cd);
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
		cd->cpdata->init(cd->cpdata, 0, dev);
	cyttsp5_del_core(dev);
	dev_set_drvdata(dev, NULL);
	kfree(cd);
#ifndef PT_PTSBC_SUPPORT
error_alloc_data:
error_no_pdata:
#endif
	pt_debug(dev, DL_ERROR, "%s failed.\n", __func__);
	cd->debug_level = current_dl;
	return rc;
}
EXPORT_SYMBOL_GPL(pt_probe);

/*******************************************************************************
 * FUNCTION: cyttsp5_release
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
int cyttsp5_release(struct cyttsp5_core_data *cd)
{
	struct device *dev = cd->dev;

	/* Release successfully probed modules */
	cyttsp5_release_modules(cd);

	cyttsp5_proximity_release(dev);
	cyttsp5_btn_release(dev);
	cyttsp5_mt_release(dev);

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
	remove_sysfs_interfaces(dev);
	free_irq(cd->irq, cd);
	if (cd->cpdata->init)
		cd->cpdata->init(cd->cpdata, 0, dev);
	dev_set_drvdata(dev, NULL);
	cyttsp5_del_core(dev);
	cyttsp5_free_si_ptrs(cd);
	cyttsp5_free_hid_reports(cd);
	kfree(cd);
	return 0;
}
EXPORT_SYMBOL_GPL(cyttsp5_release);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Parade TrueTouch(R) Standard Product Core Driver");
MODULE_AUTHOR("Parade Technologies <ttdrivers@paradetech.com>");
