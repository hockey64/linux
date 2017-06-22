/*
 * cyttsp5_i2c.c
 * Parade TrueTouch(TM) Standard Product V5 I2C Module.
 * For use with Parade touchscreen controllers.
 * Supported parts include:
 * CYTMA5XX
 * CYTMA448
 * CYTMA445A
 * CYTT21XXX
 * CYTT31XXX
 *
 * Copyright (C) 2017 Parade Technologies
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

#include <linux/i2c.h>
#include <linux/version.h>

#define CY_I2C_DATA_SIZE  (2 * 256)
#define _base(x) ((x >= '0' && x <= '9') ? '0' : \
	(x >= 'a' && x <= 'f') ? 'a' - 10 : \
	(x >= 'A' && x <= 'F') ? 'A' - 10 : \
	'\255')
#define HEXOF(x) (x - _base(x))

#ifdef TTDL_DIAGNOSTICS
static unsigned char pt_dut_cmd_buf[300];
static unsigned char pt_dut_out_buf[300];
static int pt_dut_cmd_len;
static int pt_dut_out_len;

/*******************************************************************************
 * FUNCTION: virt_i2c_transfer
 *
 * SUMMARY: Copies the current i2c output message to the temporary buffer
 *	used by the dut_cmd sysfs node
 *
 * RETURN VALUE:
 *	Number of messages transfered which in this function will be 1
 *
 * PARAMETERS:
 *      *buf - pointer to i2c command
 *	 len - length of command in the buffer
 ******************************************************************************/
static int virt_i2c_transfer(u8 *buf, int len)
{
	if (len <= sizeof(pt_dut_cmd_buf)) {
		memcpy(pt_dut_cmd_buf, buf, len);
		pt_dut_cmd_len = len;
		return 1;
	} else
		return 0;
}

/*******************************************************************************
 * FUNCTION: virt_i2c_master_recv
 *
 * SUMMARY: Copies the i2c input message from the dut_out sysfs node into a
 *	temporary buffer. If the entire message is read then clear the buffer
 *	length varible to indicate the contents have been consumed. TTDL will
 *	perform a double read on a buffer, first reading the 2 byte length
 *	followed by reading the entire packet based on that length.
 *
 * RETURN VALUE:
 *	Length of data transfered
 *
 * PARAMETERS:
 *	*dev - pointer to device struct
 *      *buf - pointer to i2c incoming report
 ******************************************************************************/
static int virt_i2c_master_recv(struct device *dev, u8 *buf)
{
	int ret = pt_dut_out_len;

	pt_debug(dev, DL_INFO,
		"%s: Copy msg from dut_out to i2c buffer, len=%d\n",
		__func__, pt_dut_out_len);

	memcpy(buf, pt_dut_out_buf, pt_dut_out_len + 1);

	/* After copying msg into buf clear length ready for next msg */
	pt_dut_out_len = 0;

	/* Return original pt_dut_out_len */
	return ret;
}

/*******************************************************************************
 * FUNCTION: pt_dut_cmd_show
 *
 * SUMMARY: The show function for the dut_cmd sysfs node. Provides read access
 *	to the pt_dut_cmd_buf and clears it after it has been read.
 *
 * RETURN VALUE:
 *	Number of bytes transfered
 *
 * PARAMETERS:
 *      *dev  - pointer to device structure
 *      *attr - pointer to device attributes
 *	*buf  - pointer to output buffer
 ******************************************************************************/
static ssize_t pt_dut_cmd_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int i;
	int index = 0;

	/* Only print to sysfs if the buffer has data */
	if (pt_dut_cmd_len > 0) {
		for (i = 0; i < pt_dut_cmd_len; i++)
			index += sprintf(buf + index, "%02X",
				pt_dut_cmd_buf[i]);
		index += sprintf(buf + index, "\n");
	}
	pt_dut_cmd_len = 0;
	return index;
}
static DEVICE_ATTR(dut_cmd, S_IRUGO, pt_dut_cmd_show, NULL);

/*******************************************************************************
 * FUNCTION: pt_dut_out_store
 *
 * SUMMARY: The store function for the dut_out sysfs node. Provides write
 *	access to the pt_dut_out_buf. The smallest valid PIP response is 2
 *	bytes so don't update buffer if only 1 byte passed in.
 *
 * RETURN VALUE:
 *	Number of bytes read from virtual DUT
 *
 * PARAMETERS:
 *	*dev  - pointer to device structure
 *	*attr - pointer to device attributes
 *	*buf  - pointer to buffer that hold the command parameters
 *	 size - size of buf
 ******************************************************************************/
static ssize_t pt_dut_out_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int loop_max = ARRAY_SIZE(pt_dut_out_buf);
	int hex_str_len = strlen(buf)/2;
	int i;
	const char *pos = buf;

	/* Only update the dut_out buffer if at least 2 byte payload */
	pt_debug(dev, DL_INFO,
		"%s: Virtual DUT Response written to TTDL, len = %d\n",
		__func__, hex_str_len);

	if (size >= 2 && hex_str_len <= loop_max) {
		/* Convert string of hex values to byte array */
		for (i = 0; i < hex_str_len; i++) {
			pt_dut_out_buf[i] = ((HEXOF(*pos)) << 4) +
					     HEXOF(*(pos + 1));
			pos += 2;
		}
		pt_dut_out_len = hex_str_len;
	}
	return size;
}
static DEVICE_ATTR(dut_out, S_IWUSR, NULL, pt_dut_out_store);
#endif  /* TTDL_DIAGNOSTICS */

/*******************************************************************************
 * FUNCTION: cyttsp5_i2c_read_default
 *
 * SUMMARY: Read a certain number of bytes from the I2C bus
 *
 * PARAMETERS:
 *      *dev  - pointer to Device structure
 *      *buf  - pointer to buffer where the data read will be stored
 *       size - size to be read
 ******************************************************************************/
static int cyttsp5_i2c_read_default(struct device *dev, void *buf, int size)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);
	struct i2c_client *client = to_i2c_client(dev);
	int rc;

	if (!buf || !size || size > CY_I2C_DATA_SIZE)
		return -EINVAL;

#ifdef TTDL_DIAGNOSTICS
	if (cd->route_i2c_virt_dut) {
		size = pt_dut_out_len;
		rc = virt_i2c_master_recv(dev, buf);
	} else
		rc = i2c_master_recv(client, buf, size);
#else
	rc = i2c_master_recv(client, buf, size);
#endif

	return (rc < 0) ? rc : rc != size ? -EIO : 0;
}

/*******************************************************************************
 * FUNCTION: cyttsp5_i2c_read_default_nosize
 *
 * SUMMARY: Read from the I2C bus in two transactions first reading the HID
 *	packet size (2 bytes) followed by reading the rest of the packet based
 *	on the size initially read.
 *	NOTE: The empty buffer 'size' was redefined in PIP version 1.7.
 *
 * PARAMETERS:
 *      *dev  - pointer to Device structure
 *      *buf  - pointer to buffer where the data read will be stored
 *       max  - max size that can be read
 ******************************************************************************/
static int cyttsp5_i2c_read_default_nosize(struct device *dev, u8 *buf, u32 max)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);
	struct i2c_msg msgs[2];
	u8 msg_count = 1;
	int rc;
	u32 size;

#ifdef TTDL_DIAGNOSTICS
	if (cd->route_i2c_virt_dut) {
		size = pt_dut_out_len;
		goto skip_read_len;
	}
#endif

	if (!buf)
		return -EINVAL;

	msgs[0].addr = client->addr;
	msgs[0].flags = (client->flags & I2C_M_TEN) | I2C_M_RD;
	msgs[0].len = 2;
	msgs[0].buf = buf;
	rc = i2c_transfer(client->adapter, msgs, msg_count);
	if (rc < 0 || rc != msg_count)
		return (rc < 0) ? rc : -EIO;

	size = get_unaligned_le16(&buf[0]);
	/* Length field in PIP2.0 does not include itself */
	if (cd->pip_2p0_prot_active)
		size += 2;
	if (!size || size == 2 || size >= CY_PIP_1P7_EMPTY_BUF)
		/*
		 * Before PIP 1.7, empty buffer is 0x0002;
		 * From PIP 1.7, empty buffer is 0xFFXX
		 */
		return 0;

	if (size > max)
		return -EINVAL;

skip_read_len:
#ifdef TTDL_DIAGNOSTICS
	if (cd->route_i2c_virt_dut)
		rc = virt_i2c_master_recv(dev, buf);
	else
		rc = i2c_master_recv(client, buf, size);
#else
	rc = i2c_master_recv(client, buf, size);
#endif

	pt_debug(dev, DL_INFO, "%s: rc = %d\n", __func__, rc);
	return (rc < 0) ? rc : rc != (int)size ? -EIO : 0;
}

/*******************************************************************************
 * FUNCTION: cyttsp5_i2c_write_read_specific
 *
 * SUMMARY: Write the contents of write_buf to the I2C device and then read
 *	the response using cyttsp5_i2c_read_default_nosize()
 *
 * PARAMETERS:
 *      *dev       - pointer to Device structure
 *	 write_len - length of data buffer write_buf
 *      *write_buf - pointer to buffer to write
 *      *read_buf  - pointer to buffer to read response into
 ******************************************************************************/
static int cyttsp5_i2c_write_read_specific(struct device *dev, u8 write_len,
		u8 *write_buf, u8 *read_buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);
	struct i2c_msg msgs[2];
	u8 msg_count = 1;
	int rc;

	if (!write_buf || !write_len) {
		if (!write_buf)
			pt_debug(dev, DL_ERROR,
				"%s write_buf is NULL", __func__);
		if (!write_len)
			pt_debug(dev, DL_ERROR,
				"%s write_len is NULL", __func__);
		return -EINVAL;

	}

	msgs[0].addr = client->addr;
	msgs[0].flags = client->flags & I2C_M_TEN;
	msgs[0].len = write_len;
	msgs[0].buf = write_buf;
#ifdef TTDL_DIAGNOSTICS
	if (cd->route_i2c_virt_dut) {
		rc = virt_i2c_transfer(msgs[0].buf, msgs[0].len);
		pt_debug(dev, DL_DEBUG, "%s: Virt transfer size = %d",
			__func__, msgs[0].len);
	} else
		rc = i2c_transfer(client->adapter, msgs, msg_count);
#else
	rc = i2c_transfer(client->adapter, msgs, msg_count);
#endif

	if (rc < 0 || rc != msg_count)
		return (rc < 0) ? rc : -EIO;

	rc = 0;

	if (read_buf) {
		rc = cyttsp5_i2c_read_default_nosize(dev, read_buf,
				CY_I2C_DATA_SIZE);
	}

	return rc;
}

static struct cyttsp5_bus_ops cyttsp5_i2c_bus_ops = {
	.bustype = BUS_I2C,
	.read_default = cyttsp5_i2c_read_default,
	.read_default_nosize = cyttsp5_i2c_read_default_nosize,
	.write_read_specific = cyttsp5_i2c_write_read_specific,
};

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_DEVICETREE_SUPPORT
static const struct of_device_id cyttsp5_i2c_of_match[] = {
	{ .compatible = "cy,cyttsp5_i2c_adapter", },
	{ }
};
MODULE_DEVICE_TABLE(of, cyttsp5_i2c_of_match);
#endif


/*******************************************************************************
 * FUNCTION: cyttsp5_i2c_probe
 *
 * SUMMARY: Probe functon for the I2C module
 *
 * PARAMETERS:
 *      *client - pointer to i2c client structure
 *      *i2c_id - pointer to i2c device structure
 ******************************************************************************/
static int cyttsp5_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *i2c_id)
{
	struct device *dev = &client->dev;
#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_DEVICETREE_SUPPORT
	const struct of_device_id *match;
#endif
	int rc;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pt_debug(dev, DL_ERROR, "I2C functionality not Supported\n");
		return -EIO;
	}

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_DEVICETREE_SUPPORT
	match = of_match_device(of_match_ptr(cyttsp5_i2c_of_match), dev);
	if (match) {
		rc = cyttsp5_devtree_create_and_get_pdata(dev);
		if (rc < 0)
			return rc;
	}
#endif

	rc = pt_probe(&cyttsp5_i2c_bus_ops, &client->dev, client->irq,
			  CY_I2C_DATA_SIZE);

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_DEVICETREE_SUPPORT
	if (rc && match)
		cyttsp5_devtree_clean_pdata(dev);
#endif
#ifdef TTDL_DIAGNOSTICS
	device_create_file(dev, &dev_attr_dut_cmd);
	device_create_file(dev, &dev_attr_dut_out);
#endif

	return rc;
}

/*******************************************************************************
 * FUNCTION: cyttsp5_i2c_remove
 *
 * SUMMARY: Remove functon for the I2C module
 *
 * PARAMETERS:
 *      *client - pointer to i2c client structure
 ******************************************************************************/
static int cyttsp5_i2c_remove(struct i2c_client *client)
{
#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_DEVICETREE_SUPPORT
	struct device *dev = &client->dev;
	const struct of_device_id *match;
#endif
	struct cyttsp5_core_data *cd = i2c_get_clientdata(client);

	cyttsp5_release(cd);

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_DEVICETREE_SUPPORT
	match = of_match_device(of_match_ptr(cyttsp5_i2c_of_match), dev);
	if (match)
		cyttsp5_devtree_clean_pdata(dev);
#endif

	return 0;
}

static const struct i2c_device_id cyttsp5_i2c_id[] = {
	{ CYTTSP5_I2C_NAME, 0, },
	{ }
};
MODULE_DEVICE_TABLE(i2c, cyttsp5_i2c_id);

static struct i2c_driver cyttsp5_i2c_driver = {
	.driver = {
		.name = CYTTSP5_I2C_NAME,
		.owner = THIS_MODULE,
		.pm = &cyttsp5_pm_ops,
#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_DEVICETREE_SUPPORT
		.of_match_table = cyttsp5_i2c_of_match,
#endif
	},
	.probe = cyttsp5_i2c_probe,
	.remove = cyttsp5_i2c_remove,
	.id_table = cyttsp5_i2c_id,
};

#if (KERNEL_VERSION(3, 3, 0) <= LINUX_VERSION_CODE)
module_i2c_driver(cyttsp5_i2c_driver);
#else
static int __init cyttsp5_i2c_init(void)
{
	int rc = i2c_add_driver(&cyttsp5_i2c_driver);

	pr_info("%s: Parade TTSP I2C Driver (Built %s) rc=%d\n",
			__func__, CY_DRIVER_VERSION, rc);
	return rc;
}
module_init(cyttsp5_i2c_init);

static void __exit cyttsp5_i2c_exit(void)
{
	i2c_del_driver(&cyttsp5_i2c_driver);
}
module_exit(cyttsp5_i2c_exit);
#endif

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Parade TrueTouch(R) Standard Product I2C driver");
MODULE_AUTHOR("Parade Technologies <ttdrivers@paradetech.com>");
