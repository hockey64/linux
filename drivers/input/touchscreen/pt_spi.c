/*
 * pt_spi.c
 * Parade TrueTouch(TM) Standard Product V5 SPI Module.
 * For use with Parade touchscreen controllers.
 * Supported parts include:
 * CYTMA5XX
 * CYTMA448
 * CYTMA445A
 * CYTT21XXX
 * CYTT31XXX
 *
 * Copyright (C) 2015-2017 Parade Technologies
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

#include "pt_regs.h"

#include <linux/spi/spi.h>
#include <linux/version.h>

#define PT_SPI_WR_OP		0x00 /* r/~w */
#define PT_SPI_RD_OP		0x01
#define PT_SPI_BITS_PER_WORD	8
#define PT_SPI_SYNC_ACK         0x62

#define PT_SPI_CMD_BYTES	0
#define PT_SPI_DATA_SIZE	(2 * 256)
#define PT_SPI_DATA_BUF_SIZE	(PT_SPI_CMD_BYTES + PT_SPI_DATA_SIZE)

/*******************************************************************************
 * FUNCTION: pt_spi_add_rw_msg
 *
 * SUMMARY: Add spi message to work queue for reading or writing data.
 *
 * PARAMETERS:
 *      *msg - pointer to spi_message structure
 *     *xfer - pointer to spi_transfer structure
 * *w_header - pointer to write buffer
 * *r_header - pointer to read buffer
 *        op - flag to write or read data
 ******************************************************************************/
static void pt_spi_add_rw_msg(struct spi_message *msg,
		struct spi_transfer *xfer, u8 *w_header, u8 *r_header, u8 op)
{
	xfer->tx_buf = w_header;
	xfer->rx_buf = r_header;
	w_header[0] = op;
	xfer->len = 1;
	spi_message_add_tail(xfer, msg);
}

/*******************************************************************************
 * FUNCTION: pt_spi_xfer
 *
 * SUMMARY: Read or write date for SPI device.
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *      *dev - pointer to device structure
 *        op - flag to write or read data
 *      *buf - pointer to data buffer
 *    length - data length
 ******************************************************************************/
static int pt_spi_xfer(struct device *dev, u8 op, u8 *buf, int length)
{
	struct spi_device *spi = to_spi_device(dev);
	struct spi_message msg;
	struct spi_transfer xfer[2];
	u8 w_header[2];
	u8 r_header[2];
	int rc;

	memset(xfer, 0, sizeof(xfer));

	spi_message_init(&msg);
	pt_spi_add_rw_msg(&msg, &xfer[0], w_header, r_header, op);

	switch (op) {
	case PT_SPI_RD_OP:
		xfer[1].rx_buf = buf;
		xfer[1].len = length;
		spi_message_add_tail(&xfer[1], &msg);
		break;
	case PT_SPI_WR_OP:
		xfer[1].tx_buf = buf;
		xfer[1].len = length;
		spi_message_add_tail(&xfer[1], &msg);
		break;
	default:
		rc = -EIO;
		goto exit;
	}

	rc = spi_sync(spi, &msg);
exit:
	if (rc < 0)
		pt_debug(dev, DL_INFO, "%s: spi_sync() error %d\n",
			__func__, rc);

	if (r_header[0] != PT_SPI_SYNC_ACK)
		return -EIO;

	return rc;
}

/*******************************************************************************
 * FUNCTION: pt_spi_read_default
 *
 * SUMMARY: Read a certain number of bytes from the SPI bus
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *      *dev  - pointer to Device structure
 *      *buf  - pointer to buffer where the data read will be stored
 *       size - size to be read
 ******************************************************************************/
static int pt_spi_read_default(struct device *dev, void *buf, int size)
{
	if (!buf || !size)
		return 0;

	return pt_spi_xfer(dev, PT_SPI_RD_OP, buf, size);
}

/*******************************************************************************
 * FUNCTION: pt_spi_read_default_nosize
 *
 * SUMMARY: Read from the SPI bus in two transactions first reading the HID
 *	packet size (2 bytes) followed by reading the rest of the packet based
 *	on the size initially read.
 *	NOTE: The empty buffer 'size' was redefined in PIP version 1.7.
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *      *dev  - pointer to Device structure
 *      *buf  - pointer to buffer where the data read will be stored
 *       max  - max size that can be read
 ******************************************************************************/
static int pt_spi_read_default_nosize(struct device *dev, u8 *buf, u32 max)
{
	u32 size;
	int rc;

	if (!buf)
		return 0;

	rc = pt_spi_xfer(dev, PT_SPI_RD_OP, buf, 2);
	if (rc < 0)
		return rc;

	size = get_unaligned_le16(&buf[0]);
	if (!size)
		return rc;

	if (size > max)
		return -EINVAL;

	return pt_spi_read_default(dev, buf, size);
}

/*******************************************************************************
 * FUNCTION: pt_spi_write_read_specific
 *
 * SUMMARY: Write the contents of write_buf to the SPI device and then read
 *	the response using pt_spi_read_default_nosize()
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *      *dev       - pointer to Device structure
 *       write_len - length of data buffer write_buf
 *      *write_buf - pointer to buffer to write
 *      *read_buf  - pointer to buffer to read response into
 ******************************************************************************/
static int pt_spi_write_read_specific(struct device *dev, u8 write_len,
		u8 *write_buf, u8 *read_buf)
{
	int rc;

	rc = pt_spi_xfer(dev, PT_SPI_WR_OP, write_buf, write_len);
	if (rc < 0)
		return rc;

	if (read_buf)
		rc = pt_spi_read_default_nosize(dev, read_buf,
				PT_SPI_DATA_SIZE);

	return rc;
}

static struct pt_bus_ops pt_spi_bus_ops = {
	.bustype = BUS_SPI,
	.read_default = pt_spi_read_default,
	.read_default_nosize = pt_spi_read_default_nosize,
	.write_read_specific = pt_spi_write_read_specific,
};

#ifdef CONFIG_TOUCHSCREEN_PARADE_DEVICETREE_SUPPORT
static const struct of_device_id pt_spi_of_match[] = {
	{ .compatible = "pt,pt_spi_adapter", },
	{ }
};
MODULE_DEVICE_TABLE(of, pt_spi_of_match);
#endif

/*******************************************************************************
 * FUNCTION: pt_spi_probe
 *
 * SUMMARY: Probe functon for the SPI module
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *      *spi - pointer to spi device structure
 ******************************************************************************/
static int pt_spi_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
#ifdef CONFIG_TOUCHSCREEN_PARADE_DEVICETREE_SUPPORT
	const struct of_device_id *match;
#endif
	int rc;

	/* Set up SPI*/
	spi->bits_per_word = PT_SPI_BITS_PER_WORD;
	spi->mode = SPI_MODE_0;
	rc = spi_setup(spi);
	if (rc < 0) {
		pt_debug(dev, DL_ERROR, "%s: SPI setup error %d\n",
			__func__, rc);
		return rc;
	}

#ifdef CONFIG_TOUCHSCREEN_PARADE_DEVICETREE_SUPPORT
	match = of_match_device(of_match_ptr(pt_spi_of_match), dev);
	if (match) {
		rc = pt_devtree_create_and_get_pdata(dev);
		if (rc < 0)
			return rc;
	}
#endif

	rc = pt_probe(&pt_spi_bus_ops, &spi->dev, spi->irq,
			  PT_SPI_DATA_BUF_SIZE);

#ifdef CONFIG_TOUCHSCREEN_PARADE_DEVICETREE_SUPPORT
	if (rc && match)
		pt_devtree_clean_pdata(dev);
#endif

	return rc;
}

/*******************************************************************************
 * FUNCTION: pt_spi_remove
 *
 * SUMMARY: Remove functon for the SPI module
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *      *spi - pointer to spi device structure
 ******************************************************************************/
static int pt_spi_remove(struct spi_device *spi)
{
#ifdef CONFIG_TOUCHSCREEN_PARADE_DEVICETREE_SUPPORT
	struct device *dev = &spi->dev;
	const struct of_device_id *match;
#endif
	struct pt_core_data *cd = dev_get_drvdata(&spi->dev);

	pt_release(cd);

#ifdef CONFIG_TOUCHSCREEN_PARADE_DEVICETREE_SUPPORT
	match = of_match_device(of_match_ptr(pt_spi_of_match), dev);
	if (match)
		pt_devtree_clean_pdata(dev);
#endif

	return 0;
}

static const struct spi_device_id pt_spi_id[] = {
	{ PT_SPI_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, pt_spi_id);

static struct spi_driver pt_spi_driver = {
	.driver = {
		.name = PT_SPI_NAME,
		.bus = &spi_bus_type,
		.owner = THIS_MODULE,
		.pm = &pt_pm_ops,
#ifdef CONFIG_TOUCHSCREEN_PARADE_DEVICETREE_SUPPORT
		.of_match_table = pt_spi_of_match,
#endif
	},
	.probe = pt_spi_probe,
	.remove = (pt_spi_remove),
	.id_table = pt_spi_id,
};

#if (KERNEL_VERSION(3, 3, 0) <= LINUX_VERSION_CODE)
module_spi_driver(pt_spi_driver);
#else
/*******************************************************************************
 * FUNCTION: pt_spi_init
 *
 * SUMMARY: Initialize function to register spi module to kernel.
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 ******************************************************************************/
static int __init pt_spi_init(void)
{
	int err = spi_register_driver(&pt_spi_driver);

	pr_info("%s: Parade TTSP SPI Driver (Built %s) rc=%d\n",
		 __func__, PT_DRIVER_VERSION, err);
	return err;
}
module_init(pt_spi_init);

/*******************************************************************************
 * FUNCTION: pt_spi_exit
 *
 * SUMMARY: Exit function to unregister spi module from kernel.
 *
 ******************************************************************************/
static void __exit pt_spi_exit(void)
{
	spi_unregister_driver(&pt_spi_driver);
}
module_exit(pt_spi_exit);
#endif

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Parade TrueTouch(R) Standard Product SPI Driver");
MODULE_AUTHOR("Parade Technologies <ttdrivers@paradetech.com>");
