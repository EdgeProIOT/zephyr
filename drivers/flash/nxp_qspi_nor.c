/*
 * Copyright (c) 2020 Jernej Turnsek
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nxp_qspi_nor

#include <zephyr.h>
#include <kernel.h>
#include <device.h>
#include <string.h>
#include <drivers/flash.h>
#include <errno.h>
#include <init.h>
#include <soc.h>
#include <devicetree.h>

#include "fsl_flexspi.h"

#define FLASH_SECTOR_SIZE 	4096
#define FLASH_PAGE_SIZE 	256

#define CUSTOM_LUT_LENGTH 60
#define FLASH_QUAD_ENABLE 0x40
#define FLASH_BUSY_STATUS_POL 1
#define FLASH_BUSY_STATUS_OFFSET 0
#define FLASH_ERROR_STATUS_MASK 0x0e

#define NOR_CMD_LUT_SEQ_IDX_READ_NORMAL        0
#define NOR_CMD_LUT_SEQ_IDX_READ_FAST          1
#define NOR_CMD_LUT_SEQ_IDX_READ_FAST_QUAD     2
#define NOR_CMD_LUT_SEQ_IDX_READSTATUS         3
#define NOR_CMD_LUT_SEQ_IDX_WRITEENABLE        4
#define NOR_CMD_LUT_SEQ_IDX_ERASESECTOR        5
#define NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM_SINGLE 6
#define NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM_QUAD   7
#define NOR_CMD_LUT_SEQ_IDX_READID             8
#define NOR_CMD_LUT_SEQ_IDX_WRITESTATUSREG     9
#define NOR_CMD_LUT_SEQ_IDX_ENTERQPI           10
#define NOR_CMD_LUT_SEQ_IDX_EXITQPI            11
#define NOR_CMD_LUT_SEQ_IDX_READSTATUSREG      12

const uint32_t customLUT[CUSTOM_LUT_LENGTH] = {
    /* Normal read mode -SDR */
    [4 * NOR_CMD_LUT_SEQ_IDX_READ_NORMAL] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x03, kFLEXSPI_Command_RADDR_SDR, kFLEXSPI_1PAD, 0x18),
    [4 * NOR_CMD_LUT_SEQ_IDX_READ_NORMAL + 1] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_READ_SDR, kFLEXSPI_1PAD, 0x04, kFLEXSPI_Command_STOP, kFLEXSPI_1PAD, 0),

    /* Fast read mode - SDR */
    [4 * NOR_CMD_LUT_SEQ_IDX_READ_FAST] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x0B, kFLEXSPI_Command_RADDR_SDR, kFLEXSPI_1PAD, 0x18),
    [4 * NOR_CMD_LUT_SEQ_IDX_READ_FAST + 1] = FLEXSPI_LUT_SEQ(
        kFLEXSPI_Command_DUMMY_SDR, kFLEXSPI_1PAD, 0x08, kFLEXSPI_Command_READ_SDR, kFLEXSPI_1PAD, 0x04),

    /* Fast read quad mode - SDR */
    [4 * NOR_CMD_LUT_SEQ_IDX_READ_FAST_QUAD] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x6B, kFLEXSPI_Command_RADDR_SDR, kFLEXSPI_1PAD, 0x18),
    [4 * NOR_CMD_LUT_SEQ_IDX_READ_FAST_QUAD + 1] = FLEXSPI_LUT_SEQ(
        kFLEXSPI_Command_DUMMY_SDR, kFLEXSPI_4PAD, 0x08, kFLEXSPI_Command_READ_SDR, kFLEXSPI_4PAD, 0x04),

    /* Read extend parameters */
    [4 * NOR_CMD_LUT_SEQ_IDX_READSTATUS] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x81, kFLEXSPI_Command_READ_SDR, kFLEXSPI_1PAD, 0x04),

    /* Write Enable */
    [4 * NOR_CMD_LUT_SEQ_IDX_WRITEENABLE] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x06, kFLEXSPI_Command_STOP, kFLEXSPI_1PAD, 0),

    /* Erase Sector  */
    [4 * NOR_CMD_LUT_SEQ_IDX_ERASESECTOR] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0xD7, kFLEXSPI_Command_RADDR_SDR, kFLEXSPI_1PAD, 0x18),

    /* Page Program - single mode */
    [4 * NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM_SINGLE] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x02, kFLEXSPI_Command_RADDR_SDR, kFLEXSPI_1PAD, 0x18),
    [4 * NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM_SINGLE + 1] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_WRITE_SDR, kFLEXSPI_1PAD, 0x04, kFLEXSPI_Command_STOP, kFLEXSPI_1PAD, 0),

    /* Page Program - quad mode */
    [4 * NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM_QUAD] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x32, kFLEXSPI_Command_RADDR_SDR, kFLEXSPI_1PAD, 0x18),
    [4 * NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM_QUAD + 1] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_WRITE_SDR, kFLEXSPI_4PAD, 0x04, kFLEXSPI_Command_STOP, kFLEXSPI_1PAD, 0),

    /* Read ID */
    [4 * NOR_CMD_LUT_SEQ_IDX_READID] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x9F, kFLEXSPI_Command_READ_SDR, kFLEXSPI_1PAD, 0x04),

    /* Enable Quad mode */
    [4 * NOR_CMD_LUT_SEQ_IDX_WRITESTATUSREG] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x01, kFLEXSPI_Command_WRITE_SDR, kFLEXSPI_1PAD, 0x04),

    /* Enter QPI mode */
    [4 * NOR_CMD_LUT_SEQ_IDX_ENTERQPI] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x35, kFLEXSPI_Command_STOP, kFLEXSPI_1PAD, 0),

    /* Exit QPI mode */
    [4 * NOR_CMD_LUT_SEQ_IDX_EXITQPI] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_4PAD, 0xF5, kFLEXSPI_Command_STOP, kFLEXSPI_1PAD, 0),

    /* Read status register */
    [4 * NOR_CMD_LUT_SEQ_IDX_READSTATUSREG] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x05, kFLEXSPI_Command_READ_SDR, kFLEXSPI_1PAD, 0x04),
};


flexspi_device_config_t deviceconfig = {
	.flexspiRootClk = DT_INST_PROP(0, spi_max_frequency),
	.flashSize = DT_INST_PROP(0, size) / 1024,
	.CSIntervalUnit = kFLEXSPI_CsIntervalUnit1SckCycle,
	.CSInterval = 2,
	.CSHoldTime = 3,
	.CSSetupTime = 3,
	.dataValidTime = 0,
	.columnspace = 0,
	.enableWordAddress = 0,
	.AWRSeqIndex = 0,
	.AWRSeqNumber = 0,
	.ARDSeqIndex = NOR_CMD_LUT_SEQ_IDX_READ_FAST_QUAD,
	.ARDSeqNumber = 1,
	.AHBWriteWaitUnit = kFLEXSPI_AhbWriteWaitUnit2AhbCycle,
	.AHBWriteWaitInterval = 0,
};

struct flash_priv {
	struct k_sem write_lock;
	FLEXSPI_Type  *pflash_block_base;
};

static const struct flash_parameters
	flash_mcux_flexspi_qspi_flash_parameters = {
	.write_block_size = 4,
	.erase_value = 0xff,
};


static status_t flash_mcux_flexspi_qspi_wait_bus_busy(const struct device *dev,
						      off_t offset)
{
	bool is_busy = true;
	uint32_t read_value;
	status_t status;
	flexspi_transfer_t flash_transfer;

	struct flash_priv *priv = dev->data;
	FLEXSPI_Type *base_address = priv->pflash_block_base;

	flash_transfer.deviceAddress = offset;
	flash_transfer.port = kFLEXSPI_PortA1;
	flash_transfer.cmdType = kFLEXSPI_Read;
	flash_transfer.SeqNumber = 1;
	flash_transfer.seqIndex = NOR_CMD_LUT_SEQ_IDX_READSTATUSREG;
	flash_transfer.data = &read_value;
	flash_transfer.dataSize = 1;

	do {
		status = FLEXSPI_TransferBlocking(base_address, &flash_transfer);
		if (status != kStatus_Success) {
			return status;
		}
		/*
		   Status bit information for IS25LP064.
		   The Write In Progress (WIP) bit is read-only, and can be used to
		   detect the progress or completion of a program or erase operation.
		   When the WIP bit is “0”, the device is ready for write status register,
		   program or erase operation. When the WIP bit is “1”,
		   the device is busy.
		 */
		is_busy = ((read_value & 0x01UL) == 0x01UL) ? true : false;
	} while (is_busy);

	return status;
}

status_t flash_mcux_flexspi_qspi_write_enable(const struct device *dev, off_t offset)
{
	status_t status;
	flexspi_transfer_t flash_transfer;

	struct flash_priv *priv = dev->data;
	FLEXSPI_Type *base_address = priv->pflash_block_base;

	flash_transfer.deviceAddress = offset;
	flash_transfer.port = kFLEXSPI_PortA1;
	flash_transfer.cmdType = kFLEXSPI_Command;
	flash_transfer.SeqNumber = 1;
	flash_transfer.seqIndex = NOR_CMD_LUT_SEQ_IDX_WRITEENABLE;

	status = FLEXSPI_TransferBlocking(base_address, &flash_transfer);

	return status;
}

status_t flash_mcux_flexspi_enable_quad_mode(const struct device *dev, off_t offset)
{
    flexspi_transfer_t flashXfer;
    status_t status;
    uint32_t writeValue = 0x40;

    struct flash_priv *priv = dev->data;
    FLEXSPI_Type *base_address = priv->pflash_block_base;

    /* Write neable */
    status = flash_mcux_flexspi_qspi_write_enable(dev, offset);

    if (status != kStatus_Success) {
        return status;
    }

    /* Enable quad mode. */
    flashXfer.deviceAddress = offset;
    flashXfer.port          = kFLEXSPI_PortA1;
    flashXfer.cmdType       = kFLEXSPI_Write;
    flashXfer.SeqNumber     = 1;
    flashXfer.seqIndex      = NOR_CMD_LUT_SEQ_IDX_WRITESTATUSREG;
    flashXfer.data          = &writeValue;
    flashXfer.dataSize      = 1;

    status = FLEXSPI_TransferBlocking(base_address, &flashXfer);
    if (status != kStatus_Success) {
        return status;
    }

    status = flash_mcux_flexspi_qspi_wait_bus_busy(dev, offset);

    return status;
}

static int flash_mcux_flexspi_qspi_read(const struct device *dev, off_t offset,
					void *data, size_t len)
{
	status_t status;
	flexspi_transfer_t flash_transfer;

	struct flash_priv *priv = dev->data;
	FLEXSPI_Type *base_address = priv->pflash_block_base;

	flash_transfer.deviceAddress = offset;
	flash_transfer.port = kFLEXSPI_PortA1;
	flash_transfer.cmdType = kFLEXSPI_Read;
	flash_transfer.SeqNumber = 1;
	flash_transfer.seqIndex = NOR_CMD_LUT_SEQ_IDX_READ_FAST_QUAD;
	flash_transfer.data = (uint32_t *)data;
	flash_transfer.dataSize = len;

	status = FLEXSPI_TransferBlocking(base_address, &flash_transfer);
	if (status == kStatus_Success) {
		status = flash_mcux_flexspi_qspi_wait_bus_busy(dev, offset);
	}

	return status;
}

static int flash_mcux_flexspi_qspi_write(const struct device *dev, off_t offset,
					 const void *data, size_t len)
{
	status_t status = kStatus_Fail;
	flexspi_transfer_t flash_transfer;

	int pages = len / FLASH_PAGE_SIZE;
	struct flash_priv *priv = dev->data;
	FLEXSPI_Type *base_address = priv->pflash_block_base;

	if (k_sem_take(&priv->write_lock, K_NO_WAIT)) {
		return -EACCES;
	}

	if (len % FLASH_PAGE_SIZE) {
		pages++;
	}

	for (int i = 0; i < pages; i++) {
		off_t offset_to_sector = offset + i * FLASH_PAGE_SIZE;
		uint32_t *offset_to_data =
			(uint32_t *)((uint8_t *)data + i * FLASH_PAGE_SIZE);
		uint32_t bytes_to_write =
			len - i * FLASH_PAGE_SIZE >= FLASH_PAGE_SIZE ?
			FLASH_PAGE_SIZE : len - i * FLASH_PAGE_SIZE;

		status = flash_mcux_flexspi_qspi_write_enable(dev, offset);
		if (status == kStatus_Success) {
			flash_transfer.deviceAddress = offset_to_sector;
			flash_transfer.port = kFLEXSPI_PortA1;
			flash_transfer.cmdType = kFLEXSPI_Write;
			flash_transfer.SeqNumber = 1;
			flash_transfer.seqIndex = NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM_QUAD;
			flash_transfer.data = offset_to_data,
			flash_transfer.dataSize = bytes_to_write;

			status = FLEXSPI_TransferBlocking(base_address, &flash_transfer);
			if (status == kStatus_Success) {
				status = flash_mcux_flexspi_qspi_wait_bus_busy(dev, offset);
			}
		}
	}

	k_sem_give(&priv->write_lock);

	return status;
}

static int flash_mcux_flexspi_qspi_erase(const struct device *dev,
					 off_t offset, size_t len)
{
	status_t status = kStatus_Fail;
	flexspi_transfer_t flash_transfer;

	struct flash_priv *priv = dev->data;
	FLEXSPI_Type *base_address = priv->pflash_block_base;

	if (k_sem_take(&priv->write_lock, K_NO_WAIT)) {
		return -EACCES;
	}

	int sectors = len / FLASH_SECTOR_SIZE;
	if (len % FLASH_SECTOR_SIZE) {
		sectors++;
	}

	for (int i = 0; i < sectors; i++) {
		off_t offset_to_sector =
			offset + i * FLASH_SECTOR_SIZE;

		status = flash_mcux_flexspi_qspi_write_enable(dev, offset);
		if (status == kStatus_Success) {
			flash_transfer.deviceAddress = offset_to_sector;
			flash_transfer.port = kFLEXSPI_PortA1;
			flash_transfer.cmdType = kFLEXSPI_Command;
			flash_transfer.SeqNumber = 1;
			flash_transfer.seqIndex = NOR_CMD_LUT_SEQ_IDX_ERASESECTOR;

			status = FLEXSPI_TransferBlocking(base_address, &flash_transfer);
			if (status == kStatus_Success) {
				status = flash_mcux_flexspi_qspi_wait_bus_busy(dev, offset);
			}
		}
	}

	k_sem_give(&priv->write_lock);

	return status;
}

/* Write/erase operations in this driver is protected by a semaphore. This
 * prevents access from multiple threads, but using this function the semaphore
 * can be locked, preventing all write/erase operations. */
static int flash_mcux_flexspi_qspi_write_protection(const struct device *dev,
						    bool enable)
{
	struct flash_priv *priv = dev->data;
	int rc = 0;

	if (enable) {
		rc = k_sem_take(&priv->write_lock, K_FOREVER);
	} else {
		k_sem_give(&priv->write_lock);
	}

	return rc;
}

#if defined(CONFIG_FLASH_PAGE_LAYOUT)
static const struct flash_pages_layout dev_layout = {
	.pages_count = KB(CONFIG_FLASH_SIZE) / FLASH_SECTOR_SIZE,
	.pages_size = FLASH_SECTOR_SIZE,
};

static void flash_mcux_flexspi_qspi_pages_layout(const struct device *dev,
						 const struct flash_pages_layout **layout,
						 size_t *layout_size)
{
	*layout = &dev_layout;
	*layout_size = 1;
}
#endif /* CONFIG_FLASH_PAGE_LAYOUT */

static const struct flash_parameters *
flash_mcux_flexspi_qspi_get_parameters(const struct device *dev)
{
	ARG_UNUSED(dev);

	return &flash_mcux_flexspi_qspi_flash_parameters;
}

static struct flash_priv flash_data;

static const struct flash_driver_api flash_mcux_flexspi_qspi_api = {
	.write_protection = flash_mcux_flexspi_qspi_write_protection,
	.erase = flash_mcux_flexspi_qspi_erase,
	.write = flash_mcux_flexspi_qspi_write,
	.read = flash_mcux_flexspi_qspi_read,
	.get_parameters = flash_mcux_flexspi_qspi_get_parameters,
#if defined(CONFIG_FLASH_PAGE_LAYOUT)
	.page_layout = flash_mcux_flexspi_qspi_pages_layout,
#endif
};

static int flash_mcux_flexspi_qspi_init(const struct device *dev)
{
	/*
	   Get the parent node of the chosen flash chip to get
	   the reg address of the FlexSPI device
	 */
	FLEXSPI_Type *base =
		(FLEXSPI_Type *)DT_REG_ADDR_BY_IDX(DT_INST(0, nxp_imx_flexspi), 0);

	struct flash_priv *priv = dev->data;

	priv->pflash_block_base = base;

	flexspi_config_t config;

	/*Get FLEXSPI default settings and configure the flexspi. */
	FLEXSPI_GetDefaultConfig(&config);

	/* Set AHB buffer size for reading data through AHB bus. */
    config.ahbConfig.enableAHBPrefetch = true;
    config.rxSampleClock               = kFLEXSPI_ReadSampleClkLoopbackFromDqsPad;
    FLEXSPI_Init(base, &config);

    /* Configure flash settings according to serial flash feature. */
    FLEXSPI_SetFlashConfig(base, &deviceconfig, kFLEXSPI_PortA1);

    /* Update LUT table. */
    FLEXSPI_UpdateLUT(base, 0, customLUT, CUSTOM_LUT_LENGTH);

    /* Enter quad mode. */
    flash_mcux_flexspi_enable_quad_mode(dev, 0);

	k_sem_init(&priv->write_lock, 0, 1);

	return 0;
}

DEVICE_AND_API_INIT(flash_mcux, DT_INST_LABEL(0),
		    flash_mcux_flexspi_qspi_init, &flash_data, NULL, POST_KERNEL,
		    CONFIG_NXP_QSPI_NOR_INIT_PRIORITY, &flash_mcux_flexspi_qspi_api);