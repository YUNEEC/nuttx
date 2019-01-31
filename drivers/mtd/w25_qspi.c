/************************************************************************************
 * drivers/mtd/w25_qspi.c
 * Driver for SPI-based W25x16, x32, and x64 and W25q16, q32, q64, and q128 FLASH
 *
 *   Copyright (C) 2012-2013, 2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/spi/qspi.h>
#include <nuttx/mtd/mtd.h>

#include "spi_flash.h"

#define CONFIG_W25_SYNC_WRITE
//#define CONFIG_W25_DEBUG
//#define CONFIG_W25_SPI_DEBUG
//#define CONFIG_W25_COMMAND_DEBUG
//#define CONFIG_W25_TEST

/* Status bitmask */
#define W25_ERR_ERASE              0x04
#define W25_ERR_PROGRAM            0x08
#define W25_ERR_ECC                0x20

/* W25 Instructions *****************************************************************/
/*      Command                    Value      Description                           */
/*                                                                                  */
#define W25_WREN                   0x06    /* Write enable                          */
#define W25_WRDI                   0x04    /* Write Disable                         */
#define W25_RDSR                   0x05    /* Read status register                  */
#define W25_WRSR                   0x01    /* Write Status Register                 */
#define W25_RDDATA                 0x03    /* Read data bytes                       */
#define W25_FRD                    0x0b    /* Higher speed read                     */
#define W25_FRDD                   0x3b    /* Fast read, dual output                */
#define W25_FRDQ                   0x6b    /* Fast read, quad output                */
#define W25_FRDQIO                 0xeb    /* Fast read, quad I/O                   */
#define W25_PP                     0x02    /* Program page                          */
#define W25_PPQ                    0x32    /* Program page, quad input              */
#define W25_BE                     0xd8    /* Block Erase (64KB)                    */
#define W25_SE                     0x20    /* Sector erase (4KB)                    */
#define W25_CE                     0xc7    /* Chip erase                            */
#define W25_PD                     0xb9    /* Power down                            */
#define W25_PURDID                 0xab    /* Release PD, Device ID                 */
#define W25_RDMFID                 0x90    /* Read Manufacturer / Device            */
#define W25_JEDEC_ID               0x9f    /* JEDEC ID read                         */
#define W25_RDPAGE                 0x13    /* Read page                             */
#define W25_WRPAGE                 0x10    /* Program execute in page address       */
#define W25_DIE_SEL                0xc2    /* Die select                            */
#define W25_RDBBM                  0xa5    /* Read bad block management lookup table*/
#define W25_RESET                  0xff    /* Reset                                 */
#define W25_ECC_ADDR               0xa9    /* Last ECC address                      */
#define W25_STATUS1                0xA0    /* Status 1                              */
#define W25_STATUS2                0xB0    /* Status 2                              */
#define W25_STATUS3                0xC0    /* Status 3                              */

/* W25 Registers ********************************************************************/
/* Read ID (RDID) register values */

#define W25_MANUFACTURER           0xef   /* Winbond Serial Flash */
#define W25X16_DEVID               0x14   /* W25X16 device ID (0xab, 0x90) */
#define W25X32_DEVID               0x15   /* W25X16 device ID (0xab, 0x90) */
#define W25X64_DEVID               0x16   /* W25X16 device ID (0xab, 0x90) */

/* JEDEC Read ID register values */

#define W25_JEDEC_MANUFACTURER     0xef  /* SST manufacturer ID */
#define W25X_JEDEC_MEMORY_TYPE     0x30  /* W25X memory type */
#define W25Q_JEDEC_MEMORY_TYPE_A   0x40  /* W25Q memory type */
#define W25Q_JEDEC_MEMORY_TYPE_B   0x60  /* W25Q memory type */
#define W25Q_JEDEC_MEMORY_TYPE_C   0x50  /* W25Q memory type */
#define W25Q_JEDEC_MEMORY_TYPE_D   0xab  /* W25M memory type */

#define W25_JEDEC_CAPACITY_8MBIT   0x14  /* 256x4096  = 8Mbit memory capacity */
#define W25_JEDEC_CAPACITY_16MBIT  0x15  /* 512x4096  = 16Mbit memory capacity */
#define W25_JEDEC_CAPACITY_32MBIT  0x16  /* 1024x4096 = 32Mbit memory capacity */
#define W25_JEDEC_CAPACITY_64MBIT  0x17  /* 2048x4096 = 64Mbit memory capacity */
#define W25_JEDEC_CAPACITY_128MBIT 0x18  /* 4096x4096 = 128Mbit memory capacity */
#define W25_JEDEC_CAPACITY_2048MBIT 0x21  /* 2048x131072 = 256Mbit memory capacity */

#define NBLOCKS_8MBIT             256   /* 256 blocks x 4096 bytes/block = 1Mb */
#define NBLOCKS_16MBIT            512   /* 512 blocks x 4096 bytes/block = 2Mb */
#define NBLOCKS_32MBIT            1024  /* 1024 blocks x 4096 bytes/block = 4Mb */
#define NBLOCKS_64MBIT            2048  /* 2048 blocks x 4096 bytes/block = 8Mb */
#define NBLOCKS_128MBIT           4096  /* 4096 blocks x 4096 bytes/block = 16Mb */
#define NBLOCKS_2048MBIT          2048  /* 2048 blocks x 128K bytes/block = 256Mb */

/* Status register bit definitions */

#define W25_SR_BUSY                (1 << 0)  /* Bit 0: Write in progress */
#define W25_SR_WEL                 (1 << 1)  /* Bit 1: Write enable latch bit */
#define W25_SR_BP_SHIFT            (2)       /* Bits 2-5: Block protect bits */
#define W25_SR_BP_MASK             (15 << W25_SR_BP_SHIFT)
#  define W25X16_SR_BP_NONE        (0 << W25_SR_BP_SHIFT)  /* Unprotected */
#  define W25X16_SR_BP_UPPER32nd   (1 << W25_SR_BP_SHIFT)  /* Upper 32nd */
#  define W25X16_SR_BP_UPPER16th   (2 << W25_SR_BP_SHIFT)  /* Upper 16th */
#  define W25X16_SR_BP_UPPER8th    (3 << W25_SR_BP_SHIFT)  /* Upper 8th */
#  define W25X16_SR_BP_UPPERQTR    (4 << W25_SR_BP_SHIFT)  /* Upper quarter */
#  define W25X16_SR_BP_UPPERHALF   (5 << W25_SR_BP_SHIFT)  /* Upper half */
#  define W25X16_SR_BP_ALL         (6 << W25_SR_BP_SHIFT)  /* All sectors */
#  define W25X16_SR_BP_LOWER32nd   (9 << W25_SR_BP_SHIFT)  /* Lower 32nd */
#  define W25X16_SR_BP_LOWER16th   (10 << W25_SR_BP_SHIFT) /* Lower 16th */
#  define W25X16_SR_BP_LOWER8th    (11 << W25_SR_BP_SHIFT) /* Lower 8th */
#  define W25X16_SR_BP_LOWERQTR    (12 << W25_SR_BP_SHIFT) /* Lower quarter */
#  define W25X16_SR_BP_LOWERHALF   (13 << W25_SR_BP_SHIFT) /* Lower half */

#  define W25X32_SR_BP_NONE        (0 << W25_SR_BP_SHIFT)  /* Unprotected */
#  define W25X32_SR_BP_UPPER64th   (1 << W25_SR_BP_SHIFT)  /* Upper 64th */
#  define W25X32_SR_BP_UPPER32nd   (2 << W25_SR_BP_SHIFT)  /* Upper 32nd */
#  define W25X32_SR_BP_UPPER16th   (3 << W25_SR_BP_SHIFT)  /* Upper 16th */
#  define W25X32_SR_BP_UPPER8th    (4 << W25_SR_BP_SHIFT)  /* Upper 8th */
#  define W25X32_SR_BP_UPPERQTR    (5 << W25_SR_BP_SHIFT)  /* Upper quarter */
#  define W25X32_SR_BP_UPPERHALF   (6 << W25_SR_BP_SHIFT)  /* Upper half */
#  define W25X32_SR_BP_ALL         (7 << W25_SR_BP_SHIFT)  /* All sectors */
#  define W25X32_SR_BP_LOWER64th   (9 << W25_SR_BP_SHIFT)  /* Lower 64th */
#  define W25X32_SR_BP_LOWER32nd   (10 << W25_SR_BP_SHIFT) /* Lower 32nd */
#  define W25X32_SR_BP_LOWER16th   (11 << W25_SR_BP_SHIFT) /* Lower 16th */
#  define W25X32_SR_BP_LOWER8th    (12 << W25_SR_BP_SHIFT) /* Lower 8th */
#  define W25X32_SR_BP_LOWERQTR    (13 << W25_SR_BP_SHIFT) /* Lower quarter */
#  define W25X32_SR_BP_LOWERHALF   (14 << W25_SR_BP_SHIFT) /* Lower half */

#  define W25X64_SR_BP_NONE        (0 << W25_SR_BP_SHIFT)  /* Unprotected */
#  define W25X64_SR_BP_UPPER64th   (1 << W25_SR_BP_SHIFT)  /* Upper 64th */
#  define W25X64_SR_BP_UPPER32nd   (2 << W25_SR_BP_SHIFT)  /* Upper 32nd */
#  define W25X64_SR_BP_UPPER16th   (3 << W25_SR_BP_SHIFT)  /* Upper 16th */
#  define W25X64_SR_BP_UPPER8th    (4 << W25_SR_BP_SHIFT)  /* Upper 8th */
#  define W25X64_SR_BP_UPPERQTR    (5 << W25_SR_BP_SHIFT)  /* Upper quarter */
#  define W25X64_SR_BP_UPPERHALF   (6 << W25_SR_BP_SHIFT)  /* Upper half */
#  define W25X46_SR_BP_ALL         (7 << W25_SR_BP_SHIFT)  /* All sectors */
#  define W25X64_SR_BP_LOWER64th   (9 << W25_SR_BP_SHIFT)  /* Lower 64th */
#  define W25X64_SR_BP_LOWER32nd   (10 << W25_SR_BP_SHIFT) /* Lower 32nd */
#  define W25X64_SR_BP_LOWER16th   (11 << W25_SR_BP_SHIFT) /* Lower 16th */
#  define W25X64_SR_BP_LOWER8th    (12 << W25_SR_BP_SHIFT) /* Lower 8th */
#  define W25X64_SR_BP_LOWERQTR    (13 << W25_SR_BP_SHIFT) /* Lower quarter */
#  define W25X64_SR_BP_LOWERHALF   (14 << W25_SR_BP_SHIFT) /* Lower half */
                                             /* Bit 6: Reserved */
#define W25_SR_SRP                 (1 << 7)  /* Bit 7: Status register write protect */

#define W25_DUMMY                  0xa5

/* Chip Geometries ******************************************************************/
/* All members of the family support uniform 4K-byte sectors and 256 byte pages */

#define W25_BLOCK_SHIFT            17        /* Sector size 1 << 17 = 128Kb */
#define W25_BLOCK_SIZE             (1 << 17) /* Sector size 1 << 17 = 128Kb */
#define W25_PAGE_SHIFT             11        /* Sector size 1 << 11 = 2048b */
#define W25_PAGE_SIZE              (1 << 11)  /* Sector size 1 << 11 = 2048b */
#define W25_DIE_SHIFT              27
#define W25_SPARE_SIZE             64        /* 16*4 bytes spare size */

static void w25_lock(FAR struct qspi_dev_s *spi)
{
    /* On SPI busses where there are multiple devices, it will be necessary to
    * lock SPI to have exclusive access to the busses for a sequence of
    * transfers.  The bus should be locked before the chip is selected.
    *
    * This is a blocking call and will not return until we have exclusiv access to
    * the SPI buss.  We will retain that exclusive access until the bus is unlocked.
    */

    (void)QSPI_LOCK(spi, true);
}

static void w25_unlock(FAR struct qspi_dev_s *spi)
{
    (void)QSPI_LOCK(spi, false);
}

static int w25_command_read(FAR struct qspi_dev_s *spi, uint8_t cmd,
                            FAR void *buffer, size_t buflen,
                            off_t addr, uint8_t addrlen)
{
    struct qspi_cmdinfo_s cmdinfo;

    cmdinfo.flags = 0;

    if (buflen) {
        cmdinfo.flags |= QSPICMD_READDATA;
    } else {
        buffer = NULL;
    }

    if (addrlen) {
        cmdinfo.flags |=  QSPICMD_ADDRESS;
    } else {
        addr = 0;
    }

    cmdinfo.cmd     = cmd;
    cmdinfo.buflen  = buflen;
    cmdinfo.buffer  = (FAR void *)buffer;
    cmdinfo.addrlen = addrlen;
    cmdinfo.addr    = addr;

    int ret = QSPI_COMMAND(spi, &cmdinfo);

#ifdef CONFIG_W25_COMMAND_DEBUG
    ferr("CMD: %02x buflen: %d addr: %08lx addrlen: %d ret: %d\n",
          cmd, buflen, addr, addrlen, ret);
#endif

    return ret;
}

static int w25_data_read(FAR struct qspi_dev_s *spi, FAR int8_t *buffer,
                        size_t buflen, off_t addr, uint8_t addrlen)
{
    struct qspi_meminfo_s meminfo;

    meminfo.flags   = QSPIMEM_READ | QSPIMEM_QUADIO;
    meminfo.cmd     = W25_FRDQIO;
    meminfo.dummies = 2;
    meminfo.buflen  = buflen;
    meminfo.buffer  = buffer;
    meminfo.addrlen = addrlen;
    meminfo.addr    = addr;

    int ret = QSPI_MEMORY(spi, &meminfo);

#ifdef CONFIG_W25_COMMAND_DEBUG
    ferr("addr: %08lx addrlen: %d nbytes: %d ret: %d\n", addr, addrlen, (int)buflen, ret);
#endif

    return ret;
}

static int w25_command_write(FAR struct qspi_dev_s *spi, uint8_t cmd,
                             FAR const void *buffer, size_t buflen,
                             off_t addr, uint8_t addrlen)
{
    struct qspi_cmdinfo_s cmdinfo;

    cmdinfo.flags = 0;

    if (buflen) {
        cmdinfo.flags |= QSPICMD_WRITEDATA;
    } else {
        buffer = NULL;
    }

    if (addrlen) {
        cmdinfo.flags |=  QSPICMD_ADDRESS;
    } else {
        addr = 0;
    }

    cmdinfo.cmd     = cmd;
    cmdinfo.buflen  = buflen;
    cmdinfo.buffer  = (FAR void *)buffer;
    cmdinfo.addrlen = addrlen;
    cmdinfo.addr    = addr;

    int ret = QSPI_COMMAND(spi, &cmdinfo);

#ifdef CONFIG_W25_COMMAND_DEBUG
    ferr("CMD: %02x buflen: %d addr: %08lx addrlen: %d ret: %d\n",
          cmd, buflen, addr, addrlen, ret);
#endif

    return ret;
}

static int w25_command(FAR struct qspi_dev_s *spi, uint8_t cmd,
                       off_t addr, uint8_t addrlen)
{
    struct qspi_cmdinfo_s cmdinfo;

    cmdinfo.flags   = 0;
    if (addrlen) {
        cmdinfo.flags |= QSPICMD_ADDRESS;
    } else {
        addr = 0;
    }

    cmdinfo.cmd     = cmd;
    cmdinfo.buflen  = 0;
    cmdinfo.buffer  = NULL;
    cmdinfo.addr    = addr;
    cmdinfo.addrlen = addrlen;

    int ret = QSPI_COMMAND(spi, &cmdinfo);

#ifdef CONFIG_W25_COMMAND_DEBUG
    ferr("CMD: %02x addr: %08lx addrlen: %d ret: %d\n",
          cmd, addr, addrlen, ret);
#endif

    return ret;
}

static int w25_data_write(FAR struct qspi_dev_s *spi, const FAR int8_t *buffer,
                        size_t buflen, off_t addr, uint8_t addrlen)
{
    struct qspi_meminfo_s meminfo;

    meminfo.flags   = QSPIMEM_WRITE | QSPIMEM_QUADIO;
    meminfo.cmd     = W25_PPQ;

    meminfo.dummies = 0;
    meminfo.buflen  = buflen;
    meminfo.buffer  = (FAR void *)buffer;
    meminfo.addrlen = addrlen;
    meminfo.addr    = addr;

    int ret = QSPI_MEMORY(spi, &meminfo);

#ifdef CONFIG_W25_COMMAND_DEBUG
    ferr("addr: %08lx addrlen: %d nbytes: %d ret: %d\n", addr, addrlen, (int)buflen, ret);
#endif

    return ret;
}

static inline void w25_wren_locked(FAR struct qspi_dev_s *spi)
{
    w25_command(spi, W25_WREN, 0, 0);
}

static inline void w25_wrdi_locked(FAR struct qspi_dev_s *spi)
{
    w25_command(spi, W25_WRDI, 0, 0);
}

static uint8_t w25_read_status_locked(FAR struct spi_flash_dev_s *priv, uint8_t regaddr)
{
    w25_command_read(priv->qspi, W25_RDSR, priv->cmdbuf, 1, regaddr, 1);
    return priv->cmdbuf[0];
}

static void w25_write_status_locked(FAR struct spi_flash_dev_s *priv, uint8_t regaddr, uint8_t value)
{
    uint8_t status;

    do {
        priv->cmdbuf[0] = value;
        w25_command_write(priv->qspi, W25_WRSR, priv->cmdbuf, 1, regaddr, 1);

        status = w25_read_status_locked(priv, regaddr);
#ifdef CONFIG_W25_DEBUG
        if (value != status)
            ferr("regaddr = %02x, value = %02x, status = %02x\n", regaddr, value, status);
#endif
    } while (value != status);
}

static void w25_select_die_locked(FAR struct spi_flash_dev_s *priv, uint8_t die)
{
    w25_command(priv->qspi, W25_DIE_SEL, die, 1);
}

static void w25_reset(FAR struct qspi_dev_s *spi)
{
    w25_command(spi, W25_RESET, 0, 0);
}

#ifdef CONFIG_W25_DEBUG
static void w25_read_last_ecc_address(FAR struct qspi_dev_s *spi, uint16_t *page)
{
}
#endif

static inline int w25_readid(FAR struct spi_flash_dev_s *priv)
{
    uint16_t manufacturer;
    uint16_t memory;
    uint16_t capacity;

    /* Lock the QuadSPI bus and configure the bus. */

    w25_lock(priv->qspi);

    /* Read the JEDEC ID */

    w25_command_read(priv->qspi, W25_JEDEC_ID, priv->cmdbuf, 4, 0, 1);

    manufacturer = priv->cmdbuf[0];
    memory = priv->cmdbuf[1];
    capacity = priv->cmdbuf[2];

    /* Unlock the bus */

    w25_unlock(priv->qspi);

    finfo("Manufacturer: %02x Device Type %02x, Capacity: %02x\n",
            manufacturer, memory, capacity);

    /* Check for W25 chip */

    if (manufacturer != W25_JEDEC_MANUFACTURER ||
        memory != W25Q_JEDEC_MEMORY_TYPE_D) {
        ferr("ERROR: Unrecognized device type: 0x%02x 0x%02x 0x%02x\n",
            manufacturer, memory, capacity);
        return -ENODEV;
    }

    priv->manufacturer = manufacturer;
    priv->memory = memory;
    priv->capacity = capacity;

    /* Check for a supported capacity */

    switch (capacity)
    {
        case W25_JEDEC_CAPACITY_2048MBIT:
            priv->nblocks = NBLOCKS_2048MBIT;
            break;

        default:
            ferr("ERROR: Unsupported memory capacity: %02x\n", capacity);
            return -ENODEV;
    }

    return OK;
}

static uint8_t w25_waitcomplete_locked(FAR struct spi_flash_dev_s *priv)
{
    uint8_t status;

    /* Loop as long as the memory is busy with a write cycle. Device sets BUSY
     * flag to a 1 state whhen previous write or erase command is still executing
     * and during this time, device will ignore further instructions except for
     * "Read Status Register" and "Erase/Program Suspend" instructions.
     */

    do
    {
        status = w25_read_status_locked(priv, W25_STATUS3);

        if (!(status & W25_SR_BUSY))
            break;

        /* Given that writing could take up to few tens of milliseconds, and erasing
         * could take more.  The following short delay in the "busy" case will allow
         * other peripherals to access the SPI bus.  Delay would slow down writing
         * too much, so go to sleep only if previous operation was not a page program
         * operation.
         */

#if 0
        if (priv->prev_instr != W25_WRPAGE && (status & W25_SR_BUSY) != 0)
        {
            w25_unlock(priv->qspi);
            usleep(1000);
            w25_lock(priv->qspi);
        }
#endif
        usleep(500);
    } while (1);

    if (status & W25_ERR_ERASE) {
        w25_unlock(priv->qspi);

#ifdef CONFIG_W25_DEBUG
        ferr("erase error block = %08x\n", priv->lastaddr >> priv->block_shift);
#endif
        spi_mark_badblock(priv, priv->lastaddr >> priv->block_shift);

        w25_lock(priv->qspi);
    }

    if (status & W25_ERR_PROGRAM) {
        w25_unlock(priv->qspi);

#ifdef CONFIG_W25_DEBUG
        ferr("program error block = %08x\n", priv->lastaddr >> priv->block_shift);
#endif
        spi_mark_badblock(priv, priv->lastaddr >> priv->block_shift);

        w25_lock(priv->qspi);
    }

    return status;
}

static int w25_blockerase(FAR struct spi_flash_dev_s *priv, size_t block)
{
    int ret = OK;
    off_t address = block << priv->block_shift;

#ifdef CONFIG_W25_SPI_DEBUG
    ferr("block: %08lx\n", (long)block);
#endif

    if (spi_is_badblock(priv, block)) {
        ferr("bad block %d\n", block);
        ret = -EIO;
        goto errout;
    }

    /* Lock and configure the SPI bus */

    w25_lock(priv->qspi);

    w25_select_die_locked(priv, (uint8_t)(address>>W25_DIE_SHIFT));

#ifndef CONFIG_W25_SYNC_WRITE
    /* Wait for any preceding write or erase operation to complete. */

    (void)w25_waitcomplete_locked(priv);
#endif
    priv->lastaddr = address;

    /* Send write enable instruction */

    w25_wren_locked(priv->qspi);

    /* Send the sector address high byte first. Only the most significant bits (those
     * corresponding to the sector) have any meaning.
     */
    uint16_t page = address >> priv->page_shift;

    off_t commandaddr = (off_t)page;

    /* Send the "Sector Erase (SE)" instruction */
    w25_command(priv->qspi, W25_BE, commandaddr, 3);
    priv->prev_instr = W25_BE;

#ifdef CONFIG_W25_SYNC_WRITE
    /* Wait for any preceding write or erase operation to complete. */
    uint8_t status = w25_waitcomplete_locked(priv);
#endif

    w25_wrdi_locked(priv->qspi);

    w25_unlock(priv->qspi);

#ifdef CONFIG_W25_SYNC_WRITE
    if (status & W25_ERR_ERASE) {
#ifdef CONFIG_W25_DEBUG
        ferr("erase error block = %08x\n", priv->lastaddr >> priv->block_shift);
#endif
        spi_mark_badblock(priv, address >> priv->block_shift);
        ret = -EIO;
    }
#endif

errout:
    return ret;
}

static ssize_t w25_pageread(FAR struct spi_flash_dev_s *priv, off_t address, size_t nbytes,
                            bool spare, FAR uint8_t *buffer)
{
#ifdef CONFIG_W25_SPI_DEBUG
    ferr("address: %08lx nbytes: %d\n", (long)address, (int)nbytes);
#endif

    if ((spare && (nbytes != priv->spare_size)) || (nbytes > priv->page_size))
        return -EFAULT;

    w25_lock(priv->qspi);

    w25_select_die_locked(priv, (uint8_t)(address>>W25_DIE_SHIFT));

#ifndef CONFIG_W25_SYNC_WRITE
    /* Wait for any preceding write or erase operation to complete. */

    (void)w25_waitcomplete_locked(priv);
#endif

    /* Make sure that writing is disabled */

    w25_wrdi_locked(priv->qspi);

    /* Read page data into cache */

    uint16_t page = address >> priv->page_shift;

    off_t commandaddr = (off_t)page;

    w25_command(priv->qspi, W25_RDPAGE, commandaddr, 3);
    priv->prev_instr = W25_RDPAGE;

    uint8_t status = w25_waitcomplete_locked(priv);
    if (status & W25_ERR_ECC ) {
#ifdef CONFIG_W25_DEBUG
        uint8_t status1 = w25_read_status_locked(priv, W25_STATUS1);
        uint8_t status2 = w25_read_status_locked(priv, W25_STATUS2);
#endif

        w25_unlock(priv->qspi);

        spi_mark_badblock(priv, address >> priv->block_shift);

#ifdef CONFIG_W25_DEBUG
        uint16_t ecc_page;
        w25_read_last_ecc_address(priv->qspi, &ecc_page);
        ferr("ecc error block = %08x, page = %04x, last ecc page = %04x, status = (%02x, %02x, %02x)\n", address>>priv->block_shift, page, ecc_page, status1, status2, status);
#else
        ferr("ecc error block = %08x, page = %04x, status = %02x\n", address>>priv->block_shift, page, status);
#endif
        return -EIO;
    }

    /* Send "Read from Memory " instruction */

    off_t readaddr = 0;
    w25_data_read(priv->qspi, priv->page_buf, priv->page_size+priv->spare_size, readaddr<<8, 3);

    priv->prev_instr = W25_FRDQIO;
    w25_unlock(priv->qspi);

    if (spare)
        memcpy(buffer, (FAR uint8_t *)(priv->page_buf+priv->page_size), priv->spare_size);
    else {
        if ((priv->page_buf[priv->page_size] != SPI_ERASED_STATE) ||
          (priv->page_buf[priv->page_size+1] != SPI_ERASED_STATE)) {
            ferr("bad block = %08x, page = %04x (%02x %02x)\n",
                    address>>priv->block_shift, page,
                    priv->page_buf[priv->page_size],
                    priv->page_buf[priv->page_size+1]);
            /* bad block */
            return -EIO;
        }

        uint16_t column = address & ((1 << priv->page_shift) - 1);
        memcpy(buffer, (FAR uint8_t *)(priv->page_buf+column), nbytes);
    }

    return nbytes;
}

static ssize_t w25_pagewrite(FAR struct spi_flash_dev_s *priv, off_t address,
                          size_t nbytes, bool spare, FAR const uint8_t *buffer)
{
    int ret = nbytes;
    uint16_t page = address >> priv->page_shift;
    uint16_t column = address & ((1 << priv->page_shift) - 1);

#ifdef CONFIG_W25_SPI_DEBUG
    ferr("address: %08lx nwords: %d\n", (long)address, (int)nbytes);
#endif

    if ((spare && (nbytes > priv->spare_size)) || (nbytes > priv->page_size))
        return -EFAULT;

    w25_lock(priv->qspi);

    w25_select_die_locked(priv, (uint8_t)(address>>W25_DIE_SHIFT));

#ifndef CONFIG_W25_SYNC_WRITE
    /* Wait for any preceding write or erase operation to complete. */

    (void)w25_waitcomplete_locked(priv);
#endif

    /* Enable write access to the FLASH */

    w25_wren_locked(priv->qspi);

    /* Align column to end of page to write spare */
    if (spare)
        column = priv->page_size;

    /* Send the "Page Program (W25_PPQ)" Command */

    w25_data_write(priv->qspi, buffer, nbytes, column, 2);
    priv->prev_instr = W25_PPQ;

    /* Program  Execute */

    off_t commandaddr = (off_t)page;
    w25_command(priv->qspi, W25_WRPAGE, commandaddr, 3);

#ifdef CONFIG_W25_SYNC_WRITE
    /* Wait for any preceding write or erase operation to complete. */
    uint8_t status = w25_waitcomplete_locked(priv);
#endif

    w25_wrdi_locked(priv->qspi);

    w25_unlock(priv->qspi);

#ifdef CONFIG_W25_SYNC_WRITE
    if (status & W25_ERR_PROGRAM) {
        if (!spare) {
          spi_mark_badblock(priv, address >> priv->block_shift);
        }
#ifdef CONFIG_W25_DEBUG
        ferr("program error block = %08x\n", address >> priv->block_shift);
#endif
        ret = -EIO;
    }
#endif

    return ret;
}

#ifdef CONFIG_SPI_TEST
void spi_test_internal(FAR struct spi_flash_dev_s *priv)
{
    for (int block = 0; block < 64; block++) {
        w25_blockerase(priv, block);
    }

    for (int block = 0; block < 64; block++) {
        w25_blockerase(priv, 0x400 + block);
    }

    FAR uint8_t *buffer;
    buffer = (FAR uint8_t *)kmm_malloc(W25_PAGE_SIZE);

    const int nbytes = W25_PAGE_SIZE;

    bool isfailed = false;
    for (int page = 0; page < 64 * 16; page++) {

        for (int index = 0; index < nbytes; index++)
            buffer[index] = index & 0xff;

        w25_pagewrite(priv, (page << W25_PAGE_SHIFT), nbytes, false, buffer);

        memset(buffer, 0, W25_PAGE_SIZE);
        w25_pageread(priv, (page << W25_PAGE_SHIFT), nbytes, false, buffer);

        for (int index = 0; index < nbytes; index++) {
            if (buffer[index] != (index & 0xff)) {
                ferr("data incorrect: page %d index %d value %02x\n", page, index, buffer[index]);

                index = 0;
                while (index < nbytes) {
                    printf("%02x ", buffer[index++]);
                }
                printf("\n");

                isfailed = true;
                break;
            } else {
                //ferr("data correct: page %d index %d value %02x\n", page, index, buffer[index]);
            }
        }

        if (isfailed) {
            break;
        }
    }

    if (isfailed) {
        ferr("QSPI test is failed\n");
    } else {
        ferr("QSPI test is passed\n");
    }

    kmm_free(buffer);
}
#endif

static void w25_unprotect(FAR struct spi_flash_dev_s *priv, int die)
{
    w25_lock(priv->qspi);

    w25_select_die_locked(priv, die);

#ifndef CONFIG_W25_SYNC_WRITE
    /* Wait for any preceding write or erase operation to complete. */

    (void)w25_waitcomplete_locked(priv);
#endif

    /* Send "Write enable (WREN)" */

    w25_wren_locked(priv->qspi);

    w25_write_status_locked(priv, W25_STATUS1, 0);

    w25_unlock(priv->qspi);
}

int w25_qspi_flash_initialize(FAR struct spi_flash_dev_s *priv)
{
    int ret;

    if (!priv) {
        return -EFAULT;
    }

    priv->block_size = W25_BLOCK_SIZE;
    priv->block_shift = W25_BLOCK_SHIFT;
    priv->page_size = W25_PAGE_SIZE;
    priv->page_shift = W25_PAGE_SHIFT;
    priv->spare_size = W25_SPARE_SIZE;

    priv->blockerase = w25_blockerase;
    priv->pageread = w25_pageread;
    priv->pagewrite = w25_pagewrite;

    QSPI_SETFREQUENCY(priv->qspi, CONFIG_QSPI_FREQUENCY);
    QSPI_SETBITS(priv->qspi, 8);
    QSPI_SETMODE(priv->qspi, CONFIG_SPI_MODE);

    /* Allocate a 4-byte buffer to support DMA-able command data */

    priv->cmdbuf = (FAR uint8_t *)QSPI_ALLOC(priv->qspi, 4);
    if (priv->cmdbuf == NULL)
      {
        ferr("Failed to allocate command buffer\n");
        return -ENOMEM;
      }

    w25_reset(priv->qspi);
    usleep(5000);

    /* Identify the FLASH chip and get its capacity */

    ret = w25_readid(priv);
    if (ret != OK) {
        /* Unrecognized! Discard all of that work we just did and return NULL */

        ferr("Unrecognized QSPI device\n");
        goto errout;
    }

    /* Make sure that the FLASH is unprotected so that we can write into it */

    w25_unprotect(priv, 0);
    w25_unprotect(priv, 1);

    return OK;

errout:
    QSPI_FREE(priv->qspi, priv->cmdbuf);

    return -EFAULT;
}

