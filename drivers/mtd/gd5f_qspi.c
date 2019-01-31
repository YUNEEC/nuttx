/************************************************************************************
 * drivers/mtd/gd5f_qspi.c
 * Driver for SPI-based GD5Fx16, x32, and x64 and GD5Fq16, q32, q64, and q128 FLASH
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

#define CONFIG_GD5F_SYNC_WRITE
//#define CONFIG_GD5F_DEBUG
//#define CONFIG_GD5F_SPI_DEBUG
//#define CONFIG_GD5F_COMMAND_DEBUG
//#define CONFIG_GD5F_TEST

/* GD5F Instructions *****************************************************************/
/*      Command                    Value      Description                           */
/*                                                                                  */
#define GD5F_WREN                   0x06    /* Write enable                          */
#define GD5F_WRDI                   0x04    /* Write Disable                         */
#define GD5F_RDSR                   0x0f    /* Read status register                  */
#define GD5F_WRSR                   0x1f    /* Write Status Register                 */
#define GD5F_RDDATA                 0x03    /* Read data bytes                       */
#define GD5F_FRD                    0x0b    /* Higher speed read                     */
#define GD5F_FRDD                   0x3b    /* Fast read, dual output                */
#define GD5F_FRDQ                   0x6b    /* Fast read, quad output                */
#define GD5F_FRDQIO                 0xeb    /* Fast read, quad I/O                   */
#define GD5F_PP                     0x02    /* Program page                          */
#define GD5F_PPQ                    0x32    /* Program page, quad input              */
#define GD5F_BE                     0xd8    /* Block Erase (64KB)                    */
#define GD5F_JEDEC_ID               0x9f    /* JEDEC ID read                         */
#define GD5F_RDPAGE                 0x13    /* Read page                             */
#define GD5F_WRPAGE                 0x10    /* Program execute in page address       */
#define GD5F_RESET                  0xff    /* Reset                                 */

#define GD5F_STATUS1                0xA0    /* Status 1                              */
#define GD5F_STATUS2                0xB0    /* Status 2                              */
#define GD5F_STATUS3                0xC0    /* Status 3                              */

/* GD5F Registers ********************************************************************/
/* Read ID (RDID) register values */

#define GD5F_MANUFACTURER           0xC8   /* GigaDevice Serial Flash */
#define GD5F2GQ4U_DEVID             0xD2   /* GD5F2GQ4U device ID (0xD2) */

#define NBLOCKS_2048MBIT          2048  /* 2048 blocks x 128K bytes/block = 256Mb */

/* Status register bit definitions */

#define GD5F_SR_BUSY                0x01  /* Bit 0: Write in progress */
#define GD5F_SR_WEL                 0x02  /* Bit 1: Write enable latch bit */
#define GD5F_SR_ERR_ERASE           0x04  /* Bit 2: Erase failure bit */
#define GD5F_SR_ERR_PROGRAM         0x08  /* Bit 3: Program failure bit */
#define GD5F_SR_ERR_ECC             0x20  /* Bit 4-5: Bit5=1,Bit4=0: Bits errors greater than ECC capability */

#define GD5F_DUMMY                  0x00

/* Chip Geometries ******************************************************************/
/* All members of the family support uniform 4K-byte sectors and 256 byte pages */

#define GD5F_BLOCK_SHIFT            17        /* Sector size 1 << 17 = 128Kb */
#define GD5F_BLOCK_SIZE             (1 << 17) /* Sector size 1 << 17 = 128Kb */
#define GD5F_PAGE_SHIFT             11        /* Sector size 1 << 11 = 2048b */
#define GD5F_PAGE_SIZE              (1 << 11)  /* Sector size 1 << 11 = 2048b */

#define GD5F_SPARE_SIZE             64        /* 16*4 bytes spare size */

static void gd5f_lock(FAR struct qspi_dev_s *spi)
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

static void gd5f_unlock(FAR struct qspi_dev_s *spi)
{
    (void)QSPI_LOCK(spi, false);
}

static int gd5f_command_read(FAR struct qspi_dev_s *spi, uint8_t cmd,
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

#ifdef CONFIG_GD5F_COMMAND_DEBUG
    ferr("CMD: %02x buflen: %d addr: %08lx addrlen: %d ret: %d\n",
          cmd, buflen, addr, addrlen, ret);
#endif

    return ret;
}

static int gd5f_data_read(FAR struct qspi_dev_s *spi, FAR int8_t *buffer,
                        size_t buflen, off_t addr, uint8_t addrlen)
{
    struct qspi_meminfo_s meminfo;

    meminfo.flags   = QSPIMEM_READ | QSPIMEM_QUADIO;
    meminfo.cmd     = GD5F_FRDQIO;
    meminfo.dummies = 2;
    meminfo.buflen  = buflen;
    meminfo.buffer  = buffer;
    meminfo.addrlen = addrlen;
    meminfo.addr    = addr;

    int ret = QSPI_MEMORY(spi, &meminfo);

#ifdef CONFIG_GD5F_COMMAND_DEBUG
    ferr("addr: %08lx addrlen: %d nbytes: %d ret: %d\n", addr, addrlen, (int)buflen, ret);
#endif

    return ret;
}

static int gd5f_command_write(FAR struct qspi_dev_s *spi, uint8_t cmd,
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

#ifdef CONFIG_GD5F_COMMAND_DEBUG
    ferr("CMD: %02x buflen: %d addr: %08lx addrlen: %d ret: %d\n",
          cmd, buflen, addr, addrlen, ret);
#endif

    return ret;
}

static int gd5f_command(FAR struct qspi_dev_s *spi, uint8_t cmd,
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

#ifdef CONFIG_GD5F_COMMAND_DEBUG
    ferr("CMD: %02x addr: %08lx addrlen: %d ret: %d\n",
          cmd, addr, addrlen, ret);
#endif

    return ret;
}

static int gd5f_data_write(FAR struct qspi_dev_s *spi, const FAR int8_t *buffer,
                        size_t buflen, off_t addr, uint8_t addrlen)
{
    struct qspi_meminfo_s meminfo;

    meminfo.flags   = QSPIMEM_WRITE | QSPIMEM_QUADIO;
    meminfo.cmd     = GD5F_PPQ;

    meminfo.dummies = 0;
    meminfo.buflen  = buflen;
    meminfo.buffer  = (FAR void *)buffer;
    meminfo.addrlen = addrlen;
    meminfo.addr    = addr;

    int ret = QSPI_MEMORY(spi, &meminfo);

#ifdef CONFIG_GD5F_COMMAND_DEBUG
    ferr("addr: %08lx addrlen: %d nbytes: %d ret: %d\n", addr, addrlen, (int)buflen, ret);
#endif

    return ret;
}

static inline void gd5f_wren_locked(FAR struct qspi_dev_s *spi)
{
    gd5f_command(spi, GD5F_WREN, 0, 0);
}

static inline void gd5f_wrdi_locked(FAR struct qspi_dev_s *spi)
{
    gd5f_command(spi, GD5F_WRDI, 0, 0);
}

static uint8_t gd5f_read_status_locked(FAR struct spi_flash_dev_s *priv, uint8_t regaddr)
{
    gd5f_command_read(priv->qspi, GD5F_RDSR, priv->cmdbuf, 1, regaddr, 1);
    return priv->cmdbuf[0];
}

static void gd5f_write_status_locked(FAR struct spi_flash_dev_s *priv, uint8_t regaddr, uint8_t value)
{
    uint8_t status;

    do {
        priv->cmdbuf[0] = value;
        gd5f_command_write(priv->qspi, GD5F_WRSR, priv->cmdbuf, 1, regaddr, 1);

        status = gd5f_read_status_locked(priv, regaddr);
    #ifdef CONFIG_GD5F_DEBUG
        if (value != status)
            ferr("regaddr = %02x, value = %02x, status = %02x\n", regaddr, value, status);
    #endif
    } while (value != status);
}



static void gd5f_reset(FAR struct qspi_dev_s *spi)
{
    gd5f_command(spi, GD5F_RESET, 0, 0);
}

#ifdef CONFIG_GD5F_DEBUG
static void gd5f_read_last_ecc_address(FAR struct qspi_dev_s *spi, uint16_t *page)
{
}
#endif

static inline int gd5f_readid(FAR struct spi_flash_dev_s *priv)
{
    uint16_t manufacturer;
    uint16_t memory;
    uint16_t capacity;

    /* Lock the QuadSPI bus and configure the bus. */

    gd5f_lock(priv->qspi);

    /* Read the JEDEC ID */

    gd5f_command_read(priv->qspi, GD5F_JEDEC_ID, priv->cmdbuf, 4, 0, 1);

    manufacturer = priv->cmdbuf[0];
    memory = priv->cmdbuf[1];
    capacity = 0x00;

    /* Unlock the bus */

    gd5f_unlock(priv->qspi);

    ferr("Manufacturer: %02x Device Type %02x, Capacity: %02x\n",
            manufacturer, memory, capacity);

    /* Check for GD5F chip */

    if (manufacturer != GD5F_MANUFACTURER ||
        memory != GD5F2GQ4U_DEVID) {
        ferr("ERROR: Unrecognized device type: 0x%02x 0x%02x 0x%02x\n",
            manufacturer, memory, capacity);
        return -ENODEV;
    }

    priv->manufacturer = manufacturer;
    priv->memory = memory;
    priv->capacity = capacity;

    /* Check for a supported capacity */

    switch (memory)
    {
        case GD5F2GQ4U_DEVID:
            priv->nblocks = NBLOCKS_2048MBIT;
            break;

        default:
            ferr("ERROR: Unsupported memory capacity: %02x\n", memory);
            return -ENODEV;
    }

    return OK;
}

static uint8_t gd5f_waitcomplete_locked(FAR struct spi_flash_dev_s *priv)
{
    uint8_t status;

    /* Loop as long as the memory is busy with a write cycle. Device sets BUSY
     * flag to a 1 state whhen previous write or erase command is still executing
     * and during this time, device will ignore further instructions except for
     * "Read Status Register" and "Erase/Program Suspend" instructions.
     */

    do
    {
        status = gd5f_read_status_locked(priv, GD5F_STATUS3);

        if (!(status & GD5F_SR_BUSY))
            break;

        /* Given that writing could take up to few tens of milliseconds, and erasing
         * could take more.  The following short delay in the "busy" case will allow
         * other peripherals to access the SPI bus.  Delay would slow down writing
         * too much, so go to sleep only if previous operation was not a page program
         * operation.
         */

#if 0
        if (priv->prev_instr != GD5F_WRPAGE && (status & GD5F_SR_BUSY) != 0)
        {
            gd5f_unlock(priv->qspi);
            usleep(1000);
            gd5f_lock(priv->qspi);
        }
#endif
        usleep(500);
    } while (1);

    if (status & GD5F_SR_ERR_ERASE) {
        gd5f_unlock(priv->qspi);

#ifdef CONFIG_GD5F_DEBUG
        ferr("erase error block = %08x\n", priv->lastaddr >> priv->block_shift);
#endif
        spi_mark_badblock(priv, priv->lastaddr >> priv->block_shift);

        gd5f_lock(priv->qspi);
    }

    if (status & GD5F_SR_ERR_PROGRAM) {
        gd5f_unlock(priv->qspi);

#ifdef CONFIG_GD5F_DEBUG
        ferr("program error block = %08x\n", priv->lastaddr >> priv->block_shift);
#endif
        spi_mark_badblock(priv, priv->lastaddr >> priv->block_shift);

        gd5f_lock(priv->qspi);
    }

    return status;
}

static int gd5f_blockerase(FAR struct spi_flash_dev_s *priv, size_t block)
{
    int ret = OK;
    off_t address = block << priv->block_shift;

#if 1//def CONFIG_GD5F_SPI_DEBUG
    ferr("block: %08lx\n", (long)block);
#endif

    if (spi_is_badblock(priv, block)) {
        ferr("bad block %d\n", block);
        ret = -EIO;
        goto errout;
    }

    /* Lock and configure the SPI bus */

    gd5f_lock(priv->qspi);

#ifndef CONFIG_GD5F_SYNC_WRITE
    /* Wait for any preceding write or erase operation to complete. */

    (void)gd5f_waitcomplete_locked(priv);
#endif
    priv->lastaddr = address;

    /* Send write enable instruction */

    gd5f_wren_locked(priv->qspi);

    /* Send the sector address high byte first. Only the most significant bits (those
     * corresponding to the sector) have any meaning.
     */
    uint32_t page = address >> priv->page_shift;

    off_t commandaddr = (off_t)page;

    /* Send the "Sector Erase (SE)" instruction */
    gd5f_command(priv->qspi, GD5F_BE, commandaddr, 3);
    priv->prev_instr = GD5F_BE;

#ifdef CONFIG_GD5F_SYNC_WRITE
    /* Wait for any preceding write or erase operation to complete. */
    uint8_t status = gd5f_waitcomplete_locked(priv);
#endif

    gd5f_wrdi_locked(priv->qspi);

    gd5f_unlock(priv->qspi);

#ifdef CONFIG_GD5F_SYNC_WRITE
    if (status & GD5F_SR_ERR_ERASE) {
#ifdef CONFIG_GD5F_DEBUG
        ferr("erase error block = %08x\n", priv->lastaddr >> priv->block_shift);
#endif
        spi_mark_badblock(priv, address >> priv->block_shift);
        ret = -EIO;
    }
#endif

errout:
    return ret;
}

static ssize_t gd5f_pageread(FAR struct spi_flash_dev_s *priv, off_t address, size_t nbytes,
                            bool spare, FAR uint8_t *buffer)
{
#ifdef CONFIG_GD5F_SPI_DEBUG
    ferr("address: %08lx nbytes: %d\n", (long)address, (int)nbytes);
#endif

    if ((spare && (nbytes != priv->spare_size)) || (nbytes > priv->page_size))
        return -EFAULT;

    gd5f_lock(priv->qspi);

#ifndef CONFIG_GD5F_SYNC_WRITE
    /* Wait for any preceding write or erase operation to complete. */

    (void)gd5f_waitcomplete_locked(priv);
#endif

    /* Make sure that writing is disabled */

    gd5f_wrdi_locked(priv->qspi);

    /* Read page data into cache */

    uint32_t page = address >> priv->page_shift;

    off_t commandaddr = (off_t)page;

    gd5f_command(priv->qspi, GD5F_RDPAGE, commandaddr, 3);
    priv->prev_instr = GD5F_RDPAGE;

    uint8_t status = gd5f_waitcomplete_locked(priv);
    if (status & GD5F_SR_ERR_ECC ) {
#ifdef CONFIG_GD5F_DEBUG
        uint8_t status1 = gd5f_read_status_locked(priv, GD5F_STATUS1);
        uint8_t status2 = gd5f_read_status_locked(priv, GD5F_STATUS2);
#endif

        gd5f_unlock(priv->qspi);

        spi_mark_badblock(priv, address >> priv->block_shift);

#ifdef CONFIG_GD5F_DEBUG
        uint16_t ecc_page;
        gd5f_read_last_ecc_address(priv->qspi, &ecc_page);
        ferr("ecc error block = %08x, page = %04x, last ecc page = %04x, status = (%02x, %02x, %02x)\n", address>>priv->block_shift, page, ecc_page, status1, status2, status);
#else
        ferr("ecc error block = %08x, page = %04x, status = %02x\n", address>>priv->block_shift, page, status);
#endif
        return -EIO;
    }

    /* Send "Read from Memory " instruction */

    off_t readaddr = 0;
    gd5f_data_read(priv->qspi, priv->page_buf, priv->page_size+priv->spare_size, readaddr, 2);

    priv->prev_instr = GD5F_FRDQIO;
    gd5f_unlock(priv->qspi);

    if (spare) {
        memcpy(buffer, (FAR uint8_t *)(priv->page_buf + priv->page_size), priv->spare_size);
    } else {
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

static ssize_t gd5f_pagewrite(FAR struct spi_flash_dev_s *priv, off_t address,
                          size_t nbytes, bool spare, FAR const uint8_t *buffer)
{
    int ret = nbytes;
    uint32_t page = address >> priv->page_shift;
    uint16_t column = address & ((1 << priv->page_shift) - 1);

#ifdef CONFIG_GD5F_SPI_DEBUG
    ferr("address: %08lx nwords: %d\n", (long)address, (int)nbytes);
#endif

    if ((spare && (nbytes > priv->spare_size)) || (nbytes > priv->page_size))
        return -EFAULT;

    gd5f_lock(priv->qspi);

#ifndef CONFIG_GD5F_SYNC_WRITE
    /* Wait for any preceding write or erase operation to complete. */

    (void)gd5f_waitcomplete_locked(priv);
#endif

    /* Enable write access to the FLASH */

    gd5f_wren_locked(priv->qspi);

    /* Align column to end of page to write spare */
    if (spare)
       column = priv->page_size;

    /* Send the "Page Program (GD5F_PPQ)" Command */

    gd5f_data_write(priv->qspi, buffer, nbytes, column, 2);
    priv->prev_instr = GD5F_PPQ;

    /* Program  Execute */

    off_t commandaddr = (off_t)page;
    gd5f_command(priv->qspi, GD5F_WRPAGE, commandaddr, 3);

#ifdef CONFIG_GD5F_SYNC_WRITE
    /* Wait for any preceding write or erase operation to complete. */
    uint8_t status = gd5f_waitcomplete_locked(priv);
#endif

    gd5f_wrdi_locked(priv->qspi);

    gd5f_unlock(priv->qspi);

#ifdef CONFIG_GD5F_SYNC_WRITE
    if (status & GD5F_SR_ERR_PROGRAM) {
        if(!spare) {
         spi_mark_badblock(priv, address >> priv->block_shift);
        }
#ifdef CONFIG_GD5F_DEBUG
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
        gd5f_blockerase(priv, block);
    }

    for (int block = 0; block < 64; block++) {
        gd5f_blockerase(priv, 0x400 + block);
    }

    FAR uint8_t *buffer;
    buffer = (FAR uint8_t *)kmm_malloc(GD5F_PAGE_SIZE);

    const int nbytes = GD5F_PAGE_SIZE;

    bool isfailed = false;
    for (int page = 0; page < 64 * 16; page++) {

        for (int index = 0; index < nbytes; index++)
            buffer[index] = index & 0xff;

        gd5f_pagewrite(priv, (page << GD5F_PAGE_SHIFT), nbytes, false, buffer);

        memset(buffer, 0, GD5F_PAGE_SIZE);
        gd5f_pageread(priv, (page << GD5F_PAGE_SHIFT), nbytes, false, buffer);

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

static void gd5f_quad_io_enable(FAR struct spi_flash_dev_s *priv)
{
    uint8_t status = gd5f_read_status_locked(priv, GD5F_STATUS2);
    ferr("status=%x\n",status);

    if (!(status & 0x01))
    {
      uint16_t addr = (GD5F_STATUS2 << 8) | (status | 0x01);
      ferr("addr=%x\n",addr);
      gd5f_command_write(priv->qspi, GD5F_WRSR, NULL, 0, addr, 2);

      status = gd5f_read_status_locked(priv, GD5F_STATUS2);
      ferr("status=%x\n",status);
    }
}

static void gd5f_unprotect(FAR struct spi_flash_dev_s *priv)
{
    gd5f_lock(priv->qspi);

#ifndef CONFIG_GD5F_SYNC_WRITE
    /* Wait for any preceding write or erase operation to complete. */

    (void)gd5f_waitcomplete_locked(priv);
#endif

    /* Send "Write enable (WREN)" */

    gd5f_wren_locked(priv->qspi);

    gd5f_write_status_locked(priv, GD5F_STATUS1, 0);

    gd5f_unlock(priv->qspi);
}

int gd5f_qspi_flash_initialize(FAR struct spi_flash_dev_s *priv)
{
    int ret;

    if (!priv) {
        return -EFAULT;
    }

    priv->block_size = GD5F_BLOCK_SIZE;
    priv->block_shift = GD5F_BLOCK_SHIFT;
    priv->page_size = GD5F_PAGE_SIZE;
    priv->page_shift = GD5F_PAGE_SHIFT;
    priv->spare_size = GD5F_SPARE_SIZE;

    priv->blockerase = gd5f_blockerase;
    priv->pageread = gd5f_pageread;
    priv->pagewrite = gd5f_pagewrite;

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

    gd5f_reset(priv->qspi);
    usleep(5000);

    /* Identify the FLASH chip and get its capacity */

    ret = gd5f_readid(priv);
    if (ret != OK) {
        /* Unrecognized! Discard all of that work we just did and return NULL */

        ferr("Unrecognized QSPI device\n");
        goto errout;
    }

    /*Enable QE, then the quad io operations can be executed.*/

    gd5f_quad_io_enable(priv);

    /* Make sure that the FLASH is unprotected so that we can write into it */

    gd5f_unprotect(priv);

    return OK;

errout:
    QSPI_FREE(priv->qspi, priv->cmdbuf);

    return -EFAULT;
}

