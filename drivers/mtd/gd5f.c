/************************************************************************************
 * drivers/mtd/gd5f.c
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
#include <nuttx/spi/spi.h>
#include <nuttx/mtd/mtd.h>

#include "spi_flash.h"

#define CONFIG_GD5F_SYNC_WRITE
//#define CONFIG_GD5F_DEBUG
//#define CONFIG_GD5F_SPI_DEBUG


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
#define GD5F_PP                     0x02    /* Program page                          */
#define GD5F_BE                     0xd8    /* Block Erase (64KB)                    */
#define GD5F_JEDEC_ID               0x9f    /* JEDEC ID read                         */
#define GD5F_RDPAGE                 0x13    /* Read page                             */
#define GD5F_WRPAGE                 0x10    /* Program execute in page address       */
#define GD5F_RESET                  0xff    /* Reset                                 */

/* GD5F Registers ********************************************************************/
/* Read ID (RDID) register values */

#define GD5F_MANUFACTURER           0xC8   /* Winbond Serial Flash */
#define GD5F2GQ4U_DEVID             0xD2   /* GD5F2GQ4U device ID (0xD2) */
#define GD5F2GQ4R_DEVID             0xC2   /* GD5F2GQ4U device ID (0xC2) */

#define NBLOCKS_2048MBIT            2048   /* 2048 blocks x 128K bytes/block = 256Mb */

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
#define GD5F_DIE_SHIFT              27
#define GD5F_SPARE_SIZE             64        /* 16*4 bytes spare size */

static void gd5f_lock(FAR struct spi_dev_s *spi)
{
  /* On SPI busses where there are multiple devices, it will be necessary to
   * lock SPI to have exclusive access to the busses for a sequence of
   * transfers.  The bus should be locked before the chip is selected.
   *
   * This is a blocking call and will not return until we have exclusiv access to
   * the SPI buss.  We will retain that exclusive access until the bus is unlocked.
   */

  (void)SPI_LOCK(spi, true);
}

static void gd5f_unlock(FAR struct spi_dev_s *spi)
{
  (void)SPI_LOCK(spi, false);
}

static inline void gd5f_wren_locked(FAR struct spi_dev_s *spi)
{
  /* Select this FLASH part */

  SPI_SELECT(spi, SPIDEV_FLASH(0), true);

  /* Send "Write Enable (WREN)" command */

  (void)SPI_SEND(spi, GD5F_WREN);

  /* Deselect the FLASH */

  SPI_SELECT(spi, SPIDEV_FLASH(0), false);
}

static inline void gd5f_wrdi_locked(FAR struct spi_dev_s *spi)
{
  /* Select this FLASH part */

  SPI_SELECT(spi, SPIDEV_FLASH(0), true);

  /* Send "Write Disable (WRDI)" command */

  (void)SPI_SEND(spi, GD5F_WRDI);

  /* Deselect the FLASH */

  SPI_SELECT(spi, SPIDEV_FLASH(0), false);
}


static void gd5f_reset(FAR struct spi_dev_s *spi)
{
  gd5f_lock(spi);

  SPI_SELECT(spi, SPIDEV_FLASH(0), true);

  (void)SPI_SEND(spi, GD5F_RESET);

  SPI_SELECT(spi, SPIDEV_FLASH(0), false);

  gd5f_unlock(spi);
}

static uint8_t gd5f_read_reg_locked(FAR struct spi_dev_s *spi, uint8_t regaddr)
{
  SPI_SELECT(spi, SPIDEV_FLASH(0), true);
  (void)SPI_SEND(spi, GD5F_RDSR);
  (void)SPI_SEND(spi, regaddr);

  uint8_t status = SPI_SEND(spi, GD5F_DUMMY);

  SPI_SELECT(spi, SPIDEV_FLASH(0), false);

  return status;
}


static inline int gd5f_readid(struct spi_flash_dev_s *priv)
{
  uint16_t manufacturer;
  uint16_t memory;
  uint16_t capacity;
  int ret;

  ferr("priv: %p\n", priv);

  gd5f_lock(priv->spi);

#ifndef CONFIG_GD5F_SYNC_WRITE
  /* Wait for any preceding write or erase operation to complete. */

  (void)gd5f_waitcomplete_locked(priv);
#endif

  /* Select this FLASH part. */

  SPI_SELECT(priv->spi, SPIDEV_FLASH(0), true);

  /* Send the "Read ID (RDID)" command and read the first three ID bytes */

  (void)SPI_SEND(priv->spi, GD5F_JEDEC_ID);
  SPI_SEND(priv->spi, GD5F_DUMMY);
  manufacturer = SPI_SEND(priv->spi, GD5F_DUMMY);
  memory       = SPI_SEND(priv->spi, GD5F_DUMMY);
  capacity     = 0x00;

  /* Deselect the FLASH and unlock the bus */

  SPI_SELECT(priv->spi, SPIDEV_FLASH(0), false);
  gd5f_unlock(priv->spi);

  priv->manufacturer = manufacturer;
  priv->memory = memory;
  priv->capacity = capacity;

  ferr("manufacturer: %02x memory: %02x capacity: %02x\n",
        manufacturer, memory, capacity);

  /* Check for a valid manufacturer and memory type */

  if (manufacturer == GD5F_MANUFACTURER)
  {
    /* Okay.. is it a FLASH capacity that we understand? If so, save
      * the FLASH capacity.
      */

    /* 2048M-bit / 256M-byte
      * GD5F2GQ4U
      * 3.3V
      */
    if (memory == GD5F2GQ4U_DEVID)
    {
      priv->nblocks = NBLOCKS_2048MBIT;
    }

    /* 2048M-bit / 256M-byte
      * GD5F2GQ4R
      * 1.8V
      */
    else if (memory == GD5F2GQ4U_DEVID)
    {
      priv->nblocks = NBLOCKS_2048MBIT;
    }
    else
    {
      /* Nope.. we don't understand this capacity. */
      ret =  -ENODEV;
      goto errout;
    }

    ret = OK;
    goto errout;
  }

  /* We don't understand the manufacturer or the memory type */

  ret = -ENODEV;

errout:
  return ret;
}

static void gd5f_unprotect(FAR struct spi_flash_dev_s *priv)
{
  gd5f_lock(priv->spi);

#ifndef CONFIG_GD5F_SYNC_WRITE
  /* Wait for any preceding write or erase operation to complete. */

  (void)gd5f_waitcomplete_locked(priv);
#endif

  /* Send "Write enable (WREN)" */

  gd5f_wren_locked(priv->spi);

  /* Select this FLASH part */

  SPI_SELECT(priv->spi, SPIDEV_FLASH(0), true);

  /* Send "Write enable status (EWSR)" */

  SPI_SEND(priv->spi, GD5F_WRSR);
  SPI_SEND(priv->spi, 0xA0);

  /* Following by the new status value */

  SPI_SEND(priv->spi, 0);

  /* Deselect the FLASH and unlock the bus */

  SPI_SELECT(priv->spi, SPIDEV_FLASH(0), false);

  gd5f_unlock(priv->spi);
}

static uint8_t gd5f_waitcomplete_locked(struct spi_flash_dev_s *priv)
{
  uint8_t status;

  /* Loop as long as the memory is busy with a write cycle. Device sets BUSY
   * flag to a 1 state whhen previous write or erase command is still executing
   * and during this time, device will ignore further instructions except for
   * "Read Status Register" and "Erase/Program Suspend" instructions.
   */

  do
    {
      status = gd5f_read_reg_locked(priv->spi, 0xC0);

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
          gd5f_unlock(priv->spi);
          usleep(1000);
          gd5f_lock(priv->spi);
        }
#endif
      usleep(500);
    }
  while (1);

  if (status & GD5F_SR_ERR_ERASE) {
    gd5f_unlock(priv->spi);

#ifdef CONFIG_GD5F_DEBUG
    ferr("erase error block = %08x\n", priv->lastaddr >> priv->block_shift);
#endif
    spi_mark_badblock(priv, priv->lastaddr >> priv->block_shift);

    gd5f_lock(priv->spi);
  }

  if (status & GD5F_SR_ERR_PROGRAM) {
    gd5f_unlock(priv->spi);

#ifdef CONFIG_GD5F_DEBUG
    ferr("program error block = %08x\n", priv->lastaddr >> priv->block_shift);
#endif
    spi_mark_badblock(priv, priv->lastaddr >> priv->block_shift);

    gd5f_lock(priv->spi);
  }

  return status;
}

static int gd5f_blockerase(struct spi_flash_dev_s *priv, size_t block)
{
  int ret = OK;
  off_t address = block << priv->block_shift;

#ifdef CONFIG_GD5F_SPI_DEBUG
  ferr("block: %08lx\n", (long)block);
#endif

  if (spi_is_badblock(priv, block)) {
    ferr("bad block %d\n", block);
    ret = -EIO;
    goto errout;
  }

  /* Lock and configure the SPI bus */

  gd5f_lock(priv->spi);

#ifndef CONFIG_GD5F_SYNC_WRITE
  /* Wait for any preceding write or erase operation to complete. */

  (void)gd5f_waitcomplete_locked(priv);
#endif

  priv->lastaddr = address;

  /* Send write enable instruction */

  gd5f_wren_locked(priv->spi);

  /* Select this FLASH part */

  SPI_SELECT(priv->spi, SPIDEV_FLASH(0), true);

  /* Send the "Sector Erase (SE)" instruction */

  (void)SPI_SEND(priv->spi, GD5F_BE);
  priv->prev_instr = GD5F_BE;

  /* Send the sector address high byte first. Only the most significant bits (those
   * corresponding to the sector) have any meaning.
   */
  uint32_t page = address >> priv->page_shift;

  (void)SPI_SEND(priv->spi, (page >> 16) & 0xff);
  (void)SPI_SEND(priv->spi, (page >> 8) & 0xff);
  (void)SPI_SEND(priv->spi, page & 0xff);

  /* Deselect the FLASH */

  SPI_SELECT(priv->spi, SPIDEV_FLASH(0), false);

#ifdef CONFIG_GD5F_SYNC_WRITE
  /* Wait for any preceding write or erase operation to complete. */
  uint8_t status = gd5f_waitcomplete_locked(priv);
#endif

  gd5f_wrdi_locked(priv->spi);

  gd5f_unlock(priv->spi);

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
  
  gd5f_lock(priv->spi);

#ifndef CONFIG_GD5F_SYNC_WRITE
  /* Wait for any preceding write or erase operation to complete. */

  (void)gd5f_waitcomplete_locked(priv);
#endif

  /* Make sure that writing is disabled */

  gd5f_wrdi_locked(priv->spi);

  /* Read page data into cache */

  uint32_t page = address >> priv->page_shift;

  SPI_SELECT(priv->spi, SPIDEV_FLASH(0), true);
  (void)SPI_SEND(priv->spi, GD5F_RDPAGE);
  priv->prev_instr = GD5F_RDPAGE;

  (void)SPI_SEND(priv->spi, (page >> 16) & 0xff);
  (void)SPI_SEND(priv->spi, (page >> 8) & 0xff);
  (void)SPI_SEND(priv->spi, page & 0xff);
  SPI_SELECT(priv->spi, SPIDEV_FLASH(0), false);

  uint8_t status = gd5f_waitcomplete_locked(priv);
  if (status & GD5F_SR_ERR_ECC ) {
#ifdef CONFIG_GD5F_DEBUG
    uint8_t status2 = gd5f_read_reg_locked(priv->spi, 0xA0);
    uint8_t status3 = gd5f_read_reg_locked(priv->spi, 0xB0);
#endif

    gd5f_unlock(priv->spi);

    spi_mark_badblock(priv, address >> priv->block_shift);

#ifdef CONFIG_GD5F_DEBUG
    ferr("ecc error block = %08x, page = %04x, status = (%02x, %02x, %02x)\n", address>>priv->block_shift, page,  status2, status3, status);
#else
    ferr("ecc error block = %08x, page = %04x, status = %02x\n", address>>priv->block_shift, page, status);
#endif
    return -EIO;
  }

  SPI_SELECT(priv->spi, SPIDEV_FLASH(0), true);

  /* Send "Read from Memory " instruction */

#ifdef CONFIG_GD5F_SLOWREAD
  (void)SPI_SEND(priv->spi, GD5F_RDDATA);
  priv->prev_instr = GD5F_RDDATA;
#else
  (void)SPI_SEND(priv->spi, GD5F_FRD);
  priv->prev_instr = GD5F_FRD;
#endif

  /* Send the address high byte first. */

  (void)SPI_SEND(priv->spi, 0);
  (void)SPI_SEND(priv->spi, 0);
  (void)SPI_SEND(priv->spi, GD5F_DUMMY);

  /* Then read all of the requested bytes */

  SPI_RECVBLOCK(priv->spi, priv->page_buf, priv->page_size+priv->spare_size);

  /* Deselect the FLASH */

  SPI_SELECT(priv->spi, SPIDEV_FLASH(0), false);

  gd5f_unlock(priv->spi);

  if (spare)
    memcpy(buffer, (FAR uint8_t *)(priv->page_buf+priv->page_size), priv->spare_size);
  else {
    if ((priv->page_buf[priv->page_size] != SPI_ERASED_STATE) ||
        (priv->page_buf[priv->page_size+1] != SPI_ERASED_STATE)) {
      ferr("bad block = %08x, page = %04x\n", address>>priv->block_shift, page);
      /* bad block */
      return -EIO;
    }

    uint16_t column = address & ((1 << priv->page_shift) - 1);
    memcpy(buffer, (FAR uint8_t *)(priv->page_buf+column), nbytes);
  }

  return nbytes;
}

static ssize_t gd5f_pagewrite(struct spi_flash_dev_s *priv, off_t address,
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


  gd5f_lock(priv->spi);

#ifndef CONFIG_GD5F_SYNC_WRITE
  /* Wait for any preceding write or erase operation to complete. */
  (void)gd5f_waitcomplete_locked(priv);
#endif

  /* Enable write access to the FLASH */

  gd5f_wren_locked(priv->spi);

  /* Select this FLASH part */

  SPI_SELECT(priv->spi, SPIDEV_FLASH(0), true);

  /* Send the "Page Program (GD5F_PP)" Command */

  SPI_SEND(priv->spi, GD5F_PP);
  priv->prev_instr = GD5F_PP;

  /* Align column to end of page to write spare */
  if (spare)
    column = priv->page_size;

  /* Send the address high byte first. */

  (void)SPI_SEND(priv->spi, (column >> 8) & 0xff);
  (void)SPI_SEND(priv->spi, column & 0xff);

  /* Then send the page of data */

  SPI_SNDBLOCK(priv->spi, buffer, nbytes);
  SPI_SELECT(priv->spi, SPIDEV_FLASH(0), false);

  SPI_SELECT(priv->spi, SPIDEV_FLASH(0), true);

  /* Program  Execute */

  (void)SPI_SEND(priv->spi, GD5F_WRPAGE);
  priv->prev_instr = GD5F_WRPAGE;

  (void)SPI_SEND(priv->spi, (page >> 16) & 0xff);
  (void)SPI_SEND(priv->spi, (page >> 8) & 0xff);
  (void)SPI_SEND(priv->spi, page & 0xff);

  /* Deselect the FLASH and setup for the next pass through the loop */

  SPI_SELECT(priv->spi, SPIDEV_FLASH(0), false);

#ifdef CONFIG_GD5F_SYNC_WRITE
  /* Wait for any preceding write or erase operation to complete. */
  uint8_t status = gd5f_waitcomplete_locked(priv);
#endif

  gd5f_wrdi_locked(priv->spi);

  gd5f_unlock(priv->spi);

#ifdef CONFIG_GD5F_SYNC_WRITE
  if (status & GD5F_SR_ERR_PROGRAM) {
    spi_mark_badblock(priv, address >> priv->block_shift);
#ifdef CONFIG_GD5F_DEBUG
    ferr("program error block = %08x\n", address >> priv->block_shift);
#endif
    ret = -EIO;
  }
#endif

  return ret;
}

int spi_flash_initialize(FAR struct spi_flash_dev_s *priv)
{
  int ret;
  if (!priv)
    {
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

  SPI_SETFREQUENCY(priv->spi, 104000000);
  SPI_SETBITS(priv->spi, 8);
  SPI_SETMODE(priv->spi, SPIDEV_MODE0);

  /* Deselect the FLASH */

  SPI_SELECT(priv->spi, SPIDEV_FLASH(0), false);

  gd5f_reset(priv->spi);
  usleep(1000);

  /* Identify the FLASH chip and get its capacity */

  ret = gd5f_readid(priv);
  if (ret != OK)
    {
      /* Unrecognized! Discard all of that work we just did and return NULL */

      ferr("ERROR: Unrecognized\n");
      return -EFAULT;
    }

  /* Make sure that the FLASH is unprotected so that we can write into it */

  gd5f_unprotect(priv);

  return OK;
}

