/************************************************************************************
 * drivers/mtd/w25.c
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

#define CONFIG_W25_SYNC_WRITE
//#define CONFIG_W25_DEBUG
//#define CONFIG_W25_SPI_DEBUG

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
#define W25_PP                     0x02    /* Program page                          */
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
#define W25_SECTOR_SHIFT           15        /* Sector size 1 << 15 = 32768 bytes */
#define W25_SECTOR_SIZE            (1 << 15) /* Sector size 1 << 15 = 32768 bytes */
#define W25_PAGE_SHIFT             11        /* Sector size 1 << 11 = 2048b */
#define W25_PAGE_SIZE              (1 << 11)  /* Sector size 1 << 11 = 2048b */
#define W25_DIE_SHIFT              27
#define W25_SPARE_SIZE             64        /* 16*4 bytes spare size */

static void w25_lock(FAR struct spi_dev_s *spi)
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

static void w25_unlock(FAR struct spi_dev_s *spi)
{
  (void)SPI_LOCK(spi, false);
}

static inline void w25_wren_locked(FAR struct spi_dev_s *spi)
{
  /* Select this FLASH part */

  SPI_SELECT(spi, SPIDEV_FLASH(0), true);

  /* Send "Write Enable (WREN)" command */

  (void)SPI_SEND(spi, W25_WREN);

  /* Deselect the FLASH */

  SPI_SELECT(spi, SPIDEV_FLASH(0), false);
}

static inline void w25_wrdi_locked(FAR struct spi_dev_s *spi)
{
  /* Select this FLASH part */

  SPI_SELECT(spi, SPIDEV_FLASH(0), true);

  /* Send "Write Disable (WRDI)" command */

  (void)SPI_SEND(spi, W25_WRDI);

  /* Deselect the FLASH */

  SPI_SELECT(spi, SPIDEV_FLASH(0), false);
}

static void w25_select_die_locked(FAR struct spi_dev_s *spi, uint8_t die)
{
  SPI_SELECT(spi, SPIDEV_FLASH(0), true);

  (void)SPI_SEND(spi, W25_DIE_SEL);
  (void)SPI_SEND(spi, die);

  SPI_SELECT(spi, SPIDEV_FLASH(0), false);
}

static void w25_reset(FAR struct spi_dev_s *spi)
{
  w25_lock(spi);

  SPI_SELECT(spi, SPIDEV_FLASH(0), true);

  (void)SPI_SEND(spi, W25_RESET);

  SPI_SELECT(spi, SPIDEV_FLASH(0), false);

  w25_unlock(spi);
}

static uint8_t w25_read_reg_locked(FAR struct spi_dev_s *spi, uint8_t regaddr)
{
  SPI_SELECT(spi, SPIDEV_FLASH(0), true);
  (void)SPI_SEND(spi, W25_RDSR);
  (void)SPI_SEND(spi, regaddr);

  uint8_t status = SPI_SEND(spi, W25_DUMMY);

  SPI_SELECT(spi, SPIDEV_FLASH(0), false);

  return status;
}

#ifdef CONFIG_W25_DEBUG
static void w25_read_last_ecc_address(FAR struct spi_dev_s *spi, uint16_t *page)
{
  uint8_t buf[2];

  w25_lock(spi);

  SPI_SELECT(spi, SPIDEV_FLASH(0), true);

  /* Select die */
  (void)SPI_SEND(spi, W25_ECC_ADDR);
  (void)SPI_SEND(spi, W25_DUMMY);

  SPI_RECVBLOCK(spi, buf, 2);

  SPI_SELECT(spi, SPIDEV_FLASH(0), false);

  w25_unlock(spi);

  *page = ((uint16_t)buf[0] << 8) + (uint16_t)buf[1];
}
#endif

static inline int w25_readid(struct spi_flash_dev_s *priv)
{
  uint16_t manufacturer;
  uint16_t memory;
  uint16_t capacity;
  int ret;

  finfo("priv: %p\n", priv);

  w25_lock(priv->spi);

#ifndef CONFIG_W25_SYNC_WRITE
  /* Wait for any preceding write or erase operation to complete. */

  (void)w25_waitcomplete_locked(priv);
#endif

  /* Select this FLASH part. */

  SPI_SELECT(priv->spi, SPIDEV_FLASH(0), true);

  /* Send the "Read ID (RDID)" command and read the first three ID bytes */

  (void)SPI_SEND(priv->spi, W25_JEDEC_ID);
  SPI_SEND(priv->spi, W25_DUMMY);
  manufacturer = SPI_SEND(priv->spi, W25_DUMMY);
  memory       = SPI_SEND(priv->spi, W25_DUMMY);
  capacity     = SPI_SEND(priv->spi, W25_DUMMY);

  /* Deselect the FLASH and unlock the bus */

  SPI_SELECT(priv->spi, SPIDEV_FLASH(0), false);
  w25_unlock(priv->spi);

  finfo("manufacturer: %02x memory: %02x capacity: %02x\n",
        manufacturer, memory, capacity);

  /* Check for a valid manufacturer and memory type */

  if (manufacturer == W25_JEDEC_MANUFACTURER &&
      (memory == W25X_JEDEC_MEMORY_TYPE   ||
       memory == W25Q_JEDEC_MEMORY_TYPE_A ||
       memory == W25Q_JEDEC_MEMORY_TYPE_B ||
       memory == W25Q_JEDEC_MEMORY_TYPE_C ||
       memory == W25Q_JEDEC_MEMORY_TYPE_D))
    {
      /* Okay.. is it a FLASH capacity that we understand? If so, save
       * the FLASH capacity.
       */

      /* 8M-bit / 1M-byte
       *
       * W25Q80BV
       */

      if (capacity == W25_JEDEC_CAPACITY_8MBIT)
        {
           priv->nblocks = NBLOCKS_8MBIT;
        }

      /* 16M-bit / 2M-byte (2,097,152)
       *
       * W24X16, W25Q16BV, W25Q16CL, W25Q16CV, W25Q16DW
       */

      else if (capacity == W25_JEDEC_CAPACITY_16MBIT)
        {
           priv->nblocks = NBLOCKS_16MBIT;
        }

      /* 32M-bit / M-byte (4,194,304)
       *
       * W25X32, W25Q32BV, W25Q32DW
       */

      else if (capacity == W25_JEDEC_CAPACITY_32MBIT)
        {
           priv->nblocks = NBLOCKS_32MBIT;
        }

      /* 64M-bit / 8M-byte (8,388,608)
       *
       * W25X64,  W25Q64BV, W25Q64CV, W25Q64DW
       */

      else if (capacity == W25_JEDEC_CAPACITY_64MBIT)
        {
           priv->nblocks = NBLOCKS_64MBIT;
        }

      /* 128M-bit / 16M-byte (16,777,216)
       *
       * W25Q128BV
       */

      else if (capacity == W25_JEDEC_CAPACITY_128MBIT)
        {
           priv->nblocks = NBLOCKS_128MBIT;
        }
      else if (capacity == W25_JEDEC_CAPACITY_2048MBIT)
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

static void w25_unprotect(FAR struct spi_flash_dev_s *priv, int die)
{
  w25_lock(priv->spi);

#ifndef CONFIG_W25_SYNC_WRITE
  /* Wait for any preceding write or erase operation to complete. */

  (void)w25_waitcomplete_locked(priv);
#endif

  w25_select_die_locked(priv->spi, die);

  /* Send "Write enable (WREN)" */

  w25_wren_locked(priv->spi);

  /* Select this FLASH part */

  SPI_SELECT(priv->spi, SPIDEV_FLASH(0), true);

  /* Send "Write enable status (EWSR)" */

  SPI_SEND(priv->spi, W25_WRSR);
  SPI_SEND(priv->spi, 0xA0);

  /* Following by the new status value */

  SPI_SEND(priv->spi, 0);

  /* Deselect the FLASH and unlock the bus */

  SPI_SELECT(priv->spi, SPIDEV_FLASH(0), false);

  w25_unlock(priv->spi);
}

static uint8_t w25_waitcomplete_locked(struct spi_flash_dev_s *priv)
{
  uint8_t status;

  /* Loop as long as the memory is busy with a write cycle. Device sets BUSY
   * flag to a 1 state whhen previous write or erase command is still executing
   * and during this time, device will ignore further instructions except for
   * "Read Status Register" and "Erase/Program Suspend" instructions.
   */

  do
    {
      status = w25_read_reg_locked(priv->spi, 0xC0);

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
          w25_unlock(priv->spi);
          usleep(1000);
          w25_lock(priv->spi);
        }
#endif
      usleep(500);
    }
  while (1);

  if (status & W25_ERR_ERASE) {
    w25_unlock(priv->spi);

#ifdef CONFIG_W25_DEBUG
    ferr("erase error block = %08x\n", priv->lastaddr >> priv->block_shift);
#endif
    spi_mark_badblock(priv, priv->lastaddr >> priv->block_shift);

    w25_lock(priv->spi);
  }

  if (status & W25_ERR_PROGRAM) {
    w25_unlock(priv->spi);

#ifdef CONFIG_W25_DEBUG
    ferr("program error block = %08x\n", priv->lastaddr >> priv->block_shift);
#endif
    spi_mark_badblock(priv, priv->lastaddr >> priv->block_shift);

    w25_lock(priv->spi);
  }

  return status;
}

static int w25_blockerase(struct spi_flash_dev_s *priv, size_t block)
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

  w25_lock(priv->spi);

#ifndef CONFIG_W25_SYNC_WRITE
  /* Wait for any preceding write or erase operation to complete. */

  (void)w25_waitcomplete_locked(priv);
#endif

  priv->lastaddr = address;

  w25_select_die_locked(priv->spi, (uint8_t)(address>>W25_DIE_SHIFT));

  /* Send write enable instruction */

  w25_wren_locked(priv->spi);

  /* Select this FLASH part */

  SPI_SELECT(priv->spi, SPIDEV_FLASH(0), true);

  /* Send the "Sector Erase (SE)" instruction */

  (void)SPI_SEND(priv->spi, W25_BE);
  priv->prev_instr = W25_BE;

  /* Send the sector address high byte first. Only the most significant bits (those
   * corresponding to the sector) have any meaning.
   */
  uint16_t page = address >> priv->page_shift;

  (void)SPI_SEND(priv->spi, W25_DUMMY);
  (void)SPI_SEND(priv->spi, (page >> 8) & 0xff);
  (void)SPI_SEND(priv->spi, page & 0xff);

  /* Deselect the FLASH */

  SPI_SELECT(priv->spi, SPIDEV_FLASH(0), false);

#ifdef CONFIG_W25_SYNC_WRITE
  /* Wait for any preceding write or erase operation to complete. */
  uint8_t status = w25_waitcomplete_locked(priv);
#endif

  w25_wrdi_locked(priv->spi);

  w25_unlock(priv->spi);

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

  w25_lock(priv->spi);

#ifndef CONFIG_W25_SYNC_WRITE
  /* Wait for any preceding write or erase operation to complete. */

  (void)w25_waitcomplete_locked(priv);
#endif

  w25_select_die_locked(priv->spi, (uint8_t)(address>>W25_DIE_SHIFT));

  /* Make sure that writing is disabled */

  w25_wrdi_locked(priv->spi);

  /* Read page data into cache */

  uint16_t page = address >> priv->page_shift;

  SPI_SELECT(priv->spi, SPIDEV_FLASH(0), true);
  (void)SPI_SEND(priv->spi, W25_RDPAGE);
  priv->prev_instr = W25_RDPAGE;

  (void)SPI_SEND(priv->spi, W25_DUMMY);
  (void)SPI_SEND(priv->spi, (page >> 8) & 0xff);
  (void)SPI_SEND(priv->spi, page & 0xff);
  SPI_SELECT(priv->spi, SPIDEV_FLASH(0), false);

  uint8_t status = w25_waitcomplete_locked(priv);
  if (status & W25_ERR_ECC ) {
#ifdef CONFIG_W25_DEBUG
    uint8_t status2 = w25_read_reg_locked(priv->spi, 0xA0);
    uint8_t status3 = w25_read_reg_locked(priv->spi, 0xB0);
#endif

    w25_unlock(priv->spi);

    spi_mark_badblock(priv, address >> priv->block_shift);

#ifdef CONFIG_W25_DEBUG
    uint16_t ecc_page;
    w25_read_last_ecc_address(priv->spi, &ecc_page);
    ferr("ecc error block = %08x, page = %04x, last ecc page = %04x, status = (%02x, %02x, %02x)\n", address>>priv->block_shift, page, ecc_page, status2, status3, status);
#else
    ferr("ecc error block = %08x, status = %02x\n", address>>priv->block_shift, page, status);
#endif
    return -EIO;
  }

  SPI_SELECT(priv->spi, SPIDEV_FLASH(0), true);

  /* Send "Read from Memory " instruction */

#ifdef CONFIG_W25_SLOWREAD
  (void)SPI_SEND(priv->spi, W25_RDDATA);
  priv->prev_instr = W25_RDDATA;
#else
  (void)SPI_SEND(priv->spi, W25_FRD);
  priv->prev_instr = W25_FRD;
#endif

  /* Send the address high byte first. */

  (void)SPI_SEND(priv->spi, 0);
  (void)SPI_SEND(priv->spi, 0);
  (void)SPI_SEND(priv->spi, W25_DUMMY);

  /* Then read all of the requested bytes */

  SPI_RECVBLOCK(priv->spi, priv->page_buf, priv->page_size+priv->spare_size);

  /* Deselect the FLASH */

  SPI_SELECT(priv->spi, SPIDEV_FLASH(0), false);

  w25_unlock(priv->spi);

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

static ssize_t w25_pagewrite(struct spi_flash_dev_s *priv, off_t address,
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

  w25_lock(priv->spi);

#ifndef CONFIG_W25_SYNC_WRITE
  /* Wait for any preceding write or erase operation to complete. */
  (void)w25_waitcomplete_locked(priv);
#endif

  w25_select_die_locked(priv->spi, (uint8_t)(address>>W25_DIE_SHIFT));

  /* Enable write access to the FLASH */

  w25_wren_locked(priv->spi);

  /* Select this FLASH part */

  SPI_SELECT(priv->spi, SPIDEV_FLASH(0), true);

  /* Send the "Page Program (W25_PP)" Command */

  SPI_SEND(priv->spi, W25_PP);
  priv->prev_instr = W25_PP;

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

  (void)SPI_SEND(priv->spi, W25_WRPAGE);
  priv->prev_instr = W25_WRPAGE;

  (void)SPI_SEND(priv->spi, W25_DUMMY);
  (void)SPI_SEND(priv->spi, (page >> 8) & 0xff);
  (void)SPI_SEND(priv->spi, page & 0xff);

  /* Deselect the FLASH and setup for the next pass through the loop */

  SPI_SELECT(priv->spi, SPIDEV_FLASH(0), false);

#ifdef CONFIG_W25_SYNC_WRITE
  /* Wait for any preceding write or erase operation to complete. */
  uint8_t status = w25_waitcomplete_locked(priv);
#endif

  w25_wrdi_locked(priv->spi);

  w25_unlock(priv->spi);

#ifdef CONFIG_W25_SYNC_WRITE
  if (status & W25_ERR_PROGRAM) {
    spi_mark_badblock(priv, address >> priv->block_shift);
#ifdef CONFIG_W25_DEBUG
    ferr("program error block = %08x\n", address >> priv->block_shift);
#endif
    ret = -EIO;
  }
#endif

  return ret;
}

FAR int spi_flash_initialize(FAR struct spi_flash_dev_s *priv)
{
  int ret;

  if (!priv)
    {
      return -EFAULT;
    }

  priv->block_size = W25_BLOCK_SIZE;
  priv->block_shift = W25_BLOCK_SHIFT;
  priv->sector_size = W25_SECTOR_SIZE;
  priv->sector_shift = W25_SECTOR_SHIFT;
  priv->page_size = W25_PAGE_SIZE;
  priv->page_shift = W25_PAGE_SHIFT;
  priv->spare_size = W25_SPARE_SIZE;

  priv->blockerase = w25_blockerase;
  priv->pageread = w25_pageread;
  priv->pagewrite = w25_pagewrite;

  SPI_SETFREQUENCY(priv->spi, 104000000);
  SPI_SETBITS(priv->spi, 8);
  SPI_SETMODE(priv->spi, SPIDEV_MODE0);

  /* Deselect the FLASH */

  SPI_SELECT(priv->spi, SPIDEV_FLASH(0), false);

  w25_reset(priv->spi);
  usleep(1000);

  /* Identify the FLASH chip and get its capacity */

  ret = w25_readid(priv);
  if (ret != OK)
    {
      /* Unrecognized! Discard all of that work we just did and return NULL */

      ferr("ERROR: Unrecognized\n");
      return -EFAULT;
    }

  /* Make sure that the FLASH is unprotected so that we can write into it */

  w25_unprotect(priv, 0);
  w25_unprotect(priv, 1);

  return OK;
}

