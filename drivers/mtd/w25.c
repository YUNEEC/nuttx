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

#define CONFIG_W25_SYNC_WRITE
//#define CONFIG_W25_DEBUG
//#define CONFIG_W25_SPI_DEBUG
//#define CONFIG_W25_CACHE_DEBUG
//#define CONFIG_W25_API_DEBUG

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* Configuration ********************************************************************/
/* Per the data sheet, the W25 parts can be driven with either SPI mode 0 (CPOL=0
 * and CPHA=0) or mode 3 (CPOL=1 and CPHA=1). But I have heard that other devices
 * can operate in mode 0 or 1.  So you may need to specify CONFIG_W25_SPIMODE to
 * select the best mode for your device.  If CONFIG_W25_SPIMODE is not defined,
 * mode 0 will be used.
 */

#ifndef CONFIG_W25_SPIMODE
#  define CONFIG_W25_SPIMODE SPIDEV_MODE0
#endif

/* SPI Frequency.  May be up to 25MHz. */

#ifndef CONFIG_W25_SPIFREQUENCY
#  define CONFIG_W25_SPIFREQUENCY 20000000
#endif

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
#define W25_PAGE_SHIFT             11        /* Sector size 1 << 11 = 2048b */
#define W25_PAGE_SIZE              (1 << 11)  /* Sector size 1 << 11 = 2048b */
#define W25_DIE_SHIFT              27
#define W25_SPARE_SIZE             64        /* 16*4 bytes spare size */

#define W25_SECTOR_SHIFT           15        /* Sector size 1 << 15 = 32768 bytes */
#define W25_SECTOR_SIZE            (1 << 15) /* Sector size 1 << 15 = 32768 bytes */

#define W25_ERASED_STATE           0xff      /* State of FLASH when erased */

/* Cache flags */

#define W25_CACHE_VALID            (1 << 0)  /* 1=Cache has valid data */
#define W25_CACHE_DIRTY            (1 << 1)  /* 1=Cache is dirty */
#define W25_CACHE_ERASED           (1 << 2)  /* 1=Backing FLASH is erased */

#define IS_VALID(p)                ((((p)->flags) & W25_CACHE_VALID) != 0)
#define IS_DIRTY(p)                ((((p)->flags) & W25_CACHE_DIRTY) != 0)
#define IS_ERASED(p)               ((((p)->flags) & W25_CACHE_ERASED) != 0)

#define SET_VALID(p)               do { (p)->flags |= W25_CACHE_VALID; } while (0)
#define SET_DIRTY(p)               do { (p)->flags |= W25_CACHE_DIRTY; } while (0)
#define SET_ERASED(p)              do { (p)->flags |= W25_CACHE_ERASED; } while (0)

#define CLR_VALID(p)               do { (p)->flags &= ~W25_CACHE_VALID; } while (0)
#define CLR_DIRTY(p)               do { (p)->flags &= ~W25_CACHE_DIRTY; } while (0)
#define CLR_ERASED(p)              do { (p)->flags &= ~W25_CACHE_ERASED; } while (0)

/************************************************************************************
 * Private Types
 ************************************************************************************/

/* This type represents the state of the MTD device.  The struct mtd_dev_s must
 * appear at the beginning of the definition so that you can freely cast between
 * pointers to struct mtd_dev_s and struct w25_dev_s.
 */

struct w25_dev_s
{
  struct mtd_dev_s      mtd;         /* MTD interface */
  FAR struct spi_dev_s *spi;         /* Saved SPI interface instance */
  uint16_t              nblocks;    /* Number of erase blocks */
  uint8_t               prev_instr;  /* Previous instruction given to W25 device */

#if !defined(CONFIG_W25_READONLY)
  uint8_t               flags;       /* Buffered sector flags */
  uint16_t              esectno;     /* Erase sector number in the cache*/
  FAR uint8_t           *block;      /* Allocated block data */
#endif

  uint32_t              lastaddr;    /* Last erase or program address */
  FAR uint8_t           *page_buf;   /* page buffer */
};

/************************************************************************************
 * Private Function Prototypes
 ************************************************************************************/

/* Helpers */

static void w25_lock(FAR struct spi_dev_s *spi);
static inline void w25_unlock(FAR struct spi_dev_s *spi);
static inline int w25_readid(FAR struct w25_dev_s *priv);
#ifndef CONFIG_W25_READONLY
static void w25_unprotect(FAR struct w25_dev_s *priv);
#endif
static uint8_t w25_waitcomplete(FAR struct w25_dev_s *priv);
static inline void w25_wren(FAR struct w25_dev_s *priv);
static inline void w25_wrdi(FAR struct w25_dev_s *priv);
static ssize_t w25_byteread(FAR struct w25_dev_s *priv, off_t offset,
                          size_t nbytes, FAR uint8_t *buffer);
static ssize_t w25_pageread(FAR struct w25_dev_s *priv, off_t address,
                          size_t nbytes, bool spare, FAR uint8_t *buffer);
#ifndef CONFIG_W25_READONLY
static int w25_erase2(FAR struct w25_dev_s *priv, off_t startblock, size_t nblocks);
static int w25_blockerase(FAR struct w25_dev_s *priv, size_t block);
static ssize_t w25_bytewrite(FAR struct w25_dev_s *priv, off_t address,
                             size_t nbytes, FAR const uint8_t *buffer);
static ssize_t w25_pagewrite(FAR struct w25_dev_s *priv, off_t address,
                          size_t nbytes, bool spare, FAR const uint8_t *buffer);
static int w25_mark_badblock(FAR struct w25_dev_s *priv, size_t block);
static bool w25_is_badblock(FAR struct w25_dev_s *priv, size_t block);
#endif

static int w25_cacheflush(struct w25_dev_s *priv);
static ssize_t w25_cacheread(struct w25_dev_s *priv, size_t sector, FAR uint8_t **buffer);
static int w25_cacheerase(struct w25_dev_s *priv, size_t sector);
static ssize_t w25_cachewrite(FAR struct w25_dev_s *priv, size_t sector, FAR const uint8_t *buffer);
#ifdef CONFIG_MTD_BYTE_WRITE
static ssize_t w25_cachebytewrite(FAR struct w25_dev_s *priv, size_t offset,
                               size_t nbytes, FAR const uint8_t *buffer);
#endif

/* MTD driver methods */

static int w25_erase(FAR struct mtd_dev_s *dev, off_t startsector, size_t nsectors);
static ssize_t w25_bread(FAR struct mtd_dev_s *dev, off_t startsector,
                           size_t nsectors, FAR uint8_t *buf);
static ssize_t w25_bwrite(FAR struct mtd_dev_s *dev, off_t startsector,
                            size_t nsectors, FAR const uint8_t *buf);
static ssize_t w25_read(FAR struct mtd_dev_s *dev, off_t offset,
                        size_t nbytes, FAR uint8_t *buffer);
static int w25_ioctl(FAR struct mtd_dev_s *dev, int cmd, unsigned long arg);
#if defined(CONFIG_MTD_BYTE_WRITE)
static ssize_t w25_write(FAR struct mtd_dev_s *dev, off_t offset,
                         size_t nbytes, FAR const uint8_t *buffer);
#endif

/************************************************************************************
 * Private Data
 ************************************************************************************/

#ifndef CONFIG_W25_READONLY
static int w25_mark_badblock(FAR struct w25_dev_s *priv, size_t block)
{
  int ret;
  off_t address = (block << W25_BLOCK_SHIFT);
  uint8_t spare_buf[W25_SPARE_SIZE];

  ssize_t nbytes;

  nbytes = w25_pageread(priv, address, W25_SPARE_SIZE, true, (FAR uint8_t *)spare_buf);
  if (nbytes != W25_SPARE_SIZE) {
    ret = -EFAULT;
    goto errout;
  }

  /* Mark bad block */
  spare_buf[0] = 0;
  spare_buf[1] = 0;

  nbytes = w25_pagewrite(priv, address, W25_SPARE_SIZE, true, (FAR uint8_t *)spare_buf);
  if (nbytes != W25_SPARE_SIZE) {
    ret = -EFAULT;
    goto errout;
  }

  ret = OK;

errout:
  return ret;
}

static bool w25_is_badblock(FAR struct w25_dev_s *priv, size_t block)
{
  off_t address = (block << W25_BLOCK_SHIFT);
  uint8_t spare_buf[W25_SPARE_SIZE];

  ssize_t nbytes;

  nbytes = w25_pageread(priv, address, W25_SPARE_SIZE, true, (FAR uint8_t *)spare_buf);
  if (nbytes != W25_SPARE_SIZE) {
    //TODO: Check whether set block as good or bad when read spare data failed
    return false;
  }

  if ((spare_buf[0] != W25_ERASED_STATE) || (spare_buf[1] != W25_ERASED_STATE))
    return true;

  return false;
}
#endif

#ifdef CONFIG_W25_DUMP
static void w25_dump(FAR struct mtd_dev_s *dev)
{
  FAR struct w25_dev_s *priv = (FAR struct w25_dev_s *)dev;
  uint8_t buffer[256];

  for (int sector = 0; sector < priv->nblocks; sector ++) {
      w25_byteread(priv, sector << W25_SECTOR_SHIFT, 256, buffer);

      bool hasdata = false;
      for (int i=0; i<256; i++) {
         if (buffer[i] != 0xff) {
            hasdata = true;
         }
      }

      if (!hasdata) continue;

      printf("** block %d:\n", sector);
      for (int i=0; i<256; i++) {
          printf("%02x ", buffer[i]);
          if ((i&15)==15)
             printf("\n");
      }
  }
}
#endif

/************************************************************************************
 * Private Functions
 ************************************************************************************/

static void w25_select_die(FAR struct spi_dev_s *spi, uint8_t die)
{
  SPI_SELECT(spi, SPIDEV_FLASH(0), true);

  /* Select die */
  (void)SPI_SEND(spi, W25_DIE_SEL);
  (void)SPI_SEND(spi, die);

  SPI_SELECT(spi, SPIDEV_FLASH(0), false);
}

static void w25_reset(FAR struct spi_dev_s *spi)
{
  SPI_SELECT(spi, SPIDEV_FLASH(0), true);

  /* Select die */
  (void)SPI_SEND(spi, W25_RESET);

  SPI_SELECT(spi, SPIDEV_FLASH(0), false);
}

static uint8_t w25_read_reg(FAR struct spi_dev_s *spi, uint8_t regaddr)
{

  SPI_SELECT(spi, SPIDEV_FLASH(0), true);
  (void)SPI_SEND(spi, W25_RDSR);
  (void)SPI_SEND(spi, regaddr);

  /* Send a dummy byte to generate the clock needed to shift out the status */

  uint8_t status = SPI_SEND(spi, W25_DUMMY);

  SPI_SELECT(spi, SPIDEV_FLASH(0), false);

  return status;
}

#ifdef CONFIG_W25_DEBUG
static void w25_read_last_ecc_address(FAR struct spi_dev_s *spi, uint16_t *page)
{
  uint8_t buf[2];
  SPI_SELECT(spi, SPIDEV_FLASH(0), true);

  /* Select die */
  (void)SPI_SEND(spi, W25_ECC_ADDR);
  (void)SPI_SEND(spi, W25_DUMMY);

  SPI_RECVBLOCK(spi, buf, 2);
  *page = ((uint16_t)buf[0] << 8) + (uint16_t)buf[1];

  SPI_SELECT(spi, SPIDEV_FLASH(0), false);
}
#endif

#if 0
static void w25_read_bbm(FAR struct spi_dev_s *spi)
{
  uint8_t *buffer = (FAR uint8_t *)kmm_malloc(80);

  SPI_SELECT(spi, SPIDEV_FLASH(0), true);

  (void)SPI_SEND(spi, W25_RDBBM);
  (void)SPI_SEND(spi, W25_DUMMY);

  SPI_RECVBLOCK(spi, buffer, 80);

  SPI_SELECT(spi, SPIDEV_FLASH(0), false);

  kmm_free(buffer);
}
#endif

/************************************************************************************
 * Name: w25_lock
 ************************************************************************************/

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

#if 0
  /* After locking the SPI bus, the we also need call the setfrequency, setbits, and
   * setmode methods to make sure that the SPI is properly configured for the device.
   * If the SPI buss is being shared, then it may have been left in an incompatible
   * state.
   */

  SPI_SETMODE(spi, CONFIG_W25_SPIMODE);
  SPI_SETBITS(spi, 8);
  (void)SPI_HWFEATURES(spi, 0);
  (void)SPI_SETFREQUENCY(spi, CONFIG_W25_SPIFREQUENCY);
#endif
}

/************************************************************************************
 * Name: w25_unlock
 ************************************************************************************/

static inline void w25_unlock(FAR struct spi_dev_s *spi)
{
  (void)SPI_LOCK(spi, false);
}

/************************************************************************************
 * Name: w25_readid
 ************************************************************************************/

static inline int w25_readid(struct w25_dev_s *priv)
{
  uint16_t manufacturer;
  uint16_t memory;
  uint16_t capacity;

  finfo("priv: %p\n", priv);

  /* Lock and configure the SPI bus */

  w25_lock(priv->spi);

#ifndef CONFIG_W25_SYNC_WRITE
  /* Wait for any preceding write or erase operation to complete. */

  (void)w25_waitcomplete(priv);
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

          return -ENODEV;
        }

      return OK;
    }

  /* We don't understand the manufacturer or the memory type */

  return -ENODEV;
}

/************************************************************************************
 * Name: w25_unprotect
 ************************************************************************************/

#ifndef CONFIG_W25_READONLY
static void w25_unprotect(FAR struct w25_dev_s *priv)
{
  /* Lock and configure the SPI bus */

  w25_lock(priv->spi);

#ifndef CONFIG_W25_SYNC_WRITE
  /* Wait for any preceding write or erase operation to complete. */

  (void)w25_waitcomplete(priv);
#endif

  /* Send "Write enable (WREN)" */

  w25_wren(priv);

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
#endif

/************************************************************************************
 * Name: w25_waitcomplete
 ************************************************************************************/

static uint8_t w25_waitcomplete(struct w25_dev_s *priv)
{
  uint8_t status;

  /* Loop as long as the memory is busy with a write cycle. Device sets BUSY
   * flag to a 1 state whhen previous write or erase command is still executing
   * and during this time, device will ignore further instructions except for
   * "Read Status Register" and "Erase/Program Suspend" instructions.
   */

  do
    {
      status = w25_read_reg(priv->spi, 0xC0);

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
#ifdef CONFIG_W25_DEBUG
    ferr("erase error block = %08x\n", priv->lastaddr >> W25_BLOCK_SHIFT);
#endif
    w25_mark_badblock(priv, priv->lastaddr >> W25_BLOCK_SHIFT);
  }

  if (status & W25_ERR_PROGRAM) {
#ifdef CONFIG_W25_DEBUG
    ferr("program error block = %08x\n", priv->lastaddr >> W25_BLOCK_SHIFT);
#endif
    w25_mark_badblock(priv, priv->lastaddr >> W25_BLOCK_SHIFT);
  }

  return status;
}

/************************************************************************************
 * Name:  w25_wren
 ************************************************************************************/

static inline void w25_wren(struct w25_dev_s *priv)
{
  /* Select this FLASH part */

  SPI_SELECT(priv->spi, SPIDEV_FLASH(0), true);

  /* Send "Write Enable (WREN)" command */

  (void)SPI_SEND(priv->spi, W25_WREN);

  /* Deselect the FLASH */

  SPI_SELECT(priv->spi, SPIDEV_FLASH(0), false);
}

/************************************************************************************
 * Name:  w25_wrdi
 ************************************************************************************/

static inline void w25_wrdi(struct w25_dev_s *priv)
{
  /* Select this FLASH part */

  SPI_SELECT(priv->spi, SPIDEV_FLASH(0), true);

  /* Send "Write Disable (WRDI)" command */

  (void)SPI_SEND(priv->spi, W25_WRDI);

  /* Deselect the FLASH */

  SPI_SELECT(priv->spi, SPIDEV_FLASH(0), false);
}

/************************************************************************************
 * Name:  w25_blockerase
 ************************************************************************************/

static int w25_blockerase(struct w25_dev_s *priv, size_t block)
{
  int ret = OK;
  off_t address = block << W25_BLOCK_SHIFT;

#ifdef CONFIG_W25_SPI_DEBUG
  ferr("sector: %08lx\n", (long)sector);
#endif

  if (w25_is_badblock(priv, block)) {
    ferr("bad block %d\n", block);
    ret = -EIO;
    goto errout;
  }

#ifndef CONFIG_W25_SYNC_WRITE
  /* Wait for any preceding write or erase operation to complete. */

  (void)w25_waitcomplete(priv);
#endif

  priv->lastaddr = address;

  w25_select_die(priv->spi, (uint8_t)(address>>W25_DIE_SHIFT));

  /* Send write enable instruction */

  w25_wren(priv);

  /* Select this FLASH part */

  SPI_SELECT(priv->spi, SPIDEV_FLASH(0), true);

  /* Send the "Sector Erase (SE)" instruction */

  (void)SPI_SEND(priv->spi, W25_BE);
  priv->prev_instr = W25_BE;

  /* Send the sector address high byte first. Only the most significant bits (those
   * corresponding to the sector) have any meaning.
   */
  uint16_t page = address >> W25_PAGE_SHIFT;

  (void)SPI_SEND(priv->spi, W25_DUMMY);
  (void)SPI_SEND(priv->spi, (page >> 8) & 0xff);
  (void)SPI_SEND(priv->spi, page & 0xff);

  /* Deselect the FLASH */

  SPI_SELECT(priv->spi, SPIDEV_FLASH(0), false);

#ifdef CONFIG_W25_SYNC_WRITE
  /* Wait for any preceding write or erase operation to complete. */
  uint8_t status = w25_waitcomplete(priv);
  if (status & W25_ERR_ERASE) {
#ifdef CONFIG_W25_DEBUG
    ferr("erase error block = %08x\n", priv->lastaddr >> W25_BLOCK_SHIFT);
#endif
    w25_mark_badblock(priv, address >> W25_BLOCK_SHIFT);
    ret = -EIO;
  }
#endif

errout:
  return ret;
}

/************************************************************************************
 * Name: w25_pageread
 ************************************************************************************/

static ssize_t w25_pageread(FAR struct w25_dev_s *priv, off_t address, size_t nbytes,
                            bool spare, FAR uint8_t *buffer)
{
#ifdef CONFIG_W25_SPI_DEBUG
  ferr("address: %08lx nbytes: %d\n", (long)address, (int)nbytes);
#endif

  if ((spare && (nbytes != W25_SPARE_SIZE)) || (nbytes > W25_PAGE_SIZE))
    return -EFAULT;

#ifndef CONFIG_W25_SYNC_WRITE
  /* Wait for any preceding write or erase operation to complete. */

  (void)w25_waitcomplete(priv);
#endif

  w25_select_die(priv->spi, (uint8_t)(address>>W25_DIE_SHIFT));

  /* Make sure that writing is disabled */

  w25_wrdi(priv);

  /* Read page data into cache */

  uint16_t page = address >> W25_PAGE_SHIFT;

  SPI_SELECT(priv->spi, SPIDEV_FLASH(0), true);
  (void)SPI_SEND(priv->spi, W25_RDPAGE);
  priv->prev_instr = W25_RDPAGE;

  (void)SPI_SEND(priv->spi, W25_DUMMY);
  (void)SPI_SEND(priv->spi, (page >> 8) & 0xff);
  (void)SPI_SEND(priv->spi, page & 0xff);
  SPI_SELECT(priv->spi, SPIDEV_FLASH(0), false);

  uint8_t status = w25_waitcomplete(priv);
  if (status & W25_ERR_ECC ) {
    w25_mark_badblock(priv, address >> W25_BLOCK_SHIFT);
#ifdef CONFIG_W25_DEBUG
    uint16_t ecc_page;

    w25_read_last_ecc_address(priv->spi, &ecc_page);
    uint8_t status2 = w25_read_reg(priv->spi, 0xA0);
    uint8_t status3 = w25_read_reg(priv->spi, 0xB0);
    ferr("ecc error block = %08x, page = %04x, last ecc page = %04x, status = (%02x, %02x, %02x)\n", address>>W25_BLOCK_SHIFT, page, ecc_page, status2, status3, status);
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

  SPI_RECVBLOCK(priv->spi, priv->page_buf, W25_PAGE_SIZE+W25_SPARE_SIZE);

  /* Deselect the FLASH */

  SPI_SELECT(priv->spi, SPIDEV_FLASH(0), false);

  if (spare)
    memcpy(buffer, (FAR uint8_t *)(priv->page_buf+W25_PAGE_SIZE), W25_SPARE_SIZE);
  else {
    if ((priv->page_buf[W25_PAGE_SIZE] != W25_ERASED_STATE) ||
        (priv->page_buf[W25_PAGE_SIZE+1] != W25_ERASED_STATE)) {
      ferr("bad block = %08x, page = %04x\n", address>>W25_BLOCK_SHIFT, page);
      /* bad block */
      return -EIO;
    }

    uint16_t column = address & ((1 << W25_PAGE_SHIFT) - 1);
    memcpy(buffer, (FAR uint8_t *)(priv->page_buf+column), nbytes);
  }

  return nbytes;
}

/************************************************************************************
 * Name:  w25_pagewrite
 ************************************************************************************/

#ifndef CONFIG_W25_READONLY
static ssize_t w25_pagewrite(struct w25_dev_s *priv, off_t address,
                          size_t nbytes, bool spare, FAR const uint8_t *buffer)
{
  int ret = nbytes;

#ifdef CONFIG_W25_SPI_DEBUG
  ferr("address: %08lx nwords: %d\n", (long)address, (int)nbytes);
#endif

  if ((spare && (nbytes > W25_SPARE_SIZE)) || (nbytes > W25_PAGE_SIZE))
    return -EFAULT;

#ifndef CONFIG_W25_SYNC_WRITE
  /* Wait for any preceding write or erase operation to complete. */
  (void)w25_waitcomplete(priv);
#endif

   uint16_t page = address >> W25_PAGE_SHIFT;
   uint16_t column = address & ((1 << W25_PAGE_SHIFT) - 1);

   w25_select_die(priv->spi, (uint8_t)(address>>W25_DIE_SHIFT));

   /* Enable write access to the FLASH */

   w25_wren(priv);

   /* Select this FLASH part */

   SPI_SELECT(priv->spi, SPIDEV_FLASH(0), true);

   /* Send the "Page Program (W25_PP)" Command */

   SPI_SEND(priv->spi, W25_PP);
   priv->prev_instr = W25_PP;

   /* Align column to end of page to write spare */
   if (spare)
     column = W25_PAGE_SIZE;

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
  uint8_t status = w25_waitcomplete(priv);

  if (status & W25_ERR_PROGRAM) {
    w25_mark_badblock(priv, address >> W25_BLOCK_SHIFT);
#ifdef CONFIG_W25_DEBUG
    ferr("program error block = %08x\n", address >> W25_BLOCK_SHIFT);
#endif
    ret = -EIO;
  }
#endif

   /* Disable writing */

   w25_wrdi(priv);

   return ret;
}
#endif

/************************************************************************************
 * Name:  w25_bytewrite
 ************************************************************************************/

#ifndef CONFIG_W25_READONLY
static ssize_t w25_bytewrite(FAR struct w25_dev_s *priv, off_t offset,
                         size_t nbytes, FAR const uint8_t *buffer)
{
  int    startpage;
  int    endpage;
  int    count;
  int    index;
  int    bytestowrite;
  ssize_t ret;

#ifdef CONFIG_W25_SPI_DEBUG
  ferr("offset = %08lx nbytes = %d\n", (long)offset, nbytes);
#endif

  /* We must test if the offset + count crosses one or more pages
   * and perform individual writes.  The devices can only write in
   * page increments.
   */

  startpage = offset / W25_PAGE_SIZE;
  endpage = (offset + nbytes) / W25_PAGE_SIZE;

  if (startpage == endpage)
    {
      /* All bytes within one programmable page.  Just do the write. */

      ret = w25_pagewrite(priv, offset, nbytes, false, buffer);
      if (ret != nbytes)
        goto errout;
    }
  else
    {
      /* Write the 1st partial-page */

      count = nbytes;
      bytestowrite = W25_PAGE_SIZE - (offset & (W25_PAGE_SIZE-1));
      ret = w25_pagewrite(priv, offset, bytestowrite, false, buffer);
      if (ret != bytestowrite)
        goto errout;

      /* Update offset and count */

      offset += bytestowrite;
      count -=  bytestowrite;
      index = bytestowrite;

      /* Write full pages */

      while (count >= W25_PAGE_SIZE)
        {
          ret = w25_pagewrite(priv, offset, W25_PAGE_SIZE, false, &buffer[index]);
          if (ret != W25_PAGE_SIZE)
            goto errout;

          /* Update offset and count */

          offset += W25_PAGE_SIZE;
          count -= W25_PAGE_SIZE;
          index += W25_PAGE_SIZE;
        }

      /* Now write any partial page at the end */

      if (count > 0)
        {
          ret = w25_pagewrite(priv, offset, count, false, &buffer[index]);
          if (ret != count)
            goto errout;
        }
    }
  ret = nbytes;

errout:
  return ret;
}
#endif

/************************************************************************************
 * Name: w25_cacheflush
 ************************************************************************************/

#if !defined(CONFIG_W25_READONLY)
static int w25_cacheflush(struct w25_dev_s *priv)
{
  int ret;

  /* If the cached is dirty (meaning that it no longer matches the old FLASH contents)
   * or was erased (with the cache containing the correct FLASH contents), then write
   * the cached erase block to FLASH.
   */

  if (IS_DIRTY(priv) || IS_ERASED(priv))
    {
      /* Write entire erase block to FLASH */

#ifdef CONFIG_W25_CACHE_DEBUG
      ferr("byte write: esectno = %d\n", priv->esectno);
#endif
      ssize_t nwrite = w25_bytewrite(priv, (off_t)priv->esectno << W25_BLOCK_SHIFT,
                                     W25_BLOCK_SIZE, priv->block);
      if (nwrite != W25_BLOCK_SIZE) {
        ret = (int)nwrite;
        goto errout;
      }

      /* The case is no long dirty and the FLASH is no longer erased */

      CLR_DIRTY(priv);
      CLR_ERASED(priv);
    }

  ret = OK;

errout:
  return ret;
}
#endif

/************************************************************************************
 * Name: w25_cacheread
 ************************************************************************************/

#if !defined(CONFIG_W25_READONLY)
static ssize_t w25_cacheread(struct w25_dev_s *priv, size_t sector, FAR uint8_t **buffer)
{
  off_t esectno;
  int   shift;
  int   index;
  ssize_t ret;

  /* Convert from the 512 byte sector to the erase sector size of the device.  For
   * exmample, if the actual erase sector size if 4Kb (1 << 12), then we first
   * shift to the right by 3 to get the sector number in 4096 increments.
   */

  shift    = W25_BLOCK_SHIFT - W25_SECTOR_SHIFT;
  esectno  = sector >> shift;
#ifdef CONFIG_W25_CACHE_DEBUG
  ferr("sector: %ld esectno: %d shift=%d\n", sector, esectno, shift);
#endif

  /* Check if the requested erase block is already in the cache */

  if (!IS_VALID(priv) || esectno != priv->esectno)
    {
      /* No.. Flush any dirty erase block currently in the cache */

      ret = w25_cacheflush(priv);
      if (ret < 0)
         goto errout;

      /* Read the erase block into the cache */

      ret = w25_byteread(priv, (esectno << W25_BLOCK_SHIFT), W25_BLOCK_SIZE, priv->block);
      if (ret != W25_BLOCK_SIZE)
         goto errout;

      /* Mark the sector as cached */

      priv->esectno = esectno;

      SET_VALID(priv);          /* The data in the cache is valid */
      CLR_DIRTY(priv);          /* It should match the FLASH contents */
      CLR_ERASED(priv);         /* The underlying FLASH has not been erased */
    }

    ret = W25_SECTOR_SIZE;

errout:

  /* Get the index to the 512 sector in the erase block that holds the argument */

  index = sector & ((1 << shift) - 1);

  /* Return the address in the cache that holds this sector */

  *buffer = &priv->block[index << W25_SECTOR_SHIFT];

  return ret;
}
#endif

/************************************************************************************
 * Name: w25_cacheerase
 ************************************************************************************/

#if !defined(CONFIG_W25_READONLY)
static int w25_cacheerase(struct w25_dev_s *priv, size_t sector)
{
  FAR uint8_t *cache_buf;
  int ret;

#ifdef CONFIG_W25_CACHE_DEBUG
  ferr("sector = %d\n", sector);
#endif

  /* First, make sure that the erase block containing the 512 byte sector is in
   * the cache.
   */

  ret = w25_cacheread(priv, sector, &cache_buf);
  if (ret != W25_SECTOR_SIZE)
    goto errout;

  /* Erase the block containing this sector if it is not already erased.
   * The erased indicated will be cleared when the data from the erase sector
   * is read into the cache and set here when we erase the block.
   */

  if (!IS_ERASED(priv))
    {
      off_t esectno  = sector >> (W25_BLOCK_SHIFT - W25_SECTOR_SHIFT);
      finfo("sector: %ld esectno: %d\n", sector, esectno);

      ret = w25_blockerase(priv, esectno);
      if (ret < 0)
        goto errout;

      SET_ERASED(priv);
    }

  /* Put the cached sector data into the erase state and mart the cache as dirty
   * (but don't update the FLASH yet.  The caller will do that at a more optimal
   * time).
   */

  memset(cache_buf, W25_ERASED_STATE, W25_SECTOR_SIZE);
  SET_DIRTY(priv);

  ret = OK;

errout:
  return ret;
}
#endif

/************************************************************************************
 * Name: w25_cachewrite
 ************************************************************************************/

#if !defined(CONFIG_W25_READONLY)
static ssize_t w25_cachewrite(FAR struct w25_dev_s *priv, size_t sector, FAR const uint8_t *buffer)
{
  FAR uint8_t *cache_buf;
  ssize_t ret;

#ifdef CONFIG_W25_CACHE_DEBUG
  ferr("sector = %d\n", sector);
#endif
  /* First, make sure that the erase block containing 512 byte sector is in
   * memory.
   */

  ret = w25_cacheread(priv, sector, &cache_buf);
  if (ret != W25_SECTOR_SIZE)
    goto errout;

  /* Erase the block containing this sector if it is not already erased.
   * The erased indicated will be cleared when the data from the erase sector
   * is read into the cache and set here when we erase the sector.
   */

  if (!IS_ERASED(priv))
    {
      off_t esectno  = sector >> (W25_BLOCK_SHIFT - W25_SECTOR_SHIFT);
#ifdef CONFIG_W25_CACHE_DEBUG
      ferr("sector: %ld esectno: %d\n", sector, esectno);
#endif

      ret = w25_blockerase(priv, esectno);
      if (ret < 0)
        goto errout;

      SET_ERASED(priv);
    }

  /* Copy the new sector data into cached erase block */

  memcpy(cache_buf, buffer, W25_SECTOR_SIZE);
  SET_DIRTY(priv);

  ret = W25_SECTOR_SIZE;

errout:
  return ret;
}
#endif

#if defined(CONFIG_MTD_BYTE_WRITE) && !defined(CONFIG_W25_READONLY)
static ssize_t w25_cachebytewrite(FAR struct w25_dev_s *priv, size_t offset,
                               size_t nbytes, FAR const uint8_t *buffer)
{
  FAR uint8_t *cache_buf;
  off_t sector = offset >> W25_SECTOR_SHIFT;
  off_t buff_offset = offset - (sector << W25_SECTOR_SHIFT);
  ssize_t ret;

#ifdef CONFIG_W25_CACHE_DEBUG
  ferr("offset = %d, nbytes = %d\n", offset, nbytes);
#endif

  /* First, make sure that the erase block containing 512 byte sector is in
   * memory.
   */

  ret = w25_cacheread(priv, sector, &cache_buf);
  if (ret != W25_SECTOR_SIZE)
    goto errout;

  cache_buf += buff_offset;

  /* Erase the block containing this sector if it is not already erased.
   * The erased indicated will be cleared when the data from the erase sector
   * is read into the cache and set here when we erase the sector.
   */

  if (!IS_ERASED(priv))
    {
      off_t esectno  = sector >> (W25_BLOCK_SHIFT - W25_SECTOR_SHIFT);
#ifdef CONFIG_W25_CACHE_DEBUG
      ferr("sector: %ld esectno: %d\n", sector, esectno);
#endif

      ret = w25_blockerase(priv, esectno);
      if (ret < 0)
        goto errout;

      SET_ERASED(priv);
    }

  /* Copy the new sector data into cached erase block */

  memcpy(cache_buf, buffer, nbytes);
  SET_DIRTY(priv);

  ret = nbytes;

errout:
  return ret;
}
#endif

/************************************************************************************
 * Name: w25_erase
 ************************************************************************************/

static int w25_erase(FAR struct mtd_dev_s *dev, off_t startsector, size_t nsectors)
{
#ifdef CONFIG_W25_READONLY
  return -EACESS;
#else
  FAR struct w25_dev_s *priv = (FAR struct w25_dev_s *)dev;
  size_t sectorleft = nsectors;
  int ret;

#ifdef CONFIG_W25_API_DEBUG
  ferr("startsector: %08lx nsectors: %d\n", (long)startsector, (int)nsectors);
#endif

  /* Lock access to the SPI bus until we complete the erase */

  w25_lock(priv->spi);

  while (sectorleft-- > 0)
    {
      /* Erase each sector */

      ret = w25_cacheerase(priv, startsector);
      if (ret < 0)
        goto errout;

      startsector++;
    }

  ret = (int)nsectors;

errout:

  w25_unlock(priv->spi);
  return ret;
#endif
}

static int w25_erase2(FAR struct w25_dev_s *priv, off_t startblock, size_t nblocks)
{
#ifdef CONFIG_W25_READONLY
  return -EACESS
#else
  size_t blocksleft = nblocks;
  int ret;

#ifdef CONFIG_W25_API_DEBUG
  ferr("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);
#endif

  /* Lock access to the SPI bus until we complete the erase */

  w25_lock(priv->spi);

  while (blocksleft-- > 0)
    {
      /* Erase each sector */

      ret = w25_blockerase(priv, startblock);
      if (ret < 0)
        ferr("find bad block %d \n",startblock);

      startblock++;
    }

  memset(priv->block, W25_ERASED_STATE, W25_BLOCK_SIZE);
  SET_ERASED(priv);

  ret = (int)nblocks;

  w25_unlock(priv->spi);
  return ret;
#endif
}

/************************************************************************************
 * Name: w25_bread
 ************************************************************************************/

static ssize_t w25_bread(FAR struct mtd_dev_s *dev, off_t startsector,
                         size_t nsectors, FAR uint8_t *buffer)
{
  ssize_t ret;
  FAR struct w25_dev_s *priv = (FAR struct w25_dev_s *)dev;

#ifdef CONFIG_W25_API_DEBUG
  ferr("startsector: %08lx nsectors: %d\n", (long)startsector, (int)nsectors);
#endif

  /* Lock the SPI bus and write all of the pages to FLASH */

  w25_lock(priv->spi);

  /* On this device, we can handle the sector read just like the byte-oriented read */

  for (int i=0; i<nsectors; i++)
   {
      FAR uint8_t *cache_buf;
      ret = w25_cacheread(priv, startsector+i, &cache_buf);
      if (ret != W25_SECTOR_SIZE)
        goto errout;

      memcpy(buffer + (i<<W25_SECTOR_SHIFT), cache_buf, W25_SECTOR_SIZE);
    }
  ret = nsectors;

errout:

  w25_unlock(priv->spi);

  return ret;
}

/************************************************************************************
 * Name: w25_bwrite
 ************************************************************************************/

static ssize_t w25_bwrite(FAR struct mtd_dev_s *dev, off_t startsector,
                          size_t nsectors, FAR const uint8_t *buffer)
{
#ifdef CONFIG_W25_READONLY
  return -EACCESS;
#else
  FAR struct w25_dev_s *priv = (FAR struct w25_dev_s *)dev;
  ssize_t ret;

#ifdef CONFIG_W25_API_DEBUG
  ferr("startsector: %08lx nsectors: %d\n", (long)startsector, (int)nsectors);
#endif

  /* Lock the SPI bus and write all of the pages to FLASH */

  w25_lock(priv->spi);

  for (int i=0; i<nsectors; i++)
   {
    ret = w25_cachewrite(priv, startsector+i, buffer);
    if (ret != W25_SECTOR_SIZE)
      goto errout;
   }

  ret = nsectors;

errout:
  w25_unlock(priv->spi);

  return ret;
#endif
}

static ssize_t w25_byteread(FAR struct w25_dev_s *priv, off_t offset,
                            size_t nbytes, FAR uint8_t *buffer)
{
  int    startpage;
  int    endpage;
  int    count;
  int    index;
  int    bytestoread;
  ssize_t ret;

#ifdef W25_SPI_DEBUG
  ferr("offset = %d, nbytes = %d\n", offset, nbytes);
#endif

  startpage = offset / W25_PAGE_SIZE;
  endpage = (offset + nbytes) / W25_PAGE_SIZE;

  if (startpage == endpage)
    {
      ret = w25_pageread(priv, offset, nbytes, false, buffer);
      if (ret != nbytes)
        goto errout;
    }
  else {
      /* Write the 1st partial-page */

      count = nbytes;
      bytestoread = W25_PAGE_SIZE - (offset & (W25_PAGE_SIZE-1));
      ret = w25_pageread(priv, offset, bytestoread, false, buffer);
      if (ret != bytestoread)
        goto errout;

      /* Update offset and count */

      offset += bytestoread;
      count -=  bytestoread;
      index = bytestoread;

      /* Write full pages */

      while (count >= W25_PAGE_SIZE)
        {
          ret = w25_pageread(priv, offset, W25_PAGE_SIZE, false, &buffer[index]);
          if (ret != W25_PAGE_SIZE)
             goto errout;

          /* Update offset and count */

          offset += W25_PAGE_SIZE;
          count -= W25_PAGE_SIZE;
          index += W25_PAGE_SIZE;
        }

      /* Now write any partial page at the end */

      if (count > 0)
        {
          ret = w25_pageread(priv, offset, count, false, &buffer[index]);
          if (ret != count)
             goto errout;
        }
  }

  ret = nbytes;

errout:
  return ret;
}

/************************************************************************************
 * Name: w25_read
 ************************************************************************************/

static ssize_t w25_read(FAR struct mtd_dev_s *dev, off_t offset,
                        size_t nbytes, FAR uint8_t *buffer)
{
  FAR struct w25_dev_s *priv = (FAR struct w25_dev_s *)dev;
  ssize_t ret;

#ifdef CONFIG_W25_API_DEBUG
  ferr("offset: %08lx nbytes: %d\n", (long)offset, (int)nbytes);
#endif

  /* Lock the SPI bus and select this FLASH part */

  w25_lock(priv->spi);

  ssize_t nread = nbytes;
  while (nbytes > 0) {
    uint16_t sector = offset >> W25_SECTOR_SHIFT;
    uint16_t buffer_offset = offset - (sector << W25_SECTOR_SHIFT);
    uint16_t read_count = W25_SECTOR_SIZE - buffer_offset;
    if (read_count > nbytes)
        read_count = nbytes;

    FAR uint8_t *cache_buf;
    ret = w25_cacheread(priv, sector, &cache_buf);
    if (ret != W25_SECTOR_SIZE)
      goto errout;

    memcpy((FAR uint8_t *)buffer, cache_buf + buffer_offset, read_count);
    buffer += read_count;
    offset += read_count;
    nbytes -= read_count;
  }
  ret = nread;

errout:

  w25_unlock(priv->spi);

  finfo("return nbytes: %d\n", (int)ret);

  return ret;
}

/************************************************************************************
 * Name: w25_write
 ************************************************************************************/

#if defined(CONFIG_MTD_BYTE_WRITE)
static ssize_t w25_write(FAR struct mtd_dev_s *dev, off_t offset,
                         size_t nbytes, FAR const uint8_t *buffer)
{
  FAR struct w25_dev_s *priv = (FAR struct w25_dev_s *)dev;
  ssize_t ret;

#ifdef CONFIG_W25_API_DEBUG
  ferr("offset: %08lx nbytes: %d\n", (long)offset, (int)nbytes);
#endif

  /* Lock the SPI bus and select this FLASH part */

  w25_lock(priv->spi);

  ssize_t nwritten = nbytes;
  while (nbytes > 0) {
    uint16_t sector = offset >> W25_SECTOR_SHIFT;
    uint16_t buffer_offset = offset - (sector << W25_SECTOR_SHIFT);
    uint16_t write_count = W25_SECTOR_SIZE - buffer_offset;
    if (write_count > nbytes)
      write_count = nbytes;

    ret = w25_cachebytewrite(priv, offset, write_count, buffer);
    if (ret != write_count)
      goto errout;

    buffer += write_count;
    offset += write_count;
    nbytes -= write_count;
  }

  ret = nwritten;

errout:
  w25_unlock(priv->spi);

  finfo("return nbytes: %d\n", (int)nwritten);

  return ret;
}
#endif

/************************************************************************************
 * Name: w25_ioctl
 ************************************************************************************/

static int w25_ioctl(FAR struct mtd_dev_s *dev, int cmd, unsigned long arg)
{
  FAR struct w25_dev_s *priv = (FAR struct w25_dev_s *)dev;
  int ret = -EINVAL; /* Assume good command with bad parameters */

#ifdef CONFIG_W25_API_DEBUG
  ferr("cmd: %d \n", cmd);
#endif

  switch (cmd)
    {
      case MTDIOC_GEOMETRY:
        {
          FAR struct mtd_geometry_s *geo = (FAR struct mtd_geometry_s *)((uintptr_t)arg);
          if (geo)
            {
              /* Populate the geometry structure with information need to know
               * the capacity and how to access the device.
               *
               * NOTE: that the device is treated as though it where just an array
               * of fixed size blocks.  That is most likely not true, but the client
               * will expect the device logic to do whatever is necessary to make it
               * appear so.
               */

              geo->blocksize    = (1 << W25_SECTOR_SHIFT);
              geo->erasesize    = (1 << W25_SECTOR_SHIFT);
              geo->neraseblocks = priv->nblocks << (W25_BLOCK_SHIFT - W25_SECTOR_SHIFT);
              ret               = OK;

              finfo("blocksize: %d erasesize: %d neraseblocks: %d\n",
                    geo->blocksize, geo->erasesize, geo->neraseblocks);
            }
#ifdef CONFIG_W25_DUMP
            w25_dump(dev);
#endif
        }
        break;

      case MTDIOC_BULKERASE:
        {
          /* Erase the entire device */

          ret = w25_erase2(priv, 0, priv->nblocks);
          if (ret == priv->nblocks)
            ret = OK;
        }
        break;

      case MTDIOC_FLUSH:
      {
        ret = w25_cacheflush(priv);
        break;
      }

      case MTDIOC_XIPBASE:
      default:
        ret = -ENOTTY; /* Bad command */
        break;
    }

  finfo("return %d\n", ret);
  return ret;
}

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: w25_initialize
 *
 * Description:
 *   Create an initialize MTD device instance.  MTD devices are not registered
 *   in the file system, but are created as instances that can be bound to
 *   other functions (such as a block or character driver front end).
 *
 ************************************************************************************/

FAR struct mtd_dev_s *w25_initialize(FAR struct spi_dev_s *spi)
{
  FAR struct w25_dev_s *priv;
  int ret;

  finfo("spi: %p\n", spi);

  /* Allocate a state structure (we allocate the structure instead of using
   * a fixed, static allocation so that we can handle multiple FLASH devices.
   * The current implementation would handle only one FLASH part per SPI
   * device (only because of the SPIDEV_FLASH(0) definition) and so would have
   * to be extended to handle multiple FLASH parts on the same SPI bus.
   */

  priv = (FAR struct w25_dev_s *)kmm_zalloc(sizeof(struct w25_dev_s));
  if (priv)
    {
      /* Initialize the allocated structure (unsupported methods were
       * nullified by kmm_zalloc).
       */

      priv->mtd.erase  = w25_erase;
      priv->mtd.bread  = w25_bread;
      priv->mtd.bwrite = w25_bwrite;
      priv->mtd.read   = w25_read;
      priv->mtd.ioctl  = w25_ioctl;
#if defined(CONFIG_MTD_BYTE_WRITE)
      priv->mtd.write  = w25_write;
#endif
      priv->spi        = spi;

      /* Deselect the FLASH */

      SPI_SELECT(spi, SPIDEV_FLASH(0), false);

      w25_reset(priv->spi);
      usleep(1000);

      /* Identify the FLASH chip and get its capacity */

      ret = w25_readid(priv);
      if (ret != OK)
        {
          /* Unrecognized! Discard all of that work we just did and return NULL */

          ferr("ERROR: Unrecognized\n");
          kmm_free(priv);
          return NULL;
        }
      else
        {
          /* Make sure that the FLASH is unprotected so that we can write into it */

#ifndef CONFIG_W25_READONLY
          w25_select_die(priv->spi, 0);
          w25_unprotect(priv);
          w25_select_die(priv->spi, 1);
          w25_unprotect(priv);
#endif

          /* Allocate a buffer for the erase block cache */

          priv->block = (FAR uint8_t *)kmm_malloc(W25_BLOCK_SIZE);
          if (!priv->block)
            {
              /* Allocation failed! Discard all of that work we just did and return NULL */

              ferr("ERROR: Allocation failed\n");
              kmm_free(priv);
              return NULL;
            }
        }
    }

    priv->lastaddr = 0xffffffff;

    priv->page_buf = (FAR uint8_t *)kmm_malloc(W25_PAGE_SIZE+W25_SPARE_SIZE);

  //w25_read_bbm(priv->spi);

  /* Register the MTD with the procfs system if enabled */

#ifdef CONFIG_MTD_REGISTRATION
  mtd_register(&priv->mtd, "w25");
#endif

  /* Return the implementation-specific state structure as the MTD device */

  finfo("Return %p\n", priv);
  return (FAR struct mtd_dev_s *)priv;
}
