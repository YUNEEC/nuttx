/************************************************************************************
 * drivers/mtd/spi.c
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

//#define CONFIG_SPI_CACHE_DEBUG
//#define CONFIG_SPI_API_DEBUG

/* Cache flags */

#define SPI_CACHE_VALID            (1 << 0)  /* 1=Cache has valid data */
#define SPI_CACHE_DIRTY            (1 << 1)  /* 1=Cache is dirty */
#define SPI_CACHE_ERASED           (1 << 2)  /* 1=Backing FLASH is erased */

#define IS_VALID(p)                ((((p)->flags) & SPI_CACHE_VALID) != 0)
#define IS_DIRTY(p)                ((((p)->flags) & SPI_CACHE_DIRTY) != 0)
#define IS_ERASED(p)               ((((p)->flags) & SPI_CACHE_ERASED) != 0)

#define SET_VALID(p)               do { (p)->flags |= SPI_CACHE_VALID; } while (0)
#define SET_DIRTY(p)               do { (p)->flags |= SPI_CACHE_DIRTY; } while (0)
#define SET_ERASED(p)              do { (p)->flags |= SPI_CACHE_ERASED; } while (0)

#define CLR_VALID(p)               do { (p)->flags &= ~SPI_CACHE_VALID; } while (0)
#define CLR_DIRTY(p)               do { (p)->flags &= ~SPI_CACHE_DIRTY; } while (0)
#define CLR_ERASED(p)              do { (p)->flags &= ~SPI_CACHE_ERASED; } while (0)

/* Helpers */

static ssize_t spi_byteread(FAR struct spi_flash_dev_s *priv, off_t offset,
                          size_t nbytes, FAR uint8_t *buffer);
static int spi_bulkerase(FAR struct spi_flash_dev_s *priv, off_t startblock, size_t nblocks, uint8_t *freecount);
static ssize_t spi_bytewrite(FAR struct spi_flash_dev_s *priv, off_t address,
                             size_t nbytes, FAR const uint8_t *buffer);

static int spi_cacheflush(struct spi_flash_dev_s *priv);
static ssize_t spi_cacheread(struct spi_flash_dev_s *priv, size_t block, FAR uint8_t **buffer);
static ssize_t spi_cachewrite(FAR struct spi_flash_dev_s *priv, size_t block, FAR const uint8_t *buffer);
#ifdef CONFIG_MTD_BYTE_WRITE
static ssize_t spi_cachebytewrite(FAR struct spi_flash_dev_s *priv, size_t offset,
                               size_t nbytes, FAR const uint8_t *buffer);
#endif

/* MTD driver methods */

static int spi_erase(FAR struct mtd_dev_s *dev, off_t startblock, size_t nblocks);
static ssize_t spi_bread(FAR struct mtd_dev_s *dev, off_t startblock,
                           size_t nblocks, FAR uint8_t *buf);
static ssize_t spi_bwrite(FAR struct mtd_dev_s *dev, off_t startblock,
                            size_t nblocks, FAR const uint8_t *buf);
static ssize_t spi_read(FAR struct mtd_dev_s *dev, off_t offset,
                        size_t nbytes, FAR uint8_t *buffer);
static int spi_ioctl(FAR struct mtd_dev_s *dev, int cmd, unsigned long arg);
#if defined(CONFIG_MTD_BYTE_WRITE)
static ssize_t spi_write(FAR struct mtd_dev_s *dev, off_t offset,
                         size_t nbytes, FAR const uint8_t *buffer);
#endif

int spi_mark_badblock(FAR struct spi_flash_dev_s *priv, size_t block)
{
  int ret;
  off_t address = (block << priv->block_shift);
  uint8_t *spare_buf = kmm_malloc(priv->spare_size);

  /* Mark bad block */
  memset(spare_buf, 0, priv->spare_size);

  ssize_t nbytes = priv->pagewrite(priv, address, priv->spare_size, true, (FAR uint8_t *)spare_buf);
  if (nbytes != priv->spare_size) {
    ret = -EFAULT;
    goto errout;
  }

  ret = OK;

errout:
  kmm_free(spare_buf);
  return ret;
}

bool spi_is_badblock(FAR struct spi_flash_dev_s *priv, size_t block)
{
  bool ret;
  off_t address = (block << priv->block_shift);
  uint8_t *spare_buf = kmm_malloc(priv->spare_size);

  ssize_t nbytes;

  nbytes = priv->pageread(priv, address, priv->spare_size, true, (FAR uint8_t *)spare_buf);
  if (nbytes != priv->spare_size) {
    //TODO: Check whether set block as good or bad when read spare data failed
    ret = false;
    goto errout;
  }

  if ((spare_buf[0] != SPI_ERASED_STATE) || (spare_buf[1] != SPI_ERASED_STATE)) {
    ret = true;
    goto errout;
  }

errout:
  kmm_free(spare_buf);
  return ret;
}

#ifdef CONFIG_SPI_DUMP
static void spi_dump(FAR struct mtd_dev_s *dev)
{
  FAR struct spi_flash_dev_s *priv = (FAR struct spi_flash_dev_s *)dev;
  uint8_t buffer[256];

  for (int block = 0; block < priv->nblocks; block++) {
      spi_byteread(priv, block << priv->block_shift, 256, buffer);

      bool hasdata = false;
      for (int i=0; i<256; i++) {
         if (buffer[i] != 0xff) {
            hasdata = true;
         }
      }

      if (!hasdata) continue;

      printf("** block %d:\n", block);
      for (int i=0; i<256; i++) {
          printf("%02x ", buffer[i]);
          if ((i&15)==15)
             printf("\n");
      }
  }
}
#endif

static ssize_t spi_bytewrite(FAR struct spi_flash_dev_s *priv, off_t offset,
                         size_t nbytes, FAR const uint8_t *buffer)
{
  int    startpage;
  int    endpage;
  int    count;
  int    index;
  int    bytestowrite;
  ssize_t ret;

#ifdef CONFIG_SPI_API_DEBUG
  ferr("offset = %08lx nbytes = %d\n", (long)offset, nbytes);
#endif

  /* We must test if the offset + count crosses one or more pages
   * and perform individual writes.  The devices can only write in
   * page increments.
   */

  startpage = offset / priv->page_size;
  endpage = (offset + nbytes) / priv->page_size;

  if (startpage == endpage)
    {
      /* All bytes within one programmable page.  Just do the write. */

      ret = priv->pagewrite(priv, offset, nbytes, false, buffer);
      if (ret != nbytes)
        goto errout;
    }
  else
    {
      /* Write the 1st partial-page */

      count = nbytes;
      bytestowrite = priv->page_size - (offset & (priv->page_size-1));
      ret = priv->pagewrite(priv, offset, bytestowrite, false, buffer);
      if (ret != bytestowrite)
        goto errout;

      /* Update offset and count */

      offset += bytestowrite;
      count -=  bytestowrite;
      index = bytestowrite;

      /* Write full pages */

      while (count >= priv->page_size)
        {
          ret = priv->pagewrite(priv, offset, priv->page_size, false, &buffer[index]);
          if (ret != priv->page_size)
            goto errout;

          /* Update offset and count */

          offset += priv->page_size;
          count -= priv->page_size;
          index += priv->page_size;
        }

      /* Now write any partial page at the end */

      if (count > 0)
        {
          ret = priv->pagewrite(priv, offset, count, false, &buffer[index]);
          if (ret != count)
            goto errout;
        }
    }
  ret = nbytes;

errout:
  return ret;
}

static int spi_cacheflush(struct spi_flash_dev_s *priv)
{
  int ret;

  /* If the cached is dirty (meaning that it no longer matches the old FLASH contents)
   * or was erased (with the cache containing the correct FLASH contents), then write
   * the cached erase block to FLASH.
   */

  if (IS_DIRTY(priv) || IS_ERASED(priv))
    {
      /* Write entire erase block to FLASH */

#ifdef CONFIG_SPI_CACHE_DEBUG
      ferr("byte write: block = %d\n", priv->block);
#endif
      ssize_t nwrite = spi_bytewrite(priv, (off_t)priv->block << priv->block_shift,
                                     priv->block_size, priv->block_buf);
      if (nwrite != priv->block_size) {
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

static ssize_t spi_cacheread(struct spi_flash_dev_s *priv, size_t block, FAR uint8_t **buffer)
{
  ssize_t ret;

#ifdef CONFIG_SPI_CACHE_DEBUG
  ferr("block: %d shift=%d\n", block, shift);
#endif

  /* Check if the requested erase block is already in the cache */

  if (!IS_VALID(priv) || block != priv->block)
    {
      /* No.. Flush any dirty erase block currently in the cache */

      ret = spi_cacheflush(priv);
      if (ret < 0)
         goto errout;

      /* Read the erase block into the cache */

      ret = spi_byteread(priv, (block << priv->block_shift), priv->block_size, priv->block_buf);
      if (ret != priv->block_size)
         goto errout;

      /* Mark the sector as cached */

      priv->block = block;

      SET_VALID(priv);          /* The data in the cache is valid */
      CLR_DIRTY(priv);          /* It should match the FLASH contents */
      CLR_ERASED(priv);         /* The underlying FLASH has not been erased */
    }

  ret = priv->block_size;

errout:

  /* Return the address in the cache that holds this sector */

  *buffer = (FAR uint8_t *)priv->block_buf;

  return ret;
}

static ssize_t spi_cachewrite(FAR struct spi_flash_dev_s *priv, size_t block, FAR const uint8_t *buffer)
{
  FAR uint8_t *cache_buf;
  ssize_t ret;

#ifdef CONFIG_SPI_CACHE_DEBUG
  ferr("block = %d\n", block);
#endif
  /* First, make sure that the erase block containing 512 byte sector is in
   * memory.
   */

  ret = spi_cacheread(priv, block, &cache_buf);
  if (ret != priv->block_size)
    goto errout;

  /* Erase the block containing this sector if it is not already erased.
   * The erased indicated will be cleared when the data from the erase sector
   * is read into the cache and set here when we erase the sector.
   */

  if (!IS_ERASED(priv))
    {
#ifdef CONFIG_SPI_CACHE_DEBUG
      ferr("block: %d\n", block);
#endif

      ret = priv->blockerase(priv, block);
      if (ret < 0)
        goto errout;

      SET_ERASED(priv);
    }

  /* Copy the new sector data into cached erase block */

  memcpy(cache_buf, buffer, priv->block_size);
  SET_DIRTY(priv);

  ret = priv->block_size;

errout:
  return ret;
}

#ifdef CONFIG_MTD_BYTE_WRITE
static ssize_t spi_cachebytewrite(FAR struct spi_flash_dev_s *priv, size_t offset,
                               size_t nbytes, FAR const uint8_t *buffer)
{
  FAR uint8_t *cache_buf;
  off_t block = offset >> priv->block_shift;
  off_t buff_offset = offset - (block << priv->block_shift);
  ssize_t ret;

#ifdef CONFIG_SPI_CACHE_DEBUG
  ferr("offset = %d, nbytes = %d\n", offset, nbytes);
#endif

  /* First, make sure that the erase block containing 512 byte sector is in
   * memory.
   */

  ret = spi_cacheread(priv, block, &cache_buf);
  if (ret != priv->block_size)
    goto errout;

  cache_buf += buff_offset;

  /* Erase the block containing this sector if it is not already erased.
   * The erased indicated will be cleared when the data from the erase sector
   * is read into the cache and set here when we erase the sector.
   */

  if (!IS_ERASED(priv))
    {
#ifdef CONFIG_SPI_CACHE_DEBUG
      ferr("block: %d\n", block);
#endif

      ret = priv->blockerase(priv, block);
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

static int spi_erase_internal(FAR struct spi_flash_dev_s *priv,
                              off_t startblock, size_t nblocks, uint8_t *freecount)
{
  size_t blockleft = nblocks;
  int ret;

  while (blockleft-- > 0)
    {
      /* Erase each block */

      ret = priv->blockerase(priv, startblock);
      if (ret < 0) {
        if (freecount) {
          freecount[startblock] = MTD_BADBLOCK_MARK;
          ferr("find bad block %d\n", startblock);
        } else {
          goto errout;
        }
      }

      if (startblock == priv->block)
        {
          memset(priv->block_buf, SPI_ERASED_STATE, priv->block_size);
          SET_ERASED(priv);
        }

      startblock++;
    }

  ret = (int)nblocks;

errout:
  return ret;
}

static int spi_erase(FAR struct mtd_dev_s *dev, off_t startblock, size_t nblocks)
{
  FAR struct spi_flash_dev_s *priv = (FAR struct spi_flash_dev_s *)dev;
  int ret;

#ifdef CONFIG_SPI_API_DEBUG
  ferr("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);
#endif

  ret = spi_erase_internal(priv, startblock, nblocks, 0);

  return ret;
}

static int spi_bulkerase(FAR struct spi_flash_dev_s *priv,
                         off_t startblock, size_t nblocks, uint8_t *freecount)
{
  int ret;

#ifdef CONFIG_SPI_API_DEBUG
  ferr("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);
#endif

  ret = spi_erase_internal(priv, startblock, nblocks, freecount);

  return ret;
}

static ssize_t spi_bread(FAR struct mtd_dev_s *dev, off_t startblock,
                         size_t nblocks, FAR uint8_t *buffer)
{
  ssize_t ret;
  FAR struct spi_flash_dev_s *priv = (FAR struct spi_flash_dev_s *)dev;

#ifdef CONFIG_SPI_API_DEBUG
  ferr("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);
#endif

  /* On this device, we can handle the sector read just like the byte-oriented read */

  for (int block=0; block<nblocks; block++)
   {
     /* read from cache if hits, otherwise read from flash */
     if (block == priv->block) {
       FAR uint8_t *cache_buf;
       ret = spi_cacheread(priv, startblock+block, &cache_buf);
       if (ret != priv->block_size)
         goto errout;

       memcpy(buffer + (block<<priv->block_shift), cache_buf, priv->block_size);
     } else {
       ret = spi_byteread(priv, (startblock+block) << priv->block_shift,
                          priv->block_size, buffer + (block<<priv->block_shift));
       if (ret != priv->block_size)
         goto errout;
     }
   }

  ret = nblocks;

errout:

  return ret;
}

static ssize_t spi_bwrite(FAR struct mtd_dev_s *dev, off_t startblock,
                          size_t nblocks, FAR const uint8_t *buffer)
{
  FAR struct spi_flash_dev_s *priv = (FAR struct spi_flash_dev_s *)dev;
  ssize_t ret;

#ifdef CONFIG_SPI_API_DEBUG
  ferr("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);
#endif

  for (int i=0; i<nblocks; i++)
   {
    ret = spi_cachewrite(priv, startblock+i, buffer);
    if (ret != priv->block_size)
      goto errout;
   }

  ret = nblocks;

errout:

  return ret;
}

static ssize_t spi_byteread(FAR struct spi_flash_dev_s *priv, off_t offset,
                            size_t nbytes, FAR uint8_t *buffer)
{
  int    startpage;
  int    endpage;
  int    count;
  int    index;
  int    bytestoread;
  ssize_t ret;

#ifdef CONFIG_SPI_API_DEBUG
  ferr("offset = %d, nbytes = %d\n", offset, nbytes);
#endif

  startpage = offset / priv->page_size;
  endpage = (offset + nbytes) / priv->page_size;

  if (startpage == endpage)
    {
      ret = priv->pageread(priv, offset, nbytes, false, buffer);
      if (ret != nbytes)
        goto errout;
    }
  else {
      /* Write the 1st partial-page */

      count = nbytes;
      bytestoread = priv->page_size - (offset & (priv->page_size-1));
      ret = priv->pageread(priv, offset, bytestoread, false, buffer);
      if (ret != bytestoread)
        goto errout;

      /* Update offset and count */

      offset += bytestoread;
      count -=  bytestoread;
      index = bytestoread;

      /* Write full pages */

      while (count >= priv->page_size)
        {
          ret = priv->pageread(priv, offset, priv->page_size, false, &buffer[index]);
          if (ret != priv->page_size)
             goto errout;

          /* Update offset and count */

          offset += priv->page_size;
          count -= priv->page_size;
          index += priv->page_size;
        }

      /* Now write any partial page at the end */

      if (count > 0)
        {
          ret = priv->pageread(priv, offset, count, false, &buffer[index]);
          if (ret != count)
             goto errout;
        }
  }

  ret = nbytes;

errout:
  return ret;
}

static ssize_t spi_read(FAR struct mtd_dev_s *dev, off_t offset,
                        size_t nbytes, FAR uint8_t *buffer)
{
  FAR struct spi_flash_dev_s *priv = (FAR struct spi_flash_dev_s *)dev;
  ssize_t ret;

#ifdef CONFIG_SPI_API_DEBUG
  ferr("offset: %08lx nbytes: %d\n", (long)offset, (int)nbytes);
#endif

  ssize_t nread = nbytes;
  while (nbytes > 0) {
    uint16_t block = offset >> priv->block_shift;
    size_t buffer_offset = offset - (block << priv->block_shift);
    size_t read_count = priv->block_size - buffer_offset;

    if (read_count > nbytes)
      read_count = nbytes;

    /* read from cache if hits, otherwise read from flash */
    if (block == priv->block) {
      FAR uint8_t *cache_buf;
      ret = spi_cacheread(priv, block, &cache_buf);
      if (ret != priv->block_size)
        goto errout;

       memcpy(buffer, cache_buf + buffer_offset, read_count);
    } else {
       ret = spi_byteread(priv, offset, read_count, buffer);
       if (ret != read_count)
         goto errout;
    }

    buffer += read_count;
    offset += read_count;
    nbytes -= read_count;
  }
  ret = nread;

errout:
  return ret;
}

#if defined(CONFIG_MTD_BYTE_WRITE)
static ssize_t spi_write(FAR struct mtd_dev_s *dev, off_t offset,
                         size_t nbytes, FAR const uint8_t *buffer)
{
  FAR struct spi_flash_dev_s *priv = (FAR struct spi_flash_dev_s *)dev;
  ssize_t ret;

#ifdef CONFIG_SPI_API_DEBUG
  ferr("offset: %08lx nbytes: %d\n", (long)offset, (int)nbytes);
#endif

  ssize_t nwritten = nbytes;
  while (nbytes > 0) {
    uint16_t block = offset >> priv->block_shift;
    size_t buffer_offset = offset - (block << priv->block_shift);
    size_t write_count = priv->block_size - buffer_offset;

    if (write_count > nbytes)
      write_count = nbytes;

    ret = spi_cachebytewrite(priv, offset, write_count, buffer);
    if (ret != write_count)
      goto errout;

    buffer += write_count;
    offset += write_count;
    nbytes -= write_count;
  }

  ret = nwritten;

errout:
  return ret;
}
#endif

static int spi_ioctl(FAR struct mtd_dev_s *dev, int cmd, unsigned long arg)
{
  FAR struct spi_flash_dev_s *priv = (FAR struct spi_flash_dev_s *)dev;
  int ret = -EINVAL; /* Assume good command with bad parameters */

#ifdef CONFIG_SPI_API_DEBUG
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

              geo->blocksize    = (1 << priv->block_shift);
              geo->erasesize    = (1 << priv->block_shift);
              geo->neraseblocks = priv->nblocks;
              ret               = OK;

              finfo("blocksize: %d erasesize: %d neraseblocks: %d\n",
                    geo->blocksize, geo->erasesize, geo->neraseblocks);
            }
#ifdef CONFIG_SPI_DUMP
            spi_dump(dev);
#endif
        }
        break;

      case MTDIOC_BULKERASE:
        {
          /* Erase the entire device */

          ret = spi_bulkerase(priv, 0, priv->nblocks, (uint8_t *) arg);
          if (ret == priv->nblocks)
            ret = OK;
        }
        break;

      case MTDIOC_FLUSH:
      {
        ret = spi_cacheflush(priv);
        break;
      }

      case MTDIOC_XIPBASE:
      default:
        ret = -ENOTTY; /* Bad command */
        break;
    }

  return ret;
}

FAR struct mtd_dev_s *spi_initialize(FAR struct spi_dev_s *spi)
{
  FAR struct spi_flash_dev_s *priv;
  int ret;

  /* Allocate a state structure (we allocate the structure instead of using
   * a fixed, static allocation so that we can handle multiple FLASH devices.
   * The current implementation would handle only one FLASH part per SPI
   * device (only because of the SPIDEV_FLASH(0) definition) and so would have
   * to be extended to handle multiple FLASH parts on the same SPI bus.
   */

  priv = (FAR struct spi_flash_dev_s *)kmm_zalloc(sizeof(struct spi_flash_dev_s));
  if (priv)
    {
      /* Initialize the allocated structure (unsupported methods were
       * nullified by kmm_zalloc).
       */

      priv->mtd.erase  = spi_erase;
      priv->mtd.bread  = spi_bread;
      priv->mtd.bwrite = spi_bwrite;
      priv->mtd.read   = spi_read;
      priv->mtd.ioctl  = spi_ioctl;
#if defined(CONFIG_MTD_BYTE_WRITE)
      priv->mtd.write  = spi_write;
#endif
      priv->spi        = spi;

      ret = spi_flash_initialize(priv);

      if (ret != OK)
        {
          kmm_free(priv);
          return NULL;
        }
      else
        {
          /* Allocate a buffer for the erase block cache */

          CLR_VALID(priv);
          priv->block = 0;
          priv->block_buf = (FAR uint8_t *)kmm_malloc(priv->block_size);
          if (!priv->block_buf)
            {
              /* Allocation failed! Discard all of that work we just did and return NULL */

              ferr("ERROR: Allocation failed\n");
              kmm_free(priv);
              return NULL;
            }
        }

      priv->lastaddr = 0xffffffff;

      priv->page_buf = (FAR uint8_t *)kmm_malloc(priv->page_size+priv->spare_size);

      /* Register the MTD with the procfs system if enabled */

#ifdef CONFIG_MTD_REGISTRATION
      mtd_register(&priv->mtd, "spi");
#endif
    }

  /* Return the implementation-specific state structure as the MTD device */

  return (FAR struct mtd_dev_s *)priv;
}

