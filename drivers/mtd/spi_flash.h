
/************************************************************************************
 * drivers/mtd/spi.h
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

/* This type represents the state of the MTD device.  The struct mtd_dev_s must
 * appear at the beginning of the definition so that you can freely cast between
 * pointers to struct mtd_dev_s and struct spi_flash_dev_s.
 */

#define SPI_ERASED_STATE           0xff      /* State of FLASH when erased */

struct spi_flash_dev_s
{
  struct mtd_dev_s      mtd;         /* MTD interface */
  FAR struct spi_dev_s  *spi;        /* Saved SPI interface instance */
  uint16_t              nblocks;     /* Number of erase blocks */
  uint8_t               prev_instr;  /* Previous instruction given to W25 device */

  uint8_t               flags;       /* Buffered block flags */
  off_t                 block;       /* Erase block number in the cache*/
  FAR uint8_t           *block_buf;  /* Allocated block data */

  uint32_t              lastaddr;    /* Last erase or program address */
  FAR uint8_t           *page_buf;   /* page buffer */

  /* information filled by SPI device driver */
  int                   block_size;
  int                   block_shift;
  int                   page_size;
  int                   page_shift;
  int                   spare_size;

  uint16_t              manufacturer;
  uint16_t              memory;
  uint16_t              capacity;

  int (*blockerase)(struct spi_flash_dev_s *priv, size_t block);
  ssize_t (*pageread)(FAR struct spi_flash_dev_s *priv, off_t address, size_t nbytes,
                            bool spare, FAR uint8_t *buffer);
  ssize_t (*pagewrite)(struct spi_flash_dev_s *priv, off_t address,
                          size_t nbytes, bool spare, FAR const uint8_t *buffer);
};

FAR int spi_flash_initialize(FAR struct spi_flash_dev_s *priv);
int spi_mark_badblock(FAR struct spi_flash_dev_s *priv, size_t block);
bool spi_is_badblock(FAR struct spi_flash_dev_s *priv, size_t block);

