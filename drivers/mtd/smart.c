/****************************************************************************
 * drivers/mtd/smart.c
 *
 * Sector Mapped Allocation for Really Tiny (SMART) Flash block driver.
 *
 *   Copyright (C) 2013-2016 Ken Pettit. All rights reserved.
 *   Author: Ken Pettit <pettitkd@gmail.com>
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/mount.h>
#include <sys/stat.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <debug.h>
#include <errno.h>

#include <crc8.h>
#include <crc16.h>
#include <crc32.h>
#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/mtd/smart.h>
#include <nuttx/fs/smart.h>

//#define CONFIG_MTD_SMART_LOGICAL_SECTOR
//#define CONFIG_MTD_SMART_DEBUG

#define SMART_SIGNATURE_SECTOR     0
#define SMART_METADATA_SECTOR      1

#define SMART_FREECOUNT_BADBLOCK  0xee

#define SMART_INTERNAL_VERSION     1

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/


#define SMART_STATUS_COMMITTED    0x80
#define SMART_STATUS_RELEASED     0x40
#define SMART_STATUS_CRC          0x20
#define SMART_STATUS_SIZEBITS     0x1c
#define SMART_STATUS_VERBITS      0x03

#define SMART_STATUS_VERSION      0x01

#define SMART_SECTSIZE_256        0x00
#define SMART_SECTSIZE_512        0x04
#define SMART_SECTSIZE_1024       0x08
#define SMART_SECTSIZE_2048       0x0c
#define SMART_SECTSIZE_4096       0x10
#define SMART_SECTSIZE_8192       0x14
#define SMART_SECTSIZE_16384      0x18

#define SMART_FMT_STAT_UNKNOWN    0
#define SMART_FMT_STAT_FORMATTED  1
#define SMART_FMT_STAT_NOFMT      2

#define SMART_FMT_POS1            sizeof(struct smart_sect_header_s)
#define SMART_FMT_POS2            (SMART_FMT_POS1 + 1)
#define SMART_FMT_POS3            (SMART_FMT_POS1 + 2)
#define SMART_FMT_POS4            (SMART_FMT_POS1 + 3)

#define SMART_FMT_SIG1            'S'
#define SMART_FMT_SIG2            'M'
#define SMART_FMT_SIG3            'R'
#define SMART_FMT_SIG4            'T'

#define SMART_FMT_VERSION_POS     (SMART_FMT_POS1 + 4)
#define SMART_FMT_INTERNAL_VERSION_POS     (SMART_FMT_POS1 + 5)
#define SMART_FMT_NAMESIZE_POS    (SMART_FMT_POS1 + 7)
#define SMART_FMT_ROOTDIRS_POS    (SMART_FMT_POS1 + 8)
#define SMART_SIGNATURE_SIZE      (SMART_FMT_ROOTDIRS_POS+1)
#define SMART_PARTNAME_SIZE         4

#ifdef CONFIG_MTD_SMART_LOGICAL_SECTOR
#define SMART_FIRST_ALLOC_SECTOR    12      /* First logical sector number we will
                                             * use for assignment of requested Alloc
                                             * sectors.  All enries below this are
                                             * reserved (some for root dir entries,
                                             * other for our use, such as format
                                             * sector, etc. */
#else
#define SMART_FIRST_ALLOC_SECTOR    20      /* Reserve 5 physical blocks */
#endif

#if defined(CONFIG_MTD_SMART_READAHEAD) || (defined(CONFIG_DRVR_WRITABLE) && \
    defined(CONFIG_MTD_SMART_WRITEBUFFER))
#  define SMART_HAVE_RWBUFFER 1
#endif

#ifndef CONFIG_MTD_SMART_SECTOR_SIZE
#  define  CONFIG_MTD_SMART_SECTOR_SIZE 1024
#endif

#ifndef offsetof
#define offsetof(type, member) ( (size_t) &( ( (type *) 0)->member))
#endif

#define SMART_MAX_ALLOCS        10

#ifndef CONFIG_MTD_SMART_ALLOC_DEBUG
#define smart_malloc(d, b, n)   kmm_malloc(b)
#define smart_zalloc(d, b, n)   kmm_zalloc(b)
#define smart_free(d, p)        kmm_free(p)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct smart_struct_s
{
  FAR struct mtd_dev_s *mtd;              /* Contained MTD interface */
  struct mtd_geometry_s geo;              /* Device geometry */

#if defined(CONFIG_FS_PROCFS) && !defined(CONFIG_FS_PROCFS_EXCLUDE_SMARTFS)
  uint32_t              unusedsectors;    /* Count of unused sectors (i.e. free when erased) */
  uint32_t              blockerases;      /* Count of unused sectors (i.e. free when erased) */
#endif
  uint16_t              neraseblocks;     /* Number of erase blocks or sub-sectors */
  uint16_t              lastallocblock;   /* Last  block we allocated a sector from */
  uint16_t              releasesectors;   /* Total number of released sectors */
  uint16_t              mtdBlksPerSector; /* Number of MTD blocks per SMART Sector */
  uint16_t              sectorsPerBlk;    /* Number of sectors per erase block */
  uint16_t              sectorsize;       /* Sector size on device */
  uint16_t              totalsectors;     /* Total number of sectors on device */
  uint32_t              erasesize;        /* Size of an erase block */
#ifdef CONFIG_MTD_SMART_LOGICAL_SECTOR
  FAR uint8_t          *releasecount;     /* Count of released sectors per erase block */
#endif
  FAR uint8_t          *freecount;        /* Count of free sectors per erase block */
  uint16_t              rootphyssector;   /* root physical sector */
  uint16_t              mapphyssector;    /* map physical sector */
  uint16_t              entryphyssector;  /* directory entry physical sector */
#ifndef CONFIG_MTD_BYTE_WRITE
  FAR char             *rwbuffer;         /* Our sector read/write buffer */
#endif
  char                  partname[SMART_PARTNAME_SIZE]; /* Optional paßrtition name */
  uint8_t               formatversion;    /* Format version on the device */
  uint8_t               formatstatus;     /* Indicates the status of the device format */
  uint8_t               namesize;         /* Length of filenames on this device */
  uint8_t               debuglevel;       /* Debug reporting level */
  uint8_t               availSectPerBlk;  /* Number of usable sectors per erase block */
#ifdef CONFIG_MTD_SMART_LOGICAL_SECTOR
  FAR uint8_t          *sBitMap;          /* Virtual sector used bit-map */
  FAR struct smart_cache_s *sCache;       /* Sector cache */
  uint16_t              cache_entries;    /* Number of valid entries in the cache */
  uint16_t              cache_lastlog;    /* Keep track of the last sector accessed */
  uint16_t              cache_lastphys;   /* Keep the physical sector number also */
  uint16_t              cache_nextbirth;  /* Sector cache aging value */
#endif
#ifdef CONFIG_MTD_SMART_SECTOR_ERASE_DEBUG
  FAR uint8_t          *erasecounts;      /* Number of erases for each erase block */
#endif
#ifdef CONFIG_MTD_SMART_ALLOC_DEBUG
  size_t                bytesalloc;
  struct smart_alloc_s  alloc[SMART_MAX_ALLOCS];   /* Array of memory allocations */
#endif
};

/* Format 1 sector header definition */

#if SMART_STATUS_VERSION == 1
#define SMART_FMT_VERSION           1
struct smart_sect_header_s
{
#ifdef CONFIG_MTD_SMART_LOGICAL_SECTOR
  uint8_t               logicalsector[2]; /* The logical sector number */
#endif
  uint8_t               seq;              /* Incrementing sequence number */
  uint8_t               crc8;             /* CRC-8 or seq number MSB */
  uint8_t               status;           /* Status of this sector:
                                           * Bit 7:   1 = Not commited
                                           *          0 = commited
                                           * Bit 6:   1 = Not released
                                           *          0 = released
                                           * Bit 5:   Sector CRC enable
                                           * Bit 4-2: Sector size on volume
                                           * Bit 1-0: Format version (0x1) */
};
typedef uint8_t crc_t;

/* Format 2 sector header definition.  This is for a 16-bit CRC */

#elif SMART_STATUS_VERSION == 2
#define SMART_FMT_VERSION           2
struct smart_sect_header_s
{
  uint8_t               logicalsector[2]; /* The logical sector number */
  uint8_t               crc16[2];         /* CRC-16 for this sector */
  uint8_t               status;           /* Status of this sector:
                                           * Bit 7:   1 = Not commited
                                           *          0 = commited
                                           * Bit 6:   1 = Not released
                                           *          0 = released
                                           * Bit 5:   Sector CRC enable
                                           * Bit 4-2: Sector size on volume
                                           * Bit 1-0: Format version (0x2) */
  uint8_t               seq;              /* Incrementing sequence number */
};
typedef uint16_t crc_t;

/* Format 3 (32-bit) sector header definition.  Actually, this format
 * isn't used yet and will likely be changed to a format to support
 * NAND devices (possibly with an 18-bit sector size, allowing up to
 * 256K sectors on a larger NAND device, though this would take a fair
 * amount of RAM for management).
 */

#elif SMART_STATUS_VERSION == 3
#error "32-Bit mode not supported yet"
#define SMART_FMT_VERSION           3
struct smart_sect_header_s
{
  uint8_t               logicalsector[4]; /* The logical sector number */
  uint8_t               crc32[4];         /* CRC-32 for this sector */
  uint8_t               status;           /* Status of this sector:
                                           * Bit 7:   1 = Not commited
                                           *          0 = commited
                                           * Bit 6:   1 = Not released
                                           *          0 = released
                                           * Bit 5:   Sector CRC enable
                                           * Bit 4-2: Sector size on volume
                                           * Bit 1-0: Format version (0x3) */
  uint8_t               seq;              /* Incrementing sequence number */
};
typedef uint32_t crc_t;

#endif


/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     smart_open(FAR struct inode *inode);
static int     smart_close(FAR struct inode *inode);
static ssize_t smart_reload(struct smart_struct_s *dev, FAR uint8_t *buffer,
                 off_t startblock, size_t nblocks);
static ssize_t smart_read(FAR struct inode *inode, unsigned char *buffer,
                 size_t start_sector, unsigned int nsectors);
#ifdef CONFIG_FS_WRITABLE
static ssize_t smart_write(FAR struct inode *inode, const unsigned char *buffer,
                           size_t start_sector, unsigned int nsectors);
static ssize_t smart_bytewrite(FAR struct smart_struct_s *dev, size_t offset,
                               int nbytes, FAR const uint8_t *buffer);
#endif
static int     smart_geometry(FAR struct inode *inode, struct geometry *geometry);
static int     smart_ioctl(FAR struct inode *inode, int cmd, unsigned long arg);

static uint16_t smart_findfreephyssector(FAR struct smart_struct_s *dev, uint16_t requested);

#ifdef CONFIG_FS_WRITABLE
static int smart_writesector(FAR struct smart_struct_s *dev, FAR struct smart_read_write_s *req);
static inline int smart_allocsector(FAR struct smart_struct_s *dev,
                 uint16_t requested);
static inline int smart_allocsector2(FAR struct smart_struct_s *dev,
                 uint16_t logsector);
#endif
static int smart_readsector(FAR struct smart_struct_s *dev, FAR struct smart_read_write_s *req);

#ifdef CONFIG_SMART_DEV_LOOP
static ssize_t smart_loop_read(FAR struct file *filep, FAR char *buffer,
                 size_t buflen);
static ssize_t smart_loop_write(FAR struct file *filep, FAR const char *buffer,
                 size_t buflen);
static int     smart_loop_ioctl(FAR struct file *filep, int cmd, unsigned long arg);
#endif /* CONFIG_SMART_DEV_LOOP */

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct block_operations g_bops =
{
  smart_open,     /* open     */
  smart_close,    /* close    */
  smart_read,     /* read     */
#ifdef CONFIG_FS_WRITABLE
  smart_write,    /* write    */
#else
  NULL,           /* write    */
#endif
  smart_geometry, /* geometry */
  smart_ioctl     /* ioctl    */
};

#ifdef CONFIG_SMART_DEV_LOOP
static const struct file_operations g_fops =
{
  0,                /* open */
  0,                /* close */
  smart_loop_read,  /* read */
  smart_loop_write, /* write */
  0,                /* seek */
  smart_loop_ioctl  /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  , 0               /* poll */
#endif
};
#endif /* CONFIG_SMART_DEV_LOOP */

#ifdef CONFIG_SMART_DUMP
#ifdef CONFIG_MTD_SMART_LOGICAL_SECTOR
static void smart_dump(FAR struct smart_struct_s *dev, char *message)
{
  printf("** sector mapping: %s\n", message);
  for (uint16_t sector = 0; sector < dev->totalsectors; sector++)
    {
      if (dev->sMap[sector] != SMART_SMAP_INVALID)
          printf("logical -> physical : %d -> %d\n", sector, dev->sMap[sector]);
    }
}
#endif
#endif

static void smart_save_meta(FAR struct smart_struct_s *dev)
{
  /* save sMap at block 1 */

#ifdef CONFIG_MTD_SMART_LOGICAL_SECTOR
  MTD_BWRITE(dev->mtd, dev->mapphyssector, dev->mtdBlksPerSector,
                 (FAR uint8_t *) dev->sMap);
#else
  MTD_WRITE(dev->mtd, dev->mapphyssector * dev->geo.blocksize, (dev->neraseblocks << 1),
                 (FAR uint8_t *) dev->freecount);
#endif

#ifdef CONFIG_MTD_SMART_DEBUG
  ferr("save meta\n");
#endif
}

static void smart_load_meta(FAR struct smart_struct_s *dev)
{
  /* load sMap at block 1 */

  uint16_t readaddress = dev->mapphyssector * dev->geo.blocksize;
#ifdef CONFIG_MTD_SMART_LOGICAL_SECTOR
  MTD_READ(dev->mtd, readaddress,
           sizeof(uint16_t) * dev->totalsectors + (dev->neraseblocks << 1),
           (FAR uint8_t *) dev->sMap);
#else
  MTD_READ(dev->mtd, readaddress, (dev->neraseblocks << 1), (FAR uint8_t *) dev->freecount);
#endif

#ifdef CONFIG_MTD_SMART_DEBUG
  ferr("load meta\n");
#endif
}

static uint16_t smart_get_freesectors(FAR struct smart_struct_s *dev)
{
  uint16_t freesectors = 0;

  for (int block = 0; block < dev->neraseblocks; block++)
  {
    if ((dev->freecount[block] > 0) && (dev->freecount[block] <= dev->sectorsPerBlk)) {
        freesectors += dev->freecount[block];
    }
#ifdef CONFIG_MTD_SMART_DEBUG
    else if (dev->freecount[block] == SMART_FREECOUNT_BADBLOCK) {
      ferr("Bad block: block %d\n", block);
    } else {
      ferr("Unknown freecount: block %d freecount %d\n", block, dev->freecount[block]);
    }
#endif
  }

  return freesectors;
}

/****************************************************************************
 * Name: smart_open
 *
 * Description: Open the block device
 *
 ****************************************************************************/

static int smart_open(FAR struct inode *inode)
{
  finfo("Entry\n");
  return OK;
}

/****************************************************************************
 * Name: smart_close
 *
 * Description: close the block device
 *
 ****************************************************************************/

static int smart_close(FAR struct inode *inode)
{
  FAR struct smart_struct_s *dev;

  finfo("Entry\n");

  dev = (struct smart_struct_s *)inode->i_private;

  smart_save_meta(dev);

  int ret = MTD_IOCTL(dev->mtd, MTDIOC_FLUSH, 0);
  if (ret < 0)
    {
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: smart_malloc
 *
 * Description:  Perform allocations and keep track of amount of allocated
 *               memory for this context.
 *
 ****************************************************************************/

#ifdef CONFIG_MTD_SMART_ALLOC_DEBUG
FAR static void *smart_malloc(FAR struct smart_struct_s *dev,
                    size_t bytes, const char *name)
{
  FAR void *ret = kmm_malloc(bytes);
  uint8_t x;

  /* Test if we are allocating the dev struct */

  if (dev == NULL)
    {
      dev = ret;
      dev->bytesalloc = 0;
      for (x = 0; x < SMART_MAX_ALLOCS; x++)
        {
          dev->alloc[x].ptr = NULL;
        }
    }

  /* Keep track of the total allocation */

  if (ret != NULL)
    {
      dev->bytesalloc += bytes;
    }

  /* Keep track of individual allocs */

  for (x = 0; x < SMART_MAX_ALLOCS; x++)
    {
      if (dev->alloc[x].ptr == NULL)
        {
          dev->alloc[x].ptr = ret;
          dev->alloc[x].size = bytes;
          dev->alloc[x].name = name;
          break;
        }
    }

  finfo("SMART alloc: %ld\n", dev->bytesalloc);
  return ret;
}
#endif

/****************************************************************************
 * Name: smart_zalloc
 *
 * Description:  Perform allocations and keep track of amount of allocated
 *               memory for this context.
 *
 ****************************************************************************/

#ifdef CONFIG_MTD_SMART_ALLOC_DEBUG
FAR static void *smart_zalloc(FAR struct smart_struct_s *dev,
                    size_t bytes, const char *name)
{
  void *mem;

  mem = smart_malloc(dev, bytes, name);
  if (mem != NULL)
    {
      memset(mem, 0, bytes);
    }

  return mem;
}
#endif

/****************************************************************************
 * Name: smart_free
 *
 * Description:  Perform smart memory free operation.
 *
 ****************************************************************************/

#ifdef CONFIG_MTD_SMART_ALLOC_DEBUG
static void smart_free(FAR struct smart_struct_s *dev, FAR void *ptr)
{
  uint8_t x;

  for (x = 0; x < SMART_MAX_ALLOCS; x++)
    {
      if (dev->alloc[x].ptr == ptr)
        {
          dev->alloc[x].ptr = NULL;
          dev->bytesalloc -= dev->alloc[x].size;
          kmm_free(ptr);
          break;
        }
    }
}
#endif

/****************************************************************************
 * Name: smart_checkfree
 *
 * Description: A debug routine for validating the free sector count used
 *              during development of the wear leveling code.
 *
 ****************************************************************************/

#ifdef CONFIG_SMART_LOCAL_CHECKFREE
int smart_checkfree(FAR struct smart_struct_s *dev, int lineno)
{
  uint16_t        x, freecount;
#ifdef CONFIG_DEBUG_FS
  uint16_t        blockfree, blockrelease;
  static uint16_t prev_freesectors = 0;
  static uint16_t prev_releasesectors = 0;
  static uint8_t  *prev_freecount = NULL;
  static uint8_t  *prev_releasecount = NULL;
#endif
  uint16_t        freesectors = smart_get_freesectors(dev);

  freecount = 0;
  for (x = 0; x < dev->neraseblocks; x++)
    {
      freecount += dev->freecount[x];
    }

  /* Test if the calculated freesectors equals the reported value */

#ifdef CONFIG_DEBUG_FS
  if (freecount != freesectors)
    {
      fwarn("WARNING: Free count incorrect in line %d!  Calculated=%d, freesectors=%d\n",
           lineno, freecount, freesectors);

      /* Determine what changed from the last time which caused this error */

      fwarn("   ... Prev freesectors=%d, prev releasesectors=%d\n",
           prev_freesectors, prev_releasesectors);

      if (prev_freecount)
        {
          for (x = 0; x < dev->neraseblocks; x++)
            {
              blockfree = dev->freecount[x];
              blockrelease = dev->releasecount[x];
              if (prev_freecount[x] != blockfree || prev_releasecount[x] != blockrelease)
                {
                  /* This block's values are different from the last time ... report it */

                  fwarn("   ... Block %d:  Old Free=%d, old release=%d,    New free=%d, new release = %d\n",
                      x, prev_freecount[x], prev_releasecount[x], blockfree, blockrelease);
                }
            }
        }

      /* Modifiy the freesector count to reflect the actual calculated freecount
       * to get us back in line.
       */

      return -EIO;
    }

  /* Make a copy of the freecount and releasecount arrays to compare the
   * differences between successive calls so we can evaluate what changed
   * in the event an error is detected.
   */

  if (prev_freecount == NULL)
    {
      prev_freecount = (FAR uint8_t *) smart_malloc(dev, dev->neraseblocks << 1, "Free backup");
      prev_releasecount = prev_freecount + dev->neraseblocks;
    }

  if (prev_freecount != NULL)
    {
      for (x = 0; x < dev->neraseblocks; x++)
        {
          prev_freecount[x] = dev->freecount[x];
          prev_releasecount[x] = dev->releasecount[x];
        }
    }

  /* Save the previous freesectors count */

  prev_freesectors = freesectors;
  prev_releasesectors = dev->releasesectors;
#endif

  return OK;
}
#endif

/****************************************************************************
 * Name: smart_reload
 *
 * Description:  Read the specified numer of sectors
 *
 ****************************************************************************/

static ssize_t smart_reload(struct smart_struct_s *dev, FAR uint8_t *buffer,
                            off_t startblock, size_t nblocks)
{
  ssize_t nread;
  ssize_t mtdBlocks, mtdStartBlock;

  /* Calculate the number of MTD blocks to read */

  mtdBlocks = nblocks * dev->mtdBlksPerSector;

  /* Calculate the first MTD block number */

  mtdStartBlock = startblock * dev->mtdBlksPerSector;

  /* Read the full erase block into the buffer */

  finfo("Read %d blocks starting at block %d\n", mtdBlocks, mtdStartBlock);

  nread = MTD_BREAD(dev->mtd, mtdStartBlock, mtdBlocks, buffer);
  if (nread != mtdBlocks)
    {
      ferr("ERROR: Read %d blocks starting at block %d failed: %d\n",
           nblocks, startblock, nread);

      if (nread == -EIO)
        {
          /* Mark it as invalid */
#ifdef CONFIG_MTD_SMART_LOGICAL_SECTOR
          dev->sMap[mtdStartBlock] = SMART_SMAP_INVALID;
#endif
          dev->freecount[mtdStartBlock] = SMART_FREECOUNT_BADBLOCK;
        }
    }

  return nread;
}

/****************************************************************************
 * Name: smart_read
 *
 * Description:  Read the specified numer of sectors
 *
 ****************************************************************************/

static ssize_t smart_read(FAR struct inode *inode, unsigned char *buffer,
                          size_t start_sector, unsigned int nsectors)
{
  FAR struct smart_struct_s *dev;

  finfo("SMART: sector: %d nsectors: %d\n", start_sector, nsectors);

  DEBUGASSERT(inode && inode->i_private);
  dev = (struct smart_struct_s *)inode->i_private;
  return smart_reload(dev, buffer, start_sector, nsectors);
}

/****************************************************************************
 * Name: smart_write
 *
 * Description: Write (or buffer) the specified number of sectors
 *
 ****************************************************************************/

#ifdef CONFIG_FS_WRITABLE
static ssize_t smart_write(FAR struct inode *inode,
                FAR const unsigned char *buffer,
                size_t start_sector, unsigned int nsectors)
{
  FAR struct smart_struct_s *dev;
  off_t  alignedblock;
  off_t  mask;
  off_t  blkstowrite;
  off_t  offset;
  off_t  nextblock;
  off_t  mtdBlksPerErase;
  off_t  eraseblock;
  size_t remaining;
  size_t nxfrd;
  int    ret;
  off_t  mtdstartblock, mtdblockcount;

  finfo("sector: %d nsectors: %d\n", start_sector, nsectors);

  DEBUGASSERT(inode && inode->i_private);
  dev = (FAR struct smart_struct_s *)inode->i_private;

  /* I think maybe we need to lock on a mutex here */

  /* Get the aligned block.  Here is is assumed: (1) The number of R/W blocks
   * per erase block is a power of 2, and (2) the erase begins with that same
   * alignment.
   */

  mask         = dev->sectorsPerBlk - 1;
  alignedblock = ((start_sector + mask) & ~mask) * dev->mtdBlksPerSector;

  /* Convert SMART blocks into MTD blocks */

  mtdstartblock = start_sector * dev->mtdBlksPerSector;
  mtdblockcount = nsectors * dev->mtdBlksPerSector;
  mtdBlksPerErase = dev->mtdBlksPerSector * dev->sectorsPerBlk;

  finfo("mtdsector: %d mtdnsectors: %d\n", mtdstartblock, mtdblockcount);

  /* Start at first block to be written */

  remaining = mtdblockcount;
  nextblock = mtdstartblock;
  offset = 0;

  /* Loop for all blocks to be written */

  while (remaining > 0)
    {
      /* If this is an aligned block, then erase the block */

      if (alignedblock == nextblock)
        {
          /* Erase the erase block */

          eraseblock = alignedblock / mtdBlksPerErase;
          ret = MTD_ERASE(dev->mtd, eraseblock, 1);
          if (ret < 0)
            {
              ferr("ERROR: Erase block %d ret %d\n", eraseblock, ret);

              if (ret == -EIO) {
                dev->freecount[eraseblock] = SMART_FREECOUNT_BADBLOCK;
              }

              /* Unlock the mutex if we add one */

              return ret;
            }
        }

      /* Calculate the number of blocks to write. */

      blkstowrite = mtdBlksPerErase;
      if (nextblock != alignedblock)
        {
          blkstowrite = alignedblock - nextblock;
        }

      if (blkstowrite > remaining)
        {
          blkstowrite = remaining;
        }

      /* Try to write to the sector. */

      finfo("Write MTD block %d from offset %d\n", nextblock, offset);
      nxfrd = MTD_BWRITE(dev->mtd, nextblock, blkstowrite, &buffer[offset]);
      if (nxfrd != blkstowrite)
        {
          /* The block is not empty!!  What to do? */

          ferr("ERROR: Write block %d ret = %d.\n", nextblock, nxfrd);

          if (nxfrd == -EIO) {
            dev->freecount[nextblock] = SMART_FREECOUNT_BADBLOCK;
          }

          /* Unlock the mutex if we add one */

          if (nxfrd < 0)
            return nxfrd;
          else
            return -EIO;
        }

      /* Then update for amount written */

      nextblock += blkstowrite;
      remaining -= blkstowrite;
      offset += blkstowrite * dev->geo.blocksize;
      alignedblock += mtdBlksPerErase;
    }

  return nsectors;
}
#endif /* CONFIG_FS_WRITABLE */

/****************************************************************************
 * Name: smart_geometry
 *
 * Description: Return device geometry
 *
 ****************************************************************************/

static int smart_geometry(FAR struct inode *inode, struct geometry *geometry)
{
  FAR struct smart_struct_s *dev;
  uint32_t  erasesize;

  finfo("Entry\n");

  DEBUGASSERT(inode);
  if (geometry)
    {
      dev = (FAR struct smart_struct_s *)inode->i_private;
      geometry->geo_available     = true;
      geometry->geo_mediachanged  = false;
#ifdef CONFIG_FS_WRITABLE
      geometry->geo_writeenabled  = true;
#else
      geometry->geo_writeenabled  = false;
#endif

      erasesize                   = dev->geo.erasesize;
      geometry->geo_nsectors      = dev->geo.neraseblocks * erasesize /
                                    dev->sectorsize;
      geometry->geo_sectorsize    = dev->sectorsize;

      finfo("available: true mediachanged: false writeenabled: %s\n",
            geometry->geo_writeenabled ? "true" : "false");
      finfo("nsectors: %d sectorsize: %d\n",
            geometry->geo_nsectors, geometry->geo_sectorsize);

      return OK;
    }

  return -EINVAL;
}

/****************************************************************************
 * Name: smart_setsectorsize
 *
 * Description: Sets the device's sector size and recalculates sector size
 *              dependant variables.
 *
 ****************************************************************************/

static int smart_setsectorsize(FAR struct smart_struct_s *dev, uint16_t size)
{
  uint32_t  erasesize;
  uint32_t  totalsectors;
  uint32_t  allocsize;

  /* Validate the size isn't zero so we don't divide by zero below */

  if (size == 0)
    {
      size = CONFIG_MTD_SMART_SECTOR_SIZE;
    }

  if (size == dev->sectorsize)
    {
      return OK;
    }

  erasesize             = dev->geo.erasesize;
  dev->neraseblocks     = dev->geo.neraseblocks;
  dev->erasesize        = erasesize;
  dev->sectorsize       = size;
  dev->mtdBlksPerSector = dev->sectorsize / dev->geo.blocksize;

  DEBUGASSERT(dev->sectorsize >= dev->geo.blocksize);
  DEBUGASSERT(erasesize / dev->sectorsize <= 256);

  if (erasesize / dev->sectorsize > 256)
    {
      /* We can't throw a error message here becasue it is too early.
       * set the erasesize to zero and exit, then we will detect
       * it during mksmartfs or mount.
       */

      dev->erasesize = 0;
      dev->sectorsPerBlk = 256;
      dev->availSectPerBlk = 255;
    }
  else
    {
      /* Set the sectors per erase block and available sectors per erase block */

      dev->sectorsPerBlk = erasesize / dev->sectorsize;
      if (dev->sectorsPerBlk == 256)
        {
          dev->availSectPerBlk = 255;
        }
      else if (dev->sectorsPerBlk == 0)
        {
          return -EINVAL;
        }
      else
        {
          dev->availSectPerBlk = dev->sectorsPerBlk;
        }
    }

#if defined(CONFIG_FS_PROCFS) && !defined(CONFIG_FS_PROCFS_EXCLUDE_SMARTFS)
  dev->unusedsectors = 0;
  dev->blockerases = 0;
#endif

#ifdef CONFIG_MTD_SMART_LOGICAL_SECTOR
  /* Release any existing rwbuffer and sMap */

  if (dev->sMap != NULL)
    {
      smart_free(dev, dev->sMap);
      dev->sMap = NULL;
    }
#else
  if (dev->freecount != NULL)
    {
      smart_free(dev, dev->freecount);
      dev->freecount = NULL;
    }
#endif

#ifndef CONFIG_MTD_BYTE_WRITE
  if (dev->rwbuffer != NULL)
    {
      smart_free(dev, dev->rwbuffer);
      dev->rwbuffer = NULL;
    }
#endif

  /* Allocate a virtual to physical sector map buffer.  Also allocate
   * the storage space for releasecount and freecounts.
   */

  totalsectors = dev->neraseblocks * dev->sectorsPerBlk;

  /* Validate the number of total sectors is small enough for a uint16_t */

  if (totalsectors > 65536)
    {
      ferr("ERROR: Invalid SMART sector count %ld\n", totalsectors);
      return -EINVAL;
    }
  else if (totalsectors == 65536)
    {
      /* Special case.  We allow 65536 sectors and simply waste 2 sectors
       * to allow a smaller sector size with almost maximum flash usage.
       */

      totalsectors -= 2;
    }

  dev->totalsectors = (uint16_t) totalsectors;

  allocsize = dev->neraseblocks << 1;
#ifdef CONFIG_MTD_SMART_LOGICAL_SECTOR
  dev->sMap = (FAR uint16_t *) smart_malloc(dev, totalsectors * sizeof(uint16_t) +
              allocsize, "Sector map");
  if (!dev->sMap)
    {
      ferr("ERROR: Error allocating SMART virtual map buffer\n");
      goto errexit;
    }

  dev->releasecount = (FAR uint8_t *) dev->sMap + (totalsectors * sizeof(uint16_t));
  dev->freecount = dev->releasecount + dev->neraseblocks;
#else
  dev->freecount = (FAR uint8_t *) smart_malloc(dev, allocsize, "Release count");
#endif

  dev->rootphyssector = SMART_SMAP_INVALID;
  dev->mapphyssector = SMART_SMAP_INVALID;

#ifdef CONFIG_MTD_SMART_SECTOR_ERASE_DEBUG
  /* Allocate a buffer to hold the erase counts */

  if (dev->erasecounts == NULL)
    {
      dev->erasecounts = (FAR uint8_t *) smart_malloc(dev, dev->neraseblocks, "Erase counts");
    }

  if (!dev->erasecounts)
    {
      ferr("ERROR: Error allocating erase count array\n");
      goto errexit;
    }

  memset(dev->erasecounts, 0, dev->neraseblocks);
#endif

#ifndef CONFIG_MTD_BYTE_WRITE
  /* Allocate a read/write buffer */

  dev->rwbuffer = (FAR char *) smart_malloc(dev, size, "RW Buffer");
  if (!dev->rwbuffer)
    {
      ferr("ERROR: Error allocating SMART read/write buffer\n");
      goto errexit;
    }
#endif

  return OK;

  /* On error for any allocation, we jump here and free anything that had
   * previously been allocated.
   */

#ifdef CONFIG_MTD_SMART_LOGICAL_SECTOR
errexit:
#endif

#ifdef CONFIG_MTD_SMART_LOGICAL_SECTOR
  if (dev->sMap)
    {
      smart_free(dev, dev->sMap);
      dev->sMap = NULL;
    }
#endif

#ifdef CONFIG_MTD_SMART_SECTOR_ERASE_DEBUG
  if (dev->erasecounts)
    {
      smart_free(dev, dev->erasecounts);
      dev->erasecounts = NULL;
    }
#endif

  return -ENOMEM;
}

/****************************************************************************
 * Name: smart_bytewrite
 *
 * Description: Writes a non-page size count of bytes to the underlying
 *              MTD device.  If the MTD driver supports a direct impl of
 *              write, then it uses it, otherwise it does a read-modify-write
 *              and depends on the architecture of the flash to only program
 *              bits that actually changed.
 *
 ****************************************************************************/

static ssize_t smart_bytewrite(FAR struct smart_struct_s *dev, size_t offset,
        int nbytes, FAR const uint8_t *buffer)
{
  ssize_t       ret;

#ifdef CONFIG_MTD_BYTE_WRITE
  /* Use the MTD's write method to write individual bytes */

  ret = MTD_WRITE(dev->mtd, offset, nbytes, buffer);

#else
      /* Perform block-based read-modify-write */

      uint32_t  startblock;
      uint16_t  nblocks;

      /* First calculate the start block and number of blocks affected */

      startblock = offset / dev->geo.blocksize;
      nblocks    = (offset - startblock * dev->geo.blocksize + nbytes +
                    dev->geo.blocksize-1) / dev->geo.blocksize;

      DEBUGASSERT(nblocks <= dev->mtdBlksPerSector);

      /* Do a block read */

      ret = MTD_BREAD(dev->mtd, startblock, nblocks, (FAR uint8_t *) dev->rwbuffer);
      if (ret < 0)
      {
        ferr("ERROR: Error %d reading from device\n", -ret);
        goto errout;
      }

      /* Modify the data */

      memcpy(&dev->rwbuffer[offset - startblock * dev->geo.blocksize], buffer, nbytes);

      /* Write the data back to the device */

      ret = MTD_BWRITE(dev->mtd, startblock, nblocks, (FAR uint8_t *) dev->rwbuffer);
      if (ret < 0)
        {
          ferr("ERROR: Error %d writing to device\n", -ret);
          goto errout;
        }

  ret = nbytes;
#endif

#ifndef CONFIG_MTD_BYTE_WRITE
errout:
#endif

  return ret;
}

/****************************************************************************
 * Name: smart_scan
 *
 * Description: Performs a scan of the MTD device searching for format
 *              information and fills in logical sector mapping, freesector
 *              count, etc.
 *
 ****************************************************************************/

static int smart_scan(FAR struct smart_struct_s *dev, bool is_format)
{
  int       physsector;
  int       ret;
  uint16_t  totalsectors;
  uint16_t  sectorsize;
  uint16_t  prerelease;
#ifdef CONFIG_MTD_SMART_LOGICAL_SECTOR
  uint16_t  logicalsector;
#endif
  uint32_t  readaddress;
  struct    smart_sect_header_s header;

  finfo("Entry\n");

  sectorsize = CONFIG_MTD_SMART_SECTOR_SIZE;
  ret = smart_setsectorsize(dev, sectorsize);
  if (ret != OK)
    {
      goto err_out;
    }

  /* Initialize the device variables */

  totalsectors = dev->totalsectors;
  dev->formatstatus = SMART_FMT_STAT_NOFMT;
  dev->releasesectors = 0;
  dev->rootphyssector = SMART_SMAP_INVALID;
  dev->mapphyssector = SMART_SMAP_INVALID;

  /* Initialize the freecount and releasecount arrays */

  for (physsector = 0; physsector < dev->neraseblocks; physsector++)
    {
      if (physsector == dev->neraseblocks - 1 && dev->totalsectors == 65534)
        {
          prerelease = 2;
        }
      else
        {
          prerelease = 0;
        }

      dev->freecount[physsector / dev->sectorsPerBlk] = dev->availSectPerBlk - prerelease;
#ifdef CONFIG_MTD_SMART_LOGICAL_SECTOR
      dev->releasecount[physsector / dev->sectorsPerBlk] = prerelease;
#endif
    }

  /* Initialize the sector map */

#ifdef CONFIG_SMART_LOGICAL_SECTOR
  for (physsector = 0; physsector < totalsectors; physsector++)
    {
      dev->sMap[physsector] = SMART_SMAP_INVALID;
    }
#endif

#ifdef CONFIG_MTD_SMART_DEBUG
  int totalfreecount = 0;
  ferr("===============neraseblocks = %d\n", dev->neraseblocks);
  for (int i=0; i<dev->neraseblocks; i++) {
    if (dev->freecount[i] > dev->sectorsPerBlk) {
      ferr("Abnormal freecount: block %d freecount %d\n", i, dev->freecount[i]);
    }
    totalfreecount += dev->freecount[i];
  }
  ferr("Total sectors: %d, free sectors: %d\n", totalsectors, totalfreecount);
#endif

  /* Now scan the MTD device */

  for (physsector = 0; physsector < totalsectors; physsector++)
    {

      /* Find root sector from sector 0 */

      while (dev->rootphyssector == SMART_SMAP_INVALID)
        {
          /* Read the sector data */
          FAR uint8_t headerbuf[SMART_SIGNATURE_SIZE];

          readaddress = physsector * dev->mtdBlksPerSector * dev->geo.blocksize;

          ret = MTD_READ(dev->mtd, readaddress, SMART_SIGNATURE_SIZE,
                         (FAR uint8_t *)headerbuf);
          if (ret != SMART_SIGNATURE_SIZE)
            {
              ferr("ERROR: Error reading physical sector %d, ret = %d\n", physsector, ret);

              /* Skip current bad block */
              if (ret == -EIO) {
                dev->freecount[physsector] = SMART_FREECOUNT_BADBLOCK;

                physsector++;
#ifdef CONFIG_MTD_SMART_LOGICAL_SECTOR
                if (physsector >= totalsectors)
#else
                if (physsector >= (SMART_FIRST_ALLOC_SECTOR-1))
#endif
                  goto err_out;

                continue;
              }
              else
                goto err_out;
            }

          uint16_t smart_internal_version = *((uint16_t *)(headerbuf+SMART_FMT_INTERNAL_VERSION_POS));

          /* Validate the format signature */

          if (headerbuf[SMART_FMT_POS1] != SMART_FMT_SIG1 ||
              headerbuf[SMART_FMT_POS2] != SMART_FMT_SIG2 ||
              headerbuf[SMART_FMT_POS3] != SMART_FMT_SIG3 ||
              headerbuf[SMART_FMT_POS4] != SMART_FMT_SIG4 ||
              smart_internal_version != SMART_INTERNAL_VERSION)
            {
              /* Invalid signature on a sector claiming to be sector 0!
               * What should we do?  Release it?
               */

              if (!is_format) {
                if (smart_internal_version != SMART_INTERNAL_VERSION)
                  ferr("ERROR: Error reading internal version %d, expect %d.\n",
                       smart_internal_version, SMART_INTERNAL_VERSION);
                else
                  ferr("ERROR: Error reading signature in physical sector %d.\n", physsector);
              }

              /* The root sector is not formatted yet */
              dev->rootphyssector = physsector;
              dev->mapphyssector = physsector+1;

              /* Return error to let uppper layer format the partition */
              goto err_out;
            }

          /* Mark the volume as formatted and set the sector size */

          dev->formatstatus = SMART_FMT_STAT_FORMATTED;
          dev->namesize = headerbuf[SMART_FMT_NAMESIZE_POS];
          dev->formatversion = headerbuf[SMART_FMT_VERSION_POS];

          dev->rootphyssector = physsector;

          /* We asssume map sector is on the same block as root sector,
             so map sector is on good block if root sector is found */
          dev->mapphyssector = physsector+1;

          if (!is_format)
            smart_load_meta(dev);

          dev->entryphyssector = physsector+2;

#ifdef CONFIG_MTD_SMART_LOGICAL_SECTOR
          dev->sMap[SMART_SIGNATURE_SECTOR] = dev->rootphyssector;
          dev->sMap[SMART_METADATA_SECTOR] = dev->rootphyssector+1;
#endif
          break;
        }

      /* Do not do full scan to save boot time */
      if (!is_format) {

        /* Do not scan block if freecount is normal or bad block */
        if ((dev->freecount[physsector / dev->sectorsPerBlk] >= 0) &&
            (dev->freecount[physsector / dev->sectorsPerBlk] <= dev->sectorsPerBlk)) {
          continue;
        } else if (dev->freecount[physsector / dev->sectorsPerBlk] == SMART_FREECOUNT_BADBLOCK) {
          continue;
        } else {
          /* There is problem in freecount of this block.
             Reset freecount and do physical sector in following logic */
          ferr("Abnormal freecount found: block %d count %d\n", physsector / dev->sectorsPerBlk,
                dev->freecount[physsector / dev->sectorsPerBlk]);
          dev->freecount[physsector / dev->sectorsPerBlk] = dev->availSectPerBlk;
        }
      }

      finfo("Scan phyical sector %d\n", physsector);

      /* Calculate the read address for this sector */

      readaddress = physsector * dev->mtdBlksPerSector * dev->geo.blocksize;

      /* Read the header for this sector */

      ret = MTD_READ(dev->mtd, readaddress, sizeof(struct smart_sect_header_s),
                     (FAR uint8_t *) &header);
      if (ret != sizeof(struct smart_sect_header_s))
        {
          ferr("Error: Read physical sector %d error, ret = %d\n", physsector, ret);
          if (ret == -EIO) {
            dev->freecount[physsector / dev->sectorsPerBlk] = SMART_FREECOUNT_BADBLOCK;
          }
          continue;
        }

#ifdef CONFIG_MTD_SMART_LOGICAL_SECTOR
      /* Get the logical sector number for this physical sector */

      logicalsector = *((FAR uint16_t *) header.logicalsector);
#endif

      /* Test if this sector has been release and if it has,
       * update the erase block's releasecount.
       */

      if ((header.status & SMART_STATUS_RELEASED) !=
              (CONFIG_SMARTFS_ERASEDSTATE & SMART_STATUS_RELEASED))
        {
          /* Keep track of the total number of released sectors and
           * released sectors per erase block.
           */

          dev->releasesectors++;
#ifdef CONFIG_MTD_SMART_LOGICAL_SECTOR
          dev->releasecount[physsector / dev->sectorsPerBlk]++;
#endif
          continue;
        }

      /* Test if this sector has been committed */

      if ((header.status & SMART_STATUS_COMMITTED) ==
              (CONFIG_SMARTFS_ERASEDSTATE & SMART_STATUS_COMMITTED))
        {
          /* uncommited, erased */
          continue;
        }

      /* used */

      /* This block is commited, therefore not free. Update the
       * erase block's freecount.
       */

      if (dev->freecount[physsector / dev->sectorsPerBlk] > 0)
        dev->freecount[physsector / dev->sectorsPerBlk]--;
      if ((header.status & SMART_STATUS_VERBITS) != SMART_STATUS_VERSION)
        {
          continue;
        }

#ifdef CONFIG_MTD_SMART_LOGICAL_SECTOR
      /* Validate the logical sector number is in bounds */

      if (logicalsector >= totalsectors)
        {
          /* Error in logical sector read from the MTD device */

          ferr("ERROR: Invalid logical sector %d at physical %d.\n",
               logicalsector, physsector);
          continue;
        }

      /* Update the logical to physical sector map */

      dev->sMap[logicalsector] = physsector;
#endif
    }

  ferr("SMART Scan\n");
  ferr("   Erase size:   %10d\n", dev->sectorsPerBlk * dev->sectorsize);
  ferr("   Erase count:  %10d\n", dev->neraseblocks);
  ferr("   Sect/block:   %10d\n", dev->sectorsPerBlk);
  ferr("   MTD Blk/Sect: %10d\n", dev->mtdBlksPerSector);
  ferr("   Free setors:  %10d\n", smart_get_freesectors(dev));

  /* Validate the geometry */

  if (dev->mtdBlksPerSector == 0 || dev->sectorsPerBlk == 0 ||
      dev->sectorsPerBlk == 0 || dev->sectorsize == 0)
    {
      ferr("ERROR: Invalid Geometry!\n");
      ret = -EINVAL;
      goto err_out;
    }

#ifdef CONFIG_MTD_SMART_ALLOC_DEBUG
  finfo("   Allocations:\n");
  for (int alloc = 0; alloc < SMART_MAX_ALLOCS; alloc++)
    {
      if (dev->alloc[alloc].ptr != NULL)
        {
          finfo("       %s: %d\n", dev->alloc[alloc].name, dev->alloc[alloc].size);
        }
    }
#endif

  ret = OK;

err_out:
  return ret;
}

/****************************************************************************
 * Name: smart_getformat
 *
 * Description:  Populates the SMART format structure based on the format
 *               information for the inode.
 *
 ****************************************************************************/

static inline int smart_getformat(FAR struct smart_struct_s *dev,
                                  FAR struct smart_format_s *fmt)
{
  int ret;

  finfo("Entry\n");
  DEBUGASSERT(fmt);

  /* Test if we know the format status or not.  If we don't know the
   * status, then we must perform a scan of the device to search
   * for the format marker
   */

  if (dev->formatstatus != SMART_FMT_STAT_FORMATTED)
    {
      /* Perform the scan */

      ret = smart_scan(dev, false);

      if (ret != OK)
        {
          goto err_out;
        }
    }

  /* Now fill in the structure */

  if (dev->formatstatus == SMART_FMT_STAT_FORMATTED)
    {
      fmt->flags = SMART_FMT_ISFORMATTED;
    }
  else
    {
      fmt->flags = 0;
    }

  fmt->sectorsize = dev->sectorsize;
  fmt->availbytes = dev->sectorsize - sizeof(struct smart_sect_header_s);
  fmt->nsectors = dev->totalsectors;

  fmt->nfreesectors = smart_get_freesectors(dev);
  fmt->namesize = dev->namesize;
  fmt->entrysector = dev->entryphyssector;

  /* Add the released sectors to the reported free sector count */

  //fmt->nfreesectors += dev->releasesectors;

  /* Subtract the reserved sector count */

  //fmt->nfreesectors -= dev->sectorsPerBlk + 4;

  ret = OK;

err_out:
  return ret;
}

/****************************************************************************
 * Name: smart_llformat
 *
 * Description:  Performs a low-level format of the flash device.  This
 *               involves erasing the device and writing a valid sector
 *               zero (logical) with proper format signature.
 *
 * Input Parameters:
 *
 *   arg:        Upper 16 bits contains the sector size
 *               Lower 16 bits contains the number of root dir entries
 *
 ****************************************************************************/

#ifdef CONFIG_FS_WRITABLE
static inline int smart_llformat(FAR struct smart_struct_s *dev, unsigned long arg)
{
  FAR struct  smart_sect_header_s  *sectorheader;
  size_t      wrcount;
  int         x;
  int         ret;
  uint8_t     sectsize, prerelease;
  uint16_t    sectorsize = CONFIG_MTD_SMART_SECTOR_SIZE;

  finfo("Entry\n");

  /* Check for invalid format */

  if (dev->erasesize == 0 || dev->sectorsPerBlk == 0)
    {
      dev->erasesize = dev->geo.erasesize;

      ferr("ERROR:  Invalid geometery ... Sectors per erase block must be 1-256\n");
      ferr("        Erase block size    = %d\n", dev->erasesize);
      ferr("        Sector size         = %d\n", dev->sectorsize);
      ferr("        Sectors/erase block = %d\n", dev->erasesize / dev->sectorsize);

      return -EINVAL;
    }

  /* Erase the MTD device */

  ret = MTD_IOCTL(dev->mtd, MTDIOC_BULKERASE, 0);
  if (ret < 0)
    {
      return ret;
    }

  /* Do a scan of the device */

  ret = smart_scan(dev, true);
  if (ret < 0)
    {
      ferr("ERROR: smart_scan failed: %d\n", -ret);
      return ret;
    }

#ifdef CONFIG_MTD_BYTE_WRITE
  char headerbuf[SMART_SIGNATURE_SIZE];
  memset(headerbuf, CONFIG_SMARTFS_ERASEDSTATE, SMART_SIGNATURE_SIZE);
#else
  char *headerbuf = dev->rwbuffer;
  memset(dev->rwbuffer, CONFIG_SMARTFS_ERASEDSTATE, dev->sectorsize);
#endif

  /* Now construct a logical sector zero header to write to the device. */

  sectorheader = (FAR struct smart_sect_header_s *) headerbuf;

#if SMART_STATUS_VERSION == 1
  /* CRC not enabled.  Using a 16-bit sequence number */

  *((FAR uint16_t *) &sectorheader->seq) = 0;
#else   /* SMART_STATUS_VERSION == 1 */
  sectorheader->seq = 0;
#endif  /* SMART_STATUS_VERSION == 1 */

  /* Set the sector size of this sector */

  sectsize = (sectorsize >> 9) << 2;

  /* Set the sector logical sector to zero and setup the header status */

#ifdef CONFIG_MTD_SMART_LOGICAL_SECTOR
  *((FAR uint16_t *) sectorheader->logicalsector) = 0;
#endif
  sectorheader->status = (uint8_t) ~(SMART_STATUS_COMMITTED | SMART_STATUS_VERBITS |
          SMART_STATUS_SIZEBITS) | SMART_STATUS_VERSION |
          sectsize;

  /* Now add the format signature to the sector */

  headerbuf[SMART_FMT_POS1] = SMART_FMT_SIG1;
  headerbuf[SMART_FMT_POS2] = SMART_FMT_SIG2;
  headerbuf[SMART_FMT_POS3] = SMART_FMT_SIG3;
  headerbuf[SMART_FMT_POS4] = SMART_FMT_SIG4;

  headerbuf[SMART_FMT_VERSION_POS] = SMART_FMT_VERSION;
  *((uint16_t *)(headerbuf+SMART_FMT_INTERNAL_VERSION_POS)) = SMART_INTERNAL_VERSION;
  headerbuf[SMART_FMT_NAMESIZE_POS] = CONFIG_SMARTFS_MAXNAMLEN;

  /* Record the number of root directory entries we have */

  headerbuf[SMART_FMT_ROOTDIRS_POS] = (uint8_t) (arg & 0xff);

#ifdef CONFIG_MTD_BYTE_WRITE

  wrcount = MTD_WRITE(dev->mtd, dev->rootphyssector, SMART_SIGNATURE_SIZE, headerbuf);
  if (wrcount != SMART_SIGNATURE_SIZE) {
      ferr("ERROR: write signature failed\n");
      if (wrcount < 0)
        return wrcount;
      else
        return -EFAULT;
  }

#else
  /* Write the sector to the flash */

  wrcount = MTD_BWRITE(dev->mtd, dev->rootphyssector, dev->mtdBlksPerSector,
          (FAR uint8_t *) headerbuf);
  if (wrcount != dev->mtdBlksPerSector)
    {
      /* The block is not empty!!  What to do? */

      ferr("ERROR: Write block 0 failed: %d.\n", wrcount);

      /* Unlock the mutex if we add one */

      if (wrcount < 0)
        return wrcount;
      else
        return -EFAULT;
    }
#endif

  dev->formatstatus = SMART_FMT_STAT_UNKNOWN;
  dev->releasesectors = 0;

  /* Initialize the released and free counts */

  for (x = 0; x < dev->neraseblocks; x++)
    {
      /* Test for a geometry with 65536 sectors.  We allow this, though
       * we never use the last two sectors in this mode.
       */

      if (x == dev->neraseblocks && dev->totalsectors == 65534)
        {
          prerelease = 2;
        }
      else
        {
          prerelease = 0;
        }
#ifdef CONFIG_MTD_SMART_LOGICAL_SECTOR
      dev->releasecount[x] = prerelease;
#endif
      dev->freecount[x] = dev->availSectPerBlk-prerelease;
    }

  /* Account for the format sector */

  if (dev->freecount[dev->rootphyssector] > 0)
    dev->freecount[dev->rootphyssector]--;

  /* Now initialize the logical to physical sector map */

#ifdef CONFIG_MTD_SMART_LOGICAL_SECTOR
  dev->sMap[SMART_SIGNATURE_SECTOR] = 0;     /* Logical sector zero = physical sector 0 */
  for (x = 1; x < dev->totalsectors; x++)
    {
      /* Mark all other logical sectors as non-existant */

      dev->sMap[x] = SMART_SMAP_INVALID;
    }
#endif

#ifdef CONFIG_MTD_SMART_LOGICAL_SECTOR
  dev->sMap[SMART_METADATA_SECTOR] = 1;
#endif
  if (dev->freecount[SMART_METADATA_SECTOR] > 0)
    dev->freecount[SMART_METADATA_SECTOR]--;

  smart_save_meta(dev);

  ret = MTD_IOCTL(dev->mtd, MTDIOC_FLUSH, 0);
  if (ret < 0)
    {
      return ret;
    }

  return OK;
}
#endif /* CONFIG_FS_WRITABLE */

/****************************************************************************
 * Name: smart_findfreephyssector
 *
 * Description:  Finds a free physical sector based on free and released
 *               count logic, taking into account reserved sectors.
 *
 ****************************************************************************/

static uint16_t smart_findfreephyssector(FAR struct smart_struct_s *dev,
                           uint16_t requested)
{
  uint16_t  count, allocfreecount, allocblock;
  uint16_t  physicalsector;
  uint16_t  x, block;
  uint32_t  readaddr;
  struct    smart_sect_header_s header;
  int       ret;

  /* Determine which erase block we should allocate the new
   * sector from. This is based on the number of free sectors
   * available in each erase block.
   */

retry:
  allocfreecount = 0;
  allocblock = 0xffff;
  physicalsector = SMART_SMAP_INVALID;

  if (requested >= SMART_SMAP_INVALID2) {
    if (++dev->lastallocblock >= dev->neraseblocks)
      {
#ifdef CONFIG_MTD_SMART_LOGICAL_SECTOR
        dev->lastallocblock = 0;
#else
        dev->lastallocblock = SMART_FIRST_ALLOC_SECTOR/dev->sectorsPerBlk - 1;
#endif
      }

    block = dev->lastallocblock;
  } else {
    block = requested;
  }
#ifdef CONFIG_MTD_SMART_LOGICAL_SECTOR
  for (x = 0; x < dev->neraseblocks; x++)
#else
  for (x = SMART_FIRST_ALLOC_SECTOR/dev->sectorsPerBlk; x < dev->neraseblocks; x++)
#endif
    {
      /* Test if this block has more free blocks than the
       * currently selected block
       */

      count = dev->freecount[block];

      if ((count != SMART_FREECOUNT_BADBLOCK) && (count > allocfreecount))
        {
          /* Assign this block to alloc from */

          if (x < dev->neraseblocks - 1 || !allocfreecount)
            {
              allocblock = block;
              allocfreecount = count;
              break;
            }
        }

      if (++block >= dev->neraseblocks)
        {
#ifdef CONFIG_MTD_SMART_LOGICAL_SECTOR
          block = 0;
#else
          block = SMART_FIRST_ALLOC_SECTOR/dev->sectorsPerBlk;
#endif
        }
    }

  /* Check if we found an allocblock. */

  if (allocblock == 0xffff)
    {
      ferr("ERROR: Cannot find a free sector, free=%d\n", smart_get_freesectors(dev));

      /* No free sectors found!  Bug? */

      physicalsector = SMART_SMAP_INVALID;

      goto errout;
    }

  /* Now find a free physical sector within this selected
   * erase block to allocate. */

  for (x = allocblock * dev->sectorsPerBlk;
       x < allocblock * dev->sectorsPerBlk + dev->availSectPerBlk; x++)
    {
      /* Check if this physical sector is available. */

      /* Now check on the physical media */

      readaddr = x * dev->mtdBlksPerSector * dev->geo.blocksize;
      ret = MTD_READ(dev->mtd, readaddr, sizeof(struct smart_sect_header_s),
              (FAR uint8_t *) &header);
      if (ret != sizeof(struct smart_sect_header_s))
        {
          ferr("ERROR: Error reading phys sector %d, ret = %d\n", physicalsector, ret);

          if (ret == -EIO) {
            /* Mark it as bad block */
            dev->freecount[x / dev->sectorsPerBlk] = SMART_FREECOUNT_BADBLOCK;
          }

          /* Try next sector */
          continue;
        }

      /* Use this sector, if it is not committed or it is released */

      if (((header.status & SMART_STATUS_COMMITTED) ==
           (CONFIG_SMARTFS_ERASEDSTATE & SMART_STATUS_COMMITTED)) ||
           ((header.status & SMART_STATUS_RELEASED) !=
           (CONFIG_SMARTFS_ERASEDSTATE & SMART_STATUS_RELEASED)))
        {
#ifdef CONFIG_MTD_SMART_DEBUG
          if ((header.status & SMART_STATUS_COMMITTED) ==
           (CONFIG_SMARTFS_ERASEDSTATE & SMART_STATUS_COMMITTED)) {
            ferr("choose uncommited physical sector %d\n", x);
          } else {
            ferr("choose released physical sector %d\n", x);
          }
#endif
          physicalsector = x;
          if (requested >= SMART_SMAP_INVALID2) {
            dev->lastallocblock = allocblock;
          }
          break;
        }
    }

  if (physicalsector == SMART_SMAP_INVALID)
    {
      if (dev->freecount[allocblock]>0)
        dev->freecount[allocblock]--;

      ferr("Cannot find free sector in alloc block %d, try next block\n", allocblock);
      goto retry;
    }

  if (physicalsector >= dev->totalsectors)
    {
      ferr("ERROR: Program bug!  Selected sector %d too big!!!\n", physicalsector);
    }

errout:
  return physicalsector;
}

/****************************************************************************
 * Name: smart_write_alloc_sector
 *
 * Description:  Writes a newly allocated sector's header to the RW buffer
 *               and updates sector mapping variables.  If CRC isn't enabled
 *               it also writes the header to the device.
 *
 ****************************************************************************/

#ifdef CONFIG_FS_WRITABLE
static int smart_write_alloc_sector(FAR struct smart_struct_s *dev,
                    uint16_t logical, uint16_t physical)
{
  int       ret = 1;
  uint8_t   sectsize;
  FAR struct smart_sect_header_s  *header;

#ifdef CONFIG_MTD_BYTE_WRITE
  struct smart_sect_header_s header_buf;
  header = (FAR struct smart_sect_header_s *)&header_buf;
  memset(header, CONFIG_SMARTFS_ERASEDSTATE, sizeof(struct smart_sect_header_s));
#else
  memset(dev->rwbuffer, CONFIG_SMARTFS_ERASEDSTATE, dev->sectorsize);
  header = (FAR struct smart_sect_header_s *) dev->rwbuffer;
#endif
#ifdef CONFIG_MTD_SMART_LOGICAL_SECTOR
  *((FAR uint16_t *) header->logicalsector) = logical;
#endif
#if SMART_STATUS_VERSION == 1
  *((FAR uint16_t *) &header->crc8) = 0;
#else
  header->seq = 0;
#endif
  sectsize = dev->sectorsize >> 7;

  header->status = ~(SMART_STATUS_COMMITTED | SMART_STATUS_SIZEBITS |
          SMART_STATUS_VERBITS) | SMART_STATUS_VERSION | sectsize;

  /* Write the header to the physical sector location */

#ifdef CONFIG_MTD_BYTE_WRITE
  MTD_WRITE(dev->mtd, physical * dev->mtdBlksPerSector * dev->geo.blocksize,
            sizeof(struct smart_sect_header_s), (FAR uint8_t *)header);
#else
  finfo("Write MTD block %d\n", physical * dev->mtdBlksPerSector);
  ret = MTD_BWRITE(dev->mtd, physical * dev->mtdBlksPerSector, 1,
      (FAR uint8_t *) dev->rwbuffer);
  if (ret != 1)
    {
      if (ret == -EIO) {
        dev->freecount[physical / dev->sectorsPerBlk] = SMART_FREECOUNT_BADBLOCK;
#ifdef CONFIG_MTD_SMART_LOGICAL_SECTOR
        dev->sMap[logical] = SMART_SMAP_INVALID;
#endif
      }

      ferr("ERROR: Write block %d failed, ret = %d\n", physical *
           dev->mtdBlksPerSector, ret);

      /* Unlock the mutex if we add one */

      return -EIO;
    }
#endif

  return ret;
}
#endif

/****************************************************************************
 * Name: smart_writesector
 *
 * Description:  Writes data to the specified logical sector.  The sector
 *               should have already been allocated prior to the write.  If
 *               the logical sector already has data on the device, it will
 *               be released and a new physical sector will be created and
 *               mapped to the logical sector.
 *
 ****************************************************************************/

#ifdef CONFIG_FS_WRITABLE
static int smart_writesector(FAR struct smart_struct_s *dev,
                    FAR struct smart_read_write_s *req)
{
  int         ret;
  uint32_t    mtdblock;
  uint16_t    physsector;
  size_t      offset;

  finfo("Entry\n");
  DEBUGASSERT(req->offset <= dev->sectorsize);
  DEBUGASSERT(req->offset+req->count <= dev->sectorsize);

  /* Ensure the logical sector has been allocated */

  if (req->logsector >= dev->totalsectors)
    {
      ferr("ERROR: Logical sector %d too large\n", req->logsector);

      ret = -EINVAL;
      goto errout;
    }

#ifdef CONFIG_MTD_SMART_LOGICAL_SECTOR
  physsector = dev->sMap[req->logsector];
#else
  physsector = req->logsector;
#endif
  if (physsector == SMART_SMAP_INVALID)
    {
      ferr("ERROR: Logical sector %d not allocated\n", req->logsector);
      ret = -EINVAL;
      goto errout;
    }

  mtdblock = physsector * dev->mtdBlksPerSector;

#ifndef CONFIG_MTD_BYTE_WRITE
  /* Read the sector data into our buffer */

  ret = MTD_BREAD(dev->mtd, mtdblock, dev->mtdBlksPerSector, (FAR uint8_t *)
        dev->rwbuffer);
  if (ret != dev->mtdBlksPerSector)
  {
    ferr("ERROR: Error reading phys sector %d\n", physsector);
    ret = -EIO;
    goto errout;
  }
#endif

  /* Not relocated.  Just write the portion of the sector that needs
   * to be written.
   */

  offset = mtdblock * dev->geo.blocksize +
      sizeof(struct smart_sect_header_s) + req->offset;
  ret = smart_bytewrite(dev, offset, req->count, req->buffer);
  if (ret == -EIO) {
    dev->freecount[physsector / dev->sectorsPerBlk] = SMART_FREECOUNT_BADBLOCK;
#ifdef CONFIG_MTD_SMART_LOGICAL_SECTOR
    dev->sMap[req->logsector] = SMART_SMAP_INVALID;
#endif
  }

  if (ret != req->count) {
    goto errout;
  }

  ret = OK;

errout:
  return ret;
}
#endif /* CONFIG_FS_WRITABLE */

/****************************************************************************
 * Name: smart_readsector
 *
 * Description:  Reads data from the specified logical sector.  The sector
 *               should have already been allocated prior to the read.
 *
 ****************************************************************************/

static int smart_readsector(FAR struct smart_struct_s *dev,
                    FAR struct smart_read_write_s *req)
{
  int       ret;
  uint16_t  physsector;
  uint32_t  readaddr;

  finfo("Entry\n");
  DEBUGASSERT(req->offset < dev->sectorsize);
  DEBUGASSERT(req->offset+req->count+ sizeof(struct smart_sect_header_s) <=
              dev->sectorsize);

  /* Ensure the logical sector has been allocated */

  if (req->logsector >= dev->totalsectors)
    {
      ferr("ERROR: Logical sector %d too large\n", req->logsector);

      ret = -EINVAL;
      goto errout;
    }

#ifdef CONFIG_MTD_SMART_LOGICAL_SECTOR
  physsector = dev->sMap[req->logsector];
#else
  physsector = req->logsector;
#endif
  if (physsector == SMART_SMAP_INVALID)
    {
#ifndef CONFIG_SMART_DUMP
      ferr("ERROR: Logical sector %d not allocated\n", req->logsector);
#endif
      ret = -EINVAL;
      goto errout;
    }

  /* Read the sector data into the buffer */

  readaddr = (uint32_t) physsector * dev->mtdBlksPerSector * dev->geo.blocksize +
    req->offset + sizeof(struct smart_sect_header_s);

  ret = MTD_READ(dev->mtd, readaddr, req->count, (FAR uint8_t *)
          req->buffer);
  if (ret != req->count)
    {
      ferr("ERROR: Error reading phys sector %d\n", physsector);
      ret = -EIO;
      goto errout;
    }

errout:
    return ret;
}

/****************************************************************************
 * Name: smart_allocsector
 *
 * Description:  Allocates a new logical sector.  If an argument is given,
 *               then it tries to allocate the specified sector number.
 *
 ****************************************************************************/

#ifdef CONFIG_FS_WRITABLE
static inline int smart_allocsector(FAR struct smart_struct_s *dev,
                    uint16_t requested)
{
  int       ret;
  uint16_t  logsector = SMART_SMAP_INVALID; /* Logical sector number selected */
#ifdef CONFIG_MTD_SMART_LOGICAL_SECTOR
  int       x;
  uint16_t  freesectors = smart_get_freesectors(dev);

  /* Validate that we have enough sectors available to perform an
   * allocation.  We have to ensure we keep enough reserved sectors
   * on hand to do released sector garbage collection.
   */

  if (freesectors <= (dev->sectorsPerBlk << 0) + 4)
    return -ENOSPC;

  /* Check if a specific sector is being requested and allocate that
   * sector if it isn't already in use.
   */

  if ((requested > 0) && (requested < dev->totalsectors))
    {
      /* Validate the sector is not already allocated */

      if (dev->sMap[requested] == (uint16_t) SMART_SMAP_INVALID)
        {
          logsector = requested;
        }
    }

  /* Check if we need to scan for an available logical sector */

  if (logsector == SMART_SMAP_INVALID || logsector == SMART_SMAP_INVALID2)
    {
      /* Loop through all sectors and find one to allocate */

      for (x = SMART_FIRST_ALLOC_SECTOR; x < dev->totalsectors; x++)
        {
          if (dev->sMap[x] == (uint16_t) SMART_SMAP_INVALID)
            {
              /* Unused logical sector found.  Use this one */

              logsector = x;
              break;
            }
        }
    }

  /* Test for an error allocating a sector */

  if (logsector == SMART_SMAP_INVALID)
    {
      /* Hmmm.  We think we had enough logical sectors, but
       * something happened and we didn't find any free
       * logical sectors.  What do do?  Report an error?
       * rescan and try again to "self heal" in case of a
       * bug in our code?
       */

      ferr("ERROR: No free logical sector numbers!  Free sectors = %d\n",
              freesectors);

      return -ENOSPC;
    }
#else
    /* Find a free physical sector */

    logsector  = smart_findfreephyssector(dev, requested);
    if (logsector == SMART_SMAP_INVALID) {
      ret = -ENOSPC;
      goto errout;
    }
#endif

   if (requested != SMART_SMAP_INVALID2) {
     ret = smart_allocsector2(dev, logsector);
     if (ret != OK)
       goto errout;
   }

   /* Return the logical sector number */
   ret = (int)logsector;

errout:
   return ret;
}

static inline int smart_allocsector2(FAR struct smart_struct_s *dev, uint16_t logsector)
{
  uint16_t  physicalsector;     /* The selected physical sector */
  int       ret=OK;

  if (logsector == SMART_SMAP_INVALID)
     return -ENOSPC;

  /* Find a free physical sector */

#ifdef CONFIG_MTD_SMART_LOGICAL_SECTOR
  physicalsector = smart_findfreephyssector(dev, SMART_SMAP_INVALID);
#else
  physicalsector = logsector;
#endif
  finfo("Alloc: log=%d, phys=%d, erase block=%d, free=%d, released=%d\n",
          logsector, physicalsector, physicalsector /
          dev->sectorsPerBlk, smart_get_freesectors(dev), dev->releasecount);

  if (physicalsector == SMART_SMAP_INVALID)
    {
      return -ENOSPC;
    }


  /* Write the logical sector to the flash.  We will fill it in with data later. */

  ret = smart_write_alloc_sector(dev, logsector, physicalsector);
  if (ret != 1)
    {
      /* Error writing sector, return error */

      return ret;
    }

  /* Map the sector and update the free sector counts */

#ifdef CONFIG_MTD_SMART_LOGICAL_SECTOR
  dev->sMap[logsector] = physicalsector;
#endif

  dev->freecount[physicalsector / dev->sectorsPerBlk]--;
#ifdef CONFIG_MTD_SMART_DEBUG
  ferr("freecount --: logical %d physical %d freecount = %d\n", logsector, physicalsector, dev->freecount[physicalsector / dev->sectorsPerBlk]);
#endif

  return OK;
}
#endif /* CONFIG_FS_WRITABLE */

/****************************************************************************
 * Name: smart_freesector
 *
 * Description:  Frees a logical sector from the device.  Freeing (also
 *               called releasing) is performed by programming the released
 *               bit in the sector header's status byte.
 *
 ****************************************************************************/

#ifdef CONFIG_FS_WRITABLE
static inline int smart_freesector(FAR struct smart_struct_s *dev,
                    uint16_t logicalsector)
{
  int       ret;
  int       readaddr;
  uint16_t  physsector;
  uint16_t  block;
  struct    smart_sect_header_s  header;
  size_t    offset;

#ifdef CONFIG_MTD_SMART_LOGICAL_SECTOR
  /* Check if the logical sector is within bounds */

  if ((logicalsector > 2) && (logicalsector < dev->totalsectors))
    {
      /* Validate the sector is actually allocated */

      if (dev->sMap[logicalsector] == (uint16_t) SMART_SMAP_INVALID)
        {
          ferr("ERROR: Invalid release - sector %d not allocated\n", logicalsector);
          ret = -EINVAL;
          goto errout;
        }
    }

  /* Okay to release the sector.  Read the sector header info */

  physsector = dev->sMap[logicalsector];
#else
  physsector = logicalsector;
#endif

  readaddr = physsector * dev->mtdBlksPerSector * dev->geo.blocksize;
  ret = MTD_READ(dev->mtd, readaddr, sizeof(struct smart_sect_header_s),
                 (FAR uint8_t *) &header);
  if (ret != sizeof(struct smart_sect_header_s))
    {
      goto errout;
    }

#ifdef CONFIG_MTD_SMART_LOGICAL_SECTOR
  /* Do a sanity check on the logical sector number */

  if (*((FAR uint16_t *) header.logicalsector) != (uint16_t) logicalsector)
    {
      /* Hmmm... something is wrong.  This should always match!  Bug in our code? */

      ferr("ERROR: Sector %d logical sector in header doesn't match\n", logicalsector);
      ret = -EINVAL;
      goto errout;
    }
#endif

  /* Mark the sector as released */

  header.status &= ~SMART_STATUS_RELEASED;

  /* Write the status back to the device */

  offset = readaddr + offsetof(struct smart_sect_header_s, status);
  ret = smart_bytewrite(dev, offset, 1, &header.status);
  if (ret != 1)
    {
      ferr("ERROR: Error updating physical sector %d status\n", physsector);
      if (ret == -EIO) {
        dev->freecount[physsector / dev->sectorsPerBlk] = SMART_FREECOUNT_BADBLOCK;
#ifdef CONFIG_MTD_SMART_LOGICAL_SECTOR
        dev->sMap[logicalsector] = SMART_SMAP_INVALID;
#endif
      }

      goto errout;
    }

  /* Update the erase block's release count */

  dev->releasesectors++;
  block = physsector / dev->sectorsPerBlk;
#ifdef CONFIG_MTD_SMART_LOGICAL_SECTOR
  dev->releasecount[block]++;
#endif
  if (dev->freecount[block] < dev->sectorsPerBlk)
    dev->freecount[block]++;

#ifdef CONFIG_MTD_SMART_DEBUG
  ferr("freecount ++: block %d logical %d physical %d count %d\n", block, logicalsector, physsector, dev->freecount[block]);
#endif

  /* Unmap this logical sector */

#ifdef CONFIG_MTD_SMART_LOGICAL_SECTOR
  dev->sMap[logicalsector] = (uint16_t) SMART_SMAP_INVALID;
#endif

  smart_save_meta(dev);

  ret = OK;

errout:
  return ret;
}
#endif /* CONFIG_FS_WRITABLE */

/****************************************************************************
 * Name: smart_ioctl
 *
 * Description: Return device geometry
 *
 ****************************************************************************/

static int smart_ioctl(FAR struct inode *inode, int cmd, unsigned long arg)
{
  FAR struct smart_struct_s *dev ;
  int ret;
#if defined(CONFIG_FS_PROCFS) && !defined(CONFIG_FS_PROCFS_EXCLUDE_SMARTFS)
  FAR struct mtd_smart_procfs_data_s *procfs_data;
  FAR struct mtd_smart_debug_data_s *debug_data;
#endif

  finfo("Entry\n");
  DEBUGASSERT(inode && inode->i_private);

  dev = (FAR struct smart_struct_s *)inode->i_private;

  /* Process the ioctl's we care about first, pass any we don't respond
   * to directly to the underlying MTD device.
   */

  switch (cmd)
    {
    case BIOC_XIPBASE:
      /* The argument accompanying the BIOC_XIPBASE should be non-NULL.  If
       * DEBUG is enabled, we will catch it here instead of in the MTD
       * driver.
       */

#ifdef CONFIG_DEBUG_FEATURES
      if (arg == 0)
        {
          ferr("ERROR: BIOC_XIPBASE argument is NULL\n");
          return -EINVAL;
        }
#endif

      /* Just change the BIOC_XIPBASE command to the MTDIOC_XIPBASE command. */

      cmd = MTDIOC_XIPBASE;
      break;

    case BIOC_GETFORMAT:

      /* Return the format information for the device */

      ret = smart_getformat(dev, (FAR struct smart_format_s *) arg);
      goto ok_out;

    case BIOC_READSECT:

      /* Do a logical sector read and return the data */
      ret = smart_readsector(dev, (FAR struct smart_read_write_s *) arg);
      goto ok_out;

#ifdef CONFIG_FS_WRITABLE
    case BIOC_LLFORMAT:

      /* Perform a low-level format on the flash */

      ret = smart_llformat(dev, arg);
      goto ok_out;

    case BIOC_ALLOCSECT:

      /* Ensure the FS is not trying to allocate a reserved sector */

      if (arg < 3)
        {
          arg = (unsigned long) -1;
        }

      /* Allocate a logical sector for the upper layer file system */

      ret = smart_allocsector(dev, (uint16_t)arg);
      goto ok_out;

    case BIOC_FREESECT:

      /* Free the specified logical sector */

      ret = smart_freesector(dev, (uint16_t)arg);
      goto ok_out;

    case BIOC_WRITESECT:

      /* Write to the sector */

      ret = smart_writesector(dev, (FAR struct smart_read_write_s *)arg);

      goto ok_out;
#endif /* CONFIG_FS_WRITABLE */

#if defined(CONFIG_FS_PROCFS) && !defined(CONFIG_FS_PROCFS_EXCLUDE_SMARTFS)
    case BIOC_GETPROCFSD:

      /* Get ProcFS data */

      procfs_data = (FAR struct mtd_smart_procfs_data_s *) arg;
      procfs_data->totalsectors = dev->totalsectors;
      procfs_data->sectorsize = dev->sectorsize;
      procfs_data->freesectors = smart_get_freesectors(dev);
      procfs_data->releasesectors = dev->releasesectors;
      procfs_data->namelen = dev->namesize;
      procfs_data->formatversion = dev->formatversion;
      procfs_data->unusedsectors = dev->unusedsectors;
      procfs_data->blockerases = dev->blockerases;
      procfs_data->sectorsperblk = dev->sectorsPerBlk;

#ifdef CONFIG_MTD_SMART_LOGICAL_SECTOR
      procfs_data->formatsector = dev->sMap[SMART_SIGNATURE_SECTOR];
      procfs_data->dirsector = dev->sMap[SMARTFS_ROOT_DIR_SECTOR];
#else
      procfs_data->formatsector = SMART_SIGNATURE_SECTOR;
      procfs_data->dirsector = SMARTFS_ROOT_DIR_SECTOR;
#endif

#ifdef CONFIG_MTD_SMART_SECTOR_ERASE_DEBUG
      procfs_data->neraseblocks = dev->geo.neraseblocks;
      procfs_data->erasecounts = dev->erasecounts;
#endif
#ifdef CONFIG_MTD_SMART_ALLOC_DEBUG
      procfs_data->allocs = dev->alloc;
      procfs_data->alloccount = SMART_MAX_ALLOCS;
#endif
      ret = OK;
      goto ok_out;
#endif

    case BIOC_DEBUGCMD:
#if defined(CONFIG_FS_PROCFS) && !defined(CONFIG_FS_PROCFS_EXCLUDE_SMARTFS)
      debug_data = (FAR struct mtd_smart_debug_data_s *) arg;
      switch (debug_data->debugcmd)
        {
        case SMART_DEBUG_CMD_SET_DEBUG_LEVEL:
          dev->debuglevel = debug_data->debugdata;
          finfo("Debug level set to %d\n", dev->debuglevel);

          ret = OK;
          goto ok_out;
        }
#endif

      break;

    case BIOC_FLUSH:
      smart_save_meta(dev);
      ret = MTD_IOCTL(dev->mtd, MTDIOC_FLUSH, 0);
      if (ret < 0)
      {
         return ret;
      }

      ret = OK;
      goto ok_out;

    case BIOC_ALLOCSECT2:
      ret = smart_allocsector2(dev, (uint16_t)arg);
      goto ok_out;
    }

  /* No other block driver ioctl commands are not recognized by this
   * driver.  Other possible MTD driver ioctl commands are passed through
   * to the MTD driver (unchanged).
   */

  ret = MTD_IOCTL(dev->mtd, cmd, arg);
  if (ret < 0)
    {
      ferr("ERROR: MTD ioctl(%04x) failed: %d\n", cmd, ret);
    }

ok_out:
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: smart_initialize
 *
 * Description:
 *   Initialize to provide a block driver wrapper around an MTD interface
 *
 * Input Parameters:
 *   minor - The minor device number.  The MTD block device will be
 *      registered as as /dev/smartN where N is the minor number.
 *   mtd - The MTD device that supports the FLASH interface.
 *
 ****************************************************************************/

int smart_initialize(int minor, FAR struct mtd_dev_s *mtd, FAR const char *partname)
{
  FAR struct smart_struct_s *dev;
  int ret = -ENOMEM;
  uint32_t  totalsectors;

  /* Sanity check */

#ifdef CONFIG_DEBUG_FEATURES
  if (minor < 0 || minor > 255 || !mtd)
    {
      return -EINVAL;
    }
#endif

  /* Allocate a SMART device structure */

  dev = (FAR struct smart_struct_s *)smart_zalloc(NULL, sizeof(struct smart_struct_s),
          "Dev struct");
  if (dev)
    {
      /* Initialize the SMART device structure */

      dev->mtd = mtd;

      /* Get the device geometry. (casting to uintptr_t first eliminates
       * complaints on some architectures where the sizeof long is different
       * from the size of a pointer).
       */

      /* Set these to zero in case the device doesn't support them */

      ret = MTD_IOCTL(mtd, MTDIOC_GEOMETRY, (unsigned long)((uintptr_t)&dev->geo));
      if (ret < 0)
        {
          ferr("ERROR: MTD ioctl(MTDIOC_GEOMETRY) failed: %d\n", ret);
          goto errout;
        }

      /* Set the sector size to the default for now */

      dev->sectorsize = 0;
      ret = smart_setsectorsize(dev, CONFIG_MTD_SMART_SECTOR_SIZE);
      if (ret != OK)
        {
          goto errout;
        }

      /* Calculate the totalsectors on this device and validate */

      totalsectors = dev->neraseblocks * dev->sectorsPerBlk;
      if (totalsectors > 65536)
        {
          ferr("ERROR: SMART Sector size too small for device\n");
          ret = -EINVAL;
          goto errout;
        }
      else if (totalsectors == 65536)
        {
          totalsectors -= 2;
        }

      dev->totalsectors = (uint16_t) totalsectors;
#ifdef CONFIG_MTD_SMART_LOGICAL_SECTOR
      dev->lastallocblock = 0;
#else
      dev->lastallocblock = SMART_FIRST_ALLOC_SECTOR/dev->sectorsPerBlk - 1;
#endif
      dev->debuglevel = 0;

      /* Mark the device format status an unknown */

      dev->formatstatus = SMART_FMT_STAT_UNKNOWN;
      dev->namesize = CONFIG_SMARTFS_MAXNAMLEN;
      if (partname)
        {
          strncpy(dev->partname, partname, SMART_PARTNAME_SIZE);
        }
      else
        {
          dev->partname[0] = '\0';
        }

      /* Create a MTD block device name */

      char device_name[20];
      if (partname != NULL)
        {
          snprintf(device_name, 18, "/dev/smart%d%s", minor, partname);
        }
      else
        {
          snprintf(device_name, 18, "/dev/smart%d", minor);
        }

      /* Inode private data is a reference to the SMART device structure */

      ret = register_blockdriver(device_name, &g_bops, 0, dev);

      if (ret < 0)
        {
          ferr("ERROR: register_blockdriver failed: %d\n", -ret);
          goto errout;
        }
    }

#ifdef CONFIG_SMART_DEV_LOOP
  (void)register_driver("/dev/smart", &g_fops, 0666, NULL);
#endif

  return OK;

errout:
#ifdef CONFIG_MTD_SMART_LOGICAL_SECTOR
  smart_free(dev, dev->sMap);
#endif
#ifndef CONFIG_MTD_BYTE_WRITE
  smart_free(dev, dev->rwbuffer);
#endif
#ifdef CONFIG_MTD_SMART_SECTOR_ERASE_DEBUG
  smart_free(dev, dev->erasecounts);
#endif

  kmm_free(dev);
  return ret;
}

 /****************************************************************************
 * Name: smart_losetup
 *
 * Description: Dynamically setups up a SMART enabled loop device that
 *              is backed by a file.  The resulting loop device is a
 *              MTD type block device vs. a generic block device.
 *
 ****************************************************************************/

#ifdef CONFIG_SMART_DEV_LOOP
static int smart_losetup(int minor, FAR const char *filename,
                int sectsize, int erasesize, off_t offset, bool readonly)
{
  FAR struct mtd_dev_s *mtd;
  struct stat           sb;
  int                   x, ret;
  char                  devpath[20];

  /* Try to create a filemtd device using the filename provided */

  mtd = filemtd_initialize(filename, offset, sectsize, erasesize);
  if (mtd == NULL)
    {
      return -ENOENT;
    }

  /* Check if we need to dynamically assign a minor number */

  if (minor == -1)
    {
      /* Start at zero and stat /dev/smartX until no entry found.
       * Searching 0 to 256 should be sufficient.
       */

      for (x = 0; x < 256; x++)
        {
          snprintf(devpath, sizeof(devpath), "/dev/smart%d", x);
          ret = stat(devpath, &sb);
          if (ret != 0)
            {
              /* We can use this minor number */

              minor = x;
              break;
            }
        }
    }

  /* Now create a smart MTD using the filemtd backing it */

  ret = smart_initialize(minor, mtd, NULL);

  if (ret != OK)
    {
      filemtd_teardown(mtd);
    }

  return ret;
}
#endif /* CONFIG_SMART_DEV_LOOP */

/****************************************************************************
 * Name: loteardown
 *
 * Description:
 *   Undo the setup performed by losetup
 *
 ****************************************************************************/

#ifdef CONFIG_SMART_DEV_LOOP
static int smart_loteardown(FAR const char *devname)
{
  FAR struct smart_struct_s *dev;
  FAR struct inode *inode;
  int ret;

  /* Sanity check */

#ifdef CONFIG_DEBUG_FEATURES
  if (!devname)
    {
      return -EINVAL;
    }
#endif

  /* Open the block driver associated with devname so that we can get the inode
   * reference.
   */

  ret = open_blockdriver(devname, MS_RDONLY, &inode);
  if (ret < 0)
    {
      ferr("ERROR: Failed to open %s: %d\n", devname, -ret);
      return ret;
    }

  /* Inode private data is a reference to the loop device structure */

  dev = (FAR struct smart_struct_s *)inode->i_private;

  /* Validate this is a filemtd backended device */

  if (!filemtd_isfilemtd(dev->mtd))
    {
      ferr("ERROR: Device is not a SMART loop: %s\n", devname);
      return -EINVAL;
    }

  close_blockdriver(inode);

  /* Now teardown the filemtd */

  filemtd_teardown(dev->mtd);
  unregister_blockdriver(devname);

  kmm_free(dev);

  return OK;
}
#endif /* CONFIG_SMART_DEV_LOOP */

/****************************************************************************
 * Name: smart_loop_read
 ****************************************************************************/

#ifdef CONFIG_SMART_DEV_LOOP
static ssize_t smart_loop_read(FAR struct file *filep, FAR char *buffer, size_t len)
{
  return 0; /* Return EOF */
}
#endif /* CONFIG_SMART_DEV_LOOP */

/****************************************************************************
 * Name: smart_loop_write
 ****************************************************************************/

#ifdef CONFIG_SMART_DEV_LOOP
static ssize_t smart_loop_write(FAR struct file *filep, FAR const char *buffer, size_t len)
{
  return len; /* Say that everything was written */
}
#endif /* CONFIG_SMART_DEV_LOOP */

/****************************************************************************
 * Name: smart_loop_ioctl
 ****************************************************************************/

#ifdef CONFIG_SMART_DEV_LOOP
static int smart_loop_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  int ret;

  switch (cmd)
    {
    /* Command:      LOOPIOC_SETUP
     * Description:  Setup the loop device
     * Argument:     A pointer to a read-only instance of struct losetup_s.
     * Dependencies: The loop device must be enabled (CONFIG_DEV_LOOP=y)
     */

    case SMART_LOOPIOC_SETUP:
      {
         FAR struct smart_losetup_s *setup = (FAR struct smart_losetup_s *)((uintptr_t)arg);

        if (setup == NULL)
          {
            ret = -EINVAL;
          }
        else
          {
            ret = smart_losetup(setup->minor, setup->filename, setup->sectsize,
                         setup->erasesize, setup->offset, setup->readonly);
          }
      }
      break;

    /* Command:      LOOPIOC_TEARDOWN
     * Description:  Teardown a loop device previously setup vis LOOPIOC_SETUP
     * Argument:     A read-able pointer to the path of the device to be
     *               torn down
     * Dependencies: The loop device must be enabled (CONFIG_DEV_LOOP=y)
     */

    case SMART_LOOPIOC_TEARDOWN:
      {
        FAR const char *devname = (FAR const char *)((uintptr_t)arg);

        if (devname == NULL)
          {
            ret = -EINVAL;
          }
        else
          {
            ret = smart_loteardown(devname);
          }
       }
       break;

     default:
       ret = -ENOTTY;
    }

  return ret;
}
#endif /* CONFIG_SMART_DEV_LOOP */

