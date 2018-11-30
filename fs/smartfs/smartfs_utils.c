/****************************************************************************
 * fs/smartfs/smartfs_utils.c
 *
 *   Copyright (C) 2013-2014 Ken Pettit. All rights reserved.
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
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <semaphore.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/mtd/smart.h>

#include "smartfs.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

#if defined(CONFIG_SMARTFS_MULTI_ROOT_DIRS) || \
  (defined(CONFIG_FS_PROCFS) && !defined(CONFIG_FS_PROCFS_EXCLUDE_SMARTFS))
static struct smartfs_mountpt_s *g_mounthead = NULL;
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int smartfs_readsector(struct smartfs_mountpt_s *fs, uint16_t sector)
{
  struct smart_read_write_s readwrite;
  int ret;

  readwrite.logsector = sector;
  readwrite.count = fs->fs_llformat.availbytes;
  readwrite.buffer = (uint8_t *)fs->fs_rwbuffer;
  readwrite.offset = 0;
  ret = FS_IOCTL(fs, BIOC_READSECT, (unsigned long) &readwrite);

  return ret;
}

int smartfs_writesector(struct smartfs_mountpt_s *fs, uint16_t sector, uint8_t *buffer, uint16_t offset, uint16_t count)
{
  struct smart_read_write_s readwrite;

  readwrite.logsector = sector;
  readwrite.count = count;
  readwrite.buffer = buffer;
  readwrite.offset = offset;
  return FS_IOCTL(fs, BIOC_WRITESECT, (unsigned long) &readwrite);
}

int smartfs_readchain(struct smartfs_mountpt_s *fs, uint16_t sector)
{
  struct smart_read_write_s readwrite;

  readwrite.logsector = sector;
  readwrite.count = sizeof(struct smartfs_chain_header_s);
  readwrite.buffer = (uint8_t *)fs->fs_chainbuffer;
  readwrite.offset = 0;
  return FS_IOCTL(fs, BIOC_READSECT, (unsigned long) &readwrite);
}

/****************************************************************************
 * Name: smartfs_semtake
 ****************************************************************************/

void smartfs_semtake(struct smartfs_mountpt_s *fs)
{
  /* Take the semaphore (perhaps waiting) */

  while (sem_wait(fs->fs_sem) != 0)
    {
      /* The only case that an error should occur here is if
       * the wait was awakened by a signal.
       */

      ASSERT(*get_errno_ptr() == EINTR);
    }
}

/****************************************************************************
 * Name: smartfs_semgive
 ****************************************************************************/

void smartfs_semgive(struct smartfs_mountpt_s *fs)
{
   sem_post(fs->fs_sem);
}

/****************************************************************************
 * Name: smartfs_rdle16
 *
 * Description:
 *   Get a (possibly unaligned) 16-bit little endian value.
 *
 * Input Parameters:
 *   val - A pointer to the first byte of the little endian value.
 *
 * Returned Value:
 *   A uint16_t representing the whole 16-bit integer value
 *
 ****************************************************************************/

uint16_t smartfs_rdle16(FAR const void *val)
{
  return (uint16_t)((FAR const uint8_t *)val)[1] << 8 |
    (uint16_t)((FAR const uint8_t *)val)[0];
}

/****************************************************************************
 * Name: smartfs_wrle16
 *
 * Description:
 *   Put a (possibly unaligned) 16-bit little endian value.
 *
 * Input Parameters:
 *   dest - A pointer to the first byte to save the little endian value.
 *   val - The 16-bit value to be saved.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void smartfs_wrle16(FAR void *dest, uint16_t val)
{
  ((FAR uint8_t *) dest)[0] = val & 0xff; /* Little endian means LS byte first in byte stream */
  ((FAR uint8_t *) dest)[1] = val >> 8;
}

/****************************************************************************
 * Name: smartfs_rdle32
 *
 * Description:
 *   Get a (possibly unaligned) 32-bit little endian value.
 *
 * Input Parameters:
 *   val - A pointer to the first byte of the little endian value.
 *
 * Returned Value:
 *   A uint32_t representing the whole 32-bit integer value
 *
 ****************************************************************************/

uint32_t smartfs_rdle32(FAR const void *val)
{
  /* Little endian means LS halfword first in byte stream */

  return (uint32_t)smartfs_rdle16(&((FAR const uint8_t *)val)[2]) << 16 |
    (uint32_t)smartfs_rdle16(val);
}

/****************************************************************************
 * Name: smartfs_wrle32
 *
 * Description:
 *   Put a (possibly unaligned) 32-bit little endian value.
 *
 * Input Parameters:
 *   dest - A pointer to the first byte to save the little endian value.
 *   val - The 32-bit value to be saved.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void smartfs_wrle32(uint8_t *dest, uint32_t val)
{
  /* Little endian means LS halfword first in byte stream */

  smartfs_wrle16(dest, (uint16_t)(val & 0xffff));
  smartfs_wrle16(dest+2, (uint16_t)(val >> 16));
}

/****************************************************************************
 * Name: smartfs_mount
 *
 * Desciption: This function is called only when the mountpoint is first
 *   established.  It initializes the mountpoint structure and verifies
 *   that a valid SMART filesystem is provided by the block driver.
 *
 *   The caller should hold the mountpoint semaphore
 *
 ****************************************************************************/

int smartfs_mount(struct smartfs_mountpt_s *fs, bool writeable)
{
  FAR struct inode *inode;
  struct geometry geo;
  int ret = OK;
#if defined(CONFIG_SMARTFS_MULTI_ROOT_DIRS)
  struct smartfs_mountpt_s *nextfs;
#endif

  /* Assume that the mount is not successful */

  fs->fs_mounted = false;

  /* Check if there is media available */

  inode = fs->fs_blkdriver;
  if (!inode || !inode->u.i_bops || !inode->u.i_bops->geometry ||
      inode->u.i_bops->geometry(inode, &geo) != OK || !geo.geo_available)
    {
      ret = -ENODEV;
      goto errout;
    }

  /* Make sure that that the media is write-able (if write access is needed) */

  if (writeable && !geo.geo_writeenabled)
    {
      ret = -EACCES;
      goto errout;
    }

  /* Get the SMART low-level format information to validate the device has been
   * formatted and scan properly for logical to physical sector mapping.
   */

  ret = FS_IOCTL(fs, BIOC_GETFORMAT, (unsigned long) &fs->fs_llformat);
  if (ret != OK)
    {
      ferr("ERROR: Error getting device low level format: %d\n", ret);
      goto errout;
    }

  /* Validate the low-level format is valid */

  if (!(fs->fs_llformat.flags & SMART_FMT_ISFORMATTED))
    {
      ferr("ERROR: No low-level format found\n");
      ret = -ENODEV;
      goto errout;
    }

  /* Allocate a read/write buffer */

#ifdef CONFIG_SMARTFS_MULTI_ROOT_DIRS
  /* Scan linked list of mounted file systems to find another FS with
   * the same blockdriver.  We will reuse the buffers.
   */

  nextfs = g_mounthead;
  while (nextfs != NULL)
    {
      /* Test if this FS uses the same block driver */

      if (nextfs->fs_blkdriver == fs->fs_blkdriver)
        {
          /* Yep, it's the same block driver.  Reuse the buffers.
           * we can do this because we are protected by the same
           * semaphore.
           */

          fs->fs_rwbuffer = nextfs->fs_rwbuffer;
          fs->fs_workbuffer = nextfs->fs_workbuffer;
          break;
        }

      /* Advance to next FS */

      nextfs = nextfs->fs_next;
    }

  /* If we didn't find a FS above, then allocate some buffers */

  if (nextfs == NULL)
    {
      fs->fs_rwbuffer = (char *) kmm_malloc(fs->fs_llformat.availbytes);
      fs->fs_workbuffer = (char *) kmm_malloc(256);
    }

  /* Now add ourselves to the linked list of SMART mounts */

  fs->fs_next = g_mounthead;
  g_mounthead = fs;

  /* Set our root directory sector based on the directory entry
   * reported by the block driver (based on which device is
   * associated with this mount.
   */

  fs->fs_rootsector = SMARTFS_ROOT_DIR_SECTOR + fs->fs_llformat.rootdirnum;

#else  /* CONFIG_SMARTFS_MULTI_ROOT_DIRS */
#if defined(CONFIG_FS_PROCFS) && !defined(CONFIG_FS_PROCFS_EXCLUDE_SMARTFS)
  /* Now add ourselves to the linked list of SMART mounts */

  fs->fs_next = g_mounthead;
  g_mounthead = fs;
#endif

  fs->fs_rootsector = SMARTFS_ROOT_DIR_SECTOR;

#endif /* CONFIG_SMARTFS_MULTI_ROOT_DIRS */

  fs->fs_rwbuffer = (char *) kmm_malloc(fs->fs_llformat.availbytes);
  fs->fs_chainbuffer = (char *)kmm_malloc(sizeof(struct smartfs_chain_header_s));
  fs->fs_workbuffer = (char *) kmm_malloc(256);

  /* We did it! */

  fs->fs_mounted = TRUE;

  finfo("SMARTFS:\n");
  finfo("\t    Sector size:     %d\n", fs->fs_llformat.sectorsize);
  finfo("\t    Bytes/sector     %d\n", fs->fs_llformat.availbytes);
  finfo("\t    Num sectors:     %d\n", fs->fs_llformat.nsectors);
  finfo("\t    Free sectors:    %d\n", fs->fs_llformat.nfreesectors);
  finfo("\t    Max filename:    %d\n", CONFIG_SMARTFS_MAXNAMLEN);
#ifdef CONFIG_SMARTFS_MULTI_ROOT_DIRS
  finfo("\t    RootDirEntries:  %d\n", fs->fs_llformat.nrootdirentries);
#endif
  finfo("\t    RootDirSector:   %d\n", fs->fs_rootsector);

errout:
  return ret;
}

/****************************************************************************
 * Name: smartfs_unmount
 *
 * Desciption: This function is called only when the mountpoint is being
 *   unbound.  If we are serving multiple directories, then we have to
 *   remove ourselves from the mount linked list, and potentially free
 *   the shared buffers.
 *
 *   The caller should hold the mountpoint semaphore
 *
 ****************************************************************************/

int smartfs_unmount(struct smartfs_mountpt_s *fs)
{
  int           ret = OK;
  struct inode *inode;
#if defined(CONFIG_SMARTFS_MULTI_ROOT_DIRS) || \
  (defined(CONFIG_FS_PROCFS) && !defined(CONFIG_FS_PROCFS_EXCLUDE_SMARTFS))
  struct smartfs_mountpt_s *nextfs;
  struct smartfs_mountpt_s *prevfs;
  int           count = 0;
  int           found = FALSE;
#endif

#if defined(CONFIG_SMARTFS_MULTI_ROOT_DIRS) || \
  (defined(CONFIG_FS_PROCFS) && !defined(CONFIG_FS_PROCFS_EXCLUDE_SMARTFS))
  /* Start at the head of the mounts and search for our entry.  Also
   * count the number of entries that match our blkdriver.
   */

  nextfs = g_mounthead;
  prevfs = NULL;
  while (nextfs != NULL)
    {
      /* Test if this FS's blkdriver matches ours (it could be us) */

      if (nextfs->fs_blkdriver == fs->fs_blkdriver)
        count++;

      /* Test if this entry is our's */

      if (nextfs == fs)
        {
          found = TRUE;
        }

      /* Keep track of the previous entry until our's is found */

      if (!found)
        {
          /* Save this entry as the previous entry */

          prevfs = nextfs;
        }

      /* Advance to the next entry */

      nextfs = nextfs->fs_next;
    }

  /* Ensure we found our FS */

  if (!found)
    {
      /* Our entry not found!  Invalid unmount or bug somewhere */

      return -EINVAL;
    }

  /* If the count is only one, then we need to delete the shared
   * buffers because we are the last ones.
   */

  if (count == 1)
    {
      /* Close the block driver */

      if (fs->fs_blkdriver)
        {
          inode = fs->fs_blkdriver;
          if (inode)
            {
              if (inode->u.i_bops && inode->u.i_bops->close)
                {
                  (void)inode->u.i_bops->close(inode);
                }
            }
        }

      /* Free the buffers */

      kmm_free(fs->fs_rwbuffer);
      kmm_free(fs->fs_chainbuffer);
      kmm_free(fs->fs_workbuffer);

      /* Set the buffer's to invalid value to catch program bugs */

      fs->fs_rwbuffer = (char *) 0xDEADBEEF;
      fs->fs_workbuffer = (char *) 0xDEADBEEF;
    }

  /* Now removed ourselves from the linked list */

  if (fs == g_mounthead)
    {
      /* We were the first ones.  Set a new head */

      g_mounthead = fs->fs_next;
    }
  else
    {
      /* Remove from the middle of the list somewhere */

      prevfs->fs_next = fs->fs_next;
    }
#else
  if (fs->fs_blkdriver)
    {
     inode = fs->fs_blkdriver;
      if (inode)
        {
          if (inode->u.i_bops && inode->u.i_bops->close)
            {
              (void)inode->u.i_bops->close(inode);
            }
        }
    }

  /* Release the mountpoint private data */

  kmm_free(fs->fs_rwbuffer);
  kmm_free(fs->fs_workbuffer);
#endif

  return ret;
}

/****************************************************************************
 * Name: smartfs_finddirentry
 *
 * Description: Finds an entry in the filesystem as specified by relpath.
 *              If found, the direntry will be populated with information
 *              for accessing the entry.
 *
 *              If the final directory segment of relpath just before the
 *              last segment (the target file/dir) is valid, then the
 *              parentdirsector will indicate the logical sector number of
 *              the parent directory where a new entry should be created,
 *              and the filename pointer will point to the final segment
 *              (i.e. the "filename").
 *
 ****************************************************************************/

int smartfs_finddirentry(struct smartfs_mountpt_s *fs,
        struct smartfs_entry_s *direntry, const char *relpath,
        uint16_t *poffset, const char **filename)
{
  int ret = -ENOENT;
  const char *segment;
  const char *ptr;
  uint16_t    seglen;
  uint16_t    depth = 0;
  uint16_t    curr_poffset;
  uint16_t    entrysize;
  uint16_t    headersize;
  uint16_t    offset;
  struct      smartfs_entry_header_s *entry;

  /* If entry is not found, but parent is found:
       poffset = parent entry offset
       direntry->doffset = SMARTFS_INVALID_OFFSET
       ret < 0
     If entry is not found, and parent is invalid
       poffset = SMARTFS_INVALID_OFFSET
       direntry->doffset = SMARTFS_INVALID_OFFSET
       ret < 0
     If entry is found
       poffset = parent entry offset
       direntry->doffset = current entry offset
       ret = OK
   */

  entrysize = sizeof(struct smartfs_entry_header_s) + fs->fs_llformat.namesize;
  headersize = sizeof(struct smartfs_chain_header_s);

  /* Initial parent offset as root directory entry offset */

  *poffset = headersize;

  /* Test if this is a request for the root directory */

  if (*relpath == '\0')
    {
      direntry->firstsector = fs->fs_rootsector;
      direntry->flags = SMARTFS_DIRENT_TYPE_DIR | 0777;
      direntry->doffset = headersize;
      direntry->utc = 0;
      direntry->name = NULL;
      direntry->datlen = 0;

      ret = OK;
      goto errout;
    }
  else
    {
      /* Initial entry offset as invalid */

      direntry->doffset = SMARTFS_INVALID_OFFSET;
    }

  /* search from root entry at end of chain header */

  curr_poffset = headersize;

  /* Read the directory sector */

  ret = smartfs_readsector(fs, SMARTFS_ROOT_DIR_SECTOR);//fs->fs_rootsector);
  if (ret < 0)
    {
       goto errout;
    }

  /* Parse through each segment of relpath */

  segment = relpath;
  while (segment != NULL && *segment != '\0')
    {
      /* Find the end of this segment.  It will be '/' or NULL. */

      ptr = segment;
      seglen = 0;
      while (*ptr != '/' && *ptr != '\0')
        {
          seglen++;
          ptr++;
        }

      strncpy(fs->fs_workbuffer, segment, seglen);
      fs->fs_workbuffer[seglen] = '\0';

      /* Search for "." and ".." as segment names */

      if (strcmp(fs->fs_workbuffer, ".") == 0)
        {
          /* Just ignore this segment.  Advance ptr if not on NULL */

          if (*ptr == '/')
            {
              ptr++;
            }

          segment = ptr;
          continue;
        }
      else if (strcmp(fs->fs_workbuffer, "..") == 0)
        {
          /* Up one level */

          if (depth == 0)
            {
              /* We went up one level past our mount point! */

              goto errout;
            }

          /* "Pop" to the previous directory level */

          depth--;
          if (*ptr == '/')
            {
              ptr++;
            }

          segment = ptr;
          continue;
        }
      else
        {

          /* Search for second entry. The first entry is for root directory. */

          offset = headersize + entrysize;
          entry = (struct smartfs_entry_header_s *) &fs->fs_rwbuffer[offset];

          while (offset < fs->fs_llformat.availbytes)
            {
              /* Test if this entry is valid and active */

#ifdef CONFIG_SMARTFS_ALIGNED_ACCESS
              if (((smartfs_rdle16(&entry->flags) & SMARTFS_DIRENT_EMPTY) ==
                  (SMARTFS_ERASEDSTATE_16BIT & SMARTFS_DIRENT_EMPTY)) ||
                  ((smartfs_rdle16(&entry->flags) & SMARTFS_DIRENT_ACTIVE) !=
                  (SMARTFS_ERASEDSTATE_16BIT & SMARTFS_DIRENT_ACTIVE)))
#else
              if (((entry->flags & SMARTFS_DIRENT_EMPTY) ==
                  (SMARTFS_ERASEDSTATE_16BIT & SMARTFS_DIRENT_EMPTY)) ||
                  ((entry->flags & SMARTFS_DIRENT_ACTIVE) !=
                  (SMARTFS_ERASEDSTATE_16BIT & SMARTFS_DIRENT_ACTIVE)))
#endif
                {
                   /* This entry isn't valid, skip it */

                   offset += entrysize;
                   entry = (struct smartfs_entry_header_s *)&fs->fs_rwbuffer[offset];

                   continue;
                }

             /* Test if the name matches */

             if ((entry->poffset == curr_poffset) &&
                 (strncmp(entry->name, fs->fs_workbuffer, fs->fs_llformat.namesize) == 0))
               {
                 /* We found it!  If this is the last segment entry,
                  * then report the entry.  If it isn't the last
                  * entry, then validate it is a directory entry and
                  * open it and continue searching.
                  */


                 if (*ptr == '\0')
                   {
                     /* We are at the last segment.  Report the entry */

                     /* Fill in the entry */

#ifdef CONFIG_SMARTFS_ALIGNED_ACCESS
                     direntry->firstsector = smartfs_rdle16(&entry->firstsector);
                     direntry->flags = smartfs_rdle16(&entry->flags);
                     direntry->utc = smartfs_rdle32(&entry->utc);
#else
                     direntry->firstsector = entry->firstsector;
                     direntry->flags = entry->flags;
                     direntry->utc = entry->utc;
#endif
                     direntry->doffset = offset;
                     if (direntry->name == NULL)
                       {
                         direntry->name = (char *) kmm_malloc(fs->fs_llformat.namesize+1);
                       }

                     memset(direntry->name, 0, fs->fs_llformat.namesize + 1);
                     strncpy(direntry->name, entry->name, fs->fs_llformat.namesize);
#ifdef CONFIG_SMARTFS_ENTRY_DATLEN
                     direntry->datlen = entry->datlen;
#else
                     direntry->datlen = 0;

                     /* Scan the file's sectors to calculate the length and perform
                      * a rudimentary check.
                      */

#ifdef CONFIG_SMARTFS_ALIGNED_ACCESS
                     if ((smartfs_rdle16(&entry->flags) & SMARTFS_DIRENT_TYPE) ==
                         SMARTFS_DIRENT_TYPE_FILE)
                       {
                         dirsector = smartfs_rdle16(&entry->firstsector);
#else
                     if ((entry->flags & SMARTFS_DIRENT_TYPE) ==
                              SMARTFS_DIRENT_TYPE_FILE)
                       {
                         dirsector = entry->firstsector;
#endif
                         header = (struct smartfs_chain_header_s *) fs->fs_chainbuffer;

                         while (dirsector != SMARTFS_ERASEDSTATE_16BIT)
                           {
                             /* Read the next sector of the file */

                             ret = smartfs_readchain(fs, dirsector);
                             if (ret < 0)
                               {
                                 ferr("ERROR: Error in sector chain at %d!\n",
                                           dirsector);
                                 break;
                               }

                             /* Add used bytes to the total and point to next sector */

                             if (*((uint16_t *) header->used) != SMARTFS_ERASEDSTATE_16BIT)
                               {
                                 direntry->datlen += *((uint16_t *) header->used);
                               }

                            dirsector = SMARTFS_NEXTSECTOR(header);
                           }
                       }
#endif

                     *filename = segment;
                     ret = OK;
                     goto errout;
                   } /* if (*ptr == '\0') */
                   else
                     {
                       /* Validate it's a directory */

#ifdef CONFIG_SMARTFS_ALIGNED_ACCESS
                       if ((smartfs_rdle16(&entry->flags) & SMARTFS_DIRENT_TYPE) !=
                           SMARTFS_DIRENT_TYPE_DIR)
#else
                       if ((entry->flags & SMARTFS_DIRENT_TYPE) !=
                              SMARTFS_DIRENT_TYPE_DIR)
#endif
                         {
                           /* Not a directory!  Report the error */

                           ret = -ENOTDIR;
                           goto errout;
                         }

                       /* "Push" the directory and continue searching */

                       if (depth >= CONFIG_SMARTFS_DIRDEPTH - 1)
                         {
                           /* Directory depth too big */

                           ret = -ENAMETOOLONG;
                           goto errout;
                         }


                       break;
                     }
               }

             /* Not this entry.  Skip to the next one */

             offset += entrysize;
             entry = (struct smartfs_entry_header_s *)
                    &fs->fs_rwbuffer[offset];
            } /* while (offset < fs->fs_llformat.availbytes) */

          if (offset < fs->fs_llformat.availbytes)
            {
              /* If we found a dir entry, then continue searching */



              /* Save parent directory entry offset */

              *poffset = offset;

              curr_poffset = *poffset;

              /* Update the segment pointer */

              if (*ptr != '\0')
                {
                  ptr++;
                }

              segment = ptr;
              continue;
            }

          /* Entry not found!  Report the error.  Also, if this is the last
           * segment, then report the parent directory sector.
           */

          if (*ptr == '\0')
            {
              *filename = segment;
            }
          else
            {
              *poffset = SMARTFS_INVALID_OFFSET;
              *filename = NULL;
            }

          ret = -ENOENT;
          goto errout;
        }
    }

errout:
  direntry->poffset = *poffset;
  return ret;
}

/****************************************************************************
 * Name: smartfs_createentry
 *
 * Description: Creates a new entry in the specified parent directory, using
 *              the specified type and name.  If the given sectorno is
 *              0xFFFF, then a new sector is allocated for the new entry,
 *              otherwise the supplied sectorno is used.
 *
 ****************************************************************************/

int smartfs_createentry(FAR struct smartfs_mountpt_s *fs,
                        uint16_t poffset, uint16_t doffset, FAR const char *filename,
                        uint16_t type,  mode_t mode,
                        FAR struct smartfs_entry_s *direntry,
                        uint16_t sectorno, FAR struct smartfs_ofile_s *sf)
{
  int       ret;
  uint16_t  nextsector;
  uint16_t  offset;
  uint16_t  found;
  uint16_t  entrysize;
  struct    smartfs_entry_header_s *entry;
  struct    smartfs_chain_header_s *header;

  /* Start at the 1st sector in the parent directory */

  found = FALSE;
  entrysize = sizeof(struct smartfs_entry_header_s) + fs->fs_llformat.namesize;

  /* Validate the name isn't too long */

  if (strlen(filename) > fs->fs_llformat.namesize)
    {
      ret = -ENAMETOOLONG;
      goto errout;
    }

  /* Read the root directory sector and find a place to insert
   * the new entry.
   */
  ret = smartfs_readsector(fs, fs->fs_rootsector);
  if (ret < 0)
    {
      goto errout;
    }

  /* Get the next chained sector */

  header = (struct smartfs_chain_header_s *) fs->fs_rwbuffer;

  /* Search for an empty entry in this sector from second entry */

  if (doffset) {
    offset = doffset;
  } else {
    offset = sizeof(struct smartfs_chain_header_s) + sizeof(struct smartfs_entry_header_s) + fs->fs_llformat.namesize;
  }
  entry = (struct smartfs_entry_header_s *) &fs->fs_rwbuffer[offset];
  while (offset + entrysize < fs->fs_llformat.availbytes)
    {
      /* Check if this entry is available */

#ifdef CONFIG_SMARTFS_ALIGNED_ACCESS
      if ((smartfs_rdle16(&entry->flags) == SMARTFS_ERASEDSTATE_16BIT) ||
          ((smartfs_rdle16(&entry->flags) &
#else
      if ((entry->flags == SMARTFS_ERASEDSTATE_16BIT) ||
          ((entry->flags &
#endif
            (SMARTFS_DIRENT_EMPTY | SMARTFS_DIRENT_ACTIVE)) ==
            (~SMARTFS_ERASEDSTATE_16BIT &
            (SMARTFS_DIRENT_EMPTY | SMARTFS_DIRENT_ACTIVE))))
        {
          /* We found an empty entry.  Use it. */

          found = TRUE;
          break;
        }

      /* Not available.  Skip to next entry */

      offset += entrysize;
      entry = (struct smartfs_entry_header_s *) &fs->fs_rwbuffer[offset];
    }

  if (!found) {
      ret = -ENOSPC;
      ferr("Cannot find a free entry\n");
      goto errout;
  }

  /* We found an insertion point.  Create the entry at sector,offset */

#if CONFIG_SMARTFS_ERASEDSTATE == 0xFF
#ifdef CONFIG_SMARTFS_ALIGNED_ACCESS
  smartfs_wrle16(&entry->flags, (uint16_t) (SMARTFS_DIRENT_ACTIVE |
        SMARTFS_DIRENT_DELETING | SMARTFS_DIRENT_RESERVED | type | (mode &
        SMARTFS_DIRENT_MODE)));
#else
  entry->flags = (uint16_t) (SMARTFS_DIRENT_ACTIVE |
        SMARTFS_DIRENT_DELETING | SMARTFS_DIRENT_RESERVED | type | (mode &
        SMARTFS_DIRENT_MODE));
#endif
#else   /* CONFIG_SMARTFS_ERASEDSTATE == 0xFF */
#ifdef CONFIG_SMARTFS_ALIGNED_ACCESS
  smartfs_wrle16(&entry->flags, (uint16_t) (SMARTFS_DIRENT_EMPTY | type |
        (mode & SMARTFS_DIRENT_MODE)));
#else
  entry->flags = (uint16_t) (SMARTFS_DIRENT_EMPTY | type |
        (mode & SMARTFS_DIRENT_MODE));
#endif
#endif  /* CONFIG_SMARTFS_ERASEDSTATE == 0xFF */

  if (sectorno != 0xFFFF)
    {
      /* Use the provided sector number */

      nextsector = sectorno;
    }
  else if ((type & SMARTFS_DIRENT_TYPE) == SMARTFS_DIRENT_TYPE_DIR)
    {
      /* Do not allocate sector for directory entry */
      nextsector = 0xFFFF;
    }
  else {
      /* Allocate a new sector for the file / dir */

      ret = FS_IOCTL(fs, BIOC_ALLOCSECT, SMART_SMAP_INVALID);
      if (ret < 0)
        {
          goto errout;
        }

      nextsector = (uint16_t) ret;

      /* Set the newly allocated sector's type (file or dir) */

#ifdef CONFIG_SMARTFS_USE_SECTOR_BUFFER
      if (sf)
        {
          /* Using sector buffer and we have an open file context.  Just update
           * the sector buffer in the open file context.
           */

          memset(sf->buffer, CONFIG_SMARTFS_ERASEDSTATE, fs->fs_llformat.availbytes);
          header = (struct smartfs_chain_header_s *) sf->buffer;
          header->type = SMARTFS_SECTOR_TYPE_FILE;
          sf->bflags = SMARTFS_BFLAG_DIRTY | SMARTFS_BFLAG_NEWALLOC;
        }
      else
#endif
        {
          if ((type & SMARTFS_DIRENT_TYPE) == SMARTFS_DIRENT_TYPE_DIR)
            {
              header->type = SMARTFS_SECTOR_TYPE_DIR;
            }
          else
            {
              header->type = SMARTFS_SECTOR_TYPE_FILE;
            }

          ret = smartfs_writesector(fs, nextsector, (uint8_t *) &header->type,
                                    offsetof(struct smartfs_chain_header_s, type), 1);
          if (ret < 0)
            {
              ferr("ERROR: Error %d setting new sector type for sector %d\n",
                   ret, nextsector);
              goto errout;
            }
        }
    }

  /* Create the directory entry to be written in the parent's sector */

#ifdef CONFIG_SMARTFS_ALIGNED_ACCESS
  smartfs_wrle16(&entry->firstsector, nextsector);
  smartfs_wrle16(&entry->utc, time(NULL));
#else
  entry->firstsector = nextsector;
#ifdef CONFIG_SMARTFS_ENTRY_DATLEN
  entry->datlen = 0;
#endif
  entry->utc = time(NULL);
#endif
  entry->poffset = poffset;
  memset(entry->name, 0, fs->fs_llformat.namesize);
  strncpy(entry->name, filename, fs->fs_llformat.namesize);

  /* Now write the new entry to the parent directory sector */

  ret = smartfs_writesector(fs, fs->fs_rootsector, (uint8_t *) &fs->fs_rwbuffer[offset],
                            offset, entrysize);
  if (ret < 0)
    {
      goto errout;
    }

  /* Now fill in the entry */

  direntry->firstsector = nextsector;
  direntry->poffset = poffset;
  direntry->doffset = offset;
#ifdef CONFIG_SMARTFS_ALIGNED_ACCESS
  direntry->flags = smartfs_rdle16(&entry->flags);
  direntry->utc = smartfs_rdle32(&entry->utc);
#else
  direntry->flags = entry->flags;
  direntry->utc = entry->utc;
#endif
  direntry->datlen = 0;
  if (direntry->name == NULL)
    {
      direntry->name = (FAR char *) kmm_malloc(fs->fs_llformat.namesize+1);
    }

  memset(direntry->name, 0, fs->fs_llformat.namesize+1);
  strncpy(direntry->name, filename, fs->fs_llformat.namesize);

  ret = OK;

errout:
  return ret;
}

/****************************************************************************
 * Name: smartfs_deleteentry
 *
 * Description: Deletes an entry from the filesystem (file or dir) by
 *              freeing all the entry's sectors and then marking it inactive
 *              in it's parent's directory sector.  For a directory, it
 *              does not validate the directory is empty, nor does it do
 *              a recursive delete.
 *
 ****************************************************************************/

int smartfs_deleteentry(struct smartfs_mountpt_s *fs,
        struct smartfs_entry_s *entry)
{
  int                             ret;
  uint16_t                        nextsector;
  uint16_t                        sector;
  struct smartfs_entry_header_s  *direntry;
  struct smartfs_chain_header_s  *header;

  /* Okay, delete the file.  Loop through each sector and release them
   *
   * TODO:  We really should walk the list backward to avoid lost
   *        sectors in the event we lose power. However this requires
   *        allocating a buffer to build the sector list since we don't
   *        store a doubly-linked list of sectors on the device.  We
   *        could test if the sector data buffer is big enough and
   *        just use that, and only allocate a new buffer if the
   *        sector buffer isn't big enough.  Do do this, however, we
   *        need to change the code below as it is using the a few
   *        bytes of the buffer to read in header info.
   */

  nextsector = entry->firstsector;
  header = (struct smartfs_chain_header_s *) fs->fs_chainbuffer;
  while (nextsector != SMARTFS_ERASEDSTATE_16BIT)
    {
      /* Read the next sector into our buffer */

      sector = nextsector;
      ret = smartfs_readchain(fs, sector);
      if (ret < 0)
        {
          ferr("ERROR: Error reading sector %d\n", nextsector);
          break;
        }

      /* Release this sector */

      nextsector = SMARTFS_NEXTSECTOR(header);
      ret = FS_IOCTL(fs, BIOC_FREESECT, sector);
    }

  /* Remove the entry from the directory tree */

  ret = smartfs_readsector(fs, fs->fs_rootsector);
  if (ret < 0)
    {
      ferr("ERROR: Error reading directory info at sector %d\n",
           fs->fs_rootsector);
      goto errout;
    }

  /* Mark this entry as inactive */

  direntry = (struct smartfs_entry_header_s *) &fs->fs_rwbuffer[entry->doffset];
#if CONFIG_SMARTFS_ERASEDSTATE == 0xFF
#ifdef CONFIG_SMARTFS_ALIGNED_ACCESS
  smartfs_wrle16(&direntry->flags, smartfs_rdle16(&direntry->flags) & ~SMARTFS_DIRENT_ACTIVE);
#else
  direntry->flags &= ~SMARTFS_DIRENT_ACTIVE;
#endif
#else   /* CONFIG_SMARTFS_ERASEDSTATE == 0xFF */
#ifdef CONFIG_SMARTFS_ALIGNED_ACCESS
  smartfs_wrle16(&direntry->flags, smartfs_rdle16(&direntry->flags) | SMARTFS_DIRENT_ACTIVE);
#else
  direntry->flags |= SMARTFS_DIRENT_ACTIVE;
#endif
#endif  /* CONFIG_SMARTFS_ERASEDSTATE == 0xFF */

  /* Write the updated flags back to the sector */

  ret = smartfs_writesector(fs, fs->fs_rootsector, (uint8_t *) &direntry->flags,
                            entry->doffset, sizeof(uint16_t));
  if (ret < 0)
    {
      ferr("ERROR: Error marking entry inactive at sector %d\n",
           fs->fs_rootsector);
      goto errout;
    }

errout:
  return ret;
}

/****************************************************************************
 * Name: smartfs_countdirentries
 *
 * Description: Counts the number of items in the specified directory entry.
 *              This routine assumes you have validated the entry you are
 *              passing is in fact a directory sector, though it checks
 *              just in case you were stupid :-)
 *
 ****************************************************************************/

int smartfs_countdirentries(struct smartfs_mountpt_s *fs,
        struct smartfs_entry_s *entry)
{
  int                             ret;
  uint16_t                        offset;
  uint16_t                        entrysize;
  int                             count;
  struct smartfs_entry_header_s  *direntry;
  struct smartfs_chain_header_s  *header;

  /* Walk through the directory's sectors and count entries */

  count = 0;

  /* Read root sector into our buffer */

  ret = smartfs_readsector(fs, fs->fs_rootsector);
  if (ret < 0)
    {
      ferr("ERROR: Error reading sector %d\n", fs->fs_rootsector);
      goto errout;
    }

  /* Validate this is a directory type sector */

  header = (struct smartfs_chain_header_s *) fs->fs_rwbuffer;
  if (header->type != SMARTFS_SECTOR_TYPE_DIR)
    {
      ferr("ERROR: Sector %d is not a DIR sector! type = %d\n", fs->fs_rootsector, header->type);
      goto errout;
    }

  /* Loop for all entries in this sector and count them */

  offset = sizeof(struct smartfs_chain_header_s);
  entrysize = sizeof(struct smartfs_entry_header_s) + fs->fs_llformat.namesize;
  direntry = (struct smartfs_entry_header_s *) &fs->fs_rwbuffer[offset];
  while (offset + entrysize < fs->fs_llformat.availbytes)
    {
#ifdef CONFIG_SMARTFS_ALIGNED_ACCESS
      if (((smartfs_rdle16(&direntry->flags) & SMARTFS_DIRENT_EMPTY) !=
          (SMARTFS_ERASEDSTATE_16BIT & SMARTFS_DIRENT_EMPTY)) &&
          ((smartfs_rdle16(&direntry->flags) & SMARTFS_DIRENT_ACTIVE) ==
          (SMARTFS_ERASEDSTATE_16BIT & SMARTFS_DIRENT_ACTIVE)))
#else
      if (((direntry->flags & SMARTFS_DIRENT_EMPTY) !=
              (SMARTFS_ERASEDSTATE_16BIT & SMARTFS_DIRENT_EMPTY)) &&
              ((direntry->flags & SMARTFS_DIRENT_ACTIVE) ==
              (SMARTFS_ERASEDSTATE_16BIT & SMARTFS_DIRENT_ACTIVE)))
#endif
        {
          if (direntry->poffset == entry->doffset)
          {
            /* Count this entry */
            count++;
          }
        }

      offset += entrysize;
      direntry = (struct smartfs_entry_header_s *) &fs->fs_rwbuffer[offset];
    }

  ret = count;

errout:
  return ret;
}

/****************************************************************************
 * Name: smartfs_truncatefile
 *
 * Description: Truncates an existing file on the device so that it occupies
 *              zero bytes and can be completely re-written.
 *
 ****************************************************************************/

int smartfs_truncatefile(struct smartfs_mountpt_s *fs,
        struct smartfs_entry_s *entry, FAR struct smartfs_ofile_s *sf)
{
  int                             ret;
  uint16_t                        nextsector;
  uint16_t                        sector;
  struct smartfs_chain_header_s  *header;

  /* Walk through the directory's sectors and count entries */

  nextsector = entry->firstsector;
  header = (struct smartfs_chain_header_s *) fs->fs_chainbuffer;

  while (nextsector != SMARTFS_ERASEDSTATE_16BIT)
    {
      /* Read the next sector's header into our buffer */

      ret = smartfs_readchain(fs, nextsector);
      if (ret < 0)
        {
          ferr("ERROR: Error reading sector %d header\n", nextsector);
          goto errout;
        }

      /* Get the next chained sector */

      sector = SMARTFS_NEXTSECTOR(header);

      /* If this is the 1st sector of the file, then just overwrite
       * the sector data with the erased state value.  The underlying
       * SMART block driver will detect this and release the old
       * sector and create a new one with the new (blank) data.
       */

      if (nextsector == entry->firstsector)
        {
#ifdef CONFIG_SMARTFS_USE_SECTOR_BUFFER

          /* When we have a sector buffer in use, simply skip the first sector */

          nextsector = sector;
          continue;

#else

          /* Fill our buffer with erased data */

          memset(fs->fs_rwbuffer, CONFIG_SMARTFS_ERASEDSTATE, fs->fs_llformat.availbytes);
          header->type = SMARTFS_SECTOR_TYPE_FILE;

          /* Now write the new sector data */

          ret = smartfs_writesector(fs, nextsector, (uint8_t *) fs->fs_rwbuffer, 0, fs->fs_llformat.availbytes);
          if (ret < 0)
            {
              ferr("ERROR: Error blanking 1st sector (%d) of file\n", nextsector);
              goto errout;
            }

          /* Set the entry's data length to zero ... we just truncated */

          entry->datlen = 0;
#endif  /* CONFIG_SMARTFS_USE_SECTOR_BUFFER */
        }
      else
        {
          /* Not the 1st sector -- release it */

          ret = FS_IOCTL(fs, BIOC_FREESECT, (unsigned long) nextsector);
          if (ret < 0)
            {
              ferr("ERROR: Error freeing sector %d\n", nextsector);
              goto errout;
            }
        }

      /* Now move on to the next sector */

      nextsector = sector;
    }

  /* Now deal with the first sector in the event we are using a sector buffer
   * like we would be if CRC is enabled.
   */

#ifdef CONFIG_SMARTFS_USE_SECTOR_BUFFER
  if (sf)
    {
      /* Using sector buffer and we have an open file context.  Just update
       * the sector buffer in the open file context.
       */

      //smartfs_readchain(fs, entry->firstsector);

      memset(sf->buffer, CONFIG_SMARTFS_ERASEDSTATE, fs->fs_llformat.availbytes);
      header = (struct smartfs_chain_header_s *) sf->buffer;
      header->type = SMARTFS_SECTOR_TYPE_FILE;
      sf->bflags = SMARTFS_BFLAG_DIRTY;
      entry->datlen = 0;
    }
#endif

  ret = OK;

errout:
  return ret;
}

/****************************************************************************
 * Name: smartfs_get_first_mount
 *
 * Description: Returns a pointer to the first mounted smartfs volume.
 *
 ****************************************************************************/

#if defined(CONFIG_FS_PROCFS) && !defined(CONFIG_FS_PROCFS_EXCLUDE_SMARTFS)
FAR struct smartfs_mountpt_s *smartfs_get_first_mount(void)
{
  return g_mounthead;
}
#endif
