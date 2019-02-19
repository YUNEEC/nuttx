/************************************************************************************
 * arch/arm/src/stm32f7/stm32_flash.c
 *
 *   Copyright (C) 2011 Uros Platise. All rights reserved.
 *   Author: Uros Platise <uros.platise@isotel.eu>
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

/* Provides standard flash access functions, to be used by the  flash mtd driver.
 * The interface is defined in the include/nuttx/progmem.h
 *
 * Requirements during write/erase operations on FLASH:
 *  - HSI must be ON.
 *  - Low Power Modes are not permitted during write/erase
 */

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/arch.h>

#include <stdbool.h>
#include <semaphore.h>
#include <assert.h>
#include <errno.h>

#include "chip/stm32_flash.h"
#include "stm32_rcc.h"
#include "stm32_waste.h"

#include "up_arch.h"

/* Only for the STM32F76xx family for now */

#if defined(CONFIG_STM32F7_STM32F76XX)

#if defined(CONFIG_STM32_FLASH_CONFIG_DEFAULT) && \
    (defined(CONFIG_STM32_STM32F20XX) || defined(CONFIG_STM32_STM32F4XXX))
#  warning "Default Flash Configuration Used - See Override Flash Size Designator"
#endif

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

#  define FLASH_KEY1      0x45670123
#  define FLASH_KEY2      0xCDEF89AB
#  define FLASH_OPTKEY1   0x08192A3B
#  define FLASH_OPTKEY2   0x4C5D6E7F

#  define FLASH_CR_PAGE_ERASE              FLASH_CR_SER
#  define FLASH_SR_WRITE_PROTECTION_ERROR  FLASH_SR_WRPERR

/************************************************************************************
 * Private Data
 ************************************************************************************/

static sem_t g_sem = SEM_INITIALIZER(1);

/************************************************************************************
 * Private Functions
 ************************************************************************************/

static void sem_lock(void)
{
  while (sem_wait(&g_sem) < 0)
    {
      DEBUGASSERT(errno == EINTR);
    }
}

static inline void sem_unlock(void)
{
  sem_post(&g_sem);
}

static void flash_unlock(void)
{
  while (getreg32(STM32_FLASH_SR) & FLASH_SR_BSY)
    {
      up_waste();
    }

  if (getreg32(STM32_FLASH_CR) & FLASH_CR_LOCK)
    {
      /* Unlock sequence */

      putreg32(FLASH_KEY1, STM32_FLASH_KEYR);
      putreg32(FLASH_KEY2, STM32_FLASH_KEYR);
    }
}

static void flash_lock(void)
{
  modifyreg32(STM32_FLASH_CR, 0, FLASH_CR_LOCK);
}

#if defined(CONFIG_STM32_FLASH_WORKAROUND_DATA_CACHE_CORRUPTION_ON_RWW)
static void data_cache_disable(void)
{
  modifyreg32(STM32_FLASH_ACR, FLASH_ACR_DCEN, 0);
}

static void data_cache_enable(void)
{
  /* Reset data cache */

  modifyreg32(STM32_FLASH_ACR, 0, FLASH_ACR_DCRST);

  /* Enable data cache */

  modifyreg32(STM32_FLASH_ACR, 0, FLASH_ACR_DCEN);
}
#endif /* defined(CONFIG_STM32_FLASH_WORKAROUND_DATA_CACHE_CORRUPTION_ON_RWW) */

/************************************************************************************
 * Public Functions
 ************************************************************************************/

void stm32_flash_unlock(void)
{
  sem_lock();
  flash_unlock();
  sem_unlock();
}

void stm32_flash_lock(void)
{
  sem_lock();
  flash_lock();
  sem_unlock();
}

#if 0
static void stm32_flash_dump_regs(void)
{
  ferr("ACR=%08x, KEYR=%08x, OPTKEYR=%08x, SR=%08x\n",
        getreg32(STM32_FLASH_ACR),
        getreg32(STM32_FLASH_KEYR),
        getreg32(STM32_FLASH_OPTKEYR),
        getreg32(STM32_FLASH_SR));
    ferr("CR=%08x, OPTCR=%08x, OPTCR1=%08x\n",
        getreg32(STM32_FLASH_CR),
        getreg32(STM32_FLASH_OPTCR),
        getreg32(STM32_FLASH_OPTCR1));
}
#endif

/************************************************************************************
 * Name: stm32_flash_writeprotect
 *
 * Description:
 *   Enable or disable the write protection of a flash sector.
 *
 ************************************************************************************/

int stm32_flash_writeprotect(size_t pages, bool enabled)
{
  uint32_t reg;
  uint32_t val;

  if (pages > ((1 << STM32_FLASH_NPAGES) - 1))
    {
      return -EFAULT;
    }

  /* Select the register that contains the bit to be changed */

  if (pages & ((1 << STM32_FLASH_NPAGES) - 1))
    {
      reg = STM32_FLASH_OPTCR;
    }
  else
    {
      return -EFAULT;
    }

  /* Unlock options */

  putreg32(FLASH_OPTKEY1, STM32_FLASH_OPTKEYR);
  putreg32(FLASH_OPTKEY2, STM32_FLASH_OPTKEYR);

  /* Read the option status */
  val = getreg32(reg);

  /* Set or clear the protection */

  if (enabled)
    {
      val &= ~(pages << 16);
    }
  else
    {
      val |=  (pages << 16);
    }

    /* Wait for completion */

  while(getreg32(STM32_FLASH_SR) & FLASH_SR_BSY) up_waste();

  /* Write options */

  putreg32(val, reg);

  /* Trigger programmation */

  modifyreg32(STM32_FLASH_OPTCR, 0, FLASH_OPTCR_OPTSTRT);

  /* Wait for completion */

  while(getreg32(STM32_FLASH_SR) & FLASH_SR_BSY) up_waste();

  /* Relock options */

  modifyreg32(STM32_FLASH_OPTCR, 0, FLASH_OPTCR_OPTLOCK);

  return 0;
}

size_t up_progmem_pagesize(size_t page)
{
  static const size_t page_sizes[STM32_FLASH_NPAGES] = STM32_FLASH_SIZES;

  if (page >= sizeof(page_sizes) / sizeof(*page_sizes))
    {
      return 0;
    }
  else
    {
      return page_sizes[page];
    }
}

ssize_t up_progmem_getpage(size_t addr)
{
  size_t page_end = 0;
  size_t i;

  if (addr >= STM32_FLASH_AXIM)
    {
      addr -= STM32_FLASH_AXIM;
    }

  if (addr >= STM32_FLASH_SIZE)
    {
      return -EFAULT;
    }

  for (i = 0; i < STM32_FLASH_NPAGES; ++i)
    {
      page_end += up_progmem_pagesize(i);
      if (page_end > addr)
        {
          return i;
        }
    }

  return -EFAULT;
}

size_t up_progmem_getaddress(size_t page)
{
  size_t base_address = STM32_FLASH_AXIM;
  size_t i;

  if (page >= STM32_FLASH_NPAGES)
    {
      return SIZE_MAX;
    }

  for (i = 0; i < page; ++i)
    {
      base_address += up_progmem_pagesize(i);
    }

  return base_address;
}

size_t up_progmem_npages(void)
{
  return STM32_FLASH_NPAGES;
}

bool up_progmem_isuniform(void)
{
#ifdef STM32_FLASH_PAGESIZE
  return true;
#else
  return false;
#endif
}

ssize_t up_progmem_erasepage(size_t page)
{
  if (page >= STM32_FLASH_NPAGES)
    {
      return -EFAULT;
    }

  sem_lock();

  /* Get flash ready and begin erasing single page */

  flash_unlock();

  modifyreg32(STM32_FLASH_CR, 0, FLASH_CR_PAGE_ERASE);
  modifyreg32(STM32_FLASH_CR, FLASH_CR_SNB_MASK, FLASH_CR_SNB(page));
  modifyreg32(STM32_FLASH_CR, 0, FLASH_CR_STRT);

  while (getreg32(STM32_FLASH_SR) & FLASH_SR_BSY) up_waste();

  modifyreg32(STM32_FLASH_CR, FLASH_CR_PAGE_ERASE, 0);
  sem_unlock();
  
  if (getreg32(STM32_FLASH_SR) & FLASH_SR_WRITE_PROTECTION_ERROR)
  {
      ferr("FLASH_SR_WRITE_PROTECTION_ERROR\n");
      modifyreg32(STM32_FLASH_SR, 0, FLASH_SR_WRITE_PROTECTION_ERROR);
      return -EROFS;
  }
 
  /* Verify */
  if (up_progmem_ispageerased(page) == 0)
    {
      return up_progmem_pagesize(page); /* success */
    }
  else
    {
      return -EIO; /* failure */
    }
}

ssize_t up_progmem_ispageerased(size_t page)
{
  size_t addr;
  size_t count;
  size_t bwritten = 0;

  if (page >= STM32_FLASH_NPAGES)
    {
      return -EFAULT;
    }

  /* Verify */

  for (addr = up_progmem_getaddress(page), count = up_progmem_pagesize(page);
       count; count--, addr++)
    {
      if (getreg8(addr) != 0xff)
        {
          bwritten++;
        }
    }

  return bwritten;
}

ssize_t up_progmem_write(size_t addr, const void *buf, size_t count)
{
  uint16_t *hword = (uint16_t *)buf;
  size_t written = count;

  /* STM32 requires half-word access */

  if (count & 1)
    {
      return -EINVAL;
    }

  /* Check for valid address range */

  if (addr >= STM32_FLASH_AXIM)
    {
      addr -= STM32_FLASH_AXIM;
    }

  if ((addr+count) > STM32_FLASH_SIZE)
    {
      return -EFAULT;
    }

  sem_lock();

  /* Get flash ready and begin flashing */

  flash_unlock();

#if defined(CONFIG_STM32_FLASH_WORKAROUND_DATA_CACHE_CORRUPTION_ON_RWW)
  data_cache_disable();
#endif

  modifyreg32(STM32_FLASH_CR, 0, FLASH_CR_PG);

  /* TODO: implement up_progmem_write() to support other sizes than 16-bits */
  modifyreg32(STM32_FLASH_CR, FLASH_CR_PSIZE_MASK, FLASH_CR_PSIZE_X16);

  for (addr += STM32_FLASH_AXIM; count; count -= 2, hword++, addr += 2)
    {
      /* Write half-word and wait to complete */

      putreg16(*hword, addr);

      /* Use DSB to complete write etal above. So that wait is not skipped */
      __asm__ volatile("DSB \n");

      while (getreg32(STM32_FLASH_SR) & FLASH_SR_BSY) up_waste();

      /* Verify */

      if (getreg32(STM32_FLASH_SR) & FLASH_SR_WRITE_PROTECTION_ERROR)
        {
          ferr("FLASH_SR_WRITE_PROTECTION_ERROR\n");
          modifyreg32(STM32_FLASH_CR, FLASH_CR_PG, 0);
	  modifyreg32(STM32_FLASH_SR, 0, FLASH_SR_WRITE_PROTECTION_ERROR);
          sem_unlock();
          return -EROFS;
        }

      if (getreg16(addr) != *hword)
        {
          modifyreg32(STM32_FLASH_CR, FLASH_CR_PG, 0);
          sem_unlock();
          return -EIO;
        }
    }

  modifyreg32(STM32_FLASH_CR, FLASH_CR_PG, 0);

#if defined(CONFIG_STM32_FLASH_WORKAROUND_DATA_CACHE_CORRUPTION_ON_RWW)
  data_cache_enable();
#endif

  sem_unlock();
  return written;
}

#endif /* defined(CONFIG_STM32F7_STM32F76XX) */
