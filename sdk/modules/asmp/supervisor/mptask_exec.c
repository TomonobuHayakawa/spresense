/****************************************************************************
 * modules/asmp/supervisor/mptask_exec.c
 *
 *   Copyright (C) 2016 Sony Corporation. All rights reserved.
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

#include <sdk/config.h>

#include <mm/tile.h>
#include <sys/stat.h>

#include <string.h>
#include <fcntl.h>
#include <stdio.h>
#include <debug.h>
#include <errno.h>

#include <asmp/types.h>
#include <asmp/mptask.h>

#include <arch/chip/pm.h>

#include "up_arch.h"
#include "cxd56_clock.h"
#include "cxd56_icc.h"
#include "cxd56_sysctl.h"

#include "rawelf/rawelf.h"
#include "mptask.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Symbol name of bind data */

#define WORKER_BINDDATA_SYMNAME "mpframework_reserved"

/****************************************************************************
 * Public Data
 ****************************************************************************/

struct pm_cpu_wakelock_s g_mptask_wlock =
{
  .count = 0,
  .info = PM_CPUWAKELOCK_TAG('M', 'T', 0),
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int rawelf_initsymtab(struct rawelf_loadinfo_s *loadinfo)
{
  int ret;

  ret = rawelf_findsymtab(loadinfo);
  if (ret < 0)
    {
      return ret;
    }

  /* Allocate an I/O buffer.  This buffer is used by elf_symname() to
   * accumulate the variable length symbol name.
   */

  ret = rawelf_allocbuffer(loadinfo);
  if (ret < 0)
    {
      mperr("rawelf_allocbuffer failed: %d\n", ret);
      return -ENOMEM;
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int mptask_exec(mptask_t *task)
{
  struct rawelf_loadinfo_s loadinfo;
  Elf32_Sym sym;
  uint32_t binddata;
  int cpuid;
  int ret;

  if (!task)
    {
      return -EINVAL;
    }

  if (!task_is_init(task))
    {
      return -EPERM;
    }

  cpuid = mptask_getcpuid(task);
  if (cpuid < 0)
    {
      ret = mptask_assign(task);
      if (ret < 0)
        {
          return ret;
        }
      cpuid = mptask_getcpuid(task);
    }

  /* Clear allocated CPU's address converter */

  mptask_mapclear(task);

  if (task_is_secure(task))
    {
      return mptask_exec_secure(task);
    }

  /* Allocated CPU ID is APP local ID, so I convert it to global CPU ID */

  ret = cxd56_iccinitmsg(cpuid);
  if (ret < 0)
    {
      return ret;
    }

  cxd56_iccregistersighandler(cpuid, mptask_sighandler, task);

  /* Load ELF image */

  memset(&loadinfo, 0, sizeof(struct rawelf_loadinfo_s));

  loadinfo.filfd = task->fd;
  loadinfo.filelen = task->filelen;
  ret = rawelf_init(&loadinfo);
  if (ret < 0)
    {
      mperr("Failed to initialize for load of ELF program: %d\n", ret);
      return ret;
    }

  ret = rawelf_load(&loadinfo);
  if (ret < 0)
    {
      mperr("Failed to load ELF program binary: %d\n", ret);
      rawelf_uninit(&loadinfo);
      return ret;
    }

  binddata = 0;

  if (task->nbounds)
    {
      ret = rawelf_initsymtab(&loadinfo);
      if (ret < 0)
        {
          mperr("Failed to initialize symbol table: %d\n", ret);
          rawelf_uninit(&loadinfo);
          return ret;
        }

      ret = rawelf_getsymbolbyname(&loadinfo, WORKER_BINDDATA_SYMNAME,
                                   strlen(WORKER_BINDDATA_SYMNAME),
                                   &sym);
      if (ret < 0)
        {
          mperr("Bind area not found.\n");
        }
      else
        {
          binddata = sym.st_value;
          mpinfo("Bind area at %08x\n", binddata);
        }
    }

  task->loadaddr = loadinfo.textalloc;
  task->loadsize = loadinfo.textsize + loadinfo.datasize;

  mpinfo("Load at %08x (size: %x)\n", task->loadaddr, task->loadsize);

  /* Map physical address for allocated CPU address map to based on zero */

  mptask_map(task);

  /* Opened ELF file will be closed in rawelf_uninit() */

  rawelf_uninit(&loadinfo);

  /* Set bind data for sharing MP objects with worker */

  if (binddata)
    {
      void *wenv = (void *)(task->loadaddr + binddata);
      uint32_t *term;

      /* Copy bind object */

      memcpy(wenv, task->bounds, task->nbounds * sizeof(mpbindobj_t));

      /* Terminate bind object list */

      term = (uint32_t *)((uintptr_t)wenv + task->nbounds * sizeof(mpbindobj_t));
      *term = 0;
    }

  /* Suppress hot sleep */

  up_pm_acquire_wakelock(&g_mptask_wlock);

  cxd56_cpu_reset(task->cpuid);
  cxd56_cpu_clock_enable(task->cpuid);

  /* If bindata is given, it is inside the MP framework. So I wait
   * a start signal from worker.
   * Otherwise through for any independent ELFs.
   */

  if (binddata)
    {
      mptask_semtake(&task->wait);
    }

  return OK;
}
