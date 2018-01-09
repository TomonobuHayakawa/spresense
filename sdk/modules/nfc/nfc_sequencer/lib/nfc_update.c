/*
 * Sony Advanced Instrument of Libraries
 *
 * This program is subject to copyright protection in accordance with the
 * applicable law. It must not, except where allowed by law, by any means or
 * in any form be reproduced, distributed or lent. Moreover, no part of the
 * program may be used, viewed, printed, disassembled or otherwise interfered
 * with in any form, except where allowed by law, without the express written
 * consent of the copyright holder.
 *
 * Copyright 2013,2014 Sony Corporation
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "nf_common.h"

#define FW_PATCH "/vendor/firmware/cxd224x_firmware.bin"
#define MAX_PATCH_FILE_SZ (64*1024)
#define PATCH_MAC_SZ 0x10

#define PATCH_TYPE "pFul"

#define PATCH_ADDR_START 0x0000
#define PATCH_ADDR_HWVER 0x0004
#define PATCH_ADDR_FWVER 0x0008
#define PATCH_ADDR_PATCH 0x000C
#define PATCH_ADDR_NUM   0x001d

static unsigned char *patch_buf = NULL;
static long patch_len = 0;
static long file_posi = 0;

/*******************************************************************************
**
** Function         getFileLen
**
** Description      This function is 
**
** Parameters:
**
** Returns
**
*******************************************************************************/
static long getFileLen(FILE* fp)
{
    long sz;
    fseek(fp, 0L, SEEK_END);
    sz = ftell(fp);
    fseek(fp, 0L, SEEK_SET);
    
    return (sz > 0) ? sz : 0;
}

/*******************************************************************************
**
** Function         update_get_patch
**
** Description      This function is 
**
** Parameters:
**
** Returns
**
*******************************************************************************/
int update_get_patch(void)
{
    int result = E_OK; /* result of no problem patch */
    FILE* fd;          /* patch file pointer */
    int act_len;       /* patch file length */

    DBG_LOG_DEBUG("update_get_patch() start\n");

    fd = fopen(FW_PATCH, "rb");
    if(fd != NULL)
    {
        patch_len = getFileLen(fd);
        DBG_LOGF_DEBUG("update_get_patch() Downloading patchfile (size: %lu)\n", patch_len);
        if(patch_len <= MAX_PATCH_FILE_SZ)
        {
            patch_buf = malloc(patch_len);
            if(patch_buf != NULL)
            {
                act_len = fread(patch_buf, 1, patch_len, fd);
                if(act_len == patch_len)
                {
                     DBG_LOG_DEBUG("update_get_patch() fread success\n");
                     if(strncmp((char *)&patch_buf[PATCH_ADDR_START], PATCH_TYPE, 4) != 0)
                     {
                         DBG_LOG_DEBUG("update_get_patch() patch type fail\n");
                         result = E_FAIL;
                     }

                     if(patch_buf[PATCH_ADDR_NUM] == 0)
                     {
                         DBG_LOG_DEBUG("update_get_patch() patch number fail\n");
                         result = E_FAIL;
                     }
                }
                else
                {
                     DBG_LOG_ERROR("update_get_patch() fread failed\n");
                     result = E_FAIL;
                }
            }
            else
            {
                 DBG_LOG_ERROR("update_get_patch() malloc fail\n");
                 result = E_FAIL;
            }
        }
        else
        {
             DBG_LOG_ERROR("update_get_patch() unable to buffer\n");
             result = E_FAIL;
        }

        fclose(fd);
    }
    else
    {
        DBG_LOG_ERROR("update_get_patch() fopen failed\n");
        result = E_FAIL;
    }

    DBG_LOG_DEBUG("update_get_patch() end\n");

    return result;
}

/*******************************************************************************
**
** Function         update_delete_patch
**
** Description      This function is 
**
** Parameters:
**
** Returns
**
*******************************************************************************/
void update_delete_patch(void)
{
    if(patch_buf != NULL)
        free(patch_buf);

    patch_buf = NULL;
    patch_len = 0;
    file_posi = 0;
    return;
}

/*******************************************************************************
**
** Function         update_chk_need_update
**
** Description      
**
** Parameters:      
**
** Returns          
**
*******************************************************************************/
int update_chk_need_update(NVM_VER *nvm_ver)
{
    if(memcmp(nvm_ver->hw_ver, &patch_buf[4], 2) != 0)
    {
        DBG_LOG_WARN("update_chk_need_update() different hw ver\n");
        return(E_FAIL);
    }

    if(memcmp(nvm_ver->fw_ver, &patch_buf[10], 2) != 0)
    {
        DBG_LOG_WARN("update_chk_need_update() different fw ver\n");
        return(E_FAIL);
    }

    if(memcmp(nvm_ver->patch_ver, &patch_buf[12], 4) == 0)
    {
        DBG_LOG_WARN("update_chk_need_update() same patch ver\n");
        return(E_FAIL);
    }

    return(E_OK);
}

/*******************************************************************************
**
** Function         update_create_nci_patch_cmd
**
** Description      
**
** Parameters:      
**
** Returns          
**
*******************************************************************************/
int update_create_nci_patch_cmd(unsigned char *pd, unsigned char *next_flg)
{
    unsigned char cp_sz; /* copy size of patch buffer */

    *next_flg = NEXT_CMD;

    if(pd[1] == 0x1c)
    {
        DBG_LOG_DEBUG("update_create_nci_patch_cmd() 0x1c\n");
        return(E_OK);
    }
    else if(pd[1] == 0x1d)
    {
        DBG_LOG_DEBUG("update_create_nci_patch_cmd() 0x1d\n");
        cp_sz = 0x40;
    }
    else if(pd[1] == 0x1e)
    {
        DBG_LOG_DEBUG("update_create_nci_patch_cmd() 0x1e\n");

        if((patch_len - file_posi) >= 0xF0)
        {
            *next_flg = REPEAT_CMD;
            cp_sz = 0xF0;
        }
        else
        {
            cp_sz = (patch_len - file_posi) - PATCH_MAC_SZ;
        }

    }
    else if(pd[1] == 0x1f)
    {
        DBG_LOG_DEBUG("update_create_nci_patch_cmd() 0x1f\n");
        cp_sz = PATCH_MAC_SZ;
    }
    else
    {
        DBG_LOG_ERROR("update_create_nci_patch_cmd() other\n");
        return(E_FAIL);
    }

    memcpy(&(pd[3]), &patch_buf[file_posi], cp_sz);
    pd[2] = cp_sz;

    file_posi += cp_sz;

    return(E_OK);
}

