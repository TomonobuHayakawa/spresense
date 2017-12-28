/******************************************************************************
 *
 *  Copyright (C) 2013 Sony Corporation
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at:
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 ******************************************************************************/
#include <stdio.h>
#include <time.h>
#include <unistd.h>
#include <sys/time.h>
#include <stdarg.h>
#include <pthread.h>

#include <cutils/log.h>

#include "config.h"
#include "cxd224x_debug.h"

#ifdef LOG_TAG
#undef LOG_TAG
#define LOG_TAG "CXDDEBUG"
#endif

static const char* get_indent(uint32_t indent)
{
const char *str[]={
    "",
    "  ",
    "    ",
    "      ",
    "        ",
    "          ",
    "            ",
    "              ",
    "                ",
    "                  ",
    "                    ",
    "                      ",
    "                        ",
    "                          ",
    "                            ",
    "                              ",
};
uint32_t cnt = sizeof(str)/sizeof(str[0]);

    if(indent >= cnt-1){
        indent=cnt-1;
    }
    return str[indent];
}

void cxd224x_debug(uint32_t indent, const char *fmt,...)
{
#if 0
    struct timeval  tv;
    gettimeofday(&tv, NULL);

    va_list argp;
    char buf[1024];
    uint8_t id;
    const char *task_name="";
    int32_t space_cnt=0;
    char space[128];
    char buf_thread_info[128];

    va_start( argp, fmt );
    vsnprintf(buf, sizeof(buf), fmt, argp );
    va_end(argp);

    buf_thread_info[0]='\0';
    space[0]='\0';

    id=GKIH_get_taskid();
    task_name=(const char*)GKIH_map_taskname(id);

    if(strlen(task_name)==0){
        // do nothing
    }else{
        if(strcmp(task_name, "BAD")==0){
            space_cnt=1;
        }else if(strcmp(task_name, "NFCA_TASK")==0){
            space_cnt=2;
        }else if(strcmp(task_name, "NFCA_THREAD")==0){
            space_cnt=3;
        }else if(strcmp(task_name, "NFC_TASK")==0){
            space_cnt=4;
        }else if(strcmp(task_name, "NFC_HAL_TASK")==0){
            space_cnt=5;
        }else if(strcmp(task_name, "USERIAL_HAL_TASK")==0){
            space_cnt=6;
        }else{
            space_cnt=7;
        }
        space_cnt*=4;
        memset(space, 0x20, space_cnt);
        space[space_cnt]='\0';
    }

    snprintf(buf_thread_info, sizeof(buf_thread_info), "%p/%s", (void*)pthread_self(), task_name);
    ALOGD ("[%04ld.%03ld]%s@%s:  %s%s",  (tv.tv_sec%3600), tv.tv_usec/1000, space, buf_thread_info, get_indent(indent), buf);
#else
    char buf[1024];
    va_list argp;
    va_start( argp, fmt );
    vsnprintf(buf, sizeof(buf), fmt, argp );
    va_end(argp);

    ALOGD ("%s%s",  get_indent(indent), buf);
#endif
}

static void cxd224x_dump0(uint32_t indent, const char *str, const uint8_t *p, int32_t len, int32_t flag)
{
    int32_t offset;
    char str_buf[16+1];

    if(str==NULL || p==NULL || len<=0){
        return;
    }
    offset=0;
    while(1){
        int32_t i;
        char buf[256];
        char temp[32];
        
        buf[0] = '\0';
        if(len <= 0){
            break;
        }else{
            int32_t n;
            if(len >= 16){
                n=16;
            }else{
                n=len;
            }
            for(i=0;i<n;i++){
                uint8_t c0=p[offset+i];
                snprintf(temp, sizeof(temp), "%02X ", c0);
                strcat(buf, temp);
                if(c0<0x20||c0>0x7E){
                    c0='_';
                }
                str_buf[i]=c0;
            }
            str_buf[i]='\0';
            if(flag == 0){
                cxd224x_debug(indent, "DUMP(%s) %-48s:%s", str, buf, str_buf);
            }else{
                cxd224x_debug(indent, "%s @hex=%-48s:%s",str, buf, str_buf);
            }
            offset += n;
            len -= n;
        }
    }
}

void cxd224x_dump1(uint32_t indent, const char *str, const uint8_t *p, int32_t len)
{
    cxd224x_dump0(indent, str, p, len, 0);
}

void cxd224x_dump2(uint32_t indent, const char *str, const uint8_t *p, int32_t len)
{
    cxd224x_dump0(indent, str, p, len, 1);
}

static int extra_trace_level=-1;
void cxd224x_init_extra_trace_level()
{
    extra_trace_level=1;
}
int cxd224x_get_extra_trace_level()
{
    if( extra_trace_level < 0 ){
        cxd224x_init_extra_trace_level();
    }
    return extra_trace_level;
}
