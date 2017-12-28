/* Dummy log functions for linux build */

#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#include <pthread.h>
#include <unistd.h>
#include <sys/types.h>
#ifndef __CYGWIN32__ 
#include <sys/syscall.h>
#endif
#include <sys/time.h>
#include "log.h"
#include "cutils/log.h"

#ifdef SPZ1_IMPL
#include "spz_nfc_log.h"
#define DBG_MODULE NF
#endif /* SPZ1_IMPL */

extern unsigned char appl_trace_level;

int isApplTrace = 1; /* 1:true 0:false */
static pthread_mutex_t         mutex = PTHREAD_MUTEX_INITIALIZER;
static int print_log_level = 5;
int g_log_time_info=1;


static char prio2c( int prio )
{
    switch(prio){
    case ANDROID_LOG_VERBOSE:
        return 'V';
    case ANDROID_LOG_DEBUG:
        return 'D';
    case ANDROID_LOG_INFO:
        return 'I';
    case ANDROID_LOG_WARN:
        return 'W';
    case ANDROID_LOG_ERROR:
        return 'E';
    case ANDROID_LOG_FATAL:
        return 'F';
    default:
        ;
    }
    return 'U';
}

#ifndef SPZ1_IMPL
int __android_log_print(int prio, const char *tag,  const char *fmt, ...)
{
    size_t n;
    va_list arg_ptr;
	if( isApplTrace == 0 )
		appl_trace_level = 0;
    struct timeval tv;
    struct tm tm;
    unsigned int ms;
#if !defined(ANDROID) && !defined(__CYGWIN__) && !defined(SPZ2_IMPL)
     pid_t pid = syscall(SYS_gettid);
#else
     pid_t pid = getpid();
#endif
    switch(prio){
    case ANDROID_LOG_UNKNOWN: // 0
    case ANDROID_LOG_DEFAULT:
    case ANDROID_LOG_VERBOSE:
    case ANDROID_LOG_DEBUG:
    case ANDROID_LOG_INFO:

    case ANDROID_LOG_WARN:
    case ANDROID_LOG_ERROR:
    case ANDROID_LOG_FATAL:  // 7
        if ( print_log_level > prio )
        {
            return 0;
        }
        break;
    case ANDROID_LOG_SILENT:
    default:
        ;
    }
    pthread_mutex_lock(&mutex);
    gettimeofday(&tv, NULL);
    localtime_r(&tv.tv_sec, &tm);
    ms = (unsigned int )tv.tv_usec/1000;

    if(g_log_time_info){
        printf("%02d-%02d %02d:%02d:%02d.%03d %c/%s(%4d): ",
               tm.tm_mon+1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec, ms,
               prio2c(prio), tag, pid);
    }else{
        printf("                              ");
    }

    va_start (arg_ptr, fmt); /* format string */
    vprintf (fmt, arg_ptr);
    va_end (arg_ptr);
    
    n = strlen(fmt);
    if( fmt[n-1] != '\n' )
    {
        printf("\n");
    }
    pthread_mutex_unlock(&mutex);
    return 0;       
}
#endif /* SPZ1_IMPL */

int __android_log_write(int prio, const char *tag,  const char *fmt)
{
#ifndef SPZ1_IMPL
    __android_log_print(prio, tag, fmt);
#else /* SPZ1_IMPL */
    size_t n;
    pthread_t pid = 0;

    if(prio == ANDROID_LOG_DEBUG) {
        DBG_LOGF_DEBUG("%s(%4d): %s\n", tag, pid, fmt);
    }
    else if(prio == ANDROID_LOG_INFO) {
        DBG_LOGF_INFO("%s(%4d): %s\n", tag, pid, fmt);
    }
    else if(prio == ANDROID_LOG_WARN) {
        DBG_LOGF_WARN("%s(%4d): %s\n", tag, pid, fmt);
    }
    else if(prio == ANDROID_LOG_ERROR) {
        DBG_LOGF_ERROR("%s(%4d): %s\n", tag, pid, fmt);
    }
    else if(prio == ANDROID_LOG_FATAL) {
        DBG_LOGF_FATAL("%s(%4d): %s\n", tag, pid, fmt);
    }
#endif /* SPZ1_IMPL */
    return 0;
}

int property_get(const char *key, char *value, const char *default_value)
{
    return 0;
}

int get_log_level( void )
{
    return print_log_level;
}

int set_log_level( int n )
{
    print_log_level = n;
    return 0;
}
