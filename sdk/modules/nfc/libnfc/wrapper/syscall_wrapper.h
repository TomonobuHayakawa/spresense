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
 * Copyright 2014 Sony Corporation
 */

#ifndef SYSCALL_WRAPPER_H
#define SYSCALL_WRAPPER_H

#include <kernel.h>
#include <semaphore.h>
#include "time.h"
#include <include/sched.h>
#include <include/poll.h>
#include <stdio.h> 

#ifdef __cplusplus
extern "C" {
#endif
#define times(...)  spz_times(__VA_ARGS__)
#define fopen(...)  spz_fopen(__VA_ARGS__)
#define fclose(...) spz_fclose(__VA_ARGS__)
#define fread(...)  spz_fread(__VA_ARGS__)
#define fseek(...)  spz_fseek(__VA_ARGS__)
#define ftell(...)  spz_ftell(__VA_ARGS__)
#define feof(...)   spz_feof(__VA_ARGS__)

#define ioctl(...)  spz_ioctl(__VA_ARGS__)

#define malloc(...) SYS_AllocMemory(__VA_ARGS__)
#define free(...)   SYS_FreeMemory(__VA_ARGS__)

#define fprintf(...)


extern int clock_gettime(int clk_id, struct timespec* tp);
extern clock_t clock(void);

extern long spz_times(struct tms* buf);
extern unsigned int sleep(unsigned int sec);
extern int usleep(useconds_t usec);
extern int nanosleep(const struct timespec* req, struct timespec* rem);
extern pid_t getpid(void);

extern FILE *spz_fopen(const char * filename, const char * mode);
extern size_t spz_fread( void * ptr, size_t size, size_t nmemb, FILE * stream);
extern int spz_fclose(FILE *stream);
extern int spz_fseek(FILE *stream, long offset, int whence);
extern int spz_ftell(FILE *stream);
extern int spz_feof(FILE *stream);

extern int open(const char* pathname, int flags, ...);
extern int close(int fd);
extern ssize_t read(int fd, void *buf, size_t count);
extern ssize_t write(int fd, const char *buf, size_t count);
extern ssize_t recv(int sockfd, void *buf, size_t len, int flags);
extern ssize_t send(int sockfd, const void* buf, size_t len, int flags);
extern int poll(struct pollfd *fds, nfds_t nfds, int timeout);
extern int spz_ioctl(int fd, int request, char arg);

extern int sem_init(sem_t *sem, int pshared, unsigned int value);
extern int sem_post(sem_t *sem);
extern int sem_wait(sem_t *sem);
extern int sem_destroy(sem_t *sem);

extern int kill(pid_t pid, int sig);

#ifdef __cplusplus
}
#endif

#endif /* SYSCALL_WRAPPER_H */
