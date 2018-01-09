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

#include <string.h>
#include <stdio.h>

#include "nf_common.h"

#include <termios.h>
#include <fcntl.h>

#ifndef CXD224X_I2C

#define UART_DEVICE_NAME "/dev/ttyUSB0"     //< UART device name
static int _fd = -1;    ///< File Descriptor

int uart_open()
{
    struct termios termios;

    memset(&termios, 0, sizeof(struct termios));
    termios.c_iflag = IGNPAR;
    termios.c_cflag = B115200 | 0000060 | 0  | 0 | CLOCAL | CREAD ;
    termios.c_cc[VTIME] = 0;
    termios.c_cc[VMIN] = 0;     // not wait for read

    tcsetattr(_fd, TCSANOW, &termios);

    if ( (_fd = open(UART_DEVICE_NAME, O_RDWR)) < 0 )
    {
        printf("Error: open()\n");
        goto UART_OPEN_END;
    }
    printf("open(fd=%d)\n", _fd);

    tcflush(_fd, TCIOFLUSH);
    tcgetattr(_fd, &termios);

    termios.c_cflag &= ~(CSIZE | PARENB);
    termios.c_cflag = CLOCAL|CREAD|0000060|0|0;
    termios.c_cflag |= IGNPAR;

    termios.c_oflag = 0;
    termios.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
    termios.c_iflag &= ~(BRKINT | ICRNL | INLCR | ISTRIP | IXON | IGNBRK | PARMRK | INPCK);
    termios.c_lflag = 0;
    termios.c_iflag = 0;
    cfsetospeed(&termios, B115200);
    cfsetispeed(&termios, B115200);

    termios.c_cc[VTIME] = 0;
    termios.c_cc[VMIN] = 1;
    tcsetattr(_fd, TCSANOW, &termios);

    tcflush(_fd, TCIOFLUSH);

UART_OPEN_END:
    return E_OK;

}

void uart_close(void)
{
    if (0 <= _fd)
    {
        close(_fd);
    }

    _fd = -1;
}

int uart_write(const uchar *data, int len)
{
    if ( write(_fd, data, len) != len )
    {
        printf("Error: uart_send()\n");
        return E_FAIL;
    }

    return E_OK;
}


int uart_read(uchar *buff, int len)
{
    int read_len = 0;

    if (_fd < 0) return E_PARAM;

    read_len = read(_fd, buff, len);

    return read_len;
}

int uart_get_fd(void)
{
    return _fd;
}

#endif /* CXD224X_I2C */
