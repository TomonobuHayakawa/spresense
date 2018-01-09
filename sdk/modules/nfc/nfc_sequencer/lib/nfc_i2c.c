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

#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <arch/chip/cxd224x.h>

#define I2C_DEVICE_NAME "/dev/cxd224x-i2c"     //< UART device name
static int _fd = -1;    ///< File Descriptor

#ifndef SPZ_IMPL
#define DBG_LOGF_ERROR(...) printf(__VA_ARGS__)
#define DBG_LOGF_DEBUG(...) printf(__VA_ARGS__)
#endif /* SPZ_IMPL */

int nfc_i2c_open(void)
{
    if (0 <= _fd) close(_fd);

    if ( (_fd = open(I2C_DEVICE_NAME, O_RDWR)) < 0 )
    {
        DBG_LOGF_ERROR("Error: open()\n");
        goto I2C_OPEN_END;
    }

I2C_OPEN_END:
    return E_OK;
}

void nfc_i2c_close(void)
{
    if (0 <= _fd)
    {
        close(_fd);
    }

    _fd = -1;
}

int nfc_i2c_write(const uchar *data, int len)
{
    if (_fd < 0) return E_FAIL;

    if ( write(_fd, data, len) != len )
    {
        DBG_LOGF_ERROR("Error: i2c_write()\n");;
        return E_FAIL;
    }

    return E_OK;
}


int nfc_i2c_read(uchar *buff, int len)
{
    int read_len = 0;

    if (_fd < 0) return E_FAIL;

    read_len = read(_fd, buff, len);

    return read_len;
}

int nfc_i2c_get_fd(void)
{
    return _fd;
}

void nfc_i2c_ctrl_io(I2C_IO_TYPE io_type, I2C_IO_CTRL ctrl)
{
    switch(io_type)
    {
    case I2C_IO_RST_CTL:
        if(ioctl(_fd, CXDNFC_RST_CTL, ctrl) < 0)
        {
            DBG_LOGF_ERROR("Error: ioctl(fd=%d), CXDNFC_RST_CTL=0\n", _fd);
        }
        break;
    case I2C_IO_WAKE_CTL:
        if(ioctl(_fd, CXDNFC_WAKE_CTL, ctrl) < 0)       // PON HIGH (normal power mode)
        {
            DBG_LOGF_ERROR("Error: ioctl(), CXDNFC_WAKE_CTL=0\n");
        }
        break;
    case I2C_IO_POWER_CTL:
        if(ioctl(_fd, CXDNFC_POWER_CTL, ctrl) < 0)      // power on
        {
            DBG_LOGF_ERROR("Error: ioctl(), CXDNFC_POWER_CTL=1\n");
        }
        break;
    default:
        {
            DBG_LOGF_ERROR("Error: ioctl(), parameter error (io_type=%d)\n", io_type);
        }
        break;
    }

    return;
}

