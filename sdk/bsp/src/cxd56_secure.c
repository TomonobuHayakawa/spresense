/****************************************************************************
 * configs/sp_yoc/src/cxd56_secure.c
 *
 *   Copyright (C) 2017 Sony Corporation. All rights reserved.
 *   Author: Yuchi.Wen <Yuchi.Wen@sony.com>
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
 * 3. Neither the name NuttX nor Sony nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
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

#include <stdio.h>
#include <debug.h>
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <sys/ioctl.h>
#include <nuttx/board.h>
#include <nuttx/pwm.h>
#include <nuttx/arch.h>
#include <arch/chip/cxd56_secure_element.h>

#include "cxd56_gpio.h"
#include "cxd56_pinconfig.h"

#ifdef CONFIG_CXD56_SECURE_ELEMENT

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define _DEBUG
#ifdef _DEBUG
#define ISO7816_LOG(fmt, arg...)    printf(fmt, ## arg)
#else
#define ISO7816_LOG(fmt, arg...)
#endif

#define TLENGTH_MAX 20
#define HLENGTH_MAX 20
#define LC_MAX      300
#define MAX_ATR_LENGTH  40
#define BUF_SIZE 10
#define MYKEY 25
#define PD_UART_DATABIT8 3
#define PD_UART_PARITY_EVEN 2
#define ISO7816_UART_CH 0
#define ISO7816_DEFAULT_FREQUENCE 2000
#define PWM_FREQUENCE   2000000
#define PWM_DUTY        1024

#define NO_ERROR        0
#define GETATR_ERROR    (-1)
#define TRANSMIT_ERROR  (-2)
#define RESPONSE_ERROR  1
#define TIMEOUT_ERROR   2
#define PWM_ERROR       (-3)

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* ATR */
typedef struct {
  uint8_t TS;
  uint8_t T0;
  uint8_t T[TLENGTH_MAX]; /* TAi~TDi */
  uint8_t H[HLENGTH_MAX]; /* historical bytes */
  uint8_t Tlength; /* TAi~TDi len */
  uint8_t Hlength; /* historical bytes len */
} iso7816_atr_t;

/* apdu header */
typedef struct {
  uint8_t CLA;
  uint8_t INS;
  uint8_t P1;
  uint8_t P2;
} iso7816_header_t;

/* apdu body */
typedef struct {
  uint8_t LC;
  uint8_t Data[LC_MAX];
  uint8_t LE;
} iso7816_body_t;

/* apdu */
typedef struct {
  iso7816_header_t header;
  iso7816_body_t body;
} iso7816_apdu_command_t;

/* response */
typedef struct {
  uint8_t data[LC_MAX];
  uint8_t SW1;
  uint8_t SW2;
} iso7816_apdu_response_t;

/* clock control command */
typedef enum {
  CLOCK_START = 0, /* the clock of the secure element start */
  CLOCK_STOP       /* the clock of the secure element stop */
} iso7816_clock_command_t;

/* secure element pin control command */
typedef enum {
  PIN_HIGH = 0, /* set the RST pin high */
  PIN_LOW       /* set the RST pin low */
} iso7816_pin_command_t;

/* iso7816 etu config & table */
const uint32_t F_Value_Table[] = {372, 372, 558, 744, 1116, 1488, 1860, 372, 372, 512, 768, 1024, 1536, 2048};
const uint32_t D_Value_Table[] = {1, 1, 2, 4, 8, 16, 32, 1, 12, 20};

static iso7816_atr_t g_card_atr;
static uint8_t g_string_atr[MAX_ATR_LENGTH] = {0};
static uint32_t g_iso7816Etu = 372/ISO7816_DEFAULT_FREQUENCE;

extern int PD_UartInit(int ch);
extern int PD_UartUninit(int ch);
extern int PD_UartConfiguration(int ch, int baudrate, int databits, int parity, int stopbit, int flowctrl);
extern int PD_UartEnable(int ch);
extern int PD_UartDisable(int ch);
extern int PD_UartReceive(int ch, void *buf, int size, int leave);
extern int PD_UartSend(int ch, void *buf, int size, int leave);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int iso7816_send_byte(uint8_t *data)
{
  uint8_t buff = 0x00;
  PD_UartSend(ISO7816_UART_CH, data, 1, 0);
  PD_UartReceive(ISO7816_UART_CH, &buff, 1, 0);
  return 0;
}

static int iso7816_pin_control(iso7816_pin_command_t command)
{
  switch(command)
  {
    case PIN_HIGH:
      cxd56_gpio_config(PIN_SPI2_MOSI, false);
      cxd56_gpio_write(PIN_SPI2_MOSI, true);
      break;
    case PIN_LOW:
      cxd56_gpio_write(PIN_SPI2_MOSI, false);
      break;
    default:
      break;
  }
  return NO_ERROR;
}

static int iso7816_pwm_control(iso7816_clock_command_t command)
{
  struct pwm_info_s info;
  int ret = 0;
  int fd = 0;
  fd = open("/dev/pwm1", O_RDONLY);
  if (fd < 0)
    {
      ISO7816_LOG("open /dev/pwm1 failed\n");
      return PWM_ERROR;
    }
  switch(command)
  {
    case CLOCK_START:
      info.frequency = PWM_FREQUENCE;
      info.duty = PWM_DUTY;
      ret = ioctl(fd, PWMIOC_SETCHARACTERISTICS, (unsigned long)((uintptr_t)&info));
      if (ret < 0)
        {
          ISO7816_LOG("pwm ioctl(PWMIOC_SETCHARACTERISTICS) failed\n");
          return ret;
        }
      ret = ioctl(fd, PWMIOC_START, 0);
      if (ret < 0)
        {
          ISO7816_LOG("pwm ioctl(PWMIOC_START) failed\n");
          return ret;
        }
      break;

    case CLOCK_STOP:
      ret = ioctl(fd, PWMIOC_STOP, 0);
      if (ret < 0)
        {
          ISO7816_LOG("pwm ioctl(PWMIOC_STOP) failed\n");
          return ret;
        }
      break;

    default:
      break;
  }
  close(fd);
  return NO_ERROR;
}

/**
 * get atr
 *
 * 7816 Recieve ATR data
 *
 * atr : atr buff
 * return i :The length of ATR data
 **/
static int iso7816_get_atr(uint8_t *atr)
{
  uint32_t i = 0;
  uint8_t temp = 0;//TAi ~ TDi exists?
  uint8_t h_len = 0;//historical bytes len

  PD_UartReceive(ISO7816_UART_CH, &atr[i++], 1, 0);/* get TS */
  PD_UartReceive(ISO7816_UART_CH, &atr[i], 1, 0);/* get T0 */
  temp = atr[i] >> 4;
  h_len = atr[i] & 0xf;
  i++;
  while (temp)
  {
    if (temp & 0x1)
      {
        PD_UartReceive(ISO7816_UART_CH, &atr[i++], 1, 0);/* TAi */
      }
    if (temp & 0x2)
      {
        PD_UartReceive(ISO7816_UART_CH, &atr[i++], 1, 0);/* TBi */
      }
    if (temp & 0x4)
      {
        PD_UartReceive(ISO7816_UART_CH, &atr[i++], 1, 0);/* TCi */
      }
    if (temp & 0x8)
      {
        PD_UartReceive(ISO7816_UART_CH, &atr[i], 1, 0);/* TCi */
        temp = atr[i++] >> 4;
        continue;
      }
      temp = 0;
  }

  while (h_len)
  {
    h_len--;
    PD_UartReceive(ISO7816_UART_CH, &atr[i++], 1, 0);/* historical bytes */
  }
  PD_UartReceive(ISO7816_UART_CH, &atr[i++], 1, 0);/* check bytes */
  return i;
}

/**
 * set baudrate
 *
 * The value of baudrate is calculated from D and F
 * D：Bit rate adjustment factor
 * F：Clock frequency conversion factor
 * return 0 :success
 **/
static int iso7816_set_baud(int D, int F)
{
  uint32_t iso7816_baudrate = F_Value_Table[F]/D_Value_Table[D];
  uint32_t baudrate = (uint32_t)(1000 * ISO7816_DEFAULT_FREQUENCE / iso7816_baudrate);

  PD_UartUninit(ISO7816_UART_CH);
  PD_UartInit(ISO7816_UART_CH);
  PD_UartConfiguration(ISO7816_UART_CH, baudrate, PD_UART_DATABIT8, PD_UART_PARITY_EVEN, 1, 0);
  PD_UartEnable(ISO7816_UART_CH);
  return NO_ERROR;
}


/**
 * usi7816 clod reset, Recieve ATR data
 **/
static int iso7816_cold_reset(uint8_t *atr)
{
  int len = 0;
  int i = 0;

  iso7816_set_baud(1, 1); /* set baudrate and init UART0 */
  iso7816_pwm_control(CLOCK_START); /* start clk */
  up_mdelay(20); /* 40000～45000 clock cycles */
  iso7816_pin_control(PIN_HIGH); /* high RST */
  up_mdelay(1); /*400～40000 clock cycles */

  len = iso7816_get_atr(atr); /* get ATR */
  up_mdelay(g_iso7816Etu*12);
  for (i = 0; i < len; i++) {
    //ISO7816_LOG("%02x ", atr[i]);//atr
  }
  return len;
}


/**
 * decode the atr data
 *
 * card : the atr data received from the card
 **/
static uint8_t iso7816_decode_atr(uint8_t *card)
{
  uint32_t i = 0;
  uint32_t flag = 0;
  uint32_t buf = 0;
  uint32_t protocol = 0;

  g_card_atr.TS = card[0];  /* Initial character */
  g_card_atr.T0 = card[1];  /* Format character */

  g_card_atr.Hlength = g_card_atr.T0 & (uint8_t)0x0F;
  if ((g_card_atr.T0 & (uint8_t)0x80) == 0x80)
    {
      flag = 1;
    }

  for (i = 0; i < 4; i++)
    {
      g_card_atr.Tlength = g_card_atr.Tlength +
                    (((g_card_atr.T0 & (uint8_t)0xF0) >> (4 + i)) & (uint8_t)0x1);
    }

  for (i = 0; i < g_card_atr.Tlength; i++)
    {
      g_card_atr.T[i] = card[i + 2];
    }

  protocol = g_card_atr.T[g_card_atr.Tlength - 1] & (uint8_t)0x0F;

  while (flag)
    {
      if ((g_card_atr.T[g_card_atr.Tlength - 1] & (uint8_t)0x80) == 0x80)
        {
          flag = 1;
        }
      else
        {
          flag = 0;
        }

      buf = g_card_atr.Tlength;
      g_card_atr.Tlength = 0;

      for (i = 0; i < 4; i++)
        {
          g_card_atr.Tlength = g_card_atr.Tlength + (((g_card_atr.T[buf - 1] & (uint8_t)0xF0) >> (4 + i)) & (uint8_t)0x1);
        }

      for (i = 0; i < g_card_atr.Tlength; i++)
        {
          g_card_atr.T[buf + i] = card[i + 2 + buf];
        }
        g_card_atr.Tlength += (uint8_t)buf;
    }

  for (i = 0; i < g_card_atr.Hlength; i++)
    {
      g_card_atr.H[i] = card[i + 2 + g_card_atr.Tlength];
    }
  return (uint8_t)protocol;
}

static int string_to_apdu(iso7816_apdu_command_t *apdu, uint8_t *string, int length)
{
  int i = 0;

  apdu->header.CLA = string[0];
  apdu->header.INS = string[1];
  apdu->header.P1 = string[2];
  apdu->header.P2 = string[3];

  if (length > 5)
    {
      apdu->body.LC = string[4];
      for (i=0; i<apdu->body.LC; i++)
        {
          apdu->body.Data[i] = string[5+i];
        }
    }
  else
    {
      apdu->body.LE = string[4];
    }
  ISO7816_LOG("iso7816 driver, %s get apdu: ", __func__);
  for (i = 0; i < length; i++)
    {
      ISO7816_LOG("%02x ", string[i]);
    }
  ISO7816_LOG("\n");
  return NO_ERROR;
}

static int uart_transmit(iso7816_apdu_command_t *apdu, iso7816_apdu_response_t *response)
{
  int i = 0;
  memset(response->data, 0, LC_MAX);
  response->SW1 = 0;
  response->SW2 = 0;

  iso7816_send_byte(&apdu->header.CLA);
  iso7816_send_byte(&apdu->header.INS);
  iso7816_send_byte(&apdu->header.P1);
  iso7816_send_byte(&apdu->header.P2);

  if (apdu->body.LC > 0)
    {
      iso7816_send_byte(&apdu->body.LC);
    }
  else if (apdu->body.LE > 0)
    {
      iso7816_send_byte(&apdu->body.LE);
    }
  else
    {
      iso7816_send_byte(&apdu->body.LE);
    }

wait_for_response:
  if (PD_UartReceive(ISO7816_UART_CH, &response->SW1, 1, 0) != 0) /* get response,SW1,SW2 */
    {
      if ((response->SW1 & 0xFF) == 0x60)
        {
          goto wait_for_response; /* get 0x60,wait */
        }
      else if ((response->SW1 & 0xF0) == 0x90)
        {
          PD_UartReceive(ISO7816_UART_CH, &response->SW2, 1, 0);
          return NO_ERROR;
        }
      else if (((response->SW1 & 0xFE) == apdu->header.INS) || ((response->SW1 & 0xFE) == (uint8_t)~(apdu->header.INS)))
        {
          response->data[0] = response->SW1;
          response->SW1 = 0;
        }
      else if ((response->SW1 & 0xF0) == 0x60)
        {
          PD_UartReceive(ISO7816_UART_CH, &response->SW2, 1, 0);/* get error response */
          return RESPONSE_ERROR;
        }
    }
  else
    {
      return TIMEOUT_ERROR;
    }

  if (response->SW1 == 0)
    {
      if (apdu->body.LC > 0)
        {
          for (i=0; i<apdu->body.LC; i++)
            {
              iso7816_send_byte(&apdu->body.Data[i]);
            }
        }
      else if (apdu->body.LE > 0)
        {
          for (i=0; i<apdu->body.LE; i++)
            {
              PD_UartReceive(ISO7816_UART_CH, &response->data[i], 1, 0);
            }
        }
    }

  if (PD_UartReceive(ISO7816_UART_CH, &response->SW1, 1, 0) == 0)
    {
      return TIMEOUT_ERROR;
    }

  if (PD_UartReceive(ISO7816_UART_CH, &response->SW2, 1, 0) == 0)
    {
      return TIMEOUT_ERROR;
    }
  return NO_ERROR;
}

static int response_to_string(uint8_t *string, iso7816_apdu_response_t *response, int data_length)
{
  int i = 0;
  for (i = 0; i < data_length; i++)
    {
      string[i] = response->data[i];
    }
  string[i++] = response->SW1;
  string[i++] = response->SW2;
  ISO7816_LOG("iso78176 driver, %s get response: ", __func__);
  for (i = 0; i < data_length + 2; i++)
    {
      ISO7816_LOG("%02x ", string[i]);
    }
  ISO7816_LOG("\n");
  return i;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int cxd56_iso7816transmit(uint8_t *input, int input_len, uint8_t *output, int *output_len)
{
  iso7816_apdu_command_t apdu;
  iso7816_apdu_response_t response;
  int ret = 0;

  memset(&apdu, 0, sizeof(iso7816_apdu_command_t));
  string_to_apdu(&apdu, input, input_len);
  ret = uart_transmit(&apdu, &response);
  if (ret != NO_ERROR)
    {
      return TRANSMIT_ERROR;
    }
  else if (ret == RESPONSE_ERROR)
    {
      output[0] = response.SW1;
      output[1] = response.SW2;
      *output_len = 2;
      return NO_ERROR;
    }

  *output_len = response_to_string(output, &response, apdu.body.LE);
  if (response.SW1 == 0 && response.SW2 ==0)
    {
      return TRANSMIT_ERROR;
    }
  ISO7816_LOG("func = %s\n", __func__);
  return NO_ERROR;
}

/**
 * iso7816 start
 * init uart for iso7816， Receive and decode atr
 **/
int cxd56_iso7816start(void)
{
  int len = 0;

  memset(g_string_atr, 0, MAX_ATR_LENGTH);
  memset(&g_card_atr, 0, sizeof(iso7816_atr_t));
  len = iso7816_cold_reset(g_string_atr);
  if (len <= 0x0 || (*g_string_atr != 0x3b))
    {
      return GETATR_ERROR;
    }

  iso7816_decode_atr(g_string_atr);
  up_mdelay(20);
  ISO7816_LOG("func = %s\n", __func__);
  return NO_ERROR;
}

int cxd56_iso7816stop(void)
{
  iso7816_pwm_control(CLOCK_STOP); /* stop clk */
  iso7816_pin_control(PIN_LOW); /* low RST */
  PD_UartDisable(ISO7816_UART_CH); /* disable UART0 */
  PD_UartUninit(ISO7816_UART_CH);
  ISO7816_LOG("func = %s\n", __func__);
  return NO_ERROR;
}

#endif /* CONFIG_CXD56_SECURE_ELEMENT */
