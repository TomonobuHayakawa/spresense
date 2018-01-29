/****************************************************************************
 * modules/location/manager/location_manager.c
 *
 *   Copyright (C) 2016 Sony Corporation. All rights reserved.
 *   Author: Takanori Yoshino <Takanori.x.Yoshino@sony.com>
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
#include <sdk/config.h>

#include <errno.h>
#include <debug.h>
#include <pthread.h>
#include <stdlib.h>
#include <sys/types.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <poll.h>
#include <string.h>

#include <location/location_manager.h>
#ifdef CONFIG_LTE
#include <lte/lte_if_agps.h>
#endif /* CONFIG_LTE */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define LOC_MNG_GNSS_DRIVER                     "/dev/gps"
#define LOC_MNG_GNSS_POLL_FD_NUM                (1)
#define LOC_MNG_GNSS_POLL_TIMEOUT_FOREVER       (-1)
#define LOC_MNG_GNSS_GNSS_POS_THREAD_STACK_SIZE (2048)
#define LOC_MNG_GNSS_POS_SND_THREAD_STACK_SIZE  (2048)
#define LOC_MNG_LOCK()                          pthread_mutex_lock( &g_loc_mng_node_lock )
#define LOC_MNG_UNLOCK()                        pthread_mutex_unlock( &g_loc_mng_node_lock )
#define LOC_MNG_POS_LOCK()                      pthread_mutex_lock( &g_loc_mng_pos_data_lock )
#define LOC_MNG_POS_UNLOCK()                    pthread_mutex_unlock( &g_loc_mng_pos_data_lock )

/****************************************************************************
 * Private Data
 ****************************************************************************/
typedef enum
{
  E_LOC_MNG_STS_UNINIT,
  E_LOC_MNG_STS_STOP,
  E_LOC_MNG_STS_MEASUREMENT
} E_LOC_MNG_STATUS;

struct loc_mng_node
{
  int                        ctl_id;
  struct loc_mng_start_param sta_prm;
  struct loc_mng_node *previous;
  struct loc_mng_node *next;
};

static pthread_mutex_t                   g_loc_mng_node_lock;
static pthread_mutex_t                   g_loc_mng_pos_data_lock;
static struct cxd56_gnss_positiondata_s *g_gnss_pos_data         = NULL;
static int                               g_fd                    = 0;
static pthread_t                         g_gnss_rcv_thread_id    = 0;
static pthread_t                         g_pos_snd_thread_id     = 0;
static struct loc_mng_pos_data           g_pos_gata              = {0};
static E_LOC_MNG_STATUS                  g_sts        = E_LOC_MNG_STS_UNINIT;
static struct loc_mng_node              *g_head                  = NULL;
static int                               g_ctl_id                = 1;

#ifdef CONFIG_LTE
static LTECommand_Agps            *g_agps_cmd_buf      = NULL;
static LTECommand_Agps            *g_agps_cmd_req_buf  = NULL;
int                               g_id                 = 0;
#endif /* CONFIG_LTE */
/****************************************************************************
 * Public Functions
 ****************************************************************************/
#ifdef CONFIG_LTE
void LocMngGnssCtl(LTECommand_Agps *agps_cmd)
{
  int                       ret           = 0;
  LTECommand_AgpsGnssCtlReq *gnss_ctl_req = &agps_cmd->message.gnss_ctl_req;
  LTECommand_Agps           *snd_cmd      = NULL;
  LTECommandHeader          *header       = NULL;
  LTECommand_AgpsGnssCtlCnf *gnss_ctl_cnf = NULL;

  snd_cmd = (LTECommand_Agps *)malloc(sizeof(LTECommand_Agps));
  if (NULL == snd_cmd)
  {
    _err("L%d snd_cmd == NULL\n", __LINE__);
    ret = -EIO;
    return;
  }

  memset(snd_cmd, 0, sizeof(LTECommand_Agps));
  header       = &snd_cmd->header;
  gnss_ctl_cnf = &snd_cmd->message.gnss_ctl_cnf;

  switch (gnss_ctl_req->ctl_id)
    {
      case LTE_AGPS_GNSS_CTL_SELECT_SATELLITE_SYSTEM:
        ret = ioctl(g_fd, gnss_ctl_req->ctl_id, (unsigned long)gnss_ctl_req->param.u.use_sat);
        break;
      case LTE_AGPS_GNSS_CTL_START:
        ret = ioctl(g_fd, gnss_ctl_req->ctl_id, CXD56_GNSS_STMOD_GSPQ);
        break;
      case LTE_AGPS_GNSS_CTL_STOP:
        ret = ioctl(g_fd, gnss_ctl_req->ctl_id, 0);
        break;
      case LTE_AGPS_GNSS_CTL_SET_TIME_GPS:
        ret = ioctl(g_fd, gnss_ctl_req->ctl_id, (unsigned long)&gnss_ctl_req->param.u.agps_time_gps);
        break;
      case LTE_AGPS_GNSS_CTL_SET_RECEIVER_POSITION:
        ret = ioctl(g_fd, gnss_ctl_req->ctl_id, (unsigned long)&gnss_ctl_req->param.u.ellipsoidal_position);
        break;
      case LTE_AGPS_GNSS_CTL_SET_EPHEMERIS:
        ret = ioctl(g_fd, gnss_ctl_req->ctl_id, (unsigned long)&gnss_ctl_req->param.u.orbital_param);
        break;
      case LTE_AGPS_GNSS_CTL_SET_ACQUIST:
        ret = ioctl(g_fd, gnss_ctl_req->ctl_id, (unsigned long)&gnss_ctl_req->param.u.agps_acquist);
        break;
      case LTE_AGPS_GNSS_CTL_SAVE_BACKUP_DATA:
        ret = ioctl(g_fd, gnss_ctl_req->ctl_id, 0);
        break;
      case LTE_AGPS_GNSS_CTL_SET_FRAMETIME:
        ret = ioctl(g_fd, gnss_ctl_req->ctl_id, (unsigned long)&gnss_ctl_req->param.u.agps_frametime);
        break;
      case LTE_AGPS_GNSS_CTL_SET_TAU_GPS:
        ret = ioctl(g_fd, gnss_ctl_req->ctl_id, (unsigned long)&gnss_ctl_req->param.u.tau_gps);
        break;
      case LTE_AGPS_GNSS_CTL_CLEAR_RECEIVER_INFO:
        ret = ioctl(g_fd, gnss_ctl_req->ctl_id, (unsigned long)gnss_ctl_req->param.u.clear_inf);
        break;
      case LTE_AGPS_GNSS_CTL_GET_TCXO_OFFSET:
        ret = ioctl(g_fd, gnss_ctl_req->ctl_id, (unsigned long)&gnss_ctl_cnf->param.u.tcxo_offset);
        break;
      case LTE_AGPS_GNSS_CTL_SET_TOW_ASSIST:
        ret = ioctl(g_fd, gnss_ctl_req->ctl_id, (unsigned long)&gnss_ctl_req->param.u.tow_assist);
        break;
      case LTE_AGPS_GNSS_CTL_SET_UTC_MODEL:
        ret = ioctl(g_fd, gnss_ctl_req->ctl_id, (unsigned long)&gnss_ctl_req->param.u.utc_model);
        break;
      default:
        _err("L%d invalid ctl_id = %d\n", __LINE__, gnss_ctl_req->ctl_id);
        goto exit;
        break;
    }

  _info("L%d Exec ctl_id = %d (ret:%d)\n", __LINE__, gnss_ctl_req->ctl_id, ret);

  header->code             = LTE_COMMAND_AGPS_GNSS_CTL_CNF;
  header->id               = agps_cmd->header.id;
  header->type             = LTE_STRUCT_TYPE_AGPS_GNSS_CTL_CNF;
  gnss_ctl_cnf->ctl_id     = gnss_ctl_req->ctl_id;
  gnss_ctl_cnf->ctl_result = ret;
  LT_SendAgps(snd_cmd);

exit:
  free(snd_cmd);
}

static void LocMngSetNwPosData(LTECommand_AgpsNetPosInd *nw_pos_ind)
{
  struct tm *nw_tow_tm = NULL;

  nw_tow_tm = gmtime(&nw_pos_ind->tow);

  LOC_MNG_POS_LOCK();
  memset(&g_pos_gata, 0, sizeof(struct loc_mng_pos_data));
  g_pos_gata.date.year  = nw_tow_tm->tm_year + 1900;
  g_pos_gata.date.month = nw_tow_tm->tm_mon + 1;
  g_pos_gata.date.day   = nw_tow_tm->tm_mday;

  g_pos_gata.time.hour   = nw_tow_tm->tm_hour;
  g_pos_gata.time.minute = nw_tow_tm->tm_min;
  g_pos_gata.time.sec    = nw_tow_tm->tm_sec;

  g_pos_gata.latitude = nw_pos_ind->latitude;
  g_pos_gata.longitude = nw_pos_ind->longitude;
  if (0 != nw_pos_ind->is_altitude)
    {
      g_pos_gata.altitude = nw_pos_ind->altitude;
    }
  LOC_MNG_POS_UNLOCK();
}

void LocMngLTECallbackFunctionAgps(void *data)
{
  LTECommand_Agps    *agps_cmd  = NULL;
  LTECommandHeader   *header    = NULL;

  if (NULL == data)
    {
      _err("L%d data = NULL\n", __LINE__);
      return;
    }

  agps_cmd     = (LTECommand_Agps *)data;
  header       = &agps_cmd->header;

  _info("L%d Callback recv code = 0x%04X result = %d\n", __LINE__, header->code, header->result);

  switch (header->code)
    {
      case LTE_COMMAND_AGPS_START:
        if (0 != header->result)
          {
            ioctl(g_fd, CXD56_GNSS_IOCTL_START, CXD56_GNSS_STMOD_GSPQ);
          }
        break;
      case LTE_COMMAND_AGPS_STOP:
        if (0 != header->result)
          {
            ioctl(g_fd, CXD56_GNSS_IOCTL_STOP, 0);
          }
        break;
      case LTE_COMMAND_AGPS_GNSS_CTL_REQ:
        LocMngGnssCtl(agps_cmd);
        break;
      case LTE_COMMAND_AGPS_NET_POS_IND:
        LocMngSetNwPosData(&agps_cmd->message.nw_pos_ind);
        break;
      case LTE_COMMAND_AGPS_QUALITY:
      case LTE_COMMAND_AGPS_SATELLITE:
      case LTE_COMMAND_AGPS_GNSS_CTL_CNF:
      case LTE_COMMAND_AGPS_GNSS_MEAS_IND:
      default:
          _err("L%d invalid code = 0x%04X\n", __LINE__, (header->code & 0x0000FFFF));
        break;
    }
}
#endif /* CONFIG_LTE */

static int LocMngGnssRead(int fd)
{
  int                        ret                           = 0;
#ifdef CONFIG_LTE
  LTECommand_Agps            *meas_ind_buf  = NULL;
  LTECommandHeader           *header        = NULL;
  LTECommand_AgpsGnssMeasInd *gnss_meas_ind = NULL;
  static int                 s_id           = 0;
#endif /* CONFIG_LTE */

/* position data */

  memset(g_gnss_pos_data, 0, sizeof(struct cxd56_gnss_positiondata_s));
  ret = read(fd, g_gnss_pos_data, sizeof(struct cxd56_gnss_positiondata_s));
  if (ret < 0)
    {
      _err("L%d read error\n", __LINE__);
      goto exit;
    }

  LOC_MNG_POS_LOCK();

  if ((0 == g_gnss_pos_data->status)
      && (CXD56_GNSS_PVT_POSFIX_3D == g_gnss_pos_data->receiver.pos_fixmode))
    {
        memcpy(&g_pos_gata.date, &g_gnss_pos_data->receiver.date, sizeof(struct cxd56_gnss_date_s));
        memcpy(&g_pos_gata.time, &g_gnss_pos_data->receiver.time, sizeof(struct cxd56_gnss_time_s));
        g_pos_gata.latitude = g_gnss_pos_data->receiver.latitude;
        g_pos_gata.longitude = g_gnss_pos_data->receiver.longitude;
        g_pos_gata.altitude = g_gnss_pos_data->receiver.altitude;
    }

  /* Unlock */

  LOC_MNG_POS_UNLOCK();

#ifdef CONFIG_LTE

  /* measurement data */

  meas_ind_buf = (LTECommand_Agps *)malloc(sizeof(LTECommand_Agps));
  if (NULL == meas_ind_buf)
    {
      _err("L%d meas_ind_buf == NULL\n", __LINE__);
      ret = -EIO;
      goto exit;
    }

  memset(meas_ind_buf, 0, sizeof(LTECommand_Agps));

  header        = &meas_ind_buf->header;
  gnss_meas_ind = &meas_ind_buf->message.gnss_meas_ind;

  header->code   = LTE_COMMAND_AGPS_GNSS_MEAS_IND;
  header->id     = s_id++;
  header->result = 0;
  header->type   = LTE_STRUCT_TYPE_AGPS_GNSS_MEAS_IND;

  memcpy(&gnss_meas_ind->gnss_pos_data, g_gnss_pos_data, sizeof(struct cxd56_gnss_positiondata_s));

  ret = lseek(fd, CXD56_GNSS_READ_OFFSET_AGPS, SEEK_SET);
  if (0 > ret)
    {
      _err("L%d file_seek error\n", __LINE__);
      goto exit;
    }

  ret = read(fd, &gnss_meas_ind->supl_meas_data, sizeof(struct cxd56_supl_mesurementdata_s));
  if (0 > ret)
    {
      _err("L%d file_seek error\n", __LINE__);
      goto exit;
    }

  LT_SendAgps(meas_ind_buf);

  free(meas_ind_buf);
#endif /* CONFIG_LTE */

  ret = 0;

exit:

  return ret;
}

static void LocMngGnssRecvRun(void)
{
  int                        ret                           = 0;
  struct pollfd              fds[LOC_MNG_GNSS_POLL_FD_NUM] = {{0}};

  fds[0].fd     = g_fd;
  fds[0].events = POLLIN;

  while (1)
    {
      ret = poll(fds, LOC_MNG_GNSS_POLL_FD_NUM, LOC_MNG_GNSS_POLL_TIMEOUT_FOREVER);
      if (0 >= ret)
        {
          _err("L%d poll error %d,%x,%x\n", __LINE__, ret, fds[0].events, fds[0].revents);
          break;;
        }

      ret = LocMngGnssRead(g_fd);
      if (0 != ret)
        {
          _err("L%d LocMngGnssRead() %s\n", __LINE__, strerror( ret ));
          break;;
        }
    }
}

static void *LocMngGnssRecvThread(void *arg)
{
  _info("L%d start\n", __LINE__);

  LocMngGnssRecvRun();

  pthread_exit( NULL );
  _info("L%d end\n", __LINE__);
  return (void *)NULL;
}

/* Initialize */
int LocMngInit(void)
{
  int            ret                  = 0;
  pthread_attr_t gnss_pos_thread_attr = {0};


  if (E_LOC_MNG_STS_UNINIT != g_sts)
    {
      _err("L%d status error == %d\n", __LINE__, g_sts);
      ret = -EPERM;
      goto exit;
    }

  g_gnss_pos_data = (struct cxd56_gnss_positiondata_s *)malloc(sizeof(struct cxd56_gnss_positiondata_s));
  if (NULL == g_gnss_pos_data)
    {
      _err("L%d g_gnss_pos_data == NULL\n", __LINE__);
      ret = -EIO;
      goto exit;
    }

  memset(g_gnss_pos_data, 0, sizeof(struct cxd56_gnss_positiondata_s));

  g_fd = open(LOC_MNG_GNSS_DRIVER, O_RDONLY);
  if (0 >= g_fd)
    {
      _err("L%d open error:%d\n", __LINE__, errno);
      return -ENODEV;
    }

  ret = pthread_attr_init(&gnss_pos_thread_attr);
  if (0 != ret)
    {
      _err("L%d pthread_attr_init() %s\n", __LINE__, strerror( ret ));
      ret = -ret;
      goto exit;
    }

  ret = pthread_attr_setstacksize(&gnss_pos_thread_attr, LOC_MNG_GNSS_GNSS_POS_THREAD_STACK_SIZE);
  if (0 != ret)
    {
      _err("L%d pthread_attr_setstacksize() %s\n", __LINE__, strerror( ret ));
      ret = -ret;
      goto exit;
    }

  ret = pthread_create(&g_gnss_rcv_thread_id, &gnss_pos_thread_attr, LocMngGnssRecvThread, NULL);
  if (0 != ret)
    {
      _err("L%d pthread_create() %s\n", __LINE__, strerror( ret ));
      ret = -ret;
      goto exit;
    }

#ifdef CONFIG_LTE
  g_agps_cmd_buf = (LTECommand_Agps *)malloc(sizeof(LTECommand_Agps));
  if (NULL == g_agps_cmd_buf)
    {
      _err("L%d g_agps_cmd_buf == NULL\n", __LINE__);
      ret = -EIO;
      goto exit;
    }

  memset(g_agps_cmd_buf, 0, sizeof(LTECommand_Agps));

  g_agps_cmd_req_buf = (LTECommand_Agps *)malloc(sizeof(LTECommand_Agps));
  if (NULL == g_agps_cmd_req_buf)
    {
      _err("L%d g_agps_cmd_req_buf == NULL\n", __LINE__);
      ret = -EIO;
      goto exit;
    }

  memset(g_agps_cmd_req_buf, 0, sizeof(LTECommand_Agps));

  LT_SetCallbackAgps(LocMngLTECallbackFunctionAgps, g_agps_cmd_buf);
#endif /* CONFIG_LTE */

  pthread_mutex_init(&g_loc_mng_pos_data_lock, 0);
  pthread_mutex_init(&g_loc_mng_node_lock, 0);
  memset(&g_pos_gata, 0, sizeof(struct loc_mng_pos_data));

  g_sts = E_LOC_MNG_STS_STOP;

exit:
  return ret;
}

static struct loc_mng_node *LocMngFindNode( int ctl_id )
{
  struct loc_mng_node *p_node = g_head;

  while(NULL != p_node)
  {
    if (p_node->ctl_id == ctl_id )
      {
        break;
      }

    p_node = p_node->next;
  }

  return p_node;
}

static void LocMngAddList( struct loc_mng_node *add_node)
{
  if(g_head)
  {
    add_node->next   = g_head;
    g_head->previous = add_node;
    g_head           = add_node;
  } else {
    g_head           = add_node;
  }
}

static struct loc_mng_node *LocMngCreateNode(struct loc_mng_start_param *sta_prm)
{
  struct loc_mng_node *p_new_node;

  p_new_node = (struct loc_mng_node *)malloc(sizeof(struct loc_mng_node));

  if(!p_new_node)
  {
    _err("L%d p_new_node == NULL\n", __LINE__);
    return p_new_node;
  }

  memset(p_new_node, 0, sizeof(struct loc_mng_node));
  if (LOC_MNG_NUM_OF_FIX_INFNITE == sta_prm->num_of_fixes)
    {
      p_new_node->sta_prm.num_of_fixes = -1;
    }
  else
    {
      p_new_node->sta_prm.num_of_fixes = sta_prm->num_of_fixes;
    }

  p_new_node->sta_prm.cycle = sta_prm->cycle;
  p_new_node->sta_prm.cbs   = sta_prm->cbs;

  p_new_node->ctl_id        = g_ctl_id++;

  return p_new_node;
}

static void LocMngDeleteNode( struct loc_mng_node *del_node )
{
  if(del_node)
    {
      if(NULL != del_node->previous)
        {
          del_node->previous->next = del_node->next;
        } 
      else
        {
          g_head = del_node->next;
        }

      if(NULL != del_node->next)
        {
          del_node->next->previous = del_node->previous;
        }

      free(del_node);
    }
}

static void LocMngPosSndRun(void)
{
  unsigned int        tim_cnt     = 1;
  struct loc_mng_node *p_node     = NULL;
  struct loc_mng_node *p_node_tmp = NULL;

  while(1)
    {
      LOC_MNG_LOCK();
      LOC_MNG_POS_LOCK();

      if (NULL == g_head)
        {
          break;
        }

      if (0 != g_pos_gata.date.year)
        {
          p_node = g_head;
          do
            {
              if(0 == (tim_cnt%p_node->sta_prm.cycle))
                {
                  g_pos_gata.ctl_id = p_node->ctl_id;
                  p_node->sta_prm.cbs(&g_pos_gata);
                  if (-1 != p_node->sta_prm.num_of_fixes)
                    {
                      p_node->sta_prm.num_of_fixes--;
                    }
                  p_node_tmp = p_node;
                  p_node = p_node->next;

                  if (0 == p_node_tmp->sta_prm.num_of_fixes)
                    {
                      LocMngDeleteNode(p_node_tmp);
                    }
                }
            } while(NULL != p_node);
        }
      LOC_MNG_POS_UNLOCK();
      LOC_MNG_UNLOCK();

      sleep(1);
      tim_cnt++;
    }
}

static void *LocMngPosSndThread(void *arg)
{
  _info("L%d start\n", __LINE__);

  LocMngPosSndRun();

  pthread_exit( NULL );

  /* Lock */

  LOC_MNG_POS_LOCK();

  g_pos_snd_thread_id = 0;

  /* Unlock */

  LOC_MNG_POS_UNLOCK();
  _info("L%d end\n", __LINE__);
  return (void *)NULL;
}

/* Start */
int LocMngStart(struct loc_mng_start_param *sta_prm, int *ctl_id)
{
  int                 ret                 = 0;
  pthread_attr_t      pos_snd_thread_attr = {0};
  struct loc_mng_node *p_new_node         = NULL;

  if (E_LOC_MNG_STS_UNINIT == g_sts)
    {
      _err("L%d status error == %d\n", __LINE__, g_sts);
      ret = -EPERM;
      goto exit;
    }

  if (NULL == sta_prm || NULL == sta_prm->cbs)
    {
      _err("L%d sta_prm == NULL\n", __LINE__);
      ret = -EIO;
      goto exit;
    }

  LOC_MNG_LOCK();

  p_new_node = LocMngCreateNode(sta_prm);
  if(NULL == p_new_node){
    ret = -ENOMEM;
     LOC_MNG_UNLOCK();
    goto exit;
  }

  LocMngAddList(p_new_node);

  *ctl_id = p_new_node->ctl_id;

  LOC_MNG_UNLOCK();

  if(E_LOC_MNG_STS_STOP == g_sts)
    {
      memset(&g_pos_gata, 0, sizeof(struct loc_mng_pos_data));

      ret = pthread_attr_init(&pos_snd_thread_attr);
      if (0 != ret)
        {
          _err("L%d pthread_attr_init() %s\n", __LINE__, strerror( ret ));
          ret = -ret;
          goto exit;
        }

      ret = pthread_attr_setstacksize(&pos_snd_thread_attr, LOC_MNG_GNSS_POS_SND_THREAD_STACK_SIZE);
      if (0 != ret)
        {
          _err("L%d pthread_attr_setstacksize() %s\n", __LINE__, strerror( ret ));
          ret = -ret;
          goto exit;
        }

      ret = pthread_create(&g_pos_snd_thread_id, &pos_snd_thread_attr, LocMngPosSndThread, NULL);
      if (0 != ret)
        {
          _err("L%d pthread_create() %s\n", __LINE__, strerror( ret ));
          ret = -ret;
          goto exit;
        }

#ifdef CONFIG_LTE
      memset(g_agps_cmd_req_buf, 0, sizeof(LTECommand_Agps));
      g_agps_cmd_req_buf->header.code   = LTE_COMMAND_AGPS_START;
      g_agps_cmd_req_buf->header.id     = g_id++;

      LT_SendAgps(g_agps_cmd_req_buf);
#else
      ret = ioctl(g_fd, CXD56_GNSS_IOCTL_START, CXD56_GNSS_STMOD_GSPQ);
      if (0 != ret)
        {
          _err("L%d ioctl() %s\n", __LINE__, strerror( ret ));
          goto exit;
        }
#endif /* CONFIG_LTE */

      g_sts = E_LOC_MNG_STS_MEASUREMENT;
   }

exit:
  return ret;
}

/* Stop */
int LocMngStop(int ctl_id)
{
  int                 ret               = 0;
  void                *res              = NULL;
  pthread_t           pos_snd_thread_id = 0;
  struct loc_mng_node *p_locmng_node    = NULL;

  LOC_MNG_LOCK();

  if (E_LOC_MNG_STS_UNINIT == g_sts)
    {
      _err("L%d status error == %d\n", __LINE__, g_sts);
      ret = -EPERM;
      goto exit;
    }

  p_locmng_node = LocMngFindNode(ctl_id);
  if (NULL != p_locmng_node)
    {
      LocMngDeleteNode(p_locmng_node);
    }
  else
    {
      _err("L%d ctl_id error == %d\n", __LINE__, ctl_id);
      ret = -EPERM;
      goto exit;
    }

  if (NULL==g_head)
    {

#ifdef CONFIG_LTE
      memset(g_agps_cmd_req_buf, 0, sizeof(LTECommand_Agps));
      g_agps_cmd_req_buf->header.code   = LTE_COMMAND_AGPS_STOP;
      g_agps_cmd_req_buf->header.id     = g_id++;

      LT_SendAgps(g_agps_cmd_req_buf);
#else
      ret = ioctl(g_fd, CXD56_GNSS_IOCTL_STOP, 0);
      if (0 != ret)
        {
          _err("L%d ioctl() %s\n", __LINE__, strerror( ret ));
          goto exit;
        }
#endif /* CONFIG_LTE */

      /* Lock */

      LOC_MNG_POS_LOCK();

      pos_snd_thread_id =  g_pos_snd_thread_id;

      /* Unlock */

      LOC_MNG_POS_UNLOCK();

      if (0 != pos_snd_thread_id)
        {
          ret = pthread_cancel(g_pos_snd_thread_id);
          if (0 != ret)
            {
              _err("L%d pthread_cancel() %s\n", __LINE__, strerror( ret ));
              ret = -ret;
              goto exit;
            }

          ret = pthread_join(g_pos_snd_thread_id, &res);
          if (0 != ret)
            {
              _err("L%d pthread_join() %s\n", __LINE__, strerror( ret ));
              ret = -ret;
              goto exit;
            }
        }

      g_sts = E_LOC_MNG_STS_STOP;
    }

exit:
  LOC_MNG_UNLOCK();
  return ret;
}

/* Finalize */
int LocMngFin(void)
{
  int  ret  = 0;
  void *res = NULL;

  if (E_LOC_MNG_STS_STOP != g_sts)
    {
      _err("L%d status error == %d\n", __LINE__, g_sts);
      ret = -EPERM;
      goto exit;
    }

  pthread_cancel(g_gnss_rcv_thread_id);
  if (0 != ret)
    {
      _err("L%d pthread_cancel() %s\n", __LINE__, strerror( ret ));
      ret = -ret;
      goto exit;
    }

  pthread_join(g_gnss_rcv_thread_id, &res);
  if (0 != ret)
    {
      _err("L%d pthread_join() %s\n", __LINE__, strerror( ret ));
      ret = -ret;
      goto exit;
    }

  ret = close(g_fd);
  if (0 > ret)
    {
      _err("L%d close error\n", __LINE__);
    }

  free(g_gnss_pos_data);

#ifdef CONFIG_LTE
  free(g_agps_cmd_buf);
  free(g_agps_cmd_req_buf);
#endif /* CONFIG_LTE */

  LOC_MNG_LOCK();
  while(g_head)
    {
      LocMngDeleteNode(g_head);
    }
  LOC_MNG_UNLOCK();
  g_sts = E_LOC_MNG_STS_UNINIT;

exit:
  return ret;
}
