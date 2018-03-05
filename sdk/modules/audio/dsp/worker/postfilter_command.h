/**********************************************************************
 *
 *      File Name: postfilter_command.h
 *
 *      Description: Postfilter Command definition 
 *
 *      Notes: (C) Copyright 2018 Sony Corporation
 *
 *      Author: -
 *
 **********************************************************************
 */

#ifndef __POSTFILTER_COMMAND_H__
#define __POSTFILTER_COMMAND_H__

#include <stdint.h>

class PostFilterCommand
{
public:

  enum command_type
  {
    Boot = 0,
    Init,
    Exec,
    Flush,
    SetParam,
    Tuning,
    Error,
    CmdTypeNum
  };
  typedef command_type CmdType;

  enum process_mode 
  {
    CommonMode = 0,
    FilterMode = 2,
  };
  typedef process_mode ProcMode;

  enum data_type
  {
    DataTypeAddr = 0,
    DataTypeValue,
  };
  typedef data_type DataType;

  enum result_code
  {
    ExecOk = 0,
    ExecWarn,
    ExecError,
  };
  typedef result_code ResultCode;

  struct Buffer
  {
    void     *addr;
    uint32_t sample;
  };

  /*! Command header */

  struct command_header_s
  {
    uint8_t cmd_type;
    uint8_t process_mode;
  };
  typedef command_header_s CmdHeader;

  /*! Initialize command */

  struct init_command_s
  {
    uint32_t ch_num;
    uint32_t bit_width;
    uint32_t sample;
  };
  typedef init_command_s InitCmd;

  /*! Execution command */

  struct exec_command_s
  {
    Buffer input;
    Buffer output;
  };
  typedef exec_command_s ExecCmd;

  /*! Flush command */

  struct flush_command_s
  {
    Buffer output;
  };
  typedef flush_command_s FlushCmd;

  /*! Result */

  struct result_s
  {
    ResultCode result_code;
  };
  typedef result_s RestltParam;


  /*! Command structure definition */

  struct command_s
  {
    CmdHeader header;

    union
    {
      InitCmd  init_cmd;
      ExecCmd  exec_cmd;
      FlushCmd flush_cmd;
    };

    RestltParam result;
  };
  typedef command_s Cmd;
};


#endif /* __POSTFILTER_COMMAND_H__ */

