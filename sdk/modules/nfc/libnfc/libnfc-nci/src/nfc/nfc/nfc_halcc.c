/******************************************************************************
 *
 *  Copyright (C) 2014 Sony Corporation
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


/******************************************************************************
 *
 *  This file contains functions that vendor specific interface with the
 *  HAL. On the receive side, it routes events to the appropriate handler
 *  (callback). On the transmit side, it manages the command transmission.
 *
 ******************************************************************************/
#include <string.h>
#include "gki.h"
#include "nfc_target.h"

#if (NFC_INCLUDED == TRUE)
#include "nfc_int.h"

/****************************************************************************
** Declarations
****************************************************************************/

#if (defined (NFA_HALCC_INCLUDED) && (NFA_HALCC_INCLUDED==TRUE))

/*******************************************************************************
**
** Function         NFC_SendHalControlCommand
**
** Description      This function is called to send the given vendor specific
**                  command to NFCC. The response from NFCC is reported to the
**                  given tNFC_VS_CBACK as (oid).
**
** Parameters       oid - The opcode of the VS command.
**                  p_data - The parameters for the VS command
**
** Returns          tNFC_STATUS
**
*******************************************************************************/
tNFC_STATUS NFC_SendHalControlCommand (UINT8          id,
                                       BT_HDR        *p_data,
                                       tNFC_HALCC_CBACK *p_cback)
{
    tNFC_STATUS     status = NFC_STATUS_OK;
    UINT8           *pp;

    /* Allow HALCC with 0-length payload */
    if (p_data == NULL)
    {
        p_data = NCI_GET_CMD_BUF (0);
        if (p_data)
        {
            p_data->offset  = NCI_VSC_MSG_HDR_SIZE;
            p_data->len     = 0;
        }
    }

    /* Validate parameters */ /* use VSC message struct */
    if ((p_data == NULL) || (p_data->offset < NCI_VSC_MSG_HDR_SIZE) || (p_data->len > NCI_MAX_VSC_SIZE))
    {
        NFC_TRACE_ERROR1 ("buffer offset must be >= %d", NCI_VSC_MSG_HDR_SIZE);
        if (p_data)
            GKI_freebuf (p_data);
        return NFC_STATUS_INVALID_PARAM;
    }

    p_data->event           = BT_EVT_TO_NFC_NCI;
    p_data->layer_specific  = 0;
    /* save the callback function in the BT_HDR, to receive the response */
    nfc_cb.p_halcc_cb = p_cback;

    p_data->offset -= NCI_MSG_HDR_SIZE;
    pp              = (UINT8 *) (p_data + 1) + p_data->offset;
    NCI_MSG_BLD_HDR0 (pp, NCI_MT_HALMSG, NCI_GID_PROP);
    *pp++           = id;
    *pp             = (UINT8) p_data->len;
    p_data->len    += NCI_MSG_HDR_SIZE;

    HAL_WRITE( p_data ); /* <= free p_data */

    return status;
}

#endif /*(defined (NFA_HALCC_INCLUDED) && (NFA_HALCC_INCLUDED==TRUE))*/

#endif /* NFC_INCLUDED == TRUE */
