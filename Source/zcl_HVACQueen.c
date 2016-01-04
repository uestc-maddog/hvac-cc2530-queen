/**************************************************************************************************
  Filename:       zcl_HVACQueen.c
  Revised:        $Date: 2015-08-19 17:11:00 -0700 (Wed, 19 Aug 2015) $
  Revision:       $Revision: 44460 $

  Description:    Zigbee Cluster Library - sample device application.


  Copyright 2006-2013 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/

/*********************************************************************
  This device will be like an On/Off Switch device. This application
  is not intended to be a On/Off Switch device, but will use the device
  description to implement this sample code.

  ----------------------------------------
  Main:
    - SW1: Toggle remote light
    - SW2: Invoke EZMode
    - SW4: Enable/Disable Permit Join
    - SW5: Go to Help screen
  ----------------------------------------
*********************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "ZComDef.h"
#include "OSAL.h"
#include "AF.h"
#include "ZDApp.h"
#include "ZDObject.h"
#include "ZDProfile.h"
#include "MT_SYS.h"

#include "zcl.h"
#include "zcl_general.h"
#include "zcl_ha.h"
#include "zcl_HVACQueen.h"
#include "zcl_ezmode.h"

#include "onboard.h"

/* HAL */
#include "hal_lcd.h"
#include "hal_led.h"
#include "hal_uart.h"

#if defined (OTA_CLIENT) && (OTA_CLIENT == TRUE)
#include "zcl_ota.h"
#include "hal_ota.h"
#endif

/* MT */
#include "MT_UART.h"
#include "MT.h"

#include "hvac_protocol0.h"

/* ZDO */
#include "ZDObject.h"

/* NWK */
#include "NLMEDE.h"
#include "AddrMgr.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */
/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
byte zclHVACQueen_TaskID;

uint8 zclHVACQueenSeqNum;

uint8 zclHVACQueen_OnOffSwitchType = ON_OFF_SWITCH_TYPE_TOGGLE;

uint8 zclHVACQueen_OnOffSwitchActions = ON_OFF_SWITCH_ACTIONS_2;   // Toggle -> Toggle

/*********************************************************************
 * GLOBAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
#ifdef ZCL_ON_OFF
afAddrType_t zclHVACQueen_DstAddr;
#endif

#ifdef ZCL_EZMODE
static void zclHVACQueen_ProcessZDOMsgs( zdoIncomingMsg_t *pMsg );
static void zclHVACQueen_EZModeCB( zlcEZMode_State_t state, zclEZMode_CBData_t *pData );

static const zclEZMode_RegisterData_t zclHVACQueen_RegisterEZModeData =
{
  &zclHVACQueen_TaskID,
  HVACQUEEN_EZMODE_NEXTSTATE_EVT,
  HVACQUEEN_EZMODE_TIMEOUT_EVT,
  &zclHVACQueenSeqNum,
  zclHVACQueen_EZModeCB
};

// NOT ZLC_EZMODE, Use EndDeviceBind
#else

static cId_t bindingOutClusters[] =
{
  ZCL_CLUSTER_ID_GEN_ON_OFF
};
#define ZCLHVACQUEEN_BINDINGLIST   (sizeof(bindingOutClusters)/sizeof(bindingOutClusters[0]))
#endif  // ZLC_EZMODE

// Endpoint to allow SYS_APP_MSGs
static endPointDesc_t HVACQueen_TestEp =
{
  HVACQUEEN_ENDPOINT,                  // endpoint
  &zclHVACQueen_TaskID,
  (SimpleDescriptionFormat_t *)NULL,  // No Simple description for this test endpoint
  (afNetworkLatencyReq_t)0            // No Network Latency req
};

uint8 giSwScreenMode = SW_MAINMODE;   // display the main screen mode first

static uint8 aProcessCmd[] = { 1, 0, 0, 0 }; // used for reset command, { length + cmd0 + cmd1 + data }

uint8 gPermitDuration = 0;    // permit joining default to disabled

devStates_t zclHVACQueen_NwkState = DEV_INIT;

#if defined (OTA_CLIENT) && (OTA_CLIENT == TRUE)
#define DEVICE_POLL_RATE                 8000   // Poll rate for end device
#endif

/*********************************************************************
 * LOCAL FUNCTIONS
 */
//static void zclHVACQueen_HandleKeys( byte shift, byte keys );
static void zclHVACQueen_BasicResetCB( void );
static void zclHVACQueen_IdentifyCB( zclIdentify_t *pCmd );
static void zclHVACQueen_IdentifyQueryRspCB(  zclIdentifyQueryRsp_t *pRsp );
static void zclHVACQueen_ProcessIdentifyTimeChange( void );

#ifdef HAL_UART
static void HVACQueen_HandleUart (mtOSALSerialData_t *pMsg);
#endif

// Functions to process ZCL Foundation incoming Command/Response messages
static void zclHVACQueen_ProcessIncomingMsg( zclIncomingMsg_t *msg );
#ifdef ZCL_READ
static uint8 zclHVACQueen_ProcessInReadRspCmd( zclIncomingMsg_t *pInMsg );
#endif
#ifdef ZCL_WRITE
static uint8 zclHVACQueen_ProcessInWriteRspCmd( zclIncomingMsg_t *pInMsg );
#endif
static uint8 zclHVACQueen_ProcessInDefaultRspCmd( zclIncomingMsg_t *pInMsg );
#ifdef ZCL_DISCOVER
static uint8 zclHVACQueen_ProcessInDiscCmdsRspCmd( zclIncomingMsg_t *pInMsg );
static uint8 zclHVACQueen_ProcessInDiscAttrsRspCmd( zclIncomingMsg_t *pInMsg );
static uint8 zclHVACQueen_ProcessInDiscAttrsExtRspCmd( zclIncomingMsg_t *pInMsg );
#endif

#if defined (OTA_CLIENT) && (OTA_CLIENT == TRUE)
static void zclHVACQueen_ProcessOTAMsgs( zclOTA_CallbackMsg_t* pMsg );
#endif

// Zigbee announce handler
uint8 hvacHandleZDOAnnounce(zdoIncomingMsg_t * );

// UART handler functions
static void hvacUART_PTL0_PING( void );
static void hvacUART_PTL0_ACK( void );
static void hvacUART_PTL0_TRS_TRANS( PTL0_InitTypeDef * );
static void hvacUART_PTL0_NWK_STATUS_RP( PTL0_InitTypeDef * );
static void hvacUART_PTL0_NWK_CMD( PTL0_InitTypeDef * );
static void hvacUART_PTL0_LOC_STATUS_RP( PTL0_InitTypeDef * );
static void hvacUART_PTL0_LOC_CMD( PTL0_InitTypeDef * );

/*********************************************************************
 * ZCL General Profile Callback table
 */
static zclGeneral_AppCallbacks_t zclHVACQueen_CmdCallbacks =
{
  zclHVACQueen_BasicResetCB,               // Basic Cluster Reset command
  zclHVACQueen_IdentifyCB,                 // Identify command
#ifdef ZCL_EZMODE
  NULL,                                   // Identify EZ-Mode Invoke command
  NULL,                                   // Identify Update Commission State command
#endif
  NULL,                                   // Identify Trigger Effect command
  zclHVACQueen_IdentifyQueryRspCB,         // Identify Query Response command
  NULL,                                   // On/Off cluster commands
  NULL,                                   // On/Off cluster enhanced command Off with Effect
  NULL,                                   // On/Off cluster enhanced command On with Recall Global Scene
  NULL,                                   // On/Off cluster enhanced command On with Timed Off
#ifdef ZCL_LEVEL_CTRL
  NULL,                                   // Level Control Move to Level command
  NULL,                                   // Level Control Move command
  NULL,                                   // Level Control Step command
  NULL,                                   // Level Control Stop command
#endif
#ifdef ZCL_GROUPS
  NULL,                                   // Group Response commands
#endif
#ifdef ZCL_SCENES
  NULL,                                   // Scene Store Request command
  NULL,                                   // Scene Recall Request command
  NULL,                                   // Scene Response command
#endif
#ifdef ZCL_ALARMS
  NULL,                                   // Alarm (Response) commands
#endif
#ifdef SE_UK_EXT
  NULL,                                   // Get Event Log command
  NULL,                                   // Publish Event Log command
#endif
  NULL,                                   // RSSI Location command
  NULL                                    // RSSI Location Response command
};


/*********************************************************************
 * STATUS STRINGS
 */


/*********************************************************************
 * @fn          zclHVACQueen_Init
 *
 * @brief       Initialization function for the zclGeneral layer.
 *
 * @param       none
 *
 * @return      none
 */
void zclHVACQueen_Init( byte task_id )
{
  zclHVACQueen_TaskID = task_id;

#ifdef ZCL_ON_OFF
  // Set destination address to indirect
  zclHVACQueen_DstAddr.addrMode = (afAddrMode_t)AddrNotPresent;
  zclHVACQueen_DstAddr.endPoint = 0;
  zclHVACQueen_DstAddr.addr.shortAddr = 0;
#endif

  // This app is part of the Home Automation Profile
  zclHA_Init( &zclHVACQueen_SimpleDesc );

  // Register the ZCL General Cluster Library callback functions
  zclGeneral_RegisterCmdCallbacks( HVACQUEEN_ENDPOINT, &zclHVACQueen_CmdCallbacks );

  // Register the application's attribute list
  zcl_registerAttrList( HVACQUEEN_ENDPOINT, HVACQUEEN_MAX_ATTRIBUTES, zclHVACQueen_Attrs );

  // Register the Application to receive the unprocessed Foundation command/response messages
  zcl_registerForMsg( zclHVACQueen_TaskID );

  // Initialize UART
  MT_UartInit ();
  MT_UartRegisterTaskID (zclHVACQueen_TaskID);
    
  // Register for a test endpoint
  afRegister( &HVACQueen_TestEp );

  ZDO_RegisterForZDOMsg( zclHVACQueen_TaskID, Device_annce );
  ZDO_RegisterForZDOMsg( zclHVACQueen_TaskID, End_Device_Bind_rsp ); //? Consider remove
  ZDO_RegisterForZDOMsg( zclHVACQueen_TaskID, Match_Desc_rsp ); //? Consider remove
  
  // Init critical resource
  ptl0_initPTL0Status();
  
#if defined (OTA_CLIENT) && (OTA_CLIENT == TRUE)
  // Register for callback events from the ZCL OTA
  zclOTA_Register(zclHVACQueen_TaskID);
#endif

}

/*********************************************************************
 * @fn          zclSample_event_loop
 *
 * @brief       Event Loop Processor for zclGeneral.
 *
 * @param       none
 *
 * @return      none
 */
uint16 zclHVACQueen_event_loop( uint8 task_id, uint16 events )
{
  afIncomingMSGPacket_t *MSGpkt;
  (void)task_id;  // Intentionally unreferenced parameter

  if ( events & SYS_EVENT_MSG )
  {
    while ( (MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( zclHVACQueen_TaskID )) )
    {
      switch ( MSGpkt->hdr.event )
      { 
        case ZCL_INCOMING_MSG:
          // Incoming ZCL Foundation command/response messages
          zclHVACQueen_ProcessIncomingMsg( (zclIncomingMsg_t *)MSGpkt );
          break;

        case CMD_SERIAL_MSG:
          // UART data, uart handler
          HVACQueen_HandleUart ((mtOSALSerialData_t *)MSGpkt);
          
        /*case KEY_CHANGE:
          zclHVACQueen_HandleKeys( ((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys );
          break;*/
        
        case ZDO_CB_MSG:
          // in coming network announce
          hvacHandleZDOAnnounce((zdoIncomingMsg_t *)MSGpkt);
          
        case ZDO_STATE_CHANGE:
          zclHVACQueen_NwkState = (devStates_t)(MSGpkt->hdr.status);

          // now on the network
          if ( (zclHVACQueen_NwkState == DEV_ZB_COORD) ||
               (zclHVACQueen_NwkState == DEV_ROUTER)   ||
               (zclHVACQueen_NwkState == DEV_END_DEVICE) )
          {
#ifndef HOLD_AUTO_START
            giSwScreenMode = SW_MAINMODE;
            //zclHVACQueen_LcdDisplayUpdate();
#endif
          }
          break;

#if defined (OTA_CLIENT) && (OTA_CLIENT == TRUE)
        case ZCL_OTA_CALLBACK_IND:
          zclHVACQueen_ProcessOTAMsgs( (zclOTA_CallbackMsg_t*)MSGpkt  );
          break;
#endif

        default:
          break;
      }

      // Release the memory
      osal_msg_deallocate( (uint8 *)MSGpkt );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  if ( events & HVACQUEEN_IDENTIFY_TIMEOUT_EVT )
  {
    zclHVACQueen_IdentifyTime = 10;
    zclHVACQueen_ProcessIdentifyTimeChange();

    return ( events ^ HVACQUEEN_IDENTIFY_TIMEOUT_EVT );
  }

  // Discard unknown events
  return 0;
}


/*********************************************************************
 * @fn      hvacHandleZDOAnnounce
 *
 * @brief   Handles all network announce. Once a new device join 
 *          network, a announcement will send to network. 
 *
 * @param   MSGpkt - incoming message 
 *
 * @return  true or false
 */
uint8 hvacHandleZDOAnnounce(zdoIncomingMsg_t * MSGpkt)
{
  ZDO_DeviceAnnce_t Annce;
  AddrMgrEntry_t annouEntry;
  PTL0_InitTypeDef outGoingPTL0Msg;
  uint8 *ptl0_payloadbuf;
  
  // Parse message
  ZDO_ParseDeviceAnnce( MSGpkt, &Annce );
  
  annouEntry.user = ADDRMGR_USER_DEFAULT;
  osal_memcpy( annouEntry.extAddr, Annce.extAddr, Z_EXTADDR_LEN );
  
  // check devcie available? Already in table?
  if(AddrMgrEntryLookupExt( &annouEntry ))
  {
    // mac address already in table, child reset, rejoin network.
    // Do nothing for now. 
    asm("NOP");
  }
  else
  {
    // its a new child. Prepare a network status update for STM32.
    // use memory allocation to save data, must release memory after process
    
    // prepare the buffer to store the data payload
    ptl0_payloadbuf = (uint8 *)osal_mem_alloc( PTL0_NWK_STATUS_RP_NEWDEV_DATALENGTH );
    
    // check valid
    if (ptl0_payloadbuf == NULL)
      return false;
    
    // copy mac address to data payload
    osal_memcpy( ptl0_payloadbuf, Annce.extAddr, Z_EXTADDR_LEN );
    
    // assemble message.
    // (not finish)
    outGoingPTL0Msg.CMD1 = PTL0_NWK_STATUS_RP;
    outGoingPTL0Msg.CMD2 = PTL0_NWK_STATUS_RP_NEWDEV;
    outGoingPTL0Msg.datapointer = ptl0_payloadbuf;
    outGoingPTL0Msg.length = Z_EXTADDR_LEN;
    outGoingPTL0Msg.SOF = PTL0_SOF;
    outGoingPTL0Msg.version = PTL0_FRAMEVER;

    // push event into event stack
    ptl0_pushEvent(outGoingPTL0Msg);   
  }
  return true;
}


/*********************************************************************
 * @fn      HVACQueen_HandleUart
 *
 * @brief   Handles all UART events for this device.
 *
 * @param   pMsg - incoming uart msg
 *          pMsg->hdr : msg header. 
 *          pMsg->msg : msg buffer
 *          pMsg->msg[0], data length high 8 bits    
 *          pMsg->msg[1], data length low 8 bits
 *          pMsg->msg[2], CMD1
 *          pMsg->msg[3], CMD2
 *          pMsg->msg[4...], data payload (if there is one)
 *
 * @return  none
 */
static void HVACQueen_HandleUart (mtOSALSerialData_t *pMsg) 
{
  PTL0_InitTypeDef inComing_ptl0;
  
  /* load the frame */
  inComing_ptl0.length = (pMsg->msg[0] << 8) + pMsg->msg[1]; //load length
  inComing_ptl0.CMD1 = pMsg->msg[2];    // load CMD1
  inComing_ptl0.CMD2 = pMsg->msg[3];    // load CMD2
  if(inComing_ptl0.length)              // if there is a data payload       
    inComing_ptl0.datapointer = &pMsg->msg[4]; // load data payload
  
  /* msg handler, react according to different tasks 
   *
   * refer to hvac_protocol0.h for more cmd detail
   */
  switch(inComing_ptl0.CMD1)
  { 
    case PTL0_PING:
      // Ping command, send ack
      hvacUART_PTL0_PING();
      break;
           
    case PTL0_ACK:
      // ACK received, clear flag
      hvacUART_PTL0_ACK();
      break;   
      
    case PTL0_TRS_TRANS:
      // Transmission command, send to destination
      hvacUART_PTL0_TRS_TRANS(&inComing_ptl0);
      break;
        
    case PTL0_NWK_STATUS_RP:
      // Network status report 
      hvacUART_PTL0_NWK_STATUS_RP(&inComing_ptl0);
      break;
          
    case PTL0_NWK_CMD:
      // Network command  
      hvacUART_PTL0_NWK_CMD(&inComing_ptl0);
      break;
          
    case PTL0_LOC_STATUS_RP:
      // Local status report
      hvacUART_PTL0_LOC_STATUS_RP(&inComing_ptl0);
      break;
      
    case PTL0_LOC_CMD:
      // Local command
      hvacUART_PTL0_LOC_CMD(&inComing_ptl0);
      break;
      
    default:
      break;
  }
}

/*********************************************************************
 * @fn      hvacUART_PTL0_PING
 *
 * @brief   Receive PING CMD. Responce with ACK frame
 *
 * @param   none
 *
 * @return  none
 */
static void hvacUART_PTL0_PING( void )
{
  PTL0_InitTypeDef uartPtl0Temp;
  

  // If PTL0 idel, send ACK
  if(ptl0_queryStat() == PTL0_STA_IDEL)
  {
    // update PTL0 status
    ptl0_updateStat(PTL0_STA_PING_REC);
  
    // configurate the ACK structure
    uartPtl0Temp.SOF = PTL0_SOF;
    uartPtl0Temp.version = PTL0_FRAMEVER;
    uartPtl0Temp.length = 0;      // ACK frame, no data payload
    uartPtl0Temp.CMD1 = PTL0_ACK;
    uartPtl0Temp.CMD2 = PTL0_EMPTYCMD;
    
    // send through PTL0 UART
    ptl0_sendMsg(uartPtl0Temp);
    
    // back to idel
    ptl0_updateStat(PTL0_STA_IDEL);
  }
}

/*********************************************************************
 * @fn      hvacUART_PTL0_ACK
 *
 * @brief   Receive ACK. Set PTL0 status.
 *
 * @param   none
 *
 * @return  none
 */
static void hvacUART_PTL0_ACK( void )
{
  // check current state, see whether anything need to do
  switch(ptl0_queryStat())
  {
    // currently nothing to do.
    default:
      break;
  }
  
  // receive ACK, communication complete, back to idel
  ptl0_updateStat(PTL0_STA_IDEL);
}

/*********************************************************************
 * @fn      hvacUART_PTL0_TRS_TRANS
 *
 * @brief   Receiving transparent sending frame, send Msg
 *
 * @param   none
 *
 * @return  none
 */
static void hvacUART_PTL0_TRS_TRANS( PTL0_InitTypeDef *ptl0_buf )
{
  asm("NOP");
}

/*********************************************************************
 * @fn      hvacUART_PTL0_NWK_STATUS_RP
 *
 * @brief   Receiving network status report frame. Not available for 
 *          CC2530.
 *
 * @param   none
 *
 * @return  none
 */
static void hvacUART_PTL0_NWK_STATUS_RP( PTL0_InitTypeDef *ptl0_buf )
{
  // Not an option for CC2530, network report only initialize by 
  // CC2530 and received by STM32. 
  asm("NOP");
}

/*********************************************************************
 * @fn      hvacUART_PTL0_NWK_CMD
 *
 * @brief   Receiving network cmd report frame. 
 *
 * @param   none
 *
 * @return  none
 */
static void hvacUART_PTL0_NWK_CMD( PTL0_InitTypeDef *ptl0_buf )
{
  // update status, receive network cmd frame
  ptl0_updateStat(PTL0_STA_NWKCMD_REC);
  
  switch(ptl0_buf->CMD2)
  {
    // response according to different command
    default:
      break;
  }
  asm("NOP");
}

/*********************************************************************
 * @fn      hvacUART_PTL0_LOC_STATUS_RP
 *
 * @brief   Receiving local status report.
 *
 * @param   none
 *
 * @return  none
 */
static void hvacUART_PTL0_LOC_STATUS_RP( PTL0_InitTypeDef *ptl0_buf )
{
  asm("NOP");
}

/*********************************************************************
 * @fn      hvacUART_PTL0_LOC_CMD
 *
 * @brief   receiving local cmd report.
 *
 * @param   none
 *
 * @return  none
 */
static void hvacUART_PTL0_LOC_CMD( PTL0_InitTypeDef *ptl0_buf )
{
  asm("NOP");
}

/*********************************************************************
 * @fn      zclHVACQueen_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_5
 *                 HAL_KEY_SW_4
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */
/*static void zclHVACQueen_HandleKeys( byte shift, byte keys )
{
  // toggle remote light
  if ( keys & HAL_KEY_SW_1 )
  {
    giSwScreenMode = SW_MAINMODE;   // remove help screen if there

    // Using this as the "Light Switch"
#ifdef ZCL_ON_OFF
    zclGeneral_SendOnOff_CmdToggle( HVACQUEEN_ENDPOINT, &zclHVACQueen_DstAddr, FALSE, 0 );
#endif
#ifdef LCD_SUPPORTED
    HalLcdWriteString( (char *)sCmdSent, HAL_LCD_LINE_2 );

    // clear message on screen after 3 seconds
    osal_start_timerEx( zclHVACQueen_TaskID, HVACQUEEN_MAIN_SCREEN_EVT, 3000 );
#endif
  }

  // invoke EZ-Mode
  if ( keys & HAL_KEY_SW_2 )
  {
    giSwScreenMode = SW_MAINMODE;   // remove help screen if there

#ifdef ZCL_EZMODE
    {
      zclEZMode_InvokeData_t ezModeData;
      static uint16 clusterIDs[] = { ZCL_CLUSTER_ID_GEN_ON_OFF };   // only bind on the on/off cluster

      // Invoke EZ-Mode
      ezModeData.endpoint = HVACQUEEN_ENDPOINT; // endpoint on which to invoke EZ-Mode
      if ( (zclHVACQueen_NwkState == DEV_ZB_COORD) ||
               (zclHVACQueen_NwkState == DEV_ROUTER)   ||
               (zclHVACQueen_NwkState == DEV_END_DEVICE) )
      {
        ezModeData.onNetwork = TRUE;      // node is already on the network
      }
      else
      {
        ezModeData.onNetwork = FALSE;     // node is not yet on the network
      }
      ezModeData.initiator = TRUE;        // OnOffSwitch is an initiator
      ezModeData.numActiveOutClusters = 1;   // active output cluster
      ezModeData.pActiveOutClusterIDs = clusterIDs;
      ezModeData.numActiveInClusters = 0;  // no active input clusters
      ezModeData.pActiveInClusterIDs = NULL;
      zcl_InvokeEZMode( &ezModeData );

 #ifdef LCD_SUPPORTED
      HalLcdWriteString( "EZMode", HAL_LCD_LINE_2 );
 #endif
    }

#else // NOT ZCL_EZMODE
    // bind to remote light
    zAddrType_t dstAddr;
    HalLedSet ( HAL_LED_4, HAL_LED_MODE_OFF );

    // Initiate an End Device Bind Request, this bind request will
    // only use a cluster list that is important to binding.
    dstAddr.addrMode = afAddr16Bit;
    dstAddr.addr.shortAddr = 0;   // Coordinator makes the match
    ZDP_EndDeviceBindReq( &dstAddr, NLME_GetShortAddr(),
                           HVACQUEEN_ENDPOINT,
                           ZCL_HA_PROFILE_ID,
                           0, NULL,   // No incoming clusters to bind
                           ZCLHVACQUEEN_BINDINGLIST, bindingOutClusters,
                           TRUE );
#endif // ZCL_EZMODE
  }

  // toggle permit join
  if ( keys & HAL_KEY_SW_4 )
  {
    giSwScreenMode = SW_MAINMODE;   // remove help screen if there

    if ( ( zclHVACQueen_NwkState == DEV_ZB_COORD ) ||
         ( zclHVACQueen_NwkState == DEV_ROUTER ) )
    {
      zAddrType_t tmpAddr;

      tmpAddr.addrMode = Addr16Bit;
      tmpAddr.addr.shortAddr = NLME_GetShortAddr();

      // toggle permit join
      gPermitDuration = gPermitDuration ? 0 : 0xff;

      // Trust Center significance is always true
      ZDP_MgmtPermitJoinReq( &tmpAddr, gPermitDuration, TRUE, FALSE );
    }
  }

  if ( shift && ( keys & HAL_KEY_SW_5 ) )
  {
    zclHVACQueen_BasicResetCB();
  }
  else if ( keys & HAL_KEY_SW_5 )
  {
    giSwScreenMode = giSwScreenMode ? SW_MAINMODE : SW_HELPMODE;
#ifdef LCD_SUPPORTED
    HalLcdWriteString( (char *)sClearLine, HAL_LCD_LINE_2 );
#endif
  }

  // update the display
  zclHVACQueen_LcdDisplayUpdate();
}
*/

/*********************************************************************
 * @fn      zclHVACQueen_LcdDisplayUpdate
 *
 * @brief   Called to update the LCD display.
 *
 * @param   none
 *
 * @return  none
 */
/*void zclHVACQueen_LcdDisplayUpdate(void)
{
  if ( giSwScreenMode == SW_HELPMODE )
  {
    zclHVACQueen_LcdDisplayHelpMode();
  }
  else
  {
    zclHVACQueen_LcdDisplayMainMode();
  }
}*/

/*********************************************************************
 * @fn      zclHVACQueen_LcdDisplayMainMode
 *
 * @brief   Called to display the main screen on the LCD.
 *
 * @param   none
 *
 * @return  none
 */
/*void zclHVACQueen_LcdDisplayMainMode(void)
{
  if ( zclHVACQueen_NwkState == DEV_ZB_COORD )
  {
    zclHA_LcdStatusLine1(0);
  }
  else if ( zclHVACQueen_NwkState == DEV_ROUTER )
  {
    zclHA_LcdStatusLine1(1);
  }
  else if ( zclHVACQueen_NwkState == DEV_END_DEVICE )
  {
    zclHA_LcdStatusLine1(2);
  }

  if ( ( zclHVACQueen_NwkState == DEV_ZB_COORD ) ||
       ( zclHVACQueen_NwkState == DEV_ROUTER ) )
  {
    // display help key with permit join status
    if ( gPermitDuration )
    {
      HalLcdWriteString("SW5: Help      *", HAL_LCD_LINE_3);
    }
    else
    {
      HalLcdWriteString("SW5: Help       ", HAL_LCD_LINE_3);
    }
  }
  else
  {
    // display help key
    HalLcdWriteString((char *)sSwHelp, HAL_LCD_LINE_3);
  }
#endif
}*/

/*********************************************************************
 * @fn      zclHVACQueen_LcdDisplayHelpMode
 *
 * @brief   Called to display the SW options on the LCD.
 *
 * @param   none
 *
 * @return  none
 */
/*void zclHVACQueen_LcdDisplayHelpMode(void)
{
#ifdef LCD_SUPPORTED
  HalLcdWriteString( (char *)sSwLight, HAL_LCD_LINE_1 );
  HalLcdWriteString( (char *)sSwEZMode, HAL_LCD_LINE_2 );
  HalLcdWriteString( (char *)sSwHelp, HAL_LCD_LINE_3 );
#endif
}*/

/*********************************************************************
 * @fn      zclHVACQueen_ProcessIdentifyTimeChange
 *
 * @brief   Called to process any change to the IdentifyTime attribute.
 *
 * @param   none
 *
 * @return  none
 */
static void zclHVACQueen_ProcessIdentifyTimeChange( void )
{
  if ( zclHVACQueen_IdentifyTime > 0 )
  {
    osal_start_timerEx( zclHVACQueen_TaskID, HVACQUEEN_IDENTIFY_TIMEOUT_EVT, 1000 );
    HalLedBlink ( HAL_LED_4, 0xFF, HAL_LED_DEFAULT_DUTY_CYCLE, HAL_LED_DEFAULT_FLASH_TIME );
  }
  else
  {
    if ( zclHVACQueen_OnOff )
      HalLedSet ( HAL_LED_4, HAL_LED_MODE_ON );
    else
      HalLedSet ( HAL_LED_4, HAL_LED_MODE_OFF );
    osal_stop_timerEx( zclHVACQueen_TaskID, HVACQUEEN_IDENTIFY_TIMEOUT_EVT );
  }
}

/*********************************************************************
 * @fn      zclHVACQueen_BasicResetCB
 *
 * @brief   Callback from the ZCL General Cluster Library
 *          to set all the Basic Cluster attributes to  default values.
 *
 * @param   none
 *
 * @return  none
 */
static void zclHVACQueen_BasicResetCB( void )
{
  // Put device back to factory default settings
  zgWriteStartupOptions( ZG_STARTUP_SET, 3 );   // bit set both default configuration and default network

  // restart device
  //MT_SysCommandProcessing( aProcessCmd );
  
  /* Need reset code here!! */
}

/*********************************************************************
 * @fn      zclHVACQueen_IdentifyCB
 *
 * @brief   Callback from the ZCL General Cluster Library when
 *          it received an Identity Command for this application.
 *
 * @param   srcAddr - source address and endpoint of the response message
 * @param   identifyTime - the number of seconds to identify yourself
 *
 * @return  none
 */
static void zclHVACQueen_IdentifyCB( zclIdentify_t *pCmd )
{
  zclHVACQueen_IdentifyTime = pCmd->identifyTime;
  zclHVACQueen_ProcessIdentifyTimeChange();
}

/*********************************************************************
 * @fn      zclHVACQueen_IdentifyQueryRspCB
 *
 * @brief   Callback from the ZCL General Cluster Library when
 *          it received an Identity Query Response Command for this application.
 *
 * @param   srcAddr - source address
 * @param   timeout - number of seconds to identify yourself (valid for query response)
 *
 * @return  none
 */
static void zclHVACQueen_IdentifyQueryRspCB(  zclIdentifyQueryRsp_t *pRsp )
{
  (void)pRsp;
#ifdef ZCL_EZMODE
  {
    zclEZMode_ActionData_t data;
    data.pIdentifyQueryRsp = pRsp;
    zcl_EZModeAction ( EZMODE_ACTION_IDENTIFY_QUERY_RSP, &data );
  }
#endif
}

/******************************************************************************
 *
 *  Functions for processing ZCL Foundation incoming Command/Response messages
 *
 *****************************************************************************/

/*********************************************************************
 * @fn      zclHVACQueen_ProcessIncomingMsg
 *
 * @brief   Process ZCL Foundation incoming message
 *
 * @param   pInMsg - pointer to the received message
 *
 * @return  none
 */
static void zclHVACQueen_ProcessIncomingMsg( zclIncomingMsg_t *pInMsg )
{
  switch ( pInMsg->zclHdr.commandID )
  {
#ifdef ZCL_READ
    case ZCL_CMD_READ_RSP:
      zclHVACQueen_ProcessInReadRspCmd( pInMsg );
      break;
#endif
#ifdef ZCL_WRITE
    case ZCL_CMD_WRITE_RSP:
      zclHVACQueen_ProcessInWriteRspCmd( pInMsg );
      break;
#endif
#ifdef ZCL_REPORT
    // See ZCL Test Applicaiton (zcl_testapp.c) for sample code on Attribute Reporting
    case ZCL_CMD_CONFIG_REPORT:
      //zclHVACQueen_ProcessInConfigReportCmd( pInMsg );
      break;

    case ZCL_CMD_CONFIG_REPORT_RSP:
      //zclHVACQueen_ProcessInConfigReportRspCmd( pInMsg );
      break;

    case ZCL_CMD_READ_REPORT_CFG:
      //zclHVACQueen_ProcessInReadReportCfgCmd( pInMsg );
      break;

    case ZCL_CMD_READ_REPORT_CFG_RSP:
      //zclHVACQueen_ProcessInReadReportCfgRspCmd( pInMsg );
      break;

    case ZCL_CMD_REPORT:
      //zclHVACQueen_ProcessInReportCmd( pInMsg );
      break;
#endif
    case ZCL_CMD_DEFAULT_RSP:
      zclHVACQueen_ProcessInDefaultRspCmd( pInMsg );
      break;
#ifdef ZCL_DISCOVER
    case ZCL_CMD_DISCOVER_CMDS_RECEIVED_RSP:
      zclHVACQueen_ProcessInDiscCmdsRspCmd( pInMsg );
      break;

    case ZCL_CMD_DISCOVER_CMDS_GEN_RSP:
      zclHVACQueen_ProcessInDiscCmdsRspCmd( pInMsg );
      break;

    case ZCL_CMD_DISCOVER_ATTRS_RSP:
      zclHVACQueen_ProcessInDiscAttrsRspCmd( pInMsg );
      break;

    case ZCL_CMD_DISCOVER_ATTRS_EXT_RSP:
      zclHVACQueen_ProcessInDiscAttrsExtRspCmd( pInMsg );
      break;
#endif
    default:
      break;
  }

  if ( pInMsg->attrCmd )
    osal_mem_free( pInMsg->attrCmd );
}

#ifdef ZCL_READ
/*********************************************************************
 * @fn      zclHVACQueen_ProcessInReadRspCmd
 *
 * @brief   Process the "Profile" Read Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclHVACQueen_ProcessInReadRspCmd( zclIncomingMsg_t *pInMsg )
{
  zclReadRspCmd_t *readRspCmd;
  uint8 i;

  readRspCmd = (zclReadRspCmd_t *)pInMsg->attrCmd;
  for (i = 0; i < readRspCmd->numAttr; i++)
  {
    // Notify the originator of the results of the original read attributes
    // attempt and, for each successfull request, the value of the requested
    // attribute
  }

  return TRUE;
}
#endif // ZCL_READ

#ifdef ZCL_WRITE
/*********************************************************************
 * @fn      zclHVACQueen_ProcessInWriteRspCmd
 *
 * @brief   Process the "Profile" Write Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclHVACQueen_ProcessInWriteRspCmd( zclIncomingMsg_t *pInMsg )
{
  zclWriteRspCmd_t *writeRspCmd;
  uint8 i;

  writeRspCmd = (zclWriteRspCmd_t *)pInMsg->attrCmd;
  for (i = 0; i < writeRspCmd->numAttr; i++)
  {
    // Notify the device of the results of the its original write attributes
    // command.
  }

  return TRUE;
}
#endif // ZCL_WRITE

/*********************************************************************
 * @fn      zclHVACQueen_ProcessInDefaultRspCmd
 *
 * @brief   Process the "Profile" Default Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclHVACQueen_ProcessInDefaultRspCmd( zclIncomingMsg_t *pInMsg )
{
  // zclDefaultRspCmd_t *defaultRspCmd = (zclDefaultRspCmd_t *)pInMsg->attrCmd;
  // Device is notified of the Default Response command.
  (void)pInMsg;
  return TRUE;
}

#ifdef ZCL_DISCOVER
/*********************************************************************
 * @fn      zclHVACQueen_ProcessInDiscCmdsRspCmd
 *
 * @brief   Process the Discover Commands Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclHVACQueen_ProcessInDiscCmdsRspCmd( zclIncomingMsg_t *pInMsg )
{
  zclDiscoverCmdsCmdRsp_t *discoverRspCmd;
  uint8 i;

  discoverRspCmd = (zclDiscoverCmdsCmdRsp_t *)pInMsg->attrCmd;
  for ( i = 0; i < discoverRspCmd->numCmd; i++ )
  {
    // Device is notified of the result of its attribute discovery command.
  }

  return TRUE;
}

/*********************************************************************
 * @fn      zclHVACQueen_ProcessInDiscAttrsRspCmd
 *
 * @brief   Process the "Profile" Discover Attributes Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclHVACQueen_ProcessInDiscAttrsRspCmd( zclIncomingMsg_t *pInMsg )
{
  zclDiscoverAttrsRspCmd_t *discoverRspCmd;
  uint8 i;

  discoverRspCmd = (zclDiscoverAttrsRspCmd_t *)pInMsg->attrCmd;
  for ( i = 0; i < discoverRspCmd->numAttr; i++ )
  {
    // Device is notified of the result of its attribute discovery command.
  }

  return TRUE;
}

/*********************************************************************
 * @fn      zclHVACQueen_ProcessInDiscAttrsExtRspCmd
 *
 * @brief   Process the "Profile" Discover Attributes Extended Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclHVACQueen_ProcessInDiscAttrsExtRspCmd( zclIncomingMsg_t *pInMsg )
{
  zclDiscoverAttrsExtRsp_t *discoverRspCmd;
  uint8 i;

  discoverRspCmd = (zclDiscoverAttrsExtRsp_t *)pInMsg->attrCmd;
  for ( i = 0; i < discoverRspCmd->numAttr; i++ )
  {
    // Device is notified of the result of its attribute discovery command.
  }

  return TRUE;
}
#endif // ZCL_DISCOVER


#if defined (OTA_CLIENT) && (OTA_CLIENT == TRUE)
/*********************************************************************
 * @fn      zclHVACQueen_ProcessOTAMsgs
 *
 * @brief   Called to process callbacks from the ZCL OTA.
 *
 * @param   none
 *
 * @return  none
 */
static void zclHVACQueen_ProcessOTAMsgs( zclOTA_CallbackMsg_t* pMsg )
{
  uint8 RxOnIdle;

  switch(pMsg->ota_event)
  {
  case ZCL_OTA_START_CALLBACK:
    if (pMsg->hdr.status == ZSuccess)
    {
      // Speed up the poll rate
      RxOnIdle = TRUE;
      ZMacSetReq( ZMacRxOnIdle, &RxOnIdle );
      NLME_SetPollRate( 2000 );
    }
    break;

  case ZCL_OTA_DL_COMPLETE_CALLBACK:
    if (pMsg->hdr.status == ZSuccess)
    {
      // Reset the CRC Shadow and reboot.  The bootloader will see the
      // CRC shadow has been cleared and switch to the new image
      HalOTAInvRC();
      SystemReset();
    }
    else
    {
      // slow the poll rate back down.
      RxOnIdle = FALSE;
      ZMacSetReq( ZMacRxOnIdle, &RxOnIdle );
      NLME_SetPollRate(DEVICE_POLL_RATE);
    }
    break;

  default:
    break;
  }
}
#endif // defined (OTA_CLIENT) && (OTA_CLIENT == TRUE)

/****************************************************************************
****************************************************************************/


