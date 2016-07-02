/**************************************************************************************************
  Filename:       znwk_config.h
  Revised:        $Date: 2015-01-20 08:50:11 -0800 (Tue, 20 Jan 2015) $
  Revision:       $Revision: 41917 $

  Description:    This file contains the ZStack Network configuration
                  settings.


  Copyright 2014 - 2015 Texas Instruments Incorporated. All rights reserved.

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
  PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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

#ifndef ZNWK_CONFIG_H
#define ZNWK_CONFIG_H

#if !defined( ZNWK_DEFAULT_CHANLIST )
  /**
   * Maximum Channel List - all channels
   */
  #define ZSTART_MAX_CHANNELS_24GHZ      0x07FFF800
  /**
   * Default channel is Channel 11 - 0x0B
   * Channels are defined in the following:
   *         0      : 868 MHz     0x00000001
   *         1 - 10 : 915 MHz     0x000007FE
   *        11 - 26 : 2.4 GHz     0x07FFF800
   */
  //#define ZNWK_DEFAULT_CHANLIST 0x04000000  // 26 - 0x1A
  //#define ZNWK_DEFAULT_CHANLIST 0x02000000  // 25 - 0x19
  //#define ZNWK_DEFAULT_CHANLIST 0x01000000  // 24 - 0x18
  //#define ZNWK_DEFAULT_CHANLIST 0x00800000  // 23 - 0x17
  //#define ZNWK_DEFAULT_CHANLIST 0x00400000  // 22 - 0x16
  //#define ZNWK_DEFAULT_CHANLIST 0x00200000  // 21 - 0x15
  //#define ZNWK_DEFAULT_CHANLIST 0x00100000  // 20 - 0x14
  //#define ZNWK_DEFAULT_CHANLIST 0x00080000  // 19 - 0x13
  //#define ZNWK_DEFAULT_CHANLIST 0x00040000  // 18 - 0x12
  //#define ZNWK_DEFAULT_CHANLIST 0x00020000  // 17 - 0x11
  //#define ZNWK_DEFAULT_CHANLIST 0x00010000  // 16 - 0x10
  //#define ZNWK_DEFAULT_CHANLIST 0x00008000  // 15 - 0x0F
  //#define ZNWK_DEFAULT_CHANLIST 0x00004000  // 14 - 0x0E
  //#define ZNWK_DEFAULT_CHANLIST 0x00002000  // 13 - 0x0D
  //#define ZNWK_DEFAULT_CHANLIST 0x00001000  // 12 - 0x0C
  //#define ZNWK_DEFAULT_CHANLIST 0x00000800  // 11 - 0x0B
  #define ZNWK_DEFAULT_CHANLIST ZSTART_MAX_CHANNELS_24GHZ
#endif // ZNWK_DEFAULT_CHANLIST

/**
 * Define the default PAN ID.
 *
 * Setting this to a value other than 0xFFFF causes
 * ZDO_COORD to use this value as its PAN ID and
 * Routers and end devices to join PAN with this ID
 */
#if !defined( ZNWK_CONFIG_PAN_ID )
  #define ZNWK_CONFIG_PAN_ID 0xFFFF
#endif

/**
 * Define the default Extended PAN ID.
 *
 * Setting this to a value other than all 0xFF causes
 * ZDO_COORD to use this value as its extended PAN ID and
 * Routers and end devices to join extended PAN with this ID
 */
#if !defined( ZNWK_CONFIG_EXTENDED_PAN_ID )
  #define ZNWK_CONFIG_EXTENDED_PAN_ID {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
#endif

/**
 * End Device Poll Rate -
 * The number of milliseconds to wait between data request polls to the coordinator.
 */
#if !defined ( ZNWK_POLL_RATE )
  #define ZNWK_POLL_RATE 5000
#endif

#endif // ZNWK_CONFIG_H