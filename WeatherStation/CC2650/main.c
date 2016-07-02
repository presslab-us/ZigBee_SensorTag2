/**
  @file  main.c
  $Date: 2015-02-10 10:10:47 -0800 (Tue, 10 Feb 2015) $
  $Revision: 42474 $

  @brief Main entry of the Z-Stack sample application

  <!--
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
  -->
*/

#include <string.h>
#include <xdc/std.h>
#include <xdc/runtime/Error.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/hal/Hwi.h>

#include <ioc.h>

#include "ICall.h"
#include "Board.h"
#include "TIMACBoard.h"
#include "CryptoBoard.h"
#include <ti/sysbios/family/arm/cc26xx/Power.h>
#include <ti/drivers/crypto/CryptoCC26XX.h>

#include <ti/sysbios/family/arm/cc26xx/Power.h>
#include <ti/sysbios/family/arm/cc26xx/PowerCC2650.h>
#include <aon_rtc.h>
#include <prcm.h>

/* Header files required to enable instruction fetch cache */
#include <vims.h>
#include <hw_memmap.h>

#include "cpu.h"
#include "hw_types.h"
#include <inc/hw_ccfg.h>
#include <inc/hw_ccfg_simple_struct.h>

#include "pwrmon.h"
#include "nvoctp.h"
#include "sensortagapp.h"
#include "zstackconfig.h"

/******************************************************************************
 * CONSTANTS
 */

// IEEE Mode
#define RFC_MODE_IEEE PRCM_RFCMODESEL_CURR_MODE2

// Size of the Application Thread's Stack
#define MY_TASK_STACK_SIZE 1024

// Exented Address Length
#define EXTADDR_LEN 8

// Extended Address offset in FCFG (LSB..MSB)
#define EXTADDR_OFFSET 0x2F0

// Minimum voltage level need to run
#if !defined (MIN_VDD_RUN)
#define MIN_VDD_RUN         MIN_VDD_POLL
#endif

/******************************************************************************
 * MACROS
 */

// Macro to set the RF Mode
#define SET_RFC_MODE(mode) HWREG(PRCM_BASE + PRCM_O_RFCMODESEL) = (mode)

// Macro used to break a uint32_t into individual bytes
#define BREAK_UINT32(var, ByteNum) \
    (uint8)((uint32)(((var) >> ((ByteNum) * 8)) & 0x00FF))

/******************************************************************************
 * ZStack Configuration Structure
 */
zstack_Config_t user0Cfg =
{
    {0, 0, 0, 0, 0, 0, 0, 0}, // Extended Address
    {0, 0, 0, 0, 0, 0, 0, 0}, // NV function pointers
    0,                        // ICall application thread ID
    0,                        // stack image init fail flag
    MAC_USER_CFG
};

/******************************************************************************
 * LOCAL VARIABLES
 */

// Main application stack space
static Task_Struct myTask;
static Char myTaskStack[MY_TASK_STACK_SIZE];

// Used to check for a valid extended address
static const uint8_t dummyExtAddr[] =
    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

/******************************************************************************
 * EXTERNALS
 */

extern const ccfg_t __ccfg;

/*******************************************************************************
 * @fn          taskFxn
 *
 * @brief       Application thread starting function
 *
 * @param       a0 - argument 0, not used
 * @param       a1 - argument 1, not used
 *
 * @return      none
 */
Void taskFxn(UArg a0, UArg a1)
{
    /* Disallow shutting down JTAG, VIMS, SYSBUS during idle state
     * since TIMAC requires SYSBUS during idle. */
    Power_setConstraint(Power_IDLE_PD_DISALLOW);

    /* Initialize the Power Monitor */
    PWRMON_init();

    /* Initialize ICall module */
    ICall_init();

    {
        CryptoCC26XX_Params CryptoCC26XXParams;
        extern CryptoCC26XX_Handle CryptoCC26XXHandle;

        // Initialize the Crypto Driver
        CryptoCC26XX_init();
        CryptoCC26XX_Params_init(&CryptoCC26XXParams);
        CryptoCC26XXHandle = CryptoCC26XX_open(Board_CRYPTO, false,
                                               &CryptoCC26XXParams);
        if(!CryptoCC26XXHandle)
        {
            Task_exit();
        }
        Crypto_init();
    }

    /*
     * Copy the extended address from the CCFG area
     * Assumption: the memory in CCFG_IEEE_MAC_0 and CCFG_IEEE_MAC_1
     * is contiguous and LSB first.
     */
    memcpy(user0Cfg.extendedAddress,
           (uint8_t *)&(__ccfg.CCFG_IEEE_MAC_0), EXTADDR_LEN);

    // Check to see if the CCFG IEEE is valid
    if(memcmp(user0Cfg.extendedAddress, dummyExtAddr, EXTADDR_LEN) == 0)
    {
        // No, it isn't valid.  Get the Primary IEEE Address
        memcpy(user0Cfg.extendedAddress,
               (uint8_t *)(FCFG1_BASE + EXTADDR_OFFSET), EXTADDR_LEN);
    }

    // Make sure the voltage level is high enough to operate
    while ( PWRMON_check( MIN_VDD_RUN ) == false )
    {
      // Add your own code to do something here, flash LED, sleep, something
      // For now, we will loop and wait for power comes up to the
      // MIN_VDD_RUN level.
    }

#if !defined (FEATURE_APP_NO_NV_INIT)
    /* Setup the NV driver for the ZStack thread */
    NVOCTP_loadApiPtrs(&user0Cfg.nvFps);
#endif // !FEATURE_APP_NO_NV_INIT

    if(user0Cfg.nvFps.initNV)
    {
        user0Cfg.nvFps.initNV(NULL);
    }

    /* Start tasks of external images */
    ICall_createRemoteTasks();

    /* Kick off application */
    SensorTagApp_task(&user0Cfg.nvFps);
}

/*******************************************************************************
 * @fn          main
 *
 * @brief       entry function - standard C [main()]
 *
 * @param       none
 *
 * @return      none
 */
Void main()
{
    Task_Params taskParams;

    // set RFC mode to support IEEE802.15.4
    // Note: This must be done before the RF Core is released from reset!
    SET_RFC_MODE(RFC_MODE_IEEE);

    // enable iCache prefetching
    VIMSConfigure(VIMS_BASE, TRUE, TRUE);

    // Enable cache
    VIMSModeSet(VIMS_BASE, VIMS_MODE_ENABLED);

    /* Initialization for board related stuff such as LEDs
     * following TI-RTOS convention */
    PIN_init(BoardGpioInitTable);

    // Configure task.
    Task_Params_init(&taskParams);
    taskParams.stack = myTaskStack;
    taskParams.stackSize = MY_TASK_STACK_SIZE;
    taskParams.priority = 1;
    Task_construct(&myTask, taskFxn, &taskParams, NULL);

    BIOS_start();   /* enable interrupts and start SYS/BIOS */
}

/*******************************************************************************
 * @fn          halAssertHandler
 *
 * @brief       HAL assert handler required by OSAL memory module.
 *
 * @param       none
 *
 * @return      none
 */
void halAssertHandler(void)
{
    Hwi_disable();
    while(1) {}
}

/*******************************************************************************
 * @fn          exceptionHandler
 *
 * @brief       Exception Handler Loop
 *
 * @param       none
 *
 * @return      none
 */
void exceptionHandler()
{
    while(1) {}
}

/*******************************************************************************
 * @fn          smallErrorHook
 *
 * @brief       Error handler to be hooked into TI-RTOS
 *
 * @param       none
 *
 * @return      none
 */
Void smallErrorHook(Error_Block *eb)
{
    while(1) {}
}
