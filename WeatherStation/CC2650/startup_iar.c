//*****************************************************************************
//! @file       startup_iar.c
//! @brief      Startup code for CC26xx for use with IAR EWARM.
//!
//! Revised     $Date: 2014-11-24 13:25:29 -0800 (Mon, 24 Nov 2014) $
//! Revision    $Revision: 41230 $
//
//  Copyright (C) 2013 Texas Instruments Incorporated - http://www.ti.com/
//
//
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions
//  are met:
//
//    Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
//    Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//
//    Neither the name of Texas Instruments Incorporated nor the names of
//    its contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
//  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
//  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//****************************************************************************/

#include "hw_memmap.h"
#include "hw_types.h"
#include "hw_ioc.h"
#include "hw_adi_2_refsys.h"
#include "hw_adi_3_refsys.h"
#include "hw_aon_sysctrl12.h"
#include "hw_factory_cfg.h"
#include "hw_osc_dig.h"
#include "hw_aon_wuc.h"
#include "hw_aux_wuc.h"
#include "ddi.h"
#include "adi.h"
#include "ioc.h"

// We need intrinsic functions for IAR (if used in source code)
#ifdef __IAR_SYSTEMS_ICC__
#include <intrinsics.h>
#endif

//*****************************************************************************
//
// 'Magic' trim value
//
//*****************************************************************************
#define FINAL_TRIM_DONE               0xC5

#define BACKDOOR_TRAP

#ifdef BACKDOOR_TRAP
#include "hw_prcm.h"
#include "prcm.h"
#include "interrupt.h"
#include "hw_ints.h"
#include "gpio.h"

//*****************************************************************************
//
// Backdoor modes
//
//*****************************************************************************
#define BACKDOOR_POLLED     0x00000001
#define BACKDOOR_INTERRUPT  0x00000002

// HAL Boot State
#define HAL_BOOT_IDLE                  0xFFFFFFFF
#define HAL_BOOT_COLD                  0x80402010
#define HAL_BOOT_WARM                  0x08040201

//*****************************************************************************
//
// Prototypes for the APIs.
//
//*****************************************************************************

void BackdoorInit(uint32_t ui32Pin, uint32_t ui32TrigLevel, uint32_t ui32Mode);
void BackdoorOpen(void);
void BackdoorCheck(void);
#endif // BACKDOOR_TRAP

#ifdef BACKDOOR_TRAP
#define BACKDOOR_CHECK()  BackdoorCheck();
#else
#define BACKDOOR_CHECK()
#endif // BACKDOOR_TRAP

#ifdef BACKDOOR_TRAP
typedef struct tBackdoor {
    uint32_t ui32Pin;
    uint32_t ui32Level;
} tBackdoor;

static tBackdoor g_tBackdoor;
bool g_bBackdoorActivated;

// variable to indicate cold/warm boot
volatile uint32_t halBootState = HAL_BOOT_COLD;

/*******************************************************************************
 * @fn          BackdoorCheck
 *
 * @brief       Used to trap the device in a spin when a button is pressed out
 *              of reset. Can be used to get control of the device in the event
 *              any flash issue occurs that prevents proper boot or a debugger
 *              connection.
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
void BackdoorCheck( void )
{
  // Turn on peripheral domain
  HWREG( PRCM_BASE + PRCM_O_SWPWR_CTRL ) = PRCM_SWPWR_CTRL_PERIPH;
  while (( HWREG( PRCM_BASE + PRCM_O_SWPWR_STATUS ) & PRCM_SWPWR_CTRL_PERIPH ) != PRCM_SWPWR_CTRL_PERIPH ) { }

  BackdoorInit( (1<<IOID_11), 0, BACKDOOR_POLLED );

  BackdoorOpen();

  return;
}


//*****************************************************************************
//
//! Initializes a GPIO to function as backdoor.
//!
//! \param ui32Pin is the pin to setup as backdoor
//! \param iu32TrigLevel is the pin value that triggers the backdoor.
//! \param iu32Mode is the pin value that triggers the backdoor.
//!
//! This function will initialize the specified GPIO to function as a backdoor.
//! The backdoor should be initialized and checked as the first thing the
//! program does.
//!
//! The 'backdoor' can run in polled or interrupt mode. If using polled mode,
//! the program should query the backdoor before executing dobious code that
//! might lock up the chip. If using interrupt mode, the backdoor can be
//! triggered at any point in the program (before the chip locks...).
//! The two modes are selected using
//! - \b BACKDOOR_POLLED
//! - \b BACKDOOR_INTERRUPT
//!
//! The 'backdoor' is basically and endless loop, that conserves the chip in the
//! state it was in when the backdoor was triggered, allowing the debugger to
//! access the device and erase faulty code.
//!
//! It is assumed that the caller has previously turned on the peripheral power
//! domain such that the GPIO module is powered.
//!
//! \return None.
//
//*****************************************************************************
void
BackdoorInit(uint32_t ui32Pin, uint32_t ui32TrigLevel, uint32_t ui32Mode)
{
    uint32_t ui32IoConfig;
    uint32_t ui32IoId;

    //
    // Enable the GPIO peripheral module in run mode
    //
    PRCMPeripheralRunEnable(PRCM_PERIPH_GPIO);
    PRCMPeripheralSleepEnable(PRCM_PERIPH_GPIO);
    PRCMPeripheralDeepSleepEnable(PRCM_PERIPH_GPIO);
    PRCMLoadSet();
    while(!PRCMLoadGet())
    { }

    //
    // Convert the pin number go an IO Id
    //
#ifdef __IAR_SYSTEMS_ICC__
    ui32IoId = 31 - __CLZ(ui32Pin);
#else
    ui32IoId = 0;
    while(!(ui32Pin & (1 << ui32IoId)))
    {
    	ui32IoId++;
    }
#endif

    //
    // Disable and clear any pending interrupts
    //
    IOCIntDisable(ui32IoId);
    IOCIntClear(ui32IoId);

    //
    // Configure the IO...
    //
    ui32IoConfig = IOC_STD_INPUT & ~(IOC_NO_IOPULL | IOC_BOTH_EDGES);

    //
    // Check if using interrupt mode
    //
    if(ui32Mode == BACKDOOR_INTERRUPT)
    {
        g_bBackdoorActivated = false;
        ui32IoConfig |= IOC_INT_ENABLE;
        if(ui32TrigLevel == 1)
        {
            ui32IoConfig |= (IOC_RISING_EDGE | IOC_IOPULL_DOWN);
        }
        else
        {
            ui32IoConfig |= (IOC_FALLING_EDGE | IOC_IOPULL_UP);
        }

        //
        // Enable global interrupts and edge interrupt
        //
        IntEnable(INT_EDGE_DETECT);
        IntMasterEnable();
    }
    else
    {
        if(ui32TrigLevel == 1)
        {
            ui32IoConfig |= IOC_IOPULL_DOWN;
        }
        else
        {
            ui32IoConfig |= IOC_IOPULL_UP;
        }
    }

    //
    // Write the configuration
    //
    GPIODirModeSet(ui32Pin, GPIO_DIR_MODE_IN);
    IOCPortConfigureSet(ui32IoId, IOC_PORT_GPIO, ui32IoConfig);

    //
    // Update the struct
    //
    g_tBackdoor.ui32Level = ui32TrigLevel;
    g_tBackdoor.ui32Pin = ui32Pin;
}


//*****************************************************************************
//
//! Query the state of the backdoor
//!
//! If the backdoor is open, this function will never return.
//!
//! \return None
//
//*****************************************************************************
void BackdoorOpen(void)
{
    //
    // Check backdoor
    //
    if((GPIOPinRead(g_tBackdoor.ui32Pin) ? 1 : 0) == g_tBackdoor.ui32Level)
    {
      halBootState = HAL_BOOT_COLD;
      while(1);
    }

    PRCMPeripheralRunDisable(PRCM_PERIPH_GPIO);
    PRCMPeripheralSleepDisable(PRCM_PERIPH_GPIO);
    PRCMPeripheralDeepSleepDisable(PRCM_PERIPH_GPIO);
    PRCMLoadSet();
    // turn off periperal domain
    HWREG( PRCM_BASE + PRCM_O_SWPWR_CTRL ) = 0;
}
#endif // BACKDOOR_TRAP
//*****************************************************************************
//
//! Perform the necessary trim of the device which is not done in boot code
//
//*****************************************************************************
void
trimDevice(void)
{
    uint32_t ui32Trim;
    uint32_t ui32Reg;
    uint32_t ui32ResetEvents;

    //
    // Check if trimming is required
    //
    if(HWREGB(ADI3_BASE + ADI_3_REFSYS_O_ADI3_SPARE3) != FINAL_TRIM_DONE)
    {
        //
        // Force AUX on and enable clocks
        //
        // No need to save the current status of the power/clock registers.
        // At this point both AUX and AON should have been reset to 0x0.
        //        
        HWREG(AON_WUC_BASE + AON_WUC_O_AUXEVENT) = AON_WUC_AUXEVENT_FORCEON;
        HWREG(AUX_WUC_BASE + AUX_WUC_O_MODCLKEN) = AUX_WUC_MODCLKEN_OSCCTRL;

        // --- DCDC 1A trim ---
        // Set VDDR to 1.7V
        //
        // -- Disable reset sources while trimming
        ui32ResetEvents = HWREG(AON_SYSCTRL12_BASE + AON_SYSCTRL12_O_RESET);
        HWREG(AON_SYSCTRL12_BASE + AON_SYSCTRL12_O_RESET) = 0x00;
        // Patric Cruise: Make this staircase this because  applying large step in
        // VDDR can cause system reset
        // TBD : The final value is device dependent!!!
        ADI8SetValBit(ADI3_BASE, ADI_3_REFSYS_O_DCDC_CTRL_1,
                      ADI_3_REFSYS_DCDC_CTRL_1_VDDR_TRIM_3P3V_M,0x01);
        ADI8SetValBit(ADI3_BASE, ADI_3_REFSYS_O_DCDC_CTRL_1,
                      ADI_3_REFSYS_DCDC_CTRL_1_VDDR_TRIM_3P3V_M,0x03);
        ADI8SetValBit(ADI3_BASE, ADI_3_REFSYS_O_DCDC_CTRL_1,
                      ADI_3_REFSYS_DCDC_CTRL_1_VDDR_TRIM_3P3V_M,0x05);
        // ----- Default trim sequence end, re-enable reset sources -----
        HWREG(AON_SYSCTRL12_BASE + AON_SYSCTRL12_O_RESET) = ui32ResetEvents;

        //
        // Following sequence is required for using XTAL, if not included
        // devices crashes when trying to switch to XTAL.
        //
        ui32Reg = HWREG(FCFG1_BASE + FACTORY_CFG_O_CONFIG_OSC_TOP);

        // Ryan Smith: If this register is zero the boot process should be halted!
        // TBD - is this a likely scenario???
        if(ui32Reg == 0)
        {
            while(1);
        }
        // Sets CAP SETTINGS
        DDI32RegWrite(AUX_OSCDDI_BASE, OSC_DIG_O_ANABYPASS_VALUE1, 0x000F0FFF);
        /*
        ui32Trim = (ui32Reg & FACTORY_CFG_CONFIG_OSC_TOP_XOSC_HF_CAP_TRIM_M) >>
            FACTORY_CFG_CONFIG_OSC_TOP_XOSC_HF_CAP_TRIM_S;
        DDI16BitfieldWrite(AUX_OSCDDI_BASE, OSC_DIG_O_ANABYPASS_VALUE1,
                           OSC_DIG_ANABYPASS_VALUE1_XOSC_HF_COLUMN_Q12_M,
                           OSC_DIG_ANABYPASS_VALUE1_XOSC_HF_COLUMN_Q12_S, ui32Trim);
        */
        // Sets default RCOSC_LF trim
        ui32Trim = (ui32Reg & FACTORY_CFG_CONFIG_OSC_TOP_RCOSCLF_CTUNE_TRIM_M) >>
            FACTORY_CFG_CONFIG_OSC_TOP_RCOSCLF_CTUNE_TRIM_S;
        DDI16BitfieldWrite(AUX_OSCDDI_BASE, OSC_DIG_O_XOSCLF_RCOSCLF_CTRL,
                           OSC_DIG_XOSCLF_RCOSCLF_CTRL_RCOSCLF_CTUNE_TRIM_M,
                           OSC_DIG_XOSCLF_RCOSCLF_CTRL_RCOSCLF_CTUNE_TRIM_S,
                           ui32Trim);
        // Sets XOSCHF IBIAS THERM
        DDI32RegWrite(AUX_OSCDDI_BASE, OSC_DIG_O_ANABYPASS_VALUE2, 0x000003FF);
        // Sets AMPCOMP settings for switching to XOSCHF
        DDI32RegWrite(AUX_OSCDDI_BASE, OSC_DIG_O_AMPCOMP_TH2, 0x68880000);
        // Sets AMPCOMP settings for switching to XOSCHF
        DDI32RegWrite(AUX_OSCDDI_BASE, OSC_DIG_O_AMPCOMP_TH1, 0x886876A4);
        // Sets AMPCOMP settings for switching to XOSCHF
        DDI32RegWrite(AUX_OSCDDI_BASE, OSC_DIG_O_AMPCOMP_CTRL, 0x00713F27);
        // TBD : What does this do???
        DDI32RegWrite(AUX_OSCDDI_BASE, OSC_DIG_O_RADC_EXTERNAL_CFG, 0x403F4000);

        // Disable clock for OSC_DIG and release power on AUX
        HWREG(AUX_WUC_BASE + AUX_WUC_O_MODCLKEN) = 0x0;
        HWREG(AON_WUC_BASE + AON_WUC_O_AUXEVENT) = 0x0;

        // Make sure to flag that trim has been performed
        HWREGB(ADI3_BASE + ADI_3_REFSYS_O_ADI3_SPARE3) = FINAL_TRIM_DONE;
    }

  // check if backdoor trap is being used
  BACKDOOR_CHECK();

}
