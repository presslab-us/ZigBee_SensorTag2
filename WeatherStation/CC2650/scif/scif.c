/// \addtogroup module_scif_driver_setup
//@{
#include "scif.h"
#include "scif_framework.h"
#include <inc/hw_types.h>
#include <inc/hw_memmap.h>
#include <inc/hw_aon_event.h>
#include <inc/hw_aon_rtc.h>
#include <inc/hw_aon_wuc.h>
#include <inc/hw_aux_sce.h>
#include <inc/hw_aux_smph.h>
#include <inc/hw_aux_evctl.h>
#include <inc/hw_aux_aiodio.h>
#include <inc/hw_aux_timer.h>
#include <inc/hw_aux_wuc.h>
#include <inc/hw_event.h>
#include <inc/hw_ints.h>
#include <inc/hw_ioc.h>
#include <string.h>
#if defined(__IAR_SYSTEMS_ICC__)
    #include <intrinsics.h>
#endif


// OSAL function prototypes
uint32_t scifOsalEnterCriticalSection();
void scifOsalLeaveCriticalSection(uint32_t key);




/// Firmware image to be uploaded to the AUX RAM
static const uint16_t pAuxRamImage[] = {
    /*0x0000*/ 0x1408, 0x040C, 0x1408, 0x042D, 0x1408, 0x0449, 0x045C, 0xFD47, 0x4436, 0x2437, 0xAEFE, 0xADB7, 0x0001, 0x8B42, 0x7000, 0x7C6C, 
    /*0x0020*/ 0x6874, 0x0069, 0x1426, 0x6875, 0x006A, 0x1426, 0x6876, 0x006B, 0x1426, 0x786C, 0xF801, 0xFA01, 0xBEF2, 0x7871, 0x6874, 0xFD0E, 
    /*0x0040*/ 0x6876, 0xED92, 0xFD06, 0x7C71, 0x642D, 0x045C, 0x786C, 0x8F1F, 0xED8F, 0xEC01, 0xBE01, 0xADB7, 0x8DB7, 0x6630, 0x0002, 0x8B42, 
    /*0x0060*/ 0x0000, 0x1871, 0x9D88, 0x9C01, 0xB60D, 0x1068, 0xAF19, 0xAA00, 0xB609, 0xA8FF, 0xAF39, 0xBE06, 0x0C6C, 0x886A, 0x8F08, 0xFD47, 
    /*0x0080*/ 0x9DB7, 0x086C, 0x8801, 0x8A01, 0xBEEC, 0x262F, 0xAEFE, 0x4630, 0x044E, 0x0004, 0x8B42, 0x86FF, 0x03FF, 0x0C70, 0x786D, 0xCD47, 
    /*0x00A0*/ 0x686E, 0xCD0E, 0x5870, 0xCD05, 0xB607, 0x0000, 0x0C6D, 0x7C72, 0x0C6E, 0x6C73, 0x652D, 0x0C70, 0x7870, 0xF801, 0xE92B, 0xFD0E, 
    /*0x00C0*/ 0xBE01, 0x6436, 0xBDB7, 0x241A, 0xA6FE, 0xADB7, 0x641A, 0xADB7, 0x0000, 0x007C, 0x007F, 0x00A6, 0x0000, 0x0000, 0x0000, 0x0000, 
    /*0x00E0*/ 0xFFFF, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x7001, 0x14A7, 0xADB7, 0x5006, 
    /*0x0100*/ 0x14AB, 0x8D47, 0x5005, 0x14AB, 0x8A00, 0xBE03, 0x0879, 0x8801, 0x0C79, 0xFA00, 0xBE06, 0x0879, 0x88FF, 0x0C79, 0x087A, 0x8801, 
    /*0x0120*/ 0x0C7A, 0x087B, 0x8801, 0x0C7B, 0x087B, 0x86FF, 0x8AFF, 0xBE0B, 0x0879, 0x0C77, 0x087A, 0x0C78, 0x0000, 0x0C7B, 0x0000, 0x0C79, 
    /*0x0140*/ 0x0000, 0x0C7A, 0x14B2, 0x7001, 0x14A7, 0xADB7, 0xADB7, 0x686C, 0xE868, 0xFF2E, 0xADB7, 0xE913, 0xEDA0, 0xF912, 0xFD0E, 0xFD8D, 
    /*0x0160*/ 0xF001, 0xADB7, 0x786C, 0x6001, 0xED87, 0x786D, 0xFD0E, 0x7C6D, 0xADB7
};


/// Look-up table that converts from AUX I/O index to MCU IOCFG offset
static const uint8_t pAuxIoIndexToMcuIocfgOffsetLut[] = {
    120, 116, 112, 108, 104, 100, 96, 92, 28, 24, 20, 16, 12, 8, 4, 0
};


/** \brief Look-up table of data structure information for each task
  *
  * There is one entry per data structure (\c cfg, \c input, \c output and \c state) per task:
  * - [31:24] Data structure size (number of 16-bit words)
  * - [23:16] Buffer count (when 2+, first data structure is preceded by buffering control variables)
  * - [15:0] Address of the first data structure
  */
static const uint32_t pScifTaskDataStructInfoLut[] = {
//  cfg         input       output      state       
    0x00000000, 0x00000000, 0x020100EE, 0x030100F2  // Dust Sensor
};




// No task-specific initialization functions




// No task-specific uninitialization functions




/** \brief Initilializes task resource hardware dependencies
  *
  * This function is called by the internal driver initialization function, \ref scifInit().
  */
static void scifTaskResourceInit(void) {
    scifInitIo(6, AUXIOMODE_INPUT, -1, 0);
    scifInitIo(5, AUXIOMODE_INPUT, -1, 0);
} // scifTaskResourceInit




/** \brief Uninitilializes task resource hardware dependencies
  *
  * This function is called by the internal driver uninitialization function, \ref scifUninit().
  */
static void scifTaskResourceUninit(void) {
    scifUninitIo(6, -1);
    scifUninitIo(5, -1);
} // scifTaskResourceUninit




/// Driver setup data, to be used in the call to \ref scifInit()
const SCIF_DATA_T scifDriverSetup = {
    (volatile SCIF_TASK_CTRL_T*) 0x400E00E2,
    (volatile uint16_t*) 0x400E00D0,
    0x0000,
    sizeof(pAuxRamImage),
    pAuxRamImage,
    pScifTaskDataStructInfoLut,
    pAuxIoIndexToMcuIocfgOffsetLut,
    scifTaskResourceInit,
    scifTaskResourceUninit
};




/** \brief Starts or modifies RTC-based task scheduling tick generation
  *
  * RTC-based tick generation will wake up the Sensor Controller first at the specified value of the RTC
  * and then periodically at the specified interval. The application must call this function after
  * calling \ref scifInit().
  *
  * The application must ensure that:
  * - \a tickStart is not in the past (prefer using \ref scifStartRtcTicksNow() to avoid this)
  * - \a tickPeriod is not too short for the Sensor Controller to complete its tasks within a single tick
  *   interval
  *
  * \param[in]      tickStart
  *     RTC value when the first tick is generated:
  *     - Bits 31:16 = seconds
  *     - Bits 15:0 = 1/65536 of a second
  * \param[in]      tickPeriod
  *     Interval at which subsequent ticks are generated:
  *     - Bits 31:16 = seconds
  *     - Bits 15:0 = 1/65536 of a second
  */
void scifStartRtcTicks(uint32_t tickStart, uint32_t tickPeriod) {
    HWREG(AON_RTC_BASE + AON_RTC_O_CH2CMP) = tickStart;
    HWREG(AON_RTC_BASE + AON_RTC_O_CH2CMPINC) = tickPeriod;
    HWREG(AON_RTC_BASE + AON_RTC_O_CHCTL) |= AON_RTC_CHCTL_CH2_EN_M | AON_RTC_CHCTL_CH2_CONT_EN_M;
    HWREG(AON_EVENT_BASE + AON_EVENT_O_AUXWUSEL) =
        (HWREG(AON_EVENT_BASE + AON_EVENT_O_AUXWUSEL) & ~AON_EVENT_AUXWUSEL_WU0_EV_M) |
        (AON_EVENT_AUXWUSEL_WU0_EV_RTC_CH2);
} // scifStartRtcTicks




/** \brief Starts or modifies RTC-based task scheduling tick generation
  *
  * RTC-based tick generation will wake up the Sensor Controller first after approximately 128 ms and
  * then periodically at the specified interval. The application must call this function after calling
  * \ref scifInit().
  *
  * The application must ensure that \a tickPeriod is not too short for the Sensor Controller to complete
  * its tasks within a single tick interval.
  *
  * \param[in]      tickPeriod
  *     Interval at which subsequent ticks are generated:
  *     - Bits 31:16 = seconds
  *     - Bits 15:0 = 1/65536 of a second
  */
void scifStartRtcTicksNow(uint32_t tickPeriod) {
    uint32_t key, sec, subsec;
    key = scifOsalEnterCriticalSection();
    sec = HWREG(AON_RTC_BASE + AON_RTC_O_SEC);
    subsec = HWREG(AON_RTC_BASE + AON_RTC_O_SUBSEC);
    scifStartRtcTicks(((sec << 16) | (subsec >> 16)) + 8, tickPeriod);
    scifOsalLeaveCriticalSection(key);
} // scifStartRtcTicksNow




/** \brief Stops RTC-based task scheduling tick generation
  *
  * The application must call this function before calling \ref scifUninit().
  */
void scifStopRtcTicks(void) {
    HWREG(AON_RTC_BASE + AON_RTC_O_CHCTL) &= ~(AON_RTC_CHCTL_CH2_EN_M | AON_RTC_CHCTL_CH2_CONT_EN_M);
    HWREG(AON_EVENT_BASE + AON_EVENT_O_AUXWUSEL) =
        (HWREG(AON_EVENT_BASE + AON_EVENT_O_AUXWUSEL) & ~AON_EVENT_AUXWUSEL_WU0_EV_M) |
        (AON_EVENT_AUXWUSEL_WU0_EV_NONE);
    HWREG(AON_RTC_BASE + AON_RTC_O_SYNC);
} // scifStopRtcTicks


//@}
