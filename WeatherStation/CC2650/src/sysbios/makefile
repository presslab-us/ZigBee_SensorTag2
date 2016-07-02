
MAKEFLAGS = -j16

RTSLIB = -iC:/PROGRA~2/IARSYS~1/EMBEDD~1.2/arm/lib/ -llibc.a

XDCLIB = -l "z:/ti/xdctools_3_30_06_67_core/packages/iar/targets/arm/rts/lib/release/boot.arm3" 

XOPTS = -I"Z:/ti/xdctools_3_30_06_67_core/packages/" -I"z:/ti/xdctools_3_30_06_67_core/packages/" -Dxdc_target_types__=Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/iar/targets/arm/std.h -Dxdc_target_name__=M3

vpath % Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ti/sysbios/
vpath %.c z:/ti/xdctools_3_30_06_67_core/packages/

CCOPTS    = --aeabi --cpu=Cortex-M3 --diag_suppress=Pa050,Go005 --endian=little -e --thumb --silent  --mfc -Ohs  -Dti_sysbios_family_arm_cc26xx_Boot_driverlibVersion=2 -Dti_sysbios_family_arm_m3_Hwi_enableException__D=TRUE -Dti_sysbios_family_arm_m3_Hwi_disablePriority__D=32U -Dti_sysbios_family_arm_m3_Hwi_numSparseInterrupts__D=0U  --dlib_config C:/PROGRA~2/IARSYS~1/EMBEDD~1.2/arm/inc/c/DLib_Config_Normal.h

XDC_ROOT  = z:/ti/xdctools_3_30_06_67_core/packages/

BIOS_ROOT = Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ti/sysbios/

BIOS_DEFS =  -Dti_sysbios_BIOS_swiEnabled__D=TRUE -Dti_sysbios_BIOS_taskEnabled__D=TRUE -Dti_sysbios_BIOS_clockEnabled__D=TRUE -Dti_sysbios_BIOS_runtimeCreatesEnabled__D=TRUE -Dti_sysbios_hal_Hwi_DISABLE_ALL_HOOKS -Dti_sysbios_knl_Swi_DISABLE_ALL_HOOKS -Dti_sysbios_knl_Task_DISABLE_ALL_HOOKS -Dti_sysbios_BIOS_smpEnabled__D=FALSE -Dti_sysbios_Build_useHwiMacros -Dti_sysbios_knl_Swi_numPriorities__D=6 -Dti_sysbios_knl_Task_deleteTerminatedTasks__D=FALSE -Dti_sysbios_knl_Task_numPriorities__D=6 -Dti_sysbios_knl_Task_checkStackFlag__D=FALSE -Dti_sysbios_knl_Clock_TICK_SOURCE=ti_sysbios_knl_Clock_TickSource_TIMER -Dti_sysbios_knl_Clock_TICK_MODE=ti_sysbios_knl_Clock_TickMode_DYNAMIC -Dti_sysbios_hal_Core_delegate_getId=ti_sysbios_hal_CoreNull_getId__E -Dti_sysbios_hal_Core_delegate_interruptCore=ti_sysbios_hal_CoreNull_interruptCore__E -Dti_sysbios_hal_Core_delegate_lock=ti_sysbios_hal_CoreNull_lock__E -Dti_sysbios_hal_Core_delegate_unlock=ti_sysbios_hal_CoreNull_unlock__E -Dti_sysbios_hal_Core_numCores__D=1 -Dti_sysbios_hal_CoreNull_numCores__D=1 -Dti_sysbios_utils_Load_taskEnabled__D=TRUE -Dti_sysbios_utils_Load_swiEnabled__D=FALSE -Dti_sysbios_family_arm_m3_Hwi_dispatcherSwiSupport__D=TRUE -Dti_sysbios_family_arm_m3_Hwi_dispatcherTaskSupport__D=TRUE -Dti_sysbios_family_arm_m3_Hwi_dispatcherAutoNestingSupport__D=TRUE -Dti_sysbios_family_arm_m3_Hwi_dispatcherIrpTrackingSupport__D=TRUE -Dti_sysbios_knl_Semaphore_supportsEvents__D=FALSE -Dti_sysbios_knl_Semaphore_supportsPriority__D=FALSE -Dxdc_runtime_Assert_DISABLE_ALL -Dxdc_runtime_Log_DISABLE_ALL

INTERN_FUNCS = memcpy|memset|ti_sysbios_rom_cortexm_cc26xx_CC26xx_getRevision__E|ti_sysbios_knl_Queue_get__E|ti_sysbios_knl_Swi_enabled__E|ti_sysbios_knl_Clock_scheduleNextTick__E|ti_sysbios_knl_Swi_runLoop__I|ti_sysbios_knl_Clock_getTicks__E|ti_sysbios_gates_GateMutex_Object__destruct__S|ti_sysbios_knl_Queue_enqueue__E|ti_sysbios_knl_Queue_put__E|ti_sysbios_family_arm_m3_Hwi_Object__create__S|ti_sysbios_gates_GateHwi_Instance_init__E|ti_sysbios_hal_Hwi_Instance_finalize__E|ti_sysbios_BIOS_RtsGateProxy_leave__E|ti_sysbios_heaps_HeapMem_Object__create__S|xdc_runtime_Error_raiseX__E|ti_sysbios_knl_Semaphore_construct|ti_sysbios_knl_Clock_Object__destruct__S|ti_sysbios_knl_Clock_TimerProxy_getMaxTicks__E|ti_sysbios_knl_Swi_Object__destruct__S|ti_sysbios_family_arm_cc26xx_TimestampProvider_getFreq__E|ti_sysbios_gates_GateMutex_Handle__label__S|ti_sysbios_knl_Mailbox_delete|ti_sysbios_knl_Semaphore_destruct|ti_sysbios_BIOS_RtsGateProxy_enter__E|ti_sysbios_knl_Task_processVitalTaskFlag__I|ti_sysbios_knl_Mailbox_create|xdc_runtime_Core_deleteObject__I|ti_sysbios_knl_Queue_delete|ti_sysbios_family_arm_m3_Hwi_doSwiRestore__I|xdc_runtime_System_atexit__E|ti_sysbios_gates_GateMutex_Params__init__S|ti_sysbios_knl_Clock_getTimerHandle__E|ti_sysbios_knl_Task_enable__E|ti_sysbios_knl_Clock_TimerProxy_getExpiredTicks__E|ti_sysbios_knl_Queue_Object__destruct__S|ti_sysbios_knl_Clock_Object__delete__S|ti_sysbios_gates_GateMutex_delete|ti_sysbios_heaps_HeapMem_restore__E|ti_sysbios_knl_Swi_create|ti_sysbios_heaps_HeapMem_Module_GateProxy_leave__E|ti_sysbios_knl_Semaphore_pend__E|ti_sysbios_knl_Mailbox_Instance_finalize__E|xdc_runtime_Startup_startMods__I|ti_sysbios_heaps_HeapMem_init__I|ti_sysbios_knl_Swi_Object__delete__S|ti_sysbios_hal_Hwi_HwiProxy_enableInterrupt__E|ti_sysbios_knl_Clock_removeI__E|xdc_runtime_System_abort__E|ti_sysbios_family_arm_m3_Hwi_dispatchC__I|ti_sysbios_knl_Swi_construct|ti_sysbios_knl_Task_sleepTimeout__I|ti_sysbios_knl_Queue_remove__E|ti_sysbios_knl_Semaphore_Instance_finalize__E|ti_sysbios_gates_GateMutex_destruct|ti_sysbios_knl_Task_SupportProxy_Module__startupDone__S|ti_sysbios_knl_Queue_Object__delete__S|ti_sysbios_knl_Mailbox_Object__get__S|ti_sysbios_family_arm_m3_Hwi_Instance_init__E|ti_sysbios_knl_Clock_delete|ti_sysbios_knl_Clock_walkQueueDynamic__E|ti_sysbios_knl_Mailbox_Object__destruct__S|ti_sysbios_knl_Mailbox_post__E|ti_sysbios_knl_Clock_Instance_init__E|ti_sysbios_knl_Task_allBlockedFunction__I|ti_sysbios_knl_Task_postInit__I|ti_sysbios_knl_Task_enter__I|ti_sysbios_hal_Hwi_switchFromBootStack__E|ti_sysbios_knl_Semaphore_Object__destruct__S|ti_sysbios_hal_Hwi_HwiProxy_Object__create__S|ti_sysbios_family_arm_cc26xx_Timer_postInit__I|ti_sysbios_knl_Swi_Module_startup__E|ti_sysbios_gates_GateMutex_Instance_finalize__E|xdc_runtime_Core_assignParams__I|ti_sysbios_hal_Hwi_HwiProxy_switchFromBootStack__E|ti_sysbios_knl_Swi_post__E|ti_sysbios_hal_Hwi_initStack|xdc_runtime_Memory_alloc__E|ti_sysbios_knl_Queue_next__E|ti_sysbios_knl_Clock_Instance_finalize__E|ti_sysbios_knl_Queue_elemClear__E|ti_sysbios_knl_Clock_Params__init__S|ti_sysbios_knl_Task_Instance_init__E|ti_sysbios_hal_Hwi_HwiProxy_startup__E|ti_sysbios_knl_Task_self__E|ti_sysbios_knl_Task_startup__E|ti_sysbios_gates_GateHwi_Object__delete__S|ti_sysbios_family_arm_cc26xx_Timer_startup__E|xdc_runtime_Memory_free__E|ti_sysbios_hal_Hwi_delete|ti_sysbios_knl_Queue_Instance_init__E|ti_sysbios_family_arm_cc26xx_Timer_Module_startup__E|xdc_runtime_Assert_raise__I|ti_sysbios_hal_Hwi_create|ti_sysbios_knl_Task_destruct|ti_sysbios_hal_Hwi_Module_startup__E|ti_sysbios_family_arm_m3_Hwi_excHandler__I|xdc_runtime_Core_destructObject__I|ti_sysbios_knl_Swi_disable__E|ti_sysbios_BIOS_setThreadType__E|ti_sysbios_knl_Task_disable__E|ti_sysbios_knl_Swi_Instance_init__E|ti_sysbios_knl_Semaphore_pendTimeout__I|ti_sysbios_knl_Clock_create|ti_sysbios_knl_Idle_loop__E|ti_sysbios_gates_GateHwi_leave__E|ti_sysbios_family_arm_m3_Hwi_enableInterrupt__E|ti_sysbios_knl_Semaphore_Params__init__S|ti_sysbios_knl_Task_unblock__E|ti_sysbios_knl_Swi_destruct|ti_sysbios_BIOS_getCpuFreq__E|xdc_runtime_Memory_calloc__E|ti_sysbios_family_arm_m3_Hwi_startup__E|xdc_runtime_SysCallback_exit__E|ti_sysbios_knl_Queue_empty__E|ti_sysbios_knl_Clock_logTick__E|ti_sysbios_knl_Task_yield__E|ti_sysbios_knl_Task_SupportProxy_getStackAlignment__E|ti_sysbios_family_arm_m3_Hwi_create|xdc_runtime_Timestamp_SupportProxy_get32__E|ti_sysbios_family_arm_m3_Hwi_destruct|ti_sysbios_knl_Clock_stop__E|ti_sysbios_family_arm_m3_Hwi_doTaskRestore__I|ti_sysbios_knl_Swi_run__I|ti_sysbios_family_arm_cc26xx_Timer_Module__startupDone__S|xdc_runtime_Core_createObject__I|ti_sysbios_knl_Queue_create|ti_sysbios_hal_Hwi_Object__delete__S|ti_sysbios_knl_Clock_construct|xdc_runtime_System_abortSpin__E|ti_sysbios_family_arm_m3_Hwi_Object__destruct__S|ti_sysbios_hal_Hwi_HwiProxy_delete|ti_sysbios_gates_GateMutex_Object__create__S|ti_sysbios_family_arm_m3_Hwi_getStackInfo__E|ti_sysbios_heaps_HeapMem_Module_GateProxy_enter__E|ti_sysbios_knl_Semaphore_post__E|ti_sysbios_knl_Task_exit__E|ti_sysbios_heaps_HeapMem_Instance_init__E|ti_sysbios_knl_Swi_restore__E|ti_sysbios_knl_Task_startCore__E|ti_sysbios_knl_Semaphore_create|ti_sysbios_gates_GateHwi_enter__E|ti_sysbios_knl_Task_blockI__E|ti_sysbios_heaps_HeapMem_free__E|ti_sysbios_knl_Task_Object__destruct__S|ti_sysbios_family_arm_cc26xx_Timer_periodicStub__E|ti_sysbios_hal_Hwi_Instance_init__E|ti_sysbios_gates_GateHwi_query__E|xdc_runtime_System_processAtExit__E|ti_sysbios_family_arm_cc26xx_Timer_setPeriod__E|xdc_runtime_Error_init__E|ti_sysbios_knl_Semaphore_Instance_init__E|ti_sysbios_knl_Queue_head__E|xdc_runtime_Error_check__E|xdc_runtime_Error_policySpin__E|ti_sysbios_gates_GateMutex_create|xdc_runtime_Gate_leaveSystem__E|ti_sysbios_knl_Swi_restoreHwi__E|ti_sysbios_knl_Task_sleep__E|ti_sysbios_knl_Task_create|ti_sysbios_knl_Mailbox_Params__init__S|ti_sysbios_knl_Task_restoreHwi__E|ti_sysbios_knl_Mailbox_postInit__I|ti_sysbios_knl_Task_delete|ti_sysbios_heaps_HeapMem_isBlocking__E|ti_sysbios_knl_Clock_startI__E|ti_sysbios_knl_Clock_start__E|ti_sysbios_family_arm_m3_Hwi_Object__delete__S|ti_sysbios_knl_Clock_TimerProxy_getPeriod__E|ti_sysbios_knl_Task_SupportProxy_start__E|ti_sysbios_heaps_HeapMem_Handle__label__S|ti_sysbios_family_arm_m3_Hwi_delete|ti_sysbios_knl_Semaphore_Object__delete__S|ti_sysbios_hal_Hwi_HwiProxy_getStackInfo__E|ti_sysbios_knl_Idle_run__E|ti_sysbios_knl_Swi_delete|xdc_runtime_Memory_valloc__E|ti_sysbios_knl_Mailbox_Object__delete__S|ti_sysbios_family_arm_m3_TaskSupport_start__E|ti_sysbios_family_arm_m3_Hwi_Module__startupDone__S|ti_sysbios_knl_Swi_startup__E|ti_sysbios_knl_Task_schedule__I|ti_sysbios_gates_GateMutex_leave__E|ti_sysbios_heaps_HeapMem_Object__delete__S|ti_sysbios_knl_Clock_TimerProxy_setNextTick__E|ti_sysbios_knl_Swi_Object__get__S|ti_sysbios_knl_Task_restore__E|xdc_runtime_Memory_HeapProxy_alloc__E|ti_sysbios_gates_GateHwi_Object__create__S|ti_sysbios_hal_Hwi_HwiProxy_disableInterrupt__E|ti_sysbios_BIOS_start__E|ti_sysbios_BIOS_exit__E|ti_sysbios_family_arm_m3_TaskSupport_getStackAlignment__E|xdc_runtime_SysCallback_abort__E|ti_sysbios_knl_Queue_destruct|ti_sysbios_family_arm_m3_Hwi_postInit__I|ti_sysbios_gates_GateMutex_Instance_init__E|ti_sysbios_knl_Task_Instance_finalize__E|ti_sysbios_knl_Clock_TimerProxy_getCurrentTick__E|ti_sysbios_family_arm_m3_Hwi_disableFxn__E|xdc_runtime_Memory_HeapProxy_free__E|ti_sysbios_knl_Mailbox_Module_startup__E|ti_sysbios_knl_Task_Object__delete__S|ti_sysbios_gates_GateHwi_Handle__label__S|xdc_runtime_Text_ropeText__E|ti_sysbios_knl_Clock_destruct|ti_sysbios_knl_Queue_construct|ti_sysbios_family_arm_m3_Hwi_switchFromBootStack__E|ti_sysbios_heaps_HeapMem_Object__get__S|ti_sysbios_hal_Hwi_HwiProxy_create|ti_sysbios_gates_GateMutex_query__E|ti_sysbios_knl_Swi_schedule__I|ti_sysbios_knl_Task_Params__init__S|ti_sysbios_family_arm_m3_Hwi_Params__init__S|ti_sysbios_family_arm_m3_Hwi_plug__E|xdc_runtime_System_exitSpin__E|ti_sysbios_gates_GateMutex_construct|xdc_runtime_System_Module_GateProxy_leave__E|ti_sysbios_knl_Mailbox_pend__E|ti_sysbios_family_arm_m3_TaskSupport_Module__startupDone__S|xdc_runtime_Core_assignLabel__I|xdc_runtime_System_Module_GateProxy_enter__E|xdc_runtime_System_exit__E|ti_sysbios_knl_Swi_Params__init__S|ti_sysbios_knl_Clock_workFunc__E|ti_sysbios_family_arm_m3_Hwi_restoreFxn__E|ti_sysbios_family_arm_cc26xx_TimestampProvider_Module_startup__E|ti_sysbios_knl_Semaphore_delete|ti_sysbios_family_arm_cc26xx_Timer_getPeriod__E|ti_sysbios_family_arm_m3_Hwi_initNVIC__E|ti_sysbios_knl_Clock_addI__E|ti_sysbios_family_arm_m3_Hwi_Instance_finalize__E|ti_sysbios_heaps_HeapMem_alloc__E|ti_sysbios_knl_Task_unblockI__E|ti_sysbios_knl_Swi_Instance_finalize__E|ti_sysbios_family_arm_m3_Hwi_disableInterrupt__E|ti_sysbios_family_arm_m3_Hwi_enableFxn__E|xdc_runtime_Gate_enterSystem__E|ti_sysbios_gates_GateMutex_Object__delete__S|ti_sysbios_family_arm_cc26xx_TimestampProvider_get64__E|ti_sysbios_knl_Mailbox_Instance_init__E|xdc_runtime_Text_cordText__E|xdc_runtime_Startup_exec__E|ti_sysbios_hal_Hwi_HwiProxy_Module__startupDone__S|ti_sysbios_heaps_HeapMem_getStats__E|ti_sysbios_knl_Task_SupportProxy_swap__E|xdc_runtime_Memory_getMaxDefaultTypeAlign__E|ti_sysbios_knl_Task_Object__get__S|ti_sysbios_family_arm_m3_Hwi_construct|ti_sysbios_knl_Clock_TimerProxy_Module__startupDone__S|ti_sysbios_knl_Clock_Module_startup__E|ti_sysbios_knl_Mailbox_construct|ti_sysbios_knl_Task_construct|xdc_runtime_Core_constructObject__I|ti_sysbios_knl_Queue_dequeue__E|ti_sysbios_knl_Task_Module_startup__E|ti_sysbios_family_arm_cc26xx_Timer_getExpiredTicks__E|ti_sysbios_family_arm_m3_Hwi_Object__get__S|ti_sysbios_knl_Mailbox_destruct|xdc_runtime_System_Module_startup__E|ti_sysbios_knl_Swi_postInit__I|ti_sysbios_family_arm_m3_Hwi_Module_startup__E|ti_sysbios_gates_GateMutex_enter__E|ti_sysbios_family_arm_m3_Hwi_setPriority__E|ti_sysbios_knl_Queue_Object__get__S|ti_sysbios_knl_Clock_setTimeout__E|ti_sysbios_knl_Task_swapReturn|ti_sysbios_family_arm_m3_TaskSupport_glue|ti_sysbios_family_arm_m3_TaskSupport_buildTaskStack|ti_sysbios_family_arm_m3_TaskSupport_swap__E|ti_sysbios_family_arm_m3_Hwi_excHandlerAsm__I|ti_sysbios_family_arm_m3_Hwi_return|ti_sysbios_family_arm_m3_Hwi_pendSV__I|ti_sysbios_family_arm_m3_Hwi_dispatch__I|ti_sysbios_family_xxx_Hwi_switchAndRunFunc|ti_sysbios_family_arm_m3_Hwi_initStacks__E|ti_sysbios_BIOS_RtsGateProxy_Object__delete__S|ti_sysbios_BIOS_RtsGateProxy_Params__init__S|ti_sysbios_BIOS_RtsGateProxy_Handle__label__S|ti_sysbios_BIOS_RtsGateProxy_query__E|ti_sysbios_knl_Clock_TimerProxy_startup__E|ti_sysbios_hal_Hwi_disableInterrupt__E|ti_sysbios_hal_Hwi_enableInterrupt__E|ti_sysbios_hal_Hwi_getStackInfo__E|ti_sysbios_hal_Hwi_startup__E|ti_sysbios_hal_Hwi_HwiProxy_Object__delete__S|ti_sysbios_hal_Hwi_HwiProxy_Params__init__S|ti_sysbios_heaps_HeapMem_Module_GateProxy_query__E|ti_sysbios_heaps_HeapMem_Module_GateProxy_Object__delete__S|ti_sysbios_heaps_HeapMem_Module_GateProxy_Params__init__S|ti_sysbios_heaps_HeapMem_Module_GateProxy_Handle__label__S|xdc_runtime_Timestamp_SupportProxy_get64__E|xdc_runtime_Timestamp_SupportProxy_getFreq__E|xdc_runtime_Timestamp_get32__E|xdc_runtime_Timestamp_get64__E|xdc_runtime_Timestamp_getFreq__E|xdc_runtime_Memory_HeapProxy_Object__delete__S|xdc_runtime_Memory_HeapProxy_Handle__label__S|xdc_runtime_System_Module_GateProxy_Object__delete__S|xdc_runtime_System_Module_GateProxy_Handle__label__S|xdc_runtime_System_Module_GateProxy_query__E

BIOS_INC  = -I"Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/"

TARGET_INC = -I"Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/"

INCS = $(BIOS_INC) $(TARGET_INC)   -IZ:/ti/simplelink/zstack_home_1_02_02a_44539/Projects/zstack/HomeAutomation/WeatherStation/CC2650/   -IZ:/ti/simplelink/zstack_home_1_02_02a_44539/Projects/zstack/HomeAutomation/WeatherStation/CC2650/scif/   -IZ:/ti/simplelink/zstack_home_1_02_02a_44539/Projects/zstack/HomeAutomation/WeatherStation/CC2650/../../Source/   -IZ:/ti/simplelink/zstack_home_1_02_02a_44539/Projects/zstack/HomeAutomation/WeatherStation/CC2650/../../../common/CC26xx/   -IZ:/ti/simplelink/zstack_home_1_02_02a_44539/Projects/zstack/HomeAutomation/WeatherStation/CC2650/../../../common/CC26xx/appport/   -IZ:/ti/simplelink/zstack_home_1_02_02a_44539/Projects/zstack/HomeAutomation/WeatherStation/CC2650/../../../common/CC26xx/SensorTag/   -IZ:/ti/simplelink/zstack_home_1_02_02a_44539/Projects/zstack/HomeAutomation/WeatherStation/CC2650/../../../../../Components/applib/heap/   -IZ:/ti/simplelink/zstack_home_1_02_02a_44539/Projects/zstack/HomeAutomation/WeatherStation/CC2650/../../../../../Components/mac/include/cc26xx/   -IZ:/ti/simplelink/zstack_home_1_02_02a_44539/Projects/zstack/HomeAutomation/WeatherStation/CC2650/../../../../../Components/hal/include/   -IZ:/ti/simplelink/zstack_home_1_02_02a_44539/Projects/zstack/HomeAutomation/WeatherStation/CC2650/../../../../../Components/hal/target/_common/cc26xx/   -IZ:/ti/simplelink/zstack_home_1_02_02a_44539/Projects/zstack/HomeAutomation/WeatherStation/CC2650/../../../../../Components/hal/target/CC2650/rom/   -IZ:/ti/simplelink/zstack_home_1_02_02a_44539/Projects/zstack/HomeAutomation/WeatherStation/CC2650/../../../../../Components/hal/target/CC2650TIRTOS/   -IZ:/ti/simplelink/zstack_home_1_02_02a_44539/Projects/zstack/HomeAutomation/WeatherStation/CC2650/../../../../../Components/icall/include/   -IZ:/ti/simplelink/zstack_home_1_02_02a_44539/Projects/zstack/HomeAutomation/WeatherStation/CC2650/../../../../../Components/osal/include/   -IZ:/ti/simplelink/zstack_home_1_02_02a_44539/Projects/zstack/HomeAutomation/WeatherStation/CC2650/../../../../../Components/services/nv/   -IZ:/ti/simplelink/zstack_home_1_02_02a_44539/Projects/zstack/HomeAutomation/WeatherStation/CC2650/../../../../../Components/services/nv/cc26xx/   -IZ:/ti/simplelink/zstack_home_1_02_02a_44539/Projects/zstack/HomeAutomation/WeatherStation/CC2650/../../../../../Components/services/saddr/   -IZ:/ti/simplelink/zstack_home_1_02_02a_44539/Projects/zstack/HomeAutomation/WeatherStation/CC2650/../../../../../Components/stack/icall/   -IZ:/ti/simplelink/zstack_home_1_02_02a_44539/Projects/zstack/HomeAutomation/WeatherStation/CC2650/../../../../../Components/stack/icall/client/   -IZ:/ti/simplelink/zstack_home_1_02_02a_44539/Projects/zstack/HomeAutomation/WeatherStation/CC2650/../../../../../Components/stack/zcl/   -IZ:/ti/tirtos_simplelink_2_11_01_09/products/cc26xxware_2_00_06_14829//   -IZ:/ti/tirtos_simplelink_2_11_01_09/products/cc26xxware_2_00_06_14829/inc/   -IZ:/ti/tirtos_simplelink_2_11_01_09/products/cc26xxware_2_00_06_14829/driverlib/   -IZ:/ti/tirtos_simplelink_2_11_01_09/products/cc26xxware_2_00_06_14829/../../packages/ti/boards/SensorTag/CC26XXST_0120/   -IZ:/ti/tirtos_simplelink_2_11_01_09/products/cc26xxware_2_00_06_14829/../../packages/ti/boards/SensorTag/Devices/   -IZ:/ti/tirtos_simplelink_2_11_01_09/products/cc26xxware_2_00_06_14829/../../packages/ti/boards/SensorTag/Interfaces/   -IZ:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ 

CC = LC_ALL=C  C:/PROGRA~2/IARSYS~1/EMBEDD~1.2/arm/bin/iccarm $(CCOPTS)
ASM = LC_ALL=C  C:/PROGRA~2/IARSYS~1/EMBEDD~1.2/arm/bin/iasmarm --cpu Cortex-M3 --endian little -S
AR = LC_ALL=C  C:/PROGRA~2/IARSYS~1/EMBEDD~1.2/arm/bin/iarchive 

CC_LIB = C:/PROGRA~2/IARSYS~1/EMBEDD~1.2/arm/bin/iccarm --aeabi --cpu=Cortex-M3 --diag_suppress=Pa050,Go005 --endian=little -e --thumb --silent  --mfc -Ohs  -Dti_sysbios_family_arm_cc26xx_Boot_driverlibVersion=2 -Dti_sysbios_family_arm_m3_Hwi_enableException__D=TRUE -Dti_sysbios_family_arm_m3_Hwi_disablePriority__D=32U -Dti_sysbios_family_arm_m3_Hwi_numSparseInterrupts__D=0U -I C:/PROGRA~2/IARSYS~1/EMBEDD~1.2/arm/include

CC_PP = C:/PROGRA~2/IARSYS~1/EMBEDD~1.2/arm/bin/iccarm --aeabi --cpu=Cortex-M3 --diag_suppress=Pa050,Go005 --endian=little -e --thumb --silent  --mfc   -Dti_sysbios_family_arm_cc26xx_Boot_driverlibVersion=2 -Dti_sysbios_family_arm_m3_Hwi_enableException__D=TRUE  -Dti_sysbios_family_arm_m3_Hwi_numSparseInterrupts__D=0U -I C:/PROGRA~2/IARSYS~1/EMBEDD~1.2/arm/include  --dlib_config C:/PROGRA~2/IARSYS~1/EMBEDD~1.2/arm/inc/c/DLib_Config_Normal.h

DEL = cmd.exe /q /c del /f
CP = cmd.exe /q /c copy /Y

define RM
	$(if $(wildcard $1),$(DEL) $1,:)
endef

define ASSEMBLE
	@echo asmrm3 $< ...
	@$(ASM) $(BIOS_DEFS) $(XOPTS) $(INCS) $<
endef

all: sysbios.arm3

PHONY: intro
intro:
	@echo Preprocessing library source files ...

arm_m3_Hwi_asm_iar_lib.obj: family/arm/m3/Hwi_asm_iar.sv7M
	@$(XDC_ROOT)/../xs -c Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleAsmFile.xs $(BIOS_ROOT)family/arm/m3/Hwi_asm_iar.sv7M arm_m3_Hwi_asm_iar_lib.asm romInternFuncs.txt
	@$(ASM) -o arm_m3_Hwi_asm_iar_lib.obj -I$(BIOS_ROOT)family/arm/m3 $(BIOS_DEFS) $(XOPTS) $(INCS) arm_m3_Hwi_asm_iar_lib.asm

arm_m3_Hwi_asm_switch_iar_lib.obj: family/arm/m3/Hwi_asm_switch_iar.sv7M
	@$(XDC_ROOT)/../xs -c Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleAsmFile.xs $(BIOS_ROOT)family/arm/m3/Hwi_asm_switch_iar.sv7M arm_m3_Hwi_asm_switch_iar_lib.asm romInternFuncs.txt
	@$(ASM) -o arm_m3_Hwi_asm_switch_iar_lib.obj -I$(BIOS_ROOT)family/arm/m3 $(BIOS_DEFS) $(XOPTS) $(INCS) arm_m3_Hwi_asm_switch_iar_lib.asm

arm_m3_IntrinsicsSupport_asm_iar_lib.obj: family/arm/m3/IntrinsicsSupport_asm_iar.sv7M
	@$(XDC_ROOT)/../xs -c Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleAsmFile.xs $(BIOS_ROOT)family/arm/m3/IntrinsicsSupport_asm_iar.sv7M arm_m3_IntrinsicsSupport_asm_iar_lib.asm romInternFuncs.txt
	@$(ASM) -o arm_m3_IntrinsicsSupport_asm_iar_lib.obj -I$(BIOS_ROOT)family/arm/m3 $(BIOS_DEFS) $(XOPTS) $(INCS) arm_m3_IntrinsicsSupport_asm_iar_lib.asm

arm_m3_TaskSupport_asm_iar_lib.obj: family/arm/m3/TaskSupport_asm_iar.sv7M
	@$(XDC_ROOT)/../xs -c Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleAsmFile.xs $(BIOS_ROOT)family/arm/m3/TaskSupport_asm_iar.sv7M arm_m3_TaskSupport_asm_iar_lib.asm romInternFuncs.txt
	@$(ASM) -o arm_m3_TaskSupport_asm_iar_lib.obj -I$(BIOS_ROOT)family/arm/m3 $(BIOS_DEFS) $(XOPTS) $(INCS) arm_m3_TaskSupport_asm_iar_lib.asm


mangled__BIOS_lib.pp: BIOS.c
	@$(CC_PP) --preprocess=nl _BIOS_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ti/sysbios/BIOS.c
	@$(XDC_ROOT)/../xs -c Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs _BIOS_lib.pp mangled__BIOS_lib.pp romInternFuncs.txt  "$(INTERN_FUNCS)"

mangled_ti.sysbios.BIOS_config_lib.pp: Z:/ti/simplelink/zstack_home_1_02_02a_44539/Projects/zstack/HomeAutomation/WeatherStation/CC2650/configPkg/package/cfg/ti.sysbios.BIOS_config.c
	@$(CC_PP) --preprocess=nl ti.sysbios.BIOS_config_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	Z:/ti/simplelink/zstack_home_1_02_02a_44539/Projects/zstack/HomeAutomation/WeatherStation/CC2650/configPkg/package/cfg/ti.sysbios.BIOS_config.c
	@$(XDC_ROOT)/../xs -c Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs ti.sysbios.BIOS_config_lib.pp mangled_ti.sysbios.BIOS_config_lib.pp romInternFuncs.txt  "$(INTERN_FUNCS)"

mangled_ti.sysbios.BIOS_RtsGateProxy_config_lib.pp: Z:/ti/simplelink/zstack_home_1_02_02a_44539/Projects/zstack/HomeAutomation/WeatherStation/CC2650/configPkg/package/cfg/ti.sysbios.BIOS_RtsGateProxy_config.c
	@$(CC_PP) --preprocess=nl ti.sysbios.BIOS_RtsGateProxy_config_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	Z:/ti/simplelink/zstack_home_1_02_02a_44539/Projects/zstack/HomeAutomation/WeatherStation/CC2650/configPkg/package/cfg/ti.sysbios.BIOS_RtsGateProxy_config.c
	@$(XDC_ROOT)/../xs -c Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs ti.sysbios.BIOS_RtsGateProxy_config_lib.pp mangled_ti.sysbios.BIOS_RtsGateProxy_config_lib.pp romInternFuncs.txt  "$(INTERN_FUNCS)"

mangled_knl_Clock_lib.pp: knl/Clock.c
	@$(CC_PP) --preprocess=nl knl_Clock_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ti/sysbios/knl/Clock.c
	@$(XDC_ROOT)/../xs -c Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs knl_Clock_lib.pp mangled_knl_Clock_lib.pp romInternFuncs.txt  "$(INTERN_FUNCS)"

mangled_ti.sysbios.knl.Clock_config_lib.pp: Z:/ti/simplelink/zstack_home_1_02_02a_44539/Projects/zstack/HomeAutomation/WeatherStation/CC2650/configPkg/package/cfg/ti.sysbios.knl.Clock_config.c
	@$(CC_PP) --preprocess=nl ti.sysbios.knl.Clock_config_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	Z:/ti/simplelink/zstack_home_1_02_02a_44539/Projects/zstack/HomeAutomation/WeatherStation/CC2650/configPkg/package/cfg/ti.sysbios.knl.Clock_config.c
	@$(XDC_ROOT)/../xs -c Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs ti.sysbios.knl.Clock_config_lib.pp mangled_ti.sysbios.knl.Clock_config_lib.pp romInternFuncs.txt  "$(INTERN_FUNCS)"

mangled_knl_Idle_lib.pp: knl/Idle.c
	@$(CC_PP) --preprocess=nl knl_Idle_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ti/sysbios/knl/Idle.c
	@$(XDC_ROOT)/../xs -c Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs knl_Idle_lib.pp mangled_knl_Idle_lib.pp romInternFuncs.txt  "$(INTERN_FUNCS)"

mangled_knl_Intrinsics_lib.pp: knl/Intrinsics.c
	@$(CC_PP) --preprocess=nl knl_Intrinsics_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ti/sysbios/knl/Intrinsics.c
	@$(XDC_ROOT)/../xs -c Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs knl_Intrinsics_lib.pp mangled_knl_Intrinsics_lib.pp romInternFuncs.txt  "$(INTERN_FUNCS)"

mangled_knl_Event_lib.pp: knl/Event.c
	@$(CC_PP) --preprocess=nl knl_Event_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ti/sysbios/knl/Event.c
	@$(XDC_ROOT)/../xs -c Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs knl_Event_lib.pp mangled_knl_Event_lib.pp romInternFuncs.txt  "$(INTERN_FUNCS)"

mangled_knl_Mailbox_lib.pp: knl/Mailbox.c
	@$(CC_PP) --preprocess=nl knl_Mailbox_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ti/sysbios/knl/Mailbox.c
	@$(XDC_ROOT)/../xs -c Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs knl_Mailbox_lib.pp mangled_knl_Mailbox_lib.pp romInternFuncs.txt  "$(INTERN_FUNCS)"

mangled_ti.sysbios.knl.Mailbox_config_lib.pp: Z:/ti/simplelink/zstack_home_1_02_02a_44539/Projects/zstack/HomeAutomation/WeatherStation/CC2650/configPkg/package/cfg/ti.sysbios.knl.Mailbox_config.c
	@$(CC_PP) --preprocess=nl ti.sysbios.knl.Mailbox_config_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	Z:/ti/simplelink/zstack_home_1_02_02a_44539/Projects/zstack/HomeAutomation/WeatherStation/CC2650/configPkg/package/cfg/ti.sysbios.knl.Mailbox_config.c
	@$(XDC_ROOT)/../xs -c Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs ti.sysbios.knl.Mailbox_config_lib.pp mangled_ti.sysbios.knl.Mailbox_config_lib.pp romInternFuncs.txt  "$(INTERN_FUNCS)"

mangled_knl_Queue_lib.pp: knl/Queue.c
	@$(CC_PP) --preprocess=nl knl_Queue_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ti/sysbios/knl/Queue.c
	@$(XDC_ROOT)/../xs -c Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs knl_Queue_lib.pp mangled_knl_Queue_lib.pp romInternFuncs.txt  "$(INTERN_FUNCS)"

mangled_ti.sysbios.knl.Queue_config_lib.pp: Z:/ti/simplelink/zstack_home_1_02_02a_44539/Projects/zstack/HomeAutomation/WeatherStation/CC2650/configPkg/package/cfg/ti.sysbios.knl.Queue_config.c
	@$(CC_PP) --preprocess=nl ti.sysbios.knl.Queue_config_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	Z:/ti/simplelink/zstack_home_1_02_02a_44539/Projects/zstack/HomeAutomation/WeatherStation/CC2650/configPkg/package/cfg/ti.sysbios.knl.Queue_config.c
	@$(XDC_ROOT)/../xs -c Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs ti.sysbios.knl.Queue_config_lib.pp mangled_ti.sysbios.knl.Queue_config_lib.pp romInternFuncs.txt  "$(INTERN_FUNCS)"

mangled_knl_Semaphore_lib.pp: knl/Semaphore.c
	@$(CC_PP) --preprocess=nl knl_Semaphore_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ti/sysbios/knl/Semaphore.c
	@$(XDC_ROOT)/../xs -c Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs knl_Semaphore_lib.pp mangled_knl_Semaphore_lib.pp romInternFuncs.txt  "$(INTERN_FUNCS)"

mangled_ti.sysbios.knl.Semaphore_config_lib.pp: Z:/ti/simplelink/zstack_home_1_02_02a_44539/Projects/zstack/HomeAutomation/WeatherStation/CC2650/configPkg/package/cfg/ti.sysbios.knl.Semaphore_config.c
	@$(CC_PP) --preprocess=nl ti.sysbios.knl.Semaphore_config_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	Z:/ti/simplelink/zstack_home_1_02_02a_44539/Projects/zstack/HomeAutomation/WeatherStation/CC2650/configPkg/package/cfg/ti.sysbios.knl.Semaphore_config.c
	@$(XDC_ROOT)/../xs -c Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs ti.sysbios.knl.Semaphore_config_lib.pp mangled_ti.sysbios.knl.Semaphore_config_lib.pp romInternFuncs.txt  "$(INTERN_FUNCS)"

mangled_knl_Swi_lib.pp: knl/Swi.c
	@$(CC_PP) --preprocess=nl knl_Swi_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ti/sysbios/knl/Swi.c
	@$(XDC_ROOT)/../xs -c Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs knl_Swi_lib.pp mangled_knl_Swi_lib.pp romInternFuncs.txt  "$(INTERN_FUNCS)"

mangled_knl_Swi_andn_lib.pp: knl/Swi_andn.c
	@$(CC_PP) --preprocess=nl knl_Swi_andn_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ti/sysbios/knl/Swi_andn.c
	@$(XDC_ROOT)/../xs -c Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs knl_Swi_andn_lib.pp mangled_knl_Swi_andn_lib.pp romInternFuncs.txt  "$(INTERN_FUNCS)"

mangled_ti.sysbios.knl.Swi_config_lib.pp: Z:/ti/simplelink/zstack_home_1_02_02a_44539/Projects/zstack/HomeAutomation/WeatherStation/CC2650/configPkg/package/cfg/ti.sysbios.knl.Swi_config.c
	@$(CC_PP) --preprocess=nl ti.sysbios.knl.Swi_config_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	Z:/ti/simplelink/zstack_home_1_02_02a_44539/Projects/zstack/HomeAutomation/WeatherStation/CC2650/configPkg/package/cfg/ti.sysbios.knl.Swi_config.c
	@$(XDC_ROOT)/../xs -c Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs ti.sysbios.knl.Swi_config_lib.pp mangled_ti.sysbios.knl.Swi_config_lib.pp romInternFuncs.txt  "$(INTERN_FUNCS)"

mangled_knl_Task_lib.pp: knl/Task.c
	@$(CC_PP) --preprocess=nl knl_Task_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ti/sysbios/knl/Task.c
	@$(XDC_ROOT)/../xs -c Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs knl_Task_lib.pp mangled_knl_Task_lib.pp romInternFuncs.txt  "$(INTERN_FUNCS)"

mangled_ti.sysbios.knl.Task_config_lib.pp: Z:/ti/simplelink/zstack_home_1_02_02a_44539/Projects/zstack/HomeAutomation/WeatherStation/CC2650/configPkg/package/cfg/ti.sysbios.knl.Task_config.c
	@$(CC_PP) --preprocess=nl ti.sysbios.knl.Task_config_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	Z:/ti/simplelink/zstack_home_1_02_02a_44539/Projects/zstack/HomeAutomation/WeatherStation/CC2650/configPkg/package/cfg/ti.sysbios.knl.Task_config.c
	@$(XDC_ROOT)/../xs -c Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs ti.sysbios.knl.Task_config_lib.pp mangled_ti.sysbios.knl.Task_config_lib.pp romInternFuncs.txt  "$(INTERN_FUNCS)"

mangled_ti.sysbios.knl.Clock_TimerProxy_config_lib.pp: Z:/ti/simplelink/zstack_home_1_02_02a_44539/Projects/zstack/HomeAutomation/WeatherStation/CC2650/configPkg/package/cfg/ti.sysbios.knl.Clock_TimerProxy_config.c
	@$(CC_PP) --preprocess=nl ti.sysbios.knl.Clock_TimerProxy_config_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	Z:/ti/simplelink/zstack_home_1_02_02a_44539/Projects/zstack/HomeAutomation/WeatherStation/CC2650/configPkg/package/cfg/ti.sysbios.knl.Clock_TimerProxy_config.c
	@$(XDC_ROOT)/../xs -c Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs ti.sysbios.knl.Clock_TimerProxy_config_lib.pp mangled_ti.sysbios.knl.Clock_TimerProxy_config_lib.pp romInternFuncs.txt  "$(INTERN_FUNCS)"

mangled_ti.sysbios.knl.Task_SupportProxy_config_lib.pp: Z:/ti/simplelink/zstack_home_1_02_02a_44539/Projects/zstack/HomeAutomation/WeatherStation/CC2650/configPkg/package/cfg/ti.sysbios.knl.Task_SupportProxy_config.c
	@$(CC_PP) --preprocess=nl ti.sysbios.knl.Task_SupportProxy_config_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	Z:/ti/simplelink/zstack_home_1_02_02a_44539/Projects/zstack/HomeAutomation/WeatherStation/CC2650/configPkg/package/cfg/ti.sysbios.knl.Task_SupportProxy_config.c
	@$(XDC_ROOT)/../xs -c Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs ti.sysbios.knl.Task_SupportProxy_config_lib.pp mangled_ti.sysbios.knl.Task_SupportProxy_config_lib.pp romInternFuncs.txt  "$(INTERN_FUNCS)"

mangled_arm_m3_Hwi_lib.pp: family/arm/m3/Hwi.c
	@$(CC_PP) --preprocess=nl arm_m3_Hwi_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ti/sysbios/family/arm/m3/Hwi.c
	@$(XDC_ROOT)/../xs -c Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs arm_m3_Hwi_lib.pp mangled_arm_m3_Hwi_lib.pp romInternFuncs.txt  "$(INTERN_FUNCS)"

mangled_ti.sysbios.family.arm.m3.Hwi_config_lib.pp: Z:/ti/simplelink/zstack_home_1_02_02a_44539/Projects/zstack/HomeAutomation/WeatherStation/CC2650/configPkg/package/cfg/ti.sysbios.family.arm.m3.Hwi_config.c
	@$(CC_PP) --preprocess=nl ti.sysbios.family.arm.m3.Hwi_config_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	Z:/ti/simplelink/zstack_home_1_02_02a_44539/Projects/zstack/HomeAutomation/WeatherStation/CC2650/configPkg/package/cfg/ti.sysbios.family.arm.m3.Hwi_config.c
	@$(XDC_ROOT)/../xs -c Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs ti.sysbios.family.arm.m3.Hwi_config_lib.pp mangled_ti.sysbios.family.arm.m3.Hwi_config_lib.pp romInternFuncs.txt  "$(INTERN_FUNCS)"

mangled_arm_m3_TaskSupport_lib.pp: family/arm/m3/TaskSupport.c
	@$(CC_PP) --preprocess=nl arm_m3_TaskSupport_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ti/sysbios/family/arm/m3/TaskSupport.c
	@$(XDC_ROOT)/../xs -c Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs arm_m3_TaskSupport_lib.pp mangled_arm_m3_TaskSupport_lib.pp romInternFuncs.txt  "$(INTERN_FUNCS)"

mangled_ti.sysbios.family.arm.m3.TaskSupport_config_lib.pp: Z:/ti/simplelink/zstack_home_1_02_02a_44539/Projects/zstack/HomeAutomation/WeatherStation/CC2650/configPkg/package/cfg/ti.sysbios.family.arm.m3.TaskSupport_config.c
	@$(CC_PP) --preprocess=nl ti.sysbios.family.arm.m3.TaskSupport_config_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	Z:/ti/simplelink/zstack_home_1_02_02a_44539/Projects/zstack/HomeAutomation/WeatherStation/CC2650/configPkg/package/cfg/ti.sysbios.family.arm.m3.TaskSupport_config.c
	@$(XDC_ROOT)/../xs -c Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs ti.sysbios.family.arm.m3.TaskSupport_config_lib.pp mangled_ti.sysbios.family.arm.m3.TaskSupport_config_lib.pp romInternFuncs.txt  "$(INTERN_FUNCS)"

mangled_arm_cc26xx_Boot_lib.pp: family/arm/cc26xx/Boot.c
	@$(CC_PP) --preprocess=nl arm_cc26xx_Boot_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ti/sysbios/family/arm/cc26xx/Boot.c
	@$(XDC_ROOT)/../xs -c Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs arm_cc26xx_Boot_lib.pp mangled_arm_cc26xx_Boot_lib.pp romInternFuncs.txt  "$(INTERN_FUNCS)"

mangled_arm_cc26xx_Power_lib.pp: family/arm/cc26xx/Power.c
	@$(CC_PP) --preprocess=nl arm_cc26xx_Power_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ti/sysbios/family/arm/cc26xx/Power.c
	@$(XDC_ROOT)/../xs -c Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs arm_cc26xx_Power_lib.pp mangled_arm_cc26xx_Power_lib.pp romInternFuncs.txt  "$(INTERN_FUNCS)"

mangled_arm_cc26xx_Power_standbyPolicy_lib.pp: family/arm/cc26xx/Power_standbyPolicy.c
	@$(CC_PP) --preprocess=nl arm_cc26xx_Power_standbyPolicy_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ti/sysbios/family/arm/cc26xx/Power_standbyPolicy.c
	@$(XDC_ROOT)/../xs -c Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs arm_cc26xx_Power_standbyPolicy_lib.pp mangled_arm_cc26xx_Power_standbyPolicy_lib.pp romInternFuncs.txt  "$(INTERN_FUNCS)"

mangled_arm_cc26xx_Power_calibrateRCOSC_lib.pp: family/arm/cc26xx/Power_calibrateRCOSC.c
	@$(CC_PP) --preprocess=nl arm_cc26xx_Power_calibrateRCOSC_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ti/sysbios/family/arm/cc26xx/Power_calibrateRCOSC.c
	@$(XDC_ROOT)/../xs -c Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs arm_cc26xx_Power_calibrateRCOSC_lib.pp mangled_arm_cc26xx_Power_calibrateRCOSC_lib.pp romInternFuncs.txt  "$(INTERN_FUNCS)"

mangled_arm_cc26xx_pg2_leakage_workaround_lib.pp: family/arm/cc26xx/pg2_leakage_workaround.c
	@$(CC_PP) --preprocess=nl arm_cc26xx_pg2_leakage_workaround_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ti/sysbios/family/arm/cc26xx/pg2_leakage_workaround.c
	@$(XDC_ROOT)/../xs -c Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs arm_cc26xx_pg2_leakage_workaround_lib.pp mangled_arm_cc26xx_pg2_leakage_workaround_lib.pp romInternFuncs.txt  "$(INTERN_FUNCS)"

mangled_arm_cc26xx_TimerGPT_lib.pp: family/arm/cc26xx/TimerGPT.c
	@$(CC_PP) --preprocess=nl arm_cc26xx_TimerGPT_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ti/sysbios/family/arm/cc26xx/TimerGPT.c
	@$(XDC_ROOT)/../xs -c Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs arm_cc26xx_TimerGPT_lib.pp mangled_arm_cc26xx_TimerGPT_lib.pp romInternFuncs.txt  "$(INTERN_FUNCS)"

mangled_arm_cc26xx_Timer_lib.pp: family/arm/cc26xx/Timer.c
	@$(CC_PP) --preprocess=nl arm_cc26xx_Timer_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ti/sysbios/family/arm/cc26xx/Timer.c
	@$(XDC_ROOT)/../xs -c Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs arm_cc26xx_Timer_lib.pp mangled_arm_cc26xx_Timer_lib.pp romInternFuncs.txt  "$(INTERN_FUNCS)"

mangled_ti.sysbios.family.arm.cc26xx.Timer_config_lib.pp: Z:/ti/simplelink/zstack_home_1_02_02a_44539/Projects/zstack/HomeAutomation/WeatherStation/CC2650/configPkg/package/cfg/ti.sysbios.family.arm.cc26xx.Timer_config.c
	@$(CC_PP) --preprocess=nl ti.sysbios.family.arm.cc26xx.Timer_config_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	Z:/ti/simplelink/zstack_home_1_02_02a_44539/Projects/zstack/HomeAutomation/WeatherStation/CC2650/configPkg/package/cfg/ti.sysbios.family.arm.cc26xx.Timer_config.c
	@$(XDC_ROOT)/../xs -c Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs ti.sysbios.family.arm.cc26xx.Timer_config_lib.pp mangled_ti.sysbios.family.arm.cc26xx.Timer_config_lib.pp romInternFuncs.txt  "$(INTERN_FUNCS)"

mangled_arm_cc26xx_TimestampProvider_lib.pp: family/arm/cc26xx/TimestampProvider.c
	@$(CC_PP) --preprocess=nl arm_cc26xx_TimestampProvider_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ti/sysbios/family/arm/cc26xx/TimestampProvider.c
	@$(XDC_ROOT)/../xs -c Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs arm_cc26xx_TimestampProvider_lib.pp mangled_arm_cc26xx_TimestampProvider_lib.pp romInternFuncs.txt  "$(INTERN_FUNCS)"

mangled_ti.sysbios.family.arm.cc26xx.TimestampProvider_config_lib.pp: Z:/ti/simplelink/zstack_home_1_02_02a_44539/Projects/zstack/HomeAutomation/WeatherStation/CC2650/configPkg/package/cfg/ti.sysbios.family.arm.cc26xx.TimestampProvider_config.c
	@$(CC_PP) --preprocess=nl ti.sysbios.family.arm.cc26xx.TimestampProvider_config_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	Z:/ti/simplelink/zstack_home_1_02_02a_44539/Projects/zstack/HomeAutomation/WeatherStation/CC2650/configPkg/package/cfg/ti.sysbios.family.arm.cc26xx.TimestampProvider_config.c
	@$(XDC_ROOT)/../xs -c Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs ti.sysbios.family.arm.cc26xx.TimestampProvider_config_lib.pp mangled_ti.sysbios.family.arm.cc26xx.TimestampProvider_config_lib.pp romInternFuncs.txt  "$(INTERN_FUNCS)"

mangled_hal_Hwi_lib.pp: hal/Hwi.c
	@$(CC_PP) --preprocess=nl hal_Hwi_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ti/sysbios/hal/Hwi.c
	@$(XDC_ROOT)/../xs -c Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs hal_Hwi_lib.pp mangled_hal_Hwi_lib.pp romInternFuncs.txt  "$(INTERN_FUNCS)"

mangled_hal_Hwi_stack_lib.pp: hal/Hwi_stack.c
	@$(CC_PP) --preprocess=nl hal_Hwi_stack_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ti/sysbios/hal/Hwi_stack.c
	@$(XDC_ROOT)/../xs -c Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs hal_Hwi_stack_lib.pp mangled_hal_Hwi_stack_lib.pp romInternFuncs.txt  "$(INTERN_FUNCS)"

mangled_hal_Hwi_startup_lib.pp: hal/Hwi_startup.c
	@$(CC_PP) --preprocess=nl hal_Hwi_startup_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ti/sysbios/hal/Hwi_startup.c
	@$(XDC_ROOT)/../xs -c Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs hal_Hwi_startup_lib.pp mangled_hal_Hwi_startup_lib.pp romInternFuncs.txt  "$(INTERN_FUNCS)"

mangled_ti.sysbios.hal.Hwi_config_lib.pp: Z:/ti/simplelink/zstack_home_1_02_02a_44539/Projects/zstack/HomeAutomation/WeatherStation/CC2650/configPkg/package/cfg/ti.sysbios.hal.Hwi_config.c
	@$(CC_PP) --preprocess=nl ti.sysbios.hal.Hwi_config_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	Z:/ti/simplelink/zstack_home_1_02_02a_44539/Projects/zstack/HomeAutomation/WeatherStation/CC2650/configPkg/package/cfg/ti.sysbios.hal.Hwi_config.c
	@$(XDC_ROOT)/../xs -c Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs ti.sysbios.hal.Hwi_config_lib.pp mangled_ti.sysbios.hal.Hwi_config_lib.pp romInternFuncs.txt  "$(INTERN_FUNCS)"

mangled_ti.sysbios.hal.Hwi_HwiProxy_config_lib.pp: Z:/ti/simplelink/zstack_home_1_02_02a_44539/Projects/zstack/HomeAutomation/WeatherStation/CC2650/configPkg/package/cfg/ti.sysbios.hal.Hwi_HwiProxy_config.c
	@$(CC_PP) --preprocess=nl ti.sysbios.hal.Hwi_HwiProxy_config_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	Z:/ti/simplelink/zstack_home_1_02_02a_44539/Projects/zstack/HomeAutomation/WeatherStation/CC2650/configPkg/package/cfg/ti.sysbios.hal.Hwi_HwiProxy_config.c
	@$(XDC_ROOT)/../xs -c Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs ti.sysbios.hal.Hwi_HwiProxy_config_lib.pp mangled_ti.sysbios.hal.Hwi_HwiProxy_config_lib.pp romInternFuncs.txt  "$(INTERN_FUNCS)"

mangled_heaps_HeapMem_lib.pp: heaps/HeapMem.c
	@$(CC_PP) --preprocess=nl heaps_HeapMem_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ti/sysbios/heaps/HeapMem.c
	@$(XDC_ROOT)/../xs -c Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs heaps_HeapMem_lib.pp mangled_heaps_HeapMem_lib.pp romInternFuncs.txt  "$(INTERN_FUNCS)"

mangled_ti.sysbios.heaps.HeapMem_config_lib.pp: Z:/ti/simplelink/zstack_home_1_02_02a_44539/Projects/zstack/HomeAutomation/WeatherStation/CC2650/configPkg/package/cfg/ti.sysbios.heaps.HeapMem_config.c
	@$(CC_PP) --preprocess=nl ti.sysbios.heaps.HeapMem_config_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	Z:/ti/simplelink/zstack_home_1_02_02a_44539/Projects/zstack/HomeAutomation/WeatherStation/CC2650/configPkg/package/cfg/ti.sysbios.heaps.HeapMem_config.c
	@$(XDC_ROOT)/../xs -c Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs ti.sysbios.heaps.HeapMem_config_lib.pp mangled_ti.sysbios.heaps.HeapMem_config_lib.pp romInternFuncs.txt  "$(INTERN_FUNCS)"

mangled_ti.sysbios.heaps.HeapMem_Module_GateProxy_config_lib.pp: Z:/ti/simplelink/zstack_home_1_02_02a_44539/Projects/zstack/HomeAutomation/WeatherStation/CC2650/configPkg/package/cfg/ti.sysbios.heaps.HeapMem_Module_GateProxy_config.c
	@$(CC_PP) --preprocess=nl ti.sysbios.heaps.HeapMem_Module_GateProxy_config_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	Z:/ti/simplelink/zstack_home_1_02_02a_44539/Projects/zstack/HomeAutomation/WeatherStation/CC2650/configPkg/package/cfg/ti.sysbios.heaps.HeapMem_Module_GateProxy_config.c
	@$(XDC_ROOT)/../xs -c Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs ti.sysbios.heaps.HeapMem_Module_GateProxy_config_lib.pp mangled_ti.sysbios.heaps.HeapMem_Module_GateProxy_config_lib.pp romInternFuncs.txt  "$(INTERN_FUNCS)"

mangled_gates_GateHwi_lib.pp: gates/GateHwi.c
	@$(CC_PP) --preprocess=nl gates_GateHwi_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ti/sysbios/gates/GateHwi.c
	@$(XDC_ROOT)/../xs -c Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs gates_GateHwi_lib.pp mangled_gates_GateHwi_lib.pp romInternFuncs.txt  "$(INTERN_FUNCS)"

mangled_ti.sysbios.gates.GateHwi_config_lib.pp: Z:/ti/simplelink/zstack_home_1_02_02a_44539/Projects/zstack/HomeAutomation/WeatherStation/CC2650/configPkg/package/cfg/ti.sysbios.gates.GateHwi_config.c
	@$(CC_PP) --preprocess=nl ti.sysbios.gates.GateHwi_config_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	Z:/ti/simplelink/zstack_home_1_02_02a_44539/Projects/zstack/HomeAutomation/WeatherStation/CC2650/configPkg/package/cfg/ti.sysbios.gates.GateHwi_config.c
	@$(XDC_ROOT)/../xs -c Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs ti.sysbios.gates.GateHwi_config_lib.pp mangled_ti.sysbios.gates.GateHwi_config_lib.pp romInternFuncs.txt  "$(INTERN_FUNCS)"

mangled_gates_GateMutex_lib.pp: gates/GateMutex.c
	@$(CC_PP) --preprocess=nl gates_GateMutex_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ti/sysbios/gates/GateMutex.c
	@$(XDC_ROOT)/../xs -c Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs gates_GateMutex_lib.pp mangled_gates_GateMutex_lib.pp romInternFuncs.txt  "$(INTERN_FUNCS)"

mangled_ti.sysbios.gates.GateMutex_config_lib.pp: Z:/ti/simplelink/zstack_home_1_02_02a_44539/Projects/zstack/HomeAutomation/WeatherStation/CC2650/configPkg/package/cfg/ti.sysbios.gates.GateMutex_config.c
	@$(CC_PP) --preprocess=nl ti.sysbios.gates.GateMutex_config_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	Z:/ti/simplelink/zstack_home_1_02_02a_44539/Projects/zstack/HomeAutomation/WeatherStation/CC2650/configPkg/package/cfg/ti.sysbios.gates.GateMutex_config.c
	@$(XDC_ROOT)/../xs -c Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs ti.sysbios.gates.GateMutex_config_lib.pp mangled_ti.sysbios.gates.GateMutex_config_lib.pp romInternFuncs.txt  "$(INTERN_FUNCS)"

mangled_xdc_runtime_xdc_noinit_lib.pp: xdc/runtime/xdc_noinit.c
	@$(CC_PP) --preprocess=nl xdc_runtime_xdc_noinit_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	Z:/ti/xdctools_3_30_06_67_core/packages/xdc/runtime/xdc_noinit.c
	@$(XDC_ROOT)/../xs -c Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs xdc_runtime_xdc_noinit_lib.pp mangled_xdc_runtime_xdc_noinit_lib.pp romInternFuncs.txt  "$(INTERN_FUNCS)"

mangled_xdc_runtime_Assert_lib.pp: xdc/runtime/Assert.c
	@$(CC_PP) --preprocess=nl xdc_runtime_Assert_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	Z:/ti/xdctools_3_30_06_67_core/packages/xdc/runtime/Assert.c
	@$(XDC_ROOT)/../xs -c Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs xdc_runtime_Assert_lib.pp mangled_xdc_runtime_Assert_lib.pp romInternFuncs.txt  "$(INTERN_FUNCS)"

mangled_xdc_runtime_Core-mem_lib.pp: xdc/runtime/Core-mem.c
	@$(CC_PP) --preprocess=nl xdc_runtime_Core-mem_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	Z:/ti/xdctools_3_30_06_67_core/packages/xdc/runtime/Core-mem.c
	@$(XDC_ROOT)/../xs -c Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs xdc_runtime_Core-mem_lib.pp mangled_xdc_runtime_Core-mem_lib.pp romInternFuncs.txt  "$(INTERN_FUNCS)"

mangled_xdc_runtime_Core-smem_lib.pp: xdc/runtime/Core-smem.c
	@$(CC_PP) --preprocess=nl xdc_runtime_Core-smem_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	Z:/ti/xdctools_3_30_06_67_core/packages/xdc/runtime/Core-smem.c
	@$(XDC_ROOT)/../xs -c Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs xdc_runtime_Core-smem_lib.pp mangled_xdc_runtime_Core-smem_lib.pp romInternFuncs.txt  "$(INTERN_FUNCS)"

mangled_xdc_runtime_Core-label_lib.pp: xdc/runtime/Core-label.c
	@$(CC_PP) --preprocess=nl xdc_runtime_Core-label_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	Z:/ti/xdctools_3_30_06_67_core/packages/xdc/runtime/Core-label.c
	@$(XDC_ROOT)/../xs -c Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs xdc_runtime_Core-label_lib.pp mangled_xdc_runtime_Core-label_lib.pp romInternFuncs.txt  "$(INTERN_FUNCS)"

mangled_xdc_runtime_Core-params_lib.pp: xdc/runtime/Core-params.c
	@$(CC_PP) --preprocess=nl xdc_runtime_Core-params_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	Z:/ti/xdctools_3_30_06_67_core/packages/xdc/runtime/Core-params.c
	@$(XDC_ROOT)/../xs -c Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs xdc_runtime_Core-params_lib.pp mangled_xdc_runtime_Core-params_lib.pp romInternFuncs.txt  "$(INTERN_FUNCS)"

mangled_xdc_runtime_Diags_lib.pp: xdc/runtime/Diags.c
	@$(CC_PP) --preprocess=nl xdc_runtime_Diags_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	Z:/ti/xdctools_3_30_06_67_core/packages/xdc/runtime/Diags.c
	@$(XDC_ROOT)/../xs -c Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs xdc_runtime_Diags_lib.pp mangled_xdc_runtime_Diags_lib.pp romInternFuncs.txt  "$(INTERN_FUNCS)"

mangled_xdc_runtime_Error_lib.pp: xdc/runtime/Error.c
	@$(CC_PP) --preprocess=nl xdc_runtime_Error_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	Z:/ti/xdctools_3_30_06_67_core/packages/xdc/runtime/Error.c
	@$(XDC_ROOT)/../xs -c Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs xdc_runtime_Error_lib.pp mangled_xdc_runtime_Error_lib.pp romInternFuncs.txt  "$(INTERN_FUNCS)"

mangled_xdc.runtime.Error_config_lib.pp: Z:/ti/simplelink/zstack_home_1_02_02a_44539/Projects/zstack/HomeAutomation/WeatherStation/CC2650/configPkg/package/cfg/xdc.runtime.Error_config.c
	@$(CC_PP) --preprocess=nl xdc.runtime.Error_config_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	Z:/ti/simplelink/zstack_home_1_02_02a_44539/Projects/zstack/HomeAutomation/WeatherStation/CC2650/configPkg/package/cfg/xdc.runtime.Error_config.c
	@$(XDC_ROOT)/../xs -c Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs xdc.runtime.Error_config_lib.pp mangled_xdc.runtime.Error_config_lib.pp romInternFuncs.txt  "$(INTERN_FUNCS)"

mangled_xdc_runtime_Gate_lib.pp: xdc/runtime/Gate.c
	@$(CC_PP) --preprocess=nl xdc_runtime_Gate_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	Z:/ti/xdctools_3_30_06_67_core/packages/xdc/runtime/Gate.c
	@$(XDC_ROOT)/../xs -c Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs xdc_runtime_Gate_lib.pp mangled_xdc_runtime_Gate_lib.pp romInternFuncs.txt  "$(INTERN_FUNCS)"

mangled_xdc_runtime_Log_lib.pp: xdc/runtime/Log.c
	@$(CC_PP) --preprocess=nl xdc_runtime_Log_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	Z:/ti/xdctools_3_30_06_67_core/packages/xdc/runtime/Log.c
	@$(XDC_ROOT)/../xs -c Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs xdc_runtime_Log_lib.pp mangled_xdc_runtime_Log_lib.pp romInternFuncs.txt  "$(INTERN_FUNCS)"

mangled_xdc_runtime_Memory_lib.pp: xdc/runtime/Memory.c
	@$(CC_PP) --preprocess=nl xdc_runtime_Memory_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	Z:/ti/xdctools_3_30_06_67_core/packages/xdc/runtime/Memory.c
	@$(XDC_ROOT)/../xs -c Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs xdc_runtime_Memory_lib.pp mangled_xdc_runtime_Memory_lib.pp romInternFuncs.txt  "$(INTERN_FUNCS)"

mangled_xdc.runtime.Memory_config_lib.pp: Z:/ti/simplelink/zstack_home_1_02_02a_44539/Projects/zstack/HomeAutomation/WeatherStation/CC2650/configPkg/package/cfg/xdc.runtime.Memory_config.c
	@$(CC_PP) --preprocess=nl xdc.runtime.Memory_config_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	Z:/ti/simplelink/zstack_home_1_02_02a_44539/Projects/zstack/HomeAutomation/WeatherStation/CC2650/configPkg/package/cfg/xdc.runtime.Memory_config.c
	@$(XDC_ROOT)/../xs -c Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs xdc.runtime.Memory_config_lib.pp mangled_xdc.runtime.Memory_config_lib.pp romInternFuncs.txt  "$(INTERN_FUNCS)"

mangled_xdc_runtime_Registry_lib.pp: xdc/runtime/Registry.c
	@$(CC_PP) --preprocess=nl xdc_runtime_Registry_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	Z:/ti/xdctools_3_30_06_67_core/packages/xdc/runtime/Registry.c
	@$(XDC_ROOT)/../xs -c Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs xdc_runtime_Registry_lib.pp mangled_xdc_runtime_Registry_lib.pp romInternFuncs.txt  "$(INTERN_FUNCS)"

mangled_xdc_runtime_Startup_lib.pp: xdc/runtime/Startup.c
	@$(CC_PP) --preprocess=nl xdc_runtime_Startup_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	Z:/ti/xdctools_3_30_06_67_core/packages/xdc/runtime/Startup.c
	@$(XDC_ROOT)/../xs -c Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs xdc_runtime_Startup_lib.pp mangled_xdc_runtime_Startup_lib.pp romInternFuncs.txt  "$(INTERN_FUNCS)"

mangled_xdc_runtime_System_lib.pp: xdc/runtime/System.c
	@$(CC_PP) --preprocess=nl xdc_runtime_System_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	Z:/ti/xdctools_3_30_06_67_core/packages/xdc/runtime/System.c
	@$(XDC_ROOT)/../xs -c Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs xdc_runtime_System_lib.pp mangled_xdc_runtime_System_lib.pp romInternFuncs.txt  "$(INTERN_FUNCS)"

mangled_xdc.runtime.System_config_lib.pp: Z:/ti/simplelink/zstack_home_1_02_02a_44539/Projects/zstack/HomeAutomation/WeatherStation/CC2650/configPkg/package/cfg/xdc.runtime.System_config.c
	@$(CC_PP) --preprocess=nl xdc.runtime.System_config_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	Z:/ti/simplelink/zstack_home_1_02_02a_44539/Projects/zstack/HomeAutomation/WeatherStation/CC2650/configPkg/package/cfg/xdc.runtime.System_config.c
	@$(XDC_ROOT)/../xs -c Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs xdc.runtime.System_config_lib.pp mangled_xdc.runtime.System_config_lib.pp romInternFuncs.txt  "$(INTERN_FUNCS)"

mangled_xdc_runtime_SysCallback_lib.pp: xdc/runtime/SysCallback.c
	@$(CC_PP) --preprocess=nl xdc_runtime_SysCallback_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	Z:/ti/xdctools_3_30_06_67_core/packages/xdc/runtime/SysCallback.c
	@$(XDC_ROOT)/../xs -c Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs xdc_runtime_SysCallback_lib.pp mangled_xdc_runtime_SysCallback_lib.pp romInternFuncs.txt  "$(INTERN_FUNCS)"

mangled_xdc.runtime.SysCallback_config_lib.pp: Z:/ti/simplelink/zstack_home_1_02_02a_44539/Projects/zstack/HomeAutomation/WeatherStation/CC2650/configPkg/package/cfg/xdc.runtime.SysCallback_config.c
	@$(CC_PP) --preprocess=nl xdc.runtime.SysCallback_config_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	Z:/ti/simplelink/zstack_home_1_02_02a_44539/Projects/zstack/HomeAutomation/WeatherStation/CC2650/configPkg/package/cfg/xdc.runtime.SysCallback_config.c
	@$(XDC_ROOT)/../xs -c Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs xdc.runtime.SysCallback_config_lib.pp mangled_xdc.runtime.SysCallback_config_lib.pp romInternFuncs.txt  "$(INTERN_FUNCS)"

mangled_xdc_runtime_Text_lib.pp: xdc/runtime/Text.c
	@$(CC_PP) --preprocess=nl xdc_runtime_Text_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	Z:/ti/xdctools_3_30_06_67_core/packages/xdc/runtime/Text.c
	@$(XDC_ROOT)/../xs -c Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs xdc_runtime_Text_lib.pp mangled_xdc_runtime_Text_lib.pp romInternFuncs.txt  "$(INTERN_FUNCS)"

mangled_xdc_runtime_Timestamp_lib.pp: xdc/runtime/Timestamp.c
	@$(CC_PP) --preprocess=nl xdc_runtime_Timestamp_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	Z:/ti/xdctools_3_30_06_67_core/packages/xdc/runtime/Timestamp.c
	@$(XDC_ROOT)/../xs -c Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs xdc_runtime_Timestamp_lib.pp mangled_xdc_runtime_Timestamp_lib.pp romInternFuncs.txt  "$(INTERN_FUNCS)"

mangled_xdc.runtime.Timestamp_config_lib.pp: Z:/ti/simplelink/zstack_home_1_02_02a_44539/Projects/zstack/HomeAutomation/WeatherStation/CC2650/configPkg/package/cfg/xdc.runtime.Timestamp_config.c
	@$(CC_PP) --preprocess=nl xdc.runtime.Timestamp_config_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	Z:/ti/simplelink/zstack_home_1_02_02a_44539/Projects/zstack/HomeAutomation/WeatherStation/CC2650/configPkg/package/cfg/xdc.runtime.Timestamp_config.c
	@$(XDC_ROOT)/../xs -c Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs xdc.runtime.Timestamp_config_lib.pp mangled_xdc.runtime.Timestamp_config_lib.pp romInternFuncs.txt  "$(INTERN_FUNCS)"

mangled_xdc.runtime.Memory_HeapProxy_config_lib.pp: Z:/ti/simplelink/zstack_home_1_02_02a_44539/Projects/zstack/HomeAutomation/WeatherStation/CC2650/configPkg/package/cfg/xdc.runtime.Memory_HeapProxy_config.c
	@$(CC_PP) --preprocess=nl xdc.runtime.Memory_HeapProxy_config_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	Z:/ti/simplelink/zstack_home_1_02_02a_44539/Projects/zstack/HomeAutomation/WeatherStation/CC2650/configPkg/package/cfg/xdc.runtime.Memory_HeapProxy_config.c
	@$(XDC_ROOT)/../xs -c Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs xdc.runtime.Memory_HeapProxy_config_lib.pp mangled_xdc.runtime.Memory_HeapProxy_config_lib.pp romInternFuncs.txt  "$(INTERN_FUNCS)"

mangled_xdc.runtime.System_Module_GateProxy_config_lib.pp: Z:/ti/simplelink/zstack_home_1_02_02a_44539/Projects/zstack/HomeAutomation/WeatherStation/CC2650/configPkg/package/cfg/xdc.runtime.System_Module_GateProxy_config.c
	@$(CC_PP) --preprocess=nl xdc.runtime.System_Module_GateProxy_config_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	Z:/ti/simplelink/zstack_home_1_02_02a_44539/Projects/zstack/HomeAutomation/WeatherStation/CC2650/configPkg/package/cfg/xdc.runtime.System_Module_GateProxy_config.c
	@$(XDC_ROOT)/../xs -c Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs xdc.runtime.System_Module_GateProxy_config_lib.pp mangled_xdc.runtime.System_Module_GateProxy_config_lib.pp romInternFuncs.txt  "$(INTERN_FUNCS)"

mangled_xdc.runtime.Timestamp_SupportProxy_config_lib.pp: Z:/ti/simplelink/zstack_home_1_02_02a_44539/Projects/zstack/HomeAutomation/WeatherStation/CC2650/configPkg/package/cfg/xdc.runtime.Timestamp_SupportProxy_config.c
	@$(CC_PP) --preprocess=nl xdc.runtime.Timestamp_SupportProxy_config_lib.pp $(BIOS_DEFS) $(XOPTS) $(INCS) \
	Z:/ti/simplelink/zstack_home_1_02_02a_44539/Projects/zstack/HomeAutomation/WeatherStation/CC2650/configPkg/package/cfg/xdc.runtime.Timestamp_SupportProxy_config.c
	@$(XDC_ROOT)/../xs -c Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages/ti/sysbios/rom/cortexm/cc26xx/CC26xx/../mangleCFile.xs xdc.runtime.Timestamp_SupportProxy_config_lib.pp mangled_xdc.runtime.Timestamp_SupportProxy_config_lib.pp romInternFuncs.txt  "$(INTERN_FUNCS)"


mangled__BIOS_lib.obj: mangled__BIOS_lib.pp mangled_ti.sysbios.BIOS_config_lib.pp mangled_ti.sysbios.BIOS_RtsGateProxy_config_lib.pp mangled_knl_Clock_lib.pp mangled_ti.sysbios.knl.Clock_config_lib.pp mangled_knl_Idle_lib.pp mangled_knl_Intrinsics_lib.pp mangled_knl_Event_lib.pp mangled_knl_Mailbox_lib.pp mangled_ti.sysbios.knl.Mailbox_config_lib.pp mangled_knl_Queue_lib.pp mangled_ti.sysbios.knl.Queue_config_lib.pp mangled_knl_Semaphore_lib.pp mangled_ti.sysbios.knl.Semaphore_config_lib.pp mangled_knl_Swi_lib.pp mangled_knl_Swi_andn_lib.pp mangled_ti.sysbios.knl.Swi_config_lib.pp mangled_knl_Task_lib.pp mangled_ti.sysbios.knl.Task_config_lib.pp mangled_ti.sysbios.knl.Clock_TimerProxy_config_lib.pp mangled_ti.sysbios.knl.Task_SupportProxy_config_lib.pp mangled_arm_m3_Hwi_lib.pp mangled_ti.sysbios.family.arm.m3.Hwi_config_lib.pp mangled_arm_m3_TaskSupport_lib.pp mangled_ti.sysbios.family.arm.m3.TaskSupport_config_lib.pp mangled_arm_cc26xx_Boot_lib.pp mangled_arm_cc26xx_Power_lib.pp mangled_arm_cc26xx_Power_standbyPolicy_lib.pp mangled_arm_cc26xx_Power_calibrateRCOSC_lib.pp mangled_arm_cc26xx_pg2_leakage_workaround_lib.pp mangled_arm_cc26xx_TimerGPT_lib.pp mangled_arm_cc26xx_Timer_lib.pp mangled_ti.sysbios.family.arm.cc26xx.Timer_config_lib.pp mangled_arm_cc26xx_TimestampProvider_lib.pp mangled_ti.sysbios.family.arm.cc26xx.TimestampProvider_config_lib.pp mangled_hal_Hwi_lib.pp mangled_hal_Hwi_stack_lib.pp mangled_hal_Hwi_startup_lib.pp mangled_ti.sysbios.hal.Hwi_config_lib.pp mangled_ti.sysbios.hal.Hwi_HwiProxy_config_lib.pp mangled_heaps_HeapMem_lib.pp mangled_ti.sysbios.heaps.HeapMem_config_lib.pp mangled_ti.sysbios.heaps.HeapMem_Module_GateProxy_config_lib.pp mangled_gates_GateHwi_lib.pp mangled_ti.sysbios.gates.GateHwi_config_lib.pp mangled_gates_GateMutex_lib.pp mangled_ti.sysbios.gates.GateMutex_config_lib.pp mangled_xdc_runtime_xdc_noinit_lib.pp mangled_xdc_runtime_Assert_lib.pp mangled_xdc_runtime_Core-mem_lib.pp mangled_xdc_runtime_Core-smem_lib.pp mangled_xdc_runtime_Core-label_lib.pp mangled_xdc_runtime_Core-params_lib.pp mangled_xdc_runtime_Diags_lib.pp mangled_xdc_runtime_Error_lib.pp mangled_xdc.runtime.Error_config_lib.pp mangled_xdc_runtime_Gate_lib.pp mangled_xdc_runtime_Log_lib.pp mangled_xdc_runtime_Memory_lib.pp mangled_xdc.runtime.Memory_config_lib.pp mangled_xdc_runtime_Registry_lib.pp mangled_xdc_runtime_Startup_lib.pp mangled_xdc_runtime_System_lib.pp mangled_xdc.runtime.System_config_lib.pp mangled_xdc_runtime_SysCallback_lib.pp mangled_xdc.runtime.SysCallback_config_lib.pp mangled_xdc_runtime_Text_lib.pp mangled_xdc_runtime_Timestamp_lib.pp mangled_xdc.runtime.Timestamp_config_lib.pp mangled_xdc.runtime.Memory_HeapProxy_config_lib.pp mangled_xdc.runtime.System_Module_GateProxy_config_lib.pp mangled_xdc.runtime.Timestamp_SupportProxy_config_lib.pp
	@echo Creating the SYS/BIOS library that contains the APIs not included in the ROM ...
	@$(CC) -o mangled__BIOS_lib.obj $(BIOS_DEFS) $(XOPTS) $(INCS) \
	mangled__BIOS_lib.pp \
	mangled_ti.sysbios.BIOS_config_lib.pp \
	mangled_ti.sysbios.BIOS_RtsGateProxy_config_lib.pp \
	mangled_knl_Clock_lib.pp \
	mangled_ti.sysbios.knl.Clock_config_lib.pp \
	mangled_knl_Idle_lib.pp \
	mangled_knl_Intrinsics_lib.pp \
	mangled_knl_Event_lib.pp \
	mangled_knl_Mailbox_lib.pp \
	mangled_ti.sysbios.knl.Mailbox_config_lib.pp \
	mangled_knl_Queue_lib.pp \
	mangled_ti.sysbios.knl.Queue_config_lib.pp \
	mangled_knl_Semaphore_lib.pp \
	mangled_ti.sysbios.knl.Semaphore_config_lib.pp \
	mangled_knl_Swi_lib.pp \
	mangled_knl_Swi_andn_lib.pp \
	mangled_ti.sysbios.knl.Swi_config_lib.pp \
	mangled_knl_Task_lib.pp \
	mangled_ti.sysbios.knl.Task_config_lib.pp \
	mangled_ti.sysbios.knl.Clock_TimerProxy_config_lib.pp \
	mangled_ti.sysbios.knl.Task_SupportProxy_config_lib.pp \
	mangled_arm_m3_Hwi_lib.pp \
	mangled_ti.sysbios.family.arm.m3.Hwi_config_lib.pp \
	mangled_arm_m3_TaskSupport_lib.pp \
	mangled_ti.sysbios.family.arm.m3.TaskSupport_config_lib.pp \
	mangled_arm_cc26xx_Boot_lib.pp \
	mangled_arm_cc26xx_Power_lib.pp \
	mangled_arm_cc26xx_Power_standbyPolicy_lib.pp \
	mangled_arm_cc26xx_Power_calibrateRCOSC_lib.pp \
	mangled_arm_cc26xx_pg2_leakage_workaround_lib.pp \
	mangled_arm_cc26xx_TimerGPT_lib.pp \
	mangled_arm_cc26xx_Timer_lib.pp \
	mangled_ti.sysbios.family.arm.cc26xx.Timer_config_lib.pp \
	mangled_arm_cc26xx_TimestampProvider_lib.pp \
	mangled_ti.sysbios.family.arm.cc26xx.TimestampProvider_config_lib.pp \
	mangled_hal_Hwi_lib.pp \
	mangled_hal_Hwi_stack_lib.pp \
	mangled_hal_Hwi_startup_lib.pp \
	mangled_ti.sysbios.hal.Hwi_config_lib.pp \
	mangled_ti.sysbios.hal.Hwi_HwiProxy_config_lib.pp \
	mangled_heaps_HeapMem_lib.pp \
	mangled_ti.sysbios.heaps.HeapMem_config_lib.pp \
	mangled_ti.sysbios.heaps.HeapMem_Module_GateProxy_config_lib.pp \
	mangled_gates_GateHwi_lib.pp \
	mangled_ti.sysbios.gates.GateHwi_config_lib.pp \
	mangled_gates_GateMutex_lib.pp \
	mangled_ti.sysbios.gates.GateMutex_config_lib.pp \
	mangled_xdc_runtime_xdc_noinit_lib.pp \
	mangled_xdc_runtime_Assert_lib.pp \
	mangled_xdc_runtime_Core-mem_lib.pp \
	mangled_xdc_runtime_Core-smem_lib.pp \
	mangled_xdc_runtime_Core-label_lib.pp \
	mangled_xdc_runtime_Core-params_lib.pp \
	mangled_xdc_runtime_Diags_lib.pp \
	mangled_xdc_runtime_Error_lib.pp \
	mangled_xdc.runtime.Error_config_lib.pp \
	mangled_xdc_runtime_Gate_lib.pp \
	mangled_xdc_runtime_Log_lib.pp \
	mangled_xdc_runtime_Memory_lib.pp \
	mangled_xdc.runtime.Memory_config_lib.pp \
	mangled_xdc_runtime_Registry_lib.pp \
	mangled_xdc_runtime_Startup_lib.pp \
	mangled_xdc_runtime_System_lib.pp \
	mangled_xdc.runtime.System_config_lib.pp \
	mangled_xdc_runtime_SysCallback_lib.pp \
	mangled_xdc.runtime.SysCallback_config_lib.pp \
	mangled_xdc_runtime_Text_lib.pp \
	mangled_xdc_runtime_Timestamp_lib.pp \
	mangled_xdc.runtime.Timestamp_config_lib.pp \
	mangled_xdc.runtime.Memory_HeapProxy_config_lib.pp \
	mangled_xdc.runtime.System_Module_GateProxy_config_lib.pp \
	mangled_xdc.runtime.Timestamp_SupportProxy_config_lib.pp \

sysbios.arm3: intro mangled__BIOS_lib.obj arm_m3_Hwi_asm_iar_lib.obj arm_m3_Hwi_asm_switch_iar_lib.obj arm_m3_IntrinsicsSupport_asm_iar_lib.obj arm_m3_TaskSupport_asm_iar_lib.obj
	@-$(call RM, $@)
	@$(AR) $@ $(filter-out intro,$^)
	@echo Library build complete.

clean:
	@$(DEL) ..\makefile.libs
