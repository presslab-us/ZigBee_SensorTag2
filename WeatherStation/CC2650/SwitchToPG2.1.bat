REM This file switches the IAR ARGVARS to point to PG2.1 TI RTOS/Driverlib for the ZStack Library Project
RMDIR configPkg /s /q
RMDIR settings /s /q
RMDIR src /s /q
RMDIR SensorTag /s /q
copy ..\..\..\Core\Thread\ZStackpg2.1.custom_argvars  .\SensorTag.custom_argvars