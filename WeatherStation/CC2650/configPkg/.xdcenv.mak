#
_XDCBUILDCOUNT = 0
ifneq (,$(findstring path,$(_USEXDCENV_)))
override XDCPATH = Z:/ti/tirtos_simplelink_2_11_01_09/packages;Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages;Z:/ti/tirtos_simplelink_2_11_01_09/products/cc26xxware_2_00_06_14829/inc;Z:/ti/tirtos_simplelink_2_11_01_09/products/cc26xxware_2_00_06_14829/driverlib;Z:/ti/tirtos_simplelink_2_11_01_09/products/cc26xxware_2_00_06_14829
override XDCROOT = z:/ti/xdctools_3_30_06_67_core
override XDCBUILDCFG = ./config.bld
endif
ifneq (,$(findstring args,$(_USEXDCENV_)))
override XDCARGS = 
override XDCTARGETS = 
endif
#
ifeq (0,1)
PKGPATH = Z:/ti/tirtos_simplelink_2_11_01_09/packages;Z:/ti/tirtos_simplelink_2_11_01_09/products/bios_6_41_02_41/packages;Z:/ti/tirtos_simplelink_2_11_01_09/products/cc26xxware_2_00_06_14829/inc;Z:/ti/tirtos_simplelink_2_11_01_09/products/cc26xxware_2_00_06_14829/driverlib;Z:/ti/tirtos_simplelink_2_11_01_09/products/cc26xxware_2_00_06_14829;z:/ti/xdctools_3_30_06_67_core/packages;..
HOSTOS = Windows
endif
