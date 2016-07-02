## THIS IS A GENERATED FILE -- DO NOT EDIT
.configuro: .libraries,rm3 linker.cmd package/cfg/app_prm3.orm3

# To simplify configuro usage in makefiles:
#     o create a generic linker command file name 
#     o set modification times of compiler.opt* files to be greater than
#       or equal to the generated config header
#
linker.cmd: package/cfg/app_prm3.xdl
	$(SED) 's"^\"\(package/cfg/app_prm3cfg.cmd\)\"$""\"Z:/ti/simplelink/zstack_home_1_02_02a_44539/Projects/zstack/HomeAutomation/WeatherStation/CC2650/configPkg/\1\""' package/cfg/app_prm3.xdl > $@
	-$(SETDATE) -r:max package/cfg/app_prm3.h compiler.opt compiler.opt.defs
