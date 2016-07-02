# invoke SourceDir generated makefile for app.prm3
app.prm3: .libraries,app.prm3
.libraries,app.prm3: package/cfg/app_prm3.xdl
	$(MAKE) -f Z:\ti\simplelink\zstack_home_1_02_02a_44539\Projects\zstack\HomeAutomation\WeatherStation\CC2650/src/makefile.libs

clean::
	$(MAKE) -f Z:\ti\simplelink\zstack_home_1_02_02a_44539\Projects\zstack\HomeAutomation\WeatherStation\CC2650/src/makefile.libs clean

