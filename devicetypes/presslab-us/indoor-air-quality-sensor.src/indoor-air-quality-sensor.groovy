/**
 *  Indoor Air Quality Sensor
 *
 *  Copyright 2015 Ryan Press
 *
 *  Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except
 *  in compliance with the License. You may obtain a copy of the License at:
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software distributed under the License is distributed
 *  on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License
 *  for the specific language governing permissions and limitations under the License.
 *
 */
metadata {
	definition (name: "Indoor Air Quality Sensor", namespace: "presslab-us", author: "Ryan Press") {
		capability "Carbon Monoxide Detector"
		capability "Illuminance Measurement"
		capability "Relative Humidity Measurement"
		capability "Sensor"
		capability "Temperature Measurement"
		capability "Configuration"
        capability "Refresh"
        
        command "calibrate"

		attribute "dust", "Number"
		attribute "gasCO", "Number"
		attribute "gasNO2", "Number"
		attribute "gasNH3", "Number"
		attribute "smog", "Number"
		attribute "airPressure", "Number"
        
		fingerprint profileId: "0104", deviceId: "0277", inClusters: "0000,0003,0277,0278", outClusters: "0003"
	}

	simulator {
		// TODO: define status and reply messages here
	}

	tiles {
		valueTile("temperature", "device.temperature", width: 1, height: 1) {
			state("temperature", label:'${currentValue}°', unit:"F",
				backgroundColors:[
					[value: 31, color: "#153591"],
					[value: 44, color: "#1e9cbb"],
					[value: 59, color: "#90d2a7"],
					[value: 74, color: "#44b621"],
					[value: 84, color: "#f1d801"],
					[value: 95, color: "#d04e00"],
					[value: 96, color: "#bc2323"]
				]
			)
		}

		valueTile("humidity", "device.humidity", width: 1, height: 1) {
			state("humidity", label:'${currentValue}% RH', unit:"% RH")
		}

		valueTile("airPressure", "device.airPressure", width: 1, height: 1) {
			state("airPressure", label:'${currentValue} kPa', unit:"kPa")
		}

		valueTile("illuminance", "device.illuminance", width: 1, height: 1) {
			state("illuminance", label:'${currentValue} Lux', unit:"lux")
		}

		valueTile("dust", "device.dust", width: 1, height: 1) {
			state("dust", label:'${currentValue} Pcs/min', unit:"pcs", inactiveLabel: false)
		}

		valueTile("gasCO", "device.gasCO", width: 1, height: 1) {
			state("gasCO", label:'${currentValue} PPM CO', unit:"ppm", inactiveLabel: false)
		}

		valueTile("gasNO2", "device.gasNO2", width: 1, height: 1) {
			state("gasNO2", label:'${currentValue} PPM NO2', unit:"ppm", inactiveLabel: false)
		}

		valueTile("gasNH3", "device.gasNH3", width: 1, height: 1) {
			state("gasNH3", label:'${currentValue} PPM NH3', unit:"ppm", inactiveLabel: false)
		}

		standardTile("refresh", "device.temperature", decoration: "flat") {
			state "default", action:"refresh.refresh", icon:"st.secondary.refresh"
		}

		main "temperature"
		details(["temperature", "humidity", "airPressure", "dust", "gasCO", "gasNO2", "gasNH3", "illuminance"/*, "refresh"*/])
	}
}

// parse events into attributes
def parse(String description) {
    Map map = [:]
	Map msg = [:]

   	log.trace "description: $description"
	if (description?.startsWith('catchall:')) {     
     	def catchall = zigbee.parseDescriptionAsMap(description)
   		//log.trace "catchall: $catchall"

	    if (catchall.profileId != "0104" || (catchall.command != "00" && catchall.command != "01")) return
		msg.cluster = catchall.clusterId
        msg.value = catchall.data[-2..-1].reverse().collect().join()
        msg.attrId = catchall.data[0..1].reverse().collect().join()
    } else if (description?.startsWith('read attr -')) {
		msg = zigbee.parseDescriptionAsMap(description)
	} else if (description?.startsWith('temperature:')) {     
		msg.cluster = "0402"
    	msg.value = String.format("%04X", (Float.parseFloat(description[-5..-1]) * 100).toInteger())
	} else if (description?.startsWith('illuminance:')) {
		msg.cluster = "0400"
    	msg.value = String.format("%04X", Float.parseFloat(description[13..-1]).toInteger())
	} else if (description?.startsWith('humidity:')) {
		msg.cluster = "0405"
    	msg.value = String.format("%04X", (Float.parseFloat(description[-6..-2]) * 100).toInteger())
    }

   	//log.trace "parsed: $msg"
	def value = Integer.parseInt(msg.value, 16)

	switch(msg.cluster) {
    case "0400": // Illuminance
        map = getIlluminanceResult(value)
       	break
    case "0402": // Temperature
        map = getTemperatureResult(value)
       	break
	case "0403": // Pressure
		if (msg.attrId != "0010") return
        map = getPressureResult(value / 100.0)
       	break
	case "0405": // Humidity
        map = getHumidityResult(value)
       	break
	case "0277": // Dust
		if (msg.attrId == "0000") {
			map = getDustResult(value, 0)
		} else if (msg.attrId == "0001") {
			map = getDustResult(value, 1)
		}
        break
	case "0278": // Gas
		if (msg.attrId == "0000") {
			map = getGasResult(value, 0)
		} else if (msg.attrId == "0001") {
			map = getGasResult(value, 1)
		} else if (msg.attrId == "0002") {
			map = getGasResult(value, 2)
		}
        break
	}
	// TODO: handle 'carbonMonoxide' attribute
	// TODO: handle 'sensor' attribute
	// TODO: handle 'smog' attribute
    log.debug "Event: $map"
    return map ? createEvent(map) : null
}

def refresh() {
	log.debug("Refresh.");
    
    [
		"st rattr 0x${device.deviceNetworkId} 9 0x0402 0x0000", "delay 2000", // Temperature
		"st rattr 0x${device.deviceNetworkId} 9 0x0405 0x0000", "delay 2000", // Humidity

		"st rattr 0x${device.deviceNetworkId} 10 0x0403 0x0010", "delay 2000", // Pressure

		"st rattr 0x${device.deviceNetworkId} 11 0x0400 0x0000", "delay 2000", // Illuminance

		"st rattr 0x${device.deviceNetworkId} 12 0x0277 0x0000", "delay 2000", // Dust 1um
		"st rattr 0x${device.deviceNetworkId} 12 0x0277 0x0001", "delay 2000", // Dust 2.5um

		"st rattr 0x${device.deviceNetworkId} 12 0x0278 0x0000", "delay 2000", // Gas NH3
		"st rattr 0x${device.deviceNetworkId} 12 0x0278 0x0001", "delay 2000", // Gas CO
		"st rattr 0x${device.deviceNetworkId} 12 0x0278 0x0002", "delay 2000", // Gas NO2
	]
}

def calibrate() {
	log.debug "Calibrate gas sensor"
	"st cmd 0x${device.deviceNetworkId} 12 0x278 1 {}"
}

def configure() {
	state.dustCtr = 0
	state.dustInt = 0

	log.debug("Binding endpoints and configuring reporting.");

	[
		// Temperature Reporting, min 30s, max 360s, 0.10 deg C change
		"zdo bind 0x${device.deviceNetworkId} 9 1 0x0402 {${device.zigbeeId}} {}", "delay 2500",
        "zcl global send-me-a-report 0x0402 0x0000 0x29 30 360 {0A00}", "delay 200",
        "send 0x${device.deviceNetworkId} 1 9", "delay 2500",

		// Humidity Reporting, min 30s, max 360s, 2.00 % change
		"zdo bind 0x${device.deviceNetworkId} 9 1 0x0405 {${device.zigbeeId}} {}", "delay 2500",
        "zcl global send-me-a-report 0x0405 0x0000 0x21 30 360 {C800}", "delay 200",
        "send 0x${device.deviceNetworkId} 1 9", "delay 2500",

		// Air Pressure Reporting, min 60s, max 720s, 1.00 kPa change
		"zdo bind 0x${device.deviceNetworkId} 10 1 0x0403 {${device.zigbeeId}} {}", "delay 2500",
        "zcl global send-me-a-report 0x0403 0x0010 0x29 60 720 {6400}", "delay 200",
        "send 0x${device.deviceNetworkId} 1 10", "delay 2500",

		// Illuminance Reporting, min 1s, max 360s, 1000 unit change
		"zdo bind 0x${device.deviceNetworkId} 11 1 0x0400 {${device.zigbeeId}} {}", "delay 2500",
        "zcl global send-me-a-report 0x0400 0x0000 0x21 1 360 {E803}", "delay 200",
        "send 0x${device.deviceNetworkId} 1 11", "delay 2500",

		// Dust Reporting, min 60s, max 60s, 0.1 pcs/0.01 cuft change
		"zdo bind 0x${device.deviceNetworkId} 12 1 0x0277 {${device.zigbeeId}} {}", "delay 2500",
        "zcl global send-me-a-report 0x0277 0x0000 0x21 60 60 {0100}", "delay 200",
        "send 0x${device.deviceNetworkId} 1 12", "delay 2500",
//        "zcl global send-me-a-report 0x0277 0x0001 0x21 60 60 {0100}", "delay 200",
//        "send 0x${device.deviceNetworkId} 1 12", "delay 2500",

		// Gas Reporting, min 30s, max 30s, 10 ppm change
		"zdo bind 0x${device.deviceNetworkId} 12 1 0x0278 {${device.zigbeeId}} {}", "delay 2500",
        "zcl global send-me-a-report 0x0278 0x0000 0x21 30 30 {0A00}", "delay 200",
        "send 0x${device.deviceNetworkId} 1 12", "delay 2500",
//        "zcl global send-me-a-report 0x0278 0x0001 0x21 30 30 {0A00}", "delay 200",
//        "send 0x${device.deviceNetworkId} 1 12", "delay 2500",
//        "zcl global send-me-a-report 0x0278 0x0002 0x21 30 30 {0A00}", "delay 200",
//        "send 0x${device.deviceNetworkId} 1 12", "delay 2500",
	]
}

private Map getDustResult(value, type) {
    def typeText = type ? "2.5um" : "1um"
	def descriptionText = "Dust ${typeText} is ${value} pcs per 0.01 cuft"

	if (type == 1) {
        state.dustCtr = state.dustCtr + 1
        state.dustInt = state.dustInt + device.currentState("dust0")?.value.toInteger() + device.currentState("dust1")?.value.toInteger()
        if (state.dustCtr == 10) {
        	def temp = state.dustInt / 10.0
			sendEvent(name: "dust", descriptionText: "Average dust is ${temp} pcs per 0.01 cuft", value: temp)
        	state.dustCtr = 0
        	state.dustInt = 0
        }
	}

    return [
		name: "dust$type",
		value: value,
		descriptionText: descriptionText
	]
}

private Map getGasResult(value, type) {
    def temp = round(value / 10.0, 1)
    def typeText = (type == 0) ? "NH3" : (type == 1) ? "CO" : "NO2"
	def descriptionText = "Gas ${typeText} is ${temp}ppm"
	return [
		name: "gas${typeText}",
		value: temp,
		descriptionText: descriptionText
	]
}

private Map getTemperatureResult(value) {
	def linkText = getLinkText(device)
	def celsius = value / 100.0
	def temp
    if(getTemperatureScale() == "C"){
		temp = round(celsius, 1)
	} else {
		temp = round(celsiusToFahrenheit(celsius), 1)
	}
	if (tempOffset) {
		def offset = tempOffset as int
		def v = temp as int
		temp = v + offset
	}
	def descriptionText = "${linkText} was ${temp}°${temperatureScale}"

	return [
		name: 'temperature',
		value: temp,
		descriptionText: descriptionText
	]
}

private Map getPressureResult(value) {
	def linkText = getLinkText(device)
    def temp = round(value, 2)
	def descriptionText = "${linkText} was ${temp}kPa"

	return [
		name: 'airPressure',
		value: temp,
		descriptionText: descriptionText
	]
}

private Map getHumidityResult(value) {
	def linkText = getLinkText(device)
    def temp = round(value / 100.0, 1)
	def descriptionText = "${linkText} was ${temp}%RH"

	return [
		name: 'humidity',
		value: temp,
		descriptionText: descriptionText
	]
}

private Map getIlluminanceResult(value) {
	def linkText = getLinkText(device)
    def temp
    if (value == 0) {
    	temp = 0 //too low to be measured
    } else if (value == 65535) {
    	return null // invalid measurement
    } else {
		temp = Math.pow(10, (value - 1) / 10000.0).toInteger()
	}
    
    def descriptionText = "${linkText} was ${temp}lux"

	return [
		name: 'illuminance',
		value: temp,
		descriptionText: descriptionText
	]
}

private static float round(float d, int decimalPlace) {
	return BigDecimal.valueOf(d).setScale(decimalPlace,BigDecimal.ROUND_HALF_UP).floatValue();
}
