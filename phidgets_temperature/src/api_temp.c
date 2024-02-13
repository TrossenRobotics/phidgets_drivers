#include <phidget22.h>
#include <stdio.h>
#include <unistd.h>
#define Sleep(x) usleep((x) * 1000)

//Declare any event handlers here. These will be called every time the associated event occurs.

static void CCONV onTemperatureChange(PhidgetTemperatureSensorHandle ch, void * ctx, double temperature) {
	printf("Temperature: %lf\n", temperature);
}

int main() {
	//Declare your Phidget channels and other variables
	PhidgetTemperatureSensorHandle temperatureSensor0;

	//Register a network server that your program will try to connect to.
	PhidgetNet_addServer("serverName", "192.168.1.1", 5661, "", 0);

	//Create your Phidget channels
	PhidgetTemperatureSensor_create(&temperatureSensor0);

	//Set addressing parameters to specify which channel to open (if any)
	Phidget_setHubPort((PhidgetHandle)temperatureSensor0, 3);
	Phidget_setIsRemote((PhidgetHandle)temperatureSensor0, 1);
	Phidget_setDeviceSerialNumber((PhidgetHandle)temperatureSensor0, 664919);

	//Assign any event handlers you need before calling open so that no events are missed.
	PhidgetTemperatureSensor_setOnTemperatureChangeHandler(temperatureSensor0, onTemperatureChange, NULL);

	//Open your Phidgets and wait for attachment
	Phidget_openWaitForAttachment((PhidgetHandle)temperatureSensor0, 20000);

	//Do stuff with your Phidgets here or in your event handlers.

	Sleep(5000);

	//Close your Phidgets once the program is done.
	Phidget_close((PhidgetHandle)temperatureSensor0);

	PhidgetTemperatureSensor_delete(&temperatureSensor0);
}