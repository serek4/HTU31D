#include <HTU31D.h>

HTU31D htu31;
uint32_t readTimer;
bool enableHeater = false;

void setup() {
	Serial.begin(115200);
	Serial.println();

	htu31.setResolution(HTU31D::res_0, HTU31D::res_0);
	if (!htu31.begin()) {
		Serial.println("sensor init fail!");
		while (1);
	}
	Serial.printf("sensor serial: 0x%06X\n", htu31.readSerial());
}

void loop() {
	// every 5 seconds
	if ((millis() - readTimer) > 5000) {
		readTimer = millis();
		enableHeater = !enableHeater;
		if (htu31.enableHeater(enableHeater)) {
			Serial.printf("Heater %s\n", enableHeater ? "ON" : "OFF");
		} else {
			Serial.println("Heater cmd fail");
		}
		htu31.requestData();
	}
	if (htu31.available()) {
		htu31.readData();
		Serial.printf("Temperature: %6.2f°C\n", htu31.getTemperature());
		Serial.printf("Humidity:    %6.2f%%RH\n", htu31.getHumidity());
		Serial.printf("Dew point:   %6.2f°C\n", htu31.getDewPoint());
		Serial.printf("Diag data:    0x%02X\n", htu31.readDiagnostic());
		Serial.println();
	}
}