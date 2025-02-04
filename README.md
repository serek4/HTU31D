# HTU31D sensor driver
Non blocking Arduino library for HTU31D sensor

## Constructor
````c++
HTU31D_DEFAULT_ADDR    // Default I2C address for HTU31D
HTU31D_ALT_ADDR        // Alternative I2C address for HTU31D
HTU31D::HTU31D(TwoWire* theWire = &Wire, uint8_t address = HTU31D_DEFAULT_ADDR);
````
## Sensor initialization
````c++
bool HTU31D::begin(bool begin = true);
````
## Commands
````c++
void     HTU31D::setResolution(uint8_t tempRes, uint8_t humiRes);
bool     HTU31D::reset();
uint32_t HTU31D::readSerial();
uint8_t  HTU31D::readDiagnostic();
bool     HTU31D::enableHeater(bool enable);
````
## Data read flow

````c++
#include <HTU31D.h>

HTU31D htu31;

void setup() {
    htu31.begin();
}

void loop() {
    // request conversion
    htu31.requestData();
    // if conversion is done
    if (htu31.available()) {
        // read temperature and humidity
        htu31.readData();
        // or read temperature only
        htu31.readTemp();
        // or read humidity only
        htu31.readHumi();
        // get readings
        Serial.printf("Temperature: %6.2f°C\n", htu31.getTemperature());
        Serial.printf("Humidity:    %6.2f%%RH\n", htu31.getHumidity());
        Serial.printf("Dew point:   %6.2f°C\n", htu31.getDewPoint());
    }
}
````