/*
MIT License

Copyright (c) 2025 serek4

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#ifndef HTU31D_H
#define HTU31D_H

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <unordered_map>

// clang-format off
#define HTU31D_DEFAULT_ADDR    0x40    // Default I2C address for HTU31D
#define HTU31D_ALT_ADDR        0x41    // Alternative I2C address for HTU31D

#define HTU31D_CONVERSION      0x40    // Start conversion
#define HTU31D_READ_TEMP_HUMI  0x00    // Read temperature and humidity
#define HTU31D_READ_HUMI       0x10    // Read humidity
#define HTU31D_RESET           0x1E    // Reset
#define HTU31D_HEATER_ON       0x04    // Enable heater
#define HTU31D_HEATER_OFF      0x02    // Disable heater
#define HTU31D_READ_SERIAL     0x0A    // Read serial number command
#define HTU31D_READ_DIAGNOSTIC 0x08    // Read diagnostic command

#define HTU31D_CONVERSION_TIME 21      // max res temp 12.1ms + humi 7.8ms

#define HTU31D_CRC_LEN         1       // crc length in bytes
#define HTU31D_TEMP_DATA_LEN   2       // temperature data length in bytes
#define HTU31D_HUMI_DATA_LEN   2       // humidity data length in bytes
#define HTU31D_DATA_LEN        6       // data length in bytes (temp + crc + humi + crc)
#define HTU31D_SERIAL_LEN      3       // serial number data length in bytes
#define HTU31D_DIAGNOSTIC_LEN  1       // diagnostic data length in bytes

#define HTU31D_TEMP_CONV_TIMES {{res_0, 2}, {res_1, 4}, {res_2, 7}, {res_3, 13}}    // temperature conversion time in ms
#define HTU31D_HUMI_CONV_TIMES {{res_0, 1}, {res_1, 2}, {res_2, 4}, {res_3, 8}}     // humidity conversion time in ms
// clang-format on

using ConversionTime = std::unordered_map<uint8_t, uint8_t>;

class HTU31D {
  private:
	TwoWire* _i2c;
	uint8_t _address;
	uint8_t _tempRes;
	uint8_t _humiRes;
	u_long _requestTimer;
	ConversionTime _tempConvTime;
	ConversionTime _humiConvTime;
	bool _dataRequested;
	float _temperature;
	float _humidity;
	float _dewPoint;

	bool _sendCmd(uint8_t cmd);
	bool _readData(uint8_t* data, size_t len);
	float _calcDewPoint(float temp, float humi);

  public:
	enum Resolution {
		res_0,    // humi: `0.020`%RH, temp: `0.040`°C
		res_1,    // humi: `0.014`%RH, temp: `0.025`°C
		res_2,    // humi: `0.010`%RH, temp: `0.016`°C
		res_3,    // humi: `0.007`%RH, temp: `0.012`°C
	};

	HTU31D(TwoWire* theWire = &Wire, uint8_t address = HTU31D_DEFAULT_ADDR);

	bool begin(bool begin = true);
	void setResolution(uint8_t tempRes, uint8_t humiRes);
	bool reset();
	uint32_t readSerial();
	uint8_t readDiagnostic();
	bool enableHeater(bool enable);
	bool requestData();
	bool available();
	bool readData();
	bool readTemp();
	bool readHumi();
	float getTemperature();
	float getHumidity();
	float getDewPoint();
	static uint8_t crc(uint32_t data, size_t len, uint8_t crc = 0);
};

#endif
