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

#include "HTU31D.h"

HTU31D::HTU31D(TwoWire* wire, uint8_t address)
    : _i2c(wire)
    , _address(address)
    , _tempRes(res_3)
    , _humiRes(res_3)
    , _requestTimer(0)
    , _tempConvTime(HTU31D_TEMP_CONV_TIMES)
    , _humiConvTime(HTU31D_HUMI_CONV_TIMES)
    , _dataRequested(false)
    , _temperature(NAN)
    , _humidity(NAN)
    , _dewPoint(NAN) {
}
/*
* initialize sensor
* @param begin call Wire.begin() or not
*/
bool HTU31D::begin(bool begin) {
	if (begin) { _i2c->begin(); }
	if (!reset()) { return false; }
	if (readSerial() == 0) { return false; }
	setResolution(_tempRes, _humiRes);
	return true;
}

/*
* change sensor resolution
* ```
* res_0 = humi: 0.020 %RH, temp: 0.040 °C
* res_1 = humi: 0.014 %RH, temp: 0.025 °C
* res_2 = humi: 0.010 %RH, temp: 0.016 °C
* res_3 = humi: 0.007 %RH, temp: 0.012 °C
* ```
* @param tempRes temperature resolution, default `res_3`
* @param tempRes humidity resolution, default `res_3`
*/
void HTU31D::setResolution(uint8_t tempRes, uint8_t humiRes) {
	_tempRes = tempRes;
	_humiRes = humiRes;
}

/*
* reboot sensor
*/
bool HTU31D::reset() {
	if (!_sendCmd(HTU31D_RESET)) { return false; }
	delay(15);
	return true;
}

/*
* read sensor serial number
*/
uint32_t HTU31D::readSerial() {
	uint8_t buffer[HTU31D_SERIAL_LEN + HTU31D_CRC_LEN];
	uint32_t serial = 0x00FFFFFF;

	if (!_sendCmd(HTU31D_READ_SERIAL)) { return serial; }
	if (!_readData(buffer, HTU31D_SERIAL_LEN + HTU31D_CRC_LEN)) { return serial; }

	serial = (buffer[0] << 16) | (buffer[1] << 8) | buffer[2];
	if (crc(serial, HTU31D_SERIAL_LEN) != buffer[3]) { return 0x00FFFFFF; }

	return serial;
}

/*
* read sensor diagnostic register
*/
uint8_t HTU31D::readDiagnostic() {
	uint8_t buffer[HTU31D_DIAGNOSTIC_LEN + HTU31D_CRC_LEN];
	uint8_t diagnostic = 0xFF;

	if (!_sendCmd(HTU31D_READ_DIAGNOSTIC)) { return diagnostic; }
	if (!_readData(buffer, HTU31D_DIAGNOSTIC_LEN + HTU31D_CRC_LEN)) { return diagnostic; }
	if (crc(buffer[0], HTU31D_DIAGNOSTIC_LEN) != buffer[1]) { return diagnostic; }

	diagnostic = buffer[0];

	return diagnostic;
}

/*
* enable/disable sensor heater
* @param enable `true` = ON, `false` = OFF
*/
bool HTU31D::enableHeater(bool enable) {
	uint8_t cmd;
	if (enable) {
		cmd = HTU31D_HEATER_ON;
	} else {
		cmd = HTU31D_HEATER_OFF;
	}
	return _sendCmd(cmd);
}

/*
* request conversion
*/
bool HTU31D::requestData() {
	if (_dataRequested) { return false; }

	uint8_t cmd = HTU31D_CONVERSION | (_humiRes << 3) | (_tempRes << 1);
	if (!_sendCmd(cmd)) { return false; }
	_requestTimer = millis();
	_dataRequested = true;
	return true;
}

/*
* check if sensor conversion is done
*/
bool HTU31D::available() {
	if (_dataRequested && millis() - _requestTimer > _tempConvTime.at(_tempRes) + _humiConvTime.at(_humiRes)) {
		return true;
	}
	return false;
}

/*
* read temperature and humidity from sensor
*/
bool HTU31D::readData() {
	_dataRequested = false;
	if (!_sendCmd(HTU31D_READ_TEMP_HUMI)) { return false; }

	uint8_t buffer[HTU31D_DATA_LEN];
	if (!_readData(buffer, HTU31D_DATA_LEN)) { return false; }

	uint16_t rawTemp = (buffer[0] << 8) | buffer[1];
	if (crc(rawTemp, HTU31D_TEMP_DATA_LEN) != buffer[2]) { return false; }
	_temperature = rawTemp / 65535.0F * 165 - 40;

	uint16_t rawHumi = (buffer[3] << 8) | buffer[4];
	if (crc(rawHumi, HTU31D_HUMI_DATA_LEN) != buffer[5]) { return false; }
	_humidity = rawHumi / 65535.0F * 100;

	_dewPoint = _calcDewPoint(_temperature, _humidity);

	return true;
}

/*
* read temperature from sensor
*/
bool HTU31D::readTemp() {
	_dataRequested = false;
	if (!_sendCmd(HTU31D_READ_TEMP_HUMI)) { return false; }

	uint8_t buffer[HTU31D_TEMP_DATA_LEN + HTU31D_CRC_LEN];
	if (!_readData(buffer, HTU31D_TEMP_DATA_LEN + HTU31D_CRC_LEN)) { return false; }

	uint16_t rawTemp = (buffer[0] << 8) | buffer[1];
	if (crc(rawTemp, HTU31D_TEMP_DATA_LEN) != buffer[2]) { return false; }
	_temperature = rawTemp / 65535.0F * 165 - 40;

	return true;
}

/*
* read humidity from sensor
*/
bool HTU31D::readHumi() {
	_dataRequested = false;
	if (!_sendCmd(HTU31D_READ_HUMI)) { return false; }

	uint8_t buffer[HTU31D_HUMI_DATA_LEN + HTU31D_CRC_LEN];
	if (!_readData(buffer, HTU31D_HUMI_DATA_LEN + HTU31D_CRC_LEN)) { return false; }

	uint16_t rawHumi = (buffer[0] << 8) | buffer[1];
	if (crc(rawHumi, HTU31D_HUMI_DATA_LEN) != buffer[2]) { return false; }
	_humidity = rawHumi / 65535.0F * 100;

	return true;
}

/*
* get temperature in °C
*/
float HTU31D::getTemperature() {
	return _temperature;
}

/*
* get relative humidity in %
*/
float HTU31D::getHumidity() {
	return _humidity;
}

/*
* get dew point in °C
*/
float HTU31D::getDewPoint() {
	return _dewPoint;
}

/*
* crc is the same as in HTU21D sensor
* this is modified version of
* `https://github.com/sparkfun/SparkFun_HTU21D_Breakout_Arduino_Library/blob/3886ebd95291e223ec3319da37eaaf3eb0e7e47b/src/SparkFunHTU21D.cpp#L172`
* @param data data for crc calculations
* @param len data length in bytes
* @param crc crc to compare with calculated crc
* @return if `crc` param = crc to compare with, `returns 0` when valid,
* if `crc` param = 0 returns calculated data crc
*/
uint8_t HTU31D::crc(uint32_t data, size_t len, uint8_t crc) {
	uint8_t crcLen = 8;
	uint8_t dataLen = len * 8;
	uint16_t polynomial = 0x0131;                                // x^8 + x^5 + x^4 + 1
	uint32_t divisor = (uint32_t)polynomial << (dataLen - 1);    // shifted polynomial to align to data + crc len
	uint32_t remainder = data << crcLen;                         // shift data left by 8 bits because we have to add crc at the end
	remainder |= crc;                                            // add crc

	for (int i = dataLen + crcLen - 1; i >= crcLen; i--) {    // Check data bits, the remaining 8 are our remainder and should be zero when we're done.
		if (remainder & (uint32_t)1 << i) {                   // Check if there is a one in the left position
			remainder ^= divisor;                             // XOR divisor if true
		}
		divisor >>= 1;    // shift divisor 1 bit to left and reapeat for all data bits
	}
	return static_cast<uint8_t>(remainder);
}

bool HTU31D::_sendCmd(uint8_t cmd) {
	_i2c->beginTransmission(_address);
	_i2c->write(cmd);
	return _i2c->endTransmission(1U) == 0;
}

bool HTU31D::_readData(uint8_t* data, size_t len) {
	if (_i2c->requestFrom(_address, len, true) == 0) { return false; }

	for (uint8_t i = 0; i < len; i++) {
		data[i] = _i2c->read();
	}
	return true;
}

float HTU31D::_calcDewPoint(float temp, float humi) {
	float a = 8.1332, b = 1762.39, c = 235.66;
	float pp = pow(10, a - (b / (temp + c)));
	return -((b / (log10(humi * (pp / 100)) - a)) + c);
}
