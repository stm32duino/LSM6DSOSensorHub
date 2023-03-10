/**
 ******************************************************************************
 * @file    SensorHubExample.ino
 * @author  Lorenzo Bini
 * @version V1.0.0
 * @date    February 2023
 * @brief   Library for interacting with the LSM6DSO Sensor Hub function
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2019 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */


/*

LSM6DSOHub Arduino Library
By Lorenzo Bini at STMicroelectronics

This library provides functionality for interacting with the Sensor Hub functionality on LSM6DSO sensors.

BOARD SETUP
If you are using a X-NUCLEO-IKS01A3 board, you must correctly wire up the sensor hub bus to the other sensors. To do this, short the middle two pins for each of the two jumper sets JP8 and JP7, which look like this:

JP8   * * * *
JP7   * * * *

Short as following:

JP8   * O-O *
JP7   * O-O *

* represents a pin
O-O represents a jumper and the two pins it is shorting

Note that with this setup, normal sensor operation will be unavailable unless specific library functionality is used

LIBRARY USAGE
1. Create LSM6DSOHub object by passing it a valid LSM6DSOSensor representing the sensor you wish to use as master
2. Make sure the master has been initialized by calling its begin function
3. Initialize the Sensor Hub by calling the resetMaster function
4. Call the addDevice function for each device you would like to read using the sensor hub, up to four (4) devices. Provide, in this order, the I2C address of the device, the register address you wish to read, and how many times you wish to read the device at each read operation. Note that not all devices automatically increment the register address when multiple reads are issued in one operation.
5. Call the begin function to star the sensor hub
6. In your loop function, call waitAndRead whenever you wish to extract a new set of data from your sensors. Note that this function will block until all sensors have been read. Alternatively, you may manually wait and extract data by using the ready and read functions.
7. To actually obtain the data related to one of your sensors, call getSensor. You must provide, in this order, the sensor number (for example, the first sensor that was added is sensor 0, the second sensor 1, and so on), and a pointer to a uint8_t array which is as long as the number of reads you set for that sensor.

For example: a sensor added second with addDevice (0xBE, 0xEF, 4) should be read, after calling waitAndRead, by calling getSensor (1, arr) where arr is a uint8_t arr[4] of length 4.

USING PASSTHROUGH FUNCTIONALITY
Before activating the sensor hub with begin(), the enablePassthrough() function may be used to gain normal I2C access to the sensors connected to the hub. Note that in this case, sensor hub operation is impossible. Before enabling the sensor hub, the passthrough feature MUST be disabled by calling disablePassthrough(). Passthrough functions MUST NOT be used once the sensor hub is operational. To regain use of these functions afterwards, the sensor must be fully reset using the resetMaster() function, after which it must be configured again.

*/

#include "LSM6DSOSensor.h"
#include "STTS751Sensor.h"
#include "LPS22HHSensor.h"
#include "LIS2MDLSensor.h"
#include "HTS221Sensor.h"
#include "LSM6DSOHub.h"

// Uncomment this to use the dedicated temperature sensor and disable the humidity sensor
//#define USE_STTS751

#ifdef ARDUINO_SAM_DUE
#define DEV_I2C Wire1
#elif defined(ARDUINO_ARCH_STM32)
#define DEV_I2C Wire
#elif defined(ARDUINO_ARCH_AVR)
#define DEV_I2C Wire
#else
#define DEV_I2C Wire
#endif

#define INT_1 4


typedef struct lin_t
{
  float x0;
  float y0;
  float x1;
  float y1;
} lin_t;


LSM6DSOSensor Master(&DEV_I2C);
LSM6DSOHub Hub(&Master);

LIS2MDLSensor Magneto(&DEV_I2C);
HTS221Sensor Humidity(&DEV_I2C);
LPS22HHSensor Pressure(&DEV_I2C);
STTS751Sensor Temp(&DEV_I2C);


uint8_t readObj (LSM6DSOSensor &m, uint8_t reg)
{
  uint8_t res = 0xff;
  m.Read_Reg(reg, &res);
  return res; 
}

void writeObj (LSM6DSOSensor &m, uint8_t reg, uint8_t data)
{
  m.Write_Reg(reg, data);
  return;
}


static float Linear_Interpolation(lin_t *Lin, float Coeff)
{
  return (((Lin->y1 - Lin->y0) * Coeff) + ((Lin->x1 * Lin->y0) - (Lin->x0 * Lin->y1))) / (Lin->x1 - Lin->x0);
}

lin_t humLine;

void setup()
{
  // Initialize serial for output.
  Serial.begin(115200);
  delay(1000);
  Serial.println("\r\n\r\n\r\n\r\n==== Arduino Sensor Hub Demo ====\r\n");
  // Initialize I2C bus.
  DEV_I2C.begin();

  Master.begin();
  
  Hub.resetMaster();

  // Enable passthrough
  Hub.enablePassthrough();

  Magneto.begin();
  Magneto.Enable();
  Humidity.begin();
  Humidity.Enable();
  Pressure.begin();
  Pressure.Enable();
  Temp.begin();
  Temp.Enable();

  uint8_t rh0;
  uint8_t rh1;
  uint8_t lsb0r [2];
  uint8_t lsb1r [2];
  int16_t lsb0;
  int16_t lsb1;

  Humidity.ReadReg(0x30, &rh0);
  Humidity.ReadReg(0x31, &rh1);
  Humidity.ReadReg(0x36, &lsb0r[0]);
  Humidity.ReadReg(0x37, &lsb0r[1]);
  Humidity.ReadReg(0x3a, &lsb1r[0]);
  Humidity.ReadReg(0x3b, &lsb1r[1]);

  lsb0 = (int16_t) ((lsb0r[1] << 8) | lsb0r[0]);
  lsb1 = (int16_t) ((lsb1r[1] << 8) | lsb1r[0]);

  humLine.x0 = lsb0 * 1.0f;
  humLine.x1 = lsb1 * 1.0f;
  humLine.y0 = rh0 / 2.0f;
  humLine.y1 = rh1 / 2.0f;

  // Enable sensor suite
  Master.Enable_X();
  Master.Enable_G();

  Hub.disablePassthrough();
  
  delay(100);

  // Prepare the sensor hub
  Hub.addDevice (0x3D, 0x68, 6);                  // Magnetometer sensor LIS2MDL
  Hub.addDevice (0xBB, 0x28, 5);                  // Pressure sensor LPS22H
#ifndef USE_STTS751
  Hub.addDevice (0xBF, (0x28 | 0x80U), 2);        // Humidity sensor HTS221
#else
  Hub.addDevice (0x95, 0x00, 1);                  // Temperature sensor STTS751 (Note: this has no auto-increment so it will read the same register 3 times)
  Hub.addDevice (0x95, 0x02, 1);                  // Temperature sensor STTS751 (Note: this has no auto-increment so it will read the same register 3 times)
#endif
  // Engage master hub mode
  Hub.begin();

  Serial.println("Ready.");
  
  delay(1000);
}

void loop()
{
  Hub.waitAndRead();

  Serial.println("\r\n\r\n----------\r\n");
  Hub.read();

  // Gyro and acc
  int32_t acc[3];
  int32_t gyr[3];

  Master.Get_X_Axes(acc);
  Master.Get_G_Axes(gyr);
  Serial.println("LSM6DSO:");
  Serial.print("Accelerometer =");
  Serial.println(String(acc[0]) + " - " + String(acc[1]) + " - " + String(acc[2]));
  Serial.print("Gyroscope =");
  Serial.println(String(gyr[0]) + " - " + String(gyr[1]) + " - " + String(gyr[2]));
  Serial.println("");

  // Magnetometer
  uint32_t dataMag [6];

  Hub.getSensor (0, dataMag);

  // We have the data, now do concats to produce the full values
  int16_t x = (dataMag[1] << 8) | dataMag[0];
  int16_t y = (dataMag[3] << 8) | dataMag[2];
  int16_t z = (dataMag[5] << 8) | dataMag[4];
  int magX = (int) x;
  int magY = (int) y;
  int magZ = (int) z;

  // Apply sensitivity
  magX = (int) (magX * 1.5);
  magY = (int) (magY * 1.5);
  magZ = (int) (magZ * 1.5);

  Serial.println("LIS2MDL:");
  Serial.print("Mag = ");
  Serial.println(String(magX) + " - " + String(magY) + " - " + String(magZ));
  Serial.println("");

  // Pressure and temperature sensor
  uint32_t dataPress [5];
  Hub.getSensor(1, dataPress);

  int32_t press = (dataPress[2] << 16) | (dataPress[1] << 8) | dataPress[0];
  float fpress = (((float)press) / 4096.0f);

  Serial.println("LPS22HH:");
  Serial.println("Pressure = " + String(fpress));

  // As temperature
  int16_t pressTemp = (dataPress[4] << 8) | dataPress[3];
  float t = ((float) pressTemp) / 100;

  Serial.println("Temperature = " + String(t));    
  Serial.println("");

#ifdef USE_STTS751
  // Temperature Sensor
  uint32_t dataTempHi [1];
  Hub.getSensor (2, dataTempHi);
  uint32_t dataTempLo [1];
  Hub.getSensor (3, dataTempLo);

  uint32_t tempRaw = (dataTempHi[0] << 8 ) | dataTempLo[0];
  int tempInt = (int) tempRaw;
  float temp = (float) tempInt;
  temp = temp / 256.0f;

  Serial.println("STTS751:");
  Serial.println("Temperature = " + String(temp));
  Serial.println("");
#else
  // Humidity sensor
  uint32_t dataHum [2];
  Hub.getSensor (2, dataHum);

  int16_t hum = (int16_t) ((dataHum[1] << 8) | dataHum[0]);

  float fhum = (float) hum;
  float rh = Linear_Interpolation(&humLine, fhum);

  Serial.println("HTS221:");
  Serial.println("% Humidity = " + String(rh));
  Serial.println("");
#endif

  delay(1000);
}
