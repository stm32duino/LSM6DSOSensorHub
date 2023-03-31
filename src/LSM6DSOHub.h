/**
 ******************************************************************************
 * @file    LSM6DSOHub.h
 * @author  Lorenzo Bini
 * @version V1.1.0
 * @date    March 2023
 * @brief   Library for interacting with the LSM6DSO Sensor Hub function
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2023 STMicroelectronics</center></h2>
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

#ifndef LSM6DSOHub_h
#define LSM6DSOHub_h

#include "Arduino.h"
#include "LSM6DSOSensor.h"

class LSM6DSOHub {
  public:

    // Use this object to refer to the sensor hub.
    LSM6DSOHub(LSM6DSOSensor *m);

    // Fully reset the LSM6DSO sensor. Should be done once at program start.
    void resetMaster();

    // Allows normal interaction with slave sensors, but prevents the sensor hub from operating.
    void enablePassthrough();

    // Allows the sensor hub to operate, but prevents normal interaction with slave sensors.
    void disablePassthrough();

    // Enable sensor hub and start periodically reading the data from slave sensors
    bool begin();

    // Add a device to be controlled and read from the hub
    // addr = I2C address of the device
    // reg = address of the first register to be read by the sensor hub
    // reads = how many times the device should be read in succession; cannot exceed 7
    // Note: many devices will automatically provide successive registers when read in succession, but some will not. This is usually referred to as auto-increment. Check your device datasheet.
    int addDevice(uint8_t addr, uint8_t reg, uint8_t reads);

    // Returns True when the sensor hub is ready with new results, after which they should be read using read().
    bool ready();

    // Extracts the data from connected sensors at makes it available for the getSensor function.
    void read();

    // Automatic method that combines ready() and read(). This method will wait until the data is ready and immediately extract it. THIS FUNCTION IS BLOCKING until the data is ready.
    void waitAndRead();

    // Puts the contents of all read registers from device number i in arr, in order.
    bool getSensor(unsigned int i, uint32_t *arr);


  private:

    LSM6DSOSensor *master;

    uint8_t modeReg;
    uint8_t masterConfigReg;
    uint8_t masterStatusReg;

    uint8_t hubMode;
    uint8_t normalMode;

    uint8_t slaveConfigAddr;                  // Rule: add 3*n for nth slave set where n is nth set numbered 0...3
    uint8_t slaveConfigReg;
    uint8_t slaveConfigMode;
    uint8_t slaveResultFirst;                 // Rule: increment by 1 to access next register until 0x13
    uint8_t slaveResultLast;                  // Highest possible result register

    uint8_t results [18];                                // If you are reading 0xff there is probably an error
    int resultsOffsets [4];
    int numSensors;

    bool incorrectSensors;

    void hubEnter();
    void hubExit();
    void readHub();
};

#endif

