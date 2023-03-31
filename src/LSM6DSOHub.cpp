/**
 ******************************************************************************
 * @file    LSM6DSOHub.cpp
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

#include "Arduino.h"
#include "LSM6DSOHub.h"

void LSM6DSOHub::hubEnter()
{
  master->Write_Reg(modeReg, hubMode);
  return;
}

void LSM6DSOHub::hubExit()
{
  master->Write_Reg(modeReg, normalMode);
  return;
}

void LSM6DSOHub::readHub()
{
  if ((numSensors < 0) || (numSensors > 3)) {
    incorrectSensors = true;  // ERROR: either no sensors have been added or something very strange happened
  }
  // Besides this, this function perform no error checking of any sort!
  hubEnter();
  for (int i = 0; i < 18; i++) {
    master->Read_Reg(slaveResultFirst + i, &results[i]);
  }

  hubExit();
  return;
}

LSM6DSOHub::LSM6DSOHub(LSM6DSOSensor *m) : master(m)
{
  modeReg = 0x01;
  masterConfigReg = 0x14;
  masterStatusReg = 0x22;

  hubMode = 0x40;
  normalMode = 0x00;

  slaveConfigAddr = 0x15;                     // Rule: add 3*n for nth slave set where n is nth set numbered 0...3
  slaveConfigReg = 0x16;
  slaveConfigMode = 0x17;
  slaveResultFirst = 0x02;                    // Rule: increment by 1 to access next register until 0x13
  slaveResultLast = 0x13;                     // Highest possible result register

  memset(results, 0xff, 18 * sizeof(uint8_t)); // If you are reading 0xff there is probably an error
  memset(resultsOffsets, 0, 4 * sizeof(int));
  numSensors = -1;

  incorrectSensors = false;
}

void LSM6DSOHub::resetMaster()                                                // Fully reset the LSM6DSO sensor. Should be done once at program start.
{
  hubExit();

  master->Write_Reg(0x10, 0b0);
  master->Write_Reg(0x11, 0b0);
  master->Write_Reg(0x12, 0b10000100);
  delay(20);
  master->Write_Reg(0x10, 0b0);
  master->Write_Reg(0x11, 0b0);
  master->Write_Reg(0x12, 0b00000101);
  delay(20);

  return;
}

void LSM6DSOHub::enablePassthrough()
{
  hubEnter();

  uint8_t reg = 0xff;
  master->Read_Reg(masterConfigReg, &reg);                          // Get status of master register
  reg = reg | 0b00010000;                                           // Raise passthrough flag
  master->Write_Reg(masterConfigReg, reg);                          // Write status of master register

  hubExit();
  return;
}

void LSM6DSOHub::disablePassthrough()
{
  hubEnter();

  uint8_t reg = 0xff;
  master->Read_Reg(masterConfigReg, &reg);                          // Get status of master register
  reg = reg & 0b11101111;                                           // Lower passthrough flag
  master->Write_Reg(masterConfigReg, reg);                          // Write status of master register

  hubExit();
  return;
}


bool LSM6DSOHub::begin()
{
  hubEnter();

  if ((numSensors < 0) || (numSensors > 3)) {
    return true;  // ERROR: no sensors set up or too many sensors
  }
  uint8_t reg = 0b01000100 | ((uint8_t) numSensors);
  master->Write_Reg(0x14, reg);

  hubExit();
  return false;
}

int LSM6DSOHub::addDevice(uint8_t addr, uint8_t reg, uint8_t reads)
{
  if (reads > 7) {
    return -20;  // ERROR: each device can't be read more than 7 times
  }

  hubEnter();
  if (numSensors > 3) {
    return -10;  // ERROR: too many sensors are being added. Maximum is four, numbered 0...3
  }
  numSensors = numSensors + 1;

  // Configure hub to read from the device at "address"
  master->Write_Reg(slaveConfigAddr + (numSensors * 3), addr);

  // Configure hub to read the register "reg" from the device
  master->Write_Reg(slaveConfigReg + (numSensors * 3), reg);

  // Configure the read
  master->Write_Reg(slaveConfigMode + (numSensors * 3), reads);

  resultsOffsets [numSensors] = reads;
  hubExit();

  return numSensors;
}

bool LSM6DSOHub::ready()                               // Returns True when the sensor hub is ready with new results, after which they should be read using read()
{
  hubEnter();
  uint8_t ready = 0;
  master->Read_Reg(masterStatusReg, &ready);
  hubExit();

  return ((bool)(ready & 0b00000001));
}

void LSM6DSOHub::read()                                   // Actually get the results from the sensor hub
{
  readHub();
  return;
}

void LSM6DSOHub::waitAndRead()                            // Alternative automatic way of obtaining results: BLOCK AND WAIT until results are available, then record them immediately
{
  hubEnter();
  uint8_t ready = 0;
  master->Read_Reg(masterStatusReg, &ready);

  while (!(ready & 0b00000001)) {
    // Do nothing while waiting
  }
  hubExit();

  readHub();
  return;
}

bool LSM6DSOHub::getSensor(unsigned int i, uint32_t *arr)                // Gets the read registers from the i-th sensor on the sensor hub. i must be a valid sensor. Does not actually access the sensor hub
{
  // 32 bits ought to be enough for anybody
  if ((i < 0) || (i > numSensors)) {
    return true;
  }

  int startOffset = 0;

  // Calculate where to start reading from, in the SHUB result registers
  for (int j = 0; j < i ; j++) {
    startOffset = startOffset + resultsOffsets[j];
  }

  // Actually read at this offset
  for (int j = 0; j < resultsOffsets[i]; j++) {
    arr[j] = results [startOffset + j];
  }

  return false;
}
