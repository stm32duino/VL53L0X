/**
 ******************************************************************************
 * @file    X_NUCLEO_53L0A1_Gesture_Tap1.ino
 * @author  AST
 * @version V1.0.0
 * @date    21 April 2017
 * @brief   Arduino test application for the STMicrolectronics X-NUCLEO-53L0A1
 *          proximity sensor expansion board based on FlightSense and gesture library.
 *          This application makes use of C++ classes obtained from the C
 *          components' drivers.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
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


/* Includes ------------------------------------------------------------------*/
#include <Arduino.h>
#include <Wire.h>
#include <vl53l0x_class.h>
#include <stmpe1600_class.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <assert.h>
#include <tof_gestures.h>
#include <tof_gestures_TAP_1.h>

#if defined(ARDUINO_SAM_DUE)
#define DEV_I2C Wire1   //Define which I2C bus is used. Wire1 for the Arduino Due
#define SerialPort Serial
#elif defined(ARDUINO_STM32_STAR_OTTO)
#define DEV_I2C Wire    //Or Wire
#define SerialPort SerialUSB
#else
#define DEV_I2C Wire    //Or Wire
#define SerialPort Serial
#endif

// Components.
STMPE1600DigiOut *xshutdown_top;
STMPE1600DigiOut *xshutdown_left;
STMPE1600DigiOut *xshutdown_right;
VL53L0X *sensor_vl53l0x_top;
VL53L0X *sensor_vl53l0x_left;
VL53L0X *sensor_vl53l0x_right;

// Gesture structure.
Gesture_TAP_1_Data_t gestureTapData;

// Range value
uint32_t distance_top;

/**
 *  Setup all sensors for single shot mode
 */
void SetupSingleShot(void){
  int status;
  uint8_t VhvSettings;
  uint8_t PhaseCal;
  uint32_t refSpadCount;
  uint8_t isApertureSpads;

  status = sensor_vl53l0x_top->StaticInit();
  if( status ){
    SerialPort.println("StaticInit top sensor failed");
  }

  status = sensor_vl53l0x_top->PerformRefCalibration(&VhvSettings, &PhaseCal);
  if( status ){
    SerialPort.println("PerformRefCalibration top sensor failed");
  }

  status = sensor_vl53l0x_top->PerformRefSpadManagement(&refSpadCount, &isApertureSpads);
  if( status ){
    SerialPort.println("PerformRefSpadManagement top sensor failed");
  }

  status = sensor_vl53l0x_top->SetDeviceMode(VL53L0X_DEVICEMODE_SINGLE_RANGING); // Setup in single ranging mode
  if( status ){
    SerialPort.println("SetDeviceMode top sensor failed");
  }

  status = sensor_vl53l0x_top->SetMeasurementTimingBudgetMicroSeconds(20*1000);
  if( status ){
    SerialPort.println("SetMeasurementTimingBudgetMicroSeconds top sensor failed");
  }
}

/* Setup ---------------------------------------------------------------------*/

void setup() {
  int status;
  // Led.
  pinMode(13, OUTPUT);

  // Initialize serial for output.
  SerialPort.begin(115200);

  // Initialize I2C bus.
  DEV_I2C.begin();

  // Create VL53L0X top component.
  xshutdown_top = new STMPE1600DigiOut(&DEV_I2C, GPIO_15, (0x42 * 2));
  sensor_vl53l0x_top = new VL53L0X(&DEV_I2C, xshutdown_top, A2);
  
  // Switch off VL53L0X top component.
  sensor_vl53l0x_top->VL53L0X_Off();
  
  // Create (if present) VL53L0X left component.
  xshutdown_left = new STMPE1600DigiOut(&DEV_I2C, GPIO_14, (0x43 * 2));
  sensor_vl53l0x_left = new VL53L0X(&DEV_I2C, xshutdown_left, D8);
  
  // Switch off (if present) VL53L0X left component.
  sensor_vl53l0x_left->VL53L0X_Off();
  
  // Create (if present) VL53L0X right component.
  xshutdown_right = new STMPE1600DigiOut(&DEV_I2C, GPIO_15, (0x43 * 2));
  sensor_vl53l0x_right = new VL53L0X(&DEV_I2C, xshutdown_right, D2);
  
  // Switch off (if present) VL53L0X right component.
  sensor_vl53l0x_right->VL53L0X_Off();
  
  // Initialize VL53L0X top component.
  status = sensor_vl53l0x_top->InitSensor(0x10);
  if(status)
  {
    SerialPort.println("Init sensor_vl53l0x_top failed...");
  }
  
  // Initialize VL6180X gesture library.
  tof_gestures_initTAP_1(&gestureTapData);

  SetupSingleShot();
}


/* Loop ----------------------------------------------------------------------*/

void loop() {
  int gesture_code;
  int status;
  
  sensor_vl53l0x_top->StartMeasurement();
  
  int top_done = 0;
  uint8_t NewDataReady=0;
  VL53L0X_RangingMeasurementData_t pRangingMeasurementData;

  do
  {
    if(top_done == 0)
    {
      NewDataReady = 0;
      int status = sensor_vl53l0x_top->GetMeasurementDataReady(&NewDataReady);

      if( status ){
        SerialPort.println("GetMeasurementDataReady top sensor failed");
      }
      
      if(NewDataReady)
      {
        status = sensor_vl53l0x_top->ClearInterruptMask(0);
        if( status ){
          SerialPort.println("ClearInterruptMask top sensor failed");
        }

        status = sensor_vl53l0x_top->GetRangingMeasurementData(&pRangingMeasurementData);
        if( status ){
          SerialPort.println("GetRangingMeasurementData top sensor failed");
        }

        if (pRangingMeasurementData.RangeStatus == 0) {
          // we have a valid range.
          distance_top = pRangingMeasurementData.RangeMilliMeter;
        }else {
          distance_top = 1200;
        }
        
        top_done = 1;
      }
    }
  }while(top_done == 0);
  
  // Launch gesture detection algorithm.
  gesture_code = tof_gestures_detectTAP_1(distance_top, &gestureTapData);

  // Check the result of the gesture detection algorithm.
  switch(gesture_code)
  {
    case GESTURES_SINGLE_TAP:
      SerialPort.println("GESTURES_SINGLE_TAP DETECTED!!!");
      break;
    default:
      // Do nothing
      break;
  }
}

