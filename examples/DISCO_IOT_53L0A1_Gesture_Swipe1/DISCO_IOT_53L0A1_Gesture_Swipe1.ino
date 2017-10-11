/**
 ******************************************************************************
 * @file    DISCO_IOT_53L0A1_Gesture_Swipe1.ino
 * @author  WI6LABS from AST
 * @version V1.0.0
 * @date    07 July 2017
 * @brief   Arduino test application for the STMicrolectronics STM32 IOT Discovery Kit
 *          based on FlightSense and gesture library.
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
#include <Wire.h>
#include <vl53l0x_class.h>
#include <tof_gestures.h>
#include <tof_gestures_SWIPE_1.h>

// Create components.
TwoWire WIRE1(PB11, PB10);  //SDA=PB11 & SCL=PB10
VL53L0X sensor_vl53l0x(&WIRE1, PC6, PC7); //XSHUT=PC6 & INT=PC7

// Gesture structure.
Gesture_SWIPE_1_Data_t gestureSwipeData;

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

  status = sensor_vl53l0x.StaticInit();
  if( status ){
    Serial.println("StaticInit sensor failed");
  }

  status = sensor_vl53l0x.PerformRefCalibration(&VhvSettings, &PhaseCal);
  if( status ){
    Serial.println("PerformRefCalibration sensor failed");
  }

  status = sensor_vl53l0x.PerformRefSpadManagement(&refSpadCount, &isApertureSpads);
  if( status ){
    Serial.println("PerformRefSpadManagement sensor failed");
  }

  status = sensor_vl53l0x.SetDeviceMode(VL53L0X_DEVICEMODE_SINGLE_RANGING); // Setup in single ranging mode
  if( status ){
    Serial.println("SetDeviceMode sensor failed");
  }

  status = sensor_vl53l0x.SetMeasurementTimingBudgetMicroSeconds(20*1000);
  if( status ){
    Serial.println("SetMeasurementTimingBudgetMicroSeconds sensor failed");
  }
}

/* Setup ---------------------------------------------------------------------*/

void setup() {
  int status;

  // Initialize serial for output.
  Serial.begin(9600);

  // Initialize I2C bus.
  WIRE1.begin();

  // Switch off VL53L0X top component.
  sensor_vl53l0x.VL53L0X_Off();

  // Initialize VL53L0X top component.
  status = sensor_vl53l0x.InitSensor(0x10);
  if(status)
  {
    Serial.println("Init sensor_vl53l0x failed...");
  }

  // Initialize gesture library.
  tof_gestures_initSWIPE_1(&gestureSwipeData);

  SetupSingleShot();
}


/* Loop ----------------------------------------------------------------------*/

void loop() {
  int gesture_code;
  int status;

  sensor_vl53l0x.StartMeasurement();

  int top_done = 0;
  uint8_t NewDataReady=0;
  VL53L0X_RangingMeasurementData_t pRangingMeasurementData;

  do
  {
    if(top_done == 0)
    {
      NewDataReady = 0;
      status = sensor_vl53l0x.GetMeasurementDataReady(&NewDataReady);

      if( status ){
        Serial.println("GetMeasurementDataReady sensor failed");
      }

      if(NewDataReady)
      {
        status = sensor_vl53l0x.ClearInterruptMask(0);
        if( status ){
          Serial.println("ClearInterruptMask sensor failed");
        }

        status = sensor_vl53l0x.GetRangingMeasurementData(&pRangingMeasurementData);
        if( status ){
          Serial.println("GetRangingMeasurementData sensor failed");
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
  gesture_code = tof_gestures_detectSWIPE_1(distance_top, &gestureSwipeData);

  // Check the result of the gesture detection algorithm.
  switch(gesture_code)
  {
    case GESTURES_SINGLE_SWIPE:
      Serial.println("GESTURES_SINGLE_SWIPE DETECTED!!!");
      break;
    default:
      // Do nothing
      break;
  }
}
