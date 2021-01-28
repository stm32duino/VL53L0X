/*******************************************************************************
 Copyright Ã‚Â© 2016, STMicroelectronics International N.V.
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 * Neither the name of STMicroelectronics nor the
 names of its contributors may be used to endorse or promote products
 derived from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, AND
 NON-INFRINGEMENT OF INTELLECTUAL PROPERTY RIGHTS ARE DISCLAIMED.
 IN NO EVENT SHALL STMICROELECTRONICS INTERNATIONAL N.V. BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

#ifndef __VL53L0X_CLASS_H
#define __VL53L0X_CLASS_H


#ifdef _MSC_VER
#   ifdef VL53L0X_API_EXPORTS
#       define VL53L0X_API  __declspec(dllexport)
#   else
#       define VL53L0X_API
#   endif
#else
#   define VL53L0X_API
#endif


/* Includes ------------------------------------------------------------------*/
#include "Arduino.h"
#include "RangeSensor.h"
#include "Wire.h"

#include "vl53l0x_def.h"
#include "vl53l0x_platform.h"


/**
 * The device model ID
 */
//#define IDENTIFICATION_MODEL_ID                 0x000


//#define STATUS_OK              0x00
//#define STATUS_FAIL            0x01


#define VL53L0X_OsDelay(...) delay(2) // 2 msec delay. can also use wait(float secs)/wait_us(int)

#ifdef USE_EMPTY_STRING
	#define  VL53L0X_STRING_DEVICE_INFO_NAME                             ""
	#define  VL53L0X_STRING_DEVICE_INFO_NAME_TS0                         ""
	#define  VL53L0X_STRING_DEVICE_INFO_NAME_TS1                         ""
	#define  VL53L0X_STRING_DEVICE_INFO_NAME_TS2                         ""
	#define  VL53L0X_STRING_DEVICE_INFO_NAME_ES1                         ""
	#define  VL53L0X_STRING_DEVICE_INFO_TYPE                             ""

	/* PAL ERROR strings */
	#define  VL53L0X_STRING_ERROR_NONE                                   ""
	#define  VL53L0X_STRING_ERROR_CALIBRATION_WARNING                    ""
	#define  VL53L0X_STRING_ERROR_MIN_CLIPPED                            ""
	#define  VL53L0X_STRING_ERROR_UNDEFINED                              ""
	#define  VL53L0X_STRING_ERROR_INVALID_PARAMS                         ""
	#define  VL53L0X_STRING_ERROR_NOT_SUPPORTED                          ""
	#define  VL53L0X_STRING_ERROR_RANGE_ERROR                            ""
	#define  VL53L0X_STRING_ERROR_TIME_OUT                               ""
	#define  VL53L0X_STRING_ERROR_MODE_NOT_SUPPORTED                     ""
	#define  VL53L0X_STRING_ERROR_BUFFER_TOO_SMALL                       ""
	#define  VL53L0X_STRING_ERROR_GPIO_NOT_EXISTING                      ""
	#define  VL53L0X_STRING_ERROR_GPIO_FUNCTIONALITY_NOT_SUPPORTED       ""
	#define  VL53L0X_STRING_ERROR_CONTROL_INTERFACE                      ""
	#define  VL53L0X_STRING_ERROR_INVALID_COMMAND                        ""
	#define  VL53L0X_STRING_ERROR_DIVISION_BY_ZERO                       ""
	#define  VL53L0X_STRING_ERROR_REF_SPAD_INIT                          ""
	#define  VL53L0X_STRING_ERROR_NOT_IMPLEMENTED                        ""

	#define  VL53L0X_STRING_UNKNOW_ERROR_CODE                            ""



	/* Range Status */
	#define  VL53L0X_STRING_RANGESTATUS_NONE                             ""
	#define  VL53L0X_STRING_RANGESTATUS_RANGEVALID                       ""
	#define  VL53L0X_STRING_RANGESTATUS_SIGMA                            ""
	#define  VL53L0X_STRING_RANGESTATUS_SIGNAL                           ""
	#define  VL53L0X_STRING_RANGESTATUS_MINRANGE                         ""
	#define  VL53L0X_STRING_RANGESTATUS_PHASE                            ""
	#define  VL53L0X_STRING_RANGESTATUS_HW                               ""


	/* Range Status */
	#define  VL53L0X_STRING_STATE_POWERDOWN                              ""
	#define  VL53L0X_STRING_STATE_WAIT_STATICINIT                        ""
	#define  VL53L0X_STRING_STATE_STANDBY                                ""
	#define  VL53L0X_STRING_STATE_IDLE                                   ""
	#define  VL53L0X_STRING_STATE_RUNNING                                ""
	#define  VL53L0X_STRING_STATE_UNKNOWN                                ""
	#define  VL53L0X_STRING_STATE_ERROR                                  ""


	/* Device Specific */
	#define  VL53L0X_STRING_DEVICEERROR_NONE                             ""
	#define  VL53L0X_STRING_DEVICEERROR_VCSELCONTINUITYTESTFAILURE       ""
	#define  VL53L0X_STRING_DEVICEERROR_VCSELWATCHDOGTESTFAILURE         ""
	#define  VL53L0X_STRING_DEVICEERROR_NOVHVVALUEFOUND                  ""
	#define  VL53L0X_STRING_DEVICEERROR_MSRCNOTARGET                     ""
	#define  VL53L0X_STRING_DEVICEERROR_SNRCHECK                         ""
	#define  VL53L0X_STRING_DEVICEERROR_RANGEPHASECHECK                  ""
	#define  VL53L0X_STRING_DEVICEERROR_SIGMATHRESHOLDCHECK              ""
	#define  VL53L0X_STRING_DEVICEERROR_TCC                              ""
	#define  VL53L0X_STRING_DEVICEERROR_PHASECONSISTENCY                 ""
	#define  VL53L0X_STRING_DEVICEERROR_MINCLIP                          ""
	#define  VL53L0X_STRING_DEVICEERROR_RANGECOMPLETE                    ""
	#define  VL53L0X_STRING_DEVICEERROR_ALGOUNDERFLOW                    ""
	#define  VL53L0X_STRING_DEVICEERROR_ALGOOVERFLOW                     ""
	#define  VL53L0X_STRING_DEVICEERROR_RANGEIGNORETHRESHOLD             ""
	#define  VL53L0X_STRING_DEVICEERROR_UNKNOWN                          ""

	/* Check Enable */
	#define  VL53L0X_STRING_CHECKENABLE_SIGMA_FINAL_RANGE                ""
	#define  VL53L0X_STRING_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE          ""
	#define  VL53L0X_STRING_CHECKENABLE_SIGNAL_REF_CLIP                  ""
	#define  VL53L0X_STRING_CHECKENABLE_RANGE_IGNORE_THRESHOLD           ""

	/* Sequence Step */
	#define  VL53L0X_STRING_SEQUENCESTEP_TCC                             ""
	#define  VL53L0X_STRING_SEQUENCESTEP_DSS                             ""
	#define  VL53L0X_STRING_SEQUENCESTEP_MSRC                            ""
	#define  VL53L0X_STRING_SEQUENCESTEP_PRE_RANGE                       ""
	#define  VL53L0X_STRING_SEQUENCESTEP_FINAL_RANGE                     ""
#else
	#define  VL53L0X_STRING_DEVICE_INFO_NAME          "VL53L0X cut1.0"
	#define  VL53L0X_STRING_DEVICE_INFO_NAME_TS0      "VL53L0X TS0"
	#define  VL53L0X_STRING_DEVICE_INFO_NAME_TS1      "VL53L0X TS1"
	#define  VL53L0X_STRING_DEVICE_INFO_NAME_TS2      "VL53L0X TS2"
	#define  VL53L0X_STRING_DEVICE_INFO_NAME_ES1      "VL53L0X ES1 or later"
	#define  VL53L0X_STRING_DEVICE_INFO_TYPE          "VL53L0X"

	/* PAL ERROR strings */
	#define  VL53L0X_STRING_ERROR_NONE \
			"No Error"
	#define  VL53L0X_STRING_ERROR_CALIBRATION_WARNING \
			"Calibration Warning Error"
	#define  VL53L0X_STRING_ERROR_MIN_CLIPPED \
			"Min clipped error"
	#define  VL53L0X_STRING_ERROR_UNDEFINED \
			"Undefined error"
	#define  VL53L0X_STRING_ERROR_INVALID_PARAMS \
			"Invalid parameters error"
	#define  VL53L0X_STRING_ERROR_NOT_SUPPORTED \
			"Not supported error"
	#define  VL53L0X_STRING_ERROR_RANGE_ERROR \
			"Range error"
	#define  VL53L0X_STRING_ERROR_TIME_OUT \
			"Time out error"
	#define  VL53L0X_STRING_ERROR_MODE_NOT_SUPPORTED \
			"Mode not supported error"
	#define  VL53L0X_STRING_ERROR_BUFFER_TOO_SMALL \
			"Buffer too small"
	#define  VL53L0X_STRING_ERROR_GPIO_NOT_EXISTING \
			"GPIO not existing"
	#define  VL53L0X_STRING_ERROR_GPIO_FUNCTIONALITY_NOT_SUPPORTED \
			"GPIO funct not supported"
	#define  VL53L0X_STRING_ERROR_INTERRUPT_NOT_CLEARED \
			"Interrupt not Cleared"
	#define  VL53L0X_STRING_ERROR_CONTROL_INTERFACE \
			"Control Interface Error"
	#define  VL53L0X_STRING_ERROR_INVALID_COMMAND \
			"Invalid Command Error"
	#define  VL53L0X_STRING_ERROR_DIVISION_BY_ZERO \
			"Division by zero Error"
	#define  VL53L0X_STRING_ERROR_REF_SPAD_INIT \
			"Reference Spad Init Error"
	#define  VL53L0X_STRING_ERROR_NOT_IMPLEMENTED \
			"Not implemented error"

	#define  VL53L0X_STRING_UNKNOW_ERROR_CODE \
			"Unknown Error Code"



	/* Range Status */
	#define  VL53L0X_STRING_RANGESTATUS_NONE                 "No Update"
	#define  VL53L0X_STRING_RANGESTATUS_RANGEVALID           "Range Valid"
	#define  VL53L0X_STRING_RANGESTATUS_SIGMA                "Sigma Fail"
	#define  VL53L0X_STRING_RANGESTATUS_SIGNAL               "Signal Fail"
	#define  VL53L0X_STRING_RANGESTATUS_MINRANGE             "Min Range Fail"
	#define  VL53L0X_STRING_RANGESTATUS_PHASE                "Phase Fail"
	#define  VL53L0X_STRING_RANGESTATUS_HW                   "Hardware Fail"


	/* Range Status */
	#define  VL53L0X_STRING_STATE_POWERDOWN               "POWERDOWN State"
	#define  VL53L0X_STRING_STATE_WAIT_STATICINIT \
			"Wait for staticinit State"
	#define  VL53L0X_STRING_STATE_STANDBY                 "STANDBY State"
	#define  VL53L0X_STRING_STATE_IDLE                    "IDLE State"
	#define  VL53L0X_STRING_STATE_RUNNING                 "RUNNING State"
	#define  VL53L0X_STRING_STATE_UNKNOWN                 "UNKNOWN State"
	#define  VL53L0X_STRING_STATE_ERROR                   "ERROR State"


	/* Device Specific */
	#define  VL53L0X_STRING_DEVICEERROR_NONE                   "No Update"
	#define  VL53L0X_STRING_DEVICEERROR_VCSELCONTINUITYTESTFAILURE \
			"VCSEL Continuity Test Failure"
	#define  VL53L0X_STRING_DEVICEERROR_VCSELWATCHDOGTESTFAILURE \
			"VCSEL Watchdog Test Failure"
	#define  VL53L0X_STRING_DEVICEERROR_NOVHVVALUEFOUND \
			"No VHV Value found"
	#define  VL53L0X_STRING_DEVICEERROR_MSRCNOTARGET \
			"MSRC No Target Error"
	#define  VL53L0X_STRING_DEVICEERROR_SNRCHECK \
			"SNR Check Exit"
	#define  VL53L0X_STRING_DEVICEERROR_RANGEPHASECHECK \
			"Range Phase Check Error"
	#define  VL53L0X_STRING_DEVICEERROR_SIGMATHRESHOLDCHECK \
			"Sigma Threshold Check Error"
	#define  VL53L0X_STRING_DEVICEERROR_TCC \
			"TCC Error"
	#define  VL53L0X_STRING_DEVICEERROR_PHASECONSISTENCY \
			"Phase Consistency Error"
	#define  VL53L0X_STRING_DEVICEERROR_MINCLIP \
			"Min Clip Error"
	#define  VL53L0X_STRING_DEVICEERROR_RANGECOMPLETE \
			"Range Complete"
	#define  VL53L0X_STRING_DEVICEERROR_ALGOUNDERFLOW \
			"Range Algo Underflow Error"
	#define  VL53L0X_STRING_DEVICEERROR_ALGOOVERFLOW \
			"Range Algo Overlow Error"
	#define  VL53L0X_STRING_DEVICEERROR_RANGEIGNORETHRESHOLD \
			"Range Ignore Threshold Error"
	#define  VL53L0X_STRING_DEVICEERROR_UNKNOWN \
			"Unknown error code"

	/* Check Enable */
	#define  VL53L0X_STRING_CHECKENABLE_SIGMA_FINAL_RANGE \
			"SIGMA FINAL RANGE"
	#define  VL53L0X_STRING_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE \
			"SIGNAL RATE FINAL RANGE"
	#define  VL53L0X_STRING_CHECKENABLE_SIGNAL_REF_CLIP \
			"SIGNAL REF CLIP"
	#define  VL53L0X_STRING_CHECKENABLE_RANGE_IGNORE_THRESHOLD \
			"RANGE IGNORE THRESHOLD"
	#define  VL53L0X_STRING_CHECKENABLE_SIGNAL_RATE_MSRC \
			"SIGNAL RATE MSRC"
	#define  VL53L0X_STRING_CHECKENABLE_SIGNAL_RATE_PRE_RANGE \
			"SIGNAL RATE PRE RANGE"

	/* Sequence Step */
	#define  VL53L0X_STRING_SEQUENCESTEP_TCC                   "TCC"
	#define  VL53L0X_STRING_SEQUENCESTEP_DSS                   "DSS"
	#define  VL53L0X_STRING_SEQUENCESTEP_MSRC                  "MSRC"
	#define  VL53L0X_STRING_SEQUENCESTEP_PRE_RANGE             "PRE RANGE"
	#define  VL53L0X_STRING_SEQUENCESTEP_FINAL_RANGE           "FINAL RANGE"
#endif /* USE_EMPTY_STRING */





/* sensor operating modes */
typedef enum
{
   range_single_shot_polling=1,
   range_continuous_polling,
   range_continuous_interrupt,
   range_continuous_polling_low_threshold,
   range_continuous_polling_high_threshold,
   range_continuous_polling_out_of_window,
   range_continuous_interrupt_low_threshold,
   range_continuous_interrupt_high_threshold,
   range_continuous_interrupt_out_of_window,
}OperatingMode;

/** default device address */
#define VL53L0x_DEFAULT_DEVICE_ADDRESS		0x52 /* (8-bit) */

/* Classes -------------------------------------------------------------------*/
/** Class representing a VL53L0 sensor component
 */
class VL53L0X : public RangeSensor
{
 public:
    /** Constructor
     * @param[in] i2c device I2C to be used for communication
     * @param[in] pin shutdown pin to be used as component GPIO0
     */
    VL53L0X(TwoWire *i2c, int pin) : RangeSensor(), dev_i2c(i2c), gpio0(pin)
    {
       Device=&MyDevice;
       memset((void *)Device, 0x0, sizeof(VL53L0X_Dev_t));
       MyDevice.I2cDevAddr=VL53L0x_DEFAULT_DEVICE_ADDRESS;
       MyDevice.comms_type=1; // VL53L0X_COMMS_I2C
       MyDevice.comms_speed_khz=400;
    }
    
   /** Destructor
    */
    virtual ~VL53L0X(){}
    /* warning: VL53L0X class inherits from GenericSensor, RangeSensor and LightSensor, that haven`t a destructor.
       The warning should request to introduce a virtual destructor to make sure to delete the object */

    virtual int begin()
    {
       if(gpio0 >= 0)
       {
          pinMode(gpio0, OUTPUT);
       }
       return 0;
    }

    virtual int end()
    {
       if(gpio0 >= 0)
       {
          pinMode(gpio0, INPUT);
       }
       return 0;
    }

	/*** Interface Methods ***/
	/*** High level API ***/
	/**
	 * @brief       PowerOn the sensor
	 * @return      void
	 */
    /* turns on the sensor */
    virtual void VL53L0X_On(void)
    {
       if(gpio0 >= 0)
       {
         digitalWrite(gpio0, HIGH);
       }
       delay(10);
    }

	/**
	 * @brief       PowerOff the sensor
	 * @return      void
	 */
    /* turns off the sensor */
    virtual void VL53L0X_Off(void)
    {
       if(gpio0 >= 0)
       {
         digitalWrite(gpio0, LOW);
       }
       delay(10);
    }

	/**
	 * @brief       Initialize the sensor with default values
	 * @return      0 on Success
	 */
    int InitSensor(uint8_t NewAddr);

	/**
	 * @brief       Start the measure indicated by operating mode
	 * @param[in]   operating_mode specifies requested measure
	 * @param[in]   fptr specifies call back function must be !NULL in case of interrupt measure
	 * @return      0 on Success
	 */
    int StartMeasurementSimplified(OperatingMode operating_mode, void (*fptr)(void));

	/**
	 * @brief       Get results for the measure indicated by operating mode
	 * @param[in]   operating_mode specifies requested measure results
	 * @param[out]  Data pointer to the MeasureData_t structure to read data in to
	 * @return      0 on Success
	 */
    int GetMeasurementSimplified(OperatingMode operating_mode, VL53L0X_RangingMeasurementData_t *Data);

	/**
	 * @brief       Stop the currently running measure indicate by operating_mode
	 * @param[in]   operating_mode specifies requested measure to stop
	 * @return      0 on Success
	 */
    int StopMeasurementSimplified(OperatingMode operating_mode);

    /** Wrapper functions */
/** @defgroup api_init Init functions
 *  @brief    API init functions
 *  @ingroup api_hl
 *  @{
 */
/**
 * @brief Wait for device booted after chip enable (hardware standby)
 * @par Function Description
 * After Chip enable Application you can also simply wait at least 1ms to ensure device is ready
 * @warning After device chip enable (gpio0) de-asserted  user must wait gpio1 to get asserted (hardware standby).
 * or wait at least 400usec prior to do any low level access or api call .
 *
 * This function implements polling for standby but you must ensure 400usec from chip enable passed\n
 * @warning if device get prepared @a VL53L0X_Prepare() re-using these function can hold indefinitely\n
 *
 * @param 		void
 * @return     0 on success
 */
    int WaitDeviceBooted()
    {
       return VL53L0X_WaitDeviceBooted(Device);
    }

/**
 *
 * @brief One time device initialization
 *
 * To be called once and only once after device is brought out of reset (Chip enable) and booted see @a VL6180x_WaitDeviceBooted()
 *
 * @par Function Description
 * When not used after a fresh device "power up" or reset, it may return @a #CALIBRATION_WARNING
 * meaning wrong calibration data may have been fetched from device that can result in ranging offset error\n
 * If application cannot execute device reset or need to run VL6180x_InitData  multiple time
 * then it  must ensure proper offset calibration saving and restore on its own
 * by using @a VL6180x_GetOffsetCalibrationData() on first power up and then @a VL6180x_SetOffsetCalibrationData() all all subsequent init
 *
 * @param void
 * @return     0 on success,  @a #CALIBRATION_WARNING if failed
 */
    virtual int Init()
    {
       return VL53L0X_DataInit(Device);
    }

/**
  * @brief  Prepare device for operation
  * @par Function Description
  * Does static initialization and reprogram common default settings \n
  * Device is prepared for new measure, ready single shot ranging or ALS typical polling operation\n
  * After prepare user can : \n
  * @li Call other API function to set other settings\n
  * @li Configure the interrupt pins, etc... \n
  * @li Then start ranging or ALS operations in single shot or continuous mode
  *
  * @param void
  * @return      0 on success
  */
    int Prepare()
    {
        // taken from rangingTest() in vl53l0x_SingleRanging_Example.c
		VL53L0X_Error Status = VL53L0X_ERROR_NONE;
        uint32_t refSpadCount;
        uint8_t isApertureSpads;
        uint8_t VhvSettings;
        uint8_t PhaseCal;

        if(Status == VL53L0X_ERROR_NONE)
        {
            Status = VL53L0X_StaticInit(Device); // Device Initialization
        }

        if(Status == VL53L0X_ERROR_NONE)
        {
           Status = VL53L0X_PerformRefCalibration(Device, &VhvSettings, &PhaseCal); // Device Initialization
        }

        if(Status == VL53L0X_ERROR_NONE)
        {
            Status = VL53L0X_PerformRefSpadManagement(Device, &refSpadCount, &isApertureSpads); // Device Initialization
        }

        return Status;
    }

/**
 * @brief Get ranging result and only that
 *
 * @par Function Description
 * Unlike @a VL6180x_RangeGetMeasurement() this function only retrieves the range in millimeter \n
 * It does any required up-scale translation\n
 * It can be called after success status polling or in interrupt mode \n
 * @warning these function is not doing wrap around filtering \n
 * This function doesn't perform any data ready check!
 *
 * @param pRange_mm  Pointer to range distance
 * @return           0 on success
 */
    virtual int GetDistance(uint32_t *piData)
    {
        int status=0;
        VL53L0X_RangingMeasurementData_t pRangingMeasurementData;

        status=StartMeasurementSimplified(range_single_shot_polling, NULL);
        if (!status) {
            status=GetMeasurementSimplified(range_single_shot_polling, &pRangingMeasurementData);
        }
        if (pRangingMeasurementData.RangeStatus == 0) {
        // we have a valid range.
            *piData = pRangingMeasurementData.RangeMilliMeter;
        }
        else {
            *piData = 0;
            status = VL53L0X_ERROR_RANGE_ERROR;
        }
        StopMeasurementSimplified(range_single_shot_polling);
        return status;
    }

/**
 * @brief Low level ranging and ALS register static settings (you should call @a VL6180x_Prepare() function instead)
 *
 * @return 0 on success
 */
    int StaticInit()
    {
	  return VL53L0X_StaticInit(Device);
    }

/**
 * @brief Set new device i2c address
 *
 * After completion the device will answer to the new address programmed.
 *
 * @sa AN4478: Using multiple VL6180X's in a single design
 * @param NewAddr   The new i2c address (7bit)
 * @return          0 on success
 */
    int SetDeviceAddress(int NewAddr)
    {
       int status;

       status=VL53L0X_SetDeviceAddress(Device, NewAddr);
       if(!status)
          Device->I2cDevAddr=NewAddr;
       return status;
    }

	int PerformRefCalibration(uint8_t *pVhvSettings, uint8_t *pPhaseCal)
	{
		return VL53L0X_PerformRefCalibration(Device, pVhvSettings, pPhaseCal);
	}

	int PerformRefSpadManagement(uint32_t *refSpadCount, uint8_t *isApertureSpads)
	{
		return VL53L0X_PerformRefSpadManagement(Device, refSpadCount, isApertureSpads);
	}

	int SetDeviceMode(VL53L0X_DeviceModes DeviceMode)
	{
		return VL53L0X_SetDeviceMode(Device, DeviceMode);
	}

	int SetMeasurementTimingBudgetMicroSeconds(uint32_t MeasurementTimingBudgetMicroSeconds)
	{
		return VL53L0X_SetMeasurementTimingBudgetMicroSeconds(Device, MeasurementTimingBudgetMicroSeconds);
	}

	int StartMeasurement()
	{
		return VL53L0X_StartMeasurement(Device);
	}

	int StopMeasurement()
	{
		return VL53L0X_StopMeasurement(Device);
	}

	int GetMeasurementDataReady(uint8_t *pMeasurementDataReady)
	{
		return VL53L0X_GetMeasurementDataReady(Device, pMeasurementDataReady);
	}

	int GetRangingMeasurementData(VL53L0X_RangingMeasurementData_t *pRangingMeasurementData)
	{
		return VL53L0X_GetRangingMeasurementData(Device, pRangingMeasurementData);
	}

	int ClearInterruptMask(uint32_t InterruptMask)
	{
		return VL53L0X_ClearInterruptMask(Device, InterruptMask);
	}


/** @}  */

/** @defgroup api_ll_intr Interrupts management functions
 *  @brief    Interrupts management functions
 *  @ingroup api_ll
 *  @{
 */

 protected:
    /* api.h functions */
    VL53L0X_Error VL53L0X_DataInit(VL53L0X_DEV Dev);
    VL53L0X_Error VL53L0X_GetOffsetCalibrationDataMicroMeter(VL53L0X_DEV Dev, int32_t *pOffsetCalibrationDataMicroMeter);
    VL53L0X_Error VL53L0X_SetOffsetCalibrationDataMicroMeter(VL53L0X_DEV Dev,
    	int32_t OffsetCalibrationDataMicroMeter);
    VL53L0X_Error VL53L0X_GetDeviceParameters(VL53L0X_DEV Dev,
    	VL53L0X_DeviceParameters_t *pDeviceParameters);
    VL53L0X_Error VL53L0X_GetDeviceMode(VL53L0X_DEV Dev,
    	VL53L0X_DeviceModes *pDeviceMode);
    VL53L0X_Error VL53L0X_GetInterMeasurementPeriodMilliSeconds(VL53L0X_DEV Dev,
    	uint32_t *pInterMeasurementPeriodMilliSeconds);
    VL53L0X_Error VL53L0X_GetXTalkCompensationRateMegaCps(VL53L0X_DEV Dev,
    	FixPoint1616_t *pXTalkCompensationRateMegaCps);
    VL53L0X_Error VL53L0X_GetLimitCheckValue(VL53L0X_DEV Dev, uint16_t LimitCheckId,
    	FixPoint1616_t *pLimitCheckValue);
    VL53L0X_Error VL53L0X_GetLimitCheckEnable(VL53L0X_DEV Dev, uint16_t LimitCheckId,
    	uint8_t *pLimitCheckEnable);
    VL53L0X_Error VL53L0X_GetWrapAroundCheckEnable(VL53L0X_DEV Dev,
    	uint8_t *pWrapAroundCheckEnable);
    VL53L0X_Error VL53L0X_GetMeasurementTimingBudgetMicroSeconds(VL53L0X_DEV Dev,
    	uint32_t *pMeasurementTimingBudgetMicroSeconds);
    VL53L0X_Error VL53L0X_GetSequenceStepEnables(VL53L0X_DEV Dev,
    	VL53L0X_SchedulerSequenceSteps_t *pSchedulerSequenceSteps);
    VL53L0X_Error sequence_step_enabled(VL53L0X_DEV Dev,
    	VL53L0X_SequenceStepId SequenceStepId, uint8_t SequenceConfig,
    	uint8_t *pSequenceStepEnabled);
    VL53L0X_Error VL53L0X_GetVcselPulsePeriod(VL53L0X_DEV Dev,
    	VL53L0X_VcselPeriod VcselPeriodType, uint8_t *pVCSELPulsePeriodPCLK);
    VL53L0X_Error VL53L0X_GetDeviceInfo(VL53L0X_DEV Dev,
    	VL53L0X_DeviceInfo_t *pVL53L0X_DeviceInfo);
    VL53L0X_Error VL53L0X_StaticInit(VL53L0X_DEV Dev);
    VL53L0X_Error VL53L0X_GetMeasurementDataReady(VL53L0X_DEV Dev,
    	uint8_t *pMeasurementDataReady);
    VL53L0X_Error VL53L0X_GetInterruptMaskStatus(VL53L0X_DEV Dev,
    	uint32_t *pInterruptMaskStatus);
    VL53L0X_Error VL53L0X_ClearInterruptMask(VL53L0X_DEV Dev, uint32_t InterruptMask);
    VL53L0X_Error VL53L0X_PerformSingleRangingMeasurement(VL53L0X_DEV Dev,
    	VL53L0X_RangingMeasurementData_t *pRangingMeasurementData);
    VL53L0X_Error VL53L0X_SetDeviceMode(VL53L0X_DEV Dev, VL53L0X_DeviceModes DeviceMode);
    VL53L0X_Error VL53L0X_PerformSingleMeasurement(VL53L0X_DEV Dev);
    VL53L0X_Error VL53L0X_StartMeasurement(VL53L0X_DEV Dev);
    VL53L0X_Error VL53L0X_CheckAndLoadInterruptSettings(VL53L0X_DEV Dev,
    	uint8_t StartNotStopFlag);
    VL53L0X_Error VL53L0X_GetInterruptThresholds(VL53L0X_DEV Dev,
    	VL53L0X_DeviceModes DeviceMode, FixPoint1616_t *pThresholdLow,
    	FixPoint1616_t *pThresholdHigh);
    VL53L0X_Error VL53L0X_GetRangingMeasurementData(VL53L0X_DEV Dev,
    	VL53L0X_RangingMeasurementData_t *pRangingMeasurementData);
    VL53L0X_Error VL53L0X_GetXTalkCompensationEnable(VL53L0X_DEV Dev,
    	uint8_t *pXTalkCompensationEnable);
    VL53L0X_Error VL53L0X_WaitDeviceBooted(VL53L0X_DEV Dev);
    VL53L0X_Error VL53L0X_PerformRefCalibration(VL53L0X_DEV Dev, uint8_t *pVhvSettings,
    	uint8_t *pPhaseCal);
    VL53L0X_Error VL53L0X_PerformRefSpadManagement(VL53L0X_DEV Dev,
    	uint32_t *refSpadCount, uint8_t *isApertureSpads);
    VL53L0X_Error VL53L0X_SetDeviceAddress(VL53L0X_DEV Dev, uint8_t DeviceAddress);
    VL53L0X_Error VL53L0X_SetGpioConfig(VL53L0X_DEV Dev, uint8_t Pin,
    	VL53L0X_DeviceModes DeviceMode, VL53L0X_GpioFunctionality Functionality,
    	VL53L0X_InterruptPolarity Polarity);
    VL53L0X_Error VL53L0X_GetFractionEnable(VL53L0X_DEV Dev, uint8_t *pEnabled);
    VL53L0X_Error VL53L0X_SetSequenceStepEnable(VL53L0X_DEV Dev,
    	VL53L0X_SequenceStepId SequenceStepId, uint8_t SequenceStepEnabled);
    VL53L0X_Error VL53L0X_SetMeasurementTimingBudgetMicroSeconds(VL53L0X_DEV Dev,
    	uint32_t MeasurementTimingBudgetMicroSeconds);
    VL53L0X_Error VL53L0X_SetLimitCheckEnable(VL53L0X_DEV Dev, uint16_t LimitCheckId,
    	uint8_t LimitCheckEnable);
    VL53L0X_Error VL53L0X_SetLimitCheckValue(VL53L0X_DEV Dev, uint16_t LimitCheckId,
    	FixPoint1616_t LimitCheckValue);
    VL53L0X_Error VL53L0X_StopMeasurement(VL53L0X_DEV Dev);
    VL53L0X_Error VL53L0X_GetStopCompletedStatus(VL53L0X_DEV Dev,
    	uint32_t *pStopStatus);
    VL53L0X_Error VL53L0X_SetVcselPulsePeriod(VL53L0X_DEV Dev,
    	VL53L0X_VcselPeriod VcselPeriodType, uint8_t VCSELPulsePeriod);

    /* api_core.h functions */
    VL53L0X_Error VL53L0X_get_info_from_device(VL53L0X_DEV Dev, uint8_t option);
    VL53L0X_Error VL53L0X_device_read_strobe(VL53L0X_DEV Dev);
    VL53L0X_Error VL53L0X_get_measurement_timing_budget_micro_seconds(VL53L0X_DEV Dev,
    		uint32_t *pMeasurementTimingBudgetMicroSeconds);
    VL53L0X_Error VL53L0X_get_vcsel_pulse_period(VL53L0X_DEV Dev,
    	VL53L0X_VcselPeriod VcselPeriodType, uint8_t *pVCSELPulsePeriodPCLK);
    uint8_t VL53L0X_decode_vcsel_period(uint8_t vcsel_period_reg);
    uint32_t VL53L0X_decode_timeout(uint16_t encoded_timeout);
    uint32_t VL53L0X_calc_timeout_us(VL53L0X_DEV Dev,
    		uint16_t timeout_period_mclks,
    		uint8_t vcsel_period_pclks);
    uint32_t VL53L0X_calc_macro_period_ps(VL53L0X_DEV Dev, uint8_t vcsel_period_pclks);
    VL53L0X_Error VL53L0X_measurement_poll_for_completion(VL53L0X_DEV Dev);
    VL53L0X_Error VL53L0X_load_tuning_settings(VL53L0X_DEV Dev,
    		uint8_t *pTuningSettingBuffer);
    VL53L0X_Error VL53L0X_get_pal_range_status(VL53L0X_DEV Dev,
    		uint8_t DeviceRangeStatus,
    		FixPoint1616_t SignalRate,
    		uint16_t EffectiveSpadRtnCount,
    		VL53L0X_RangingMeasurementData_t *pRangingMeasurementData,
    		uint8_t *pPalRangeStatus);
    VL53L0X_Error VL53L0X_calc_sigma_estimate(VL53L0X_DEV Dev,
    	VL53L0X_RangingMeasurementData_t *pRangingMeasurementData,
    	FixPoint1616_t *pSigmaEstimate,
    	uint32_t *pDmax_mm);
    VL53L0X_Error VL53L0X_get_total_signal_rate(VL53L0X_DEV Dev,
    	VL53L0X_RangingMeasurementData_t *pRangingMeasurementData,
    	FixPoint1616_t *ptotal_signal_rate_mcps);
    VL53L0X_Error VL53L0X_get_total_xtalk_rate(VL53L0X_DEV Dev,
    	VL53L0X_RangingMeasurementData_t *pRangingMeasurementData,
    	FixPoint1616_t *ptotal_xtalk_rate_mcps);
    uint32_t VL53L0X_calc_timeout_mclks(VL53L0X_DEV Dev,
    		uint32_t timeout_period_us,
    		uint8_t vcsel_period_pclks);
    uint32_t VL53L0X_isqrt(uint32_t num);
    VL53L0X_Error VL53L0X_calc_dmax(
    	VL53L0X_DEV Dev,
    	FixPoint1616_t totalSignalRate_mcps,
    	FixPoint1616_t totalCorrSignalRate_mcps,
    	FixPoint1616_t pwMult,
    	uint32_t sigmaEstimateP1,
    	FixPoint1616_t sigmaEstimateP2,
    	uint32_t peakVcselDuration_us,
    	uint32_t *pdmax_mm);
    VL53L0X_Error VL53L0X_set_measurement_timing_budget_micro_seconds(VL53L0X_DEV Dev,
    		uint32_t MeasurementTimingBudgetMicroSeconds);
    VL53L0X_Error get_sequence_step_timeout(VL53L0X_DEV Dev,
    				VL53L0X_SequenceStepId SequenceStepId,
    				uint32_t *pTimeOutMicroSecs);
    VL53L0X_Error set_sequence_step_timeout(VL53L0X_DEV Dev,
    					VL53L0X_SequenceStepId SequenceStepId,
    					uint32_t TimeOutMicroSecs);
    uint16_t VL53L0X_encode_timeout(uint32_t timeout_macro_clks);
    VL53L0X_Error VL53L0X_set_vcsel_pulse_period(VL53L0X_DEV Dev,
    	VL53L0X_VcselPeriod VcselPeriodType, uint8_t VCSELPulsePeriodPCLK);
    uint8_t VL53L0X_encode_vcsel_period(uint8_t vcsel_period_pclks);

    /* api_calibration.h functions */
    VL53L0X_Error VL53L0X_apply_offset_adjustment(VL53L0X_DEV Dev);
    VL53L0X_Error VL53L0X_get_offset_calibration_data_micro_meter(VL53L0X_DEV Dev,
    		int32_t *pOffsetCalibrationDataMicroMeter);
    VL53L0X_Error VL53L0X_set_offset_calibration_data_micro_meter(VL53L0X_DEV Dev,
    		int32_t OffsetCalibrationDataMicroMeter);
    VL53L0X_Error VL53L0X_perform_ref_spad_management(VL53L0X_DEV Dev,
    				uint32_t *refSpadCount,
    				uint8_t *isApertureSpads);
    VL53L0X_Error VL53L0X_perform_ref_calibration(VL53L0X_DEV Dev,
    	uint8_t *pVhvSettings, uint8_t *pPhaseCal, uint8_t get_data_enable);
    VL53L0X_Error VL53L0X_perform_vhv_calibration(VL53L0X_DEV Dev,
    	uint8_t *pVhvSettings, const uint8_t get_data_enable,
    	const uint8_t restore_config);
    VL53L0X_Error VL53L0X_perform_single_ref_calibration(VL53L0X_DEV Dev,
    		uint8_t vhv_init_byte);
    VL53L0X_Error VL53L0X_ref_calibration_io(VL53L0X_DEV Dev, uint8_t read_not_write,
    	uint8_t VhvSettings, uint8_t PhaseCal,
    	uint8_t *pVhvSettings, uint8_t *pPhaseCal,
    	const uint8_t vhv_enable, const uint8_t phase_enable);
    VL53L0X_Error VL53L0X_perform_phase_calibration(VL53L0X_DEV Dev,
    	uint8_t *pPhaseCal, const uint8_t get_data_enable,
    	const uint8_t restore_config);
    VL53L0X_Error enable_ref_spads(VL53L0X_DEV Dev,
    				uint8_t apertureSpads,
    				uint8_t goodSpadArray[],
    				uint8_t spadArray[],
    				uint32_t size,
    				uint32_t start,
    				uint32_t offset,
    				uint32_t spadCount,
    				uint32_t *lastSpad);
    void get_next_good_spad(uint8_t goodSpadArray[], uint32_t size,
    			uint32_t curr, int32_t *next);
    uint8_t is_aperture(uint32_t spadIndex);
    VL53L0X_Error enable_spad_bit(uint8_t spadArray[], uint32_t size,
    	uint32_t spadIndex);
    VL53L0X_Error set_ref_spad_map(VL53L0X_DEV Dev, uint8_t *refSpadArray);
    VL53L0X_Error get_ref_spad_map(VL53L0X_DEV Dev, uint8_t *refSpadArray);
    VL53L0X_Error perform_ref_signal_measurement(VL53L0X_DEV Dev,
    		uint16_t *refSignalRate);
    VL53L0X_Error VL53L0X_set_reference_spads(VL53L0X_DEV Dev,
    				 uint32_t count, uint8_t isApertureSpads);

    /* api_strings.h functions */
    VL53L0X_Error VL53L0X_get_device_info(VL53L0X_DEV Dev,
    				VL53L0X_DeviceInfo_t *pVL53L0X_DeviceInfo);
    VL53L0X_Error VL53L0X_check_part_used(VL53L0X_DEV Dev,
    		uint8_t *Revision,
    		VL53L0X_DeviceInfo_t *pVL53L0X_DeviceInfo);

    /* Read function of the ID device */
    virtual int ReadID();

    VL53L0X_Error WaitMeasurementDataReady(VL53L0X_DEV Dev);
    VL53L0X_Error WaitStopCompleted(VL53L0X_DEV Dev);

    /* Write and read functions from I2C */

    VL53L0X_Error VL53L0X_WrByte(VL53L0X_DEV dev, uint8_t index, uint8_t data);
    VL53L0X_Error VL53L0X_WrWord(VL53L0X_DEV dev, uint8_t index, uint16_t data);
    VL53L0X_Error VL53L0X_WrDWord(VL53L0X_DEV dev, uint8_t index, uint32_t data);
    VL53L0X_Error VL53L0X_RdByte(VL53L0X_DEV dev, uint8_t index, uint8_t *data);
    VL53L0X_Error VL53L0X_RdWord(VL53L0X_DEV dev, uint8_t index, uint16_t *data);
    VL53L0X_Error VL53L0X_RdDWord(VL53L0X_DEV dev, uint8_t index, uint32_t *data);
    VL53L0X_Error VL53L0X_UpdateByte(VL53L0X_DEV dev, uint8_t index, uint8_t AndData, uint8_t OrData);

    VL53L0X_Error VL53L0X_WriteMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t *pdata, uint32_t count);
    VL53L0X_Error VL53L0X_ReadMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t *pdata, uint32_t count);

	VL53L0X_Error VL53L0X_I2CWrite(uint8_t dev, uint8_t index, uint8_t *data, uint16_t number_of_bytes);
	VL53L0X_Error VL53L0X_I2CRead(uint8_t dev, uint8_t index, uint8_t *data, uint16_t number_of_bytes);

    VL53L0X_Error VL53L0X_PollingDelay(VL53L0X_DEV Dev); /* usually best implemented as a real function */

    int IsPresent()
    {
       int status;

       status=ReadID();
       if(status)
          VL53L0X_ErrLog("Failed to read ID device. Device not present!\n\r");
       return status;
    }
    int StopRangeMeasurement(OperatingMode operating_mode);
    int GetRangeMeas(OperatingMode operating_mode, VL53L0X_RangingMeasurementData_t *Data);
    int RangeSetLowThreshold(uint16_t threshold);
    int RangeSetHighThreshold(uint16_t threshold);
    int GetRangeError(VL53L0X_RangingMeasurementData_t *Data, VL53L0X_RangingMeasurementData_t RangeData);
    int RangeMeasPollSingleShot();
    int RangeMeasPollContinuousMode();
    int RangeMeasIntContinuousMode(void (*fptr)(void));

 protected:
    VL53L0X_DeviceInfo_t DeviceInfo;

    /* IO Device */
    TwoWire *dev_i2c;
    /* Digital out pin */
	int gpio0;
    /* Device data */
	VL53L0X_Dev_t MyDevice;
    VL53L0X_DEV Device;
};


#endif /* _VL53L0X_CLASS_H_ */
