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
#include "stmpe1600_class.h"


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
     * @param[in] &i2c device I2C to be used for communication
     * @param[in] &pin_gpio1 pin Mbed InterruptIn PinName to be used as component GPIO_1 INT
     * @param[in] DevAddr device address, 0x29 by default  
     */
    VL53L0X(TwoWire *i2c, int pin, int pin_gpio1, uint8_t DevAddr=VL53L0x_DEFAULT_DEVICE_ADDRESS) : RangeSensor(), dev_i2c(i2c), gpio0(pin), gpio1Int(pin_gpio1)
    {
       MyDevice.I2cDevAddr=DevAddr;		 
       MyDevice.comms_type=1; // VL53L0X_COMMS_I2C
       MyDevice.comms_speed_khz=400;
       Device=&MyDevice;
       pinMode(gpio0, OUTPUT);
       expgpio0=NULL;
    }  
    
    /** Constructor 2 (STMPE1600DigiOut)
     * @param[in] i2c device I2C to be used for communication
     * @param[in] &pin Gpio Expander STMPE1600DigiOut pin to be used as component GPIO_0 CE
     * @param[in] pin_gpio1 pin Mbed InterruptIn PinName to be used as component GPIO_1 INT
     * @param[in] device address, 0x29 by default  
     */		
    VL53L0X(TwoWire *i2c, STMPE1600DigiOut *pin, int pin_gpio1, uint8_t DevAddr=VL53L0x_DEFAULT_DEVICE_ADDRESS) : RangeSensor(), dev_i2c(i2c), expgpio0(pin), gpio1Int(pin_gpio1)
    {
       MyDevice.I2cDevAddr=DevAddr;		 
       MyDevice.comms_type=1; // VL53L0X_COMMS_I2C
       MyDevice.comms_speed_khz=400;
       Device=&MyDevice;
       gpio0=0;
    }  	 
    
   /** Destructor
    */
    virtual ~VL53L0X(){}     
    /* warning: VL53L0X class inherits from GenericSensor, RangeSensor and LightSensor, that haven`t a destructor.
       The warning should request to introduce a virtual destructor to make sure to delete the object */

	/*** Interface Methods ***/	
	/*** High level API ***/		
	/**
	 * @brief       PowerOn the sensor
	 * @return      void
	 */		
    /* turns on the sensor */		 
    void VL53L0X_On(void)
    {
       if(gpio0)
         digitalWrite(gpio0, HIGH);		   
       else if(expgpio0) 
         expgpio0->write(1);
       delay(10);    
    } 

	/**
	 * @brief       PowerOff the sensor
	 * @return      void
	 */		
    /* turns off the sensor */
    void VL53L0X_Off(void) 
    {
       if(gpio0) 
         digitalWrite(gpio0, LOW);
       else if(expgpio0) 
         expgpio0->write(0);
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
 * @brief Start continuous ranging mode
 *
 * @details End user should ensure device is in idle state and not already running
 * @return      0 on success
 */		
//    int RangeStartContinuousMode()
//    {
//       return VL6180x_RangeStartContinuousMode(Device);
//			return 1;
//    }

/**
 * @brief Start single shot ranging measure
 *
 * @details End user should ensure device is in idle state and not already running
 * @return      0 on success 
 */		
//    int RangeStartSingleShot()
//    {
//       return VL6180x_RangeStartSingleShot(Device);
//			return 1;
//    }

/**
 * @brief Set maximum convergence time
 *
 * @par Function Description
 * Setting a low convergence time can impact maximal detectable distance.
 * Refer to VL6180x Datasheet Table 7 : Typical range convergence time.
 * A typical value for up to x3 scaling is 50 ms
 *
 * @param MaxConTime_msec
 * @return 0 on success. <0 on error. >0 for calibration warning status
 */		
//    int RangeSetMaxConvergenceTime(uint8_t MaxConTime_msec)
//    {
//       return VL6180x_RangeSetMaxConvergenceTime(Device, MaxConTime_msec);
//			return 1;
//    }

/**
  * @brief Single shot Range measurement in polling mode.
  *
  * @par Function Description
  * Kick off a new single shot range  then wait for ready to retrieve it by polling interrupt status \n
  * Ranging must be prepared by a first call to  @a VL6180x_Prepare() and it is safer to clear  very first poll call \n
  * This function reference VL6180x_PollDelay(dev) porting macro/call on each polling loop,
  * but PollDelay(dev) may never be called if measure in ready on first poll loop \n
  * Should not be use in continuous mode operation as it will stop it and cause stop/start misbehaviour \n
  * \n This function clears Range Interrupt status , but not error one. For that uses  @a VL6180x_ClearErrorInterrupt() \n
  * This range error is not related VL6180x_RangeData_t::errorStatus that refer measure status \n
  * 
  * @param pRangeData   Will be populated with the result ranging data @a  VL6180x_RangeData_t
  * @return 0 on success , @a #RANGE_ERROR if device reports an error case in it status (not cleared) use
  *
  * \sa ::VL6180x_RangeData_t
  */		
//    int RangePollMeasurement(VL53L0X_RangingMeasurementData_t *pRangeData)
//    {
//       return VL6180x_RangePollMeasurement(Device, pRangeData);
//			return 1;
//    }

/**
 * @brief Check for measure readiness and get it if ready
 *
 * @par Function Description
 * Using this function is an alternative to @a VL6180x_RangePollMeasurement() to avoid polling operation. This is suitable for applications
 * where host CPU is triggered on a interrupt (not from VL6180X) to perform ranging operation. In this scenario, we assume that the very first ranging
 * operation is triggered by a call to @a VL6180x_RangeStartSingleShot(). Then, host CPU regularly calls @a VL6180x_RangeGetMeasurementIfReady() to
 * get a distance measure if ready. In case the distance is not ready, host may get it at the next call.\n
 *
 * @warning 
 * This function does not re-start a new measurement : this is up to the host CPU to do it.\n 
 * This function clears Range Interrupt for measure ready , but not error interrupts. For that, uses  @a VL6180x_ClearErrorInterrupt() \n
 *
 * @param pRangeData  Will be populated with the result ranging data if available
 * @return  0 when measure is ready pRange data is updated (untouched when not ready),  >0 for warning and @a #NOT_READY if measurement not yet ready, <0 for error @a #RANGE_ERROR if device report an error,
 */		
//    int RangeGetMeasurementIfReady(VL53L0X_RangingMeasurementData_t *pRangeData)
//    {
//       return VL6180x_RangeGetMeasurementIfReady(Device, pRangeData);
//			return 1;
//    }

/**
 * @brief Retrieve range measurements set  from device
 *
 * @par Function Description
 * The measurement is made of range_mm status and error code @a VL6180x_RangeData_t \n
 * Based on configuration selected extra measures are included.
 *
 * @warning should not be used in continuous if wrap around filter is active \n
 * Does not perform any wait nor check for result availability or validity.
 *\sa VL6180x_RangeGetResult for "range only" measurement
 *
 * @param pRangeData  Pointer to the data structure to fill up
 * @return            0 on success
 */		
//    int RangeGetMeasurement(VL53L0X_RangingMeasurementData_t *pRangeData)
//    {
//       return VL6180x_RangeGetMeasurement(Device, pRangeData);
//			return 1;
//    }

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
 * @brief Wait for device to be ready (before a new ranging command can be issued by application)
 * @param MaxLoop    Max Number of i2c polling loop see @a #msec_2_i2cloop
 * @return           0 on success. <0 when fail \n
 *                   @ref VL6180x_ErrCode_t::TIME_OUT for time out \n
 *                   @ref VL6180x_ErrCode_t::INVALID_PARAMS if MaxLop<1
 */		
//    int RangeWaitDeviceReady(int MaxLoop )
//    {
//       return VL6180x_RangeWaitDeviceReady(Device, MaxLoop);
//			return 1;
//    }

/**
 * @brief Program Inter measurement period (used only in continuous mode)
 *
 * @par Function Description
 * When trying to set too long time, it returns #INVALID_PARAMS
 *
 * @param InterMeasTime_msec Requires inter-measurement time in msec
 * @return 0 on success
 */		
//    int RangeSetInterMeasPeriod(uint32_t  InterMeasTime_msec)
//    {
//       return VL6180x_RangeSetInterMeasPeriod(Device, InterMeasTime_msec);
//			return 1;
//    }

/**
 * @brief Set device ranging scaling factor
 *
 * @par Function Description
 * The ranging scaling factor is applied on the raw distance measured by the device to increase operating ranging at the price of the precision.
 * Changing the scaling factor when device is not in f/w standby state (free running) is not safe.
 * It can be source of spurious interrupt, wrongly scaled range etc ...
 * @warning __This  function doesns't update high/low threshold and other programmed settings linked to scaling factor__.
 *  To ensure proper operation, threshold and scaling changes should be done following this procedure: \n
 *  @li Set Group hold  : @a VL6180x_SetGroupParamHold() \n
 *  @li Get Threshold @a VL6180x_RangeGetThresholds() \n
 *  @li Change scaling : @a VL6180x_UpscaleSetScaling() \n
 *  @li Set Threshold : @a VL6180x_RangeSetThresholds() \n
 *  @li Unset Group Hold : @a VL6180x_SetGroupParamHold()
 *
 * @param scaling  Scaling factor to apply (1,2 or 3)
 * @return          0 on success when up-scale support is not configured it fail for any
 *                  scaling than the one statically configured.
 */
//    int UpscaleSetScaling(uint8_t scaling)
//    {
//       return VL6180x_UpscaleSetScaling(Device, scaling);
//			return 1;
//    }

/**
 * @brief Get current ranging scaling factor
 *
 * @return    The current scaling factor
 */				
//    int UpscaleGetScaling()
//    {
//       return VL6180x_UpscaleGetScaling(Device);
//			return 1;
//    }

/**
 * @brief Get the maximal distance for actual scaling
 * @par Function Description
 * Do not use prior to @a VL6180x_Prepare() or at least @a VL6180x_InitData()
 *
 * Any range value more than the value returned by this function is to be considered as "no target detected"
 * or "no target in detectable range" \n
 * @warning The maximal distance depends on the scaling
 *
 * @return    The maximal range limit for actual mode and scaling
 */		
//    uint16_t GetUpperLimit()
//    {
//       return VL6180x_GetUpperLimit(Device);
//			return 1;
//    }

/**
 * @brief Apply low and high ranging thresholds that are considered only in continuous mode
 *
 * @par Function Description
 * This function programs low and high ranging thresholds that are considered in continuous mode : 
 * interrupt will be raised only when an object is detected at a distance inside this [low:high] range.  
 * The function takes care of applying current scaling factor if any.\n
 * To be safe, in continuous operation, thresholds must be changed under "group parameter hold" cover.
 * Group hold can be activated/deactivated directly in the function or externally (then set 0)
 * using /a VL6180x_SetGroupParamHold() function.
 *
 * @param low      Low threshold in mm
 * @param high     High threshold in mm
 * @param SafeHold  Use of group parameters hold to surround threshold programming.
 * @return  0 On success
 */		
//    int RangeSetThresholds(uint16_t low, uint16_t high, int SafeHold)
//    {
//       return VL6180x_RangeSetThresholds(Device, low, high, SafeHold);
//			return 1;
//    }

/**
 * @brief  Get scaled high and low threshold from device
 *
 * @par Function Description
 * Due to scaling factor, the returned value may be different from what has been programmed first (precision lost).
 * For instance VL6180x_RangeSetThresholds(dev,11,22) with scale 3
 * will read back 9 ((11/3)x3) and 21 ((22/3)x3).
 *
 * @param low  scaled low Threshold ptr  can be NULL if not needed
 * @param high scaled High Threshold ptr can be NULL if not needed
 * @return 0 on success, return value is undefined if both low and high are NULL
 * @warning return value is undefined if both low and high are NULL
 */
//    int RangeGetThresholds(uint16_t *low, uint16_t *high)
//    {
//       return VL6180x_RangeGetThresholds(Device, low, high);
//			return 1;
//    }

/**
 * @brief Set ranging raw thresholds (scaling not considered so not recommended to use it)
 *
 * @param low  raw low threshold set to raw register
 * @param high raw high threshold set to raw  register
 * @return 0 on success
 */			
//    int RangeSetRawThresholds(uint8_t low, uint8_t high)
//    {
//       return VL6180x_RangeSetRawThresholds(Device, low, high);
//			return 1;
//    }

/**
 * @brief Set Early Convergence Estimate ratio
 * @par Function Description
 * For more information on ECE check datasheet
 * @warning May return a calibration warning in some use cases
 *
 * @param FactorM    ECE factor M in M/D
 * @param FactorD    ECE factor D in M/D
 * @return           0 on success. <0 on error. >0 on warning
 */		
//    int RangeSetEceFactor(uint16_t  FactorM, uint16_t FactorD)
//    {
//       return VL6180x_RangeSetEceFactor(Device, FactorM, FactorD);
//			return 1;
//    }

/**
 * @brief Set Early Convergence Estimate state (See #SYSRANGE_RANGE_CHECK_ENABLES register)
 * @param enable    State to be set 0=disabled, otherwise enabled
 * @return          0 on success
 */		
//    int RangeSetEceState(int enable)
//    {
//       return VL6180x_RangeSetEceState(Device, enable);
//			return 1;
//    }

/**
 * @brief Set activation state of the wrap around filter
 * @param state New activation state (0=off,  otherwise on)
 * @return      0 on success
 */			
//    int FilterSetState(int state)
//    {
//       return VL6180x_FilterSetState(Device, state);
//			return 1;
//    }

/**
 * Get activation state of the wrap around filter
 * @return     Filter enabled or not, when filter is not supported it always returns 0S
 */			
//    int FilterGetState()
//    {
//       return VL6180x_FilterGetState(Device);
//			return 1;
//    }

/**
 * @brief Set activation state of  DMax computation
 * @param state New activation state (0=off,  otherwise on)
 * @return      0 on success
 */		
//    int DMaxSetState(int state)
//    {
//       return VL6180x_DMaxSetState(Device, state);
//			return 1;
//    }

/**
 * Get activation state of DMax computation
 * @return     Filter enabled or not, when filter is not supported it always returns 0S
 */		
//    int DMaxGetState()
//    {
//       return VL6180x_DMaxGetState(Device);
//			return 1;
//    }

/**
 * @brief Set ranging mode and start/stop measure (use high level functions instead : @a VL6180x_RangeStartSingleShot() or @a VL6180x_RangeStartContinuousMode())
 *
 * @par Function Description
 * When used outside scope of known polling single shot stopped state, \n
 * user must ensure the device state is "idle" before to issue a new command.
 *
 * @param mode  A combination of working mode (#MODE_SINGLESHOT or #MODE_CONTINUOUS) and start/stop condition (#MODE_START_STOP) \n
 * @return      0 on success
 */		
//    int RangeSetSystemMode(uint8_t mode)
//    {
//       return VL6180x_RangeSetSystemMode(Device, mode);
//			return 1;
//    }

/** @}  */ 

/** @defgroup api_ll_range_calibration Ranging calibration functions
 *  @brief    Ranging calibration functions
 *  @ingroup api_ll
 *  @{  
 */
/**
 * @brief Get part to part calibration offset
 *
 * @par Function Description
 * Should only be used after a successful call to @a VL6180x_InitData to backup device nvm value
 *
 * @return part to part calibration offset from device
 */		
//    int8_t GetOffsetCalibrationData()
//    {
//       return VL6180x_GetOffsetCalibrationData(Device);
//			return 1;
//    }

/**
 * Set or over-write part to part calibration offset
 * \sa VL6180x_InitData(), VL6180x_GetOffsetCalibrationData()
 * @param offset   Offset
 */		
//    void SetOffsetCalibrationData(int8_t offset)
//    {
//       return VL6180x_SetOffsetCalibrationData(Device, offset);
//			return;
//    }

/**
 * @brief Set Cross talk compensation rate
 *
 * @par Function Description
 * It programs register @a #SYSRANGE_CROSSTALK_COMPENSATION_RATE
 *
 * @param Rate Compensation rate (9.7 fix point) see datasheet for details
 * @return     0 on success
 */		
//    int SetXTalkCompensationRate(uint16_t Rate)
//    {
//       return VL6180x_SetXTalkCompensationRate(Device, Rate);
//			return 1;
//    }
/** @}  */

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
	
 private:		
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


    VL53L0X_DeviceInfo_t DeviceInfo;
		
    /* IO Device */
    TwoWire *dev_i2c;
    /* Digital out pin */
	int gpio0;
    /* GPIO expander */
    STMPE1600DigiOut *expgpio0;
	int gpio1Int;
    /* Device data */
	VL53L0X_Dev_t MyDevice;
    VL53L0X_DEV Device;  
};


#endif /* _VL53L0X_CLASS_H_ */


