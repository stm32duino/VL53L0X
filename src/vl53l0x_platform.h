/*******************************************************************************
Copyright © 2015, STMicroelectronics International N.V.
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
********************************************************************************/


#ifndef _VL53L0X_PLATFORM_H_
#define _VL53L0X_PLATFORM_H_

#include "vl53l0x_def.h"
#include "vl53l0x_platform_log.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file vl53l0_platform.h
 *
 * @brief All end user OS/platform/application porting
 */
 
/**
 * @defgroup VL53L0X_platform_group VL53L0 Platform Functions
 * @brief    VL53L0 Platform Functions
 *  @{
 */

/**
 * @struct  VL53L0X_Dev_t
 * @brief    Generic PAL device type that does link between API and platform abstraction layer
 *
 */
typedef struct {
    VL53L0X_DevData_t Data;               /*!< embed ST Ewok Dev  data as "Data"*/

    /*!< user specific field */
    uint8_t   I2cDevAddr;                /*!< i2c device address user specific field */
    uint8_t   comms_type;                /*!< Type of comms : VL53L0X_COMMS_I2C or VL53L0X_COMMS_SPI */
    uint16_t  comms_speed_khz;           /*!< Comms speed [kHz] : typically 400kHz for I2C           */

} VL53L0X_Dev_t;


/**
 * @brief   Declare the device Handle as a pointer of the structure @a VL53L0X_Dev_t.
 *
 */
typedef VL53L0X_Dev_t* VL53L0X_DEV;

/**
 * @def PALDevDataGet
 * @brief Get ST private structure @a VL53L0X_DevData_t data access
 *
 * @param Dev       Device Handle
 * @param field     ST structure field name
 * It maybe used and as real data "ref" not just as "get" for sub-structure item
 * like PALDevDataGet(FilterData.field)[i] or PALDevDataGet(FilterData.MeasurementIndex)++
 */
#define PALDevDataGet(Dev, field) (Dev->Data.field)

/**
 * @def PALDevDataSet(Dev, field, data)
 * @brief  Set ST private structure @a VL53L0X_DevData_t data field
 * @param Dev       Device Handle
 * @param field     ST structure field name
 * @param data      Data to be set
 */
#define PALDevDataSet(Dev, field, data) (Dev->Data.field)=(data)

/** @} end of VL53L0X_platform_group */

#ifdef __cplusplus
}
#endif

#endif  /* _VL53L0X_PLATFORM_H_ */




