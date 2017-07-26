/**
 ******************************************************************************
 * @file    stmpe1600_class.h
 * @author  AST / EST
 * @version V0.0.1
 * @date    14-April-2015
 * @brief   Header file for component stmpe1600
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
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
 *       without specific prior written permission.
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
#ifndef     __STMPE1600_CLASS
#define     __STMPE1600_CLASS
/* Includes ------------------------------------------------------------------*/
#include <Wire.h>

#define STMPE1600_DEF_DEVICE_ADDRESS  (uint8_t)0x42*2   
#define STMPE1600_DEF_DIGIOUT_LVL      1

/**  STMPE1600 registr map **/
#define ChipID_0_7      (uint8_t)0x00
#define ChipID_8_15     (uint8_t)0x01
#define VersionId       (uint8_t)0x02
#define SYS_CTRL        (uint8_t)0x03
#define IEGPIOR_0_7     (uint8_t)0x08
#define IEGPIOR_8_15    (uint8_t)0x09
#define ISGPIOR_0_7	    (uint8_t)0x0A
#define ISGPIOR_8_15    (uint8_t)0x0B
#define GPMR_0_7        (uint8_t)0x10
#define GPMR_8_15       (uint8_t)0x11
#define GPSR_0_7        (uint8_t)0x12
#define GPSR_8_15       (uint8_t)0x13
#define GPDR_0_7        (uint8_t)0x14
#define GPDR_8_15       (uint8_t)0x15
#define GPIR_0_7        (uint8_t)0x16
#define GPIR_8_15	    (uint8_t)0x17

#define SOFT_RESET      (uint8_t)0x80

typedef enum {
  // GPIO Expander pin names
  GPIO_0=0,
  GPIO_1,
  GPIO_2,
  GPIO_3,            
  GPIO_4,
  GPIO_5,
  GPIO_6,
  GPIO_7,            
  GPIO_8,
  GPIO_9,
  GPIO_10,
  GPIO_11,            
  GPIO_12,
  GPIO_13,
  GPIO_14,
  GPIO_15,
  NOT_CON
} ExpGpioPinName;   

typedef enum {
    EXP_GPIO_INPUT = 0,
    EXP_GPIO_OUTPUT,
    EXP_GPIO_NOT_CONNECTED
} ExpGpioPinDirection;

/* Classes -------------------------------------------------------------------*/
/** Class representing a single stmpe1600 GPIO expander output pin
 */
class STMPE1600DigiOut {
	
 public: 
    /** Constructor
     * @param[in] &i2c device I2C to be used for communication
     * @param[in] outpinname the desired out pin name to be created
     * @param[in] DevAddr the stmpe1600 I2C device addres (deft STMPE1600_DEF_DEVICE_ADDRESS)
     * @param[in] lvl the default ot pin level  
     */ 
    STMPE1600DigiOut (TwoWire *i2c, ExpGpioPinName outpinname, uint8_t DevAddr=STMPE1600_DEF_DEVICE_ADDRESS, bool lvl=STMPE1600_DEF_DIGIOUT_LVL): dev_i2c(i2c), expdevaddr(DevAddr), exppinname(outpinname) 
    {
       uint8_t data[2];				
       if (exppinname == NOT_CON) return;
       /* set the exppinname as output */
       STMPE1600DigiOut_I2CRead(data, expdevaddr, GPDR_0_7, 1);
       STMPE1600DigiOut_I2CRead(&data[1], expdevaddr, GPDR_8_15, 1);			
       *(uint16_t*)data = *(uint16_t*)data | (1<<(uint16_t)exppinname);  // set gpio as out 			
       STMPE1600DigiOut_I2CWrite(data, expdevaddr, GPDR_0_7, 1);
       STMPE1600DigiOut_I2CWrite(&data[1], expdevaddr, GPDR_8_15, 1);			
       write(lvl);
    }   

	/**
	 * @brief       Write on the out pin
	 * @param[in]   lvl level to write
	 * @return      0 on Success
	 */			
    void write (int lvl) 
    {
       uint8_t data[2];			
       if (exppinname == NOT_CON) return;			
       /* set the exppinname state to lvl */
       STMPE1600DigiOut_I2CRead(data, expdevaddr, GPSR_0_7, 2);
       *(uint16_t*)data = *(uint16_t*)data & (uint16_t)(~(1<<(uint16_t)exppinname));  // set pin mask 			
       if (lvl) *(uint16_t*)data = *(uint16_t*)data | (uint16_t)(1<<(uint16_t)exppinname);
       STMPE1600DigiOut_I2CWrite(data, expdevaddr, GPSR_0_7, 2);
    }
		
 private:
    TwoWire *dev_i2c; 
    uint8_t expdevaddr;
    ExpGpioPinName exppinname;

    int STMPE1600DigiOut_I2CWrite(uint8_t* pBuffer, uint8_t DeviceAddr, uint8_t RegisterAddr, uint16_t NumByteToWrite)
    {
      dev_i2c->beginTransmission(((uint8_t)(((DeviceAddr) >> 1) & 0x7F)));
  
      dev_i2c->write(RegisterAddr);
      for (int i = 0 ; i < NumByteToWrite ; i++)
        dev_i2c->write(pBuffer[i]);
  
      dev_i2c->endTransmission(true);
  
      return 0;
    }

    int STMPE1600DigiOut_I2CRead(uint8_t* pBuffer, uint8_t DeviceAddr, uint8_t RegisterAddr, uint16_t NumByteToRead)
    {
      dev_i2c->beginTransmission(((uint8_t)(((DeviceAddr) >> 1) & 0x7F)));
      dev_i2c->write(RegisterAddr);
      dev_i2c->endTransmission(false);
  
      dev_i2c->requestFrom(((uint8_t)(((DeviceAddr) >> 1) & 0x7F)), (byte) NumByteToRead);

      int i=0;
      while (dev_i2c->available())
      {
        pBuffer[i] = dev_i2c->read();
        i++;
      }
  
      return 0;
    }
};

#endif // __STMPE1600_CLASS
