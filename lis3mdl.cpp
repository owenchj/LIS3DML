/**
******************************************************************************
* @file    x_cube_mems_lis3mdl.h
* @author  AST / EST
* @version V0.0.1
* @date    9-December-2014
* @brief   Implementation file for component LIS3MDL
******************************************************************************
* @attention
*
* <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
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
#include "mbed.h"
#include "lis3mdl.h"
#include "lis3mdl_platform.h"
#include <math.h>

/* Methods -------------------------------------------------------------------*/

/**
 * @brief Read data from LIS3MDL Magnetic sensor and calculate Magnetic in mgauss.
 * @param float *pfData
 * @retval None.
 */
void LIS3MDL::GetAxes(AxesRaw_TypeDef *pData)
{
    char tempReg[2];
    int16_t pDataRaw[3];
    float sensitivity = 0;
    int ret;

    GetAxesRaw(pDataRaw);

    tempReg[0] = LIS3MDL_M_CTRL_REG2_M;
    _i2c.write(LIS3MDL_M_MEMS_ADDRESS, tempReg, 1, true);
    ret = _i2c.read(LIS3MDL_M_MEMS_ADDRESS, &tempReg[1], 1);

    if (ret == 0)
    {
      tempReg[1] &= LIS3MDL_M_FS_MASK;

      switch(tempReg[1])
      {
        case LIS3MDL_M_FS_4:
          sensitivity = 0.14;
          break;
        case LIS3MDL_M_FS_8:
          sensitivity = 0.29;
          break;
        case LIS3MDL_M_FS_12:
          sensitivity = 0.43;
          break;
        case LIS3MDL_M_FS_16:
          sensitivity = 0.58;
          break;
      }
    }

    pData->AXIS_X = (int32_t)(pDataRaw[0] * sensitivity);
    pData->AXIS_Y = (int32_t)(pDataRaw[1] * sensitivity);
    pData->AXIS_Z = (int32_t)(pDataRaw[2] * sensitivity);

}

/**
 * @brief Read raw data from LIS3MDL Magnetic sensor output register.
 * @param float *pfData
 * @retval None.
 */
void LIS3MDL::GetAxesRaw(int16_t *pData)
{
    char tempReg[7];
    int ret;

    pData[0] = pData[1] = pData[2] = 0;

    tempReg[0] = LIS3MDL_M_OUT_X_L_M;

    _i2c.write(LIS3MDL_M_MEMS_ADDRESS, tempReg, 1, true);
    ret = _i2c.read(LIS3MDL_M_MEMS_ADDRESS, &tempReg[1], 6);

    if (ret == 0)
    {
      pData[0] = ((((int16_t)tempReg[1]) << 8)+(int16_t)tempReg[0]);
      pData[1] = ((((int16_t)tempReg[3]) << 8)+(int16_t)tempReg[2]);
      pData[2] = ((((int16_t)tempReg[5]) << 8)+(int16_t)tempReg[4]);
    }
}

/**
 * @brief  Read ID address of LIS3MDL
 * @param  Device ID address
 * @retval ID name
 */
uint8_t LIS3MDL::ReadID(void)
{
    char tmp[2];
    int ret;

    tmp[0] = LIS3MDL_M_WHO_AM_I_ADDR;

    /* Read WHO I AM register */
    _i2c.write(LIS3MDL_M_MEMS_ADDRESS, tmp, 1, true);
    ret =_i2c.read(LIS3MDL_M_MEMS_ADDRESS, &tmp[1], 1);

    /* Return the ID */
    return ((ret == 0) ? (uint8_t)tmp[1] : 0);
}

/**
 * @brief  Set LIS3MDL Initialization.
 * @param  InitStruct: it contains the configuration setting for the LIS3MDL.
 * @retval None
 */
void LIS3MDL::Init() {

    char tmp1[2];
    int ret;

    /****** Magnetic sensor *******/
    tmp1[0] = LIS3MDL_M_CTRL_REG3_M;
    _i2c.write(LIS3MDL_M_MEMS_ADDRESS, tmp1, 1, true);
    ret =_i2c.read(LIS3MDL_M_MEMS_ADDRESS, &tmp1[1], 1);

    /* Conversion mode selection */
    if (ret == 0)
    {
      tmp1[1] &= ~(LIS3MDL_M_MD_MASK);
      tmp1[1] |= LIS3MDL_M_MD_CONTINUOUS;

      ret = _i2c.write(LIS3MDL_M_MEMS_ADDRESS, tmp1, 2, true);
    }

    if (ret == 0)
    {
      tmp1[0] = LIS3MDL_M_CTRL_REG1_M;
      _i2c.write(LIS3MDL_M_MEMS_ADDRESS, tmp1, 1, true);
      ret =_i2c.read(LIS3MDL_M_MEMS_ADDRESS, &tmp1[1], 1);
    }

    if (ret == 0)
    {
      /* Output data rate selection */
      tmp1[1] &= ~(LIS3MDL_M_DO_MASK);
      tmp1[1] |= LIS3MDL_M_DO_80;

      /* X and Y axes Operative mode selection */
      tmp1[1] &= ~(LIS3MDL_M_OM_MASK);
      tmp1[1] |= LIS3MDL_M_OM_HP;

      ret = _i2c.write(LIS3MDL_M_MEMS_ADDRESS, tmp1, 2, true);
    }

    if (ret == 0)
    {
      tmp1[0] = LIS3MDL_M_CTRL_REG2_M;
      _i2c.write(LIS3MDL_M_MEMS_ADDRESS, tmp1, 1, true);
      ret =_i2c.read(LIS3MDL_M_MEMS_ADDRESS, &tmp1[1], 1);
    }

    if (ret == 0)
    {
      /* Full scale selection */
      tmp1[1] &= ~(LIS3MDL_M_FS_MASK);
      tmp1[1] |= LIS3MDL_M_FS_4;

      ret = _i2c.write(LIS3MDL_M_MEMS_ADDRESS, tmp1, 2, true);
    }

    /******************************/

    if (ret == 0)
    {
      if(ReadID() == I_AM_LIS3MDL_M)
      {
          LIS3MDLInitialized = 1;
      }
    }

    return;
}
