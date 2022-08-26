
//  Copyright (c) 2003-2021 Xsens Technologies B.V. or subsidiaries worldwide.
//  All rights reserved.
//  
//  Redistribution and use in source and binary forms, with or without modification,
//  are permitted provided that the following conditions are met:
//  
//  1.	Redistributions of source code must retain the above copyright notice,
//  	this list of conditions, and the following disclaimer.
//  
//  2.	Redistributions in binary form must reproduce the above copyright notice,
//  	this list of conditions, and the following disclaimer in the documentation
//  	and/or other materials provided with the distribution.
//  
//  3.	Neither the names of the copyright holders nor the names of their contributors
//  	may be used to endorse or promote products derived from this software without
//  	specific prior written permission.
//  
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
//  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
//  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
//  THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
//  OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
//  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY OR
//  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.THE LAWS OF THE NETHERLANDS 
//  SHALL BE EXCLUSIVELY APPLICABLE AND ANY DISPUTES SHALL BE FINALLY SETTLED UNDER THE RULES 
//  OF ARBITRATION OF THE INTERNATIONAL CHAMBER OF COMMERCE IN THE HAGUE BY ONE OR MORE 
//  ARBITRATORS APPOINTED IN ACCORDANCE WITH SAID RULES.
//  

#include "mtssp_driver_i2c.h"
#include "stm32f4xx_hal.h"
#include "init_i2c.h"
#include "wait.h"
#include <string.h>

/*!	\class MtsspDriverI2c
	\brief MtsspDriver for the I2C bus
*/

#define TIMEOUT_MS 100


/*!	\brief Constructs a MtsspDriverI2c
	\param[in] deviceAddress The 7 bit I2C address of the MTi
*/
MtsspDriverI2c::MtsspDriverI2c(uint8_t deviceAddress)
	: m_deviceAddress(deviceAddress)
{
}


/*!	\brief Perform a blocking write transfer on the I2C bus
	\param[in] opcode Opcode to use
	\param[in] data Pointer to data to be written
	\param[in] dataLength Number of data bytes to write
*/
void MtsspDriverI2c::write(uint8_t opcode, uint8_t const* data, int dataLength)
{
	uint8_t transferBuffer[8];
	confirm(dataLength < sizeof(transferBuffer));
	transferBuffer[0] = opcode;
	memcpy(&transferBuffer[1], data, dataLength);
	HAL_I2C_Master_Transmit(&hi2c, (m_deviceAddress << 1), transferBuffer, 1 + dataLength, TIMEOUT_MS);
}


/*!	\brief Perform a blocking read transfer on the I2C bus
	\param[in] opcode Opcode to use
	\param[out] data Pointer to result buffer
	\param[in] dataLength Number of data bytes to read
*/
void MtsspDriverI2c::read(uint8_t opcode, uint8_t* dest, int dataLength)
{
	HAL_I2C_Mem_Read(&hi2c, (m_deviceAddress << 1), opcode, 1, dest, dataLength, TIMEOUT_MS);
}


/*!	\brief Perform a blocking write transfer on the I2C bus
	\param[in] data Pointer to data to be written
	\param[in] dataLength Number of data bytes to write
*/
void MtsspDriverI2c::writeRaw(uint8_t const* data, int dataLength)
{
	HAL_I2C_Master_Transmit(&hi2c, (m_deviceAddress << 1), (uint8_t*)data, dataLength, TIMEOUT_MS);
}








