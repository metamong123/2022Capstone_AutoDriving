
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

#include "init_gpio.h"
#include "board.h"
#include "stm32f4xx_hal.h"
#include "gpio_interrupt_interface.h"


static GpioInterruptInterface* g_gpioInterruptInterface = 0;


/*!	\brief Initializes the GPIO pins of the Nucleo board to support MTSSP in either SPI or I2C mode
	\param[in] busInterfaceType: BUS_Spi or BUS_I2c
*/
void initGpio(BusType busType)
{
	// Enable GPIO Port clocks:
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();

	// Configure debug toggles:
	{
		HAL_GPIO_WritePin(DEBUG_PORT, DEBUG_PIN_1 | DEBUG_PIN_2 | DEBUG_PIN_3, GPIO_PIN_RESET);
		GPIO_InitTypeDef GPIO_InitStruct;
		GPIO_InitStruct.Pin = DEBUG_PIN_1 | DEBUG_PIN_2 | DEBUG_PIN_3;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		HAL_GPIO_Init(DEBUG_PORT, &GPIO_InitStruct);
	}

	// Configure DataReady as input with IRQ:
	{
		GPIO_InitTypeDef GPIO_InitStruct;
		GPIO_InitStruct.Pin = DATA_READY_PIN;
		GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
		GPIO_InitStruct.Pull = GPIO_PULLDOWN;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(DATA_READY_PORT, &GPIO_InitStruct);
		HAL_NVIC_SetPriority(DATA_READY_IRQ_NR, 0, 0);
		HAL_NVIC_EnableIRQ(DATA_READY_IRQ_NR);
	}

	// Configure Reset as output:
	{
		HAL_GPIO_WritePin(RESET_PORT, RESET_PIN, GPIO_PIN_RESET);
		GPIO_InitTypeDef GPIO_InitStruct;
		GPIO_InitStruct.Pin = RESET_PIN;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(RESET_PORT, &GPIO_InitStruct);
	}

	// Configure PSEL0 pin as output:
	{
		GPIO_InitTypeDef GPIO_InitStruct;
		GPIO_InitStruct.Pin = PSEL0_PIN;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(PSEL0_PORT, &GPIO_InitStruct);
	}

	// Configure PSEL1 pin as output:
	{
		GPIO_InitTypeDef GPIO_InitStruct;
		GPIO_InitStruct.Pin = PSEL1_PIN;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(PSEL1_PORT, &GPIO_InitStruct);
	}

	if (busType == BUS_Spi)
	{
		// Configure Chip-select as output:
		HAL_GPIO_WritePin(CHIP_SELECT_PORT, CHIP_SELECT_PIN, GPIO_PIN_SET);
		GPIO_InitTypeDef GPIO_InitStruct;
		GPIO_InitStruct.Pin = CHIP_SELECT_PIN;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(CHIP_SELECT_PORT, &GPIO_InitStruct);

		// Set board in SPI mode:
		HAL_GPIO_WritePin(PSEL0_PORT, PSEL0_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(PSEL1_PORT, PSEL1_PIN, GPIO_PIN_SET);
	}

	if (busType == BUS_I2c)
	{
		// Configure Address lines as output:
		GPIO_InitTypeDef GPIO_InitStruct;
		GPIO_InitStruct.Pin = I2C_ADD0_PIN | I2C_ADD1_PIN | I2C_ADD2_PIN;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(I2C_ADD_PORT, &GPIO_InitStruct);
		// Default address (0x68) corresponds to all address lines high
		HAL_GPIO_WritePin(I2C_ADD_PORT, I2C_ADD0_PIN | I2C_ADD1_PIN | I2C_ADD2_PIN, GPIO_PIN_SET);

		// Set board in I2C mode:
		HAL_GPIO_WritePin(PSEL0_PORT, PSEL0_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(PSEL1_PORT, PSEL1_PIN, GPIO_PIN_SET);
	}
}


/*!	\brief Set an optional handler for interrupts on the DataReady line
*/
void setGpioInterruptInterface(GpioInterruptInterface* gpioInterruptInterface)
{
	g_gpioInterruptInterface = gpioInterruptInterface;
}


/*!	\brief Called by the GPIO IRQ handler
*/
extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (g_gpioInterruptInterface)
	{
		g_gpioInterruptInterface->handleGpioInterrupt();
	}
}


/*!	\brief GPIO IRQ handler
*/
extern "C" void EXTI3_IRQHandler(void)
{
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3);
}






