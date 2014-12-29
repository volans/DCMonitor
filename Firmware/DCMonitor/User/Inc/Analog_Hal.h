/**
  ******************************************************************************
  * File Name          : Analog_Hal.h
  * Date               : 29/11/2014 12:27:56
  * Description        : This file provides AD7714, DAC124S085 interface
  ******************************************************************************
  */
#ifndef _ANALOG_HAL_H
#define _ANALOG_HAL_H

#include "stm32f10x.h"
#include "stm32f10x_it.h"

/*-------------------------------------------------------------------
              AD7714 Registers
--------------------------------------------------------------------*/
/*-------------------  Communication Register   --------------------*/

// Registers select, 3bit <<4
#define COMM_SET_SHIFT      4
#define COMMUNICATION_REG   0
#define MODE_REG            1
#define FILTER_H_REG        2
#define FILTER_L_REG        3
#define TEST_REG            4
#define DATA_REG            5
#define ZERO_SCALE_CAL_REG  6
#define FULL_SCALE_CAL_REG  7

// Next R/W operation define, read(1) and write(0), 1bit <<3
#define RW_OP_SHIFT 3
#define NEXT_WRITE  0
#define NEXT_READ   1

// Channel Selection, 3bit<<0
#define ADC_CHNL_SEL_SHIFT    0
#define AIN_1_6     0
#define AIN_2_6     1
#define AIN_3_6     2
#define AIN_4_6     3
#define AIN_1_2     4
#define AIN_3_4     5
#define AIN_5_6     6
#define AIN_TEST    7

/*--------------------------  Mode Register   ------------------------*/

// Calibration Mode, 3bit<<5
#define CAL_MODE_SET_SHIFT      5
#define NORMAL_MODE             0
#define SELF_CAL_MODE           1
#define ZERO_SYS_CAL_MODE       2
#define FULL_SYS_CAL_MODE       3
#define SYS_OFFSET_CAL_MODE     4
#define BACKGROUND_MODE         5
#define ZERO_SELF_CAL_MODE      6
#define FULL_SELF_CAL_MODE      7

// Gain, 3bit<<2
#define GAIN_SET_SHIFT  2
#define GAIN_1      0
#define GAIN_2      1
#define GAIN_4      2
#define GAIN_8      3
#define GAIN_16     4
#define GAIN_32     5
#define GAIN_64     6
#define GAIN_128    7

// Burnout Current, off(0), on(1), 1bit<<<1
#define BO_SET_SHIFT    1
#define BO_OFF  0
#define BO_ON   1

// Filter Synchronization, enable(1), disable(0), 1bit<<0
#define FSYNC_SET_SHIFT 0
#define FSYNC_ON        1
#define FSYNC_OFF       0

/*---------------------- Filter Hight Register -----------------------*/

// POLAR 1bit <<7
#define POLAR_SET_SHIFT     7
#define POLAR_BI    0
#define POLAR_UNI   1

// Word Length, 1bit <<6
#define WL_SET_SHIFT    6
#define WL_16_BIT       0
#define WL_24_BIT       1

// Current Boost, 1bit <<5
#define BST_SET_SHIFT   5
#define BST_OFF     0
#define BST_ON      1

// Zearo or Clock Disalbe, 1bit <<4
#define ZERO_CLKDIS_SET_SHIFT   4
#define CLK_EN_OR_A     0
#define CLK_DIS         1

// Filter Selection High 4bit<<0
/*------------------- Filter Low Register --------------------*/
// Filter Selection Low 8bit
/**
    12bit code = (CLOCK/128)/(Filter Frequency)
    For example, if we set the first notch of the filter occurs at 50HZ, 
    and the we use 2.4576MHZ clock for AD7714,we should set the value to:
    (2457600/128)/50=384=0x180
    And then, low 4bit of Filter High Register should be 0001, 
    and Filter Low Register should be 0x80.
 */


/*-----------------------------------------------------------------
              DAC124S085  Registers
------------------------------------------------------------------*/
// Output chennel select
#define DAC_CHNL_SEL_SHIFT  14
#define DAC_A   0
#define DAC_B   1
#define DAC_C   2
#define DAC_D   3

// Operation mode
#define DAC_OP_SET_SHIFT    12
#define DAC_WRITE_SPE_REG_NOT_UPDATE_OUT    0
#define DAC_WRITE_SPE_REG_UPDATE_OUT        1
#define DAC_WRITE_ALL_REG_UPDATE_OUT        2
#define DAC_PWR_DOWN                        3


/*-----------------------------------------------------------------
              DAC and ADC communicatin interface
------------------------------------------------------------------*/
#define DAC_SPI_SCLK_PORT   GPIOA
#define DAC_SPI_SCLK_PIN    GPIO_Pin_6
#define DAC_SPI_DIN_PORT    GPIOB
#define DAC_SPI_DIN_PIN     GPIO_Pin_0
#define DAC_SYNC_PORT       GPIOA
#define DAC_SYNC_PIN        GPIO_Pin_7

#define ADC_SPI_SCLK_PORT   GPIOB
#define ADC_SPI_SCLK_PIN    GPIO_Pin_1
#define ADC_SPI_DIN_PORT    GPIOB
#define ADC_SPI_DIN_PIN     GPIO_Pin_2
#define ADC_SPI_DOUT_PORT   GPIOB
#define ADC_SPI_DOUT_PIN    GPIO_Pin_10
#define ADC_DRDY_PORT       GPIOB
#define ADC_DRDY_PIN        GPIO_Pin_11



#define SPI_DI(port, pin, value)    GPIO_WriteBit(port, pin, value)
#define SPI_SCLK(port, pin, value)  GPIO_WriteBit(port, pin, value)
#define SPI_DO(port, pin)           GPIO_ReadInputDataBit(port, pin)

#define DRDY(port, pin)             GPIO_ReadInputDataBit(port, pin)
#define SYNC(port, pin, value)      GPIO_WriteBit(port, pin, value)


#define ADC_DI(value)       SPI_DI(ADC_SPI_DIN_PORT, ADC_SPI_DIN_PIN, value)
#define ADC_SCLK(value)     SPI_SCLK(ADC_SPI_SCLK_PORT, ADC_SPI_SCLK_PIN, value)
#define ADC_DO()            SPI_DO(ADC_SPI_DOUT_PORT, ADC_SPI_DOUT_PIN)
#define ADC_SETDRDY(value)       GPIO_WriteBit(ADC_DRDY_PORT, ADC_DRDY_PIN, value)
#define ADC_DRDY()          DRDY(ADC_DRDY_PORT, ADC_DRDY_PIN)


#define DAC_DI(value)       SPI_DI(DAC_SPI_DIN_PORT, DAC_SPI_DIN_PIN, value)
#define DAC_SCLK(value)     SPI_SCLK(DAC_SPI_SCLK_PORT, DAC_SPI_SCLK_PIN, value)
#define DAC_SYNC(value)     SYNC(DAC_SYNC_PORT, DAC_SYNC_PIN, value)


void DAC_SPI_Start(void);
void DAC_Write(u16 data);

void Int_ADC_DMA_NVIC_Configuration();
void Int_ADC_DMA_Configration(void);
void Int_ADC_Configuration(void);

void Ext_ADC_Init(void);
u32 ADC_ReadVTO();
u32 ADC_ReadVOG();


#endif
