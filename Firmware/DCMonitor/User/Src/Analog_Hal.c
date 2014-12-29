/**
  ******************************************************************************
  * File Name          : Analog_Hal.c
  * Date               : 29/11/2014 12:27:56
  * Description        : This file provides code to communicate with ext ADC/DAC
  *                      through the simulated SPI interface
  ******************************************************************************
  */

/**
    GPIOA.6  - DA_SCLK 
    GPIOA.7  - DA_SYNC
    GPIOB.0  - DA_DIN       data from controller to DAC
    GPIOB.1  - AD_SCLK
    GPIOB.2  - AD_DIN       data from controller to ADC
    GPIOB.10 - AD_DOUT      data from ADC to controller
    GPIOB.11 - AD_DRDY
  */
#include "Analog_Hal.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_adc.h"


#define BIT(n) (1<<n)
    
#define SPI_DELAY           500
static void delay(u32 nCount)
{
#if BLOCK
    for(; nCount != 0; nCount--)
      IWDG->KR=0xAAAA;
#else
    //TODO: freertos delay    
#endif
}               
    
/*
        AD7714 Operations
*/

static void ADC_SPI_Start(void)
{
    ADC_DI(Bit_SET);
    ADC_SCLK(Bit_SET);
}

void ADC_Write_Byte(u8 data)
{
    u8 i = 0;

    ADC_SPI_Start();
    delay(SPI_DELAY);
    
    __disable_irq();
    for(i=0;i<8;i++)
    {
        ADC_SCLK(Bit_RESET);
        delay(SPI_DELAY);
        
        if(data&BIT(7-i))
            ADC_DI(Bit_SET);
        else
            ADC_DI(Bit_RESET);       
        delay(SPI_DELAY);
        
        ADC_SCLK(Bit_SET);
        delay(SPI_DELAY);
    }
    __enable_irq();
}

u32 ADC_Read(u8 NumBit)
{
    u8 i = 0;
    u32 data = 0;
    
    ADC_SPI_Start();
    delay(SPI_DELAY);
    
    __disable_irq();
    for(i=0;i<NumBit;i++)
    {
        ADC_SCLK(Bit_RESET);
        delay(SPI_DELAY);
        
        if(ADC_DO())
          data |= BIT((NumBit-1)-i);
        
        ADC_SCLK(Bit_SET);
        delay(SPI_DELAY);
        //delay(SPI_DELAY);
    }
    __enable_irq();
    return data;
}

/*
  Soft check DRDY
*/
void ADC_Check_DRDY(void)
{
    u8 data = 0;
    do{
        ADC_Write_Byte(0x0c);
        data = ADC_Read(8);
        data &= 0x80;
    }while(data);
    
}

/* If a logic 1 is written to the AD7714 DIN line for at
least 32 serial clock cycles the serial interface is reset. */
void ADC_SoftReset(void)
{
    u8 i;
    for(i=0;i<64;i++)
    {
        ADC_SCLK(Bit_RESET);
        ADC_DI(Bit_SET);
        ADC_SCLK(Bit_SET);
    }
}

void Ext_ADC_Init(void)
{
    ADC_SoftReset();
  
    // chose AIN 1-2 differential, filter high register
    ADC_Write_Byte((FILTER_H_REG<<COMM_SET_SHIFT)|(AIN_1_2<<ADC_CHNL_SEL_SHIFT)); 
    ADC_Write_Byte((POLAR_UNI<<POLAR_SET_SHIFT)|(BST_ON<<BST_SET_SHIFT)|(WL_24_BIT<<WL_SET_SHIFT)|0x1); // 24bit, 50HZ filter
    
    // chose AIN 1-2 differential, filter low register
    ADC_Write_Byte((FILTER_L_REG<<COMM_SET_SHIFT)|(AIN_1_2<<ADC_CHNL_SEL_SHIFT)); 
    ADC_Write_Byte(0x80); // 50HZ filter
    
    // chose AIN 1-2 differential, mode register
    ADC_Write_Byte((MODE_REG<<COMM_SET_SHIFT)|(AIN_1_2<<ADC_CHNL_SEL_SHIFT)); 
    // self calibration, gain=1, no burnout current, no filter sync
    ADC_Write_Byte((SELF_CAL_MODE<<CAL_MODE_SET_SHIFT)|(GAIN_1<<GAIN_SET_SHIFT));

    while(!ADC_DRDY());//wait for calibration done
    while(ADC_DRDY());//wait for calibration done

    // chose AIN 3-4 differential, filter high register
    ADC_Write_Byte((FILTER_H_REG<<COMM_SET_SHIFT)|(AIN_3_4<<ADC_CHNL_SEL_SHIFT)); 
    ADC_Write_Byte((POLAR_UNI<<POLAR_SET_SHIFT)|(BST_ON<<BST_SET_SHIFT)|(WL_24_BIT<<WL_SET_SHIFT)|0x1); // 24bit, 50HZ filter
    
    // chose AIN 3-4 differential, filter low register
    ADC_Write_Byte((FILTER_L_REG<<COMM_SET_SHIFT)|(AIN_3_4<<ADC_CHNL_SEL_SHIFT)); 
    ADC_Write_Byte(0x80); // 50HZ filter
    
    // chose AIN 3-4 differential, mode register
    ADC_Write_Byte((MODE_REG<<COMM_SET_SHIFT)|(AIN_3_4<<ADC_CHNL_SEL_SHIFT)); 
    // self calibration, gain=4, no burnout current, no filter sync
    ADC_Write_Byte((SELF_CAL_MODE<<CAL_MODE_SET_SHIFT)|(GAIN_4<<GAIN_SET_SHIFT));

    while(!ADC_DRDY());
    while(ADC_DRDY());

}

/*
  Read the voltage of VT <-> Out-, result*5.7=OUT+ <-> OUT-
*/
u32 ADC_ReadVTO()
{
    u32 vto = 0;
    u8 status = 0;
    
    //select read AIN_3_4 differential result from data register
    ADC_Write_Byte((DATA_REG<<COMM_SET_SHIFT)|(NEXT_READ<<RW_OP_SHIFT)|(AIN_1_2<<ADC_CHNL_SEL_SHIFT));

    while(!ADC_DRDY());
    while(ADC_DRDY());
        
    vto = ADC_Read(24);
    
    return vto&0xffffff;
}

/*
  Read the voltage of Out- <-> GND, result*10/gain = total current
*/
u32 ADC_ReadVOG()
{
    u32 vog = 0;
    
    //select read AIN_3_4 differential result from data register
    ADC_Write_Byte((DATA_REG<<COMM_SET_SHIFT)|(NEXT_READ<<RW_OP_SHIFT)|(AIN_3_4<<ADC_CHNL_SEL_SHIFT));
        
    while(!ADC_DRDY());
    while(ADC_DRDY());
        
    vog = ADC_Read(24);

    return vog&0xffffff;  
}

/*
  DAC124S085 Operations
*/

void DAC_SPI_Start(void)
{
    DAC_DI(Bit_SET);
    DAC_SCLK(Bit_SET);
    DAC_SYNC(Bit_SET);
}

void DAC_Write(u16 data)
{
    u8 i = 0;
    DAC_SPI_Start();
    delay(SPI_DELAY);
    
    DAC_SYNC(Bit_RESET);
    delay(SPI_DELAY);
    
    __disable_irq();
    for(i=0;i<16;i++)
    {
        DAC_SCLK(Bit_SET);
        delay(SPI_DELAY);
        
        if(data&BIT(15-i))
            DAC_DI(Bit_SET);
        else
            DAC_DI(Bit_RESET);       
        delay(SPI_DELAY);
        
        DAC_SCLK(Bit_RESET);
        delay(SPI_DELAY);        
    }
    __enable_irq();
    
    DAC_SPI_Start();
    delay(SPI_DELAY);
}



/**
  STM32 internal ADC Configuration & Read
    PA0   ------> ADC1_IN0 -------------- VFEEDB
    PA1   ------> ADC1_IN1 -------------- VCON
    PA4   ------> ADC1_IN4 -------------- VREF
    PA5   ------> ADC1_IN5 -------------- VIN

  The reference voltage of internal ADC is 3.3V.

  VFEEDB: The output of differential amplifier.
  VCON: The output of external DAC. 
  VREF: The reference voltage of DAC. It should be 2.5V.
  VIN:  The input power, should be 5V.

    */

/*
  [0]: VFEEDB, VCON, VREF, VIN

  */
enum{
  ADC_VFEEDB=0,
  ADC_VCON,
  ADC_VREF,
  ADC_VIN  
};
__IO u16 INT_ADC_4_Value[10][4];
#define  ADC1_DR_Address    ((u32)0x40012400+0x4c)


void Int_ADC_DMA_NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
  NVIC_InitStructure.NVIC_IRQChannel=DMA1_Channel1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority=0;
  NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}


void Int_ADC_DMA_Configration(void)
{
    DMA_InitTypeDef DMA_InitStructure_DMA1;
    
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    DMA_DeInit(DMA1_Channel1);                                                      // 配置DMA1 的通道1寄存器为缺省值

    DMA_InitStructure_DMA1.DMA_PeripheralBaseAddr = ADC1_DR_Address;                     // 配置DMA1 外设基地址为 0x4001244C , 在datasheet上可以查到
    DMA_InitStructure_DMA1.DMA_MemoryBaseAddr = (u32)(&INT_ADC_4_Value);        // 配置DMA1 内存基地址
    DMA_InitStructure_DMA1.DMA_DIR = DMA_DIR_PeripheralSRC;                         // 配置DMA1 外设作为数据传输的来源 ，就是外设向内存中搬运数据
    DMA_InitStructure_DMA1.DMA_BufferSize = 40;                                     // 配置DMA1 数据缓存大小为 10个数据单位
    DMA_InitStructure_DMA1.DMA_PeripheralInc = DMA_PeripheralInc_Disable;           // 配置DMA1 外设地址寄存器不变
    DMA_InitStructure_DMA1.DMA_MemoryInc = DMA_MemoryInc_Enable;                    // 配置DMA1 内存地址寄存器递增
    DMA_InitStructure_DMA1.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;// 配置DMA1 外设数据宽度为 16位 ，这里注意ADC是12位的
    DMA_InitStructure_DMA1.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;        // 配置DMA1 内存数据宽度为 16位
    DMA_InitStructure_DMA1.DMA_Mode = DMA_Mode_Circular;                            // 配置DMA1 工作在循环缓存模式
    DMA_InitStructure_DMA1.DMA_Priority = DMA_Priority_High;                      // 配置DMA1 的相应通道(channel x)的优先级为 中优先级
    DMA_InitStructure_DMA1.DMA_M2M = DMA_M2M_Disable;                               // 配置DMA1 的相应通道(channel x)内存到内存传输为 使能状态


    DMA_Init(DMA1_Channel1, &DMA_InitStructure_DMA1);                               // 初始化 DMA1_Channel1 的相关寄存器 
    DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);
    DMA_Cmd(DMA1_Channel1, ENABLE);                                                 // 使能 DMA1_Channel1

}

void Int_ADC_Configuration(void)
{
    ADC_InitTypeDef ADC_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    
    /* ADC1 configuration ------------------------------------------------------*/
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 4;
    ADC_Init(ADC1, &ADC_InitStructure);
  
  
    /* ADC1 regular channel 0,1,4,5 configuration */ 
    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_239Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_239Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 3, ADC_SampleTime_239Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 4, ADC_SampleTime_239Cycles5);

    ADC_DMACmd(ADC1, ENABLE); 
    /* Enable ADC1 */
    ADC_Cmd(ADC1, ENABLE);
    
    /* Enable ADC1 reset calibaration register */   
    ADC_ResetCalibration(ADC1);
    
    /* Check the end of ADC1 reset calibration register */
    while(ADC_GetResetCalibrationStatus(ADC1));

    /* Start ADC1 calibaration */
    ADC_StartCalibration(ADC1);
    
    /* Check the end of ADC1 calibration */
    while(ADC_GetCalibrationStatus(ADC1));
}


u16 readVIN()
{
    return INT_ADC_4_Value[5][ADC_VIN];  
}

u16 readVREF()
{
    return INT_ADC_4_Value[5][ADC_VREF];  
}

u16 readVCON()
{
    return INT_ADC_4_Value[5][ADC_VCON];
}

u16 readVFEEDB()
{
    return INT_ADC_4_Value[5][ADC_VFEEDB];
}


