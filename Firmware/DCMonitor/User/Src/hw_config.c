/**
  ******************************************************************************
  * File Name          : hw_configuration.c
  * Date               : 29/11/2014 12:27:56
  * Description        : This file provides code for the configuration
  *                      of all used GPIO pins, clocks, adc, usb...
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "hw_config.h"
//#include "mass_mal.h"
#include "usb_desc.h"
#include "usb_pwr.h"


/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------
  Port     - Function           - Initial Status
  GPIOA.0  - AIN                
  GPIOA.1  - AIN
  GPIOA.2  - TXD                - GPIO_Mode_AF_PP
  GPIOA.3  - RXD                - GPIO_Mode_IN_FLOATING
  GPIOA.4  - AIN
  GPIOA.5  - AIN
  GPIOA.6  - DA_SCLK            - GPIO_Mode_Out_PP
  GPIOA.7  - DA_SYNC            - GPIO_Mode_Out_PP
  GPIOA.8  - LED_POWER_OUT      - GPIO_Mode_Out_PP
  GPIOA.9  - NC
  GPIOA.10 - NC
  GPIOA.11 - USB-T
  GPIOA.12 - USB+T
  GPIOA.13 - MCU_TMS
  GPIOA.14 - MCU_TCLK
  GPIOA.15 - MCU_TDI

  GPIOB.0  - DA_DIN             - GPIO_Mode_Out_PP
  GPIOB.1  - AD_SCLK            - GPIO_Mode_Out_PP
  GPIOB.2  - AD_DIN             - GPIO_Mode_Out_PP
  GPIOB.3  - MCU_TDO
  GPIOB.4  - MCU_TRST
  GPIOB.5  - NC
  GPIOB.6  - BEEP               - GPIO_Mode_Out_PP
  GPIOB.7  - FAN_SPEED          - GPIO_Mode_IN_FLOATING
  GPIOB.8  - FAN                - GPIO_Mode_Out_PP
  GPIOB.9  - NC
  GPIOB.10 - AD_DOUT            - GPIO_Mode_IN_FLOATING
  GPIOB.11 - AD_DRDY            - GPIO_Mode_IN_FLOATING
  GPIOB.12 - SHUTDOWN           - GPIO_Mode_Out_PP
  GPIOB.13 - POWERKEY           - GPIO_Mode_IN_FLOATING
  GPIOB.14 - POWERCON           - GPIO_Mode_Out_PP
  GPIOB.15 - LED_WORK           - GPIO_Mode_Out_PP
------------------------------------------------------------------------------*/

void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    /** Configure pins as 
          * Analog 
          * Input 
          * Output
          * EVENT_OUT
          * EXTI
    */

    /*Enable or disable APB2 peripheral clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    /*Configure GPIO pin : PA */
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Enable or disable APB2 peripheral clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    
    /*Configure GPIO pin : PB Out */
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_6 
                            |GPIO_Pin_8|GPIO_Pin_12|GPIO_Pin_14|GPIO_Pin_15;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    /*Configure GPIO pin : PB In */
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_7|GPIO_Pin_10|GPIO_Pin_13; 
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    /* PB.11, input pull high */
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_11; 
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    /** ADC1 GPIO Configuration  
    PA0-WKUP   ------> ADC1_IN0
    PA1   ------> ADC1_IN1
    PA4   ------> ADC1_IN4
    PA5   ------> ADC1_IN5
    */

    /*Enable or disable APB2 peripheral clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    /*Configure GPIO pin : PA */
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_4|GPIO_Pin_5;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    /** USART2 GPIO Configuration  
    PA2   ------> USART2_TX
    PA3   ------> USART2_RX
    */

    /*Enable or disable APB2 peripheral clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB1Periph_USART2, ENABLE);

    /*Configure TX */
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_Mode =  GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
    /*Configure TX */
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_Mode =  GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    /** USB GPIO Configuration  
    PA11   ------> USB_DM
    PA12   ------> USB_DP
    */

    /*Enable or disable APB2 peripheral clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    /*Configure GPIO pin : PA */
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_11|GPIO_Pin_12;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    
    /* Set out put pins to right status initially */
    GPIO_SetBits(GPIOA, GPIO_Pin_8); //close LED_POWER_OUT
    GPIO_SetBits(GPIOB, GPIO_Pin_15); //close LED_WORK
    GPIO_ResetBits(GPIOB, GPIO_Pin_6|GPIO_Pin_8|GPIO_Pin_12|GPIO_Pin_14); // out 0 to BEEP, FAN, POWERCON and SHUTDOWN

}

/*
  Controlling circuit part Power control:
  SET: open main power, which happens after first key press
  RESET: close main power, which happens key press after booted up
*/
void MainPower_CON(FlagStatus p)
{
   if(p==SET)
      GPIO_SetBits(GPIOB, GPIO_Pin_14);
   else
      GPIO_ResetBits(GPIOB, GPIO_Pin_14);
}


/*
  Monitored Power control:
  SET: open monitored power, which happens after user open power output
  RESET: close monitored power, which happens after user close power output
*/
void DCMoitor_CON(FlagStatus p)
{
   if(p==SET)
      GPIO_SetBits(GPIOB, GPIO_Pin_12);
   else
      GPIO_ResetBits(GPIOB, GPIO_Pin_12);
}

/*
  Fan control:
  SET: Open Fan
  RESET: Close Fan
*/
void Fan_CON(FlagStatus p)
{
   if(p==SET)
      GPIO_SetBits(GPIOB, GPIO_Pin_8);
   else
      GPIO_ResetBits(GPIOB, GPIO_Pin_8);
}

/*
  Beep control:
  SET: Open Beep
  RESET: Close Beep
*/
void Beep_CON(FlagStatus p)
{
   if(p==SET)
      GPIO_SetBits(GPIOB, GPIO_Pin_6);
   else
      GPIO_ResetBits(GPIOB, GPIO_Pin_6);
}

/*
  LedWork control
  SET: Open LED, open this LED after controller boot up
  RESET: Close LED, close this LED after controller shut down
*/
void LEDWork_CON(FlagStatus p)
{
   if(p==SET)
      GPIO_SetBits(GPIOB, GPIO_Pin_15);
   else
      GPIO_ResetBits(GPIOB, GPIO_Pin_15);
}

/*
  LedPower control
  SET: Open LED, open this LED after monitored power opened.
  RESET: Close LED, close this LED after monitored power closed.
*/
void LEDPower_CON(FlagStatus p)
{
   if(p==SET)
      GPIO_SetBits(GPIOA, GPIO_Pin_8);
   else
      GPIO_ResetBits(GPIOA, GPIO_Pin_8);
}

/*
  System RRC config
*/
void RCC_Config(void)
{
    ErrorStatus HSEStartUpStatus;
    
    /* RCC system reset(for debug purpose) */
    RCC_DeInit();

    /* Enable HSE */
    RCC_HSEConfig(RCC_HSE_ON);

    /* Wait till HSE is ready */
    HSEStartUpStatus = RCC_WaitForHSEStartUp();

    if (HSEStartUpStatus == SUCCESS)
    {
        /* Enable Prefetch Buffer */
        FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

        /* Flash 2 wait state */
        FLASH_SetLatency(FLASH_Latency_2);

        /* HCLK = SYSCLK */
        RCC_HCLKConfig(RCC_SYSCLK_Div1);

        /* PCLK2 = HCLK */
        RCC_PCLK2Config(RCC_HCLK_Div1);

        /* PCLK1 = HCLK/2 */
        RCC_PCLK1Config(RCC_HCLK_Div2);

        /* ADCCLK = PCLK2/6 */
        RCC_ADCCLKConfig(RCC_PCLK2_Div6);


        RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);

        /* Enable PLL */
        RCC_PLLCmd(ENABLE);

        /* Wait till PLL is ready */
        while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
        {}

        /* Select PLL as system clock source */
        RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

        /* Wait till PLL is used as system clock source */
        while (RCC_GetSYSCLKSource() != 0x08)
        {}
        
        /* Configure HCLK clock as SysTick clock source. */
        SysTick_CLKSourceConfig( SysTick_CLKSource_HCLK );
    }
}

/*
  Set USB Clock to 
*/
void Set_USBClock(void)
{
    /* USBCLK = PLLCLK */
    RCC_USBCLKConfig(RCC_USBCLKSource_PLLCLK_1Div5);

    /* Enable USB clock */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, ENABLE);
}


void USB_Interrupts_Config(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Set the Vector Table base address at 0x08000000 */
    NVIC_SetVectorTable( NVIC_VectTab_FLASH, 0x0 );
   
    /* Enable the USB interrupt */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
   
    NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    NVIC_InitStructure.NVIC_IRQChannel = USB_HP_CAN1_TX_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/*******************************************************************************
* Function Name  : Leave_LowPowerMode
* Description    : Restores system clocks and power while exiting suspend mode
* Input          : None.
* Return         : None.
*******************************************************************************/
void Leave_LowPowerMode(void)
{
  DEVICE_INFO *pInfo = &Device_Info;

  /* Set the device state to the correct state */
  if (pInfo->Current_Configuration != 0)
  {
    /* Device configured */
    bDeviceState = CONFIGURED;
  }
  else
  {
    bDeviceState = ATTACHED;
  }

}

/*******************************************************************************
* Function Name  : USB_Configured_LED
* Description    : Turn ON the Read/Write LEDs.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void USB_Configured_LED(void)
{
//  GPIO_SetBits(USB_LED_PORT, GPIO_Pin_6);
}

/*******************************************************************************
* Function Name  : USB_NotConfigured_LED
* Description    : Turn off the Read/Write LEDs.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void USB_NotConfigured_LED(void)
{
 // GPIO_ResetBits(USB_LED_PORT, GPIO_Pin_6);
}

/*******************************************************************************
* Function Name  : USB_Cable_Config
* Description    : Software Connection/Disconnection of USB Cable.
* Input          : None.
* Return         : Status
*******************************************************************************/
void USB_Cable_Config (FunctionalState NewState)
{/*
  if (NewState != DISABLE)
  {
 //   GPIO_ResetBits(USB_DISCONNECT, USB_DISCONNECT_PIN);
  }
  else
  {
 //   GPIO_SetBits(USB_DISCONNECT, USB_DISCONNECT_PIN);
  }*/
}

/*******************************************************************************
* Function Name  : Get_SerialNum.
* Description    : Create the serial number string descriptor.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Get_SerialNum(void)
{
    u32 Device_Serial0, Device_Serial1, Device_Serial2;

    Device_Serial0 = *(vu32*)(0x1FFFF7E8);
    Device_Serial1 = *(vu32*)(0x1FFFF7EC);
    Device_Serial2 = *(vu32*)(0x1FFFF7F0);

    if (Device_Serial0 != 0)
    {
      MASS_StringSerial[2] = (u8)(Device_Serial0 & 0x000000FF);
      MASS_StringSerial[4] = (u8)((Device_Serial0 & 0x0000FF00) >> 8);
      MASS_StringSerial[6] = (u8)((Device_Serial0 & 0x00FF0000) >> 16);
      MASS_StringSerial[8] = (u8)((Device_Serial0 & 0xFF000000) >> 24);

      MASS_StringSerial[10] = (u8)(Device_Serial1 & 0x000000FF);
      MASS_StringSerial[12] = (u8)((Device_Serial1 & 0x0000FF00) >> 8);
      MASS_StringSerial[14] = (u8)((Device_Serial1 & 0x00FF0000) >> 16);
      MASS_StringSerial[16] = (u8)((Device_Serial1 & 0xFF000000) >> 24);

      MASS_StringSerial[18] = (u8)(Device_Serial2 & 0x000000FF);
      MASS_StringSerial[20] = (u8)((Device_Serial2 & 0x0000FF00) >> 8);
      MASS_StringSerial[22] = (u8)((Device_Serial2 & 0x00FF0000) >> 16);
      MASS_StringSerial[24] = (u8)((Device_Serial2 & 0xFF000000) >> 24);
    }
}


/*******************************************************************************
* Function Name  : USART_Configuration
* Description    : Initialization of USART 2 for BT connection
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void USART_Configuration(void)  

{
    USART_InitTypeDef USART2_InitStructure; 

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2|RCC_APB2Periph_GPIOA,ENABLE);

    USART2_InitStructure.USART_BaudRate=9600;
    USART2_InitStructure.USART_WordLength=USART_WordLength_8b;
    USART2_InitStructure.USART_StopBits=USART_StopBits_1;
    USART2_InitStructure.USART_Parity=USART_Parity_No;
    USART2_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
    USART2_InitStructure.USART_Mode=USART_Mode_Rx|USART_Mode_Tx;
    USART_Init(USART2,&USART2_InitStructure);

    USART_Cmd(USART2,ENABLE);
    USART_ClearFlag(USART2,USART_FLAG_RXNE);
} 

void USART_SendByte(u8 d)
{
   USART_SendData(USART2, d);  
   while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET){}  
}

/* Is there any data in USART received rigister*/
bool IsEmpty(void)
{
   if(USART_GetFlagStatus(USART2, USART_FLAG_RXNE) == RESET)
     return TRUE;
   else
     return FALSE;
}

u8 ReceiveByte(void)
{
   while(USART_GetFlagStatus(USART2, USART_FLAG_RXNE) == RESET){}  
     return (USART_ReceiveData(USART2));
}

/*******************************************************************************
* Function Name  : Monitor_Calibration
* Description    : Calibrate the monitor
* Input          : vol, output voltage value.
* Output         : None.
* Return         : TRUE/FALSE.
*******************************************************************************/
bool Monitor_Calibration(u8 vol)
{

}
