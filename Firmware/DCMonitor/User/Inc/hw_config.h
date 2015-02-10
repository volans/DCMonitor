/**
  ******************************************************************************
  * File Name          : hw_configuration.h
  * Date               : 29/11/2014 12:27:57
  * Description        : This file contains all the functions prototypes for 
  *                      the hw  
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __hw_configuration_H
#define __hw_configuration_H
#ifdef __cplusplus;
 extern "C" {
#endif
   
#define BULK_MAX_PACKET_SIZE  0x00000040
   
/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
   
   
void MX_GPIO_Init(void);
void MainPower_CON(FlagStatus p)；
void DCMoitor_CON(FlagStatus p);
void Fan_CON(FlagStatus p);
void Beep_CON(FlagStatus p);
void LEDWork_CON(FlagStatus p);
void LEDPower_CON(FlagStatus p);
void RCC_Config(void);
void Set_USBClock(void);
void USB_Interrupts_Config(void);
void ADC_Configuration(void);
void Leave_LowPowerMode(void)；
void USB_Configured_LED(void);
void USB_NotConfigured_LED(void);
void USB_Cable_Config (FunctionalState NewState);
void Get_SerialNum(void);


#ifdef __cplusplus
}
#endif
#endif /*__hw_configuration_H */

