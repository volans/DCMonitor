/**
  ******************************************************************************
  * File Name          : os.c
  * Date               : 25/12/2014 12:27:56
  * Description        : This file provides code to freeRTOS tasks implementation
  ******************************************************************************
  */

#include "os.h"

/*-----------------------------------------------------------*/
xSemaphoreHandle xSemaphore_volans = NULL;
xSemaphoreHandle xSemaphore_Int_ADC = NULL;
xSemaphoreHandle xSemaphore_Ext_ADC = NULL; 
/*-----------------------------------------------------------*/
   

void vLCDTask( void *pvParameters )
{
  //volans
  for(;;)
  {
    if(xSemaphoreTake(xSemaphore_volans,(portTickType)10) ==pdTRUE)
    {
//    vTaskDelay(300);
 //   GPIO_WriteBit(GPIOD,GPIO_Pin_2,1);
    GPIO_SetBits(GPIOA, GPIO_Pin_8);
   vTaskDelay(900);
        xSemaphoreGive( xSemaphore_volans );
        vTaskDelay(300);
    GPIO_ResetBits(GPIOA, GPIO_Pin_8);
 //   GPIO_WriteBit(GPIOD,GPIO_Pin_2,0);

     vTaskDelay(6);
    }
  }
}
/*-----------------------------------------------------------*/

static void vCheckTask( void *pvParameters )
{
  //volans
  for(;;)
  {
    if(xSemaphoreTake(xSemaphore_volans,(portTickType)10) ==pdTRUE)
    {
    GPIO_SetBits(GPIOB, GPIO_Pin_15);
    vTaskDelay(300);
    
    GPIO_ResetBits(GPIOB, GPIO_Pin_15);
    xSemaphoreGive( xSemaphore_volans );
    vTaskDelay(6);
    }
  }
  
}

/*-----------------------------------------------------------*/
