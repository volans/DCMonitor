/**
  ******************************************************************************
  * File Name          : os.c
  * Date               : 25/12/2014 12:27:56
  * Description        : This file provides code to freeRTOS tasks implementation
  ******************************************************************************
  */

#include "os.h"


/*-------------------- Control Message from HOST ---------*/
#define CMD_HEAD_FROM_HOST      0xAA
#define CMD_OPEN_OUT            0x71
#define CMD_CLOSE_OUT           0x72
#define CMD_SET_VOL             0x73
#define CMD_SET_INTERVAL        0x74
#define CMD_CALIBRATION         0x75
#define CMD_OPEN_FAN            0x76
#define CMD_CLOSE_FAN           0x77
#define CMD_OPEN_BEEP           0x78
#define CMD_CLOSE_BEEP          0x79
#define CMD_SHUTDOWN            0x80


/*-------------------- Message from Monitor -------------*/
#define CMD_HEAD_FROM_SLAVE     0xBB




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

void vCheckTask( void *pvParameters )
{
  //volans
  for(;;)
  {
    if(xSemaphoreTake(xSemaphore_volans,(portTickType)10) == pdTRUE)
    {
    GPIO_SetBits(GPIOB, GPIO_Pin_15);
    vTaskDelay(300);
    
    GPIO_ResetBits(GPIOB, GPIO_Pin_15);
    xSemaphoreGive( xSemaphore_volans );
    vTaskDelay(6);
    }
  }
  
}

void vMainTask(void *pvParameters)
{
    for(;;)
    {

    }
}

void vUsartTask(void *pvParameters)
{
    for(;;)
    {
        if(IsEmpty() != TRUE){
            if(ReceiveByte() == CMD_HEAD_FROM_HOST){
                switch(ReceiveByte()){
                    case CMD_OPEN_OUT:

                        break;
                    case CMD_CLOSE_OUT:

                        break;
                    case CMD_SET_VOL:

                        break;
                    case CMD_SET_INTERVAL:

                        break;
                    case CMD_CALIBRATION:

                        break;
                    case CMD_OPEN_FAN:

                        break;
                    case CMD_CLOSE_FAN:

                        break;
                    case CMD_OPEN_BEEP:

                        break;
                    case CMD_CLOSE_BEEP:

                        break;
                    case CMD_SHUTDOWN:

                        break;
                    default:
                        continue;
                }
            }
        }
    }
}

void vUsbTask(void *pvParameters)
{
    for(;;)
    {
        if(USB_Receive_Flag == 0xff){
            USB_Receive_Flag = 0;
            if(USB_R_Buffer[0]==CMD_HEAD_FROM_HOST){
                switch(USB_R_Buffer[1]){
                    case CMD_OPEN_OUT:

                        break;
                    case CMD_CLOSE_OUT:

                        break;
                    case CMD_SET_VOL:

                        break;
                    case CMD_SET_INTERVAL:

                        break;
                    case CMD_CALIBRATION:

                        break;
                    case CMD_OPEN_FAN:

                        break;
                    case CMD_CLOSE_FAN:

                        break;
                    case CMD_OPEN_BEEP:

                        break;
                    case CMD_CLOSE_BEEP:

                        break;
                    case CMD_SHUTDOWN:

                        break;
                    default:
                        continue;
                }
            }
        }

        EP1_Send_Callback();
    }
    
}

/*-----------------------------------------------------------*/
