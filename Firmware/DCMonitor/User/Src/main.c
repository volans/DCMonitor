/*
  
*/

/* Standard includes. */
#include <stdio.h>

/* Library includes. */
#include "stm32f10x_it.h"
#include "stm32f10x_gpio.h"

/* User Include */
#include "hw_config.h"
#include "Analog_Hal.h"
#include "os.h"


__IO u16 vIN=0;
__IO u16 vREF=0;


//for USB Port
__IO u8 USB_R_Buffer[256];
__IO u8 USB_S_Buffer[256];
__IO u16 USB_S_Size, USB_R_Size;
__IO u8 USB_Receive_Flag, USB_Send_Flag;


int main( void )
{
	RCC_Config();
    Set_USBClock();  //72MHz
    MX_GPIO_Init();
    MainPower_CON(SET);
    USART_Configuration();
    DAC_SPI_Start();

    Int_ADC_DMA_NVIC_Configuration();
    Int_ADC_DMA_Configration();
    Int_ADC_Configuration();
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);

    DCMoitor_CON(RESET);
          
    //USB  
    USB_Interrupts_Config(); 
    USB_Init();
    
    
    //USART
    while(1){
      
      
        if(!IsEmpty())
        {
          printf("received: 0x%x \n", ReceiveByte());
        }
        USART_SendByte('a');
    }
    //int i =25;  //4.5v
    int i = 1024;
    //int i = 2048; // 
    //int i = 3072;//
    //int i = 4070; // 1.5v
    __IO u32 a=0;
    __IO u32 b=0;
    do{
        for(i=25;i<=4070;i+=100){
            DAC_Write((DAC_A<<DAC_CHNL_SEL_SHIFT)|(DAC_WRITE_SPE_REG_UPDATE_OUT<<DAC_OP_SET_SHIFT)|(0xfff&i));
            DCMoitor_CON(SET);
            Ext_ADC_Init();
            a = ADC_ReadVTO();
            b = ADC_ReadVOG();
        }
    }while(1);
    
    
    
    
    
    while(1);
#ifdef RTOS
    //volans semaphore
    //xSemaphore_volans = xSemaphoreCreateBinary();
    //xSemaphoreGive( xSemaphore_volans );
    if(xSemaphore_volans != NULL)
    {
        xTaskCreate(vMainTask, "Main", configMINIMAL_STACK_SIZE, NULL, mainTASK_PRIORI, NULL);
        xTaskCreate(vUsartTask, "Usart", mainCHECK_TASK_STACK_SIZE, NULL, mainUSART_LOOP_PRIORITY, NULL);
        xTaskCreate(vUsbTask, "Usb", mainCHECK_TASK_STACK_SIZE, NULL, mainUSB_LOOP_PRIORITY, NULL);

        /* Start the scheduler. */
        vTaskStartScheduler();
    }
#endif
        
        
	return 0;
}


/*-----------------------------------------------------------*/


/*-----------------------------------------------------------*/

