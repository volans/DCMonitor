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
    DAC_SPI_Start();

    Int_ADC_DMA_NVIC_Configuration();
    Int_ADC_DMA_Configration();
    Int_ADC_Configuration();
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);

    DCMoitor_CON(RESET);
          
    //USB  
    USB_Interrupts_Config(); 
    USB_Init();
    
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
        xSemaphore_volans = xSemaphoreCreateBinary();
        xSemaphoreGive( xSemaphore_volans );
         
         if(xSemaphore_volans != NULL)
         {
        
        
	/* Start the tasks defined within this file/specific to this demo. */
    xTaskCreate( vCheckTask, "Check", mainCHECK_TASK_STACK_SIZE, NULL, mainCHECK_TASK_PRIORITY, NULL );
	xTaskCreate( vLCDTask, "LCD", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL );

	/* The suicide tasks must be created last as they need to know how many
	tasks were running prior to their creation in order to ascertain whether
	or not the correct/expected number of tasks are running at any given time. */
    //vCreateSuicidalTasks( mainCREATOR_TASK_PRIORITY );

	/* Configure the timers used by the fast interrupt timer test. */
	//vSetupTimerTest();

	/* Start the scheduler. */
	vTaskStartScheduler();

	/* Will only get here if there was not enough heap space to create the
	idle task. */
         }
#endif
        
        
	return 0;
}


/*-----------------------------------------------------------*/


/*-----------------------------------------------------------*/

