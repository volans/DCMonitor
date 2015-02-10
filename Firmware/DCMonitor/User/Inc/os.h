/**
  ******************************************************************************
  * File Name          : os.h
  * Date               : 25/12/2014 12:27:56
  * Description        : This file provides code to freeRTOS tasks implementation
  ******************************************************************************
  */

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* Task priorities. */
#define mainTASK_PRIORI                     ( tskIDLE_PRIORITY )
#define mainUSART_LOOP_PRIORITY             ( tskIDLE_PRIORITY + 3 )
#define mainUSB_LOOP_PRIORITY               ( tskIDLE_PRIORITY + 2 )



/* The check task uses the sprintf function so requires a little more stack. */
#define mainCHECK_TASK_STACK_SIZE			( configMINIMAL_STACK_SIZE + 50 )

/* Dimensions the buffer into which the jitter time is written. */
#define mainMAX_MSG_LEN						25

/* The time between cycles of the 'check' task. */
#define mainCHECK_DELAY						( ( TickType_t ) 5000 / portTICK_PERIOD_MS )

/* The number of nano seconds between each processor clock. */
#define mainNS_PER_CLOCK ( ( unsigned long ) ( ( 1.0 / ( double ) configCPU_CLOCK_HZ ) * 1000000000.0 ) )

/* Baud rate used by the comtest tasks. */
#define mainCOM_TEST_BAUD_RATE		( 115200 )

/* The LED used by the comtest tasks. See the comtest.c file for more
information. */
#define mainCOM_TEST_LED			( 3 )

/*-----------------------------------------------------------*/
void vMainTask(void *pvParameters);
void vUsartTask(void *pvParameters);
void vUsbTask(void *pvParameters);

/*-----------------------------------------------------------*/


