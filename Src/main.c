/*
 * FreeRTOS Kernel V10.2.0
 * Copyright (C) 2019 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */

/* 
	NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
	The processor MUST be in supervisor mode when vTaskStartScheduler is 
	called.  The demo applications included in the FreeRTOS.org download switch
	to supervisor mode prior to main being called.  If you are not using one of
	these demo application projects then ensure Supervisor mode is used.
*/


/*
 * Creates all the demo application tasks, then starts the scheduler.  The WEB
 * documentation provides more details of the demo application tasks.
 * 
 * Main.c also creates a task called "Check".  This only executes every three 
 * seconds but has the highest priority so is guaranteed to get processor time.  
 * Its main function is to check that all the other tasks are still operational.
 * Each task (other than the "flash" tasks) maintains a unique count that is 
 * incremented each time the task successfully completes its function.  Should 
 * any error occur within such a task the count is permanently halted.  The 
 * check task inspects the count of each task to ensure it has changed since
 * the last time the check task executed.  If all the count variables have 
 * changed all the tasks are still executing error free, and the check task
 * toggles the onboard LED.  Should any task contain an error at any time 
 * the LED toggle rate will change from 3 seconds to 500ms.
 *
 */


/* Standard includes. */
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "lpc21xx.h"

/* Peripheral includes. */
#include "serial.h"
#include "GPIO.h"

#include <stdint.h>
														
																																				
/* Constants to setup I/O and processor. */
#define mainBUS_CLK_FULL	( ( unsigned char ) 0x01 )

/* Constants for the ComTest demo application tasks. */
#define mainCOM_TEST_BAUD_RATE	( ( unsigned long ) 115200 )

typedef enum{
	RE,          //rising
	FE,					//faling
	NC					// no change
}buttonEvent_t;

typedef struct{
		uint8_t messageID;
		char messageData[((uint8_t)25)];
}message_t;


void Button_1_Monitor(void *vParam);
void Button_2_Monitor(void *vParam);
void Periodic_Transmitter(void *vParam);
void Uart_Receiver(void *vParam);
void Load_1_Simulation(void *vParam);
void Load_2_Simulation(void *vParam);

void vApplicationTickHook(void);

static void prvSetupHardware( void );


#if ( configUSE_EDF_SCHEDULER == 1 )
	BaseType_t xTaskPeriodicCreate( TaskFunction_t pxTaskCode,
													const char * const pcName, 
													const configSTACK_DEPTH_TYPE usStackDepth,
													void * const pvParameters,
													UBaseType_t uxPriority,
													TaskHandle_t * const pxCreatedTask ,
													TickType_t period);
													
QueueHandle_t Queue;
uint8_t i;
#endif
				
int main( void )
{
	/* Setup the hardware for use with the Keil demo board. */
	prvSetupHardware();

	Queue = xQueueCreate( ((uint8_t)10), sizeof( message_t ) );

	xTaskPeriodicCreate(
		Button_1_Monitor, 
		"Button_1_Monitor", 
		configMINIMAL_STACK_SIZE, 
		(void *)NULL,
		(UBaseType_t) 0, 
		(TaskHandle_t *)NULL, 
		((uint8_t)50)	);
		
	xTaskPeriodicCreate(
		Button_2_Monitor, 
		"Button_2_Monitor", 
		configMINIMAL_STACK_SIZE, 
		(void *)NULL,
		(UBaseType_t)0, 
		(TaskHandle_t *)NULL, 
		((uint8_t)50)	);
		
	xTaskPeriodicCreate(
		Periodic_Transmitter, 
		"Periodic_Transmitter", 
		configMINIMAL_STACK_SIZE, 
		(void *)NULL,
		(UBaseType_t)0, 
		(TaskHandle_t *)NULL, 
		((uint8_t)100)	);

		
	xTaskPeriodicCreate(
		Uart_Receiver, 
		"Uart_Receiver", 
		configMINIMAL_STACK_SIZE, 
		(void *)NULL,
		(UBaseType_t)0, 
		(TaskHandle_t *)NULL, 
		((uint8_t)20)	);
		
	xTaskPeriodicCreate(
		Load_1_Simulation, 
		"Load_1_Simulation", 
		configMINIMAL_STACK_SIZE, 
		(void *)NULL,
		(UBaseType_t)0, 
		(TaskHandle_t *)NULL, 
		((uint8_t)10)	);
		
	xTaskPeriodicCreate(
		Load_2_Simulation, 
		"Load_2_Simulation", 
		configMINIMAL_STACK_SIZE, 
		(void *)NULL, 
		(UBaseType_t) 0, 
		(TaskHandle_t *)NULL, 
		((uint8_t)100)	);
	
	
	/* Now all the tasks have been started - start the scheduler.
	NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
	The processor MUST be in supervisor mode when vTaskStartScheduler is 
	called.  The demo applications included in the FreeRTOS.org download switch
	to supervisor mode prior to main being called.  If you are not using one of
	these demo application projects then ensure Supervisor mode is used here. */
	vTaskStartScheduler();

	/* Should never reach here!  If you do then there was not enough heap
	available for the idle task to be created. */
	for( ;; );
}
/*-----------------------------------------------------------*/

/* Function to reset timer 1 */
void timer1Reset(void)
{
	T1TCR |= 0x2;
	T1TCR &= ~0x2;
}

/* Function to initialize and start timer 1 */
static void configTimer1(void)
{
	T1PR = 1000;
	T1TCR |= 0x1;
}

static void prvSetupHardware( void )
{
	/* Perform the hardware setup required.  This is minimal as most of the
	setup is managed by the settings in the project file. */

	/* Configure UART */
	xSerialPortInitMinimal(mainCOM_TEST_BAUD_RATE);

	/* Configure GPIO */
	GPIO_init();
	
	/* Config trace timer 1 and read T1TC to get current tick */
	configTimer1();

	/* Setup the peripheral bus to be the same as the PLL output. */
	VPBDIV = mainBUS_CLK_FULL;
}
/*-----------------------------------------------------------*/

				
void vApplicationTickHook(void){
		GPIO_write(PORT_0, PIN0, PIN_IS_HIGH);						\
		GPIO_write(PORT_0, PIN0, PIN_IS_LOW);							\
}

void Button_1_Monitor(void *vParam){
	TickType_t last_time;
	message_t Message = {0, 0};
	bool send = false;
	
	pinState_t CurrState, PrevState = GPIO_read(PORT_1, PIN0);
	buttonEvent_t event;
	
	vTaskSetApplicationTaskTag(NULL, (TaskHookFunction_t)1);
	
	last_time = xTaskGetTickCount();
	for(;;){
		
		//NOTE: This is bad practice and it definitely should be performed whitin the ISR with the EDGE detection hardware not in a task
		CurrState = GPIO_read(PORT_1, PIN0);
		if(CurrState != PrevState){
			//start Critical Section
			PrevState = CurrState;
			if(CurrState == PIN_IS_HIGH){
				 event = RE;
			}else{
				 event = FE;
			}
			//End Critical Section
		}else{
			event = NC;
		}
		
		switch((int)event){
			case RE:{
					strcpy((char *)(Message.messageData), "Button_1_Rising_Edge");		
					Message.messageData[((uint8_t)25)-1] = '\n';
					send = true;
				break;
			}
			case FE:{
					strcpy((char *)(Message.messageData), "Button_1_Falling_Edge");		
					Message.messageData[((uint8_t)25)-1] = '\n';
					send = true;
				break;
			}
			default: {
					send = false;
				break;
			}
		}
		
		if(send == true){
			xQueueSendToBack(Queue, &Message, ((uint8_t)5));
		}
		vTaskDelayUntil( &last_time, ((uint8_t)50)	 );
	}
}

void Button_2_Monitor(void *vParam){
	TickType_t last_time;
	message_t Message = {0, 0};
	bool send = false;
	
	pinState_t CurrState, PrevState = GPIO_read(PORT_1, PIN1);
	
	buttonEvent_t event;
	
	vTaskSetApplicationTaskTag(NULL, (TaskHookFunction_t)2);
	
	last_time = xTaskGetTickCount();
	
	for(;;){
		
		//NOTE: This is bad practice and it definitely should be performed whitin the ISR with the EDGE detection hardware not in a task
		CurrState = GPIO_read(PORT_1, PIN1);
		if(CurrState != PrevState){
			//start Critical Section
			PrevState = CurrState;
			if(CurrState == PIN_IS_HIGH){
				 event = RE;
			}else{
				 event = FE;
			}
			//End Critical Section
		}else{
			event = NC;
		}
		
		switch((int)event){
			case RE:{
					strcpy((char *)(Message.messageData), "Button_2_Rising_Edge");		
					Message.messageData[((uint8_t)25)-1] = '\n';
					send = true;
				break;
			}
			case FE:{
					strcpy((char *)(Message.messageData), "Button_2_Falling_Edge");		
					Message.messageData[((uint8_t)25)-1] = '\n';
					send = true;
				break;
			}
			default: {
					send = false;
				break;
			}
		}
		
		if(send == true){
			xQueueSendToBack(Queue, &Message, ((uint8_t)5));
		}
		vTaskDelayUntil( &last_time, ((uint8_t)50)	);
	}
}

void Periodic_Transmitter(void *vParam){
	TickType_t last_time;
	message_t Message = {0, 0};
	Message.messageID = '3';

	vTaskSetApplicationTaskTag(NULL, (TaskHookFunction_t)3);
	
	last_time = xTaskGetTickCount();
	for(;;){
	  strcpy((char *)(Message.messageData), "Periodic_Transmitter");		
		Message.messageData[((uint8_t)25)-1] = '\n';
		
		xQueueSendToBack(Queue, &Message, ((uint8_t)5));
		vTaskDelayUntil( &last_time, ((uint8_t)100)	 );
	}
}

 
void Uart_Receiver(void *vParam){
	TickType_t last_time;
	message_t xMessageBuffer;
	
	
	vTaskSetApplicationTaskTag(NULL, (TaskHookFunction_t)4);
	
	last_time = xTaskGetTickCount();
	for(;;){

			if( xQueueReceive( Queue,
                         &( xMessageBuffer ),
                         ( TickType_t ) ((uint8_t)5)) == pdPASS )
      {
         vSerialPutString((signed char *)(xMessageBuffer.messageData), ((uint8_t)25));
      }
			
		vTaskDelayUntil( &last_time, ((uint8_t)20)	);
	}
}


void Load_1_Simulation(void *vParam){
	TickType_t last_time;
	uint32_t  i;																						
	vTaskSetApplicationTaskTag(NULL, (TaskHookFunction_t)5);
	
	last_time = xTaskGetTickCount();
	for(;;){
			for(i=0; i<(((uint8_t)5) * ((uint16_t)6666)); i++){									
				i=i;																									
			}
		vTaskDelayUntil( &last_time, ((uint8_t)10)	 );
	}
}


void Load_2_Simulation(void *vParam){
	TickType_t last_time;
	uint32_t  i;																						
	vTaskSetApplicationTaskTag(NULL, (TaskHookFunction_t)6);
	
	last_time = xTaskGetTickCount();
	for(;;){	
		for(i=0; i<(((uint8_t)12) * ((uint16_t)6666)); i++){									
				i=i;																									
			}
		vTaskDelayUntil( &last_time, ((uint8_t)100)	 );
	}
}
