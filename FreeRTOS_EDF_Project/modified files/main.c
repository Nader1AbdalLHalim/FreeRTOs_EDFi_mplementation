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

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "lpc21xx.h"

/* Peripheral includes. */
#include "serial.h"
#include "GPIO.h"


/*-----------------------------------------------------------*/

/* Constants to setup I/O and processor. */
#define mainBUS_CLK_FULL	( ( unsigned char ) 0x01 )

/* Constants for the ComTest demo application tasks. */
#define mainCOM_TEST_BAUD_RATE	( ( unsigned long ) 115200 )

TaskHandle_t ButtonTaskHandler = NULL;
volatile int Misses=0;
/*
 * Configure the processor for use with the Keil demo board.  This is very
 * minimal as most of the setup is managed by the settings in the project
 * file.
 */
static void prvSetupHardware( void );
/*-----------------------------------------------------------*/

/*-----------------------------------------------------------*/
                         /* TASK PERIOD */
/*-----------------------------------------------------------*/


#define Button1_Period          (TickType_t)50
#define Button2_Period          (TickType_t)50
#define TRANSMITTER_Period      (TickType_t)100
#define RECEIVER_Period         (TickType_t)20
#define LOAD1_Period            (TickType_t)10
#define LOAD2_Period            (TickType_t)100

/*-----------------------------------------------------------*/
                         /* NEW TYPES */
/*-----------------------------------------------------------*/


/* Queue message datatypes */
typedef enum 
{
	Button1,
	Button2,
	Transmitter
}TaskID_t;

typedef enum 
{
	Button_FALL=-1,
	Button_STABLE,
	Button_RISE
}ButtonStat_t;

/* Queue message struct */
typedef struct 
{
	TaskID_t TaskID;
	ButtonStat_t Button_Status;
	char String[sizeof("Monitoring buttons...!")];
}QueM_t;


/*-----------------------------------------------------------*/
                 /* Tasks handles */
/*-----------------------------------------------------------*/
TaskHandle_t Button1_TaskHandle = NULL;
TaskHandle_t Button2_TaskHandle = NULL;
TaskHandle_t Transmitter_TaskHandle = NULL;
TaskHandle_t Receiver_TaskHandle = NULL;
TaskHandle_t Load1_TaskHandle = NULL;
TaskHandle_t Load2_TaskHandle = NULL;
/* QUE HANDLERS*/
QueueHandle_t xQueue1 ;														
QueueHandle_t xQueue2 ;
QueueHandle_t xQueue3 ;

/*-----------------------------------------------------------*/
/*-----------------------------------------------------------*/
                 /*TASKS IMPLEMENTAION*/
/*-----------------------------------------------------------*/
/*-----------------------------------------------------------*/

#if (configUSE_TICK_HOOK == 1) 
void vApplicationTickHook( void )
{

      GPIO_write(PORT_0,PIN0,PIN_IS_HIGH);
			GPIO_write(PORT_0,PIN0,PIN_IS_LOW);
}

#endif

#if (configUSE_IDLE_HOOK == 1) 
void vApplicationIdleHook( void )
{
GPIO_write(PORT_0,PIN5,PIN_IS_HIGH);
}
#endif

/* *************** Button 1 monitoring task *************** */
void Button_1_Monitor( void * pvParameters )
{
	volatile TickType_t xLastWakeTime = xTaskGetTickCount();	
    /* Create message struct */
	QueM_t B1_State;
    /* Create state variables and set initial values */
	pinState_t  CurrentState = PIN_IS_LOW;
	pinState_t  PrevState    = GPIO_read(PORT_0,PIN1);
	/* Assign task ID to message struct */
	B1_State.TaskID=Button1;
	/* Assign task tag */
    vTaskSetApplicationTaskTag(NULL,(void*)1);
    for( ;; )
	
    {
		/* Read GPIO pin and insert status into message*/
	    CurrentState = GPIO_read(PORT_0, PIN1);

/*Check */
		if( CurrentState == PIN_IS_HIGH && PrevState == PIN_IS_LOW)
		{
			B1_State.Button_Status= Button_RISE;

        }
		else if (CurrentState == PIN_IS_LOW && PrevState == PIN_IS_HIGH)
		{
	      	B1_State.Button_Status= Button_FALL;
		
		}
        else if (CurrentState == PIN_IS_LOW && PrevState == PIN_IS_LOW)
		{
	      	B1_State.Button_Status= Button_STABLE;
		
		}		
		/* Send message to xqueue1 */
   if( B1_State.Button_Status != Button_STABLE && xQueue1 != NULL)
   {
    xQueueOverwrite( xQueue1 , & B1_State );              /* Send Data  */  
   }
	
     PrevState = CurrentState ;
	 /* Put task to sleep untill  period */
	 vTaskDelayUntil( &xLastWakeTime , Button1_Period); 
	 /* Unset Ideal task pin on task entry */
	 GPIO_write(PORT_0,PIN5,PIN_IS_LOW);
	}
}


/* *************** Button 2 monitoring task *************** */
void Button_2_Monitor( void * pvParameters )
{
 volatile TickType_t xLastWakeTime = xTaskGetTickCount();	
    /* Create message struct */
	QueM_t B2_State;
    /* Create state variables and set initial values */
	pinState_t  CurrentState = PIN_IS_LOW;
	pinState_t  PrevState    = GPIO_read(PORT_0,PIN2);
	/* Assign task ID to message struct */
	B2_State.TaskID=Button2;
	/* Assign task tag */
    vTaskSetApplicationTaskTag(NULL,(void*)2);
    for( ;; )
	
    {
		/* Read GPIO pin and insert status into message*/
	    CurrentState = GPIO_read(PORT_0, PIN2);

/*Check */
		if( CurrentState == PIN_IS_HIGH && PrevState == PIN_IS_LOW)
		{
			B2_State.Button_Status= Button_RISE;

        }
		else if (CurrentState == PIN_IS_LOW && PrevState == PIN_IS_HIGH)
		{
	      	B2_State.Button_Status= Button_FALL;
		
		}
        else if (CurrentState == PIN_IS_LOW && PrevState == PIN_IS_LOW)
		{
	      	B2_State.Button_Status= Button_STABLE;
		
		}		
		/* Send message to xqueue2 */
   if( B2_State.Button_Status != Button_STABLE && xQueue2 != NULL)
   {
    xQueueOverwrite( xQueue2 , & B2_State );              /* Send Data  */  
   }
	
     PrevState = CurrentState ;
	 /* Put task to sleep untill next period */
	 vTaskDelayUntil( &xLastWakeTime , Button2_Period); 
	 /* Unset Ideal task pin on task entry */
	 GPIO_write(PORT_0,PIN5,PIN_IS_LOW);
	}
    
}

/* ***************  UART transmitter task *************** */
void Task_Transmitter( void * pvParameters )
{
	/* Create variable to store wake time for delays */
	volatile TickType_t xLastWakeTime = xTaskGetTickCount();
	/* Create message struct */
	QueM_t Trans_Message;
	/* Periodic string */
	volatile char PeriodicMsg[]="\nMonitoring buttons...!";
	/* Assign string to message struct */
	strcpy( Trans_Message.String, PeriodicMsg );
	/* Assign button ID to message struct */
	Trans_Message.TaskID=Transmitter;
	/* Assign task tag */
	vTaskSetApplicationTaskTag( NULL, ( void * ) 3 );
 for( ;; )
	{
		/* Send message to Consumer_Queue */
		if( xQueue3 != NULL )
		 {
			xQueueSend(  xQueue3,(void * ) &Trans_Message,(TickType_t) 0 );
		 }
	 /* Put task to sleep untill next period */
	 vTaskDelayUntil( &xLastWakeTime ,TRANSMITTER_Period); 
	 /* Unset Ideal task pin on task entry */
	 GPIO_write(PORT_0,PIN5,PIN_IS_LOW);
	}
}


/* ***************  UART receiver task *************** */
void Uart_Receiver( void * pvParameters )
{
	/* Create variable to store wake time for delays */
	volatile TickType_t xLastWakeTime = xTaskGetTickCount();
	/* Create message struct */
	QueM_t Rec_Message;
	/* Last UART msg flag */
	TaskID_t SentFlag;
	/* Assign task tag */
	vTaskSetApplicationTaskTag( NULL, ( void * ) 4 );
	for( ;; )
	{
	 /* Send message to Consumer_Queue */
	 if( xQueue3 != NULL )
	 {
		xQueueReceive(  xQueue3, &Rec_Message,(TickType_t) 0 );
	 }
	 /* Check received message task ID */
	 if(Rec_Message.TaskID==Transmitter && SentFlag != Transmitter)
	 {
		 vSerialPutString((const signed char * const)Rec_Message.String,sizeof(Rec_Message.String));
		 SentFlag = Transmitter;
	 }
	 else if (Rec_Message.TaskID==Button1 && SentFlag != Button1)
	 {
		 if (Rec_Message.Button_Status== Button_RISE)
		 {
			vSerialPutString((const signed char * const)"\nButton 1 is ON!",sizeof("\nButton 1 is OFF!"));
		 }
		 else if (Rec_Message.Button_Status==Button_FALL)
		 {
			vSerialPutString((const signed char * const)"\nButton 1 is OFF!",sizeof("\nButton 1 is OFF!"));
		 }
		 SentFlag = Button1;
	 }
	 else if (Rec_Message.TaskID==Button2 && SentFlag != Button2)
	 {
		 if (Rec_Message.Button_Status==Button_RISE)
		 {
			vSerialPutString((const signed char * const)"\nButton 2 is ON!",sizeof("\nButton 2 is OFF!"));
		 }
		 else if (Rec_Message.Button_Status==Button_FALL)
		 {
			vSerialPutString((const signed char * const)"\nButton 2 is OFF!",sizeof("\nButton 2 is OFF!"));
		 }
		 SentFlag = Button2;
	 }
	 /* Put task to sleep untill next period */
	 vTaskDelayUntil( &xLastWakeTime , RECEIVER_Period); 
	 /* Unset Ideal task pin on task entry */
	 GPIO_write(PORT_0,PIN5,PIN_IS_LOW);
	}
}
	
/* *************** Load task 1 with 5 ms execution time *************** */
void Load_1_Simulation( void * pvParameters )
{		
	/* Create variable to store wake time for delays */
	volatile TickType_t xLastWakeTime = xTaskGetTickCount();
	/* calculate delay time using 
	(XTAL / 1000U)*time_in_ms  */
	uint32_t delay = 12000*5;  
	/* For loop iterator */
	uint32_t i=0;
	/* Assign task tag */
	vTaskSetApplicationTaskTag( NULL, ( void * ) 5 );
	for( ;; )
	{
		for(i=0;i<=delay;i++)
		{
			/* Do something to avoid 
			removing for loop for optimization */
			//i=i;
		}
	 /* Put task to sleep untill next period */
	 vTaskDelayUntil( &xLastWakeTime , LOAD1_Period); 
	 /* Unset Ideal task pin on task entry */
	 GPIO_write(PORT_0,PIN5,PIN_IS_LOW);
	}
}

/* *************** Load task 2 with 12 ms execution time *************** */
void Load_2_Simulation( void * pvParameters )
{
	/* Create variable to store wake time for delays */
	volatile TickType_t xLastWakeTime = xTaskGetTickCount();
	/* calculate delay time using equation
	(XTAL / 1000U)*time_in_ms  */
	uint32_t delay = 12000*12;  
	/* For loop iterator */
	uint32_t i=0;
	/* Assign task tag */
	vTaskSetApplicationTaskTag( NULL, ( void * ) 6 );
	for( ;; )
	{
		for(i=0;i<=delay;i++)
		{
			/* Do something to avoid 
			removing for loop for optimization */
			//i=i;
		}
	 /* Put task to sleep untill next period */
	 vTaskDelayUntil( &xLastWakeTime , LOAD2_Period); 
	 /* Unset Ideal task pin on task entry */
	 GPIO_write(PORT_0,PIN5,PIN_IS_LOW);
	}
}


/*
 * Application entry point:
 * Starts all the other tasks, then starts the scheduler. 
 */
unsigned int task1_IN_TIME, task1_OUT_TIME, task1_TOTAL_TIME;
unsigned int task2_IN_TIME, task2_OUT_TIME, task2_TOTAL_TIME;
unsigned int task3_IN_TIME, task3_OUT_TIME, task3_TOTAL_TIME;
unsigned int task4_IN_TIME, task4_OUT_TIME, task4_TOTAL_TIME;
unsigned int task5_IN_TIME, task5_OUT_TIME, task5_TOTAL_TIME;
unsigned int task6_IN_TIME, task6_OUT_TIME, task6_TOTAL_TIME;

int CPU_LOAD = 0;
int SYS_TIME = 0;


int main( void )
 {
	/* Setup the hardware for use with the Keil demo board. */
	prvSetupHardware();

  xQueue1=xQueueCreate( 10, sizeof( char ));
  xQueue2=xQueueCreate( 10, sizeof( char ));
  xQueue3=xQueueCreate( 29, sizeof( char ));
		
  xTaskPeriodicCreate( Button_1_Monitor, "Button_1_Monitor", 100, ( void * ) 0, 1, &Button1_TaskHandle, Button1_Period );
  xTaskPeriodicCreate( Button_2_Monitor, "Button_2_Monitor", 100, ( void * ) 0, 1, &Button2_TaskHandle, Button2_Period );
	xTaskPeriodicCreate( Task_Transmitter, " Task_Transmitter", 100, ( void * ) 0, 1, &Transmitter_TaskHandle,TRANSMITTER_Period);
  xTaskPeriodicCreate( Uart_Receiver , "Uart_Receiver", 100, ( void * ) 0, 1, &Receiver_TaskHandle, RECEIVER_Period);
  xTaskPeriodicCreate( Load_1_Simulation , " Load_1_Simulation", 100, ( void * ) 0, 1, &Load1_TaskHandle, LOAD1_Period );
  xTaskPeriodicCreate( Load_2_Simulation, " Load_2_Simulation", 100, ( void * ) 0, 1, &Load2_TaskHandle, LOAD2_Period ); 
	 
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


