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
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "lpc21xx.h"
#include "queue.h"

/* Peripheral includes. */
#include "serial.h"
#include "GPIO.h"

/*********************************************************************
*********************************************************************/

/* Constants to setup I/O and processor. */
#define mainBUS_CLK_FULL	( ( unsigned char ) 0x01 )

/* Constants for the ComTest demo application tasks. */
#define mainCOM_TEST_BAUD_RATE	( ( unsigned long ) 115200 )


unsigned int Button1_inTime;
unsigned int Button2_inTime;
unsigned int Simulation1_inTime;
unsigned int Simulation2_inTime;
unsigned int Tx_inTime;
unsigned int Rx_inTime;

unsigned int execution_time=0;
unsigned int cpu_load=0;

/*-----------------------------------------------------------*/
/*Queue Handlers*/
QueueHandle_t Queue1 = NULL;
QueueHandle_t Queue2 = NULL;
QueueHandle_t Queue3 = NULL;

/*Task Handlers*/
TaskHandle_t Reciever_Handler = NULL;
TaskHandle_t Transmitter_Handler = NULL;
TaskHandle_t Sim1_Handler = NULL;
TaskHandle_t Sim2_Handler = NULL;
TaskHandle_t BTN1_Handler = NULL;
TaskHandle_t BTN2_Handler = NULL;

/*Tasks Functions*/
void Button_1_Monitor( void * pvParameters )
{
    pinState_t BTN1_Current_State;
    pinState_t Previous_State = GPIO_read(PORT_1 , PIN0);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    signed char EdgeFlag = 0;

    for( ;; )
    {
        /* Read GPIO Input */
        BTN1_Current_State = GPIO_read(PORT_1 , PIN0);

			
			switch(Previous_State)
				case PIN_IS_LOW:
					if(BTN1_Current_State == PIN_IS_HIGH){
						EdgeFlag = 'R';
					}
					break;
				case(PIN_IS_HIGH):
					if(BTN1_Current_State == PIN_IS_LOW){
						EdgeFlag = 'F';
					}
					break;
				

        /*Update Button State*/
        Previous_State = BTN1_Current_State;

        /*Send Data to consumer*/
        xQueueOverwrite( Queue1 , &EdgeFlag );

        /*Periodicity: 50*/
        vTaskDelayUntil( &xLastWakeTime , 50);
    }
}



void Button_2_Monitor( void * pvParameters )
{
    pinState_t Previous_State = GPIO_read(PORT_1 , PIN0);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    signed char EdgeFlag = 0;
    pinState_t BTN2_Current_State;

    for( ;; )
    {
        /* Read GPIO Input */
        BTN2_Current_State = GPIO_read(PORT_1 , PIN1);

			switch(Previous_State)
			{
				case PIN_IS_LOW:
					if(BTN2_Current_State == PIN_IS_HIGH){
						EdgeFlag = 'R';
					}
					break;
				case(PIN_IS_HIGH):
					if(BTN2_Current_State == PIN_IS_LOW){
						EdgeFlag = 'F';
					}
					break;
				}

        /*Update Button State*/
        Previous_State = BTN2_Current_State;

        /*Send Data to consumer*/
        xQueueOverwrite( Queue2 , &EdgeFlag );

        /*Periodicity: 50*/
        vTaskDelayUntil( &xLastWakeTime , 50);
    }
}

void Periodic_Transmitter (void * pvParameters )
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint16_t i = 0;
    char * pcString = "Periodic String"; /*100ms Marker.*/
    while(1)
    {
        /*Send string characters over Queue Queue3 to Uart_Receiver*/
        while(i<16)
        {
            xQueueSend( Queue3 , pcString+i ,100);
						i++;
        }

        /*Periodicity: 100*/
        vTaskDelayUntil( &xLastWakeTime , 100);
    }
}

void Uart_Receiver (void * pvParameters )
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    signed char BTN1;
    signed char BTN2;
    char rxString[15];
    uint8_t i=0;
    while(1)
    {
        /*Receive Button 1 State*/
        if( xQueueReceive( Queue1, &BTN1 , 0) && BTN1 == 'R')
        {
					vSerialPutString("Btn 1:",5);
            xSerialPutChar(BTN1);
        }
        else if(xQueueReceive( Queue1, &BTN1 , 0) && BTN1 == 'F')
{
            vSerialPutString("Btn 1:",5);
            xSerialPutChar(BTN1);
}
        else
        {
        vSerialPutString("       ",8);
        }

        /*Receive Button 2 state*/
        if( xQueueReceive( Queue2, &BTN2 , 0) && BTN2 == 'R')
        {

            vSerialPutString("Btn 2:",5);
            xSerialPutChar(BTN2);
        }
        else if(xQueueReceive( Queue2, &BTN2 , 0) && BTN2 == 'F')
{
          vSerialPutString("Btn 2:",5);
            xSerialPutChar(BTN2);
}
        else
        {
					vSerialPutString("       ",8);
        }

        /*Receive String from Periodic_Transmitter*/
        if( uxQueueMessagesWaiting(Queue3) != 0)
        {
						i=1;
            while(i<16)
            {
                xQueueReceive( Queue3, (rxString+i-1) , 0);
            }
            vSerialPutString( (signed char *) rxString, strlen(rxString));
            xQueueReset(Queue3);
        }


        /*Periodicity: 20*/
        vTaskDelayUntil( &xLastWakeTime , 20);
    }
}

void Load_1_Simulation ( void * pvParameters )
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint32_t i = 0;
    uint32_t x = 60000; /* (XTAL / 1000U)*time_in_ms  */
    for( ; ; )
    {
        for( i=0 ; i <= x; i++)
        {
        }
        /*Periodicity: 10*/
        vTaskDelayUntil( &xLastWakeTime , 10);
    }
}

void Load_2_Simulation ( void * pvParameters )
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint32_t i = 0;
    uint32_t x = 60000; /* (XTAL / 1000U)*time_in_ms  */

    for( ; ; )
    {
        for( i=0 ; i <= x; i++)
        {
        }

        /*Periodicity: 100*/
        vTaskDelayUntil( &xLastWakeTime , 100);

    }
}



void vApplicationTickHook (void){

    GPIO_write(PORT_0, PIN0, PIN_IS_HIGH);
    GPIO_write(PORT_0, PIN0, PIN_IS_LOW);

}


/*
 * Configure the processor for use with the Keil demo board.  This is very
 * minimal as most of the setup is managed by the settings in the project
 * file.
 */
static void prvSetupHardware( void );
/*-----------------------------------------------------------*/




/*
 * Application entry point:
 * Starts all the other tasks, then starts the scheduler.
 */
int main( void ){

    /* Setup the hardware for use with the Keil demo board. */
    prvSetupHardware();
Queue1 = xQueueCreate( 1, sizeof(char) );
Queue2 = xQueueCreate( 1, sizeof(char) );
Queue3 = xQueueCreate( 15, sizeof(char) );

    /* Create Tasks here */
xTaskPeriodicCreate(
                    Button_1_Monitor,       /* Function that implements the task. */
                    "Button1",     /* Text name for the task. */
                    configMINIMAL_STACK_SIZE,      				/* Stack size in words, not bytes. */
                    ( void * ) 0,    		/* Parameter passed into the task. */
                    1,						/* Priority at which the task is created. */
                    &BTN1_Handler		/* Used to pass out the created task's handle. */
                                        ,50);      				/* Task Deadline */

xTaskPeriodicCreate(
                    Button_2_Monitor,       /* Function that implements the task. */
                    "Button2",     /* Text name for the task. */
                    configMINIMAL_STACK_SIZE,      				/* Stack size in words, not bytes. */
                    ( void * ) 0,    		/* Parameter passed into the task. */
                    1,						/* Priority at which the task is created. */
                    &BTN2_Handler,		/* Used to pass out the created task's handle. */
                                        50);      				/* Task Deadline */


xTaskPeriodicCreate(
                    Periodic_Transmitter,       /* Function that implements the task. */
                    "Tx",     /* Text name for the task. */
                    configMINIMAL_STACK_SIZE,      				/* Stack size in words, not bytes. */
                    ( void * ) 0,    		/* Parameter passed into the task. */
                    1,						/* Priority at which the task is created. */
                    &Transmitter_Handler,		/* Used to pass out the created task's handle. */
                                        100);          /*Task Deadline */



xTaskPeriodicCreate(
                    Uart_Receiver,       /* Function that implements the task. */
                    "Rx",     /* Text name for the task. */
                    configMINIMAL_STACK_SIZE,      				/* Stack size in words, not bytes. */
                    ( void * ) 0,    		/* Parameter passed into the task. */
                    1,						/* Priority at which the task is created. */
                    &Reciever_Handler,		/* Used to pass out the created task's handle. */
                                        20);      				/* Task Deadline */



xTaskPeriodicCreate(
                    Load_1_Simulation,       /* Function that implements the task. */
                    "L1",     /* Text name for the task. */
                    configMINIMAL_STACK_SIZE,      				/* Stack size in words, not bytes. */
                    ( void * ) 0,    		/* Parameter passed into the task. */
                    1,						/* Priority at which the task is created. */
                    &Sim1_Handler,		/* Used to pass out the created task's handle. */
                                        10);      				/* Task Deadline */



xTaskPeriodicCreate(
                    Load_2_Simulation,       /* Function that implements the task. */
                    "L2",     /* Text name for the task. */
                    configMINIMAL_STACK_SIZE,      				/* Stack size in words, not bytes. */
                    ( void * ) 0,    		/* Parameter passed into the task. */
                    1,						/* Priority at which the task is created. */
                    &Sim2_Handler,		/* Used to pass out the created task's handle. */
                                        100);      				/* Task Deadline */





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



