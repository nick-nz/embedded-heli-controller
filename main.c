/**  @file   main.c
     @author Nick Wareing
     @date   12 August 2013
*/

#include "heli_control.h"
#include "heli_init.h"

#include "queue_structs.h"

#include <stdio.h>

/* FreeRTOS includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* Standard Stellaris includes */
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"

#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"

/* Other Stellaris include */
#include "drivers/rit128x96x4.h"

/* Constants used when writing strings to the display. */
#define mainCHARACTER_HEIGHT				( 9 )
#define mainMAX_ROWS_96						( mainCHARACTER_HEIGHT * 10 )

/* The maximum number of message that can be waiting for display at any one
time. */
#define mainSEND_QUEUE_SIZE					( 3 )

xQueueHandle xSendQueue;

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, unsigned long ulLine)
{
}
#endif

/*
 * Hook functions that can get called by the kernel.
 */
void vApplicationIdleHook( void );
void vApplicationTickHook( void );
void vApplicationStackOverflowHook( xTaskHandle *pxTask, signed portCHAR *pcTaskName );
void vApplicationTickHook( void );


/* Send a string to the UART. */
static void UARTSend(portCHAR *pucBuffer, unsigned long ulCount)
{
    // Loop while there are more characters to send.
    while(ulCount--)
    {
        // Write the next character to the UART.
        UARTCharPutNonBlocking(UART0_BASE, *pucBuffer++);
    }
}

/* Pop messages from the queue and send to the OLED
 * display and UART.
 */
static void vSendTask( void *pvParameters )
{
	portBASE_TYPE xStatus;
	xQueueMessage xMessage;
	static portCHAR cMessage[ 100 ];
	static unsigned int ulY, ulMaxY = mainMAX_ROWS_96;

	ulY = ulMaxY;

	for( ;; )
	{
		/* Wait for a message to arrive that requires displaying. */
		xStatus = xQueueReceive( xSendQueue, &xMessage, portMAX_DELAY );

		/* Write the message on the next available row. */
		ulY += mainCHARACTER_HEIGHT;
		if( ulY >= ulMaxY )
		{
			ulY = mainCHARACTER_HEIGHT;
			RIT128x96x4Clear();
		}
        if (xStatus == pdPASS)
        {
        	switch (xMessage.type)
        	{
        		case (CURRENT_ALTITUDE):
					sprintf(cMessage, "Current Alt: %d", xMessage.pcMessage);
        			break;
        		case (DESIRED_ALTITUDE):
					sprintf(cMessage, "Desired Alt: %d", xMessage.pcMessage);
        			break;
        		case (PWM_DUTY):
					sprintf(cMessage, "PWM Duty: %d", xMessage.pcMessage);
					break;
        		default:
        			sprintf(cMessage, "Other: %d", xMessage.pcMessage);
        	}
            RIT128x96x4StringDraw(cMessage, 0, ulY, 8);
            /* Uncomment to send on the UART as well */
            // UARTSend( cMessage, 100 );
        }
	}
}

/* A blinking LED for purely cosmetic reasons. */
static void vLedBlink( void *pvParameters )
{
    volatile unsigned long ul;
    static long toggle = 0x00;

    /* As per most tasks, this task is implemented in an infinite loop. */
    for( ;; )
    {
        toggle ^= 0x01;
        if (toggle)
        {
            GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_2, GPIO_PIN_2);
        }
        else
        {
            GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_2, 0);
        }

        vTaskDelay( 500 / portTICK_RATE_MS );
    }
}
/*-----------------------------------------------------------*/

int main(void)
{
    // If running on Rev A2 silicon, turn the LDO voltage up to 2.75V.  This is
    // a workaround to allow the PLL to operate reliably.
    if (REVISION_IS_A2)
    {
        SysCtlLDOSet(SYSCTL_LDO_2_75V);
    }

    // Set the clocking to run at 50MHz from the PLL.
    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                   SYSCTL_XTAL_8MHZ);

    SysCtlPWMClockSet(SYSCTL_PWMDIV_1);

    // Initialise the OLED display.
    RIT128x96x4Init(1000000);

    // Initialise the required peripherals.
    initStatusLight();
    initialisePortB();
    initialiseButtons();
    initialisePWM();
    initialiseADC();
    initialiseUART();

	/* Create the queue used by the OLED task.  Messages for display on the OLED
	are received via this queue. */
	xSendQueue = xQueueCreate( mainSEND_QUEUE_SIZE, sizeof( xQueueMessage ) );
	vCreateQueuesAndSemaphore();

    /*-------------------------------------------
         Create tasks and start scheduler
    -------------------------------------------*/

    /* Create the required tasks */
    xTaskCreate( vSendTask, "Send Task", 240, NULL, 1, NULL);
    xTaskCreate( vLedBlink, "LED Blink", configMINIMAL_STACK_SIZE, NULL, 4, NULL );
    vStartControlTasks( xSendQueue );

	// Enable interrupts to the processor.
	IntMasterEnable();

    /* Start the scheduler so our tasks start executing. */
    vTaskStartScheduler();

    /* If all is well we will never reach here as the scheduler will now be
    running.  If we do reach here then it is likely that there was insufficient
    heap available for the idle task to be created. */
    while (1)
    {
    }
}

/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
}

/*-----------------------------------------------------------*/

void vApplicationTickHook( void )
{
}

/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook( void )
{
    /* This function will only be called if an API call to create a task, queue
    or semaphore fails because there is too little heap RAM remaining. */
    for( ;; );
}

/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( xTaskHandle *pxTask, signed portCHAR *pcTaskName )
{
    /* This function will only be called if a task overflows its stack.  Note
    that stack overflow checking does slow down the context switch
    implementation. */
    for( ;; );
}

