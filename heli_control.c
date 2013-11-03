/**  @file   heli_control.c
     @author Nick Wareing
     @date   12 August 2013
*/

#include "heli_control.h"

#include "queue_structs.h"

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"

#include "driverlib/debug.h"
#include "driverlib/sysctl.h"
#include "driverlib/adc.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"

#define UP 0
#define DOWN 1
#define INITIAL_ALT 20

static void vGetAltitude ( void *pvParameters );
static unsigned int getRawADC( void );
static unsigned int desiredAltitude = INITIAL_ALT;

xQueueHandle xButtonQueue;
xQueueHandle xAltQueue;
xSemaphoreHandle xBinarySelectSemaphore;

typedef struct
{
	double error_integrated;
	double error_previous ;
} pid_state_t ;

static pid_state_t APID;

/* The handler for the GPIOPortB Pin4, Pin5 & Pin6 state change interrupt. */
void portBIntHandler(void) {
    //clear the interrupt
    GPIOPinIntClear (GPIO_PORTB_BASE, (GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6));

    int button;
    //Read the state of the pins

    //Select icon clicked to start/stop heli
    if (!GPIOPinRead( GPIO_PORTB_BASE, GPIO_PIN_4 )) {
    	xSemaphoreGiveFromISR( xBinarySelectSemaphore, pdFALSE );
	}

    //Up icon clicked
    if (!GPIOPinRead( GPIO_PORTB_BASE, GPIO_PIN_5 )) {
    	button = UP;
    	xQueueSendFromISR( xButtonQueue, &button, NULL );
	}

    //Down icon clicked
    if (!GPIOPinRead( GPIO_PORTB_BASE, GPIO_PIN_6 )) {
    	button = DOWN;
    	xQueueSendFromISR( xButtonQueue, &button, NULL );
	}

    //Reset icon clicked
    if (!GPIOPinRead( GPIO_PORTB_BASE, GPIO_PIN_1 )) {
		SysCtlReset();
	}
}

static unsigned int getRawADC( void ) {
	
	unsigned int rawADCvalue;
	unsigned long rawADCarray[1];
	
	// Trigger the ADC conversion.
    ADCProcessorTrigger(ADC0_BASE, 3);  
    //
    // Wait for conversion to be completed.
    while(!ADCIntStatus(ADC0_BASE, 3, false))  
    {
    }
    
    ADCIntClear(ADC0_BASE, 3);
   
    // Read ADC Value.
    ADCSequenceDataGet(ADC0_BASE, 3, rawADCarray);
    rawADCvalue = rawADCarray[0];
    
    return rawADCvalue;
}

static void pid_init ()
{
	APID.error_integrated = 0.0;
	APID.error_previous = 0.0;
}

static float pid_update (float error, double proportional_gain, double integral_gain, double derivative_gain, double delta_t)
{
	float error_derivative;
	float control;

	APID.error_integrated += error * delta_t;
	error_derivative = (error - APID.error_previous)/ delta_t;

	control = error * proportional_gain + APID.error_integrated * integral_gain + error_derivative * derivative_gain;

	APID.error_previous = error;

	//Limit the value to the duty cycle limits.
	if (control > 95){
		control = 95;
	}
	if (control < 5){
		control = 5;
	}

	return control;
}

/* Control the flight of the helicopter. */
static void vPIDControlHeli( void *pvParameters)
{
	//Set the altitude gains.
	static const float Aki = 0.4, Akp = 3.0, Akd = 0.25;
	static const float delta_t = 0.02;
	float altError, altResponse;
	float currentAltitude = 0;

	portBASE_TYPE xStatus;
	int buttonMessage;
	xAltMessage altMessage;
	xQueueHandle xOLEDQueue = ( xQueueHandle ) pvParameters;

	portTickType xLastWakeTime = xTaskGetTickCount();
	/* Define a period of 2 milliseconds (500Hz) */
	const portTickType xPeriod = ( 3 / portTICK_RATE_MS );

	for ( ;; ) {
		vTaskDelayUntil( &xLastWakeTime, xPeriod );

		// Act on button presses sent from the ISR.
		xStatus = xQueueReceive( xButtonQueue, &buttonMessage, 0 );
		if (xStatus == pdPASS)
		{
			switch (buttonMessage)
			{
				case UP:
					if (desiredAltitude <= 80)
						desiredAltitude += 10;
					break;
				case DOWN:
					if (desiredAltitude >= 10)
						desiredAltitude -= 10;
					break;
			}

			xQueueMessage message = { DESIRED_ALTITUDE, desiredAltitude };
			xQueueSendToBack( xOLEDQueue, &message, 0 );
		}

		// Recompute the PID output using the latest altitude value.
		xStatus = xQueueReceive( xAltQueue, &altMessage, 0 );
		if (xStatus == pdPASS)
		{
			currentAltitude = altMessage.pcMessage;
			altError = desiredAltitude - currentAltitude;
			altResponse = pid_update(altError, Akp, Aki, Akd, delta_t);
			PWMPulseWidthSet(PWM_BASE, PWM_OUT_1, ( SysCtlClockGet() / 150 ) * altResponse / 100);
		}
	}
}

/* Poll the ADC for altitude values and put the
 * latest values on the altitude queue.
 */
static void vGetAltitude(void *pvParameters) {
	unsigned int groundValue = getRawADC();
	unsigned int maxValue = groundValue - 270;
	unsigned int rawADCvalue;
	float altitude;
	xQueueHandle xOLEDQueue = ( xQueueHandle ) pvParameters;

	portTickType xLastWakeTime = xTaskGetTickCount();
	/* Define a period of 2 milliseconds (500Hz) */
	const portTickType xPeriod = ( 3 / portTICK_RATE_MS );

	for ( ;; ) {
		vTaskDelayUntil( &xLastWakeTime, xPeriod );

		rawADCvalue = getRawADC();

		//Dynamically adjust the ground voltage value (differs on each helicopter).
		if (rawADCvalue > groundValue) {
			groundValue = rawADCvalue;
		}
		//Dynamically adjust the max altitude voltage value.
		if (rawADCvalue < (groundValue - 320)) {
			rawADCvalue = maxValue;
		}

		//Convert the voltage values to a percentage of the total height.
		altitude =  100.0f * (groundValue - rawADCvalue) / (groundValue - maxValue);

		/* Uncomment to send the current altitude to the OLED */
		//xQueueMessage message = { CURRENT_ALTITUDE, altitude };
		//xQueueSendToBack( xOLEDQueue, &message, 0 );

		xAltMessage altMessage = { altitude };
		xQueueSendToBack( xAltQueue, &altMessage, 0 );
	}
}

/* Act on select button pushes received from the ISR.
 * Toggle the helicopter rotor on and off with each push.
 */
static void vStartStopPWM ( void *pvParameters )
{
    /* Take the semaphore once to start with so the semaphore is empty before the
    infinite loop is entered.  The semaphore was created before the scheduler
    was started so before this task ran for the first time.*/
    xSemaphoreTake( xBinarySelectSemaphore, 0 );
    portBASE_TYPE flying = pdFALSE;

    for( ;; )
    {
        /* Use the semaphore to wait for the event. */
        xSemaphoreTake( xBinarySelectSemaphore, portMAX_DELAY );

        if (flying == pdFALSE) {
		    pid_init();
		    // Turn the main rotor on.
			PWMOutputState(PWM_BASE, PWM_OUT_1_BIT, true);
			// Reset the desired height
			desiredAltitude = INITIAL_ALT;
			flying = pdTRUE;
        } else {
        	// Turn the thing off.
        	while (desiredAltitude > 5) {
        		desiredAltitude -= 2;
        		vTaskDelay( 100 / portTICK_RATE_MS );
        	}
		    PWMOutputState(PWM_BASE, PWM_OUT_1_BIT, false);
        	flying = pdFALSE;
        }
    }
}

void vCreateQueuesAndSemaphore( void )
{
	vSemaphoreCreateBinary( xBinarySelectSemaphore );
	xButtonQueue = xQueueCreate( 1, sizeof( int ) );
	xAltQueue = xQueueCreate( 3, sizeof( xAltMessage ) );
}

void vStartControlTasks( xQueueHandle xOLEDQueue )
{
	xTaskCreate( vPIDControlHeli, "PID Control", 240, (void *) xOLEDQueue, 2, NULL );
	xTaskCreate( vGetAltitude, "Get Altitude", 240, (void *) xOLEDQueue, 2, NULL );
	xTaskCreate( vStartStopPWM, "Start/Stop Heli", 180, NULL, 5, NULL );
}
