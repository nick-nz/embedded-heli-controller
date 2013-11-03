/**  @file   heli_init.c
     @author Nick Wareing
     @date   12 August 2013
*/

#include "heli_init.h"
#include "heli_control.h"

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"

#include "driverlib/gpio.h"
#include "driverlib/adc.h"
#include "driverlib/pwm.h"
#include "driverlib/uart.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"

void initialiseUART(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    // Set GPIO A0 and A1 as UART pins.
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // Configure the UART for 115,200, 8-N-1 operation.
    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                         UART_CONFIG_PAR_NONE));
}

void initialisePWM(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

    // Set GPIO G2 and H0 as PWM pins.  They are used to output the PWM0 and
    // PWM1 signals.
    GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_1);

    // Set the PWM period to 150Hz.
    unsigned long ulPeriod = SysCtlClockGet() / 150;

    PWMGenConfigure(PWM_BASE, PWM_GEN_0,
                    PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM_BASE, PWM_GEN_0, ulPeriod);

    // Set the pulse width of PWM1 for a 75% duty cycle.
    PWMPulseWidthSet(PWM_BASE, PWM_OUT_1, ulPeriod * 1 / 100);

	PWMGenEnable(PWM_BASE, PWM_GEN_0);

    // Enable the outputs.
    PWMOutputState(PWM_BASE, PWM_OUT_1_BIT, false);
}

void initialiseADC( void )
{
  //
  // The ADC0 peripheral must be enabled for use.
  SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);

  // Enable sample sequence 3 with a processor signal trigger.  Sequence 3
  // will do a single sample when the processor sends a signal to start the
  // conversion.  Each ADC module has 4 programmable sequences, sequence 0
  // to sequence 3.  This example is arbitrarily using sequence 3.
  ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);

  //
  // Configure step 0 on sequence 3.  Sample channel 0 (ADC_CTL_CH0) in
  // single-ended mode (default) and configure the interrupt flag
  // (ADC_CTL_IE) to be set when the sample is done.  Tell the ADC logic
  // that this is the last conversion on sequence 3 (ADC_CTL_END).  Sequence
  // 3 has only one programmable step.  Sequence 1 and 2 have 4 steps, and
  // sequence 0 has 8 programmable steps.  Since we are only doing a single
  // conversion using sequence 3 we will only configure step 0.  For more
  // information on the ADC sequences and steps, reference the datasheet.
  ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH0 | ADC_CTL_IE |
                             ADC_CTL_END);

  //
  // Since sample sequence 3 is now configured, it must be enabled.
  ADCSequenceEnable(ADC0_BASE, 3);

  //
  // Clear the interrupt status flag.  This is done to make sure the
  // interrupt flag is cleared before we sample.
  ADCIntClear(ADC0_BASE, 3);
}

void
initStatusLight (void) {
    // Enable the GPIO port that is used for the on-board LED.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);

    // Enable the GPIO pin for the LED (PG2).
    GPIOPinTypeGPIOOutput(GPIO_PORTG_BASE, GPIO_PIN_2);
}

void
initialiseButtons(void) {
    //
    //Initialise the push buttons
    //
    GPIODirModeSet(GPIO_PORTG_BASE, GPIO_PIN_3, GPIO_DIR_MODE_IN);
    GPIOPadConfigSet(GPIO_PORTG_BASE, GPIO_PIN_3, GPIO_STRENGTH_2MA,
                     GPIO_PIN_TYPE_STD_WPU);

  	GPIODirModeSet(GPIO_PORTG_BASE, GPIO_PIN_4, GPIO_DIR_MODE_IN);
  	GPIOPadConfigSet(GPIO_PORTG_BASE, GPIO_PIN_4, GPIO_STRENGTH_2MA,
                     GPIO_PIN_TYPE_STD_WPU);

	GPIODirModeSet(GPIO_PORTG_BASE, GPIO_PIN_5, GPIO_DIR_MODE_IN);
	GPIOPadConfigSet(GPIO_PORTG_BASE, GPIO_PIN_5, GPIO_STRENGTH_2MA,
	                     GPIO_PIN_TYPE_STD_WPU);

	GPIODirModeSet(GPIO_PORTG_BASE, GPIO_PIN_6, GPIO_DIR_MODE_IN);
	GPIOPadConfigSet(GPIO_PORTG_BASE, GPIO_PIN_6, GPIO_STRENGTH_2MA,
	                    GPIO_PIN_TYPE_STD_WPU);

	GPIOPinTypeGPIOInput (GPIO_PORTG_BASE, GPIO_PIN_7);
}

void
initialisePortB (void) {
    // GPIO PortB must be enabled for configuration and use.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    // Set pins 5 & 6 as input, SW controlled.
    GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6);

    // Register the port-level interrupt handler.
    GPIOPortIntRegister(GPIO_PORTB_BASE, portBIntHandler);

    // Make pins 5 & 6 a "both edges" triggered interrupt.
    GPIOIntTypeSet(GPIO_PORTB_BASE, (GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6), GPIO_FALLING_EDGE);

    // Set the interrupt priority to '0x40'.
    IntPrioritySet(INT_GPIOB, 0x40);

    // Enable the pin interrupts.
    GPIOPinIntEnable(GPIO_PORTB_BASE, (GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6));

    IntEnable (INT_GPIOB);  // Note: INT_GPIOB defined in inc/hw_ints.h
}
