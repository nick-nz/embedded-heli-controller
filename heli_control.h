#ifndef ALTITUDE_H_
#define ALTITUDE_H_

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

void portBIntHandler(void);
void vStartControlTasks ( xQueueHandle xOLEDQueue );
void vCreateQueuesAndSemaphore( void );

#endif /*ALTITUDE_H_*/
