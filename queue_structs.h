#ifndef LCD_MESSAGE_H
#define LCD_MESSAGE_H

#define CURRENT_ALTITUDE 1
#define DESIRED_ALTITUDE 2
#define PWM_DUTY 3

typedef struct
{
	int type;
	int pcMessage;
} xQueueMessage;

typedef struct
{
	float pcMessage;
} xAltMessage;

#endif /* LCD_MESSAGE_H */
