#ifndef __REMOTE_CONTROL__H
#define __REMOTE_CONTROL__H

#include "rc_sensor.h"

typedef enum
{
	UP,
	DOWN,
	PRESS,
	RELAX
}KEY_STATE;

typedef struct remote_t
{
	rc_sensor_t rc_info;
	
	
}remote;

void RC_key_scan(void);
void RC_remote_scan(void);


#endif
