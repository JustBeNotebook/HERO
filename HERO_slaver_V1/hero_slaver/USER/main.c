
#include "link.h"
#include "delay.h"
#include "usart.h"
#include "systick.h"
#include "task.h"


u8 receive[8];

int main(void)
{ 
	
	System_Init();

	Task_start();
	
	while(1)
	{
		Task_disapart();
	}
}






