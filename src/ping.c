//ping.c

#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "ping.h"


void WaitUS(unsigned long us)
{							
	unsigned long ticks;
	static tBoolean initialized = false;
	if(!initialized)
	{
		SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
		TimerConfigure(TIMER1_BASE, (TIMER_CFG_32_BIT_OS));
		initialized = true;	
	}
	ticks = us*(SysCtlClockGet()/1000000);
	TimerLoadSet(TIMER1_BASE, TIMER_A, ticks);	
	TimerEnable(TIMER1_BASE, TIMER_A);		 
	while(TimerValueGet(TIMER1_BASE, TIMER_A) != ticks);		  
	TimerDisable(TIMER1_BASE, TIMER_BOTH);
}

void Ping(void){ //run this as a periodic function
    
}
#define PING_TIMEOUT 10000
unsigned long PingSingle(void){ //PA6
    static tBoolean initialized = false;
    unsigned long output = 0;
    unsigned long timeout = 0;
    if(!initialized){
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
        initialized = true;
    }
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE,GPIO_PIN_6);
    GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_6,GPIO_PIN_6);
    WaitUS(15);
    GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_6,0);
    GPIOPinTypeGPIOInput(GPIO_PORTA_BASE,GPIO_PIN_6);
    WaitUS(500);
    while((GPIOPinRead(GPIO_PORTA_BASE,GPIO_PIN_6) == 0) && (timeout < PING_TIMEOUT)) timeout++;
    while((GPIOPinRead(GPIO_PORTA_BASE,GPIO_PIN_6) != 0) && (output < PING_TIMEOUT)) output++;
    return output;
}

