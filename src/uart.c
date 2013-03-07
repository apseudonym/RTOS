// uart.c

#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/uart.h"
#include "driverlib/gpio.h"
#include "utils/uartstdio.h"
#include "uart.h"


/* param: device:   device number (0 or 1) / (TOP or BOTTOM)
 *		  line_num: line number (0 through 3)
 *		  string:   string to print to screen
 *		  value:    number to print to screen
 */

typedef struct{
	char *string; 
	long value;
} uart_message;
#define UART_MESSAGE_QUEUE_SIZE 64
uart_message uart_message_queue[UART_MESSAGE_QUEUE_SIZE];
uart_message* uart_message_head;
uart_message* uart_message_tail;
void UART_Message_Queue_Init(void){
	uart_message_head = uart_message_queue;
	uart_message_tail = uart_message_queue;
}
extern void OS_Suspend(void);
char * uart_message_pull(long *value){
	char * string;
	//while(message_head == message_tail); //busy-wait
	while(uart_message_head == uart_message_tail) OS_Suspend(); //sleep till there's something in the FIFO
	string = uart_message_tail->string;
	*value = uart_message_tail->value;
	uart_message_tail = (uart_message_tail == &uart_message_queue[UART_MESSAGE_QUEUE_SIZE]) ? &uart_message_queue[0] : &uart_message_tail[1];
	return string;
}


int UART_Message(char *string, long value) {
	if(((&uart_message_head[1] == &uart_message_queue[UART_MESSAGE_QUEUE_SIZE]) ? &uart_message_queue[0]: &uart_message_head[1]) == uart_message_tail) return 1; //FIFO full check, return 1 if full (drop message)
	else{
		uart_message_head->string = string;
		uart_message_head->value = value;
	    uart_message_head = (uart_message_head == &uart_message_queue[UART_MESSAGE_QUEUE_SIZE]) ? &uart_message_queue[0] : &uart_message_head[1];
		return 0;
	}
}

void UART_Output_Thread(void){
	char *string;
	long value;
	while(1){
		string = uart_message_pull(&value);
        if(string[0] == '\f')
            UARTprintf(string);
        else{
            UARTprintf(string);
            UARTprintf(": %d\n",value);
        }
	}
}


void UART_Init(void){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);				
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UART_Message_Queue_Init();
    UARTStdioInit(0);	
}
