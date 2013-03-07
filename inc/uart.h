//uart.h
void UART_Init(void);
void UART_Message_Queue_Init(void);
int UART_Message(char *string, long value);
void UART_Output_Thread(void);
