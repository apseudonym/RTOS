//os.c
//RTOS written for EE445M SP'13
//Rolando Gonzalez and (Frank) Yifan Weng

#include <stdio.h>
#include <stdlib.h>
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#include "os.h"
#include "Output.h"
#include "lm3s8962.h"
#include "rit128x96x4.h"

#define SEC_PER_DAY 86400
#define SEC_PER_HOUR 3600
#define	SEC_PER_MIN 60
#define MS_PER_SEC 1000
#define OS_TIME_RATE 1000000 
#define NUMTHREADS 8
#define STACKSIZE  100
#define FIFOFAIL 0
#define FIFOSUCCESS 1
#define ACTIVE 0
#define BLOCKED 1
#define NO_LL_PRESENT 0
#define LL_PRESENT 1

volatile unsigned long os_per_time;
volatile unsigned long os_time;
volatile unsigned long os_seconds;
volatile unsigned long timeslice;
long Stacks[NUMTHREADS][STACKSIZE];

void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
long StartCritical (void);    // previous I bit, disable interrupts
void EndCritical(long sr);    // restore I bit to previous value
void WaitForInterrupt(void);  // low power mode

tcbType tcbs[NUMTHREADS];
tcbType *RunPt;					// running thread
tcbType *head = 0;
tcbType *tail = 0;
static int tcbs_index = 0;		// current index in tcbs[]

// ******** StartOS ************
// Assembly code for the thread switcher.
// Program 4.10 from Embedded Systems Volume 3 (pg 171)
// output: none
__asm void StartOS(void) {
		IMPORT RunPt
		LDR     R0, =RunPt         ; currently running thread
		LDR     R2, [R0]           ; R2 = value of RunPt
		LDR     SP, [R2]           ; new thread SP; SP = RunPt->stackPointer;
		POP     {R4-R11}           ; restore regs r4-11
		POP     {R0-R3}            ; restore regs r0-3
		POP     {R12}
		POP     {LR}               ; discard LR from initial stack
		POP     {LR}               ; start location
		POP     {R1}               ; discard PSR
		CPSIE   I                  ; Enable interrupts at processor level
		BX      LR                 ; start first thread
}
 
void checkSleep(void){
	do{ 
		RunPt = RunPt->next;
		if(RunPt->sleep > 0) RunPt->sleep--;
		if(RunPt->priority > 0) RunPt->priority--;
	}while(RunPt->sleep != 0 || RunPt->priority != 0 || RunPt->status != ACTIVE);
	RunPt->priority = RunPt->base_priority; 
}

__asm void PendSV_Handler(void) { // 1) Saves R0-R3,R12,LR,PC,PSR
  PRESERVE8
  IMPORT checkSleep
  CPSID   I                  ; 2) Prevent interrupt during switch
  PUSH    {R4-R11}           ; 3) Save remaining regs r4-11
  LDR     R0, =RunPt         ; 4) R0=pointer to RunPt, old thread
  LDR     R1, [R0]           ;    R1 = RunPt
  STR     SP, [R1]           ; 5) Save SP into TCB
  BL      checkSleep
  LDR     R0, =RunPt         ; 6) R0=pointer to RunPt, new thread
  LDR     R1, [R0]           ;    R1 = RunPt
  LDR     SP, [R1]           ; 7) new thread SP; SP = RunPt->sp;
  POP     {R4-R11}           ; 8) restore regs r4-11
  LDR     LR, =0xFFFFFFF9
  CPSIE   I                  ; 9) tasks run with interrupts enabled
  BX      LR                 ; 10) restore R0-R3,R12,LR,PC
} 	 

void SysTick_Handler(void) {
  NVIC_INT_CTRL_R = NVIC_INT_CTRL_PEND_SV; //Trigger PendSV (Thread Switch)
} 
  
// ******** SetInitialStack ************
// Function from Lab2.c
// ISR pushes R0-R3, R12, LR, PC, PSR
void SetInitialStack(int i){
  tcbs[i].sp = &Stacks[i][STACKSIZE-16]; // thread stack pointer
  Stacks[i][STACKSIZE-1]  = 0x01000000;   // thumb bit
  Stacks[i][STACKSIZE-3]  = 0x14141414;   // R14
  Stacks[i][STACKSIZE-4]  = 0x12121212;   // R12
  Stacks[i][STACKSIZE-5]  = 0x03030303;   // R3
  Stacks[i][STACKSIZE-6]  = 0x02020202;   // R2
  Stacks[i][STACKSIZE-7]  = 0x01010101;   // R1
  Stacks[i][STACKSIZE-8]  = 0x00000000;   // R0
  Stacks[i][STACKSIZE-9]  = 0x11111111;   // R11
  Stacks[i][STACKSIZE-10] = 0x10101010;  // R10
  Stacks[i][STACKSIZE-11] = 0x09090909;  // R9
  Stacks[i][STACKSIZE-12] = 0x08080808;  // R8
  Stacks[i][STACKSIZE-13] = 0x07070707;  // R7
  Stacks[i][STACKSIZE-14] = 0x06060606;  // R6
  Stacks[i][STACKSIZE-15] = 0x05050505;  // R5
  Stacks[i][STACKSIZE-16] = 0x04040404;  // R4
}

// ******** OS_InitSemaphore ************
// initialize semaphore 
// input:  pointer to a semaphore
// output: none
void OS_InitSemaphore(Sema4Type *semaPt, long value){
  semaPt->value = value;
}

//******** dummy *************** 
// does nothing
// Inputs: none
// Outputs: none
void dummy(void){}
	
// ******** OS_Wait ************
// decrement semaphore and spin/block if less than zero
// input:  pointer to a counting semaphore
// output: none
void OS_Wait(Sema4Type *semaPt){
	long status;
	status = StartCritical();
		semaPt->value = semaPt->value - 1;
	if(semaPt->value < 0){
		RunPt->status = BLOCKED;
	  if(semaPt->blockedCheck == NO_LL_PRESENT){ // no tcbs in LL 
			semaPt->blockedCheck = LL_PRESENT;
			semaPt->head = RunPt;
			semaPt->head->blockedNext = RunPt;
			semaPt->tail = head;
		} else if(semaPt->blockedSize == 1) {  // only one tcb in LL
			// head has a higher priority than RunPt
		  if(RunPt->priority > semaPt->head->priority){
				semaPt->head->blockedNext = RunPt;
		    semaPt->tail = RunPt;
		  } else {  // RunPt has a higher priority than head
				RunPt->blockedNext = semaPt->head;
				semaPt->tail = semaPt->head;
				semaPt->head = RunPt;
			}
		} else {  // two or more tcbs in LL
			int i = 0;
			tcbType *indexPt = semaPt->head;
			if(RunPt->priority < indexPt->priority){
				RunPt->blockedNext = semaPt->head;
				semaPt->head = RunPt;
			} else if(RunPt->priority >= semaPt->tail->priority){
				semaPt->tail->blockedNext = RunPt;
				semaPt->tail = RunPt;
			} else {
			  while(i < semaPt->blockedSize-1){
				  if((RunPt->priority >= indexPt->priority) && (RunPt->priority < indexPt->blockedNext->priority)){
						RunPt->blockedNext = indexPt->blockedNext;
						indexPt->blockedNext = RunPt;
						i = semaPt->blockedSize-2;
					}
			    i++;
			  }
		  }
		}
		semaPt->blockedSize++;
		OS_Suspend();
	}
	EndCritical(status);
}

// ******** OS_Signal ************
// increment semaphore, wakeup blocked thread if appropriate 
// input:  pointer to a counting semaphore
// output: none
void OS_Signal(Sema4Type *semaPt){
	long status;
	struct tcb* awakePt = semaPt->head;
	status = StartCritical();
	semaPt->value = semaPt->value + 1;
	if(semaPt->value <= 0) {
		// round-robin implementation
		// TODO:
		//   suspend running thread if priority is less
		semaPt->blockedSize--;
		semaPt->head->status = ACTIVE;
	  if(semaPt->blockedSize == 0) {
			semaPt->head = 0;
			semaPt->tail = 0;
			semaPt->blockedCheck = NO_LL_PRESENT;
		} else {
			semaPt->head = semaPt->head->blockedNext;
		}
		if(RunPt->priority > awakePt->priority){
		  OS_Suspend();
	  }
	}
	EndCritical(status);
}

// ******** OS_bWait ************
// if the semaphore is 0 then spin/block
// if the semaphore is 1, then clear semaphore to 0
// input:  pointer to a binary semaphore
// output: none
void OS_bWait(Sema4Type *semaPt){
	DisableInterrupts();
	if(semaPt->value == 0) {
		RunPt->status = BLOCKED;
	  if(semaPt->blockedCheck == NO_LL_PRESENT){ // no tcbs in LL 
			semaPt->blockedCheck = LL_PRESENT;
			semaPt->head = RunPt;
			semaPt->head->blockedNext = RunPt;
			semaPt->tail = head;
		} else if(semaPt->blockedSize == 1) {  // only one tcb in LL
			// head has a higher priority than RunPt
		  if(RunPt->priority > semaPt->head->priority){
				semaPt->head->blockedNext = RunPt;
		    semaPt->tail = RunPt;
		  } else {  // RunPt has a higher priority than head
				RunPt->blockedNext = semaPt->head;
				semaPt->tail = semaPt->head;
				semaPt->head = RunPt;
			}
		} else {  // two or more tcbs in LL
			int i = 0;
			tcbType *indexPt = semaPt->head;
			if(RunPt->priority < indexPt->priority){
				RunPt->blockedNext = semaPt->head;
				semaPt->head = RunPt;
			} else if(RunPt->priority >= semaPt->tail->priority){
				semaPt->tail->blockedNext = RunPt;
				semaPt->tail = RunPt;
			} else {
			  while(i < semaPt->blockedSize-1){
				  if((RunPt->priority >= indexPt->priority) && (RunPt->priority < indexPt->blockedNext->priority)){
						RunPt->blockedNext = indexPt->blockedNext;
						indexPt->blockedNext = RunPt;
						i = semaPt->blockedSize-2;
					}
			    i++;
			  }
		  }
		}
		semaPt->blockedSize++;
	}
	semaPt->value = 0;
	EnableInterrupts();
}

// ******** OS_bSignal ************
// set semaphore to 1, wakeup blocked thread if appropriate 
// input:  pointer to a binary semaphore
// output: none
void OS_bSignal(Sema4Type *semaPt){
	long status;
	struct tcb* awakePt = semaPt->head;
	status = StartCritical();
	semaPt->value = 1;
	// round-robin implementation
	// TODO: 
	//   suspend running thread if priority is less
	semaPt->blockedSize--;
	semaPt->head->status = ACTIVE;
	if(semaPt->blockedSize == 0) {
		semaPt->head = 0;
		semaPt->tail = 0;
		semaPt->blockedCheck = NO_LL_PRESENT;
	} else {
		semaPt->head = semaPt->head->blockedNext;
	}
	// change this (maybe)
	if(RunPt->priority > awakePt->priority){
		OS_Suspend();
	}
	EndCritical(status);
}

//******** OS_Id *************** 
// returns the thread ID for the currently running thread
// Inputs:  none
// Outputs: Thread ID, number greater than zero 
unsigned long OS_Id(void) {
	static unsigned long i = 0;
	return i++;
}

// ******** OS_Suspend ************
// suspend execution of currently running thread
// scheduler will choose another thread to execute
// Can be used to implement cooperative multitasking 
// Same function as OS_Sleep(0)
// input:  none
// output: none
void OS_Suspend(void) {
	NVIC_ST_CURRENT_R = 0;				// clear counter
	NVIC_INT_CTRL_R = 0x04000000; // trigger SysTick
}

//********Get_OS_Time *************** 
// Returns the current time.
// Inputs:  none
// Outputs: seconds
int Get_OS_Time(void){
	return os_time;
}

//******** OS_AddThread *************** 
// add a foregound thread to the scheduler
// Inputs: pointer to a void/void foreground task
//         number of bytes allocated for its stack
//         priority (0 is highest)
// Outputs: 1 if successful, 0 if this thread can not be added
// stack size must be divisable by 8 (aligned to double word boundary)
// Currently holds 3 threads.
// In Lab 2, you can ignore both the stackSize and priority fields
// In Lab 3, you can ignore the stackSize fields
int OS_AddThread(void(*task)(void), 
    unsigned long stackSize, unsigned long priority) {
	long status;
	int i;
	status = StartCritical();

	if( tcbs_index == 0 ) {
		head = &tcbs[0];
		tail = &tcbs[0]; 
		tcbs[0].next = &tcbs[0];
		tcbs[0].prev = &tcbs[0];
		RunPt = &tcbs[0]; 		// initial running thread
	}
	   for(i = 0; i < tcbs_index; i++){
				if(tcbs[i].sleep  == -1){ //
						tail->next = &tcbs[i];
						tcbs[i].prev = tail;
						tcbs[i].next = head;
						head->prev = &(tcbs[i]);
					  tail = &tcbs[i];
					  SetInitialStack(i);
						Stacks[i][STACKSIZE-2] = (long)(task);
						tcbs[i].sp = &Stacks[i][STACKSIZE-16]; // R4
						tcbs[i].id = OS_Id();
						tcbs[i].sleep = 0;
					  tcbs[i].priority = tcbs[i].base_priority = priority;
					  EndCritical(status);
					  return 1;
				}
			}
	  if (tcbs_index >= NUMTHREADS){
				EndCritical(status);
				return 0;
		}
		tcbs[tcbs_index].next = head;
		tail->next = &tcbs[tcbs_index];
		head->prev = &(tcbs[tcbs_index]);
		tcbs[tcbs_index].prev = tail;
		SetInitialStack(tcbs_index);
	  Stacks[tcbs_index][STACKSIZE-2] = (long)(task);
	  tcbs[tcbs_index].sp = &Stacks[tcbs_index][STACKSIZE-16]; // R4
	  tcbs[tcbs_index].id = OS_Id();	
	  tcbs[tcbs_index].sleep = 0;
		tail = &tcbs[tcbs_index];
		tcbs[tcbs_index].status = ACTIVE;
		tcbs[tcbs_index].priority = tcbs[tcbs_index].base_priority = priority;
	  tcbs_index++;
		EndCritical(status);
	  return 1;			// successful
}

//******** OS_AddThreads *************** 
//  from Lab2.c
// Add new thread
// Inputs:  three pointers to functions
// Outputs: 1 for success confirmation
int OS_AddThreads(void(*task0)(void), void(*task1)(void), void(*task2)(void)){
	long status;
	status = StartCritical();
	tcbs[0].next = &tcbs[1];
	tcbs[1].next = &tcbs[2];
	tcbs[2].next = &tcbs[0];
	// set the initial stacks or whatever
	SetInitialStack(0);
	Stacks[0][STACKSIZE-2] = (long)(task0);
	SetInitialStack(1);
	Stacks[1][STACKSIZE-2] = (long)(task1);
	SetInitialStack(2);
	Stacks[2][STACKSIZE-2] = (long)(task2);
	EndCritical(status);
	return 1; 								// successful
}

typedef struct{
	void(*task)(void); 
    unsigned long period; 
	unsigned long priority;
}funct;

#define PERIODIC_FUNCTION_BUFFER_SIZE 10
volatile funct per_functions[PERIODIC_FUNCTION_BUFFER_SIZE];

//******** Periodic_Function_Controller_Init *************** 
// initialize the task, period, and priority of a new
//   thread
// Inputs: none
// Outputs: none
void Periodic_Function_Controller_Init(void){
	int i;
	for( i = 0; i < PERIODIC_FUNCTION_BUFFER_SIZE; i++){
		per_functions[i].task = dummy;
		per_functions[i].period = 0;
		per_functions[i].priority = 0;
	}
}

//******** OS_AddPeriodicThread *************** 
// Adds a new thread after a certain amount of time
// Inputs: pointer to a function, length of period,
//   thread priority
// Outputs: none
int OS_AddPeriodicThread(void(*task_in)(void), unsigned long period_in, 
    unsigned long priority_in){
	int i = 0;
	while(per_functions[i].period != 0){
		i++;
		if(i == PERIODIC_FUNCTION_BUFFER_SIZE)
			return 1;
	}
	per_functions[i].task = task_in;
	per_functions[i].period = period_in;
	per_functions[i].priority = priority_in;
	return 0;
}

//******** OS_AddButtonTask *************** 
// add a background task to run whenever the Select button is pushed
// Inputs: pointer to a void/void background function
//         priority 0 is highest, 5 is lowest
// Outputs: 1 if successful, 0 if this thread can not be added
// It is assumed that the user task will run to completion and return
// This task can not spin, block, loop, sleep, or kill
// This task can call OS_Signal  OS_bSignal	 OS_AddThread
// This task does not have a Thread ID
// In labs 2 and 3, this command will be called 0 or 1 times
// In lab 2, the priority field can be ignored
// In lab 3, there will be up to four background threads, and this priority field 
//           determines the relative priority of these four threads
typedef struct{
	void(*task)(void); 
	long priority;
}button_task;
#define BUTTON_TASK_BUFFER_SIZE	2
volatile button_task button_tasks[BUTTON_TASK_BUFFER_SIZE];

void GPIOPortF_Handler(void){
	int i = 0;
	IntMasterDisable();
	while((button_tasks[i].priority != -1) && (i < BUTTON_TASK_BUFFER_SIZE)){
		button_tasks[i].task();
		i++;
	}
	GPIOPinIntClear(GPIO_PORTF_BASE, GPIOPinIntStatus(GPIO_PORTF_BASE, false));
	IntMasterEnable();
}
int OS_AddButtonTask(void(*task_in)(void), unsigned long priority_in) {
	static int initialized = 0;
	int i = 0;
	if(!initialized){
	 	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
		GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_1);
		GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_1 , GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
		IntEnable(INT_GPIOF);
		GPIOIntTypeSet(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_FALLING_EDGE);
		GPIOPinIntEnable(GPIO_PORTF_BASE, GPIO_PIN_1);
		for( i = 0; i < BUTTON_TASK_BUFFER_SIZE; i++){
			button_tasks[i].task = dummy;
			button_tasks[i].priority = -1;
		}i=0;
		initialized = 1;
	}
	while(button_tasks[i].priority != -1){
		i++;
		if(i == BUTTON_TASK_BUFFER_SIZE)
			return 1;
	}
	button_tasks[i].task = task_in;
	button_tasks[i].priority = priority_in;
	return 0;
}

volatile button_task button_tasks_down[BUTTON_TASK_BUFFER_SIZE];
void GPIOPortE_Handler(void){
	int i = 0;
	IntMasterDisable();
	while((button_tasks_down[i].priority != -1) && (i < BUTTON_TASK_BUFFER_SIZE)){
		button_tasks_down[i].task();
		i++;
	}
	GPIOPinIntClear(GPIO_PORTE_BASE, GPIOPinIntStatus(GPIO_PORTE_BASE, false));
	IntMasterEnable();
}
int OS_AddDownButtonTask(void(*task_in)(void), unsigned long priority_in) {
	static int initialized = 0;
	int i = 0;
	if(!initialized){
	 	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
		GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_1);
		GPIOPadConfigSet(GPIO_PORTE_BASE, GPIO_PIN_1 , GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
		IntEnable(INT_GPIOE);
		GPIOIntTypeSet(GPIO_PORTE_BASE, GPIO_PIN_1, GPIO_FALLING_EDGE);
		GPIOPinIntEnable(GPIO_PORTE_BASE, GPIO_PIN_1);
		for( i = 0; i < BUTTON_TASK_BUFFER_SIZE; i++){
			button_tasks_down[i].task = dummy;
			button_tasks_down[i].priority = -1;
		}i=0;
		initialized = 1;
	}
	while(button_tasks_down[i].priority != -1){
		i++;
		if(i == BUTTON_TASK_BUFFER_SIZE)
			return 1;
	}
	button_tasks_down[i].task = task_in;
	button_tasks_down[i].priority = priority_in;
	return 0;
}

void OS_Sleep(unsigned long sleepTime){
	RunPt->sleep = sleepTime;
	OS_Suspend();
}
///******** OS_Launch *************** 
// start the scheduler, enable interrupts
// Inputs: number of 20ns clock cycles for each time slice
//         (maximum of 24 bits) because its a 24-bit timer
// Outputs: none (does not return)
// In Lab 2, you can ignore the theTimeSlice field
// In Lab 3, you should implement the user-defined TimeSlice field

void OS_Launch(unsigned long theTimeSlice){
	//
  // Set up the period for the SysTick timer.  The SysTick timer period will
  // be equal to the system clock, resulting in a period of 1 second.
  //
  SysTickPeriodSet(theTimeSlice);
  //
  // Enable the SysTick Interrupt.
  //
  SysTickIntEnable();
  //
  // Enable SysTick.
  //
  SysTickEnable();
  StartOS();                   // start on the first task
}

//******** OS_Time_Init *************** 
// Inputs: none
// Outputs: none
void OS_Time_Init(void){
	os_per_time = 0;
	os_time = 0;
	os_seconds = 0;
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);     // enable Timer 0
  	TimerConfigure(TIMER0_BASE, 						// configure two 16-bit
		TIMER_CFG_16_BIT_PAIR | TIMER_CFG_A_PERIODIC | TIMER_CFG_B_PERIODIC );  //   periodic timers
	
	IntEnable(INT_TIMER0B);
	IntEnable(INT_TIMER0A);
	TimerIntEnable(TIMER0_BASE, TIMER_TIMB_TIMEOUT );
	TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT );
	TimerLoadSet(TIMER0_BASE, TIMER_B, SysCtlClockGet() / OS_TIME_RATE);
	TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet() / PERIODIC_FUNCTION_RATE);
	TimerEnable(TIMER0_BASE, TIMER_BOTH);
}
// ******** OS_Kill ************
// kill the currently running thread, release its TCB memory
// input:  none
// output: none
void OS_Kill(void) {
	IntMasterDisable();
	RunPt->prev->next = RunPt->next;
	RunPt->next->prev = RunPt->prev;
	if(RunPt == head) head = head->next;
	if(RunPt == tail) tail = tail->prev;
	RunPt->sleep = -1;
	IntMasterEnable();
	OS_Suspend(); 
}

//******** OS_Init *************** 
// For testing 
// Original: MCU_Init()
// RTOS initialization 
void OS_Init(void) {
	DisableInterrupts();		// set processor to 50MHz
	SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                   SYSCTL_XTAL_8MHZ);
	Periodic_Function_Controller_Init(); 
	OS_Time_Init();
}

//******** Timer0A_Handler *************** 
// ISR for Timer0B
// Inputs: none
// Outputs: none
unsigned long MaxJitter = 0;
unsigned long MinJitter = 0x7FFFFFFF;
void Timer0A_Handler(void){
	static unsigned long lastTime = 0;
	int t0AIndex=0;
	TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
  os_per_time++;
	while( t0AIndex != PERIODIC_FUNCTION_BUFFER_SIZE && per_functions[t0AIndex].period != 0){
		if(os_per_time%per_functions[t0AIndex].period == 0){
			per_functions[t0AIndex].task();
			if( t0AIndex == 0){
					int currentTimeDif = os_time - lastTime;
				  if (currentTimeDif > MaxJitter) MaxJitter = currentTimeDif;
					if (currentTimeDif < MinJitter) MinJitter = currentTimeDif;
					lastTime = os_time;
			}
		}
		t0AIndex++;	
	}
	//GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_3, GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_3) ^ GPIO_PIN_3);
}
unsigned long Jitter(void){return MaxJitter-MinJitter; }


//******** Timer0B_Handler *************** 
// ISR for Timer0B
// Inputs: none
// Outputs: none
void Timer0B_Handler(void){
	TimerIntClear(TIMER0_BASE, TIMER_TIMB_TIMEOUT);
	os_time++;
}

unsigned long FIFOSIZE;
unsigned long volatile *PutPt;
unsigned long volatile *GetPt;
unsigned long *Fifo;
Sema4Type CurrentSize;
Sema4Type RoomLeft;
Sema4Type mutex;

// ******** OS_Fifo_Init ************
// Initialize the Fifo to be empty
// Inputs: size
// Outputs: none 
// In Lab 2, you can ignore the size field
// In Lab 3, you should implement the user-defined fifo size
// In Lab 3, you can put whatever restrictions you want on size
//    e.g., 4 to 64 elements
//    e.g., must be a power of 2,4,8,16,32,64,128
void OS_Fifo_Init(unsigned long size){
	FIFOSIZE = size;
	Fifo = (unsigned long *)malloc(size*sizeof(unsigned long));
	CurrentSize.value = 0;
	RoomLeft.value = FIFOSIZE;
	mutex.value = 1;
}

// ******** OS_Fifo_Put ************
// Enter one data sample into the Fifo
// Called from the background, so no waiting 
// Inputs:  data
// Outputs: true if data is properly saved,
//          false if data not saved, because it was full
// Since this is called by interrupt handlers 
//  this function can not disable or enable interrupts
int OS_Fifo_Put(unsigned long data){
	unsigned long volatile *nextPutPt;
	nextPutPt = &PutPt[1];
	if(nextPutPt == &Fifo[FIFOSIZE]) {
		nextPutPt = Fifo;
	}
	if(nextPutPt == GetPt) {
		return(FIFOFAIL);
	} else {
		*(PutPt) = data;
		PutPt = nextPutPt;
		OS_Signal(&CurrentSize);
		return(FIFOSUCCESS);
	}
}  
unsigned long *datapt;
// ******** OS_Fifo_Get ************
// Remove one data sample from the Fifo
// Called in foreground, will spin/block if empty
// Inputs:  none
// Outputs: data 
unsigned long OS_Fifo_Get(void){
	OS_Wait(&CurrentSize);
	OS_Wait(&mutex);
	*datapt = *(GetPt++);		// not thread safe
	if(GetPt == &Fifo[FIFOSIZE]){
		GetPt = &Fifo[0];
	}
	OS_Signal(&mutex);
	return *datapt;
}

// ******** OS_Fifo_Size ************
// Check the status of the Fifo
// Inputs: none
// Outputs: returns the number of elements in the Fifo
//          greater than zero if a call to OS_Fifo_Get will return right away
//          zero or less than zero if the Fifo is empty 
//          zero or less than zero  if a call to OS_Fifo_Get will spin or block
long OS_Fifo_Size(void){
	return CurrentSize.value;
}

unsigned long Mail;
Sema4Type Mailbox;
Sema4Type Send;
Sema4Type Ack;

// ******** OS_MailBox_Init ************
// Initialize communication channel
// Inputs:  none
// Outputs: none
void OS_MailBox_Init(void){
	Send.value = 0;
	Ack.value = 0;
}

// ******** OS_MailBox_Send ************
// enter mail into the MailBox
// Inputs:  data to be sent
// Outputs: none
// This function will be called from a foreground thread
// It will spin/block if the MailBox contains data not yet received 
void OS_MailBox_Send(unsigned long data){
	Mail = data;
	OS_Signal(&Send);
	OS_Wait(&Ack);
}

// ******** OS_MailBox_Recv ************
// remove mail from the MailBox
// Inputs:  none
// Outputs: data received
// This function will be called from a foreground thread
// It will spin/block if the MailBox is empty 
unsigned long OS_MailBox_Recv(void){
	unsigned long data;			// probably not thread safe
	OS_Wait(&Send);
	data = Mail;
	OS_Signal(&Ack);
	return data;
}

// ******** OS_Time ************
// reads a timer value 
// Inputs:  none
// Outputs: time in 20ns units, 0 to max
// The time resolution should be at least 1us, and the precision at least 12 bits
// It is ok to change the resolution and precision of this function as long as 
//   this function and OS_TimeDifference have the same resolution and precision 
unsigned long OS_Time(void){
	return os_time;
}

// ******** OS_TimeDifference ************
// Calculates difference between two times
// Inputs:  two times measured with OS_Time
// Outputs: time difference in 20ns units 
// The time resolution should be at least 1us, and the precision at least 12 bits
// It is ok to change the resolution and precision of this function as long as 
//   this function and OS_Time have the same resolution and precision 
unsigned long OS_TimeDifference(unsigned long start, unsigned long stop){
	return stop > start ? stop-start : start-stop; // false is wrong, DGAF
}

// ******** OS_ClearMsTime ************
// sets the system time to zero from Lab 1)
// Inputs:  none
// Outputs: none
// You are free to change how this works
void OS_ClearMsTime(void){}

// ******** OS_MsTime ************
// reads the current time in msec (from Lab 1)
// Inputs:  none
// Outputs: time in ms units
// You are free to select the time resolution for this function
unsigned long OS_MsTime(void){return os_time*MS_PER_SEC/OS_TIME_RATE;}
