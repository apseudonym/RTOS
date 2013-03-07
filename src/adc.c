//#include "inc/hw_types.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/adc.h"
#include "driverlib/interrupt.h"
#include "inc/hw_types.h"
#include "lm3s8962.h"
#include "adc.h"
#include "os.h"

unsigned short ulSensorData;
#define ADC_SAMPLE_RATE 50
// ADC module initialization.
// Write a '1' to bit 16 (ADC bit) of the RCGC0
//  register (address 0x400FE100).
void ADC_Init(int sample) {
  // Enable the clock to the ADC module
  // System Control RCGC0 register
  SYSCTL_RCGC0_R |= SYSCTL_RCGC0_ADC;

  // Configure the ADC to sample at 500KSps
  // System Control RCGC0 register
  SYSCTL_RCGC0_R |= sample;
/*
  // Enable the clock to the ADC module using 
  //  DriverLib functions.
  SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC);
*/
}

/*unsigned long ADC_In(unsigned int channelNum) {
  //
  // Retrieve data from sample sequence 1 FIFO
  //
  // ADC Sample Sequence 1 FIFO register
  //
  unsigned long ulSeq1DataBuffer;
  ADCSequenceDataGet(ADC_BASE, channelNum, &ulSeq1DataBuffer);
  return ulSeq1DataBuffer;

  switch(channelNum) {
    case 0:	ulSensorData = ADC_SSFIFO0_R; break;
    case 1:	ulSensorData = ADC_SSFIFO1_R; break;
    case 2:	ulSensorData = ADC_SSFIFO2_R; break;
    case 3: ulSensorData = ADC_SSFIFO3_R; break;
  }
  reutrn ulSensorData;
}*/

void adc_dummy(unsigned short in){};

void ADC_Trigger(void){
	ADCProcessorTrigger(ADC_BASE, 0 );
}

#define ADC_FUNCTION_BUFFER_SIZE 4
typedef struct {void(*task)(unsigned short); unsigned long value; }adc_function;
adc_function adc_functions[ADC_FUNCTION_BUFFER_SIZE];
void ADC_Collect(unsigned int channelNum, unsigned int fs, void(*task)(unsigned short)) {
	adc_functions[channelNum].task = task;		
}
void ADC_Open(void) {
	 int i;
     SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC);
	 ADCSequenceConfigure(ADC_BASE,0, ADC_TRIGGER_PROCESSOR, 0);
	 ADCSequenceStepConfigure(ADC_BASE, 0, 0, ADC_CTL_CH0);
	 ADCSequenceStepConfigure(ADC_BASE, 0, 1, ADC_CTL_CH1);
	 ADCSequenceStepConfigure(ADC_BASE, 0, 2, ADC_CTL_CH2);
	 ADCSequenceStepConfigure(ADC_BASE, 0, 3, ADC_CTL_CH3 | ADC_CTL_IE | ADC_CTL_END);
	 ADCSequenceEnable(ADC_BASE, 0);
	 ADCIntEnable(ADC_BASE, 0);
     for(i = 0; i < 4; i++){
         adc_functions[i].value = 0;
	     adc_functions[i].task = adc_dummy;
     }
     OS_AddPeriodicThread(ADC_Trigger,PERIODIC_FUNCTION_RATE/ADC_SAMPLE_RATE,0);
     IntEnable(INT_ADC0SS0);     
}  

unsigned long ADC_In(unsigned int channelNum){
	return adc_functions[channelNum].value;
}	

void ADC0_Handler(void){
	int i;
 	unsigned long ADCValues[4] = {0,};
	ADCSequenceDataGet(ADC_BASE, 0, ADCValues); 
	ADCIntClear(ADC_BASE, 0);
    for(i = 0; i < 4; i++){
        adc_functions[i].value = ADCValues[i];
	    adc_functions[i].task(ADCValues[i]);
    }
}
/*
int ADC_Collect(unsigned int channelNum, unsigned int fs, 
    unsigned short buffer[], unsigned int numberOfSamples) {
  int i = 0;
  for(i = 0; i <numberOfSamples; i++) {
    ADC_In(channelNum);
  } 
  return 0;
}	 
*/

