// ******** ADC_Init ************
// initialize the ADC with a specified sample
// input:  sample rate
// output: none
void ADC_Init(int sample);

// ******** ADC_Open ************
// Alternative ADC initialization.
// input:  none
// output: none
void ADC_Open(void); 
// ******** ADC_In ************
// Take a specified amount of samples from ADC.
// input:  sample rate
// output: none
unsigned long ADC_In(unsigned int channelNum); 
// ******** ADC_Collect ************
// Take a specified amount of samples from ADC at a certain 
//   channel and/or sampling frequency.
// input:  channel, sample frequency, data buffer, sample counter
// output: none
void ADC_Collect(unsigned int channelNum, unsigned int fs, 
		void(*task)(unsigned short));
//int ADC_Collect(unsigned int channelNum, unsigned int fs, unsigned short buffer[], unsigned int numberOfSamples); 
