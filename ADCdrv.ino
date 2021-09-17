//
// This file contails ADC routines that support reading the ARM ADC channels. Low level
// functions allow you to set the channel gain and read raw data from a channel. Additionally
// there are two higher level functions to support high speed data acquisition and ADC
// value change detection.
//
// High speed acquisition
//   This mode acquires one of more vectors and streams the data out the USB port.
//   The vector acquire can be triggered by software or an external trigger using the
//   delay trigger MIPS ca[ability. The acquire function blocks during the vector acquire and
//   you need to be running the MIPS host app to record the data. This fuction will 
//   support speeds up to 600KHz and long vectors, 100K points or more.
//
// Change detection
//   This mode uses the ADC window function to signal an acquire if the ADC value 
//   is outside of a programmed window. The window is repositioned around the ADC value
//   every time its read. The uses can define the window width in ADC counts. The user
//   can also define a function to call when a change is detected.
//
// Note: Arduino DUE adc channel 0 thru 7 are channels 7 thru 0 on the ARM processor!
//
// Gordon Anderson
//    - Added gain control and change detection, Jan 26, 2020
//

uint16_t  *ADCbuffer = NULL;  // Pointer to ADC buffer used to hold the raw data.
MIPStimer *ADCclock = NULL;   // Timer used to set the digitization rate

#define      SMPS 32

bool         ADCacquire     = true;   // Sets the ADC mode, true if in acquire mode and false if in 
                                      //change detect mode
volatile int bufn,obufn;
volatile int ADCchannel     = 0;
volatile int ADCnumsamples  = 10000;
volatile int ADCrate        = 200000;
volatile int ADCvectors     = 1;
volatile int Threshold      = 4;
volatile int WindowCount    = 4;
volatile int SameCount      = 6;
uint16_t     ADCvectorNum   = 0;
uint32_t     ADCsamples     = 0;
uint32_t     ADCsamplesSent = 0;
uint8_t      ADCheader[6]   = {0x55,0x55,0x55,0x55,0xAA,0};
uint8_t      ADCtrailer[6]  = {0xAE,0xAE,0xAE,0xAE,0xEA,0};

static bool ADCinuse = false;
static bool ADCready = false;

void  (*ADCchangeFunc)(int) = NULL;
float ADCchangeGain = 1.0;
float ADCvalue;
bool  ADCchanged=false;

// These values are used for the ADC averaging mode. In this mode the ADC is read and sumed
// the number of times defined by ADCnumsamples. The change detection mode must be enabled
// for this function to operate. The value saved in ADCcount is the result and its the ADC
// value sumed ADCnumsamples times. The reporting function will perform the division.
uint32_t   ADCcount;
uint32_t   ADCsum=0;
uint32_t   ADCsampleNum = 0;

bool AcquireADC(void)
{
  if(ADCinuse) return false;
  ADCinuse = true;
  return true;
}

void ReleaseADC(void)
{
  ADCinuse = false;
}

// ADC change interrupt call back function
void ADCattachInterrupt(void (*isr)(int))
{
  ADCchangeFunc = isr;
}

// This interrupt fires after the first block of samples are collected in
// the ADC acquire mode or fires when a value is converted in the window
// change detect mode.
void ADC_Handler(void)
{
  int f = ADC->ADC_ISR;
  int ll,hl;
  int val;
  int static count;
  unsigned int static countatvalue;

  if(!ADCacquire) 
  {
    if ((f & ADC_IER_DRDY) == ADC_IER_DRDY) 
    {
      val=adc_get_latest_value(ADC); 
      if(ADCnumsamples <= 1) ADCcount = val;
      else
      {
        ADCsum+=val;
        if(++ADCsampleNum > ADCnumsamples)
        {
          ADCcount = ADCsum;
          ADCsampleNum = ADCsum = 0;
        }
      }
    }
    if ((f & ADC_IER_COMPE) == ADC_IER_COMPE)
    {
      if(++count >= WindowCount)
      {
        countatvalue = count = 0;
        //val=adc_get_latest_value(ADC);
        //serial->println(val);
        ll = val - Threshold;
        if(ll < 0) ll = 0;
        hl = val + Threshold;
        if(hl > 4095) hl = 4095;
        adc_set_comparison_window(ADC,ll,hl);
        //if(ADCchangeFunc != NULL) ADCchangeFunc(val);
      }
    }
    else { count = 0; countatvalue++; }
    if(countatvalue == SameCount) if(ADCchangeFunc != NULL) ADCchangeFunc(val);
    f = ADC->ADC_ISR;
    return;
  }
  if (f & ADC_IER_ENDRX)
  {
    bufn = (bufn+1) & 3;
    if(ADCsamples > SMPS)
    {
      ADC->ADC_RNPR = (uint32_t) &ADCbuffer[bufn * SMPS];
      ADC->ADC_RNCR = SMPS;
      ADCsamples -= SMPS;
    }
    else if(ADCsamples == 0)
    {
      // Stop the clock, vector is collected
      ADCclock->stop();
      NVIC_DisableIRQ(ADC_IRQn);
    }
    else
    {
//    ADC->ADC_RNPR = (uint32_t) &ADCbuffer[bufn * ADCsamples];
      ADC->ADC_RNPR = (uint32_t) &ADCbuffer[bufn * SMPS];
      ADC->ADC_RNCR = ADCsamples;
      ADCsamples = 0;
    }
  }
}

// This function will setup the ADC to record a vector or a number of vectors and
// send the resulting data to the host.
// Data sent to host first sends a header with start signature and vector number
// as well as last vector flag. The buffer also has a trailer to flag the end.
// Header:  0x55,0x55,0x55,0x55,0xAA
//          (24 bit binary vector size)
//          (16 bit binary vector number, 0 to n-1)
//          (8 bin last vector flag) = 0xFF on last vector
// Data:    (16 bit words for each value)
// Trailer: 0xAE,0xAE,0xAE,0xAE,0xEA
// Returns an error code number on error else 0 if ok
int ADCsetup(void)
{
  if(ADCready) return(ERR_ADCALREARYSETUP);
  if(!AcquireADC()) return(ERR_ADCNOTAVALIABLE);
  ADCacquire = true;
  // Allocate the buffers, four sets of sample buffers
  if(ADCbuffer == NULL) ADCbuffer = new uint16_t [SMPS * 4];
  if(ADCbuffer == NULL) return(ERR_CANTALLOCATE);
  // Setup the timer used to set the ADC trigger rate
  if(ADCclock == NULL) ADCclock = new MIPStimer(TMR_ADCclock);
  if(ADCclock == NULL) 
  {
    delete[] ADCbuffer;
    ADCbuffer = NULL;
    return(ERR_CANTALLOCATE);
  }
  ADCready = true;
  ADCvectorNum = 0;
  analogRead(ADC8);  // This sets the last read ADC flag in the arduino driver to 8
  ADCclock->begin();
  ADCclock->setClock(TC_CMR_TCCLKS_TIMER_CLOCK1);
  ADCclock->setRC((VARIANT_MCK/2) / ADCrate);    // Sets the frequency
  ADCclock->setTIOAeffect(((VARIANT_MCK/2) / ADCrate) - 1,TC_CMR_ACPA_TOGGLE | TC_CMR_ACPC_TOGGLE | TC_CMR_AEEVT_TOGGLE);
  ADCclock->enableTrigger();
  ADCclock->stop();
  // Setup the ADC
  adc_set_writeprotect(ADC, 1);  // Enable the write registers
  pmc_enable_periph_clk(ID_ADC);
  adc_init(ADC,SystemCoreClock, ADC_FREQ_MAX, ADC_STARTUP_FAST);
  adc_configure_trigger(ADC, ADC_TRIG_SW, 0); // Disable hardware trigger.
  adc_disable_interrupt(ADC, 0xFFFFFFFF); // Disable all ADC interrupts.
  adc_disable_all_channel(ADC);
  adc_set_writeprotect(ADC, 1);  // Enable the write registers
  ADC->ADC_MR |= ADC_MR_TRGSEL_ADC_TRIG2 | ADC_MR_TRGEN_EN;
  ADC->ADC_CHER = 1 << (7 - ADCchannel);     // Select the channel
  ADC->ADC_IDR = ~ADC_IER_ENDRX;
  ADC->ADC_IER = ADC_IER_ENDRX;
  ADC->ADC_IMR = ADC_IER_ENDRX;
  ADC->ADC_RPR = (uint32_t) &ADCbuffer[0];
  ADC->ADC_RCR = SMPS;
  ADC->ADC_RNPR = (uint32_t) &ADCbuffer[SMPS];
  ADC->ADC_RNCR = SMPS;
  bufn = obufn = 1;
  ADC->ADC_PTCR = 1;
  NVIC_ClearPendingIRQ(ADC_IRQn);
  NVIC_EnableIRQ(ADC_IRQn);
  return(0);
}

// Trigger function to start the ADC vector collection. 
// Called from the command processor for software trigger
// or called by input change interrupt. This function
// blocks until the vector is sent to host.
void ADCtrigger(void)
{
  if(!ADCready) return;
  ADCsamplesSent = ADCsamples = ADCnumsamples;
  ADCclock->softwareTrigger();   // Start the collection;
  ADCvectorNum++;
  // Send the header to the host
  serial->write((const char *)ADCheader);
  serial->write((byte)(ADCsamples & 0xFF));
  serial->write((byte)((ADCsamples >> 8) & 0xFF));
  serial->write((byte)((ADCsamples >> 16) & 0xFF));
  serial->write((byte)(ADCvectorNum & 0xFF));
  serial->write((byte)((ADCvectorNum >> 8) & 0xFF));
  // If the current vector number matches the number of vectors then flag it 
  // as last vector
  if(ADCvectorNum >= ADCvectors) serial->write(0xFF);
  else serial->write((byte)0);
  // Send all the data to host as the buffers fill
  while(1)
  {
    while(obufn==bufn); // wait for buffer to fill
    obufn = (obufn - 1) & 3; 
    if(ADCsamplesSent > SMPS)
    {
       serial->write((uint8_t *)&ADCbuffer[obufn * SMPS],SMPS*2); // send filled buffer
       ADCsamplesSent -= SMPS;
    }
    else if(ADCsamplesSent == 0)
    {
      break;
    }
    else
    {
       serial->write((uint8_t *)&ADCbuffer[obufn * SMPS],ADCsamplesSent * 2); // send filled buffer
       ADCsamplesSent = 0;      
    }
    obufn=(obufn+2)&3;
  }
  // Send trailer
  serial->write((const char *)ADCtrailer);
  // If we have more vectors to record setup for the next vector, else edit
  if(ADCvectorNum < ADCvectors)
  {
    // Set up for the next vector
    ADC->ADC_RPR = (uint32_t) &ADCbuffer[0];
    ADC->ADC_RCR = SMPS;
    ADC->ADC_RNPR = (uint32_t) &ADCbuffer[SMPS];
    ADC->ADC_RNCR = SMPS;
    bufn = obufn = 0;
    ADC->ADC_PTCR = 1;
    NVIC_ClearPendingIRQ(ADC_IRQn);
    NVIC_EnableIRQ(ADC_IRQn);
    return;
  }
  ADCready = false;
  // Initialize Analog Controller
  pmc_enable_periph_clk(ID_ADC);
  adc_set_writeprotect(ADC, 1);  // Enable the write registers
  adc_init(ADC, SystemCoreClock, ADC_FREQ_MAX, ADC_STARTUP_FAST);
  adc_configure_timing(ADC, 0, ADC_SETTLING_TIME_3, 1);
  adc_configure_trigger(ADC, ADC_TRIG_SW, 0); // Disable hardware trigger.
  adc_disable_interrupt(ADC, 0xFFFFFFFF); // Disable all ADC interrupts.
  adc_disable_all_channel(ADC);
  analogReadResolution(12);
  analogRead(ADC0);
  ReleaseADC();
}

// This function enables the ADC system to monitor the selected channel or a change in value. The ADC interrupt fires
// when the ADC value is out of the range window. The range window is recentered around the ADC value
// in the ISR. When the ISR fires the ADC has detected a change. 
int ADCchangeDet(void)
{
  if(ADCready) return(ERR_ADCALREARYSETUP);
  if(!AcquireADC()) return(ERR_ADCNOTAVALIABLE);
  ADCacquire = false;
  ADCready = true;
  // Setup the ADC
  adc_set_writeprotect(ADC, 1);  // Enable the write registers
  pmc_enable_periph_clk(ID_ADC); // To use peripheral, we must enable clock distributon to it
  adc_init(ADC, SystemCoreClock, ADC_FREQ_MIN, ADC_STARTUP_NORM); // initialize, set minumum speed
  adc_disable_interrupt(ADC, 0xFFFFFFFF);
  adc_set_resolution(ADC, ADC_12_BITS);
  adc_configure_power_save(ADC, 0, 0); // Disable sleep
  adc_configure_timing(ADC, 15, ADC_SETTLING_TIME_3, 3); // Set timings - standard values
  adc_set_bias_current(ADC, 1); // Bias current - maximum performance over current consumption
  adc_stop_sequencer(ADC); // not using it
  adc_disable_tag(ADC);    // it has to do with sequencer, not using it
  adc_disable_ts(ADC);     // disable temperature sensor
  adc_disable_channel_differential_input(ADC, (adc_channel_num_t)ADCchannel);
  adc_configure_trigger(ADC, ADC_TRIG_SW, 1); // triggering from software, freerunning mode
  adc_disable_all_channel(ADC);
  adc_enable_channel(ADC, (adc_channel_num_t)(7-ADCchannel)); // just one channel enabled
  adc_set_comparison_channel(ADC,(adc_channel_num_t)(7-ADCchannel));
  adc_set_comparison_mode(ADC,ADC_EMR_CMPMODE_OUT);
  adc_set_comparison_window(ADC,0,Threshold);
  // Setup interrupt
  //adc_enable_interrupt(ADC,ADC_IER_COMPE);
  adc_enable_interrupt(ADC,ADC_IER_DRDY);
  // Set filter to max
  ADC->ADC_EMR |= 0x0000;
  NVIC_EnableIRQ(ADC_IRQn);
  // Start the ADC
  adc_start(ADC);  
  return(0);
}

//
// Host interface commands
//

// This function prepairs the ADC system to record a set of vectors based on the 
// ADC parameters.
void ADCprep(void)
{
  int iStat = ADCsetup();
  if(iStat != 0)
  {
     SetErrorCode(iStat);
     SendNAK;
     return;    
  }
  SendACK;
}

// Issue a software trigger to the ADC system. This will cause a vector to be read
// if the ADC system is ready. 
void ADCsoftTrigger(void)
{
  ADCtrigger();
}

// This function will abort the ADC vector recording mode.
void ADCabort(void)
{
  if(!ADCready) 
  {
    SetErrorCode(ERR_ADCNOTSETUP);
    SendNAK;
    return;    
  }
  ADCready = false;
  // Initialize Analog Controller
  pmc_enable_periph_clk(ID_ADC);
  adc_set_writeprotect(ADC, 1);  // Enable the write registers
  adc_init(ADC, SystemCoreClock, ADC_FREQ_MAX, ADC_STARTUP_FAST);
  adc_configure_timing(ADC, 0, ADC_SETTLING_TIME_3, 1);
  adc_configure_trigger(ADC, ADC_TRIG_SW, 0); // Disable hardware trigger.
  adc_disable_interrupt(ADC, 0xFFFFFFFF); // Disable all ADC interrupts.
  adc_disable_all_channel(ADC);
  analogReadResolution(12);
  analogRead(ADC0);
  ReleaseADC();
  SendACK;
}

// This host command sets up the ADC change monitor function
void ADCchangeDet(int chan, int thres)
{
   if((chan < 0) || (chan > 3) || (thres < 0) || (thres > 100))
   {
     SetErrorCode(ERR_BADARG);
     SendNAK;
     return;
   }
   ADCchannel = chan;
   Threshold = thres;
   int iStat = ADCchangeDet();
   if(iStat != 0)
   {
     SetErrorCode(iStat);
     SendNAK;
     return;
   }
   SendACK;
}

void SetADCchangeParms(int wc, int sc)
{
  WindowCount = wc;
  SameCount = sc;
  SendACK;
}

void ReportADCchange(void)
{
  if(!ADCchanged) return;
  if(SerialMute) return;
  serial->print("ADC changed: ");
  serial->println(ADCvalue);
  ADCchanged = false;
}

void RecordADCchangeISR(int ADCval)
{
   ADCvalue = (float)ADCval * ADCchangeGain;
   ADCchanged = true;  
}

// This function will monitor for a ADC value change, when change is detected the ADC value
// is multiplied by gain and reported by MIPS to the host. This function is used for testing
void MonitorADCchange(char *gain)
{
  String sToken;

  sToken = gain;
  ADCchangeGain = sToken.toFloat();
  ADCattachInterrupt(RecordADCchangeISR);
}

// ADC output function to support the ADC command. This function takes an integer values and returns the 
// Counts for the channel selected.
// Valid channels numbers are 0 through 3.
void ADCread(int chan)
{
   int i;

   if((chan >= 0) && (chan <=3))
   {
     SendACKonly;
     i = analogRead(chan);
     if(!SerialMute) serial->println(i);
     return;
   }
   SetErrorCode(ERR_BADARG);
   SendNAK;
}

// This function reports the average ADC value that is recored by the change detection
// function. The ADCnumsamples value defines the number of samples.
void ADCreportAverage(void)
{
  SendACKonly;
  if(!SerialMute) serial->println(ADCcount/ADCnumsamples);
}

// Set ADC channel gain, valid gains are 1,2, or 4.
void ADRsetGain(int chan, int gain)
{
   if((chan >= 0) && (chan <=3) && ((gain==1)||(gain==2)||(gain==4)))
   {
     adc_set_writeprotect(ADC, 1);  // Enable the write registers
     if(gain == 1) gain = 0;
     if(gain == 4) gain = 3;
     ADC->ADC_MR |= 0x800000;
     ADC->ADC_CGR &= ~(0x03u << (2 * (7-chan)));
     ADC->ADC_CGR |= (gain << (2 * (7-chan)));
     SendACK;
     return;
   }
   SetErrorCode(ERR_BADARG);
   SendNAK;
  
}

// This function reads analog input on A8 and returns the value. A8 is connected to Vin through
// a voltage divider, 10K in series with 1K. This is used to determine in MIPS power is applied
// or if the USB is powering up the DUE.
// This function returns the Vin voltage as a float. The ADC input is left in its default 10 bit mode.
// A8 input is D62
//
// Updated on 2/4/2015 to read return average of 100 readings.
float ReadVin(void)
{
  int ADCvalue;
  int i;

  ADCvalue = 0;
  for (i = 0; i < 100; i++) ADCvalue += analogRead(62);
  ADCvalue = ADCvalue / 100;
  return ((((float)ADCvalue * 3.3) / 4096.0) * 11.0);
}

void ReportV12(void)
{
  SendACKonly;
  if(SerialMute) return;
  serial->println(((float)analogRead(1)/4095.0) * 3.3 * 6.3);
}
void ReportV24(void)
{
  SendACKonly;
  if(SerialMute) return;
  serial->println(((float)analogRead(0)/4095.0) * 3.3 * 12.0);  
}
void ReportCur(void)
{
  SendACKonly;
  if(SerialMute) return;
  serial->println(((float)analogRead(3)/4095.0) * 3.3 / 0.2); 
}
void ReportPower(void)
{
  SendACKonly;
  if(SerialMute) return;
  serial->println((((float)analogRead(0)/4095.0) * 3.3 * 12.0) * (((float)analogRead(3)/4095.0) * 3.3 / 0.2));
}
