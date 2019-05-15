// This file contails ADC routines that support reading the ARM ADC channels

uint16_t *ADCbuffer = NULL;   // Pointer to ADC buffer used to hold the raw data.
MIPStimer *ADCclock = NULL;   // Timer used to set the digitization rate

volatile int bufn,obufn;
volatile int ADCchannel     = 0;
volatile int ADCnumsamples  = 10000;
volatile int ADCvectors     = 1;
volatile int ADCrate        = 200000;
uint16_t     ADCvectorNum   = 0;
uint32_t     ADCsamples     = 0;
uint32_t     ADCsamplesSent = 0;
uint8_t      ADCheader[6]   = {0x55,0x55,0x55,0x55,0xAA,0};
uint8_t      ADCtrailer[6]  = {0xAE,0xAE,0xAE,0xAE,0xEA,0};

static bool ADCinuse = false;
static bool ADCready = false;

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

// This interrupt fires after the first 256 samples are collected
void ADC_Handler(void)
{
  int f = ADC->ADC_ISR;
  if (f & ADC_IER_ENDRX)
  {
    bufn = (bufn+1) & 3;
    if(ADCsamples > 256)
    {
      ADC->ADC_RNPR = (uint32_t) &ADCbuffer[bufn * 256];
      ADC->ADC_RNCR = 256;
      ADCsamples -= 256;
    }
    else if(ADCsamples == 0)
    {
      // Stop the clock, vector is collected
      ADCclock->stop();
      NVIC_DisableIRQ(ADC_IRQn);
    }
    else
    {
//      ADC->ADC_RNPR = (uint32_t) &ADCbuffer[bufn * ADCsamples];
      ADC->ADC_RNPR = (uint32_t) &ADCbuffer[bufn * 256];
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
  // Allocate the buffers, four 256 sample buffers
  if(ADCbuffer == NULL) ADCbuffer = new uint16_t [1024];
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
  ADC->ADC_RCR = 256;
  ADC->ADC_RNPR = (uint32_t) &ADCbuffer[256];
  ADC->ADC_RNCR = 256;
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
    if(ADCsamplesSent > 256)
    {
       serial->write((uint8_t *)&ADCbuffer[obufn * 256],512); // send filled buffer
       ADCsamplesSent -= 256;
    }
    else if(ADCsamplesSent == 0)
    {
      break;
    }
    else
    {
       serial->write((uint8_t *)&ADCbuffer[obufn * 256],ADCsamplesSent * 2); // send filled buffer
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
    ADC->ADC_RCR = 256;
    ADC->ADC_RNPR = (uint32_t) &ADCbuffer[256];
    ADC->ADC_RNCR = 256;
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
