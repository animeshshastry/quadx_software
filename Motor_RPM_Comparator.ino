#include <TimeLib.h>

IntervalTimer FCTimer_1,FCTimer_2,FCTimer_3,FCTimer_4;

#define CMP_SCR_IER (1<<4)
#define CMP_SCR_IEF (1<<3)
#define CMP_SCR_CFR (1<<2)
#define CMP_SCR_CFF (1<<1)

#define updateRate 200
#define min_cycnt 24000

volatile uint32_t inputcycles_1,inputcycles_2,inputcycles_3,inputcycles_4;
volatile uint32_t startcount_1,startcount_2,startcount_3,startcount_4;
volatile uint32_t prevcount_1,prevcount_2,prevcount_3,prevcount_4;
volatile uint32_t endcount_1,endcount_2,endcount_3,endcount_4;
volatile float ctfrequency_1 = 0, ctfrequency_2 = 0, ctfrequency_3 = 0, ctfrequency_4 = 0;
volatile bool ctavailable_1 = false, ctavailable_2 = false, ctavailable_3 = false, ctavailable_4 = false;

volatile float starttime_1, endtime_1;

void comparatorSetup(){
  
  CCM_CCGR2 |= CCM_CCGR2_XBAR1(CCM_CCGR_ON);  //(1) turn clock on for xbara1
  CCM_CCGR3 |= 0b1111111100000000000000000000; //enable all 4 ACMP clocks

  CMP1_CR0 = 0b01110011;                      //(6) FILTER_CNT=7; HYSTCTR=3
  CMP1_CR1 = 0b00010111;                      //(7) SE=0, high power, COUTA, output pin, enable; mode #2A
  CMP1_DACCR = 0b11001010;                    //(8) Set DAC = fraction of VIN2 (3.3v) = 0.77v 
  CMP1_MUXCR = 0b00110111;                    //(9a) CMP_MUX_PSEL(0) | CMP_MUX_MSEL(7) Input pins select;
                                              //(9b)   plus = IN6 (pin 21), minus = DAC (code 7). PSTM = 0 (Pass Through Mode Disabled)

  CMP2_CR0 = 0b01110011;                      //(6) FILTER_CNT=7; HYSTCTR=3
  CMP2_CR1 = 0b00010111;                      //(7) SE=0, high power, COUTA, output pin, enable; mode #2A
  CMP2_DACCR = 0b11011111;                    //(8) Set DAC = 1/2 of VIN2 (3.3v) = 1.65v 
  CMP2_MUXCR = 0b00000111;                    //(9a) CMP_MUX_PSEL(0) | CMP_MUX_MSEL(7) Input pins select;
                                              //(9b)   plus = IN0 (pin 18), minus = DAC (code 7). PSTM = 0 (Pass Through Mode Disabled)
  
  CMP3_CR0 = 0b01110011;                      //(6) FILTER_CNT=7; HYSTCTR=3
  CMP3_CR1 = 0b00010111;                      //(7) SE=0, high power, COUTA, output pin, enable; mode #2A
  CMP3_DACCR = 0b11011111;                    //(8) Set DAC = 1/2 of VIN2 (3.3v) = 1.65v 
  CMP3_MUXCR = 0b00100111;                    //(9a) CMP_MUX_PSEL(0) | CMP_MUX_MSEL(7) Input pins select;
                                              //(9b)   plus = IN4 (pin 1), minus = DAC (code 7). PSTM = 0 (Pass Through Mode Disabled)
  
  CMP4_CR0 = 0b01110011;                      //(6) FILTER_CNT=7; HYSTCTR=3
  CMP4_CR1 = 0b00010111;                      //(7) SE=0, high power, COUTA, output pin, enable; mode #2A
  CMP4_DACCR = 0b11011111;                    //(8) Set DAC = 1/2 of VIN2 (3.3v) = 1.65v 
  CMP4_MUXCR = 0b00100111;                    //(9a) CMP_MUX_PSEL(0) | CMP_MUX_MSEL(7) Input pins select;
                                              //(9b)   plus = IN4 (pin 0), minus = DAC (code 7). PSTM = 0 (Pass Through Mode Disabled)

  attachInterruptVector(IRQ_ACMP1, &CMP1_ISR );
  NVIC_SET_PRIORITY(IRQ_ACMP1, 32); // high priority
  NVIC_ENABLE_IRQ(IRQ_ACMP1);
  inputcycles_1 = 0;
  CMP1_SCR = CMP_SCR_CFR + CMP_SCR_CFF;  // clear the flags and interrupt enables
  CMP1_SCR = CMP_SCR_IER + CMP_SCR_IEF;  // enable rising & falling edge interrupt
  
  attachInterruptVector(IRQ_ACMP2, &CMP2_ISR );
  NVIC_SET_PRIORITY(IRQ_ACMP2, 32); // high priority
  NVIC_ENABLE_IRQ(IRQ_ACMP2);
  inputcycles_2 = 0;
  CMP2_SCR = CMP_SCR_CFR + CMP_SCR_CFF;  // clear the flags and interrupt enables
  CMP2_SCR = CMP_SCR_IER;  // enable rising edge interrupt

  attachInterruptVector(IRQ_ACMP3, &CMP3_ISR );
  NVIC_SET_PRIORITY(IRQ_ACMP3, 32); // high priority
  NVIC_ENABLE_IRQ(IRQ_ACMP3);
  inputcycles_3 = 0;
  CMP3_SCR = CMP_SCR_CFR + CMP_SCR_CFF;  // clear the flags and interrupt enables
  CMP3_SCR = CMP_SCR_IER;  // enable rising edge interrupt

  attachInterruptVector(IRQ_ACMP4, &CMP4_ISR );
  NVIC_SET_PRIORITY(IRQ_ACMP4, 32); // high priority
  NVIC_ENABLE_IRQ(IRQ_ACMP4);
  inputcycles_4 = 0;
  CMP4_SCR = CMP_SCR_CFR + CMP_SCR_CFF;  // clear the flags and interrupt enables
  CMP4_SCR = CMP_SCR_IER;  // enable rising edge interrupt

  delay(10); // default first delay of 10mSec
  // now set up an intervaltimer to do auto updates
  FCTimer_1.begin(FC1_ISR, 1000000 / updateRate);
  FCTimer_2.begin(FC2_ISR, 1000000 / updateRate);
  FCTimer_3.begin(FC3_ISR, 1000000 / updateRate);
  FCTimer_4.begin(FC4_ISR, 1000000 / updateRate);
  
}

// This timer interrupt handler updates the frequency result
// at regular intervals in a background process
void FC1_ISR(void) {
  uint32_t dcycnt;
  CMP1_SCR = CMP_SCR_CFR + CMP_SCR_CFF;  // clear the flags and interrupt enables
  dcycnt = endcount_1 - startcount_1;
  if (inputcycles_1>0) inputcycles_1--;  // correct for initial count
  ctfrequency_1 = (float)inputcycles_1 * F_CPU / (float)dcycnt;
  CMP1_SCR = CMP_SCR_CFR + CMP_SCR_CFF;  // clear the flags and interrupt enables
  CMP1_SCR = CMP_SCR_IER + CMP_SCR_IEF;  // enable rising & falling edge interrupt
  ctavailable_1 = true;
  inputcycles_1 = 0; // starts next collection cycle
}
void FC2_ISR(void) {
  uint32_t dcycnt;
  CMP2_SCR = CMP_SCR_CFR + CMP_SCR_CFF;  // clear the flags and interrupt enables
  dcycnt = endcount_2 - startcount_2;
  if (inputcycles_2>0) inputcycles_2--;  // correct for initial count
  ctfrequency_2 = (float)inputcycles_2 * F_CPU / (float)dcycnt;
  CMP2_SCR = CMP_SCR_CFR + CMP_SCR_CFF;  // clear the flags and interrupt enables
  CMP2_SCR = CMP_SCR_IER + CMP_SCR_IEF;  // enable rising edge interrupt
  ctavailable_2 = true;
  inputcycles_2 = 0; // starts next collection cycle
}
void FC3_ISR(void) {
  uint32_t dcycnt;
  CMP3_SCR = CMP_SCR_CFR + CMP_SCR_CFF;  // clear the flags and interrupt enables
  dcycnt = endcount_3 - startcount_3;
  if (inputcycles_3>0) inputcycles_3--;  // correct for initial count
  ctfrequency_3 = (float)inputcycles_3 * F_CPU / (float)dcycnt;
  CMP3_SCR = CMP_SCR_CFR + CMP_SCR_CFF;  // clear the flags and interrupt enables
  CMP3_SCR = CMP_SCR_IER + CMP_SCR_IEF;  // enable rising edge interrupt
  ctavailable_3 = true;
  inputcycles_3 = 0; // starts next collection cycle
}
void FC4_ISR(void) {
  uint32_t dcycnt;
  CMP4_SCR = CMP_SCR_CFR + CMP_SCR_CFF;  // clear the flags and interrupt enables
  dcycnt = endcount_4 - startcount_4;
  if (inputcycles_4>0) inputcycles_4--;  // correct for initial count
  ctfrequency_4 = (float)inputcycles_4 * F_CPU / (float)dcycnt;
  CMP4_SCR = CMP_SCR_CFR + CMP_SCR_CFF;  // clear the flags and interrupt enables
  CMP4_SCR = CMP_SCR_IER + CMP_SCR_IEF;  // enable rising edge interrupt
  ctavailable_4 = true;
  inputcycles_4 = 0; // starts next collection cycle
}

void CMP1_ISR(void) {
  uint32_t cycnt;
  CMP1_SCR = CMP_SCR_CFR | CMP_SCR_CFF; // clear the flags
  cycnt =  ARM_DWT_CYCCNT;
  if (inputcycles_1 == 0) { // first count
    startcount_1 = cycnt;
    inputcycles_1++;
  } else {
    if ((cycnt-prevcount_1) > min_cycnt){
      inputcycles_1++;
      endcount_1 = cycnt;
    }
  }
  prevcount_1 = cycnt;
  
  CMP1_SCR = CMP_SCR_IER + CMP_SCR_IEF;  // re-enable rising & falling edge interrupt
#if defined(__IMXRT1062__)  // Teensy 4.x
  asm("DSB");  // Without this instruction, ISR is double-triggered!
#endif
}
void CMP2_ISR(void) {
  uint32_t cycnt;
  CMP2_SCR = CMP_SCR_CFR | CMP_SCR_CFF; // clear the flags
  cycnt =  ARM_DWT_CYCCNT;
  if (inputcycles_2 == 0) { // first count
    startcount_2 = cycnt;
    inputcycles_2++;
  } else {
    if ((cycnt-prevcount_2) > min_cycnt){
      inputcycles_2++;
      endcount_2 = cycnt;
    }
  }
  prevcount_2 = cycnt;
  
  CMP2_SCR = CMP_SCR_IER + CMP_SCR_IEF;  // re-enable rising & falling edge interrupt
#if defined(__IMXRT1062__)  // Teensy 4.x
  asm("DSB");  // Without this instruction, ISR is double-triggered!
#endif
}

void CMP3_ISR(void) {
  uint32_t cycnt;
  CMP3_SCR = CMP_SCR_CFR | CMP_SCR_CFF; // clear the flags
  cycnt =  ARM_DWT_CYCCNT;
  if (inputcycles_3 == 0) { // first count
    startcount_3 = cycnt;
    inputcycles_3++;
  } else {
    if ((cycnt-prevcount_3) > min_cycnt){
      inputcycles_3++;
      endcount_3 = cycnt;
    }
  }
  prevcount_3 = cycnt;
  
  CMP3_SCR = CMP_SCR_IER + CMP_SCR_IEF;  // re-enable rising & falling edge interrupt
#if defined(__IMXRT1062__)  // Teensy 4.x
  asm("DSB");  // Without this instruction, ISR is double-triggered!
#endif
}

void CMP4_ISR(void) {
  uint32_t cycnt;
  CMP4_SCR = CMP_SCR_CFR | CMP_SCR_CFF; // clear the flags
  cycnt =  ARM_DWT_CYCCNT;
  if (inputcycles_4 == 0) { // first count
    startcount_4 = cycnt;
    inputcycles_4++;
  } else {
    if ((cycnt-prevcount_4) > min_cycnt){
      inputcycles_4++;
      endcount_4 = cycnt;
    }
  }
  prevcount_4 = cycnt;
  
  CMP3_SCR = CMP_SCR_IER + CMP_SCR_IEF;  // re-enable rising & falling edge interrupt
#if defined(__IMXRT1062__)  // Teensy 4.x
  asm("DSB");  // Without this instruction, ISR is double-triggered!
#endif
}

float CTFrequency_1(void) {
  ctavailable_1 = false;
  return ctfrequency_1;
}
bool CTAvailable_1(void) {
  return ctavailable_1;
}
float CTFrequency_2(void) {
  ctavailable_2 = false;
  return ctfrequency_2;
}
bool CTAvailable_2(void) {
  return ctavailable_2;
}
float CTFrequency_3(void) {
  ctavailable_3 = false;
  return ctfrequency_3;
}
bool CTAvailable_3(void) {
  return ctavailable_3;
}
float CTFrequency_4(void) {
  ctavailable_4 = false;
  return ctfrequency_4;
}
bool CTAvailable_4(void) {
  return ctavailable_4;
}

void printMotorRPM(int print_rate) {
  if ( (current_time - print_counter) * micros2secs > (1.0 / print_rate)) {
    print_counter = micros();
    SERIAL_PORT.print(F(" m1: "));
    SERIAL_PORT.print(CTFrequency_1());
//    SERIAL_PORT.print(F(" m2: "));
//    SERIAL_PORT.print(CTFrequency_2());
//    SERIAL_PORT.print(F(" m3: "));
//    SERIAL_PORT.print(CTFrequency_3());
//    SERIAL_PORT.print(F(" m4: "));
//    SERIAL_PORT.print(CTFrequency_4());
  }
}
