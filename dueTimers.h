#define TC1_CH_UPDATE(c) TC1->TC_CHANNEL[c].TC_IER=TC_IER_CPCS; TC1->TC_CHANNEL[c].TC_IDR=~TC_IER_CPCS

volatile byte tmrSet; // 3-bit-wise "hardware timer set" flags
extern volatile byte dueIrq; // (main code) bit-wise Arduino interrupt flags

const IRQn_Type irqn[] = { TC3_IRQn, TC4_IRQn, TC5_IRQn };
const uint32_t TC_K = VARIANT_MCK/32 / 1000ul; // timer time constant
const uint32_t HWTMR_CORR = 6ul; // hardware timer delay corrector (us)
const uint32_t MIN_TIMER_TIME = 10ul; //us

uint32_t rc;
uint32_t shortTimerDelayCount[3]; // (stats)

void initTimers();
void startTimer(uint32_t us, uint32_t ch);
void stopTimer(uint32_t ch);
void onTimerElapsed(int riot_id);

void initTimers() {
  int i;
  pmc_set_writeprotect(false);
  for (i=0; i < 3; i++) {
    pmc_enable_periph_clk((uint32_t)irqn[i]);
    TC_Configure(TC1, i, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK3);
    TC1_CH_UPDATE(i); // see macro
    shortTimerDelayCount[i] = 0;
  }
  tmrSet = 0;
  dueIrq = 0;
}

// low-level DUE's hw timer interrupt setting
// ch = 0, 1, 2
void startTimer(uint32_t us, uint32_t ch) {
  //if (us < MIN_TIMER_TIME) us = MIN_TIMER_TIME;
  if (us < MIN_TIMER_TIME) { // very short time = does not use HW timer
    delayMicroseconds(us);
    dueIrq |= (byte)(1<<ch); // simulate a DUE timer irq event
    shortTimerDelayCount[ch]++;
    return;
  }
  //else if (us > 1000000ul) us = 1000000ul;
  pmc_set_writeprotect(false);
  rc = (TC_K * us) / 1000ul;
  TC_SetRA(TC1, ch, rc/2); //50% high, 50% low
  TC_SetRC(TC1, ch, rc);
  TC_Start(TC1, ch);
  TC1_CH_UPDATE(ch); // see macro
  NVIC_EnableIRQ(irqn[ch]);
  tmrSet |= (byte)(1<<ch);
}

void stopTimer(uint32_t ch) {
  NVIC_DisableIRQ(irqn[ch]);
  tmrSet &= (byte)(~(1<<ch));
}

//TC1 ch 0 - low-level timer ISR
void TC3_Handler() {
  noInterrupts();
  TC_GetStatus(TC1, 0);
  TC_Stop(TC1, 0);
  NVIC_DisableIRQ(TC3_IRQn);
  dueIrq |= 0b001;
  tmrSet &= 0b110;
  interrupts();
}

//TC1 ch 1 - low-level timer ISR
void TC4_Handler() {
  noInterrupts();
  TC_GetStatus(TC1, 1);
  TC_Stop(TC1, 1);
  NVIC_DisableIRQ(TC4_IRQn);
  dueIrq |= 0b010;
  tmrSet &= 0b101;
  interrupts();
}

//TC1 ch 2 - low-level timer ISR
void TC5_Handler() {
  noInterrupts();
  TC_GetStatus(TC1, 2);
  TC_Stop(TC1, 2);
  NVIC_DisableIRQ(TC5_IRQn);
  dueIrq |= 0b100;
  tmrSet &= 0b011;
  interrupts();
}
