// 6532 chips (RIOTs) emulator

#include "dueTimers.h"

#define RIOTS_COUNT 3

// PA7 signal edge detecion type
#define POS_EDGE 1 // 0: PA7 negative edge; 1: PA7 positive edge
// PA7 signal interrupt enable
#define PA7_IRQ  2 // 0: disabled; 1: enabled
// timer interrupt enable
#define TMR_IRQ  8 // 0: disabled; 1: enabled
// interrupt flag from PA7 edge detecion
#define PA7_INTF 64 // (state output) causes IRQ when PA7_IRQ enabled
// interrupt flag from timer
#define TMR_INTF 128 // (state output) causes IRQ when TMR_IRQ enabled

#define CLEARFLAG(r,f) r->flags &= (byte)~((byte)(f))
#define SETFLAG(r,f) r->flags |= (byte)(f)
#define FLAG(r,f) ((r->flags & (byte)(f)) != 0)
#define CLEARFL(n,f) riot[n].flags &= (byte)~((byte)(f))
#define SETFL(n,f) riot[n].flags |= (byte)(f)
#define FL(n,f) ((riot[n].flags & (byte)(f)) != 0)

#define SETIRQOUTP(n) { riot[n].irqOut = true; irqRequest |= (byte)(1<<(n)); }
#define CLRIRQOUTP(n) { riot[n].irqOut = false; irqRequest &= ~(byte)(1<<(n)); }

const uint16_t RIOT_period[] = { 1, 8, 64, 1024 };

// RIOTs
struct riotData {
  // RAM section
  byte ram[128]; // RIOT's sRAM
  // I/O section
  volatile byte irA, irB; // (virtual) input register A & B
  byte orA, orB; // output registers A & B
  byte ddrA, ddrB; // data direction registers A & B
  // timer register
  byte intervals; // interval timer register
  uint16_t period; // 1T, 8T, 64T, 1024T (T = ticks)
  volatile byte flags;   // bit 0 (0x01): edge detect type (input)
  // bit 1 (0x02): PA7 interrupt enable (input)
  // bit 3 (0x08): timer interrupt enable (input)
  // bit 6 (0x40): PA7 interrupt flag (output)
  // bit 7 (0x80): timer interrupt flag (output)

  // meta-registers
  uint32_t zeroTime; // timer zero time : 0 = disabled (running flag)
  volatile byte lastPA7; // 7-th bit of irA on last check (used for edge detection)
  volatile bool irqOut; // IRQ output signal
  //bool irqSent; // cpu IRQ signal already sent
  volatile uint16_t irqCount; // cpu IRQ signals count (stats)
  uint16_t irqColl; // cpu IRQ collisions (stats)
  uint16_t timerWrts; // timer updates (stats)
  uint16_t pioRdsA; // port A reads (stats)
  uint16_t pioRdsB; // port B reads (stats)
  uint16_t pioWrtsA; // port A updates (stats)
  uint16_t pioWrtsB; // port B updates (stats)
} riot[RIOTS_COUNT];
//struct riotData riot[RIOTS_COUNT];

extern volatile byte irqRequest; // emu6502.h

// function declarations
extern void readSysReturns(); // interface.h
extern void onRiotOutputChanged(byte riodId, byte portChanged, byte portA, byte portB); // interface.h

void resetRIOT(byte n);
void resetRIOTs();
void writeRiot(byte n, uint16_t adr, byte dta);
byte readRiot(byte n, uint16_t adr);
void updateRiotTimer(struct riotData *r);
void checkPA7Signal(byte id);
void setRiotInputs(byte id, byte port, byte data);
void onDueTimerIrq();
void set_TMR_INTF(byte n);
void clr_TMR_INTF(byte n);
void set_PA7_INTF(byte n);
void clr_PA7_INTF(byte n);
//void _updIrqOutput(byte n);
uint16_t getRiotIrqCount(byte n);
uint16_t getRiotLostIrqCount(byte n);
uint16_t getRiotTimerWritesA(byte n);
uint16_t getRiotPioWritesA(byte n);
uint16_t getRiotTimerWritesB(byte n);
uint16_t getRiotPioWritesB(byte n);

void resetRIOT(byte n) {
  struct riotData *r = &(riot[n]);
  //Serial.print(F("resetting RIOT #"));
  //Serial.print(n);
  //Serial.println(F("..."));
  r->flags = 0x00;
  r->orA = 0x00;
  r->orB = 0x00;
  r->ddrA = 0x00;
  r->ddrB = 0x00;
  r->zeroTime = 0;
  r->irqOut = false;
  //r->irqSent = false;
  r->irqCount = 0;
  r->irqColl = 0;
  r->timerWrts = 0;
  r->pioRdsA = 0;
  r->pioRdsB = 0;
  r->pioWrtsA = 0;
  r->pioWrtsB = 0;
  CLRIRQOUTP(n);
}

void resetRIOTs() {
  for (int i = 0; i < RIOTS_COUNT; i++) resetRIOT(i);
}

void writeRiot(uint16_t adr, byte dta) {
  struct riotData *r;
  byte n = (byte)((adr & 0x0180) >> 7);

  if (n >= RIOTS_COUNT) return;
  r = &(riot[n]);
  if ((adr & 0x0200) == 0) r->ram[adr & 0x7F] = dta; // A9=0 : RAM selection
  else { // A9=1 : timer/IO selection
    if (adr & 0x0010) {// A4=1 : WRITE TIMER
      /*
      r->timerWrts++;
      updateRiotTimer(r);
      if (r->zeroTime != 0) return; // RIOT already set ! -> ignore request ###CHECK
      noInterrupts();
      r->intervals = dta;
      r->period = RIOT_period[adr & 0x3];
      // updates "timer interrupt enabled" flag and reset interrupt flag
      if ((adr & 0x08) != 0) SETFLAG(r, TMR_IRQ);
      else CLEARFLAG(r, TMR_IRQ);
      CLEARFLAG(r, TMR_INTF);
      // timer already set but no timeout yet
      if (r->zeroTime != 0 && !r->irqSent) {
         //Serial.print(F("addNewTimer(): warning: Riot #"));
         //Serial.print(riot_id);
         //Serial.println(F(" timer was already set and has't finished!"));
         NVIC_DisableIRQ(irqn[n]);
      }
      unsigned long dly = ((unsigned long)dta + 1l) * (unsigned long)r->period;
      r->zeroTime = micros() + dly;
      r->irqSent = false;
      interrupts();
      startTimer(dly - HWTMR_CORR, n);
      */
      noInterrupts();
      r->timerWrts++;
      if (tmrSet[n]) stopTimer(n);
      // updates "timer interrupt enabled" flag and reset interrupt flag
      clr_TMR_INTF(n);
      if (adr & 0x08) SETFLAG(r, TMR_IRQ); else CLEARFLAG(r, TMR_IRQ);
      r->intervals = dta;
      r->period = RIOT_period[adr & 0x3];
      uint32_t dly = ((uint32_t)dta + 1l) * (uint32_t)r->period;
      r->zeroTime = micros() + dly;
      interrupts();
      startTimer(dly - HWTMR_CORR, n);
    }
    else { // A4=0
      switch (adr & 0x07) { // A2, A1, A0
        case 0x0: // write output A register
          r->pioWrtsA++;
          r->orA = dta;
          // board output update
          onRiotOutputChanged(n, RIOTPORT_A, r->orA, r->orB);
          break;
        case 0x1: // write DDR A register
          r->ddrA = dta;
          break;
        case 0x2: // write output B register
          r->pioWrtsB++;
          r->orB = dta;
          // board output update
          onRiotOutputChanged(n, RIOTPORT_B, r->orA, r->orB);
          break;
        case 0x3: // write DDR B register
          r->ddrB = dta;
          break;
        case 0x4: // write edge detect control: negative ED, disable PA7 int
        case 0x5: // write edge detect control: positive ED, disable PA7 int
        case 0x6: // write edge detect control: negative ED, enable PA7 int
        case 0x7: // write edge detect control: positive ED, enable PA7 int
          r->flags = (r->flags & 0xF8) | (adr & 0x03);
          break;
      }
    }
  }
}

byte readRiot(uint16_t adr) {
  struct riotData *r;
  byte n = (byte)((adr & 0x0180) >> 7);
  byte dta = 0;
  //unsigned long int t;

  if (n >= RIOTS_COUNT) return 0;
  r = &(riot[n]);
  if ((adr & 0x0200) == 0) dta = r->ram[adr & 0x7F]; // RAM selection
  else { // A9=1 : timer/IO selection
    switch (adr & 0x0F) { // A3, A2, A1, A0
      case 0x0: // read port A register
      case 0x8:
        r->pioRdsA++;
        if (n == 0) readSysReturns(); // gets updated returns value
        dta = r->irA | (r->orA & r->ddrA);
        break;
      case 0x1: // read A data direction register
      case 0x9:
        dta = r->ddrA;
        break;
      case 0x2: // read port B register
      case 0xa:
        r->pioRdsB++;
        if (n == 0) readSysReturns(); // only RIOT #0 is set with inputs
        dta = r->irB | (r->orA & r->ddrB);
        break;
      case 0x3: // read B data direction register
      case 0xb:
        dta = r->ddrB;
        break;
      case 0x4: // read timer + disable interrupt timer
      case 0x6:
      case 0xc: // read timer + enable interrupt timer
      case 0xe:
        clr_TMR_INTF(n);
        if (adr & 0x08) SETFLAG(r, TMR_IRQ); else CLEARFLAG(r, TMR_IRQ);
        updateRiotTimer(r);
        dta = r->intervals;
      case 0x5: // read interrupt flag register
      case 0x7:
      case 0xd:
      case 0xf:
        //updateRiotTimer(r);
        dta = (FLAG(r, TMR_INTF)) ? 0b10000000 : 0; // TMR_INTF -> D7
        if (FLAG(r, PA7_INTF)) dta |= 0b01000000; // PA7_INTF -> D6
        clr_PA7_INTF(n);
        break;
      default: // should never be reached
        //dta = 0;
        Serial.print(F("W: readRIOT(): unhandled address $"));
        Serial.println(adr, HEX);
        break;
    }
  }
  return dta;
}

// called from outside when inputs has changed -
// may yield IRQ
void setRiotInputs(byte id, byte port, byte data) {
  //if (id >= RIOTS_COUNT) return;
  if (port == RIOTPORT_A) {
    riot[id].irA = data;
    checkPA7Signal(id);
  }
  else riot[id].irB = data;
}

// checks whether given RIOT's timer reached zero and updates RIOT registers.
// does not affect TMR_INTF flag
void updateRiotTimer(struct riotData *r) {
  unsigned long int t;

  if (r->zeroTime == 0) { // timer not set! - no effect
    //r->intervals = 0;
    return;
  }
  t = micros();
  if (t <= r->zeroTime) // not timed out
    r->intervals = (byte)((r->zeroTime - t) / r->period); // may be 0
  else { // timer timeout
    //SETFLAG(r, TMR_INTF);
    t -= r->zeroTime; // us after timeout
    if (t < 0xff) r->intervals = 0xff - (byte)t;
    else {
      r->intervals = 0; // 256 us after timeout
      r->zeroTime = 0; // RIOT timer disabled
    }
  }
}

// check RIOT PA7 signal edge for interrupt generation
// works on PA7 only in input coniguration
void checkPA7Signal(byte id) {
  byte pa7;

  pa7 = riot[id].irA & 0x80;
  if (pa7 != riot[id].lastPA7) { // on PA7 change...
    if (FL(id, POS_EDGE)) { // positive edge detection
      if (pa7 > 0) set_PA7_INTF(id);
    }
    else { // negative edge detection
      if (pa7 == 0) set_PA7_INTF(id);
    }
    riot[id].lastPA7 = pa7;
  }
}

// to call after a DUE-timer irq call -
// updates corresponding RIOT's timer state after DUE's timer event.
// At least one of the first 3 bits of dueTimerIrq shoud be set:
// bit 0 set = DUE's TC3 timer irq = RIOT #0 timer event
// bit 1 set = DUE's TC4 timer irq = RIOT #1 timer event
// bit 2 set = DUE's TC5 timer irq = RIOT #2 timer event
void onDueTimerIrq() {
  byte id; // RIOT id

  for (id=0; id<3; id++) {
    if (dueTimerIrq & tmrBit[id]) {
      riot[id].zeroTime = 0xff;
      set_TMR_INTF(id);
      dueTimerIrq &= ((byte)~tmrBit[id]);
    }
  }
}

void set_TMR_INTF(byte n) {
  SETFL(n, TMR_INTF);
  if (FL(n, TMR_IRQ)) {
    riot[n].irqCount++;
    if (riot[n].irqOut) riot[n].irqColl++;
    SETIRQOUTP(n);
  }
}

void clr_TMR_INTF(byte n) {
  CLEARFL(n, TMR_INTF);
  if (FL(n, PA7_INTF) && FL(n, PA7_IRQ)) SETIRQOUTP(n)
  else CLRIRQOUTP(n);
}

void set_PA7_INTF(byte n) {
  SETFL(n, PA7_INTF);
  if (FL(n, PA7_IRQ)) {
    riot[n].irqCount++;
    if (riot[n].irqOut) riot[n].irqColl++;
    SETIRQOUTP(n);
  }
}

void clr_PA7_INTF(byte n) {
  CLEARFL(n, PA7_INTF);
  if (FL(n, TMR_INTF) && FL(n, TMR_IRQ)) SETIRQOUTP(n)
  else CLRIRQOUTP(n);
}

// void _updIrqOutput(byte n) {
//   if ( (FL(n, PA7_INTF) && FL(n, PA7_IRQ)) ||
//        (FL(n, TMR_INTF) && FL(n, TMR_IRQ)) ) SETIRQOUTP(n);
//   else CLRIRQOUTP(n);
// }

uint16_t getRiotIrqCount(byte n) {
  uint16_t i = riot[n].irqCount;
  riot[n].irqCount = 0;
  return i;
}

uint16_t getRiotLostIrqCount(byte n) {
  uint16_t i = riot[n].irqColl;
  riot[n].irqColl = 0;
  return i;
}

uint16_t getRiotTimerWrites(byte n) {
  uint16_t i = riot[n].timerWrts;
  riot[n].timerWrts = 0;
  return i;
}

uint16_t getRiotPioReadsA(byte n) {
  uint16_t i = riot[n].pioRdsA;
  riot[n].pioRdsA = 0;
  return i;
}

uint16_t getRiotPioWritesA(byte n) {
  uint16_t i = riot[n].pioWrtsA;
  riot[n].pioWrtsA = 0;
  return i;
}

uint16_t getRiotPioReadsB(byte n) {
  uint16_t i = riot[n].pioRdsB;
  riot[n].pioRdsB = 0;
  return i;
}

uint16_t getRiotPioWritesB(byte n) {
  uint16_t i = riot[n].pioWrtsB;
  riot[n].pioWrtsB = 0;
  return i;
}
