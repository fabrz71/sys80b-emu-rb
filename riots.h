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
#define PA7_INTF 64 // (state output)
// interrupt flag from timer
#define TMR_INTF 128 // (state output)

#define CLEARFLAG(r,f) r->flags &= (byte)~((byte)(f))
#define SETFLAG(r,f) r->flags |= (byte)(f)
#define FLAG(r,f) ((r->flags & (byte)(f)) != 0)

#define CLEARFL(n,f) riot[n].flags &= (byte)~((byte)(f))
#define SETFL(n,f) riot[n].flags |= (byte)(f)
#define FL(n,f) ((riot[n].flags & (byte)(f)) != 0)

const word RIOT_period[] = { 1, 8, 64, 1024 };

// RIOTs
//struct riotData {
//  // RAM section
//  volatile byte ram[128]; // RIOT's sRAM
//  // I/O section
//  volatile byte irA, irB; // (virtual) input register A & B
//  volatile byte orA, orB; // output registers A & B
//  volatile byte ddrA, ddrB; // data direction registers A & B
//  // timer register
//  volatile byte intervals; // interval timer register
//  volatile word period; // 1T, 8T, 64T, 1024T (T = ticks)
//  volatile byte flags; 	// bit 0 (0x01): edge detect type (input)
//  // bit 1 (0x02): PA7 interrupt enable (input)
//  // bit 3 (0x08): timer interrupt enable (input)
//  // bit 6 (0x40): PA7 interrupt flag (output)
//  // bit 7 (0x80): timer interrupt flag (output)
//  
//  // meta-registers
//  volatile unsigned long zeroTime; // timer zero time : 0 = disabled (running flag)
//  byte lastPA7; // 7-th bit of irA on last check (used for edge detection)
//  volatile bool irqSent; // cpu IRQ signal already sent (stats)
//  volatile word irqCount; // cpu IRQ signals count (stats)
//  volatile word irqColl; // cpu IRQ collisions (stats)
//  word timerWrts; // timer updates (stats)
//  word pioWrts; // port updates (stats)
//};

struct riotData {
  // RAM section
  byte ram[128]; // RIOT's sRAM
  // I/O section
  byte irA, irB; // (virtual) input register A & B
  byte orA, orB; // output registers A & B
  byte ddrA, ddrB; // data direction registers A & B
  // timer register
  byte intervals; // interval timer register
  word period; // 1T, 8T, 64T, 1024T (T = ticks)
  byte flags;   // bit 0 (0x01): edge detect type (input)
  // bit 1 (0x02): PA7 interrupt enable (input)
  // bit 3 (0x08): timer interrupt enable (input)
  // bit 6 (0x40): PA7 interrupt flag (output)
  // bit 7 (0x80): timer interrupt flag (output)
  
  // meta-registers
  unsigned long zeroTime; // timer zero time : 0 = disabled (running flag)
  byte lastPA7; // 7-th bit of irA on last check (used for edge detection)
  bool irqSent; // cpu IRQ signal already sent (stats)
  word irqCount; // cpu IRQ signals count (stats)
  word irqColl; // cpu IRQ collisions (stats)
  word timerWrts; // timer updates (stats)
  word pioRds; // port reads (stats)
  word pioWrts; // port updates (stats)
};

extern volatile int irqRequest; // emu6502.h
extern volatile byte dueTimerIrq; // dueTimers.h
struct riotData riot[RIOTS_COUNT];

// function declarations
extern void readSysReturns(); // interface.h
extern void onRiotOutputChanged(byte riodId, byte portChanged, byte portA, byte portB); // interface.h
void resetRIOT(byte n);
void resetRIOTs();
void writeRiot(byte n, word adr, byte dta);
byte readRiot(byte n, word adr);
void updateRiotTimer(struct riotData *r);
void checkPA7Signal(byte id);
void setRiotInputs(byte id, byte port, byte data);
//void onDueTimerIrq(int riot_id);
void onDueTimerIrq();
word getRiotIrqCount(byte n);
word getRiotLostIrqCount(byte n);
word getRiotTimerWrites(byte n);
word getRiotPioWrites(byte n);

void resetRIOT(byte n) {
  struct riotData *r = &(riot[n]);
  Serial.print(F("resetting RIOT #"));
  Serial.print(n);
  Serial.println(F("..."));
  r->flags = 0x00;
  r->orA = 0x00;
  r->orB = 0x00;
  r->ddrA = 0x00;
  r->ddrB = 0x00;
  r->zeroTime = 0;
  r->irqSent = false;
  r->irqCount = 0;
  r->irqColl = 0;
  r->timerWrts = 0;
  r->pioRds = 0;
  r->pioWrts = 0;
  //nextPendingTimer = -1;
}

void resetRIOTs() {
  for (int i = 0; i < RIOTS_COUNT; i++) resetRIOT(i);
}

void writeRiot(word adr, byte dta) {
  struct riotData *r;
  byte n = (byte)((adr & 0x0180) >> 7);
  
  if (n >= RIOTS_COUNT) return;
  r = &(riot[n]);
  if ((adr & 0x0200) == 0) r->ram[adr & 0x7F] = dta; // A9=0 : RAM selection
  else { // A9=1 : timer/IO selection
    if ((adr & 0x0010) != 0) {// A4=1 : WRITE TIMER
      r->timerWrts++;
      updateRiotTimer(r);
      if (r->zeroTime != 0) return; // RIOT already set ! -> ignore request
      noInterrupts();
      r->intervals = dta;
      r->period = RIOT_period[adr & 0x3];
      // updates "timer interrupt" flag and reset interrupt flag
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
    }
    else { // A4=0
      switch (adr & 0x07) { // A2, A1, A0
        case 0x0: // write output A register
          r->pioWrts++;
          r->orA = dta;
          onRiotOutputChanged(n, RIOTPORT_A, r->orA, r->orB);
          break;
        case 0x1: // write DDR A register
          r->ddrA = dta;
          break;
        case 0x2: // write output B register
          r->pioWrts++;
          r->orB = dta;
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

byte readRiot(word adr) {
  struct riotData *r;
  byte n = (byte)((adr & 0x0180) >> 7);
  byte dta = 0;
  unsigned long int t;

  if (n >= RIOTS_COUNT) return 0;
  r = &(riot[n]);
  if ((adr & 0x0200) == 0) dta = r->ram[adr & 0x7F]; // RAM selection
  else { // A9=1 : timer/IO selection
    switch (adr & 0x0F) { // A3, A2, A1, A0
      case 0x0: // read port A register
      case 0x8:
        r->pioRds++;
        readSysReturns();
        dta = r->irA | (r->orA & r->ddrA);
        break;
      case 0x1: // read A data direction register
      case 0x9:
        dta = r->ddrA;
        break;
      case 0x2: // read port B register
      case 0xa:
        r->pioRds++;
        readSysReturns();
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
        CLEARFLAG(r, TMR_INTF);
        if ((adr & 0x08) != 0) SETFLAG(r, TMR_IRQ); 
        else CLEARFLAG(r, TMR_IRQ);
        updateRiotTimer(r);
        dta = r->intervals;
      case 0x5: // read interrupt flag register
      case 0x7:
      case 0xd:
      case 0xf:
        updateRiotTimer(r);
        dta = (FLAG(r, TMR_INTF) != 0) ? 0x80 : 0x00; // TMR_INTF -> D7
        if (FLAG(r, PA7_INTF) != 0) dta |= 0x40; // PA7_INTF -> D6
        //CLEARFLAG(r, TMR_INTF); // NO!
        CLEARFLAG(r, PA7_INTF);
        break;
      default: // should never be reached
        dta = 0;
        Serial.print(F("W: readRIOT(): unhandled address $"));
        Serial.println(adr, HEX);
        break;
    }
  }
  return dta;
}

void setRiotInputs(byte id, byte port, byte data) {
  //if (id >= RIOTS_COUNT) return;
  if (port == RIOTPORT_A) {
    riot[id].irA = data;
    checkPA7Signal(id);
  }
  else riot[id].irB = data;
}

// checks whether given RIOT's timer reached zero and updates RIOT registers & flags
void updateRiotTimer(struct riotData *r) {
  unsigned long int t;

  if (r->zeroTime == 0) { // timer not set
    //r->intervals = 0x00;
    return; 
  }
  t = micros();
  if (t <= r->zeroTime) // not-elapsed programmed-time
    r->intervals = (byte)((r->zeroTime - t) / r->period);
  else { // elapsed programmed-time (timer timeout)
    SETFLAG(r, TMR_INTF);
    t -= r->zeroTime;
    if (t < 0xff) r->intervals = 0xff - (byte)t;
    else {
      r->intervals = 0;
      r->zeroTime = 0;
    }
  }
}

void checkPA7Signal(byte id) {
  byte pa7;

  pa7 = riot[id].irA & 0x80;
  if (pa7 != riot[id].lastPA7) { // on PA7 change...
    if (FL(id, POS_EDGE) != 0) { // positive edge detection
      if (pa7 > 0) { // PA7 = ON
        if (FL(id, PA7_IRQ) != 0 && FL(id, PA7_INTF) == 0) {
          if (irqRequest) riot[id].irqColl++;
          irqRequest++;
        }
        SETFL(id, PA7_INTF);
      }
    }
    else { // negative edge detection
      if (pa7 == 0) { // PA7 = OFF
        if (FL(id, PA7_IRQ) != 0 && FL(id, PA7_INTF) == 0) {
          if (irqRequest) riot[id].irqColl++;
          irqRequest++;
        }
        SETFL(id, PA7_INTF);
      }
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
    if ((dueTimerIrq & tmrBit[id]) != 0) {
      SETFL(id, TMR_INTF);
      riot[id].zeroTime = 0xff;
      if (FL(id, TMR_IRQ) != 0) {
        if (!riot[id].irqSent) {
          if (irqRequest > 0) riot[id].irqColl++; // 6502 IRQ collision!
          irqRequest++; // emu6502.h
          riot[id].irqSent = true;
          riot[id].irqCount++;
        }
      }
      dueTimerIrq &= ((byte)~tmrBit[id]);
    }   
  }
}

word getRiotIrqCount(byte n) {
  word i = riot[n].irqCount;
  riot[n].irqCount = 0;
  return i;
}

word getRiotLostIrqCount(byte n) {
  word i = riot[n].irqColl;
  riot[n].irqColl = 0;
  return i;
}

word getRiotTimerWrites(byte n) {
  word i = riot[n].timerWrts;
  riot[n].timerWrts = 0;
  return i;
}

word getRiotPioReads(byte n) {
  word i = riot[n].pioRds;
  riot[n].pioRds = 0;
  return i;
}

word getRiotPioWrites(byte n) {
  word i = riot[n].pioWrts;
  riot[n].pioWrts = 0;
  return i;
}
