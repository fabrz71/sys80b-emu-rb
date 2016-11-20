// RIOTs vs. SYS80B-signals interface (glue-logic implementation)

#include "mcp_io.h"
#include "dispbuf.h"
#include "ledGrid.h"

#define RETURNS_PORT PIOC // inputs
#define STROBES_PORT PIOC // outputs
#define DISPLAY_PORT PIOD // outputs
#define RETURNS_LSB_POS 1
#define STROBES_LSB_POS 12

#define IO_STREAM_MEM 100

// messages output by I/O type
//const bool inMsgEn[] = { false, false };
//const bool outMsgEn[] = { false, false, true, false, false };

const unsigned int RETURNS_bitmask = ((unsigned int)0xff) << RETURNS_LSB_POS;
const unsigned int STROBES_bitmask = ((unsigned int)0xff) << STROBES_LSB_POS;
const unsigned int DISPLAY_bitmask = 0x07ffu;

// 16 bit mux outputs
PROGMEM const uint16_t mux16[16] =
  { 0x0001, 0x0002, 0x0004, 0x0008, 0x0010, 0x0020, 0x0040, 0x0080,
    0x0100, 0x0200, 0x0400, 0x0800, 0x1000, 0x2000, 0x4000, 0x8000 };

// 8 bit demux output
PROGMEM const byte toStrobeNum[] =
  { 0, 1, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4,  5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5,
    6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6,  6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6,
    7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,  7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
    7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,  7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
    8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,  8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
    8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,  8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
    8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,  8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
    8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,  8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8 };

extern const byte GPIO_SS_PIN;
extern const byte IOEN_PIN;

// "Operator adjustables" switches setting
byte opSwtch[] = { 0, 0b00000011, 0b00000111, 0b00000001, 0b11000010 }; // first value not used

// System I/O signals
byte strobesN; // outputs
byte returnsN; // inputs
uint16_t solenoids; // outputs
//bool solenoid9; // output
byte sound; // outputs
byte lampCtrl; // outputs
uint16_t lampStrobes; // outputs
//byte displayData; // outputs: D0-D7 display data bus
volatile bool slamSw; // input

// System interface logic vars
byte displayLatch; // display D0-D7 data latch
byte displayId, prevDid = 0; // 0: not enabled; 1: display1; 2: display2; 3: both displays
byte displayResetN;
byte lampGroup;
byte prevStrobeNum;
byte sound16_L4; // "special" lamp used as 5th sound bit [0,1]
byte switchEnable, prevSwEnable;
byte switchSel;
uint16_t prevSol; // previous value of solenoids
byte prevSnd; // previous value of sound

// auxiliary/stats/debug vars
byte returnsCache[] = {0, 0, 0, 0, 0, 0, 0, 0, 0}; // strobe = 1..8 - first value 0 not used
byte forcedReturn[] = {0, 0, 0, 0, 0, 0, 0, 0, 0}; // strobe = 1..8 - first value 0 non used
volatile uint32_t slamIntCount = 0;
volatile uint16_t outpCount[5] = {0, 0, 0, 0, 0}; // output changes count (stats) - see outputType enum

extern void setRiotInputs(byte id, byte port, byte data); // riots.h
extern void debugLedsOutput(byte val);

void initInterface();
void testInterface();
void initGPIO();
void updateRIOTsInput();
void onRiotOutputChanged(byte riodId, byte portChanged, byte portA, byte portB);
void dispatchOutputData(byte type);
void readSysReturns();
//void readSysInputs();
void onSlamChanged(); // ISR
uint16_t getOutpCount(byte n);

void initInterface() {
  RETURNS_PORT->PIO_ODR |= RETURNS_bitmask; // set port bits 1-8 as INPUT (Returns)
  RETURNS_PORT->PIO_PUER |= RETURNS_bitmask; // enable port bits 1-8 internal PULL-UP R (Returns)
  STROBES_PORT->PIO_OER |= STROBES_bitmask; // set port bits 12-19 as OUTPUT (Strobes)
  DISPLAY_PORT->PIO_OER |= DISPLAY_bitmask; // set port bits 0-10 as OUTPUT (Display)
  //pinMode(12, OUTPUT); // patch for display outputs!
  pinMode(14, OUTPUT); // patch for display outputs!
  pinMode(15, OUTPUT); // patch for display outputs!
  strobesN = 0xff; // outputs
  returnsN = 0xff; // inputs
  initGPIO();
  initLedGrid(LG_CS_PIN);
  ledGridEnabled = true;
  //debugLedsOutput(5);
  readSysReturns();
  //debugLedsOutput(6);
  attachInterrupt(SLAM_PIN, onSlamChanged, CHANGE);
}

void testInterface() {
  int i;
  int p;
  uint16_t w;

  // MCP23S17s GPIO outputs
  for (p=0; p<2; p++) {
    Serial.print("GPIO");
    Serial.print(p);
    Serial.println(" test...");
    w = 1;
    for (i=0; i<16; i++) {
      mcpWrite(p, w);
      w <<= 1;
      delay(200);
    }
    mcpWrite(p, 0);
  }
  // DUE's display outputs
  w = 1;
  Serial.println("Display signals...");
  for (i=0; i<12; i++) {
    DISPLAY_PORT->PIO_SODR = (uint32_t)w; // sets 1 bits
    DISPLAY_PORT->PIO_CODR = ~((uint32_t)w) & DISPLAY_bitmask; // clears 0 bits
    w <<= 1;
    delay(200);
  }
  // DUE's strobes outputs
  w = 1;
  Serial.println("Strobes...");
  for (i=0; i<8; i++) {
    STROBES_PORT->PIO_SODR = ((uint32_t)w) << STROBES_LSB_POS; // sets 1 bits
    STROBES_PORT->PIO_CODR = (((uint32_t)~w) << STROBES_LSB_POS) & STROBES_bitmask; // clears 0 bits
    w <<= 1;
    delay(200);
  }
  // LED GRID outputs
  Serial.println("LED grid...");
  for (i=0; i<255; i++) {
    setLedRow(i%8, i);
    delay(20);
  }
}

// MCP23S17 GPIO ports setup
void initGPIO() {
  MCP_init(GPIO_SS_PIN);
  mcpWrite(0, 0x0f00); // resets MCP0 outputs
  mcpWrite(1, 0x0000); // resets MCP1 outputs
}

// update system output signals variables upon RIOTs' output ports changes
void onRiotOutputChanged(byte riotId, byte portChanged, byte portA, byte portB) {
  switch (riotId) {
    case 0: // RIOT 1: switch matrix
      if (portChanged == RIOTPORT_B) { // port B changed
        strobesN = (byte)~portB & 0xff; // Z11, Z12 outputs
        dispatchOutputData(STROBES);
      }
      break;
    case 1: // RIOT 2: display control
      if (portChanged == RIOTPORT_A) { // port A: display data latch enable (2 bits)
        if (portA & 0b00010000) // PA4 = 1 : latch low nybble (Z18) enabled
          displayLatch = (displayLatch & 0xf0) | (portB & 0x0f);
        if (portA & 0b00100000) // PA5 = 1 : latch high nybble (Z20) enabled
          displayLatch = (displayLatch & 0x0f) | ((portB & 0x0f) << 4);
      }
      else { // port B: display half-byte (4 bits), LD (2 bits), display+sound reset, switch enable
        displayId = (1 - bitRead(portB, 4)) + 2*(1 - bitRead(portB, 5));
        displayResetN = 1 - bitRead(portB, 6);
        dispatchOutputData(DISPL);
        //switchEnable = 1 - bitRead(portB, 7);
        switchEnable = bitRead(portB, 7);
        if (switchEnable != prevSwEnable) { // changed
          Serial.print("Switch enable signal: ");
          Serial.println(switchEnable);
          readSysReturns();
          prevSwEnable = switchEnable;
        }
      }
      break;
    case 2: // RIOT 3: solenoids, lamps & sound
      if (portChanged == RIOTPORT_A) { // port A changed: solenoids & sound
        solenoids = 0;
        if (bitRead(portA, 5) == 0) // low nybble (Z28, Z29)
          solenoids = mux16[~portA & 0x03];
          //solenoids = 1 << ((~portA) & 0x03);
        if (bitRead(portA, 6) == 0) // high nybble (Z28, Z30)
          solenoids |= (mux16[((~portA & 0x0c) >> 2)] << 4);
          //solenoids |= mux16[4 + ((~portA & 0x0c) >> 2)];
          //solenoids = (solenoids & 0x0f) | (1 << (4+(((~portA) & 0x0c) >> 2)) );
        if (bitRead(portA, 7) == 0) solenoids |= 0x0100;
        //solenoid9 = (bitRead(portA, 7) == 0) ? false : true;
        dispatchOutputData(SOLENOIDS);
        //sound = bitRead(portA, 4) ? 0 : ~portA & 0x0f;
        sound = (portA & 0x10) ? 0 : (byte)~portA & 0x0f;
        sound |= sound16_L4<<4;
        dispatchOutputData(SOUND);
      }
      else { // port B: lamps controlif (strobeNum != prevStrobeNum)
        lampCtrl = portB & 0x0f; // lamps strobe
        lampGroup = (portB >> 4) & 0x0f;
        lampStrobes = (mux16[lampGroup] & 0x1fff ) >> 1;
        if (lampGroup == 1) sound16_L4 = lampCtrl & 1; // sound16 = lamp 4
        switchSel = lampGroup & 0x03;
        dispatchOutputData(LAMPS);
      }
      break;
  }
}

// hadrware output signals implementation
void dispatchOutputData(byte type) {
  uint16_t ctrlw;
  uint32_t ctrlww;

  switch (type) {
    case STROBES:
      ctrlww = ((uint32_t)strobesN) << STROBES_LSB_POS;
      STROBES_PORT -> PIO_SODR = ctrlww; // sets 1 bits
      STROBES_PORT -> PIO_CODR = ~ctrlww & STROBES_bitmask; // clears 0 bits
      break;
    case SOLENOIDS:
      ctrlw = prevSol ^ solenoids;
      // solenoids 1-8 have changed ?
      if (ctrlw & 0x00ff) mcpWritePA(0, (byte)(solenoids & 0x00ff));
      // solenoid 9 has changed ?
      if (ctrlw & 0x0100) mcpWritePB(0, sound | ((solenoids & 0x0100)>>4));
      prevSol = solenoids;
      if (ctrlw > 0 && solenoids > 0) {
        Serial.print(F("Solenoids: "));
        Serial.println(solenoids, BIN);
      }
      break;
    case SOUND:
      mcpWritePB(0, (sound & 0xff) | ((solenoids & 0x0100)>>4));
      if (prevSnd != sound && sound > 0) {
        Serial.print(F("Sound: "));
        Serial.print(sound);
        Serial.print(F(" (L4:"));
        Serial.print(sound16_L4);
        Serial.println(")");
      }
      prevSnd = sound;
      break;
    case LAMPS:
      ctrlw = (lampCtrl | (lampStrobes << 4)) & 0xffff;
      mcpWrite(1, ctrlw);
      break;
    case DISPL:
      //digitalWrite(LED_PIN, (displayId == 3) ? HIGH : LOW); // FOR TEST ONLY
      ctrlww = (uint32_t)displayLatch | ((uint16_t)displayId << 8) | ((uint16_t)displayResetN << 10);
      DISPLAY_PORT->PIO_SODR = ctrlww; // sets 1 bits
      DISPLAY_PORT->PIO_CODR = ~ctrlww & DISPLAY_bitmask; // clears 0 bits
      if (!displayResetN) resetDisplays();
      else if (prevDid ^ displayId) { // displayId changed
        if (displayId) pulseLD(displayId, displayLatch);
        prevDid = displayId;
      }
      break;
  }
  outpCount[type]++;
}

// read DUE's returns input and updates returnsN and RIOT-0 inputs.
// called before every RIOT #0 port read
void readSysReturns() {
  byte portInput;
  byte strobeNum; // 1..8 (0 = no strobes)

  if (switchEnable) { // "operator adjustables" reading mode
    strobeNum = switchSel+1;
    returnsN = (byte)~opSwtch[strobeNum]; // "operator adjustables" reading mode
  }
  else { // normal switch reading
    strobeNum = toStrobeNum[(byte)~strobesN];
    if (strobeNum) {
      if (forcedReturn[strobeNum]) returnsN = (byte)~forcedReturn[strobeNum];
      else returnsN = (byte)(((RETURNS_PORT->PIO_PDSR & RETURNS_bitmask) >> RETURNS_LSB_POS) & 0xff);
    }
    else returnsN = 0xff; // no strobes
  }
  portInput = (byte)~returnsN;
  if (returnsCache[strobeNum] != portInput) {
    if (strobeNum) {
      Serial.print("strobe: ");
      //Serial.print((byte)~strobesN);
      //Serial.print("->");
      Serial.print(strobeNum);
      Serial.print(": ");
      Serial.print(portInput);
      if (switchEnable) Serial.println(" (opSwitches)"); else Serial.println("");
    }
    returnsCache[strobeNum] = portInput;
  }
  setRiotInputs(0, RIOTPORT_A, portInput); // returns
  prevStrobeNum = strobeNum;
  if (strobeNum) setLedRow(strobeNum-1, portInput);
}

/*
// read all system (DUE's) inputs and updates corresponding variables and RIOTs inputs
void readSysInputs() {
    returnsN = (byte)(((RETURNS_PORT->PIO_PDSR & RETURNS_bitmask) >> RETURNS_LSB_POS) & 0xff);
    slamSw = (digitalRead(SLAM_PIN) == HIGH) ? false : true;
    updateRIOTsInput();
}
*/

/*
// update RIOTs' input ports upon input signals change
// ( called by readSysInput() )
void updateRIOTsInput() {
  byte portInput;
  byte strobeNum = toStrobeNum[(byte)~strobesN];

  if (switchEnable == 0) portInput = (byte)~returnsN; // normal mode
  else portInput = opSwtch[switchSel]; // "operator adjustables" read mode

//  else { // normal mode: detects changes
//    byte retChange;
//    portInput = ~returnsN;
//    if (strobeNum > 0) {
//      retChange = returnsCache[strobeNum] ^ portInput;
//      if (retChange) { // returns on strobe line changed
//        Serial.print(F("Matrix S"));
//        Serial.print(strobeNum);
//        Serial.print(": ");
//        for (int i=0; i<8; i++) {
//          if (retChange && bit(i)) {
//            Serial.print("R");
//            Serial.print(i);
//            Serial.print("=");
//            Serial.print((portInput && bit(i)) ? 1 : 0);
//            Serial.print(" ");
//          }
//        }
//        Serial.println("");
//      }
//    }
//  }

  returnsCache[strobeNum] = portInput;
  setRiotInputs(0, RIOTPORT_A, portInput); // returns
  setRiotInputs(1, RIOTPORT_A, slamSw ? 0x80 : 0); // slam switch
}
*/

// ISR
void onSlamChanged() {
  if(slamIntCount++ == 0) return; // ignore first occurrence
  slamSw = (digitalRead(SLAM_PIN) == LOW);
  setRiotInputs(1, RIOTPORT_A, slamSw ? 0x80 : 0); // may yield 6502 IRQ
}

uint16_t getOutpCount(byte n) {
  uint16_t c = outpCount[n];
  outpCount[n] = 0;
  return c;
}
