// RIOTs vs. SYS80B-signals interface (glue-logic implementation)

#include "mcp_io.h"
#include "dispbuf.h"
#include "ledGrid.h"

#define RETURNS_PORT PIOC // inputs
#define STROBES_PORT PIOC // outputs
#define DISPLAY_PORT PIOD // outputs
#define RETURNS_LSB_POS 1
#define STROBES_LSB_POS 12

// messages output by I/O type
//const bool inMsgEn[] = { false, false };
//const bool outMsgEn[] = { false, false, true, false, false };

const unsigned int RETURNS_bitmask = ((unsigned int)0x00ff) << RETURNS_LSB_POS;
const unsigned int STROBES_bitmask = ((unsigned int)0x00ff) << STROBES_LSB_POS;
//const unsigned int DISPLAY_bitmask = ((unsigned int)0x07ff) << DISPLAY_LSB_POS;
const unsigned int DISPLAY_bitmask = 0x07ffu;

// 16 bit mux outputs
PROGMEM const uint16_t mux16[16] =
  { 0x0001, 0x0002, 0x0004, 0x0008, 0x0010, 0x0020, 0x0040, 0x0080,
    0x0100, 0x0200, 0x0400, 0x0800, 0x1000, 0x2000, 0x4000, 0x8000 };

// 8 bit demux output
PROGMEM const byte demux[256] = 
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
byte opSwtch[] = { 0b00000011, 0b00000111, 0b00000001, 0b11000010 };

// System I/O signals
byte strobesN = 0xff; // outputs
byte returnsN = 0xff; // inputs
uint16_t solenoids; // outputs
//bool solenoid9; // output
byte sound; // outputs
byte lampCtrl; // outputs
uint16_t lampStrobes; // outputs
//byte displayData; // outputs: D0-D7 display data bus
volatile bool slamSw; // input
volatile word outpCount[5] = {0, 0, 0, 0, 0};

// System interface logic vars
byte displayLatch; // display D0-D7 data latch
byte displayId, prevDid = 0; // 0: not enabled; 1: display1; 2: display2; 3: both displays
byte displayResetN;
byte lampGroup;
byte switchEnable, prevSwEnable;
byte switchSel;
uint16_t prevSol; // previous value of solenoids
byte prevSnd; // previous value of sound
byte returnsCache[] = {0, 0, 0, 0, 0, 0, 0, 0, 0}; // first value unused

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
word getOutpCount(byte n);

void initInterface() {
  RETURNS_PORT->PIO_ODR |= RETURNS_bitmask; // set port bits 1-8 as INPUT (Returns)
  STROBES_PORT->PIO_OER |= STROBES_bitmask; // set port bits 12-19 as OUTPUT (Strobes)
  DISPLAY_PORT->PIO_OER |= DISPLAY_bitmask; // set port bits 0-10 as OUTPUT (Display)
  //pinMode(12, OUTPUT); // patch for display outputs!
  pinMode(14, OUTPUT); // patch for display outputs!
  pinMode(15, OUTPUT); // patch for display outputs!
  initGPIO();
  debugLedsOutput(5);
  readSysReturns();
  debugLedsOutput(6);
  //initLedGrid(LG_CS_PIN, LG_LOAD_PIN);
  initLedGrid(LG_CS_PIN);
  attachInterrupt(SLAM_PIN, onSlamChanged, CHANGE);
}

void testInterface() {
  int i;
  int p;
  uint16_t w;
  
  // MCP23S17s GPIO outputs
  for (p=0; p<2; p++) {
    w = 1;
    for (i=0; i<16; i++) {
      mcpWrite(p, w);
      w <<= 1;
      delay(100);
    }
    mcpWrite(p, 0);
  }
  // DUE's display outputs
  w = 1;
  for (i=0; i<12; i++) {
    DISPLAY_PORT->PIO_SODR = (uint32_t)w; // sets 1 bits
    DISPLAY_PORT->PIO_CODR = ~((uint32_t)w) & DISPLAY_bitmask; // clears 0 bits
    w <<= 1;
    delay(100);
  }
  // DUE's strobes outputs
  w = 1;
  for (i=0; i<8; i++) {
    STROBES_PORT->PIO_SODR = ((uint32_t)w) << STROBES_LSB_POS; // sets 1 bits
    STROBES_PORT->PIO_CODR = (((uint32_t)~w) << STROBES_LSB_POS) & STROBES_bitmask; // clears 0 bits
    w <<= 1;
    delay(100);
  }
}

// MCP23S17 GPIO ports setup
void initGPIO() {
  MCP_init(GPIO_SS_PIN);
  wordWrite(0, IODIRA, 0x0000); // all 16 pins set as output
  wordWrite(1, IODIRA, 0x0000); // all 16 pins set as output
  mcpWrite(0, 0); // resets MCP0 outputs
  mcpWrite(1, 0); // resets MCP1 outputs
}

// update system output signals variables upon RIOTs' output ports changes
void onRiotOutputChanged(byte riotId, byte portChanged, byte portA, byte portB) {
  switch (riotId) {
    case 0: // RIOT 1: switch matrix
      if (portChanged == RIOTPORT_B) { // port B changed
        strobesN = ~portB & 0xff; // Z11, Z12 outputs
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
      else { // port B: display half-byte (4 bits), LD (2 bits), display reset, switch enable
        displayId = (1 - bitRead(portB, 4)) + 2*(1 - bitRead(portB, 5));
        displayResetN = 1 - bitRead(portB, 6);
        dispatchOutputData(DISPL);
        switchEnable = 1 - bitRead(portB, 7);
        if (switchEnable != prevSwEnable) { // changed
          //updateRIOTsInput();
          readSysReturns();
          prevSwEnable = switchEnable;
        }
      }
      break;
    case 2: // RIOT 3: solenoids & sound
      if (portChanged == RIOTPORT_A) { // port A changed
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
        sound = bitRead(portA, 4) ? 0 : ~portA & 0x0f;
        dispatchOutputData(SOUND);
      }
      else { // port B: lamps control
        lampCtrl = portB & 0x0f;
        lampGroup = (portB >> 4) & 0x0f;
        lampStrobes = (mux16[lampGroup] & 0x1fff ) >> 1;
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
      if (ctrlw) {
        Serial.print(F("Solenoids: "));
        Serial.println(solenoids, BIN);
      }
      break;
    case SOUND:
      mcpWritePB(0, sound | ((solenoids & 0x0100)>>4));
      if (prevSnd ^ sound) {
        Serial.print(F("Sound: "));
        Serial.println(sound);
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

// read only system (DUE's) returns and updates returnsN and RIOT-0 inputs
void readSysReturns() {
  byte portInput;
  byte strobeNum;
  
  returnsN = (byte)(((RETURNS_PORT->PIO_PDSR & RETURNS_bitmask) >> RETURNS_LSB_POS) & 0xff);
  strobeNum = demux[(byte)~strobesN];
  if (switchEnable == 0) {
    portInput = (byte)~returnsN; // normal mode  
    if (returnsCache[strobeNum] != portInput) {
    Serial.print("strobe: ");
    Serial.print((byte)~strobesN);
    Serial.print("->");
    Serial.print(strobeNum);
    Serial.print(": ");
    Serial.println(portInput);
    }
  }
  else portInput = opSwtch[switchSel]; // "operator adjustables" read mode
  returnsCache[strobeNum] = portInput;
  setRiotInputs(0, RIOTPORT_A, portInput); // returns
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

// update RIOTs' input ports upon input signals change
// ( called by readSysInput() )
void updateRIOTsInput() {  
  byte portInput;
  byte strobeNum = demux[(byte)~strobesN];
  
  if (switchEnable == 0) portInput = (byte)~returnsN; // normal mode
  else portInput = opSwtch[switchSel]; // "operator adjustables" read mode
  /*
  else { // normal mode: detects changes
    byte retChange;
    portInput = ~returnsN;
    if (strobeNum > 0) {
      retChange = returnsCache[strobeNum] ^ portInput;
      if (retChange) { // returns on strobe line changed
        Serial.print(F("Matrix S"));
        Serial.print(strobeNum);
        Serial.print(": ");
        for (int i=0; i<8; i++) {
          if (retChange && bit(i)) {
            Serial.print("R");
            Serial.print(i);
            Serial.print("=");
            Serial.print((portInput && bit(i)) ? 1 : 0);
            Serial.print(" ");
          }
        }
        Serial.println("");
      }
    }
  }
  */
  returnsCache[strobeNum] = portInput;
  setRiotInputs(0, RIOTPORT_A, portInput); // returns
  setRiotInputs(1, RIOTPORT_A, slamSw ? 0x80 : 0); // slam switch  
}

// ISR
void onSlamChanged() {
  slamSw = (digitalRead(SLAM_PIN) == LOW);
  setRiotInputs(1, RIOTPORT_A, slamSw ? 0x80 : 0); // may yield 6502 IRQ
}

word getOutpCount(byte n) {
  word c = outpCount[n];
  outpCount[n] = 0;
  return c;
}
