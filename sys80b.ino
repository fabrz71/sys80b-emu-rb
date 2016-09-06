// Gottileb System 80/B emulator for Arduino DUE

//#include <avr/pgmspace.h>
#include <SD.h>
#include <SPI.h>
#include <LiquidCrystal.h>
#include "global.h"
#include "emu6502.h"
#include "sdio.h"
#include "riots.h"
#include "interface.h"

#define BAUDRATE 57600
#define BATCH_CLK_COUNT 50000

//#define PROMFILE "PROM.BIN"
#define PROMFILE "PROM1.CPU"
#define PROMADR 0x2000
#define PROMSIZE 8192
#define ROMFILE "PROM2.CPU"
#define ROMADR 0x1000
#define ROMSIZE 4096
#define RAMFILE "RAMDATA.BIN"
#define MMASK 0x3fff // System 80/b memory mask (A14 & A15 not connected)

#define LCD_COLS 16
#define LCD_ROWS 2

LiquidCrystal lcd(LCD_RS_PIN, LCD_EN_PIN, LCD_D4_PIN, LCD_D5_PIN, LCD_D6_PIN, LCD_D7_PIN);

int irqCalls = 0;
bool ramMod = false;
int ramWrites = 0;
int slamSwPushed = false;
bool resetPushed = false;
//unsigned long process_start_t;

// external memory (except RIOTs' RAM)
byte ram[256];
PROGMEM byte rom[ROMSIZE + 1];
PROGMEM byte prom[PROMSIZE + 1];

// routines/functions
byte mread(uint16_t adr);
void mwrite(uint16_t adr, byte data);
void debugLedsOutput(byte val);
void lcdprn(char *st, byte line);

void setup() { 
  pinMode(LED_PIN, OUTPUT);
  pinMode(RXLED_PIN, OUTPUT);
  pinMode(TXLED_PIN, OUTPUT);
  pinMode(SLAM_PIN, INPUT_PULLUP);
  pinMode(AUXB_PIN, INPUT_PULLUP);
  pinMode(GPIO_SS_PIN, OUTPUT);
  pinMode(IOEN_PIN, OUTPUT);
  pinMode(IRQ_PIN, OUTPUT);
  //pinMode(NMI_PIN, OUTPUT);
  pinMode(DBG0_PIN, OUTPUT);
  pinMode(DBG1_PIN, OUTPUT);
  pinMode(DBG2_PIN, OUTPUT);
  pinMode(DBG3_PIN, OUTPUT);
  
  digitalWrite(LED_PIN, LOW);
  digitalWrite(RXLED_PIN, HIGH);
  digitalWrite(TXLED_PIN, HIGH);
  digitalWrite(IOEN_PIN, LOW);
  
  debugLedsOutput(1);
  
  // Serial COM init
  Serial.begin(BAUDRATE);
  Serial.println(F("Starting..."));
    
  // LCD display init
  lcd.begin(LCD_COLS, LCD_ROWS);
  lcd.clear();
  lcd.noCursor();
  lcdprn("Starting...", 0);
    
  // ROMs setup
  if (!initSD()) infiniteLoop();
  loadROM(prom, PROMFILE); // loads ROM in prom[]
  loadROM(rom, ROMFILE); // loads ROM in rom[]
  debugLedsOutput(2);
  loadRAM(ram, RAMFILE); // loads last RAM data in ram[]
  debugLedsOutput(3);
  //disableSD();

  Serial.println(F("RIOTs reset..."));
  resetRIOTs();
  debugLedsOutput(4);
  Serial.println(F("Interface init..."));
  initInterface();
  debugLedsOutput(7);
  Serial.println(F("Timers init..."));
  initTimers();
  debugLedsOutput(8);
  
  // 6502 reset
  resetCPU();
  Serial.print(F("CPU start at: $"));
  Serial.println(PC, HEX);
  Serial.print(F("IRQ vector at: $"));
  uint16_t ad = FETCHW(IRQ_VECTOR);
  Serial.println(ad, HEX);
  
  /*
  // GPIO test
  Serial.println(F("Testing GPIO..."));
  testInterface();
  Serial.println(F("end of test."));
  */
  
  Serial.println(F("Go..."));
  lcd.clear();
  lcdprn("Running...", 0);
  debugLedsOutput(0);
}

void loop() {
  int i;
  long iCount = 0;
  long cCount = 0;
  long startT;
  uint16_t prevAdr = 0;
  
  startT = millis();
  if (!stopExecution) {
    irqCalls = irqCount;
    
    // CPU emulation cycle
    while (millis() - startT < 1000) {
      iCount += execBatch(BATCH_CLK_COUNT);
      cCount += BATCH_CLK_COUNT;
    }
    
    // following code will be executed periodically ----------
    
    // non-volatile RAM updates
    if (ramMod) {
      updateRAM(ram, RAMFILE);
      //disableSD();
      Serial.print(ramWrites);
      Serial.println(F(" RAM bytes writes."));
      ramMod = false;
      ramWrites = 0;
    }

    if (digitalRead(SLAM_PIN) == LOW && !slamSwPushed) {
      Serial.println(F("Slam Switch pushed!"));
      slamSwPushed = true;
    }
    else slamSwPushed = false;
    
    // AUX button implementation
    if (digitalRead(AUXB_PIN) == LOW && !resetPushed) {
      Serial.println(F("CPU reset!"));
      resetCPU();
      resetPushed = true;
    }
    else resetPushed = false;
    
    // CPU stats output
    Serial.print(iCount);
    Serial.print(F(" istructions - "));
    Serial.print(cCount);
    Serial.print(F(" clock ticks (about) - "));
    Serial.print(irqCount - irqCalls);
    Serial.print(F(" IRQ proc'd - addr: $"));
    Serial.println(iADR, HEX);

    // RIOTs stats output
    Serial.print(F("RIOTs IRQ reqs: "));
    for (i=0; i<3; i++) {
      Serial.print(getRiotIrqCount(i));
      if (i == 2) Serial.print(" - "); 
      else Serial.print(", ");
    }
    Serial.print(F("RIOTs IRQ collisions: "));
    for (i=0; i<3; i++) {
      Serial.print(getRiotLostIrqCount(i));
      if (i == 2) Serial.println(""); 
      else Serial.print(", ");
    }
    Serial.print(F("RIOTs timer writes: "));
    for (i=0; i<3; i++) {
      Serial.print(getRiotTimerWrites(i));
      if (i == 2) Serial.print(" - "); 
      else Serial.print(", ");
    }
    Serial.print(F("RIOTs PIO writes: "));
    for (i=0; i<3; i++) {
      Serial.print(getRiotPioWrites(i));
      if (i == 2) Serial.println(""); 
      else Serial.print(", ");
    }

    // outputs stats
    Serial.print(F("Output changes: "));
    for (i=0; i<5; i++) {
      Serial.print(outpName[i]);
      Serial.print(":");
      Serial.print(getOutpCount(i));
      if (i == 4) Serial.println(""); 
      else Serial.print(", ");
    }

    // display output sample
    printDisplays();
    
    // CPU lock detection
    if (iADR == prevAdr) {
      Serial.print(F("Possible CPU infinite loop?"));
      dumpHistory();
    }
    prevAdr = iADR;
    
  }
  else infiniteLoop(); // CPU execution stop
}

byte mread(uint16_t adr) {
  uint16_t selector = (adr & 0x3800) >> 11; // A11, A12, A13 address bits
  switch (selector) {
    case 0b000: // RIOTs: $0000-$017F
      return readRiot(adr);
    case 0x001: // RIOTs mirroring
      return readRiot(adr & 0x3ff);
    case 0b010: // ROM (12 address bits used)
      // ROM: CPU A15 connected to ROM A11
      if ((adr & 0x8000) == 0) return rom[adr & 0x7ff]; // lower half case
      return rom[(adr & 0x7ff) | 0x0800]; // upper half case
    case 0b011: // RAM
      return ram[adr & 0x00ff];
    default: // PROM (13 address bits used)
      return prom[adr & 0x1fff];
  }
  // should never be reached
  Serial.print(F("WARNING: Unhandled address $"));
  Serial.println(adr, HEX);
  Serial.println(F("mread(): returns 0"));
  return 0;
}

void mwrite(uint16_t adr, byte data) {
  uint16_t selector = (adr & 0x3800) >> 11; // A11, A12, A13 address bits
  if (selector == 0b000) writeRiot(adr, data);
  else if (selector == 0b011) {
    ram[adr & 0x00ff] = data; // writes data in RAM
    ramWrites++;
    ramMod = true;
  }
  else {
    Serial.print(F("mwrite(): WARNING: Unhandled address $"));
    Serial.println(adr, HEX);
  }
}

void debugLedsOutput(byte val) {
  digitalWrite(DBG0_PIN, (val & 1) ? HIGH : LOW);
  digitalWrite(DBG1_PIN, (val & 2) ? HIGH : LOW);
  digitalWrite(DBG2_PIN, (val & 4) ? HIGH : LOW);
  digitalWrite(DBG3_PIN, (val & 8) ? HIGH : LOW);
}

void lcdprn(char *st, byte line) {
  lcd.setCursor(0, line);
  lcd.print(st);
}

