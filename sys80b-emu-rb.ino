// Gottileb System 80/B emulator for Arduino DUE

//#include <SD.h>
#include <SPI.h>
#include <LiquidCrystal.h>
#include "memory.h"
#include "global.h"
#include "emu6502.h"
#include "riots.h"
#include "interface.h"

#define BAUDRATE 57600
#define BATCH_CLK_COUNT 50000
#define INFO_DELAY 1000

#define LCD_COLS 16
#define LCD_ROWS 2

LiquidCrystal lcd(LCD_RS_PIN, LCD_EN_PIN, LCD_D4_PIN, LCD_D5_PIN, LCD_D6_PIN, LCD_D7_PIN);

bool slamSwPushed = false;
bool resetPushed = false;

// convenience Due's hardware interrupt flags:
// bit 0 set = DUE's TC3 timer irq = RIOT #0 timer event
// bit 1 set = DUE's TC4 timer irq = RIOT #1 timer event
// bit 2 set = DUE's TC5 timer irq = RIOT #2 timer event
// bit 3 set = DUE's slam switch interrupt
volatile byte dueIrq; 

// routines/functions
void debugLedsOutput(byte val);
void lcdprn(const char *st, byte line);
void onDueIrq();

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
  
  //debugLedsOutput(1);
  
  // Serial COM init
  Serial.begin(BAUDRATE);
  Serial.println(F("Starting..."));
    
  // LCD display init
  lcd.begin(LCD_COLS, LCD_ROWS);
  lcd.clear();
  lcd.noCursor();
  lcdprn("Starting...", 0);

  // SD init
  if (!initSD()) infiniteLoop();
  
  // Sys80b RAM/ROM setup
  Serial.println(F("RAM/ROM setup..."));
  if (!initMemory()) infiniteLoop();
  
  Serial.println(F("RIOTs reset..."));
  resetRIOTs();
  //debugLedsOutput(4);
  
  Serial.println(F("Interface init..."));
  initInterface();
  //debugLedsOutput(7);
  
  Serial.println(F("Timers init..."));
  initTimers();
  //debugLedsOutput(8);
  
  // 6502 reset
  resetCPU();
  Serial.print(F("CPU start at: $"));
  Serial.println(PC, HEX);
  Serial.print(F("IRQ vector at: $"));
  uint16_t ad = FETCHW(IRQ_VECTOR);
  Serial.println(ad, HEX);

  /*
  // GPIO test
  while(1) {
    Serial.println(F("Testing GPIO..."));
    testInterface();
    Serial.println(F("end of test."));
  }
  */

  Serial.println(F("Go..."));
  lcd.clear();
  lcdprn("Running...", 0);
  //debugLedsOutput(0);
}

void loop() {
  int i;
  long iCount = 0;
  long cCount = 0;
  long startT;
  uint16_t prevAdr = 0;
  //char ch;
  String cmd;
  
  startT = millis();
  
  if (!stopExecution) {
    // following code will be executed periodically ----------
    
    //irqCalls = irqCount;
    //irqCalls = 0;
    irqCount = 0;
     
    // CPU emulation cycle
    while (millis() - startT < INFO_DELAY) {
      iCount += execBatch(BATCH_CLK_COUNT);
      cCount += BATCH_CLK_COUNT;
    }
    
    // non-volatile RAM updates
    if (ramWrites) {
      saveRAM();
      Serial.print(ramWrites);
      Serial.println(F(" RAM bytes writes."));
      ramWrites = 0;
    }

    if (slamSw && !slamSwPushed) { // active LOW
      Serial.println(F("Slam Switch closed!"));
      slamSwPushed = true;
    }
    else if (!slamSw && slamSwPushed) {
      Serial.println(F("Slam Switch released"));
      slamSwPushed = false;
    }
    
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
    //Serial.print(irqCount - irqCalls);
    Serial.print(irqCount);
    Serial.print(F(" IRQ - adr: $"));
    Serial.println(iADR, HEX);
/*
    // RIOTs stats output
    Serial.print(F("RIOTs IRQ reqs: "));
    for (i=0; i<3; i++) {
      Serial.print(getRiotStats(i, RIOT_IRQ_COUNT));
      if (i == 2) Serial.print(" - "); 
      else Serial.print(", ");
    }
    Serial.print(F("RIOTs IRQ collisions: "));
    for (i=0; i<3; i++) {
      Serial.print(getRiotStats(i, RIOT_IRQ_COLL));
      if (i == 2) {
          Serial.print(" irqRequest="); 
          Serial.println(irqRequest);
      }
      else Serial.print(", ");
    }
    Serial.print(F("RIOTs timer writes: "));
    for (i=0; i<3; i++) {
      Serial.print(getRiotStats(i, RIOT_TMR_WRTS));
      if (i == 2) Serial.print(" - "); 
      else Serial.print(", ");
    }
*/
    
    Serial.print(F("RIOTs reads: "));
    for (i=0; i<3; i++) {
      Serial.print(i);
      Serial.print(riotName[i]);
      Serial.print(": (A=");
      Serial.print(getRiotStats(i, RIOT_PA_READS));
      Serial.print(",B=");
      Serial.print(getRiotStats(i, RIOT_PB_READS));
      if (i == 2) Serial.println(")"); else Serial.print("), ");
    }
    
    Serial.print(F("RIOTs writes: "));
    for (i=0; i<3; i++) {
      Serial.print(i);
      Serial.print(riotName[i]);
      Serial.print(": (A=");
      Serial.print(getRiotStats(i, RIOT_PA_WRTS));
      Serial.print(",B=");
      Serial.print(getRiotStats(i, RIOT_PB_WRTS));
      if (i == 2) Serial.println(")"); else Serial.print("), ");
    }
/*
    Serial.print(F("slamSwitch interrupts: "));
    Serial.println(slamIntCount);
*/
    // outputs stats
    Serial.print(F("Output changes: "));
    for (i=0; i<5; i++) {
      Serial.print(outpName[i]);
      Serial.print(":");
      Serial.print(getOutpCount(i));
      if (i == 4) Serial.println(""); 
      else Serial.print(", ");
    }
/*
    // returns cache
    Serial.print(F("Returns cache: ["));
    for (i=0; i<8; i++) {
      Serial.print(returnsCache[i]);
      if (i == 7) Serial.println("]"); 
      else Serial.print(", ");
    }
*/
    // display output
    printDisplays();
    
    // CPU lock detection
    if (iADR == prevAdr) {
      Serial.print(F("Possible CPU infinite loop?"));
      dumpHistory();
    }
    prevAdr = iADR;

    // escape key
    for (i=0; i<8; i++) forcedReturn[i] = 0; // resets forced returns
    if (SERIALINP && Serial.available() > 0) {
      Serial.println(F(">Serial input detected..."));
      cmd = Serial.readString();
      if (cmd.equals("\\")) { // STOPS EXECUTION
        stopExecution = true;
        Serial.print(F("timers short delay events: "));
        for (i=0; i<3; i++) {
          Serial.print(shortTimerDelayCount[i]);
          if (i == 2) Serial.println(""); else Serial.print(", ");
        }
        saveRAM();
        Serial.println(F("*** EXECUTION STOPPED ***"));
      }
      else {
        byte sw;
        sw = cmd.toInt();
        if (sw > 0) {
          forcedReturn[sw/10+1] = (byte)(1<<(sw%10));
          Serial.print(F("Forced return: "));
          Serial.print(sw%10);
          Serial.print(F(" on strobe "));
          Serial.println(sw/10);
        }
      }
    }
    
  }
  else infiniteLoop(); // CPU execution stop
}

void debugLedsOutput(byte val) {
  digitalWrite(DBG0_PIN, (val & 1) ? HIGH : LOW);
  digitalWrite(DBG1_PIN, (val & 2) ? HIGH : LOW);
  digitalWrite(DBG2_PIN, (val & 4) ? HIGH : LOW);
  digitalWrite(DBG3_PIN, (val & 8) ? HIGH : LOW);
}

void lcdprn(const char *st, byte line) {
  lcd.setCursor(0, line);
  lcd.print(st);
}

// to call after an hardware interrupt event (ISR) -
// updates Sys80 state after a DUE interrupt.
// At least one of the 4 LS bits of dueIrq shoud be set
void onDueIrq() {
  byte id; // RIOT id
  byte irqBit;

  noInterrupts();
  irqBit = 1;
  if (dueIrq & 0b0111) { // RIOT timer interrupt
    for (id=0; id<3; id++) { 
      if (dueIrq & irqBit) { 
        riot[id].zeroTime = 0xff;
        set_TMR_INTF(id);
        dueIrq &= (byte)~irqBit;
      }
      irqBit <<= 1;
    }
  }
  if (dueIrq & 0b1000) { // slam switch changed
    slamSw = (digitalRead(SLAM_PIN) == LOW);
    setRiotInputs(1, RIOTPORT_A, slamSw ? 0x80 : 0); // may yield 6502 IRQ
    dueIrq &= 0b11110111;
  }
  interrupts();
}
