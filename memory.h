#include "sdio.h"

// NOTE: can't use direct EEPROM memory use as RAM due to time constraints

#define PROMFILE "PROM1.CPU"
#define PROMADR 0x2000
#define PROMSIZE 8192
#define ROMFILE "PROM2.CPU"
#define ROMADR 0x1000
#define ROMSIZE 4096
#define RAMFILE "RAMDATA.BIN"
#define RAMSIZE 256
#define MMASK 0x3fff // System 80/b memory mask (A14 & A15 not connected)

#define USE_SD_FOR_NVRAM true

#if USE_SD_FOR_NVRAM==false
  #include <EEPROM.h> // not available on Arduino DUE !
#endif 

// onboard ROM & RAM (except extern RIOTs' RAM)
byte ram[RAMSIZE];
PROGMEM byte rom[ROMSIZE + 1];
PROGMEM byte prom[PROMSIZE + 1];

uint16_t ramWrites = 0;

bool initMemory();
byte mread(uint16_t adr);
void mwrite(uint16_t adr, byte data);
bool loadRAM();
bool saveRAM();
void eraseRAM();
void loadRAMfromEEPROM();
void writeRAMtoEEPROM();

extern byte readRiot(uint16_t adr);
extern void writeRiot(uint16_t adr, byte dta);

bool initMemory() {
  byte err;

  eraseRAM(); // TEMPORARY
  if (!sdOk) {
    initSD();
    if (!sdOk) {
      Serial.println(F("initMemory(): can't use SD card!"));
      return false;
    }
  }
  err = 0;
  if (!loadROMfromSD(prom, PROMFILE, PROMSIZE)) err++; // loads ROM in prom[]
  if (!loadROMfromSD(rom, ROMFILE, ROMSIZE)) err++; // loads ROM in rom[]
  if (!loadRAM()) err++; // loads last RAM data in ram[]
  if (err) {
    Serial.println(F("initMemory(): troubles reading memory data from SD!"));
    return false;
  }
  return true;
}

byte mread(uint16_t adr) {
  uint16_t selector;
  
  selector = (adr & 0x3800) >> 11; // A11, A12, A13 address bits
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
      //return ram[adr & 0x00ff];
      return ram[adr];
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
  uint16_t selector;
  
  selector = (adr & 0x3800) >> 11; // A11, A12, A13 address bits
  if (selector == 0b000) writeRiot(adr, data);
  else if (selector == 0b011) {
    ram[adr & 0x00ff] = data; // writes data in RAM
    ramWrites++;
  }
  else {
    Serial.print(F("mwrite(): WARNING: Unhandled address $"));
    Serial.println(adr, HEX);
  }
}

bool loadRAM() {
  #if USE_SD_FOR_NVRAM==true
    return loadRAMfromSD(ram, RAMFILE, RAMSIZE); 
  #else
    loadRAMfromEEPROM(ram, RAMFILE);
    return true;
  #endif
}

bool saveRAM() {
  #if USE_SD_FOR_NVRAM==true
    return writeRAMtoSD(ram, RAMFILE, RAMSIZE);
  #else
    writeRAMtoEEPROM(ram, RAMFILE);
    return true;
  #endif
}

void eraseRAM() {
  for (uint16_t i=0; i<RAMSIZE; i++) ram[i]=0;
}

#if USE_SD_FOR_NVRAM==false

  // warning: requires milliseconds to complete
  void loadRAMfromEEPROM() {
    Serial.print(F("Reading RAM from EEPROM... "));
    for (uint16_t i=0; i<RAMSIZE; i++) ram[i] = EEPROM.read(i);
    Serial.println("ok."); 
  }
  
  // warning: requires milliseconds to complete
  void writeRAMtoEEPROM() {
    Serial.print(F("Writing RAM to EEPROM... "));
    for (uint16_t i=0; i<RAMSIZE; i++) EEPROM.update(i, ram[i]);
    Serial.println("ok.");
  }

#endif

