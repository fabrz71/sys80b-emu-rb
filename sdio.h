// microSD R/W functions

//#include <stdio.h>
#include <SD.h>
#include <SPI.h>
//#include "shared.h"

#define RAMSIZE 256

boolean sdOk;

boolean initSD();
void disableSD();
void printDir();
boolean loadROM(byte m[], char *fileName);
boolean loadRAM(byte m[], char *fileName);
boolean updateRAM(byte m[], char *fileName);

boolean initSD() {
  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(SD_CS_PIN, HIGH);
  sdOk = SD.begin(SD_CS_PIN);
  if (sdOk) Serial.println(F("SD initialized."));
  else Serial.println(F("SD init failed!"));
  return sdOk;
}

void disableSD() {
  digitalWrite(SD_CS_PIN, HIGH);
  Serial.println(F("SD disabled"));
}

void printDir() {
  Serial.println(F("SD content:"));
  if (!sdOk) Serial.println(F("\tSD is not OK"));
  else {
    File root = SD.open("/");
    if (!root) Serial.println(F("\t&lt;Error while reading SD card's root&gt;"));
    else {
      bool empty = true;
      for (;;) {
        File entry = root.openNextFile();
        if (! entry) { // no more files
          if (empty) Serial.println(F("\t&lt;SD is empty&gt;"));
          break;
        }
        empty = false;
        Serial.print('\t');
        Serial.println(entry.name());
        entry.close();
      }
    }
  }
}

boolean loadROM(byte m[], char *fileName) {
  File f;
  long i, fsz;
  word adr = 0;
  byte d;

  Serial.print(F("Loading ROM '"));
  Serial.print(fileName);
  Serial.print("' ... ");
  if (!SD.exists(fileName)) {
    Serial.println(F("Can't open file!"));
    return false;
  }
  f = SD.open(fileName);
  if (!f) {
    Serial.println(F("Troubles opening file!"));
    return false;
  }
  digitalWrite(RXLED_PIN, LOW);
  fsz = f.size();
  for (i = 0; i < fsz; i++) {
    d = f.read();
    m[adr++] = d;
  }
  f.close();
  digitalWrite(RXLED_PIN, HIGH);
  Serial.println();
  Serial.print(fsz);
  Serial.println(F(" bytes loaded."));
  return true;
}

// reads only the last 256 bytes of specified file
boolean loadRAM(byte m[], char *fileName) {
  File f;
  long i, fsz;
  
  Serial.print(F("Loading RAM '"));
  Serial.print(fileName);
  Serial.println("'... ");
  if (!SD.exists(fileName)) {
    Serial.println(F("Warning: File does not exist."));
    return false;
  }
  f = SD.open(fileName);
  if (!f) {
    Serial.println(F("Troubles opening file!"));
    return false;
  }
  digitalWrite(RXLED_PIN, LOW);
  fsz = f.size();
  if ((fsz % RAMSIZE) != 0) {
    Serial.print(F("Warning: RAM file '"));
    Serial.print(fileName);
    Serial.print(F("' size ("));
    Serial.print(fsz);
    Serial.print(F(") is not multiple of"));
    Serial.println(RAMSIZE);
  }
  if (fsz > RAMSIZE) f.seek(fsz-RAMSIZE-1);
  for (i = 0; i < RAMSIZE && i < fsz; i++) {
    m[i] = (char)f.read();
    if (SERIALOUT) {
      Serial.print(m[i], HEX);
      if (i%16) Serial.print(", "); 
      else Serial.println(", ");
    }
  }
  Serial.println("- ok.");
  f.close();
  digitalWrite(RXLED_PIN, HIGH);
  return true;
}

// RAMSIZE-bytes cumulative writing (file append)
boolean updateRAM(byte m[], char *fileName) {
  File f;
  long r;

  Serial.print(F("Updating RAM file... "));
  f = SD.open(fileName, FILE_WRITE);
  if (!f) {
    Serial.println(F("Troubles opening file!"));
    return false;
  }
  digitalWrite(TXLED_PIN, LOW);
  r = f.write(m, RAMSIZE);
  f.close();
  digitalWrite(TXLED_PIN, HIGH);
  if (r < RAMSIZE) {
    Serial.println(F("Troubles writing file!"));
    return false;
  }
  else Serial.println("ok.");
  return true;
}
