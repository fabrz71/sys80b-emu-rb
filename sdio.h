// microSD R/W functions

#include <SD.h>
#include <SPI.h>

boolean sdOk;
extern const byte SD_CS_PIN;
extern const byte RXLED_PIN;
extern const byte TXLED_PIN;
extern const bool SERIALOUT;

boolean initSD();
void disableSD();
void printSDdir();
boolean loadROMfromSD(byte m[], const char *fileName, uint16_t sz);
boolean loadRAMfromSD(byte m[], const char *fileName, uint16_t sz);
boolean writeRAMtoSD(byte m[], const char *fileName, uint16_t sz);

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

void printSDdir() {
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

boolean loadROMfromSD(byte m[], const char *fileName, uint16_t sz) {
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
boolean loadRAMfromSD(byte m[], const char *fileName, uint16_t sz) {
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
  if ((fsz % sz) != 0) {
    Serial.print(F("Warning: RAM file '"));
    Serial.print(fileName);
    Serial.print(F("' size ("));
    Serial.print(fsz);
    Serial.print(F(") is not multiple of"));
    Serial.println(sz);
  }
  if (fsz > sz) f.seek(fsz-sz);
  for (i = 0; i < sz && i < fsz; i++) {
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

// sz-bytes cumulative writing (file append)
boolean writeRAMtoSD(byte m[], const char *fileName, uint16_t sz) {
  File f;
  long r;

  Serial.print(F("Writing RAM to SD... "));
  f = SD.open(fileName, FILE_WRITE);
  if (!f) {
    Serial.println(F("Troubles opening file!"));
    return false;
  }
  digitalWrite(TXLED_PIN, LOW);
  r = f.write(m, sz);
  f.close();
  digitalWrite(TXLED_PIN, HIGH);
  if (r < sz) {
    Serial.println(F("Troubles writing file!"));
    return false;
  }
  else Serial.println("ok.");
  return true;
}
