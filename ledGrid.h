// MAX7219 LED GRID driver functions

#define CMD_DECODE  0x9
#define CMD_INTNSTY 0xa
#define CMD_DIGITS  0xb
#define CMD_SHUTDWN 0xc
//#define _USE_LOAD

int cs_pin; // SPI CS signal
int load_pin; // MAX7219 LOAD signal
extern const byte SPI_CLK_DIV;

void initLedGrid(int cspin, int loadpin);
void writeCmd(byte adr, byte data);
void setLedRow(byte row, byte data);
void shutDownMode(bool shut);
void setIntensity(byte i);

void initLedGrid(int cspin, int loadpin) {
  cs_pin = cspin;
  load_pin = loadpin;
  SPI.begin(cs_pin);
  SPI.setClockDivider(cs_pin, SPI_CLK_DIV);
  SPI.setBitOrder(cs_pin, MSBFIRST);
  SPI.setDataMode(cs_pin, SPI_MODE0);
  digitalWrite(load_pin, LOW);
  writeCmd(CMD_DECODE, 0); // Decode mode off
  writeCmd(CMD_DIGITS, 7); // 7 rows ("digits")
  shutDownMode(false); // Shutdown mode off
  setIntensity(7);
}

void writeCmd(byte adr, byte data) {
  SPI.setDataMode(cs_pin, SPI_MODE0);
  digitalWrite(cs_pin, LOW);
  SPI.transfer(adr & 0xf);
  SPI.transfer(data);
  digitalWrite(cs_pin, HIGH);
  #ifdef _USE_LOAD
    digitalWrite(load_pin, HIGH);
    delay(1);
    digitalWrite(load_pin, LOW);
  #endif
}

void setLedRow(byte row, byte data) {
  writeCmd((row & 0x7)+1, data);
}

void shutDownMode(bool shut) {
  writeCmd(CMD_SHUTDWN, shut ? 0 : 1);
}

void setIntensity(byte i) {
  writeCmd(CMD_INTNSTY, i & 0xf);
}

