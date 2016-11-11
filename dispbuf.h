// display buffer

#define DISP_DBGOUT false
#define MAX_DIGIT_COUNT 20 // number of alhpanumeric chars for each line

enum alt_mode { NORMAL, BLANK, REVERSE };

volatile char display1[MAX_DIGIT_COUNT+1];
volatile char display2[MAX_DIGIT_COUNT+1];
alt_mode modeSel; // alternative mode (enabled by bit 7)
bool controlDataMode; // true when receiving control data 
byte dta; // input byte on arduino port (pins 0-7)
byte digitCount; // common to both displays
byte brightness; // value: 0-63
bool displayEnabled; // allow display refresh
bool ld1, ld2; // data trasmission signals
byte d1i, d2i; // display char index

void pulseLD(byte ld, byte dd);
void resetDisplays();
void clearDisplayBuffers();
void exeControlCode(byte cc);
void printDisplays();

void resetDisplays() {
  displayEnabled = false;
  dta = 0;
  brightness = 0;
  digitCount = MAX_DIGIT_COUNT;
  modeSel = NORMAL;
  controlDataMode = false;
  clearDisplayBuffers();
  ld1 = false;
  ld2 = false;
}

void clearDisplayBuffers() {
  int i;
  
  for (i=0; i < MAX_DIGIT_COUNT; i++) {
    display1[i] = 0x20;
    display2[i] = 0x20;
  }
  display1[digitCount] = 0;
  display2[digitCount] = 0;
  d1i = 0;
  d2i = 0;
}

void pulseLD(byte ld, byte dd) {
  char ch;
  
  if (DISP_DBGOUT) {
    Serial.print(F("pulseLD("));
    Serial.print(ld);
    Serial.print(";");
    Serial.print(dd);
    Serial.println(")");
  }
  ld1 = (ld & 1) ? true : false;
  ld2 = (ld & 2) ? true : false;
  dta = dd;
  
  // CONTROL MODE
  if (controlDataMode) {
    exeControlCode(dta);
    return;
  }
  
  // NORMAL MODE
  if (dta == 0x01) { // enters control mode
    controlDataMode = true; 
    return;
  }
  // updates display buffer
  ch = (char)(dta & 0x7f);
  if (ch == 0) ch = ' ';
  else if ((dta & 0x80) != 0 && modeSel == BLANK) ch = '_';
  if (ld1) { // row 1 update
    display1[d1i++] = ch;
    if (d1i >= digitCount) d1i = 0;
    //ld1 = false;
  }
  if (ld2) { // row 2 update
    display2[d2i++] = ch;
    if (d2i >= digitCount) d2i = 0;
    //ld2 = false;
  }
}

void exeControlCode(byte cc) {
    controlDataMode = false;
    if (cc >= 0x40 && cc <= 0x7f) { // brightness
      brightness = cc & 0x3f; // (0-63)
      if (DISP_DBGOUT) {
        Serial.print(F("B:"));
        Serial.println(brightness);
      }
      return;
    }
    if (cc >= 0x80 && cc <= 0x9f) { // number of digits
      if (DISP_DBGOUT) Serial.print(F("D:"));
      if (cc == 0x80) cc = 0xa0;
      digitCount = cc - 0x80;
      display1[digitCount] = 0;
      display2[digitCount] = 0;
      d1i = 0;
      d2i = 0;
      if (DISP_DBGOUT) Serial.println(digitCount);
      return;
    }
    if (cc >= 0xc0 && cc <= 0xd3) { // cursor position
      if (ld1) {
        d1i = cc - 0xc0;
        if (d1i >= digitCount) d1i = 0;
        //ld1 = false;
      }
      if (ld2) {
        d2i = cc - 0xc0;
        if (d2i >= digitCount) d2i = 0;
        //ld2 = false;
      }
      return;
    }
    if (cc == 0x08) { // mode selection: normal
      modeSel = NORMAL;
      return;
    }
    if (cc == 0x09) { // mode selection: blank
      modeSel = BLANK;
      return;
    }
    if (cc == 0x0a) { // mode selection: reverse
      modeSel = REVERSE;
      return;
    }
    if (cc == 0x0e) { // start display refresh cycle
      displayEnabled = true;
      return;
    }
    if (cc >= 0x05 && cc <= 0x07) { // digit time
      if (DISP_DBGOUT) {
        Serial.print(F("DT:"));
        if (cc == 5) Serial.println(16);
        else if (cc == 6) Serial.println(32);
        else Serial.println(64);
      }
      return;
    }
    if (cc == 0x01) { // write char 0x01
      if (ld1) { // row 1 update
        display1[d1i++] = 0x01;
        if (d1i >= digitCount) d1i = 0;
      }
      if (ld2) { // row 2 update
        display2[d2i++] = 0x01;
        if (d2i >= digitCount) d2i = 0;
      }
      return;
    }
    if (DISP_DBGOUT) {
      Serial.print(F("W0:"));
      Serial.println(cc, HEX);
    }
}

void printDisplays() {
  Serial.print(F("D1:'"));
  Serial.print((const char *)display1);
  Serial.println("'");
  Serial.print(F("D2:'"));
  Serial.print((const char *)display2);
  Serial.println("'");  
}

