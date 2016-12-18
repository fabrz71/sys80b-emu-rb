// 6502 emulator module -
// does not implements 6502 bugs

#include "dasm6502.h"

//typedef uint8_t byte

// emulator constants
#define C 1   // carry
#define Z 2   // zero
#define I 4   // interrupt disabled
#define D 8   // decimal mode
#define B 16  // break
#define Un 32 // unused
#define V 64  // overflow
#define N 128 // negative
#define NMI_VECTOR 0xfffa
#define RST_VECTOR 0xfffc
#define IRQ_VECTOR 0xfffe

//#define _LOCK_DETECTION
#define _IRQ_ENABLED
//#define _NMI_ENABLED
//#define _IRQNMI_OUTPUT // led output
#define PC_STREAM_MEM 64 // number of last-executed istructions to keep track of

// CPU registers
byte A, X, Y; // 8-bit regs
byte S; // 8-bit stack pointer (base: 0x0100)
uint16_t PC; // 16-bit Program Counter
byte P; // 8-bit state register NV-BDIZC
byte IR; // 8-bit Istruction Register

// utility variables
byte data; // temporary byte register
//byte ABH, ABL; // address bus high/low bytes
uint16_t tmpw; // temporary word register
uint16_t AB; // 16-bit adress bus
uint16_t iADR; // istruction adress
boolean stopExecution = false;
uint16_t lastiadr[PC_STREAM_MEM + 1];
uint16_t iadri = 0;
byte carry; // carry (ADC, SBC)
uint16_t oldSP; // old Stack Pointer
byte illegalOp = 0;
volatile byte irqRequest;
volatile byte nmiRequest;
volatile byte prevNmiReq; // previous nmiRequest
bool nmiRunning; // used for NMI priority over IRQ
uint16_t irqCount = 0;
uint16_t nmiCount = 0;
uint32_t iCount; // executed istructions count for each batch

// external variables
extern volatile byte dueIrq; // (main code)
extern const byte IRQ_PIN; // global.h
extern const byte NMI_PIN; // global.h

PROGMEM const byte clkTicks[256] =
{ 7, 6, 2, 8, 3, 3, 5, 5, 3, 2, 2, 2, 4, 4, 6, 6, 2, 5, 2, 8, 4, 4, 6, 6, 2, 4, 2, 7, 5, 5, 7, 7,
  6, 6, 2, 8, 3, 3, 5, 5, 4, 2, 2, 2, 4, 4, 6, 6, 2, 5, 2, 8, 4, 4, 6, 6, 2, 4, 2, 7, 5, 5, 7, 7,
  6, 6, 2, 8, 3, 3, 5, 5, 3, 2, 2, 2, 3, 4, 6, 6, 2, 5, 2, 8, 4, 4, 6, 6, 2, 4, 2, 7, 5, 5, 7, 7,
  6, 6, 2, 8, 3, 3, 5, 5, 4, 2, 2, 2, 5, 4, 6, 6, 2, 5, 2, 8, 4, 4, 6, 6, 2, 4, 2, 7, 5, 5, 7, 7,
  2, 6, 2, 6, 3, 3, 3, 3, 2, 2, 2, 2, 4, 4, 4, 4, 2, 6, 2, 6, 4, 4, 4, 4, 2, 5, 2, 5, 5, 5, 5, 5,
  2, 6, 2, 6, 3, 3, 3, 3, 2, 2, 2, 2, 4, 4, 4, 4, 2, 5, 2, 5, 4, 4, 4, 4, 2, 4, 2, 5, 4, 4, 4, 4,
  2, 6, 2, 8, 3, 3, 5, 5, 2, 2, 2, 2, 4, 4, 6, 6, 2, 5, 2, 8, 4, 4, 6, 6, 2, 4, 2, 7, 5, 5, 7, 7,
  2, 6, 2, 8, 3, 3, 5, 5, 2, 2, 2, 2, 4, 4, 6, 6, 2, 5, 2, 8, 4, 4, 6, 6, 2, 4, 2, 7, 5, 5, 7, 7
};

// function macros
#define WORD(lo,hi) (((uint16_t)(hi))<<8 | ((uint16_t)(lo)))
#define HIBYTE(w) ( (byte) ((w & 0xff00) >> 8) )
#define LOBYTE(w) ( (byte) (w & 0x00ff) )
#define FETCH(w) mread(w)
#define FETCH_ZP(w) mread(w)
#define FETCHW(w) WORD(mread(w),mread((w)+1))
#define FETCHW_ZP(w) WORD(mread(w),mread((w)+1))
#define ISSET(b) ((P & (b)) > 0)

// macros without return value
#define reqIrq() irqRequest++
#define reqNmi() nmiRequest++
#define WRITE(w,b) mwrite(w,b)
#define WRITE_ZP(w,b) mwrite(w,b)
#define SET(b) P |= b
#define CLEAR(b) P &= ~b
//#define POP { POP_WARNING; data = FETCH(0x0100 + (++S)); }
//#define PUSH(b) { PUSH_WARNING; WRITE(0x0100 + (S--), b); }
#define POP data = FETCH(0x0100 + (++S))
#define PUSH(b) WRITE(0x0100 + (S--), b)
#define PUSHW(w) { PUSH(HIBYTE(w)); PUSH(LOBYTE(w)); }
//#define POPW { POP; AB = (uint16_t)data; POP; AB &= ((uint16_t)data<<8); }
#define SET_ABL(b) AB = (AB & 0xff00) | (uint16_t)(b)
#define SET_ABH(b) AB = (uint16_t)(b<<8) | (AB & 0x00ff)
#define UPDATE_Z(b) { if (b == 0) SET(Z); else CLEAR(Z); }
#define UPDATE_N(b) { if ((b & 0x80) == 0) CLEAR(N); else SET(N); }
#define UPDATE_NZ(b) { UPDATE_Z(b); UPDATE_N(b); }
#define UPDATE_V(o,r) { if ( ((r) ^ (uint16_t)A) & ((r) ^ (uint16_t)(o)) & 0x0080 ) SET(V); else CLEAR(V); }

/*
#define POP_WARNING if (S == 0xff) { \
    Serial.println("WARNING: POP on empty stack!"); \
    printState(); \
  }

#define PUSH_WARNING if (S == 0x00) { \
    Serial.println("WARNING: Stack overflow!"); \
    printState(); \
  }
*/

// ADDRESS MODES HIGH-LEVEL MACROS
// immediate : #bb
#define GET_IMM { data = FETCH(PC++); }
// absolute : $wwww
#define GET_ABS { \
    SET_ABL(FETCH(PC++)); \
    SET_ABH(FETCH(PC++)); \
    data = FETCH(AB); \
  }
// X-indexed absolute : $wwww,X
#define GET_ABSX { \
    SET_ABL(FETCH(PC++)); \
    SET_ABH(FETCH(PC++)); \
    AB += X; \
    data = FETCH(AB); \
  }
// Y-indexed absolute : $wwww,Y
#define GET_ABSY { \
    SET_ABL(FETCH(PC++)); \
    SET_ABH(FETCH(PC++)); \
    AB += Y; \
    data = FETCH(AB); \
  }
// zero page : $bb
#define GET_ZP { \
    AB = (uint16_t)FETCH_ZP(PC++); \
    data = FETCH(AB); \
  }
// X-indexed zero page : $bb,X
#define GET_ZPX { \
    data = FETCH(PC++); \
    AB = (uint16_t)(data + X) & 0x00ff; \
    data = FETCH_ZP(AB); \
  }
// Y-indexed zero page : $bb,Y (LDX, STX istructions only)
#define GET_ZPY { \
    data = FETCH(PC++); \
    AB = (uint16_t)(data + Y) & 0x00ff; \
    data = FETCH_ZP(AB); \
  }
// X-indexed indirect (zero page) : ($bb,X)
#define GET_ZPIX { \
    data = FETCH(PC++); \
    data += X; \
    AB = FETCHW_ZP((uint16_t)data); \
    data = FETCH(AB); \
  }
// indirect Y-indexed (zero page) : ($bb),Y
#define GET_ZPIY { \
    data = FETCH(PC++); \
    AB = FETCHW_ZP((uint16_t)data); \
    AB += (uint16_t)Y; \
    data = FETCH(AB); \
  }
// indirect : ($wwww) (JMP istruction only)
#define GETW_ABS { \
    SET_ABL(FETCH(PC++)); \
    SET_ABH(FETCH(PC++)); \
  }
// relative : $bb
#define GETW_REL { \
    data = FETCH(PC++); \
    AB = (data <= 0x7f) ? PC + (uint16_t)data : PC - (uint16_t)((~(data-1))&0xff); \
  }
// indirect : ($wwww) (JMP istruction only)
#define GETW_IND { \
    SET_ABL(FETCH(PC++)); \
    SET_ABH(FETCH(PC++)); \
    AB = FETCH(AB); \
  }

// ISTRUCTION HIGH-LEVEL MACRO
#define ADDC { \
    if (!ISSET(D)) { \
      carry = ISSET(C) ? 1 : 0; \
      if ((int)A + (int)data + (int)carry > 255) SET(C); else CLEAR(C); \
      tmpi = ((A & 0x80) == 0) ? (int)A : -(int)((~A & 0xff)+1); \
      tmpi += ((data & 0x80) == 0) ? (int)data : -(int)((~data & 0xff)+1); \
      tmpi += (int)carry; \
      if (tmpi < -128 || tmpi > 127) SET(V); else CLEAR(V); \
      A = A + data + carry; \
      UPDATE_NZ(A); \
    } \
    else A = bcdADC(data); \
  }
#define AND { A &= data; UPDATE_NZ(A); }
#define ASL { \
    if ((data & 0x80) != 0) SET(C); else CLEAR(C); \
    data <<= 1; \
    UPDATE_NZ(data); \
  }
#define BIT { \
    if (A & data) CLEAR(Z); else SET(Z); \
    P = (data & (V|N)) | (P & ~(V|N)); \
  }
// ??? carry
#define CMP(r) { \
    if (r >= data) SET(C); else CLEAR(C); \
    data = r - data; \
    UPDATE_NZ(data); \
  }
#define DCR { --data; UPDATE_NZ(data); } // DEC
#define EOR { A ^= data; UPDATE_NZ(A); }
#define INC { ++data; UPDATE_NZ(data); }
#define LD(r) { r = data; UPDATE_NZ(r); }
#define LSR { \
    if ((data & 0x01) != 0) SET(C); else CLEAR(C); \
    data >>= 1; \
    UPDATE_NZ(data); \
  }
#define ORA { A |= data; UPDATE_NZ(A); }
#define ROL { \
    tmpw = (uint16_t)data; \
    tmpw <<= 1; \
    if (ISSET(C)) tmpw |= 0x01; \
    if ((tmpw & 0x100) == 0) CLEAR(C); else SET(C); \
    data = (byte)(tmpw & 0xff); \
    UPDATE_NZ(data); \
  }
#define ROR { \
    tmpw = (uint16_t)data; \
    if (ISSET(C)) tmpw |= 0x100; \
    if ((tmpw & 0x01) == 0) CLEAR(C); else SET(C); \
    tmpw >>= 1; \
    data = (byte)(tmpw & 0xff); \
    UPDATE_NZ(data); \
  }
#define SBC { \
    if (!ISSET(D)) { \
      carry = ISSET(C) ? 0 : 1; \
      if ((int)A - (int)data - (int)carry < 0) CLEAR(C); else SET(C); \
      tmpi = ((A & 0x80) == 0) ? (int)A : -(int)((~A & 0xff) + 1); \
      tmpi -= ((data & 0x80) == 0) ? (int)data : -(int)((~data & 0xff) + 1); \
      tmpi -= (int)carry; \
      if (tmpi < -128 || tmpi > 127) SET(V); else CLEAR(V); \
      A = A - data - carry; \
      UPDATE_NZ(A); \
    } \
    else A = bcdSBC(data); \
  }

// routines/functions
byte bcdADC(byte data);
byte bcdSBC(byte data);
void resetCPU();
void irqStart();
void nmiStart();
void printState();
void printStack();
long execBatch(long clkCount);
void dumpHistory();
void printCurrentIstruction();

// external functions
extern byte mread(uint16_t adr); // (main code)
extern void mwrite(uint16_t adr, byte data); // (main code)
extern void onDueIrq(); // (main code)

long execBatch(long clkCount) {
  unsigned long tCount = 0; // CPU clock ticks count
  unsigned long lastChkClk = 0; // used for periodic externat check-routine call
  uint16_t prevPC = 0; // previous istruction adress
  uint16_t tmpw;
  byte tmpb;
  int tmpi;

  iCount = 0;
  if (stopExecution) return iCount;

  do {
    IR = FETCH(PC);
    prevPC = PC;
    oldSP = S;
    iADR = PC++;

    switch (IR) { // istruction dispatch

      case 0x00: // BRK : software interrupt
        irqStart();
        SET(B);
//        PUSHW(PC + 1);
//        PUSH(P);
//        SET(I);
//        PC = FETCHW(IRQ_VECTOR);
        break;

      case 0x01: // ORA (zerop,X) : A <- A OR [(zerop + X)]
        GET_ZPIX;
        ORA;
        break;

      case 0x05: // ORA zerop : A <- A OR [zerop]
        GET_ZP;
        ORA;
        break;

      case 0x06: // ASL zerop : [zerop] <- LeftShift [zerop]
        GET_ZP;
        ASL;
        WRITE(AB, data);
        break;

      case 0x08: // PHP : STK <- P
        SET(B);
        PUSH(P);
        break;

      case 0x09: // ORA immed : A <- A OR immed
        GET_IMM;
        ORA;
        break;

      case 0x0a: // ASL : A <- LeftShift A
        data = A; // ??? optimizable
        ASL;
        A = data;
        break;

      case 0x0d: // ORA absol : A <- A OR [absol]
        GET_ABS;
        ORA;
        break;

      case 0x0e: // ASL absol : [absol] <- LeftShift [absol]
        GET_ABS;
        ASL;
        WRITE(AB, data);
        break;

      case 0x10: // BPL relat : branch on N=0
        GETW_REL;
        if (!ISSET(N)) PC = AB;
        break;

      case 0x11: // ORA (zp),Y : A <- A OR [(zp) + Y]
        GET_ZPIY;
        ORA;
        break;

      case 0x15: // ORA zp,X : A <- A OR [zerop + X]
        GET_ZPX;
        ORA;
        break;

      case 0x16: // ASL zp,X : [zp + X] <- LeftShift [zp + X]
        GET_ZPX;
        ASL;
        WRITE(AB, data);
        break;

      case 0x18: // CLC : flag C <- 0
        CLEAR(C);
        break;

      case 0x19: // ORA absol,Y : A <- A OR [absol + Y]
        GET_ABSY;
        ORA;
        break;

      case 0x1d: // ORA absol,X : A <- A OR [absol + X]
        GET_ABSX;
        ORA;
        break;

      case 0x1e: // ASL absol,X : [absol + X] <- LeftShift [absol + X]
        GET_ABSX;
        ASL;
        WRITE(AB, data);
        break;

      case 0x20: // JSR immed : jump to subroutine
        AB = iADR + 2;
        PUSHW(AB);
        GET_ABS;
        PC = AB;
        break;

      case 0x21: // AND (zp,X) : A <- A AND [(zp + X)]
        GET_ZPIX;
        AND;
        break;

      case 0x24: // BIT zerop
        GET_ZP;
        BIT;
        break;

      case 0x25: // AND zerop : A <- A AND [zerop]
        GET_ZP;
        AND;
        break;

      case 0x26: // ROL zerop : left rotation on [zerop]
        GET_ZP;
        ROL;
        WRITE(AB, data);
        break;

      case 0x28: // PLP : P <- STK
        POP;
        P = data | Un;
        //CLEAR(B);
        break;

      case 0x29: // AND immed : A <- A AND immed
        GET_IMM;
        AND;
        break;

      case 0x2a: // ROL : left rotation on A
        data = A;  // ??? optimizable
        ROL;
        A = data;
        break;

      case 0x2c: // BIT absol
        GET_ABS;
        BIT;
        break;

      case 0x2d: // AND absol : A <- A AND [absol]
        GET_ABS;
        AND;
        break;

      case 0x2e: // ROL absol : 9-bit left rotation on [zerop]
        // data <- [absol]
        GET_ABS;
        ROL;
        WRITE(AB, data);
        break;

      case 0x30: // BMI relat : branch on N=1
        GETW_REL;
        if (ISSET(N)) PC = AB;
        break;

      case 0x31: // AND (zp),Y : A <- A AND [(zp) + Y]
        GET_ZPIY;
        AND;
        break;

      case 0x35: // AND zp,X : A <- A AND [zp + X]
        GET_ZPX;
        AND;
        break;

      case 0x36: // ROL zp,X : left rotation on [zp + X]
        GET_ZPX;
        ROL;
        WRITE(AB, data);
        break;

      case 0x38: // SEC : flag C <- 1
        SET(C);
        break;

      case 0x39: // AND absol,Y : A <- A AND [absol + Y]
        GET_ABSY;
        AND;
        break;

      case 0x3d: // AND absol,X : A <- A AND [absol + X]
        GET_ABSX;
        AND;
        break;

      case 0x3e: // ROL absol,X :
        GET_ABSX;
        ROL;
        WRITE(AB, data);
        break;

      case 0x40: // RTI : return from Interrupt
        POP; P = data | Un;
        POP; SET_ABL(data);
        POP; SET_ABH(data);
        PC = AB;
        #ifdef _IRQNMI_OUTPUT
          if (nmiRunning) digitalWrite(NMI_PIN, LOW);
          else digitalWrite(IRQ_PIN, LOW);
        #endif
        nmiRunning = false;
        //nestedInterrupts--;
        break;

      case 0x41: // EOR (zp,X) : A <- A EOR [(zp + X)]
        GET_ZPIX;
        EOR;
        break;

      case 0x45: // EOR zerop : A <- A EOR [zerop]
        GET_ZP;
        EOR;
        break;

      case 0x46: // LSR zerop : [zerop] <- Rightshift [zerop]
        GET_ZP;
        LSR;
        WRITE(AB, data);
        break;

      case 0x48: // PHA : STK <- A
        PUSH(A);
        break;

      case 0x49: // EOR immed : A <- A EOR immed
        GET_IMM;
        EOR;
        break;

      case 0x4a: // LSR : A ->  Rightshift A
        data = A;  // ??? optimizable
        LSR;
        A = data;
        break;

      case 0x4c: // JMP absol : PCL <- [absol], PCH <- [absol+1]
        GETW_ABS;
        PC = AB;
        break;

      case 0x4d: // EOR absol : A <- A EOR [absol]
        GET_ABS;
        EOR;
        break;

      case 0x4e: // LSR absol : [absol] <- Rightshift [absol]
        GET_ABS;
        LSR;
        WRITE(AB, data);
        break;

      case 0x50: // BVC relat : branch on V=0
        GETW_REL;
        if (!ISSET(V)) PC = AB;
        break;

      case 0x51: // EOR (zp),Y : A <- A EOR [(zp) + Y]
        GET_ZPIY;
        EOR;
        break;

      case 0x55: // EOR zp,X : A <- A EOR [zp + X]
        GET_ZPX;
        EOR;
        break;

      case 0x56: // LSR zp,X : [zerop + X] <- Rightshift [zp + X]
        GET_ZPX;
        LSR;
        WRITE(AB, data);
        break;

      case 0x58: // CLI : flag_I <- 0
        CLEAR(I);
        break;

      case 0x59: // EOR absol,Y : A <- A EOR [absol + Y]
        GET_ABSY;
        EOR;
        break;

      case 0x5d: // EOR absol,X : A <- A EOR [absol + X]
        GET_ABSX;
        EOR;
        break;

      case 0x5e: // LSR absol,X : [absol + X] <- Rightshift [absol + X]
        GET_ABSX;
        LSR;
        WRITE(AB, data);
        break;

      case 0x60: // RTS : return from subroutine
        POP; SET_ABL(data);
        POP; SET_ABH(data);
        PC = AB + 1;
        break;

      case 0x61: // ADC (zp,X) : A <- A + [(zp + X)] + flag_C
        GET_ZPIX;
        ADDC;
        break;

      case 0x65: // ADC zp : A <- A + [zp] + flag_C
        GET_ZP;
        ADDC;
        break;

      case 0x66: // ROR zp : right rotation on [zp]
        GET_ZP;
        ROR;
        WRITE(AB, data);
        break;

      case 0x68: // PLA : A <- STK
        POP;
        A = data;
        UPDATE_NZ(A);
        break;

      case 0x69: // ADC immed : A <- A + immed + flag_C
        GET_IMM;
        ADDC;
        break;

      case 0x6a: // ROR : right rotation on A
        data = A; // ??? optimizable
        ROR;
        A = data;
        break;

      case 0x6c: // JMP (indir) : PCL <- [(indir)], PCH <- [(indir)+1]
        GETW_IND;
        PC = AB;
        break;

      case 0x6d: // ADC absol : A <- A + [absol] + flag_C
        GET_ABS;
        ADDC;
        break;

      case 0x6e: // ROR absol : right rotation on [absol]
        GET_ABS;
        ROR;
        WRITE(AB, data);
        break;

      case 0x70: // BVS relat : branch on V=1
        GETW_REL;
        if (ISSET(V)) PC = AB;
        break;

      case 0x71: // ADC (zp),Y : A <- A + [(zp) + Y] + flag_C
        GET_ZPIY;
        ADDC;
        break;

      case 0x75: // ADC zp,X : A <- A + [zp + X] + flag_C
        GET_ZPX;
        ADDC;
        break;

      case 0x76: // ROR zerop,X : 9-bit right rotation on [zerop + X]
        GET_ZPX;
        ROR;
        WRITE(AB, data);
        break;

      case 0x78: // SEI : flag_I <- 1
        SET(I);
        break;

      case 0x79: // ADC absol,Y : A <- A + [absol + Y] + flag_C
        GET_ABSY;
        ADDC;
        break;

      case 0x7d: // ADC absol,X : A <- A + [absol + X] + flag_C
        GET_ABSX;
        ADDC;
        break;

      case 0x7e: // ROR absol,X : 9-bit right rotation on [absol + X]
        GET_ABSX;
        ROR;
        WRITE(AB, data);
        break;

      case 0x81: // STA (zp, x) : [(zp + X)] <- A
        GET_ZPIX;
        WRITE(AB, A);
        break;

      case 0x84: // STY zerop : [zerop] <- Y
        GET_ZP;
        WRITE(AB, Y);
        break;

      case 0x85: // STA zerop : [zerop] <- A
        GET_ZP;
        WRITE(AB, A);
        break;

      case 0x86: // STX zerop : [zerop] <- X
        GET_ZP;
        WRITE(AB, X);
        break;

      case 0x88: // DEY : Y <- Y - 1
        Y--;
        UPDATE_NZ(Y);
        break;

      case 0x8a: // TXA : A <- X
        A = X;
        UPDATE_NZ(A);
        break;

      case 0x8c: // STY absol : [absol] <- Y
        GET_ABS;
        WRITE(AB, Y);
        break;

      case 0x8d: // STA absol : [absol] <- A
        GET_ABS;
        WRITE(AB, A);
        break;

      case 0x8e: // STX absol : [absol] <- X
        GET_ABS;
        WRITE(AB, X);
        break;

      case 0x90: // BCC relat : branch on C=0
        GETW_REL;
        if (!ISSET(C)) PC = AB;
        break;

      case 0x91: // STA (zp),Y : [(zp) + Y] <- A
        GET_ZPIY;
        WRITE(AB, A);
        break;

      case 0x94: // STY zerop,X : [zerop + X] <- Y
        GET_ZPX;
        WRITE(AB, Y);
        break;

      case 0x95: // STA zp,X : [zp + X] <- A
        GET_ZPX;
        WRITE(AB, A);
        break;

      case 0x96: // STX zp,Y : [zp + Y] <- X
        GET_ZPY;
        WRITE(AB, X);
        break;

      case 0x98: // TYA : A <- Y
        A = Y;
        UPDATE_NZ(A);
        break;

      case 0x99: // STA absol,Y : [absol + Y] <- A
        GET_ABSY;
        WRITE(AB, A);
        break;

      case 0x9a: // TXS : S <- X
        S = X;
        break;

      case 0x9d: // STA absol,X : [absol + X] <- A
        GET_ABSX;
        WRITE(AB, A);
        break;

      case 0xa0: // LDY immed : Y <- immed
        GET_IMM;
        LD(Y);
        break;

      case 0xa1: // LDA (zp,X) : A <- [(zp + X)]
        GET_ZPIX;
        LD(A);
        break;

      case 0xa2: // LDX immed : X <- immed
        GET_IMM;
        LD(X);
        break;

      case 0xa4: // LDY zerop : Y <- [zerop]
        GET_ZP;
        LD(Y);
        break;

      case 0xa5: // LDA zerop : A <- [zerop]
        GET_ZP;
        LD(A);
        break;

      case 0xa6: // LDX zerop : A <- [zerop]
        GET_ZP;
        LD(X);
        break;

      case 0xa8: // TAY : Y <- A
        Y = A;
        UPDATE_NZ(Y);
        break;

      case 0xa9: // LDA immed : A <- immed
        GET_IMM;
        LD(A);
        break;

      case 0xaa: // TAX : X <- A
        X = A;
        UPDATE_NZ(X);
        break;

      case 0xac: // LDY absol : Y <- [absol]
        GET_ABS;
        LD(Y);
        break;

      case 0xad: // LDA absol : A <- [absol]
        GET_ABS;
        LD(A);;
        break;

      case 0xae: // LDX absol : X <- [absol]
        GET_ABS;
        LD(X);
        break;

      case 0xb0: // BCS relat : branch on C=1
        GETW_REL;
        if (ISSET(C)) PC = AB;
        break;

      case 0xb1: // LDA (zp),Y : A <- [(zp) + Y]
        GET_ZPIY;
        LD(A);
        break;

      case 0xb4: // LDY zp, X : Y <- [zp + X]
        GET_ZPX;
        LD(Y);
        break;

      case 0xb5: // LDA zp, X : A <- [zp + X]
        GET_ZPX;
        LD(A);
        break;

      case 0xb6: // LDX zp, Y : X <- [zp + Y]
        GET_ZPY;
        LD(X);
        break;

      case 0xb8: // CLV : flag_V <- 0
        CLEAR(V);
        break;

      case 0xb9: // LDA absol,Y : A <- [absol, Y]
        GET_ABSY;
        LD(A);
        break;

      case 0xba: // TSX : X <- STK
        X = S;
        UPDATE_NZ(X);
        break;

      case 0xbc: // LDY absol,X : Y <- [absol + X]
        GET_ABSX;
        LD(Y);
        break;

      case 0xbd: // LDA absol,X : Y <- [absol + X]
        GET_ABSX;
        LD(A);
        break;

      case 0xbe: // LDX absol,Y : X <- [absol + Y]
        GET_ABSY;
        LD(X);
        break;

      case 0xc0: // CPY immed : flags_NZC << Y - immed
        GET_IMM;
        CMP(Y);
        break;

      case 0xc1: // CMP (zp,X) : flags_NZC << A - [(zp + X)]
        GET_ZPIX;
        CMP(A);
        break;

      case 0xc4: // CPY zerop : flags_NZC << Y - [zerop]
        GET_ZP;
        CMP(Y);
        break;

      case 0xc5: // CMP zerop : flags_NZC << A - [zerop]
        GET_ZP;
        CMP(A);
        break;

      case 0xc6: // DEC zerop : [zerop] <- [zerop] - 1
        GET_ZP;
        DCR;
        WRITE(AB, data);
        break;

      case 0xc8: // INY : Y <- Y + 1
        Y++;
        UPDATE_NZ(Y);
        break;

      case 0xc9: // CMP immed : flags_NZC << A - immed
        GET_IMM;
        CMP(A);
        break;

      case 0xca: // DEX : X <- X - 1
        X--;
        UPDATE_NZ(X);
        break;

      case 0xcc: // CPY absol : flags_NZC << Y - [absol]
        GET_ABS;
        CMP(Y);
        break;

      case 0xcd: // CMP absol : flags_NZC << A - [absol]
        GET_ABS;
        CMP(A);
        break;

      case 0xce: // DEC absol : [absol] <- [absol] - 1
        GET_ABS;
        DCR;
        WRITE(AB, data);
        break;

      case 0xd0: // BNE relat : branch on Z=0
        GETW_REL;
        if (!ISSET(Z)) PC = AB;
        break;

      case 0xd1: // CMP (zp),Y : flags_NZC << A - [(zp) + Y]
        GET_ZPIY;
        CMP(A);
        break;

      case 0xd5: // CMP zerop,X : flags_NZC << A - [zerop + X]
        GET_ZPX;
        CMP(A);
        break;

      case 0xd6: // DEC zp,X : [zp + X] <- [zp + X] - 1
        GET_ZPX;
        DCR;
        WRITE(AB, data);
        break;

      case 0xd8: // CLD : flag_D <- 0
        CLEAR(D);
        break;

      case 0xd9: // CMP absol,Y : flags_NZC << A - [absol + Y]
        GET_ABSY;
        CMP(A);
        break;

      case 0xdd: // CMP absol,X : flags_NZC << A - [absol + X]
        GET_ABSX;
        CMP(A);
        break;

      case 0xde: // DEC absol,X : [absol + X] <- [absol + X] - 1
        GET_ABSX;
        DCR;
        WRITE(AB, data);
        break;

      case 0xe0: // CPX immed : flags_NZC << X - immed
        GET_IMM;
        CMP(X);
        break;

      case 0xe1: // SBC (zp,X) : A <- A - [(zp + X)] - flag_C
        GET_ZPIX;
        SBC;
        break;

      case 0xe4: // CPX zerop : flags_NZC << X - [zerop]
        GET_ZP;
        CMP(X);
        break;

      case 0xe5: // SBC zerop : A <- A - [zerop] - flag_C
        GET_ZP;
        SBC;
        break;

      case 0xe6: // INC zerop : [zerop] <- [zerop] + 1
        GET_ZP;
        INC;
        WRITE(AB, data);
        break;

      case 0xe8: // INX : X <- X + 1
        X++;
        UPDATE_NZ(X);
        break;

      case 0xe9: // SBC immed : A <- A - immed - flag_C
        GET_IMM;
        SBC;
        break;

      case 0xea: // NOP : no operation
        break;

      case 0xec: // CPX absol : flags_NZC << X - [absol]
        GET_ABS;
        CMP(X);
        break;

      case 0xed: // SBC absol : A <- A - [absol] - flag_C
        GET_ABS;
        SBC;
        break;

      case 0xee: // INC absol : [absol] <- [absol] + 1
        GET_ABS;
        INC;
        WRITE(AB, data);
        break;

      case 0xf0: // BEQ relat : branch on Z=1
        GETW_REL;
        if (ISSET(Z)) PC = AB;
        break;

      case 0xf1: // SBC (zp),Y : A <- A - [(zp) + Y] - flag_C
        GET_ZPIY;
        SBC;
        break;

      case 0xf5: // SBC zp,X : A <- A - [zp + X] - flag_C
        GET_ZPX;
        SBC;
        break;

      case 0xf6: // INC zp,X : [zerop + X] <- [zp + X] + 1
        GET_ZPX;
        INC;
        WRITE(AB, data);
        break;

      case 0xf8: // SED : flag_D <- 1
        SET(D);
        break;

      case 0xf9: // SBC absol,Y : A <- A - [absol + Y] - flag_C
        GET_ABSY;
        SBC;
        break;

      case 0xfd: // SBC absol,X : A <- A - [absol + X] - flag_C
        GET_ABSX;
        SBC;
        break;

      case 0xfe: // INC absol,X : [absol + X] <- [absol + X] + 1
        GET_ABSX;
        INC;
        WRITE(AB, data);
        break;

      default:
        illegalOp = 1;
        Serial.print(F("ERROR: stop on illegalOp 6502 OpCode 0x"));
        Serial.print(IR, HEX);
        Serial.print(" @ $");
        Serial.println(iADR, HEX);
        stopExecution = true;
        break;
    }

    iCount++; // istructions count
    tCount += clkTicks[IR]; // clock ticks count

    // execution stream memory
    lastiadr[iadri++] = iADR;
    if (iadri >= PC_STREAM_MEM) iadri = 0;

    #ifdef _LOCK_DECECTION
      // lock detection
      if (PC == prevPC) {
        printf("-> ");
        printState();
        Serial.print(F("Warning: stop on trap @ $"));
        Serial.println(iADR, HEX);
        illegalOp = 2;
        stopExecution = true;
        break;
      }
    #endif

    prevPC = PC;

    #ifdef _NMI_ENABLED
      // NMI request detection
      noInterrupts();
      if (nmiRequest != prevNmiReq) nmiCount++; // edge triggered
      prevNmiReq = nmiRequest;
      interrupts();
    #endif

    #ifdef _IRQ_ENABLED
      // IRQ request detection
      noInterrupts();
      if (irqRequest != 0 && !ISSET(I) && !nmiRunning) irqStart(); // level triggered
      interrupts();
    #endif

    if (dueIrq) onDueIrq();

  } while (tCount < clkCount);

  if (stopExecution) {
    Serial.println(F("Execution stopped!"));
    printState();
    dumpHistory();
  }

  return iCount;
}

void resetCPU() {
  P = I | Un;
  A = 0;
  X = 0;
  Y = 0;
  S = 0xfd;
  PC = FETCHW(RST_VECTOR);
  irqRequest = 0;
  nmiRequest = 0;
  //nestedInterrupts = 0;
  #ifdef _IRQNMI_OUTPUT
    digitalWrite(IRQ_PIN, LOW);
    digitalWrite(NMI_PIN, LOW);
  #endif
  stopExecution = false;
  nmiRunning = false;
}

// assumes I flag is 0 !
void irqStart() {
  #ifdef _IRQNMI_OUTPUT
    digitalWrite(IRQ_PIN, HIGH);
  #endif
  CLEAR(B);
  PUSHW(PC); // PC already incremented
  PUSH(P);
  SET(I);
  PC = FETCHW(IRQ_VECTOR);
  irqCount++;
  //nestedInterrupts++;
}

void nmiStart() {
  #ifdef _IRQNMI_OUTPUT
    digitalWrite(NMI_PIN, HIGH);
  #endif
  CLEAR(B);
  PUSHW(PC); // PC altready incremented
  PUSH(P);
  SET(I);
  PC = FETCHW(NMI_VECTOR);
  nmiCount++;
  //nestedInterrupts++;
  nmiRunning = true;
}

byte bcdADC(byte d) {
  byte nybL, nybH;
  byte carry;
  uint16_t res;

  carry = ISSET(C) ? 1 : 0;
  res = (uint16_t)A + (uint16_t)d + (uint16_t)carry;
  UPDATE_NZ((byte)(res & 0x00FF));
  UPDATE_V(d, res);
  CLEAR(C);
  if ((res & 0x000F) > 0x0009) res += 0x0006;
  if ((res & 0x00F0) > 0x0090) {
    res += 0x0060;
    SET(C);
  }
  return res;
}

byte bcdSBC(byte d) {
  byte nybL, nybH;
  byte carry;
  uint16_t res;

  carry = ISSET(C) ? 1 : 0;
  res = (uint16_t)A + (uint16_t)(~d) + (uint16_t)carry;
  UPDATE_NZ((byte)(res & 0x00FF));
  UPDATE_V(d, res);
  CLEAR(C);
  res -= 0x0066;
  if ((res & 0x000F) > 0x0009) res += 0x0006;
  if ((res & 0x00F0) > 0x0090) {
    res += 0x0060;
    SET(C);
  }
  return res;
}

void printStatusBits() {
  if (ISSET(N)) Serial.print("N"); else Serial.print(".");
  if (ISSET(V)) Serial.print("V"); else Serial.print(".");
  Serial.print("-");
  if (ISSET(B)) Serial.print("B"); else Serial.print(".");
  if (ISSET(D)) Serial.print("D"); else Serial.print(".");
  if (ISSET(I)) Serial.print("I"); else Serial.print(".");
  if (ISSET(Z)) Serial.print("Z"); else Serial.print(".");
  if (ISSET(C)) Serial.print("C"); else Serial.print(".");
}

void printState() {
  Serial.print(" IR:");
  Serial.print(IR, HEX);
  Serial.print(" A:");
  Serial.print(A, HEX);
  Serial.print(" X:");
  Serial.print(X, HEX);
  Serial.print(" Y:");
  Serial.print(Y, HEX);
  Serial.print(" PC:");
  Serial.print(PC, HEX);
  Serial.print(" P:");
  Serial.print(P, HEX);
  Serial.print("(");
  printStatusBits();
  Serial.print(") SP:");
  Serial.println(S, HEX);
}

void printStack() {
  int i;

  Serial.print(" STK:(");
  Serial.print(0xff - S);
  if (S == 0xff) {
    Serial.println(") --");
    return;
  }
  for (i = 0xff; i > S; i--) {
    Serial.print(FETCH(0x0100 + i), HEX);
    Serial.print(" ");
  }
  Serial.println(".");
}

void dumpHistory() {
  int p, i;

  p = (iCount <= PC_STREAM_MEM) ? 0 : iadri;
  for (i = 0; i < PC_STREAM_MEM; i++) {
    dasm(lastiadr[p++]);
    if (p >= PC_STREAM_MEM) p = 0;
  }
  dasm(iADR);
}

void printCurrentIstruction() {
  dasm(iADR);
}
