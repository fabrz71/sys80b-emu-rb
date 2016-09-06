// 6502 code disassembler

extern byte mread(uint16_t adr);
byte dasm(uint16_t adr);
uint16_t getw(byte lo, byte hi);
void printval(byte b1);
void printval(byte b1, byte b2);
void printval(byte b1, byte b2, byte b3);

PROGMEM const char *opCode[] = {
  "BRK", "ORA", "---", "---", "---", "ORA", "ASL", "---", "PHP", "ORA", "ASL", "---", "---", "ORA", "ASL", "---", // 00-0F
  "BPL", "ORA", "---", "---", "---", "ORA", "ASL", "---", "CLC", "ORA", "---", "---", "---", "ORA", "ASL", "---", // 10-
  "JSR", "AND", "---", "---", "BIT", "AND", "ROL", "---", "PLP", "AND", "ROL", "---", "BIT", "AND", "ROL", "---", // 20-
  "BMI", "AND", "---", "---", "---", "AND", "ROL", "---", "SEC", "AND", "---", "---", "---", "AND", "ROL", "---", // 30-
  "RTI", "EOR", "---", "---", "---", "EOR", "LSR", "---", "PHA", "EOR", "LSR", "---", "JMP", "EOR", "LSR", "---", // 40-
  "BVC", "EOR", "---", "---", "---", "EOR", "LSR", "---", "CLI", "EOR", "---", "---", "---", "EOR", "LSR", "---", // 50-
  "RTS", "ADC", "---", "---", "---", "ADC", "ROR", "---", "PLA", "ADC", "ROR", "---", "JPI", "ADC", "ROR", "---", // 60-
  "BVS", "ADC", "---", "---", "---", "ADC", "ROR", "---", "SEI", "ADC", "---", "---", "---", "ADC", "ROR", "---", // 70-
  "---", "STA", "---", "---", "STY", "STA", "STX", "---", "DEY", "---", "TXA", "---", "STY", "STA", "STX", "---", // 80-
  "BCC", "STA", "---", "---", "STY", "STA", "STX", "---", "TYA", "STA", "TXS", "---", "---", "STA", "---", "---", // 90-
  "LDY", "LDA", "LDX", "---", "LDY", "LDA", "LDX", "---", "TAY", "LDA", "TAX", "---", "LDY", "LDA", "LDX", "---", // A0-
  "BCS", "LDA", "---", "---", "LDY", "LDA", "LDX", "---", "CLV", "LDA", "TSX", "---", "LDY", "LDA", "LDX", "---", // B0-
  "CPY", "CMP", "---", "---", "CPY", "CMP", "DEC", "---", "INY", "CMP", "DEX", "---", "CPY", "CMP", "DEC", "---", // C0-
  "BNE", "CMP", "---", "---", "---", "CMP", "DEC", "---", "CLD", "CMP", "---", "---", "---", "CMP", "DEC", "---", // D0-
  "CPX", "SBC", "---", "---", "CPX", "SBC", "INC", "---", "INX", "SBC", "NOP", "---", "CPX", "SBC", "INC", "---", // E0-
  "BEQ", "SBC", "---", "---", "---", "SBC", "INC", "---", "SED", "SBC", "---", "---", "---", "SBC", "INC", "---"  // F0-FF
};

byte dasm(uint16_t adr) {
  byte ir = mread(adr);
  byte a1 = mread(adr + 1);
  byte a2 = mread(adr + 2);
  byte r = 0;
  
  Serial.print(adr, HEX);

  switch (ir & 0x1f) {
    case 0x00: // impl/immed
      if (ir == 0x20) { // JSR
        printval(ir, a1, a2);
        Serial.print("$"); Serial.print(getw(a1, a2), HEX);
        r = 3;
      }
      else if (ir & 0x80) { // LDY, CPY, CPX
        printval(ir, a1);
        Serial.print("#"); Serial.print(a1, HEX);
        r = 2;
      }
      else r = 1;
      break;
    case 0x01: // (indir,X)
      printval(ir, a1);
      Serial.print("($"); Serial.print(a1, HEX); Serial.print(",X)");
      r = 2;
      break;
    case 0x02: // ?/immed
      if (ir == 0xa2) { // LDX
        printval(ir, a1);
        Serial.print("#"); Serial.print(a1, HEX);
        r = 2;
      }
      else r = 1;
      break;
    case 0x04: // zeropage
    case 0x05:
    case 0x06:
      printval(ir, a1);
      Serial.print("$"); Serial.print(a1, HEX);
      r = 2;
      break;
    case 0x08: // implied
    case 0x18:
    case 0x1a:
    case 0x0a: // accu/impl
      printval(ir);
      Serial.print(" ");
      r = 1;
      break;
    case 0x09: // immediate
      printval(ir, a1);
      Serial.print("#"); Serial.print(a1, HEX);
      r = 2;
      break;
    case 0x0c: // absolute
    case 0x0d:
    case 0x0e:
      if (ir == 0x6c) { // JMP ($xxxx)
        printval(ir, a1);
        Serial.print("($"); Serial.print(a1, HEX); Serial.print(")");
        r = 2;
      }
      else {
        printval(ir, a1, a2);
        Serial.print("$"); Serial.print(getw(a1, a2), HEX);
        r = 3;
      }
      break;
    case 0x10: // relative
      printval(ir, a1);
      Serial.print("$");
      Serial.print((a1 <= 0x7f) ? adr + 2 + (uint16_t)a1 : adr + 2 - (uint16_t)((~(a1 - 1)) & 0xff), HEX);
      r = 2;
      break;
    case 0x11: // (indir),Y
      printval(ir, a1);
      Serial.print("($"); Serial.print(a1, HEX); Serial.print("),Y");
      r = 2;
      break;
    case 0x14: // zeropage,X
    case 0x15:
    case 0x16:
      printval(ir, a1);
      if (ir == 0x96 || ir == 0xb6 || ir == 0x97 || ir == 0xb7) {
        Serial.print("$"); Serial.print(a1, HEX); Serial.print("),Y");
      }
      else {
        Serial.print("$"); Serial.print(a1, HEX); Serial.print("),X");
      }
      r = 2;
      break;
    case 0x19: // absolute,Y
      printval(ir, a1, a2);
      Serial.print("$"); Serial.print(getw(a1, a2), HEX); Serial.print(",Y");
      r = 2;
      break;
    case 0x1c: // absolute,X
    case 0x1d:
    case 0x1e:
      printval(ir, a1, a2);
      Serial.print("$"); Serial.print(getw(a1, a2), HEX); Serial.print(",X");
      r = 2;
      break;
    default:
      printval(ir);
      Serial.print("?");
      r = 1;
      break;
  }
  Serial.println(" ");
  return r;
}

void printval(byte v1) {
  Serial.print("\t");
  Serial.print(v1, HEX);
  Serial.print("\t");
  Serial.print(opCode[v1]); Serial.print(" ");
}

void printval(byte v1, byte v2) {
  Serial.print("\t");
  Serial.print(v1, HEX);
  Serial.print(" ");
  Serial.print(v2, HEX);
  Serial.print("\t");
  Serial.print(opCode[v1]); Serial.print(" ");
}

void printval(byte v1, byte v2, byte v3) {
  Serial.print("\t");
  Serial.print(v1, HEX);
  Serial.print(" ");
  Serial.print(v2, HEX);
  Serial.print(" ");
  Serial.print(v3, HEX);
  Serial.print("\t");
  Serial.print(opCode[v1]); Serial.print(" ");
}

uint16_t getw(byte lo, byte hi) {
  return ((uint16_t)(hi)) << 8 | ((uint16_t)(lo));
}
