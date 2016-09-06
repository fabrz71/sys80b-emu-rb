// MCP24S17 GPIO expanders functions

#include <SPI.h>

#define ADDR_ENABLE 0b00001000  // Configuration register for MCP23S17, the only thing we change is enabling hardware addressing
#define OPCODEW     0b01000000  // Opcode for MCP23S17 with LSB (bit0) set to write (0), address OR'd in later, bits 1-3
#define OPCODER     0b01000001  // Opcode for MCP23S17 with LSB (bit0) set to read (1), address OR'd in later, bits 1-3

#define SPI_CS_PORT PIOA // relative to pin 42
#define SPI_CS_PPIN 19   // relative to pin 42
//#define MCP_SPI_CS_OFF digitalWrite(ss_pin, LOW)
//#define MCP_SPI_CS_ON digitalWrite(ss_pin, HIGH)
#define MCP_SPI_CS_OFF  SPI_CS_PORT->PIO_CODR = MCP_SPI_CS_bitmask
#define MCP_SPI_CS_ON   SPI_CS_PORT->PIO_SODR = MCP_SPI_CS_bitmask

#define    IODIRA    (0x00)      // MCP23x17 I/O Direction Register
#define    IPOLA     (0x02)      // MCP23x17 Input Polarity Register
#define    IOCON     (0x0A)      // MCP23x17 Configuration Register
#define    GPPUA     (0x0C)      // MCP23x17 Weak Pull-Up Resistor Register
#define    GPIOA     (0x12)      // MCP23x17 GPIO Port Register
#define    GPIOB     (0x13)      // Value on the Port - Writing Sets Bits in the Output Latch

const unsigned int MCP_SPI_CS_bitmask = ((unsigned int)1) << SPI_CS_PPIN;
int ss_pin;
extern const byte SPI_CLK_DIV;

void MCP_init(int pin);
void byteWrite(uint8_t adr, uint8_t reg, uint8_t value);
void wordWrite(uint8_t adr, uint8_t reg, uint16_t value);
uint8_t byteRead(uint8_t adr, uint8_t reg);
void pullupMode(uint8_t adr, uint16_t mode);
void inputInvert(uint8_t adr, uint16_t mode);
void pinDirection(uint8_t adr, uint16_t mode);
uint16_t mcpRead(uint8_t adr);
void mcpWrite(uint8_t adr, uint16_t value);
void mcpWritePA(uint8_t adr, uint8_t value);
void mcpWritePB(uint8_t adr, uint8_t value);

void MCP_init(int pin) {
  ss_pin = pin;
  SPI.begin(ss_pin);
  SPI.setClockDivider(ss_pin, SPI_CLK_DIV);
  SPI.setBitOrder(ss_pin, MSBFIRST);
  SPI.setDataMode(ss_pin, SPI_MODE0);
  byteWrite(0, IOCON, ADDR_ENABLE);
}

// GENERIC BYTE WRITE - will write a byte to a register, arguments are register address and the value to write
void byteWrite(uint8_t adr, uint8_t reg, uint8_t value) {      // Accept the register and byte
  MCP_SPI_CS_OFF;                                 // Slave Select LOW before SPI action
  SPI.transfer(OPCODEW | (adr << 1));             // Send the MCP23S17 opcode, chip address, and write bit
  SPI.transfer(reg);                                   // Send the register we want to write
  SPI.transfer(value);                                 // Send the byte
  MCP_SPI_CS_ON;                                 // Slave Select HIGH after SPI action
}

// GENERIC WORD WRITE - will write a word to a register pair, LSB to first register, MSB to next higher value register 
void wordWrite(uint8_t adr, uint8_t reg, uint16_t value) {  // Accept the start register and word 
  MCP_SPI_CS_OFF;                               // Slave Select LOW before SPI action 
  SPI.transfer(OPCODEW | (adr << 1));             // Send the MCP23S17 opcode, chip address, and write bit
  SPI.transfer(reg);                                   // Send the register we want to write 
  SPI.transfer((uint8_t) (value & 0xff));                      // Send the low byte (register address pointer will auto-increment after write)
  SPI.transfer((uint8_t) (value >> 8));                 // Shift the high byte down to the low byte location and send
  MCP_SPI_CS_ON;                                 // Slave Select HIGH after SPI action
}

uint8_t byteRead(uint8_t adr, uint8_t reg) {        // This function will read a single register, and return it
  uint8_t value = 0;                        // Initialize a variable to hold the read values to be returned
  MCP_SPI_CS_OFF;                     // Slave Select LOW before SPI action
  SPI.transfer(OPCODER | (adr << 1));  // Send the MCP23S17 opcode, chip address, and read bit
  SPI.transfer(reg);                        // Send the register we want to read
  value = SPI.transfer(0x00);               // Send any byte, the function will return the read value
  MCP_SPI_CS_ON;                      // Slave Select HIGH after SPI action
  return value;                             // Return the constructed word, the format is 0x(register value)
}

void pinDirection(uint8_t adr, uint16_t mode) {    // Accept the word…
  wordWrite(adr, IODIRA, mode);                // Call the the generic word writer with start register and the mode cache
}

void pullupMode(uint8_t adr, uint16_t mode) { 
  wordWrite(adr, GPPUA, mode);
}

void inputInvert(uint8_t adr, uint16_t mode) { 
  wordWrite(adr, IPOLA, mode);
}

uint16_t mcpRead(uint8_t adr) {       // This function will read all 16 bits of I/O, and return them as a word in the format 0x(portB)(portA)
  uint16_t value = 0;                   // Initialize a variable to hold the read values to be returned
  MCP_SPI_CS_OFF;                      // Slave Select LOW before SPI action
  SPI.transfer(OPCODER | (adr << 1));  // Send the MCP23S17 opcode, chip address, and read bit
  SPI.transfer(GPIOA);                      // Send the register we want to read
  value = SPI.transfer(0x00);               // Send any byte, the function will return the read value (register address pointer will auto-increment after write)
  value |= (SPI.transfer(0x00) << 8);       // Read in the "high byte" (portB) and shift it up to the high location and merge with the "low byte"
  MCP_SPI_CS_ON;                     // Slave Select HIGH after SPI action
  return value;                             // Return the constructed word, the format is 0x(portB)(portA)
}

void mcpWrite(uint8_t adr, uint16_t value) { 
  MCP_SPI_CS_OFF;                               // Slave Select LOW before SPI action 
  SPI.transfer(OPCODEW | (adr << 1));             // Send the MCP23S17 opcode, chip address, and write bit
  SPI.transfer(GPIOA);                                   // Send the register we want to write 
  SPI.transfer((uint8_t) (value & 0xff));                      // Send the low byte (register address pointer will auto-increment after write)
  SPI.transfer((uint8_t) (value >> 8));                 // Shift the high byte down to the low byte location and send
  MCP_SPI_CS_ON;                                 // Slave Select HIGH after SPI action
}

void mcpWritePA(uint8_t adr, uint8_t value) { 
  MCP_SPI_CS_OFF;                               // Slave Select LOW before SPI action 
  SPI.transfer(OPCODEW | (adr << 1));             // Send the MCP23S17 opcode, chip address, and write bit
  SPI.transfer(GPIOA);                                   // Send the register we want to write 
  SPI.transfer(value); 
  MCP_SPI_CS_ON;                                 // Slave Select HIGH after SPI action
}

void mcpWritePB(uint8_t adr, uint8_t value) { 
  MCP_SPI_CS_OFF;                               // Slave Select LOW before SPI action 
  SPI.transfer(OPCODEW | (adr << 1));             // Send the MCP23S17 opcode, chip address, and write bit
  SPI.transfer(GPIOB);                                   // Send the register we want to write 
  SPI.transfer(value);                      
  MCP_SPI_CS_ON;                                 // Slave Select HIGH after SPI action
}
