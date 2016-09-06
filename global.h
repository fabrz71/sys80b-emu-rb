// global constants

const byte LED_PIN = 13;
const byte RXLED_PIN = 72; // active low
const byte TXLED_PIN = 73; // active low
const byte SLAM_PIN = 2; // INPUT, active low
const byte AUXB_PIN = 31; // INPUT, active low
const byte LG_CS_PIN = 41; // active low - ledGrid.h
const byte GPIO_SS_PIN = 42; // active low - interface.h
const byte SD_CS_PIN = 43; // active low - sdrom.h
const byte LG_LOAD_PIN = 53; // ledGrid.h
const byte IOEN_PIN = 16; // active low - interface.h
const byte IRQ_PIN = 21;
const byte NMI_PIN = 22;
const byte DBG0_PIN = 17;
const byte DBG1_PIN = 18;
const byte DBG2_PIN = 19;
const byte DBG3_PIN = 20;
const byte LCD_D4_PIN = 4;
const byte LCD_D5_PIN = 5;
const byte LCD_D6_PIN = 6;
const byte LCD_D7_PIN = 7;
const byte LCD_RS_PIN = 23;
const byte LCD_EN_PIN = 24;

const byte SPI_CLK_DIV = 12;

PROGMEM const char *inpName[] = { "RETURNS" , "SLAM SWITCH" };
PROGMEM const char *outpName[] = { "STROBES", "SOLENOIDS", "SOUND", "LAMPS", "DISPLAY" };

enum riotP { RIOTPORT_A, RIOTPORT_B };
enum inputType { RETURNS, SLAM_SW };
enum outputType { STROBES, SOLENOIDS, SOUND, LAMPS, DISPL };

void infiniteLoop();

void infiniteLoop() {
  boolean led = false;
  while (true) {
    digitalWrite(LED_PIN, led = !led);
    delay(200);
  }
}
