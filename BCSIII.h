// ----------------------------------------------------------------------------
// BSCIII.h
//
// Setup and control code needed for the BCS III
//
// Author: SWS based on code by Peter Polidoro
// ----------------------------------------------------------------------------

// ---- VERSIONS  ----
// 20150429 sws
// - remove atomic stuff to be compatible with MAX32, use Interrupts

#ifndef BCSIII_H_
#define BCSIII_H_

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <SPI.h>

//#include <util/atomic.h>
#include <CS.h>
#include <MCP23S17.h>
//#include "MCP23S17registers.h"

//#define LEDPIN 9

#ifndef LED_BUILTIN
    #define LED_BUILTIN 13
#endif


//#define DIN 0x30
//#define AIN 0x10
//#define DOUT 0x20
//#define NC 0x00

// NOTE: other I/O type use Arduino defines for OUTPUT and INPUT
#define ANALOG_IN 0x08
#define NO_CONNECT 0x0f
#define NC NO_CONNECT

 const uint8_t pin_number[] = {3,4,5,6,7,8,20,21, A0, A1, A2, A3, A4, A5, A6, A7};
 static uint8_t IOpin[]  = {  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 };
 static uint8_t IOtype[] = { NC, NC, NC, NC, NC, NC, NC, NC, NC, NC, NC, NC, NC, NC, NC, NC };

// uint16_t ANALOG_RESOLUTION;

class BCSIII
{
public:
//  BCSIII(void);
  void begin(void);
  uint8_t getPin(uint8_t BNC);
  uint8_t getMode(uint8_t BNC);
  int8_t setIO(uint8_t BNC, uint8_t IO, uint8_t type);
  int8_t setBB(uint8_t BBpin, uint8_t LEDon);
  uint8_t readBB(uint8_t BBpin);
  uint8_t readBB(void);
  void LEDon(void);
  void LEDoff(void);


    #define SPI1 1
    #define SPI2 2
    #define SPI3 3
    #define SPI4 4
    #define SPI5 5
    #define SPI6 6


  // errors
    #define BAD_BNC -1
    #define BAD_PIN -2
    #define BAD_TYPE -3
    #define BAD_ANALOG -4

  // BNCs
    #define BNC1 1
    #define BNC2 2
    #define BNC3 3
    #define BNC4 4
    #define BNC5 5
    #define BNC6 6
    #define BNC7 7
    #define BNC8 8
    #define BNC9 9
    #define BNC10 10
    #define BNC11 11
    #define BNC12 12
    #define BNC13 13
    #define BNC14 14
    #define BNC15 15
    #define BNC16 16



private:
  // Private Constants

    #define ID_SENSE_PIN A8
    #define AREF_EN_PIN 42
    #define I2C_EN_PIN    45
    #define SPARE_I2C_PIN 53

//   void SPIselect(uint8_t chip);
   void UpdateIOs(void);
   void SPIselect(uint8_t chip);

   uint8_t MCP23S17readReg( uint8_t adr, uint8_t reg);
   void MCP23S17writeReg(uint8_t adr, uint8_t reg, uint8_t data );

};

extern BCSIII bcs;

#endif // BCSIII



