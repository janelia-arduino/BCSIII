// ----------------------------------------------------------------------------
// BCSIII
//
// Provides an interface to the BCS III hardware
//  - IO setup
//  - SPI control
//
// Author: Steve Sawtelle - based on code by Peter Polidoro
// -------------------------------------------------------------


// ---- VERSIONS  ----
// 20150429 sws
// - remove atomic stuff to be compatible with MAX32, use Interrupts

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif


#include "BCSIII.h"
#include "CS.h"
#include "MCP23S17.h"



//---------- constructor  ---------------------------------------

BCSIII bcs;

void BCSIII::begin(void)
{

    SPI.begin();

    // congigure the Reference enables 
	// AREF must be disabled until we select external reference
    digitalWrite(AREF_EN_PIN, LOW);
    pinMode(AREF_EN_PIN, OUTPUT);
    analogReference(EXTERNAL);
    digitalWrite(AREF_EN_PIN, HIGH);
 //   ANALOG_RESOLUTION = analogRead(ID_SENSE_PIN);

    CS.begin(CS_EN_PIN, CS_ADR0_PIN, CS_ADR1_PIN, CS_ADR2_PIN);

//    // disable chip selects and set them as ouputs
//    SPIselect(NO_CS);
//    pinMode(CS_EN_PIN, OUTPUT);
//    pinMode(CS_ADR0_PIN, OUTPUT);
//    pinMode(CS_ADR1_PIN, OUTPUT);
//    pinMode(CS_ADR2_PIN, OUTPUT);

    // Initialize IO settings
    MCP23S17writeReg(BB_ADR, IOCON, IOCON_VAL);  // set up banks, ints, etc
    MCP23S17writeReg(DIO_ADR, IOCON, IOCON_VAL);
    MCP23S17writeReg(AIO_ADR, IOCON, IOCON_VAL);

    MCP23S17writeReg(BB_ADR, GPPUA, 0xFF);  // use pull ups on BB inputs
    MCP23S17writeReg(BB_ADR, GPPUB, 0xFF);  //
    MCP23S17writeReg(BB_ADR, GPIOA, 0x00); // set low initially
    MCP23S17writeReg(BB_ADR, GPIOB, 0x00); // set low initially
    MCP23S17writeReg(DIO_ADR, GPIOA, 0x00); // init dir control on 1T45s as inputs
    MCP23S17writeReg(DIO_ADR, GPIOB, 0x00); // not used but set it to known value
    MCP23S17writeReg(AIO_ADR, GPIOA, 0x00); // init dir control on 1T45s as inputs
    MCP23S17writeReg(AIO_ADR, GPIOB, 0x00); // disable digital part

    MCP23S17writeReg(BB_ADR, IODIRA, 0xFe); // init BB IO as inputs to start off
    MCP23S17writeReg(BB_ADR, IODIRB, 0xFF); //
    MCP23S17writeReg(DIO_ADR, IODIRA, 0x00); // dir control for digital
    MCP23S17writeReg(DIO_ADR, IODIRB, 0x00); // not used but set it to known value
    MCP23S17writeReg(AIO_ADR, IODIRA, 0x00); // dir control for digital
    MCP23S17writeReg(AIO_ADR, IODIRB, 0x00); // digital or analog

    // for now we only use interrupt on falling low
    MCP23S17writeReg(BB_ADR, GPINTENA, 0x00); // no pins enabled
    MCP23S17writeReg(BB_ADR, GPINTENB, 0x00);
    MCP23S17writeReg(BB_ADR, DEFVALA, 0xFF);  // default will be high
    MCP23S17writeReg(BB_ADR, INTCONA, 0xFF);  // compare input for change from default register
    MCP23S17writeReg(BB_ADR, DEFVALB, 0xFF);  // default will be high
    MCP23S17writeReg(BB_ADR, INTCONB, 0xFF);  // compare input for change from default register

    // see what's connected
//    Serial.println("SPI discovery");
//    for( uint8_t SPIcon = 1; SPIcon < 7; SPIcon++)
//    {
//        CS.SPIselect(SPIcon);
//        delay(10);  // let settle a bit
//        Serial.println(analogRead(A8));
//    }
//    CS.SPIselect(NO_CS);

     analogRead(A8);  // dummy read of ID Port

    // set up USER LED but turn it off for now
    digitalWrite(LED_BUILTIN, LOW);
    pinMode(LED_BUILTIN, OUTPUT);
 }


//---------- public  ------------------------------------

uint8_t BCSIII::getPin(uint8_t BNC)
{
    if( (BNC < 1) || (BNC > 16) ) return BAD_BNC;
    return(IOpin[BNC-1]);
}

uint8_t BCSIII::getMode(uint8_t BNC)
{
    if( (BNC < 1) || (BNC > 16) ) return BAD_BNC;
    return(IOtype[BNC-1]);
}


int8_t BCSIII::setIO(uint8_t BNC, uint8_t IO, uint8_t type)
{
uint8_t i;

//    Serial.print("BNC:");
//    Serial.print(BNC);
//    Serial.print(" to:");
//    Serial.print(IO);
//    Serial.print(" as:");
//    Serial.println(type);

       if( (BNC < 1) || (BNC > 16) ) return BAD_BNC;
       BNC--;   // 1-16 --> 0-15

       // check for unsupported pin number
       for( i = 0; i < 16; i++ )
       {
          if( IO == pin_number[i]) break;
       }
       if( i == 16  ) return BAD_PIN;

       // if it's an input, set the IO as input first so we don't fight BNC vs IO out
       if( type == ANALOG_IN )
       {
          if( (IO >= A0) && (IO <= A7) ) // only IOs 0-7 can be analog
          {
              analogRead(IO);         // good one - set it - Analog inputs numbered same as IO pin names
              IOpin[BNC] = IO;
              IOtype[BNC]= type;     // and remember the new connection
          }
          else  // try to connect non-analog as analog
          {
              return BAD_ANALOG;
          }
       }
       else if( type == INPUT )     // or if it's a digital in
       {
          pinMode( IO, INPUT);     // set it
          IOpin[BNC] = IO;
          IOtype[BNC] = type; // and remember the new connection
       }
#ifdef INPUT_PULLUP
       else if( type == INPUT_PULLUP )     // or if it's a digital in w/ pullup
       {
           pinMode( IO, INPUT_PULLUP);     // set it
           IOpin[BNC] = IO;
           IOtype[BNC] = type; // and remember the new connection
       }
#endif
       else if( (type == OUTPUT) || (type == NO_CONNECT) )
       {
           // hold off on making it an output until mux is right
           IOpin[BNC] = IO;
           IOtype[BNC] = type; // and remember the new connection
       }
       else // not a known IO type
       {
           return BAD_TYPE;
       }


       UpdateIOs(); // and update the physical connections

       // if it was an input, now it safe to set as output
       if( type == OUTPUT )
          pinMode(IO, OUTPUT);

       return 0;

}

int8_t BCSIII::setBB(uint8_t BBpin, uint8_t LEDon)
{
    if( (BBpin < 1) || (BBpin > 8) ) return BAD_BNC;
    BBpin--;   // 1-8 --> 0-7

    // get current LEDs port values and add new one
    uint8_t BBport = MCP23S17readReg(BB_ADR, GPIOA);

//    Serial.print(BBpin);
//    Serial.print("=");
//    Serial.print(BBport, HEX);
//    Serial.print(">");
    if( LEDon == 0 ) // 0 = off, but hardware is high to be off
        BBport |= 1 << BBpin;
    else
        BBport &= ~(1 << BBpin);
//    Serial.print(BBport, HEX);
//    Serial.print(" ");
    MCP23S17writeReg(BB_ADR, GPIOA, BBport); //

    // set this pin as output
    BBport = MCP23S17readReg(BB_ADR, IODIRA);
//    Serial.print(BBport, HEX);
//    Serial.print(">");
    BBport &= ~(1 << BBpin);
//    Serial.print(BBport, HEX);
//    Serial.print(" = ");
    MCP23S17writeReg(BB_ADR, IODIRA, BBport); //

    // set corresponding portb pin as input to read BB
    BBport = MCP23S17readReg(BB_ADR, IODIRB);
    BBport |= (1 << BBpin);
//    Serial.println(BBport, HEX);
    MCP23S17writeReg(BB_ADR, IODIRB, BBport); //

}

uint8_t BCSIII::readBB(uint8_t BBpin)
{
    if( (BBpin < 1) || (BBpin > 8) ) return BAD_BNC;
    BBpin--;   // 1-8 --> 0-7

    if( MCP23S17readReg(BB_ADR, GPIOB) & (1<<BBpin) )
       return 1;
     else
       return 0;

}

uint8_t BCSIII::readBB(void)
{
     // set coresponding portb pin as intput to read BB
    return( MCP23S17readReg(BB_ADR, GPIOB) );

}


void BCSIII::LEDon(void)
{
    digitalWrite(LED_BUILTIN, HIGH);
}

void BCSIII::LEDoff(void)
{
    digitalWrite(LED_BUILTIN, LOW);
}



// -------------- PRIVATE

// ----------------------------------------------------------------------------

// SPI Chip select routines

/*
void BCSIII::SPIselect(uint8_t chip)
{
   digitalWrite(CS_EN_PIN, HIGH); // disable all CS while changing

   if( chip < NO_CS)
   {
      digitalWrite(CS_ADR0_PIN, chip & 0x01);
      digitalWrite(CS_ADR1_PIN, chip & 0x02);
      digitalWrite(CS_ADR2_PIN, chip & 0x04);

      digitalWrite(CS_EN_PIN, LOW);  // enable selected output
   }
}

*/

// MCP23S17 interface
// we have three on the BCS: two for mux IO direction and enable, one for Beam Breaks
// BB is at adr 0, digital IO control at adr 1, and analog UIO control at adr 2

// set the mux and directions according to the setup info in the arrays
// the mux expects 256 contiguous bits, we will use 32 8 bit bytes
// Y15-X15, Y15-X14, .... etc
// Y is BNC side, X is IO side
// of course I have it messed up
// Let's call top left front BNC1
// then it goes bottom left front is BNC2
// So ---
// IOMUX[0] will be MUX0
// The value will be dir, BNC with BNC = bits 0-3
//


uint8_t BCSIII::MCP23S17readReg( uint8_t adr, uint8_t reg)
{
 uint8_t data;

    noInterrupts(); //ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
      SPI.setDataMode(SPI_MODE0);  // set proper mode, clk idle high, rising edge
      CS.SPIselect(MCP_CS);
      SPI.transfer(MCP23S17_RD | adr);
      SPI.transfer(reg);
      data = SPI.transfer(0);
      CS.SPIselect(NO_CS);
    } // end int off
    interrupts();
    return data;
}


void BCSIII::MCP23S17writeReg(uint8_t adr, uint8_t reg, uint8_t data )
{
    noInterrupts(); // ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
      SPI.setDataMode(SPI_MODE0);  // set proper mode, clk idle high, rising edge
      CS.SPIselect(MCP_CS);
      SPI.transfer(MCP23S17_WR | adr);
      SPI.transfer(reg);
      SPI.transfer(data);
      CS.SPIselect(NO_CS);
    } // end int off
    interrupts();
}



void BCSIII::UpdateIOs(void)
{

uint8_t  dig_dir = 0;
uint8_t ana_dir = 0;
uint8_t ana_en  = 0; 
uint16_t muxbit;    // only allow one connection per BNC
uint8_t pindex;


noInterrupts(); // ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
{
   CS.SPIselect(NO_CS);  // the mux loads after CS pulses low
   //Set the multiplexer connection
   SPI.setDataMode(SPI_MODE0);   // clock idles low, latched on rising edge

   for( int8_t bnc = 0; bnc < 16; bnc++) // walk through BNC array- low to high -
   // - should be high to low but then BNCs are all backwards - fix that in SW
   {
     // figure which IO channel this is
     for( pindex = 0; pindex < 16; pindex++ )
     {
        if( pin_number[pindex] == IOpin[bnc]) break;
     }
     // only set a channel if it has a direction (not a NC)
     if( IOtype[bnc] == NO_CONNECT )
        muxbit = 0;
     else
        muxbit = 1 << pindex; //put a connection '1' in right spot

 //    Serial.print("mux= 0x");
  //   Serial.println(muxbit, HEX);

     SPI.transfer( muxbit >> 8);    // do this BNC
     SPI.transfer( muxbit & 0xff);

     if( pindex < 16 ) // we have a real channel
     {
         // and while we are here, set the dir and enable lines
         if( pindex < 8 ) // lower 8 IOs are only digital so no enables
         {
             if( IOtype[bnc] == OUTPUT ) // if output
                 dig_dir |= (1 << pindex); // an output needs a '1' in right spot
         }
         else  // higher 8 - have dir and enable
         {   
             pindex -= 8;   // 8-15 => 0-7
             if( IOtype[bnc] == OUTPUT ) // if output
             {  // set direction to '1'
                 ana_dir |= (1 << pindex); // 
             }

             if( IOtype[bnc] == ANALOG_IN ) // if analog, disable digital
             {
                 ana_en &= ~(1 << pindex); // analog needs a '0' in the right spot
             }
             else  // must be digital in
             {
                 ana_en |= (1 << pindex); // digital needs a '1' in right spot
             }

         } // endif low or high ports
     }
   } // next BNC
   CS.SPIselect(MUX_CS);  // now latch the data
   delayMicroseconds(2);
   CS.SPIselect(NO_CS);

  } // end int off
interrupts();

//Serial.print("dig, ana, anaen ");
//Serial.print(dig_dir, HEX);
//Serial.print(",");
//Serial.print(ana_dir,HEX);
//Serial.print(",");
//Serial.println(ana_en,HEX);

    MCP23S17writeReg(DIO_ADR, GPIOA, dig_dir); // digital dir control on 1T45s
    MCP23S17writeReg(AIO_ADR, GPIOA, ana_dir); // analog  dir control on 1T45s
    MCP23S17writeReg(AIO_ADR, GPIOB, ana_en);  // analog  digital enables on 1T45s
}











