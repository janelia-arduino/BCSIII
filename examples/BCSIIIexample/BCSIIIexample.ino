
// BCSIIIB Example code
//
//  20150217 sws, ID&F
//  HHMI Janelia Reearch Campus
//
// This code shows the basic setups needed for a BCSIII with
// ADC, ADC, and Driver Panels installed

// ====== NOTES ===========
// mandatory setup lines for a minimal system (no I/O panels installed) have '!!!' in the comments
// Compiled under Arduino 1.05-r2 for MEGA256 board

// version
#define VERSION "20150503"

// - 20150503 - sws
// - changed LCD display a bit

// include required arduino  header  files
#include <SPI.h>           // !!! SPI.h must be included in the main program

// - and required BCSIII header files - 
#include <CS.h>                  // !!! Mandatory header for SPI chip select routines
#include <MCP23S17.h>   // !!! Mandatory header for Beam break I/O
#include <BCSIII.h>            // !!! Mandatory header file for BCSII main board functions 
#include <DRIVER.h>               // header file for Driver panel functions
#include <ADC_AD7328.h>    // header file for ADC panel functions
#include <DAC_AD57x4.h>    // header file DAC panel functions

// each Panel needs an instance started
//  the names 'adc', 'dac',  and 'driver' are arbitrary, and
//  will be used to call the object's functions.
//  The BCS also is an object, but is automatically
//  instantiated and is called 'bcs'

ADC_AD7328 adc;   // instantiate an ADC panel driver
DRIVER driver;          // instantiate a Driver panel driver
DAC_AD57x4 dac;    // instantiate a DAC panel driver

#define BBPIN 8   // test beam break on this pin

int16_t dacval = 0;    // DAC output value

// --- Initialize the boards and connections ---
void setup() 
{
  // initialize USB serial communications
  Serial.begin(115200); 
  Serial.println("BCS III");
 
  // initialize LCD communications - assume it is plugged into Serial 1
  Serial1.begin(19200);
  Serial1.write(0x0c); // clear the display
  delay(10);
  Serial1.write(0x11); // Back-light on
  Serial1.print("BCSIII ");
  Serial1.print(VERSION);  // show version #
  
  // --- initialize objects ---
  Serial.println("Find and init devices");
  bcs.begin();    // !!! initialize the BCS  
  // and the panels 
  adc.begin();      // all ADC channels on, set to +/-10V range
  driver.begin(); 
  dac.begin();     // all DAC channels on, set to +/-10V range
  
  // --- show SPI connections ---
  // example code to see what SPI port the panels are connected
  //  to and display them to the LCD - a valid ID is 1-6
  Serial1.write(0x0d);  // go to first position of next line
  Serial1.print("ADC@");
  Serial1.print(adc.getID());
  Serial1.print(" DRV@");
  Serial1.print(driver.getID());
  Serial1.print(" DA@");
  Serial1.print(dac.getID());
  
  //--- Set up some BNC connections ---
  // We will print the return value to check for errors (<0).
  // !!! mandatory to read and write to BNC connections
  // Connect BNC 5 to Arduino port A0 as an analog input
    Serial.println(bcs.setIO( 5, A0, ANALOG_IN));
  // connect BNC 3 to Arduino pin 5 as a digital input
    Serial.println(bcs.setIO( 3, 5, INPUT));
  // connect BNC 1 to Arduino pin 20 as a digital output
    Serial.println(bcs.setIO( 1, 20, OUTPUT)); 

 
  // --- List out the current connections to BNCs on main board--- 
  for( uint8_t i = 1; i <= 16; i++ )
  {
     Serial.print("BNC");
     Serial.print(i);              // BNC pin number
     Serial.print(" ");
     Serial.print(bcs.getPin(i));   // Arduino pin 
     Serial.print(" = ");           // connected to it
     Serial.println(bcs.getMode(i)); // I/O mode of the pin
  }  
 
 
  // --- set up a beam break, turn on LED ---
  bcs.setBB(BBPIN, 1);
  
  Serial.println("====== setup done, loop ========");
}

// --- main loop ---
void loop() 
{ 
// ======= for testing - 4 sec loop ========
//   -  turn solenoid on and off - 2 secs on - 2 secs off
//   - set DAC to increasing vaklues (and wrap around) 
//  -  read ADC (connect to DAC 2 to read DAC data )
//   - blink user LED
//  - toggle BNC 1 output
//  - read digital input on BNC 3
// - read analog (0-5V) on BNC 5
  
    // -- access pins using Arduino pin names ---
    // Note - the standard Arduino functions reference Arduino pin numbers
    //     not the BNC port numbers
    // However, the Arduino functions can use BNC pin names
    //  by using the bcs.getPin(bnc) function, where 'bnc' 
    //  is the BNC pin number. This allows one to think in 
    //  terms of the actual front panel connections. 
    Serial.print(" A0:");
    Serial.println( analogRead(A0) );
//    Serial.println( analogRead(bcs.getPin(5)) ); // or use BNC pin

    digitalWrite(20,LOW);
//    digitalWrite(bcs.getPin(1), LOW); 

    Serial.print(" D5:");
    Serial.println(digitalRead(5));
//    Serial.println(digitalRead(bcs.getPin(3)));    
 
    // --- Solenoid/Relay/Valve DRIVER functions ---
    // turn a valve on connected to channel 2 on the 
    //  driver board
    driver.setChannelOn(2);
    
    // --- DAC AD57x4 functions ---
    // write to ch 2
    Serial.print("DAC 2:");
    Serial.println( dacval);
    dac.write(2, dacval);
    dacval += 4000;  // ramp through different values
    
    // --- ADC AD7328 functions ---
    // read the ADC value on AD7328 I/O Panel, channel 2
    Serial.print("ADC 2: ");
    Serial.println( adc.read(2));
      
    // --- beam break functions ---
    // read status of all beam breaks
    //  this is done by not specifying a port
    Serial.print("all BBs: ");
    Serial.println( bcs.readBB(), HEX );
    // or we could just look at the one we set up - specify the port
    Serial.print("BB");
    Serial.print(BBPIN);
    Serial.print(": ");
    Serial.println( bcs.readBB(BBPIN), HEX );
    
    bcs.LEDon();   // turns on 'USER' led
 
    Serial.println(" --------------");   // loop marker
    
    // --- wait a bit ---
    delay(2000);
    
   // --- and toggle ouputs ---
   digitalWrite(bcs.getPin(1), HIGH);  // use BNC pin number  
   driver.setChannelOff(2);
    
    bcs.LEDoff();   // turns off 'USER' led
    
    // --- wait a bit ---
    delay(2000);
    
    // --- and repeat ---    
     
}
