

// ----------------------------------------------------------------------------
// MCP23S17registers.h
//
// Register names, etc for MCP23S17 as used in the BCS
//
// Author: Steve Sawtelle
// ----------------------------------------------------------------------------

#ifndef MCP23S17REGISTERS_H
#define MCP23S17REGISTERS_H

#define LED_DRIVER  0x00
#define GPIO_OUT    0x01
#define GPIO_IN     0x02
#define GPIO_IN_PU  0x03

#define BB_ADR  0
#define DIO_ADR 2  // offset one bit higher to OR in with hardware address and R/W
#define AIO_ADR 4
#define MCP23S17_ADR 0x40
#define MCP23S17_RD  0x41
#define MCP23S17_WR  0x40
#define BBPORTA 0
#define BBPORTB 1

#define IOCON_VAL  0x78  // 0111 1000

#define IODIRA      0x00
#define IODIRB      0x01
#define IPOLA       0x02
#define IPOLB       0x03
#define GPINTENA    0x04
#define GPINTENB    0x05
#define DEFVALA     0x06
#define DEFVALB     0x07
#define INTCONA     0x08
#define INTCONB     0x09
#define IOCON       0x0A
//#define IOCON       0x0B
#define GPPUA       0x0C
#define GPPUB       0x0D
#define INTFB       0x0E
#define INTFA       0x0F
#define INTCAPA     0x10
#define INTCAPB     0x11
#define GPIOA       0x12
#define GPIOB       0x13
#define OLATA       0x14
#define OLATB       0x15

#define BBINT 31




#endif // MCP23S17REGISTERS_H
