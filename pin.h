#ifndef _PIN_H
#define _PIN_H

#include <pins_arduino.h>
#include <avr/io.h>
#include <WProgram.h>

#define PA 1
#define PB 2
#define PC 3
#define PD 4
#define PE 5
#define PF 6
#define PG 7
#define PH 8
#define PJ 10
#define PL 11


//Define the Digital pin arrangements and masks for each port on the Arduino or Sanguino
#if defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644__) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega1284__)
//If on the Sanguino platform maximum number of digital IO pins available
#define _P_MAX		31

#define _PA_FIRST   24
#define _PA_LAST	31
#define _PA_MASK(n) (0x80 >> ((n) & 0x07))
#define _PA(n)		((n) > (_PA_FIRST-1)) && ((n) < (_PA_FIRST + 8))

#define _PB_FIRST   0
#define _PB_LAST	7
#define _PB_MASK(n) (0x01 << ((n) & 0x07))
#define _PB(n)		((n) > (_PB_FIRST-1)) && ((n) < (_PB_FIRST + 8))

#define _PC_FIRST   16
#define _PC_LAST	23
#define _PC_MASK(n) (0x01 << ((n) & 0x07))
#define _PC(n)		((n) > (_PC_FIRST-1)) && ((n) < (_PC_FIRST + 8))

#define _PD_FIRST   8
#define _PD_LAST	15
#define _PD_MASK(n) (0x01 << ((n) & 0x07))
#define _PD(n)		((n) > (_PD_FIRST-1)) && ((n) < (_PD_FIRST + 8))

#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)

#define _P_MAX		69

#else

//else if the Arduino platform
#define _P_MAX		15

#define _PA_FIRST   0
#define _PA_LAST	0
#define _PA_MASK(n) 0
#define _PA(n)		0

#define _PB_FIRST   8
#define _PB_LAST	15
#define _PB_MASK(n) (0x01 << ((n) & 0x07))
#define _PB(n)		(((n) > (_PB_FIRST-1)) && ((n) < (_PB_FIRST+8))

#define _PC_FIRST   0
#define _PC_LAST	0
#define _PC_MASK(n) 0
#define _PC(n)		0

#define _PD_FIRST   0
#define _PD_LAST	7
#define _PD_MASK(n) (0x01 << ((n) & 0x07))
#define _PD(n)		(((n) > (_PD_FIRST-1)) && ((n) < (_PD_LAST+8))
#endif


class pin
{
public:
	pin(void);
	pin(byte);
	pin(byte, byte);
	void setup(byte p, byte dir);

	void set(byte);
	void set(void);
	void clear(void);

	byte get(void);
	
	void setPin(byte);
	void setDir(byte);
  
  byte getPin(void) { return _pin; }
  byte getDir(void) { return _dir; }
  byte getPort(void) { return _port; }
  byte getMask(void) { return _mask; }
  
  uint16_t getDDRx(void) { return (uint16_t)_pDDRx; }
  uint16_t getPORTx(void) { return (uint16_t)_pPORTx; }
  uint16_t getPINx(void) { return (uint16_t)_pPINx; }

private:
	byte _pin;
	byte _dir;
	byte _port;
	byte _mask;
  
  volatile uint8_t* _pDDRx;
  volatile uint8_t* _pPORTx;
  volatile uint8_t* _pPINx;
};

#endif
