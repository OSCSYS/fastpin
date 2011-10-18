/***********************************************************

  FastPin Library

  Copyright (C) 2009-2011 Matt Reba, Jermeiah Dillingham

    This file is part of BrewTroller.

    BrewTroller is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    BrewTroller is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with BrewTroller.  If not, see <http://www.gnu.org/licenses/>.

    Documentation, Forums and more information available at http://www.brewtroller.com

    Original Author:  Matt Reba & Jason Vreeland (CodeRage)
    Modified By:      Tom Harkaway, Feb, 2011

    Modifications:

    1. Generalize FastPin library so that it can be used on Arduino platforms, 
       specifically the Arduino Mega.

    2. Incorporate PinChange interrupt (PCInt) support. On the BrewTroller, all
       32 pins can be configured as PCInt pins. That is not true of other Arduino
       platforms, such as the Adrunio Mega.

    The framework for the PinChange interrupt functions came from the PinChangeInt
    library found at http://www.arduino.cc/playground/Main/PinChangeInt.

***********************************************************/

// fixup issues
// ToDo: attachPCInt for pin that is already attached?
// ToDo: detachPCInt for pin that is not attached?
// ToDo: same pin assigned to two different pin objects?
// ToDo: remove if else method of get after testing

#include <pins_arduino.h>

#include <pin.h>


// Documentation on the PinChangeInt library indicated that using pointers was faster
//  and consumed less memory that using arrays and for loops. To compare, a second 
//  PCInt handler routine was created that used arrays. Results:
//
//                Pointer   Array
//                -------   -----
//  Memory(bytes)   152      148  
//  Speed (cycles)  171      182  // when PCInt is in 2nd slot
//
// Arrays code is actually smaller, but is it 6% slower. For now I would suggest sticking
//  with the pointer based method.
//
//#define ARRAY_BASED_HANDLER
#define POINTER_BASED_HANDLER



// static PCI_Pin array
//  Change MAX_PIN_CHANGE_PINS in pin.h to match requirements. Small means faster.
//
PCI_Port::PCI_Pin PCI_Port::PCI_Pin::s_pcIntPins[MAX_PIN_CHANGE_PINS];

// static PCI_Port array
//
PCI_Port PCI_Port::s_pcIntPorts[] = {
#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega238__)
  PCI_Port(0, PB, PCMSK0),   // PB0-PB5 (Pins 8-13)
  PCI_Port(1, PC, PCMSK1),   // PC0-PC5 (Pins 14-19, AIN)
  PCI_Port(2, PD, PCMSK2)    // PD0-PD7 (Pins 0-7)
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) 
  PCI_Port(0, PB, PCMSK0),   // PB0-PB7 (Pins 8-15)
  PCI_Port(1, PJ, PCMSK1),   // PE0, PJ0-1 (Pins 0, 14, 15)
  PCI_Port(2, PK, PCMSK2)    // PK0-PK7 (Pins 62-69, AIN8-15)
#elif defined(__AVR_ATmega644P__) || defined(__AVR_ATmega1284P__) 
  PCI_Port(0, PA, PCMSK0),   // PB0-PB7 (Pins 0-7)
  PCI_Port(1, PB, PCMSK1),   // PD0-PD7 (Pins 8-15)
  PCI_Port(2, PC, PCMSK2),   // PC0-PC7 (Pins 16-23)
  PCI_Port(3, PD, PCMSK3)    // PA0-PA7 (Pins 24-31, AIN)
#endif
};


/*********************************************************************/
//
// Basic FastPin Methods
//
/*********************************************************************/

// pin constructors
//
pin::pin(void)
{
	_pinID  = 0;
	_dir    = INPUT;
	_mask   = 0x00;
  _port   = NOT_A_PORT;
}

pin::pin(byte pinID)
{
	_dir  = INPUT;
	setup(pinID, INPUT);
}

pin::pin(byte pinID, byte pinDir)
{
	setup(pinID, pinDir);
}


// set/change configuration
//

void pin::setup(byte pinID, byte pinDir)
{
  if( pinID > MAX_PIN)
  {
    _port = NOT_A_PORT;
    _mask = 0;
    return;
  }

  _pinID = pinID;
  _dir   = pinDir;
  _mask  = digitalPinToBitMask(pinID);
  _port  = digitalPinToPort(pinID);

  volatile uint8_t* pDDRx  = portModeRegister(_port);
  if(_dir == OUTPUT)
    *pDDRx |= _mask;
  else
    *pDDRx &= ~_mask;
}

void pin::setPin(byte pinID)
{
	_pinID = pinID;
	setup(_pinID, _dir);
}

void pin::setDir(byte pinDir)
{
	_dir = pinDir;
	setup(_pinID, _dir);
}


// set/get pin value
//
// Original code used large switch statements to select the proper port. Before committing
//  to this approach for this update, a couple of other methods were tried. One was to 
//  create a member variable for each pin object that contain a reference to the PIN 
//  register. The second was to lookup the PIN register each time using the methods defined
//  in pins_arduino.h. Results:
//
//                      Size    Cycles
//                      -----   ------
//  switch statement     62      24   (cycles were for worst case scenario)            
//  local ref to PIN     46+2x   37   (where x is the number of pins defined)
//  lookup PIN           44      42
//
// While switch statements take up more memory, they are very efficient. Even for the 
//  Arduino Mega, which has 11 ports to deal with, the switch statement grew to 146
//  bytes, but the worst case switch time was 38 cycles.
//

void pin::set(void)
{
	switch(_port)
	{
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega1284P__)
	case PA:  PORTA |= _mask; return;
#endif
  case PB:  PORTB |= _mask; return;
	case PC:  PORTC |= _mask; return;
  case PD:  PORTD |= _mask; return;
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  case PE:  PORTE |= _mask; return;
  case PF:  PORTF |= _mask; return;
  case PG:  PORTG |= _mask; return;
  case PH:  PORTH |= _mask; return;
  case PJ:  PORTJ |= _mask; return;
  case PK:  PORTK |= _mask; return;
  case PL:  PORTL |= _mask; return;
#endif
  }
}

void pin::clear(void)
{
	switch(_port)
	{
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega1284P__)
  case PA:  PORTA &= ~_mask; return;
#endif
  case PB:  PORTB &= ~_mask; return;
  case PC:  PORTC &= ~_mask; return;
	case PD:  PORTD &= ~_mask; return;
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  case PE:  PORTE &= ~_mask; return;
  case PF:  PORTF &= ~_mask; return;
  case PG:  PORTG &= ~_mask; return;
  case PH:  PORTH &= ~_mask; return;
  case PJ:  PORTJ &= ~_mask; return;
  case PK:  PORTK &= ~_mask; return;
  case PL:  PORTL &= ~_mask; return;
#endif
  }
}

void pin::toggle(void)
{
	switch(_port)
	{
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega1284P__)
  case PA:  PORTA ^= _mask; return;
#endif
  case PB:  PORTB ^= _mask; return;
  case PC:  PORTC ^= _mask; return;
  case PD:  PORTD ^= _mask; return;
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  case PE:  PORTE ^= _mask; return;
  case PF:  PORTF ^= _mask; return;
  case PG:  PORTG ^= _mask; return;
  case PH:  PORTH ^= _mask; return;
  case PJ:  PORTJ ^= _mask; return;
  case PK:  PORTK ^= _mask; return;
  case PL:  PORTL ^= _mask; return;
#endif
  }
}

void pin::set(byte state)
{
	if(state == HIGH) 
    set();
	else  
    clear();
}

//bool pin::get(void)
//{
//  if (_port == PA)       return((PINA & _mask)?0xFF:0x0);
//  else if (_port == PB)  return((PINB & _mask)?0xFF:0x0);
//  else if (_port == PC)  return((PINC & _mask)?0xFF:0x0);
//  else if (_port == PD)  return((PIND & _mask)?0xFF:0x0);  
//#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
//  else if (_port == PE)  return((PINE & _mask)?0xFF:0x0);
//  else if (_port == PF)  return((PINF & _mask)?0xFF:0x0);
//  else if (_port == PG)  return((PING & _mask)?0xFF:0x0);
//  else if (_port == PH)  return((PINH & _mask)?0xFF:0x0);  
//  else if (_port == PJ)  return((PINJ & _mask)?0xFF:0x0);
//  else if (_port == PK)  return((PINK & _mask)?0xFF:0x0);
//  else if (_port == PL)  return((PINL & _mask)?0xFF:0x0);
//#endif
//  return 0;
//}

bool pin::get(void)
{
	switch(_port)
	{


#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega1284P__)
  case PA:  return((PINA & _mask)?0xFF:0x0);
#endif
  case PB:  return((PINB & _mask)?0xFF:0x0);
  case PC:  return((PINC & _mask)?0xFF:0x0);
  case PD:  return((PIND & _mask)?0xFF:0x0);  
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  case PE:  return((PINE & _mask)?0xFF:0x0);
  case PF:  return((PINF & _mask)?0xFF:0x0);
  case PG:  return((PING & _mask)?0xFF:0x0);
  case PH:  return((PINH & _mask)?0xFF:0x0);  
  case PJ:  return((PINJ & _mask)?0xFF:0x0);
  case PK:  return((PINK & _mask)?0xFF:0x0);
  case PL:  return((PINL & _mask)?0xFF:0x0);
#endif
  }
	return 0;
}

/*********************************************************************/
//
// PinChange Interrupt Methods
//
/*********************************************************************/

// attach to a PinChange Interrupt
// 
//  Return Codes
//    -4 - internal error, should never happen, > 8 attached to a single port
//    -3 - all PCInt slots are used (increase MAX_PIN_CHANGE_PINS)
//    -2 - pin does not support PinChange Interrupts (only on MegA
//    -1 - UserFunc is null (bad call)
//   >=0 - index of s_pcIntPins slot used
//
int pin::attachPCInt(int mode, PCIntvoidFuncPtr userFunc)
{
  // NULL function?
  if (userFunc == NULL)
    return -1;

  // get pcInt info (PCInt index & enable mask)
  byte pcintInfo = digitalPinToPCINT(_pinID);
  if (pcintInfo == NO_PCINT)
    return -2;

  byte pcintIndex = pcintInfo >> 4;
  byte pcintMask  = 1 << (pcintInfo & 0x0f);

  // add pin
  PCI_Port& port = PCI_Port::s_pcIntPorts[pcintIndex];

  int retcode = port.addPin(_pinID, mode, pcintMask, userFunc);
  //port.dumpPCIntPinArray();
  return retcode;
}


// detach a PinChange interrupt
//
bool pin::detachPCInt()
{
  byte pciInfo = digitalPinToPCINT(_pinID);
  if (pciInfo == NO_PCINT)
    return false;

  byte pciPortIndex = pciInfo >> 4;
  PCI_Port& port = PCI_Port::s_pcIntPorts[pciPortIndex];
  return port.delPin(_pinID);
}


// add (enable) PinChange pin to PinChange port
//
int PCI_Port::addPin(byte pinID, byte mode, byte mask, PCIntvoidFuncPtr userFunc)
{
  byte i,j;
  PCI_Pin* pPin;

  for (i=0; i<MAX_PIN_CHANGE_PINS; i++)
  {
    pPin = &(PCI_Pin::s_pcIntPins[i]);
    if (pPin->_func == NULL)
    {
      for (j=0; j<8; j++)
      {
        if (_pcIntPinArray[j] == NULL)
        {
          pPin->_pinID    = pinID;
          pPin->_pinMask  = mask;
          pPin->_pinMode  = mode;
          pPin->_func     = userFunc;
          _pcIntPinArray[j]   = pPin;
          _pcmskReg |= mask;
          PCICR |= _pcicrBit;
          return i;
        }
      }
      //Serial.println("  port._pcIntPins full.");
      return -4;
    }
  }
  //Serial.println("  No available s_pcIntPins slots.");
  return -3;
}

// delete (disable) PinChange pin from PinChange port
//
bool PCI_Port::delPin(byte pinID)
{
  //dumpPCIntPinArray();
  byte i;
  bool done = false;
  for (i=0; i<8; i++)
  {
    PCI_Pin* pPin = _pcIntPinArray[i];

    if (pPin == NULL)
      return false;

    if (pPin->_pinID == pinID)
    {
      byte oldSREG = SREG;
      cli();

      // disable the mask.
      _pcmskReg &= ~pPin->_pinMask;

      // if that's the last one, disable the interrupt.
      if (_pcmskReg == 0)
        PCICR &= ~_pcicrBit;

      // clean up PCintPin object
      pPin->_pinMask = 0;
      pPin->_pinMode = 0;
      pPin->_func    = NULL;

      _pcIntPinArray[i] = NULL;

      SREG = oldSREG;
      break;
    }
  }

  // fill in the gap
  for (; i<7; i++)
  {
    _pcIntPinArray[i]   = _pcIntPinArray[i+1];
    _pcIntPinArray[i+1] = NULL;
  }
  //dumpPCIntPinArray();
  return true;
}



//*********************************************************************/
//
// PinChange Interrupt Service Routines
//
/*********************************************************************/

// Interrupt Handler using pointers
//
//
#ifdef POINTER_BASED_HANDLER
void PCI_Port::PCintHandler()
{
//#ifndef DISABLE_PCINT_MULTI_SERVICE
//  uint8_t pcifr;
//  do {
//#endif

    // process first PCINT on this port (???)

    byte actual  = _inputReg;          // get actual port val
    byte changed = actual ^ _lastVal;  // xor with last to get changed

    // get the pin states for the indicated port and mask pins that have changed. 
    //    screen out non pcint pins.
    if ((changed & _pcmskReg) == 0)
      return; // no attached PinChanges to process (Should Never Happen)

    _lastVal = actual;

    PCI_Pin** t = _pcIntPinArray;
    while (*t) 
    {
      PCI_Pin& p = **t;
      if ((p._pinMask & changed) != 0) 
      { 
        if ((p._pinMode == CHANGE) ||
            (p._pinMode == FALLING && !(actual & p._pinMask)) ||
            (p._pinMode == RISING  &&  (actual & p._pinMask))) 
        {
          p._func();
        }
      }
      t++;
    }

    //#ifndef DISABLE_PCINT_MULTI_SERVICE
    //    pcifr = PCIFR & PCICRbit;
    //    PCIFR = pcifr;	// clear the interrupt if we will process is (no effect if bit is zero)
    //  } while(pcifr);
    //#endif
}
#endif

// Handler using array references.
//
#ifdef ARRAY_BASED_HANDLER
void PCI_Port::PCintHandler()
{
  byte actual  = _inputReg;          // get actual port val
  byte changed = actual ^ _lastVal;  // xor with last to get changed

  // get the pin states for the indicated port and mask pins that have changed. 
  //    screen out non pcint pins.
  if ((changed & _pcmskReg) == 0)
    return; // no attached PinChanges to process (Should Never Happen)

  _lastVal = actual;

    byte i;
    for (i=0; i<8; i++)
    {
      PCI_Pin* pPin = _pcIntPinArray[i];
      if ((pPin->_pinMask & changed) != 0) 
      { 
        if ((pPin->_pinMode == CHANGE) ||
            (pPin->_pinMode == FALLING && !(actual & pPin->_pinMask)) ||
            (pPin->_pinMode == RISING  &&  (actual & pPin->_pinMask))) 
        {
          pPin->_func();
          break;
        }
      }
    }
}
#endif



#ifndef NO_PORT0_PINCHANGES
ISR(PCINT0_vect) 
{
  PCI_Port::s_pcIntPorts[0].PCintHandler();
}
#endif

#ifndef NO_PORT1_PINCHANGES
ISR(PCINT1_vect) 
{
  PCI_Port::s_pcIntPorts[1].PCintHandler();
}
#endif

#ifndef NO_PORT2_PINCHANGES
ISR(PCINT2_vect) 
{
  PCI_Port::s_pcIntPorts[2].PCintHandler();
}
#endif

#if defined(__AVR_ATmega644P__) || defined(__AVR_ATmega1284P__) && \
  !defined(NO_PORT3_PINCHANGES)
ISR(PCINT3_vect) 
{
  PCI_Port::s_pcIntPorts[3].PCintHandler();
}
#endif



/*********************************************************************/
//
// PinChange Information Map 
//
/*********************************************************************/

// Digital Pin to PCInt Info
// 
// Array contains PCInt# & Bit information for each Pin#
//  High Nibble = PCInt Number (0,1,2,3)
//  Low Nibble  = Bit Number (0-7)
//  NO_PCINT    = PCINT is not available (no assigned PIN)
//

#define PCINT_BIT(_pcint_,_bit_) ((_pcint_<<4)+_bit_)

#if defined(__AVR_ATmega644P__) || defined(__AVR_ATmega1284P__)

const uint8_t PROGMEM digitalpin_to_pcint_PGM[] = {
  PCINT_BIT(1,0), // 0    PB0
  PCINT_BIT(1,1), // 1    PB1
  PCINT_BIT(1,2), // 2    PB2
  PCINT_BIT(1,3), // 3    PB3
  PCINT_BIT(1,4), // 4    PB4
  PCINT_BIT(1,5), // 5    PB5
  PCINT_BIT(1,6), // 6    PB6
  PCINT_BIT(1,7), // 7    PB7
  PCINT_BIT(3,0), // 8    PD0
  PCINT_BIT(3,1), // 9    PD1
  PCINT_BIT(3,2), // 10   PD2
  PCINT_BIT(3,3), // 11   PD3
  PCINT_BIT(3,4), // 12   PD4
  PCINT_BIT(3,5), // 13   PD5
  PCINT_BIT(3,6), // 14   PD6
  PCINT_BIT(3,7), // 15   PD7
  PCINT_BIT(2,0), // 16   PC0
  PCINT_BIT(2,1), // 17   PC1
  PCINT_BIT(2,2), // 18   PC2
  PCINT_BIT(2,3), // 19   PC3
  PCINT_BIT(2,4), // 20   PC4
  PCINT_BIT(2,5), // 21   PC5
  PCINT_BIT(2,6), // 22   PC6
  PCINT_BIT(2,7), // 23   PC7
  PCINT_BIT(0,7), // 24   PA7
  PCINT_BIT(0,6), // 25   PA6
  PCINT_BIT(0,5), // 26   PA5
  PCINT_BIT(0,4), // 27   PA4
  PCINT_BIT(0,3), // 28   PA3
  PCINT_BIT(0,2), // 29   PA2
  PCINT_BIT(0,1), // 30   PA1
  PCINT_BIT(0,0), // 31   PA0
};


#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)

const uint8_t PROGMEM digitalpin_to_pcint_PGM[] = {
  NO_PCINT,       // 0    //PCINT_BIT(1,0), // 0   PE0
  NO_PCINT,       // 1
  NO_PCINT,       // 2
  NO_PCINT,       // 3
  NO_PCINT,       // 4
  NO_PCINT,       // 5
  NO_PCINT,       // 6
  NO_PCINT,       // 7
  NO_PCINT,       // 8
  NO_PCINT,       // 9
  PCINT_BIT(0,4), // 10   PB4
  PCINT_BIT(0,5), // 11   PB5
  PCINT_BIT(0,6), // 12   PB6
  PCINT_BIT(0,7), // 13   PB7
  NO_PCINT,       // 14   PCINT_BIT(1,2), // 14   PJ1
  NO_PCINT,       // 15   PCINT_BIT(1,1), // 15   PJ0
  NO_PCINT,       // 16
  NO_PCINT,       // 17
  NO_PCINT,       // 18
  NO_PCINT,       // 19
  NO_PCINT,       // 20
  NO_PCINT,       // 21
  NO_PCINT,       // 22
  NO_PCINT,       // 23
  NO_PCINT,       // 24
  NO_PCINT,       // 25
  NO_PCINT,       // 26
  NO_PCINT,       // 27
  NO_PCINT,       // 28
  NO_PCINT,       // 29
  NO_PCINT,       // 30
  NO_PCINT,       // 31
  NO_PCINT,       // 32
  NO_PCINT,       // 33
  NO_PCINT,       // 34
  NO_PCINT,       // 35
  NO_PCINT,       // 36
  NO_PCINT,       // 37
  NO_PCINT,       // 38
  NO_PCINT,       // 39
  NO_PCINT,       // 40
  NO_PCINT,       // 41
  NO_PCINT,       // 42
  NO_PCINT,       // 43
  NO_PCINT,       // 44
  NO_PCINT,       // 45
  NO_PCINT,       // 46
  NO_PCINT,       // 47
  NO_PCINT,       // 48
  NO_PCINT,       // 49
  PCINT_BIT(0,3), // 50   PB3
  PCINT_BIT(0,2), // 51   PB2
  PCINT_BIT(0,1), // 52   PB1
  PCINT_BIT(0,0), // 53   PB0
  NO_PCINT,       // 55
  NO_PCINT,       // 56
  NO_PCINT,       // 57
  NO_PCINT,       // 58
  NO_PCINT,       // 59
  NO_PCINT,       // 50
  NO_PCINT,       // 60
  NO_PCINT,       // 61
  PCINT_BIT(2,0), // 62   PK0
  PCINT_BIT(2,1), // 63   PK1
  PCINT_BIT(2,2), // 64   PK2
  PCINT_BIT(2,3), // 65   PK3
  PCINT_BIT(2,4), // 66   PK4
  PCINT_BIT(2,5), // 67   PK5
  PCINT_BIT(2,6), // 68   PK6
  PCINT_BIT(2,7), // 69   PK7

};

#endif


#ifdef FASTPIN_DEBUG

void PCI_Port::dumpPCIntPin(PCI_Pin* pPin)
{
  Serial.print("  PCI_PIN pinID=");
  Serial.print(pPin->_pinID, DEC);
  Serial.print(", pinMask=0x");
  Serial.print(pPin->_pinMask, DEC);
  Serial.print(", pinMode=");
  Serial.print(pPin->_pinMode, DEC);
  Serial.print(", Func=");
  if (pPin->_func == NULL)
    Serial.print("NULL!");
  else
    Serial.print("not null!");
  Serial.println();
}

void PCI_Port::dumpPCIntPinArray()
{
  Serial.print("pcInt Array: ");
  byte i;
  for (i=0; i<8; i++)
  {
    if (_pcIntPinArray[i] == NULL)
    {
      Serial.print("NULL ");
    }
    else
    {
      Serial.print(_pcIntPinArray[i]->_pinID, DEC);
      Serial.print(" ");
    }
  }
  Serial.println();
}
#endif

