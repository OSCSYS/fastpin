#include<pin.h>

pin::pin(void)
{
	_pin    = 0;
	_dir    = INPUT;
	_mask   = 0x00;
  _port   = NOT_A_PORT;
}

pin::pin(byte p)
{
	_dir  = INPUT;
	_mask = 0x00;
  _port = NOT_A_PORT;
	this->setup(p,INPUT);
}

pin::pin(byte p, byte dir)
{
	_mask = 0x00;
  _port = NOT_A_PORT;
	this->setup(p , dir);
}


void pin::setPin(byte p)
{
	_pin = p;
	this->setup(_pin, _dir);
}

void pin::setDir(byte dir)
{
	_dir = dir;
	this->setup(_pin,_dir);
}

#if defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644__) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega1284__)

void pin::setup(byte p, byte dir)
{
	if( p > _P_MAX)
		return;

  _dir = dir;
	_pin = p;
	_mask = 0x00;

	if(_PA(_pin))
	{
		_mask = _PA_MASK(_pin);
		if(_dir == OUTPUT)
			DDRA |= _mask;
		else
			DDRA &= ~_mask;
		_port=PA;
		return;
	}

	else if(_PB(_pin))
	{
		_mask = _PB_MASK(_pin);
		if(_dir == OUTPUT)
			DDRB |= _mask;
		else
			DDRB &= ~_mask;
		_port=PB;
		return;
	}

	else if(_PC(_pin))
	{
		_mask = _PC_MASK(_pin);
		if(_dir == OUTPUT)
			DDRC |= _mask;
		else
			DDRC &= ~_mask;
		_port=PC;
		return;
	}
	else if(_PD(_pin))
	{
		_mask = _PD_MASK(_pin);
		if(_dir == OUTPUT)
			DDRD |= _mask;
		else
			DDRD &= ~_mask;
		_port=PD;
		return;
	}
	return;
}

void pin::set(void)
{
	switch(_port)
	{
	case PD:
		{
			PORTD |= _mask;
			break;
		}

	case PB:
		{
			PORTB |= _mask;
			break;
		}
	
	case PC:
		{
			PORTC |= _mask;
			break;
		}

	case PA:
		{
			PORTA |= _mask;
			break;
		}

	defualt:
			break;
	}
	return;
}

void pin::clear(void)
{
	switch(_port)
	{
	case PD:
		{
			PORTD &= ~_mask;
			break;
		}

	case PB:
		{
			PORTB &= ~_mask;
			break;
		}
	
	case PC:
		{
			PORTC &= ~_mask;
			break;
		}

	case PA:
		{
			PORTA &= ~_mask;
			break;
		}

	default:
			break;
	}
	return;
}

void pin::set(byte state)
{
	if (_dir == INPUT)
		return;

	if(state == HIGH)
		this->set();
	else
		this->clear();
}


byte pin::get(void)
{
	switch(_port)
	{
	case PD:
		{
			return((PIND & _mask)?0xFF:0x0);
			break;
		}
	case PB:
		{
			return((PINB & _mask)?0xFF:0x0);
			break;
		}

	case PC:
		{
			return((PINC & _mask)?0xFF:0x0);
			break;
		}
	case PA:
		{
			return((PINA & _mask)?0xFF:0x0);
			break;
		}
	default :
		break;
	}
	return(0);
}

#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)

void pin::setup(byte p, byte dir)
{
  _port = NOT_A_PORT;
	if( p > _P_MAX)
		return;

	_dir = dir;
	_pin = p;

  _mask = pgm_read_byte(digital_pin_to_bit_mask_PGM+p);
  _port = pgm_read_byte(digital_pin_to_port_PGM+p);
  _pDDRx  = (byte*)pgm_read_word(port_to_mode_PGM+_port);
  _pPORTx = (byte*)pgm_read_word(port_to_output_PGM+_port);
  _pPINx  = (byte*)pgm_read_word(port_to_input_PGM+_port);
  
	if(_dir == OUTPUT)
		*_pDDRx |= _mask;
	else
		*_pDDRx &= ~_mask;
}

void pin::set(void)
{
  //.byte btVal = *_pPORTx;
  //btVal |= _mask;
  //*_pPORTx = btVal;
  *_pPORTx |= _mask;
}

void pin::clear(void)
{
  //.byte btVal = *_pPORTx;
  //btVal &= ~_mask;
  //*_pPORTx = btVal;
  *_pPORTx &= ~_mask;
}

void pin::set(byte state)
{
	if (_dir == INPUT)
		return;

	if(state == HIGH)
		set();
	else
		clear();
}


byte pin::get(void)
{
  if (_port == 0) 
    return 0;

  return ((*_pPINx & _mask) ? 0xFF : 0x00);
}

#endif