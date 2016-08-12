#include <DMXSerial.h>
#include <EEPROM.h>
#include "Timer.h"	 //http://github.com/JChristensen/Timer
Timer t;

// ================================================================================================
//																			ADJUST THESE VALUES

#define DISPLAY_BRIGHTNESS		14			// values: 0 - 16

// ================================================================================================
// ================================================================================================
//																			DONT CHANGE BELOW...
unsigned int 	dmx_address;
unsigned long 	time_since_last_dmx_packet;
unsigned char 	button_state;


// ==========================================================================================
//														SETUP
// ------------------------------------------------------------------------------------------
void setup() {					

	// set pinModes per Port. LED anodes and cathodes are all outputs
	DDRD = ~B10100101;
	DDRF = ~B10001100;
	DDRE = ~B10111111;
	
	// retrieve dmx address from EEPROM
	dmx_address = 0;
	unsigned char lowbyte,highbyte,controlbyte;
	lowbyte = EEPROM.read(0);
	highbyte = EEPROM.read(1);
	controlbyte =  EEPROM.read(2);
  	if (lowbyte == controlbyte) { 	// check consitency : low byte has been stored at address 0 and 2 
		if (highbyte < 2 ) {			// dmx address cannot be higher than 511, so high byte can be either 0 or 1
			dmx_address = lowbyte | (highbyte << 8 );
		} 
  	};
	
	DMXSerial.init(DMXReceiver);

	//pullups on pushbuttons. arduino style
	pinMode(13,INPUT_PULLUP);
	pinMode(A0,INPUT_PULLUP);

	// set up timed events
	t.every (1, update_display);
	t.every (20, check_btns);
	t.every (20, check_dmx);
}

// ==========================================================================================
//														LOOP
// ------------------------------------------------------------------------------------------

void loop() {
	t.update();
}

// ==========================================================================================
//														SUBFUNCTIONS
// ------------------------------------------------------------------------------------------

// ------------------------------------------------------------------------------------------
// check for dmx data

void check_dmx() {
	time_since_last_dmx_packet = DMXSerial.noDataSince();
	
	if (time_since_last_dmx_packet < 1000) {
		
	}
}

// ------------------------------------------------------------------------------------------
// store dmx address in eeprom
void store_dmx_address(void) {
	EEPROM.write(0,(unsigned char) dmx_address);
	EEPROM.write(1,(unsigned char) (dmx_address >> 8));	// high byte
	EEPROM.write(2,(unsigned char) dmx_address);		// for error checking	
}

// ------------------------------------------------------------------------------------------
// decide button presses
void up_or_down(unsigned char state) {
	if (state == 0) return;
	if (state & (1 << 0)) dmx_address--; 
	else if (state & (1 << 1)) dmx_address++; 
	dmx_address %= 512;
}

// ------------------------------------------------------------------------------------------
// detect button presses (called by timer every 20ms)

void check_btns(void) { 

	static unsigned char last_button_state;
	static unsigned long last_buttonpress;
	unsigned long elapsed_time;
	
	// buttons are on pin A0 (PF7) and D13 (PC7)
	button_state = ((PINF & (1 << 7)) >> 7);	
	button_state |= ((PINC & (1 << 7)) >> 6);
	button_state = ~button_state & B00000011;
	if (last_buttonpress == 0) {  // never pressed a button
		last_button_state = button_state;
		last_buttonpress = millis();
	}	
	
	// if this is a new button press, measure time from now
	if (button_state != last_button_state) { 
		if (button_state) {							// a button is pressed
			last_buttonpress = millis(); 
			up_or_down(button_state);
		} else {									// buttons have been released. store new addres
			store_dmx_address();													
		}
		last_button_state = button_state;
	}
	
	if (button_state == 0) {
			return;
	}
	
	elapsed_time = millis() - last_buttonpress;
	
	if (elapsed_time < 1000) {;}
	else if (elapsed_time < 3000) {
		if ((elapsed_time % 400) == 0) up_or_down(button_state);
	} else {
		if ((elapsed_time % 20) == 0) up_or_down(button_state);
	}
	
	
 }
 
// ------------------------------------------------------------------------------------------
// get ones, tens or hundreds

unsigned char get_digit(unsigned char d, unsigned int n) {
		if (d == 0) return n % 10;
		n = n / 10;
		if (d == 1) return n % 10;
		n = n / 10;
		return n % 10;
}

// ------------------------------------------------------------------------------------------
// refresh display (called by timer every millisecond)

void update_display(void) {
		static unsigned char d;
		d++;
		d %= 3 + (16 - DISPLAY_BRIGHTNESS);			// adjust brightness here: lower numbers = brighter display. try values from 3 - 30
		if (d < 3)	display_digit(d, get_digit(d,dmx_address + 1));
		else PORTF &= B10001111;	// turn all	 digits off
		
		if (button_state == 0) { 		// dont blink when buttons are pressed
			if (time_since_last_dmx_packet > 3000 ) {
				unsigned long blink;
				blink = time_since_last_dmx_packet % 1000;
				if (blink < 500) {
					PORTF &= B10001111;	// turn all	 digits off
				}
			}
		}
}

// ------------------------------------------------------------------------------------------
// light 7 segments
//
// Weird wiring... Mapping:
//					a - PD1
//					b - PD3
//					c - PF0
//					d - PE6
//					e - PD6
//					f - PD4
//					g - PF1
void display_set_leds( unsigned char l ) {

// turn all LEDS off first
	PORTD &= B10100101;
	PORTE &= B10111111;
	PORTF &= B11111100;
	
	if ( l & (1 << 0 )) { PORTD |= ( 1 << 1 ); }
	if ( l & (1 << 1 )) { PORTD |= ( 1 << 3 ); }
	if ( l & (1 << 2 )) { PORTF |= ( 1 << 0 ); }
	if ( l & (1 << 3 )) { PORTE |= ( 1 << 6 ); }
	if ( l & (1 << 4 )) { PORTD |= ( 1 << 6 ); }
	if ( l & (1 << 5 )) { PORTD |= ( 1 << 4 ); }
	if ( l & (1 << 6 )) { PORTF |= ( 1 << 1 ); }
}

void display_digit( unsigned char n, unsigned char d ) {
	n %= 3;
	d %= 16;
	PORTF &= B10001111;	 // turn all	digits off
	switch(d) {
		case 0: display_set_leds(0x3f); break;
		case 1: display_set_leds(0x06); break;
		case 2: display_set_leds(0x5b); break;
		case 3: display_set_leds(0x4f); break;
		case 4: display_set_leds(0x66); break;
		case 5: display_set_leds(0x6d); break;
		case 6: display_set_leds(0x7d); break;
		case 7: display_set_leds(0x07); break;
		case 8: display_set_leds(0x7f); break;
		case 9: display_set_leds(0x6f); break;
		default: display_set_leds(0x00); break;
	}
	PORTF |= (1 << ( n + 4)); 
}

