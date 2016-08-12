#include <Servo.h>
#include <DMXSerial.h>
#include <EEPROM.h>
#include "Timer.h"	 //http://github.com/JChristensen/Timer

#define PWM 1
#define SERVO 2

// ================================================================================================
//																			ADJUST THESE VALUES

#define DMX_ADDRESS				10
#define	CAN_CHANGE_DMX_ADDRESS	1
#define DISPLAY_BRIGHTNESS		14			// values: 0 - 16


const char modes [] =  			{	SERVO	, 	SERVO	, 	SERVO	, 	PWM		, 	PWM		, 	PWM		};
const int lows [] = 			{	700		,	1000	,	1000	,	1000	,	1000	,	1000	};
const int highs [] = 			{	2290	,	2000	,	2000	,	2000	,	2000	,	2000	};

const int start[] = 			{	1500	,	1500	,	1500	,	1500	,	1500	,	1500	};
const char smooth [] = 			{	4		,	0		,	3		,	16		,	0		,	0		};

// ================================================================================================
// ================================================================================================
//																			DONT CHANGE BELOW...

const char 		out_pins[] = {3,5,6,9,10,11};		// Servos are on pins 3,5,6 9,10,11

unsigned int 	dmx_address;
unsigned char 	display[3];
float			dmx_buffer[6];
int 			out_buffer[6];
unsigned long 	time_since_last_dmx_packet;
unsigned char 	button_state;
Timer 			t;
Servo 			myservo[6];
unsigned char 	out_idx;

// ==========================================================================================
//														SETUP
// ------------------------------------------------------------------------------------------
void setup() {					

	// set pinModes per Port. LED anodes and cathodes are all outputs
	DDRD = ~B10100101;
	DDRF = ~B10001100;
	DDRE = ~B10111111;
	
	// retrieve dmx address from EEPROM
	dmx_address = DMX_ADDRESS;
	
#if CAN_CHANGE_DMX_ADDRESS
	unsigned char lowbyte,highbyte,controlbyte;
	lowbyte = EEPROM.read(0);
	highbyte = EEPROM.read(1);
	controlbyte =  EEPROM.read(2);
  	if (lowbyte == controlbyte) { 	// check consitency : low byte has been stored at address 0 and 2 
		if (highbyte < 2 ) {			// dmx address cannot be higher than 511, so high byte can be either 0 or 1
			dmx_address = lowbyte | (highbyte << 8 );
		} 
  	};
  	if (dmx_address < 1) dmx_address = 506;
	if (dmx_address > 506) dmx_address = 1;
	int_to_display(dmx_address);		// write dmx adress to display buffer 
#endif
	
	
	
	DMXSerial.init(DMXReceiver);
	time_since_last_dmx_packet = DMXSerial.noDataSince();

	for (char i = 0; i < 6; i++) {
		if (modes[i] == SERVO) {
				myservo[i].attach(out_pins[i]);
				out_buffer[i] = start[i];
				myservo[i].writeMicroseconds(out_buffer[i]);
		}
	}
	
	delay(1000);

	//pullups on pushbuttons. PF7 and PB3
	PORTF |= ( 1 << 7 ); DDRF &= ~( 1 << 7 );
	PORTB |= ( 1 << 3 ); DDRB &= ~( 1 << 3 );
	pinMode(13, OUTPUT);	// LED on pin 13
	
	
	// set up timed events
#if CAN_CHANGE_DMX_ADDRESS
	t.every (1, update_display);
	t.every (20, check_btns);
#endif
	t.every (10, check_dmx);
}

// ==========================================================================================
//														LOOP
// ------------------------------------------------------------------------------------------

void loop() {
	t.update();
	
	out_idx++;
	out_idx %= 6;
	
	if (modes[out_idx] == SERVO) {
		myservo[out_idx].writeMicroseconds(out_buffer[out_idx]);
	} else {
		analogWrite(out_pins[out_idx], out_buffer[out_idx]);
	}

}

// ==========================================================================================
//														SUBFUNCTIONS
// ------------------------------------------------------------------------------------------

// ------------------------------------------------------------------------------------------
// check for dmx data

void check_dmx() {
	time_since_last_dmx_packet = DMXSerial.noDataSince();
	unsigned char i;
	i= 0;
	
	if (time_since_last_dmx_packet < 1000) {
		digitalWrite(13,HIGH);
	
		unsigned int temp;
		
		for (i = 0; i < 6; i++) {
			
			temp = DMXSerial.read(dmx_address + i);

			dmx_buffer[i] = ((float)smooth[i] * dmx_buffer[i] + (float)temp) / (float)(smooth[i] + 1);
			temp = round(dmx_buffer[i]);

			//int_to_display(temp);
	
			if (modes[i] == SERVO) {
				temp = map(temp, 0, 255, lows [i], highs[i]);
			}
			
			out_buffer[i] = temp;
		}
	} else {
			digitalWrite(13,LOW);
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
	if (dmx_address < 1) dmx_address = 506;
	if (dmx_address > 506) dmx_address = 1;
	int_to_display(dmx_address);
}

// ------------------------------------------------------------------------------------------
// detect button presses (called by timer every 20ms)

void check_btns(void) { 

	static unsigned char last_button_state;
	static unsigned long last_buttonpress;
	unsigned long elapsed_time;
	
	// buttons are on pin A0 (PF7) and MISO (PB3)
	button_state = ((PINF & (1 << 7)) >> 6);	
	button_state |= ((PINB & (1 << 3)) >> 3);
		

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
// write a number to display buffer

void int_to_display (unsigned int number) {
	for (char i = 0; i < 3; i++) {
		display[i] = leds_for_digit(get_digit(i,number));
	}
}

// ------------------------------------------------------------------------------------------
// refresh display (called by timer every millisecond)

void update_display(void) {

		static unsigned char d;
		d++;
		d %= 3 + (16 - DISPLAY_BRIGHTNESS);			// adjust brightness here: lower numbers = brighter display. try values from 3 - 30
		if (d < 3)	display_set_leds(d);
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
void display_set_leds( unsigned char n ) {
	
	n %= 3;

	PORTF &= B10001111;	 // turn all cathodes off
						 // turn all anodes off
	PORTD &= B10100101;
	PORTE &= B10111111;
	PORTF &= B11111100;
	
	unsigned char l;
	l = display[n];
	
	if ( l & (1 << 0 )) { PORTD |= ( 1 << 1 ); }
	if ( l & (1 << 1 )) { PORTD |= ( 1 << 3 ); }
	if ( l & (1 << 2 )) { PORTF |= ( 1 << 0 ); }
	if ( l & (1 << 3 )) { PORTE |= ( 1 << 6 ); }
	if ( l & (1 << 4 )) { PORTD |= ( 1 << 6 ); }
	if ( l & (1 << 5 )) { PORTD |= ( 1 << 4 ); }
	if ( l & (1 << 6 )) { PORTF |= ( 1 << 1 ); }
	
	PORTF |= (1 << ( n + 4)); // turn on cathode
}

unsigned char leds_for_digit(unsigned char d ) {
	switch(d) {
		case 0: return 0x3f;
		case 1: return 0x06;
		case 2: return 0x5b;
		case 3: return 0x4f;
		case 4: return 0x66;
		case 5: return 0x6d;
		case 6: return 0x7d;
		case 7: return 0x07;
		case 8: return 0x7f;
		case 9: return 0x6f;
		default: return 0x00;
	}
}


