// CPU frequency for delay functions
#define F_CPU 16000000UL

#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/sleep.h>
#include <inttypes.h>
#include <stdlib.h>
#include <util/delay.h>

//defaults in eeprom
uint16_t EEMEM temp_hi_default = 40;
uint16_t EEMEM temp_lo_default = 30;
int8_t EEMEM temp_corr_default = 0;

#define HOURS_CAP     6 //maximum hours settable
#define TEMP_CAP      200

// Software SPI
#define SPI_PORT      PORTD
#define SPI_PIN       PIND
#define SPI_DDR       DDRD

// MAX6675 temperature sensor
#define T_CS          PORTD0
#define T_MISO        PIND1
#define T_SCK         PORTD2

// MAX7219 8 digit 7-seg indicator
#define L_CS          PORTD3
#define L_MOSI        PORTD4
#define L_SCK         PORTD5

// Rotary encoder
#define ENC_PORT      PORTC
#define ENC_DDR       DDRC
#define ENC_PIN       PINC
#define ENC_A         PINC0
#define ENC_B         PINC1
#define ENC_KEY       PINC2

// Keys
#define KEY_PORT      PORTD
#define KEY_DDR       DDRD
#define KEY_PIN       PIND
#define KEY_GO        PIND6
#define KEY_MODE      PIND7

// LEDs
#define LED_PORT      PORTB
#define LED_DDR       DDRB
#define LED_GO        PORTB2
#define LED_TIME      PORTB3
#define LED_HEAT      PORTB4

// Relay
#define RELAY_PORT    PORTB
#define RELAY_DDR     DDRB
#define RELAY         PORTB0

// Previous state of encoder/key pins to determine
// which of the pins changed on PCINT
uint8_t encoder_pins = 0x00;
uint8_t key_pins = 0x00;

// Flags to be set by interrupt handlers
// 1 = key pressed and needs to be handled
#define ENC_KEY_FLAG  0
#define ENC_KEY_FLAG_SPECIAL 1
#define GO_KEY_FLAG   3
#define MODE_KEY_FLAG 4
uint8_t key_flags = 0x00;

// Encoder counter, signed
// Each clockwise impulse is +1, each counter-clockwise is -1
// 4 impulses per 1 click
// Main routine compares as >=4 or <=-4
int8_t encoder_count = 0x00;

#define _NOP() do { __asm__ __volatile__ ("nop"); } while (0)

const uint8_t font[19] = {
	0b01111110, /* 0 */
	0b00110000, /* 1 */
	0b01101101, /* 2 */
	0b01111001, /* 3 */
	0b00110011, /* 4 */
	0b01011011, /* 5 */
	0b01011111, /* 6 */
	0b01110000, /* 7 */
	0b01111111, /* 8 */
	0b01111011, /* 9 */
	0b01100011, /* degree sign */
	0b00000101, /* r */
	0b00001110, /* L */
	0b00110111, /* H */
	0b01001111, /* E */
	0b00000000, /* space */
	0b01100111, /* P */
	0b01110110, /* N */
	0b00000001  /* - */
};

#define MAX7219_MODE_DECODE       0x09
#define MAX7219_MODE_INTENSITY    0x0A
#define MAX7219_MODE_SCAN_LIMIT   0x0B
#define MAX7219_MODE_POWER        0x0C
#define MAX7219_MODE_TEST         0x0F
#define MAX7219_MODE_NOOP         0x00
#define MAX7219_ON                1
#define MAX7219_OFF               0


//current temperature reading
uint16_t temp_raw;
uint16_t temp_degrees; // temperature up to 1024 degrees

//remaining time reading
uint8_t remaining_hours = 0; //ones of hours
uint8_t remaining_dec_minutes = 0; //tens of minutes
uint8_t remaining_minutes = 0; //ones of minutes
uint8_t remaining_seconds = 0;

//temperature correction
int8_t temp_corr;
int8_t temp_corr_edited;

//maximum temperature
uint16_t temp_hi_degrees;
uint8_t temp_hi_internal[4] = {0b01110110, 0b01111110, 0b01110110, 0b01001111}; //NONE

//minimum temperature
uint16_t temp_lo_degrees;
uint8_t temp_lo_internal[4] = {0b01110110, 0b01111110, 0b01110110, 0b01001111}; //NONE

//buffers to display temperature and time (0 - leftmost, 3 - rightmost)
uint8_t temp_display[4] = {0, 0, 0, 0};
uint8_t time_display[4] = {0, 0, 0, 0};

uint8_t go_led_on = 0;
uint8_t time_led_on = 0;
uint8_t heat_led_on = 0;

//internal temperature buffer, because we don't always
//show temperature, but need to have it on hand
uint8_t temp_internal[4] = {0b01110110, 0b01111110, 0b01110110, 0b01001111}; //NONE

//same thing goes for time
uint8_t time_internal[4] = {0b01110110, 0b01111110, 0b01110110, 0b01001111}; //NONE

//led/indicator flash, updated by timer 5Hz
//1=show, 0=blank
uint8_t flash = 0;
uint8_t temp_indicator_flashing = 0;
uint8_t time_indicator_flashing = 0;

//+1 every 200 ms, temperature is read if counter >= 5, then reset
//= read temp every 1s
uint8_t must_read_temp_counter = 0;
//same for seconds counter
uint8_t must_decrement_seconds_counter = 0;

//machine state
enum state_enum {
	TEMP_CORR,
	GO_CURRENT,
	GO_HI,
	GO_LO,
	SHOW_CURRENT,
	EDIT_TIME,
	SHOW_HI,
	EDIT_HI,
	SHOW_LO,
	EDIT_LO
};
enum state_enum state = SHOW_CURRENT;

enum go_enum {
	CURRENT,
	HI,
	LO
};

//write one 16-bit word to indicator (register address + data)
void indicator_out(uint8_t reg, uint8_t data) {
	uint16_t data_out = reg << 8 | data;
	uint8_t i;

	//SCK=0
	SPI_PORT &= ~_BV(L_SCK);
	_NOP(); _NOP(); _NOP(); _NOP();

	//CS=0
	SPI_PORT &= ~_BV(L_CS);
	_NOP(); _NOP(); _NOP(); _NOP();

	for(i=0; i<16; i++) {
		//SCK=0
		SPI_PORT &= ~_BV(L_SCK);
		_NOP(); _NOP(); _NOP(); _NOP();

		if (data_out & 0x8000) {
			SPI_PORT |= _BV(L_MOSI);
			} else {
			SPI_PORT &= ~_BV(L_MOSI);
		}

		//SCK=1
		SPI_PORT |= _BV(L_SCK);
		_NOP(); _NOP(); _NOP(); _NOP();

		data_out <<= 1;
	}

	//CS=0
	SPI_PORT |= _BV(L_CS);
	_NOP(); _NOP(); _NOP(); _NOP();

	//SCK=0
	SPI_PORT &= ~_BV(L_SCK);
	_NOP(); _NOP(); _NOP(); _NOP();
}

//initialize indicator (intensity, no decode, activate all digits)
void indicator_init() {
	indicator_out(MAX7219_MODE_DECODE, 0x00);
	indicator_out(MAX7219_MODE_SCAN_LIMIT, 0x07);
	indicator_out(MAX7219_MODE_INTENSITY, 0x08);
	indicator_out(MAX7219_MODE_POWER, MAX7219_ON);
}

//read raw temperature as 16-bit word
uint16_t temperature_read() {
	uint8_t i;
	uint16_t data_in = 0;

	//SCK=0
	SPI_PORT &= ~_BV(T_SCK);
	_NOP(); _NOP(); _NOP(); _NOP();

	//CS=0
	SPI_PORT &= ~_BV(T_CS);
	_NOP(); _NOP(); _NOP(); _NOP();

	for(i=0; i<16; i++)
	{
		//SCK=1
		SPI_PORT |= _BV(T_SCK);
		_NOP(); _NOP(); _NOP(); _NOP();

		//shift data_in, 0 shifts in
		data_in <<= 1;
		if (SPI_PIN & _BV(T_MISO)) {
			data_in |= 0x0001;
		}

		//SCK=0
		SPI_PORT &= ~_BV(T_SCK);
		_NOP(); _NOP(); _NOP(); _NOP();
	}

	//CS=0
	SPI_PORT |= _BV(T_CS);
	_NOP(); _NOP(); _NOP(); _NOP();

	return data_in;
}

//initialize interrupts/timers
void interrupt_init()
{
	TCCR1A = 0;
	TCCR1B = _BV(CS11) | _BV(CS10) | _BV(WGM12);

	// F = 5 Hz
	OCR1A = 0xF424;

	// Timer/Counter0 output compare match A interrupt enable
	TIMSK1 = _BV(OCIE1A);


	// *** KEY/ENCODER ***
	//encoder
	PCICR |= _BV(PCIE1);
	PCMSK1 |= _BV(ENC_A) | _BV(ENC_B) | _BV(ENC_KEY);
	//keys
	PCICR |= _BV(PCIE2);
	PCMSK2 |= _BV(KEY_GO) | _BV(KEY_MODE);
}

//starting indicator animation + delay
void indicator_animate()
{
	const uint8_t animation[12] = {
		0b00010000,
		0b00110000,
		0b01110000,
		0b01110010,
		0b01110110,
		0b01111110,
		0b01101110,
		0b01001110,
		0b00001110,
		0b00001100,
		0b00001000,
		0b00000000
	};

	uint8_t frame, digit, digit_in;
	for (digit = 1; digit <= 8; digit++) {
		for (frame = 0; frame < 12; frame++) {
			indicator_init(); //prophylactic
			for (digit_in = 1; digit_in <= 8; digit_in++) {
				if (digit_in == digit) {
					indicator_out(digit_in, animation[frame]);
					} else {
					indicator_out(digit_in, 0x00);
				}
			}
			_delay_ms(50);
		}
	}

	//clear indicator
	indicator_init(); //prophylactic
	for (digit = 1; digit <= 8; digit++) {
		indicator_out(digit, 0x00);
	}
}

//intitialize pins (in/out)
void pin_init()
{
	// all relevant spi ports are outputs
	SPI_DDR |= _BV(T_CS) | _BV(T_SCK) | _BV(L_CS) | _BV(L_SCK) | _BV(L_MOSI);

	// all LEDs off
	LED_PORT |= _BV(LED_GO) | _BV(LED_TIME) | _BV(LED_HEAT);
	// all LEDs are outputs
	LED_DDR |= _BV(LED_GO) | _BV(LED_TIME) | _BV(LED_HEAT);

	// relay off
	RELAY_PORT &= ~_BV(RELAY);
	// relay is output
	RELAY_DDR |= _BV(RELAY);
}

// encoder and encoder key handler
ISR(PCINT1_vect)
{
	uint8_t encoder_pins_now = ENC_PIN;

	if ((encoder_pins & _BV(ENC_KEY)) && !(encoder_pins_now & (_BV(ENC_KEY)))) {
		//special case, if encoder key is released while mode key is pressed, this is a special event
		if (key_pins & _BV(KEY_MODE)) {
			//Encoder key special (save to eeprom)
			key_flags |= _BV(ENC_KEY_FLAG_SPECIAL);
			} else {
			//Encoder key went from 1 to 0 (released)
			key_flags |= _BV(ENC_KEY_FLAG);
		}
	}

	if ((encoder_pins & (_BV(ENC_A) | _BV(ENC_B))) != (encoder_pins_now & (_BV(ENC_A) | _BV(ENC_B)))) {
		//encoder changed position

		uint8_t enc_was = 0;
		if (encoder_pins & _BV(ENC_A)) {
			enc_was |= _BV(0);
		}
		if (encoder_pins & _BV(ENC_B)) {
			enc_was |= _BV(1);
		}

		uint8_t enc_now = 0;
		if (encoder_pins_now & _BV(ENC_A)) {
			enc_now |= _BV(0);
		}
		if (encoder_pins_now & _BV(ENC_B)) {
			enc_now |= _BV(1);
		}

		//Grey code decoding clockwise 00->01->11->10 (0-1-3-2)
		if ((enc_was == 0 && enc_now == 1)
		|| (enc_was == 1 && enc_now == 3)
		|| (enc_was == 3 && enc_now == 2)
		|| (enc_was == 2 && enc_now == 0)) {
			encoder_count++;
		}

		//Grey code decoding counter-clockwise 00->10->11->01 (0-2-3-1)
		if ((enc_was == 0 && enc_now == 2)
		|| (enc_was == 2 && enc_now == 3)
		|| (enc_was == 3 && enc_now == 1)
		|| (enc_was == 1 && enc_now == 0)) {
			encoder_count--;
		}
	}

	encoder_pins = encoder_pins_now;
}

// key handler
ISR(PCINT2_vect)
{
	uint8_t key_pins_now = KEY_PIN;

	if ((key_pins & _BV(KEY_GO)) && !(key_pins_now & (_BV(KEY_GO)))) {
		//KEY_GO went from 1 to 0 (released)
		key_flags |= _BV(GO_KEY_FLAG);
	}
	if ((key_pins & _BV(KEY_MODE)) && !(key_pins_now & (_BV(KEY_MODE)))) {
		//KEY_MODE went from 1 to 0 (released)
		key_flags |= _BV(MODE_KEY_FLAG);
	}
	key_pins = key_pins_now;
}

//update indicator with current data in buffers
void indicator_update()
{
	uint8_t i;

	for (i = 0; i < 4; i++) {
		if (time_indicator_flashing && !flash) {
			indicator_out((4 - i), 0x00);
			} else {
			indicator_out((4 - i), time_display[i]);
		}

		if (temp_indicator_flashing && !flash) {
			indicator_out((8 - i), 0x00);
			} else {
			indicator_out((8 - i), temp_display[i]);
		}
	}

	if (go_led_on) {
		LED_PORT &= ~_BV(LED_GO);
		} else {
		LED_PORT |= _BV(LED_GO);
	}

	if (time_led_on && flash) {
		LED_PORT &= ~_BV(LED_TIME);
		} else {
		LED_PORT |= _BV(LED_TIME);
	}

	if (heat_led_on) {
		LED_PORT &= ~_BV(LED_HEAT);
		} else {
		LED_PORT |= _BV(LED_HEAT);
	}
}

void parse_temperature(uint16_t temp, uint8_t* result)
{
	// convert to BCD
	char buffer[6] = {0, 0, 0, 0, 0, 0};
	itoa(temp, buffer, 10);

	// clear temperature display
	result[0] = 0;
	result[1] = 0;
	result[2] = 0;
	result[3] = font[10]; // degree sign

	// go from end of buffer, and print all non-null characters
	int8_t n;
	int8_t k = 2;

	for (n=5; n >= 0; n--) {
		if (buffer[n] != 0 && k >= 0) {
			result[k] = font[buffer[n]-48];
			k--;
		}
	}
}

void read_temperature()
{
	temp_raw = temperature_read();

	//parse temperature
	if (temp_raw & _BV(2)) {
		// thermocouple open
		temp_internal[0] = font[0];  // O
		temp_internal[1] = font[16]; // P
		temp_internal[2] = font[14]; // E
		temp_internal[3] = font[17]; // N
		} else {
		// get temperature in full degrees
		// discarding 2 LSBs
		temp_degrees = (temp_raw & 0x7FF8) >> (3 + 2);

		//add correction
		temp_degrees += temp_corr;

		// guarantee 3 digit temperature
		// not much is lost from 1024 degrees anyway
		if (temp_degrees > 999) {
			temp_degrees = 999;
		}
	}
}

//returns 0 if all ok, and 1 if overflow
int time_inc(uint8_t* hours, uint8_t* dec_minutes, uint8_t* minutes)
{
	uint8_t l_hours = *hours;
	uint8_t l_dec_minutes = *dec_minutes;
	uint8_t l_minutes = *minutes;

	//+1 minute
	if (l_minutes == 9) {
		if (l_dec_minutes == 5) {
			if (l_hours == (HOURS_CAP - 1)) {
				//overflow
				return 1;
				} else {
				l_hours++;
				l_dec_minutes = 0;
				l_minutes = 0;
			}
			} else {
			l_dec_minutes++;
			l_minutes = 0;
		}
		} else {
		l_minutes++;
	}

	*hours = l_hours;
	*dec_minutes = l_dec_minutes;
	*minutes = l_minutes;
	return 0;
}

//returns 0 if all ok, and 1 if overflow
int time_dec(uint8_t* hours, uint8_t* dec_minutes, uint8_t* minutes)
{
	uint8_t l_hours = *hours;
	uint8_t l_dec_minutes = *dec_minutes;
	uint8_t l_minutes = *minutes;

	//-1 minute
	if (l_minutes == 0) {
		if (l_dec_minutes == 0) {
			if (l_hours == 0) {
				//overflow
				return 1;
				} else {
				l_hours--;
				l_dec_minutes = 5;
				l_minutes = 9;
			}
			} else {
			l_dec_minutes--;
			l_minutes = 9;
		}
		} else {
		l_minutes--;
	}

	*hours = l_hours;
	*dec_minutes = l_dec_minutes;
	*minutes = l_minutes;
	return 0;
}

void parse_time()
{
	//"increment" timer if there are remaining seconds
	uint8_t l_hours = remaining_hours;
	uint8_t l_dec_minutes = remaining_dec_minutes;
	uint8_t l_minutes = remaining_minutes;

	if (remaining_seconds > 0) {
		if (time_inc(&l_hours, &l_dec_minutes, &l_minutes)) {
			//cap at max
			l_hours = HOURS_CAP;
			l_dec_minutes = 0;
			l_minutes = 0;
		}
	}

	// clear display
	if (l_hours > 0) {
		time_internal[0] = font[l_hours];
		time_internal[1] = font[18]; // -
		} else {
		time_internal[0] = 0x00; //blank
		time_internal[1] = 0x00; //blank
	}
	time_internal[2] = font[l_dec_minutes];
	time_internal[3] = font[l_minutes];
}

void clear_all_flags()
{
	//clear encoder
	if (abs(encoder_count) % 4) {
		encoder_count = 0;
	}

	//clear keys
	key_flags = 0;
}

void handle_show_current()
{
	//all leds off
	go_led_on = 0;
	time_led_on = 0;
	heat_led_on = 0;

	//relay off
	RELAY_PORT &= ~_BV(RELAY);

	//indicator flashing
	temp_indicator_flashing = 0;
	time_indicator_flashing = 0;

	//show temperature
	temp_display[0] = temp_internal[0];
	temp_display[1] = temp_internal[1];
	temp_display[2] = temp_internal[2];
	temp_display[3] = temp_internal[3];

	//show remaining time
	time_display[0] = time_internal[0];
	time_display[1] = time_internal[1];
	time_display[2] = time_internal[2];
	time_display[3] = time_internal[3];

	if (key_flags & _BV(ENC_KEY_FLAG_SPECIAL)) {
		//go into temperature correction mode
		temp_corr_edited = temp_corr;
		temp_corr = 0;
		state = TEMP_CORR;
		clear_all_flags();
	}

	if (key_flags & _BV(MODE_KEY_FLAG)) {
		//mode key pressed
		state = SHOW_HI;
		clear_all_flags();
	}

	if (key_flags & _BV(ENC_KEY_FLAG)) {
		//encoder key pressed
		state = EDIT_TIME;
		clear_all_flags();
	}

	if (key_flags & _BV(GO_KEY_FLAG)) {
		//go key pressed
		state = GO_CURRENT;

		//if under HI, start heating
		if (temp_degrees <= temp_hi_degrees) {
			heat_led_on = 1;
			//relay on
			RELAY_PORT |= _BV(RELAY);
		}
		clear_all_flags();
	}
}

void handle_edit_time()
{
	//all leds off
	go_led_on = 0;
	time_led_on = 0;
	heat_led_on = 0;

	//relay off
	RELAY_PORT &= ~_BV(RELAY);

	//indicator flashing
	temp_indicator_flashing = 0;
	time_indicator_flashing = 1;

	//show temperature
	temp_display[0] = temp_internal[0];
	temp_display[1] = temp_internal[1];
	temp_display[2] = temp_internal[2];
	temp_display[3] = temp_internal[3];

	//show remaining time
	time_display[0] = time_internal[0];
	time_display[1] = time_internal[1];
	time_display[2] = time_internal[2];
	time_display[3] = time_internal[3];

	if (key_flags & _BV(ENC_KEY_FLAG)) {
		//encoder key pressed
		state = SHOW_CURRENT;
		clear_all_flags();
	}

	//handle encoder rotation
	if (encoder_count >= 4) {
		encoder_count -= 4;

		//+1 minute
		if (time_inc(&remaining_hours, &remaining_dec_minutes, &remaining_minutes)) {
			//overflow
			remaining_seconds = 59;
		}
		parse_time();
	}

	if (encoder_count <= -4) {
		encoder_count += 4;

		//-1 minute
		if (time_dec(&remaining_hours, &remaining_dec_minutes, &remaining_minutes)) {
			//overflow
			remaining_seconds = 0;
		}
		parse_time();
	}
}

void handle_show_hi()
{
	//all leds off
	go_led_on = 0;
	time_led_on = 0;
	heat_led_on = 0;

	//relay off
	RELAY_PORT &= ~_BV(RELAY);

	//indicator flashing
	temp_indicator_flashing = 0;
	time_indicator_flashing = 0;

	//show temperature
	temp_display[0] = temp_hi_internal[0];
	temp_display[1] = temp_hi_internal[1];
	temp_display[2] = temp_hi_internal[2];
	temp_display[3] = temp_hi_internal[3];

	//time display = " HI "
	time_display[0] = 0;
	time_display[1] = font[13];
	time_display[2] = font[1];
	time_display[3] = 0;

	if (key_flags & _BV(MODE_KEY_FLAG)) {
		//mode key pressed
		state = SHOW_LO;
		clear_all_flags();
	}

	if (key_flags & _BV(ENC_KEY_FLAG)) {
		//encoder key pressed
		state = EDIT_HI;
		clear_all_flags();
	}
}

void handle_edit_hi()
{
	//all leds off
	go_led_on = 0;
	time_led_on = 0;
	heat_led_on = 0;

	//relay off
	RELAY_PORT &= ~_BV(RELAY);

	//indicator flashing
	temp_indicator_flashing = 1;
	time_indicator_flashing = 0;

	//show temperature
	temp_display[0] = temp_hi_internal[0];
	temp_display[1] = temp_hi_internal[1];
	temp_display[2] = temp_hi_internal[2];
	temp_display[3] = temp_hi_internal[3];

	//time display = " HI "
	time_display[0] = 0;
	time_display[1] = font[13];
	time_display[2] = font[1];
	time_display[3] = 0;

	if (key_flags & _BV(ENC_KEY_FLAG_SPECIAL)) {
		//save to EEPROM
		eeprom_write_word (&temp_hi_default, temp_hi_degrees);

		//show "EE"
		time_display[0] = 0;
		time_display[1] = font[14]; //E
		time_display[2] = font[14]; //E
		time_display[3] = 0;
		indicator_update();
		_delay_ms(1000);

		state = SHOW_HI;
		clear_all_flags();
	}

	if (key_flags & _BV(ENC_KEY_FLAG)) {
		//encoder key pressed
		state = SHOW_HI;
		clear_all_flags();
	}

	//handle encoder rotation
	if (encoder_count >= 4) {
		encoder_count -= 4;
		if (temp_hi_degrees < TEMP_CAP) {
			temp_hi_degrees++;
		}
		parse_temperature(temp_hi_degrees, temp_hi_internal);
	}

	if (encoder_count <= -4) {
		encoder_count += 4;
		if (temp_hi_degrees > 0) {
			temp_hi_degrees--;
		}
		parse_temperature(temp_hi_degrees, temp_hi_internal);
	}
}

void handle_show_lo()
{
	//all leds off
	go_led_on = 0;
	time_led_on = 0;
	heat_led_on = 0;

	//relay off
	RELAY_PORT &= ~_BV(RELAY);

	//indicator flashing
	temp_indicator_flashing = 0;
	time_indicator_flashing = 0;

	//show temperature
	temp_display[0] = temp_lo_internal[0];
	temp_display[1] = temp_lo_internal[1];
	temp_display[2] = temp_lo_internal[2];
	temp_display[3] = temp_lo_internal[3];

	//time display = " LO "
	time_display[0] = 0;
	time_display[1] = font[12];
	time_display[2] = font[0];
	time_display[3] = 0;

	if (key_flags & _BV(MODE_KEY_FLAG)) {
		//mode key pressed
		state = SHOW_CURRENT;
		clear_all_flags();
	}

	if (key_flags & _BV(ENC_KEY_FLAG)) {
		//encoder key pressed
		state = EDIT_LO;
		clear_all_flags();
	}
}

void handle_edit_lo()
{
	//all leds off
	go_led_on = 0;
	time_led_on = 0;
	heat_led_on = 0;

	//relay off
	RELAY_PORT &= ~_BV(RELAY);

	//indicator flashing
	temp_indicator_flashing = 1;
	time_indicator_flashing = 0;

	//show temperature
	temp_display[0] = temp_lo_internal[0];
	temp_display[1] = temp_lo_internal[1];
	temp_display[2] = temp_lo_internal[2];
	temp_display[3] = temp_lo_internal[3];

	//time display = " LO "
	time_display[0] = 0;
	time_display[1] = font[12];
	time_display[2] = font[0];
	time_display[3] = 0;

	if (key_flags & _BV(ENC_KEY_FLAG_SPECIAL)) {
		//save to EEPROM
		eeprom_write_word (&temp_lo_default, temp_lo_degrees);

		//show "EE"
		time_display[0] = 0;
		time_display[1] = font[14]; //E
		time_display[2] = font[14]; //E
		time_display[3] = 0;
		indicator_update();
		_delay_ms(1000);

		state = SHOW_LO;
		clear_all_flags();
	}

	if (key_flags & _BV(ENC_KEY_FLAG)) {
		//encoder key pressed
		state = SHOW_LO;
		clear_all_flags();
	}

	//handle encoder rotation
	if (encoder_count >= 4) {
		encoder_count -= 4;
		if (temp_lo_degrees < TEMP_CAP) {
			temp_lo_degrees++;
		}
		parse_temperature(temp_lo_degrees, temp_lo_internal);
	}

	if (encoder_count <= -4) {
		encoder_count += 4;
		if (temp_lo_degrees > 0) {
			temp_lo_degrees--;
		}
		parse_temperature(temp_lo_degrees, temp_lo_internal);
	}
}

void handle_go(enum go_enum go_state)
{
	if (remaining_hours == 0 && remaining_dec_minutes == 0 && remaining_minutes == 0 && remaining_seconds == 0) {
		state = SHOW_CURRENT;
		clear_all_flags();
		return;
	}

	go_led_on = 1;

	if (temp_degrees >= temp_lo_degrees) {
		time_led_on = 1;
		} else {
		time_led_on = 0;
	}

	//if over HI, stop heating
	if (temp_degrees > temp_hi_degrees) {
		heat_led_on = 0;
		//relay off
		RELAY_PORT &= ~_BV(RELAY);
	}

	//if under LO, start heating
	if (temp_degrees <= temp_lo_degrees) {
		heat_led_on = 1;
		//relay on
		RELAY_PORT |= _BV(RELAY);
	}

	//indicator flashing
	temp_indicator_flashing = 0;
	time_indicator_flashing = 0;

	switch (go_state) {
		case CURRENT:
		//show temperature
		temp_display[0] = temp_internal[0];
		temp_display[1] = temp_internal[1];
		temp_display[2] = temp_internal[2];
		temp_display[3] = temp_internal[3];

		//show remaining time
		time_display[0] = time_internal[0];
		time_display[1] = time_internal[1];
		time_display[2] = time_internal[2];
		time_display[3] = time_internal[3];
		break;
		case HI:
		//show temperature
		temp_display[0] = temp_hi_internal[0];
		temp_display[1] = temp_hi_internal[1];
		temp_display[2] = temp_hi_internal[2];
		temp_display[3] = temp_hi_internal[3];

		//time display = " HI "
		time_display[0] = 0;
		time_display[1] = font[13];
		time_display[2] = font[1];
		time_display[3] = 0;
		break;
		case LO:
		//show temperature
		temp_display[0] = temp_lo_internal[0];
		temp_display[1] = temp_lo_internal[1];
		temp_display[2] = temp_lo_internal[2];
		temp_display[3] = temp_lo_internal[3];

		//time display = " LO "
		time_display[0] = 0;
		time_display[1] = font[12];
		time_display[2] = font[0];
		time_display[3] = 0;
		break;
	}

	if (key_flags & _BV(MODE_KEY_FLAG)) {
		//mode key pressed
		switch (go_state) {
			case CURRENT:
			state = GO_HI;
			break;
			case HI:
			state = GO_LO;
			break;
			case LO:
			state = GO_CURRENT;
			break;
		}
		clear_all_flags();
	}

	if (key_flags & _BV(GO_KEY_FLAG)) {
		//go key pressed
		state = SHOW_CURRENT;
		clear_all_flags();
	}
}

// Flashing LED timer handler
ISR(TIMER1_COMPA_vect)
{
	flash ^= 1;
	must_read_temp_counter++;

	//if temperature is within bounds, decrement timer
	if (time_led_on) {
		must_decrement_seconds_counter++;
		if (must_decrement_seconds_counter >= 5) {
			must_decrement_seconds_counter = 0;
			if (remaining_seconds > 0) {
				remaining_seconds--;
				} else {
				if (!time_dec(&remaining_hours, &remaining_dec_minutes, &remaining_minutes)) {
					remaining_seconds = 59;
				}
			}
			parse_time();
		}
	}
}

void handle_temp_corr()
{
	//all leds off
	go_led_on = 0;
	time_led_on = 0;
	heat_led_on = 0;

	//relay off
	RELAY_PORT &= ~_BV(RELAY);

	//indicator flashing
	temp_indicator_flashing = 0;
	time_indicator_flashing = 1;

	//show temperature
	temp_display[0] = temp_internal[0];
	temp_display[1] = temp_internal[1];
	temp_display[2] = temp_internal[2];
	temp_display[3] = temp_internal[3];

	//time display = correction
	if (temp_corr_edited >= 10) {
		temp_corr_edited = 9;
	}
	if (temp_corr_edited <= -10) {
		temp_corr_edited = -9;
	}
	time_display[0] = 0;
	time_display[1] = 0;

	if (temp_corr_edited >= 0) {
		time_display[2] = 0;
		time_display[3] = font[abs(temp_corr_edited)];
		} else {
		time_display[2] = font[18];
		time_display[3] = font[abs(temp_corr_edited)];
	}

	//handle encoder rotation
	if (encoder_count >= 4) {
		encoder_count -= 4;
		if (temp_corr_edited < 9) {
			temp_corr_edited++;
		}
	}

	if (encoder_count <= -4) {
		encoder_count += 4;
		if (temp_corr_edited > -9) {
			temp_corr_edited--;
		}
	}

	if (key_flags & _BV(ENC_KEY_FLAG)) {
		//encoder key pressed
		//save to EEPROM
		eeprom_write_byte((uint8_t *)&temp_corr_default, temp_corr_edited);
		temp_corr = temp_corr_edited;

		//show "EE"
		time_display[0] = 0;
		time_display[1] = font[14]; //E
		time_display[2] = font[14]; //E
		time_display[3] = 0;
		time_indicator_flashing = 0;
		indicator_update();
		_delay_ms(1000);

		state = SHOW_CURRENT;
		clear_all_flags();
	}

}

int main()
{
	pin_init();
	read_temperature();

	//wait 1 second for power to stabilize
	_delay_ms(1000);

	//init from EEPROM
	temp_corr = eeprom_read_byte((uint8_t *)&temp_corr_default);

	temp_hi_degrees = eeprom_read_word(&temp_hi_default);
	if (temp_hi_degrees > TEMP_CAP) {
		temp_hi_degrees = TEMP_CAP;
	}
	parse_temperature(temp_hi_degrees, temp_hi_internal);

	temp_lo_degrees = eeprom_read_word(&temp_lo_default);
	if (temp_lo_degrees  > TEMP_CAP) {
		temp_lo_degrees  = TEMP_CAP;
	}
	parse_temperature(temp_lo_degrees, temp_lo_internal);

	//test LEDs
	LED_PORT &= ~_BV(LED_GO);
	_delay_ms(500);
	LED_PORT |= _BV(LED_GO);
	LED_PORT &= ~_BV(LED_TIME);
	_delay_ms(500);
	LED_PORT |= _BV(LED_TIME);
	LED_PORT &= ~_BV(LED_HEAT);
	_delay_ms(500);
	LED_PORT |= _BV(LED_HEAT);

	//by now temperature has likely stabilized
	read_temperature();
	parse_temperature(temp_degrees, temp_internal);

	remaining_minutes = 0;
	remaining_seconds = 0;
	parse_time();

	//self-test indicator
	indicator_init();
	indicator_animate();
	interrupt_init();

	sei(); // enable interrupts

	// Write your code here
	while (1) {
		//read temperature every 1s
		if (must_read_temp_counter >= 5) {
			must_read_temp_counter = 0;
			read_temperature();
			parse_temperature(temp_degrees, temp_internal);
		}

		switch(state) {
			case TEMP_CORR:
			handle_temp_corr();
			break;
			case GO_CURRENT:
			handle_go(CURRENT);
			break;
			case GO_HI:
			handle_go(HI);
			break;
			case GO_LO:
			handle_go(LO);
			break;
			case SHOW_CURRENT:
			handle_show_current();
			break;
			case EDIT_TIME:
			handle_edit_time();
			break;
			case SHOW_HI:
			handle_show_hi();
			break;
			case EDIT_HI:
			handle_edit_hi();
			break;
			case SHOW_LO:
			handle_show_lo();
			break;
			case EDIT_LO:
			handle_edit_lo();
			break;
		}

		indicator_init();
		indicator_update();
	}

	return 0;
}
