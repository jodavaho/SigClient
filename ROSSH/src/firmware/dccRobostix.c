/*
 * dccRobostix.c
 *
 *  Created on: Dec 7, 2011
 *      Author: tokekar
 */

#define SCL_CLOCK			100000
#define DCC_SMALL_TIME		100
#define DCC_LARGE_TIME		1000
#define DCC_MIN_FREQ		48000
#define DCC_MAX_FREQ		51000
#define MAX_PACKET_SIZE		100 //Just as a precaution to avoid memory overflow
#define SCREEN				9

//	User files
#include "protocol.h"
#include "keymap.h"

//	Compiler header files for avr processor
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <stdio.h>
#include <util/atomic.h>
#include <util/delay.h>
#include <inttypes.h>
#include <compat/twi.h>

/*----------------------------------------------------------*/
#define TRUE 				1
#define FALSE 				0

#define RED_LED_ON()		(PORTG &= 0b11101111)
#define RED_LED_OFF()		(PORTG |= 0b00010000)
#define	BLUE_LED_ON()		(PORTG &= 0b11110111)
#define BLUE_LED_OFF()		(PORTG |= 0b00001000)
#define	YELLOW_LED_ON()		(PORTB &= 0b11101111)
#define YELLOW_LED_OFF()	(PORTB |= 0b00010000)

#define LIST_TABLE			1
#define SINGLE_TABLE_48 	2
#define SINGLE_TABLE_49 	3
#define MAX_LIST_SIZE		30

/*----------------------------------------------------------*/
// Prototypes
void bootUp(void);

void initLEDs(void);

void startSerial(void);
void startLoopTimer(void);
void sendAck(void);
void putCharUART1(unsigned char);

void initI2C(void);
unsigned char startI2C(unsigned char);
void stopI2C(void);
unsigned char writeI2C(unsigned char);
unsigned char readI2CAck(void);
unsigned char readI2CNack(void);
void changeGain(unsigned char gain);

void initKeypad(void);
void setKeypadCoords(unsigned char row, unsigned char col);
void setKeypadMap(enum keymap key);
void setFreq(unsigned char *table);
void resetToMenu(void);
void startAerialLogging(unsigned char choice, unsigned char table);
void startManualLogging(unsigned long freq);
void sendDeviceId(void);
void resetAll(void);

/*----------------------------------------------------------*/
unsigned char LOOP_COMPLETE = FALSE;
unsigned int err = 99;

volatile unsigned int time = 0;
volatile unsigned int millisec = 0;
volatile unsigned char rxBuffer[MAX_PACKET_SIZE-1];
volatile unsigned int rxInx = 1;
volatile unsigned int rxPacketSize = 0;
volatile unsigned char PACKET_RECEIVED = FALSE;

volatile unsigned char rxScreenBuffer[SCREEN];
volatile unsigned int rxScreenInx = 0;
volatile unsigned int rxScreenLine = 1;
volatile unsigned int rxLineInx = 0;
volatile unsigned char rxScreenReady = FALSE;
volatile unsigned char SCREEN_RECEIVED = FALSE;
unsigned char MANUAL_MODE = FALSE;

volatile unsigned char NEW_SCREEN = TRUE;

unsigned char rxBufferBk[MAX_PACKET_SIZE-1];

unsigned int freqTable[MAX_LIST_SIZE];
unsigned char freqTableSize = 0;

static FILE myStdout = FDEV_SETUP_STREAM(putCharUART1, NULL, _FDEV_SETUP_WRITE);

/*----------------------------------------------------------*/
void toggleRedLed()
{
	if(bit_is_set(PORTG,4))
		RED_LED_ON();
	else
		RED_LED_OFF();
}

void toggleBlueLed()
{
	if(bit_is_set(PORTG,3))
		BLUE_LED_ON();
	else
		BLUE_LED_OFF();
}

void toggleYellowLed()
{
	if(bit_is_set(PORTB,4))
		YELLOW_LED_ON();
	else
		YELLOW_LED_OFF();
}

/*----------------------------------------------------------*/
/*
 * This function calls other init functions
 */
void bootUp (void)
{
	initLEDs();
	startSerial();
	startLoopTimer();
	initI2C();
	initKeypad();

	// Some LED toggling at the start
	for(char i=0; i<10; i++)
	{
		toggleRedLed();
		toggleYellowLed();
		toggleBlueLed();
		_delay_ms(60);
	}
	RED_LED_OFF();
	YELLOW_LED_OFF();
	BLUE_LED_OFF();

	sei();
}

/*----------------------------------------------------------*/
/*
 * Initialise the ports for input output
 */
void initLEDs(void)
{
	// output port definitions
	DDRG |= 0b00011000;
	DDRB |= 0b00010000;
}

/*----------------------------------------------------------*/

void startSerial0(void){
	DDRE &= 0b11111110;
	DDRE |= 0b00000010;

	UCSR0A = 0b00000010;
	UCSR0B = 0b10011000;
	UCSR0C = 0b00000110;

	UBRR0H = 0;
	UBRR0L = 207;
}

/*
 * Initialise and start USART1 for serial comm. with laptop
 * 57.6kbps, 8-N-1
 * Interrupt on receive complete
 */
void startSerial(void)
{
	// output/input pin definitions
	DDRD &= 0b11111011;
	DDRD |= 0b00001000;

	UCSR1A = 0b00000010;
	UCSR1B = 0b10011000;
	UCSR1C = 0b00000110;

	UBRR1H = 0;
	UBRR1L = 34;

	stdout = &myStdout; //Required for printf init
	startSerial0();
}

/*----------------------------------------------------------*/
/*
 * Check for the start of communication character first
 * The next byte gives the no. of bytes in the packet
 * starting from the start of comm. character
 * Stores it in the serial buffer
 */
ISR(USART1_RX_vect)
{
	toggleYellowLed();
	volatile unsigned int startTime;
	volatile unsigned char temp;

	// Exit if previous packet hasn't been used
	//	if (PACKET_RECEIVED){
	//		YELLOW_LED_ON();
	//		putCharUART1(err);
	//		return;
	//	}


	if((temp = UDR1) == START_COMM)
	{
		//sendAck();
		startTime = time;
		while(!(UCSR1A & (1<<RXC1))) {
			if(time!=startTime) {
				return;
			}
		}

		rxPacketSize = UDR1;
		rxBuffer[0] = rxPacketSize;
		rxInx = 1;
		//		putCharUART1(rxPacketSize);

		//RED_LED_OFF();
		//BLUE_LED_ON();
		//  _delay_ms(3000);

		/*for(int i = 0;i<rxPacketSize;i++){
			RED_LED_ON();
			_delay_ms(DCC_LARGE_TIME);
			RED_LED_OFF();
			_delay_ms(DCC_LARGE_TIME);
		}*/

		return;
	}else{
		// wait for the next character for 1 sec
		if(rxInx < (rxPacketSize-1)){
			rxBuffer[rxInx++] = temp;
			//putCharUART1(temp);
		}else{

			putCharUART1(err);
			return;
		}

		if(rxInx == (rxPacketSize-1)){
			PACKET_RECEIVED = TRUE;
		}
	}
}

ISR(USART0_RX_vect)
{

	volatile unsigned char character = UDR0;

	// TODO fix return signal packets in manual mode
	switch(rxScreenLine){
	case 1:
		if(MANUAL_MODE==FALSE){
			switch(rxLineInx){
			case 0:
				if(character == '1'){
					rxLineInx++;
				}else{
					rxLineInx = 0;
					rxScreenLine = 1;
				}
				return;
			case 1:
				if(character == ':'){
					rxLineInx++;
				}else{
					rxLineInx = 0;
					rxScreenLine = 1;
				}
				return;
			case 2:
			case 3:
			case 4:
			case 5:
			case 6:
			case 7:
			case 8:
			case 9:
			case 10:
				rxLineInx++;
				return;
			case 11:
			case 12:
			case 13:
			case 14:
				rxScreenBuffer[rxScreenInx++] = character;
				rxLineInx++;
				return;
			case 15:
				rxScreenBuffer[rxScreenInx++] = character;
				char a[6];
				for (int i = 0; i < 5;i++)
					a[i] = rxScreenBuffer[i];
				a[5] = '\0';
				long ret = atol(a);
				long retc = ret;
				rxScreenBuffer[0] = ((unsigned char)(ret>>8));
				rxScreenBuffer[1] = ((unsigned char)(retc));
				rxScreenInx = 2;
				rxScreenLine++;
				rxLineInx = 0;
				return;
			default:
				rxLineInx = 0;
				rxScreenLine = 1;
				return;
			}
		} else { //MANUAL MODE
			switch(rxLineInx){
			case 0:
				if(character == '1'){
					rxLineInx++;
				}else{
					rxLineInx = 0;
					rxScreenLine = 1;
				}
				return;
			case 1:
				if(character == ':'){
					rxLineInx++;
				}else{
					rxLineInx = 0;
					rxScreenLine = 1;
				}
				return;
			case 2:
			case 3:
			case 4:
			case 5:
			case 6:
			case 7:
			case 8:
			case 9:
			case 10:
				rxLineInx++;
				return;
			case 11:
			case 12:
			case 13:
			case 14:
				rxScreenBuffer[rxScreenInx++] = character;
				rxLineInx++;
				return;
			case 15:
				rxScreenBuffer[rxScreenInx++] = character;
				char a[6];
				for (int i = 0; i < 5;i++)
					a[i] = rxScreenBuffer[i];
				a[5] = '\0';
				long ret = atol(a);
				long retc = ret;
				rxScreenBuffer[0] = ((unsigned char)(ret>>8));
				rxScreenBuffer[1] = ((unsigned char)(retc));
				rxScreenInx = 2;
				rxScreenLine++;
				rxLineInx = 0;
				return;
			default:
				rxLineInx = 0;
				rxScreenLine = 1;
				return;
			}
		}
		return;
	case 2:
		switch(rxLineInx){
		case 0:
			if(character == '2'){
				rxLineInx++;
			}else{
				rxLineInx = 0;
				rxScreenLine = 2;
			}
			return;
		case 1:
			if(character == ':'){
				rxLineInx++;
			}else{
				rxLineInx = 0;
				rxScreenLine = 2;
			}
			return;
		case 2:
		case 3:
		case 4:
		case 5:
		case 6:
		case 7:
		case 8:
		case 9:
		case 10:
			rxLineInx++;
			return;
		case 11:
		case 12:
		case 13:
			rxScreenBuffer[rxScreenInx++] = character;
			rxLineInx++;
			return;
		case 14:
			rxScreenBuffer[rxScreenInx++] = character;
			char a[5];
			rxScreenInx = 2;
			for (int i = 0; i < 4;i++)
				a[i] = rxScreenBuffer[rxScreenInx + i];
			a[4] = '\0';
			long ret = atol(a);
			long retc = ret;
			rxScreenBuffer[rxScreenInx] = ((unsigned char)(ret>>8));
			rxScreenBuffer[rxScreenInx + 1] = ((unsigned char)(retc));
			rxScreenInx = 4;
			rxScreenLine++;
			rxLineInx = 0;
			return;
		default:
			rxLineInx = 0;
			rxScreenLine = 2;
		}
		return;
		case 3:
			switch(rxLineInx){
			case 0:
				if(character == '3'){
					rxLineInx++;
				}else{
					rxLineInx = 0;
					rxScreenLine = 3;
				}
				return;
			case 1:
				if(character == ':'){
					rxLineInx++;
				}else{
					rxLineInx = 0;
					rxScreenLine = 3;
				}
				return;
			case 2:
			case 3:
			case 4:
			case 5:
			case 6:
			case 7:
			case 8:
			case 9:
			case 10:
			case 11:
			case 12:
			case 13:
			case 14:
				rxLineInx++;
				return;
			case 15:
			case 16:
				rxScreenBuffer[rxScreenInx++] = character;
				rxLineInx++;
				return;
			case 17:
				rxScreenBuffer[rxScreenInx++] = character;
				char a[4];
				rxScreenInx = 4;
				for (int i = 0; i < 3;i++)
					a[i] = rxScreenBuffer[rxScreenInx + i];
				a[3] = '\0';
				long ret = atol(a);
				rxScreenBuffer[rxScreenInx] = ((unsigned char)ret);
				//rxScreenBuffer[rxScreenInx + 1] = ((unsigned char)(ret>>8));
				rxScreenInx = 5;
				rxScreenLine++;
				rxLineInx = 0;
				return;
			default:
				rxLineInx = 0;
				rxScreenLine = 3;
			}
			return;
			case 4:
				switch(rxLineInx){
				case 0:
					if(character == '4'){
						rxLineInx++;
					}else{
						rxLineInx = 0;
						rxScreenLine = 4;
					}
					return;
				case 1:
					if(character == ':'){
						rxLineInx++;
					}else{
						rxLineInx = 0;
						rxScreenLine = 4;
					}
					return;
				case 2:
				case 3:
				case 4:
				case 5:
				case 6:
					rxLineInx++;
					return;
				case 7:
				case 8:
					rxScreenBuffer[rxScreenInx++] = character;
					rxLineInx++;
					return;
				case 9:
					rxScreenBuffer[rxScreenInx++] = character;
					char a[4];
					rxScreenInx = 5;
					for (int i = 0; i < 3;i++)
						a[i] = rxScreenBuffer[rxScreenInx + i];
					a[3] = '\0';
					long ret = atol(a);
					rxScreenBuffer[rxScreenInx] = ((unsigned char)ret);
					rxScreenLine = 1;
					rxLineInx = 0;
					rxScreenInx = 0;
					SCREEN_RECEIVED = TRUE;
					return;
				default:
					rxLineInx = 0;
					rxScreenLine = 4;
				}
				return;
	}
	return;
}

/*----------------------------------------------------------*/
/*
 * Timer2 is used for setting the control loop
 * Timer runs at clk/64, interrupt on compare match (1000Hz)
 * Counter incremented every interrupt, reset in main loop
 */
void startLoopTimer(void)
{
	TCCR2 = 0b00001011;
	OCR2 = 250;

	LOOP_COMPLETE = FALSE;

	TIMSK |= 0b10000000;
}

ISR(TIMER2_COMP_vect)
{
	LOOP_COMPLETE = TRUE;

	if(millisec > 1000)
	{
		millisec = 0;
		time++;
	}
	else
	{
		millisec++;
	}
}

/*----------------------------------------------------------*/
/*
 * Sends acknowledgment packet back to laptop
 * START_COMM | 3 | ACK
 */
void sendAck(void)
{
	putCharUART1(START_COMM);
	putCharUART1(3);
	putCharUART1(ACK);
}

/*----------------------------------------------------------*/
/*
 * Sends a character over USART0
 */
void putCharUART0(unsigned char c)
{
	loop_until_bit_is_set(UCSR0A, UDRE0);
	UDR0 = c;
}


/*----------------------------------------------------------*/
/*
 * Sends a character over USART1
 */
void putCharUART1(unsigned char c)
{
	loop_until_bit_is_set(UCSR1A, UDRE1);
	UDR1 = c;
}


/*----------------------------------------------------------*/
/*
 * Initialize TWI clock: 100kHz, TWPS = 0 => prescaler = 1
 */
void initI2C(void)
{

	DDRD &= 0b11111100;
	// Pull ups for SDA, SCL
	PORTD |= 0b00000011;

	TWSR = 0;
	TWBR = ((F_CPU/SCL_CLOCK)-16)/2;
}

/*----------------------------------------------------------*/
/*
 * Issues a start cond., sends address and transfer direction
 * Return 0 = device accessible, 1= failed to access device
 */
unsigned char startI2C(unsigned char address)
{
	uint8_t twst;

	// send START condition
	TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);

	// wait until transmission completed
	while(!(TWCR & (1<<TWINT)));

	// check value of TWI Status Register. Mask prescaler bits.
	twst = TW_STATUS & 0xF8;
	if ( (twst != TW_START) && (twst != TW_REP_START)) return 1;

	// send device address
	TWDR = address;
	TWCR = (1<<TWINT) | (1<<TWEN);

	// wail until transmission completed and ACK/NACK has been received
	while(!(TWCR & (1<<TWINT)));

	// check value of TWI Status Register. Mask prescaler bits.
	twst = TW_STATUS & 0xF8;
	if ( (twst != TW_MT_SLA_ACK) && (twst != TW_MR_SLA_ACK) ) return 1;

	return 0;
}

/*----------------------------------------------------------*/
/*
 * Terminates the data transfer and releases the I2C bus
 */
void stopI2C(void)
{
	// send stop condition
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);

	// wait until stop condition is executed and bus released
	while(TWCR & (1<<TWSTO));
}

/*----------------------------------------------------------*/
/*
 * Send one byte to I2C device
 * Input:	byte to be transfered
 * Return:  0 write successful
 *          1 write failed
 */
unsigned char writeI2C( unsigned char data )
{
	uint8_t   twst;

	// send data to the previously addressed device
	TWDR = data;
	TWCR = (1<<TWINT) | (1<<TWEN);

	// wait until transmission completed
	while(!(TWCR & (1<<TWINT)));

	// check value of TWI Status Register. Mask prescaler bits
	twst = TW_STATUS & 0xF8;
	if( twst != TW_MT_DATA_ACK) return 1;
	return 0;

}

/*----------------------------------------------------------*/
/*
 * Read one byte from the I2C device, request more data from device
 * Return:  byte read from I2C device
 */
unsigned char readI2CAck(void)
{
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
	while(!(TWCR & (1<<TWINT)));

	return TWDR;
}

/*----------------------------------------------------------*/
/*
 * Read 1 byte from I2C device, read is followed by stop cond.
 * Return:  byte read from I2C device
 */
unsigned char readI2CNack(void)
{
	TWCR = (1<<TWINT) | (1<<TWEN);
	while(!(TWCR & (1<<TWINT)));

	return TWDR;
}

/*----------------------------------------------------------*/
/*
 * Change gain from 0 - 100
 */
void changeGain(unsigned char gain)
{
	if (startI2C(44)){ // Transmit to device 0x2C
		// If there's a problem it flashes the red led
		RED_LED_ON();
		YELLOW_LED_ON();
		_delay_ms(DCC_LARGE_TIME);
		RED_LED_OFF();
		YELLOW_LED_OFF();
		_delay_ms(DCC_LARGE_TIME);
		RED_LED_ON();
		YELLOW_LED_ON();
		_delay_ms(DCC_LARGE_TIME);
		RED_LED_OFF();
		YELLOW_LED_OFF();
		return;
	}
	if (!writeI2C(0x00)){
		if(!writeI2C(gain)){
			return;
		}
	}
	stopI2C();

	// If there's a problem it flashes the red led
	RED_LED_ON();
	_delay_ms(DCC_LARGE_TIME);
	RED_LED_OFF();
}

/*----------------------------------------------------------*/
/*
 * Initializes IO ports for interfacing with the keypad
 */
void initKeypad(void)
{
	DDRC |= 0b00011111;
}

/*----------------------------------------------------------*/
/*
 * Simulates a key press using coordinates
 * corresponding to the row [0-3] and col [0-4]
 */
void setKeypadCoords(unsigned char row, unsigned char col)
{
	// First make sure that the DEMUX is high
	PORTC |= 0b00000111;

	_delay_ms(DCC_SMALL_TIME);

	// Now select the correct row on MUX
	PORTC |= 0b00011000;
	switch(row)
	{
	case 0: PORTC &= 0b11100111; break;
	case 1: PORTC &= 0b11101111; break;
	case 2: PORTC &= 0b11110111; break;
	case 3: break;
	}
	_delay_ms(DCC_SMALL_TIME);

	// Now select the correct col on DEMUX
	switch(col)
	{
	case 0: PORTC &= 0b11111000; break;
	case 1: PORTC &= 0b11111001; break;
	case 2: PORTC &= 0b11111010; break;
	case 3: PORTC &= 0b11111011; break;
	case 4: PORTC &= 0b11111100; break;
	}
	_delay_ms(DCC_SMALL_TIME);

	// Finally make sure that the DEMUX is high
	PORTC |= 0b00000111;
	toggleRedLed();
}

/*----------------------------------------------------------*/
/*
 * Simulates pressing key, using a keymap instead raw coords
 */
void setKeypadMap(enum keymap key)
{
	switch(key)
	{

	// Row Zero
	case PGRM:	{setKeypadMap(SHIFT); _delay_ms(DCC_SMALL_TIME); setKeypadCoords(0,0); break;}
	case SETUP:	{setKeypadMap(SHIFT); _delay_ms(DCC_SMALL_TIME); setKeypadCoords(0,1); break;}
	case FREQ:	{setKeypadMap(SHIFT); _delay_ms(DCC_SMALL_TIME); setKeypadCoords(0,2); break;}
	case SCR:	{setKeypadMap(SHIFT); _delay_ms(DCC_SMALL_TIME); setKeypadCoords(0,3); break;}
	case TEST:	{setKeypadMap(SHIFT); _delay_ms(DCC_SMALL_TIME); setKeypadCoords(0,4); break;}
	case ZERO:	setKeypadCoords(0,0); break;
	case ONE:	setKeypadCoords(0,1); break;
	case TWO:	setKeypadCoords(0,2); break;
	case THREE:	setKeypadCoords(0,3); break;
	case FOUR:	setKeypadCoords(0,4); break;

	// Row One
	case MEM:	{setKeypadMap(SHIFT); _delay_ms(DCC_SMALL_TIME); setKeypadCoords(1,1); break;}
	case GPS:	{setKeypadMap(SHIFT); _delay_ms(DCC_SMALL_TIME); setKeypadCoords(1,4); break;}
	case FIVE:	setKeypadCoords(1,0); break;
	case SIX:	setKeypadCoords(1,1); break;
	case SEVEN:	setKeypadCoords(1,2); break;
	case EIGHT:	setKeypadCoords(1,3); break;
	case NINE:	setKeypadCoords(1,4); break;

	// Row Two
	case ADDFR:	{setKeypadMap(SHIFT); _delay_ms(DCC_SMALL_TIME); setKeypadCoords(2,2); break;}
	case LEFT:	setKeypadCoords(2,0); break;
	case UP:	setKeypadCoords(2,1); break;
	case CHANUP:	setKeypadCoords(2,2); break;
	case ESC:	setKeypadCoords(2,3); break;
	case ENTER:	setKeypadCoords(2,4); break;

	// Row Three
	case MERGE:	{setKeypadMap(SHIFT); _delay_ms(DCC_SMALL_TIME); setKeypadCoords(3,1); break;}
	case DELFR:	{setKeypadMap(SHIFT); _delay_ms(DCC_SMALL_TIME); setKeypadCoords(3,2); break;}
	case RECORD:	{setKeypadMap(SHIFT); _delay_ms(DCC_SMALL_TIME); setKeypadCoords(3,3); break;}
	case RIGHT:	setKeypadCoords(3,0); break;
	case DOWN:	setKeypadCoords(3,1); break;
	case CHANDOWN:	setKeypadCoords(3,2); break;
	case HOLD:	setKeypadCoords(3,3); break;
	case SHIFT:	setKeypadCoords(3,4); break;

	}
}

/*----------------------------------------------------------*/
/*
 * Starts logging in the aerial mode
 * @param choice Which scan method to use
 * 	choice=0: restart previous scan
 * 	choice=1: scan using default
 *  choice=2: new scan
 * @param table to use for logging
 */
void startAerialLogging(unsigned char choice, unsigned char table)
{

	// Check args
	if(table > 10)
	{
		return;
	}

	resetToMenu();

	setKeypadMap(RIGHT); //cursor to Aerial
	_delay_ms(DCC_SMALL_TIME);
	setKeypadMap(ENTER);
	_delay_ms(DCC_LARGE_TIME);

	MANUAL_MODE = FALSE;

	switch(choice)
	{

	case 2: //new scan
		setKeypadMap(RIGHT);
		_delay_ms(DCC_SMALL_TIME);
		setKeypadMap(RIGHT);
		_delay_ms(DCC_SMALL_TIME);
		setKeypadMap(ENTER);
		_delay_ms(DCC_LARGE_TIME);

		setKeypadMap(table); //Table #
		_delay_ms(DCC_SMALL_TIME);
		setKeypadMap(ENTER);
		_delay_ms(DCC_LARGE_TIME);

		setKeypadMap(THREE); //Scan rate
		_delay_ms(DCC_SMALL_TIME);
		setKeypadMap(ENTER);
		_delay_ms(DCC_SMALL_TIME);
		setKeypadMap(ENTER);
		_delay_ms(DCC_SMALL_TIME);

		setKeypadMap(ENTER); //Auto record
		_delay_ms(DCC_LARGE_TIME);

		setKeypadMap(ONE); //No. of diff. pulserates
		_delay_ms(DCC_SMALL_TIME);
		setKeypadMap(ENTER);
		_delay_ms(DCC_LARGE_TIME);

		setKeypadMap(FOUR); //PR1
		_delay_ms(DCC_SMALL_TIME);
		setKeypadMap(ZERO); //PR1
		_delay_ms(DCC_SMALL_TIME);
		setKeypadMap(ENTER);
		_delay_ms(DCC_LARGE_TIME);
		setKeypadMap(NINE); //ppm
		_delay_ms(DCC_SMALL_TIME);
		setKeypadMap(ENTER);
		_delay_ms(DCC_LARGE_TIME);

		break;

	case 1: //scan using defaults
		setKeypadMap(RIGHT); //PR1
		_delay_ms(DCC_SMALL_TIME);
		setKeypadMap(ENTER);
		_delay_ms(DCC_LARGE_TIME);

		setKeypadMap(table); //Table #
		_delay_ms(DCC_SMALL_TIME);
		setKeypadMap(ENTER);
		_delay_ms(DCC_LARGE_TIME);
		break;

	case 0: //restart previous scan
		// fall through

	default:
		setKeypadMap(ENTER); //restart prev. scan
		_delay_ms(DCC_LARGE_TIME);
		setKeypadMap(table); //Table #
		_delay_ms(DCC_SMALL_TIME);
		setKeypadMap(ENTER); //restart prev. scan
		_delay_ms(DCC_LARGE_TIME);
	}

}

/*----------------------------------------------------------*/
/*
 * Goes back to the main menu by pressing a few escapes
 * The cursor is reset to Manual
 */
void resetToMenu(void)
{
	for (unsigned char i=0; i<2; i++) //Exit from everything you are doing
	{
		setKeypadMap(ESC);
		_delay_ms(DCC_LARGE_TIME);
	}
	setKeypadMap(PGRM); //Resets cursor to Manual
	_delay_ms(DCC_LARGE_TIME);
}

/*----------------------------------------------------------*/
/*
 * Set a list of frequencies
 */
void setFreq(unsigned char *table)
{
	unsigned char passedNumFreq = (table[0]-3)/2;
	unsigned char validNumFreq=0;

	unsigned int currTable[MAX_LIST_SIZE];
	unsigned char currTableSize=0;

	// Check number of valid frequencies
	for (unsigned char i=0; i<2*passedNumFreq; i=i+2)
	{
		unsigned long val = table[i+2];
		val = (val << 8) + table[i+3];
		if (val > DCC_MAX_FREQ || val < DCC_MIN_FREQ)
		{
		} else {
			currTable[currTableSize] = val; //Store
			currTableSize++;
			validNumFreq++;
		}
	}

	// Now check if the current table consists of same frequency
	// only if we have a list of more than 1
	// else just record the particular frequency given
	if(currTableSize>1)
	{
		unsigned char cIdx=0,tIdx=0;
		unsigned char same=TRUE;

		// Check if sizes are equal
		if(currTableSize != freqTableSize)
		{
			same = FALSE;
		}

		// Check if currTable is in freqTable
		for(cIdx=0; (cIdx<currTableSize) && (same == TRUE);cIdx++)
		{
			unsigned char found=FALSE;
			for(tIdx=0; tIdx<freqTableSize; tIdx++)
			{
				if(currTable[cIdx]==freqTable[tIdx])
				{
					found=TRUE;
					break;
				}
			}
			if(found==FALSE)
			{
				same=FALSE;
				break;
			}
		}

		// Check if freqTable is in currTable
		for(tIdx=0;(tIdx<freqTableSize) && (same==TRUE);tIdx++)
		{
			unsigned char found=FALSE;
			for(cIdx=0; cIdx<freqTableSize; cIdx++)
			{
				if(currTable[cIdx]==freqTable[tIdx])
				{
					found=TRUE;
					break;
				}
			}
			if(found==FALSE)
			{
				same=FALSE;
				break;
			}
		}

		if(same==TRUE) //Same tables, don't write just log
		{
			startAerialLogging(0,LIST_TABLE);
			return;
		} else
		{ //update table and fall through
			for(tIdx=0; tIdx<currTableSize; tIdx++)
			{
				freqTable[tIdx] = currTable[tIdx];
			}
			freqTableSize = currTableSize;
		}
	}
	// Either lists are not same or single frequency
	// Check if same frequency as that in table

	if(validNumFreq==0) {
		return;
	}

	resetToMenu();
	setKeypadMap(FREQ);
	_delay_ms(DCC_LARGE_TIME);
	if(currTableSize>1)
	{
		setKeypadMap(LIST_TABLE);
	} else if(currTableSize==1)
	{
		if(currTable[0] < 49000) //Check the table to use
			setKeypadMap(SINGLE_TABLE_48);
		else
			setKeypadMap(SINGLE_TABLE_49);
	} else {
		//TODO error?
	}
	_delay_ms(DCC_SMALL_TIME);
	setKeypadMap(ENTER);
	_delay_ms(DCC_SMALL_TIME);

	if(validNumFreq < 10)
	{
		if(validNumFreq == 8) { //TODO hack to avoid typing 8
			setKeypadMap(7);
			_delay_ms(DCC_SMALL_TIME);
			setKeypadMap(UP);
			_delay_ms(DCC_SMALL_TIME);
		} else {
			setKeypadMap(validNumFreq); //No. of freqs.
		}
	} else {
		if (validNumFreq < 100)
		{
			unsigned char msb = 0;
			unsigned char tempNumFreq=validNumFreq;
			while(tempNumFreq >= 10)
			{
				msb++;
				tempNumFreq-=10;
			}
			if(msb == 8) { //TODO hack to avoid typing 8
				setKeypadMap(7);
				_delay_ms(DCC_SMALL_TIME);
				setKeypadMap(UP);
				_delay_ms(DCC_SMALL_TIME);
			} else {
				setKeypadMap(msb);
			}
			_delay_ms(DCC_SMALL_TIME);
			if(tempNumFreq == 8) { //TODO hack to avoid typing 8
				setKeypadMap(7);
				_delay_ms(DCC_SMALL_TIME);
				setKeypadMap(UP);
				_delay_ms(DCC_SMALL_TIME);
			} else {
				setKeypadMap(tempNumFreq);
			}
		} else {
			return; //TODO can't handle more than 99
		}
	}
	_delay_ms(DCC_SMALL_TIME);
	setKeypadMap(ENTER);
	_delay_ms(DCC_LARGE_TIME);

	// Now enter all freqs one by one
	for (unsigned char i=0; i<2*validNumFreq; i=i+2)
	{
		unsigned char str[6];
		unsigned long val = table[i+2];
		val = (val << 8) + table[i+3];

		if (val > DCC_MAX_FREQ || val < DCC_MIN_FREQ)
		{
			continue;
		}
		ltoa(val,str,10);

		//TODO hack to avoid typing 48
		//We're starting assuming cursor is on third digit
		for (unsigned char j=2; j<5; j++) //set last 3 digits
		{
			setKeypadMap(str[j]-0x30); //Send digits (ascii conversion)
			_delay_ms(DCC_SMALL_TIME);
		}
		setKeypadMap(ENTER);
		_delay_ms(DCC_LARGE_TIME);
	}
	if(currTableSize>1)
	{
		startAerialLogging(0,LIST_TABLE);
	} else
	{
		if(currTable[0] < 49000) //Check the table to use
			startAerialLogging(0,SINGLE_TABLE_48);
		else
			startAerialLogging(0,SINGLE_TABLE_49);
	}
}

/*----------------------------------------------------------*/
/*
 * Starts logging in the manual mode
 * First set the frequency
 */
void startManualLogging(unsigned long freq)
{
	unsigned char str[6];
	if (freq > DCC_MAX_FREQ || freq < DCC_MIN_FREQ)
	{
		return;
	}
	ltoa(freq,str,10);

	resetToMenu(); //Takes you to the manual button
	setKeypadMap(ENTER);
	_delay_ms(DCC_LARGE_TIME);
	setKeypadMap(RIGHT); //TODO fix 8
	_delay_ms(DCC_SMALL_TIME);
	//TODO hack to avoid typing 48
	//We're starting assuming cursor is on third digit
	for (unsigned char j=2; j<5; j++) //set last 3 digits
	{
		setKeypadMap(str[j]-0x30); //Send digits (ascii conversion)
		_delay_ms(DCC_SMALL_TIME);
	}
	setKeypadMap(ENTER);

	MANUAL_MODE = TRUE; //Let the signal packet sender know
}

/*----------------------------------------------------------*/
/*
 * Sends back the device id = 2
 */
void sendDeviceId(void)
{
	putCharUART1(START_COMM);
	putCharUART1(3);
	putCharUART1(2);
}

/*----------------------------------------------------------*/
/*
 * Resets DCC back to the main menu
 * Removes any internal stored flags, variables
 * e.g. removes last stored frequency/table in use
 */
void resetAll(void)
{
	resetToMenu();
	freqTableSize = 0;
	for(unsigned char i=0; i<MAX_LIST_SIZE; i++)
	{
		freqTable[i] = 0;
	}
}


/*----------------------------------------------------------*/
int main (void)
{
	bootUp();

	while(1)
	{
		while(LOOP_COMPLETE == FALSE)
		{
			if(PACKET_RECEIVED == TRUE)
			{
				PACKET_RECEIVED = FALSE;
				ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
				{
					for(unsigned char i=0; i<rxBuffer[0]-1; i++)
					{
						rxBufferBk[i] = rxBuffer[i];
					}
				}

				switch(rxBufferBk[1])
				{
				// Main control goes here
				case CHANGE_GAIN:
					sendAck();
					changeGain(rxBufferBk[2]);
					break;

				case SET_FREQ:
					sendAck();
					setFreq(rxBufferBk);
					break;

				case GET_DEV_ID:
					sendDeviceId();
					break;

				case LOG_AERIAL:
					sendAck();
					startAerialLogging(0,rxBufferBk[2]);
					break;

				case LOG_MANUAL:
					sendAck();
					unsigned long freq = rxBufferBk[2];
					freq = (freq << 8) + rxBufferBk[3];
					startManualLogging(freq);
					break;

				case RESET:
					sendAck();
					resetAll();
					break;

				case SET_RAW:
					sendAck();
					setKeypadMap(rxBufferBk[2]);
					break;
				}
			}

			if(SCREEN_RECEIVED == TRUE){
				SCREEN_RECEIVED = FALSE;
				toggleRedLed();
				ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
					for(unsigned char i = 0; i<SCREEN;i++){
						if(i==0){
							putCharUART1(START_COMM);
						}else if(i==1){
							putCharUART1(SCREEN);
						}else if(i==2){
							putCharUART1('S');
						}else{
							putCharUART1(rxScreenBuffer[i-3]);
						}
					}
				}
			}
		}

		if(millisec < 100)	BLUE_LED_ON();
		else	BLUE_LED_OFF();

		LOOP_COMPLETE = FALSE;
	}
}

/*----------------------------------------------------------*/
