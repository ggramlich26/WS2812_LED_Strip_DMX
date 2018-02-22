/*
 * main.c
 *
 *  Created on: Nov 9, 2017
 *      Author: tsugua
 */

#define BAUD 		250000	//DMX Baud rate

#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/setbaud.h>
#include <compat/deprecated.h>
#include <stdlib.h>

#include "dmx_decoder.h"
#include "effect_generator.h"

#define	DMX_MODE	ws2812


//#define ON_DMX_BOARD


#define	RLEDDDR		DDRD
#define	RLEDPORT	PORTD
#define	RLED		PD5

#define GLEDDDR		DDRD
#define	GLEDPORT	PORTD
#define	GLED		PD6

#define BLEDDDR		DDRB
#define	BLEDPORT	PORTB
#define BLED  		PB3

#define SW10DDR		DDRB
#define	SW10PORT	PORTB
#define	SW10PIN		PINB
#define	SW10		PB0

#define RXTXSELDDR	DDRB
#define	RXTXSELPORT	PORTB
#define	RXTXSEL		PB1

#ifdef ON_DMX_BOARD

#define	WSDDR		DDRD
#define	WSPORT		PORTD
#define WSDPIN  	PD7
//necessary for Assembler routine. Update if changing WS2812 Pin!
#define ws2812_port 0x0B  // Number of the data port register
#define ws2812_pin 	PD7    // Number of the data out pin

#else
#define LEDDDR		DDRB
#define	LEDPORT		PORTB
#define BOARDLED    PB5
#define	WSDDR		DDRD
#define	WSPORT		PORTD
#define WSDPIN  	PD7
//necessary for Assembler routine. Update if changing WS2812 Pin!
#define ws2812_port 0x0B  // Number of the data port register
#define ws2812_pin 	WSDPIN    // Number of the data out pin
#endif

void serialInit();
uint16_t getDmxAddress();
uint8_t getStripMode();
void timerInit();
void set_red_led(uint8_t);
void set_green_led(uint8_t);
void set_blue_led(uint8_t);
//void ws2812_sendarray_8Mhz(uint8_t *data,uint16_t datlen);
void ws2812_sendarray_16Mhz(uint8_t *data,uint16_t n);

//DMX Definitions
#define NDMXVALUES  9      //Number of DMX Values to be recieved starting from the startaddress
volatile uint8_t  DmxRxField[NDMXVALUES]; //array of DMX vals (raw)
volatile uint16_t DmxAddress;    //start address
volatile uint8_t scheduler_update_led;
volatile uint8_t scheduler_reception_complete;
volatile uint8_t ws2812_transmission_corrupted;

enum {IDLE, BREAK, STARTB, STARTADR}; //DMX states

volatile uint8_t gDmxState;

void setup(){
	RLEDDDR |= (1<<RLED);
	RLEDPORT &= ~(1<<RLED);

	GLEDDDR |= (1<<GLED);
	GLEDPORT &= ~(1<<GLED);

	BLEDDDR |= (1<<BLED);
	BLEDPORT &= ~(1<<BLED);

	WSDDR |= (1<<WSDPIN);
	WSPORT &= ~(1<<WSDPIN);

	RXTXSELDDR |= (1<<RXTXSEL);
	RXTXSELPORT &= ~(1<<RXTXSEL);

#ifdef ON_DMX_BOARD
	//Set Dip Switch data direction and activate pullup
	DDRC &= 0b11000000;
	DDRD &= 0b11100011;
	PORTC |= 0b00111111;
	PORTD |= 0b00011100;
	SW10DDR &= ~(1<<SW10);
	SW10PORT |= (1<<SW10);
#else
	LEDDDR |= (1<<BOARDLED);
	LEDPORT &= ~(1<<BOARDLED);
#endif

	//DMX Initialization
	gDmxState= IDLE; // initial state
	DmxAddress = getDmxAddress(); // The desired DMX Start Address
	scheduler_reception_complete = 0;
	scheduler_update_led = 1;
	serialInit();

	//Timer initialization
	timerInit();

	efg_init();
	ws2812_transmission_corrupted = 0;
	sei();
}

void loop(){
	if(scheduler_reception_complete | scheduler_update_led){
		if(scheduler_reception_complete){
			scheduler_reception_complete = 0;
			dmx_decode((uint8_t*)DmxRxField, getStripMode());
		}
		if(getStripMode() == ws2812){
			efg_update();
			ws2812_transmission_corrupted = 0;
			cli();
			ws2812_sendarray_16Mhz(efg_get_data(), NUMBER_LEDS*3);
			if(!ws2812_transmission_corrupted){
				scheduler_update_led = 0;
			}
			//reset DMX Controller so that it will start listening at the next transmission again
			gDmxState = IDLE;
			//clear interrupt flag by reading UDR0
			uint8_t tmp = UCSR0A;
			tmp = UDR0;
			tmp = UCSR0A;
			tmp = UDR0;
			sei();
		}
		else{
			//write to pwm
			set_red_led(efg_get_red());
			set_green_led(efg_get_green());
			set_blue_led(efg_get_blue());
			scheduler_update_led = 0;
		}
	}
}

uint16_t getDmxAddress(){
#ifdef ON_DMX_BOARD
	uint16_t adr = (~PINC) & (0b00111111);
	adr |= (uint16_t)((~PIND) & 0b00011100)<<4;
	return adr;
#else
	return 1;
#endif
}

uint8_t getStripMode(){
	return DMX_MODE;
}

//#ifdef USART_RXC_vect
//ISR(USART_RXC_vect) {
//#elif defined USART_RX_vect
//	ISR(USART_RX_vect) {
//#endif
//		static  uint16_t DmxCount;
//#ifdef UCSRA
//		uint8_t  USARTstate= UCSRA;    //get state before data!
//		uint8_t  DmxByte   = UDR;          //get data
//#elif defined UCSR0A
//		uint8_t  USARTstate= UCSR0A;    //get state before data!
//		uint8_t  DmxByte   = UDR0;          //get data
//#endif
//		uint8_t  DmxState  = gDmxState; //just load once from SRAM to increase speed
//
//#ifdef FE
//		if (USARTstate &(1<<FE))               //check for break
//#elif defined FE0
//			if (USARTstate &(1<<FE0))               //check for break
//#endif
//			{
//				gDmxState= BREAK;
//			}
//
//			else if (DmxState == BREAK)
//			{
//				if (DmxByte == 0) {
//					DmxAddress = getDmxAddress();    //update DMX Address
//					DmxCount =  DmxAddress;         //reset channel counter (count channels before start address)
//					gDmxState= STARTB;  //normal start code detected
//				}
//				else{
//					gDmxState= IDLE;
//				}
//			}
//
//			else if (DmxState == STARTB)
//			{
//				if (--DmxCount == 0)    //start address reached?
//				{
//					DmxCount= 1;            //set up counter for required channels
//					DmxRxField[0]= DmxByte; //get 1st DMX channel of device
//					gDmxState= STARTADR;
//				}
//			}
//
//			else if (DmxState == STARTADR)
//			{
//				DmxRxField[DmxCount++]= DmxByte;        //get channel
//				if (DmxCount >= sizeof(DmxRxField)) //all ch received?
//				{
//					gDmxState= IDLE;        //wait for next break
//					scheduler_update_led = 1;
//				}
//			}
//	}

ISR(USART_RX_vect) {
	static  uint16_t DmxCount;
	uint8_t  USARTstate= UCSR0A;    //get state before data!
	uint8_t  DmxByte   = UDR0;          //get data
	uint8_t  DmxState  = gDmxState; //just load once from SRAM to increase speed
	ws2812_transmission_corrupted = 1;

		if (USARTstate &(1<<FE0))               //check for break
		{
			gDmxState= BREAK;
		}

		else if (DmxState == BREAK)
		{
			if (DmxByte == 0) {
				DmxAddress = getDmxAddress();    //update DMX Address
				DmxCount =  DmxAddress;         //reset channel counter (count channels before start address)
				gDmxState= STARTB;  //normal start code detected
			}
			else{
				gDmxState= IDLE;
			}
		}

		else if (DmxState == STARTB)
		{
			if (--DmxCount == 0)    //start address reached?
			{
				DmxCount= 1;            //set up counter for required channels
				DmxRxField[0]= DmxByte; //get 1st DMX channel of device
				gDmxState= STARTADR;
			}
		}

		else if (DmxState == STARTADR)
		{
			DmxRxField[DmxCount++]= DmxByte;        //get channel
			if (DmxCount >= sizeof(DmxRxField)) //all ch received?
			{
				gDmxState= IDLE;        //wait for next break
				scheduler_reception_complete = 1;
			}
		}
}

int main(){
	setup();
	while(1){
		loop();
	}
	return 0;
}

void timerInit(){
//	PRTIM0 &= ~(1<<TC0);
	//Timer 0 as PWM
	TCCR0A = (1<<WGM00)|(1<<WGM01)|(1<<COM0A1)|(1<<COM0A0)|(1<<COM0B1)|(1<<COM0B0);	//fast pwm mode inverted
	TCCR0B = (1<<CS02); //prescaler 64
	OCR0A = 0xFF;
	OCR0B = 0xFF;

	//Timer 1 as clock
	TCCR1B = (1<<WGM12)|(1<<CS11) | (1<<CS10);	//prescaler 64, clear timer on OCR1A compare match
	TIMSK1 = (1<<OCIE1A);	//enable interrupt for compare match
	//Compare at 8750 = 0x222E for compare match each 35ms
	//1250 are equivalent to 5ms
	OCR1A = 0x222E;

	//Timer2 as PWM
	TCCR2A = (1<<WGM20)|(1<<WGM21)|(1<<COM2A1)|(1<<COM2A0);	//fast pwm mode inverted
	TCCR2B = (1<<CS22); //prescaler 64
	OCR2A = 0xFF;
}

//the interrupt routine executed for timer 1 every 35ms
ISR(TIMER1_COMPA_vect) {
	ws2812_transmission_corrupted = 1;
	efg_inc_time_diff(35);
//	static uint8_t count = 6;	//refresh LEDs each 35ms
//	count--;
//	if(count == 0){
//		count = 6;
		scheduler_update_led = 1;
//	}
}

void set_green_led(uint8_t val){
	OCR0A = 255-val;
}

void set_red_led(uint8_t val){
	OCR0B = 255-val;
}

void set_blue_led(uint8_t val){
	OCR2A = 255-val;
}

void serialInit(){
#ifdef UBRRH
	UBRRH = UBRRH_VALUE;
	UBRRL = UBRRL_VALUE;
#elif defined UBRR0H
	UBRR0H = UBRRH_VALUE;
	UBRR0L = UBRRL_VALUE;
#endif

#ifdef UCSRA
#if USE_2X
	UCSRA |= (1 << U2X);
#else
	UCSRA &= ~(1 << U2X);
#endif
#elif defined UCSR0A
#if USE_2X
	UCSR0A |= (1 << U2X0);
#else
	UCSR0A &= ~(1 << U2X0);
#endif
#endif

#ifdef UCSRB
	sbi(UCSRB, RXEN);	//enable USART Reception
	cbi(UCSRB, TXEN);	//disable USART Transmission
	sbi(UCSRB, RXCIE);	//enable USART receive complete interrupts
	cbi(UCSRB, TXCIE);	//disable USART transmit complete interrupts
#elif defined UCSR0B
	sbi(UCSR0B, RXEN0);		//enable USART Reception
	cbi(UCSR0B, TXEN0);		//disable USART Transmission
	sbi(UCSR0B, RXCIE0);	//enable USART receive complete interrupts
	cbi(UCSR0B, TXCIE0);	//disable USART transmit complete interrupts
#endif

}

//Pro LED 3 Byte daten: (G8:R8:B8)
//void ws2812_sendarray_8Mhz(uint8_t *data,uint16_t datlen)
//{
//  uint8_t curbyte,ctr;
//
//  while (datlen--) {
//    curbyte=*data++;
//
//    asm volatile(
//		"    ldi  %0,8       \n\t"    // 0
//
//		"loop%=:lsl  %1      \n\t"    // 1     Datenbit ins Carry
//		"    dec  %0         \n\t"    // 2     Schleifenzähler in Z-Flag
//		"    sbi  %2,  %3    \n\t"    // 4
//
//		"    brcs .+2        \n\t"    // 5l / 6h
//		"    cbi  %2,  %3    \n\t"    // 7l / -    Überspringen bei C=1
//									  // 7l / 6h
//		"    brcc .+2        \n\t"    // 9l / 7h
//		"    cbi  %2,  %3    \n\t"    // -  / 9h   Überspringen bei C=0
//		"    brne loop%=     \n\t"    // 11 loop /10nt  Weiter, wenn Z=1
//		:  "=&d" (ctr)
//		:  "r" (curbyte), "I" (ws2812_port), "I" (ws2812_pin)
//		);
//
//    // loop overhead including byte load is 6+1 cycles
//  }
//
//}

//Pro LED 3 Byte daten: (G8:R8:B8)
void ws2812_sendarray_16Mhz(uint8_t *data,uint16_t n)
{
  uint8_t curbyte,ctr;
  uint8_t sreg_on = SREG;
//	  cli();

  while (n--) {
    curbyte=*data++;
    sreg_on = SREG;
    if(ws2812_transmission_corrupted){
    	return;
    }
    cli();

    asm volatile(
        "    ldi  %0,8       \n\t"    // 0
        "    nop\n\t"

        "loop%=:lsl  %1      \n\t"    //  1     Datenbit ins Carry
//    	"	cli\n\t"	//added
        "    dec  %0         \n\t"    //  2     Schleifenzähler in Z-Flag
        "    sbi  %2,  %3    \n\t"    //  4

        "    nop\n\t"                  //  5
        "    nop\n\t"                  //  6

        "    brcs .+2        \n\t"    //  7l / 8h
        "    cbi  %2,  %3    \n\t"    //  9l / -    Überspringen bei C=1

        "    nop\n\t"                  // 10l /  9h
        "    nop\n\t"                  // 11l / 10h
        "    nop\n\t"                  // 12l / 11h
        "    nop\n\t"                  // 13l / 12h

        "    brcc .+2        \n\t"    // 15l / 13h
        "    cbi  %2,  %3    \n\t"    // -  / 15h   Überspringen bei C=0

//    	"	sei\n\t"	//added
        "    nop\n\t"                  // 16
        "    nop\n\t"                  // 17
        "    nop\n\t"                  // 18
        "    nop\n\t"                  // 19

        "    brne loop%=     \n\t"    // 11 loop /10nt  Weiter, wenn Z=1

    :  "=&d" (ctr)
    :  "r" (curbyte), "I" (ws2812_port), "I" (ws2812_pin)
    );
    SREG=sreg_on;

    // loop overhead including byte load is 6+1 cycles
  }
}

//void ws2812_sendarray_16Mhz(uint8_t *data,uint16_t n)
//{
//  uint8_t curbyte,ctr;
//
//  while (n--) {
//    curbyte=*data++;
//
//    asm volatile(
//        "    ldi  %0,8       \n\t"    // 0
//        "    nop\n\t"
//
//        "loop%=:lsl  %1      \n\t"    //  1     Datenbit ins Carry
//        "    dec  %0         \n\t"    //  2     Schleifenzähler in Z-Flag
//        "    sbi  %2,  %3    \n\t"    //  4
//
//        "    nop\n\t"                  //  5
//        "    nop\n\t"                  //  6
//
//        "    brcs .+2        \n\t"    //  7l / 8h
//        "    cbi  %2,  %3    \n\t"    //  9l / -    Überspringen bei C=1
//
//        "    nop\n\t"                  // 10l /  9h
//        "    nop\n\t"                  // 11l / 10h
//        "    nop\n\t"                  // 12l / 11h
//        "    nop\n\t"                  // 13l / 12h
//
//        "    brcc .+2        \n\t"    // 15l / 13h
//        "    cbi  %2,  %3    \n\t"    // -  / 15h   Überspringen bei C=0
//
//        "    nop\n\t"                  // 16
//        "    nop\n\t"                  // 17
//        "    nop\n\t"                  // 18
//        "    nop\n\t"                  // 19
//
//        "    brne loop%=     \n\t"    // 11 loop /10nt  Weiter, wenn Z=1
//
//    :  "=&d" (ctr)
//    :  "r" (curbyte), "I" (ws2812_port), "I" (ws2812_pin)
//    );
//
//    // loop overhead including byte load is 6+1 cycles
//  }
//}
