/*
 * FinalCompetition.c
 *
 *
 * This is the code that runs at the final competition.

 */


#include <avr/io.h>


#define F_CPU 16000000UL
#include <avr/io.h>
#include "serial.h"
#include <util/delay.h>
#include <avr/interrupt.h>

int color_1=0;
int starting_color=0;
int i=0;	// for loop counter
int back_up = 0;
int leftQTI = 0;
int rightQTI = 0;

// gets period of output wave signal from color sensor

ISR(PCINT0_vect)
{

	if((PINB & 0b00010000) == 0b00010000) // check if pin 4 (PB4) is high
	{
		TCNT1 = 0b00000000; // reset the timer
		} else {
		color_1 = TCNT1; // set period to current timer value
	}
}

// detectBorder executes when external interrupt for either QTI is triggered (border is detected)
// this function gets the robot to turn accordingly to avoid the border.
// or, if the robot is supposed to drop off cubes, this gets the robot to back up upon detecting the border, allowing it to drop off the cubes there.

void detectBorder()
{
	if(back_up == 1)	{	// this is the code for dropping off blocks on own side once QTI detects the border after reaching own side
		// go backwards
		PORTD &= 0b00000000; // reset motors
		PORTB &= 0b00000000;
		PORTB |= 0b00001000; // set right motor reverse
		PORTD |= 0b00100000; // set left motor reverse
		_delay_ms(2000);	// just enough time for arms to be clear of cubes
		PORTD &= 0b00000000; // reset motors
		PORTB &= 0b00000000;
		// turn 180 degrees
		PORTD |= 0b01000000; // set left motor forward
		PORTB |= 0b00001000; // right motor reverse
		_delay_ms(2900);
		PORTD &= 0b00000000; // reset motors
		PORTB &= 0b00000000;
		back_up = 0;	// finished dropping off cubes; now detectBorder() operates as usual, ready for next round of cube collection
		// drive forward
		PORTD |= 0b01000000; // set P0 (PD5, pin 5) to 0 and P1 (PD6, pin 6) to 1 for forward drive
		PORTB |= 0b00000100; // set P0 (PB3, pin 11) to 0 and P1 (PB2, pin 10) to 1 for forward drive
	}
	else {		// regular response; if robot is NOT dropping off cubes and just wants to avoid the border
		while((PIND &= 0b00000100) == 0b00000100)	{	// while left QTI reads high
			// turn right a bit
			PORTD &= 0b00000000; // reset motors
			PORTB &= 0b00000000;
			PORTD |= 0b01000000; // set left motor forward
			PORTB |= 0b00001000; // right motor reverse
			_delay_ms(50);
			PORTD &= 0b00000000; // reset motors
			PORTB &= 0b00000000;
		}
		while((PIND &= 0b00001000) == 0b00001000)	{	// while right QTI reads high
			// turn left a bit
			PORTD &= 0b00000000; // reset motors
			PORTB &= 0b00000000;
			PORTD |= 0b00100000; // set left motor reverse
			PORTB |= 0b00000100; // right motor forward
			_delay_ms(50);
			PORTD &= 0b00000000; // reset motors
			PORTB &= 0b00000000;
		}
	}

	// drive forward
		PORTD |= 0b01000000; // set P0 (PD5, pin 5) to 0 and P1 (PD6, pin 6) to 1 for forward drive
		PORTB |= 0b00000100; // set P0 (PB3, pin 11) to 0 and P1 (PB2, pin 10) to 1 for forward drive

}

ISR(INT0_vect) { // interrupt vector for left QTI
	detectBorder();
}

ISR(INT1_vect) { // interrupt vector for right QTI
	detectBorder();
}


// initColor initializes all the pins necessary for color detection, border detection and any resulting actions
void initColor()
{
	sei(); // enable interrupts globally

	// INITIALIZE I/O PINS
	DDRB &= 0b11001111; // sets pin 12 (PB4) and pin 13 (PB5) to be input pins for color sensors 1 and 2
	DDRD &= 0b11100011; // sets pin 2 (PD2) and pin 3 (PD3) as input pins for QTI

	// INITIALIZE MOTOR PINS
	// H-Bridge 1: P0 on pin 5, P1 on pin 6
	DDRD |= 0b01100000; //set PWM pins 5 and 6 (PD5 and PD6) as outputs

	// H-Bridge 2: P0 on pin 11, P1 on pin 10
	DDRB |= 0b00001100; //set PWM pins 10 and 11 (PB2 and PB3) to be outputs

	// initialize pin change interrupt
	PCICR = 0b00000101; // initialize register B for PC interrupts

	// initialize timer 1, normal mode (WGM10, WGM11, WGM12, WGM13 to 0), prescaler to 1 (CS12, CS11 to 0, CS 10 to 1)
	TCCR1A = 0b00000000;
	TCCR1B = 0b00000001;

	// initialize external interrupts
	EICRA |= 0b00001111;	// set external interrupts at pins 2 and 3 to trigger at rising edge
	EIMSK |= 0b00000011;	// enable external interrupts 0 and 1

}

// getColor1 returns the period of timer 1 in microseconds, which is related to the input from color sensor 1

int getColor1()
{
	PCMSK0 |= 0b00010000; // enable pin 12 (PCINT4) as a PC interrupt
	_delay_ms(10); // Give the interrupt a sec
	return color_1*.0625*2; // Change to units of microseconds
	PCMSK0 &= 0b11101111;
}



// main function

int main(void)
{
	init_uart(); // initialize serial
	initColor(); // initialize everything else
	starting_color = getColor1(); // assigns the starting color period to the variable starting_color


	// ROBOT STARTS FACING FRONT


	// deploy arms by driving back and forth 3 times
	for(i=0; i<3; i++)	{	// executes 3 times
		PORTD |= 0b01000000; // set P0 (PD5, pin 5) to 0 and P1 (PD6, pin 6) to 1 for forward drive
		PORTB |= 0b00000100; // set P0 (PB3, pin 11) to 0 and P1 (PB2, pin 10) to 1 for forward drive
		_delay_ms(300);
		PORTD &= 0b00000000; // reset motors
		PORTB &= 0b00000000;
		PORTD |= 0b00100000; // set left motor reverse
		PORTB |= 0b00001000; // right motor reverse
		_delay_ms(300);
		PORTD &= 0b00000000; // reset motors
		PORTB &= 0b00000000;
	}

	// drive forward a bit, then turn left 90 degrees
	// (this is to make sure the QTIs on the arms will not be off the border after deploying)

	PORTD |= 0b01000000; // set P0 (PD5, pin 5) to 0 and P1 (PD6, pin 6) to 1 for forward drive
	PORTB |= 0b00000100; // set P0 (PB3, pin 11) to 0 and P1 (PB2, pin 10) to 1 for forward drive
	_delay_ms(900);
	PORTD &= 0b00000000; // reset motors
	PORTB &= 0b00000000;
	// turn left 90 degrees
	PORTD |= 0b00100000; // set left motor reverse
	PORTB |= 0b00000100; // right motor forward
	_delay_ms(600);
	PORTD &= 0b00000000; // reset motors
	PORTB &= 0b00000000;


	// drive forward (if not detecting border), or follow border on left hand side (if detected) until other color is detected
	while ((getColor1() >= (starting_color-100)) && (getColor1() <= (starting_color+100)))	{	// while color is same as starting color
		// drive forward
		PORTD |= 0b01000000; // set P0 (PD5, pin 5) to 0 and P1 (PD6, pin 6) to 1 for forward drive
		PORTB |= 0b00000100; // set P0 (PB3, pin 11) to 0 and P1 (PB2, pin 10) to 1 for forward drive
	}

	// we are out of the while loop, which means that the other color has now been detected
	EIMSK = 0b00000011;	// enable external interrupts 0 and 1

	// GO ACROSS BOARD TO GET CUBES

	PORTD &= 0b00000000; // reset motors
	PORTB &= 0b00000000;

	// drive forward a bit
	PORTD |= 0b01000000; // set P0 (PD5, pin 5) to 0 and P1 (PD6, pin 6) to 1 for forward drive
	PORTB |= 0b00000100; // set P0 (PB3, pin 11) to 0 and P1 (PB2, pin 10) to 1 for forward drive
	_delay_ms(200);

	// turn 90 degrees right
	PORTD &= 0b00000000; // reset motors
	PORTB &= 0b00000000;
	PORTD |= 0b01000000; // set left motor forward
	PORTB |= 0b00001000; // right motor reverse
	_delay_ms(800);	// modify this time until it gives a clean 90 degree turn
	PORTD &= 0b00000000; // reset motors
	PORTB &= 0b00000000;

	// drive forward across
	PORTD |= 0b01000000; // set P0 (PD5, pin 5) to 0 and P1 (PD6, pin 6) to 1 for forward drive
	PORTB |= 0b00000100; // set P0 (PB3, pin 11) to 0 and P1 (PB2, pin 10) to 1 for forward drive
	_delay_ms(4000);


	// begin return to own side
	// turn a little more than 90 degrees right
	PORTD &= 0b00000000; // reset motors
	PORTB &= 0b00000000;
	PORTD |= 0b01000000; // set left motor forward
	PORTB |= 0b00001000; // right motor reverse
	_delay_ms(1000);


	// start driving back to its own side. drive straight (utilizing QTI interrupts if necessary) until reaching own side again
	PORTD &= 0b00000000; // reset motors
	PORTB &= 0b00000000;
	PORTD |= 0b01000000; // set P0 (PD5, pin 5) to 0 and P1 (PD6, pin 6) to 1 for forward drive
	PORTB |= 0b00000100; // set P0 (PB3, pin 11) to 0 and P1 (PB2, pin 10) to 1 for forward drive


	while(!((getColor1() >= (starting_color-100)) && (getColor1() <= (starting_color+100)))) { // if not detecting own color
		// continue driving straight (QTI interrupts will trigger as necessary)
	}

	// exited while loop. it is now back on its own color!

	// keep going straight until a QTI is triggered (until it reaches a border on its own side)
		back_up = 1;		// if back_up=1, it will get the robot to back up the next time the QTI triggers
		while(back_up == 1) { 	// blocks have not yet been finished dropping off
			while(!((getColor1() >= (starting_color-100)) && (getColor1() <= (starting_color+100)))) { // while not on own color
				back_up = 0; // don't back up to drop off cubes; if QTI triggers, execute avoiding border, not dropoff
				// drive forward
				PORTD |= 0b01000000; // set P0 (PD5, pin 5) to 0 and P1 (PD6, pin 6) to 1 for forward drive
				PORTB |= 0b00000100; // set P0 (PB3, pin 11) to 0 and P1 (PB2, pin 10) to 1 for forward drive
			}
			// now it is on its own color
			back_up = 1;	// if QTI triggers, execute dropoff
			// drive forward
			PORTD |= 0b01000000; // set P0 (PD5, pin 5) to 0 and P1 (PD6, pin 6) to 1 for forward drive
			PORTB |= 0b00000100; // set P0 (PB3, pin 11) to 0 and P1 (PB2, pin 10) to 1 for forward drive
		}

		// back_up = 0 now, which means that the blocks have finished being dropped off; robot has backed up and turned around 180 degrees



	// FIRST TRIP TO COLLECT BLOCKS COMPLETE! now, we will continue to scavenge for blocks. they can be anywhere on the other side.
	// basically, randomly drive around on the other side's color and hope we can grab some blocks. bring the blocks over to our own side. repeat forever.

	while(1)	{
		// drive forward (with QTI interrupts as necessary) until it reaches the other side
		while ((getColor1() >= (starting_color-100)) && (getColor1() <= (starting_color+100)))	{ // while color is same as starting color
			// drive forward
			PORTD |= 0b01000000; // set P0 (PD5, pin 5) to 0 and P1 (PD6, pin 6) to 1 for forward drive
			PORTB |= 0b00000100; // set P0 (PB3, pin 11) to 0 and P1 (PB2, pin 10) to 1 for forward drive
		}

		// we have reached the other side! keep driving forward and hopefully get some blocks, until we reach our own side again

		while(!((getColor1() >= (starting_color-100)) && (getColor1() <= (starting_color+100)))) { // if not detecting own color
			// continue driving straight (QTI interrupts will trigger as necessary)
			PORTD |= 0b01000000; // set P0 (PD5, pin 5) to 0 and P1 (PD6, pin 6) to 1 for forward drive
			PORTB |= 0b00000100; // set P0 (PB3, pin 11) to 0 and P1 (PB2, pin 10) to 1 for forward drive
		}

		// we have returned to our own side!
		// keep going straight until a QTI is triggered (until it reaches a border on its own side)

		back_up = 1;		// get robot to back up the next time the QTI triggers
		while(back_up == 1) { 	// blocks have not yet been finished dropping off
			while(!((getColor1() >= (starting_color-100)) && (getColor1() <= (starting_color+100)))) { // while not on own color
				back_up = 0; // don't back up to drop off cubes; if QTI triggers, execute avoiding border, not dropoff
				// drive forward
				PORTD |= 0b01000000; // set P0 (PD5, pin 5) to 0 and P1 (PD6, pin 6) to 1 for forward drive
				PORTB |= 0b00000100; // set P0 (PB3, pin 11) to 0 and P1 (PB2, pin 10) to 1 for forward drive
			}

			// it is on own color
			back_up = 1;	// if QTI triggers, execute dropoff
			// drive forward
			PORTD |= 0b01000000; // set P0 (PD5, pin 5) to 0 and P1 (PD6, pin 6) to 1 for forward drive
			PORTB |= 0b00000100; // set P0 (PB3, pin 11) to 0 and P1 (PB2, pin 10) to 1 for forward drive

		}

		// blocks finished dropping off; robot has backed up and turned around 180 degrees

	}

}
