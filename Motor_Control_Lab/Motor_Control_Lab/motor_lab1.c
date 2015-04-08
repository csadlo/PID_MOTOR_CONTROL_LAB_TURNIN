/* Motor_Control_Lab - an application for the Pololu Orangutan SVP
 *
 * This application uses the Pololu AVR C/C++ Library.  For help, see:
 * -User's guide: http://www.pololu.com/docs/0J20
 * -Command reference: http://www.pololu.com/docs/0J18
 *
 * Created: 3/5/2015 11:10:22 AM
 *  Author: csadlo
 */

#define ECHO2LCD

#include <pololu/orangutan.h>

#include "menu.h"

//Gives us uintX_t (e.g. uint32_t - unsigned 32 bit int)
//On the ATMega128 int is actually 16 bits, so it is better to use
//  the int32_t or int16_t so you know exactly what is going on
#include <inttypes.h> //gives us uintX_t

// useful stuff from libc
#include <avr/io.h>
#include <avr/interrupt.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <pololu/OrangutanAnalog/OrangutanAnalog.h>

// define the data direction registers
#define DD_REG_GREEN    DDRD

// define the output ports by which you send signals to the LEDs
#define PORT_GREEN    PORTD

// define the bit-masks for each port that the LEDs are attached to
#define BIT_GREEN    ( 1 << 5)

// define "function" calls for turning LEDs on and off
#define LED_ON(x)     (PORT_##x |= BIT_##x)
#define LED_OFF(x)    (PORT_##x &= ~(BIT_##x))
#define LED_TOGGLE(x) (PORT_##x ^= BIT_##x)

#define FOR_COUNT_10MS 5687
volatile uint32_t __ii;
#define WAIT_10MS {for (__ii=0;__ii<FOR_COUNT_10MS; __ii++);}
#define G_TIMER_RESOLUTION 100
#define ENCODER_DRIVEAXLE_RATIO 2249	// 2256 = 47 * 48

enum DIR {Forward='F', Reverse='R', Stop='S'};
	
enum STATUS {WAIT=0, GO=1, RUN=2, DONE=3};
	
#define USE_PID

void process_Trajectory();

/*
#define DD_MOTOR1 DDRA
#define DD_MOTOR2 DDRA

#define MOTOR1_CHANNEL_A  (1 << 0)
#define MOTOR1_CHANNEL_B  (1 << 1)
#define MOTOR2_CHANNEL_A  (1 << 2)
#define MOTOR2_CHANNEL_B  (1 << 3)
*/

#define REVERSE_MOTOR_TOGGLE (PORTC ^= (1<<6))
#define MOTOR_FORWARD (PORTC &= (~0x40))
#define MOTOR_BACKWARD (PORTC |= 0x40)
#define G_green_period 50	//50 milliseconds
#define seconds_to_log 10

/*
2249 bits per rotation
10 rotations per minute
*/

// GLOBALS
volatile uint32_t G_green_toggles;

int command_list[] = {WAIT,WAIT,WAIT,WAIT,WAIT,WAIT};

bool TARGET_ROT = false;
bool TARGET_RPM = false;

volatile double goal = 0.0;
double dt = 0.0;

volatile int16_t voltage = 0;
int16_t pre_voltage = 0;

volatile double rotations = 0.0;
volatile double RPM;
volatile double error;
volatile double pre_error;
volatile double integral;
volatile double derivative;

//From Pololu
char global_error_m1;
bool global_last_m1a_val;
bool global_last_m1b_val;
int32_t  global_counts_m1;
int32_t last_global_counts_m1 = 0;

volatile bool updateLCD = false;
volatile bool ready_to_dump_log = false;
volatile int maneuver_complete = WAIT;
volatile bool running_trajectory = false;

bool log_values_now = false;
bool display_values_flag = false;
bool USE_DEGREES = false;


volatile uint16_t curr_timestep = 0;
uint16_t log_start_timestep = 0;
uint16_t log_stop_timestep = 0;
int MAX_TIMESTEP = seconds_to_log * (1000 / G_green_period);

static int equilibrium = 0;

int  length;
char tempBuffer[80];



struct datapoint_t
{
	int16_t voltage;
	float rotations;
	float RPM;
	float error;
	float integral;
	float derivative;
};


/* FOR voltage += PID
volatile double Kp = 1.2;
//volatile double Ki = 15.0;
//volatile double Kd = 0.0005;
volatile double Ki = 2.4;
volatile double Kd = 0.1;
*/

volatile double Kp;
volatile double Ki;
volatile double Kd;
double error_threshold = 0.0;

/* position high gains */
/*
double P_Kp = 400.0;
double P_Ki = 200.0;
double P_Kd = 0.3;
*/

/* position slow gains */
/*
double P_Kp = 200.0;
double P_Ki = 200.0;
double P_Kd = 0.3;
*/

/* position very slow gains */
/*
double P_Kp = 100.0;
double P_Ki = 200.0;
double P_Kd = 0.3;
*/

/* position slow gains */

double P_Kp = 40.0;
double P_Ki = 00.0;
double P_Kd = 0.03;


void loadPositionGains()
{
	Kp = P_Kp;
	Ki = P_Ki;
	Kd = P_Kd;
	error_threshold = 0.004;	//0.003 * 2249 == 6.747 encoder counts == ~1 degree precision
}

/* speed high gains */
double S_Kp = 0.8;
double S_Ki = 10.0;
double S_Kd = 0.0008;

void loadSpeedGains()
{
	Kp = S_Kp;
	Ki = S_Ki;
	Kd = S_Kd;
	error_threshold = 0.6;
	USE_DEGREES = false;
}

void resetValues()
{
	maneuver_complete = WAIT;
	error = 0.0;
	pre_error = 0.0;		//reset the integral component's history if moving to a new position
	if (TARGET_ROT)
		integral = 0.0;			//reset the integral component's history if moving to a new position
	if (TARGET_RPM)
		voltage = 0.0;
	derivative = 0.0;
}

void init_leds()
{
	DD_REG_GREEN |= BIT_GREEN;
	
	LED_ON(GREEN);
	
	// leave on for 2 seconds
	for (int i=0;i<200;i++)
	WAIT_10MS;
		
	LED_OFF(GREEN);
}


void init_timers() 
{
	//--------------------------- GREEN ----------------------------------//
	// Set-up of interrupt for toggling green LED.
	// This "task" is implemented in hardware, because the OC1A pin will be toggled when
	// a COMPARE MATCH is made of
	//      Timer/Counter 1 to OCR1A.
	// We will keep track of the number of matches (thus toggles) inside the ISR (in the LEDs.c file)
	// Limits are being placed on the frequency because the frequency of the clock
	// used to toggle the LED is limited.

	// Using CTC mode with OCR1A for TOP. This is mode XX, thus WGM3/3210 = .
	TCCR1A = 0x82;  //table 15-9
	//TCCR1A =   10 00 | 00 10
	//TCCR1A =   clear_OC1A ignore | notused WGM1_WGM0
	
	TCCR1B = 0x15; //table 15-9
	//TCCR1B =   00 0 1 |  0 101
	//TCCR1B =   ignore notused WGM3 | WGM2 prescaler

	// Toggle OC1A on a compare match. Thus COM1A_10 = 10

	// Using pre-scaler 1024. This is CS1/2/1/0 = XXX
	
	TIMSK1 = 0x02;
	
	//20000000 / ( 1024 ) = 19531 times per second that TNCT1A is incremented
	
	// Interrupt Frequency: ? = f_IO / (1024*OCR1A)
	// Set OCR1A appropriately for TOP to generate desired frequency.
	// NOTE: This IS the toggle frequency.
	//printf("green period %d\n",G_green_period);
	OCR1A = 19531*((float)G_green_period/2000);
	ICR1  = 39062*((float)G_green_period/2000);
	//printf("Set OCR1A to %d\n",OCR1A);

	PCICR = (1<<PCIE3) | (0<<PCIE2 ) | (0<<PCIE1 ) | (0<<PCIE0 );
	PCMSK3 =  (0<<PCINT31) | (0<<PCINT30) | (0<<PCINT29) | (0<<PCINT28) | (1<<PCINT27) | (1<<PCINT26) | (1<<PCINT25) | (1<<PCINT24);

}

void init_motors()
{
	DDRD |= 0xC0;
	DDRC |= 0xC0;
	TCCR2A = 0xA3;
	TCCR2B = 0x02;
	OCR2B = 0;
}

void set_motors_custom(int16_t voltage_m1, int16_t voltage_m2)		//can't pass negative numbers to OCR2A
{
	int temp = abs(voltage_m1);
	if (temp >= 255)			OCR2B = 255;
	else if( temp <= 0)			OCR2B = 0;
	else						OCR2B = temp;
	
}

struct datapoint_t LOG[seconds_to_log*(1000/G_green_period)];

int main(void) {
	
	delay_ms(100);
	
	init_leds();
	init_timers();
	init_motors();

	// Used to print to serial comm window
	char tempBuffer[80];
	
	// Initialization here.
	lcd_init_printf();	// required if we want to use printf() for LCD printing
	init_menu();	// this is initialization of serial comm through USB
	
	clear();	// clear the LCD

	//enable interrupts
	sei();
	
	while (1) {

		process_Trajectory();

		if (updateLCD)
		{
			//first line
			lcd_goto_xy(8,0);
			print("        ");
			
			lcd_goto_xy(8,0);
			//print_long(global_counts_m1);
			sprintf( tempBuffer, "V=%d\r", voltage);
			print(tempBuffer);

			//next line
			lcd_goto_xy(0,1);
			sprintf( tempBuffer, "R=%3.2f RPM=%3.2f\r", rotations, RPM);
			print(tempBuffer);
			
			updateLCD = false;
		}
		if (display_values_flag)
		{
			if (TARGET_ROT)
				length = sprintf( tempBuffer, "Kp:%f Ki:%f Kd:%f Pr:%f Pm:%f T:%d\r\n", Kp, Ki, Kd, goal, rotations, voltage);
			else if (TARGET_RPM)
				length = sprintf( tempBuffer, "Kp:%f Ki:%f Kd:%f Pr:%f Pm:%f T:%d\r\n", Kp, Ki, Kd, goal, RPM, voltage);
			else 
				length = sprintf( tempBuffer, "Kp:%f Ki:%f Kd:%f Pr:%f Pm:NA T:%d\r\n", Kp, Ki, Kd, goal, voltage);
			
			print_usb( tempBuffer, length );
			
			length = sprintf( tempBuffer, "C0:%d C1:%d C2:%d C3:%d C4:%d C5:%d Man:%d Eq:%d\r\n",	command_list[0], 
																						command_list[1],
																						command_list[2],
																						command_list[3],
																						command_list[4],
																						command_list[5],
																						maneuver_complete,
																						equilibrium);
			print_usb( tempBuffer, length );
							
			length = sprintf( tempBuffer, "%f, %f, %f\r\n",	error, integral, derivative);
			print_usb( tempBuffer, length );
			
			display_values_flag = false;
		}
		if (ready_to_dump_log)
		{
			ready_to_dump_log = false;
			
			length = sprintf( tempBuffer, "Goal, Kp, Ki, Kd, Rot, RPM, Volt\r\n");
			print_usb( tempBuffer, length );
			length = sprintf( tempBuffer, "%f, %f, %f, %f, %f, %f, %d\r\n", goal, Kp, Ki, Kd, rotations, RPM, voltage);
			print_usb( tempBuffer, length );
			
			int delta = (log_stop_timestep-log_start_timestep);
			
			length = sprintf( tempBuffer, "Time, Error, KpError, KiIntegral, KdDerivative, Rotations, RPM, Voltage\r\n");
			print_usb( tempBuffer, length );
			
			float temp_Kp = (Kp == 0.0) ? 1.0 : Kp;
			float temp_Ki = (Ki == 0.0) ? 1.0 : Ki;
			float temp_Kd = (Kd == 0.0) ? 1.0 : Kd;
			
			float time;
			
			for(int i=0; i<delta; i++)
			{
				time = (float)((i*dt)*60);
				length = sprintf( tempBuffer, "%f %f %f %f %f %f %f %d\r\n", time,	//time expressed in seconds
																			LOG[i].error,
																			(float)(temp_Kp * LOG[i].error),
																			(float)(temp_Ki * LOG[i].integral),
																			(float)(temp_Kd * LOG[i].derivative),
																			LOG[i].rotations,
																			LOG[i].RPM,
																			LOG[i].voltage);
				print_usb( tempBuffer, length );	
			}
		}

		//Process Control from the USB Port
		serial_check();
		check_for_new_bytes_received();
					
	} //end while loop
} //end main

/*
MM		MM
11		11
AB		AB
_____________
00		00
10		01
11		11
01		10
00		00
*/

//INTERRUPT HANDLER for the MOTOR
ISR(PCINT3_vect) {
	
	bool m1a_val = ((PIND & (1<<PIND3)) > 0);
	bool m1b_val = ((PIND & (1<<PIND2)) > 0);
	
	bool plus_m1 = m1a_val ^ global_last_m1b_val;
	bool minus_m1 = m1b_val ^ global_last_m1a_val;

	if(plus_m1)
		global_counts_m1 += 1;
	if(minus_m1)
		global_counts_m1 -= 1;

	if(m1a_val != global_last_m1a_val && m1b_val != global_last_m1b_val)
		global_error_m1 = 1;

	global_last_m1a_val = m1a_val;
	global_last_m1b_val = m1b_val;
}

double curr_RPM = 0.0;
double prev_delta_count1 = 0.0;
double prev_delta_count2 = 0.0;
double prev_delta_count3 = 0.0;

bool just_started_maneuver = false;


//INTERRUPT HANDLER for green LED
ISR(TIMER1_COMPA_vect) {

	// This the Interrupt Service Routine for tracking green toggles. The toggling is done in hardware.
	// Each time the TCNT count is equal to the OCRxx register, this interrupt is enabled.
	// This interrupts at the user-specified frequency for the green LED.

	int delta_counts_m1;
	double delta_rotations;
	dt = ((double)G_green_period / 1000) / 60;	/* seconds / 60 = minutes */
	//dt = ((double)G_green_period / 1000);	/* seconds */

	curr_timestep++;
	updateLCD = true;
	
	G_green_toggles++;
	LED_TOGGLE(GREEN);

	delta_counts_m1 = global_counts_m1 - last_global_counts_m1;
	delta_rotations = (double)((double)delta_counts_m1 / ENCODER_DRIVEAXLE_RATIO);
	rotations += delta_rotations;
	
	curr_RPM = (delta_counts_m1 / dt) / ENCODER_DRIVEAXLE_RATIO;
	//RPM = curr_RPM;
	
	//if (curr_timestep % 4 == 0)		//only calculate speed every fourth interupt
	RPM = curr_RPM;
	
	//rolling average RPM calculation
	//RPM = ((delta_counts_m1 + prev_delta_count1 + prev_delta_count2 + prev_delta_count3) / (4 * dt)) / ENCODER_DRIVEAXLE_RATIO;

	prev_delta_count3 = prev_delta_count2;
	prev_delta_count2 = prev_delta_count1;
	prev_delta_count1 = delta_counts_m1;

	last_global_counts_m1 = global_counts_m1;

#ifdef USE_PID

	if (TARGET_ROT)
	{
		error = goal - rotations;			
	} 
	else if (TARGET_RPM)
	{
		error = goal - RPM;
	}


	if (TARGET_ROT)
	{
		if (fabs(error) < 360*error_threshold)	//only care about the integral gain if we're within one rotation of the target
		{
			integral = integral + error*dt;	
		}
	}
	else
	{
		integral = integral + error*dt;
	}
	
	
	if (!just_started_maneuver)			//if this is NOT the first ISR iteration for the current maneuver
		derivative = (error - pre_error)/dt;
	just_started_maneuver = false;
	
	
	if (TARGET_ROT)
		voltage = (Kp*error + Ki*integral + Kd*derivative);
	if (TARGET_RPM)
		voltage += (Kp*error + Ki*integral + Kd*derivative);
	
	
	if		(voltage > 0)		MOTOR_FORWARD;
	if		(voltage < 0)		MOTOR_BACKWARD;
	
	if		(voltage > 250.0)	voltage = 250.0;
	else if (voltage < -250.0)	voltage = -250.0;
	
	if (TARGET_ROT)
	{
		if		((9 > voltage) && (voltage >= 1))		voltage = 9.0;			//helps converge on solution faster
		else if ((-1 >= voltage) && (voltage > -9))		voltage = -9.0;			//helps converge on solution faster
		else											voltage = voltage;
	}
	if (TARGET_RPM)
		voltage = voltage;
	
	
	set_motors_custom(voltage,0);
	
	
	if (voltage != pre_voltage)			//if we just adjusted the voltage...
		equilibrium = 0;				//then reset our steady_state counter

	if ( (abs(voltage) < 250) && (fabs(error) <= error_threshold) ) //if we've been at the same voltage for 20 timesteps and it wasn't at max speed and we have low error.....
		equilibrium++;
		
	if ((equilibrium >= 3) && (abs(voltage) < 250) && (fabs(error) <= error_threshold) ) //if we've been at the same voltage for 5 timesteps and it wasn't at max speed and we have low error.....
	{
		maneuver_complete = DONE;
	}
		
	pre_voltage = voltage;
	
	if (log_values_now)
	{
		int delta = (curr_timestep - log_start_timestep);
		if (delta < MAX_TIMESTEP)			//then continue logging
		{
			LOG[delta].voltage		= (int16_t)voltage;
			LOG[delta].rotations	= (float)rotations;
			LOG[delta].RPM			= (float)RPM;
			
			LOG[delta].error		= (float)error;
			LOG[delta].integral		= (float)integral;
			LOG[delta].derivative	= (float)derivative;
			
		}
		else	//then we've reached our logging capactity and will dump to the results to the terminal
		{
			log_stop_timestep = curr_timestep;
			log_values_now = false;
			ready_to_dump_log = true;
		}
		
		//If we've determined that the maneuver is complete, go ahead and dump the results early to the terminal
		if (maneuver_complete == DONE && !running_trajectory)
		{
			log_stop_timestep = curr_timestep;
			log_values_now = false;
			ready_to_dump_log = true;
		}
	}

	pre_error = error;
#endif
	
}


void execute_Trajectory()
{
	command_list[0] = GO;		//0 means don't go, 1 means go, 2 means running, 3 means done
	command_list[1] = WAIT;
	command_list[2] = WAIT;
	
	maneuver_complete = WAIT;
	running_trajectory = true;
	
	process_Trajectory();
}

void process_Trajectory()
{
	if (running_trajectory == false)
		return;
	
		if (command_list[0] == GO) 
		{
			command_list[0] == RUN;			//start running
			maneuver_complete = RUN;
			USE_DEGREES = true;
			process_received_string("R90");
		}
		
		while (1)
		{
			if ( (fabs(error) < error_threshold) && (maneuver_complete == DONE) )
			{	
				maneuver_complete = false;
				command_list[0] = DONE;
				command_list[1] = GO;
				delay_ms(500);
				break;
			}
			else 
			{
				delay_ms(10);
			}
		}
		
		if (command_list[1] == GO)
		{
			command_list[1] == RUN;			//start running
			maneuver_complete = RUN;
			USE_DEGREES = true;
			process_received_string("R-270");
		}
		
		while (1)
		{
			if ( (fabs(error) < error_threshold) && (maneuver_complete == DONE) )
			{
				maneuver_complete = false;
				command_list[1] = DONE;
				command_list[2] = GO;
				delay_ms(500);
				break;
			}
			else
			{
				delay_ms(10);
			}
		}
		
		if (command_list[2] == GO)
		{
			command_list[2] == RUN;			//start running
			maneuver_complete = RUN;
			USE_DEGREES = true;
			process_received_string("R-265");
		}
		
		while (1)
		{
			if ( (fabs(error) < error_threshold) && (maneuver_complete == DONE) )
			{			
				maneuver_complete = false;
				command_list[2] = DONE;
				//command_list[2] = GO;
				break;
			}
			else
			{
				delay_ms(10);
			}
		}
	
		
	//command_list[0] = WAIT;
	//command_list[1] = WAIT;
	//command_list[2] = WAIT;
	
	running_trajectory = false;
	maneuver_complete = WAIT;
	
	
	if (TARGET_ROT)
		length = sprintf( tempBuffer, "Kp:%f Ki:%f Kd:%f Pr:%f Pm:%f T:%d\r\n", Kp, Ki, Kd, goal, rotations, voltage);
	if (TARGET_RPM)	{
		length = sprintf( tempBuffer, "Kp:%f Ki:%f Kd:%f Pr:%f Pm:%f T:%d\r\n", Kp, Ki, Kd, goal, RPM, voltage);
	}
	print_usb( tempBuffer, length );
				
	length = sprintf( tempBuffer, "C0:%d C1:%d C2:%d C3:%d C4:%d C5:%d Man:%d\r\n",	command_list[0],
	command_list[1],
	command_list[2],
	command_list[3],
	command_list[4],
	command_list[5],
	maneuver_complete);
	
	USE_DEGREES = false;		//set things back to normal
}
