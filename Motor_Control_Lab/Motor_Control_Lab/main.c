/* Motor_Control_Lab - an application for the Pololu Orangutan SVP
 *
 * This application uses the Pololu AVR C/C++ Library.  For help, see:
 * -User's guide: http://www.pololu.com/docs/0J20
 * -Command reference: http://www.pololu.com/docs/0J18
 *
 * Created: 3/5/2015 11:10:22 AM
 *  Author: csadlo
 */

#define ECHO2CLD

#include "menu.h"

#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <inttypes.h>

#undef THROTTLE_MODE
#undef LAB_MODE

//#define	THROTTLE_MODE
#define LAB_MODE

extern void loadPositionGains();
extern void loadSpeedGains();
extern void execute_Trajectory();
extern void resetValues();

enum DIR {Forward='F', Reverse='R', Stop='S'};

// GLOBALS
extern uint32_t G_red_toggles;
extern uint32_t G_green_toggles;
extern uint32_t G_yellow_toggles;

extern double Kp;
extern double Ki;
extern double Kd;
extern double goal;
extern int16_t voltage;

extern bool TARGET_ROT;
extern bool TARGET_RPM;
extern bool USE_DEGREES;
extern bool running_Trajectory;

extern double rotations;


//Logging Variables
extern uint16_t curr_timestep;
extern uint16_t log_start_timestep;
extern uint16_t log_stop_timestep;
extern bool log_values_now;
extern bool display_values_flag;
bool prepare_to_start_log = false;
extern bool ready_to_dump_log;
extern bool just_started_maneuver;


// local "global" data structures
char receive_buffer[32];
unsigned char receive_buffer_position;
char send_buffer[32];

// A generic function for whenever you want to print to your serial comm window.
// Provide a string and the length of that string. My serial comm likes "\r\n" at 
// the end of each string (be sure to include in length) for proper linefeed.
void print_usb( char *buffer, int n ) {
	serial_send( USB_COMM, buffer, n );
	wait_for_sending_to_finish();
}	
		
//------------------------------------------------------------------------------------------
// Initialize serial communication through USB and print menu options
// This immediately readies the board for serial comm
void init_menu() {
	
	//char printBuffer[32];
	
	// Set the baud rate to 9600 bits per second.  Each byte takes ten bit
	// times, so you can get at most 960 bytes per second at this speed.
	serial_set_baud_rate(USB_COMM, 9600);

	// Start receiving bytes in the ring buffer.
	serial_receive_ring(USB_COMM, receive_buffer, sizeof(receive_buffer));

	//memcpy_P( send_buffer, PSTR("USB Serial Initialized\r\n"), 24 );
	//snprintf( printBuffer, 24, "USB Serial Initialized\r\n");
	//print_usb( printBuffer, 24 );
	print_usb( "USB Serial Initialized\r\n", 24);

	//memcpy_P( send_buffer, MENU, MENU_LENGTH );
	print_usb( MENU, MENU_LENGTH );
}



//------------------------------------------------------------------------------------------
// process_received_byte: Parses a menu command (series of keystrokes) that 
// has been received on USB_COMM and processes it accordingly.
// The menu command is buffered in check_for_new_bytes_received (which calls this function).
void process_received_string(const char* buffer)
{
	// Used to pass to USB_COMM for serial communication
	int length;
	char tempBuffer[80];
	
	// parse and echo back to serial comm window (and optionally the LCD)
	char op_cmd;
	float op_value = 0.0;
	
    int temp_goal = 0;
	
//THROTTLE MODE requires setting putty to force sending text immediately	
#ifdef THROTTLE_MODE

	length = sscanf(buffer, "%c", &op_cmd);
	lcd_goto_xy(0,0);
	printf("Got %c", op_cmd);

	// convert  to upper and check if valid
	op_cmd -= 32*(op_cmd>='a' && op_cmd<='z');
	switch (op_cmd) {
		case 'R':
			print_usb("Switching direction\r\n",22);
			int difference = 2*abs(voltage);
			if (voltage > 0)
			{
				for (int i=0; i<difference; i+=10)
				{
					voltage -= 10;
					delay_ms(50);
					//OCR2A = voltage;
					//set_motors(voltage, 0);
				}
			} else {
				for (int i=0; i<difference; i+=10)
				{
					voltage += 10;
					delay_ms(50);
					//OCR2A = voltage;
					//set_motors(voltage, 0);
				}
			}
			break;
		case 'S':
			print_usb("Stopped\r\n",16);
			//motor1_dir = Stop;
			voltage = (int16_t)0;
			break;
		case 'U':
			voltage += 10;
			if (voltage > 255)
			voltage = (int16_t)250;
			break;
		case 'J':
			voltage -= 10;
			if (voltage < -255)
			voltage = (int16_t)-250;
			break;
		default:
			print_usb( "Command not recognized. Try {UJS}\r\n", 32 );
			print_usb( MENU, MENU_LENGTH);
		return;
	}

	length = sprintf( tempBuffer, "Op:%c V:%d\r\n", op_cmd, voltage);
	print_usb( tempBuffer, length );	
#endif	


#ifdef LAB_MODE
	
	//length = sprintf( tempBuffer, "Op:%c V:%f\n\r", op_cmd, op_value );
	//print_usb( tempBuffer, length );
	
	if (strlen(buffer) >= 2)
		length = sscanf(buffer, "%c %d\n", &op_cmd, &temp_goal);
	else
		length = sscanf(buffer, "%c", &op_cmd);

	op_value = (float)temp_goal;
	
	//G_green_period = 10;
		
	op_cmd -= 0;
	
	switch (op_cmd) {
	
	//Toggle Logging	
		case 'l':
		case 'L':
			prepare_to_start_log = log_values_now ? false : true;	//prepare to start logging if we weren't already logging, otherwise we must stop logging early
			
			if (log_values_now) 
			{
				log_values_now = false;	//we're done logging
				log_stop_timestep = curr_timestep;
				length = sprintf( tempBuffer, "We're done logging values.\n\r");
				print_usb( tempBuffer, length );
				ready_to_dump_log = true;
			}
			else
			{
				ready_to_dump_log = false;
				length = sprintf( tempBuffer, "Use the R/r or S/s command to begin logging.\n\r");
				print_usb( tempBuffer, length );
				
			}
			break;
			
	//Trigger display
		case 'V':
			display_values_flag = true;
			break;
		case 'v':
			display_values_flag = true;
			break;

	//Set reference position (in Rotations)			
		case 'R':
		case 'r':
			if (!TARGET_ROT)			//if we weren't doing position
				loadPositionGains();	//then reset position constants (don't refresh between every command to make experiments easier)
				
			TARGET_ROT = true;
			TARGET_RPM = false;
				
			resetValues();

			if (USE_DEGREES)
				goal = (double)(op_value/360.0);
			else
				goal = op_value;
			
			//if (!running_Trajectory)
			//	log_start_timestep = curr_timestep;
			just_started_maneuver = true;
			
			if (prepare_to_start_log)
			{
				length = sprintf( tempBuffer, "Beginning to log position test...\n\r");
				print_usb( tempBuffer, length );
				prepare_to_start_log = false;  //preparation is done; Ready, Set, GO!!!
				log_values_now = true;
				log_start_timestep = curr_timestep;
			}
			break;
			
	//Set reference speed (in RPM)			
		case 'S':
		case 's':
			if (!TARGET_RPM)		//if we weren't doing speed
				loadSpeedGains();	//then reset speed constants (don't refresh between every command to make experiments easier)

			TARGET_ROT = false;
			TARGET_RPM = true;
				
			resetValues();

			if (USE_DEGREES)
				goal = (double)(op_value/360.0);
			else
				goal = op_value;
			
			//log_start_timestep = curr_timestep;
			just_started_maneuver = true;
			
			if (prepare_to_start_log)
			{
				length = sprintf( tempBuffer, "Beginning to log speed test...\n\r");
				print_usb( tempBuffer, length );
				prepare_to_start_log = false;  //preparation is done; Ready, Set, GO!!!
				log_values_now = true;
				log_start_timestep = curr_timestep;
			}
			break;
			
	//Adjusting Kp
		case 'P':
			Kp += 0.1;
			break;
		case 'p':
			Kp -= 0.1;
			break;
			
	//Adjusting Ki			
		case 'I':
			Ki += 1.0;
			break;
		case 'i':
			Ki -= 1.0;
			break;
			
	//Adjusting Kd
		case 'D':
			Kd += 0.01;
			break;
		case 'd':
			Kd -= 0.01;
			break;
		
		case 'B':
		case 'b':
			Kp = op_value;
			break;
		
		case 'N':
		case 'n':
			Ki = op_value;
			break;
		
		case 'M':
		case 'm':
			Kd = op_value;
			break;
		
		case 'T':
		case 't':
			execute_Trajectory();
			break;
		
	//Execute trajectory		

		default:
			//print_usb("Command not recognized. Try again\n\r", 35 );
			break;
			//print_usb( MENU, MENU_LENGTH);
	}
	
	//length = sprintf( tempBuffer, "Op:%c V:%d\r\n", op_dir, op_value );
	//print_usb( tempBuffer, length );
	
#endif
	
} //end menu()

//---------------------------------------------------------------------------------------
// If there are received bytes to process, this function loops through the receive_buffer
// accumulating new bytes (keystrokes) in another buffer for processing.
void check_for_new_bytes_received()
{
	/* 
	The receive_buffer is a ring buffer. The call to serial_check() (you should call prior to this function) fills the buffer.
	serial_get_received_bytes is an array index that marks where in the buffer the most current received character resides. 
	receive_buffer_position is an array index that marks where in the buffer the most current PROCESSED character resides. 
	Both of these are incremented % (size-of-buffer) to move through the buffer, and once the end is reached, to start back at the beginning.
	This process and data structures are from the Pololu library. See examples/serial2/test.c and src/OrangutanSerial/ *.*
	
	A carriage return from your comm window initiates the transfer of your keystrokes.
	All key strokes prior to the carriage return will be processed with a single call to this function (with multiple passes through this loop).
	On the next function call, the carriage return is processes with a single pass through the loop.
	The menuBuffer is used to hold all keystrokes prior to the carriage return. The "received" variable, which indexes menuBuffer, is reset to 0
	after each carriage return.
	*/ 
	char menuBuffer[32];
	static int received = 0;
	
	// while there are unprocessed keystrokes in the receive_buffer, grab them and buffer
	// them into the menuBuffer
	while(serial_get_received_bytes(USB_COMM) != receive_buffer_position)
	{
		// place in a buffer for processing
		menuBuffer[received] = receive_buffer[receive_buffer_position];
		++received;
		
		// Increment receive_buffer_position, but wrap around when it gets to
		// the end of the buffer. 
		if ( receive_buffer_position == sizeof(receive_buffer) - 1 )
		{
			receive_buffer_position = 0;
		}			
		else
		{
			receive_buffer_position++;
		}
	}
	// If there were keystrokes processed, check if a menu command
	if (received) {
		// if only 1 received, it was MOST LIKELY a carriage return. 
		// Even if it was a single keystroke, it is not a menu command, so ignore it.
		//if (!throttle_mode) {
//#ifndef THROTTLE_MODE
			if ( 1 == received ) {
//				received = 0;
//				return;
			}
//#endif
		//}
		// Process buffer: terminate string, process, reset index to beginning of array to receive another command
		menuBuffer[received] = '\0';
//#ifdef ECHO2LCD
/*
		lcd_goto_xy(0,1);			
		print("RX: (");
		print_long(received);
		print_character(')');
		for (int i=0; i<received; i++)
		{
			print_character(menuBuffer[i]);
		}*/
//#endif
		process_received_string(menuBuffer);
		received = 0;
	}
	return;
}
	
//-------------------------------------------------------------------------------------------
// wait_for_sending_to_finish:  Waits for the bytes in the send buffer to
// finish transmitting on USB_COMM.  We must call this before modifying
// send_buffer or trying to send more bytes, because otherwise we could
// corrupt an existing transmission.
void wait_for_sending_to_finish()
{
	while(!serial_send_buffer_empty(USB_COMM))
		serial_check();		// USB_COMM port is always in SERIAL_CHECK mode
}


void launch_Trajectory()
{
	
	process_received_string("R0.25");
	
	process_received_string("R-0.75");
	
	process_received_string("R0.25");	//5 degrees is 0.0138888 rotations
	
}