/*
	File: src.c

	Description: Stair climbing robot code for final project

	Author: Team 6
*/
#include <stdlib.h> 
#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"
	
/*
	MACROS 
*/

/* For sweep tone */
#define DRIVE_MOTOR NXT_PORT_B

#define LIFT_MOTOR_1 NXT_PORT_A
#define LIFT_MOTOR_2 NXT_PORT_C

#define COLOR_F1 NXT_PORT_S1
#define COLOR_F2 NXT_PORT_S2
#define COLOR_B1 NXT_PORT_S3
#define COLOR_B2 NXT_PORT_S4

/* 
	GLOBAL VARS 
*/
int spd = 45;
int delay = 500;
int turning = 0;
int behavior = 0;
int color_delay = 15;
int running = 0;
int black = 0;
int start_time = 0;
int start_time2 = 0;

/* Color sensor params */
U32 U32_freq;
S16 buf_f1[3] = {-1,-1,-1};
S16 buf_f2[3] = {-1,-1,-1};
S16 buf_b1[3] = {-1,-1,-1};
S16 buf_b2[3] = {-1,-1,-1};

/*
	FUNCTION PROTOTYPES
*/
int getRandom(int min, int max);
void disp(int row, char *str, int val);
void drive(int spd);
void extend(int spd);
void retract(int spd);
void reverse(int spd);
void turn(int spd);

/* LEJOS OSEK hooks */
void ecrobot_device_initialize() 
{
	ecrobot_init_color_sensor(COLOR_F1);	
	ecrobot_init_color_sensor(COLOR_F2);	
	ecrobot_init_color_sensor(COLOR_B1);	
	ecrobot_init_color_sensor(COLOR_B2);	
}
void ecrobot_device_terminate() 
{
	ecrobot_term_color_sensor(COLOR_F1);
	ecrobot_term_color_sensor(COLOR_F2);
	ecrobot_term_color_sensor(COLOR_B1);
	ecrobot_term_color_sensor(COLOR_B2);
}

/* nxtOSEK hook to be invoked from an ISR in cateory 2 */
void user_1ms_isr_type2(void){ /* do nothing unless we find a way to use it */ }

/* 
	Task for stair climber
*/
TASK(Task1)
{
	running = 1;
	behavior = 1;

	int temp_start = 0;

	start_time = systick_get_ms();

	while(systick_get_ms() < (start_time + color_delay)){
		ecrobot_get_color_sensor(COLOR_F1, buf_f1);
		ecrobot_get_color_sensor(COLOR_F2, buf_f2);
		ecrobot_get_color_sensor(COLOR_B1, buf_b1);
		ecrobot_get_color_sensor(COLOR_B2, buf_b2);
	}

	while(running){
		
		switch(behavior){

			/* Drive forward */
			case 0: 
				drive(30);

				if((buf_f1[0] == 0) && (buf_f1[1] == 0) && (buf_f1[2] == 0) 
					&& (buf_f2[0] == 0) && (buf_f2[1] == 0) && (buf_f2[2] == 0)){

					behavior = 1; // set to extend
					drive(0);
				}
				break;

			/* Extend lift */
			case 1: 
				start_time = systick_get_ms();
				while(systick_get_ms() < start_time + 2950){
					extend(50);
					drive(50);
				}
				extend(0);
				drive(0);
				start_time2 = systick_get_ms();

				while(systick_get_ms() < start_time2 + 750){
					drive(80);
				}
				drive(0);

				/* Change to Retract behavior */
				behavior = 2; 
				break;

			/* Retract lift */
			case 2: 
				start_time = systick_get_ms();

				//over a 5 second interval, alternate between retracting and driving
				while(systick_get_ms() < start_time + 10000){
					temp_start = systick_get_ms();

					while(systick_get_ms() < temp_start + 250){
						drive(0);
						retract(50);
					}

					temp_start = systick_get_ms();

					while(systick_get_ms() < temp_start + 1000){
						retract(0);
						drive(100);
					}
					
				}
				retract(0);
				drive(0);
				start_time2 = systick_get_ms();

				while(systick_get_ms() < start_time2 + 250){
					retract(30);
					drive(30);
				}
				retract(0);
				drive(0);
				behavior = 3;
				break;

			case 3:
				drive(0);
				behavior = 3;

			//case 4:
			//case 5:		
		}
	}

	extend(0);
	TerminateTask();
}


/* Sub functions */

/* 
 * LEJOS OSEK V1.07 has an interrupt based LCD display feature thanks to LEJOS NXJ develpment team.
 * Therefore, LCD display performance is dramatically improved. Actually, LCD update is too
 * fast to see. The purpose of this speed test is to measure the performance of LEJOS OSEK under
 * a common proram load compared to other prorammin lanuaes. (The maximum LCD refresh rate
 * on hardware level is 60Hz, so faster refresh rate than 60Hz has no meanin in practical use.)
 * So the result could be seen only at the end of the test
 */
void disp(int row, char *str, int val)
{
#define DISPLAY_ON
#ifdef DISPLAY_ON 
	display_clear(0);
	display_goto_xy(0, row);
	display_string(str);
	display_int(val, 0);
	display_update();
#endif
}

/*
	Function Name: retract

	Description: 
		Retract lift gears

	Params:
		int spd - Desired speed

	Returns: 
		Nothing
*/
void retract(int spd){
	nxt_motor_set_speed(LIFT_MOTOR_1, -spd, 1);
	nxt_motor_set_speed(LIFT_MOTOR_2, -spd, 1);
}

/*
	Function Name: extend

	Description: 
		Extend lift gears

	Params:
		int spd - Desired speed

	Returns: 
		Nothing
*/
void extend(int spd){

	nxt_motor_set_speed(LIFT_MOTOR_1, spd, 1);
	nxt_motor_set_speed(LIFT_MOTOR_2, spd, 1);
}

/*
	Function Name: drive

	Description: 
		Pulses robot drive motors forward at desired rate indicated by spd parameter.

	Params:
		int spd - Desired speed

	Returns: 
		Nothing
*/
void drive(int spd) {
	if (spd != 0) {

		nxt_motor_set_speed(DRIVE_MOTOR, spd, 1); 			
	}else{
		nxt_motor_set_speed(DRIVE_MOTOR, spd, 1); 	
	}
}

/*
	Function Name: reverse

	Description: 
		Pulses robot drive motors backward at desired rate indicated by spd parameter.

	Params:
		int spd - Desired speed

	Returns: 
		Nothing
*/
void reverse(int spd) {
		nxt_motor_set_speed(NXT_PORT_B, -spd, 1); 			
		nxt_motor_set_speed(NXT_PORT_C, -spd, 1); 			
}

/*
	Function Name: turn

	Description: 
		Pulses robot drive motors in opposite directions at desired rate indicated by spd parameter.

	Params:
		int spd - Desired speed

	Returns: 
		Nothing
*/
void turn(int spd) {
		nxt_motor_set_speed(NXT_PORT_B, -spd, 1); 			
		nxt_motor_set_speed(NXT_PORT_C, spd, 1); 			
}

