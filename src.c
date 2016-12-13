/**TODO
	1. The color detection works, and switches behavior, but it doesn't appear to detect the color
		change quick enough in all cases. The spin can also cause it too lose site as well, and 
		switch behaviors.

	2. Now that the contorller code is removed from the drive functions, using the sonar may now
		also work, as the major problems may have been a result of the controller code.

	3. Might be able to use light sensor to slow down when flame is detected to give more time
		for the color sensor to register the black paper.

	4. Define macros for port names as the relate to thier sensors and motors to make code easier
		to understand.
*/

/*
	File: src.c

	Description: Fire detection robot code for Group Project 2.

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
#define FREQ_MAX 2093 /* C,7 */
#define FREQ_MIN 31 /* B,0 */
#define ASCEND 1.059463094F /* 2^(1/12) */
#define DESCEND 0.943874313F /* 2^(-1/12) */

#define DRIVE_MOTOR NXT_PORT_B
#define LIFT_MOTOR_1 NXT_PORT_PORT_A
#define LIFT_MOTOR_2 NXT_PORT_PORT_C
#define COLOR_F1 NXT_PORT_PORT_S1
#define COLOR_F2 NXT_PORT_PORT_S2
#define COLOR_B1 NXT_PORT_PORT_S3
#define COLOR_B2 NXT_PORT_PORT_S4

/* 
	GLOBAL VARS 
*/
int g_spd = 45;
int g_delay = 500;
int g_turning = 0;
int g_behavior = 0;
int g_color_delay = 15;
int g_bump = 0;
int g_bump_time = 0;
int g_fire_found = 0;
int g_running = 0;
int black = 0;
int g_start_time = 0;
int g_start_time2 = 0;

/* For sweep tone */
double g_freq = FREQ_MIN;
double g_gain = ASCEND;

/* Color sensor params */
U32 g_U32_freq;
S16 g_buf1[3] = {-1,-1,-1};
S16 g_buf2[3] = {-1,-1,-1};
S16 g_buf3[3] = {-1,-1,-1};
S16 g_buf4[3] = {-1,-1,-1};

/*
	FUNCTION PROTOTYPES
*/
int getRandom(int min, int max);
void disp(int row, char *str, int val);
void drive(int spd);
void go_up(int spd);
void retract(int spd);
void reverse(int spd);
void turn(int spd);
void alarm(void);

/* LEJOS OSEK hooks */
void ecrobot_device_initialize() 
{
	ecrobot_init_color_sensor(NXT_PORT_S1);	
	ecrobot_init_color_sensor(NXT_PORT_S2);	
	ecrobot_init_color_sensor(NXT_PORT_S3);	
	ecrobot_init_color_sensor(NXT_PORT_S4);	
}
void ecrobot_device_terminate() 
{
	ecrobot_term_color_sensor(NXT_PORT_S1);
	ecrobot_term_color_sensor(NXT_PORT_S2);
	ecrobot_term_color_sensor(NXT_PORT_S3);
	ecrobot_term_color_sensor(NXT_PORT_S4);
}

/* nxtOSEK hook to be invoked from an ISR in category 2 */
void user_1ms_isr_type2(void){ /* do unless we find a way to use it */ }

/* 
	Task for stair climber
*/
TASK(Task1)
{
	//int start_time = systick_get_ms();
	//while(systick_get_ms() < start_time + 2250){
	//	drive(30);
	//}
	
	//go_up(0);
	//int start_time2 = systick_get_ms();
	//while(systick_get_ms() < start_time2 + 2250){
	//	retract(30);
	//}
	g_running = 1;
	g_behavior = 1;
	int temp_start = 0;
	g_start_time = systick_get_ms();
	while(systick_get_ms() < (g_start_time + g_color_delay)){
		ecrobot_get_color_sensor(NXT_PORT_S4, g_buf1);
		ecrobot_get_color_sensor(NXT_PORT_S4, g_buf2);
		ecrobot_get_color_sensor(NXT_PORT_S4, g_buf3);
		ecrobot_get_color_sensor(NXT_PORT_S4, g_buf4);
	}
	while(g_running){
		
		switch(g_behavior){
			case 0: // "drive forward" state
				drive(30);
				if((g_buf1[0] == 0) && (g_buf1[1] == 0) && (g_buf1[2] == 0) 
				&& (g_buf2[0] == 0) && (g_buf2[1] == 0) && (g_buf2[2] == 0)){
					g_behavior = 1; // set to go_up
					drive(0);
				}
				break;
			case 1: // "go up" state
				g_start_time = systick_get_ms();
				while(systick_get_ms() < g_start_time + 2950){
					go_up(50);
					drive(50);
				}
				go_up(0);
				drive(0);
				g_start_time2 = systick_get_ms();
				while(systick_get_ms() < g_start_time2 + 750){
					drive(80);
				}
				drive(0);
				g_behavior = 2; // set to retract
				break;
			case 2: // "retract" state
				g_start_time = systick_get_ms();
				//over a 5 second interval, alternate between retracting and driving
				while(systick_get_ms() < g_start_time + 10000){
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
				g_start_time2 = systick_get_ms();
				while(systick_get_ms() < g_start_time2 + 250){
					retract(30);
					drive(30);
				}
				retract(0);
				drive(0);
				g_behavior = 3;
				//g_running = 0;
				break;
			case 3:
				drive(0);
				//retract(0);
				g_behavior = 3;
			//case 4:
			//case 5:		
		}
	}

	
	go_up(0);
	TerminateTask();
}


/* Sub functions */

/* 
 * LEJOS OSEK V1.07 has an interrupt based LCD display feature thanks to LEJOS NXJ develpment team.
 * Therefore, LCD display performance is dramatically improved. Actually, LCD update is too
 * fast to see. The purpose of this speed test is to measure the performance of LEJOS OSEK under
 * a common program load compared to other programming languages. (The maximum LCD refresh rate
 * on hardware level is 60Hz, so faster refresh rate than 60Hz has no meaning in practical use.)
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

void retract(int spd){
	nxt_motor_set_speed(NXT_PORT_A, -spd, 1);
	nxt_motor_set_speed(NXT_PORT_C, -spd, 1);
}

void go_up(int spd){
	//sound_freq(233, 400);
	//int rev_constant = 50;
        //int rev_a = nxt_motor_get_count(NXT_PORT_A);
	//int rev_c = nxt_motor_get_count(NXT_PORT_C);
	//int total_rev_a = rev_a + rev_constant; 
	//int total_rev_c = rev_c + rev_constant;
	//while((rev_a < total_rev_a) && (rev_c < total_rev_c)){
	//	nxt_motor_set_speed(NXT_PORT_A, spd, 1);
	//	nxt_motor_set_speed(NXT_PORT_C, spd, 1);
	//	rev_a = nxt_motor_get_count(NXT_PORT_A);
	//	rev_c = nxt_motor_get_count(NXT_PORT_C);
	//}

	nxt_motor_set_speed(NXT_PORT_A, spd, 1);
	nxt_motor_set_speed(NXT_PORT_C, spd, 1);
	//sound_freq(233*2, 400);
}
/*
	Function Name: drivev_a = nxt_motor_get_count(NXT_PORT_A)

	Description: 
		Pulses robot drive motors forward at desired rate indicated by spd parameter.

	Params:
		int spd - Desired speed

	Returns: 
		Nothing
*/
void drive(int spd) {
	if (spd != 0) {

		nxt_motor_set_speed(NXT_PORT_B, spd, 1); 			
	}else{
		nxt_motor_set_speed(NXT_PORT_B, spd, 1); 	
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

/*
	Function Name: alarm

	Description: 
		Plays sweeping alarm tone. Not completely sure how it works. Taken from 
			../nxtOSEK/samples_c/soundtest/soundtest.c

	Params:
		None

	Returns: 
		Nothing
*/
void alarm(void) {

	/* Rounding */
	g_U32_freq = (U32)(g_freq + 0.5); 
	sound_freq(g_U32_freq, 5000);

	/* Fire fire!!! sweep tone */
	if (g_freq <= FREQ_MAX) g_gain = DESCEND;
	else if (g_freq <= FREQ_MIN) g_gain = ASCEND;
	g_freq *= g_gain;
}
