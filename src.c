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
/* Two drive one extend version */
#define DRIVE_MOTOR_1 NXT_PORT_A
#define DRIVE_MOTOR_2 NXT_PORT_C
#define LIFT_MOTOR NXT_PORT_B

/* One Drive two extend version */
#define DRIVE_MOTOR NXT_PORT_B
#define LIFT_MOTOR_1 NXT_PORT_A
#define LIFT_MOTOR_2 NXT_PORT_C

#define EXTEND_DELAY 2750
#define RETRACT_DELAY 1750

#define DRIVE_SPEED 50

/* 
	GLOBAL VARS 
*/
int behavior = 0;
int start_time = 0;
int start_time2 = 0;
int stair_cnt = 0;

/*
	FUNCTION PROTOTYPES
*/
void disp(int row, char *str, int val);
void drive(int spd);
void extend(int spd);
void retract(int spd);
void reverse(int spd);

/* LEJOS OSEK hooks */
void ecrobot_device_initialize() {}
void ecrobot_device_terminate() {}

/* nxtOSEK hook to be invoked from an ISR in cateory 2 */
void user_1ms_isr_type2(void){ /* do nothing unless we find a way to use it */ }

/* 
	Task for stair climber
*/
TASK(Task1)
{
	int temp_start = 0;

	start_time = systick_get_ms();

	/* Main loop */
	while(1){
		
		switch(behavior){

			/* Lift Extend Behavior */
			case 0: 

				/* Grab behavior start time */
				start_time = systick_get_ms();

				/* Begin lifting */
				while(systick_get_ms() < start_time + EXTEND_DELAY){
					extend(50);
					drive(50);
				}

				/* Stop everything */
				extend(0);
				drive(0);

				start_time2 = systick_get_ms();

				/* Drive forward after catching step */
				while(systick_get_ms() < start_time2 + 750){
					drive(80);
				}
				drive(0);

				/* Change to Retract behavior */
				behavior = 1; 
				break;

			/* Lift Retract Behavior */
			case 1: 
				systick_wait_ms(500);

				/* Grab behavior start time */
				start_time = systick_get_ms();

				/* Begin retracting */
				while(systick_get_ms() < start_time + RETRACT_DELAY){
					temp_start = systick_get_ms();
					retract(50);
				}

				/* Stop everything */
				retract(0);
				drive(0);

				start_time2 = systick_get_ms();

				/* Scoot forward after catching step */
				while(systick_get_ms() < start_time2 + 100){
					drive(30);
				}
				systick_wait_ms(1000);
				while(systick_get_ms() < start_time2 + 100){
					drive(30);
				}
				systick_wait_ms(1000);
				while(systick_get_ms() < start_time2 + 100){
					drive(30);
				}
				drive(0);

				/* Stabilize */
				systick_wait_ms(500);

				/* Climb two stepss */
				if(stair_cnt < 1) {
					behavior = 0;
				}
				else {
					behavior = 2;
				}

				stair_cnt++;

				break;

			case 2:
				break;

			default:
				break;
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
	nxt_motor_set_speed(DRIVE_MOTOR, spd, 1); 			
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
	nxt_motor_set_speed(DRIVE_MOTOR, -spd, 1); 			
}

