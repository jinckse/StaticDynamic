/*
	File: src.c
	Description: Robot manipulator that draws polygons
	Author: Team 6
*/
#include <stdlib.h> 
#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"	
#include <math.h>

/*
	MACROS 
*/

#ifndef M_PI
#define M_PI 3.1415927
#endif

/* Lengths */
#define L0 0.25
#define L1 0.2
#define L2 0.2
#define L3 0.15

/* Offsets */
#define D1 0.04
#define D2 0.04

/* Angles */
#define N_ANGLES 6

/* Movements */
#define MAX_ROTATION 90
#define MAX_SPEED 50 

/* Polygon */
#define MAX_POLYGON 10

/* Joints */
#define J0 NXT_PORT_A
#define J1 NXT_PORT_B
#define J2 NXT_PORT_C

/* Canvas */
#define CANVAS_M 0.267
#define CANVAS_N 0.203
#define CANVAS_MIDPOINT_X (CANVAS_N / 2)F
#define CANVAS_MIDPOINT_Y (CANVAS_M / 2)F
	
/* 
	GLOBAL VARS 
*/

/* Current angles calcuated using inverse kinematics */
float theta[N_ANGLES] = {-1, -1, -1, -1, -1 ,-1};

/* Current point desired */
float point[3] = {-1, -1, -1};

/* Coordinates of each polygon apex */
float polygon[MAX_POLYGON][3]= { {-1, -1, -1},
										{-1, -1, -1}, 
										{-1, -1, -1}, 
										{-1, -1, -1}, 
										{-1, -1, -1},
										{-1, -1, -1}, 
										{-1, -1, -1}, 
										{-1, -1, -1}, 
										{-1, -1, -1},
										{-1, -1, -1} };

/*
	FUNCTION PROTOTYPES
*/
void I_Kin(float point[3], float theta[N_ANGLES]);
void Move(float theta[N_ANGLES]);

/* LEJOS OSEK hooks */
void ecrobot_device_initialize() {
}

/* nxtOSEK hook to be invoked from an ISR in category 2 */
void user_1ms_isr_type2(void){ /* do nothing */ }

/* 
	Task for manipulator  
*/
TASK(Task1)
{
	/* 
		Main loop 
	*/
	while(1) {

		/* Read Coordinate */

		/* Is current coordinate in first quadrant? */

		/* Calculate new angles using inverse kinematics */

		/* Move motors to new angles */

	}
	
	TerminateTask();
}

/* Functions */

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

/*
	Function Name: I_Kin(float point[], float theta[])
	Description: 
		Pulses robot drive motors backward at desired rate indicated by spd parameter.
	Params:
		float coord[3] - Desired coordinate
	Returns: 
		Nothing
*/
void I_Kin(float point[3], float theta[N_ANGLES]) {

}

/*
	Function Name: move(float theta[])
	Description: 
		Pulses manipulator joint motors with assistance from PD-controller to move to given angle.
	Params:
		float angle - New angle
	Returns: 
		Nothing
*/
void Move(float theta[N_ANGLES]) {
	
}

