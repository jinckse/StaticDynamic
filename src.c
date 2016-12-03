/** TODO
	1. Build manipulator to determine exact measurements for links and offsets
	2. Implement PD controller for Move function
*/

/*
	File: src.c
	Description: Robot manipulator that draws polygons (all measurements in SI units)
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

/* Canvas based on 8.5" by 11" sheet of paper with 0.5" margins */
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

/* Counter */
U8 n = 0;

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
		
	/* Is current point in first quadrant? */
	while(polygon[n][0] > 0) {

		/* Read point */
		point[0] = polygon[n][0];
		point[1] = polygon[n][1];
		point[2] = polygon[n][2];

		/* Calculate new angles using inverse kinematics */
		I_Kin(point, theta);

		/* Move motors to new angles */
		Move(theta);

		/* Read to next point */
		n++;
	}

	/* Brake all motors when complete */
	
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

	/* Angle formulas */
	float alpha = atan2(point[1], point[0]);
	float beta = asin( D1 / sqrt( ((point[0] * point[0]) + (point[1] * point[1]))) );
	theta[0] = alpha - beta;

	/* Adjsut for offsets */
	float point_0[3] = {0, 0, L0};

	float p_trans[3] = { ( point[0] - (D2 * cos(theta[0])) + (D1 * sin(theta[0])) ),
							 ( point[1] - (D1 * sin(theta[0])) - (D2 * cos(theta[0])) ),
							 ( point[2] + L3 )
						  };

	/* Set up remaining parameters */
	float L4 = sqrt( ( (p_trans[0] - point_0[0]) * (p_trans[1] - point_0[0]) )
		+ ( (p_trans[1] - point_0[1]) * (p_trans[1] - point_0[1]) )
		+ ( (p_trans[2] - point_0[2]) * (p_trans[2] - point_0[2]) ) );

	float beta2 = acos( ( (2* (L1 * L1)) - (L4 * L4) ) / (2 * (L1 * L1)) );

	float alpha2 = acos( (p_trans[2] - L0) / L4 );

	float delta = ( M_PI - beta2) / 2;

	/* Determine angles */
	theta[1] = alpha2 - delta - (M_PI / 2);
	theta[2] = M_PI - beta2;
	theta[3] = (M_PI / 2) - theta[1] - theta[2];
	theta[4] = 0;
	theta[5] = 0;

}

/*
	Function Name: move(float theta[])
	Description: 
		Pulses manipulator joint motors with assistance from PD-controller to move to given angle.
	Params:
		float theta[] - New angles
	Returns: 
		Nothing
*/
void Move(float theta[N_ANGLES]) {
	/* Move to new angles using current angles (rev count) from motors */	
}

