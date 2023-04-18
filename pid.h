/*
 * pid.h
 *
 *  Created on: 10-Mar-2023
 *      Author: dhine
 */

#ifndef INC_PID_H_
#define INC_PID_H_

typedef enum
	{
	Anti_windup_disabled=0,
	Anti_windup_enabled
	}Anti_windup_t;


typedef struct
	{
	float Kp;
	float Ki;
	float Kd;
	float Ts;
	float Set_point;
	float Anti_windup_error;
	float Outmin;
	float Outmax;
	int Anti_windup;

	}PID_Param_t;


void PID_init(PID_Param_t *par);
float PID_Calculation(float input);


#endif /* INC_PID_H_ */
