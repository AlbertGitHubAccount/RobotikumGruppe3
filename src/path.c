/*
 * path.c
 *
 * Created: 12.07.2023 15:10:24
 *  Author: Daniel
 */ 

#include "path.h"
#include "robotControl.h"
#include "Position.h"
#include <motor/motor.h>
#include <math.h>
#include <communication/packetTypes.h>

#define MIN_PWM 850
#define MAX_PWM 3000 //8191
#define MAX_V 150.0f //380.0f

float calcDifAngle(const Pose_t* pose, const FPoint_t* lookahead);
//static float thetaRobot = pose.theta;

void calculateDriveCommand(const Pose_t* pose, const FPoint_t* lookahead){
	float dif;
	dif = calcDifAngle(pose, lookahead);
	float normalV = 50.0f;//mm pro s
	//float speedToPWM = 10.0f;//true Value unknown
	float vDif = 2.5f * dif * value_robotParams.axleWidth / 2.0f;
	float mappingValue = 19.0476f;
	
	float vL = normalV - vDif;
	int16_t pwmL = vL * mappingValue + MIN_PWM;
	if (vL < 0.0f)
		pwmL = pwmL * -1.0f;
	if(pwmL > MAX_PWM)
		pwmL = MAX_PWM;
	
	
	float vR = normalV + vDif;
	int16_t pwmR = vR * mappingValue + MIN_PWM;
	if (vR < 0.0f)
		pwmR = pwmR * -1.0f;
	if(pwmR > MAX_PWM)
		pwmR = MAX_PWM;
	
	Motor_setPWM(pwmL,pwmR);
}

float calcDifAngle(const Pose_t* pose, const FPoint_t* lookahead){
	float pLx = lookahead->x;
	float pLy = lookahead->y;
	float pRx = pose->x;
	float pRy = pose->y;
	float a = pLy - pRy;
	float b = pLx - pRx;
	float gamma = atan2(a, b);
	float dif = gamma - pose->theta;
	
	if(dif < -M_PI){
		dif += 2.0f * M_PI;
	}
	else if(dif > M_PI){
		dif -= 2.0f * M_PI;
	}
	
	return dif;
}
