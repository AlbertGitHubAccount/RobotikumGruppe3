/*
 * Position.c
 *
 * Created: 07.07.2023 16:04:48
 *  Author: Marius
 */ 

#include "Position.h"
#include "robotControl.h"

#include <avr/io.h>
#include <communication/packetTypes.h>
#include <math.h>
#include <Encoder.h>
#include <tools/timeTask/timeTask.h>


RobotParameters_t value_robotParams = { .axleWidth = 134.0f, .distPerTick = 45.0f * 1.1494f * M_PI / 1024.0f, .user1 = 0.0f, .user2 = 0.0f };
static Pose_t expectedPose;

static Pose_t truePose;
static Pose_t poseDifference;

RobotParameters_t position_getRobotParams(){
	return value_robotParams;
}

void position_setRobotParams(const RobotParameters_t* robotParams){
	value_robotParams = *robotParams;
}

const Pose_t* position_getAprilTagPose(){
	return &truePose; //For better path
}

void position_setAprilTagPose(const Pose_t* aprilTagPose){
	truePose = *aprilTagPose; //
}

void position_updateExpectedPose() {
	int16_t l;
	int16_t r;
	encoder_getCountersAndReset(&l, &r);
	
	float dx;
	float dy;
	int16_t diffLR = r - l;
	if (diffLR == 0) {
		float d = (float) r * value_robotParams.distPerTick;
		dx = d * cosf(expectedPose.theta);
		dy = d * sinf(expectedPose.theta);
	} 
	else {
		float dTheta	= diffLR * value_robotParams.distPerTick / value_robotParams.axleWidth; //dTheta wird für dx und dy verwendet
	
		float R = ((float)(r + l) / (float)diffLR) * (value_robotParams.axleWidth/2.0f); //zwischenrechnung für Übersichtilichen Code
		dx		= R * (sinf(expectedPose.theta + dTheta) - sinf(expectedPose.theta));
		dy		= R * (cosf(expectedPose.theta) - cosf(expectedPose.theta + dTheta));
		expectedPose.theta	+= dTheta;
		
		if (expectedPose.theta > 2.0f * M_PI)
			expectedPose.theta -= 2.0f * M_PI;
		if (expectedPose.theta < 0.0f)
			expectedPose.theta += 2.0f * M_PI;
	}
	
	expectedPose.x		+= dx;
	expectedPose.y		+= dy;
}

const Pose_t* position_getExpectedPose(){
	return &expectedPose;
}

/*
const Pose_t* position_getCurrentPose(){
	return &expectedPose;
}
*/

void position_setPoseDifference(){
	poseDifference.x		= truePose.x		- expectedPose.x;
	poseDifference.y		= truePose.y		- expectedPose.y;
	poseDifference.theta	= truePose.theta	- expectedPose.theta;
}
	
Pose_t position_getPoseDifference(){
	return poseDifference;
}