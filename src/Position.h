/*
 * Position.h
 *
 * Created: 07.07.2023 16:23:19
 *  Author: Marius
 */ 


#ifndef POSITION_H_
#define POSITION_H_

#include <tools/variablesAccess.h>
#include <stdint.h>
#include <communication/packetTypes.h>
#include <tools/timeTask/timeTask.h>


RobotParameters_t value_robotParams;

RobotParameters_t position_getRobotParams();
void position_setRobotParams(const RobotParameters_t* robotParams);

void position_updateExpectedPose();
Pose_t* position_getExpectedPose();
//const Pose_t* position_getCurrentPose();

void position_setAprilTagPose(Pose_t* aprilTagPose);
Pose_t* position_getAprilTagPose();

void position_setExpectedPose(Pose_t* truePose);

void position_setPoseDifference();
Pose_t position_getPoseDifference();

void position_init();


#endif /* POSITION_H_ */