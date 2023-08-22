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
Pose_t expectedPose;

RobotParameters_t position_getRobotParams();
void position_setRobotParams(const RobotParameters_t* robotParams);

void position_updateExpectedPose();
Pose_t position_getExpectedPose();
const Pose_t* position_getCurrentPose();

void position_setAprilTagPose(const Pose_t* aprilTagPose);
const Pose_t* position_getAprilTagPose();

void position_setPoseDifference();
Pose_t position_getPoseDifference();


#endif /* POSITION_H_ */