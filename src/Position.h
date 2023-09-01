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

Pose_t expectedPose;
void position_updateExpectedPose(Pose_t* expectedPose);
const Pose_t* position_getExpectedPose();
//void position_setExpectedPose(Pose_t* newExpectedPose);
//const Pose_t* position_getCurrentPose();

Pose_t truePose;
void position_setAprilTagPose(const Pose_t* aprilTagPose);
const Pose_t* position_getAprilTagPose();

void position_setTruePoseToExpectedPose(Pose_t* truePose);

void position_setPoseDifference();
Pose_t position_getPoseDifference();

void position_init();


#endif /* POSITION_H_ */