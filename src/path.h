/*
 * path.h
 *
 * Created: 13.07.2023 17:21:06
 *  Author: malus
 */ 


#ifndef PATH_H_
#define PATH_H_

#include <communication/packetTypes.h>

void calculateDriveCommand(const Pose_t* pose, const FPoint_t* lookahead);
float calcDifAngle(const Pose_t* pose, const FPoint_t* lookahead);


#endif /* PATH_H_ */