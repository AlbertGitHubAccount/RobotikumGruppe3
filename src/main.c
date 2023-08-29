#include "robotControl.h"
#include "Bumper.h"
#include "Encoder.h"
#include "Position.h"
#include "path.h"
#include "OwnLaby.h"

#include <tools/labyrinth/labyrinth.h>
#include <stdbool.h>
#include <io/uart/uart.h>
#include <communication/communication.h>
#include <tools/timeTask/timeTask.h>
#include <tools/powerSaver.h>
#include <io/led/led.h>
#include <motor/motor.h>
#include <communication/packetTypes.h>
#include <tools/variablesAccess.h>
#include <io/adc/adc.h>
#include <pathFollower/pathFollower.h>
#include <avr/io.h>                 // AVR IO ports
#include <avr/interrupt.h>          // AVR Interrupts
#include <avr/pgmspace.h>           // AVR Program Space Utilities
#include <math.h>                   // Math functions and constants
#include <inttypes.h>               // Integer type conversions for printf, scanf and alike
#include <time.h>

/*
 *******************************************************************************
 * PRIVATE VARIABLES
 *******************************************************************************
 */
static bool explorerFlag = false;


/*
 *******************************************************************************
 * PRIVATE FUNCTIONS
 *******************************************************************************
 */


// callback function for communication channel CH_IN_DEBUG (Debug View in HWPCS)
static void commDebug(__attribute__((unused)) const uint8_t* packet, const uint16_t size) {
    communication_log(LEVEL_FINE, "received %" PRIu16 " bytes", size);
}


// callback function for communication channel CH_IN_USER_COMMAND (User Command View in HWPCS)
static void commUserCommand(const uint8_t* packet, __attribute__((unused)) const uint16_t size) {
    
    Path_t _path;
	const Pose_t* expectedPose;
	timeTask_time_t startTime;
	//timeTask_time_t stopTime;
	//timeTask_time_t currentTime;
	timeTask_getTimestamp(&startTime);

	UserCommand_t* cmd = (UserCommand_t*) packet;
    switch (cmd->id) {
    case 0: // command ID 0: stop motors
        setState(STOP);
        break;
	case 1: // command ID 1: turn on spot
		setState(TURN_AROUND);
		break;
    case 2: // command ID 2: Turn Left
		setState(TURN_LEFT);
        break;
    case 3: // command ID 3
		setState(TURN_RIGHT);
		break;
	case 4: 
		expectedPose = position_getExpectedPose();
		Point_t points[2];
		_path.points = points; 
		_path.pathLength = 2;
		_path.points[0].x = expectedPose->x;
		_path.points[0].y = expectedPose->y - LABY_CELLSIZE_2;
		_path.points[1].x = expectedPose->x;
		_path.points[1].y = expectedPose->y + LABY_CELLSIZE + LABY_CELLSIZE_2;
		pathFollower_setNewPath(&_path);
		pathFollower_command(FOLLOWER_CMD_START);
		break;
	case 5:
		setState(DRIVE_FORWARD);
		break;
	case 6:
		setState(EXPLORE);
		break;
	case 7:
		setState(DRIVE_BACKWARD);
		break;
	case 8: //CHANGE_EXPLORE_FLAG
		if (explorerFlag == false)
			explorerFlag = true;
		else
			explorerFlag = false;		
		break;
	}
}


void commSendRobotParams(const uint8_t* packet, __attribute__((unused)) const uint16_t size){
	RobotParameters_t* robotParams = (RobotParameters_t*)packet;
	position_setRobotParams(robotParams);
}
	
	//Callbackfuntion für Achsenweite und Distance per Tick
	
void commSetAprilTagPose(const uint8_t* packet, __attribute__((unused)) const uint16_t size){
	Pose_t* aprilTagPose = (Pose_t*)packet;
	position_setAprilTagPose(aprilTagPose);
}


// initialization
static void init(void) {
    powerSaver_init(); // must be the first call!
    LED_init();
    uart_init();
    communication_init();		

    // register communication callback functions which are executed by
    // communication_readPackets() in main loop when a packet is received from
    // HWPCS on the corresponding communication channel
	communication_setCallback(CH_IN_DEBUG, commDebug);
	communication_setCallback(CH_IN_USER_COMMAND, commUserCommand);
	communication_setCallback(CH_IN_ROBOT_PARAMS, commSendRobotParams);
	communication_setCallback(CH_IN_POSE, commSetAprilTagPose);
	
	//TODO:  communication_setCallback(i, cbf) für alles was wir implementieren

    Motor_init();
    timeTask_init();
	ADC_init(true);
	pathFollower_init();	
	
	//position_init();
	bumper_init();
	encoder_init();
	ownLaby_init();
	
    // global interrupt enable
    sei();
}




int main(void) {
    init();

    communication_log_P(LEVEL_INFO, PSTR("Booted"));

    // do forever
    for (;;) {

        // TODO: Daten einlesen
		// 
		bumper_checkCollision();					//Lies Bumperwerte und schick sie als Telemetriedaten zu HWPCS
		
		stateMachine();
		
		/*if(getState() == IDLE && i == 1) {
			stateMachine(DRIVE_FORWARD);	//Aufruf von Control System in Hauptschleife. If Abfrage, damit nur einmalig der Dri
			i++;
		}*/

        TIMETASK(LED_TASK, 500) { // execute block approximately every 500ms
            LED2_TOGGLE();
        }

        TIMETASK(TELEMETRY_TASK, 300) { // execute block approximately every 300ms
            // send telemetry data to HWPCS
            Telemetry_t telemetry;
            telemetry.bumpers = bumper_getBumpers(); // initialize with zero
            telemetry.contacts = bumper_getContacts();
            telemetry.encoder1 = encoder_getCounterR();
            telemetry.encoder2 = encoder_getCounterL();
            telemetry.infrared1 = ADC_getFilteredValue(0); //Front
            telemetry.infrared2 = ADC_getFilteredValue(1); //Right
            telemetry.infrared3 = ADC_getFilteredValue(2); //Left
            telemetry.infrared4 = 0;
            telemetry.infrared5 = explorerFlag; //zu wenige Telmetrie userdaten
            telemetry.user1 = position_getExpectedPose()->theta;
            telemetry.user2 = encoder_getStopCounter();
            communication_writePacket(CH_OUT_TELEMETRY, (uint8_t*)&telemetry, sizeof(telemetry));
        }
	
		TIMETASK(POSE_TASK, 150) { // execute block approximately every 150ms				alter Timetask der OHNE APRILTAG arbeitet
			Pose_t* truePose = position_getAprilTagPose();
			position_setExpectedPose(truePose);
			const Pose_t* expectedPose = position_getExpectedPose();						
			// send pose update to HWPCS
			communication_writePacket(CH_OUT_POSE, (uint8_t*)expectedPose, sizeof(*expectedPose));
		}
		
		TIMETASK(APRIL_TAG_TASK, 150){ //GetPose_t um Daten von MAIN_APRIL_TAG zu requesten
			GetPose_t aprilTag;
			aprilTag.aprilTagType = APRIL_TAG_MAIN;
			communication_writePacket(CH_OUT_GET_POSE, (uint8_t*)&aprilTag, sizeof(aprilTag));
		}
		
		TIMETASK(APRIL_POSE_TASK, 150) { // TimeTask der Pose von AprilTag nimmt
			Pose_t* truePose = position_getAprilTagPose();
			const LPose_t* labyPose = ownLaby_getPose();
			//Pose_t* expectedPose = position_getAprilTagPose();
			ownLaby_setPose(truePose);
			ownLaby_setRobotPose(labyPose);
			// send pose update to HWPCS
			communication_writePacket(CH_OUT_POSE, (uint8_t*)&truePose, sizeof(truePose));
		}
        
		 TIMETASK(FOLLOWER_TASK, 20) {	
			const PathFollowerStatus_t* pathFollower_status = pathFollower_getStatus();
			if (pathFollower_status->enabled) {
				const Pose_t* expectedPose = position_getExpectedPose();
				if (pathFollower_update(expectedPose))
					calculateDriveCommand(expectedPose, &pathFollower_status->lookahead);
				else{
					Motor_stopAll();
					pathFollower_command(FOLLOWER_CMD_RESET);	
				}
				communication_writePacket(CH_OUT_PATH_FOLLOW_STATUS, (uint8_t*)pathFollower_status, sizeof(*pathFollower_status)); // send pathFollower_status on channel CH_OUT_PATH_FOLLOW_STATUS
			}
		 }
		 
		 TIMETASK(WALL_TASK, 200) {
			 const LabyrinthWalls_t* wallData = labyrinth_getAllWalls();
			 communication_writePacket(CH_OUT_LABY_WALLS, (uint8_t*)wallData, sizeof(*wallData));
		 }
		 
		 TIMETASK(EXPLORE_TASK, 300) {
			 if ((getState() == IDLE) && (explorerFlag == true)){
				 setState(EXPLORE);
			 }
		 }
		 
        // poll receive buffer (read and parse all available packets from UART buffer)
        // and execute registered callback functions
        communication_readPackets();
    }

    return 0;
}