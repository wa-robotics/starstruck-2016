#pragma config(I2C_Usage, I2C1, i2cSensors)
#pragma config(Sensor, in1,    AutonSelector,  sensorPotentiometer)
#pragma config(Sensor, dgtl1,  DRFLED1,        sensorQuadEncoder)
#pragma config(Sensor, dgtl3,  DRFRED1,        sensorQuadEncoder)
#pragma config(Sensor, dgtl5,  DRBRED1,        sensorQuadEncoder)
#pragma config(Sensor, dgtl7,  DRBLED1,        sensorQuadEncoder)
#pragma config(Sensor, dgtl11, FBBotLim,       sensorTouch)
#pragma config(Sensor, dgtl12, FBTopLim,       sensorTouch)
#pragma config(Sensor, I2C_1,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Sensor, I2C_3,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Sensor, I2C_4,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Motor,  port2,           FR,            tmotorVex393TurboSpeed_MC29, openLoop, reversed)
#pragma config(Motor,  port4,           FL,            tmotorVex393TurboSpeed_MC29, openLoop)
#pragma config(Motor,  port5,           LFBB,          tmotorVex393_MC29, openLoop, driveLeft, encoderPort, I2C_4)
#pragma config(Motor,  port6,           RFBB,          tmotorVex393_MC29, openLoop, driveRight, encoderPort, I2C_3)
#pragma config(Motor,  port7,           RFBM,          tmotorVex393_MC29, openLoop, driveRight, encoderPort, I2C_1)
#pragma config(Motor,  port9,           BR,            tmotorVex393TurboSpeed_MC29, openLoop, reversed)
#pragma config(Motor,  port10,          BL,            tmotorVex393TurboSpeed_HBridge, openLoop)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

int threshold = 15; //minimum read value for the encoders to prevent stall

//Switch Statement Assignments
const int forward = 1;
const int rotateRight = 2
const int rotateLeft = 3;
const int backwards = 4;

float wheelCircumference = 3.25*PI; //stores value for the circumference of VEX 3.25" wheel
float revolutions = 0; //zeros current number of revolutions
float averageEncoderValue = 0; //
int buttonCtrlVal = 0; //this variable is going to be switched in the control loop
const float rotPulses = 0; //zeros pulses gl

//Zeros encoders - used whenever new driving function is called
void zeroEncoders(){
	SensorValue[DRFLED1] = 0;
	SensorValue[DRFRED1] = 0;
	SensorValue[DRBRED1] = 0;
	SensorValue[DRBLED1] = 0;
}

//Controls indv. motors in nice, clean function
void setDriveMotors(int FLSpeed, int FRspeed, int BRspeed, int BLspeed){ //FL, FR, BR, BL
	motor[FL] = FLspeed;
	motor[FR] = FRspeed;
	motor[BR] = BRspeed;
	motor[BL] = BLspeed;
}

//When called, gets the average of the absolute value of all four drivetrain shaft encoders
float getDTEncoderAverage(){
	averageEncoderValue = (abs(SensorValue(DRFLED1)) + abs(SensorValue(DRFRED1)) + abs(SensorValue(DRBRED1)) + abs(SensorValue(DRBLED1)))/4;
	return averageEncoderValue;
}

//Movement Function 1-forward 2-strafeRight 3-strafeLeft 4-backwards
void move(int direction, int speed, int distance){
	revolutions = distance/wheelCircumference;
	switch (direction) {

	case forward:
		zeroEncoders();
		while(getDTEncoderAverage() < distance){
			setDriveMotors(127,127,127, 127); //FL, FR, BR, BL
		}
		break;

		case rotateLeft:
		zeroEncoders();
		while(getDTEncoderAverage() < distance){
		setDriveMotors(-127, 127, 127, -127); //FL, FR, BR, BL
		}
		break;

		case rotateRight:
		zeroEncoders();
		while(getDTEncoderAverage() < distance){
		setDriveMotors(127, -127, -127, 127); //FL, FR, BR, BL
		}
		break;

	case backwards:
		zeroEncoders();
		while(getDTEncoderAverage() < distance){
			setDriveMotors(-127, -127, -127, -127); //FL, FR, BR, BL
		}
		break;

	default:
		setDriveMotors(0,0,0,0);

	}
}
task main()
{
	while(true){
		//Drivetrain controls and threshold (15)
		motor[FL] = (abs(vexRT[Ch3]) > threshold) ? vexRT[Ch3] : 0;
		motor[BL] = (abs(vexRT[Ch3]) > threshold) ? vexRT[Ch3] : 0;
		motor[FR] = (abs(vexRT[Ch2]) > threshold) ? vexRT[Ch2] : 0;
		motor[BR] = (abs(vexRT[Ch2]) > threshold) ? vexRT[Ch2] : 0;

		/*Auton Straightening Debug Test Controls
		All center around 8x buttons
		8U - forward 6 ft(3 tiles)
		8D - backard 6 ft(3 tiles)
		8L - rotate left one full rotation
		8R - rotate right one full rotation
		7R - stops commands for safety reasons (in case it stalls, runs into walls, etc)
		*/
		/*const int debugDistance = 72; //inches
		revolutions = (debugDistance / wheelCircumference)*90; //90 pulses  = 1 revolution inside a shaft encoder
		if(Btn8U){
			while(getDTEncoderAverage() < revolutions){
				setDriveMotors(127, 127, 127, 127);//(int FLSpeed, int FRspeed, int BRspeed, int BLspeed)
				if(Btn7R){break;}
			}
		}
		else if(Btn8D){
			while(getDTEncoderAverage() < revolutions){
				setDriveMotors(-127, -127, -127, -127);////(int FLSpeed, int FRspeed, int BRspeed, int BLspeed)
				if(Btn7R){break;}
			}
		}
		/*else if(Btn8L){
		while(getDTEncoderAverage() < revolutions){
		setDriveMotors
		if(Btn7R){break;}
		}
		}
		else if(Btn8R){
		while(getDTEncoderAverage() < revolutions){

		if(Btn7R){break;}
		}
		}*/
		/*------------------------------------------------------------------------

													Autonomous Play #1 - Fence

		------------------------------------------------------------------------*/


		/*------------------------------------------------------------------------

								Autonomous Play #2 - Scoring Stars in the Far Zone

		------------------------------------------------------------------------*/


		/*------------------------------------------------------------------------

					  Autonomous Play #3 - Scoring Stars and Cubes in the Far Zone

		 Play-By-Play
		1. Shimmy and release the platform
		2. Turn 90 deg. counter-clockwise and pick up three stars in our team's far zone (24")
		3. Lift up platform 30 deg. to secure the stars
		4. Drive backwards 1 tile to be back in the starting tile
		5. Turn 90 deg. clockwise
		6. Drive forward 12" (half a tile) and turn 180 deg. counterclockwise
		7. Driveforward 1.5 tiles (36") and launch stars into far zone of opponent
		8. Lower platform 90 deg. so it rests on the ground
		9. Rotate robot 45 deg clockwise and pick up the cube
		10. Raise platform 30 deg to secure cube
		11. Rotate 45 deg. counter-clockwise and score cube

		------------------------------------------------------------------------*/

	}
}
