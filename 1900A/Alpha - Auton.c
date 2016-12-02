#pragma config(I2C_Usage, I2C1, i2cSensors)
#pragma config(Sensor, in1,    AutonSelector,  sensorPotentiometer)
#pragma config(Sensor, dgtl1,  DRFLED1,        sensorQuadEncoder)
#pragma config(Sensor, dgtl3,  DRFRED1,        sensorQuadEncoder)
#pragma config(Sensor, dgtl5,  DRBRED1,        sensorQuadEncoder)
#pragma config(Sensor, dgtl7,  DRBLED1,        sensorQuadEncoder)
#pragma config(Sensor, I2C_1,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Sensor, I2C_2,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Motor,  port1,           LLM,           tmotorVex393HighSpeed_HBridge, openLoop, reversed)
#pragma config(Motor,  port2,           FR,            tmotorVex393TurboSpeed_MC29, openLoop, reversed)
#pragma config(Motor,  port3,           FL,            tmotorVex393TurboSpeed_MC29, openLoop, reversed)
#pragma config(Motor,  port4,           LFBB,          tmotorVex393HighSpeed_MC29, openLoop, reversed)
#pragma config(Motor,  port5,           LFBT,          tmotorVex393HighSpeed_MC29, openLoop, reversed, encoderPort, I2C_1)
#pragma config(Motor,  port6,           RFBB,          tmotorVex393HighSpeed_MC29, openLoop)
#pragma config(Motor,  port7,           RFBT,          tmotorVex393HighSpeed_MC29, openLoop, encoderPort, I2C_2)
#pragma config(Motor,  port8,           RLM,           tmotorVex393HighSpeed_MC29, openLoop)
#pragma config(Motor,  port9,           BR,            tmotorVex393TurboSpeed_MC29, openLoop, reversed)
#pragma config(Motor,  port10,          BL,            tmotorVex393TurboSpeed_HBridge, openLoop)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

//Switch Cases
const int forward = 1;
const int turnRight = 2;
const int turnLeft = 3;
const int backwards = 4;
const int up = 5;
const int down= 6;

//Encoder Variables
float wheelCircumference = 3.25*PI;
float revolutions = 0;
float averageEncoderValue = 0;
int speedMCounts = 392; //number of counts per rotation for VEX High Speed Motor

void zeroEncoders(){ //Zeros Encoders
	SensorValue[DRFLED1] = 0;
	SensorValue[DRFRED1] = 0;
	SensorValue[DRBRED1] = 0;
	SensorValue[DRBLED1] = 0;
}
void setFBMotors(int speed){ //automatically sets the four bar power (reduce clutter)
	motor[RFBT] = speed;
	motor[RFBB] = speed;
	motor[LFBT] = speed;
	motor[LFBB] = speed;

}

void setDriveMotors(int FLSpeed, int FRspeed, int BRspeed, int BLspeed){ //int FLSpeed, int FRspeed, int BRspeed, int BLspeed
	motor[FL] = FLspeed;
	motor[FR] = FRspeed;
	motor[BR] = BRspeed;
	motor[BL] = BLspeed;
}

void Break(int direction){
	switch (direction) {
	case forward:
		setDriveMotors(-127,-127,-127,-127);
		wait10Msec(7);
		break;

	case turnLeft:
		setDriveMotors(127, -127, -127, 127);
		wait10Msec(7);
		break;

	case turnRight:
		setDriveMotors(-127, 127, 127, -127);
		wait10Msec(7);
		break;

	case backwards:
		setDriveMotors(127,127,127,127);
		wait10Msec(7);
		break;

	default:
		setDriveMotors(0,0,0,0);
	}
}
float getDTEncoderAverage(){
	averageEncoderValue = (abs(SensorValue(DRFLED1)) + abs(SensorValue(DRFRED1)) + abs(SensorValue(DRBRED1)) + abs(SensorValue(DRBLED1)))/4;
	return averageEncoderValue;
}
float getFBEncoderAverage(){

}
void moveFB(int direction, int degrees){
	int pinionTeeth = 12;
	int largeGearTeeth = 84;
	int percentGearTurned = degrees/360;
	int lGearTeethUsed = largeGearTeeth*percentGearTurned;
	int pinionRots = lGearTeethUsed/pinionTeeth;
	float ticks = pinionRots*speedMCounts;
	if(direction == up){
		while(getFBEncoderAverage() < ticks){
			setFBMotors(127);
		}
		if(direction == down){
			while(getFBEncoderAverage() < ticks){
				setFBMotors(-127);
			}
		}
		else {
			setFBMotors(0);
		}

	}
}
void move(int direction, int speed, int distance){
	revolutions = 360*(distance/wheelCircumference);
	switch (direction) {

	case forward:
		zeroEncoders();
		while(getDTEncoderAverage() < revolutions){
			setDriveMotors(127,127,127, 127);
		}
		zeroEncoders();
		Break(forward);
		break;

	case turnLeft: //int FLSpeed, int FRspeed, int BRspeed, int BLspeed
		zeroEncoders();
		while(getDTEncoderAverage() < revolutions){
			setDriveMotors(-127, 127, 127, -127);

		}
		Break(turnLeft);
		break;

	case turnRight: //int FLSpeed, int FRspeed, int BRspeed, int BLspeed
		zeroEncoders();
		while(getDTEncoderAverage() < revolutions){
			setDriveMotors(127, -127, -127, 127);
		}
		Break(turnRight);
		break;

	case backwards:
		zeroEncoders();
		while(getDTEncoderAverage() < revolutions){
			setDriveMotors(-127, -127, -127, -127);

		}
		Break(backwards);
		break;

	default:
		setDriveMotors(0,0,0,0);
		zeroEncoders();

	}
	zeroEncoders();
}
void encoderTest(int ticks){
	zeroEncoders();
	while(getDTEncoderAverage() < ticks){
		setDriveMotors(127,127,127, 127);
	}
}
void doTheShimmy(){ //release the platform on plays that require that

	int shimmyDistance = 1.5;
	move(forward, 127, shimmyDistance);
	Break(forward);
	move(backwards, 127, shimmyDistance);
	Break(backwards);
	move(forward, 127, shimmyDistance);
	Break(forward);
	move(backwards, 127, shimmyDistance);
	Break(backwards);
	move(forward, 127, shimmyDistance);
	Break(forward);
	move(backwards, 127, shimmyDistance);
	Break(backwards);
	move(forward, 127, shimmyDistance);
	Break(forward);
	move(backwards, 127, shimmyDistance);
	Break(backwards);

}
//Auton Selector Wizard
int getPTVal(){
	int sectorRange = 260/7; //Sector 1 would range between 0 and 37.14 degrees
	float PTVal = SensorValue(AutonSelector);
	float sectorID = PTVal / sectorRange;

	if (sectorID >= 0 && sectorID <1){ //Left Fence Play
		sectorID = 0;
	}
	else if (sectorID >= 1 && sectorID <2){//Right Fence Play
		sectorID =  1;
	}
	else if (sectorID >= 2 && sectorID <3){//Left Star Play
		sectorID =  2;
	}
	else if (sectorID >= 3 && sectorID <4){//Right Star Play
		sectorID =  3;
	}
	else if (sectorID >= 4 && sectorID <5){//Left Cube Play
		sectorID =  4;
	}
	else if (sectorID >= 5 && sectorID <6){//Right Cube Play
		sectorID =  5;
	}
	else if (sectorID >= 6 && sectorID <7){ //skills play
		sectorID =  6;
	}
	return sectorID;
}
/*------------------------------------------------------------------------

Autonomous Play #1 - Fence

Play-by-Play

Pre-Match
1. Place robot FORWARD FACING
2. Set dial to LFen or RFen to run playImmediateTone

(A) Left Field

1. Rotate four-bar 80 degrees up (positive speed value)
2. Move forward 40 inches (give or take) which knocks off two stars (+2 points)
3. Move backwards 3.25 inches (1 wheel rotation)
4. Rotate 90 degrees clockwise
5. Move forward 15 inches and knock off 1-2 more stars
6. Stop

(B) Right Field

1. Rotate four-bar 80 degrees up (positive speed value)
2. Move forward 40 inches (give or take) which knocks off two stars (+2 points)
3. Move backwards 3.25 inches (1 wheel rotation)
4. Rotate 90 degrees counterclockwise
5. Move forward 15 inches and knock off 1-2 more stars
6. Stop

Total Points: 2
------------------------------------------------------------------------*/


/*------------------------------------------------------------------------

Autonomous Play #2 - Scoring Stars in the Far Zone

Pre-Match
1. Place robot facing right (left field) or left (right field)
2. Set dial to LStar or RStar to run playImmediateTone

(A) Left Field

1. Move backwards 12 inches
2. Do the shimmy to lower platform
3. Move forward 36 inches
4. Lift platform 30 degree or until stars clear the fence (+6 pts)
5. Move backward 36" (to clear cube)
6. Rotate 90 degrees clockwise
7. Move backwards roughly 38 inches
9. Lower platform 30 degrees then rotate 90 deg up to score in far zone (+6 pts)

(B) Right Field

1. Move backwards 12 inches
2. Do the shimmy to lower platform
3. Move forward 36 inches
4. Lift platform 30 degree or until stars clear the fence (+6 pts)
5. Move backward 36" (to clear cube)
6. Rotate 90 degrees counterclockwise
7. Move backwards roughly 38 inches
9. Lower platform 30 degrees then rotate 90 deg up to score in far zone (+6 pts)

Total Points: 12
------------------------------------------------------------------------*/


/*------------------------------------------------------------------------

Autonomous Play #3 - Scoring Stars and Cubes in the Far Zone

Pre-Match
1. Place robot facing right (left field) or left (right field)
2. Set dial to LCube or RCube to run playImmediateTone

(A) Left Field

1. Move backwards 12 inches
2. Do the shimmy to lower platform
3. Move forward 36 inches
4. Lift platform 30 degree or until stars clear the fence (+6 pts)
5. Move backward 36" (to clear cube)
6. Rotate 90 degrees clockwise
7. Move backwards roughly 38 inches
9. Lower platform 30 degrees then rotate 90 deg up to score in far zone (+6 pts)
10. Lower platform 60 degrees
11. Rotate 45 degrees clockwise
12. Move forward 30" or until in line with the cube
12. Rotate 135 degree counter-clockwise
13. Lower platform 30 degrees, or until resting on ground
14. Move forward to pick up cube
15. Move platform up 20 degrees, slowly (+2 pts)
16. Rotate 90 degrees clockwise, slowly
17. Move forward 25" or so
18. Lower platform 20 degrees, then rotate up quickly 90 degrees to score (+4 pts)

(B) Right Field

1. Move backwards 12 inches
2. Do the shimmy to lower platform
3. Move forward 36 inches
4. Lift platform 30 degree or until stars clear the fence (+6 pts)
5. Move backward 36" (to clear cube)
6. Rotate 90 degrees counterclockwise
7. Move backwards roughly 38 inches
9. Lower platform 30 degrees then rotate 90 deg up to score in far zone (+6 pts)
10. Lower platform 60 degrees
11. Rotate 45 degrees counter-clockwise
12. Move forward 30" or until in line with the cube
12. Rotate 135 degree clockwise
13. Lower platform 30 degrees, or until resting on ground
14. Move forward to pick up cube
15. Move platform up 20 degrees, slowly (+2 pts)
16. Rotate 90 degrees counter-clockwise, slowly
17. Move forward 25" or so
18. Lower platform 20 degrees, then rotate up quickly 90 degrees to score (+4 pts)

Total Points: 18
------------------------------------------------------------------------*/

task main()
{
	/*Auton Switching Wizard
	switch getPTVal(){
	case 0:

	break;
	case 1:

	break;
	case 2:

	break;
	case 3:

	break;
	case 4:

	break;
	case 5:

	break;
	case 6:

	break;

	default:
	setDriveMotors(0,0,0,0);
	}

	*/
	int wt =775;
	int fbs = 40;
	float tsp = 80;
	/*
	//LFence Play

	//Raise Four-Bar (Uses time at the moment)
	setFBMotors(fbs);
	wait1Msec(wt);
	setFBMotors(0);

	move(forward, 127, 55); //Move forward to knock off stars
	move(backwards, 127, 25); //Move back to give room to turn
	move(turnLeft, tsp, 10.5); //rotate 90 deg to the left
	move(forward, 127, 13); //Move forward to align with the third star
	move(turnRight, tsp, 10.5); //Rotate 90 deg to the right
	move(forward, 127, 2); //Move forward to knock of third star

	//RFence
	/*
	//Raise Four-Bar (Uses time at the moment)
	setFBMotors(fbs);
	wait1Msec(wt);
	setFBMotors(0);

	move(forward, 127, 55); //Move forward to knock off stars
	move(backwards, 127, 25); //Move back to give room to turn
	move(turnRight, tsp, 10.5); //rotate 90 deg to the left
	move(forward, 127, 13); //Move forward to align with the third star
	move(turnLeft, tsp, 10.5); //Rotate 90 deg to the right
	move(forward, 127, 2); //Move forward to knock of third star

	*/


	/* Work in Progress - Gets a fourth start. This one is harder because it tends to get off course a little
	move(backwards, 127, 25);
	setDriveMotors(0,0,0,0);
	wait10Msec(2);
	move(turnRight, tsp, 10.5);
	setFBMotors(fbs);
	wait1Msec(150);
	setFBMotors(0);
	move(forward, 127, 60);
	move(turnLeft, tsp, 10.5)
	*/

	//Left Square Star Shooting
	move(backwards, 50, 20); //move back to allow room to release platform
	dotheShimmy(); //release the platform
	move(forward, 75, 48); //move to pick up stars
	//Secure the stars by lifting up the platform
	setFBMotors(fbs); //
	wait1Msec(300);
	setFBMotors(0);
	//Get in position to launch the stars
	move(turnRight, 40, 10.5); //Rotate 90 degrees clockwise
	move(backwards, 50, 48); //Drive backwards toward the fence
	//Lower the platform
	setFBMotors(-fbs);
	wait1Msec(300);
	setFBMotors(0);
	//Launch the stars
	setFBMotors(127);
	wait1Msec(900);
	setFBMotors(0);





}
