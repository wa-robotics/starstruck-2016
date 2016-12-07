//This file is included above task main() in Catapult with Drum Test.c

float positionKp = 0.36, //proportional constant for positional PID
			straighteningKpLeft = 0.48,//.43,//.195, //proportional constant for straightening response for the left side
			straighteningKpRight = 0.55,//.22,//.16, //proportional constant for straightening response for the right side
			straighteningKpLeftTurn = 0.4,//.43,//.195, //proportional constant for straightening response for the left side when turning
			straighteningKpRightTurn = 0.4,//.22,//.16, //proportional constant for straightening response for the right side when turning
			positionKi = 0.000350, //integral constant
			positionKd = 4; //derivative constant

static int STRAIGHT = 2; //the 2 here shouldn't matter as long as no variables are multiplied by 'direction' in driveDistancePID
static int ROTATE_LEFT = -1;
static int ROTATE_RIGHT = 1;

//drivetrain movement functions

//positive powers will go forward or right
//negative powers values go backwards or left
//encoder counts is how many counts to go, always positive
//power is the power to run the motors at before straightening is applied
//time is a maximum time to complete the operation
void driveDistance(int power, int encoderCounts, int direction) {
	//writeDebugStreamLine("nPgmTime,error,nMotorEncoder[lDriveFront], nMotorEncoder[rDriveFront],pTerm,iTerm,dTerm,lPower,rPower");
	//reset encoder values
	SensorValue[lDriveEnc] = 0;
	SensorValue[rDriveEnc] = 0;

	int straighteningError = 0;

	float lPower,
	rPower;

	time1[T1] = 0;
	if (direction == STRAIGHT) { //validate direction
			//limit the values of the power term to only be those that can be taken by the motors
			if (power > 127) {
				power = 127;
			} else if (power < -127) {
				power = -127;
			}


				while(encoderCounts > abs(SensorValue[lDriveEnc] + SensorValue[rDriveEnc])/2.0) {
					//adjust the powers sent to each side if the encoder values don't match
						straighteningError = SensorValue[lDriveEnc] - SensorValue[rDriveEnc];


						if (straighteningError > 0) { //left side is ahead, so slow it down
							lPower = power - straighteningError*straighteningKpLeft;
						} else { //otherwise, just set the right side to the power
							lPower = power;
						}
						if (straighteningError < 0) { //right side is ahead, so slow it down
							rPower = power + straighteningError*straighteningKpRight;
						} else { //otherwise, just set the right side to the power
							rPower = power;
						}
						writeDebugStreamLine("lPower = %d, rPower = %d",lPower,rPower);

					setLeftDtMotors(lPower);
					setRightDtMotors(rPower);
					wait1Msec(25);
			}

			//writeDebugStreamLine("%d,%f,%f,%f,%f,%f,%f,%f,%f",nPgmTime,target,error,nMotorEncoder[lDriveFront], nMotorEncoder[rDriveFront],pTerm,iTerm,dTerm,lPower,rPower);
			setLeftDtMotors(0);
			setRightDtMotors(0);

	} else if (direction == ROTATE_LEFT || direction == ROTATE_RIGHT) {
			//update error terms

			//limit the values of the power term to only be those that can be taken by the motors
			if (power > 127) {
				power = 127;
			} else if (power < -127) {
				power = -127;
			}


			//apply a slew rate to limit acceleration/deceleration


			//adjust the powers sent to each side if the encoder values don't match
			straighteningError = abs(nMotorEncoder[lDriveFront]) - abs(nMotorEncoder[rDriveFront]);

			if (straighteningError > 0) { //left side is ahead, so speed up the right side
				rPower = power + straighteningError*straighteningKpLeftTurn;
			} else { //otherwise, just set the right side to the power
				rPower = power;
			}
			if (straighteningError < 0) { //right side is ahead, so speed up the left side
				lPower = power - straighteningError*straighteningKpRightTurn;
			} else { //otherwise, just set the right side to the power
				lPower = power;
			}

			//lastPower = power; //update the last power
			//writeDebugStreamLine("%d,%f,%f,%f,%f,%f,%f,%f,%f",nPgmTime,error,nMotorEncoder[lDriveFront], nMotorEncoder[rDriveFront],pTerm,iTerm,dTerm,lPower,rPower);
			setLeftDtMotors(lPower * direction); //for a left turn, ROTATE_LEFT = -1 so this moves the left side backwards for a left turn. For a right turn will go forward since ROTATE_RIGHT = 1
			setRightDtMotors(rPower * -1 * direction); //same idea as for a left turn, except this side needs to go the opposite way as the left side in order to turn, hence the * -1 in the calculation
			wait1Msec(25);
		}
	}


//rotate the robot a certain amount
//@param deg The number of degrees to turn; positive values are counterclockwise, negative values are clockwise.
//@param direction The direction to turn in to get to the position; 1 is counterclockwise, -1 is clockwise
/*void rotateDegrees(int position, int direction) {//This function is for turning
	SensorValue[gyro] = 0;
	//Clear gyro
	if(direction == 1){
		//If direction == Left
		while(abs(SensorValue[gyro]) < position){
			//While the gyro is less than a set degrees, turn Left
			setRightDtMotors(45,45);
			setLeftDtMotors(-45,-45);
		}
		setRightDtMotors(-30,-30);
		setLeftDtMotors(30,30);
		wait1Msec(100); //brief brake
	}
	//end of LEFT turn
	else{
		//if direction == right
		while(abs(SensorValue[gyro]) < position){
			//While the gyro is less than a set degrees, turn right
			setRightDtMotors(-45,-45);
			setLeftDtMotors(45,45);
		}

		setRightDtMotors(30,30);
		setLeftDtMotors(-30,-30);
		wait1Msec(100); //brief brake
	} //end of RIGHT turn
	setRightDtMotors(0,0);
	setLeftDtMotors(0,0);

}
*/
task autonSkills()
{


}

task autonKnockStarsRight() //go to the fence and knock 3 stars off
{
	SensorValue[hangLock] = 0;
	//sensorValue[platformLock] = 1;
	//setDumpMotors(40)
	//wait10Msec(5);
	//setDumpMotors(0);
	SensorValue[rDriveEnc] = 0;
	wait10Msec(50);
	while(SensorValue[rDriveEnc] > -2100)
	{
		setRightDtMotors(-75);
		setLeftDtMotors(-75);
	}
	//setDumpMotors(-50);
	//wait10Msec(18);
	//setDumpMotors(0);
	while(SensorValue[rDriveEnc] > -2775)
	{
		setLeftDtMotors(-124.5);
		setRightDtMotors(-26.5);
	}
	setLeftDtMotors(0);
	setRightDtMotors(0);
}

task autonStarsCubeRight() { //knock the stars off the fence (play 1 code same as autonKnockStarsRight), then pick up cube and score it
	SensorValue[hangLock] = 0;
	//sensorValue[platformLock] = 1;
	//setDumpMotors(40)
	//wait10Msec(5);
	//setDumpMotors(0);
	SensorValue[rDriveEnc] = 0;
	wait10Msec(50);
	while(SensorValue[rDriveEnc] > -2100)
	{
		setRightDtMotors(-75);
		setLeftDtMotors(-75);
	}
	//setDumpMotors(-50);
	//wait10Msec(18);
	//setDumpMotors(0);
	while(SensorValue[rDriveEnc] > -2775)
	{
		setLeftDtMotors(-124.5);
		setRightDtMotors(-26.5);
	}
	setLeftDtMotors(0);
	setRightDtMotors(0);
	//start play 2
	wait10Msec(50);
	while(SensorValue[rDriveEnc] < -2275)
	{
		setLeftDtMotors(85);
		setRightDtMotors(85);
	}
	setLeftDtMotors(0);
	setRightDtMotors(0);
	wait10Msec(50);
	SensorValue[rDriveEnc] = 0;
	while(SensorValue[rDriveEnc] > -350)
	{
		setLeftDtMotors(85);
		setRightDtMotors(-85);
	}
	setLeftDtMotors(0);
	setRightDtMotors(0);
	wait10Msec(50);
	SensorValue[rDriveEnc] = 0;
	while(SensorValue[rDriveEnc] < 1300)
	{
		setLeftDtMotors(85);
		setRightDtMotors(85);
	}
	setLeftDtMotors(0);
	setRightDtMotors(0);
	SensorValue[platformLock] = 1;
	wait10Msec(34);
	setDumpMotors(18);
	wait10Msec(25);
	setDumpMotors(-12);
	SensorValue[rDriveEnc] = 0;
	wait10Msec(50);
	while(SensorValue[rDriveEnc] < 600)
	{
		setLeftDtMotors(-60);
		setRightDtMotors(60);
	}
	setLeftDtMotors(0);
	setRightDtMotors(0);
	wait10Msec(50);
	SensorValue[rDriveEnc] = 0;
	while(SensorValue[rDriveEnc] > -242)
	{
		setLeftDtMotors(-60);
		setRightDtMotors(-60);
	}
	setLeftDtMotors(0);
	setRightDtMotors(0);
	setDumpMotors(100);
	wait10Msec(50);
	setDumpMotors(0);
}
