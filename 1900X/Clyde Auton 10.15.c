//This file is included above task main() in Catapult with Drum Test.c

float positionKp = 0.36, //proportional constant for positional PID
			straighteningKpLeft = 0.48,//.43,//.195, //proportional constant for straightening response for the left side
			straighteningKpRight = 0.55,//.22,//.16, //proportional constant for straightening response for the right side
			straighteningKpLeftTurn = 0.4,//.43,//.195, //proportional constant for straightening response for the left side when turning
			straighteningKpRightTurn = 0.4,//.22,//.16, //proportional constant for straightening response for the right side when turning
			positionKi = 0.000350, //integral constant
			positionKd = 4; //derivative constant

static int STRAIGHT = 2; //the 2 here shouldn't matter as long as no variables are multiplied by 'direction' in driveDistancePID
static int STRAFE_LEFT = 3; //don't multiply values by this variable!
static int STRAFE_RIGHT = 4;
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
	nMotorEncoder[rDriveFront] = 0;
	nMotorEncoder[lDriveFront] = 0;

	int straighteningError = 0;

	float lPower,
	rPower;

	time1[T1] = 0;
	if (direction == STRAIGHT || direction == STRAFE_LEFT || direction == STRAFE_RIGHT) { //validate direction
			//limit the values of the power term to only be those that can be taken by the motors
			if (power > 127) {
				power = 127;
			} else if (power < -127) {
				power = -127;
			}

			if (direction == STRAIGHT) {
				while(encoderCounts > abs(nMotorEncoder[lDriveFront] + nMotorEncoder[rDriveFront])/2.0) {
					//adjust the powers sent to each side if the encoder values don't match
						straighteningError = nMotorEncoder[lDriveFront] - nMotorEncoder[rDriveFront];


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

					setLeftDtMotors(lPower,lPower);
					setRightDtMotors(rPower,rPower);
					wait1Msec(25);
				}

				setLeftDtMotors(0,0);
				setRightDtMotors(0,0);

			} else if (direction == STRAFE_LEFT) {

					while(encoderCounts > (abs(nMotorEncoder[lDriveFront]) + abs(nMotorEncoder[rDriveFront]))/2.0) {
							//adjust the powers sent to each side if the encoder values don't match
						straighteningError = abs(nMotorEncoder[lDriveFront]) - abs(nMotorEncoder[rDriveFront]);

						if (straighteningError > 0) { //left side is ahead, so slow it down
							lPower = power - straighteningError*straighteningKpLeft*-1;
						} else { //otherwise, just set the right side to the power
							lPower = power;
						}
						if (straighteningError < 0) { //right side is ahead, so slow it down
							rPower = power + straighteningError*straighteningKpRight*-1;
						} else { //otherwise, just set the right side to the power
							rPower = power;
						}

					setLeftDtMotors(lPower*-1,lPower*1);
					setRightDtMotors(rPower*1,rPower*-1);
					wait1Msec(25);
				}

			//writeDebugStreamLine("%d,%f,%f,%f,%f,%f,%f,%f,%f",nPgmTime,target,error,nMotorEncoder[lDriveFront], nMotorEncoder[rDriveFront],pTerm,iTerm,dTerm,lPower,rPower);
			setLeftDtMotors(0,0);
			setRightDtMotors(0,0);

	} else if (direction == STRAFE_RIGHT) {

					while(encoderCounts > (abs(nMotorEncoder[lDriveFront]) + abs(nMotorEncoder[rDriveFront]))/2.0) {
							//adjust the powers sent to each side if the encoder values don't match
						straighteningError = abs(nMotorEncoder[lDriveFront]) - abs(nMotorEncoder[rDriveFront]);

						//writeDebugStreamLine("%d,%d,%d,%d",lPower*lfMult,lPower*lbMult,rPower*rfMult,rPower*rbMult);
						//positive powers:
						//  if left side is ahead, straightening error is positive
						// 	lPower will decrease
						//  if right side is ahead, straightening error is negative
						//  rPower will decrease
						//negative powers:
						//  if left side is ahead, straightening error is positive
						//  power is negative, straighteningError is positive
						//  power would go higher (more negative) because power - (positive error)
						//  fix: multiply by sign of power - after this change
						//  power is positive, straighteningError is still positive (as we want)
						//  power is negative, straighteningError is multiplied by -1, so:
						//  (-127) - (12)*(.3)*-1 = -127 + 4 = -123, slower power
						//
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

					setLeftDtMotors(lPower*1,lPower*-1);
					setRightDtMotors(rPower*-1,rPower*1);
					wait1Msec(25);
				}


			//writeDebugStreamLine("%d,%f,%f,%f,%f,%f,%f,%f,%f",nPgmTime,target,error,nMotorEncoder[lDriveFront], nMotorEncoder[rDriveFront],pTerm,iTerm,dTerm,lPower,rPower);
			setLeftDtMotors(0,0);
			setRightDtMotors(0,0);

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
			setLeftDtMotors(lPower * direction, lPower * direction); //for a left turn, ROTATE_LEFT = -1 so this moves the left side backwards for a left turn. For a right turn will go forward since ROTATE_RIGHT = 1
			setRightDtMotors(rPower * -1 * direction,rPower * -1 * direction); //same idea as for a left turn, except this side needs to go the opposite way as the left side in order to turn, hence the * -1 in the calculation
			wait1Msec(25);
		}
	}
}

//rotate the robot a certain amount
//@param deg The number of degrees to turn; positive values are counterclockwise, negative values are clockwise.
//@param direction The direction to turn in to get to the position; 1 is counterclockwise, -1 is clockwise
void rotateDegrees(int position, int direction) {//This function is for turning
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

void strafeNoStraightening(int frontPower, int backPower, int encoderCounts, int direction) {
	nMotorEncoder[lDriveFront] = 0;
	nMotorEncoder[rDriveFront] = 0;
	if (direction == STRAFE_LEFT) {
				while(encoderCounts > (abs(nMotorEncoder[lDriveFront]) + abs(nMotorEncoder[rDriveFront]))/2.0) {
					setLeftDtMotors(frontPower*-1,backPower*1);
					setRightDtMotors(frontPower*1,backPower*-1);
					wait1Msec(25);
				}

			//writeDebugStreamLine("%d,%f,%f,%f,%f,%f,%f,%f,%f",nPgmTime,target,error,nMotorEncoder[lDriveFront], nMotorEncoder[rDriveFront],pTerm,iTerm,dTerm,lPower,rPower);
			setLeftDtMotors(0,0);
			setRightDtMotors(0,0);

	} else if (direction == STRAFE_RIGHT) {

		while(encoderCounts > (abs(nMotorEncoder[lDriveFront]) + abs(nMotorEncoder[rDriveFront]))/2.0) {
			//adjust the powers sent to each side if the encoder values don't match
			setLeftDtMotors(frontPower*1,backPower*-1);
			setRightDtMotors(frontPower*-1,backPower*1);
			wait1Msec(25);
		}


			//writeDebugStreamLine("%d,%f,%f,%f,%f,%f,%f,%f,%f",nPgmTime,target,error,nMotorEncoder[lDriveFront], nMotorEncoder[rDriveFront],pTerm,iTerm,dTerm,lPower,rPower);
			setLeftDtMotors(0,0);
			setRightDtMotors(0,0);

	}
}

task auton()
{
	//resetDrumPosition();
	//wait10Msec(10000);
	writeDebugStreamLine("ran");
	SensorValue[tongue] = 1;
	SensorValue[platformLock] = 1;
	driveDistance(127,500,STRAIGHT); //500 drop the platform
	driveDistance(-127,190,STRAIGHT);
	wait10Msec(50);
	setCatapultPosition(1);
	wait10Msec(50);
	driveDistance(100,1500,STRAIGHT);
	driveDistance(-127,300,STRAIGHT);
	driveDistance(100,1000,STRAFE_LEFT);
	driveDistance(127,300,STRAIGHT);
	driveDistance(100,1500,STRAFE_LEFT);
	//actual autonomous
	//driveDistance(100,340,STRAIGHT); //push the preload forward
	//wait1Msec(500);
	//driveDistance(-100,500,STRAIGHT); //500 drop the platform
	//driveDistance(127,190,STRAIGHT);
	//driveDistance(-127,190,STRAIGHT);
	//wait1Msec(500);
	//driveDistance(100,1500,STRAIGHT); //collect the 4 stars
	//wait1Msec(500);
	//strafeNoStraightening(55,127,550,STRAFE_LEFT);
	//driveDistance(100,80,STRAIGHT);

	wait10Msec(50);
	fireCatapult();
}
