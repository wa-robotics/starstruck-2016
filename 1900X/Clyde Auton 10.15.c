//This file is included above task main() in Catapult with Drum Test.c

float positionKp = 0.36, //proportional constant for positional PID
			straighteningKpLeft = 0.4,//.43,//.195, //proportional constant for straightening response for the left side
			straighteningKpRight = 0.4,//.22,//.16, //proportional constant for straightening response for the right side
			straighteningKpLeftTurn = 0.4,//.43,//.195, //proportional constant for straightening response for the left side when turning
			straighteningKpRightTurn = 0.4,//.22,//.16, //proportional constant for straightening response for the right side when turning
			positionKi = 0.000350, //integral constant
			positionKd = 4; //derivative constant

static int STRAIGHT = 2; //the 2 here shouldn't matter as long as no variables are multiplied by 'direction' in driveDistancePID
static int STRAFE = 3; //don't multiply values by this variable!
static int ROTATE_LEFT = -1;
static int ROTATE_RIGHT = 1;

//drivetrain movement functions

//positive values will go forward or right
//negative values go backwards or left
//encoder counts is how many counts to go, see note about positive/negative values
//power is the power to run the motors at before straightening is applied
//time is a maximum time to complete the operation
void driveDistance(int power, int encoderCounts, int direction) {
	//writeDebugStreamLine("nPgmTime,error,nMotorEncoder[lDriveFront], nMotorEncoder[rDriveFront],pTerm,iTerm,dTerm,lPower,rPower");
	//reset encoder values
	nMotorEncoder[rDriveFront] = 0;
	nMotorEncoder[lDriveFront] = 0;

	int error = 0,
	straighteningError = 0,
	errorSum = 0,
	lastError = 0,
	target = encoderCounts;

	float straighteningCorrection,
	lPower,
	rPower;

	int lfMult = 1,
			lbMult = 1,
			rfMult = 1,
			rbMult = 1;


	time1[T1] = 0;
	if (direction == STRAIGHT || direction == STRAFE) { //validate direction
			//limit the values of the power term to only be those that can be taken by the motors
			if (power > 127) {
				power = 127;
			} else if (power < -127) {
				power = -127;
			}

			if (encoderCounts < 0) {
				lfMult = -1;
				lbMult = -1;
				rfMult = -1;
				rbMult = -1;
			}

			if (direction == STRAIGHT) {
				while(encoderCounts > (nMotorEncoder[lDriveFront] + nMotorEncoder[rDriveFront])/2.0) {
							//adjust the powers sent to each side if the encoder values don't match
						straighteningError = nMotorEncoder[lDriveFront] - nMotorEncoder[rDriveFront];
						writeDebugStreamLine("%d",straighteningError);

						if (straighteningError > 0) { //left side is ahead, so slow it down
							lPower = power - straighteningError*straighteningKpLeft;
						} else { //otherwise, just set the right side to the power
							rPower = power;
						}
						if (straighteningError < 0) { //right side is ahead, so slow it down
							rPower = power - straighteningError*straighteningKpRight;
						} else { //otherwise, just set the right side to the power
							lPower = power;
						}

					setLeftDtMotors(lPower*lfMult,lPower*lbMult);
					setRightDtMotors(rPower*rfMult,rPower*rbMult);
					wait1Msec(25);
				}

			}



			//writeDebugStreamLine("%d,%f,%f,%f,%f,%f,%f,%f,%f",nPgmTime,target,error,nMotorEncoder[lDriveFront], nMotorEncoder[rDriveFront],pTerm,iTerm,dTerm,lPower,rPower);
			setLeftDtMotors(0,0);
			setRightDtMotors(0,0);

	} else if (direction == ROTATE_LEFT || direction == ROTATE_RIGHT) {
			//update error terms
			error = target - (abs(nMotorEncoder[lDriveFront]) + abs(nMotorEncoder[rDriveFront]))/2; //need to use absolute values here because one of
			errorSum += error;

			//limit the values of the power term to only be those that can be taken by the motors
			if (power > 127) {
				power = 127;
			} else if (power < -127) {
				power = -127;
			}

			lastError = error; //update last error

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

task auton()
{
	driveDistance(70,500,STRAIGHT)

	//wait1Msec(5000);

//	bool buttonPressed = false;
//	while(1) {
//	if(vexRT[Btn8D] == 1) {
//		if (!buttonPressed) {
//			buttonPressed = true;
//			driveDistancePID(0, FORWARD, 4000);
//			buttonPressed = false;
//		}
//	}
//}

	//configure gyro.  normally done in pre_auton function
	//SensorType[gyro] = sensorNone;
 // wait1Msec(500);
 // //Reconfigure Analog Port 8 as a Gyro sensor and allow time for ROBOTC to calibrate it
 // SensorType[gyro] = sensorGyro;
 // wait1Msec(2000);

	//rotate left, negative target
	//rotate right, positive target
  //driveDistancePID(-300, ROTATE_LEFT, 1500);

	//starting from red back tile
	//motor[intake] = 127;
	//driveDistancePID(450, FORWARD, 1250); //drive forward from the tile
	//wait1Msec(250); //wait a bit to let things settle out before turning
	//rotateDegrees(500,1); //turn left to face the blue side wall
	//wait1Msec(250);
	//driveDistancePID(725, FORWARD, 1750);
	//wait1Msec(250);
	//rotateDegrees(900,-1);

	//driveDistancePID(-300, FORWARD,1000);
	//driveDistancePID(200, FORWARD, 1000);
	////rotateDegrees(75, -1);
	//driveDistancePID(-300, FORWARD,1000);
	//setLeftDtMotors(0);
	//setRightDtMotors(0);
	//wait1Msec(1500);

  //starting from red side tile
	//wait1Msec(7000);
  //driveDistancePID(525, STRAIGHT, 1000); //move forward from tile
  //driveDistancePID(285,ROTATE_RIGHT, 750); //turn towards blue net
  //driveDistancePID(815,STRAIGHT,1250); //move forward towards blue net
  //driveDistancePID(460,ROTATE_LEFT, 1250); //turn so intake faces ball stack on side wall
  //motor[intake] = 127; //start the intake to prepare to pick up balls
  //driveDistancePID(-445,STRAIGHT,1000); //first pass at ball stack
  ////driveDistancePID doesn't stop motors when it's done.  Stop the motors here while we wait for a little to pick up balls on the first pass
  //setLeftDtMotors(0);
  //setRightDtMotors(0);
  //wait1Msec(750);
  //driveDistancePID(200,STRAIGHT,750);
  //driveDistancePID(-300,STRAIGHT,750);
  //wait1Msec(750);
  //motor[intake] = 0;
  //startTask(moveIntakeBack);
}
