//drivetrain movement functions

void setLeftDtMotorsSeparate(int lPower, int rPower) {
	motor[lDriveFront] = lPower;
	motor[lDriveBack] = rPower;
}

void setRightDtMotorsSeparate(int lPower, int rPower) {
	motor[rDriveFront] = lPower;
	motor[rDriveBack] = rPower;
}

//positive powers will go forward or right
//negative powers values go backwards or left
//encoder counts is how many counts to go, always positive
//power is the power to run the motors at before straightening is applied
//time is a maximum time to complete the operation
void driveDistance(int power, int encoderCounts, int direction) {
	//writeDebugStreamLine("nPgmTime,error,nMotorEncoder[lDriveFront], nMotorEncoder[rDriveFront],pTerm,iTerm,dTerm,lPower,rPower");
	//reset encoder values
	SensorValue[lDriveFront] = 0;
	SensorValue[rDriveFront] = 0;

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
				while(encoderCounts > abs(SensorValue[lDriveFront] + SensorValue[rDriveFront])/2.0) {
					//adjust the powers sent to each side if the encoder values don't match
						straighteningError = SensorValue[lDriveFront] - SensorValue[rDriveFront];


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

					setLeftDtMotorsSeparate(lPower,lPower);
					setRightDtMotorsSeparate(rPower,rPower);
					wait1Msec(25);
				}

				setLeftDtMotorsSeparate(0,0);
				setRightDtMotorsSeparate(0,0);

			} else if (direction == STRAFE_LEFT) {

					while(encoderCounts > (abs(SensorValue[lDriveFront]) + abs(SensorValue[rDriveFront]))/2.0) {
							//adjust the powers sent to each side if the encoder values don't match
						straighteningError = abs(SensorValue[lDriveFront]) - abs(SensorValue[rDriveFront]);

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

					setLeftDtMotorsSeparate(lPower*-1,lPower*1);
					setRightDtMotorsSeparate(rPower*1,rPower*-1);
					wait1Msec(25);
				}

			//writeDebugStreamLine("%d,%f,%f,%f,%f,%f,%f,%f,%f",nPgmTime,target,error,nMotorEncoder[lDriveFront], nMotorEncoder[rDriveFront],pTerm,iTerm,dTerm,lPower,rPower);
			setLeftDtMotorsSeparate(0,0);
			setRightDtMotorsSeparate(0,0);

	} else if (direction == STRAFE_RIGHT) {

					while(encoderCounts > (abs(SensorValue[lDriveFront]) + abs(SensorValue[rDriveFront]))/2.0) {
							//adjust the powers sent to each side if the encoder values don't match
						straighteningError = abs(SensorValue[lDriveFront]) - abs(SensorValue[rDriveFront]);

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

					setLeftDtMotorsSeparate(lPower*1,lPower*-1);
					setRightDtMotorsSeparate(rPower*-1,rPower*1);
					wait1Msec(25);
				}


			//writeDebugStreamLine("%d,%f,%f,%f,%f,%f,%f,%f,%f",nPgmTime,target,error,nMotorEncoder[lDriveFront], nMotorEncoder[rDriveFront],pTerm,iTerm,dTerm,lPower,rPower);
			setLeftDtMotorsSeparate(0,0);
			setRightDtMotorsSeparate(0,0);

	}
	}
}
