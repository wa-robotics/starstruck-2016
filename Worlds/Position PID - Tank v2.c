
float positionKp = .7, //proportional constant for positional PID
			straighteningKpLeft = 0.29,//.43,//.195, //proportional constant for straightening response for the left side
			straighteningKpRight = 0.29,//.22,//.16, //proportional constant for straightening response for the right side
			straighteningKpLeftTurn = 0.4,//.43,//.195, //proportional constant for straightening response for the left side when turning
			straighteningKpRightTurn = 0.4,//.22,//.16, //proportional constant for straightening response for the right side when turning
			positionKi = 0.000350, //integral constant
			positionKd = 4.25, //derivative constant
			encoderScale = 0.6; //0.6 is 9:15.  This is a constant to multiply encoder values by, for example to scale down a high number


static int STRAIGHT = 2; //the 2 here shouldn't matter as long as no variables are multiplied by 'direction' in driveDistancePID
static int STRAFE_LEFT = 3;
static int STRAFE_RIGHT = 4;
static int STRAFE = 5;
static int ROTATE_LEFT = -1;
static int ROTATE_RIGHT = 1;

float getScaledEncVal(float val) {
	return val*encoderScale;
}

//Check if motor power is > 127 or < -127, and if so, fix it
float clampPower(float power) {
	if (power > 127) {
		return 127;
	} else if (power < -127) {
		return -127;
	}
	return power;
}

//time is the max time, the PID may stop earlier if it is close to target
void liftToTargetPIDEnc(int target, int time, float kP, float kI, float kD) {
	int slewRateLimit = 15;

	float pTerm,
	iTerm,
	dTerm = 0,
	power,
	lastPower = 0,
	error = 0,
	errorSum = 0,
	lastError = 0;

	time1[T1] = 0;

	while (time1[T1] < time) {
		//update error terms
		error = target - SensorValue[liftEnc];
		errorSum += error;

		pTerm = error * (float) kP;
		iTerm = errorSum * (float) kI;
		dTerm = (error - lastError) * (float) kD; //calculate motor power
		power = pTerm + iTerm + dTerm;

		//limit the values of the power term to only be those that can be taken by the motors
		if (power > 127) {
			power = 127;
		} else if (power < -127) {
			power = -127;
		}

		lastError = error; //update last error

		//apply a slew rate to limit acceleration/deceleration
		if(abs(power-lastPower) > slewRateLimit) {
			if(power > lastPower) { //if the power is increasing (and the difference is greater than the slew rate allows)
				power = lastPower + slewRateLimit; //increment the power to only add
			} else { //if the power is decreasing (and the differene is greater than the slew rate allows)
				power = lastPower - slewRateLimit;
			}
		}

		lastPower = power; //update the last power
		writeDebugStreamLine("%d,%f,%f,%f,%f,%f,%f,%f",nPgmTime,target,error,SensorValue[liftEnc],pTerm,iTerm,dTerm,power);
		setDumpMotors(power);
		wait1Msec(25);
	}
}

//positive powers will go forward or right
//negative powers values go backwards or left
//encoder counts is how many counts to go, always positive
//power is the power to run the motors at before straightening is applied
//time is a maximum time to complete the operation
void driveDistancePID(int encoderCounts, int direction, int time, unsigned int maxPower = 127) {
	writeDebugStreamLine("nPgmTime,target,error,lEncVal,rEncVal,pTerm,iTerm,dTerm,lPower,rPower");
	//reset encoder values
	Sensorvalue[lDriveEnc] = 0;
	SensorValue[rDriveEnc] = 0;

	int target = encoderCounts,
	slewRateLimit = 15;

	float pTerm,
	iTerm,
	dTerm = 0,
	straighteningCorrection,
	power,
	lPower,
	rPower,
	lastPower = 0,
	error = 0,
	straighteningError = 0,
	errorSum = 0,
	lastError = 0,
	lEncVal = 0,
	rEncVal = 0;

	time1[T1] = 0;
	if (direction == STRAIGHT || direction == ROTATE_LEFT || direction == ROTATE_RIGHT) { //validate direction

	 if (direction == STRAIGHT) {
		while ((time1[T1] < time) && (time1[T1] < 250 || abs(lEncVal - rEncVal) > 15 || abs(error) > 15 || abs(error - lastError) > 5)) { //the PID can stop early if a little time as passed and the error is low and not changing much

					//get scaled encoder values
					lEncVal = getScaledEncVal(SensorValue[lDriveEnc]);
					rEncVal = getScaledEncVal(SensorValue[rDriveEnc]);

					//update error terms
					error = target - (lEncVal + rEncVal)/2.0;
					errorSum += error;

					pTerm = error * (float) positionKp;
					iTerm = errorSum * (float) positionKi;
					dTerm = (error - lastError) * (float) positionKd; //calculate motor power
					power = pTerm + iTerm + dTerm;

					//limit the values of the power term to only be those that can be taken by the motors
					if (abs(power) > 127) {
						power = sgn(power)*maxPower;
					}

					lastError = error; //update last error

					//apply a slew rate to limit acceleration/deceleration
					if(abs(power-lastPower) > slewRateLimit) {
						if(power > lastPower) { //if the power is increasing (and the difference is greater than the slew rate allows)
							power = lastPower + slewRateLimit; //increment the power to only add
						} else { //if the power is decreasing (and the differene is greater than the slew rate allows)
							power = lastPower - slewRateLimit;
						}
					}

					//adjust the powers sent to each side if the encoder values don't match
					straighteningError = abs(lEncVal) - abs(rEncVal);

					if (straighteningError < 0) { //right side is ahead, so speed up the right side
						rPower = power + straighteningError*straighteningKpRight*sgn(power);
					} else { //otherwise, just set the right side to the power
						rPower = power;
					}
					if (straighteningError > 0) { //left side is ahead, so speed up the left side
						lPower = power - straighteningError*straighteningKpLeft*sgn(power);
					} else { //otherwise, just set the right side to the power
						lPower = power;
					}

					lastPower = power; //update the last power
					writeDebugStreamLine("%d,%f,%f,%f,%f,%f,%f,%f,%f,%f",nPgmTime,target,error,lEncVal,rEncVal,pTerm,iTerm,dTerm,lPower,rPower);
					setLeftDtMotors(lPower);
					setRightDtMotors(rPower);
					wait1Msec(25);
				}
					writeDebugStreamLine("%d,%f,%f,%f,%f,%f,%f,%f,%f,%f",nPgmTime,target,error,lEncVal,rEncVal,pTerm,iTerm,dTerm,lPower,rPower); //write to debug stream a final time so exit conditions for the while loop are visible
					setLeftDtMotors(0);
					setRightDtMotors(0);
		 } else if (direction == ROTATE_LEFT || direction == ROTATE_RIGHT) {
					while ((time1[T1] < time) && (time1[T1] < 250 || abs(error) > 5 || fabs(abs(lEncVal) - abs(rEncVal)) > 15 || abs(error - lastError) > 5)) { //the PID can stop early if a little time as passed and the error is low and not changing much
						//get scaled encoder values
						lEncVal = getScaledEncVal(SensorValue[lDriveEnc]);
						rEncVal = getScaledEncVal(SensorValue[rDriveEnc]);

						//update error terms
						error = target - (abs(lEncVal) + abs(rEncVal))/2; //need to use absolute values here because one of
						errorSum += error;

						pTerm = error * (float) positionKp;
						iTerm = errorSum * (float) positionKi;
						dTerm = (error - lastError) * (float) positionKd; //calculate motor power
						power = pTerm + iTerm + dTerm;

						//limit the values of the power term to only be those that can be taken by the motors
						if (power > 127) {
							power = 127;
						} else if (power < -127) {
							power = -127;
						}

						lastError = error; //update last error

						//apply a slew rate to limit acceleration/deceleration
						if(abs(power-lastPower) > slewRateLimit) {
							if(power > lastPower) { //if the power is increasing (and the difference is greater than the slew rate allows)
								power = lastPower + slewRateLimit; //increment the power to only add
							} else { //if the power is decreasing (and the differene is greater than the slew rate allows)
								power = lastPower - slewRateLimit;
							}
						}

						//adjust the powers sent to each side if the encoder values don't match
						straighteningError = abs(lEncVal) - abs(rEncVal);

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

						lastPower = power; //update the last power
						writeDebugStreamLine("%d,%f,%f,%f,%f,%f,%f,%f,%f",nPgmTime,error,lEncVal,rEncVal,pTerm,iTerm,dTerm,lPower,rPower);
						setLeftDtMotors(lPower * direction); //for a left turn, ROTATE_LEFT = -1 so this moves the left side backwards for a left turn. For a right turn will go forward since ROTATE_RIGHT = 1
						setRightDtMotors(rPower * -1 * direction); //same idea as for a left turn, except this side needs to go the opposite way as the left side in order to turn, hence the * -1 in the calculation
						wait1Msec(25);
				}
				setLeftDtMotors(0);
				setRightDtMotors(0);
			}
		}
	}
