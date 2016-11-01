
//Contains variables and funtions for Alpha (1900A)
int threshold = 15;
const int forward = 1;
const int strafeRight = 2;
const int strafeLeft = 3;
const int backwards = 4;
float wheelCircumference = 3.25*PI;
float revolutions = 0;
float averageEncoderValue = 0;

void zeroEncoders(){
	nMotorEncoder[DRFLED1] = 0;
	nMotorEncoder[DRFRED1] = 0;
	nMotorEncoder[DRBRED1] = 0;
	nMotorEncoder[DRBLED1] = 0;
}
void setFBMotors(int speed){ //automatically sets the four bar power (reduce clutter)
	motor[RFBT] = speed;
	motor[RFBM] = speed;
	motor[RFBB] = speed;
	motor[LFBT] = speed;
	motor[LFBM] = speed;
	motor[LFBB] = speed;

}
void setDriveMotors(int FLSpeed, int FRspeed, int BRspeed, int BLspeed){
	motor[FL] = FLspeed;
	motor[FR] = FRspeed;
	motor[BR] = BRspeed;
	motor[BL] = BLspeed;
}
float getDTEncoderAverage(){
	averageEncoderValue = (abs(SensorValue(DRFLED1)) + abs(SensorValue(DRFRED1)) + abs(SensorValue(DRBRED1)) + abs(SensorValue(DRBLED1)))/4;
	return averageEncoderValue;
}
void move(int direction, int speed, int distance){
	revolutions = distance/wheelCircumference;
	switch (direction) {

	case forward:
		zeroEncoders();
		while(getDTEncoderAverage() < distance){
			setDriveMotors(127,127,127, 127);
		}
		break;

	case strafeLeft:
		zeroEncoders();
		while(getDTEncoderAverage() < distance){
			setDriveMotors(-127, 127, 127, -127);
		}
		break;

	case strafeRight:
		zeroEncoders();
		while(getDTEncoderAverage() < distance){
			setDriveMotors(127, -127, -127, 127);
		}
		break;

	case backwards:
		zeroEncoders();
		while(getDTEncoderAverage() < distance){
			setDriveMotors(-127, -127, -127, -127);
		}
		break;

	default:
		setDriveMotors(0,0,0,0);

	}
}
