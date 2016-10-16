
//Contains variables and funtions for Alpha (1900A)

int threshold = 15;
int forward = 1;
int strafeRight = 2;
int strafeLeft = 3;
int backwards = 4;
float wheelCircumference = 3.25*PI();
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
	motor[RFBB] = speed;
	motor[LFBT] = speed;
	motor[LFBB] = speed;

}
float getDTEncoderValue(){
	averageEncoderValue = abs(SensorValue(DRFLED1)) + abs(SensorValue

}
int move(int direction, int speed, int distance){
	revolutions = distance/wheelCircumference;


}
