void clearLCD() {
	clearLCDLine(0);
	clearLCDLine(1);
}

void showOnLCD(string line0, string line1) {
	clearLCD();
	displayLCDString(0,0,line0);
	displayLCDString(1,0,line1);
}

void waitForPress() {
	while(nLCDButtons == 0) {
		wait1Msec(25);
	}
}

void waitForRelease() {
	while (nLCDButtons > 0) {
		wait1Msec(25);
	}
}

	//L-R-Sk
	//#1 Big
	//#2 Small
	//#3 Cube (score middle cube and block)
	//#4 Fence (3 stars, corner)
	//#5 Nothing
	//#6 Prog skills
task LCDSelect() {
	clearLCD();
	displayLCDCenteredString(0,"LCD Select v1.0"); //briefly display version for debug purposes
	wait1Msec(750);


	//Q1: Which side?
	showOnLCD("Which side?","Left  Right  Sk.");
	waitForPress();
	int q1Response = nLCDButtons;
	waitForRelease();

	int q2Page = 1; //1 - Big/Small choices shown, 2 - Cube/Fence choices shown, 3 - Nothing choice shown
	int q2Response = 0;
	while (q2Response == 0) {
		if (q2Page == 1) {
			showOnLCD("Choose play: 1/3","Big   Small   >>");
		}
	}

}
