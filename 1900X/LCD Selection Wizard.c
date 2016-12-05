string startingTile = "";
int waitTime = -1;

void waitForPress() {
	while(nLCDButtons == 0) {
		wait1Msec(50);
	}
}

void waitForRelease() {
	while(nLCDButtons > 0) {
		wait1Msec(50);
	}
}

task lcdSelection();

task stopLcdSelectionMonitor() {
	while(1) {
		if (abs(vexRT[Ch1]) > 35 || abs(vexRT[Ch2]) > 35 || abs(vexRT[Ch3]) > 35 || abs(vexRT[Ch4]) > 35) {
			clearLCDLine(0);
			clearLCDLine(1);
			stopTask(lcdSelection);
			stopTask(stopLcdSelectionMonitor);
		}
		wait1Msec(250);
	}
}

task lcdSelection()
{
	startTask(stopLcdSelectionMonitor);
	bDisplayCompetitionStatusOnLcd = false;
	bLCDBacklight = true;
	clearLCDLine(0);
	clearLCDLine(1);

	displayLCDCenteredString(0,"Starting tile:");
	displayLCDCenteredString(1,"Stars  Sk.  Fence");
	waitForPress();
	int startingTileBtn = nLCDButtons;
	waitForRelease();

	displayLCDCenteredString(0,"Wait time:");
	displayLCDCenteredString(1,"0      1       3");
	waitForPress();
	int waitTimeBtn = nLCDButtons;

	switch (startingTileBtn) {
		case 1:
			startingTile = "stars";
			break;
		case 2:
			startingTile = "programmingSkills";
			break;
		case 4:
			startingTile = "fence";
			break;
	}

	switch (waitTimeBtn) {
		case 1:
			waitTime = 0;
			break;
		case 2:
			waitTime = 1;
			break;
		case 4:
			waitTime = 3;
			break;
	}

	displayLCDString(0,0,startingTile);
	displayLCDString(0,6,"wait");
	displayLCDNumber(0,12,waitTime);
	displayLCDCenteredString(1,"selected");

}
