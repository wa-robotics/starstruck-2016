#pragma config(UART_Usage, UART1, uartVEXLCD, baudRate19200, IOPins, None, None)

task main()
{
	bLCDBacklight = true;
	string mainBattery, backupBattery;
	while (true){
	clearLCDLine(0);
	clearLCDLine(1);
	//Display the Primary Robot battery voltage
displayLCDString(0, 0, "Primary: ");
sprintf(mainBattery, "%1.2f%c", nImmediateBatteryLevel/1000.0,'V'); //Build the value to be displayed
displayNextLCDString(mainBattery);
//Short delay for the LCD refresh rate
wait1Msec(100);

	}



}
