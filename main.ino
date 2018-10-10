
/*
***************************************************
  RoasterOven v3.03
  12/30 bjh

  v2.00 is ported to arduino pro micro 5v (mini)
  program as a leonardo
  v3.00 uses 3.5" adafruit TFT instead of LiquidCrystal and arduino pro mini with new hw design
  v3.01 fixes lots of shit in 3.00
  v3.02 is the first steps towards something useful. Should work as is (need to uncomment fnOperateOven), can't change temp tho
  v3.03 tempadjust screen


  TODO
  -actually calibrate touchscreen
  -incorporate LED for heater on (D7)
  -rotate screen (ugh)
  
***************************************************
*/

// Color definitions
#define BLACK 0x0000
#define BLUE 0x001F
#define RED 0xF800
#define GREEN 0x07E0
#define CYAN 0x07FF
#define MAGENTA 0xF81F
#define YELLOW 0xFFE0
#define WHITE 0xFFFF


//#include <PID_AutoTune_v0.h>
#include <PID_v1.h>
//#include <LiquidCrystal.h>
#include <EEPROM.h>
#include <Adafruit_HX8357.h>
#include <Adafruit_GFX.h>
#include <SPI.h>
#include <TouchScreen1.h>


//LCD setup
// These are 'flexible' lines that can be changed
#define TFT_CS 10
#define TFT_DC 9
#define TFT_RST -1 // RST can be set to -1 if you tie it to Arduino's reset

// Use hardware SPI (on Uno, #13, #12, #11) and the above for CS/DC
Adafruit_HX8357 tft = Adafruit_HX8357(TFT_CS, TFT_DC, TFT_RST);

#define YP A3  // must be an analog pin, use "An" notation!
#define XM A0  // must be an analog pin, use "An" notation!
#define YM A2   // can be a digital pin
#define XP A1   // can be a digital pin
#define TS_MINX 123
#define TS_MAXX 930
#define TS_MINY 77
#define TS_MAXY 930
#define MapX1 239
#define MapX2 0
#define MapY1 0
#define MapY2 319
#define MINPRESS 250
#define TFTROTATION 2  //3=landscape, SPI pins on left //2 is portrait, header at top
TouchScreen ts = TouchScreen(XP, YP, XM, YM, 275, TFTROTATION);

//#include <Fonts/FreeSans12pt7b.h>
//#include <Fonts/FreeSans18pt7b.h>	
//#include <Fonts/FreeSans24pt7b.h>		
//#include <Fonts/FreeSans9pt7b.h>
//#define FONT12 &FreeSans12pt7b
//#define FONT18 &FreeSans18pt7b
//#define FONT24 &FreeSans24pt7b
//#define FONT9 &FreeSans9pt7b

const int thermocouplePin = A4;
const int relayPin = 8;  //output digital pin for relay
const int ledPin = 6; //output ledPin
const int btnUp = 9; //digital Input for btnUp
const int btnDn = 7;  //digital Input for btnDn
//both btns need to have pullup resistors

double setPoint; //target temp in deg F. Update through user menu, not here
float tempF;   //this is sensed temp


double pidInput;  //this is the sensed temp in PID parlance
double pidOutput;  //this is the PID output, to be translated into heatOnCmd
int pidWindowSize = 5000;   // SSR slow PWM window: 5 sec
unsigned long pidWindowStartTime;   // SSR slow PWM time counter
unsigned long previousMillis = 0;
const int voltageThermocouple = 5; //volt ref that thermocouple uses (3.3 or 5)
unsigned long previousNow2 = 0;
float previousTemp = 0.0;
unsigned long previousDisplayTime = 0;
bool tempAdjustFirstPass;
bool previousError = false;
const int setPointMax = 525; // deg F
bool tempAdjustMode = false;
bool tempDn = false;
bool tempUp = false;
//bool saveTrigger = true;
bool pauseMode = false;
unsigned long crack1StartTime = 0;
unsigned long crack2StartTime = 0;
bool heatOnCmd = false; // heater relay on or off command. initialize off.
bool previousHeat = true;  //used to redraw on change. Initialize true to cause a draw
bool reDrawTrigger = false;  //used to redraw displays after a button push
bool reDrawItem[] = {true,true,true,true,true,true};
bool endTrigger = false;
unsigned long previousSaveTime = 0;
bool previousPauseMode = false;

//Display locations
const byte B1x = 10;  //1st crack btn
const byte B1y = 168;
const byte B1h = 50;
const byte B1w = 146;
const byte B2x = B1x;  //2nd crack btn
const byte B2y = 228;
const byte B2h = B1h;
const byte B2w = B1w;
const byte B3x = 10;  //temp adjust btn
const int  B3y = 400;
const byte B3h = 75;
const byte B3w = 140;
const byte B4x = 160;  //pause heat btn
const int  B4y = B3y;
const byte B4h = B3h;
const byte B4w = B3w;
const byte B5x = B4x;	// END heat and freeze all timers btn
const byte B5w = B4w;
const byte B5h = B4h;
const int  B5y = B4y - B5h - 5;
const byte disp1x = 55;	//Main timer 
const byte disp1y = 25;
const byte disp1w = 210;
const byte disp1h = 57;
const byte disp2x = 60;	//Oven temp
const byte disp2y = 92;
const byte disp3x = disp2x+5;	//Target temp
const byte disp3y = disp2y+34;
const byte disp3w = 188;
const byte disp3h = 28;
const byte disp4x = 10;	//1C timer
const byte disp4y = 168;
const byte disp4w = 146;
const byte disp4h = 50;
const byte disp5x = 10;	//2C timer
const byte disp5y = 228;
const byte disp5w = 146;
const byte disp5h = 50;


PID myPID(&pidInput, &pidOutput, &setPoint, 185, 5, 1, DIRECT); //configure the PID


void setup() {

  pinMode(relayPin, OUTPUT); //set the relay output pin
  pinMode(btnDn, INPUT_PULLUP);  //NOTE: the pullup means they are opposite mode. normally true, GND for false
  pinMode(btnUp, INPUT_PULLUP);


  heatOnCmd = false;  //zero on powerup

  previousMillis = 0;  //start the clock

  //PID setup stuff
  pidInput = 0;   //this isnt what the example does. This is hack?
  pidWindowStartTime = millis();
  myPID.SetOutputLimits(0, pidWindowSize);
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(1000);

  //read temp setting from memory
  setPoint = EEPROM.read(1) + 350;  //offset storage by 350 cause it must be 0-255
  if (setPoint == 350) {  //If its 0 (first time ever..) then set to 490
    setPoint = 490;
    //    EEPROM.update(1, 140);
  }
  if (setPoint > setPointMax) {
    setPoint = setPointMax;
  }
 // int _setPointInt = setPoint;  //otherwise prints ugly numbers
  
  //Splash Screen on powerup
  tft.begin(HX8357D);
  tft.setRotation(TFTROTATION);
  tft.fillScreen(BLACK);
  //tft.setFont(FONT24);
  tft.setCursor(12,100);
 // tft.setTextSize(4);
  tft.setTextColor(WHITE);
	tft.println("Bryan's hipster");
	tft.println(" coffee roaster");
	tft.setFont();
	int _barSizeFraction = 0;
	int _previousBarSizeFraction = 0;
	unsigned long _timeNow = millis()/1000;
	unsigned long _startTime = _timeNow;
	int _x = 0;
	int _w = 0;
	int _screenWidth = 320;
	int _barDuration = 5;
	for (int i=0;(i <= _screenWidth); i++) {
		tft.fillRect(_x,28,i,12,GREEN);
		delay(_barDuration/_screenWidth);
   }

	
	tft.fillScreen(BLACK);

	//testing purposes only!!! Remove these lines!!
//		heatOnCmd = true;
//	previousHeat = !heatOnCmd;
}

void loop() {
  fnOperateOven();

//  testing purposes only! Remove these lines!
//	setPoint = random(200,500);
//	tempF = tempF++;
//	if (tempF == 500) {
//		tempF = 400;
//	}

/*	reDrawItem:
		0: 1C button
		1: 2C button
		2: Temp Adjust button
		3: Pause / DN button
		4: End / UP button
		5: Heat Status light
*/

	TSPoint p = ts.getPoint();  //read touchscreen and set object p with xyz data
    p.x = map(p.x, TS_MINX, TS_MAXX, tft.width(), 0);
	p.y = map(p.y, TS_MINY, TS_MAXY, 0, tft.height());
	
	if (p.z > ts.pressureThreshhold) {  //while loop as long as button is pressed. Prevents stacking commands
		delay(100);
		p.z = 0;
		TSPoint p = ts.getPoint();  //read touchscreen and set object p with xyz data
		p.x = map(p.x, TS_MINX, TS_MAXX, tft.width(), 0);
		p.y = map(p.y, TS_MINY, TS_MAXY, 0, tft.height());
		if ( (p.x > B1x ) && (p.x < (B1x+B1w)) && (p.y > B1y) && (p.y < (B1y+B1h)) && (crack1StartTime==0) ){  //Set 1C
			crack1StartTime = millis();
			tft.fillRoundRect(B1x,B1y,B1w,B1h,10,CYAN);
			tft.setCursor(B1x+20,B1y+12);
			tft.setTextColor(BLACK);
			tft.setTextSize(3);
			tft.print("Set 1C");
			reDrawItem[0] = true;
			delay(150);
			tft.fillRect(B1x,B1y-10,B1w,B1h+10,BLACK);
		}
		else if ( (p.x > B2x ) && (p.x < (B2x+B2w)) && (p.y > B2y) && (p.y < (B2y+B2h)) && (crack2StartTime==0) ){  //Set 2C
			crack2StartTime = millis();
			tft.fillRoundRect(B2x,B2y,B2w,B2h,10,CYAN);
			tft.setCursor(B2x+20,B2y+12);
			tft.setTextColor(BLACK);
			tft.setTextSize(3);
			tft.print("Set 2C");
			reDrawItem[1] = true;
			delay(150);
			tft.fillRect(B2x,B2y-10,B2w,B2h+10,BLACK);
		}
		else if ( (p.x > B3x ) && (p.x < (B3x+B3w)) && (p.y > B3y) && (p.y < (B3y+B3h)) ){  //Temp adjust mode btn
			tempAdjustMode = !tempAdjustMode;		
			if (tempAdjustMode) {
				tempAdjustFirstPass = true;
			}
			tft.fillRoundRect(B3x,B3y,B3w,B3h,10,CYAN);
			reDrawItem[2] = true;
			reDrawItem[3] = true;
			reDrawItem[4] = true;
		}
		else if ( (p.x > B4x ) && (p.x < (B4x+B4w)) && (p.y > B4y) && (p.y < (B4y+B4h)) ){   //Pause btn / DN btn
			reDrawItem[3] = true;
			reDrawItem[5] = true;
			if (tempAdjustMode) {
				tempDn = true;
				previousSaveTime = millis();
//				_saveTrigger = true;
			}
			else {
			pauseMode = !pauseMode;
			}
			tft.fillRoundRect(B4x,B4y,B4w,B4h,10,CYAN);
		}
		else if ( (p.x > B5x ) && (p.x < (B5x+B5w)) && (p.y > B5y) && (p.y < (B5y+B5h)) ){ 	//END btn / UP btn
			reDrawItem[4] = true;
			if (tempAdjustMode) {
				tempUp = true;
				previousSaveTime = millis();
//				_saveTrigger = true;
			}
			else {
				endTrigger = true;
			}		
			tft.fillRoundRect(B5x,B5y,B5w,B5h,10,CYAN);
		}
	}
	if (reDrawTrigger) {
		tft.fillScreen(BLACK);
		reDrawItem[0] = true;
		reDrawItem[1] = true;
		reDrawItem[2] = true;
		reDrawItem[3] = true;
		reDrawItem[4] = true;
		reDrawItem[5] = true;
		reDrawTrigger = false;		
	}
	if (tempAdjustMode) {
		fnTempAdj();
	}
	if (endTrigger) {
		fnEnd();
	}
	fnPrintNumbers(tempF, heatOnCmd, setPoint);   //print the standard screen
	fnPrintHeatStatus();
	fnPrintButtons();
	fnSaveSetting();
}

void fnOperateOven() {
  // read the thermocouple
  int _rawTemp = analogRead(thermocouplePin);
  float _Vout = _rawTemp * (5.0 / 1023.0);  //read A2D and convert counts to volts
  float _tempC = (_Vout - 1.25) / 0.005 ;  //convert volts to deg C
  tempF = (_tempC * 9.0 / 5.0) + 32.0;  //convert C to F
  //tempF = tempF; //temp calibration, if required


  

  
  //  Thermocouple health check
  //check to make sure theromocouple reading is reasonable.
  // Only works if target temp is << 600 and its not freezing outside.
  
  if (tempF > 600 || tempF < 25) {
    fnErrorMsg(true);
	pauseMode = true;
  }


  //  Runaway oven preventer:
  //check to make sure the thermocouple changes by 10 deg in 30 sec.
  // Ensures that the thermocouple is actually inside the oven and
  // responding to the heater. Disabled if tempF is near setPoint.
  // This may false error if you change target temp or reset during a cooldown
  // cycle. As temp reaches local min to heat back up, may become true.

	unsigned long _now2 = millis() / 1000;
	if ( (_now2 - previousNow2) >= 30) {  //run every 30 sec
		int _errorTemp = setPoint - tempF; //these lines are clunky due to abs. Read about abs()
		_errorTemp = abs(_errorTemp);
		if ( _errorTemp > 25.0 ) {  //true if not near target temp
			if ( (tempF - previousTemp) < 10.0 ) {  //true if temp isn't changing
			//       if (tempF < 250.0)  {  //prevent false warnings at low temps. Runaway heater = hot
			// commented out high temp reqt. If temp  sensor is outside oven, wont be > 250
			pauseMode = true;
			fnErrorMsg(true);
			//       }
			}
		}
		previousNow2 = millis() / 1000;
		previousTemp = tempF;
	}
	else {
	fnErrorMsg(false);
	}

  //run the PID
  pidInput = tempF;
  myPID.Compute();

  // slow down the control loop to 1 change / (windowSize)
  unsigned long _now = millis();
  if ((_now - pidWindowStartTime) > pidWindowSize) {
    pidWindowStartTime += pidWindowSize;
  }
  if (pidOutput > _now - pidWindowStartTime) {
    heatOnCmd = true;
  }
  else {
    heatOnCmd = false;
  }
  
  //run the heater, as long as heat isn't paused
  if (!pauseMode) {
  digitalWrite(relayPin, heatOnCmd);
  }
}

void fnTempAdj () {

	if (tempUp) {
		setPoint = setPoint + 5;
		if (setPoint > setPointMax) {
			setPoint = setPointMax;
		}
	}
	if (tempDn) {
		setPoint = setPoint - 5;
	}
	tempUp = false;
	tempDn = false;
}

void fnPrintNumbers(int _currentTemp, bool _heatOnCmd, int _targetTemp) {
  unsigned long _timeNow = millis();
  unsigned long _timeElapsedsec = _timeNow / 1000;
  unsigned long _minElapsed = (_timeElapsedsec / 60);
  unsigned long _secElapsedLong = (_timeElapsedsec - (_minElapsed * 60));
  unsigned long _secElapsed = _secElapsedLong;

  //textsize		x	y	pixels
  //			1:	6	8 
  //			2:	12	16 
  //			3:	18	24 
	//			4:	24	32
	//			5:	30	40
	//			6:	36	48
	//			7:	42	56
		
	if (_timeNow >= (previousDisplayTime + 1000)) {         //only change screen @ 1hz
		previousDisplayTime = millis();
		
		//print main timer
/*		const byte disp1x = 50;
		const byte disp1y = 25;
		const byte disp1w = 210;
		const byte disp1h = 57; */
//		tft.fillRect(disp1x,disp1y,disp1w,disp1h,GREEN);

		//use alternate font
//		tft.fillRect(disp1x-8,15,disp1w+18,70,BLACK);  //all the distances are screwed up using alternate fonts
//		tft.setCursor(disp1x-10,disp1y+50);
//		tft.setFont(FONT24);
//		tft.setTextSize(2);
		
		//use std font
		tft.setCursor(disp1x,disp1y);
		tft.setFont();
		tft.setTextSize(7);
		
		tft.setTextColor(WHITE,BLACK);
		    //print time elapsed
		if (_minElapsed < 10) {
		  tft.print("0");
		}
		tft.print(_minElapsed);
		tft.print(":");
		if (_secElapsed < 10) {
		  tft.print("0");
		}
		tft.print(_secElapsed);
		tft.setFont();
		tft.setCursor(disp1x-10,2);
		tft.setTextSize(1);
		tft.print("total roast time");

		//print temperatures
		tft.setTextSize(3);

		if ((_targetTemp-_currentTemp) < 25) {
			tft.setTextColor(GREEN,BLACK);
		}
		else if (_currentTemp < 300) {
			tft.setTextColor(YELLOW,BLACK);
		}
		else {
			tft.setTextColor(WHITE,BLACK);
		}
		
/*		const byte disp2x = 55;
		const byte disp2y = 92;
		const byte disp3x = disp2x+5;
		const byte disp3y = disp2y+34;
		const byte disp3w = 188;
		const byte disp3h = 28; */
		tft.setCursor(disp2x,disp2y); 
		tft.print("Oven:");
		tft.print("  Set:");
//		tft.fillRect(disp3x,disp3y,disp3w,disp3h,BLACK);
		tft.setCursor(disp3x,disp3y);
		tft.setTextSize(4);
		tft.print((_currentTemp));
		tft.print("  ");
		tft.setTextSize(4);
		tft.print(_targetTemp);


		//display 1C timer
		if (crack1StartTime > 0) {  
			_timeElapsedsec = (_timeNow-crack1StartTime) / 1000;
			_minElapsed = (_timeElapsedsec / 60);
			_secElapsedLong = (_timeElapsedsec - (_minElapsed * 60));
			_secElapsed = _secElapsedLong;
			
/*			const byte disp4x = 10;
			const byte disp4y = 168;
			const byte disp4w = 146;
			const byte disp4h = 50; */
//			tft.fillRect(disp4x,disp4y,disp4w,disp4h,BLACK);
			tft.setCursor(disp4x,disp4y);
			tft.setTextSize(1);
			tft.setTextColor(WHITE,BLACK);
			tft.print("time since 1st crack");
			tft.setTextSize(5);
			tft.setCursor(disp4x,disp4y+10);
			if (_minElapsed < 10) {
			  tft.print("0");
			}
			tft.print(_minElapsed);
			tft.print(":");
			if (_secElapsed < 10) {
			  tft.print("0");
			}
			tft.print(_secElapsed);
		}
		//display 2C timer
		if (crack2StartTime > 0) {
			_timeElapsedsec = (_timeNow-crack2StartTime) / 1000;
			_minElapsed = (_timeElapsedsec / 60);
			_secElapsedLong = (_timeElapsedsec - (_minElapsed * 60));
			_secElapsed = _secElapsedLong;
			
/*			const byte disp5x = 10;
			const byte disp5y = 228;
			const byte disp5w = 146;
			const byte disp5h = 50; */
//			tft.fillRect(disp5x,disp5y,disp5w,disp5h,BLACK);
			tft.setCursor(disp5x,disp5y);
			tft.setTextSize(1);
			tft.setTextColor(WHITE,BLACK);
			tft.print("time since 2nd crack");
			tft.setTextSize(5);
			tft.setCursor(disp5x,disp5y+10);
			if (_minElapsed < 10) {
			  tft.print("0");
			}
			tft.print(_minElapsed);
			tft.print(":");
			if (_secElapsed < 10) {
			  tft.print("0");
			}
			tft.print(_secElapsed);
		}
	}
}

void fnErrorMsg(bool _errorStatus)  {
	if (_errorStatus) {
		previousError = true;
		tft.setCursor(B2x+6,B2y+70);
		tft.setTextColor(RED,BLACK);
		tft.setTextSize(4);
		tft.print("Sensor");
		tft.setCursor(B2x+6,B2y+70+40);
		tft.print("Error!");
	}
	if ( (!_errorStatus) && (!pauseMode) ){  //this will catch a falling edge
		tft.fillRect(B2x+6,B2y+70,141,68,BLACK);
	}
	previousError = false;
}

void fnPrintHeatStatus() {
	const byte x0 = 190;
	const byte y0 = 180;
	const byte h = 80;
	const byte w = 100;

	if (pauseMode) {
		heatOnCmd = false;
	}
	if (heatOnCmd != previousHeat) {
		reDrawItem[5] = true;
	}
	if (reDrawItem[5]) {
		if (heatOnCmd) {
			tft.fillRoundRect(x0,y0,w,h,10,RED);
			tft.setCursor(x0+15,y0+15);
			tft.setTextColor(BLACK,RED);
			tft.setTextSize(3);
			tft.print("Heat");
			tft.setCursor(x0+32,y0+15+30);
			tft.print("ON");
		}
		else  {
			tft.fillRoundRect(x0,y0,w,h,10,BLACK);
			tft.drawRoundRect(x0,y0,w,h,10,WHITE);
			tft.setCursor(x0+15,y0+15);
			tft.setTextColor(WHITE,BLACK);
			tft.setTextSize(3);
			tft.print("Heat");
			tft.setCursor(x0+23,y0+15+30);
			tft.print("OFF");
		}
	}
	reDrawItem[5] = false;
	previousHeat = heatOnCmd;
}

void fnPrintButtons() {
/*	const byte B1x = 10;  //1st crack btn
	const byte B1y = 168;
	const byte B1h = 50;
	const byte B1w = 146;
	const byte B2x = B1x;  //2nd crack btn
	const byte B2y = 228;
	const byte B2h = B1h;
	const byte B2w = B1w;
	const byte B3x = 10;  //temp adjust btn
	const int  B3y = 400;
	const byte B3h = 75;
	const byte B3w = 140;
	const byte B4x = 160;  //heat off btn
	const int  B4y = B3y;
	const byte B4h = B3h;
	const byte B4w = B3w; */
	
/*	reDrawItem:
		0: 1C button
		1: 2C button
		2: Temp Adjust button
		3: Pause / DN button
		4: End / UP button
		5: Heat Status light
*/

	// 1st crack btn
	if ( (crack1StartTime == 0) && (reDrawItem[0]) ) {
		tft.fillRoundRect(B1x,B1y,B1w,B1h,10,BLACK);
		tft.drawRoundRect(B1x,B1y,B1w,B1h,10,CYAN);
		tft.setCursor(B1x+20,B1y+12);
		tft.setTextColor(CYAN);
		tft.setTextSize(3);
		tft.print("Set 1C");
	}
	
	//2nd crack btn
	if ( (crack1StartTime == 0) && (reDrawItem[1]) ) {
		tft.fillRoundRect(B2x,B2y,B2w,B2h,10,BLACK);
		tft.drawRoundRect(B2x,B2y,B2w,B2h,10,CYAN);
		tft.setCursor(B2x+20,B2y+12);
		tft.setTextColor(CYAN);
		tft.setTextSize(3);
		tft.print("Set 2C");
	}
	
	//temp adjust btn
	if (reDrawItem[2]) {
		tft.fillRoundRect(B3x,B3y,B3w,B3h,10,BLACK);
		tft.drawRoundRect(B3x,B3y,B3w,B3h,10,CYAN);
		if (tempAdjustMode) {
			tft.setCursor(B3x+20,B3y+25);
			tft.setTextColor(CYAN);
			tft.setTextSize(3);
			tft.print("Return");
		}
		else {
			tft.setCursor(B3x+45,B3y+12);
			tft.setTextColor(CYAN);
			tft.setTextSize(3);
			tft.print("Chg");
			tft.setCursor(B3x+42,B3y+12+25);	
			tft.print("Temp");
		}
	}

	//Pause / DN button
	if ( (pauseMode) && (!previousPauseMode) ) {  //look for a rising edge
		reDrawItem[3] = true;
	}
	previousPauseMode = pauseMode;
	if (reDrawItem[3]) {
		tft.fillRoundRect(B4x,B4y,B4w,B4h,10,BLACK);
		tft.drawRoundRect(B4x,B4y,B4w,B4h,10,CYAN);
		if (tempAdjustMode) {
			tft.setTextColor(CYAN);
			tft.setCursor(B4x+52,B4y+25);
			tft.print("DN");
		}
		else if (!pauseMode) { //if pause is false, show pause option
			tft.fillRect(B4x+53,B4y+22,10,30,CYAN);
			tft.fillRect(B4x+53+25,B4y+22,10,30,CYAN);
		}
		else if (pauseMode) {  //if pause is true, show play option
			tft.fillRoundRect(B4x,B4y,B4w,B4h,10,CYAN); //fill in the button
			//draw triangle play button in black
			tft.fillTriangle(B4x+53,B4y+22,B4x+53,B4y+22+30,B4x+53+25+10,B4y+22+15,BLACK);
		}
	}
	
	//END ALL / UP  button
	if (reDrawItem[4]) {
		tft.fillRoundRect(B5x,B5y,B5w,B5h,10,BLACK);
		tft.drawRoundRect(B5x,B5y,B5w,B5h,10,CYAN);
		tft.setTextColor(CYAN);
		tft.setTextSize(3);
		if (tempAdjustMode) {
			tft.setCursor(B5x+52,B5y+25);
			tft.print("UP");
		}
		else {
			tft.setCursor(B5x+45,B5y+25);
			tft.print("END");
		}
	}
	reDrawItem[0] = false;
	reDrawItem[1] = false;
	reDrawItem[2] = false;
	reDrawItem[3] = false;
	reDrawItem[4] = false;
}

void fnEnd(){
	heatOnCmd = false;
	digitalWrite(relayPin, heatOnCmd);
	tft.fillScreen(BLACK);
	tft.setCursor(50,100);
	tft.setTextColor(RED);
	tft.setTextSize(3);
	tft.print("Are you sure?");
	byte const x = 90;
	byte const y = 200;
	byte const w = 150;
	byte const h = 150;
	tft.drawRoundRect(x,y,w,h,10,RED);
	tft.setCursor(x+w/2-41,y+w/2-20);
	tft.setTextSize(5);
	tft.print("YES");
	unsigned long _timeNow = millis(); 
	while (millis() < _timeNow + 10000) {  //need confirmation within 10 sec or return to normal
		TSPoint p = ts.getPoint();  //read touchscreen and set object p with xyz data
		p.x = map(p.x, TS_MINX, TS_MAXX, tft.width(), 0);
		p.y = map(p.y, TS_MINY, TS_MAXY, 0, tft.height());
		if ( (p.x > x ) && (p.x < (x+w)) && (p.y > y) && (p.y < (y+h)) ){
			tft.fillScreen(BLACK);
			_timeNow = millis();
			while (!heatOnCmd) { //will always be true until reset
				unsigned long _timeElapsedsec = _timeNow / 1000;
				unsigned long _minElapsed = (_timeElapsedsec / 60);
				unsigned long _secElapsedLong = (_timeElapsedsec - (_minElapsed * 60));
				unsigned long _secElapsed = _secElapsedLong;
								
				//display total roast time
				tft.setCursor(disp1x-10,disp1y+50);
				//tft.setFont(FONT24);
				tft.setTextSize(2);
				tft.setTextColor(WHITE,BLACK);
					//print time elapsed
				if (_minElapsed < 10) {
				  tft.print("0");
				}
				tft.print(_minElapsed);
				tft.print(":");
				if (_secElapsed < 10) {
				  tft.print("0");
				}
				tft.print(_secElapsed);
				tft.setFont();
				tft.setCursor(disp1x-10,0);
				tft.setTextSize(1);
				tft.print("total roast time");
								
				//display 1C timer
				if (crack1StartTime > 0) {  
					_timeElapsedsec = (_timeNow-crack1StartTime) / 1000;
					_minElapsed = (_timeElapsedsec / 60);
					_secElapsedLong = (_timeElapsedsec - (_minElapsed * 60));
					_secElapsed = _secElapsedLong;

					tft.setCursor(disp4x,disp4y);
					tft.setTextSize(1);
					tft.setTextColor(WHITE);
					tft.print("time since 1st crack");
					tft.setTextSize(5);
					tft.setCursor(disp4x,disp4y+10);
					if (_minElapsed < 10) {
					  tft.print("0");
					}
					tft.print(_minElapsed);
					tft.print(":");
					if (_secElapsed < 10) {
					  tft.print("0");
					}
					tft.print(_secElapsed);
				}
				//display 2C timer
				if (crack2StartTime > 0) {
					_timeElapsedsec = (_timeNow-crack2StartTime) / 1000;
					_minElapsed = (_timeElapsedsec / 60);
					_secElapsedLong = (_timeElapsedsec - (_minElapsed * 60));
					_secElapsed = _secElapsedLong;
					
					tft.setCursor(disp5x,disp5y);
					tft.setTextSize(1);
					tft.setTextColor(WHITE);
					tft.print("time since 2nd crack");
					tft.setTextSize(5);
					tft.setCursor(disp5x,disp5y+10);
					if (_minElapsed < 10) {
					  tft.print("0");
					}
					tft.print(_minElapsed);
					tft.print(":");
					if (_secElapsed < 10) {
					  tft.print("0");
					}
					tft.print(_secElapsed);

					
				}
			tft.setCursor(disp5x,disp5y+10+40+15);
			int _targetTemp = setPoint;
			tft.print(_targetTemp);
			tft.setTextSize(4);
			tft.print(" deg F");
			}
		}
	}
	endTrigger = false;  //this will exit and return to normal after 10 sec if not confirmed
	reDrawTrigger = true;
	tft.fillScreen(BLACK);
}

void fnSaveSetting(){

	if ( millis() > (previousSaveTime + 60000) ) {  //will only save every 60 sec no matter what
		previousSaveTime = millis();
		byte _setPointTrunc = setPoint - 350;               //cause it has to be 0-255
		_setPointTrunc = constrain(_setPointTrunc, 0, 255);  //just in case
		EEPROM.update(1, _setPointTrunc);
	}

}


/*
struct fnRunTS() {
	TSPoint p = ts.getPoint();  //read touchscreen and set object p with xyz data
	p.x = map(p.x, TS_MINX, TS_MAXX, tft.width(), 0);
	p.y = map(p.y, TS_MINY, TS_MAXY, 0, tft.height());
	return p;
}
*/
