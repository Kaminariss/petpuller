#include <Arduino.h>
#include <LiquidCrystal.h>
#include <AccelStepper.h>

// LCD Pins
#define pin_RS 8
#define pin_EN 9
#define pin_d4 4
#define pin_d5 5
#define pin_d6 6
#define pin_d7 7

// Motor Pins
#define motorEnablePin 12
#define dirPin 2
#define stepPin 3

// Define motor interface type
#define motorInterfaceType 1

// Heater Pin
#define heaterPin 11

long thermistorTimestamp;
long periodI = 1000;

// buttons
unsigned long lastReadButtons = 0;

// Stepper motor variables
bool motorEnabled = false;
int motorSpeed = 2300;

// Temperature variables
float thermistorTemp;

#define RT0 100000 // Ω
#define B 4036 // K
#define VCC 5    // Supply voltage
#define R 10000  // R = 10KΩ

// Variables
float RT, VR, ln, VRT;
float T0 = 25 + 273.15;

// PID Variables

float pidError = 0;
float previousError = 0;
float elapsedTime;
float pidValue = 0;

float pidP = 0;
float pidI = 0;
float pidD = 0;

unsigned long timePrev = 0;

// PID settings
float setTemp = 145;

float kp = 9.1f;
float ki = 0.6;
float kd = 1.8;

bool heaterEnabled = false;

char tvBuff[4];
char tsBuff[4];
char pwmBuff[4];

byte charUp[] = {
	B00100,
	B01110,
	B10101,
	B00100,
	B00100,
	B00100,
	B00100,
	B00000
};

byte charDown[] = {
	B00100,
	B00100,
	B00100,
	B00100,
	B10101,
	B01110,
	B00100,
	B00000
};

LiquidCrystal lcd(pin_RS, pin_EN, pin_d4, pin_d5, pin_d6, pin_d7);
AccelStepper myStepper(motorInterfaceType, stepPin, dirPin);

void setup() {
	Serial.begin(250000);

	// LCD test
	lcd.begin(16, 2);
	lcd.createChar(0, charUp);
	lcd.createChar(1, charDown);

	// Heater switch
	pinMode(heaterPin, OUTPUT);
	// digitalWrite(heaterPin, LOW);

	// Motor pins
	pinMode(motorEnablePin, OUTPUT);
	digitalWrite(motorEnablePin, motorEnabled ? LOW : HIGH);
	myStepper.setMaxSpeed(6000);
	myStepper.setAcceleration(800);
	myStepper.setSpeed(motorSpeed);
}

void clearLCDLine(int line) {
	lcd.setCursor(0, line);
	for (int n = 0; n < 16; n++) {
		lcd.print(" ");
	}
}

void readThermistor() {
	VRT = analogRead(A4); // Acquisition analog value of VRT
	VRT = (5.00 / 1023.00) * VRT; // Conversion to voltage
	VR = VCC - VRT;
	RT = VRT / (VR / R); // Resistance of RT

	ln = log(RT / RT0);
	thermistorTemp = (1 / ((ln / B) + (1 / T0))); // Temperature from thermistor

	thermistorTemp -= 273.15; // Conversion to Celsius
}

void refreshDisplay() {
	clearLCDLine(0);
	lcd.setCursor(0, 0);
	String value = String(motorSpeed);
	lcd.print(motorEnabled ? "*" : "-");
	lcd.print(" Speed: " + value);

	clearLCDLine(1);
	lcd.setCursor(0, 1);

	dtostrf(thermistorTemp, 3, 0, tvBuff);
	dtostrf(setTemp, 3, 0, tsBuff);
	dtostrf((pidValue), 3, 0, pwmBuff);

	// lcd.write((heaterEnabled ? (char)0 : (char)1));
	lcd.print(pwmBuff);
	lcd.print(" T:"); //  + " S:" + tsBuff
	lcd.print(tvBuff);
	lcd.print(" S:");
	lcd.print(tsBuff);
}

void readButtons(unsigned long now) {
	if (now - lastReadButtons < 150) {
		return;
	}

	int x;
	x = analogRead(0);

	if (x < 60) {
		// Right button
		motorSpeed += 50;
		myStepper.setSpeed(motorSpeed);
		refreshDisplay();
	}
	else if (x < 200) {
		// Up button
		setTemp += 5;
		refreshDisplay();
	}
	else if (x < 400) {
		// Down button
		setTemp -= 5;
		refreshDisplay();
	}
	else if (x < 600) {
		// Left button
		motorSpeed -= 50;
		myStepper.setSpeed(motorSpeed);
		refreshDisplay();
	}
	else if (x < 800) {
		// Select button
		motorEnabled = !motorEnabled;
		Serial.println("Motor");
		Serial.println(motorEnabled ? "enabled" : "disabled");
		// Pin in driver is inversed
		digitalWrite(motorEnablePin, motorEnabled ? LOW : HIGH);
		refreshDisplay();
	}

	lastReadButtons = now;
}

void heaterControl(unsigned long now) {
	if (now - timePrev < 300) {
		return;
	}

	// For derivative, we need real time to calculate speed change rate
	elapsedTime = (now - timePrev) / 1000.0f;
	timePrev = now; // the previous time is stored before the actual time read

	Serial.println("tock");
	Serial.println(pidValue);

	pidError = setTemp - thermistorTemp;

	// Calculate the P value
	pidP = kp * pidError;

	// Calculate the I value in a range on +-3
	if (pidError > -3 && pidError < 3) {
		pidI += (ki * pidError);
	}

	// Now we can calculate the D value
	pidD = kd * ((pidError - previousError) / elapsedTime);
	// Final total PID value is the sum of P + I + D
	pidValue = pidP + pidI + pidD;

	// We define PWM range between 0 and 255
	if (pidValue < 0) {
		pidValue = 0;
	}

	if (pidValue > 255) {
		pidValue = 255;
	}

	// Now we can write the PWM signal to the mosfet on digital pin D3
	analogWrite(heaterPin, (int)pidValue);

	previousError = pidError;     // Remember to store the previous error for next loop.
}

void loop() {
	unsigned long now = millis();

	// Reading temperature
	readThermistor();

	// Pid controller
	heaterControl(now);

	// Print temperature to serial port
	if (now - thermistorTimestamp > periodI) {
		thermistorTimestamp = now;
		Serial.println(thermistorTemp);
		refreshDisplay();
	}

	readButtons(now);

	myStepper.runSpeed();
}