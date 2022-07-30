#include <Arduino.h>
#include <LiquidCrystal.h>
#include <AccelStepper.h>
#include <pidcontroller.h>
#include <PIDAutotuner.h>
#include <MovingAveragePlus.h>
#include <thermistor.h>
#define TEMP_SENSOR_0 51

// LCD Pins
#define pin_RS 2
#define pin_EN 3
#define pin_d4 4
#define pin_d5 5
#define pin_d6 6
#define pin_d7 7

// Motor Pins
#define motorEnablePin 12
#define dirPin 13
#define stepPin 8
#define dividerRef 329

// Define motor interface type
#define motorInterfaceType 1

// Heater Pin
#define heaterPin 9
#define heaterInterval 500

// buttons
#define buttonPin A7
unsigned long lastReadButtons = 0;

// Stepper motor variables
bool motorEnabled = false;
int motorSpeed = 2300;

// Temperature variables
#define thermistorPin A6
#define staticDividerPin A5
float thermistorTemp;

#define RT0 47000 // Ω
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
float setTemp = 10;

float kp = 35.0f;
float ki = 2.0f;
float kd = 5.0f;

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
thermistor therm1(A4, 0);
PID pidController = PID(kp, ki, kd, 0, 255);

int analogX;
unsigned long lastReadThermistor = 0;
unsigned long lastRefreshThermistor = 0;
// MedianFilter2<float> thermistorReadings(30);
MovingAveragePlus<float> thermistorReadings(30);

float readThermistorRaw() {
	analogX = analogRead(staticDividerPin);
	analogX -= dividerRef;
	VRT = analogRead(thermistorPin); // Acquisition analog value of VRT
	VRT -= analogX;

	VRT = (5.00 / 1023.00) * VRT; // Conversion to voltage
	VR = VCC - VRT;
	RT = VRT / (VR / R); // Resistance of RT

	ln = log(RT / RT0);
	float temp = (1 / ((ln / B) + (1 / T0))); // Temperature from thermistor
	// float temp = therm1.analog2temp();

	return temp - 273.15; // Conversion to Celsius
}

void autotunePID() {
	PIDAutotuner tuner = PIDAutotuner();

	tuner.setTargetInputValue(setTemp);
	tuner.setLoopInterval(heaterInterval);
	tuner.setOutputRange(0, 255);

	// Set the Ziegler-Nichols tuning mode
	// Set it to either PIDAutotuner::znModeBasicPID, PIDAutotuner::znModeLessOvershoot,
	// or PIDAutotuner::znModeNoOvershoot. Test with znModeBasicPID first, but if there
	// is too much overshoot you can try the others.
	tuner.setZNMode(PIDAutotuner::znModeBasicPID);

	// This must be called immediately before the tuning loop
	tuner.startTuningLoop();

	// Run a loop until tuner.isFinished() returns true
	unsigned long microseconds;
	while (!tuner.isFinished()) {
		// This loop must run at the same speed as the PID control loop being tuned
		unsigned long prevMicroseconds = microseconds;
		microseconds = micros();

		// Get input value here (temperature, encoder position, velocity, etc)

		double input = readThermistorRaw();

		// Call tunePID() with the input value
		double output = tuner.tunePID(input);

		// Set the output - tunePid() will return values within the range configured
		// by setOutputRange(). Don't change the value or the tuning results will be
		// incorrect.
		// doSomethingToSetOutput(output);
		analogWrite(heaterPin, 255-output);

		Serial.print(millis()/1000.0f);
		Serial.print(',');
		Serial.print(output);
		Serial.print(',');
		Serial.println(input);
		// refreshDisplay();

		// This loop must run at the same speed as the PID control loop being tuned
		//while (micros() - microseconds < heaterInterval)
		delay(heaterInterval);
	}

	// Turn the output off here.
	// doSomethingToSetOutput(0);

	// Get PID gains - set your PID controller's gains to these
	kp = tuner.getKp();
	ki = tuner.getKi();
	kd = tuner.getKd();

	Serial.print("Kp: ");
	Serial.print(kp);
	Serial.print(" ki: ");
	Serial.print(ki);
	Serial.print(" Kp: ");
	Serial.print(kd);
}

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

	//TCCR2B = 0b00000111; // x1024
	//TCCR2A = 0b00000011; // fast pwm

	pidController.setSetpoint(setTemp);
	//autotunePID();
}

void clearLCDLine(int line) {
	lcd.setCursor(0, line);
	for (int n = 0; n < 16; n++) {
		lcd.print(" ");
	}
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


void readThermistor(unsigned long now) {
	// read thermistor once per 50ms
	if (now - lastReadThermistor < 50) {
		return;
	}

	float temp = readThermistorRaw();

	thermistorReadings.push(temp);
	thermistorTemp = thermistorReadings.get();

	// refresh display once per 500 ms
	if (now - lastRefreshThermistor > 500) {
		Serial.print(now/1000.0f);
		Serial.print(',');
		Serial.print(pidValue);
		Serial.print(',');
		Serial.print(thermistorTemp);
		Serial.print(',');
		Serial.println(analogX);
		refreshDisplay();
		lastRefreshThermistor = now;
	}

	lastReadThermistor = now;
}

void readButtons(unsigned long now) {
	if (now - lastReadButtons < 150) {
		return;
	}

	int x;
	x = analogRead(buttonPin);

	if (x < 60) {
		// Right button
		motorSpeed += 50;
		myStepper.setSpeed(motorSpeed);
		refreshDisplay();
	}
	else if (x < 200) {
		// Up button
		setTemp += 5;
		pidController.setSetpoint(setTemp);
		refreshDisplay();
	}
	else if (x < 400) {
		// Down button
		setTemp -= 5;
		pidController.setSetpoint(setTemp);
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
	if (now - timePrev < heaterInterval) {
		return;
	}

	pidValue = pidController.calculate(thermistorTemp);

	// Now we can write the PWM signal to the mosfet on digital pin D3
	analogWrite(heaterPin, 255 - (int)pidValue);
}

void loop() {
	unsigned long now = millis();

	// Reading temperature
	readThermistor(now);

	// Pid controller
	heaterControl(now);

	readButtons(now);

	myStepper.runSpeed();
}
