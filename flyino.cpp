#include "Arduino.h"
#include "flyino.h"
#include <avr/eeprom.h>
#include "EEPROM.h"
long readBatterymV() {
  // Source: http://provideyourown.com/2012/secret-arduino-voltmeter-measure-battery-voltage/
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    //DigiKeyboard.println(1);
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
     //DigiKeyboard.println(2);
     ADMUX = _BV(MUX5) | _BV(MUX0) ;
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
     //DigiKeyboard.println(3);
     ADMUX = _BV(MUX3) | _BV(MUX2);   
  #else
    //DigiKeyboard.println(4);
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif  
 
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring
 
  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both
  
  long result = (high<<8) | low;

  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  //result = 1211861L / result; 
  return result; // Vcc in millivolts
}

float readBatteryVolts() {
	return(((float)readBatterymV())/1000.0);
}
float smoothedBatteryVolts() {
	static long lastReadingMillis = 0;
	long now = millis();
	static float smoothedReading = 0;
	if ( (lastReadingMillis + 50) < now ) {
		float newReading = readBatteryVolts();
		if ( smoothedReading == 0) {
			smoothedReading = newReading;
		} else {
			// Percentage factor for running average calculation
			const float battSmoothingFactor = 10.0;
			smoothedReading = runningAverage(battSmoothingFactor,smoothedReading,newReading);
		}
		lastReadingMillis = now;
	}
	return smoothedReading;
}
float runningAverage(float percentageNew,float average, float newReading) {
	float newAverage = ((percentageNew * newReading) + ((100.0-percentageNew)*average))/100.0;
	return newAverage;
}
long running100xAverage(long percentageNew, long averageX100, long newReading) {
//	Serial.print("runningAverage ");
//	Serial.print(percentageNew);
//	Serial.print(", ");
//	Serial.print(averageX100);
//	Serial.print(", ");
//	Serial.print(newReading);
//	Serial.print(", ");
	long new100xAverage = (percentageNew*newReading) + (((100-percentageNew)*averageX100)/100);
//	Serial.print(new100xAverage);
//	Serial.println("");
	return new100xAverage;
}
// battery voltage will be noisy. The motors can draw as much as two amps
// continuously, with larger peaks. So we calculate a running average
long smoothedVBatt() {
	static long lastReadingMillis = 0;
	long now = millis();
	static long smoothedReadingX100 = 0;
	if ( (lastReadingMillis + 50) < now ) {
		long newReading = readBatterymV();
		if ( smoothedReadingX100 == 0) {
			smoothedReadingX100 = newReading*100;
		} else {
			// Percentage factor for running average calculation
			const int battSmoothingFactor = 10;
			smoothedReadingX100 = running100xAverage(battSmoothingFactor,smoothedReadingX100,newReading);
		}
		lastReadingMillis = now;
	}
	return smoothedReadingX100/100;
}
long readSensorInMilliV(int sensorAPin) {
	long reading = analogRead(sensorAPin);
	long battVoltage = smoothedVBatt();
	return (reading*battVoltage)/1023;
}
float readSensorVolts(int sensorAPin) {
	float reading = analogRead(sensorAPin);
	float battVoltage = smoothedBatteryVolts();
	return (reading*battVoltage)/1023.0;
}
void eepromWriteFloat(int addr,float val){
	eeprom_write_block((void*)&val,(void*)addr,sizeof(float));
}
float eepromReadFloat(int addr){
	float value;
	eeprom_read_block((void*)&value,(void*)addr,sizeof(float));
	return value;
}
void eepromWriteLong(int addr,long val){
	eeprom_write_block((void*)&val,(void*)addr,sizeof(long));
}
long eepromReadLong(int addr){
	long value;
	eeprom_read_block((void*)&value,(void*)addr,sizeof(long));
	return value;
}
int blinkVal = 0;

void updateLED() {
	
	// Blink the LED blinks times in a row, then pause for two seconds.
	// With positive numbers, the LED will be on for two thirds of a second,
	// then off for one third. Negative numbers use short on times, long off times.
	// Repeat.
	// This function has to be called several times a second to keep the LED
	// up to date.
	int blinkOnTime;
	if ( blinkVal == 0)
	{
		digitalWrite(ledPin, LOW);
		return;
	}
	else if ( blinkVal > 0 )
	{
		blinkOnTime = 666; // Long blinkVal for positive counts
	}
	else if ( blinkVal < 0 )
	{
		blinkOnTime = 333;  // Short blinks for negative counts
		blinkVal = -blinkVal;
	}
	long period = (blinkVal + 2) * 1000;
	long delta = millis() % period;
	int secNo = delta / 1000;
	if ( secNo < blinkVal )
	{
		if ( delta % 1000 < 500 )
		{
			digitalWrite(ledPin, HIGH);
		}
		else
		{
			digitalWrite(ledPin, LOW);
		}
	}
	else
	{
		digitalWrite(ledPin, LOW);
	}
}
void clockedBlink(int blinks)
{
	blinkVal = blinks;
	updateLED();
}
unsigned char errCount = 0;
void upErrCount()
{
	if (errCount < 250 )
	{
		errCount ++;
	}
}
void saveErrCount()
{
	EEPROM.write(ERRCOUNT_ADDR, errCount);
}
// __brkval is the end of memory allocations. Since we aren't using
// malloc() and free() it is at 0, so it has no information.
// __heap_start is the address of the heap, which is another pool of memory.
// This function has a local variable, v, local variables are stored on the
// stack. at the time this function is called, it will be at the bottom of the
// stack.
int freeRam ()
{
	extern int __heap_start, *__brkval;
	int v;
	return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}
void memReport()
{
	Serial.print(F("Free RAM space: "));
	Serial.println(freeRam());
}
void waitForText(char *msg) {
	int count = 0;
	while( !Serial.available()) {
		count++;
		delay(100);
		if ((count % 300) == 0)
		{
			Serial.println(msg);
		}
	}
	delay(100); // Wait for the rest of the number to come in. Serial Monitor sends stuff line at a time.
	
}
float getSerialFloat()
{
    waitForText("Please type a number with decimal point");
	long number = 0;
	long decimal = 0;
	boolean isNegative = false;
	boolean inFraction = false;
	long divider = 1;
	while ( Serial.available() )
	{
		int c = Serial.read();
		if ( c == '-')
		{
			isNegative = true;
		}
		else if ( c == '.' ) {
			inFraction = true;
		}
		else if ( c >= '0' && c <= '9')
		{
			if ( inFraction ) {
				decimal = (decimal * 10) + (c - '0');
				divider*=10;
			} else {
				number = (number * 10) + (c - '0');
			}
		}
	}
	float value = (float)number + (float)decimal/divider;
	Serial.print("Float debug, got ");
	Serial.print(number);
	Serial.print(F(", "));
	Serial.print(decimal);
	Serial.print(F(", "));
	Serial.print(divider);
	Serial.print(F(", "));
	Serial.println(value);
	return value;
}
long getSerialLong()
{
	waitForText("Please type a whole number");
	long number = 0;
	boolean isNegative = false;
	while ( Serial.available() )
	{
		int c = Serial.read();
		if ( c == '-')
		{
			isNegative = true;
		}
		else if ( c >= '0' && c <= '9')
		{
			number = (number * 10) + (c - '0');
		}
	}
	Serial.print("You entered ");
	if ( isNegative)
	{
		Serial.println(-number);
		return -number;
	}
	else
	{
		Serial.println(number);
		return number;
	}
}
