const int ledPin = 13;
const int ERRCOUNT_ADDR = 36;
// Maintain a running average.
// Distance(cm) = c1/(Sensor(V)-c2)
// First test sample c1=16.28, c2=0.2333
// Second test sample c1=18.85, c2 = 0.1208
// third c1=13.9, c2=0.235
// const float c1=13.9; // unit is centimeter volts
// const float c2=0.235; // unit is volts
// average of 5 sensors:
const float c1=12.326; // unit is centimeter volts
const float c2=0.244; // unit is volts
const float maxDistance=150.0;
const float voltageAtMaxReading=(c1/maxDistance)+c2;

float runningAverage(float percentageNew, float average, float newReading);

long readBatterymV(); // Get Power supply voltage in millivolts.
                      // Should be about 5000 mV if plugged into USB port.

long smoothedVBatt(); // Filter out the noise in the battery voltage, and
					  // avoid reading the battery repeatedly in a short
					  // period of time. There are capacitors on the board,
					  // so the reading shouldn't change rapidly.
long readSensorInMilliV(int sensorAPin); // convert a sensor reading in counts
                                         // into a reading in millivolts;
                                         
float readSensorVolts(int sensorAPin);
//float readBatteryVolts();  // Use smoothedBatteryVolts() instead.
float smoothedBatteryVolts();
void eepromWriteFloat(int addr, float val);
float eepromReadFloat(int addr);
void eepromWriteLong(int addr, long val);
long eepromReadLong(int addr);
void clockedBlink(int blinks);
void upErrCount();
void saveErrCount();
void memReport();
long getSerialLong();
float getSerialFloat();
