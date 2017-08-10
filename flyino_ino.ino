
#include "flyino.h"
#include "EEPROM.h"

const int gyroAPin = 4;
const int gyroRefAPin = 3;
const int leftRangeAPin = 1;
const int rightRangeAPin = 0;
const int downRangeAPin = 2;
// Serial communication is on D0 and D1




const int upperMotor = 9; // digital D9
//#define CALEB
#ifdef CALEB
const int lowerMotor = 5; // digital D10 for most, but D5 for Caleb, his pin 10 is busted.
#else
const int lowerMotor = 10;
#endif
// LED is on D13


//
const int OFFLINE_CMD_ADDR = 0;
const int TRIM_ADDR = 1;
const int BASE_HOVER_POWER_ADDR = 2;
const int BATTERY_VOLTAGE_READING_ADDR = 3;
const int BATTERY_V_MIN_ADDR = 4; // floats take 4 bytes each
const int BATTERY_V_MAX_ADDR = 8;
const int GYRO_V_MIN_ADDR = 12;
const int GYRO_V_MAX_ADDR = 16;
const int GYRO_REF_V_MIN_ADDR = 20;
const int GYRO_REF_V_MAX_ADDR = 24;
const int DEBUG0_ADDR = 28;
const int DEBUG1_ADDR = 29;
const int DEBUG2_ADDR = 30;
const int SECONDS_ADDR = 31;
const int ALTITUDE_ADDR = 32;
const int LOG_PERIOD_ADDR = 33;
const int LOG_TYPE_ADDR = 34;
const int ITERMS_ADDR = 35;
// const int ERRCOUNT_ADDR = 36;

const int ALT_KP_ADDR = 40;
const int ALT_KD_ADDR = 44;
const int GYRO_KP_ADDR = 48;
const int GYRO_KD_ADDR = 52;
const int WOBBLE_DURATION_ADDR = 56;
const int WOBBLE_MAGNITUDE_ADDR = 60;
const int DOLPHIN_DURATION_ADDR = 64;
const int DOLPHIN_MAGNITUDE_ADDR = 68;
const int TURN_STRENGTH_ADDR = 72;
const int UNUSED_FLOAT_ADDR = 76;

const int LOG_ADDR = 512;

void hoverTest(int unUsed);
void eepromWriteUserByte(int eeAddr);
void eepromWriteUserFloat(int eeAddr);
const boolean USE_FLIGHTPLAN = false;
typedef struct
{
  int eeAddr;
  char *cmdC;
  int minVal;
  int maxVal;
  char *name;
  void (*actionFunc)(int);
} cmdStruct;
void stopAndBlink(int blinkVal);
cmdStruct cStructs[] =
{
  {DEBUG0_ADDR, "D", 0, 254, "debug0", eepromWriteUserByte}, // Used when a bogus address is sent.
  {TRIM_ADDR, "t", -100, 100, "trim", eepromWriteUserByte},
  {BASE_HOVER_POWER_ADDR, "b", 0, 254, "base hover power", eepromWriteUserByte},
  {ALTITUDE_ADDR, "a", 10, 150, "altitude in cm", eepromWriteUserByte},
  {SECONDS_ADDR, "s", 1, 120, "seconds of flight", eepromWriteUserByte},
  {ITERMS_ADDR, "i", 3, 250, "milliseconds per iteration", eepromWriteUserByte},
  {LOG_PERIOD_ADDR, "p", 1, 100, "tenths of seconds between measurements", eepromWriteUserByte},
  {LOG_TYPE_ADDR, "l", 1, 254, "log type", eepromWriteUserByte},
  {ALT_KP_ADDR, "K", 0, 0, "Altitude proportional constant", eepromWriteUserFloat },
  {ALT_KD_ADDR, "L", 0, 0, "Altitude differential constant", eepromWriteUserFloat },
  {GYRO_KP_ADDR, "M", 0, 0, "Gyro propertional constant", eepromWriteUserFloat },
  {GYRO_KD_ADDR, "N", 0, 0, "Gyro differential constant", eepromWriteUserFloat },
  {WOBBLE_DURATION_ADDR, "P",0 ,0 , "Wobble duration, in seconds", eepromWriteUserFloat },
  {WOBBLE_MAGNITUDE_ADDR, "Q",0,0, "Wobble magnitude factor", eepromWriteUserFloat },
  {DOLPHIN_DURATION_ADDR, "R",0,0, "dolphin duration in seconds", eepromWriteUserFloat },
  {DOLPHIN_MAGNITUDE_ADDR, "S", 0, 0, "dolphin magnitude in cm", eepromWriteUserFloat },
  {TURN_STRENGTH_ADDR, "T", 0,0, "turn strength factor", eepromWriteUserFloat},
  {UNUSED_FLOAT_ADDR, "U", 0,0, "Unused float 1", eepromWriteUserFloat},
  {0, "0", 0, 0, "end of list", stopAndBlink},
};

int trim = 0;
int iterMs = 10;
int baseHoverPower;
int secondsOfFlight = 1;
int targetAltitude = 40;
float startGyro = 1.48;
int logPeriod = 5; // (1/10)s
int logType = 0; //1 is altitude, 2 is X, 4 is Z ...

int cstNum(int eepromAddr) // Look up an entry in the command struct list using the eeprom address as a key.
{
  for (int cst = 0; cStructs[cst].eeAddr != 0; cst++)
  {
    if ( eepromAddr == cStructs[cst].eeAddr )
    {
      return cst;
    }
  }
  return 0;  // the debug value
}
int eepromMinValue(int eepromAddr)
{
  return cStructs[cstNum(eepromAddr)].minVal;
}
int eepromMaxValue(int eepromAddr)
{
  return cStructs[cstNum(eepromAddr)].maxVal;
}
void eepromPrintValueOrStatus(int eepromAddr)
{
  int currentValue = eepromTranslatedRead(eepromAddr);
  if ( eepromValueIsBad(eepromAddr, currentValue))
  {
    Serial.println(F("current value has not been set since eeprom was cleared."));
  }
  else
  {
    Serial.print(F(" current value is "));
    Serial.println(currentValue);
  }
}
void showCommand(int eepromAddr)
{
  Serial.print(cStructs[cstNum(eepromAddr)].cmdC);
  Serial.print(F(" Set or change the \""));
  Serial.print(cStructs[cstNum(eepromAddr)].name);
  Serial.print(F("\","));
  eepromPrintValueOrStatus(eepromAddr);
}
void showAllCommands()   // Except for command 0
{
  for (int cst = 1; cStructs[cst].eeAddr != 0; cst++)
  {
    cStructs[cst].actionFunc(-cStructs[cst].eeAddr);
  }
}
float getRange(int sensorAPin)
{
  float reading = readSensorVolts(sensorAPin);
  if ( reading < voltageAtMaxReading)
  {
    return maxDistance;
  }
  float rangeCM = c1 / (reading - c2);
  return rangeCM;
}

void waitIfBattVoltageOutOfRange(float minVolts, float maxVolts)
{
  float battVolt = smoothedBatteryVolts();
  while ( battVolt < minVolts || battVolt > maxVolts)
  {
    if ( battVolt < minVolts )
    {
      clockedBlink(2);
    }
    else
    {
      clockedBlink(3);
    }
    battVolt = smoothedBatteryVolts();
  }
}
void logVoltageAndGyroMinMax()
{
  float lowBatt = 3.2; // we are losing power.
  float maxBatt = 4.5; // Above this voltage we are plugged in.
  waitIfBattVoltageOutOfRange(lowBatt, maxBatt);
  float battVolt = smoothedBatteryVolts();
  float battMin = battVolt;
  float battMax = battVolt;
  float gyroVal = readSensorVolts(gyroAPin);
  float gyroMax = gyroVal;
  float gyroMin = gyroVal;
  float gyroRefVal = readSensorVolts(gyroRefAPin);
  float gyroRefMax = gyroRefVal;
  float gyroRefMin = gyroRefVal;
  long nextSave = millis() + 1000;
  while(true)
  {
    waitIfBattVoltageOutOfRange(lowBatt, maxBatt);
    battVolt = smoothedBatteryVolts();
    battMax = max(battVolt, battMax);
    battMin = min(battVolt, battMin);
    gyroVal = readSensorVolts(gyroAPin);
    gyroMax = max(gyroVal, gyroMax);
    gyroMin = min(gyroVal, gyroMin);
    gyroRefVal = readSensorVolts(gyroRefAPin);
    gyroRefMax = max(gyroRefVal, gyroRefMax);
    gyroRefMin = min(gyroRefVal, gyroRefMin);
    clockedBlink(1);
    long now = millis();
    if ( now > nextSave)
    {
      nextSave = now + 1000;
      eepromWriteFloat(BATTERY_V_MIN_ADDR, battMin);
      eepromWriteFloat(BATTERY_V_MAX_ADDR, battMax);
      eepromWriteFloat(GYRO_V_MIN_ADDR, gyroMin);
      eepromWriteFloat(GYRO_V_MAX_ADDR, gyroMax);
      eepromWriteFloat(GYRO_REF_V_MIN_ADDR, gyroMin);
      eepromWriteFloat(GYRO_REF_V_MAX_ADDR, gyroMax);
    }
  }
}
//Includes a 1s delay
void trimGyro()
{
  delay(100);
  while (true)
  {
    float gyroReadings[10];
    float r2 = 0.0;
    float avgReading = 0.0;
    for (int i = 0; i < 10; i++)
    {
      gyroReadings[i] = readSensorVolts(gyroAPin);
      avgReading += gyroReadings[i];
    }
    avgReading /= 10.0;
    for (int i = 0; i < 10; i++)
    {
      r2 += (avgReading - gyroReadings[i]) * (avgReading - gyroReadings[i]);
    }
    Serial.print(r2);
  }
}
void doOfflineStuff()
{
  int offlineCmd = EEPROM.read(OFFLINE_CMD_ADDR);

  switch (offlineCmd)
  {
    case '4':
      logVoltageAndGyroMinMax();
      EEPROM.write(OFFLINE_CMD_ADDR, 255);
      break;
    case '5':
      hoverTest(0);
      break;
    case '6':
      hoverGyroTest();
      break;
    case '7':
      hoverHeightTest();
      break;
    case '8':
      break;
    case '9':
      break;
    default:
      break;
  }
  while (true)
  {
    clockedBlink(1);
  }
}
void setup()
{
  //trimGyro();



  delay(1000); // Give things time to stabilize.
  startGyro = readSensorVolts(gyroAPin); // Read the Gyro after letting things stabilize.
  //Load Parameters
  trim = eepromTranslatedRead(TRIM_ADDR);
  secondsOfFlight = eepromTranslatedRead(SECONDS_ADDR);
  targetAltitude = eepromTranslatedRead(ALTITUDE_ADDR);
  baseHoverPower = EEPROM.read(BASE_HOVER_POWER_ADDR);
  iterMs = EEPROM.read(ITERMS_ADDR);
  logPeriod = EEPROM.read(LOG_PERIOD_ADDR);
  logType = EEPROM.read(LOG_TYPE_ADDR);

  if (smoothedBatteryVolts() < 4.5 && EEPROM.read(OFFLINE_CMD_ADDR) != 255)
  {
    doOfflineStuff();
  }

  Serial.begin(115200);
  Serial.println(F("Flyino Library Test."));
}

void printFloatRange(char *name, float low, float high)
{
  Serial.print(name);
  Serial.print(F(" range ("));
  Serial.print(low);
  Serial.print(F(" - "));
  Serial.print(high);
  Serial.print(F(")"));
  return;
}

void showValues()
{
  char buffer[2];
  float battMin = eepromReadFloat(BATTERY_V_MIN_ADDR);
  float battMax =  eepromReadFloat(BATTERY_V_MAX_ADDR);
  float gyroMin =  eepromReadFloat(GYRO_V_MIN_ADDR);
  float gyroMax =  eepromReadFloat(GYRO_V_MAX_ADDR);
  float gyroRefMin = eepromReadFloat(GYRO_REF_V_MIN_ADDR);
  float gyroRefMax = eepromReadFloat(GYRO_REF_V_MAX_ADDR);
  int offlineCmd = EEPROM.read(OFFLINE_CMD_ADDR);
  int errCount = EEPROM.read(ERRCOUNT_ADDR);
  memReport();
  Serial.print("Saved Data ");
  printFloatRange("voltage", battMin, battMax);
  printFloatRange("gyro", gyroMin, gyroMax);
  printFloatRange("gyro ref", gyroRefMin, gyroRefMax);

  Serial.println();
  if ( offlineCmd > 0 && offlineCmd < 254)
  {
    Serial.print(F("Offline test queued. Option "));
    buffer[0] = (byte)(255 & offlineCmd);
    buffer[1] = 0;
    Serial.println(buffer);
  }
  Serial.print(F("Error count "));
  Serial.println(errCount);
  showAllCommands();
}

void showMenu()
{
  // The following is very ugly, due to limitations in the Arduino and the IDE.
  // The "F()" macro places strings into program flash. Without it, the strings
  // fill up RAM, and bad things happen at run time.
  // "\n" is a new line
  //
  Serial.println(F("\n"
                   "* Indicates that you will be prompted to remove the USB cable\n"
                   "  and turn the power switch off. Turn the power back on to start the test.\n"
                   "To see the results, connect it to the computer again and start the Serial Monitor.\n"
                   "1 Single Sensor Test\n"
                   "2 All Sensor Test\n"
                   "3 Motor Wiring Test\n"
                   "   The upper blades should spin for 2 seconds, the lower blades should spin for 1 second,\n"
                   "   with 1 second pauses between motors.\n"
                   "4* Battery voltage test, will also record minimum and maximum gyro readings."));
  Serial.println(F("5* Hover Test\n"
                   "   Enter base motor speed, and trim, then record whether it falls, hovers, or climbs\n"
                   "   and whether it spins clockwise, maintains its heading, or spins counter clockwise\n"
                   "6* Hover Test with Gyro stabilization.\n"
                   "7* Altitude hold with altitude sensor stabilization with proportional and differential feedback."));
  delay(1000);
}
void singleSensorTest()
{
  Serial.print(F(" Left Range finder reading: "));
  Serial.println(analogRead(leftRangeAPin));
}
void allSensorTest()
{
  Serial.print(F("Power: "));
  Serial.print(smoothedBatteryVolts());
  Serial.print(F(" Ranges "));
  Serial.print(F(" Left: "));
  Serial.print(getRange(leftRangeAPin));
  Serial.print(", ");
  Serial.print(readSensorVolts(leftRangeAPin));
  Serial.print(F(", Right: "));
  Serial.print(getRange(rightRangeAPin));
  Serial.print(F(", "));
  Serial.print(readSensorVolts(rightRangeAPin));
  Serial.print(F(", Down: "));
  Serial.print(getRange(downRangeAPin));
  Serial.print(F(", "));
  Serial.print(readSensorVolts(downRangeAPin));
  Serial.print(F(". Gyro: "));
  Serial.print(readSensorVolts(gyroAPin));
  Serial.print(F(" Gyro aRef: "));
  Serial.print(readSensorVolts(gyroRefAPin));

  Serial.println();
}
void print3digNo(int val)
{
  if( val < 10 )
  {
    Serial.print("  ");
  }
  else if ( val < 100 )
  {
    Serial.print(" ");
  }
  Serial.print(val);
}
void dumpEeprom()
{
  for (int i = 0; i < 256; i += 16)
  {
    for(int j = 0; j < 16; j++)
    {
      int addr = i + j;
      print3digNo(addr);
      Serial.print(":");
      print3digNo(EEPROM.read(addr));
      Serial.print(" ");
    }
    Serial.println();
  }
}
void motorWiringTest()
{
  delay(1000);
  analogWrite(upperMotor, 100);
  delay(2000);
  analogWrite(upperMotor, 0);
  analogWrite(lowerMotor, 100);
  delay(1000);
  analogWrite(lowerMotor, 0);
}
void altitudeAndBatteryTest(float altitude) {
    if (altitude < 10.0)
      {
        setMotorSpeeds(0.0, 0.0);
        stopAndBlink(-3);
      }
      if ( smoothedBatteryVolts() < 3.2 )
      {
        setMotorSpeeds(0.0, 0.0);
        stopAndBlink(-2);
      }
}
unsigned char flightPlan[] = {100, 40, 60, 80, 100, 100, 20, 20, 20, 10, 5, 5, 5, 5};
void hoverGyroTest()
{
  float motorSpeed = 100.0;

  float prevAltitude, prevLeftRange, prevRightRange, prevGyro;
  float altitude = getRange(downRangeAPin);
  float leftRange = getRange(leftRangeAPin);
  float rightRange = getRange(rightRangeAPin);
  float gyro = readSensorVolts(gyroAPin);
  setMotorSpeeds(100.0, 100.0);
  //Proportional and Differential gains (0 - 1)
  float Kp =  eepromReadFloat(ALT_KP_ADDR);// .5;
  float Kd = eepromReadFloat(ALT_KD_ADDR); // .1;
  float gyroKp = eepromReadFloat(GYRO_KP_ADDR); // 40.0;
  float gyroKd = eepromReadFloat(GYRO_KD_ADDR); // 10.0;
  int wobbleDuration = (int)(1000 * eepromReadFloat(WOBBLE_DURATION_ADDR)); // Convert to milliseconds
  float wobbleMagnitude = eepromReadFloat(WOBBLE_MAGNITUDE_ADDR); //
  int dolphinDuration = (int)(1000 * eepromReadFloat(DOLPHIN_DURATION_ADDR)); // convert to milliseconds
  float dolphinMagnitude = eepromReadFloat(DOLPHIN_MAGNITUDE_ADDR); // in cm
  float turnStrength = eepromReadFloat(TURN_STRENGTH_ADDR);
  float obstacleStrength = eepromReadFloat(UNUSED_FLOAT_ADDR); // This 
  //TODO needs work
  long startTime = millis();
  long nextIter = startTime + iterMs;
  long turnEndTime = 0;
//  boolean USE_OBSTRUCTION = obstacleStrength != 0.0; 
  long endTime = startTime + secondsOfFlight * 1000;
  long now = startTime;
  float altMultiplier = 1.0;
  while (now < endTime)
  {
    if ( now > nextIter )
    {
      nextIter += iterMs;
      if ( now > nextIter )
      {
        upErrCount();
        nextIter = now + iterMs;
      }
      if ( USE_FLIGHTPLAN )
      {
        int planSteps = (millis() - startTime) / 500;
        targetAltitude = flightPlan[planSteps];
      }
      long millisLeft = endTime - now;
      if ( millisLeft > 2000 )
      {
        altMultiplier = 1.0;
      }
      else
      {
        altMultiplier = (float)millisLeft / 2000.0;
      }
      prevAltitude = altitude;
      prevLeftRange = leftRange;
      prevRightRange = rightRange;
      prevGyro = gyro;
      altitude = getRange(downRangeAPin);
      leftRange = getRange(leftRangeAPin);
      rightRange = getRange(rightRangeAPin);
      gyro = readSensorVolts(gyroAPin);
      
      float targetGyro = startGyro;
    
      float delta = rightRange - leftRange;
      float nearest = min(rightRange, leftRange);

      // targetGyro = startGyro + (delta/nearest) * 0.4; // Worked well.
      targetGyro = startGyro + (delta / nearest) * turnStrength; // allows faster travel
      //  targetGyro = startGyro + (delta/nearest) * 1.6;
      // const int wobbleDuration = 500; // how long is a wobble?
    
      int wobblePhase = (now - startTime) % wobbleDuration; // 0 to 499 over the course of each half second.
      if ( wobblePhase > (wobbleDuration / 2) )
      {
        wobblePhase = wobbleDuration - wobblePhase; // 0 to 249, then 250 to 1
      }
      wobblePhase = wobblePhase - (wobbleDuration / 4); // -125 to 124, then 125 to -124
      targetGyro = targetGyro + (float)wobblePhase / (float) wobbleDuration;
      int dolphinPhase = (now-startTime)%dolphinDuration;
      if ( dolphinPhase > (dolphinDuration/2)) {
        dolphinPhase = dolphinDuration - dolphinPhase;
      }
      float dolphinDelta = ((float)dolphinPhase/(float)dolphinDuration) * dolphinMagnitude;
      float adjustedTargetAltitude = min(targetAltitude+dolphinDelta, 149.0);
      float altError = adjustedTargetAltitude * altMultiplier - altitude;
      float altDifferential = prevAltitude - altitude;
      float gyroError = targetGyro - gyro;
      float gyroDifferential = prevGyro - gyro;
      altitudeAndBatteryTest(altitude);
      altError = constrain(altError, -50.0, 50.0);
      float gyroCorrection = (gyroKp * gyroError) + (gyroKd * gyroDifferential);
      float motorSpeed = (Kp * altError) + (Kd * altDifferential) + 100;
      //  logData();

      setMotorSpeeds(motorSpeed - gyroCorrection, motorSpeed + gyroCorrection);
    }
    now = millis();
  }
  setMotorSpeeds(0.0, 0.0);
  saveErrCount();

  logVoltageAndGyroMinMax();
}
void stopAndBlink(int blinkVal)
{
  setMotorSpeeds(0.0, 0.0);
  while(true)
  {
    clockedBlink(blinkVal);
  }
}
void hoverHeightTest()
{
  float motorSpeed = 100.0;

  float prevAltitude;
  float altitude = getRange(downRangeAPin);
  setMotorSpeeds(100.0, 100.0);
  //Proportional and Differential gains (0 - 1)
  float Kp = .3;
  float Kd = .2; //.7
  //TODO needs work
  long startTime = millis();
  long nextIter = startTime + iterMs;
  while (millis() < startTime + (secondsOfFlight * 1000))
  {
    long now = millis();
    if ( now > nextIter )
    {
      nextIter += iterMs;
      if ( now > nextIter )
      {
        upErrCount();
        nextIter = now + iterMs;
      }
      prevAltitude = altitude;
      altitude = getRange(downRangeAPin);
      float altError = targetAltitude - altitude;
      float altDifferential = prevAltitude - altitude;
      altitudeAndBatteryTest(altitude);

      if (altError > 50)
      {
        altError = 50;
        digitalWrite(ledPin, HIGH);
        //break;
      }
      else if (altError < -50)
      {
        altError = -50;
        digitalWrite(ledPin, HIGH);
        //break;
      }
      else
      {
        digitalWrite(ledPin, LOW);
      }
      float motorSpeed = (Kp * altError) + (Kd * altDifferential) + 100;
      logData();

      setMotorSpeeds(motorSpeed, motorSpeed);
    }
  }
  setMotorSpeeds(0.0, 0.0);
  saveErrCount();

  logVoltageAndGyroMinMax();
}
void hoverTest(int unUsed)
{
  long startTime = millis();
  long endTime = startTime + secondsOfFlight * 1000;
  long now = startTime;
  while (now < endTime) {
    setMotorSpeeds(100.0, 100.0);
    float altitude = getRange(downRangeAPin);
    altitudeAndBatteryTest(altitude);
    delay(10);
  }
  setMotorSpeeds(0.0, 0.0);
  logVoltageAndGyroMinMax();
}
//Note that this is currently only called in hoverHeightTest
void logData()
{
  static int INCR_LOG_ADDR = LOG_ADDR;
  static int time = millis();

  if (INCR_LOG_ADDR >= 1015)
    return;

  if ((millis() - time) >= (logPeriod * 100))
  {

    if (logType && 1)  //If altitude
    {
      EEPROM.write(INCR_LOG_ADDR, (int)getRange(downRangeAPin));
      INCR_LOG_ADDR++;
    }
    time = millis();
  }
}
void emptySerialBuffer()
{
  while(Serial.available())
  {
    int junk = Serial.read();
  }
}
void scheduleOfflineTest(int testNo)
{
  EEPROM.write(0, testNo);
  Serial.println(F("To run this test disconnect the USB cable, and turn the Flyino off"));
  Serial.println(F("Position the Flyino for the test and turn it back on."));
  Serial.println(F("To see the results reconnect to the computer and start \"Serial Monitor\" again."));
}
char *eepromLabelStr(int eepromAddr)
{
  return cStructs[cstNum(eepromAddr)].name;
}
boolean eepromValueIsBad(int eepromAddr, int value)
{
  int minValue = eepromMinValue(eepromAddr);
  int maxValue = eepromMaxValue(eepromAddr);
  return ( value < minValue) || (value > maxValue);
}
void eepromTranslatedWrite(int eepromAddr, int value)
{
  if (eepromValueIsBad(eepromAddr, value))
  {
    return;
  }
  int minValue = eepromMinValue(eepromAddr);
  byte eeValue = 255 & (value - minValue);
  EEPROM.write(eepromAddr, eeValue);
}

int eepromTranslatedRead(int eepromAddr)
{
  int minValue = eepromMinValue(eepromAddr);
  int maxValue = eepromMaxValue(eepromAddr);
  int eeValue = EEPROM.read(eepromAddr);
  int scaledValue = eeValue + minValue;
  if ( scaledValue < minValue || scaledValue > maxValue)
  {
    return maxValue + 1;
  }
  return scaledValue;
}

void eepromWriteUserFloat(int eepromAddr)
{

  if ( eepromAddr >= 0 )
  {
    float currentValue = eepromReadFloat(eepromAddr);
    Serial.print(eepromLabelStr(eepromAddr));
    Serial.print(F(" current value is "));
    Serial.print(currentValue, 6);
    Serial.print(F(" Enter new value: "));
    float newValue = getSerialFloat();
    eepromWriteFloat(eepromAddr,newValue);
  }
  else
  {
    eepromAddr = -eepromAddr;
    Serial.print(cStructs[cstNum(eepromAddr)].cmdC);
    Serial.print(F(" Set or change the \""));
    Serial.print(cStructs[cstNum(eepromAddr)].name);
    Serial.print(F("\", "));
    Serial.println(eepromReadFloat(eepromAddr),5);
  }
}
void eepromWriteUserLong(int eepromAddr) {
  if ( eepromAddr >= 0 )
  {
    long currentValue = eepromReadLong(eepromAddr);
    Serial.print(eepromLabelStr(eepromAddr));
    Serial.print(F(" current value is "));
    Serial.print(currentValue, 6);
    Serial.print(F(" Enter new value: "));
    long newValue = getSerialLong();
    eepromWriteLong(eepromAddr,newValue);
  }
  else
  {
    eepromAddr = -eepromAddr;
    Serial.print(cStructs[cstNum(eepromAddr)].cmdC);
    Serial.print(F(" Set or change the \""));
    Serial.print(cStructs[cstNum(eepromAddr)].name);
    Serial.print(F("\", "));
    Serial.println(eepromReadLong(eepromAddr),5);
  }
}
void eepromWriteUserByte(int eepromAddr)
{
  if ( eepromAddr >= 0 )
  {
    int newValue = eepromMaxValue(eepromAddr) + 1;
    while ( eepromValueIsBad(eepromAddr, newValue))
    {
      Serial.print(eepromLabelStr(eepromAddr));
      int currentValue = eepromTranslatedRead(eepromAddr);
      eepromPrintValueOrStatus(eepromAddr);
      Serial.print(F("Enter a value for "));
      Serial.print(eepromLabelStr(eepromAddr));
      Serial.print(F(" between "));
      Serial.print(eepromMinValue(eepromAddr));
      Serial.print(F(" and "));
      Serial.print(eepromMaxValue(eepromAddr));
      newValue = getSerialLong();
    }
    eepromTranslatedWrite(eepromAddr, newValue);
  }
  else
  {
    showCommand(-eepromAddr);
  }
}
void doCmd(int option)
{
  switch (option)
  {
    case '1':
      singleSensorTest();
      break;
    case '2':
      allSensorTest();
      break;
    case '3':
      motorWiringTest();
      break;
    case '4':
      scheduleOfflineTest(option);
      break;
    case '5':
      eepromWriteUserByte(BASE_HOVER_POWER_ADDR);
      eepromWriteUserByte(TRIM_ADDR);
      scheduleOfflineTest(option);
      break;
    case '6':
    case '7':
    case '8':
    case '9':
      scheduleOfflineTest(option);
      break;
  }
}

float oldCorrectionFactor(float voltage)
{
  float correction = 1.330 / (voltage - 2.025);
  return correction;
}
// Motor speed is given as a percentage of the base motor power,
// modified as needed by the trim setting, and the current
// battery voltage. If the conditions would just stay the same
// as they were during calibration, and the calibration was
// perfect, then setMotorSpeeds(100.0,100.0) would cause the
// helicopter to hover without spinning.
void setMotorSpeeds(float upperMpercent, float lowerMpercent)
{
  // apply trim
  float trimmedUpperM = upperMpercent * (100.0 - (float)trim) / 100.0;
  float trimmedLowerM = lowerMpercent * (100.0 + (float)trim) / 100.0;

  // Motor power is a function of both PWM percentage and battery voltage.
  // AnalogWrite(pin,255) is the maximum achievable power. This value
  // will drop with battery voltage. If after all of the corrections are
  // applied a calculated pwm exceeds 255, both motor speed are reduced
  // proportionally so as to avoid inducing a spin.
  //
  // baseHoverPower is the PWM value that would be used to hover when the
  // battery voltage is at the rated full charge of 4.2 volts.

  // The correction factor for purely resistive loads is simple:
  // Equivalent voltage = (PWM_value * batteryVoltage)/255
  // motors are not purely resistive loads.

  // On an earlier generation of the software and hardware, the correction
  // equation (although expressed with integers to make it more confusing) was
  //  true_pwmValue = arbitraryPower * 1.330/(batteryVoltage-2.025)
  // All we know is that for any given arbitraryPower, the helicopter would
  // produce constant lift over the battery voltage range of 3.3 to 4.2 volts

  // We want something in explanable units.
  // Thus if baseHoverPower is sent via analogWrite() to both motors, when
  // the battery is at 4.2 volts, the helicopter will hover or as close as
  // can get. It will also spin, see the explanation of trim to see how we
  // correct for that to remove the spin.
  //
  // So scaled to a percentage of hoverPWM at 4.2 volts.
  // requestedPWM was in arbitrary units. With scaling we get:

  //  corrected_pwmValue = ((percentHoverPower/100.0) * baseHoverPower) * (1.330 / (batteryVoltage-2.025)) / ( 1.330/ (4.2 - 2.025))
  // Before we simplify it, I will explain the parts of the equation:
  //percentHoverPower/100.0  switch from percentage to being normalized at 1.0
  // baseHoverPower scale to 0-255 pwm values
  // 1.330 / (batteryVoltage-2.025))  historically derived voltage correction
  // 1.330 / (4.2 - 2.025)  is derived voltage correction for 4.2 volts.
  //   Simplifying the previous term, the 4.2 volt correction factor=0.6115

  // the last two terms become
  //   (1.330 / (batteryVoltage-2.025)) / 0.6115
  //   That becomes  (0.6115*1.330)/(batteryVoltage-2.025)
  //   which simplifies to 0.8133/(batteryVoltage-2.025) for the battery correction factor.
  //
  // The whole thing becomes:
  //   (percentHoverPower/100.0) * baseHoverPower * 0.8133/(batteryVoltage-2.025)
  // Move the 100.0 around we get:
  // correctedPWM = percentHoverPower * baseHoverPower * 0.008133/(batteryVoltage-2.025)

  // scale
  const float lowBattCutOff = 2.025;
  const float magicConstant = 1.330;
  const float batteryFull = 4.200;
  // const float battCorrectionFactor = (((magicConstant / (batteryFull - lowBattCutOff)) / magicConstant) / 100.0);
  float batteryVoltage = smoothedBatteryVolts();

  float battCorrectionFactor = oldCorrectionFactor(batteryVoltage) / oldCorrectionFactor(4.2);

  float scaledUpperM = trimmedUpperM * baseHoverPower * battCorrectionFactor / 100.0;
  float scaledLowerM = trimmedLowerM * baseHoverPower * battCorrectionFactor / 100.0;

  if ( scaledUpperM > 255.0 )
  {
    scaledLowerM = scaledLowerM * (255.0 / scaledUpperM);
    scaledUpperM = 255.0;
  }
  if ( scaledLowerM > 255.0 )
  {
    scaledUpperM = scaledUpperM * (255.0 / scaledLowerM);
    scaledLowerM = 255.0;
  }
  if ( upperMpercent == 0.0)
  {
    analogWrite(upperMotor, 0); // avoid roundoff errors. A speed of 1 uses sigficantly
    // more power to not spin the blades than a speed of 0.
  }
  else
  {
    analogWrite(upperMotor, int(scaledUpperM));
  }
  if ( lowerMpercent == 0.0 )
  {
    analogWrite(lowerMotor, 0);
  }
  else
  {
    analogWrite(lowerMotor, int(scaledLowerM));
  }
}
void readDisplayReflectance(int dPin)
{
  if ( digitalRead(dPin) )
  {
    Serial.print(F("****"));
  }
  else
  {
    Serial.print(F("...."));
  }
}
void reflectanceTest()
{

}
void loop()
{
  delay(500);
  int count = 0;
  while( ! Serial.available())
  {
    if ( (count % 180) == 0)
    {
      showValues();
    }
    if ( (count % 60) == 0)
    {
      Serial.println("");
      Serial.print("Type command or \"h\" for help> ");
    }
    delay(500);
    count += 1;
  }
  while ( Serial.available())
  {
    int option = Serial.read();
    boolean done = false;
    for (int cst = 1; cStructs[cst].eeAddr != 0; cst++)
    {
      if (option == cStructs[cst].cmdC[0])
      {
        cStructs[cst].actionFunc(cStructs[cst].eeAddr);
        done = true;
        continue;
      }
    }
    // Tests that repeat
    if ( done )
    {
      // Serial.println("shortcut worked");
    }
    else if ( option >= '1' && option <= '3')
    {
      emptySerialBuffer();
      while ( ! Serial.available() )
      {
        doCmd(option);
        delay(500);
      }
    }
    else if ( option >= '4' && option <= '9' )
    {
      doCmd(option);
    }
    else if ( option == 'r')
    {
      reflectanceTest();
    }
    else if ( option == 'e')
    {
      dumpEeprom();
    }
    else if ( option == 'h' || option == 'H')
    {
      showMenu();
    }
  }
}

