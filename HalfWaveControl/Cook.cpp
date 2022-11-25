#include "Cook.h"

// Libraries for the DS18B20 Temperature Sensor
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#define ONE_WIRE A3


/* ------------------------------------------------------------------------
 *  Constructor
 */
Cook::Cook(int burner, int alarm, double temp = DefaultGoalTemp, long timeInSecs = DefaultGoalTime) {
  goalTemp = temp;
  currentTemp = 0;
  slope = 0;
  goalTimeInSeconds = timeInSecs;
  currentTimeInSeconds = 0;

  burnerPin = burner;
  pinMode(burnerPin, OUTPUT);
  // start with the burner off
  burnerStatus = LOW;
  digitalWrite(burnerPin, burnerStatus);
  
  alarmPin = alarm;
  pinMode(alarmPin, OUTPUT);
  digitalWrite(alarmPin, LOW);
  alarmMessage = "";
  
  // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
  oneWire = new OneWire(ONE_WIRE);
  // Pass our oneWire pointer to Dallas Temperature. 
  sensors = new DallasTemperature(oneWire);
  sensors->begin();
  sensors->getAddress(tempSensor, 0);
  sensors->setResolution(tempSensor, 12);
  sensors->setWaitForConversion(false);
  sensors->requestTemperatures(); // Start an asynchronous temperature reading

  randomSeed(52); 
}


/* ------------------------------------------------------------------------
 *  refresh
 */
void Cook::refresh() {
  const double minDelta = 0.1;
  double prevTemp = currentTemp;
  
  currentTemp = sensors->getTempC(tempSensor);
  sensors->requestTemperatures(); // prime the pump for the next one - but don't wait

  // the 1st iteration ends immediately
  if( prevTemp == 0 ) {
    Serial.println("Cook: 1st loop, ends here.");
    return;
  }
  
  // Slope
  int prevSlope = slope;
  if( currentTemp > (prevTemp+minDelta) ) {
    slope = 1;
  } else if( currentTemp < (prevTemp-minDelta) ) {
    slope = -1;
  } else {
    // keep the old temperature, and keep slope
    currentTemp = prevTemp;
  }
  //Serial.print("Cook: slope = ");
  //Serial.println(slope);

  // Reach the crest of the wave?
  if( prevSlope > 0 && slope <= 0 ) {
    waveCrest = currentTemp;
  }

  // Burner
  int newBurnerStatus = burnerStatus;
  if( currentTemp < goalTemp ) {
    newBurnerStatus = HIGH;
  }

  if( currentTemp > goalTemp ) {
    newBurnerStatus = LOW;
  }

  double inflectionPoint = goalTemp + ( (waveCrest - goalTemp) / 2 );  
  if( slope < 0 && currentTemp <= inflectionPoint ) {
    newBurnerStatus = HIGH;
  }

  if( prevSlope < 0 && slope >=0 ) {
    newBurnerStatus = LOW;    
  }

  if( newBurnerStatus != burnerStatus ) {
    burnerStatus = newBurnerStatus;
    digitalWrite(burnerPin, burnerStatus);
  }
  
  //Serial.print("Cook: brunerStatus = ");
  //Serial.println(burnerStatus);

  if( random(1,1000) == 5 ) {
    setAlarm("Zaraza");
  }
}


/* ------------------------------------------------------------------------
 *  adjustGoalTemp
 */
void Cook::adjustGoalTemp( double delta ) {
  goalTemp += delta;
}


/* ------------------------------------------------------------------------
 *  adjustGoalTime
 */
void Cook::adjustGoalTime( int hours, int minutes = 0 ) {
  goalTimeInSeconds += ( ( hours * 60 ) + minutes ) * 60;
}

/* ------------------------------------------------------------------------
 *  Alarm
 */

void Cook::setAlarm(String msg) {
  alarmMessage = msg;
  digitalWrite(alarmPin, HIGH);
}

bool Cook::alarm() {
  return alarmMessage.length() > 0;
}

String Cook::getAlarmMessage() {
  return alarmMessage;
}

void Cook::turnOffAlarm() {
  alarmMessage = "";
  digitalWrite(alarmPin, LOW);
}

    
/* ------------------------------------------------------------------------
 *  Proyectors
 */
int Cook::getSlope() {
  return slope;
}

double Cook::getGoalTemp() {
  return goalTemp;
}

double Cook::getCurrentTemp() {
  return currentTemp;
}

long Cook::getGoalTimeInSecs() {
  return goalTimeInSeconds;
}

long Cook::getCurrentTimeInSecs() {
  return currentTimeInSeconds;  
}
