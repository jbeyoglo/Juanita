/********************************************************
   Half Wave Control

   Github: https://github.com/jbeyoglo/Juanita
   
   The idea is to cut off power to the burner as soon as 
   it reaches the target temperature. 
   The temperature will continue raising, stabilize and 
   finally start to fall, this algorithm will turn on 
   the bruner when the temperature is half way between 
   the higher point and the target temperature.
   
   https://photos.app.goo.gl/ZZD51qur71izphSH9
      
 ********************************************************/

#include <LiquidCrystal_I2C.h>
#include "Cook.h"

const int RelayPin = A6;
const int AlarmPin = A2;

const int buttonUp = D4;
const int buttonDown = D7;    
const int buttonLeft = D8;    
const int buttonRight = D5;    
const int buttonShift = D6;   

const int EncoderSwitch = D12;
const int EncoderS1 = D2;
const int EncoderS2 = D3;

int pinAstateCurrent = LOW;                // Current state of Pin A
int pinAStateLast = pinAstateCurrent;      // Last read value of Pin A
int motorPower = 0;
int prevMotorPower = 0;
const int MaxMotorPower = 20;
long timeEncoderSwitchded = 0;

const int MotorPWM1 = D9;
const int MotorPWM2 = D10;


Cook *cook;

// set the LCD address to 0x27 for a 20 chars and 4 line display
LiquidCrystal_I2C lcd(0x27,20,4);  

const uint8_t deg_sign[] = {
        0b00110,
        0b01001,
        0b01001,
        0b00110,
        0b00000,
        0b00000,
        0b00000,
        0b00000,
    };

const uint8_t up_sign[] = {
        0b00100,
        0b00100,
        0b01110,
        0b01110,
        0b11111,
        0b00100,
        0b00100,
        0b00100,
    };

const uint8_t down_sign[] = {
        0b00100,
        0b00100,
        0b00100,
        0b11111,
        0b01110,
        0b01110,
        0b00100,
        0b00100,
    };


/* ------------------------------------------------------------------------
 *  Setup
 */
void setup() {  
  Serial.begin(9600);
  Serial.println("Starting...");

  pinMode(AlarmPin, OUTPUT);
  pinMode(buttonUp, INPUT);
  pinMode(buttonDown, INPUT);
  pinMode(buttonLeft, INPUT);
  pinMode(buttonRight, INPUT);
  pinMode(buttonShift, INPUT);
  
  pinMode (EncoderSwitch, INPUT_PULLUP); 
  pinMode (EncoderS1, INPUT); 
  pinMode (EncoderS2, INPUT); 
  pinMode(MotorPWM1, OUTPUT);
  pinMode(MotorPWM2, OUTPUT);
  // Atach a CHANGE interrupt to PinB and exectute the update function when this change occurs.
  attachInterrupt(digitalPinToInterrupt(EncoderS2), updateEncoder, CHANGE);
  cook = new Cook(RelayPin, AlarmPin);
  Serial.println("Cook created");

  // Initialize LCD DiSplay 
  lcd.init();
  lcd.clear();
  lcd.backlight();
  lcd.setCursor(2,2);
  lcd.print("La Juanita v2.22");
  lcd.createChar (1, deg_sign);
  lcd.createChar (2, up_sign);
  lcd.createChar (3, down_sign);
  delay(1000);
}

/* ------------------------------------------------------------------------
 *  Loop
 */
void loop() {
  cook->refresh();  
  lcdRefresh(lcd, *cook);

  adjustBasedOnButtons( *cook );
  
  delay(200);

  
  /*
  digitalWrite(AlarmPin, HIGH);
  delay(2000);
  digitalWrite(AlarmPin, LOW);
  */
}

/* ------------------------------------------------------------------------
 *  updateEncoder
 */
void updateEncoder() {
  // ROTATION DIRECTION
  pinAstateCurrent = digitalRead(EncoderS1);    // Read the current state of Pin A
  
  // If there is a minimal movement of 1 step
  if ((pinAStateLast == LOW) && (pinAstateCurrent == HIGH) 
                             && outsideReboundTimeframe(timeEncoderSwitchded)) {
    
    if (digitalRead(EncoderS2) == HIGH) {      // If Pin B is HIGH
      motorPower += ( motorPower < MaxMotorPower ? 1 : 0 );
      Serial.println("Right");             // Print on screen
    } else {
      motorPower -= ( motorPower > -1 * MaxMotorPower ? 1 : 0 );
      Serial.println("Left");            // Print on screen
    }    
  }  
  pinAStateLast = pinAstateCurrent;        // Store the latest read value in the currect state variable
}


/* ------------------------------------------------------------------------
 *  LCD
 */
void lcdRefresh(LiquidCrystal_I2C &lcd, Cook &cook) {

  char floatBuffer[10], timeBuffer[10];
  char printBuffer[80];
  dtostrf( cook.getCurrentTemp(), 5, 1, floatBuffer);
  sprintf(printBuffer,"Temp:%s C   %s", floatBuffer,convertSecondsToHHMM((int)(millis()/1000),timeBuffer));
  lcd.setCursor(0,0);  
  lcd.print(printBuffer);
  lcd.setCursor(10,0);
  lcd.print ((char) 0x01);
  lcd.setCursor(13,0);
  switch( cook.getSlope() ) {
    case -1:
      lcd.print((char) 0x03);
      break;     
    case 1:
      lcd.print((char) 0x02);
      break;     
    default:    
      lcd.print("-");
  }

  // line 2: Mix power and direction
  if( motorPower == 0 ) {
    sprintf(floatBuffer, " Off ");
  } else {
    sprintf(floatBuffer, "%3d%% ", (100 * abs(motorPower) / MaxMotorPower) );
  }  
  sprintf(printBuffer,"Mix:  %s %s %s" 
      , (motorPower <= 0 ? "<==" : "   ") 
      , floatBuffer
      , (motorPower >= 0 ? "==>" : "   ") );
  lcd.setCursor(0,1);  
  lcd.print(printBuffer);
  
  // line 3: alarms
  lcd.setCursor(0,2);
  for( int i=0; i<20; i++) {
    lcd.setCursor(i,2);
    lcd.print("-");    
  }
  if( cook.alarm() ) {
    lcd.setCursor(3,2);
    lcd.print( " " + cook.getAlarmMessage().substring(0,14) + " ");
  }
  
  // line 4: setup
  dtostrf( cook.getGoalTemp(), 5, 1, floatBuffer);
  sprintf(printBuffer, "Goal:%s C ~ %s", floatBuffer, convertSecondsToHHMM(cook.getGoalTimeInSecs(),timeBuffer) );
  lcd.setCursor(0,3);  
  lcd.print(printBuffer); 
  lcd.setCursor(10,3);
  lcd.print ((char) 0x01);
}


char* convertSecondsToHHMM( long seconds, char *buffer  ) {
  int minutes = seconds / 60;
  sprintf( buffer, "%02d:%02d", minutes/60, minutes%60 );
  return buffer;
}


/* ------------------------------------------------------------------------
 *  Buttons
 */
void adjustBasedOnButtons( Cook &cook) {

  int shiftState = digitalRead(buttonShift);
  bool otherButton = false;
  
  int buttonState = digitalRead(buttonUp);
  if (buttonState == HIGH) {
    otherButton = true;
    cook.adjustGoalTemp( (shiftState==HIGH ? 1.0 : 0.1) );
  } 

  buttonState = digitalRead(buttonDown);
  if (buttonState == HIGH) {
    otherButton = true;
    cook.adjustGoalTemp( (shiftState==HIGH ? -1.0 : -0.1) );
  } 

  buttonState = digitalRead(buttonRight);
  if (buttonState == HIGH) {
    otherButton = true;
    if( shiftState==HIGH ) {
      cook.adjustGoalTime(1);      
    } else {
      cook.adjustGoalTime(0,1);      
    }
  } 

  buttonState = digitalRead(buttonLeft);
  if (buttonState == HIGH) {
    otherButton = true;
    if( shiftState==HIGH ) {
      cook.adjustGoalTime(-11);      
    } else {
      cook.adjustGoalTime(0,-1);      
    }
  } 

  if( shiftState==HIGH && !otherButton ) {
    cook.turnOffAlarm();
  }

  if( digitalRead(EncoderSwitch) == LOW && outsideReboundTimeframe(timeEncoderSwitchded) ) {    
    Serial.println("SWITCH ON!!");
    //char printBuffer[80];    
    //sprintf(printBuffer, "SWITCH ON == motor: %d - prev: %d", motorPower, prevMotorPower);
    //Serial.println(printBuffer);
    
    timeEncoderSwitchded = millis();
    // resume?
    if( motorPower == 0 && prevMotorPower != 0 ) {
      motorPower = prevMotorPower;
      prevMotorPower = 0;      
    } else {
      prevMotorPower = motorPower;
      motorPower = 0;
    }
  }

  // bridgeH that moves the mixing motor, 2 x pwm signals
  int map0to255 = 0;
  if( motorPower == 0 || motorPower > MaxMotorPower || motorPower < -1 * MaxMotorPower ) {
    analogWrite( MotorPWM1, 0 );
    analogWrite( MotorPWM2, 0 );
  } else if( motorPower > 0 ) {
    analogWrite( MotorPWM1, 0 );
    map0to255 = map( motorPower, 0, MaxMotorPower, 0, 255 );
    analogWrite( MotorPWM2, map0to255 );
  } else {
    analogWrite( MotorPWM2, 0 );
    map0to255 = map( motorPower, 0, -1 * MaxMotorPower, 0, 255 );
    analogWrite( MotorPWM1, map0to255 );    
  }
  
}

/* ------------------------------------------------------------------------
 *  didPassMoreThanASecondSince
 */
bool outsideReboundTimeframe( long prevMillis ) {
  return ( prevMillis < (millis() - 1000) );
}
