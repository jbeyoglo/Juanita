/********************************************************
   Half Wave Control
   
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

const int EncoderSwitch = D12;
const int EncoderS1 = D2;
const int EncoderS2 = D3;

const int buttonUp = D4;
const int buttonDown = D7;    
const int buttonLeft = D8;    
const int buttonRight = D5;    
const int buttonShift = D6;   

Cook *cocinero;

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
  // Atach a CHANGE interrupt to PinB and exectute the update function when this change occurs.
  attachInterrupt(digitalPinToInterrupt(EncoderS2), update, CHANGE);
  cocinero = new Cook(RelayPin);
  Serial.println("Cocinero creado.");

  // Initialize LCD DiSplay 
  lcd.init();
  lcd.clear();
  lcd.backlight();
  lcd.setCursor(2,2);
  lcd.print("La Juanita v2.1");
  lcd.createChar (1, deg_sign);
  lcd.createChar (2, up_sign);
  lcd.createChar (3, down_sign);
  delay(1000);
}

int pinAstateCurrent = LOW;                // Current state of Pin A
int pinAStateLast = pinAstateCurrent;      // Last read value of Pin A

/* ------------------------------------------------------------------------
 *  Loop
 */
void loop() {
  cocinero->refresh();  
  lcdRefresh(lcd, *cocinero);

  adjustBasedOnButtons( *cocinero );
  
  delay(200);

  if( digitalRead(EncoderSwitch) == LOW ) {
    Serial.println("SWITCH ON!!");
  }

/*
  // ROTATION DIRECTION
  pinAstateCurrent = digitalRead(EncoderS1);    // Read the current state of Pin A
  // If there is a minimal movement of 1 step
  if ((pinAStateLast == LOW) && (pinAstateCurrent == HIGH)) {    
    if (digitalRead(EncoderS2) == HIGH) {      // If Pin B is HIGH
      Serial.println("Left");             // Print on screen
    } else {
      Serial.println("Right");            // Print on screen
    }
  }
  pinAStateLast = pinAstateCurrent;        // Store the latest read value in the currect state variable
  */
  
  /*
  digitalWrite(AlarmPin, HIGH);
  delay(2000);
  digitalWrite(AlarmPin, LOW);
  */
}

void update() {

  /* WARNING: For this example I've used Serial.println within the interrupt callback. The Serial 
   * library already uses interrupts which could cause errors. Therefore do not use functions 
   * of the Serial libray in your interrupt callback.
   */

  // ROTATION DIRECTION
  pinAstateCurrent = digitalRead(EncoderS1);    // Read the current state of Pin A
  
  // If there is a minimal movement of 1 step
  if ((pinAStateLast == LOW) && (pinAstateCurrent == HIGH)) {
    
    if (digitalRead(EncoderS2) == HIGH) {      // If Pin B is HIGH
      Serial.println("Right");             // Print on screen
    } else {
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

  lcd.setCursor(0,2);
  for( int i=0; i<20; i++) {
    lcd.setCursor(i,2);
    lcd.print("-");    
  }
  
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

  int buttonState = digitalRead(buttonUp);
  if (buttonState == HIGH) {
    cook.adjustGoalTemp( (shiftState==HIGH ? 1.0 : 0.1) );
  } 

  buttonState = digitalRead(buttonDown);
  if (buttonState == HIGH) {
    cook.adjustGoalTemp( (shiftState==HIGH ? -1.0 : -0.1) );
  } 

  buttonState = digitalRead(buttonRight);
  if (buttonState == HIGH) {
    if( shiftState==HIGH ) {
      cook.adjustGoalTime(1);      
    } else {
      cook.adjustGoalTime(0,1);      
    }
  } 

  buttonState = digitalRead(buttonLeft);
  if (buttonState == HIGH) {
    if( shiftState==HIGH ) {
      cook.adjustGoalTime(-11);      
    } else {
      cook.adjustGoalTime(0,-1);      
    }
  } 
}
