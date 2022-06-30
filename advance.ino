/*
SENS1.B11
SENS2.B10

DATA1.B1
LATCH1.B0
CLOCK1.A7
NONE1.A6

DATA1.A3
LATCH1.A2
CLOCK1.A1
NONE1.A0

LEDBUTTON1.C15
LEDBUTTON2.C14

INPUTBUTTON1.A4
INPUTBUTTON1.A5

DF RX.A9
DF TX.A10
 */

#include "Arduino.h"
#include "SoftwareSerial.h"
#include "DFRobotDFPlayerMini.h"
 
//todo audio
int dfrx=PA9;
int dftx=PA10;

SoftwareSerial mySoftwareSerial(dfrx, dftx); // RX, TX
DFRobotDFPlayerMini myDFPlayer;
void printDetail(uint8_t type, int value);
int currentSound;


//todo button

int button2=PA5;

bool lastButton = true;
bool currentState;
bool wasclicked = false;

void readButton(){
   currentState = digitalRead(button2);
   if(currentState != lastButton){
      if(!currentState && !wasclicked){
         tadi = millis();
         //Serial.println("cliked");
         wasclicked = true;
         }
   }
}




//todo reg 1.
int data1= PB1;
int latch1= PA7;
int clock1= PB0;

int none1= PA6;

const byte segment[]{
  B11101110,
  B00100100,
  B11010110,
  B01110110,//3
  B00111100,
  B01111010,
  B11111010,
  B00100110,
  B11111110,
  B01111110
};
const byte leds[]{
  B00000000,
  B10000000,
  B11000000,
  B11100000,
  B11110000,
  B11111000
};

int led=0;
void printLedAnimation(){
  digitalWrite(latch1, LOW);
  shiftOut(data1, clock1, MSBFIRST, leds[led]);
  shiftOut(data1, clock1, MSBFIRST, leds[led]);
  shiftOut(data1, clock1, MSBFIRST, leds[led]);
  shiftOut(data1, clock1, MSBFIRST, leds[led]);
  digitalWrite(latch1, HIGH);
}


//todo reg 2.
int data2 = PA2;
int latch2= PA1;
int clock2= PA3;

int none2= PA0;

int hasilscore1 = 1234;
int hasilscore2 = 5678;
int hasilscore3 = 8989;

int score1[]= {0,0,0,0};
int score2[]= {0,0,0,0};
int score3[]= {1,1,1,1};

int convertScoreToDigit(){
  score1[3] = (hasilscore1 / 10) % 10;
  score1[2] = (hasilscore1 / 100) % 10;
  score1[1] = (hasilscore1 / 1000) % 10;
  score1[0] = (hasilscore1 / 10000) % 10;

  score2[3] = (hasilscore2 / 10) % 10;
  score2[2] = (hasilscore2 / 100) % 10;
  score2[1] = (hasilscore2 / 1000) % 10;
  score2[0] = (hasilscore2 / 10000) % 10;

  score3[3] = (hasilscore3 / 10) % 10;
  score3[2] = (hasilscore3 / 100) % 10;
  score3[1] = (hasilscore3 / 1000) % 10;
  score3[0] = (hasilscore3 / 10000) % 10;
}

void printSegment(){
  digitalWrite(latch2, LOW);
  shiftOut(data2, clock2, MSBFIRST, segment[score1[3]]);   //
  shiftOut(data2, clock2, MSBFIRST, segment[score1[2]]);   //
  shiftOut(data2, clock2, MSBFIRST, segment[score1[1]]);   //
  shiftOut(data2, clock2, MSBFIRST, segment[score1[0]]);

  shiftOut(data2, clock2, MSBFIRST, segment[score1[3]]);   //
  shiftOut(data2, clock2, MSBFIRST, segment[score1[2]]);   //
  shiftOut(data2, clock2, MSBFIRST, segment[score1[1]]);   //
  shiftOut(data2, clock2, MSBFIRST, segment[score1[0]]);

  shiftOut(data2, clock2, MSBFIRST, segment[score1[3]]);   //
  shiftOut(data2, clock2, MSBFIRST, segment[score1[2]]);   //
  shiftOut(data2, clock2, MSBFIRST, segment[score1[1]]);   //
  shiftOut(data2, clock2, MSBFIRST, segment[score1[0]]);
  digitalWrite(latch2, HIGH);
  delay(50);
}

//todo sensors
int sens2=PB11;
int sens1=PB10;

unsigned long timeSensor1;
unsigned long timeResult;
bool openSensor = false;
bool sensor1Detected = false;
bool sensorComplete = false;

void readSensor1(){
  if(openSensor){
    timeSensor1 = millis();
    sensor1Detected = true;
    // Serial.print("time 1: ");
    // Serial.println(timeSensor1);
  }
}
unsigned long maxTime;
unsigned long minTime;
void readSensor2(){
   if (sensor1Detected && openSensor){
      timeResult = millis() - timeSensor1;
      if (timeResult > maxTime){
         //Serial.println("to late");
         timeResult = maxTime;
      }
      else if (timeResult < minTime){
         timeResult = minTime;
      }
      sensorComplete = true;
      openSensor = false;
      // Serial.print("timeResult: ");
      // Serial.println(timeResult);
   }
}
void autoReset(){
  if(sensor1Detected && !sensorComplete){
    if(millis()-timeSensor1>1000){
      sensor1Detected = false;
    }
  }
}
unsigned long lastTimeSensor;
bool sensorIsReady(){
   if(millis()- lastTimeSensor > 500){
      openSensor = true;
      sensor1Detected = false;
      sensorComplete = false;
      return true;
   }
   return false;
}


// the setup function runs once when you press reset or power the board
void setup() {
   attachInterrupt(sens1,readSensor1,FALLING);
   attachInterrupt(sens2,readSensor2,FALLING);
   pinMode(data1,OUTPUT);
   pinMode(latch1,OUTPUT);
   pinMode(clock1,OUTPUT);

   pinMode(data2,OUTPUT);
   pinMode(latch2,OUTPUT);
   pinMode(clock2,OUTPUT);

}

unsigned long tadi = 0;

// the loop function runs over and over again forever
int GAMEMODE = 0;
void loop() {
/*   if(millis() - tadi > 500){
    tadi = millis();
    led++;
    if(led > 5){
      led=0;
    }
  } */   
  // GAME
  //todo
  // press button
  // null scores
  // wait hit
  // if triggered display score blink
  // read button
  // wait hit
  // if triggered display score 2
  // jeda
  // score 3 display
  switch (GAMEMODE)
  {
  case 0:
    readButton();
    if(wasclicked){
      if(sensorIsReady()){
        wasclicked=false;
        GAMEMODE++;
      }
    }
    break;
  case 1:
    if(sensorComplete){
      if(sensorIsReady()){
        GAMEMODE++;
      }
    }
    break;
  case 3:
    readButton();
    if(sensorIsReady()){
        wasclicked=false;
        GAMEMODE++;
      }
    break;
  case 4:
    if(sensorComplete){
        GAMEMODE++;
    }
    break;
  case 5:
    readButton();
    if(wasclicked){
      wasclicked=false;
      GAMEMODE=0;
    }
    break;
      
  default:
    break;
  }
 
  
  printSegment();
  printLedAnimation();
}
