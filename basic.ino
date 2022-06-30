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
int button1=PA4;
int button2=PA5;

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

int digit_1 = 0;

int score1=1000;
int score2=1000;
int score3=2000;


void printSegment(){
  digitalWrite(latch2, LOW);
  shiftOut(data2, clock2, MSBFIRST, segment[digit_1]);   //
  shiftOut(data2, clock2, MSBFIRST, segment[digit_1]);   //
  shiftOut(data2, clock2, MSBFIRST, segment[digit_1]);   //
  shiftOut(data2, clock2, MSBFIRST, segment[digit_1]);

  shiftOut(data2, clock2, MSBFIRST, segment[digit_1]);   //
  shiftOut(data2, clock2, MSBFIRST, segment[digit_1]);   //
  shiftOut(data2, clock2, MSBFIRST, segment[digit_1]);   //
  shiftOut(data2, clock2, MSBFIRST, segment[digit_1]);

  shiftOut(data2, clock2, MSBFIRST, segment[digit_1]);   //
  shiftOut(data2, clock2, MSBFIRST, segment[digit_1]);   //
  shiftOut(data2, clock2, MSBFIRST, segment[digit_1]);   //
  shiftOut(data2, clock2, MSBFIRST, segment[digit_1]);
  digitalWrite(latch2, HIGH);
  delay(50);
}

//todo sensors
int sens1=PB11;
int sens2=PB10;

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
      // Serial.println("sensorReady");
      if(currentSound != 1){
         currentSound = 1;
         myDFPlayer.play(1);
      }
      openSensor = true;
      sensor1Detected = false;
      sensorComplete = false;
      return true;
   }
   return false;
}


// the setup function runs once when you press reset or power the board
void setup() {
  // attachInterrupt(sens1,readSensor1,FALLING);
  // attachInterrupt(sens2,readSensor2,FALLING);
   
   pinMode(data1,OUTPUT);
   pinMode(latch1,OUTPUT);
   pinMode(clock1,OUTPUT);

   pinMode(data2,OUTPUT);
   pinMode(latch2,OUTPUT);
   pinMode(clock2,OUTPUT);

   pinMode(sens1,INPUT_PULLUP);
   pinMode(sens2,INPUT_PULLUP);
   pinMode(button1,INPUT_PULLUP);
   pinMode(button2,INPUT_PULLUP);
}


unsigned long tadi = 0;

// the loop function runs over and over again forever
void loop() {
  if(millis() - tadi > 500){
    tadi = millis();
    led++;
    if(led > 5){
      led=0;
    }
    /* digit_1++;
    if(digit_1 > 9){
      digit_1=0;
    } */
  }   
  

  bool inputsensor1 = digitalRead(sens1);
  bool inputsensor2 = digitalRead(sens2);
  //button1 = pa4
  bool inputButton1= digitalRead(button1);
  bool inputButton2= digitalRead(button2);
  if(inputsensor1 == LOW){
    digit_1 = 1;
  }
  else if(inputsensor2 == LOW){
    digit_1 = 2;
  }
  else if(inputButton1 == LOW){
    digit_1 = 4;
  }
  else if(inputButton2 == LOW){
    digit_1 = 5;
  }
  else{digit_1 = 0;}
 
  printSegment();
  printLedAnimation();
}
