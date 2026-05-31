//Adding Libraries 
#include <SPI.h>                        /* to handle the communication interface with the modem*/
#include <nRF24L01.h>                   /* to handle this particular modem driver*/
#include <RF24.h>                       /* the library which helps us to control the radio modem*/

#define MAX_Controller_Val 1024
#define PI 3.141592

#define D0 0
#define D1 1
#define D2 2
#define D3 3
#define D4 4

float frontback1, leftright1;
int magnitude_1, angle_1;

RF24 radio(8,7);                        /* Creating instance 'radio'  ( CE , CSN )   CE -> D7 | CSN -> D8 */                              
const byte Address[6] = "00009" ;     /* Address to which data to be transmitted*/
int value = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  radio.begin ();                  /* Activate the modem*/
  radio.openWritingPipe (Address); /* Sets the address of transmitter to which program will send the data */
  Serial.print("Check");

}

void loop() {
  frontback1 = checksmall((MAX_Controller_Val/2)-analogRead(A1));
  leftright1 = checksmall(analogRead(A0)-(MAX_Controller_Val/2));
  magnitude_1 = magnitude(leftright1,frontback1);
  angle_1 = angle(leftright1,frontback1,magnitude_1);

  //debugPrintGPIO();

  // Pack my values for transmission
  value = angle_1*10 + magnitude_1;
  // put your main code here, to run repeatedly:
  radio.stopListening();                           /* Setting modem in transmission mode*/
  bool success = radio.write(&value, sizeof(value));                 /* Sending data over NRF 24L01*/
  Serial.print("Transmitting Data : ");
  Serial.println(value);                             /* Printing POT value on serial monitor*/

  //Serial.println(success ? "Send OK" : "Send FAILED");
} 

// Normalizer function for my data, it allows to correct for zero/displacement error
float checksmall(float value) {
  if (abs(value) < 10){
    value = 0;
  }
  else if (value > 510) {
    value = MAX_Controller_Val/2;
  }
  else if (value < -510) {
    value = -MAX_Controller_Val/2;
  }
  return value;
}

int magnitude(float x, float y) {
  int largest_val;
  if (abs(x) < abs(y)) {           // If x is smaller than y
    largest_val = (int)(4*(abs(y)/512)); // Take y as my largest value
  }
  else {
    largest_val = (int)(4*(abs(x)/512)); // Take x as my largest value
  }
  
  return largest_val;
}

int angle(float x, float y, int magnitude) {
  x = (double)x; y = (double)y;
  int angle;
  if (magnitude == 0) {
    angle = 0;
  }
  else {
    angle = (int)((atan2(y,x)*180/PI));
    if (angle < 180 && angle > -90) {
      angle = angle - 90;
    }
    else if (angle <= -90) {
      angle = angle + 270;
    }
    else {
      angle = 90;
    }
    
  }

  if (angle > 0) {
    angle = 180 - angle;
  } 
  else {
    angle = 180 + angle;
  }
         
  return angle;
}

void debugPrintGPIO() {
  const int  pins[]  = {A0, A1, A2, A3, A4};
  const char* names[] = {"D0", "D1", "D2", "D3", "D4"};
  const int  count   = sizeof(pins) / sizeof(pins[0]);

  for (int i = 0; i < count; i++) {
    pinMode(pins[i], INPUT);
  }
  pinMode(6, INPUT);
  pinMode(4, INPUT);
  pinMode(5, INPUT);

  Serial.print("GPIO: ");
  for (int i = 0; i < count; i++) {
    Serial.print(names[i]);
    Serial.print("=");
    Serial.print(analogRead(pins[i]));
    if (i < count - 1) Serial.print("  ");
  }
  Serial.print("D2 =");
  Serial.print(digitalRead(6));
  Serial.print("D3 =");
  Serial.print(digitalRead(4));
  Serial.print("D4 =");
  Serial.print(digitalRead(5));
  Serial.println();
}