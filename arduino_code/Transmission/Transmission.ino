//Adding Libraries 
#include <SPI.h>                        /* to handle the communication interface with the modem*/
#include <nRF24L01.h>                   /* to handle this particular modem driver*/
#include <RF24.h>                       /* the library which helps us to control the radio modem*/

#define MAX_Controller_Val 1024
#define PI 3.141592

#define RED_LED A0 
#define YELLOW_LED 10
#define GREEN_LED A1

#define Joy_Vert_Left A3
#define Joy_Horz_Left A2
#define Joy_Vert_Right A4
#define Joy_Horz_Right A5
#define Joy_Button_Left 4
#define Joy_Button_Right 2

#define Upper_Button 6
#define Lower_Button 5

#define Controller_LED 3

float frontback_L, leftright_L, frontback_R, leftright_R;
int magnitude_movement, angle_movement, magnitude_watch, angle_watch, operation_mode, left_joy_button, right_joy_button;

RF24 radio(8,7,0);                        /* Creating instance 'radio'  ( CE , CSN )   CE -> D7 | CSN -> D8 */                              
const byte Address[6] = "00009" ;     /* Address to which data to be transmitted*/
int value = 0, lightcycle = 0, LB = 0, RB = 0;
bool success = 0;
int sign = 1;                    // Assume sign is positive upon initialisation

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.print("Check");

  // Pin Definitions
  pinMode(RED_LED, OUTPUT);       // Red LED
  pinMode(YELLOW_LED, OUTPUT);    // Yellow LED
  pinMode(GREEN_LED, OUTPUT);     // Green LED

  // For all my buttons
  pinMode(Joy_Button_Right, INPUT_PULLUP);
  pinMode(Controller_LED, OUTPUT);             // This is for the LED
  pinMode(Joy_Button_Left, INPUT_PULLUP);
  pinMode(Lower_Button, INPUT);
  pinMode(Upper_Button, INPUT);

  // Joystick PinMode Initialisation
  const int  pins[]  = {A2, A3, A4, A5};
  const char* names[] = {"A2", "A3", "A4","A5"};
  const int  count   = sizeof(pins) / sizeof(pins[0]);

  for (int i = 0; i < count; i++) {
    pinMode(pins[i], INPUT);
  }

  // Cool Startup Sequence
  LEDstartup();

  radio.begin();                  /* Activate the modem*/
  radio.openWritingPipe(Address); /* Sets the address of transmitter to which program will send the data */
  radio.stopListening();          /* Setting modem in transmission mode*/

  /*
  radio.setRetries(1, 1);
  radio.startFastWrite(&value, sizeof(value), 0);

  while (success == 0) {
    LEDLightCycle();
    delay(200);
    if (radio.txStandBy()) {  // non-blocking check if write completed
      success = 1;
    } else {
      radio.startFastWrite(&value, sizeof(value), 0);  // retry
    }
  }
  */
}

void loop() {
  frontback_R = checksmall((MAX_Controller_Val/2)-analogRead(Joy_Vert_Right));
  leftright_R = checksmall(analogRead(Joy_Horz_Right)-(MAX_Controller_Val/2));

  frontback_L = checksmall((MAX_Controller_Val/2)-analogRead(Joy_Vert_Left));
  leftright_L = checksmall(analogRead(Joy_Horz_Left)-(MAX_Controller_Val/2));

  magnitude_movement = magnitude(leftright_R,frontback_R);
  angle_movement = angle(leftright_R,frontback_R,magnitude_movement);
  //debugPrintGPIO();
  //Serial.println(operation_mode);
  UpdateGPIO();

  // Pack my values for transmission
  value = sign * (angle_movement*10 + magnitude_movement) ;
  // put your main code here, to run repeatedly:
  radio.write(&value, sizeof(value));
  //Serial.print(value);

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
    angle = 180 - angle; sign = -1;
  } 
  else {
    angle = 180 + angle; sign = 1;
  }

  return angle;
}

void UpdateGPIO() {
  right_joy_button = digitalRead(Joy_Button_Right);
  left_joy_button  = digitalRead(Joy_Button_Left);
  LB = digitalRead(Lower_Button);
  RB = digitalRead(Upper_Button);

  if (LB == 1 && RB == 0) {
    operation_mode = 1;
    digitalWrite(RED_LED, 1);
    digitalWrite(YELLOW_LED, 0);
    digitalWrite(GREEN_LED, 0);
  }
  else if (LB == 0 && RB == 1) {
    operation_mode = 2;
    digitalWrite(RED_LED, 0);
    digitalWrite(YELLOW_LED, 1);
    digitalWrite(GREEN_LED, 0);
  }
  else {
    operation_mode = 0;
    digitalWrite(RED_LED, 0);
    digitalWrite(YELLOW_LED, 0);
    digitalWrite(GREEN_LED, 1);
  }

}

void LEDstartup() {
  LEDblink();
  LEDblink();
}

void LEDblink() {
  digitalWrite(RED_LED, 0);
  digitalWrite(YELLOW_LED,0);
  digitalWrite(GREEN_LED,0);
  delay(500);
  digitalWrite(RED_LED, 1);
  digitalWrite(YELLOW_LED,1);
  digitalWrite(GREEN_LED,1);
  delay(500);
}

void LEDLightCycle() {
  if (lightcycle == 0) {
    digitalWrite(RED_LED, 1);
    digitalWrite(YELLOW_LED,0);
    digitalWrite(GREEN_LED,0);
    lightcycle = lightcycle + 1;
  }
  else if (lightcycle == 1) {
    digitalWrite(RED_LED, 0);
    digitalWrite(YELLOW_LED,1);
    digitalWrite(GREEN_LED,0);
    lightcycle = lightcycle + 1;
  }
  else {
    digitalWrite(RED_LED, 0);
    digitalWrite(YELLOW_LED,0);
    digitalWrite(GREEN_LED,1);
    lightcycle = 0;
  }
  Serial.println(lightcycle);
  delay(500);
}

void debugPrintGPIO() {
  const int  pins[]  = {A0, A1, A2, A3, A4, A5};
  const char* names[] = {"A0", "A1", "A2", "A3", "A4","A5"};
  const int  count   = sizeof(pins) / sizeof(pins[0]);

  for (int i = 0; i < count; i++) {
    pinMode(pins[i], INPUT);
  }

  Serial.print("GPIO: ");
  for (int i = 0; i < count; i++) {
    Serial.print(names[i]);
    Serial.print("=");
    Serial.print(analogRead(pins[i]));
    if (i < count - 1) Serial.print("  ");
  }
  Serial.print("D2 = ");
  Serial.print(digitalRead(2));
  Serial.print(" D3 = ");
  Serial.print(digitalRead(3));
  Serial.print(" D4 = ");
  Serial.print(digitalRead(4));
  Serial.print(" D5 = ");
  Serial.print(digitalRead(5));
  Serial.print(" D6 = ");
  Serial.print(digitalRead(6));
  Serial.println();
}

