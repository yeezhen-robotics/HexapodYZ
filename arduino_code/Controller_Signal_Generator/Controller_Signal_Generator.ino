#define MAX_Controller_Val 1024
#define PI 3.141592

float frontback1, leftright1, frontback2, leftright2;
int magnitude_1, angle_1, magnitude_2, angle_2;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600,SERIAL_8E1);

}

void loop() {
  // put your main code here, to run repeatedly:
  frontback1 = checksmall((MAX_Controller_Val/2)-analogRead(A1));
  leftright1 = checksmall(analogRead(A0)-(MAX_Controller_Val/2));
  frontback2 = checksmall((MAX_Controller_Val/2)-analogRead(A2));
  leftright2 = checksmall(analogRead(A3)-(MAX_Controller_Val/2));
  magnitude_1 = magnitude(leftright1,frontback1);
  angle_1 = angle(leftright1,frontback1,magnitude_1);
  magnitude_2 = magnitude(leftright2,frontback2);
  angle_2 = angle(leftright2,frontback2,magnitude_2);
  Serial.print("Magnitude1: "); Serial.print(magnitude_1);
  Serial.print(" Angle1: "); Serial.println(angle_1);
  Serial.print("Magnitude2: "); Serial.print(magnitude_2);
  Serial.print(" Angle2: "); Serial.println(angle_2);
  //Serial.print(" A0: "); Serial.print(frontback1);
  //Serial.print(" A1: "); Serial.print(leftright1);
  //Serial.print(" A2: "); Serial.print(frontback2);
  //Serial.print(" A3: "); Serial.println(leftright2);
  delay(1000);
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
  return angle;
}