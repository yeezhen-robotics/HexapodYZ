//// Adding Libraries 
#include <Adafruit_PWMServoDriver.h>
#include <SPI.h>             /* to handle the communication interface with the modem*/
#include <nRF24L01.h>        /* to handle this particular modem driver*/
#include <RF24.h>            /* the library which helps us to control the radio modem*/

// Electronics Interface //

Adafruit_PWMServoDriver board1 = Adafruit_PWMServoDriver(0x40);       // called this way, it uses the default address 0x40 
Adafruit_PWMServoDriver board2 = Adafruit_PWMServoDriver(0x41);       // called this way, it uses the default address 0x41
RF24 radio(4,5);             /* Creating instance 'radio'  ( CE , CSN )   CE -> D7 | CSN -> D8 */                               
const byte Address[6] = "00009"; /* Address from which data to be received */

// Electronics Interface END //

// Defines //

#define PI 3.141592                        // PI
#define SERVOMIN  125                      // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  625                      // this is the 'maximum' pulse length count (out of 4096)
#define CoxaAddressStart 0                 // This is where the Microcontroller can start Sweepping through the Coxas
#define FemurAddressStart 3                // This is where the Microcontroller can start Sweepping through the Femurs
#define TibiaAddressStart 6                // This is where the Microcontroller can start Sweepping through the Tibias
#define TibiaAddressEnd 9
#define TrueHomePosition 90
#define CoxaTargAngle qf[0]                // Defined for pwm_response_ALT_Group
#define FemurTargAngle qf[1]               // Defined for pwm_response_ALT_Group
#define TibiaTargAngle qf[2]               // Defined for pwm_response_ALT_Group
#define CoxaTargAngleUNISON qf[group][0]   // Defined for pwm_response_UNISON_Group        
#define FemurTargAngleUNISON qf[group][1]  // Defined for pwm_response_UNISON_Group
#define TibiaTargAngleUNISON qf[group][2]  // Defined for pwm_response_UNISON_Group

// Defines END //

// Variable Declarations //

                       // L1,L2,L3,R1,R2,R3
float CoxaAngles[6] =  { 90, 90, 90, 90, 90, 90};
float FemurAngles[6] = { 90, 90, 90, 90, 90, 90};
float TibiaAngles[6] = { 90, 90, 90, 90, 90, 90};

char CoxaAngleNames[6][3] = {{"LC1"},{"LC2"},{"LC3"},{"RC1"},{"RC2"},{"RC3"}};
char FemurAngleNames[6][3] = {{"LF1"},{"LF2"},{"LF3"},{"RF1"},{"RF2"},{"RF3"}};
char TibiaAngleNames[6][3] = {{"LT1"},{"LT2"},{"LT3"},{"RT1"},{"RT2"},{"RT3"}};

// Variable Initialisation
int rx_data = 0, angle = 0, magnitude = 0;  // Variable to store received data
float step = 10*rx_data/4;
float group_sequence[7] = {0,0,1,0,1,0,1};  // Sequence corresponding to the movement loop


int movement_step_index = 0; // The index to simulate for loop

// Variable Declarations END //

// SDA -> A4 SCL -> A5
int angleToPulse(int ang)                             //gets angle in degree and returns the pulse width
  {  int pulse = map(ang,0, 180, SERVOMIN,SERVOMAX);  // map angle of 0 to 180 to Servo min and Servo max 
     //Serial.print("Angle: ");Serial.print(ang);
     //Serial.print(" pulse: ");Serial.println(pulse);
     return pulse;
  }

// Resets the robot to the home position
void home_position() {
  for(int i=0; i<9; i++) { 
    board1.setPWM(i, 0, angleToPulse(90));
  }
  for(int i=0; i<9; i++) { 
    board2.setPWM(i, 0, angleToPulse(90));
  }
}

float servos2rest[3] = {0,0,90};

void setup() {
  // Peripheral Initialisation
  Serial.begin(9600);
  Serial.println("Stop");
  board1.begin();                    // B1 Turn On
  board1.setPWMFreq(60);             // Analog servos run at ~60 Hz updates
  board2.begin();                    // B2 Turn On 
  board2.setPWMFreq(60);             // Analog servos run at ~60 Hz updates
  radio.begin();                     //Activate the modem
  radio.openReadingPipe(1, Address); //Sets the address of receiver from which program will receive the data
  
  home_position();                   // Recenter all arduino code
  delay(1000);
  stand();
  Serial.println("Initialised");
  radio.startListening();			               //Setting modem in Receiver mode
  
  //Serial.println(CoxaAngleNames[1]);
  //pwm_response_CFT(servos2rest,0.5);
  }

void loop() {
  //movement(5 ,0.0);
  //delay(100);
  //arise();
  //float angles[3] = {90,90,90};
  //pwm_response_ALT_Group(angles,10.0,0);

  if (radio.available()) {
    while (radio.available()) {              // Loop until receiving valid data
      radio.read(&rx_data, sizeof(rx_data)); // Read the received data and store in ' rx_data ' 
      Serial.print("Received Data : ");
      Serial.println(rx_data);               // Print received value on Serial Monitor;
      break;
    }
  }
  else {
    //Serial.println("Not Receiving !!!");      // If not receiving valid data print " Not Receiving !!! " on Serial Monitor
    rx_data = 0;
  }
  
  angle = floor((double)(rx_data/10));
  magnitude = rx_data - angle*10;
  int direction;
  if (abs(angle) >= 160) {
    direction = -1;
  } 
  else {
    direction = 1;
  }

  int turn = 0;
  if (angle >= 45 and angle <160) {
    turn = 1;
  }
  else if (angle <= -45 and angle > 160) {
    turn = -1;
  }
  else {
    turn = 0;
  }

  //movement(rx_data*2,0.0);
  step = direction * 20 * magnitude / 4; 

  // THIS HAS TO REMAIN AFTER STEP SO THAT THE VARIABLE UPDATES ITS MOVEMENT SIZE EVERY TIMESTEP TO ADJUST TO DIFFERENT SPEEDS (NOT INCLUDING THIS HERE WILL MAKE THE ROBOT STALL)
  float movement_loop[7][3] = {
  // Coxa  ,Femur   ,Tibia
    {90-step,90,85},    // Pick G1 Femur Up & Shift G1 Coxa by step
    {90-step,50,70},    // Put G1 Femur Down
    {90     ,90,85},    // Pick G2 Femur Up
    {90+step,50,70},    // Shift G1 Coxa Forwards
    {90     ,50,70},    // Shift G2 Coxa by Step & Put G2 Femur Down
    {90     ,90,85},    // Pick G1 Femur Up
    {90+step,50,70},    // Shift G2 Femur Forwards
  }; 

  if (magnitude > 0) {
    //Serial.println(movement_step_index);
    if (group_sequence[movement_step_index] == 2) {

      print_servo_state(movement_loop[movement_step_index], "Going to ->");   // angles now reflect post-move state
      pwm_response_UNISON_Group(movement_loop[movement_step_index],movement_loop[movement_step_index+1],0.5);

      movement_step_index += 2;
    }
    else {
      pwm_response_ALT_Group(movement_loop[movement_step_index],group_sequence[movement_step_index],0.5,turn);
      movement_step_index++;
    } 
  }

  if (movement_step_index > 6) {
    movement_step_index = 0;
  }

}

// Resets the robot to the home position
void test() {
  for(int i=0; i<9; i++) { 
    board1.setPWM(i, 0, angleToPulse(50));
  }
  for(int i=0; i<9; i++) { 
    board2.setPWM(i, 0, angleToPulse(50));
  }
}

// Makes the robot go off the ground to begin walking
void stand() {

  float standing_angles[3] = {90,50,70};

  pwm_response_CFT(standing_angles, 2.0);
}

// Makes the robot go off the ground to begin walking
void arise() {

  float standing_angles[3] = {90,90,90};

  pwm_response_CFT(standing_angles, 2.0);
}

void movement(int magnitude, int angle) {
  float step = 10*magnitude/4;
  Serial.println(step);
  float movement_loop[8][3] = {
  // Coxa  ,Femur   ,Tibia
    {90-step,65,85},    // Pick G1 Femur Up & Shift G1 Coxa by step
    {90-step,50,70},    // Put G1 Femur Down
    {90     ,65,85},    // Pick G2 Femur Up
    {90     ,50,70},    // Shift G1 Coxa Forwards
    {90-step,50,70},    // Shift G2 Coxa by Step & Put G2 Femur Down
    {90     ,65,85},    // Pick G1 Femur Up
    {90     ,50,70},    // Shift G2 Femur Forwards
    {90     ,50,70},    // Put G1 Femur Down
  };
  if (magnitude > 0) {
    float group_sequence[10] = {0,0,1,0,1,0,1,0};
    for (int i = 0; i < 8; i++) {
      pwm_response_ALT_Group(movement_loop[i],group_sequence[i],2.0,0);
    }
  }  
}

// num - Motors Used 
// Sets all of the motor positions to the same final value
void pwm_response_ALL(float qf, float period) {
  // initial, final time and real time variable
  float tf = period, current_angle, t;
  // steps for linspace resolution
  int steps = (int)(tf*60);
  // evaluate sinusoid at times for plotting
  for (int s = 0; s < steps; s++) {
    t = s*0.0167; // 1/60 Approx. 0.016666...
    for(int i = CoxaAddressStart; i < TibiaAddressStart; i++) {  // Set all my Coxa Servos to the specified angle
      current_angle = sinusoid(qf,CoxaAngles[i],PI/tf,t);        // Calculate Sinusoid Trajectory
      dispmatrixrows(CoxaAngleNames,CoxaAngles);
    }
    for(int i = FemurAddressStart; i < TibiaAddressStart; i++) {  // Set all my Femur Servos to the specified angle
      current_angle = sinusoid(qf,FemurAngles[i],PI/tf,t);         // Calculate Sinusoid Trajectory
      dispmatrixrows(FemurAngleNames,FemurAngles);
    }
    for(int i = TibiaAddressStart; i < 2*TibiaAddressStart+1; i++) {  // Set all my Tibia Servos to the specified angle
      current_angle = sinusoid(qf,TibiaAngles[i],PI/tf,t);            // Calculate Sinusoid Trajectory
      dispmatrixrows(TibiaAngleNames,TibiaAngles);
    }
    delay(1000); // Match CoppeliaSim's 0.1s timing
  }
  for (int i = 0; i < 6; i++) { CoxaAngles[i]=qf; FemurAngles[i]=qf; TibiaAngles[i]=qf; }

}

// Sets all of the motor positions of the same group (CFT) to the same final value
// qf[3] should only be between ranges of 0-90 (Home Position)
void pwm_response_CFT(float qf[3], float period) {
  // initial, final time and real time variable
  float tf = period, current_angle_LC, current_angle_RC, current_angle_LF, current_angle_RF, current_angle_LT, current_angle_RT, t;
  // steps for linspace resolution
  int steps = (int)(tf*60);
  // evaluate sinusoid at times for plotting
  for (int s = 0; s < steps; s++) {

    t = s*0.0167; // 1/60 Approx. 0.016666...

    for(int i = CoxaAddressStart; i < FemurAddressStart; i++) {       // Set all my Coxa Servos to the specified angle
      current_angle_LC = sinusoid(CoxaTargAngle,CoxaAngles[i],PI/tf,t);                    // Calculate Sinusoid Trajectory
      current_angle_RC = sinusoid(convert4symmetry(CoxaTargAngle),CoxaAngles[i+3],PI/tf,t);  // Calculate Sinusoid Trajectory
      board1.setPWM(i, 0, angleToPulse(current_angle_LC));      // Angle for the left side
      board2.setPWM(i, 0, angleToPulse(current_angle_RC));      // Angle for the right side
      //dispmatrixrows(CoxaAngleNames,i,current_angle);
    }

    for(int i = FemurAddressStart; i < TibiaAddressStart; i++) {      // Set all my Femur Servos to the specified angle
      current_angle_LF = sinusoid(FemurTargAngle,FemurAngles[i-3],PI/tf,t);                    // Calculate Sinusoid Trajectory
      current_angle_RF = sinusoid(convert4symmetry(FemurTargAngle),FemurAngles[i],PI/tf,t);  // Calculate Sinusoid Trajectory
      board1.setPWM(i, 0, angleToPulse(current_angle_LF));      // Angle for the left side
      board2.setPWM(i, 0, angleToPulse(current_angle_RF));      // Angle for the right side
      //Serial.print("End: ");
      //Serial.print(FemurTargAngle);
      //Serial.print("Start: ");
      //Serial.println(FemurAngles[i-3]);
      //dispmatrixrows(FemurAngleNames,i,current_angle);
    }

    for(int i = TibiaAddressStart; i < TibiaAddressEnd; i++) {  // Set all my Tibia Servos to the specified angle
      current_angle_LT = sinusoid(TibiaTargAngle,TibiaAngles[i-6],PI/tf,t);// Calculate Sinusoid Trajectory
      current_angle_RT = sinusoid(convert4symmetry(TibiaTargAngle),TibiaAngles[i-3],PI/tf,t);  // Calculate Sinusoid Trajectory
      board1.setPWM(i, 0, angleToPulse(current_angle_LT));      // Angle for the left side
      board2.setPWM(i, 0, angleToPulse(current_angle_RT));      // Angle for the right side
      //dispmatrixrows(TibiaAngleNames,i,current_angle);
    }

  }

  for (int i = 0; i < 3; i++) { 
    CoxaAngles[i]=CoxaTargAngle; 
    CoxaAngles[i+3]=convert4symmetry(CoxaTargAngle); 
    FemurAngles[i]=FemurTargAngle; 
    FemurAngles[i+3]=convert4symmetry(FemurTargAngle);
    TibiaAngles[i]=TibiaTargAngle; 
    TibiaAngles[i+3]=convert4symmetry(TibiaTargAngle);
  }
  Serial.println("|----------------------------|");
}

// Sets all of the motor positions of the same alternate group to the same final value
void pwm_response_ALT_Group(float qf[3], int group, float period, int turn) { // Group 0 - L1; Group 1 - L2 | For Direction, 0 = forwards, 1 = turn
  // initial, final time and real time variable
  float tf = period, current_angle_LC, current_angle_RC, current_angle_LF, current_angle_RF, current_angle_LT, current_angle_RT, t;
  // steps for linspace resolution
  int steps = (int)(tf*60);
  // evaluate sinusoid at times for plotting

  for (int s = 0; s < steps; s++) {
    t = s*0.0167; // 1/60 Approx. 0.016666...
    
    // Left Side of Hexapod (Coxa, Femur and Tibia)
    for(int i = CoxaAddressStart+group; i < FemurAddressStart; i += 2) {     // Set all my L_Coxa Servos to the specified angle
      int CoxaTrueTargAngle = turn == 1 ? convert4symmetry(CoxaTargAngle) : CoxaTargAngle;
      current_angle_LC = sinusoid(CoxaTrueTargAngle,CoxaAngles[i],PI/tf,t);      // Calculate Sinusoid Trajectory
      board1.setPWM(i, 0, angleToPulse(current_angle_LC));                   // Angle for the left side
      CoxaAngles[i]=current_angle_LC;
      //Serial.print("LC:");
      //Serial.println(current_angle_LC);
    }
    for(int i = FemurAddressStart+group; i < TibiaAddressStart; i += 2) {    // Set all my L_Femur Servos to the specified angle
      current_angle_LF = sinusoid(FemurTargAngle,FemurAngles[i-3],PI/tf,t);    // Calculate Sinusoid Trajectory
      board1.setPWM(i, 0, angleToPulse(current_angle_LF));                   // Angle for the left side 
      FemurAngles[i-3]=current_angle_LF;
      //Serial.print("LF:");
      //Serial.println(FemurAngles[i]);
      //Serial.println(current_angle_LF);
    }
    for(int i = TibiaAddressStart+group; i < TibiaAddressEnd; i += 2) {      // Set all my L_Tibia Servos to the specified angle
      current_angle_LT = sinusoid(TibiaTargAngle,TibiaAngles[i-6],PI/tf,t);    // Calculate Sinusoid Trajectory
      board1.setPWM(i, 0, angleToPulse(current_angle_LT));                   // Angle for the left side 
      TibiaAngles[i-6]=current_angle_LT;
      //Serial.print("LT:");
      //Serial.println(current_angle_LT);
    }

    // Right Side of Hexapod (Coxa, Femur and Tibia)
    for(int i = CoxaAddressStart+!group; i < FemurAddressStart; i += 2) { // Set all my R_Coxa Servos to the specified angle
      int CoxaTrueTargAngle = turn == -1 ? CoxaTargAngle : convert4symmetry(CoxaTargAngle);
      current_angle_RC = sinusoid(CoxaTrueTargAngle,CoxaAngles[i+3],PI/tf,t); // Calculate Sinusoid Trajectory
      board2.setPWM(i, 0, angleToPulse(current_angle_RC));                        // Angle for the left side
      CoxaAngles[i+3]=current_angle_RC;
      //Serial.print("RC:");
      //Serial.println(current_angle_RC); 
    }
    for(int i = FemurAddressStart+!group; i < TibiaAddressStart; i += 2) {        // Set all my R_Femur Servos to the specified angle
      current_angle_RF = sinusoid(convert4symmetry(FemurTargAngle),FemurAngles[i],PI/tf,t); // Calculate Sinusoid Trajectory
      board2.setPWM(i, 0, angleToPulse(current_angle_RF));                        // Angle for the left side
      FemurAngles[i]=current_angle_RF;
      //Serial.print("RF:");
      //Serial.println(current_angle_RF); 
    }
    for(int i = TibiaAddressStart+!group; i < TibiaAddressEnd; i += 2) {          // Set all my R_Tibia Servos to the specified angle
      current_angle_RT = sinusoid(convert4symmetry(TibiaTargAngle),TibiaAngles[i-3],PI/tf,t); // Calculate Sinusoid Trajectory
      board2.setPWM(i, 0, angleToPulse(current_angle_RT));                          // Angle for the left side
      TibiaAngles[i-3]=current_angle_RT; 
      //Serial.print("RT:");
      //Serial.println(current_angle_RT);
    }
    delay(17);
  }
}

// Moves two groups of motors in a tripod gait in sync to speed up walk cycle
void pwm_response_UNISON_Group(float qf1[3], float qf2[3], float period) { // Group 0 - L1; Group 1 - L2 
  // initial, final time and real time variable
  float tf = period, current_angle_LC, current_angle_RC, current_angle_LF, current_angle_RF, current_angle_LT, current_angle_RT, t;
  // steps for linspace resolution
  int steps = (int)(tf*60);
  // evaluate sinusoid at times for plotting

  float qf[2][3] = {
                      {qf1[0], qf1[1], qf1[2]},  // Group 1
                      {qf2[0], qf2[1], qf2[2]},  // Group 2
                    };

  for (int s = 0; s < steps; s++) {
    t = s*0.0167; // 1/60 Approx. 0.016666...
    Serial.println("Cycle Start");
    for (int group = 0; group < 2; group++) {
      // Left Side of Hexapod (Coxa, Femur and Tibia)
      for(int i = CoxaAddressStart+group; i < FemurAddressStart; i += 2) {     // Set all my L_Coxa Servos to the specified angle
        current_angle_LC = sinusoid(qf[group][0],CoxaAngles[i],PI/tf,t);      // Calculate Sinusoid Trajectory
        board1.setPWM(i, 0, angleToPulse(current_angle_LC));                   // Angle for the left side
        CoxaAngles[i]=current_angle_LC;
        //Serial.print("LC:");
        //Serial.println(current_angle_LC);
      }
      for(int i = FemurAddressStart+group; i < TibiaAddressStart; i += 2) {    // Set all my L_Femur Servos to the specified angle
        current_angle_LF = sinusoid(qf[group][1],FemurAngles[i-3],PI/tf,t);    // Calculate Sinusoid Trajectory
        board1.setPWM(i, 0, angleToPulse(current_angle_LF));                   // Angle for the left side 
        FemurAngles[i-3]=current_angle_LF;
        //Serial.print("LF:");
        //Serial.println(FemurAngles[i]);
        //Serial.println(current_angle_LF);
      }
      for(int i = TibiaAddressStart+(1-group); i < TibiaAddressEnd; i += 2) {      // Set all my L_Tibia Servos to the specified angle
        current_angle_LT = sinusoid(qf[group][2],TibiaAngles[i-6],PI/tf,t);    // Calculate Sinusoid Trajectory
        board1.setPWM(i, 0, angleToPulse(current_angle_LT));                   // Angle for the left side 
        TibiaAngles[i-6]=current_angle_LT;
        //Serial.print("LT:");
        //Serial.println(current_angle_LT);
      }

      // Right Side of Hexapod (Coxa, Femur and Tibia)
      for(int i = CoxaAddressStart+(1-group); i < FemurAddressStart; i += 2) { // Set all my R_Coxa Servos to the specified angle
        current_angle_RC = sinusoid(convert4symmetry(qf[group][0]),CoxaAngles[i+3],PI/tf,t);      // Calculate Sinusoid Trajectory
        board2.setPWM(i, 0, angleToPulse(current_angle_RC));                   // Angle for the left side
        CoxaAngles[i+3]=current_angle_RC;
        //Serial.print("RC:");
        //Serial.println(current_angle_RC); 
      }
      for(int i = FemurAddressStart+(1-group); i < TibiaAddressStart; i += 2) {    // Set all my R_Femur Servos to the specified angle
        current_angle_RF = sinusoid(convert4symmetry(qf[group][1]),FemurAngles[i],PI/tf,t);    // Calculate Sinusoid Trajectory
        board2.setPWM(i, 0, angleToPulse(current_angle_RF));                   // Angle for the left side
        FemurAngles[i]=current_angle_RF;
        //Serial.print("RF:");
        //Serial.println(current_angle_RF);
      }
      for(int i = TibiaAddressStart+!group; i < TibiaAddressEnd; i += 2) {      // Set all my R_Tibia Servos to the specified angle
        current_angle_RT = sinusoid(convert4symmetry(qf[group][2]),TibiaAngles[i-3],PI/tf,t);    // Calculate Sinusoid Trajectory
        board2.setPWM(i, 0, angleToPulse(current_angle_RT));                   // Angle for the left side
        TibiaAngles[i-3]=current_angle_RT; 
        //Serial.print("RT:");
        //Serial.println(current_angle_RT);
      }
    }
    delay(17);
  }
}

float sinusoid(float qf, float qo, float w, float t) {
  // frequency of sinusoid
  float q = 0.5*(qf - qo)*(1 - cos(w*t)) + qo;
  return q;
}

float rad(int degrees) {
  return degrees*PI/180;
}

float convert4symmetry(float angle) {
  return 180-angle;

}

void dispmatrixrows(char name[6][3], float matrix[6]) {
  for (int i; i < 6; i++) {
    for (int j; j < 3; j++) {
        Serial.print(name[i][j]);
    }
    Serial.println(matrix[i]); 
  }
}

void print_servo_state(float target[3], const char* label) {
  Serial.println(label);
  Serial.println("Joint  | Before | Target");
  Serial.print("Coxa   | "); Serial.print(CoxaAngles[0]); Serial.print(" | "); Serial.println(target[0]);
  Serial.print("Femur  | "); Serial.print(FemurAngles[0]); Serial.print(" | "); Serial.println(target[1]);
  Serial.print("Tibia  | "); Serial.print(TibiaAngles[0]); Serial.print(" | "); Serial.println(target[2]);
}