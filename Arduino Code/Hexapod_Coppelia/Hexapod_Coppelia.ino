#define PI 3.141592                                                   // PI
#define SERVOMIN  125                                                 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  625                                                 // this is the 'maximum' pulse length count (out of 4096)
#define CoxaAddressStart 0                           // This is where the Microcontroller can start Sweepping through the Coxas
#define FemurAddressStart 0                          // This is where the Microcontroller can start Sweepping through the Femurs
#define TibiaAddressStart 6                          // This is where the Microcontroller can start Sweepping through the Tibias
#define CoxaTargAngle qf[0]
#define FemurTargAngle qf[1]
#define TibiaTargAngle qf[2]

                     // L1,L2,L3,R1,R2,R3
float CoxaAngles[6] =  { 0, 0, 0, 0, 0, 0};
float FemurAngles[6] = { 0, 0, 0, 0, 0, 0};
float TibiaAngles[6] = { 0, 0, 0, 0, 0, 0};

// SDA -> A4 SCL -> A5

int angleToPulse(int ang)                             //gets angle in degree and returns the pulse width
  {  int pulse = map(ang,0, 180, SERVOMIN,SERVOMAX);  // map angle of 0 to 180 to Servo min and Servo max 
     Serial.print("Angle: ");Serial.print(ang);
     Serial.print(" pulse: ");Serial.println(pulse);
     return pulse;
  }

void setup() {
  float servos2rest[3] = {rad(0),rad(0),rad(90)};
  Serial.begin(9600);
  Serial.println("16 channel Servo test!");
  board1.begin();
  board1.setPWMFreq(60);                  // Analog servos run at ~60 Hz updates

  pwm_response_ALL(0,0.5);
  pwm_response_CFT(servos2rest,0.5);
}

void loop() {
  /*
    for(int sweep=0; sweep<90; sweep++) { 
        for(int i=0; i<6; i++) { 
          board1.setPWM(i, 0, angleToPulse(sweep));
          }
        delay(100);
      }
    for(int sweep=90; sweep>0; sweep--) { 
        for(int i=0; i<6; i++) { 
          board1.setPWM(i, 0, angleToPulse(sweep));
          }
        delay(100);
      }
      */
    movement(4,rad(0));
  }

void movement(int magnitude, int angle) {
  float step = 10*magnitude/4;
  float movement_loop[8][3] = {
  // Coxa  ,Femur   ,Tibia
    {rad(0),rad(-40),rad(90)},    // Pick G1 Femur Up
    {rad(step),rad(-40),rad(90)}, // Shift G1 Coxa by step
    {rad(step),rad(0),rad(90)},   // Put G1 Femur Down
    {rad(0),rad(-40),rad(90)},    // Pick G2 Femur Up
    {rad(0),rad(0),rad(90)},      // Shift G1 Coxa Forwards
    {rad(step),rad(-40),rad(90)}, // Shift G2 Coxa by Step
    {rad(step),rad(0),rad(90)},   // Put G2 Femur Down
    {rad(0),rad(0),rad(90)},      // Shift G2 Femur Forwards
  };
  float group_sequence[8] = {0,0,0,1,0,1,1,1};
  for (int i = 0; i < 8; i++) {
    pwm_response_ALT_Group(movement_loop[i],0.5,group_sequence[i]);
  }
}

// num - Motors Used 
void pwm_response_ALL(float qf, float period) {
  // initial, final time and real time variable
  float  tf = period, current_angle;
  // steps for linspace resolution
  int steps = (int)(tf*60);
  // evaluate sinusoid at times for plotting
  for (int s = 0; s < steps; s++) {
    float t = 1/60*steps;
    for(int i = CoxaAddressStart; i < TibiaAddressStart; i++) {  // Set all my Coxa Servos to the specified angle
      current_angle = sinusoid(qf,CoxaAngles[i],PI/tf,t);        // Calculate Sinusoid Trajectory
      board1.setPWM(i, 0, angleToPulse(current_angle));
    }
    for(int i = FemurAddressStart; i < TibiaAddressStart; i++) {  // Set all my Femur Servos to the specified angle
      current_angle = sinusoid(qf,FemurAngles[i],PI/tf,t);         // Calculate Sinusoid Trajectory
      board2.setPWM(i, 0, angleToPulse(current_angle));
    }
    for(int i = TibiaAddressStart; i < 2*TibiaAddressStart+1; i++) {  // Set all my Tibia Servos to the specified angle
      current_angle = sinusoid(qf,TibiaAngles[i],PI/tf,t);            // Calculate Sinusoid Trajectory
      board1.setPWM(i, 0, angleToPulse(current_angle));
    }
  }
  for (int i = 0; i < 6; i++) { CoxaAngles[i]=qf; FemurAngles[i]=qf; TibiaAngles[i]=qf; }

}

void pwm_response_CFT(float qf[3], float period) {
  // initial, final time and real time variable
  float tf = period, current_angle;
  // steps for linspace resolution
  int steps = (int)(tf*60);
  // evaluate sinusoid at times for plotting
  for (int s = 0; s < steps; s++) {
    float t = 1/60*steps;
    for(int i = CoxaAddressStart; i < TibiaAddressStart; i++) {  // Set all my Coxa Servos to the specified angle
      current_angle = sinusoid(CoxaTargAngle,CoxaAngles[i],PI/tf,t);        // Calculate Sinusoid Trajectory
      board1.setPWM(i, 0, angleToPulse(current_angle));
    }
    for(int i = FemurAddressStart; i < TibiaAddressStart; i++) {  // Set all my Femur Servos to the specified angle
      current_angle = sinusoid(FemurTargAngle,FemurAngles[i],PI/tf,t);         // Calculate Sinusoid Trajectory
      board2.setPWM(i, 0, angleToPulse(current_angle));
    }
    for(int i = TibiaAddressStart; i < 2*TibiaAddressStart+1; i++) {  // Set all my Tibia Servos to the specified angle
      current_angle = sinusoid(TibiaTargAngle,TibiaAngles[i],PI/tf,t);            // Calculate Sinusoid Trajectory
      board1.setPWM(i, 0, angleToPulse(current_angle));
    }
  }
  for (int i = 0; i < 6; i++) { CoxaAngles[i]=CoxaTargAngle; FemurAngles[i]=FemurTargAngle; TibiaAngles[i]=TibiaTargAngle; }

}

void pwm_response_ALT_Group(float qf[3], float period, int group) { // Group 0 - L1; Group 1 - L2
  // initial, final time and real time variable
  float tf = period, current_angle;
  // steps for linspace resolution
  int steps = (int)(tf*60);
  // evaluate sinusoid at times for plotting
  for (int s = 0; s < steps; s++) {
    float t = 1/60*steps;
    for(int i = CoxaAddressStart+group; i < TibiaAddressStart; i += 2) {  // Set all my Coxa Servos to the specified angle
      current_angle = sinusoid(CoxaTargAngle,CoxaAngles[i],PI/tf,t);        // Calculate Sinusoid Trajectory
      board1.setPWM(i, 0, angleToPulse(current_angle));
    }
    for(int i = FemurAddressStart+group; i < TibiaAddressStart; i += 2) {  // Set all my Femur Servos to the specified angle
      current_angle = sinusoid(FemurTargAngle,FemurAngles[i],PI/tf,t);         // Calculate Sinusoid Trajectory
      board2.setPWM(i, 0, angleToPulse(current_angle));
    }
    for(int i = TibiaAddressStart+group; i < 2*TibiaAddressStart+1; i += 2) {  // Set all my Tibia Servos to the specified angle
      current_angle = sinusoid(TibiaTargAngle,TibiaAngles[i],PI/tf,t);            // Calculate Sinusoid Trajectory
      board1.setPWM(i, 0, angleToPulse(current_angle));
    }
  }
  for (int i = group; i < 6; i += 2) { CoxaAngles[i]=CoxaTargAngle; FemurAngles[i]=FemurTargAngle; TibiaAngles[i]=TibiaTargAngle; }

}

float sinusoid(float qf, float qo, float w, float t) {
  // frequency of sinusoid
  float q = 0.5*(qf - qo)*(1 - cos(w*t)) + qo;
  return q;
}