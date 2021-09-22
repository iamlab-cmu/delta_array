#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <Adafruit_ADS1X15.h>

#define NUM_MOTORS 12

// Create the three motor shield object I2C address input
//Adafruit_MotorShield MC0 = Adafruit_MotorShield(0x61); 
Adafruit_MotorShield MC1 = Adafruit_MotorShield(0x60); 
//Adafruit_MotorShield MC2 = Adafruit_MotorShield(0x62); 

// Create 12 motor objects (4 motors on 3 shields)
//Adafruit_DCMotor *MC0_M1 = MC0.getMotor(1);
//Adafruit_DCMotor *MC0_M2 = MC0.getMotor(2);
//Adafruit_DCMotor *MC0_M3 = MC0.getMotor(3);
//Adafruit_DCMotor *MC0_M4 = MC0.getMotor(4);
Adafruit_DCMotor *MC1_M1 = MC1.getMotor(1);
Adafruit_DCMotor *MC1_M2 = MC1.getMotor(2);
Adafruit_DCMotor *MC1_M3 = MC1.getMotor(3);
Adafruit_DCMotor *MC1_M4 = MC1.getMotor(4);
//Adafruit_DCMotor *MC2_M1 = MC2.getMotor(1);
//Adafruit_DCMotor *MC2_M2 = MC2.getMotor(2);
//Adafruit_DCMotor *MC2_M3 = MC2.getMotor(3);
//Adafruit_DCMotor *MC2_M4 = MC2.getMotor(4);

// create array of pointers to motor objects
//Adafruit_DCMotor* motors[NUM_MOTORS] = {MC0_M1, MC0_M2, MC0_M3, MC0_M4, MC1_M1, MC1_M2, MC1_M3, MC1_M4, MC2_M1, MC2_M2, MC2_M3, MC2_M4};
Adafruit_DCMotor* motors[NUM_MOTORS] = {NULL, NULL, NULL, MC1_M1, MC1_M2, MC1_M3, MC1_M4, NULL, NULL, NULL, NULL, NULL}; //FLIPPED THE ORDER FOR M4 and M3

// create object to access off-board analog-to-digital converter
Adafruit_ADS1015 ADC0;
//Adafruit_ADS1015 ADC1;
//Adafruit_ADS1015 ADC2;

Adafruit_ADS1015* adcs[NUM_MOTORS] = {NULL, NULL, NULL, &ADC0, &ADC0, &ADC0, &ADC0, NULL, NULL, NULL, NULL, NULL};
//Adafruit_ADS1015* adcs[NUM_MOTORS] = {&ADC0, &ADC0, &ADC0, &ADC0, &ADC1, &ADC1, &ADC1, &ADC1, &ADC2, &ADC2, &ADC2, &ADC2};
int channels[NUM_MOTORS] = {0,0,0,3,2,1,0,0,0,0,0,0};
// Calculate based on max input size expected for one command
#define MAX_INPUT_SIZE 900

float last_joint_positions[12] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float joint_positions[12] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float joint_velocities[12] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
int motor_val[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

// STOP COMMAND
// Stop Command Globals
bool stop_flag = false;

// Stop Command
// Immediately halts everything by setting all the motor control pins to LOW and sets the stop_flag to true.
// Remember to publish a false msg to restart the robot.
void stop(bool stop_msg){

  // If the stop_msg contains true, stop all of the motors
  if(stop_msg)
  {
    // set stop_flag to true
    stop_flag = true;

    // Turn off all motors
    for(int i = 0; i < NUM_MOTORS; i++)
    {
      if(motors[i] != NULL){
        motors[i]->run(RELEASE);
      }
    }
  }
  // Otherwise if the stop_msg contains false, set the stop_flag back to false
  else
  {
    stop_flag = false;
  }
}

// Desired Motor Positions and Velocities (hard cap of 10 trajectory points)
float desired_joint_positions[10][12];
int desired_joint_speeds[10][12]; 
float desired_joint_velocities[10][12];
float durations[10];
float desired_motor_speed[10];

float last_time = 0.0;
int num_trajectory_points = 0;
int current_trajectory_point = 0;
bool position_trajectory = false;
bool velocity_trajectory = false;
bool speed_trajectory = false;
// Get next command from Serial (add 1 for final 0)
char inputString[MAX_INPUT_SIZE + 1];

bool newData = false;
int currentStringLength = 0;
int ndx = 0;
bool recvInProgress = false;
char startMarker = '<';
char endMarker = '>';

void recvWithStartEndMarkers() {
  char rc;
 
  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();

    if (recvInProgress == true) {
      if (rc != endMarker) {
        inputString[ndx] = rc;
        ndx++;
      }
      else {
        inputString[ndx] = '\0'; // terminate the string
        recvInProgress = false;
        currentStringLength = ndx;
        ndx = 0;
        newData = true;
      }
    }
    else if (rc == startMarker) {
      recvInProgress = true;
    }
  }
}

// Position Trajectory
// Stores the Position Trajectory to desired_joint_positions
void positionTrajectory()
{
  String received = "p ";

  Serial.println(received);

  char *strtokIndx; // this is used by strtok() as an index
  
  strtokIndx = strtok(inputString,",");      // get the first part - the string
  strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
  num_trajectory_points = atoi(strtokIndx);

  for(int i = 0; i < num_trajectory_points; i++)
  {
    strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
    durations[i] = atof(strtokIndx);
    received += String(durations[i]) + " ";
    
    for(int j = 0; j < 12; j++)
    {
      strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
      desired_joint_positions[i][j] = atof(strtokIndx);
      received += String(desired_joint_positions[i][j]) + " ";
    }

    Serial.println(received);
  }

  position_trajectory = true;
  velocity_trajectory = false;
  speed_trajectory = false;
  current_trajectory_point = 0;
  last_time = 0.0;
}

// Position with Speed Trajectory
// Stores the Position Trajectory to desired_joint_positions
// Stores the speed for each motor to desired_joint_speeds
void positionSpeedTrajectory()
{
  String received = "s ";

  Serial.println(received);

  char *strtokIndx; // this is used by strtok() as an index
  
  strtokIndx = strtok(inputString,",");      // get the first part - the string
  strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
  num_trajectory_points = atoi(strtokIndx);

  for(int i = 0; i < num_trajectory_points; i++)
  {
    //read in 12 speed commands
    for(int j = 0; j < 12; j++)
    {
      strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
      desired_joint_speeds[i][j] = atof(strtokIndx);
      received += String(desired_joint_speeds[i][j]) + " ";
    }
    //read in 12 position commands
    for(int j = 0; j < 12; j++)
    {
      strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
      desired_joint_positions[i][j] = atof(strtokIndx);
      received += String(desired_joint_positions[i][j]) + " ";
    }
    Serial.println(received);
  }

  position_trajectory = false;
  velocity_trajectory = false;
  speed_trajectory = true;
  current_trajectory_point = 0;
  last_time = 0.0;
}

// Velocity Trajectory
// Stores the Velocity Trajectory to desired_joint_velocities
void velocityTrajectory()
{
  char *strtokIndx; // this is used by strtok() as an index
  
  strtokIndx = strtok(inputString,",");      // get the first part - the string
  strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
  num_trajectory_points = atoi(strtokIndx);

  for(int i = 0; i < num_trajectory_points; i++)
  {
    strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
    durations[i] = atof(strtokIndx);
    for(int j = 0; j < 12; j++)
    {
      strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
      desired_joint_velocities[i][j] = atof(strtokIndx);
    }
  }
  position_trajectory = false;
  velocity_trajectory = true;
  speed_trajectory = false;
  current_trajectory_point = 0;
  last_time = 0.0;
}

float max_motor_speed[12] = {0.025, 0.025, 0.025, 0.025, 0.025, 0.025, 0.025, 0.025, 0.025, 0.025, 0.025, 0.025};

// RESET COMMAND
// Reset Command Callback
// Resets the robot's desired positions to the default positions.
void resetJoints(){

  num_trajectory_points = 1;
  durations[0] = 4.0;
  for(int i = 0; i < 12; i++)
  {
    desired_joint_positions[0][i] = 0.0;
  }

  position_trajectory = true;
  velocity_trajectory = false;
  current_trajectory_point = 0;
  last_time = 0.0;
}

unsigned long current_arduino_time;
unsigned long last_arduino_time;
float time_elapsed;
String joint_states;

// SETUP CODE
void setup()
{ 
  // set all the base dc motor control pins to outputs
  MC1.begin();  

  // start all the ADCs
  ADC0.begin(0x48);
//  ADC1.begin(0x49);
//  ADC2.begin(0x4B);

  ADC0.setGain(GAIN_ONE);
//  ADC1.setGain(GAIN_ONE);
//  ADC2.setGain(GAIN_ONE);

  // disable dc motors by setting their enable lines to low
  for(int i=0; i<NUM_MOTORS; i++){
    motors[i]->setSpeed(150);
    motors[i]->run(RELEASE);
  }

  Serial.begin(57600);           //  setup serial

  updateJointPositions();

  joint_states.reserve(200); // Reserve 200 Bytes for joint_states
}

void updateJointPositions()
{
  for(int i = 0; i < 12; i++)
  {
    if(motors[i] != NULL){
        motor_val[i] = adcs[i]->readADC_SingleEnded(channels[i]); 
        joint_positions[i] = motor_val[i] * 0.00006; // 100mm / 1650motors[i]->run(RELEASE);
    }
  }
}

void checkIfDoneMovingDeltas()
{
  if(current_trajectory_point > 0 && current_trajectory_point == num_trajectory_points)
  {
    position_trajectory = false;
    velocity_trajectory = false;
    current_trajectory_point = 0;
    num_trajectory_points = 0;
    last_time = 0.0;
    Serial.println("d");
    Serial.println("d");
  }
}

int sampleTime = 0;

// LOOP CODE
void loop()
{
  recvWithStartEndMarkers();

  if (newData) {

    Serial.println(inputString);
    switch (inputString[0]) {
      // Stop Command
      case 's':
        stop(inputString[2]-'0');  
        break;
      // Reset Command
      case 'r':
        resetJoints();
        break;
      // Position Command
      case 'p':
        positionTrajectory();
        break;
      // Velocity Command
      case 'v':
        velocityTrajectory();
        break;
      case 'm':
        positionSpeedTrajectory();
        break;
      default:
        // do nothing
        break;
    }
    newData = false;
  }

  // If the robot is not currently in the stop mode
  if(!stop_flag)
  {
    if(position_trajectory)
    {
      moveDeltaPosition();
    }
    else if(velocity_trajectory)
    {
      moveDeltaVelocity();
    }
    else if(speed_trajectory)
    {
      moveDeltaSpeed();
    }
    checkIfDoneMovingDeltas();
  }
  
  if (sampleTime == 0) {
    for(int i = 0; i < 12; i++)
    {
      last_joint_positions[i] = joint_positions[i];
    }
    updateJointPositions();

    current_arduino_time = millis();
    time_elapsed = float(current_arduino_time - last_arduino_time) / 1000.0;
    last_time += time_elapsed;
    for(int i = 0; i < 12; i++)
    {
      joint_velocities[i] = (joint_positions[i] - last_joint_positions[i]) / time_elapsed;
    }
    last_arduino_time = current_arduino_time;

    publishJointStates();
  }
  sampleTime += 1;
  sampleTime = sampleTime % 1;
}

float position_threshold = 0.00015;
float p = 90.0;
float i_pid = 0.25;

float d = 0.5;
float last_joint_errors[12] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float joint_errors[12] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float total_joint_errors[12] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

void moveDeltaPosition()
{
  bool reached_point = true;
  for(int i = 0; i < 12; i++)
  {
    joint_errors[i] = joint_positions[i] - desired_joint_positions[current_trajectory_point][i];
    if(fabs(joint_errors[i]) > position_threshold)
    {
      reached_point = false;
    }
  }
  if(reached_point)
  {
    current_trajectory_point += 1;
    for(int i = 0; i < 12; i++)
    {
      total_joint_errors[i] = 0.0;
    }
  }

  float time_left = max(durations[current_trajectory_point] - last_time, 0.1);

  if(current_trajectory_point < num_trajectory_points)
  {
    for(int i = 0; i < 12; i++)
    {
      if(motors[i] != NULL){
        if(joint_errors[i] > position_threshold)
        {
          float pid = p * joint_errors[i] + i_pid * total_joint_errors[i] + d * (joint_errors[i] - last_joint_errors[i]) / time_elapsed;
          int motor_speed = (int)(min(max(0.0, pid), 1.0) * 255.0); 
          motors[i]->setSpeed(motor_speed);
          motors[i]->run(BACKWARD);
          joint_velocities[i] = -max_motor_speed[i];
          if(joint_errors[i] < 0.01) {
            total_joint_errors[i] += joint_errors[i];
          }
        }
        else if(joint_errors[i] < -position_threshold)
        {
          float pid = p * joint_errors[i] + i_pid * total_joint_errors[i] + d * (joint_errors[i] - last_joint_errors[i]) / time_elapsed;
          int motor_speed = (int)(min(max(-1.0, pid), 0.0) * -255.0);
          motors[i]->setSpeed(motor_speed);
          motors[i]->run(FORWARD);
          joint_velocities[i] = max_motor_speed[i];
          if(joint_errors[i] > -0.01) {
            total_joint_errors[i] += joint_errors[i];
          }
        }
        else
        {
          motors[i]->setSpeed(0);
          motors[i]->run(RELEASE);
          joint_velocities[i] = 0.0;
          total_joint_errors[i] = 0.0;
        }
      }
    }
  }
  else
  {
    for(int i = 0; i < 12; i++)
    {
      if(motors[i] != NULL){
        motors[i]->setSpeed(0);
        motors[i]->run(RELEASE);
        joint_velocities[i] = 0.0;
        total_joint_errors[i] = 0.0;
      }
    }
  }
}

void moveDeltaSpeed()
{
  bool reached_point = true;
  for(int i = 0; i < 12; i++)
  {
    joint_errors[i] = joint_positions[i] - desired_joint_positions[current_trajectory_point][i];
    if(fabs(joint_errors[i]) > position_threshold)
    {
      reached_point = false;
    }
  }
  if(reached_point)
  {
    current_trajectory_point += 1;
    for(int i = 0; i < 12; i++)
    {
      total_joint_errors[i] = 0.0;
    }
  }

  float time_left = max(durations[current_trajectory_point] - last_time, 0.1);

  if(current_trajectory_point < num_trajectory_points)
  {
    for(int i = 0; i < 12; i++)
    {
      if(motors[i] != NULL){
        if(joint_errors[i] > position_threshold)
        {
          float pid = p * joint_errors[i] + i_pid * total_joint_errors[i] + d * (joint_errors[i] - last_joint_errors[i]) / time_elapsed;
          //int motor_speed = (int)(min(max(0.0, pid), 1.0) * 255.0); //change it to use duration input
          motors[i]->setSpeed((int)desired_joint_speeds[i]);
          motors[i]->run(BACKWARD);
          joint_velocities[i] = -max_motor_speed[i];
          if(joint_errors[i] < 0.01) {
            total_joint_errors[i] += joint_errors[i];
          }
        }
        else if(joint_errors[i] < -position_threshold)
        {
          float pid = p * joint_errors[i] + i_pid * total_joint_errors[i] + d * (joint_errors[i] - last_joint_errors[i]) / time_elapsed;
          //int motor_speed = (int)(min(max(-1.0, pid), 0.0) * -255.0);
          motors[i]->setSpeed((int)desired_joint_speeds[i]*-1);
          motors[i]->run(FORWARD);
          joint_velocities[i] = max_motor_speed[i];
          if(joint_errors[i] > -0.01) {
            total_joint_errors[i] += joint_errors[i];
          }
        }
        else
        {
          motors[i]->setSpeed(0);
          motors[i]->run(RELEASE);
          joint_velocities[i] = 0.0;
          total_joint_errors[i] = 0.0;
        }
      }
    }
  }
  else
  {
    for(int i = 0; i < 12; i++)
    {
      if(motors[i] != NULL){
        motors[i]->setSpeed(0);
        motors[i]->run(RELEASE);
        joint_velocities[i] = 0.0;
        total_joint_errors[i] = 0.0;
      }
    }
  }
}

void moveDeltaVelocity()
{
  if(last_time >= durations[current_trajectory_point])
  {
    current_trajectory_point += 1;
  } 
  if(current_trajectory_point < num_trajectory_points)
  {
    for(int i = 0; i < 12; i++)
    {
      if(motors[i] != NULL){
        joint_velocities[i] = desired_joint_velocities[current_trajectory_point][i];
        if(joint_velocities[i] < 0.0)
        {
          int motor_speed = (int)(min((-joint_velocities[i]/ max_motor_speed[i]), 1.0) * 255.0);
          motors[i]->setSpeed(motor_speed);
          motors[i]->run(BACKWARD);
        }
        else if(joint_velocities[i] > 0.0)
        {
          int motor_speed = (int)(min((joint_velocities[i]/ max_motor_speed[i]), 1.0) * 255.0);
          motors[i]->setSpeed(motor_speed);
          motors[i]->run(FORWARD);
        }
        else
        {
          motors[i]->setSpeed(0);
          motors[i]->run(RELEASE);
        }
      }
    }
  }
  else
  {
    for(int i = 0; i < 12; i++)
    {
      if(motors[i] != NULL){
        motors[i]->setSpeed(0);
        motors[i]->run(RELEASE);
        joint_velocities[i] = 0.0;
        if(joint_positions[i] < 0.0)
        {
          joint_positions[i] = 0.0;
        }
      }
    }
  }
}

void publishJointStates()
{
  joint_states = "j,";

  for(int i = 0; i < 12; i++)
  {
    joint_states += String(joint_positions[i],4) + "," + String(joint_velocities[i],4) + ",";
  }
  Serial.println(joint_states);
}
