// ARDUINO MEGA PIN LAYOUT DEFINES
//#define 0
//#define 1
#define Motor1En 2 // PWM Pin
#define Motor2En 3 // PWM Pin
#define Motor3En 4 // PWM Pin
#define Motor4En 5 // PWM Pin
#define Motor5En 6 // PWM Pin
#define Motor6En 7 // PWM Pin
#define Motor7En 8 // PWM Pin
#define Motor8En 9 // PWM Pin
#define Motor9En 10 // PWM Pin
#define Motor10En 11 // PWM Pin
#define Motor11En 12 // PWM Pin
#define Motor12En 13 // PWM Pin
//#define 14
//#define 15
//#define 16
//#define 17
//#define 18 // Interrupt Pin
//#define 19 // Interrupt Pin
//#define 20 // Interrupt Pin, SDA
//#define 21 // Interrupt Pin, SCL
//#define 22
//#define 23
//#define 24
//#define 25
//#define 26
//#define 27
//#define 28
//#define 29
#define Motor1In1 30
#define Motor1In2 31
#define Motor2In1 32
#define Motor2In2 33
#define Motor3In1 34 
#define Motor3In2 35
#define Motor4In1 36
#define Motor4In2 37
#define Motor5In1 38
#define Motor5In2 39
#define Motor6In1 40
#define Motor6In2 41
#define Motor7In1 42
#define Motor7In2 43
#define Motor8In1 44
#define Motor8In2 45
#define Motor9In1 46
#define Motor9In2 47
#define Motor10In1 48
#define Motor10In2 49
#define Motor11In1 50
#define Motor11In2 51
#define Motor12In1 52
#define Motor12In2 53

#define Motor1A A1 // Analog Pin
#define Motor2A A2 // Analog Pin
#define Motor3A A3 // Analog Pin
#define Motor4A A4 // Analog Pin
#define Motor5A A5 // Analog Pin
#define Motor6A A6 // Analog Pin
#define Motor7A A7 // Analog Pin
#define Motor8A A8 // Analog Pin
#define Motor9A A9 // Analog Pin
#define Motor10A A10 // Analog Pin
#define Motor11A A11 // Analog Pin
#define Motor12A A12 // Analog Pin

// Calculate based on max input size expected for one command
#define MAX_INPUT_SIZE 900

float last_joint_positions[12] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float joint_positions[12] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float joint_velocities[12] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
int motor_val[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

int motor_en[12] = {Motor1En, Motor2En, Motor3En, Motor4En, Motor5En, Motor6En,
                    Motor7En, Motor8En, Motor9En, Motor10En, Motor11En, Motor12En};

int motor_in1[12] = {Motor1In1, Motor2In1, Motor3In1, Motor4In1, Motor5In1, Motor6In1,
                     Motor7In1, Motor8In1, Motor9In1, Motor10In1, Motor11In1, Motor12In1};

int motor_in2[12] = {Motor1In2, Motor2In2, Motor3In2, Motor4In2, Motor5In2, Motor6In2,
                     Motor7In2, Motor8In2, Motor9In2, Motor10In2, Motor11In2, Motor12In2};

int motor_a[12] = {Motor1A, Motor2A, Motor3A, Motor4A, Motor5A, Motor6A,
                   Motor7A, Motor8A, Motor9A, Motor10A, Motor11A, Motor12A};

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

    // Turn off motors
    for(int i = 0; i < 12; i++)
    {
      analogWrite(motor_en[i], 0);
      digitalWrite(motor_in1[i], LOW);
      digitalWrite(motor_in2[i], LOW);
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
float desired_joint_velocities[10][12];
float durations[10];
float last_time = 0.0;
int num_trajectory_points = 0;
int current_trajectory_point = 0;
bool position_trajectory = false;
bool velocity_trajectory = false;

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
  for(int i = 0; i < 12; i++)
  {
    pinMode(motor_en[i], OUTPUT);
    pinMode(motor_in1[i], OUTPUT);
    pinMode(motor_in2[i], OUTPUT);
    pinMode(motor_a[i], INPUT) ;
  }

  // disable dc motors by setting their enable lines to low
  for(int i = 0; i < 12; i++)
  {
    analogWrite(motor_en[i], 0);
    digitalWrite(motor_in1[i], LOW);
    digitalWrite(motor_in2[i], LOW);
  }

  Serial.begin(57600);           //  setup serial

  updateJointPositions();

  joint_states.reserve(200); // Reserve 200 Bytes for joint_states
}

void updateJointPositions()
{
  for(int i = 0; i < 12; i++)
  {
    motor_val[i] = analogRead(motor_a[i]);
    joint_positions[i] = motor_val[i] * 0.0000978; // 100mm / 1023
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
        stop(atoi(inputString[2]));  
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
float p = 250.0;
float i_pid = 8.0;
float d = 0.0;
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
      if(joint_errors[i] > position_threshold)
      {
        float pid = p * joint_errors[i] + i_pid * total_joint_errors[i] + d * (joint_errors[i] - last_joint_errors[i]) / time_elapsed;
        int motor_speed = (int)(min(max(0.0, pid), 1.0) * 255.0);
        analogWrite(motor_en[i], motor_speed);
        digitalWrite(motor_in1[i], LOW);
        digitalWrite(motor_in2[i], HIGH);
        joint_velocities[i] = -max_motor_speed[i];
        if(joint_errors[i] < 0.01) {
          total_joint_errors[i] += joint_errors[i];
        }
      }
      else if(joint_errors[i] < -position_threshold)
      {
        float pid = p * joint_errors[i] + i_pid * total_joint_errors[i] + d * (joint_errors[i] - last_joint_errors[i]) / time_elapsed;
        int motor_speed = (int)(min(max(-1.0, pid), 0.0) * -255.0);
        analogWrite(motor_en[i], motor_speed);
        digitalWrite(motor_in1[i], HIGH);
        digitalWrite(motor_in2[i], LOW);
        joint_velocities[i] = max_motor_speed[i];
        if(joint_errors[i] > -0.01) {
          total_joint_errors[i] += joint_errors[i];
        }
      }
      else
      {
        analogWrite(motor_en[i], 0);
        digitalWrite(motor_in1[i], LOW);
        digitalWrite(motor_in2[i], LOW);
        joint_velocities[i] = 0.0;
        total_joint_errors[i] = 0.0;
      }
    }
  }
  else
  {
    for(int i = 0; i < 12; i++)
    {
      analogWrite(motor_en[i], 0);
      digitalWrite(motor_in1[i], LOW);
      digitalWrite(motor_in2[i], LOW);
      joint_velocities[i] = 0.0;
      total_joint_errors[i] = 0.0;
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
      joint_velocities[i] = desired_joint_velocities[current_trajectory_point][i];
      if(joint_velocities[i] < 0.0)
      {
        int motor_speed = (int)(min((-joint_velocities[i]/ max_motor_speed[i]), 1.0) * 255.0);
        analogWrite(motor_en[i], motor_speed);
        digitalWrite(motor_in1[i], LOW);
        digitalWrite(motor_in2[i], HIGH);
      }
      else if(joint_velocities[i] > 0.0)
      {
        int motor_speed = (int)(min((joint_velocities[i]/ max_motor_speed[i]), 1.0) * 255.0);
        analogWrite(motor_en[i], motor_speed);
        digitalWrite(motor_in1[i], HIGH);
        digitalWrite(motor_in2[i], LOW);
      }
      else
      {
        analogWrite(motor_en[i], 0);
        digitalWrite(motor_in1[i], LOW);
        digitalWrite(motor_in2[i], LOW);
      }
    }
  }
  else
  {
    for(int i = 0; i < 12; i++)
    {
      analogWrite(motor_en[i], 0);
      digitalWrite(motor_in1[i], LOW);
      digitalWrite(motor_in2[i], LOW);
      joint_velocities[i] = 0.0;
      if(joint_positions[i] < 0.0)
      {
        joint_positions[i] = 0.0;
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
