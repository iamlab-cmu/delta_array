
float p = 300.0;
float i_pid = 0.5;
float d = 4;

float position_threshold = 0.00035;
//############################# READ / WRITE JOINT POSITIONS #######################################3
void executeTrajectory(){
  if(traj_iter < 20 && go){
    memcpy(new_joint_positions, trajectory[traj_iter], 12);
    writeJointPositions();
    traj_iter += 1; 
  }
}

void readJointPositions(){
  for(int i = 0; i < NUM_MOTORS; i++){
    motor_val[i] = adcs[i]->readADC_SingleEnded(channels[i]); 
    joint_positions[i] = motor_val[i] * 0.00006; // 100mm / 1650
  }
}

void writeJointPositions(){
  bool reached_point = false;
  last_arduino_time = millis();
  is_movement_done = false;
  while (!reached_point){
    current_arduino_time = millis();
    time_elapsed = float(current_arduino_time - last_arduino_time)/1000;
    last_arduino_time = current_arduino_time;
    readJointPositions();
    reached_point = true;
    for(int i = 0; i < NUM_MOTORS; i++){
      joint_errors[i] = joint_positions[i] - new_joint_positions[i];
      
//      float pid = p * joint_errors[i] + i_pid * total_joint_errors[i] + d * (joint_errors[i] - last_joint_err/ors[i]/time_elapsed);
      float pid = p * joint_errors[i] + i_pid * total_joint_errors[i] + d * (joint_errors[i] - last_joint_errors[i])/time_elapsed;
      
      int motor_speed = (int)(min(max(-255.0, pid* 255.0), 255.0));
      motors[i]->setSpeed(abs(motor_speed));
      total_joint_errors[i] += joint_errors[i];
      last_joint_errors[i] = joint_errors[i];

      if(abs(joint_errors[i]) < position_threshold){
        last_joint_errors[i] = 0;
        reached_point &= true;
        motors[i]->setSpeed(0);
        motors[i]->run(RELEASE);
        total_joint_errors[i] = 0.0;
      }
      else if(motor_speed>0){
        reached_point &= false;
        motors[i]->run(BACKWARD);
      }
      else{
        reached_point &= false;
        motors[i]->run(FORWARD);
      }
      
    }
  }
  for(int i = 0; i < 12; i++){
      motors[i]->setSpeed(0);
      motors[i]->run(RELEASE);
      total_joint_errors[i] = 0.0;
  }
  is_movement_done = true;
   
//  Serial.println("Moved to New Position");
}

//########################### STOP OR RESET FUNCTIONS ######################################3
void resetJoints(){
  for(int i = 0; i < NUM_MOTORS; i++)
  {
    new_joint_positions[i] = 0.0;
  }
  writeJointPositions();
}

void stop(){
  // Turn off all motors
  for(int i = 0; i < NUM_MOTORS; i++)
  {
    motors[i]->run(RELEASE);
  }
}
