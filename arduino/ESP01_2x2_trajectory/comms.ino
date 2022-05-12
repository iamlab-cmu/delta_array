//############################ SERIAL COMM FUNCTIONS #######################################3
void updateTrajectory(){
  traj_iter = 0;
  go = true;
}

void printIPAddr(){
  while (Serial1.available() > 0){
    char ch = Serial1.read();
    Serial.write(ch);
  }
}

void add_new_rc(byte rc,bool ifend){
  if(!ifend){
    marker_count++;
  }else{
    marker_count = 0;
  }
  input_cmd[ndx] = rc;
  ndx++;
  if (ndx >= numChars) {
    ndx = numChars - 1;
  }
}

void recvWithStartEndMarkers() {
  byte rc;
  while (Serial1.available() > 0 && newData == false) {
    rc = Serial1.read();
//    Serial.println(rc);
    if (recvInProgress == true) {
      if (rc == endMarker) {
        add_new_rc(rc, false);
      } 
      else if(rc == confMarker){
        add_new_rc(rc, false);
        if(marker_count == 3){
          marker_count = 0;
          input_cmd[ndx] = '\0'; // terminate the string
          recvInProgress = false;
          newData = true;
          ndx -= 3;
          Serial.println(ndx);
        }
      } 
      else {
        add_new_rc(rc, true);
      }
    }

    else if (rc == startMarker) {
      marker_count++;
    }
    else if (rc == confMarker) {
      marker_count++;
      if(marker_count == 3){
        marker_count = 0;
        recvInProgress = true; 
      }
    }
    else{
      marker_count = 0;
    }
  }
}

//
//void sendJointPositions(){
//  pb_ostream_t ostream = pb_ostream_from_buffer(input_cmd, sizeof(input_cmd));
//  message.id = MY_ID;
//  for(int i = 0; i < NUM_MOTORS; i++){
//    motor_val[i] = adcs[i]->readADC_SingleEnded(channels[i]); 
//    message.trajectory[i] = motor_val[i] * 0.00006; // 100mm / 1650
////    Serial.print(message.joint_pos[i]);
//  }
//  if (pb_encode(&ostream, DeltaMessage_fields, &message)){
//    Serial1.print("AT+CIPSEND=0,");Serial1.println(ostream.bytes_written +8);
//    delay(10);
//    printIPAddr();
//    delay(10);
//    Serial1.write("~~~");
//    Serial1.write(input_cmd, ostream.bytes_written);
//    Serial1.write("~~~\r\n");
//  }
//}

//bool temp_decode_nanopb(){
//  for(int i=0; i<ndx; i++){
//    Serial.print(input_cmd[i]);Serial.print(" ");
//  }
//  Serial.println();Serial.println(ndx);
//  return false;
//}

bool decodeNanopbData(){
  pb_istream_t istream = pb_istream_from_buffer(input_cmd, ndx);
  bool ret = pb_decode(&istream, DeltaMessage_fields, &message);
  if (message.id == MY_ID){
    ret = true;
  
    if (message.request_done_state){
//        sendJointPositions();
    }
    else if (message.reset){
      for (int i=0; i<NUM_MOTORS; i++){
        new_joint_positions[i] = 0.05;
      }
    }
    else{
      for(int i=0; i<20; i++){
        for(int j=0; j<12; j++){
          trajectory[i][j] = message.trajectory[i*12 + j];
//          Serial.print(message.trajectory[i*12 + j],4);Serial.print(" ");
        }
//        Serial.println();
      }
    }
  }
  else{
    ret = false;
  }
  return ret;
}
