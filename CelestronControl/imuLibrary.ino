void queryIMU(){

    /* Get a new sensor event */
    sensors_event_t event;
    bno.getEvent(&event);

    if(bnoVerbose){
      /* Display the floating point data */
      Serial.print("X: ");
      Serial.print(getIMUPos(AZM), 4);
      Serial.print('(');
      Serial.print(imuAzmOffset, 4);
      Serial.print(")\tY: ");
      Serial.print(getIMUPos(ROLL), 4);
      Serial.print("\tZ: ");
      Serial.print(getIMUPos(ALT), 4);
      Serial.print('(');
      Serial.print(imuAltOffset, 4);
      Serial.print(')');
      displayCalStatus();

      /* New line for the next sample */
      Serial.println("");
    }    
}

double getIMUPos(char axis){
  sensors_event_t event;
  bno.getEvent(&event);

  if(axis == ALT) return constrain( ((-event.orientation.z)-90.0) + imuAltOffset, -90.0, 90.0 ) ; //Get altitude reading from IMU, shifted to account for how the IMU actually reports values
  if(axis == AZM) return constrain( event.orientation.x + imuAzmOffset, 0.0, 360.0 ); //Get azimuth reading, reporting in the proper range after the offset is added
  if(axis == ROLL) return event.orientation.y; //Get roll reading, which is pretty useless but still nice to report
  return 0.0;
}


void saveCalibrationConstants(){

  if(bno.isFullyCalibrated()){
    Serial.println("Calibration Results: ");
    adafruit_bno055_offsets_t newCalib;
    bno.getSensorOffsets(newCalib);
    displaySensorOffsets(newCalib);

    Serial.println("\n\nStoring calibration data to EEPROM...");

    int eeAddress = 0;
    sensor_t sensor;
    bno.getSensor(&sensor);
    long bnoID = sensor.sensor_id;

    EEPROM.put(eeAddress, bnoID);

    eeAddress += sizeof(long);
    EEPROM.put(eeAddress, newCalib);
    Serial.println("Data stored to EEPROM.");

    Serial.println("\n--------------------------------\n");
    blinkLED(2000); //Long blink to indicate successful coefficient storage
    
  }else{ //Angrily blink to inform user system is not fully calibrated
    blinkLED(400);
    
  }
}

void loadCalibrationConstants(){

  int eeAddress = 0;
  long bnoID;
  bool foundCalib = false;

  EEPROM.get(eeAddress, bnoID);

  adafruit_bno055_offsets_t calibrationData;
  sensor_t sensor;

  /*
  *  Look for the sensor's unique ID at the beginning oF EEPROM.
  *  This isn't foolproof, but it's better than nothing.
  */
  bno.getSensor(&sensor);
  if (bnoID != sensor.sensor_id)
  {
      Serial.println("\nNo Calibration Data for this sensor exists in EEPROM");
      delay(500);
  }
  else
  {
      Serial.println("\nFound Calibration for this sensor in EEPROM.");
      eeAddress += sizeof(long);
      EEPROM.get(eeAddress, calibrationData);

      Serial.println("\n\nRestoring Calibration data to the BNO055...");
      bno.setSensorOffsets(calibrationData);

      Serial.println("\n\nCalibration data loaded into BNO055");
      foundCalib = true;
  }

  delay(1000);
    
  /* Display some basic information on this sensor */
  displaySensorDetails();

  /* Optional: Display current status */
  displaySensorStatus();

  bno.setExtCrystalUse(true);

  sensors_event_t event;
  bno.getEvent(&event);
  if (foundCalib){
    Serial.println("Move sensor slightly to calibrate magnetometers");
    digitalWrite(LED, HIGH);
    while (!bno.isFullyCalibrated())
    {
        bno.getEvent(&event);
        delay(BNO055_SAMPLERATE_DELAY_MS);
    }
    digitalWrite(LED, LOW);
    
    Serial.println("\nFully calibrated!");
    Serial.println("--------------------------------");
    Serial.println("Calibration Results: ");
    adafruit_bno055_offsets_t newCalib;
    bno.getSensorOffsets(newCalib);
    displaySensorOffsets(newCalib);
  }
}


void imuZeroAlt() {

  double currAltPos, errorAlt;
  
  for (int i=0; i<1000; i++) {
    currAltPos = getIMUPos(ALT);
    errorAlt = imuCalcSmallestError(currAltPos, 0);
    Serial.println(abs(errorAlt));

    if(abs(errorAlt) > 0.07){
      if(errorAlt < -1.0){
        celestronDriveMotor(DOWN, 9); //Note that these directions are flipped relative to the Celestron version
      }else if(errorAlt > 1.0){
        celestronDriveMotor(UP, 9);
      }else if(errorAlt < -0.5){
        celestronDriveMotor(DOWN, 7);
      }else if(errorAlt > 0.5){
        celestronDriveMotor(UP, 7);
      }else if(errorAlt < -0.25){
        celestronDriveMotor(DOWN, 6);
      }else if(errorAlt > 0.25){
        celestronDriveMotor(UP, 6);
      }
    } else {
      celestronDriveMotor(UP, 0);
      break;
    }
    
    delay(25);
  }
  Serial.println("Went to");
  Serial.println(currAltPos);
}

//Version of the celestronGoToPos, except driven by IMU readings
void imuGoToPos(double azmPos, double altPos, int recursions){
  double currAzmPos = getIMUPos(AZM);
  double currAltPos = getIMUPos(ALT);
  double errorAzm = imuCalcSmallestError(currAzmPos, azmPos);
  double errorAlt = imuCalcSmallestError(currAltPos, altPos);

  int goodAlign = 0;
  digitalWrite(LED, LOW);
  
  while(goodAlign < 2){
    goodAlign = 0;
    currAzmPos = getIMUPos(AZM);
    currAltPos = getIMUPos(ALT);
    double lastErrorAzm = errorAzm;
    double lastErrorAlt = errorAlt;
    errorAzm = imuCalcSmallestError(currAzmPos, azmPos);
    errorAlt = imuCalcSmallestError(currAltPos, altPos);
    Serial.print('@');
    Serial.print(currAzmPos,4);
    Serial.print(',');
    Serial.println(currAltPos,4);
    Serial.print("Delta Azm/Alt: ");
    Serial.print(errorAzm,4);
    Serial.print('\t');
    Serial.println(errorAlt,4);

    if(errorAzm == 0 || abs(errorAzm-lastErrorAzm) < (0.25 * double(abs(lastErrorAzm)))){
      if(errorAzm < -1.0){
        celestronDriveMotor(LEFT, 9);
      }else if(errorAzm > 1.0){
        celestronDriveMotor(RIGHT, 9);
      }else if(errorAzm < -0.5){
        celestronDriveMotor(LEFT, 7);
      }else if(errorAzm > 0.5){
        celestronDriveMotor(RIGHT, 7);
      }else{
        celestronDriveMotor(RIGHT, 0);
        goodAlign++;
      }
    }else{
      Serial.print("AZM Transient ");
      Serial.print(errorAzm);
      Serial.print(' ');
      Serial.print(lastErrorAzm);
      Serial.print(' ');
      Serial.println(0.2 * double(lastErrorAzm));
      celestronDriveMotor(RIGHT, 0);
    }

    if(errorAlt == 0 || abs(errorAlt-lastErrorAlt) < (0.25 * double(abs(lastErrorAlt)))){
      if(errorAlt < -1.0){
        celestronDriveMotor(DOWN, 9); //Note that these directions are flipped relative to the Celestron version
      }else if(errorAlt > 1.0){
        celestronDriveMotor(UP, 9);
      }else if(errorAlt < -0.5){
        celestronDriveMotor(DOWN, 7);
      }else if(errorAlt > 0.5){
        celestronDriveMotor(UP, 7);
      }else{
        celestronDriveMotor(DOWN, 0);
        goodAlign++;
      }
    }else{
      Serial.print("ALT Transient ");
      Serial.print(errorAlt);
      Serial.print(' ');
      Serial.print(lastErrorAlt);
      Serial.print(' ');
      Serial.println(0.2 * double(lastErrorAlt));
      celestronDriveMotor(DOWN, 0);
    }
    
    Serial.println();
    delay(25);
  }
  
  if(recursions < IMU_GOTO_MAX_RECURSIONS){ //Up to a defined limit, wait a couple of seconds after aligning for the IMU to settle down and then try to align again
    blinkLED(5000);
    imuGoToPos(azmPos, altPos, recursions+1);
  }else{
    Serial.println("Aligned!");
  }
}

//Figures out whether going in the normal direction or rolling over is shorter
double imuCalcSmallestError(double currentPos, double targetPos){

  if(currentPos == targetPos) return 0; //DON'T MOVE
  
  double upDistance = (currentPos < targetPos) ? targetPos - currentPos : 360.0 - (currentPos-targetPos); //If target is greater than current, going up is easy; otherwise you have to wrap around
  double downDistance = (currentPos > targetPos) ? targetPos - currentPos : (targetPos-currentPos) - 360.0; //If target is lesser than current, going down is easy; otherwise you have to wrap around
  
  return (abs(upDistance) < abs(downDistance)) ? upDistance : downDistance;
}


/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
    */
/**************************************************************************/
void displaySensorDetails(void)
{
    sensor_t sensor;
    bno.getSensor(&sensor);
    Serial.println("------------------------------------");
    Serial.print("Sensor:       "); Serial.println(sensor.name);
    Serial.print("Driver Ver:   "); Serial.println(sensor.version);
    Serial.print("Unique ID:    "); Serial.println(sensor.sensor_id);
    Serial.print("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
    Serial.print("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
    Serial.print("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
    Serial.println("------------------------------------");
    Serial.println("");
    delay(500);
}

/**************************************************************************/
/*
    Display some basic info about the sensor status
    */
/**************************************************************************/
void displaySensorStatus(void)
{
    /* Get the system status values (mostly for debugging purposes) */
    uint8_t system_status, self_test_results, system_error;
    system_status = self_test_results = system_error = 0;
    bno.getSystemStatus(&system_status, &self_test_results, &system_error);

    /* Display the results in the Serial Monitor */
    Serial.println("");
    Serial.print("System Status: 0x");
    Serial.println(system_status, HEX);
    Serial.print("Self Test:     0x");
    Serial.println(self_test_results, HEX);
    Serial.print("System Error:  0x");
    Serial.println(system_error, HEX);
    Serial.println("");
    delay(500);
}

/**************************************************************************/
/*
    Display sensor calibration status
    */
/**************************************************************************/
void displayCalStatus(void)
{
    /* Get the four calibration values (0..3) */
    /* Any sensor data reporting 0 should be ignored, */
    /* 3 means 'fully calibrated" */
    uint8_t system, gyro, accel, mag;
    system = gyro = accel = mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);

    /* The data should be ignored until the system calibration is > 0 */
    Serial.print("\t");
    if (!system)
    {
        Serial.print("! ");
    }

    /* Display the individual values */
    Serial.print("Sys:");
    Serial.print(system, DEC);
    Serial.print(" G:");
    Serial.print(gyro, DEC);
    Serial.print(" A:");
    Serial.print(accel, DEC);
    Serial.print(" M:");
    Serial.print(mag, DEC);
}

/**************************************************************************/
/*
    Display the raw calibration offset and radius data
    */
/**************************************************************************/
void displaySensorOffsets(const adafruit_bno055_offsets_t &calibData)
{
    Serial.print("Accelerometer: ");
    Serial.print(calibData.accel_offset_x); Serial.print(" ");
    Serial.print(calibData.accel_offset_y); Serial.print(" ");
    Serial.print(calibData.accel_offset_z); Serial.print(" ");

    Serial.print("\nGyro: ");
    Serial.print(calibData.gyro_offset_x); Serial.print(" ");
    Serial.print(calibData.gyro_offset_y); Serial.print(" ");
    Serial.print(calibData.gyro_offset_z); Serial.print(" ");

    Serial.print("\nMag: ");
    Serial.print(calibData.mag_offset_x); Serial.print(" ");
    Serial.print(calibData.mag_offset_y); Serial.print(" ");
    Serial.print(calibData.mag_offset_z); Serial.print(" ");

    Serial.print("\nAccel Radius: ");
    Serial.print(calibData.accel_radius);

    Serial.print("\nMag Radius: ");
    Serial.print(calibData.mag_radius);
}
