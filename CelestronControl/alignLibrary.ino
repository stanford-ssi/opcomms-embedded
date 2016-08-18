/* Fourier-Simplex Align. Work in progress. Poke Joan (@jcreus) if it breaks.
 *  
 * What follows is completely outdated. Lesson learned: never document code.
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 * The idea is to blink the laser at some known frequency which the receiver knows. The receiver
 * then takes samples of the light sensor (something something Mr. Nyquist something something)
 * and takes the Fourier transform of it (well, for efficiency reasons we just look at the specific
 * frequency). The goal is to have the highest (relative) power at the known beacon frequency. To
 * get there we perform a simplex (Nelder-Mead) search over altitude-azimuth space until some maximum
 * is achieved.
 *
 * Considerations:
 *  - this only works when the sensor can at least see some of the laser. At short distances, lasers
 *    are, well, focused, so not really. I assume that at Lake Lag distances this is already okay.
 *    Before getting there, we can coarse align via GPS and if that doesn't work stochastically
 *    search around the area until some signal is received (still unimplemented).
 *  - Lol it's terribly untested..
 */

double BEACON_FREQUENCY = 1000; // Hertz
double SAMPLE_FREQUENCY = 5000; // Hertz. Per Mr. Nyquist, this must be bigger than 2*BEACON_FREQUENCY
double EFFECTIVE_SAMPLE_FREQUENCY = SAMPLE_FREQUENCY;
double SAMPLING_LOOP_DELAY = 0; // In microseconds. To adjust for suboptimal loop times.
int SAMPLE_COUNT = 30;
double EARTH_RADIUS = 6371009;
const int BATCH_SIZE = 20;
double POWER_THRESHOLD = 0.03;
double AWESOME_THRESHOLD = 0.25;
double POINTING_ACCURACY = 0.01;
double EARTH_FLATTENING = 1/298.257223563;

//ALTITUDE_OFFSET = ;
//AZIMUTH_OFFSET = 2638640;


bool beaconState = true;

/*
 * Main function to align each receiver. Both Rosencrantz and Guildenstern should run it at the same time.
 */
void alignAFS() {
  // Calibrate sampling loop so that we are sampling at the right frequency.
  //calibrateSampling();

  // Perform rough GPS align.
  Serial.println("Press q to quit anytime. Where do you want to point? [e for EPC, l for lake, c for custom, q for quit, s for skip]");
  while (!Serial.available());
  char c = Serial.read();
  double tgtLat, tgtLon, tgtAlt, srcLat, srcLon, srcAlt;
  if (c == 'q') return;
  if (c == 'l') {
    tgtLat = 37.423027;
    tgtLon = -122.174073;
    tgtAlt = 39.5;
    srcLat = 37.424111;
    srcLon = -122.177821;
    srcAlt = 44;
  }
  if (c == 'e') {
    tgtLat = 37.424111;
    tgtLon = -122.177821;
    tgtAlt = 44;
    srcLat = 37.423027;
    srcLon = -122.174073;
    srcAlt = 39.5;
  }
  if (c == 'c') {
    Serial.println("I'm sorry, Dave. I'm afraid I can't do that. [Read: I'm too lazy to implement this.]");
    return;
  }
  if (c != 's') {
    /*Serial.println("What's your reading for altitude/azimuth? I'm reading:");
    Serial.print("Altitude: ");
    Serial.print(celestronGetPos(ALT, false) * 360.0 / POSMAX);
    Serial.println(" deg.");
    Serial.print("Azimuth: ");
    Serial.print(celestronGetPos(AZM, false) * 360.0 / POSMAX);
    Serial.println(" deg.");
    Serial.print("Altitude offset? ");
    while (!Serial.available());
    float altOffset = Serial.parseFloat();
    Serial.println(" Alright.");
    Serial.println(altOffset);
    Serial.print("Azimuth offset? ");
    while (!Serial.available());
    float azmOffset = Serial.parseFloat();
    Serial.println(" Alright.");
    Serial.println(azmOffset);*/
  
    // Align it in the general direction via known GPS coordinates.
    coarseGPSalign(srcLat, srcLon, srcAlt, tgtLat, tgtLon, tgtAlt);
  }

  Serial.println("GPS coarse align complete. Check with the other side. Will now turn on beacon. [y for yes, q for quit]");
  while (!Serial.available());
  c = Serial.read();
  if (c == 'q') return;
  
  IntervalTimer beacon;
  beacon.begin(laserBeacon, 1000000.0 / (2 * BEACON_FREQUENCY));
  delay(25);
  Serial.println("Calibrating loop...");
  calibrateSampling();
  
  Serial.println("What's the angular size of the beacon? [try 0.2 degrees, idk]:");
  while (!Serial.available());
  double angularSize = Serial.parseFloat();

  doCircles(celestronGetPos(ALT, false), celestronGetPos(AZM, false), angularSize);

  beacon.end();
}

void generateRandomPoint(double* x, double* y) {
  int MAX = 1000000000;
  bool terminated = false;
  for (int i=0; i<100; i++) {
    double r1 = random(0,MAX)/((double)MAX);
    double r2 = random(0,MAX)/((double)MAX);
    if (r1*r1+r2*r2 <= 1) {
      *x = r1;
      *y = r2;
      terminated = true;
      break;
    }
  }
  if (!terminated) {
    Serial.println("THE UNIVERSE IS ALL WRONG! YOU BEAT THE ODDS! WHAT IS GOING ON?");
  }
}

void doCircles(long refAltitude, long refAzimuth, double refRadius) {
  long circleRadius = refRadius/360.*POSMAX;
  long initialRadius = circleRadius;
  int HYPER = 6;
  double centerPower = hyperBeacon(HYPER);
  Serial.println("Initial power:");
  Serial.println(centerPower);

  double goodAlts[BATCH_SIZE] = {0};
  double goodAzms[BATCH_SIZE] = {0};
  double goodPwrs[BATCH_SIZE] = {0};
  
  while ((centerPower <= AWESOME_THRESHOLD) && (circleRadius*360.0/POSMAX >= POINTING_ACCURACY)) {
    Serial.print("Looking at ");
    Serial.print(BATCH_SIZE);
    Serial.print(" points within a radius of ");
    Serial.print(circleRadius*360.0/((double) POSMAX),2);
    Serial.println(" deg.");
    int above_threshold = 0;
    double x, y, powah;
    for (int i=0; i<BATCH_SIZE; i++) {
      Serial.print("Moving to new point..");
      generateRandomPoint(&x, &y);
      Serial.print(".");
      ludicrousGoToPos(refAzimuth+x*circleRadius, refAltitude+y*circleRadius, circleRadius*0.05);
      powah = hyperBeacon(HYPER);
      if (powah > POWER_THRESHOLD) {
        goodAzms[above_threshold] = celestronGetPos(AZM, false);
        goodAlts[above_threshold] = celestronGetPos(ALT, false);
        goodPwrs[above_threshold] = powah;
        above_threshold++;
      }
      Serial.println(" Done.");
      delay(25);
    }

    if (above_threshold >= 1) {
      Serial.print("Found ");
      Serial.print(above_threshold);
      Serial.println(" points above threshold.");
      // find the centroid
      double totalMass = 0.0;
      double tempAlt = 0.0;
      double tempAzm = 0.0;
      double mass;

      for (int i=0; i<above_threshold; i++) {
        mass = goodPwrs[i]-POWER_THRESHOLD;
        tempAlt += mass*goodAlts[i];
        tempAzm += mass*goodAzms[i];
        totalMass += mass;
      }

      Serial.println("Moving to new reference center and shrinking radius...");

      refAltitude = tempAlt/totalMass;
      refAzimuth = tempAzm/totalMass;
      circleRadius = circleRadius/1.5;
    } else {
      Serial.println("No good points. Increasing circle radius...");
      circleRadius += 2*initialRadius;
    }

    celestronGoToPos(refAzimuth, refAltitude);
    centerPower = hyperBeacon(HYPER);
  }
  Serial.println("Done!");
  Serial.print("Center power: ");
  Serial.println(centerPower);
  Serial.print("Circle radius: ");
  Serial.println(circleRadius);
}

double hyperBeacon(int n) {
  double res = 0.0;
  for (int i=0; i<n; i++) {
    res += getBeaconPower();
  }
  return res/((double) n);
}

void calibrateSampling() {
  Serial.println("Alright. Calibrating loop time...");
  // Remove some time from each loop
  SAMPLING_LOOP_DELAY = 0.0;
  double loopTime;
  double avgTime = 0.0;
  for (int i = 0; i < 10; i++) {
    getBeaconPower(&loopTime);
    avgTime += loopTime;
  }
  avgTime = avgTime / 10.0;
  SAMPLING_LOOP_DELAY = (avgTime - SAMPLE_COUNT * 1000000.0 / SAMPLE_FREQUENCY) / ((double) SAMPLE_COUNT);
  Serial.print("Adding a correction of ");
  Serial.print(SAMPLING_LOOP_DELAY, 0);
  Serial.println(" microseconds in the loop.");

  // Find the effective sampling frequency. Should be pretty damn close, anyway.
  avgTime = 0.0;
  for (int i = 0; i < 10; i++) {
    getBeaconPower(&loopTime);
    avgTime += loopTime;
  }
  avgTime = avgTime / 10.0;
  EFFECTIVE_SAMPLE_FREQUENCY = SAMPLE_COUNT * 1000000.0 / avgTime;
  Serial.print("Now the effective sampling frequency is ");
  Serial.print(EFFECTIVE_SAMPLE_FREQUENCY);
  Serial.println(" Hz.");
}

void alignBeacon() {
  IntervalTimer beacon;
  beacon.begin(laserBeacon, 1000000 / (2 * BEACON_FREQUENCY));
  delay(1000 * 60 * 60);
  beacon.end();
}

long wrapAround(long x) {
  if (x>POSMAX) return x % POSMAX;
  if (x<0) return POSMAX+x;
  return x;
}

void alignGPS() {
  Serial.println("Alright, we'll do coarse alignment.");
  Serial.print("First we'll move to altitude 0deg. We can probably do this with the IMU sensor, since that doesn't ");
  Serial.println("require messing with the magnetometer calibration, which is useless for now.");
  Serial.println("As long as you don't disconnect the telescope, this calibration should remain useful.");
  imuZeroAlt();
  imuZeroAlt();
  Serial.println("Done! Altitude should be roughly zero. ");

  Serial.println("Now we'll sample many azimuth points, and we'll do regression on them.");
  Serial.println("I suggest entering the same point several times, as many as phones with a compass there are.");
  Serial.print("The whole procedure relies on errors being normally distributed, which might not be the case because of ");
  Serial.println("shitty magnetic fields. But I do suggest not looking at your friend's readings to avoid introducing biases.");
  Serial.println();
  Serial.println();
  Serial.println("OH BY THE WAY. Set 'No newline' on the Arduino console. Thx.");
  Serial.println();
  Serial.println();
  
  long currentAzm = celestronGetPos(AZM, false);
  long initialAzm = currentAzm;
  long currentAlt = celestronGetPos(ALT, false);

  double telPoints[100] = {0};
  double angPoints[100] = {0};
  int numPoints = 0;

  while (true) {
    Serial.println("Enter 666 to quit. Enter 420 to go to a new point. Enter 1337 to retract your last point. Otherwise, enter another reading for the current point.");
    while (!Serial.available());
    float pt = Serial.parseFloat();
    if (pt == 666) {
      break;
    } else if (pt == 420) {
      currentAzm = wrapAround(initialAzm+random(-POSMAX/8,POSMAX/8));
      celestronGoToPos(currentAzm, currentAlt);
    } else if (pt == 1337) {
      numPoints--;
    } else {
      angPoints[numPoints] = (pt < 180) ? pt : (pt-360);
      telPoints[numPoints] = (currentAzm < POSMAX/2) ? currentAzm : currentAzm-POSMAX;
      numPoints++;
    }
  }
  double avg = 0.0;
  double slope = POSMAX/360.;
  for (int i=0; i<numPoints; i++) {
    Serial.print(telPoints[i]);
    Serial.print(",");
    Serial.println(angPoints[i]);
    avg += telPoints[i]-slope*angPoints[i];
  }
  avg = avg/((double) numPoints);

  // Bootstrapping...
  double eavg = 0.0;
  double ss = 0.0;
  for (int i=0; i<100; i++) {
    double pt = 0.0;
    for (int j=0; j<numPoints; j++) {
      int rd = random(0, numPoints);
      pt += telPoints[rd]-slope*angPoints[rd];
    }
    pt = pt/((double) numPoints);
    eavg += pt;
    ss += pt*pt;
  }
  eavg = eavg/100.0;
  double err = ss/100.0 - eavg*eavg;
  err = sqrt(err);
  
  Serial.println("Alrighty, done.");
  Serial.println("Shifts are:");
  Serial.print("Altitude: ");
  Serial.println(currentAlt);
  Serial.print("Azimuth: ");
  Serial.println(avg);
  Serial.print("Azimuth error: ");
  Serial.print(err);
  Serial.print(" (");
  Serial.print(err*360./POSMAX);
  Serial.println(" deg)");

  Serial.println("Moving to true north...");
  celestronGoToPos(avg, currentAlt);
  ALTITUDE_OFFSET = currentAlt;
  AZIMUTH_OFFSET = avg;
}

/*
 * GPS coordinates to Earth-centered, Earth-fixed coordinates.
 * References:
 *  - https://www.mathworks.com/help/aeroblks/llatoecefposition.html
 *  - "Fundamentals of Astrodynamics and Applications" by Vallado (Rory has a copy).
 */
void latlon2ecef(double lat, double lon, double h, double* x, double* y, double* z) {
  lat = lat*PI/180.;
  lon = lon*PI/180.;
  double lambda_s = atan(pow(1-EARTH_FLATTENING,2)*tan(lat));
  double r_s = sqrt((EARTH_RADIUS*EARTH_RADIUS)/(1+(1./pow(1-EARTH_FLATTENING,2) - 1)*sin(lambda_s)*sin(lambda_s) ));
  *x = r_s*cos(lambda_s)*cos(lon) + h*cos(lat)*cos(lon);
  *y = r_s*cos(lambda_s)*sin(lon) + h*cos(lat)*sin(lon);
  *z = r_s*sin(lambda_s) + h*sin(lat);
}

/* Just point it in the general direction as known by GPS coordinates of the receiver and itself. */
void coarseGPSalign(double srcLat, double srcLon, double srcAlt, double tgtLat, double tgtLon, double tgtAlt) {
  double viewAlt, viewAzm;

  double x1, x2, y1, y2, z1, z2;
  latlon2ecef(srcLat, srcLon, srcAlt, &x1, &y1, &z1);
  latlon2ecef(tgtLat, tgtLon, tgtAlt, &x2, &y2, &z2);

  double dx, dy, dz;
  dx = x2-x1;
  dy = y2-y1;
  dz = z2-z1;

  // See: https://gis.stackexchange.com/questions/58923/calculate-view-angle
  viewAlt = asin((x1*dx + y1*dy + z1*dz)/sqrt((x1*x1+y1*y1+z1*z1)*(dx*dx+dy*dy+dz*dz)))*180/PI;
  viewAzm = atan2((-y1*dx + x1*dy) / sqrt((x1*x1+y2*y2)*(dx*dx+dy*dy+dz*dz)), (-z1*x1*dx - z1*y1*dy + (x1*x1+y1*y1)*dz) / sqrt((x1*x1+y1*y1)*(x1*x1+y1*y1+z1*z1)*(dx*dx+dy*dy+dz*dz)))*180/PI;

  if (viewAzm < 0) {
    viewAzm = 360+viewAzm;
  }
  Serial.println("Want to go to");
  Serial.println(viewAlt);
  Serial.println(viewAzm);

  long currentAlt = celestronGetPos(ALT, false);
  long currentAzm = celestronGetPos(AZM, false);
  long pointAzm = (currentAzm-AZIMUTH_OFFSET) + viewAzm*POSMAX / 360.;
  long pointAlt = (currentAlt-ALTITUDE_OFFSET) + viewAlt*POSMAX / 360.;
  Serial.println(pointAzm);
  Serial.println(pointAlt);
  pointAzm = wrapAround(pointAzm);
  pointAlt = wrapAround(pointAlt);
  celestronGoToPos(pointAzm, pointAlt);
  Serial.println(pointAzm);
  Serial.println(pointAlt);
}

void simplexSearch(double alt0, double az0) {
  double x0[2] = {alt0, az0};

  point_t    solution;
  optimset_t optimset;

  optimset.tolx     = 0.001;
  optimset.tolf     = 0.0001; // idk
  optimset.max_iter = 100;
  optimset.max_eval = 100;
  optimset.verbose  = 5;

  nelder_mead(x0, 2, optimset, &solution, NULL);
}

double cost_fun(int n, const double *x, void *arg) {
  double alt = x[0];
  double az = x[1];
  celestronGoToPos(az, alt);
  Serial.println("Beacon power is");
  double powah = 0;
  for (int i = 0; i < 10; i++) {
    powah += getBeaconPower();
  }
  powah /= 10.0;
  Serial.println(powah);

  return -powah; // Simplex finds the MINIMUM! not the maximum
}

double getBeaconPower() {
  double _time = 0.0;
  return getBeaconPower(&_time);
}

double getBeaconPower(double *elapsedTime) {
  //noInterrupts();

  double totalPower = 0; // By Parseval's theorem, we don't need to calculate the full transform to get total power
  double realDFT = 0.0;
  double imagDFT = 0.0;
  double sample;
  double baseFreq = 2 * 3.1415926 * BEACON_FREQUENCY / EFFECTIVE_SAMPLE_FREQUENCY;

  double initialTime = micros();
  for (int i = 0; i < SAMPLE_COUNT; i++) {
    sample = analogRead(SENSOR_PIN);
    totalPower += sample * sample;

    realDFT += sample * cos(baseFreq * i);
    imagDFT -= sample * sin(baseFreq * i);

    // Compensate for things taking a bit longer.
    delayMicroseconds(1000000.0 / SAMPLE_FREQUENCY - SAMPLING_LOOP_DELAY);
  }

  *elapsedTime = micros() - initialTime;
  //interrupts();
  
  return (realDFT * realDFT + imagDFT * imagDFT) / (SAMPLE_COUNT * totalPower);

}

void laserBeacon() {
  digitalWrite(LASER, beaconState ? HIGH : LOW);
  beaconState = !beaconState;
}


