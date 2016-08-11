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

bool beaconState = true;

void alignAFS() {
  // Calibrate sampling loop
  calibrateSampling();

  Serial.println("Loop calibrated. Press q to quit anytime. Where do you want to point? [e for EPC, l for lake, c for custom, q for quit, s for skip]");
  while (!Serial.available());
  char c = Serial.read();
  double tgtLat, tgtLon, srcLat, srcLon;
  if (c == 'q') return;
  if (c == 'l') {
    tgtLat = 37.423341;
    tgtLon = -122.175168;
    srcLat = 37.424133;
    srcLon = -122.177778;
  }
  if (c == 'e') {
    tgtLat = 37.424133;
    tgtLon = -122.177778;
    srcLat = 37.423341;
    srcLon = -122.175168;
  }
  if (c == 'c') {
    Serial.println("I'm sorry, Dave. I'm afraid I can't do that. [Read: I'm too lazy to implement this.]");
    return;
  }
  if (c != 's') {
    Serial.println("What's your reading for altitude/azimuth? I'm reading:");
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
    Serial.print("Azimuth offset? ");
    while (!Serial.available());
    float azmOffset = Serial.parseFloat();
    Serial.println(" Alright.");
  
    // Align it in the general direction via known GPS coordinates.
    coarseGPSalign(srcLat, srcLon, tgtLat, tgtLon, altOffset, azmOffset);
  }

  Serial.println("GPS coarse align complete. Check with the other side. Will now turn on beacon. [y for yes, q for quit]");
  while (!Serial.available());
  c = Serial.read();
  if (c == 'q') return;

  
  IntervalTimer beacon;
  beacon.begin(laserBeacon, 1000000.0 / (2 * BEACON_FREQUENCY));
  
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
    }
  }
  if (!terminated) {
    Serial.println("THE UNIVERSE IS ALL WRONG! YOU BEAT THE ODDS! WHAT IS GOING ON?");
  }
}

void doCircles(long refAltitude, long refAzimuth, double refRadius) {
  long circleRadius = refRadius/360.*POSMAX;
  long initialRadius = circleRadius;
  int HYPER = 10;
  double centerPower = hyperBeacon(HYPER);
  double oldPower = 0.0;
  double lastDiff = 0.0;

  double goodAlts[BATCH_SIZE] = {0};
  double goodAzms[BATCH_SIZE] = {0};
  double goodPwrs[BATCH_SIZE] = {0};
  
  while (centerPower <= AWESOME_THRESHOLD && circleRadius*2*PI/POSMAX <= POINTING_ACCURACY ) {

    int above_threshold = 0;
    double x, y, powah;
    for (int i=0; i<BATCH_SIZE; i++) {
      generateRandomPoint(&x, &y);
      celestronGoToPos(refAzimuth+x*circleRadius, refAltitude+y*circleRadius,1000);
      powah = hyperBeacon(HYPER);
      if (powah > POWER_THRESHOLD) {
        goodAzms[above_threshold] = refAzimuth+x*circleRadius;
        goodAlts[above_threshold] = refAltitude+y*circleRadius;
        goodPwrs[above_threshold] = powah;
        above_threshold++;
      }
    }

    if (above_threshold >= 1) {
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

      refAltitude = tempAlt/totalMass;
      refAzimuth = tempAzm/totalMass;
      circleRadius = circleRadius/1.5;
    } else {
      circleRadius += 2*initialRadius;
    }

    celestronGoToPos(refAzimuth, refAltitude, 1000);
    lastDiff = abs(centerPower-oldPower);
    oldPower = centerPower;
    centerPower = hyperBeacon(HYPER);
  }
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
  delay(1000 * 60 * 20);
  beacon.end();
}

/*
 * TODO: Implement actual ECEF with altitude.
 * 
 * See "Fundamentals of Astrodynamics and Applications" by Vallado (Rory has a copy).
 */
void latlon2ecef(double lat, double lon, double* x, double* y, double* z) {
  *x = EARTH_RADIUS*cos(lat*PI/180.)*cos(lon*PI/180.);
  *y = EARTH_RADIUS*cos(lat*PI/180.)*sin(lon*PI/180.);
  *z = EARTH_RADIUS*sin(lat*PI/180.);
}

/* Just point it in the general direction as known by GPS coordinates of the receiver and itself. */
void coarseGPSalign(double srcLat, double srcLon, double tgtLat, double tgtLon, long altOffset, long azmOffset) {
  double viewAlt, viewAzm;

  double x1, x2, y1, y2, z1, z2;
  latlon2ecef(srcLat, srcLon, &x1, &y1, &z1);
  latlon2ecef(tgtLat, tgtLon, &x2, &y2, &z2);

  double dx, dy, dz;
  dx = x2-x1;
  dy = y2-y1;
  dz = z2-z1;

  // See: https://gis.stackexchange.com/questions/58923/calculate-view-angle
  viewAlt = asin((x1*dx + y1*dy + z1*dz)/sqrt((x1*x1+y1*y1+z1*z1)*(dx*dx+dy*dy+dz*dz)))*180/PI;
  viewAzm = atan2((-y1*dx + x1*dy) / sqrt((x1*x1+y2*y2)*(dx*dx+dy*dy+dz*dz)), (-z1*x1*dx - z1*y1*dy + (x1*x1+y1*y1)*dz) / sqrt((x1*x1+y1*y1)*(x1*x1+y1*y1+z1*z1)*(dx*dx+dy*dy+dz*dz)))*180/PI;

  long currentAlt = celestronGetPos(ALT, false);
  long currentAzm = celestronGetPos(AZM, false);
  celestronGoToPos(currentAzm + (viewAzm - azmOffset)*POSMAX / 360., currentAlt + (viewAlt - altOffset)*POSMAX / 360.);
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
  celestronGoToPos(az, alt, true);
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
  noInterrupts();

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
  interrupts();
  
  return (realDFT * realDFT + imagDFT * imagDFT) / (SAMPLE_COUNT * totalPower);

}

void laserBeacon() {
  digitalWrite(LASER, beaconState ? HIGH : LOW);
  beaconState = !beaconState;
}

