/* Fourier-Simplex Align. Work in progress. Poke Joan (@jcreus) if it breaks.
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

float BEACON_FREQUENCY = 1000; // Hertz
float SAMPLE_FREQUENCY = 5000; // Hertz. Per Mr. Nyquist, this must be bigger than 2*BEACON_FREQUENCY

bool beaconState = true;

void alignAFS() {
  // Align it in the general direction via known GPS coordinates.
  coarseGPSalign();

  // Laser beacon for the other receiver
  IntervalTimer beacon;
  beacon.begin(laserBeacon, 1000000/(2*BEACON_FREQUENCY));

  // Start simplex search
  simplexSearch(celestronGetPos(ALT, false), celestronGetPos(AZM, false));

  beacon.end();
}

void alignBeacon() {
  IntervalTimer beacon;
  beacon.begin(laserBeacon, 1000000/(2*BEACON_FREQUENCY));
  delay(1000*60*20);
  beacon.end();
}

/* Just point it in the general direction as known by GPS coordinates of the receiver and itself. */
void coarseGPSalign() {
  // to be implemented in lake lag, not really worth it for hallway tests
}

void simplexSearch(double alt0, double az0) {
    double x0[2] = {alt0, az0};
    
    point_t    solution;
    optimset_t optimset;
    
    optimset.tolx     = 0.01;
    optimset.tolf     = 0.0001; // idk
    optimset.max_iter = 100;
    optimset.max_eval = 100;
    optimset.verbose  = 5;
    
    nelder_mead(x0, 2, optimset, &solution, NULL);
}

double cost_fun(int n, const double *x, void *arg) {
  double alt = x[0];
  double az = x[1];
  celestronGoToPos(alt, az);
  Serial.println("Beacon power is");
  double powah = getBeaconPower();
  Serial.println(powah);

  return -powah; // Simplex finds the MINIMUM! not the maximum
}

double getBeaconPower() {

  int n = 20; // Number of samples

  double totalPower = 0; // By Parseval's theorem, we don't need to calculate the full transform to get total power
  double realDFT = 0.0;
  double imagDFT = 0.0;
  double sample;
  double EFFECTIVE_SAMPLE_FREQUENCY = 3105; // TODO: be more careful
  EFFECTIVE_SAMPLE_FREQUENCY = SAMPLE_FREQUENCY; // ^ It was this bad because of Serial.println() lol
  double baseFreq = 2*3.1415926*BEACON_FREQUENCY/EFFECTIVE_SAMPLE_FREQUENCY;
  
  for (int i=0; i<n; i++) {
    sample = analogRead(SENSOR_PIN);
    totalPower += sample*sample;

    realDFT += sample * cos(baseFreq*i);
    imagDFT -= sample * sin(baseFreq*i);

    delayMicroseconds(1000000/SAMPLE_FREQUENCY-10); // 10 being the microseconds it takes to execute those steps ^ roughly,
                                                    // not that it matters particularly, with n=32 delta f ~ 156 Hz
  }

  return (realDFT*realDFT + imagDFT*imagDFT)/totalPower; // There might be a multiplicative constant N we don't care about.

}

void laserBeacon() {
  digitalWrite(LASER, beaconState ? HIGH : LOW);
  beaconState = !beaconState;
}

