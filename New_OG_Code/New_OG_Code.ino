// general setup
unsigned long tStart;
unsigned long tNow;
unsigned long cb_time;

// RW setup parameters:
struct RW_param {
  byte pwmOutPin = 20; // ESCON pin J5-1, PWM output pin
  byte motSetPin = 19; // ESCON pin J5-2, motor enable (HIGH = active)
  byte curInPin = A8; // ESCON pin J6-5, actual current (0.0 V = -curMax A, 3.3 V = curMax A)
  byte velInPin = A9; // ESCON pin J6-6, actual speed (0.0 V = -velMax rpm, 3.3 V = velMax rpm)
  int velMax = 5000;   // [rpm], maximum allowable motor velocity
  int curMax = 850;    //  [mA], maximum allowable motor current
};

// RW tracking parameters:
struct RW_track {
  bool runMode = false;
  bool motSet = false;
  int curSet = 0; //  [mA], outgoing current command
  int velMes[30] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; // [rpm], measured velocity
  //int velMes[10] = {0,0,0,0,0,0,0,0,0,0}; // [rpm], measured velocity
  //int velMes[5] = {0,0,0,0,0}; // [rpm], measured velocity
  int velMesRaw = 0;
  int velMesAvg = 0;
  int curMes[30] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; //  [mA], measured current
  //int curMes[10] = {0,0,0,0,0,0,0,0,0,0}; //  [mA], measured current
  //int curMes[5] = {0,0,0,0,0}; //  [mA], measured current
  int curMesRaw = 0;
  int curMesAvg = 0;
  int velBias;    // [rpm], estimated velocity bias
  int curBias;    //  [mA], estimated current bias
};

// RW object definition:
struct RW_def {
  RW_param param;
  RW_track track;
};
RW_def RW;

// RW setup:
bool runCmd;
int curCmd;
int velCmd;

// Arduino setup function
void setup(void) {
  // ...
  Serial.begin(115200);
  tStart = millis();
  RW = RW_init(RW);

}

// Arduino main loop function
void loop(void) {
  // regular operation
  if (millis() - cb_time < 5000) {
    // ...
    tNow = millis() - tStart;
    BNO_out(tNow, bno, imuOut, pIMU);
    // ...
    tNow = millis() - tStart;
    SS_out(tNow, ADC_CS, ssOut, pSS);
    // ...
    tNow = millis() - tStart;
    RW = RW_set(RW, runCmd, curCmd, cb_time);
    RW_out(tNow, velCmd, RW, rwOut, pRW);
  } 
  // extended blackout/shutdown operation (temporary fix)
  else {
    analogWrite(RW.param.pwmOutPin,127);
    digitalWrite(RW.param.motSetPin,false);
  }
  nh.spinOnce();
  delay(5); // delay before next sample
}

// ...
void RW_out(unsigned long tNow, int velCmd, RW_def RW, rss_ctrl::rw_status rwOut, ros::Publisher pRW) {
  // ...
  word pwmSet, binSet;
  pwmSet = map(RW.track.curSet,-RW.param.curMax,RW.param.curMax,0,10000); // map current command to set PWM percentage*100
  binSet = map(pwmSet,0,10000,1000,9000);                                 // map PWM percentage between 10% - 90%
  binSet = map(binSet,0,10000,0,255);                                     // map PWM percentage to analog output range
  analogWrite(RW.param.pwmOutPin,binSet);
  digitalWrite(RW.param.motSetPin,RW.track.motSet);

  // custom message publisher test
  rwOut.timestamp = tNow;
  rwOut.runMode = RW.track.runMode;
  rwOut.motSet = RW.track.motSet;
  rwOut.curSet = RW.track.curSet;
  rwOut.curMesRaw = RW.track.curMesRaw;
  rwOut.curMesAvg = RW.track.curMesAvg;
  rwOut.velCmd = velCmd;
  rwOut.velMesRaw = RW.track.velMesRaw;
  rwOut.velMesAvg = RW.track.velMesAvg;
  
  pRW.publish(&rwOut);
}

// ...
RW_def RW_set(RW_def RW, bool runCmd, int curCmd, unsigned long cb_time) {

  // mode handling
  if (runCmd == true) {
    // if received run command is true
    if (RW.track.runMode == false) {
      // if switching to runMode, repeat bias checks first
      RW.track.velBias = biasCheck(RW.param.velInPin, -RW.param.velMax, RW.param.velMax);
      RW.track.curBias = biasCheck(RW.param.curInPin, -RW.param.curMax, RW.param.curMax);
      RW.track.runMode = true;
    } 
    else {
      // if test currently underway
      if (millis() - cb_time >= 500) {
        // if no new command has been sent in 500 ms, switch out of runMode
        RW.track.runMode = false;
      } else {
        // keep runMode on during regular operation
        RW.track.runMode = true;
      }
    }
  } else {
    // if received run command is false
    RW.track.runMode = false;
  }

  // ...
  RW.track.velMesRaw = map(analogRead(RW.param.velInPin),0,1023,-RW.param.velMax,RW.param.velMax) - RW.track.velBias;
  RW.track.curMesRaw = map(analogRead(RW.param.curInPin),0,1023,-RW.param.curMax,RW.param.curMax) - RW.track.curBias;
  RW.track.velMesAvg = RW.track.velMesRaw;
  RW.track.curMesAvg = RW.track.curMesRaw;
  for (int i = 29; i >=1; i--) {
    RW.track.velMes[i] = RW.track.velMes[i-1];
    RW.track.velMesAvg = RW.track.velMesAvg + RW.track.velMes[i];
    RW.track.curMes[i] = RW.track.curMes[i-1];
    RW.track.curMesAvg = RW.track.curMesAvg + RW.track.curMes[i];
  }
  RW.track.velMes[0] = RW.track.velMesRaw;
  RW.track.velMesAvg = RW.track.velMesAvg/30;
  RW.track.curMes[0] = RW.track.curMesRaw;
  RW.track.curMesAvg = RW.track.curMesAvg/30;

  // ...
  if (RW.track.runMode == true) {
    RW.track.motSet = true;
    RW.track.curSet = curCmd;
  } else {
    RW.track.curSet = 0;
  }

  return RW;
}

// ...
RW_def RW_init(RW_def RW) {
  // initialize controller output pins
  pinMode(RW.param.pwmOutPin, OUTPUT);
  pinMode(RW.param.motSetPin, OUTPUT);
  
  // check measurement bias
  RW.track.velBias = biasCheck(RW.param.velInPin, -RW.param.velMax, RW.param.velMax);
  RW.track.curBias = biasCheck(RW.param.curInPin, -RW.param.curMax, RW.param.curMax);

  return RW;
}

// evaluates measurement bias
int biasCheck(byte pin, int mesMin, int mesMax) {
  // ...
  int N = 500, bias = 0;
  for (int i = 1; i <= N; i++) {
    bias = bias + analogRead(pin);
    delay(5);
  }
  bias = map(bias / N, 0, 1023, mesMin, mesMax);
  
  return bias;
}
