// ROS libraries
#include <ros.h>
#include <rss_ctrl/rw_cmd.h>
#include <rss_ctrl/imu_status.h>
#include <rss_ctrl/ss_status.h>
#include <rss_ctrl/rw_status.h>
#include <std_msgs/Bool.h>

// BNO055 libraries
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// Sun sensor libraries
#include <SPI.h>

// general setup
unsigned long tStart;
unsigned long tNow;
unsigned long cb_time;

// BNO055 setup
Adafruit_BNO055 bno;
byte calib[4] = {0, 0, 0, 0};
float orient[3];

// Sun sensor setup
byte ADC_CS[5];

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
bool runCmd = false;
int curCmd = 0;
int velCmd = 0;

// ROS setup
ros::NodeHandle nh;
rss_ctrl::imu_status imuOut;
ros::Publisher pIMU("imu_status", &imuOut);
rss_ctrl::ss_status ssOut;
ros::Publisher pSS("ss_status", &ssOut);
rss_ctrl::rw_status rwOut;
ros::Publisher pRW("rw_status", &rwOut);

// callback function: RW control parameters
void ctrl_callback(const rss_ctrl::rw_cmd &msg) {
  runCmd = msg.runCmd;
  curCmd = msg.curCmd;
  velCmd = msg.velSet;
  cb_time = millis();
}
ros::Subscriber<rss_ctrl::rw_cmd> s1("rw_ctrl", &ctrl_callback);

// Arduino setup function
void setup(void) {
  // ...
  Serial.begin(115200);
  nh.initNode();
  tStart = millis();
  
  // ...
  bno = BNO_init(bno);
  SS_init(ADC_CS);
  RW = RW_init(RW);
  
  // ...
  nh.advertise(pIMU);
  nh.advertise(pSS);
  nh.advertise(pRW);
  nh.subscribe(s1);
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

// BNO055 sensor publisher function
void BNO_out(unsigned long tNow, Adafruit_BNO055 bno, rss_ctrl::imu_status imuOut, ros::Publisher pIMU) {
  // ...
  uint8_t calib[4] = {0, 0, 0, 0};
  bno.getCalibration(&calib[0], &calib[1], &calib[2], &calib[3]);
  sensors_event_t event;
  bno.getEvent(&event);

  // ...
  imuOut.timestamp = tNow;
  imuOut.calibration[0] = calib[0];
  imuOut.calibration[1] = calib[1];
  imuOut.calibration[2] = calib[2];
  imuOut.calibration[3] = calib[3];
  imuOut.orientation[0] = event.orientation.x;
  imuOut.orientation[1] = event.orientation.y;
  imuOut.orientation[2] = event.orientation.z;
  
  pIMU.publish(&imuOut);
}

// BNO055 sensor initialization
Adafruit_BNO055 BNO_init(Adafruit_BNO055 bno) {
  bno = Adafruit_BNO055(55, 0x28, &Wire1);
  if(!bno.begin()) {
    Serial.print("BNO055_cannot_detect");
    while(1);
  } else {
    bno.setExtCrystalUse(true);
  }
  
  return bno;
}

// Sun sensor publisher function
void SS_out(unsigned long tNow, byte *ADC_CS, rss_ctrl::ss_status ssOut, ros::Publisher pSS) {
  // ...
  const byte ch0_code = 0b10010100; // ADC channel 0: used by LM20BIM7 temperature sensor
  const byte ch1_code = 0b11010100; // ADC channel 1: used by SFH-2430 photodiode (240 degree configuration)
  const byte ch2_code = 0b10100100; // ADC channel 2: used by SFH-2430 photodiode   (0 degree configuration)
  const byte ch3_code = 0b11100100; // ADC channel 3: used by SFH-2430 photodiode (120 degree configuration)
  
  // ...
  byte CS_PIN;
  word tempCheck[5] = {0, 0, 0, 0, 0};
  word SS0degCheck[5] = {0, 0, 0, 0, 0};
  word SS120degCheck[5] = {0, 0, 0, 0, 0};
  word SS240degCheck[5] = {0, 0, 0, 0, 0};

  // ...
  for (int i = 0; i <= 4; i++) {
    CS_PIN = ADC_CS[i];
    tempCheck[i] = ads7841(ch0_code, CS_PIN);     // temperature sensor reading
    SS0degCheck[i] = ads7841(ch2_code, CS_PIN);   //   0 degree Sun sensor configuration reading
    SS120degCheck[i] = ads7841(ch3_code, CS_PIN); // 120 degree Sun sensor configuration reading
    SS240degCheck[i] = ads7841(ch1_code, CS_PIN); // 240 degree Sun sensor configuration reading
  }

  // ...
  ssOut.timestamp = tNow;
  ssOut.tempOut[0] = tempCheck[0]; ssOut.tempOut[1] = tempCheck[4];
  ssOut.xpOut[0] = SS0degCheck[0]; ssOut.xpOut[1] = SS120degCheck[0]; ssOut.xpOut[2] = SS240degCheck[0];
  ssOut.xmOut[0] = SS0degCheck[1]; ssOut.xmOut[1] = SS120degCheck[1]; ssOut.xmOut[2] = SS240degCheck[1];
  ssOut.ypOut[0] = SS0degCheck[2]; ssOut.ypOut[1] = SS120degCheck[2]; ssOut.ypOut[2] = SS240degCheck[2];
  ssOut.ymOut[0] = SS0degCheck[3]; ssOut.ymOut[1] = SS120degCheck[3]; ssOut.ymOut[2] = SS240degCheck[3];
  ssOut.zmOut[0] = 0;              ssOut.zmOut[1] = 0;                ssOut.zmOut[2] = 0;
  
  pSS.publish(&ssOut);
}

// function to read ADS7841
unsigned int ads7841(const byte control, byte CS_PIN) {
  unsigned int bitnum;        // return value
  digitalWrite(CS_PIN, LOW);  // activate ADS7841
  SPI.transfer(control);      // transfer control byte
  byte msb = SPI.transfer(0); // read MSB & LSB
  byte lsb = SPI.transfer(0);
  digitalWrite(CS_PIN, HIGH); // deactivate ADS7841
  msb = msb & 0x7F;           // isolate readings and form final reading
  lsb = lsb >> 3;
  bitnum = ((word)msb << 5) | lsb;
  
  return bitnum;
}

// Sun sensor initialization
byte SS_init(byte* ADC_CS) {
  // SPI COMM pin assignment
  int ADC_CLK = 13;  // SPI clock pin assignment
  int ADC_MISO = 12; // SPI master in/slave out pin assignment
  int ADC_MOSI = 11; // SPI master out/slave in pin assignment
  // SPI CS pin assignment
  ADC_CS[0] = 27; // CS_SUN_X+
  ADC_CS[1] = 3;  // CS_SUN_X-
  ADC_CS[2] = 26; // CS_SUN_Y+
  ADC_CS[3] = 2;  // CS_SUN_Y-
  ADC_CS[4] = 25; // CS_SUN_Z-
  // set up SPI pins
  pinMode(ADC_MOSI, OUTPUT);
  pinMode(ADC_MISO, INPUT);
  pinMode(ADC_CLK, OUTPUT);
  pinMode(ADC_CS[0], OUTPUT);
  pinMode(ADC_CS[1], OUTPUT);
  pinMode(ADC_CS[2], OUTPUT);
  pinMode(ADC_CS[3], OUTPUT);
  pinMode(ADC_CS[4], OUTPUT);
  // set up SPI
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);
  SPI.setClockDivider(SPI_CLOCK_DIV16);
  // turn off all SPI CS slaves to start
  digitalWrite(ADC_CS[0], HIGH);
  digitalWrite(ADC_CS[1], HIGH);
  digitalWrite(ADC_CS[2], HIGH);
  digitalWrite(ADC_CS[3], HIGH);
  digitalWrite(ADC_CS[4], HIGH);
  
  return *ADC_CS;
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
