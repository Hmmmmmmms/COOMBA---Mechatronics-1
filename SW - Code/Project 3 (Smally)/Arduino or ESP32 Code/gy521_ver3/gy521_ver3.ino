// Made by Serif Catalgøl
// Mission Definition ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
Implemenet IMU for delta velocity tracking, and precise turn estimation.
chatgpt log for reference:
https://chatgpt.com/share/69ec94ae-78ac-832f-b268-3775ad60ace5
I added basic statistics in order to hopefully smoothen out the data, for sensitive and complicated usecases


NOTES:
last time: tried implementing turn estimation and velocity estimation

*/
// Start /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <Wire.h>
#include <vector>

const int MPU = 0x68;
float bits_to_ac = 9.815 / 16384.0;
float bits_to_gy = 1 / 131.0;
static const int sample_size = 10;


// structures /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

struct AxisData {
  //data structure with samples, offset, and average
  // meas: the data measured from the IMU
  // value: the processed data for usage
  // avg: average value of measured
  // calib: flag for calibration algorithm
  // threshold: the minimum variance value that will detech a change in movement
  float samples[sample_size];
  int index = 0;
  float meas = 0;
  float value = 0;
  float offset = 0;
  float threshold = 0;
  float avg = 0;
  bool calib = false;
};

struct Axis {  // 3D cartesian axis
  AxisData x;
  AxisData y;
  AxisData z;
};

struct Imu {
  //create an axis for a measurement type respectively
  Axis accel;
  Axis gyro;
  float temp;
};

struct Action {
  float time_start = 0;
  float time_stop = 0;
  bool flag = false;
};

struct Angle {
  float value = 0;
};

struct Robot {
  Action action;
  Angle angle;
  float speedometer = 0;
};

//create structures
Imu imu;
Robot smally;

// AP functions /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void addSample(AxisData& axisdata) {
  // adds samples untill index has reached sample size
  if (axisdata.index < sample_size) {
    axisdata.samples[axisdata.index++] = axisdata.meas;
  }
}

float sample_average(AxisData& axisdata) {
  if (axisdata.index >= 10 && !axisdata.calib) {
    float sum = 0;
    for (int i = 0; i < sample_size; i++) { sum += axisdata.samples[i]; }
    return sum / sample_size;
  }
}

void addSampleType(Axis& type) {
  //input: imu.gyro/imu.accel
  //function: save to measurement to samples if not calibrated
  if (!type.x.calib) { addSample(type.x); }
  if (!type.y.calib) { addSample(type.y); }
  if (!type.z.calib) { addSample(type.z); }
}

void calibrate(AxisData& a, int sample_size) {
  if (a.index >= sample_size) {
    a.offset = sample_average(a);
    a.calib = true;
  }
  a.value = a.meas - a.offset;
}

void calibrateAll() {
  calibrate(imu.accel.x, sample_size);
  calibrate(imu.accel.y, sample_size);
  calibrate(imu.accel.z, sample_size);
  calibrate(imu.gyro.x, sample_size);
  calibrate(imu.gyro.y, sample_size);
  calibrate(imu.gyro.z, sample_size);
}


void convertBits(Axis& type, float constant) {
  // covnert measurement bits ito usable values
  type.x.value = (type.x.meas - type.x.offset) * constant;
  type.y.value = (type.y.meas - type.y.offset) * constant;
  type.z.value = (type.z.meas - type.z.offset) * constant;

}

void imuReadSetup(Imu& a) {
  a.accel.x.meas = (int16_t)(Wire.read() << 8 | Wire.read());
  a.accel.y.meas = (int16_t)(Wire.read() << 8 | Wire.read());
  a.accel.z.meas = (int16_t)(Wire.read() << 8 | Wire.read());
  a.temp = (int16_t)(Wire.read() << 8 | Wire.read());
  a.gyro.x.meas = (int16_t)(Wire.read() << 8 | Wire.read());
  a.gyro.y.meas = (int16_t)(Wire.read() << 8 | Wire.read());
  a.gyro.z.meas = (int16_t)(Wire.read() << 8 | Wire.read());
}

void motor_drive(float distance) {
  smally.action.flag = true;
  //JENS OPAGVE:
  // make the motor go forwards for a certain distance in centimeters
  // when the motor has driven for a certain distance do ( smally.action.flag = false; )
  // also when distance value is positive it drives forwards, when its negative it drives backwards!
}

void motor_turn(bool clock_direction) {
  smally.action.flag = true;
  //JENS OPAGVE:
  // make the motor turn continuously left or right (CW: 0, CCW:1)
  //smally.speedometer =
}

void motor_stop() {
  smally.action.flag = false;
  //JENS OPAGVE:
  // make the motor stand still! or turn off basically
}

// API functions /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void imu_loop() {
  //read IMU data
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 14, true);
  imuReadSetup(imu);
  //add measurements to sample vector
  addSampleType(imu.accel);
  addSampleType(imu.gyro);
  //calibrate the IMU if samples full
  calibrateAll();
  // convert the measurements into usable SI values
  convertBits(imu.accel, bits_to_ac);  //m/s^2
  convertBits(imu.gyro, bits_to_gy);   //degrees per second
}

void turning(float degrees) {
  // determine turn direction (negative degree is CCW)
  bool direction;
  if (degrees > 0) {
    direction = 1;
  } else if (degrees < 0) {
    direction = 0;
  } else {
    return;
  }
  // type wished turn amount in degrees
  smally.angle.value = 0;
  unsigned long lastTime = micros();
  while (smally.angle.value < degrees) {
    // MOTOR CODE RUNS HERE (continuously)
    motor_turn(direction);
    unsigned long nowTime = micros();
    float dt = (nowTime - lastTime) / 1e6;
    lastTime = nowTime;
    float omega = imu.gyro.z.value;
    smally.angle.value += omega * dt;
  }

  // stop motors here
  motor_stop();
  smally.angle.value = 0;
}

void driving(float distance) {
  unsigned long lastTime = micros();
  while (smally.action.flag) {
    motor_drive(distance);
    unsigned long now = micros();
    float dt = (now - lastTime) / 1e6;
    lastTime = now;
    float accel_plane = sqrt(pow(imu.accel.y.value, 2) + pow(imu.accel.x.value, 2));
    smally.speedometer = accel_plane * dt;
    Serial.println(" ");
    Serial.print("speed (m/s) : ");
    Serial.println(smally.speedometer);
  }
  motor_stop();
  smally.speedometer = 0;
}

void movement_info() {
  Serial.println(" ");
  Serial.print("Turn angle: ");
  Serial.println(smally.angle.value);
  Serial.println(" ");
  Serial.print("speed (m/s) : ");
  Serial.println(smally.speedometer);
}

/////////////// SETUP ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  Wire.begin(21, 22);           //begin the wire communication (SDA, SCL)
  Wire.beginTransmission(MPU);  //begin, send the slave adress (in this case 68)
  Wire.write(0x6B);             //make the reset (place a 0 into the 6B register)
  Wire.write(0);
  Wire.endTransmission(true);  //end the transmission
  Serial.begin(115200);
  Serial.println("\nI2C Scanner");
  for (byte i = 1; i < 127; i++) {
    Wire.beginTransmission(i);
    if (Wire.endTransmission() == 0) {
      Serial.print("Found device at 0x");
      Serial.println(i, HEX);
    }
  }
}

/////////////// LOOP ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void loop() {

  imu_loop();
  //movement_info();

   //plotter //////////////////////
  Serial.print(imu.gyro.z.value);
  Serial.print(" ");
  Serial.print(imu.gyro.z.meas);
  Serial.print(" ");
  Serial.print(imu.gyro.z.offset);
  Serial.println(" ");
  // pls remember last print as println which triggers plotting
  delay(100);  //optional, increase time to read plot better, remove if you want max performance
  
}