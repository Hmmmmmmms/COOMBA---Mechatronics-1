# include <DHT.h>
# include <WiFi.h>
# include <PubSubClient.h>
# include <esp_wifi.h>
# include <string.h>
# include <ESP32Servo.h>
# include <AccelStepper.h>
# include <Wire.h>
# include <vector>

// Jens

#define FULLSTEP 4
#define DIRA 32
#define DIRB 33

const long STEPS_PER_REV = 2048;   // ← use 2048 if you switch to FULLSTEP

const int motorSpeed = 500;

// setup
int step1[] = {13,14,27,26};
int step2[] = {4,16,17,18};
int arrsize = sizeof(step1)/sizeof(step1[0]);

// Motor 1 on pins 4,6,5,7 (IN1, IN3, IN2, IN4 order — important!)
AccelStepper stepper1(FULLSTEP, 13, 27, 14, 26);
// Motor 2
AccelStepper stepper2(FULLSTEP, 4, 17, 16, 18);

int motorDirection = 1;   // 1 = forward, -1 = reverse
int movementMode = 0; // 0 = stop, 1 = forward, 2 = turn left, 3 = turn right, 4 = reverse
int turntimer = 0;

// Jens


// Serif /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

const int MPU = 0x68;
float bits_to_ac = 9.815 / 16384.0;
float bits_to_gy = 1 / 131.0;
static const int sample_size = 10;

// structures

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

// AP functions

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
  turntimer = 40000;

  if (turntimer > 1) {
    Serial.println("Driving");
    movementMode = 2;
    updateMotors();
    turntimer--;
  } else if (turntimer == 1) {
    turntimer = 0;
    Serial.println("Stopped");
    movementMode = 1;
    updateMotors();
  }

}

void motor_turn(bool clock_direction) {
  smally.action.flag = true;
  //JENS OPAGVE:
  // make the motor turn continuously left or right (CW: 0, CCW:1)
  //smally.speedometer =
  if (clock_direction == 0){
    movementMode = 3;
    updateMotors();
  } else if (clock_direction == 1)
    movementMode = 2;
    updateMotors();
}

void motor_stop() {
  smally.action.flag = false;
  //JENS OPAGVE:
  // make the motor stand still! or turn off basically
  movementMode = 0;
  updateMotors();
}

// API functions

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
  while (smally.angle.value <= degrees) {
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


// Serif /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



// Otto
#define PIN_SG90 23
Servo sg90;
WiFiServer server(80);
int currentAngle = 90;
// Otto






//float T, H;
char buf[20];

// Flag to control data flow
bool transmit_data = false;

// Wifi login data
const char *ssid = "Pixel_5796";
const char *password = "internet";
//const char *ssid = "Monkey";
//const char *password = "monkenet";
//const char *ssid = "SERIF WIFI";
//const char *password = "123456789";




// MQTT broker login data
const char *mqtt_user = "sorenlk@tutamail.com";
const char *mqtt_password = "*zMmQ4cvx0&n^R";

// Create wifi and mqtt connections
WiFiClient espcli;
PubSubClient client(espcli);

int loopcnt = 0;

String client_id;

void setup() 
{
  // Serif
  Wire.begin(21, 22);           //begin the wire communication (SDA, SCL)
  Wire.beginTransmission(MPU);  //begin, send the slave adress (in this case 68)
  Wire.write(0x6B);             //make the reset (place a 0 into the 6B register)
  Wire.write(0);
  Wire.endTransmission(true);  //end the transmission

  for (byte i = 1; i < 127; i++) {
    Wire.beginTransmission(i);
    if (Wire.endTransmission() == 0) {
      Serial.print("Found device at 0x");
      Serial.println(i, HEX);
    }
  }

  // Serif

  //Serial.begin(115200);
  Serial.begin(921600);


  // Otto
  sg90.setPeriodHertz(50);
  sg90.attach(PIN_SG90, 500, 2400);
  sg90.write(currentAngle);
  // Otto



  // Jens
  int value = LOW;


  for(int i=0;i<arrsize;i++){
    pinMode(step1[i], OUTPUT);
    digitalWrite(step1[i], LOW);
  }

  for(int i=0;i<arrsize;i++){
    pinMode(step2[i], OUTPUT);
    digitalWrite(step2[i], LOW);
  }

  pinMode(DIRA,OUTPUT);
  pinMode(DIRB,OUTPUT);
  digitalWrite(DIRA,LOW);
  digitalWrite(DIRB,LOW);

  // Very important for .runSpeed() — without this, max speed defaults to 1 step/sec!
  stepper1.setMaxSpeed(1200.0);     // steps per second — realistic range for 28BYJ-48: 400–1200
  stepper2.setMaxSpeed(1200.0);

  // Optional: acceleration makes startup smoother (still works with runSpeed)
  stepper1.setAcceleration(500.0);
  stepper2.setAcceleration(500.0);

  // Set constant speed — positive = clockwise (depending on your wiring)
  // Change to -300 for opposite direction
  stepper1.setSpeed(0);//1000 before   // adjust this number — higher = faster
  stepper2.setSpeed(0);//-1000 before   // same speed & direction as motor 1
  // Or different speeds: stepper2.setSpeed(600);
  // Or opposite: stepper2.setSpeed(-400);

// Jens



  //dht.begin();
  pinMode(2, OUTPUT);
  delay(100);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password); // attempt to connect

  Serial.println();
  Serial.println("Wait for WiFi.. ");

  while (WiFi.status() != WL_CONNECTED) { //wait to connect
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  delay(100);

  client.setServer("maqiatto.com", 1883);
  client.setCallback(subscription);

  delay(100);

  uint8_t mac[6];
  WiFi.macAddress(mac);
  client_id += "TempSensorEsp32_";
  for(int i = 0; i < 6; i++)
    client_id += String(mac[i], 16);

  client.connect(client_id.c_str(), mqtt_user, mqtt_password);
  client.subscribe("sorenlk@tutamail.com/forward");
  client.subscribe("sorenlk@tutamail.com/stop");
  client.subscribe("sorenlk@tutamail.com/reverse");
  client.subscribe("sorenlk@tutamail.com/turnleft");
  client.subscribe("sorenlk@tutamail.com/turnright");
  client.subscribe("sorenlk@tutamail.com/degree");

}

void moveServoTo(int angle) {
  angle = constrain(angle, 0, 180);
  int step = (angle > currentAngle) ? 1 : -1;
  for (int pos = currentAngle; pos != angle; pos += step) {
    sg90.write(pos);
    delay(15);
  }
  sg90.write(angle);
  currentAngle = angle;
}

void sweepServo() {
  for (int pos = 0; pos <= 180; pos += 2) { sg90.write(pos); delay(15); }
  for (int pos = 180; pos >= 0; pos -= 2) { sg90.write(pos); delay(15); }
  currentAngle = 0;
}


void updateMotors() {

  switch (movementMode) {

    case 0: // stop
      stepper1.setSpeed(0);
      stepper2.setSpeed(0);
      break;

    case 1: // forward
      stepper1.setSpeed(motorSpeed);
      stepper2.setSpeed(-motorSpeed);
      break;

    case 2: // turn left
      stepper1.setSpeed(motorSpeed);
      stepper2.setSpeed(motorSpeed);
      break;
    
    case 3: // turn right
      stepper1.setSpeed(-motorSpeed);
      stepper2.setSpeed(-motorSpeed);
      break;

    case 4: // reverse
      stepper1.setSpeed(-motorSpeed);
      stepper2.setSpeed(motorSpeed);
      break;
  }
}


void loop() 
{
  // Must call these as often as possible — this is what actually steps the motors
  stepper1.runSpeed();
  stepper2.runSpeed();


  
  // Serif 

  imu_loop();
  //movement_info();

  //plotter //////////////////////
  Serial.print(imu.gyro.z.value);
  //Serial.print(" ");
  //Serial.print(imu.gyro.z.meas);
  Serial.print(" ");
  Serial.print(imu.gyro.z.offset);
  Serial.println(" ");
  
  // Serif 

  client.loop();
}

// Function called when subscriped topic is received
void subscription(char *topic, byte *payload, unsigned int length)
{
  Serial.print("MQTT RX: ");
  Serial.println(topic);

  if (strcmp(topic, "sorenlk@tutamail.com/Mek26") == 0) {
    Serial.println("YAY");
  }

  if (strcmp(topic, "sorenlk@tutamail.com/degree") == 0) {
    int newDegree = (payload[0]);
    payload[length] = '\0'; // null-terminate the payload
    newDegree = atoi((char*)payload);
    moveServoTo(newDegree);
    Serial.println(newDegree);
  }


  bool isTrue = (payload[0] == 't'); // "true" or "false"

  if (strcmp(topic, "sorenlk@tutamail.com/stop") == 0) {
    movementMode = 0;
    updateMotors();
  }

  if (strcmp(topic, "sorenlk@tutamail.com/forward") == 0) {
    movementMode = 1;
    updateMotors();
  }

  if (strcmp(topic, "sorenlk@tutamail.com/turnleft") == 0) {
    movementMode = 2;
    updateMotors();
  }


  if (strcmp(topic, "sorenlk@tutamail.com/turnright") == 0) {
    movementMode = 3;
    updateMotors();
  }

  if (strcmp(topic, "sorenlk@tutamail.com/reverse") == 0) {
    movementMode = 4;
    updateMotors();
  }

}
