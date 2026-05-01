# include <DHT.h>
# include <WiFi.h>
# include <PubSubClient.h>
# include <esp_wifi.h>
# include <string.h>

// Jens

#include <AccelStepper.h>
#define HALFSTEP 8
#define DIRA 32
#define DIRB 33

// 2048 steps/rev in FULLSTEP mode
// 4096 steps/rev in HALFSTEP mode
const long STEPS_PER_REV = 4096;   // ← use 2048 if you switch to FULLSTEP
//const char *ssid = "Monkey";
//const char *password = "monkenet";
//const char *ssid = "SERIF WIFI";
//const char *password = "123456789";

const int TRIG_PIN = 19;     // HC-SR04 Trigger
const int ECHO_PIN = 21;    // HC-SR04 Echo (after voltage divider)
const int motorSpeed = 1000;

// setup
int step1[] = {13,14,27,26};
int step2[] = {4,16,17,18};
int arrsize = sizeof(step1)/sizeof(step1[0]);

// Motor 1 on pins 4,6,5,7 (IN1, IN3, IN2, IN4 order — important!)
AccelStepper stepper1(HALFSTEP, 13, 27, 14, 26);
// Motor 2
AccelStepper stepper2(HALFSTEP, 4, 17, 16, 18);

int motorDirection = 1;   // 1 = forward, -1 = reverse
int movementMode = 0; // 0 = stop, 1 = forward, 2 = turn left



// Jens


//# define DHT11PIN 18

//DHT dht(DHT11PIN, DHT11);

// Temp/humidity sensor data containers
//float T, H;
char buf[20];

// Flag to control data flow
bool transmit_data = false;

// Wifi login data
const char *ssid = "Pixel_5796";
const char *password = "internet";

// MQTT broker login data
const char *mqtt_user = "sorenlk@tutamail.com";
const char *mqtt_password = "*zMmQ4cvx0&n^R";

// Create wifi and mqtt connections
WiFiClient espcli;
PubSubClient client(espcli);

int loopcnt = 0;

String client_id;

int timer = 0;
int turntimer = 0;
bool detect = false;
int detectCount = 0;

bool power = false;

void setup() 
{

  //Serial.begin(115200);
  Serial.begin(921600);


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

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);


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
  client.subscribe("sorenlk@tutamail.com/data");
  client.subscribe("sorenlk@tutamail.com/Mek26");
  client.subscribe("sorenlk@tutamail.com/fan");
  client.subscribe("sorenlk@tutamail.com/fanOff");
  client.subscribe("sorenlk@tutamail.com/forward");
  client.subscribe("sorenlk@tutamail.com/stop");
  client.subscribe("sorenlk@tutamail.com/turn");

}


long getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000);        // timeout ≈ 5 m
  long distanceCm = duration / 58;                       // same formula as your original lesson

  //if (distanceCm <= 0 || distanceCm > 400) return -1;    // invalid reading
  return distanceCm;
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
  }
}


void loop() 
{
  // Must call these as often as possible — this is what actually steps the motors
  stepper1.runSpeed();
  stepper2.runSpeed();


  //delay(10);
  
  timer += 1;
  if (timer == 10000) {
    long dist = getDistance();
    //Serial.print("Distance: ");
    //Serial.print(dist);
    //Serial.println(" cm");
    //lastPrint = millis();

    //Serial.println("Reading sonic sensor");

    if (dist > 400 || dist <= 0) {
      dtostrf(400, 4, 1, buf);
      client.publish("sorenlk@tutamail.com/sonicsensor", buf);
    } else {
      dtostrf(dist, 4, 1, buf);
      client.publish("sorenlk@tutamail.com/sonicsensor", buf);
    }
    timer = 0;

    if (dist <= 20 && dist > 0 && detect == false && power) {
      detectCount += 1;
    }
  }

  if (detectCount >= 3) {
    detect = true;
    turntimer = 40000;
    detectCount = 0;
  }

  if (turntimer > 1) {
    Serial.println("turn");
    movementMode = 2;
    updateMotors();
    turntimer--;
  }

  if (turntimer == 1) {
    turntimer = 0;
    detect = false;
    Serial.println("forward");
    movementMode = 1;
    updateMotors();
  }

  

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

  bool isTrue = (payload[0] == 't'); // "true" or "false"

  if (strcmp(topic, "sorenlk@tutamail.com/fan") == 0) {
    if (isTrue && power) {
      Serial.println("fan on");
      digitalWrite(DIRA, HIGH);
      digitalWrite(DIRB, LOW);
    } else {
      Serial.println("fan off");
      digitalWrite(DIRA, LOW);
      digitalWrite(DIRB, LOW);
    }
  }

  if (strcmp(topic, "sorenlk@tutamail.com/forward") == 0) {
    if (isTrue) {
      Serial.println("turn on");
      movementMode = 1;
      updateMotors();
      power = true;
    } else {
      Serial.println("turn off");
      movementMode = 0;
      updateMotors();

      Serial.println("fan off");
      digitalWrite(DIRA, LOW);
      digitalWrite(DIRB, LOW);

      power = false;
    }
  }


  if (strcmp(topic, "sorenlk@tutamail.com/turn") == 0) {
    if (isTrue && power) {
      Serial.println("turning");
      movementMode = 2;
      updateMotors();
      power = true;
    } else {
      Serial.println("stop turning");
      movementMode = 0;
      updateMotors();
    }
  }


  /*if (strcmp(topic, "sorenlk@tutamail.com/fan") == 0) {
    Serial.println("fan on");
    digitalWrite(DIRA,HIGH);
    digitalWrite(DIRB,LOW);
  }

  if (strcmp(topic, "sorenlk@tutamail.com/fanOff") == 0) {
    Serial.println("fan off");
    digitalWrite(DIRA,LOW);
    digitalWrite(DIRB,LOW);
  }*/

  /*if (strcmp(topic, "sorenlk@tutamail.com/forward") == 0) {
    Serial.println("forward");
    movementMode = 1;
    updateMotors();
  }*/

  /*if (strcmp(topic, "sorenlk@tutamail.com/stop") == 0) {
    Serial.println("stop");
    movementMode = 0;
    updateMotors();
  }*/

  /*if (strcmp(topic, "sorenlk@tutamail.com/turn") == 0) {
    Serial.println("turn");
    movementMode = 2;
    updateMotors();
  }*/

}
