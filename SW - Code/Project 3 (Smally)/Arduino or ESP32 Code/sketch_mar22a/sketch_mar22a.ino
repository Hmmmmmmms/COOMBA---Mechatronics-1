# include <AccelStepper.h>
# include <WiFi.h>
# include <Arduino.h>

// Use HALFSTEP for smoother/quieter 28BYJ-48 (most common choice)
// Use FULLSTEP if you need max torque instead
//#define HALFSTEP 8
#define FULLSTEP 4   // uncomment if preferred

#define DIRA 32
#define DIRB 33

// 2048 steps/rev in FULLSTEP mode
// 4096 steps/rev in HALFSTEP mode
const long STEPS_PER_REV = 2048; // ← use 4096 if you switch to HALFSTEP
const char *ssid = "Monkey";
const char *password = "monkenet";
const int TRIG_PIN = 5;     // HC-SR04 Trigger
const int ECHO_PIN = 19;    // HC-SR04 Echo (after voltage divider)
const int motorSpeed = 600;

// setup
int step1[] = {13,14,27,26};
int step2[] = {4,16,17,18};
int arrsize = sizeof(step1)/sizeof(step1[0]);
int motorDirection = 1;   // 1 = forward, -1 = reverse
int movementMode = 0; // 0 = stop, 1 = forward, 2 = turn left, 3 = turn right

// Motor 1 on pins 4,6,5,7 (IN1, IN3, IN2, IN4 order — important!)
AccelStepper stepper1(FULLSTEP, 13, 27, 14, 26);
// Motor 2
AccelStepper stepper2(FULLSTEP, 4, 17, 16, 18);


WiFiServer server(80); //create a server

void setup() {
  Serial.begin(921600); //Previously 9600

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

  // Connect to WiFi network
  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
 
  WiFi.begin(ssid, password); // attempt to connect
 
  while (WiFi.status() != WL_CONNECTED) { //wait to connect
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
 
  // Start the server
  server.begin();
  Serial.println("Server started");
 
  // Print the IP address
  Serial.print("Use this URL to connect: ");
  Serial.print("http://");
  Serial.print(WiFi.localIP());
  Serial.println("/");



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
}

long getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000);        // timeout ≈ 5 m
  long distanceCm = duration / 58;                       // same formula as your original lesson

  if (distanceCm <= 0 || distanceCm > 400) return -1;    // invalid reading
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
    
    case 3: // turn right
      stepper1.setSpeed(-motorSpeed);
      stepper2.setSpeed(-motorSpeed);
      break;
  }
}



void loop() {

  // Must call these as often as possible — this is what actually steps the motors
  stepper1.runSpeed();
  stepper2.runSpeed();

  // Optional: you can change speed anytime (e.g. from buttons, serial, etc.)
  // Example:
  // if (someCondition) {
  //   stepper1.setSpeed(600);
  //   stepper2.setSpeed(600);
  // }

  // Check if a client has connected
  WiFiClient client = server.available();
  if (!client) {
    return;
  }
 
  // Wait until the client sends some data
  Serial.println("new client");
  while(!client.available()){
    delay(1);
  }
 
  // Read the first line of the request
  String request = client.readStringUntil('\r');
  Serial.println(request);
  //client.flush();
 
  // Match the request
  int value = LOW;
  if (request.indexOf("/LED=ON") != -1)  {
    digitalWrite(DIRA,HIGH);
    digitalWrite(DIRB,LOW);
    
    value = HIGH;
  }
  if (request.indexOf("/LED=OFF") != -1)  {
    digitalWrite(DIRA,LOW);
    digitalWrite(DIRB,LOW);

    value = LOW;
  }

  if (request.indexOf("/STOP") != -1) {
    movementMode = 0;
    updateMotors();
  }

  if (request.indexOf("/FORWARD") != -1) {
    movementMode = 1;
    updateMotors();
  }

  if (request.indexOf("/LEFT") != -1) {
    movementMode = 2;
    updateMotors();
  }

  if (request.indexOf("/LEFT") != -1) {
    movementMode = 3;
    updateMotors();
  }
 
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 1000) {
    long dist = getDistance();
    Serial.print("Distance: ");
    Serial.print(dist > 0 ? dist : -1);
    Serial.println(" cm");
    lastPrint = millis();
  }

  // Set ledPin according to the request
 
  // Return the response
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/html");
  client.println(""); //  do not forget this one
  client.println("<!DOCTYPE HTML>");
  client.println("<html>");
 
  client.print("Led pin is now: ");
 
  if(value == HIGH) {
    client.print("On");
  } else {
    client.print("Off");
  }
  client.println("<br><br>");
  client.println("<a href=\"/LED=ON\"\"><button>Turn On </button></a>");
  client.println("<a href=\"/LED=OFF\"\"><button>Turn Off </button></a><br />");  
  client.println("<a href=\"/STOP\"><button>Stop</button></a>");
  client.println("<a href=\"/FORWARD\"><button>Forward</button></a>");
  client.println("<a href=\"/LEFT\"><button>Left</button></a>");
  client.println("</html>");
  
  if (client) {
    Serial.println("New client");
    String currentLine = "";

    while (client.connected()) {
        if (client.available()) {
          char c = client.read();
          if (c == '\n') {
            if (currentLine.length() == 0) {
              long distance = getDistance();

              // HTTP response
              client.println("HTTP/1.1 200 OK");
              client.println("Content-type:text/html");
              client.println("Connection: close");
              client.println();

              // HTML page with auto-refresh every 2 seconds
              client.println("<!DOCTYPE HTML><html>");
              client.println("<head><meta http-equiv=\"refresh\" content=\"2\"></head>");
              client.println("<body><h1>ESP32 + HC-SR04 Distance Sensor</h1>");
              client.print("<p><strong>Current distance: ");
              if (distance > 0) {
                client.print(distance);
                client.println(" cm</strong></p>");
              } else {
                client.println("Out of range</strong></p>");
              }
              client.println("<p><a href=\"/\">Manual Refresh</a></p>");
              client.println("</body></html>");
              break;
            } else {
              currentLine = "";
            }
          } else if (c != '\r') {
            currentLine += c;
          }
        }
      }

    delay(1);
    Serial.println("Client disonnected");
    Serial.println("");
  
}}