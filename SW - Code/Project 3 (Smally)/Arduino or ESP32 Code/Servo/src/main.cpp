#include <Arduino.h>
#include <WiFi.h>
#include <ESP32Servo.h>

const char* ssid     = "OttoS-net 2.4GHz";
const char* password = "testingofservo";

#define PIN_SG90 22
Servo sg90;
WiFiServer server(80);
int currentAngle = 90;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n\n=== ESP32 Servo WiFi Control Starting ===");

  sg90.setPeriodHertz(50);
  sg90.attach(PIN_SG90, 500, 2400);
  sg90.write(currentAngle);

  Serial.print("Connecting to: ");
  Serial.println(ssid);
  
  WiFi.begin(ssid, password);

  int timeout = 0;
  while (WiFi.status() != WL_CONNECTED && timeout < 40) {
    delay(500);
    Serial.print(".");
    timeout++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n✅ WiFi Connected!");
    Serial.print("IP Address → http://");
    Serial.println(WiFi.localIP());
    server.begin();
    Serial.println("Web server started! ✓");
  } else {
    Serial.println("\n❌ WiFi connection failed!");
    Serial.println("Check password or try restarting hotspot.");
  }
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

void loop() {
  WiFiClient client = server.available();
  if (client) {
    String request = "";
    while (client.connected() && !client.available()) delay(1);
    while (client.available()) {
      char c = client.read();
      request += c;
      if (c == '\n' && request.endsWith("\r\n\r\n")) break;
    }

    if (request.indexOf("/?pos=") != -1) {
      int val = request.substring(request.indexOf("/?pos=") + 6).toInt();
      moveServoTo(val);
    } else if (request.indexOf("/?sweep=1") != -1) {
      sweepServo();
    }

    // Send webpage
    client.println("HTTP/1.1 200 OK");
    client.println("Content-type:text/html");
    client.println("Connection: close");
    client.println();
    client.println("<!DOCTYPE HTML><html><head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
    client.println("<title>Servo Control</title><style>button{padding:15px 30px;margin:10px;font-size:18px;}</style></head><body>");
    client.println("<h1>ESP32 Servo Control</h1>");
    client.printf("<p><strong>Current Position: %d°</strong></p>", currentAngle);
    
    client.println("<p>");
    client.println("<a href=\"/?pos=0\"><button>0°</button></a>");
    client.println("<a href=\"/?pos=45\"><button>45°</button></a>");
    client.println("<a href=\"/?pos=90\"><button>90°</button></a>");
    client.println("<a href=\"/?pos=135\"><button>135°</button></a>");
    client.println("<a href=\"/?pos=180\"><button>180°</button></a>");
    client.println("</p>");
    
    client.println("<p><a href=\"/?sweep=1\"><button style=\"background:#4CAF50;color:white;\">FULL SWEEP</button></a></p>");
    client.println("<p><a href=\"/\"><button>Refresh Page</button></a></p>");
    client.println("</body></html>");

    delay(10);
    client.stop();
  }
}