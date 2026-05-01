#include <Arduino.h>
#include <WiFi.h>

const char* ssid     = "OttoS-net 2.4GHz";        // ← CHANGE THIS
const char* password = "?";    // ← CHANGE THIS 

const int TRIG_PIN = 5;     // HC-SR04 Trigger
const int ECHO_PIN = 18;    // HC-SR04 Echo (after voltage divider)

WiFiServer server(80);

void setup() {
  Serial.begin(115200);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Connect to WiFi
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected!");
  Serial.print("IP address: http://");
  Serial.println(WiFi.localIP());

  server.begin();
  Serial.println("Web server started");
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

void loop() {
  // ── Serial Monitor output (same as original lesson) ──
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 1000) {
    long dist = getDistance();
    Serial.print("Distance: ");
    Serial.print(dist > 0 ? dist : -1);
    Serial.println(" cm");
    lastPrint = millis();
  }

  // ── Web server (adapted from your LED WiFi example) ──
  WiFiClient client = server.available();
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
    delay(10);
    client.stop();
    Serial.println("Client disconnected");
    client.print("<p><strong>Out of range</strong></p>");
  }
}
