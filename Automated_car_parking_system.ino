#include <PubSubClient.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <NewPing.h>

#define irSensor1 22
#define irSensor2 23
#define Slot1 18
#define Slot2 19
#define Slot3 21

#define SERVO_PIN 13
#define PIR_PIN 4

const int RED = 5;

const int trigPin = 26;
const int echoPin = 25;
const int buzzer = 27;

const char *ssid = "SLT-4G_B7E92"; // Enter your WiFi name
const char *password = "prolink12345";  // Enter WiFi password

const char* mqtt_server = "3.87.96.206";

WiFiClient espClient;
PubSubClient client(espClient);

Servo gateServo;
long lastMsg = 0; 

unsigned long gateOpenStartTime = 0; // Variable to store the gate open start time
const unsigned long gateOpenTime = 5000; // Gate open time in milliseconds (5 seconds)

bool autoCloseGate = true;

NewPing ultrasonicSensor(trigPin, echoPin);  // Ultrasonic sensor connected to digital pins 25 (Trig) and 26 (Echo)

void setup(){
  Serial.begin(115200);

  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  //sensors
  pinMode(irSensor1, INPUT);
  pinMode(irSensor2, INPUT);

  pinMode(Slot1, INPUT);
  pinMode(Slot2, INPUT);
  pinMode(Slot3, INPUT);

  pinMode(PIR_PIN, INPUT);
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input

  //actuators
  pinMode(RED, OUTPUT);
  pinMode(buzzer, OUTPUT);
  gateServo.attach(SERVO_PIN);

  closeGate();

}

void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

//mqtt (subscribe)
void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.println(topic);
  Serial.println("");
  Serial.print("Message: ");
  String messageTemp;
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();

  //door
  if (String(topic) == "door1/detector") {
    Serial.print("Changing output to ");
    if(messageTemp == "gate_open"){
      Serial.print("Gate Opened");
      gateOpenManually();
    }
    else if(messageTemp == "gate_close"){
      Serial.print("Gate Closed");
      closeGate();
    }
  }

  if (String(topic) == "door2/detector") {
    Serial.print("Changing output to ");
    if(messageTemp == "gate_open"){
      Serial.print("Door Opened");
      gateOpenManually();
    }
    else if(messageTemp == "gate_close"){
      Serial.print("Door Closed");
      closeGate();
    }
  }

  //door
  if (String(topic) == "gate/gateSwitch") {
    Serial.print("Changing output to ");
    if(messageTemp == "open"){
      Serial.print("Gate Opened");
      openGate();
      autoCloseGate = false;
    }
    else if(messageTemp == "close"){
      Serial.print("Gate Closed");
      closeGateManually();
      autoCloseGate = true;
    }
  }
//motion detector
  if (String(topic) == "motion/detector") {
    Serial.print("Changing output to ");
    if(messageTemp == "motion_detected"){
      Serial.println("Motion detected!");
      redOn();
    }
    else if(messageTemp == "motion_not_detected"){
      Serial.println("Motion not detected!");
      redOof();
    }
  }

 
  if (String(topic) == "distance/detector") {
    Serial.print("Changing output to ");
    if(messageTemp == "close"){
      Serial.println("Warning: too close!");
      buzzerOn();
    }
    else if(messageTemp == "not_close"){
      Serial.println("Not Close!");
      buzzerOof();
    }
  }

  if (String(topic) == "slot1/detector") {
    Serial.print("Changing output to ");
    if(messageTemp == "full"){
      Serial.println("Slot 1 is Full...!");
    }
    else if(messageTemp == "not_full"){
      Serial.println("Slot 1 is not Full..!");
    }
  }

  if (String(topic) == "slot2/detector") {
    Serial.print("Changing output to ");
    if(messageTemp == "full"){
      Serial.println("Slot 2 is Full...!");
    }
    else if(messageTemp == "not_full"){
      Serial.println("Slot 2 is not Full..!");
    }
  }

  if (String(topic) == "slot3/detector") {
    Serial.print("Changing output to ");
    if(messageTemp == "full"){
      Serial.println("Slot 3 is Full...!");
    }
    else if(messageTemp == "not_full"){
      Serial.println("Slot 3 is not Full..!");
    }
  }
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP8266Client")) {
      Serial.println("connected");
      // Subscribe
      client.subscribe("motion/detector");
      client.subscribe("door1/detector");
      client.subscribe("door2/detector");
      client.subscribe("distance/detector");

      client.subscribe("slot1/detector");
      client.subscribe("slot2/detector");
      client.subscribe("slot3/detector");

      client.subscribe("gate/gateSwitch");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  long now = millis();
  if (now - lastMsg > 1000) {
    lastMsg = now;

    //door component
    int IRSensor1 = digitalRead(irSensor1);
    Serial.println("");
    Serial.print("IRSensor1 sensor value: ");
    Serial.println(IRSensor1);
    Serial.println("");
    char door1[16];
    itoa(IRSensor1, door1, 10);
    client.publish("door1/servoGate", door1);

    //door component
    int IRSensor2 = digitalRead(irSensor2);
    Serial.println("");
    Serial.print("IRSensor2 sensor value: ");
    Serial.println(IRSensor2);
    Serial.println("");
    char door2[16];
    itoa(IRSensor2, door2, 10);
    client.publish("door2/servoGate", door2);

    //Slot1
    int slot1 = digitalRead(Slot1);
    Serial.println("");
    Serial.print("Slot1 sensor value: ");
    Serial.println(slot1);
    Serial.println("");
    char slot1_mqtt[16];
    itoa(slot1, slot1_mqtt, 10);
    client.publish("slot1/get", slot1_mqtt);

    //Slot2
    int slot2 = digitalRead(Slot2);
    Serial.println("");
    Serial.print("Slot2 sensor value: ");
    Serial.println(slot2);
    Serial.println("");
    char slot2_mqtt[16];
    itoa(slot2, slot2_mqtt, 10);
    client.publish("slot2/get", slot2_mqtt);

    //Slot3
    int slot3 = digitalRead(Slot3);
    Serial.println("");
    Serial.print("Slot3 sensor value: ");
    Serial.println(slot3);
    Serial.println("");
    char slot3_mqtt[16];
    itoa(slot3, slot3_mqtt, 10);
    client.publish("slot3/get", slot3_mqtt);

    //pir sensor
    int motion = digitalRead(PIR_PIN);
    Serial.print("motion = ");
    Serial.println(motion);
    Serial.println("");
    char pir[16];
    itoa(motion, pir, 10);
    client.publish("motion/get", pir);

    int distance = ultrasonicSensor.ping_cm();
    Serial.print("distance = ");
    Serial.println(distance);
    Serial.println("");
    char ultra[16];
    itoa(distance, ultra, 10);
    client.publish("distance/get", ultra);
  }
}

void openGate()
{
  gateServo.write(0); // the angle as needed to fully open the gate
  delay(1000); // Delay for stability
}

void closeGateManually()
{
  gateServo.write(90); // the angle as needed to fully open the gate
  delay(1000); // Delay for stability
}

void closeGate() {
  if (autoCloseGate) { // Check the flag before closing automatically
    if (gateOpenStartTime == 0) {
      gateOpenStartTime = millis();
    }

    if (millis() - gateOpenStartTime >= gateOpenTime) {
      gateServo.write(90); // Close the gate (assuming 180 is the angle for closed position)
      gateOpenStartTime = 0; // Reset the gate open start time for the next run
    }
  }
}

void gateOpenManually() {
  gateServo.write(0); // Open the gate (assuming 0 is the angle for open position)

  // Set the gate open start time if it's the first time the function is called
  if (gateOpenStartTime == 0) {
    gateOpenStartTime = millis();
  }

  // Check if the gate has been open for the specified duration (3 seconds)
  if (millis() - gateOpenStartTime >= gateOpenTime) {
    gateServo.write(90); // Close the gate (assuming 180 is the angle for closed position)
    gateOpenStartTime = 0; // Reset the gate open start time for the next run
  }
}

//on the led light
void redOn(){
  Serial.println("..................................lightOn");
  digitalWrite(RED, HIGH);
}

//off the led light
void redOof(){
  digitalWrite(RED, LOW);
}

void buzzerOn(){
  Serial.println("..................................buzzerOn");
  digitalWrite(buzzer, HIGH);
}

void buzzerOof(){
  digitalWrite(buzzer, LOW);
}