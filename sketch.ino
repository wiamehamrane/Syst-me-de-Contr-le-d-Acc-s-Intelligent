#include <Keypad.h>
#include <WiFi.h>
#include <PubSubClient.h>

/* ================== PIN CONFIG ================== */
#define SERVO_PIN   18
#define BUZZER_PIN  19
#define GREEN_LED   21
#define RED_LED     22

/* ================== KEYPAD CONFIG ================== */
const byte ROWS = 4;
const byte COLS = 4;

char keys[ROWS][COLS] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};

byte rowPins[ROWS] = {32, 33, 25, 26};
byte colPins[COLS] = {27, 14, 12, 13};

Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

/* ================== ACCESS CONFIG ================== */
String correctCode = "7777";
String enteredCode = "";

/* ================== SERVO CONFIG ================== */
uint32_t dutyClosed = 3277;   // Porte fermée
uint32_t dutyOpen   = 6553;   // Porte ouverte
int servoChannel;

/* ================== WIFI & MQTT CONFIG ================== */
const char* ssid = "Wokwi-GUEST";
const char* password = "";

const char* mqttServer = "broker.hivemq.com";
const int mqttPort = 1883;

WiFiClient espClient;
PubSubClient client(espClient);

/* ================== FUNCTIONS ================== */

void connectWiFi() {
  Serial.print("Connecting to WiFi");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
}

void connectMQTT() {
  while (!client.connected()) {
    Serial.print("Connecting to MQTT...");
    if (client.connect("ESP32_Access_Control_Wiame")) {
      Serial.println("Connected");
    } else {
      Serial.print("Failed, rc=");
      Serial.print(client.state());
      delay(2000);
    }
  }
}

void buzzerBeep() {
  digitalWrite(BUZZER_PIN, HIGH);
  delay(80);
  digitalWrite(BUZZER_PIN, LOW);
}

void moveServo(uint32_t duty) {
  ledcWrite(servoChannel, duty);
}

void checkCode() {
  if (enteredCode == correctCode) {
    Serial.println("Access Authorized");
    digitalWrite(GREEN_LED, HIGH);
    moveServo(dutyOpen);

    client.publish("access/status", "AUTHORIZED");

    delay(3000);
    moveServo(dutyClosed);
    digitalWrite(GREEN_LED, LOW);
  } else {
    Serial.println("Access Denied");
    digitalWrite(RED_LED, HIGH);

    client.publish("access/status", "DENIED");

    delay(2000);
    digitalWrite(RED_LED, LOW);
  }
  enteredCode = "";
}

/* ================== SETUP ================== */
void setup() {
  Serial.begin(115200);

  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);

  servoChannel = ledcAttach(SERVO_PIN, 50, 16);

  connectWiFi();

  client.setServer(mqttServer, mqttPort);
  connectMQTT();

  Serial.println("System Ready - Enter Code:");
}

/* ================== LOOP ================== */
void loop() {
  if (!client.connected()) {
    connectMQTT();
  }
  client.loop();

  char key = keypad.getKey();

  if (key) {
    buzzerBeep();

    if (key == '#') {
      checkCode();
    }
    else if (key == '*') {
      enteredCode = "";
      Serial.println("Code cleared");
    }
    else {
      enteredCode += key;
      Serial.print("Entered: ");
      Serial.println(enteredCode);
    }
  }
}
