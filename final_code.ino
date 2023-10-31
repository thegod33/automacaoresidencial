#include <WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <ESP32Servo.h>
#include <BH1750.h>
#include <Wire.h>

const char* ssid = ""; // Substitua pelo seu SSID
const char* password = "";
const char* mqtt_server = "10.20.38.56"; // Substitua pelo endereço do seu servidor MQTT
const int mqtt_port = 1883; // A porta padrão para MQTT é 1883
const char* mqtt_topic_servo = "ramos33/control"; // Tópico MQTT para controlar o servo
const char* mqtt_switch_topic = "ramos33/switch"; // Tópico MQTT para o interruptor
const char* mqtt_topic = "ramos33/room"; // Tópico MQTT para os sensores de temperatura e proximidade
const char* mqtt_username = "";
const char* mqtt_password = "";

WiFiClient espClient;
PubSubClient client(espClient);

const int ledPin = 15; // Pino do LED controlado via MQTT
const int proximityLedPin = 16; // Pino do LED para detecção de proximidade
const int dhtPin = 26; // Pino ao qual o sensor DHT11 está conectado
const int trigPin = 4; // Pino de Trigger do sensor HCSR04
const int echoPin = 5; // Pino de Echo do sensor HCSR04
const int servoPin = 18;
Servo myservo;
BH1750 lightMeter;

DHT dht(dhtPin, DHT11);

bool isNear = false;
unsigned long previousMillis = 0;
const long ledActiveDuration = 20000; // 20 segundos em milissegundos

int switchValue = 0; // Estado inicial do interruptor (0 = desligado)
unsigned long lastLightReadTime = 0;
const unsigned long lightReadInterval = 10000; // Intervalo de leitura do sensor de luz (em milissegundos)

int servoAngle = 90; // Posição inicial do servo (meio caminho)

bool connectMQTT();

void setup_wifi() {
  delay(10);
  Serial.begin(115200);
  Serial.println();
  Serial.print("Conectando a ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi conectado");
  Serial.println("Endereço IP: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
  String payloadStr = "";
  for (int i = 0; i < length; i++) {
    payloadStr += (char)payload[i];
  }

  if (strcmp(topic, mqtt_topic_servo) == 0) {
    int angle = payloadStr.toInt();
    if (angle >= 0 && angle <= 180) {
      servoAngle = angle;
      myservo.write(servoAngle);
    }
  } else if (strcmp(topic, mqtt_switch_topic) == 0) {
    switchValue = payloadStr.toInt();
  } else if (strcmp(topic, mqtt_topic) == 0) {
    if (payload[0] == '1') {
      // Liga a lâmpada (quando comandada via MQTT)
      digitalWrite(ledPin, HIGH);
    } else if (payload[0] == '0') {
      // Desliga a lâmpada (quando comandada via MQTT)
      digitalWrite(ledPin, LOW);
    }
  }
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Conectando ao servidor MQTT...");
    if (client.connect("ESP32Client")) {
      Serial.println("conectado");
      client.subscribe(mqtt_topic_servo); // Alterado para mqtt_topic_servo
      client.subscribe(mqtt_switch_topic);
      client.subscribe(mqtt_topic);
    } else {
      Serial.print("Falha na conexão com o servidor MQTT. Tentando novamente em 2 segundos...");
      delay(2000);
    }
  }
}

void setup() {
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  pinMode(ledPin, OUTPUT);
  pinMode(proximityLedPin, OUTPUT);
  dht.begin();

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  myservo.attach(servoPin);
  Wire.begin();
  lightMeter.begin();
  Serial.println(F("BH1750 Test begin"));
  lastLightReadTime = millis(); // Inicialize o tempo da última leitura de luz
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();

  // Envia os dados de temperatura e umidade para o servidor MQTT
  char tempStr[8];
  char humStr[8];
  dtostrf(temperature, 1, 2, tempStr);
  dtostrf(humidity, 1, 2, humStr);
  client.publish("ramos33/room/temperature", tempStr);
  client.publish("ramos33/room/humidity", humStr);



  // Verifique o valor do interruptor para determinar se o sensor de luz deve ser lido
  if (switchValue == 1) {
    unsigned long currentTime = millis();
    if (currentTime - lastLightReadTime >= lightReadInterval) {
      float lux = lightMeter.readLightLevel();
      Serial.print("Light: ");
      Serial.print(lux);
      Serial.println(" lx");

      // Defina os limites para os níveis de luz e controle o servo
            if (lux >= 450) {
        servoAngle = 0; // Posição 0%
      } else if (lux < 450 && lux >= 185) {
        servoAngle = 90; // Posição 50%
      } else {
        servoAngle = 180; // Posição 100%
      }


      // Atualize a posição do servo
      myservo.write(servoAngle);

      lastLightReadTime = currentTime; // Atualize o tempo da última leitura de luz
    }
  }

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH);
  float distance = duration * 0.034 / 2;

  // Se algo estiver a uma distância inferior a 20 cm, acende o LED de proximidade
  if (distance < 20 && !isNear) {
    isNear = true;
    digitalWrite(proximityLedPin, HIGH);
    previousMillis = millis();
    Serial.println("Algo está perto! LED de proximidade aceso.");
    delay(1000); // Aguarde 1 segundo para evitar leituras repetidas
  } else if (distance >= 20 && isNear) {
    if (millis() - previousMillis >= ledActiveDuration) {
      isNear = false;
      digitalWrite(proximityLedPin, LOW);
      Serial.println("Nada está perto. LED de proximidade apagado.");
      delay(1000); // Aguarde 1 segundo para evitar leituras repetidas
    }
  }

  // Aguarde um curto período antes de repetir o loop
  delay(100);
}

