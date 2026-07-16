#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <RoboCore_Vespa.h>

// ------- CONFIGURAÇÕES DA REDE/WIFI -------
const char* ssid = "iPhone de Alexandre A G";
const char* password = "alexagc41";
const char* mqtt_server = "172.20.10.5";
const int   mqtt_port  = 1883;
const char* mqtt_user  = "";
const char* mqtt_pass  = "";
const char* topic_subscribe = "robo/seta";

WiFiClient espClient;
PubSubClient client(espClient);
VespaMotors motores;

unsigned long lastReconnectAttempt = 0;

String ultimoComando = ""; // variável global no topo do arquivo

void interpretarComando(String comando, int velocidade, int angulo) {
  // Evita repetir o mesmo comando várias vezes
  if (comando == ultimoComando) {
    return;
  }
  ultimoComando = comando;

  Serial.println("comando recebido: " + comando);

  if (comando == "PARAR") {
    motores.stop();
  } 
  else if (comando == "FRENTE") {
    motores.turn(velocidade, velocidade);
  } 
  else if (comando == "VIRAR_ESQ_90" || comando == "VIRAR_DIR_90") {
    // Direção do giro
    int dir = (comando == "VIRAR_ESQ_90") ? -1 : 1;

    // Gira com velocidade fixa, por tempo proporcional ao ângulo
    motores.turn(velocidade * dir, -velocidade * dir);

    // Duração baseada na potência e ângulo (ajustável por testes reais)
    int tempo_ms = map(abs(angulo), 0, 180, 0, 1950); // ex: 90 graus = ~400ms
    delay(tempo_ms);

    motores.stop();
  }
  else {
    motores.stop(); // segurança
  }
}


void callback(char* topic, byte* payload, unsigned int length) {
  payload[length] = '\0';
  StaticJsonDocument<128> doc;
  DeserializationError err = deserializeJson(doc, payload);
  Serial.println((char*)payload);
  if (!err && doc.containsKey("comando")) {
    String comando = doc["comando"];
    int velocidade = doc["velocidade"] | 0;
    int angulo     = doc["angulo"]     | 0;
    interpretarComando(comando, velocidade, angulo);
  }
}

boolean reconnect() {
  if (!client.connected()) {
    String clientId = "robo-seta-" + String((uint32_t)ESP.getEfuseMac(), HEX);
    Serial.print("Tentando conectar MQTT como ");
    Serial.println(clientId);
    
    if (client.connect(clientId.c_str(), mqtt_user, mqtt_pass)) {
      Serial.println("Conectado ao broker MQTT!");
      Serial.print("Inscrevendo no tópico: ");
      Serial.println(topic_subscribe);
      client.subscribe(topic_subscribe);
      return true;
    } else {
      Serial.print("Falha na conexão MQTT, código: ");
      Serial.println(client.state());
      return false;
    }
  }
  return true;  // já estava conectado
}


void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  Serial.print("Conectando ao Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWi-Fi conectado. IP: " + WiFi.localIP().toString());

  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  Serial.print("Tentando conectar no MQTT: ");
  Serial.println(mqtt_server);
  lastReconnectAttempt = 0;
}

void loop() {
  if (!client.connected()) {
    unsigned long now = millis();
    if (now - lastReconnectAttempt > 5000) {
      lastReconnectAttempt = now;
      if (reconnect()) lastReconnectAttempt = 0;
    }
  } else {
    client.loop();
  }
}
