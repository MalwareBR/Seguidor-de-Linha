/* ESP32: PID + Web UI via LittleFS + WebSocket (PWM com analogWrite)
   Libs:
     - WebSockets by Markus Sattler
     - ArduinoJson
     - QTRSensors
     - LittleFS (ESP32 core)
*/

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <QTRSensors.h>
#include <ArduinoJson.h>
#include <LittleFS.h>

// =================== Hardware & Controle ===================
const int PwmPinME = 2;    // motor esquerdo (PWM)
const int PwmPinMD = 15;   // motor direito (PWM)
const int freq     = 10000; // Hz
const int res      = 10;    // bits (0..1023)

const float compEixo = 0.2f;
const float rodaR    = 0.016f;
float wE, wD;
float wMaxE = 85.87f, wMaxD = 86.56f;
float wMinE = 0.0f,  wMinD = 0.0f;

QTRSensors qtr;
const uint8_t SensorCount = 16;
uint16_t sensorValues[SensorCount];
uint16_t posicao;

// ===== PID =====
const int   ref = 7500;
float vel   = 0.8f;
float w;
float valorP, valorI, valorD;
long  Pterm, Dterm, Iterm;
int   erroAnt = 0;
int   erro    = 0;
float valorPID;

// Mantissas + expoentes
float Kp = 7.0f; int multiP = -3;
float Ki = 0.0f; int multiI = 1;
float Kd = 1.0f; int multiD = -3;

// Derivativo filtrado
unsigned long lastMicros = 0;
float dt = 0.01f;
float derivRaw = 0.0f;
float derivFiltered = 0.0f;
float derivTau = 0.02f;

// Sinais laterais
const uint16_t SINAL_LIMTE = 500;
const uint8_t  CONSECUTIVO = 2;
uint8_t esqConsecutivo = 0, dirConsecutivo = 0;
enum SignalState { SIN_NADA = 0, SIN_ESQ = 1, SIN_DIR = 2, SIN_AMBOS = 3 };
SignalState sinalAtual = SIN_NADA, sinalAnterior = SIN_NADA;

// ===== WiFi + HTTP + WS =====
const char* ap_ssid     = "PID_Robot";
const char* ap_password = "12345678";
WebServer server(80);
WebSocketsServer webSocket(81);

// Flag de run/stop
volatile bool running = true;

// Utils
static inline float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
  if (in_max == in_min) return out_min;
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// =================== WS: envio de parâmetros ===================
void sendParamsToClient(uint8_t clientNum){
  StaticJsonDocument<256> doc;
  doc["type"]    = "params";
  doc["Kp"]      = Kp;      doc["multiP"] = multiP;
  doc["Ki"]      = Ki;      doc["multiI"] = multiI;
  doc["Kd"]      = Kd;      doc["multiD"] = multiD;
  doc["vel"]     = vel;
  doc["running"] = running ? 1 : 0;
  String out;  serializeJson(doc, out);
  webSocket.sendTXT(clientNum, out);
}

void broadcastParams() {
  uint8_t nc = webSocket.connectedClients();
  for (uint8_t i = 0; i < nc; ++i) sendParamsToClient(i);
}

// =================== WS: eventos ===================
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  if (type == WStype_CONNECTED) { sendParamsToClient(num); return; }
  if (type != WStype_TEXT) return;

  StaticJsonDocument<384> doc;
  DeserializationError err = deserializeJson(doc, (const char*)payload, length);
  if (err) { Serial.printf("[WS] JSON err: %s\n", err.c_str()); return; }

  const char* t = doc["type"];
  if (!t) return;

  if (strcmp(t, "update_digits") == 0) {
    int kp0 = doc["kp0"] | 0, kp1 = doc["kp1"] | 0, kp2 = doc["kp2"] | 0, kp3 = doc["kp3"] | 0;
    int ki0 = doc["ki0"] | 0, ki1 = doc["ki1"] | 0, ki2 = doc["ki2"] | 0, ki3 = doc["ki3"] | 0;
    int kd0 = doc["kd0"] | 0, kd1 = doc["kd1"] | 0, kd2 = doc["kd2"] | 0, kd3 = doc["kd3"] | 0;
    int v0  = doc["v0"]  | 0, v1  = doc["v1"]  | 0, v2  = doc["v2"]  | 0;

    Kp = kp0 + kp1*0.1f + kp2*0.01f; multiP = kp3;
    Ki = ki0 + ki1*0.1f + ki2*0.01f; multiI = ki3;
    Kd = kd0 + kd1*0.1f + kd2*0.01f; multiD = kd3;
    vel = v0 + v1*0.1f + v2*0.01f;

    Serial.printf("[WS] update_digits -> Kp=%.6f(1e%d) Ki=%.6f(1e%d) Kd=%.6f(1e%d) vel=%.6f\n",
      Kp, multiP, Ki, multiI, Kd, multiD, vel);
    broadcastParams();
    return;
  }

  if (strcmp(t, "update") == 0) {
    if (doc.containsKey("Kp")) Kp = (float)doc["Kp"].as<double>();
    if (doc.containsKey("multiP")) multiP = doc["multiP"].as<int>();
    if (doc.containsKey("Ki")) Ki = (float)doc["Ki"].as<double>();
    if (doc.containsKey("multiI")) multiI = doc["multiI"].as<int>();
    if (doc.containsKey("Kd")) Kd = (float)doc["Kd"].as<double>();
    if (doc.containsKey("multiD")) multiD = doc["multiD"].as<int>();
    if (doc.containsKey("vel")) vel = (float)doc["vel"].as<double>();
    Serial.println("[WS] update (legacy) aplicado");
    broadcastParams();
    return;
  }

  if (strcmp(t, "toggle") == 0) {
    const char* s = doc["state"];
    running = (s && strcmp(s, "andar") == 0);
    Serial.printf("[WS] toggle -> running=%d\n", running ? 1 : 0);
    broadcastParams();
    return;
  }

  if (strcmp(t, "request_params") == 0) { sendParamsToClient(num); return; }
}

// =================== Motores (PWM com analogWrite) ===================
void motor(float we, float wd) {
  if (!running) {
    analogWrite(PwmPinME, 0);
    analogWrite(PwmPinMD, 0);
    return;
  }
  float porE = constrain(mapf(we, 0.0f, wMaxE, 20.0f, 100.0f), 0.0f, 78.0f);
  float porD = constrain(mapf(wd, 0.0f, wMaxD, 20.0f, 100.0f), 0.0f, 78.0f);

  int maxDuty = (1 << res) - 1;
  int dutyE = constrain((int)mapf(porE, 0.0f, 100.0f, 0.0f, (float)maxDuty), 0, maxDuty);
  int dutyD = constrain((int)mapf(porD, 0.0f, 100.0f, 0.0f, (float)maxDuty), 0, maxDuty);

  analogWrite(PwmPinME, dutyE);
  analogWrite(PwmPinMD, dutyD);
}

// =================== Sinais laterais ===================
void checkSinais() {
  bool esqCond=false, dirCond=false;

  if (sensorValues[0] < SINAL_LIMTE && sensorValues[1] < SINAL_LIMTE && sensorValues[2] < SINAL_LIMTE) {
    esqConsecutivo = min<uint8_t>(CONSECUTIVO, esqConsecutivo+1);
  } else esqConsecutivo = 0;
  esqCond = (esqConsecutivo >= CONSECUTIVO);

  if (sensorValues[15] < SINAL_LIMTE && sensorValues[14] < SINAL_LIMTE && sensorValues[13] < SINAL_LIMTE) {
    dirConsecutivo = min<uint8_t>(CONSECUTIVO, dirConsecutivo+1);
  } else dirConsecutivo = 0;
  dirCond = (dirConsecutivo >= CONSECUTIVO);

  if (esqCond && dirCond)      sinalAtual = SIN_AMBOS;
  else if (esqCond)            sinalAtual = SIN_ESQ;
  else if (dirCond)            sinalAtual = SIN_DIR;
  else                         sinalAtual = SIN_NADA;

  if (sinalAtual != sinalAnterior) {
    sinalAnterior = sinalAtual;
    switch (sinalAtual) {
      case SIN_NADA: Serial.println("Sinal: NADA"); break;
      case SIN_ESQ:  Serial.println("Sinal: ESQ detectou (0,1,2)"); break;
      case SIN_DIR:  Serial.println("Sinal: DIR detectou (15,14,13)"); break;
      case SIN_AMBOS:Serial.println("Sinal: AMBOS detectou"); break;
    }
    broadcastSignal(sinalAtual);
  }
  wsSendKV( esqCond);
}

void broadcastSignal(SignalState s){
  StaticJsonDocument<64> doc;
  doc["type"]="signal";
  doc["value"]=(int)s; // 0=NADA,1=ESQ,2=DIR,3=AMBOS
  String out; serializeJson(doc, out);
  webSocket.broadcastTXT(out);
}

void wsSendKV(bool esquerda) {
  StaticJsonDocument<192> doc;
  doc["type"] = "telemetry";
  JsonObject d = doc.createNestedObject("data");
  d["esquerda"] = esquerda;
  String out; serializeJson(doc, out);
  webSocket.broadcastTXT(out);
}

// =================== PID ===================
void pid() {
  unsigned long now = micros();
  dt = (now - lastMicros) / 1e6f; if (dt <= 0.0f) dt = 1e-6f;
  lastMicros = now;

  erro  = ref - (int)posicao;
  Pterm = erro;

  derivRaw = ((float)erro - (float)erroAnt) / dt;
  float alpha = expf(-dt / derivTau);
  derivFiltered = alpha * derivFiltered + (1.0f - alpha) * derivRaw;

  valorD = (Kd * powf(10.0f, (float)multiD)) * derivFiltered;
  valorP = (Kp * powf(10.0f, (float)multiP)) * (float)Pterm;

  long  ItermCandidate    = Iterm + erro;
  float valorI_candidate  = (Ki * powf(10.0f, (float)multiI)) * (float)ItermCandidate;
  float valorPID_candidate= valorP + valorI_candidate + valorD;

  float w_candidate  = valorPID_candidate;
  float wE_candidate = (vel - w_candidate * compEixo / 2.0f) / rodaR;
  float wD_candidate = (vel + w_candidate * compEixo / 2.0f) / rodaR;

  bool withinLeftLimits  = (wE_candidate >= wMinE) && (wE_candidate <= wMaxE);
  bool withinRightLimits = (wD_candidate >= wMinD) && (wD_candidate <= wMaxD);

  if (withinLeftLimits && withinRightLimits) {
    Iterm = ItermCandidate;  valorI = valorI_candidate;  valorPID = valorPID_candidate;
  } else {
    valorI = (Ki * powf(10.0f, (float)multiI)) * (float)Iterm;
    valorPID = valorP + valorI + valorD;
  }

  erroAnt = erro;
  w  = valorPID;
  wE = (vel - w * compEixo / 2.0f) / rodaR;
  wD = (vel + w * compEixo / 2.0f) / rodaR;

  motor(wE, wD);
}

// =================== Setup ===================
void setup() {
    
  Serial.begin(115200);
  lastMicros = micros();

  pinMode(PwmPinME, OUTPUT);
  pinMode(PwmPinMD, OUTPUT);
  // mantém o teu método de PWM (se já estava a funcionar)
  ledcAttach(PwmPinME, freq, res);
  ledcAttach(PwmPinMD, freq, res);
  ledcWrite(PwmPinME, 0);
  ledcWrite(PwmPinMD, 0);

  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){4,16,17,5,18,19,21,22,32,33,25,26,27,14,12,13}, SensorCount);
  for (uint16_t i = 0; i < 400; i++) qtr.calibrate();

  WiFi.mode(WIFI_AP);
  bool ap_ok = WiFi.softAP(ap_ssid, ap_password);
  if (ap_ok) {
    IPAddress ip = WiFi.softAPIP();
    Serial.print("AP iniciado. IP: ");
    Serial.println(ip);
  } else Serial.println("Falha ao iniciar AP");
  if (!LittleFS.begin(true)) {
    Serial.println("ERRO: LittleFS não montou");
  } else {
    Serial.println("LittleFS OK");
  }
   // HTTP: serve /index.html do LittleFS
  server.on("/", HTTP_GET, [](){
    if (LittleFS.exists("/index.html")) {
      File f = LittleFS.open("/index.html", "r");
      server.streamFile(f, "text/html");
      f.close();
    } else {
      server.send(500, "text/plain", "index.html não encontrado no LittleFS");
    }
  });

  //server.on("/", [](){ server.send_P(200, "text/html", index_html); });
  server.begin();

  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  Serial.println("Servidor Web e WebSocket iniciados");
}

// =================== Loop ===================
void loop() {
  posicao = qtr.readLineWhite(sensorValues);
  checkSinais();
  pid();

  server.handleClient();
  webSocket.loop();
}
