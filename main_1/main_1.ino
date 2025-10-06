/* ESP32: PID + Web UI via LittleFS + WebSocket (PWM com analogWrite)
   Libs:
     - WebSockets by Markus Sattler
     - ArduinoJson
     - QTRSensors
     - LittleFS_esp32 by lorol
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
float porCurva = 75.0;
float porReta = 85.0;
float porMax = porReta;


const float compEixo = 0.2f;
const float rodaR    = 0.016f;
float wE, wD;
float wMaxE = 100.00f, wMaxD = 100.00f;
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
float Kp = 4.0f; int multiP = -1;
float Ki = 0.0f; int multiI = 1;
float Kd = 5.0f; int multiD = -2;

// Derivativo filtrado
unsigned long lastMicros = 0;
float dt = 0.01f;
float derivRaw = 0.0f;
float derivFiltered = 0.0f;
float derivTau = 0.02f;

// ================= Mode / Signal =================
// modos do seguidor
enum FollowerMode { MODE_NEUTRO = 0, MODE_FRENTE = 1, MODE_CURVA = 2 };
FollowerMode modeAtual = MODE_NEUTRO;

// Sinais laterais
// tempo (ms) que bloqueia novas detecções após reconhecer AMBOS
const unsigned long BOTH_SUPPRESS_MS = 250;
unsigned long bothSuppressUntil = 0;
const uint16_t SINAL_LIMTE = 500;
#define CONSECUTIVO 1            // mantém ou ajuste
uint8_t esqConsecutivo = 0, dirConsecutivo = 0;
enum SignalState { SIN_NADA = 0, SIN_ESQ = 1, SIN_DIR = 2, SIN_AMBOS = 3 };
SignalState sinalAtual = SIN_NADA, sinalAnterior = SIN_NADA;

// auxiliares para lógica de modos
unsigned long curvaEnteredAt = 0;
int rightCountWhileFrente = 0; // conta direitas enquanto em FRENTE

// --- novo: estatística do erro enquanto em curva
int maxAbsErrorSinceCurve = 0;
const unsigned long CURVA_STABLE_WINDOW_MS = 1200; // janela usada para decisão
const int CURVA_ERROR_THRESHOLD = 500;           // threshold para "ainda em curva"

// Flag de run/stop — **começa parado** conforme pedido
volatile bool running = false;

// ===== WiFi + HTTP + WS =====
const char* ap_ssid     = "PID_Robot";
const char* ap_password = "12345678";
WebServer server(80);
WebSocketsServer webSocket(81);

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

  // adiciona o mode como string para a UI inicial saber qual é o modo
  switch (modeAtual) {
    case MODE_NEUTRO: doc["mode"] = "neutro"; break;
    case MODE_FRENTE: doc["mode"] = "frente"; break;
    case MODE_CURVA:  doc["mode"] = "curva";  break;
    default:          doc["mode"] = "neutro"; break;
  }

  String out;  serializeJson(doc, out);
  webSocket.sendTXT(clientNum, out);
}

void broadcastParams() {
  uint8_t nc = webSocket.connectedClients();
  for (uint8_t i = 0; i < nc; ++i) sendParamsToClient(i);
}

// envia modo atual (substitui envio de 'signal')
void broadcastMode() {
  StaticJsonDocument<128> doc;
  doc["type"] = "mode";
  switch(modeAtual) {
    case MODE_NEUTRO: doc["mode"] = "neutro"; break;
    case MODE_FRENTE: doc["mode"] = "frente"; break;
    case MODE_CURVA:  doc["mode"] = "curva";  break;
  }
  doc["running"] = running ? 1 : 0;
  String out; serializeJson(doc, out);
  webSocket.broadcastTXT(out);
}

// =================== aplica transição de modo a partir do sinal detectado
void applySignalToMode(SignalState s) {
  // s representa o sinal recém-confirmado (SIN_ESQ, SIN_DIR, SIN_AMBOS, SIN_NADA)
  unsigned long now = millis();

  // NADA não altera modos (apenas usado p/ transições dependendo do caso)
  if (s == SIN_NADA) {
    // não altera modo; apenas retorna
    return;
  }

  if (s == SIN_AMBOS && running == true) {
    // sempre vira pra FRENTE (e reinicia contadores)
    modeAtual = MODE_FRENTE;
    maxAbsErrorSinceCurve = 0;
    porMax = porReta;
    running = true;
    rightCountWhileFrente = 0;
    // reset curva timer
    curvaEnteredAt = 0;
    Serial.println("[MODE] AMBOS -> FRENTE");
    broadcastMode();
    return;
  }

  if (s == SIN_DIR && running == true) { // direita
    if (modeAtual == MODE_NEUTRO) {
      // primeira direita: inicia percurso -> FRENTE
      modeAtual = MODE_FRENTE;
      maxAbsErrorSinceCurve = 0;
      porMax = porReta;
      rightCountWhileFrente = 1; // primeira direita
      curvaEnteredAt = 0;
      Serial.println("[MODE] NEUTRO + DIR -> FRENTE (1)");
      broadcastMode();
      return;
    } else if (modeAtual == MODE_FRENTE || modeAtual == MODE_CURVA) {
      // se já estiver em FRENTE, incrementa contador: se >=2 -> para e volta NEUTRO
      rightCountWhileFrente++;
      Serial.printf("[MODE] DIR while FRENTE -> count=%d\n", rightCountWhileFrente);
      if (rightCountWhileFrente >= 2) {
        modeAtual = MODE_NEUTRO;
        porMax = porReta;
        maxAbsErrorSinceCurve = 0;
        //running = false;
        rightCountWhileFrente = 0;
        curvaEnteredAt = 0;
        Serial.println("[MODE] 2ª DIR detectada -> PARAR + NEUTRO");
        broadcastMode();
      }
      return;
    }
    return;
  }

  if (s == SIN_ESQ && running == true) { // esquerda
    if (modeAtual != MODE_CURVA) {
      // entrar em CURVA
      modeAtual = MODE_CURVA;
      porMax = porCurva;
      curvaEnteredAt = now;
      maxAbsErrorSinceCurve = abs(erro); // reinicia estatística
      // ao entrar em curva, resetamos contador de direitas
      rightCountWhileFrente = 0;
      Serial.println("[MODE] qualquer -> CURVA (entrando)");
      broadcastMode();
      return;
    } else {
      // já está em CURVA e recebeu outra esquerda: decidir se volta pra FRENTE
      unsigned long sinceCurve = now - curvaEnteredAt;

      // Se dentro da janela curta e o máximo erro observado ainda é grande,
      // mantemos CURVA. Caso contrário, vamos pra FRENTE.
      if ((sinceCurve <= CURVA_STABLE_WINDOW_MS) && (maxAbsErrorSinceCurve > CURVA_ERROR_THRESHOLD)) {
        // permanecer em CURVA
        Serial.printf("[MODE] CURVA: nova ESQ recebida mas maxAbsErr=%d (> %d) dentro %lums -> permanecer CURVA\n",
                      maxAbsErrorSinceCurve, CURVA_ERROR_THRESHOLD, sinceCurve);
        return;
      } else {
        // volta pra FRENTE
        modeAtual = MODE_FRENTE;
        porMax = porReta;
        running = true;
        rightCountWhileFrente = 0;
        curvaEnteredAt = 0;
        maxAbsErrorSinceCurve = 0; // reset
        Serial.println("[MODE] CURVA + ESQ -> FRENTE");
        broadcastMode();
        return;
      }
    }
  }
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
    // ao alternar running manualmente, não alteramos mode automaticamente
    broadcastParams();
    // enviamos também o mode para a UI ficar consistente
    broadcastMode();
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
  float porE = constrain(mapf(we, 0.0f, wMaxE, 20.0f, 100.0f), 0.0f, porMax);
  float porD = constrain(mapf(wd, 0.0f, wMaxD, 20.0f, 100.0f), 0.0f, porMax);

  int maxDuty = (1 << res) - 1;
  int dutyE = constrain((int)mapf(porE, 0.0f, 100.0f, 0.0f, (float)maxDuty), 0, maxDuty);
  int dutyD = constrain((int)mapf(porD, 0.0f, 100.0f, 0.0f, (float)maxDuty), 0, maxDuty);

  analogWrite(PwmPinME, dutyE);
  analogWrite(PwmPinMD, dutyD);
}

// =================== Sinais laterais ===================
void checkSinais() {
  bool esqCond = false, dirCond = false;

  // atualiza contadores por lado (usa os teus sensores 0/1 e 15/14)
  if (sensorValues[0] < SINAL_LIMTE && sensorValues[1] < SINAL_LIMTE) {
    esqConsecutivo = min<uint8_t>(CONSECUTIVO, esqConsecutivo + 1);
  } else {
    esqConsecutivo = 0;
  }
  esqCond = (esqConsecutivo >= CONSECUTIVO);

  if (sensorValues[15] < SINAL_LIMTE && sensorValues[14] < SINAL_LIMTE) {
    dirConsecutivo = min<uint8_t>(CONSECUTIVO, dirConsecutivo + 1);
  } else {
    dirConsecutivo = 0;
  }
  dirCond = (dirConsecutivo >= CONSECUTIVO);

  unsigned long now = millis();

  // verifica condição extra para 'AMBOS' (sensores 5,6,9,10 abaixo do limite)
  bool bothExtra = (sensorValues[5]  < SINAL_LIMTE) &&
                   (sensorValues[6]  < SINAL_LIMTE) &&
                   (sensorValues[9]  < SINAL_LIMTE) &&
                   (sensorValues[10] < SINAL_LIMTE);

  // calcula candidato imediato:
  SignalState candidate;
  if      (bothExtra)                      candidate = SIN_AMBOS;
  else if (esqCond)                        candidate = SIN_ESQ;
  else if (dirCond)                        candidate = SIN_DIR;
  else                                     candidate = SIN_NADA;

  // Se recentemente detectámos AMBOS, bloqueamos reconhecimento de outros sinais
  if (now < bothSuppressUntil) {
    if (candidate != SIN_AMBOS) {
      // ignorar mudanças até supressão passar (mas permitir AMBOS se reaparecer)
      return;
    }
    // se candidate == AMBOS, deixaremos processar mais abaixo
  }

  // se candidato mudou (ou se é AMBOS e for imediato), processar
  if (candidate != sinalAtual) {
    sinalAnterior = sinalAtual;
    sinalAtual = candidate;

    // quando for AMBOS, iniciar supressão
    if (sinalAtual == SIN_AMBOS) {
      bothSuppressUntil = now + BOTH_SUPPRESS_MS;
    }

    // aplicar modo baseado no sinal confirmado
    applySignalToMode(sinalAtual);
  }
}

void broadcastSignal(SignalState s){
  // OBS: função mantida apenas para compatibilidade - NÃO usaremos mais 'signal' no front.
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
  d["posicao"] = posicao;
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

  // atualiza estatística de erro se estivermos em CURVA
  if (modeAtual == MODE_CURVA) {
    int a = abs(erro);
    if (a > maxAbsErrorSinceCurve) maxAbsErrorSinceCurve = a;
  }

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
