/* ESP32: PID + Web UI + WebSocket + DIREÇÃO H-Bridge + Calibração QTR assíncrona
   - Mantém analogWrite (sem ledcSetup/ledcAttach).
   - Envia "mode" e "calib" para a UI.
*/

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <QTRSensors.h>
#include <ArduinoJson.h>
#include <LittleFS.h>

// =================== Hardware & Controle ===================
// PWM (EN dos motores)
const int PwmPinME = 2;     // motor esquerdo (PWM)
const int PwmPinMD = 15;    // motor direito (PWM)
const int freq     = 10000; // apenas p/ cálculo do duty
const int res      = 10;    // bits (0..1023)

// Direção (ponte-H): MESMOS PINOS
const int dirRetoD   = 0;   // direita frente  (GPIO0: strap; atenção no boot)
const int dirAtrasD  = 3;   // direita ré     (RX0)
const int dirRetoE   = 23;  // esquerda frente
const int dirAtrasE  = 1;   // esquerda ré    (TX0)

// Velocidades/limites
float porCurva = 75.0;
float porReta  = 85.0;
float porMax   = 85.0;

const float compEixo = 0.2f;
const float rodaR    = 0.016f;
float wE, wD;
float wMaxE = 100.0f, wMaxD = 100.0f;
float wMinE = -100.0f, wMinD = -100.0f;
const float W_DEADBAND = 0.5f;

// QTR
QTRSensors qtr;
const uint8_t SensorCount = 16;
uint16_t sensorValues[SensorCount];
uint16_t posicao = 0;

// ===== PID =====
const int   ref = 7500;
float vel   = 0.7f;
float w;
float valorP, valorI, valorD;
long  Pterm, Dterm, Iterm;
int   erroAnt = 0;
int   erro    = 0;
float valorPID;
float g_pwmE_pct = 0.0f, g_pwmD_pct = 0.0f;  // 0..100
unsigned long g_lastTel = 0;
const unsigned long TEL_INTERVAL_MS = 150;    // 6-7 Hz de telemetria
// Mantissas + expoentes
float Kp = 4.0f; int multiP = -3;
float Ki = 0.0f; int multiI = 1;
float Kd = 5.0f; int multiD = -4;

// Derivativo filtrado
unsigned long lastMicros = 0;
float dt = 0.01f;
float derivRaw = 0.0f;
float derivFiltered = 0.0f;
float derivTau = 0.02f;

// ================= Mode / Signal =================
enum FollowerMode { MODE_NEUTRO = 0, MODE_FRENTE = 1, MODE_CURVA = 2 };
FollowerMode modeAtual = MODE_NEUTRO;

const unsigned long BOTH_SUPPRESS_MS = 250;
unsigned long bothSuppressUntil = 0;
const uint16_t SINAL_LIMTE = 500;
#define CONSECUTIVO 1
uint8_t esqConsecutivo = 0, dirConsecutivo = 0;
enum SignalState { SIN_NADA = 0, SIN_ESQ = 1, SIN_DIR = 2, SIN_AMBOS = 3 };
SignalState sinalAtual = SIN_NADA, sinalAnterior = SIN_NADA;

unsigned long curvaEnteredAt = 0;
int rightCountWhileFrente = 0;

int maxAbsErrorSinceCurve = 0;
const unsigned long CURVA_STABLE_WINDOW_MS = 1200;
const int CURVA_ERROR_THRESHOLD = 500;

volatile bool running = false;

// ===== WiFi + HTTP + WS =====
const char* ap_ssid     = "PID_Robot";
const char* ap_password = "12345678";
WebServer server(80);
WebSocketsServer webSocket(81);

// =================== Calibração (state machine) ===================
bool calibActive = false;
uint16_t calibStep = 0;
const uint16_t CALIB_STEPS = 400;          // mesmo volume do setup
const uint16_t CALIB_INTERVAL_MS = 5;      // fatiado para não travar WS/HTTP
unsigned long calibNextTick = 0;

// =================== Utils ===================
static inline float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
  if (in_max == in_min) return out_min;
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void sendJSON(const JsonDocument& doc) {
  String out; serializeJson(doc, out);
  webSocket.broadcastTXT(out);
}

void sendMode() {
  StaticJsonDocument<128> doc;
  doc["type"] = "mode";
  switch (modeAtual) {
    case MODE_NEUTRO: doc["mode"] = "neutro"; break;
    case MODE_FRENTE: doc["mode"] = "frente"; break;
    case MODE_CURVA:  doc["mode"] = "curva";  break;
  }
  doc["running"] = running ? 1 : 0;
  sendJSON(doc);
}

void sendCalib(const char* state) {
  StaticJsonDocument<64> doc;
  doc["type"] = "calib";
  doc["state"] = state; // "start" | "done" | "error"
  sendJSON(doc);
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

  switch (modeAtual) {
    case MODE_NEUTRO: doc["mode"] = "neutro"; break;
    case MODE_FRENTE: doc["mode"] = "frente"; break;
    case MODE_CURVA:  doc["mode"] = "curva";  break;
    default:          doc["mode"] = "neutro"; break;
  }

  String out; serializeJson(doc, out);
  webSocket.sendTXT(clientNum, out);
}

void broadcastParams() {
  uint8_t nc = webSocket.connectedClients();
  for (uint8_t i = 0; i < nc; ++i) sendParamsToClient(i);
}

// =================== aplica transição de modo ===================
void applySignalToMode(SignalState s) {
  unsigned long now = millis();
  if (s == SIN_NADA) return;

  if (s == SIN_AMBOS && running == true) {
    modeAtual = MODE_FRENTE;
    maxAbsErrorSinceCurve = 0;
    porMax = porReta;
    rightCountWhileFrente = 0;
    curvaEnteredAt = 0;
    Serial.println("[MODE] AMBOS -> FRENTE");
    sendMode();
    return;
  }

  if (s == SIN_DIR && running == true) {
    if (modeAtual == MODE_NEUTRO) {
      modeAtual = MODE_FRENTE;
      maxAbsErrorSinceCurve = 0;
      porMax = porReta;
      rightCountWhileFrente = 1;
      curvaEnteredAt = 0;
      Serial.println("[MODE] NEUTRO + DIR -> FRENTE (1)");
      sendMode();
      return;
    } else if (modeAtual == MODE_FRENTE || modeAtual == MODE_CURVA) {
      rightCountWhileFrente++;
      Serial.printf("[MODE] DIR while FRENTE -> count=%d\n", rightCountWhileFrente);
      if (rightCountWhileFrente >= 2) {
        modeAtual = MODE_NEUTRO;
        running = false;
        porMax = porReta;
        maxAbsErrorSinceCurve = 0;
        rightCountWhileFrente = 0;
        curvaEnteredAt = 0;
        Serial.println("[MODE] 2ª DIR -> NEUTRO");
        sendMode();
      }
      return;
    }
    return;
  }

  if (s == SIN_ESQ && running == true) {
    if (modeAtual != MODE_CURVA) {
      modeAtual = MODE_CURVA;
      porMax = porCurva;
      curvaEnteredAt = now;
      maxAbsErrorSinceCurve = abs(erro);
      rightCountWhileFrente = 0;
      Serial.println("[MODE] -> CURVA");
      sendMode();
      return;
    } else {
      unsigned long sinceCurve = now - curvaEnteredAt;
      if ((sinceCurve <= CURVA_STABLE_WINDOW_MS) && (maxAbsErrorSinceCurve > CURVA_ERROR_THRESHOLD)) {
        return;
      } else {
        modeAtual = MODE_FRENTE;
        porMax = porReta;
        rightCountWhileFrente = 0;
        curvaEnteredAt = 0;
        maxAbsErrorSinceCurve = 0;
        Serial.println("[MODE] CURVA -> FRENTE");
        sendMode();
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

    Serial.printf("[WS] update_digits -> Kp=%.3f(1e%d) Ki=%.3f(1e%d) Kd=%.3f(1e%d) vel=%.3f\n",
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
    sendMode();
    return;
  }

  // ===== NOVO: comando de calibração =====
  if (strcmp(t, "calibrate") == 0) {
    const char* action = doc["action"] | "";
    if (strcmp(action, "start") == 0) {
      if (!calibActive) {
        // pausa o seguidor; zera motores; inicia máquina de estados
        running = false;
        modeAtual = MODE_NEUTRO;
        analogWrite(PwmPinME, 0);
        analogWrite(PwmPinMD, 0);
        digitalWrite(dirRetoD, LOW); digitalWrite(dirAtrasD, LOW);
        digitalWrite(dirRetoE, LOW); digitalWrite(dirAtrasE, LOW);

        // (opcional, se sua versão do QTR tiver)
        #if defined(QTRSensors_h)
        // Muitas builds têm resetCalibration():
        // qtr.resetCalibration(); // descomente se disponível na sua lib
        #endif

        calibActive = true;
        calibStep = 0;
        calibNextTick = millis(); // dispara imediato
        sendCalib("start");
        sendMode();               // sinaliza NEUTRO/running=0 para a UI
        Serial.println("[CALIB] start");
      } else {
        // já em calibração — reenvia status
        sendCalib("start");
      }
    }
    return;
  }

  if (strcmp(t, "request_params") == 0) { sendParamsToClient(num); return; }
}

// =================== Direção: helpers ===================
inline void setDirLeft(int sign) {
  if (sign > 0) { digitalWrite(dirRetoE, HIGH); digitalWrite(dirAtrasE, LOW); }
  else if (sign < 0) { digitalWrite(dirRetoE, LOW); digitalWrite(dirAtrasE, HIGH); }
  else { digitalWrite(dirRetoE, LOW); digitalWrite(dirAtrasE, LOW); }
}
inline void setDirRight(int sign) {
  if (sign > 0) { digitalWrite(dirRetoD, HIGH); digitalWrite(dirAtrasD, LOW); }
  else if (sign < 0) { digitalWrite(dirRetoD, LOW); digitalWrite(dirAtrasD, HIGH); }
  else { digitalWrite(dirRetoD, LOW); digitalWrite(dirAtrasD, LOW); }
}

// =================== Motores (PWM + direção) ===================
void motor(float we, float wd) {
  if (!running || calibActive) {
    analogWrite(PwmPinME, 0);
    analogWrite(PwmPinMD, 0);
    setDirLeft(0);
    setDirRight(0);
    return;
  }

  we = constrain(we, wMinE, wMaxE);
  wd = constrain(wd, wMinD, wMaxD);
  if (fabsf(we) < W_DEADBAND) we = 0.0f;
  if (fabsf(wd) < W_DEADBAND) wd = 0.0f;

  int signE = (we > 0) - (we < 0);
  int signD = (wd > 0) - (wd < 0);
  setDirLeft(signE);
  setDirRight(signD);

  float we_abs = fabsf(we);
  float wd_abs = fabsf(wd);

  float porE = (we_abs <= 0.0f) ? 0.0f : constrain(mapf(we_abs, 0.0f, wMaxE, 20.0f, 100.0f), 0.0f, porMax);
  float porD = (wd_abs <= 0.0f) ? 0.0f : constrain(mapf(wd_abs, 0.0f, wMaxD, 20.0f, 100.0f), 0.0f, porMax);

  int maxDuty = (1 << res) - 1;
  int dutyE = (porE <= 0.0f) ? 0 : constrain((int)mapf(porE, 0.0f, 100.0f, 0.0f, (float)maxDuty), 0, maxDuty);
  int dutyD = (porD <= 0.0f) ? 0 : constrain((int)mapf(porD, 0.0f, 100.0f, 0.0f, (float)maxDuty), 0, maxDuty);

  analogWrite(PwmPinME, dutyE);
  analogWrite(PwmPinMD, dutyD);

  g_pwmE_pct = (maxDuty > 0) ? (100.0f * dutyE / maxDuty) : 0.0f;
  g_pwmD_pct = (maxDuty > 0) ? (100.0f * dutyD / maxDuty) : 0.0f;

  if (signE == 0 && signD == 0) { setDirLeft(0); setDirRight(0); }
}
void sendTelemetryPacket() {
  StaticJsonDocument<192> doc;
  doc["type"] = "telemetry";
  JsonObject d = doc.createNestedObject("data");
  d["posicao"] = posicao;         // já tinha
  d["wE"] = wE;                   // velocidade angular calculada
  d["wD"] = wD;
  d["pwmE"] = g_pwmE_pct;         // 0..100
  d["pwmD"] = g_pwmD_pct;
  sendJSON(doc);
}
// =================== Sinais laterais ===================
void checkSinais() {
  if (calibActive) return; // pausado durante calibração

  bool esqCond = false, dirCond = false;

  if (sensorValues[0] < SINAL_LIMTE && sensorValues[1] < SINAL_LIMTE) {
    esqConsecutivo = min<uint8_t>(CONSECUTIVO, esqConsecutivo + 1);
  } else { esqConsecutivo = 0; }
  esqCond = (esqConsecutivo >= CONSECUTIVO);

  if (sensorValues[15] < SINAL_LIMTE && sensorValues[14] < SINAL_LIMTE) {
    dirConsecutivo = min<uint8_t>(CONSECUTIVO, dirConsecutivo + 1);
  } else { dirConsecutivo = 0; }
  dirCond = (dirConsecutivo >= CONSECUTIVO);

  unsigned long now = millis();
  bool bothExtra = (sensorValues[5]  < SINAL_LIMTE) &&
                   (sensorValues[6]  < SINAL_LIMTE) &&
                   (sensorValues[9]  < SINAL_LIMTE) &&
                   (sensorValues[10] < SINAL_LIMTE);

  SignalState candidate;
  if      (bothExtra) candidate = SIN_AMBOS;
  else if (esqCond)   candidate = SIN_ESQ;
  else if (dirCond)   candidate = SIN_DIR;
  else                candidate = SIN_NADA;

  if (now < bothSuppressUntil) {
    if (candidate != SIN_AMBOS) return;
  }

  if (candidate != sinalAtual) {
    sinalAnterior = sinalAtual;
    sinalAtual = candidate;
    if (sinalAtual == SIN_AMBOS) bothSuppressUntil = now + BOTH_SUPPRESS_MS;
    applySignalToMode(sinalAtual);
  }
}

// =================== PID ===================
void pid() {
  if (calibActive) { // PID pausado na calibração
    analogWrite(PwmPinME, 0); analogWrite(PwmPinMD, 0);
    return;
  }

  unsigned long now = micros();
  dt = (now - lastMicros) / 1e6f; if (dt <= 0.0f) dt = 1e-6f;
  lastMicros = now;

  erro  = ref - (int)posicao;
  Pterm = erro;

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
  analogWrite(PwmPinME, 0);
  analogWrite(PwmPinMD, 0);

  pinMode(dirRetoD,  OUTPUT);
  pinMode(dirAtrasD, OUTPUT);
  pinMode(dirRetoE,  OUTPUT);
  pinMode(dirAtrasE, OUTPUT);
  digitalWrite(dirRetoD,  LOW); digitalWrite(dirAtrasD, LOW);
  digitalWrite(dirRetoE,  LOW); digitalWrite(dirAtrasE, LOW);

  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){4,16,17,5,18,19,21,22,32,33,25,26,27,14,12,13}, SensorCount);
  for (uint16_t i = 0; i < 400; i++) qtr.calibrate();

  WiFi.mode(WIFI_AP);
  bool ap_ok = WiFi.softAP(ap_ssid, ap_password);
  if (ap_ok) { IPAddress ip = WiFi.softAPIP(); Serial.print("AP: "); Serial.println(ip); }
  else { Serial.println("Falha ao iniciar AP"); }

  if (!LittleFS.begin(true)) Serial.println("ERRO: LittleFS não montou");
  else                      Serial.println("LittleFS OK");

  server.on("/", HTTP_GET, [](){
    if (LittleFS.exists("/index.html")) {
      File f = LittleFS.open("/index.html", "r");
      server.streamFile(f, "text/html");
      f.close();
    } else server.send(500, "text/plain", "index.html não encontrado");
  });
  server.begin();

  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  Serial.println("Servidor Web e WebSocket iniciados");
}

// =================== Loop ===================
void loop() {
  // Máquina de estados de calibração (não bloqueia)
  if (calibActive) {
    unsigned long now = millis();
    if (now >= calibNextTick) {
      qtr.calibrate();             // 1 passo
      calibStep++;
      calibNextTick = now + CALIB_INTERVAL_MS;

      if (calibStep >= CALIB_STEPS) {
        calibActive = false;
        sendCalib("done");
        Serial.println("[CALIB] done");
        // permanece em NEUTRO e running=false até usuário mandar "Andar"
        sendMode();
      }
    }
  } else {
    // Operação normal
    posicao = qtr.readLineWhite(sensorValues);
    checkSinais();
    pid();
    // Telemetria periódica
unsigned long nowMs = millis();
if (nowMs - g_lastTel >= TEL_INTERVAL_MS) {
  g_lastTel = nowMs;
  sendTelemetryPacket();
}
  }

  server.handleClient();
  webSocket.loop();
}
