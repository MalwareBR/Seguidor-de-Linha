/* (seu cabeçalho e includes permanecem iguais) */

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <QTRSensors.h>

//------Variáveis dos motores-------//
const int PwmPinME = 2;   // Motor esquerdo
const int PwmPinMD = 15;  // Motor direito
const int freq = 10000;   // 10 kHz
const int res  = 10;      // 10 bits (0..1023)
const float compEixo = 0.2f;
const float rodaR = 0.016f;
float wE, wD;             // Velocidade angular atual dos motores
float wMaxE = 85.87f;     // Velocidade máxima do motor esquerdo
float wMaxD = 86.56f;     // Velocidade máxima do motor direito
float wMinE = 0.0f;       // Velocidade mínima (rad/s) - ajustar se necessário
float wMinD = 0.0f;       // Velocidade mínima (rad/s) - ajustar se necessário

//------Variáveis dos sensores-------//
QTRSensors qtr;
const uint8_t SensorCount = 16;
uint16_t sensorValues[SensorCount];
uint16_t posicao;

//------Variáveis do PID-------//
const int ref = 7500;
float vel = 0.8f;         // m/s
float w;                  // saída do PID (rad/s)
float valorP;
float valorI;
float valorD;
long Pterm, Dterm, Iterm;   // usar long para acumulador I
int erroAnt = 0;
int erro = 0;
float valorPID;
// Ganho de P
float Kp = 4.0f;
int multiP = -3;
// Ganho de I
float Ki = 0.0f;
int multiI = 1;
// Ganho de D
float Kd = 0.0f;
int multiD = 1;

// Configuração de Wi-Fi – altere o SSID/senha conforme necessário.
const char* ap_ssid     = "PID_Robot";
const char* ap_password = "12345678";

// Instancia um servidor HTTP na porta 80.
WebServer server(80);

// Função de utilidade: mapeia floats de um intervalo para outro.
static inline float mapf(float x, float in_min, float in_max,
                         float out_min, float out_max) {
  if (in_max == in_min) return out_min; // evita divisão por zero
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void handleRoot() {
  String page = "<!DOCTYPE html><html><head>";
  page += "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">";
  page += "<title>Controlador PID</title>";
  page += "<style>body{font-family:Arial,Helvetica,sans-serif;text-align:center;}";
  page += "label,input{display:block;margin:8px auto;font-size:16px;}";
  page += "input[type=text]{width:100px;padding:4px;}";
  page += "input[type=submit]{padding:6px 12px;}";
  page += "</style></head><body>";
  page += "<h2>Parâmetros PID e Velocidade</h2>";
  page += "<form action=\"/update\" method=\"get\">";
  page += "<label for=\"kp\">Kp:</label>";
  page += "<input type=\"text\" id=\"kp\" name=\"kp\" value=\"" + String(Kp, 4) + "\">";
  page += "<label for=\"multiP\">Multiplicador de P:</label>";
  page += "<input type=\"text\" id=\"multiP\" name=\"multiP\" value=\"" + String(multiP) + "\">";
  page += "<label for=\"ki\">Ki:</label>";
  page += "<input type=\"text\" id=\"ki\" name=\"ki\" value=\"" + String(Ki, 4) + "\">";
  page += "<label for=\"multiI\">Multiplicador de I:</label>";
  page += "<input type=\"text\" id=\"multiI\" name=\"multiI\" value=\"" + String(multiI) + "\">";
  page += "<label for=\"kd\">Kd:</label>";
  page += "<input type=\"text\" id=\"kd\" name=\"kd\" value=\"" + String(Kd, 4) + "\">";
  page += "<label for=\"multiD\">Multiplicador de D:</label>";
  page += "<input type=\"text\" id=\"multiD\" name=\"multiD\" value=\"" + String(multiD) + "\">";
  page += "<label for=\"vel\">Velocidade (m/s):</label>";
  page += "<input type=\"text\" id=\"vel\" name=\"vel\" value=\"" + String(vel, 3) + "\">";
  page += "<input type=\"submit\" value=\"Atualizar\"></form>";
  page += "</body></html>";
  server.send(200, "text/html", page);
}

void handleUpdate() {
  if (server.hasArg("kp")) {
    float tmp = server.arg("kp").toFloat();
    if (!isnan(tmp)) { Kp = tmp; }
  }
  if (server.hasArg("multiP")) {
    int tmp = server.arg("multiP").toInt();
    multiP = tmp;
  }
  if (server.hasArg("ki")) {
    float tmp = server.arg("ki").toFloat();
    if (!isnan(tmp)) { Ki = tmp; }
  }
  if (server.hasArg("multiI")) {
    int tmp = server.arg("multiI").toInt();
    multiI = tmp;
  }
  if (server.hasArg("kd")) {
    float tmp = server.arg("kd").toFloat();
    if (!isnan(tmp)) { Kd = tmp; }
  }
  if (server.hasArg("multiD")) {
    int tmp = server.arg("multiD").toInt();
    multiD = tmp;
  }
  if (server.hasArg("vel")) {
    float tmp = server.arg("vel").toFloat();
    if (!isnan(tmp)) { vel = tmp; }
  }
  server.sendHeader("Location", "/", true);
  server.send(302, "text/plain", "");
}

void motor(float we, float wd) {
  float porE = mapf(we, 0.0f, wMaxE, 20.0f, 100.0f);
  porE = constrain(porE, 0.0f, 100.0f);
  int maxDuty = (1 << res) - 1;
  int dutyE = (int)mapf(porE, 0.0f, 100.0f, 0.0f, (float)maxDuty);
  dutyE = constrain(dutyE, 0, maxDuty);
  ledcWrite(PwmPinME, dutyE);

  float porD = mapf(wd, 0.0f, wMaxD, 20.0f, 100.0f);
  porD = constrain(porD, 0.0f, 100.0f);
  int dutyD = (int)mapf(porD, 0.0f, 100.0f, 0.0f, (float)maxDuty);
  dutyD = constrain(dutyD, 0, maxDuty);
  ledcWrite(PwmPinMD, dutyD);

  Serial.println("Kp: "+String(Kp));
  Serial.println("multiP: "+String(multiP));
  Serial.println("Ki: "+String(Ki));
  Serial.println("multiI: "+String(multiI));
  Serial.println("Kd: "+String(Kd));
  Serial.println("multiD: "+String(multiD));
  Serial.println("vel: "+String(vel));
}

/* PID com anti-windup (integração condicional):
   - calcula candidato para Iterm e verifica se com ele as rodas saturariam;
   - aceita a integração apenas se wE/wD ficarem dentro dos limites [wMin, wMax].
*/
void pid() {
  // leitura de posição já definida fora (posicao)
  erro = ref - (int)posicao;
  Pterm = erro;
  Dterm = erro - erroAnt;

  // calcula os termos P e D imediatamente (baseados no erro atual)
  valorP = (Kp * powf(10.0f, (float)multiP)) * (float)Pterm;
  valorD = (Kd * powf(10.0f, (float)multiD)) * (float)Dterm;

  // CANDIDATO a Iterm (não o aplicamos ainda)
  long ItermCandidate = Iterm + erro;
  float valorI_candidate = (Ki * powf(10.0f, (float)multiI)) * (float)ItermCandidate;

  // calcula candidate PID output e verifica saturação nas rodas
  float valorPID_candidate = valorP + valorI_candidate + valorD;
  float w_candidate = valorPID_candidate;
  float wE_candidate = (vel - w_candidate * compEixo / 2.0f) / rodaR;
  float wD_candidate = (vel + w_candidate * compEixo / 2.0f) / rodaR;

  bool withinLeftLimits  = (wE_candidate >= wMinE) && (wE_candidate <= wMaxE);
  bool withinRightLimits = (wD_candidate >= wMinD) && (wD_candidate <= wMaxD);

  if (withinLeftLimits && withinRightLimits) {
    // aceitaremos o acumulador: atualiza Iterm e usa a saída candidata
    Iterm = ItermCandidate;
    valorI = valorI_candidate;
    valorPID = valorPID_candidate;
  } else {
    // anti-windup acionado: NÃO atualizar Iterm (mantemos o Iterm anterior)
    // recalcula valorI e valorPID usando o Iterm atual (não o candidato)
    valorI = (Ki * powf(10.0f, (float)multiI)) * (float)Iterm;
    valorPID = valorP + valorI + valorD;
    // opcional: poderia também limitar valorPID com uma regra de saturação se desejar
  }

  // atualiza erro anterior e define saída
  erroAnt = erro;
  w = valorPID;

  // Cinemática diferencial -> velocidades angulares das rodas (rad/s)
  wE = (vel - w * compEixo / 2.0f) / rodaR;
  wD = (vel + w * compEixo / 2.0f) / rodaR;

  motor(wE, wD);
}

void setup() {
  Serial.begin(115200);
  //------Configuração dos motores--------//
  pinMode(PwmPinME, OUTPUT);
  pinMode(PwmPinMD, OUTPUT);
  ledcAttach(PwmPinME, freq, res);
  ledcAttach(PwmPinMD, freq, res);
  analogWrite(PwmPinME, 0);
  analogWrite(PwmPinMD, 0);

  //------Configuração dos sensores-------//
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){4,16,17,5,18,19,21,22,32,33,25,26,27,14,12,13}, SensorCount);
  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
  }

  //--- Inicializa Wi-Fi em modo AP ---//
  WiFi.mode(WIFI_AP);
  bool ap_ok = WiFi.softAP(ap_ssid, ap_password);
  if (ap_ok) {
    IPAddress ip = WiFi.softAPIP();
    Serial.print("AP iniciado. IP: ");
    Serial.println(ip);
  } else {
    Serial.println("Falha ao iniciar AP");
  }

  // Configura rotas HTTP
  server.on("/", handleRoot);
  server.on("/update", handleUpdate);
  server.begin();
  Serial.println("Servidor Web iniciado");
}

void loop() {
  // Leitura da posição via QTR
  posicao = qtr.readLineWhite(sensorValues);
  pid();
  // Trata requisições HTTP
  server.handleClient();
}
