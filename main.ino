/*
 * Web‑enabled PID controller for a line‑following robot.
 *
 * This sketch builds upon the original motor, sensor and PID code and
 * adds a very simple HTTP interface using the ESP32’s built‑in Wi‑Fi.
 * When the board powers up it creates its own access point.  You can
 * connect your phone or laptop to this AP and browse to the root
 * address (default 192.168.4.1) to view a form that exposes the
 * current tuning parameters (Kp/Ki/Kd) along with their multipliers
 * and the commanded forward velocity.  Submitting the form will update
 * the corresponding variables at run‑time.
 *
 * This pattern follows a traditional embedded systems workflow: use
 * the core control loop you’ve already validated and bolt on a
 * management plane using a widely adopted protocol (HTTP) rather than
 * inventing your own transport.  In the world of industrial control
 * that sort of architectural layering reduces risk and maximises reuse.
 */

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

// Configuração de Wi‑Fi – altere o SSID/senha conforme necessário.
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

/*
 * Handler da página principal.  Constrói uma página HTML simples
 * contendo um formulário para actualizar parâmetros.  Para manter
 * clareza e evitar código dinâmico dentro de strings longas, as
 * substituições de marcador são feitas com chamadas replace().
 */
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
  // Campo Kp
  page += "<label for=\"kp\">Kp:</label>";
  page += "<input type=\"text\" id=\"kp\" name=\"kp\" value=\"" + String(Kp, 4) + "\">";
  // Campo multiP
  page += "<label for=\"multiP\">Multiplicador de P:</label>";
  page += "<input type=\"text\" id=\"multiP\" name=\"multiP\" value=\"" + String(multiP) + "\">";
  // Campo Ki
  page += "<label for=\"ki\">Ki:</label>";
  page += "<input type=\"text\" id=\"ki\" name=\"ki\" value=\"" + String(Ki, 4) + "\">";
  // Campo multiI
  page += "<label for=\"multiI\">Multiplicador de I:</label>";
  page += "<input type=\"text\" id=\"multiI\" name=\"multiI\" value=\"" + String(multiI) + "\">";
  // Campo Kd
  page += "<label for=\"kd\">Kd:</label>";
  page += "<input type=\"text\" id=\"kd\" name=\"kd\" value=\"" + String(Kd, 4) + "\">";
  // Campo multiD
  page += "<label for=\"multiD\">Multiplicador de D:</label>";
  page += "<input type=\"text\" id=\"multiD\" name=\"multiD\" value=\"" + String(multiD) + "\">";
  // Campo vel
  page += "<label for=\"vel\">Velocidade (m/s):</label>";
  page += "<input type=\"text\" id=\"vel\" name=\"vel\" value=\"" + String(vel, 3) + "\">";
  page += "<input type=\"submit\" value=\"Atualizar\"></form>";
  page += "</body></html>";
  server.send(200, "text/html", page);
}

/*
 * Handler para a rota /update.  Lê parâmetros transmitidos via query
 * string e actualiza as variáveis globais.  Após processar, redireciona
 * de volta à página principal.
 */
void handleUpdate() {
  // Apenas actualiza se o argumento estiver presente; evita
  // corrupções inesperadas se o campo estiver vazio.
  if (server.hasArg("kp")) {
    float tmp = server.arg("kp").toFloat();
    if (!isnan(tmp)) {
      Kp = tmp;
    }
  }
  if (server.hasArg("multiP")) {
    int tmp = server.arg("multiP").toInt();
    multiP = tmp;
  }
  if (server.hasArg("ki")) {
    float tmp = server.arg("ki").toFloat();
    if (!isnan(tmp)) {
      Ki = tmp;
    }
  }
  if (server.hasArg("multiI")) {
    int tmp = server.arg("multiI").toInt();
    multiI = tmp;
  }
  if (server.hasArg("kd")) {
    float tmp = server.arg("kd").toFloat();
    if (!isnan(tmp)) {
      Kd = tmp;
    }
  }
  if (server.hasArg("multiD")) {
    int tmp = server.arg("multiD").toInt();
    multiD = tmp;
  }
  if (server.hasArg("vel")) {
    float tmp = server.arg("vel").toFloat();
    if (!isnan(tmp)) {
      vel = tmp;
    }
  }
  // Após actualizar, redirecciona de volta ao root para ver os novos valores
  server.sendHeader("Location", "/", true);
  server.send(302, "text/plain", "");
}

void motor(float we, float wd) {
  // mapear we/wd (rad/s) para porcentagem e depois para duty PWM
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

void pid() {
  // leitura de posição já definida fora (posicao)
  erro = ref - (int)posicao;
  Pterm = erro;
  Iterm = Iterm + erro;
  Dterm = erro - erroAnt;

  // aplicar multiplicadores de ordem de grandeza
  valorP = (Kp * powf(10.0f, (float)multiP)) * (float)Pterm;
  valorI = (Ki * powf(10.0f, (float)multiI)) * (float)Iterm;
  valorD = (Kd * powf(10.0f, (float)multiD)) * (float)Dterm;
  valorPID = valorP + valorI + valorD;
  erroAnt = erro;

  // Cálculo atuador: w é velocidade angular do corpo (rad/s)
  w = valorPID;

  // Cinemática diferencial -> velocidades angulares das rodas (rad/s)
  wE = (vel - w * compEixo / 2.0f) / rodaR;
  wD = (vel + w * compEixo / 2.0f) / rodaR;
  motor(wE, wD);
}

void setup() {
  // Inicia o Serial para debug
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
  // Calibra os sensores – o laço for repete as leituras para gerar offsets
  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
  }

  //--- Inicializa Wi‑Fi em modo AP ---//
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