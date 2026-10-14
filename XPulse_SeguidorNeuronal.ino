#include <xmotionV3.h>
#include <avr/pgmspace.h>          // PROGMEM: pesos en flash, no en RAM

#define SpeedInit 140
#define SpeedPlus 125
#define Sens A1
#define s0 A2
#define s1 A4
#define s2 A5
#define s3 4
#define LPWM 10
#define LDIR 12
#define RPWM 11
#define RDIR 13
#define TMP A3
#define MicroStart A0
#define MuxTime 300

constexpr uint8_t NI = 15, NH = 7;  // constexpr: se resuelve al compilar, 0 bytes de RAM
const float W1[NH][NI] PROGMEM = {{1.246,-0.975411,0.19982,0.044461,-1.23748,-0.693337,0.409821,-1.30218,0.539552,-0.196714,-0.976619,0.77091,-1.22877,0.556926,0.329063},
{1.2228,-1.01224,0.721865,0.0221956,0.599058,-0.0645755,1.36511,-0.899823,-1.57766,-0.0492625,0.241608,-1.34174,-0.337781,0.404367,-0.160598},
{0.993554,-1.00201,0.435552,0.986471,0.855312,0.0684637,-0.810725,-0.861071,0.321784,0.929071,-0.0791656,1.26642,0.452548,1.35001,-0.129055},
{-0.894802,-0.379412,-1.8352,0.715548,-0.296278,-0.168389,0.65908,0.663694,-0.462137,-0.507089,-0.995051,-0.507261,0.335519,-1.10968,0.605622},
{0.243754,-1.07078,-1.13646,1.28596,-0.95922,0.965608,-0.626232,0.789463,1.06809,0.35342,-0.450734,0.743398,0.220643,0.630207,0.698238},
{-1.65023,0.22073,-0.743251,-0.333536,-1.12914,-0.240881,0.521135,-0.136889,-1.49022,0.143484,0.324277,-0.874447,0.783557,-0.811217,-0.517657},
{-0.235743,0.0280021,0.217062,-0.302885,-0.963497,1.52792,0.618674,0.875805,-0.126769,-1.47542,-1.04373,-0.74769,0.387572,-1.20605,0.511002}};

const float b1[NH]     PROGMEM = {-1.29003,-1.36908,-3.04598,2.63162,-0.785569,1.93541,-0.219499};
const float W2[2][NH]  PROGMEM = {{-0.0345728,0.0898968,-0.336839,0.340628,-0.0403801,-0.460376,0.275475},{0.407899,-0.356077,0.0697228,0.301524,0.215274,0.214105,0.0788622}};
const float b2[2]      PROGMEM = {0.653095,0.0821906};

int TrimSpeed, ExtraSpeed;
bool FloorColor = 0;
float yp[2];                        // última salida de la red (también sirve si se pierde la línea)

void setup() {
  pinMode(MicroStart, INPUT_PULLUP);
  pinMode(Sens, INPUT);
  pinMode(s0, OUTPUT); pinMode(s1, OUTPUT); pinMode(s2, OUTPUT); pinMode(s3, OUTPUT);
  pinMode(LDIR, OUTPUT); pinMode(LPWM, OUTPUT); pinMode(RDIR, OUTPUT); pinMode(RPWM, OUTPUT);
  TrimSpeed  = SpeedInit + analogRead(TMP) / 5;
  ExtraSpeed = min(TrimSpeed + SpeedPlus, 255);
}

// Cada pin del mux recibe un bit del número de sensor (i = 0b s3 s2 s1 s0)
void SensCall(uint8_t i) {
  digitalWrite(s0, bitRead(i, 0)); digitalWrite(s1, bitRead(i, 1));
  digitalWrite(s2, bitRead(i, 2)); digitalWrite(s3, bitRead(i, 3));
  delayMicroseconds(MuxTime);
}

void loop() {
  while (!digitalRead(MicroStart)) MOVE(0, 0, 1);

  // Color de piso: sensores extremos 0 y 14 iguales -> ese es el color de fondo
  SensCall(0);  bool ll = digitalRead(Sens);
  SensCall(14); bool rr = digitalRead(Sens);
  if (ll == rr) FloorColor = ll;

  // Lectura de los 16 sensores; XOR con el color de piso => 1 siempre significa "línea"
  uint8_t x[16], n = 0;
  for (uint8_t i = 0; i < 16; i++) {
    SensCall(i);
    x[i] = digitalRead(Sens) ^ FloorColor;
    if (i < NI) n += x[i];           // n = sensores de línea activos (excluye el trasero)
  }

  // Red neuronal (solo si hay línea; si no, se conserva yp = última orden válida)
  if (n) {
    float h[NH];
    for (uint8_t i = 0; i < NH; i++) {
      float s = pgm_read_float(&b1[i]);                       // pgm_read_float: leer de flash
      for (uint8_t j = 0; j < NI; j++) if (x[j]) s += pgm_read_float(&W1[i][j]);  // x es 0/1: solo suma pesos
      h[i] = tanh(s);
    }
    for (uint8_t k = 0; k < 2; k++) {
      float s = pgm_read_float(&b2[k]);
      for (uint8_t i = 0; i < NH; i++) s += pgm_read_float(&W2[k][i]) * h[i];
      yp[k] = s;
    }
  }

  // Velocidad: 120 si se perdió la línea, ExtraSpeed en recta (centro + trasero), si no TrimSpeed
  int vel = !n ? 120 : (x[7] && x[15]) ? ExtraSpeed : TrimSpeed;
  MOVE(yp[0] * vel, yp[1] * vel, 1);
}

void MOVE(float L, float R, int t) {
  digitalWrite(LDIR, L > 0); analogWrite(LPWM, min(abs(L), 255));   // limita a 255 aquí
  digitalWrite(RDIR, R > 0); analogWrite(RPWM, min(abs(R), 255));
  delay(t);
}