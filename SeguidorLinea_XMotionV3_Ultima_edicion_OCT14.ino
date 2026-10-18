/*
  SEGUIDOR DE LINEA XMotion V3 - Arduino Leonardo (ATmega32U4)
  -----------------------------------------------------------
  - Giros de 90° controlados (Pivotante asimétrico + Freno de inercia).
  - Arranque limpio: Avanza recto al inicio hasta encontrar línea.
  - Sensor 15: Central (Error = 0).
  - Umbrales FSM: Sensores <= 3 (Derecha) y 10 a 14 (Izquierda).
  - Monitor Serie: 115200 baudios.
*/

#include <Arduino.h>

// ---------------- Limites y velocidades ----------------
const int PID_MIN_SPEED = 20;   // Velocidad minima en curva PID
const int LimitMax = 255;       //velocidad maxima

#define SpeedInit 135   //velocidad inicial
#define SpeedPlus 35    //cuando va en recta se aumenta este pwm

// ---------------- Multiplexor XLine ----------------
#define Sens A1
#define s0 A2
#define s1 A4
#define s2 A5
#define s3 4

// ---------------- Motores XMotion V3 ----------------
#define LPWM 10   //pin para controlar velocidad de motor izquierdo
#define LDIR 12   //controla la direccion delmotor izquierdo
#define RPWM 11   //pin para controlar velocidad de motor derecho
#define RDIR 13   //controla la direccion delmotor derecho

// ---------------- Trimpot, boton, DIP y LEDs ----------------
#define TMP A3            //potenciometro para aumentar velocidad
#define MicroStart A0     // sensor infrarojo
#define DS1 5             //dip switch 
#define DS2 6             //debug switch
#define DS3 7             //debug switch
#define LLED 8            //led izquierdo
#define RLED 9            //led derecho

// ---------------- Ajustes PID ----------------
float Kp =0.35f;
float Kd = 0.001f;
float dErrorFilt = 0.01f;
//investigar

const uint16_t MuxTime = 25;               // 25 Microsegundos para estabilizar la lectura del multiplexor
const uint32_t SAMPLE_PERIOD_US = 5000UL;  // para calculos PID cada 5 ms (200 Hz)
const uint16_t DEBUG_PERIOD_MS = 80;       //tiempode muestreo del puerto serial 8 milisegundos 12 veces por segundos se actualiza el puerto serial

// ---------------- Ajustes de Giro 90° (ANTI-TROMPO) ----------------
const uint8_t TURN_SPEED_FWD = 140;        // Rueda externa empuja con fuerza moderada
const int16_t TURN_SPEED_REV = -45;        // Contrafase suave (evita dar la vuelta de 180°) -80 griro brusco -20 vuelta muysuave
const uint8_t SEARCH_SPEED = 110;           //velocidad media para encontrar la pista si sesale y pierdelapista
const uint16_t SPIN_MIN_MS = 12;           // Reducido a 18ms para no sobre-girar
const uint16_t SPIN_TIMEOUT_MS = 600;       //tiempolimite para el escape de giro

const uint8_t TURN_CONFIRM_SAMPLES = 2;    // 10 ms para validar 90°
const uint8_t LOST_CONFIRM_SAMPLES = 4;    //sammples anates de declararseperdido
const uint8_t CENTER_EXIT_SAMPLES = 1;     // Salida rapida en cuanto toca el centro
const uint8_t TURN_MAX_ACTIVE_SENSORS = 6; 
const int16_t TURN_REARM_ERROR = 60;
const uint8_t TURN_REARM_SAMPLES = 4;

// ---------------- Variables globales ----------------
int TrimSpeed = SpeedInit;                  //velocidad inicial
int ExtraSpeed = SpeedInit + SpeedPlus;     //velocidad extra para linea recta
int BaseSpeed = SpeedInit;                  //velocidad base = a la velocidad inicial
int Error = 0;                              
int LastError = 0;
int Sensed = 0;
float Correction = 0.0f;
float LSpeed = 0.0f;
float RSpeed = 0.0f;
float Old_LSpeed = 9999.0f;
float Old_RSpeed = 9999.0f;
int ll = 0;
int rr = 0;
bool FloorColor = 0;
bool HasFoundLineOnce = false; // Evita que gire en circulos al arrancar

// Variables de sensores
uint16_t SensorMask = 0;
uint8_t ActiveCount = 0;
uint8_t CenterCount = 0;
bool ExtremeRightActive = false; // 0, 1, 2, 3
bool ExtremeLeftActive = false;  // 10, 11, 12, 13, 14

// ---------------- Maquina de estados ----------------
enum RobotState : uint8_t {
  STATE_FOLLOW,
  STATE_SPIN_LEFT,
  STATE_SPIN_RIGHT,
  STATE_SEARCH,
  STATE_STOP
};

RobotState CurrentState = STATE_FOLLOW;
uint32_t StateStartMs = 0;
uint32_t NextSampleUs = 0;
uint32_t LastDebugMs = 0;
uint8_t CenterStableCount = 0;
uint8_t TurnRightCount = 0;
uint8_t TurnLeftCount = 0;
uint8_t LostLineCount = 0;
uint8_t TurnRearmCount = 0;
bool TurnRearmRequired = false;
bool MicroStartWasLow = false;

// ---------------- Prototipos ----------------
void SW(bool SW0, bool SW1, bool SW2, bool SW3);
void SensCall(int i);
void ReadSensor();
void MOVE(float LeftSpeed, float RightSpeed);
void DebugRealtime(uint32_t nowMs);
void CalibrateFloorColor();
void RunFSM(uint32_t nowMs);
void ChangeState(RobotState newState, uint32_t nowMs);
void SetStateMotors();
void UpdateLEDs(uint32_t nowMs);
const __FlashStringHelper* StateName(RobotState state);

// ===========================================================
// SETUP
// ===========================================================
void setup() {
  pinMode(MicroStart, INPUT_PULLUP);
  pinMode(DS1, INPUT_PULLUP);
  pinMode(DS2, INPUT_PULLUP);
  pinMode(DS3, INPUT_PULLUP);

  pinMode(Sens, INPUT);
  pinMode(s0, OUTPUT);
  pinMode(s1, OUTPUT);
  pinMode(s2, OUTPUT);
  pinMode(s3, OUTPUT);

  pinMode(RDIR, OUTPUT);
  pinMode(RPWM, OUTPUT);
  pinMode(LDIR, OUTPUT);
  pinMode(LPWM, OUTPUT);

  pinMode(LLED, OUTPUT);
  pinMode(RLED, OUTPUT);

  digitalWrite(s0, LOW);
  digitalWrite(s1, LOW);
  digitalWrite(s2, LOW);
  digitalWrite(s3, LOW);

  MOVE(0, 0);

  // Descartar primera lectura analogica inestable
  analogRead(TMP);
  delay(10);
  TrimSpeed = map(analogRead(TMP), 0, 1023, 115, 200);  //mapeo d velocidad de los valores 0-1023 transpolado a 115-200 pwm's
  TrimSpeed = constrain(TrimSpeed, 115, 200); //limites de veocidad en el trimSpeed
  ExtraSpeed = TrimSpeed + SpeedPlus;  // Suma para velocidad en linea REcta
  if (ExtraSpeed > 255) ExtraSpeed = 255; // limite condicional de la velocdiad de ExtraSpeed
  BaseSpeed = TrimSpeed; //Defines la velocidad base como la que leiste en el potenciometro

  // Calibracion con retardo termico/electrico
  CalibrateFloorColor();

  Serial.begin(115200);

  CurrentState = STATE_FOLLOW;
  StateStartMs = millis(); // y esto
  NextSampleUs = micros() + SAMPLE_PERIOD_US; //TODO: saber pa que sirve esto
}

// ===========================================================
// LOOP PRINCIPAL
// ===========================================================
void loop() {
  const uint32_t nowMs = millis();
  const uint32_t nowUs = micros();

  // 1. Muestreo constante a 200 Hz
  if ((int32_t)(nowUs - NextSampleUs) >= 0) { // Condicional para saber si es el primer periodo de muestreo
    NextSampleUs = nowUs + SAMPLE_PERIOD_US;  //Siguiente periodo de muestro de PID
    ReadSensor();
  }

  // 2. Gestion de MicroStart y Arranque Limpio
  if (digitalRead(MicroStart) == LOW) {
    MOVE(0, 0);
    Old_LSpeed = 9999.0f;
    Old_RSpeed = 9999.0f;
    MicroStartWasLow = true;
    digitalWrite(LLED, LOW);
    digitalWrite(RLED, LOW);
    if (digitalRead(DS2) == LOW) DebugRealtime(nowMs);
    return;
  }

  // AL MOMENTO DE SOLTAR EL MICROSTART: RESET TOTAL DE VARIABLES
  if (MicroStartWasLow) {
    MicroStartWasLow = false;
    Error = 0;
    LastError = 0;
    dErrorFilt = 0.0f;
    TurnRightCount = 0;
    TurnLeftCount = 0;
    LostLineCount = 0;
    HasFoundLineOnce = false; // Requiere confirmar linea antes de permitir busquedas
    TurnRearmRequired = true; // No permitir giros de 90° en los primeros instantes
    TurnRearmCount = 0;
    ChangeState(STATE_FOLLOW, nowMs);   //Inicializa el estado de seguimiento normal con PID
  }

  // 3. Modo Diagnostico (DS2 en LOW)
  if (digitalRead(DS2) == LOW) {
    MOVE(0, 0);
    Old_LSpeed = 9999.0f;
    Old_RSpeed = 9999.0f;
    DebugRealtime(nowMs);
    UpdateLEDs(nowMs);
    return;
  }

  // 4. Control normal
  RunFSM(nowMs);
  UpdateLEDs(nowMs);
}

// ---------------- Calibracion con filtro de arranque ----------------
void CalibrateFloorColor() {
  delay(200); // Espera critica: permite que los fototransistores se energicen

  // Si DS1 esta en LOW, fuerza modo estandar (Fondo blanco, linea negra)
  if (digitalRead(DS1) == LOW) {
    FloorColor = 0;
    return;
  }

  int blackCount = 0;
  for (int k = 0; k < 5; k++) {
    SensCall(0);
    ll = digitalRead(Sens);
    SensCall(14);
    rr = digitalRead(Sens);
    if (ll == 1 && rr == 1) blackCount++;
    delay(5);
  }

  // Solo si consistentemente lee HIGH en ambos extremos asume fondo negro
  FloorColor = (blackCount >= 4) ? 1 : 0;
}

// ===========================================================
// FSM: GIROS 90° CON CONTROL DE INERCIA
// ===========================================================
void ChangeState(RobotState newState, uint32_t nowMs) {
  CurrentState = newState;
  StateStartMs = nowMs;
  CenterStableCount = 0;
  TurnRightCount = 0;
  TurnLeftCount = 0;
  LostLineCount = 0;

  if (newState == STATE_SPIN_LEFT || newState == STATE_SPIN_RIGHT) {
    TurnRearmRequired = true;
    TurnRearmCount = 0;
  }
}

void RunFSM(uint32_t nowMs) {
  switch (CurrentState) {
    case STATE_FOLLOW: {
      if (Sensed != 0) {
        HasFoundLineOnce = true;
      }

      // Rearme: Debe ver el centro antes de admitir otro giro
      if (TurnRearmRequired) {
        if (CenterCount >= 1 && abs(Error) <= TURN_REARM_ERROR) { // TURN_REARM_ERROR es un filro ( siempre vale 60) que aseguraque esta leyendo el centro
          if (TurnRearmCount < 255) ++TurnRearmCount;
          if (TurnRearmCount >= TURN_REARM_SAMPLES) {
            TurnRearmRequired = false;
            TurnRearmCount = 0;
          }
        } else {
          TurnRearmCount = 0;
        }
      }

      // Perdida de linea: Solo busca si ya habia agarrado linea antes
      if (Sensed == 0) {
        if (!HasFoundLineOnce) {
          // Si acaba de arrancar y no la ve, avanza recto suave; no gira a un lado
          LSpeed = 90;
          RSpeed = 90;
          MOVE(LSpeed, RSpeed);
          return;
        }

        if (LostLineCount < 255) ++LostLineCount;
        if (LostLineCount >= LOST_CONFIRM_SAMPLES) {
          ChangeState(STATE_SEARCH, nowMs);
        }
        break;
      }
      LostLineCount = 0;

      // Disparo de 90° por extremos
      if (!TurnRearmRequired && ActiveCount <= TURN_MAX_ACTIVE_SENSORS) {
        if (ExtremeRightActive && !ExtremeLeftActive) {
          if (TurnRightCount < 255) ++TurnRightCount;
          TurnLeftCount = 0;
          if (TurnRightCount >= TURN_CONFIRM_SAMPLES) {
            ChangeState(STATE_SPIN_RIGHT, nowMs);
          }
        } else if (ExtremeLeftActive && !ExtremeRightActive) {
          if (TurnLeftCount < 255) ++TurnLeftCount;
          TurnRightCount = 0;
          if (TurnLeftCount >= TURN_CONFIRM_SAMPLES) {
            ChangeState(STATE_SPIN_LEFT, nowMs);
          }
        } else {
          TurnRightCount = 0;
          TurnLeftCount = 0;
        }
      } else {
        TurnRightCount = 0;
        TurnLeftCount = 0;
      }
      break;
    }

    case STATE_SPIN_LEFT:
    case STATE_SPIN_RIGHT: {
      // FRENO DE INERCIA INMEDIATO: Si el sensor central 7 o 15 tocan la linea
      bool centerHit = (SensorMask & ((uint16_t)1U << 7)) || (SensorMask & ((uint16_t)1U << 15));

      if ((uint32_t)(nowMs - StateStartMs) >= SPIN_MIN_MS && (centerHit || CenterCount >= 1)) {
        // Pulso de contra-freno activo para clavar la rotacion
        MOVE(0, 0);
        delayMicroseconds(400);

        LastError = 0;
        dErrorFilt = 0.0f;
        ChangeState(STATE_FOLLOW, nowMs);
      } else if ((uint32_t)(nowMs - StateStartMs) >= SPIN_TIMEOUT_MS) {
        ChangeState(STATE_SEARCH, nowMs);
      }
      break;
    }

    case STATE_SEARCH:
      if (CenterCount >= 1) {
        LastError = 0;
        dErrorFilt = 0.0f;
        ChangeState(STATE_FOLLOW, nowMs);
      }
      break;

    case STATE_STOP:
    default:
      break;
  }

  SetStateMotors();
}

// ===========================================================
// MOTORES Y VELOCIDADES ASIMETRICAS
// ===========================================================
void SetStateMotors() {
  switch (CurrentState) {
    case STATE_FOLLOW: {
      float rawDelta = (float)(Error - LastError);
      dErrorFilt = (0.70f * dErrorFilt) + (0.30f * rawDelta);
      LastError = Error;

      Correction = (Kp * Error) + (Kd * dErrorFilt);

      LSpeed = BaseSpeed + Correction;
      RSpeed = BaseSpeed - Correction;

      if (LSpeed > LimitMax) {
        RSpeed -= (LSpeed - LimitMax);
        LSpeed = LimitMax;
      }
      if (RSpeed > LimitMax) {
        LSpeed -= (RSpeed - LimitMax);
        RSpeed = LimitMax;
      }

      LSpeed = constrain(LSpeed, PID_MIN_SPEED, LimitMax);
      RSpeed = constrain(RSpeed, PID_MIN_SPEED, LimitMax);
      break;
    }

    // GIRO DERECHA: Izquierda empuja hacia adelante, Derecha retiene suavemente
    case STATE_SPIN_RIGHT:
      LSpeed = TURN_SPEED_FWD;
      RSpeed = TURN_SPEED_REV; // -45 PWM en lugar de -175 (No hace trompo)
      break;

    // GIRO IZQUIERDA: Derecha empuja hacia adelante, Izquierda retiene suavemente
    case STATE_SPIN_LEFT:
      LSpeed = TURN_SPEED_REV; // -45 PWM
      RSpeed = TURN_SPEED_FWD;
      break;

    case STATE_SEARCH:
      if (LastError >= 0) {
        LSpeed = SEARCH_SPEED;
        RSpeed = -((int16_t)SEARCH_SPEED);
      } else {
        LSpeed = -((int16_t)SEARCH_SPEED);
        RSpeed = SEARCH_SPEED;
      }
      break;

    case STATE_STOP:
    default:
      LSpeed = 0;
      RSpeed = 0;
      break;
  }

  if (LSpeed != Old_LSpeed || RSpeed != Old_RSpeed) {
    MOVE(LSpeed, RSpeed);
    Old_LSpeed = LSpeed;
    Old_RSpeed = RSpeed;
  }
}

// ===========================================================
// LECTURA DE SENSORES
// ===========================================================
int16_t SensorErrorForIndex(uint8_t i) {
  switch (i) {
    case 0:  return 220;
    case 1:  return 180;
    case 2:  return 140;
    case 3:  return 100;
    case 4:  return 70;
    case 5:  return 45;
    case 6:  return 20;
    case 7:  return 0;   // Centro Frontal
    case 8:  return -20;
    case 9:  return -45;
    case 10: return -70;
    case 11: return -100;
    case 12: return -140;
    case 13: return -180;
    case 14: return -220;
    case 15: return 0;   // Sensor 15 central
    default: return 0;
  }
}

void ReadSensor() {
  Sensed = 0;
  ActiveCount = 0;
  CenterCount = 0;
  SensorMask = 0;
  x=0;
  y=0;
  ExtremeRightActive = false;
  ExtremeLeftActive = false;
  int32_t errorSum = 0;

  for (uint8_t i = 0; i <= 15; i++) {
    SensCall(i);
    const int raw = digitalRead(Sens);
    const bool lineActive = (FloorColor == 0) ? (raw == HIGH) : (raw == LOW);

    if (lineActive) {
      Sensed = 1;
      ++ActiveCount;
      SensorMask |= ((uint16_t)1U << i);
      errorSum += SensorErrorForIndex(i);

      if ((i >= 6 && i <= 8) || i == 15) {
        ++CenterCount;
      }

      //mejor ar agregando un count para que si suma dos el contador si aplique el giro

      // Sensores 0 a 3: Extremo Derecho
      if (i <= 2) {
        x++;
        if (x >= 2) ExtremeRightActive = true;
      }
      // Sensores 10 a 14: Extremo Izquierdo (15 excluido)
      if (i >= 12 && i <= 14) 
      {
        y++;
        if (y >= 2) ExtremeLeftActive = true;
      }
    }
  }

  if (Sensed != 0 && ActiveCount != 0) {
    Error = (int)(errorSum / (int32_t)ActiveCount);   //La suma de los sensores activos  entre la cantidad de sensores activos 
  }

  if (Sensed != 0) {
    BaseSpeed = TrimSpeed;
    const bool center7Active = (SensorMask & ((uint16_t)1U << 7)) != 0;
    const bool center15Active = (SensorMask & ((uint16_t)1U << 15)) != 0;
    if (center7Active && center15Active && ActiveCount <= 4) {
      BaseSpeed = ExtraSpeed;
    }
  }
}

// ===========================================================
// Multiplexor y motores
// ===========================================================
void SW(bool SW0, bool SW1, bool SW2, bool SW3) {
  digitalWrite(s0, SW0 ? HIGH : LOW);
  digitalWrite(s1, SW1 ? HIGH : LOW);
  digitalWrite(s2, SW2 ? HIGH : LOW);
  digitalWrite(s3, SW3 ? HIGH : LOW);
}

void SensCall(int i) {
  if (i == 0) SW(0, 0, 0, 0);
  else if (i == 1) SW(1, 0, 0, 0);
  else if (i == 2) SW(0, 1, 0, 0);
  else if (i == 3) SW(1, 1, 0, 0);
  else if (i == 4) SW(0, 0, 1, 0);
  else if (i == 5) SW(1, 0, 1, 0);
  else if (i == 6) SW(0, 1, 1, 0);
  else if (i == 7) SW(1, 1, 1, 0);
  else if (i == 8) SW(0, 0, 0, 1);
  else if (i == 9) SW(1, 0, 0, 1);
  else if (i == 10) SW(0, 1, 0, 1);
  else if (i == 11) SW(1, 1, 0, 1);
  else if (i == 12) SW(0, 0, 1, 1);
  else if (i == 13) SW(1, 0, 1, 1);
  else if (i == 14) SW(0, 1, 1, 1);
  else if (i == 15) SW(1, 1, 1, 1);
  else return;

  delayMicroseconds(MuxTime);
}

void MOVE(float LeftSpeed, float RightSpeed) {
  LeftSpeed = constrain(LeftSpeed, -255, 255);
  RightSpeed = constrain(RightSpeed, -255, 255);

  if (LeftSpeed < 0) {
    digitalWrite(LDIR, LOW);
    analogWrite(LPWM, (int)abs(LeftSpeed));
  } else {
    digitalWrite(LDIR, HIGH);
    analogWrite(LPWM, (int)LeftSpeed);
  }

  if (RightSpeed < 0) {
    digitalWrite(RDIR, LOW);
    analogWrite(RPWM, (int)abs(RightSpeed));
  } else {
    digitalWrite(RDIR, HIGH);
    analogWrite(RPWM, (int)RightSpeed);
  }
}

// ===========================================================
// DIAGNOSTICO EN TIEMPO REAL
// ===========================================================
void DebugRealtime(uint32_t nowMs) {
  if ((uint32_t)(nowMs - LastDebugMs) < DEBUG_PERIOD_MS) return;
  LastDebugMs = nowMs;

  Serial.print(F("["));
  for (int j = 0; j <= 14; j++) {
    bool on = (SensorMask & ((uint16_t)1U << j)) != 0;
    Serial.print(on ? F("#") : F("."));
    if (j == 3 || j == 9) Serial.print(F(" "));
  }
  Serial.print(F(" | Mid15:"));
  Serial.print((SensorMask & ((uint16_t)1U << 15)) ? F("#") : F("."));
  Serial.print(F("] "));

  Serial.print(F("Err:")); Serial.print(Error);
  Serial.print(F(" | PWM:[L=")); Serial.print((int)LSpeed);
  Serial.print(F(" R=")); Serial.print((int)RSpeed);
  Serial.print(F("]"));

  if (ExtremeRightActive) Serial.println(F(" -> [TRIG 90 DERECHA]"));
  else if (ExtremeLeftActive) Serial.println(F(" -> [TRIG 90 IZQUIERDA]"));
  else Serial.println();
}

const __FlashStringHelper* StateName(RobotState state) {
  switch (state) {
    case STATE_FOLLOW: return F("FOLLOW");
    case STATE_SPIN_LEFT: return F("SPIN_LEFT");
    case STATE_SPIN_RIGHT: return F("SPIN_RIGHT");
    case STATE_SEARCH: return F("SEARCH");
    case STATE_STOP: return F("STOP");
    default: return F("UNKNOWN");
  }
}

void UpdateLEDs(uint32_t nowMs) {
  bool leftOn = false;
  bool rightOn = false;

  switch (CurrentState) {
    case STATE_FOLLOW:
      rightOn = (Error > 25);
      leftOn = (Error < -25);
      break;
    case STATE_SPIN_LEFT:
      leftOn = true;
      break;
    case STATE_SPIN_RIGHT:
      rightOn = true;
      break;
    case STATE_SEARCH:
      if (LastError >= 0) rightOn = true;
      else leftOn = true;
      break;
    case STATE_STOP:
    default:
      leftOn = (((nowMs / 250UL) & 1U) != 0);
      rightOn = leftOn;
      break;
  }

  digitalWrite(LLED, leftOn ? HIGH : LOW);
  digitalWrite(RLED, rightOn ? HIGH : LOW);
}