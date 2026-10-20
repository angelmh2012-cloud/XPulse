#include <xmotionV3.h>
#include <Fuzzy.h>

// ==========================================
// Variables de velocidad
// ==========================================
#define SpeedInit 100   // Velocidad base
#define SpeedPlus 40    // Extra de velocidad en recta (sensor 15)
#define UMBRAL_LINEA 500 // Umbral analógico para detectar la pista

// ==========================================
// Pines Xline & Multiplexor
// ==========================================
#define Sens A1
#define s0 A2
#define s1 A4
#define s2 A5
#define s3 4

// ==========================================
// Pines Motor Driver (XMotion V3)
// ==========================================
#define LPWM 10 
#define LDIR 12
#define RPWM 11
#define RDIR 13

// ==========================================
// Trimpot, Botón, Dipswitch & LEDs
// ==========================================
#define TMP A3
#define MicroStart A0   // Pin del módulo MicroStart
#define DS1 5
#define DS2 6           // DipSwitch 2: Activa/Desactiva Debug Serial
#define DS3 7
#define LLED 8
#define RLED 9

// Instancia de Lógica Difusa
Fuzzy *fuzzy = new Fuzzy();

// Lecturas de los 16 canales
int sensorValues[16];

// Pesos espaciales (Sensor 15 en el centro = peso 0)
const int pesos[16] = {
  -14, -12, -10, -8, -6, -4, -2,  // 0 al 6: Izquierda
    0,                            // 7: Centro de la barra
    2,   4,   6,  8, 10, 12, 14,  // 8 al 14: Derecha
    0                             // 15: Sensor central adelantado
};

// Temporizador para no saturar el puerto serial
unsigned long lastSerialTime = 0;

void setup() {
  // Comunicación serial rápida
  Serial.begin(115200);

  // Configuración de pines de entrada y control
  pinMode(s0, OUTPUT);
  pinMode(s1, OUTPUT);
  pinMode(s2, OUTPUT);
  pinMode(s3, OUTPUT);
  pinMode(Sens, INPUT);

  pinMode(DS1, INPUT_PULLUP);
  pinMode(DS2, INPUT_PULLUP); // Dipswitch: LOW cuando está en ON
  pinMode(DS3, INPUT_PULLUP);
  pinMode(LLED, OUTPUT);
  pinMode(RLED, OUTPUT);

  // El MicroStart debe configurarse como INPUT puro (sin pullup)
  pinMode(MicroStart, INPUT);

  // Asegurar motores detenidos desde el encendido
  xmotion.StopMotors(0);

  // ---------------------------------------------------
  // 1. CONFIGURACIÓN DE LÓGICA DIFUSA
  // ---------------------------------------------------
  FuzzyInput *errorPos = new FuzzyInput(1);
  FuzzySet *extremoIzq = new FuzzySet(-15, -15, -10, -5);
  FuzzySet *izquierda  = new FuzzySet(-8, -4, -4, 0);
  FuzzySet *centro     = new FuzzySet(-2, 0, 0, 2);
  FuzzySet *derecha    = new FuzzySet(0, 4, 4, 8);
  FuzzySet *extremoDer = new FuzzySet(5, 10, 15, 15);

  errorPos->addFuzzySet(extremoIzq);
  errorPos->addFuzzySet(izquierda);
  errorPos->addFuzzySet(centro);
  errorPos->addFuzzySet(derecha);
  errorPos->addFuzzySet(extremoDer);
  fuzzy->addFuzzyInput(errorPos);

  FuzzyOutput *correccionGiro = new FuzzyOutput(1);
  FuzzySet *giroFuerteIzq = new FuzzySet(-80, -80, -50, -20);
  FuzzySet *giroSuaveIzq  = new FuzzySet(-40, -20, -20, 0);
  FuzzySet *recto         = new FuzzySet(-10, 0, 0, 10);
  FuzzySet *giroSuaveDer  = new FuzzySet(0, 20, 20, 40);
  FuzzySet *giroFuerteDer = new FuzzySet(20, 50, 80, 80);

  correccionGiro->addFuzzySet(giroFuerteIzq);
  correccionGiro->addFuzzySet(giroSuaveIzq);
  correccionGiro->addFuzzySet(recto);
  correccionGiro->addFuzzySet(giroSuaveDer);
  correccionGiro->addFuzzySet(giroFuerteDer);
  fuzzy->addFuzzyOutput(correccionGiro);

  // Regla 1: Extremo Izquierda -> Giro Fuerte Izquierda
  FuzzyRuleAntecedent *ant1 = new FuzzyRuleAntecedent(); ant1->joinSingle(extremoIzq);
  FuzzyRuleConsequent *con1 = new FuzzyRuleConsequent(); con1->addOutput(giroFuerteIzq); // Corregido
  fuzzy->addFuzzyRule(new FuzzyRule(1, ant1, con1));

  // Regla 2: Izquierda -> Giro Suave Izquierda
  FuzzyRuleAntecedent *ant2 = new FuzzyRuleAntecedent(); ant2->joinSingle(izquierda);
  FuzzyRuleConsequent *con2 = new FuzzyRuleConsequent(); con2->addOutput(giroSuaveIzq);  // Corregido
  fuzzy->addFuzzyRule(new FuzzyRule(2, ant2, con2));

  // Regla 3: Centro -> Recto
  FuzzyRuleAntecedent *ant3 = new FuzzyRuleAntecedent(); ant3->joinSingle(centro);
  FuzzyRuleConsequent *con3 = new FuzzyRuleConsequent(); con3->addOutput(recto);
  fuzzy->addFuzzyRule(new FuzzyRule(3, ant3, con3));

  // Regla 4: Derecha -> Giro Suave Derecha
  FuzzyRuleAntecedent *ant4 = new FuzzyRuleAntecedent(); ant4->joinSingle(derecha);
  FuzzyRuleConsequent *con4 = new FuzzyRuleConsequent(); con4->addOutput(giroSuaveDer);  // Corregido
  fuzzy->addFuzzyRule(new FuzzyRule(4, ant4, con4));

  // Regla 5: Extremo Derecha -> Giro Fuerte Derecha
  FuzzyRuleAntecedent *ant5 = new FuzzyRuleAntecedent(); ant5->joinSingle(extremoDer);
  FuzzyRuleConsequent *con5 = new FuzzyRuleConsequent(); con5->addOutput(giroFuerteDer); // Corregido
  fuzzy->addFuzzyRule(new FuzzyRule(5, ant5, con5));
  // ---------------------------------------------------
  // 2. ESPERA EXCLUSIVA DEL MICROSTART
  // ---------------------------------------------------
  // Los leds parpadean indicando que el sistema está armado y esperando la señal
  while (digitalRead(MicroStart) == LOW) {
    xmotion.StopMotors(0);
    digitalWrite(LLED, HIGH);
    digitalWrite(RLED, LOW);
    delay(100);
    digitalWrite(LLED, LOW);
    digitalWrite(RLED, HIGH);
    delay(100);

    // Mensaje de espera si DS2 está activo
    if (digitalRead(DS2) == LOW) {
      Serial.println(F("[STANDBY] Esperando señal de inicio del MicroStart..."));
    }
  }

  // Al recibir el START (HIGH): Leds fijos encendidos
  digitalWrite(LLED, HIGH);
  digitalWrite(RLED, HIGH);
  if (digitalRead(DS2) == LOW) {
    Serial.println(F("[START] ¡Señal recibida! Arrancando robot..."));
  }
}

// Barrido de los 16 canales del multiplexor
void leerMultiplexor() {
  for (int i = 0; i < 16; i++) {
    digitalWrite(s0, bitRead(i, 0));
    digitalWrite(s1, bitRead(i, 1));
    digitalWrite(s2, bitRead(i, 2));
    digitalWrite(s3, bitRead(i, 3));
    delayMicroseconds(6);
    sensorValues[i] = analogRead(Sens);
  }
}

// Cálculo del error mediante centroide
// Cálculo del error con filtrado de fondo
float obtenerErrorPosicion() {
  long sumaPonderada = 0;
  long sumaTotal = 0;
  int sensoresActivos = 0;

  for (int i = 0; i < 16; i++) {
    // Si tu línea es BLANCA sobre fondo NEGRO:
    int valorLimpio = sensorValues[i] - 150; // Resta ruido base
    if (valorLimpio < 0) valorLimpio = 0;

    // (Si tu línea es NEGRA sobre fondo BLANCO, descomenta esta línea):
    // int valorLimpio = 1023 - sensorValues[i];

    if (sensorValues[i] > UMBRAL_LINEA) {
      sensoresActivos++;
    }

    sumaPonderada += (long)valorLimpio * pesos[i];
    sumaTotal += valorLimpio;
  }

  // Si ningún sensor ve la línea, retorna 0 (o mantén el último error)
  if (sumaTotal == 0 || sensoresActivos == 0) return 0.0;

  return (float)sumaPonderada / sumaTotal;
}

void loop() {
  // 1. Verificación de parada de emergencia por MicroStart (si el control envía STOP)
  if (digitalRead(MicroStart) == LOW) {
    xmotion.StopMotors(0);
    digitalWrite(LLED, LOW);
    digitalWrite(RLED, LOW);
    
    // Esperar hasta que se vuelva a dar START
    while (digitalRead(MicroStart) == LOW) {
      delay(50);
    }
    digitalWrite(LLED, HIGH);
    digitalWrite(RLED, HIGH);
  }

  // 2. Lectura y procesamiento de sensores
  leerMultiplexor();
  float error = obtenerErrorPosicion();

  // 3. Inferencia difusa
  fuzzy->setInput(1, error);
  fuzzy->fuzzify();
  float correccion = fuzzy->defuzzify(1);

  // 4. Velocidad base y turbo en recta
  int velocidadBase = SpeedInit;
  bool turboActivo = false;
  
  if (sensorValues[15] > UMBRAL_LINEA && abs(error) < 2.0) {
    velocidadBase += SpeedPlus;
    turboActivo = true;
  }

  // 5. Control de motores
  int pwmIzq = constrain(velocidadBase + (int)correccion, -255, 255);
  int pwmDer = constrain(velocidadBase - (int)correccion, -255, 255);
  xmotion.MotorControl(pwmIzq, pwmDer);

  // 6. Impresión Serial condicional por DipSwitch 2 (DS2 en ON = LOW)
  if (digitalRead(DS2) == LOW) {
    if (millis() - lastSerialTime >= 60) { // Imprime cada 60 ms para no frenar el ciclo
      lastSerialTime = millis();
      
      Serial.print(F("Err: "));
      Serial.print(error, 2);
      Serial.print(F(" | Corr: "));
      Serial.print(correccion, 1);
      Serial.print(F(" | Sens15: "));
      Serial.print(sensorValues[15]);
      Serial.print(F(" | Turbo: "));
      Serial.print(turboActivo ? F("ON ") : F("OFF"));
      Serial.print(F(" | PWM_I: "));
      Serial.print(pwmIzq);
      Serial.print(F(" | PWM_D: "));
      Serial.println(pwmDer);
    }
  }
}