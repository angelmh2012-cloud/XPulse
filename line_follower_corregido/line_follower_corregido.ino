// =====================================================================
// Seguidor de línea con PID - Versión corregida
// Cambios respecto al original:
//  - Se quitó el uso del pin 1 (TX serie) como salida digital.
//  - Se eliminó NosTime / delay() muerto dentro de la lectura de sensores.
//  - ReadSensor() reescrito con tablas en vez de 32 bloques if/else.
//  - Lógica de "línea perdida" reemplazada por una explícita y funcional.
//  - Se agregó espera del botón/switch de Start antes de arrancar.
//  - Se quitó MotorControl2() por no usarse (revisar si la necesitas).
// ========================================================

//Multiplexor: el multiplexor es el conjunto de todos nuestros 16 sensores de linea 0-15
//DeepSwitch: sirve para poner distintas funciones dentro de un mismo codigo

#define RightMotorDir     13 //indica la direccion del motor derecho
#define RightMotorPwmPin  11 //indica el pin pwm de motor derecho
#define LeftMotorDir      12 //indica la direccion del motor izquierdo
#define LeftMotorPwmPin   10 //indica el pin pwm de motor izquierdo
#define Led1 8 //indica el pin del LED indicador numero 1
#define Led2 9//indica el pin del LED indicador numero 2

#define DipSwitch1 5 //este es el DipSwitch 1 (281), en este caso para la ponderacion proporcional(kp)
#define DipSwitch2 6 //este es el DipSwitch 2 (288), en este caso para el Limite minimo y limite maximo 
#define DipSwitch3 7 //este es el DipSwitch 3 (295), en este caso para la ponderacion derivativa(kd)

#define Start A0 //aqui definimos el Microstart W si tu módulo es activo en bajo

#define s0 A2 //definimos nuestro primer pin de control del multiplexor en el pin analogico A2 
#define s1 A4 //definimos el segundo pin del multiplexor en el pin analogico A4 
#define s2 A5 //definimos nuestro tercer pin de control del mulen el Pin analogico A0 (el microstart sirve para prender el robot de manera remota )
#define MICROSTART_ACTIVO LOW // cambia a LOtlexor en el pin analogico A5 
#define s3 4 //definimos nuestro cuarto y ultimo pin de control del multiplexor en el pin 4 (el multiplexor es el conjunto de todos nuestros 16 sensores de linea 0-15)
#define Sens A1 //este "sens" ayuda con la lectura de los sensores del multiplexor (el multiplexor es el conjunto de todos nuestros 16 sensores de linea 0-15)
#define Trimpot A3 //definimos nuestro trimpot en el pin analogico A3 (el trimpot es el potenciometro que nos ayuda a regular la base_speed y la speed_plus) 

int valor;
int a = 0;              // error calculado por el sensor
int b = 0;               // marca si un sensor extremo detectó línea (+-500)
int Threshold = 300;     // umbral blanco/negro
int LimitMin = -220; // este es nuestro limite minimo original, tambien lo puede afectar el dipswitch, se representa en negativos debido a que la velocidad negativa infiere en la direccion
int LimitMax = 200; //este es nuestro limite maximo original, tambien puede ser afectado por el DipSwitch
int Error; //esta es nuestra variable del error ayuda a saber en que posicion estamos respecto a la posicion ideal, tambien afecta a la correcion 
int LastError = 0; //este es nuestra variable del ultimo error sirva para calcular el error conforme a la posicion actual del robot
int TotalError; //es la variable que nos ayuda a identificar el valor de la suma de todos los errores
int Correction;
int BaseSpeed;
int ExtraNos;
int NoNos;

float Kp;
float Ki;
float Kd;

int RightMotorPwm = 0;
int LeftMotorPwm = 0;
int zemin = 1;          // 1 = línea negra sobre fondo blanco, 0 = invertido

int lastDirection = 0;  // última dirección conocida de la línea (para recuperación)

// ---------------------------------------------------------------------
// Tabla de canales del multiplexor (S0,S1,S2,S3) para cada sensor i=0..15
// (misma asignación de canales que el código original)
// ---------------------------------------------------------------------
const bool muxChannels[16][4] = {
  {1,1,1,0}, // i=0  centro
  {0,0,0,1}, // i=1
  {0,1,1,0}, // i=2
  {1,0,1,0}, // i=3
  {1,0,0,1}, // i=4
  {0,0,1,0}, // i=5
  {0,1,0,1}, // i=6
  {0,1,0,0}, // i=7
  {1,1,0,0}, // i=8
  {1,1,0,1}, // i=9
  {0,0,1,1}, // i=10
  {1,0,1,1}, // i=11
  {0,1,1,1}, // i=12
  {1,1,1,1}, // i=13 (canal central, se reutiliza para el boost de recta)
  {0,0,0,0}, // i=14
  {1,0,0,0}, // i=15
};//matriz de datos, 16 x 4 

const int  sensorPos[16]     = {0,-30,30,80,-80,100,-100,220,150,-150,-220,-350,-490,0,490,350}; //valor de las posiciones de los sensores
const bool sensorExtreme[16] = {0,0,0,0,0,0,0,1,1,1,1,1,1,0,1,1}; // indica si seguimos centrados en la linea o salimos completamente marcandolos sensores con mayor valor absoluto |X|

// ---------------------------------------------------------------------

void setup() {
  pinMode(DipSwitch1, INPUT);
  pinMode(DipSwitch2, INPUT);
  pinMode(DipSwitch3, INPUT);
  pinMode(Start, INPUT);

  digitalWrite(DipSwitch1, HIGH);
  digitalWrite(DipSwitch2, HIGH);
  digitalWrite(DipSwitch3, HIGH);

  pinMode(2, OUTPUT);
  pinMode(Led1, OUTPUT);
  pinMode(Led2, OUTPUT);
  pinMode(s0, OUTPUT);
  pinMode(s1, OUTPUT);
  pinMode(s2, OUTPUT);
  pinMode(s3, OUTPUT);

  digitalWrite(s0, LOW);
  digitalWrite(s1, LOW);
  digitalWrite(s2, LOW);
  digitalWrite(s3, LOW);

  // Secuencia de inicio con LEDs
  for (int xp = 0; xp < 3; xp++) {
    digitalWrite(Led1, HIGH);
    digitalWrite(Led2, HIGH);
    delay(500);
    digitalWrite(Led1, LOW);
    digitalWrite(Led2, LOW);
    delay(500);
  }

  //int SpeedPlus = analogRead(Trimpot);
  BaseSpeed = 177; //(30 + (SpeedPlus / 8)); // Min 30 - Max 178
  ExtraNos = BaseSpeed + 10;
  NoNos = BaseSpeed;

  pinMode(Sens, INPUT);
  pinMode(RightMotorDir, OUTPUT);
  pinMode(RightMotorPwmPin, OUTPUT);
  pinMode(LeftMotorDir, OUTPUT);
  pinMode(LeftMotorPwmPin, OUTPUT);

  Serial.begin(9600);
}

// Selecciona un canal del multiplexor y espera a que la señal se asiente
void seleccionarCanal(int i) {
  digitalWrite(s0, muxChannels[i][0]);
  digitalWrite(s1, muxChannels[i][1]);
  digitalWrite(s2, muxChannels[i][2]);
  digitalWrite(s3, muxChannels[i][3]);
  delayMicroseconds(10);
}

int ReadSensor() {
  int zemin1, zemin2;

  // Canal 5 y 6 se usan como referencia para decidir si la línea es negra o blanca
  seleccionarCanal(5);
  zemin1 = analogRead(Sens);
  seleccionarCanal(6);
  zemin2 = analogRead(Sens);




  if (zemin1 < Threshold && zemin2 < Threshold) {
    zemin = 0;
  } else if (zemin1 > Threshold && zemin2 > Threshold) {
    zemin = 1;
  }





  a = 0;
  b = 0;
  bool lineaDetectada = false;

  for (int i = 0; i <= 15; i++) {
    if (i == 13) continue; // canal central, no aporta posición directa

    seleccionarCanal(i);
    int lectura = analogRead(Sens);
    bool detecta = (zemin == 1) ? (lectura < Threshold) : (lectura > Threshold);

    if (detecta) {
      lineaDetectada = true;
      a = sensorPos[i];
      if (sensorExtreme[i]) {
        b = (sensorPos[i] > 0) ? 500 : -500;
      }
    }
  }

  //if (lineaDetectada) {
    //lastDirection = a;
  //} else {
    // Línea perdida: gira con fuerza hacia el lado donde se vio por última vez
    a = (lastDirection >= 0) ? LimitMax : LimitMin;
  //}

  // Boost de velocidad en recta: reutiliza el canal central (i=13)
  seleccionarCanal(13);
  int centro = analogRead(Sens);
  bool rectaDetectada = (zemin == 1) ? (centro < 500) : (centro > 500);

  if (lineaDetectada && a == 0 && rectaDetectada) {
    BaseSpeed = ExtraNos;
    digitalWrite(Led1, HIGH);
    digitalWrite(Led2, HIGH);
  } else {
    BaseSpeed = NoNos;
    digitalWrite(Led1, LOW);
    digitalWrite(Led2, LOW);
  }

  return a;
}

void MotorControl(int LeftMotorPwm, int RightMotorPwm) {
  if (LeftMotorPwm <= 0) {
    LeftMotorPwm = abs(LeftMotorPwm);
    digitalWrite(LeftMotorDir, LOW);
    analogWrite(LeftMotorPwmPin, LeftMotorPwm);
  } else {
    digitalWrite(LeftMotorDir, HIGH);
    analogWrite(LeftMotorPwmPin, LeftMotorPwm);
  }

  if (RightMotorPwm <= 0) {
    RightMotorPwm = abs(RightMotorPwm);
    digitalWrite(RightMotorDir, LOW);
    analogWrite(RightMotorPwmPin, RightMotorPwm);
  } else {
    digitalWrite(RightMotorDir, HIGH);
    analogWrite(RightMotorPwmPin, RightMotorPwm);
  }
}

void loop() {

  // =============================
  // ARRANQUE AUTOMÁTICO DE PRUEBA
  // =============================

  // Espera la señal de arranque (botón / MicroStart) antes de moverse.
  // Verifica la polaridad de tu switch: si tu Start es activo en bajo,
  // cambia esta condición a "== HIGH".
  // Espera a que el microstart se active
  valor = analogRead(Trimpot);
  Serial.println(valor);

while (digitalRead(Start) != MICROSTART_ACTIVO) {
  digitalWrite(Led1, HIGH);   // LEDs parpadeando mientras espera
  digitalWrite(Led2, LOW);
  delay(100);
  digitalWrite(Led1, LOW);
  digitalWrite(Led2, HIGH);
  delay(100);
}
digitalWrite(Led1, LOW);
digitalWrite(Led2, LOW);


  // =============================
  // CONTROL DEL ROBOT
  // =============================

  static float filteredError = 0;

  while (1) {
    if (digitalRead(Start) != MICROSTART_ACTIVO) {
  MotorControl(0, 0);
  TotalError = 0;      // reinicia la integral
  LastError = 0;
  return;              // sale de loop() y vuelve a esperar el arranque
}

    ReadSensor();

    Error = a;

    filteredError = (filteredError * 0.7) + (Error * 0.3);

    TotalError += filteredError;
    TotalError = constrain(TotalError, -15000, 15000);

    Correction = (Kp * filteredError) +
                 (Ki * TotalError) +
                 (Kd * (filteredError - LastError));

    LastError = filteredError;

    RightMotorPwm = BaseSpeed + Correction;
    LeftMotorPwm  = BaseSpeed - Correction;

    RightMotorPwm = constrain(RightMotorPwm, LimitMin, LimitMax);
    LeftMotorPwm  = constrain(LeftMotorPwm, LimitMin, LimitMax);

    MotorControl(LeftMotorPwm, RightMotorPwm);

    // Ajustes PID con DIP switch
    if (digitalRead(DipSwitch1) == 1) {
      Kp = 2;
    } else {
      Kp = 1;

    }

    if (digitalRead(DipSwitch2) == 1) {
      LimitMin = -180;
      LimitMax = 180;
    } else { 
      LimitMin = -220;
      LimitMax = 220;
    }

    if (digitalRead(DipSwitch3) == 1) {
      Kd = 3.5;
    } else {
      Kd = 7.0;
    }

    Ki = 0.0005;
  }
}
