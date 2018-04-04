/*
    Sistema de control PID para bola en barra, con 3 botones para activar/desasctivar la componente proporcional, integral y derivativa, y un led de consigna alcanzada.
    Autor: Javier Vargas. El Hormiguero 04/04/2018
    Diseño basado en Estudio Roble http://roble.uno/control-pid-barra-y-bola-arduino/
    https://creativecommons.org/licenses/by-sa/4.0/
*/

//PINES
#define pinServo 6
#define pinLedOk 7
#define pinLedKp 8
#define pinLedKi 9
#define pinLedKd 10
#define pinKp 11
#define pinKi 12
#define pinKd 13
#define pinSensorIZQ A0
#define pinSensorDER A1 //**Inutilizado

//CONFIGURACION
#define muestreo 50 //milisegundos de muestreo
#define Kp 0.18 //Control proporcional
#define Ki 0.05 //Control integral
#define Kd 0.1 //Control dervativo
#define OffsetSensorIZQ 24 //Offset del sensor (0 en el centro)
#define OffsetSensorDER 0 //Offset del sensor (0 en el centro)
#define OffsetServo 50 //Offset del servomotor
#define RangoOK 25 //rango entorno al centro que el led ok enciende

//Librerias y variables
#include <Servo.h>
Servo servo;

#include <PID_v1.h>
double KpA = 0, KiA = 0, KdA = 0;
float outMax = 50;
double Setpoint, Input, Output;
PID pid(&Input, &Output, &Setpoint, KpA, KiA, KdA, DIRECT);

int dcal [] = { -193, -160, -110, -60, 0, 40, 60, 90, 120}; // Calibracion de ADC a Distancia
int ADCcal [] = {177, 189, 231, 273, 372, 483, 558, 742, 970};

float Distancia = 0;
float Distancia0 = 0;
float Velocidad = 0;
float Velocidad0[5];

int measureIZQ = 0;
int measureDER = 0;
int DIZQ = 0;
int DDER = 0;

unsigned long millisactual;

void setup() {
  Serial.begin(115200);

  // AREF conectado a 3.3V
  analogReference(EXTERNAL);

  //SERVO
  servo.attach(pinServo);

  //PINES
  pinMode(pinKp, INPUT_PULLUP);
  pinMode(pinKi, INPUT_PULLUP);
  pinMode(pinKd, INPUT_PULLUP);
  pinMode(pinLedKp, OUTPUT);
  pinMode(pinLedKi, OUTPUT);
  pinMode(pinLedKd, OUTPUT);
  pinMode(pinLedOk, OUTPUT);

  digitalWrite(pinLedKp, LOW);
  digitalWrite(pinLedKi, LOW);
  digitalWrite(pinLedKd, LOW);
  digitalWrite(pinLedOk, LOW);

  //PID
  pid.SetTunings(KpA, KiA, KdA);
  pid.SetOutputLimits(-outMax, outMax); //Límite de la salida del pid
  pid.SetMode(AUTOMATIC);

}
//////////////////
////////LOOP//////
//////////////////

void loop() {
  if (millis() / muestreo != millisactual) {
    millisactual = millis() / muestreo;
    BotonKp();
    BotonKi();
    BotonKd();
    MedirDistancia();
    ControlPID();
    ConsignaOK();
  }
}

//////////////////
//////////////////
//////////////////


void ConsignaOK() {
  if (abs(Distancia) < RangoOK - 3) {
    digitalWrite(pinLedOk, HIGH);
  }
  if (abs(Distancia) > RangoOK + 3) {
    digitalWrite(pinLedOk, LOW);
  }
}

void ControlPID() {
  Input = Distancia; //ENTRADA
  Setpoint = 0; //CONSIGNA
  pid.Compute(); //Angulo desado
  servo.write(Output + OffsetServo);
}

void CalcularVelocidad() {

  Velocidad = 1000 * (float)(Distancia - Distancia0) / muestreo;
}

void MedirDistancia() {
  Distancia0 = Distancia; //Distancia anterior

  // Medida analógica IZQUIERDA
  int DIZQt = 0;
  //Media de varias lecturas
  for (int i = 0; i < 10; i++) {

    measureIZQ = constrain(analogRead(pinSensorIZQ), ADCcal[0], ADCcal[8]);

    // Curva de Calibracion de ADC a mm
    for (int i = 0; i < 8; i++) {
      if (measureIZQ >= ADCcal[i] && measureIZQ < ADCcal[i + 1]) {
        DIZQ = map(measureIZQ, ADCcal[i], ADCcal[i + 1], dcal[i], dcal[i + 1]);
      }
    }
    DIZQt += DIZQ;
  }
  DIZQ = DIZQt / 10;

  Distancia = DIZQ + OffsetSensorIZQ;

  //Filtro paso baja variable
  float K = (float) map(abs(Distancia), 0, 10, 70, 95) / 100;
  K = constrain(K, 0.5, 0.9);
  Distancia = K * Distancia + (1 - K) * Distancia0;
}

void BotonKp() {
  static boolean s = 0;
  static boolean ON = 0;
  static unsigned long mils = 0;

  if (!s) { //Si no esta pulsado
    if (digitalRead(pinKp) == 0) {
      s = 1;
      mils = millis();
      ON = !ON;
      KpA = Kp * ON;
      pid.SetTunings(KpA, KiA, KdA);
      digitalWrite(pinLedKp, ON);
    }
  }
  if (s) { //Si esta pulsado
    if (digitalRead(pinKp) == 1 && millis() > mils + 100) {
      s = 0;
    }
  }
}

void BotonKi() {
  static boolean s = 0;
  static boolean ON = 0;
  static unsigned long mils = 0;

  if (!s) { //Si no esta pulsado
    if (digitalRead(pinKi) == 0) {
      s = 1;
      mils = millis();
      ON = !ON;
      KiA = Ki * ON;
      pid.SetTunings(KpA, KiA, KdA);
      digitalWrite(pinLedKi, ON);
      //Reinicio del integral
      pid.SetOutputLimits(0.0, 1.0);  // Forces minimum up to 0.0
      pid.SetOutputLimits(-1.0, 0.0);  // Forces maximum down to 0.0
      pid.SetOutputLimits(-outMax, outMax);
    }
  }
  if (s) { //Si esta pulsado
    if (digitalRead(pinKi) == 1 && millis() > mils + 100) {
      s = 0;
    }
  }
}

void BotonKd() {
  static boolean s = 0;
  static boolean ON = 0;
  static unsigned long mils = 0;

  if (!s) { //Si no esta pulsado
    if (digitalRead(pinKd) == 0) {
      s = 1;
      mils = millis();
      ON = !ON;
      KdA = Kd * ON;
      pid.SetTunings(KpA, KiA, KdA);
      digitalWrite(pinLedKd, ON);
    }
  }
  if (s) { //Si esta pulsado
    if (digitalRead(pinKi) == 1 && millis() > mils + 100) {
      s = 0;
    }
  }
}


