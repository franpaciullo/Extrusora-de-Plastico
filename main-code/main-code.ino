
#include <AccelStepper.h>
#include <LiquidCrystal_I2C.h>
#include <math.h>
#include <Wire.h>
#include <PID_v1.h>

#define motorInterfaceType 1

//parametros para cambiar

  const float tempBuscada = 230.0; //230 para PP
  const int velocidadPaPgrande = -70; //-70 para PP
  const int velocidadPaPchico = 1000; 
  const byte frecuenciaVariador = 105; // Valores de 0 a 255 // 105 ~= 19.5Hz // 100 ~= 18.7Hz // 80 ~= 14.9Hz

  unsigned long esperaCalentando = 120000; 

//

//PROGRAMA ON/OFF
bool estado = 0;

bool encendidoMotores = 0;

//TIEMPOS

unsigned long tiempoActual = 0;
unsigned long tiempo1 = 0;
unsigned long tiempo2 = 0;
unsigned long tiempo3 = 0;
unsigned long tiempo4 = 0;
unsigned long lastPidTime = 0;
const unsigned long pidInterval = 200;

// PULSADORES

const int redButtonPin = 35;
const int greenButtonPin = 33;
const int selectionButtonPin = 31;
const int potPin = A2;

bool currentSelectState = LOW;
bool lastSelectState = LOW;
bool currentGreenState = LOW;
bool lastGreenState = HIGH;

//Motores Paso a Paso

//Velocidades Paso a Paco en "//parametros" Line 10
const int ENA1 = 37;
const int STP1 = 39;
const int DIR1 = 41;
const int ENA2 = 43;
const int STP2 = 45;
const int DIR2 = 47;

const int FC1 = 23;
const int FC2 = 25;

AccelStepper myStepper1(motorInterfaceType, STP1, DIR1);
AccelStepper myStepper2(motorInterfaceType, STP2, DIR2);

const int thermistorPin = A0;
const int relayPin=12;

//Variador de frecuencia

//Velocidad variador de frecuencia en "//parametros" Line 10
const int motorPin = 11;

//Sensor RPM

const uint8_t sensorPin = 2;  // pin digital 2 (interrupción INT0)
const int PPR = 20;            // Pulsos por vuelta, ajustar según tu disco

volatile unsigned long lastPulseMicros = 0;
volatile unsigned long pulsePeriodMicros = 0;
volatile bool newPeriod = false;

float rpmFiltered = 0;     // RPM filtrada
const float alpha = 0.2;   // factor de suavizado (0.1 - 0.3 recomendado)

//TEMPERATURA

bool tempAlcanzada = false;
//tempBuscada en "//parametros" Line 10
float tempActual=0.0;

const float seriesResistor = 4700.0;      // resistencia en ohms
const float nominalResistance = 100000.0; // resistencia del NTC a 25°C
const float nominalTemperature = 25.0;    // temperatura nominal en °C
const float bCoefficient = 3950.0;        // B del NTC
const float vRef = 5.0;                  // voltaje de referencia
const int adcMax = 1023;

byte i = 0;

float readThermistor() {
  int adcValue = analogRead(thermistorPin);
  float voltage = adcValue * vRef / adcMax;

  float resistance = seriesResistor * (voltage / (vRef - voltage));

  float steinhart;
  steinhart = resistance / nominalResistance;          // R/Ro
  steinhart = log(steinhart);                          // ln(R/Ro)
  steinhart /= bCoefficient;                           // 1/B * ln(R/Ro)
  steinhart += 1.0 / (nominalTemperature + 273.15);   // + 1/To
  steinhart = 1.0 / steinhart;                         // invertimos
  steinhart -= 273.15;                                 // °C

  return steinhart;
}


LiquidCrystal_I2C lcd(0x27, 16, 2);

double Setpoint, Input, Output;
double Kp = 10.0;
double Ki = 0.5;
double Kd = 0.0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);


void setup() {
  lcd.init();
  lcd.backlight();

  pinMode(selectionButtonPin, INPUT);
  pinMode(greenButtonPin, INPUT);
  pinMode(redButtonPin, INPUT);
  pinMode(FC1, INPUT);
  pinMode(FC2, INPUT);

  pinMode(potPin, INPUT);

  pinMode(motorPin, OUTPUT);
  analogWrite(motorPin, 0);

  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, LOW);

  Setpoint = tempBuscada;

  myPID.SetOutputLimits(0, 255); // límites de salida
  myPID.SetSampleTime(500);      // cada 500 ms
  myPID.SetMode(AUTOMATIC);      // activar PID

  pinMode(sensorPin, INPUT); 
  attachInterrupt(digitalPinToInterrupt(sensorPin), isr_pulse, FALLING);

  pinMode(ENA1, OUTPUT);
  pinMode(ENA2, OUTPUT);

  digitalWrite(ENA1, HIGH);
  digitalWrite(ENA2, LOW);

  myStepper1.setMaxSpeed(1000);
  myStepper1.setAcceleration(50);
  myStepper1.setSpeed(velocidadPaPgrande); // Cambiar valor en "//Paremetros"

  myStepper2.setMaxSpeed(1000);
  myStepper2.setAcceleration(50);
  myStepper2.setSpeed(velocidadPaPchico); // Cambiar valor en "//Paremetros"
}

void loop() {

  currentSelectState = digitalRead(selectionButtonPin);
  currentGreenState = digitalRead(greenButtonPin);

  if (!estado){
    esperando();
  }
  else if(estado){
    temp();
    motores();
    // rpm();
    display();
  }
}

void esperando(){

  if (currentGreenState == LOW && lastGreenState == HIGH) {
    estado = 1;
    lcd.clear();
    lcd.print("INICIANDO");
    delay(2000);
  }
  else {

    if (millis() > tiempo1 + 2000) {
      i = (i + 1) %2;
      tiempo1 = millis();
      lcd.clear();

      if (i==1) {
        lcd.setCursor(0,0);
        lcd.print("EXTRUSORA       ");
        lcd.setCursor(0,1);
        lcd.print("DE PLASTICO     ");
      } 
      else if(i==0){
        lcd.setCursor(0,0);
        lcd.print("PRESIONAR VERDE ");
        lcd.setCursor(0,1);
        lcd.print("PARA INICIAR    ");
      }
    }
    

  }

  lastGreenState = currentGreenState;

}

void temp(){
  tempActual = readThermistor();
  
  Input = tempActual;
  myPID.Compute();
  
  if (Output > 10){
    digitalWrite(relayPin, HIGH);
  }
  else{
    digitalWrite(relayPin, LOW);
  }
  
  if(!tempAlcanzada) {

    lcd.setCursor(0, 0);
    lcd.print("CALENTANDO      ");
    
    if(millis() > tiempo2 + 500){
      tiempo2 = millis();
    lcd.setCursor(0, 1);
    lcd.print(tempActual);
    lcd.print(" C         ");

    if(tempActual >= tempBuscada) {
      tempAlcanzada = true;
      lcd.clear();
      lcd.print("TEMPERATURA");  
      lcd.setCursor(0, 1);
      lcd.print("ALCANZADA");
      delay(3000);
      tiempo4 = millis();
      lcd.clear();

    }

    }

    
  }  
}

void motores(){
  if(tempAlcanzada && encendidoMotores){
  analogWrite(motorPin, frecuenciaVariador); //Cambiar valor en "//Paremetros"
  
  myStepper1.runSpeed();
  /*myStepper2.runSpeed();

  if(!digitalRead(FC1)){
    myStepper2.setSpeed(-1000);
  }
  else if(!digitalRead(FC2)) {
    myStepper2.setSpeed(1000);
    
  }*/

  }
}

/*
void rpm(){
  if (newPeriod) {
    noInterrupts();
    unsigned long period = pulsePeriodMicros;
    newPeriod = false;
    interrupts();

    if (period > 0) {
      float rpm = 60.0 * 1000000.0 / (period * PPR);  // RPM instantánea
      // Promedio exponencial para filtrar
      rpmFiltered = alpha * rpm + (1 - alpha) * rpmFiltered;
    }
  }

  // Detectar motor parado
  if (micros() - lastPulseMicros > 2e6) { // 2 segundos sin pulsos
    rpmFiltered = 0;
  }
}

void isr_pulse() {
  unsigned long now = micros();
  if (lastPulseMicros != 0) {
    unsigned long period = now - lastPulseMicros;
    if (period > 80) { // descartar pulsos ruidosos
      pulsePeriodMicros = period;
      newPeriod = true;
    }
  }
  lastPulseMicros = now;
}
*/


void display(){
  if(tempAlcanzada && millis() > tiempo3 + 500) {
    lcd.setCursor(0, 0);
    lcd.print("TEMP: "); // Second screen
    lcd.print(tempActual, 1); // Display current temperature with 1 decimal
    lcd.print(" C   ");
    tiempo3 = millis();

    if(!encendidoMotores){
    if(millis() <= tiempo4 + esperaCalentando){
    lcd.setCursor(0, 1);
    lcd.print("ESPERA: ");
    lcd.print(((tiempo4 + esperaCalentando) - millis()) / 1000);
    lcd.print(" SEG  ");
    } else{
      encendidoMotores = 1;
      lcd.clear();
    }
    }
    /*lcd.setCursor(0, 1);
    lcd.print(rpmFiltered, 1);
    lcd.print(" RPM         ");
    */
  }
}





