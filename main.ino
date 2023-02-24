#include <SoftwareSerial.h>
#include <Stepper.h>
#include <PID_v1.h>

#define IN1 11 // IN1 do motor de passos
#define IN2 10 // IN2 do motor de passos
#define IN3 9  // IN3 do motor de passos
#define IN4 8  // IN4 do motor de passos
#define LEITURA A1   // Leitura do sensor

// Variaveis do motor e passos
int PASSOS = 2048;
int RPM = 10;
int contadorPassos = 0;
int posicaoMaxima = 300 * PASSOS / 360;
int posicaoMinima = -300 * PASSOS / 360;

//Define variaveis App bluetooth
String bluetooth;
int bluetooth_int;
int bluetooth_setpoint = 31;

// Parâmetros do termistor
const double beta = 3600.0;
const double r0 = 10000.0;
const double t0 = 273.0 + 25.0;
const double rx = r0 * exp(-beta/t0);
double temperatura;

// Parâmetros do circuito
const double vcc = 5.0;
const double R = 10000.0;

// Numero de amostras na leitura
const int amostras = 10;

// Variaveis do PID
double Setpoint;  // Setpoint pelo bluetooth
double Input;     // Sensor de anguloeratura
double Output;    // Motor de passos

// Parametros do PID
double Kp=0.30;
double Ki=0.01;
double Kd=0.03;

// Chama a funcao PID da biblioteca
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// Chama a funcao do Motor de passo
Stepper myStepper(PASSOS, IN1, IN3, IN2, IN4);

// TX-RX Bluetooth
SoftwareSerial HC05(2, 3);

void setup(){
    Serial.begin(9600);

    pinMode(LEITURA, INPUT);

    // Taxa de transmissao
    HC05.begin(9600);

    myStepper.setSpeed(RPM);

    // Filtro do sensor de temperatura
    int soma = 0;
    for (int i = 0; i < amostras; i++) {
        soma += analogRead(LEITURA);
        delay (1);
    }

    // Determina a resistência do termistor
    double v = (vcc*soma)/(amostras*1024.0);
    double rt = (vcc*R)/v - R;

    // Calcula a temperatura
    double t = beta / log(rt/rx);
    temperatura = t-273.0;

    Input = temperatura;

    // Liga o PID
    myPID.SetMode(AUTOMATIC);

    // Ajusta os valores do PID
    myPID.SetTunings(Kp, Ki, Kd);
    myPID.SetOutputLimits(-255,255);

}

void loop(){

    Serial.begin(9600);

    // Filtro do sensor de temperatura
    int soma = 0;
    for (int i = 0; i < amostras; i++) {
        soma += analogRead(LEITURA);
        delay (1);
    }

    // Determina a resistência do termistor
    double v = (vcc*soma)/(amostras*1024.0);
    double rt = (vcc*R)/v - R;

    // Calcula a temperatura
    double t = beta / log(rt/rx);
    temperatura = t-273.0;

    Input = temperatura;

    // Setpoint pelo bluetooth
    Setpoint = bluetooth_setpoint;
    bluetooth = HC05.read();
    bluetooth_int =  bluetooth.toInt();
    if(bluetooth_int > -1){    //verifica se foi mandado algum valor pelo bluetooth.
        Setpoint = bluetooth_setpoint = bluetooth.toInt();
    }

    // Calcula o PID
    myPID.Compute();

    // Output para o motor de passos
    // Limitador de passos
    if (contadorPassos < posicaoMaxima) {
        myStepper.step(Output);
        contadorPassos++;

        if(contadorPassos > posicaoMinima) {
            myStepper.step(Output);
            contadorPassos--;
        }
    }

    Serial.print(Setpoint);
    Serial.print(" ");
    Serial.print(Input);
    //Serial.print(" ");
    //Serial.print(Output);
    //Serial.print(" ");
    //Serial.print(atual);
    Serial.println(" ");

    delay(10);

}

// Filtro para o input do sensor
//float filtroSensor(){
//
//  for(int i=0; i<amostras; i++){
//    double temperature;
//    media += temperature;
//
//    delay(1);
//    }
//
//  leitura = media/amostras;
//
//  return leitura;
//
//  delay(1);
//}