#include <Stepper.h>
#include <PID_v1.h>

// Definições do motor de passos
#define IN1 11 // IN1
#define IN2 10 // IN2
#define IN3 9  // IN3
#define IN4 8  // IN4
#define PASSOS 2048
#define RPM 10

// Definições do PID
#define SETPOINT A1  // Controle do potenciômetro
#define LEITURA A0   // Leitura do sensor
#define SAIDA 3      // Sinal de saída

// Variável para o plotter serial
unsigned long lastSend = 0;

// Variáveis do PID
double Setpoint;  // Setpoint pelo bluetooth
double Input;     // Sensor de temperatura
double Output;    // Motor de passos

double aux;

// Parâmetros do PID
double Kp=0.02;
double Ki=0.09;
double Kd=0.00 ;

// Variáveis do motor de passo
int atual = 0;  // Variável global para armazenar o ângulo atual
int flag = 0;   // Sinalizador ativo ao inserir o ângulo através da comunicação serial
int temp = 0;   // Recebe o angulo

// Chama a função PID da biblioteca
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// Chama a função do Motor de passo
Stepper myStepper(PASSOS, IN1, IN3, IN2, IN4);

void setup(){

    Serial.begin(115200);

    myStepper.setSpeed(RPM);

    Input = analogRead(LEITURA);

    //Turn the PID on
    myPID.SetMode(AUTOMATIC);

    //Adjust PID values
    myPID.SetTunings(Kp, Ki, Kd);

    myPID.SetOutputLimits(-255, 255);

    pinMode(SETPOINT, INPUT);

}

void loop(){

    Serial.begin(115200);

    Input = analogRead(LEITURA);

    Setpoint = map(analogRead(SETPOINT),0,1023,480,800);

    // Calcula o PID
    myPID.Compute();

    // Codigo do angulo
    temp = Output;                // Recebe o output do cálculo do PID
    temp = toStep(temp);          // Converter ângulo em temp para passo
    aux = temp + atual;
    myStepper.step(temp);
    atual = (atual + temp) % 360; // Ângulo continua a armazenar o ângulo atualmente adicionado em unidades de 0 a 360 graus


    if(millis()-lastSend > 100){
        lastSend = millis();

        Serial.print(Setpoint);
        Serial.print(" ");
        Serial.print(Input);
        Serial.print(" ");
        Serial.print(Output);
        Serial.println(" ");
        //delay(100);
    }

    delay(100);
}

// Converte ângulo recebido da comunicação serial para passo
// Ambos os ângulos CW e CCW estão disponíveis
// pode lidar mesmo que exceda 360 graus
int toStep(int atual){
    int quotient = atual / 360;
    int remainder = atual % 360;
    remainder = map(remainder, -360, 360, -2048, 2048);
    if(abs(quotient) > 0)
        return (2048 * quotient) + remainder;
    else
        return remainder;
}

//descrição do código
//angle = ( angle + temp ) % 360 : Salve o ângulo absoluto do motor de passo a ser alterado na variável de ângulo. temp é o ângulo recebido através da comunicação serial.
//serialEvent() : Esta função é chamada quando há dados no buffer de recepção serial. Quando um número é inserido através da comunicação serial e então um enter é pressionado, o sinalizador é ativado como verdadeiro.
//toStep() : Retorna o número inserido através da comunicação serial como o número de voltas. O código foi escrito para processar números inseridos acima de ±360º.
//ErrorHandler() : Esta função emite um erro quando os valores normais de ângulo (número) não são recebidos através da comunicação serial.