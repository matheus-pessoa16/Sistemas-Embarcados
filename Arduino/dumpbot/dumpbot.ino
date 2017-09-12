#include <Arduino.h>

#include<Servo.h>

/*        PINOS USADOS
3 = pino de sinal do elevador
4 = pino de enable da ponte h
5 = pino de sinal do elevador
6 = sensor inferior do elevador
7 = sensor superior do elevador
9 = pino PWM para o servo da tampa
10 = pino PWM para o servo que varre o lixo 
11 = pino PWM para o servo que joga o lixo
A0 = leitura do sensor de lixo
A1 = leitura do sensor de nível do reservatório
*/


/*              COISAS QUE ESTÃO FALTANDO

- implementar as flags do sistema para indicar o movimento da plataforma;
- sensores de detecção de obstáculos para a plataforma móvel
-

*/


//#define led_lixo  4
#define sensor_elev_sup 7
#define sensor_elev_inf 6
#define sensor_lixo A0
#define sensor_nivel A1
//#define direcao 2//0 - para baixo  // 1 - para cima
#define enable_elevador 4
#define pin_elevador_1 5
#define pin_elevador_2 3


Servo servo_pa;
Servo servo_joga_lixo;
Servo servo_tampa;

//*****************************************
//            DEFINIÇÕES
//servo_pa = 9 -> servo que empurra o lixo
//servo_plat = 10 -> servo que joga o lixo
//servo_tampa = 11 -> servo que abre a tampa da lixeira
//*****************************************



void servos(int servo){
//Função que controla os servos
    switch (servo) {
      case 1:
        servo_pa.write(5);
        delay(1000);
        servo_pa.write(178);
        delay(400);
      break;

      case 2:
        servo_joga_lixo.write(180);
        delay(1000);
        servo_joga_lixo.write(5);
        delay(1000);
      break;

      case 3:
        servo_tampa.write(180);
        delay(1000);
        servo_tampa.write(5);
        delay(1000);
      break;
    }
}



//*********************************************
/*                procura_lixo()
main function
responsável por verificar os sensores de lixo
movimentar os servos quando encontrar algum lixo
movimentar o elevador
verificar o nível de reservatório
sinal para a base se movimentar e parar
***********************************************/
void procura_lixo(){
  bool tem_lixo = verifica_sensor_lixo();
  //se detectar lixo na pá
  if(tem_lixo){
    //ativa o servo que empurra o lixo para dentro da pá
    servos(1);
    //espera um tempo
    delay(400);

    sobe_elevador();
    delay(1000);
    //quando o elevador tiver chegado na parte de cima, abre a tampa
    servos(3);
    delay(200);
    //ativa a plataforma
    servos(2);
    delay(200);
    //desce a plataforma
    desce_elevador();
  }

}

bool verifica_sensor_lixo(){
  int valor_sensor = analogRead(sensor_lixo);
  bool flag_sensor = false;

    if(valor_sensor >= 300){
        flag_sensor = 1;
    }
  return flag_sensor;
  }

//Executa a rotina de movimentar o motor até que a colisão
//com o sensor de fim de curso seja detectada
void sobe_elevador(){
  //pino em HIGH (pull up) até uma colisão
  bool sensor_sup = digitalRead(sensor_elev_sup);

  while(sensor_sup){
    sensor_sup = digitalRead(sensor_elev_sup);
    //aqui eu ativo o pino do terra virtual e
    //o pino que envia o sinal para o transistor
    //que levanta a plataforma
    digitalWrite(enable_elevador, HIGH);
    digitalWrite(pin_elevador_1, HIGH);
    digitalWrite(pin_elevador_2, LOW);
  }

    digitalWrite(enable_elevador, LOW);

}

void desce_elevador(){
    bool sensor_inf = digitalRead(sensor_elev_inf);
    //bool flag_sensor = false;

    while(sensor_inf){
      
      digitalWrite(enable_elevador, HIGH);
      digitalWrite(pin_elevador_2, HIGH);
      digitalWrite(pin_elevador_1, LOW);
      sensor_inf = digitalRead(sensor_elev_inf);
    }
    digitalWrite(enable_elevador, LOW);

}


void setup() {
  servo_pa.attach(10,0,180);
  servo_joga_lixo.attach(11,0,180);
  servo_tampa.attach(9,0,180);
  pinMode(3, OUTPUT);
  pinMode(5, OUTPUT);
  //pinMode(6, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(6, INPUT_PULLUP);
  pinMode(7, INPUT_PULLUP);
  Serial.begin(9600);
}

void loop() {
  Serial.println(analogRead(A0));
  procura_lixo();
}

