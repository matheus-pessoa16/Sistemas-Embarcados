/*
	TO DO

  Curva de secagem não está seguindo o perfil correto
  Comunicação serial com o bluetooth ou com aplicação desk (funcionando no android via bluetooth)
  Botão para iniciar o processo de secagem (usar como interrupção externa ou dentro do loop mesmo? Provável que INT)


*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

// definição das macros
#define set_bit(y, bit_x) (y |= (1 << bit_x))
#define clr_bit(y, bit_x) (y &= ~(1 << bit_x))
#define cpl_bit(y, bit_x) (y ^= (1 << bit_x))
#define tst_bit(y, bit_x) (y & (1 << bit_x))

#define LED PB5
#define conv_ascii 48
#define tam_vetor 5

volatile uint8_t cont = 0, seg = 1,  tmp1, tmp2, estado = 0, aux = 0;
volatile float norm_temp;

volatile uint16_t sensor1, sensor2;


uint16_t readSensor(uint8_t sensor) {

  // configura o AD no canal 0
  ADMUX &= 0b01000000;

  if (sensor != 0) {
    ADMUX |= 0b00000001;
  }

  ADCSRA |= 0b01000000;
  while (!(ADCSRA & 0b00010000));

  return ADC;

}


ISR(TIMER2_OVF_vect) {

  sensor1 = readSensor(0);
  if (sensor1 == 0) sensor1 = 1;
  OCR1A = (sensor1 >> 2);
  sensor2 = readSensor(1);
  if (sensor2 == 0) sensor2 = 1;
  OCR1B = (sensor2 >> 2);
  norm_temp = (sensor2 / (1023.0))+1;
  // variáveis para armazenar valor do pwm da curva de secagem quando houver mudanca

  if ( !tst_bit(PINB, 0) && estado == 0) {
    estado = 1;
    set_bit(PORTB, LED);
  }
  ////
  ////    if (estado == 0 && aux == 0) {
  //      estado = 1;
  ////      aux = 1;
  //      set_bit(PORTB, LED);
  ////    }

  if (estado) {
    cont++;
    if (cont == 61) {
      seg++;
      cont = 0;
    }
    if (seg <= 10) {
//      OCR0A = 0;
      OCR0A = (int) (2.55 * ( ((4 * seg) + (2.8 / sensor1 )) * (1.2 / norm_temp)));
      tmp1 = OCR0A;
      OCR0B = OCR0A;
    } else {
      if (seg < 25) {
        OCR0A = (int) (tmp1 + (2.8 / sensor1)) * (1.2 / norm_temp);
        tmp2 =  OCR0A;
        OCR0B = OCR0A;
      } else {
        if (seg < 35) { // Funcao de secagem: PWM = (P(t) + a/sensor1) * b/sensor2
          OCR0A = (int) ((tmp2 + ( (4 * (seg - 25)) + (2.8 / sensor1)) * (1.2 / norm_temp)));// 3.3 valor de a 1.2 valor de b
          tmp1 = OCR0A;
          OCR0B = OCR0A;
        } else {
          if (seg < 50) {
            OCR0A = (int) (tmp1 + (2.8 / sensor1)) * (1.2 / norm_temp);
            tmp2 = OCR0A;
            OCR0B = OCR0A;
          } else {
            if (seg < 60) {
              OCR0A = (int) ((tmp2 - ( (8 * (seg - 50)) + (2.8 / sensor1)) * (1.2 / norm_temp) ));
              OCR0B = OCR0A;
            } else {
              seg = 1;
              OCR0A = 0;
              OCR0B = 0;
              //aux = 0;
              estado = 0;
              clr_bit(PORTB, LED);
            }
          }
        }
      }
    }
  }

  //}



}

int main() {

  // Configura o PORTD inteiro como saída
  DDRD = 0xFF;

  // Configura os pinos 1, 2 e 3 do PORTB como saídas
  DDRB  = 0b00101110;
  PORTB |= 0b00001111; // alta impedância nos pinos do PORTB que são entradas e nível alto nos pinos que estão como saída
  //PORTB &= 0b11111111;
  // Configurando o conversor A/D
  ADMUX = 0b01000000;
  ADCSRA = 0b10000111; // configurando para a frequência mais baixa para usar os 10 bits de precisão


  // Configurando PWM usando o TIMER0
  TCCR0A = 0b10100011; //PWM não invertido nos pinos OC0A e OC0B
  TCCR0B = 0b00000011; //liga TC0, prescaler = 64
  OCR0A = 0;    //controle do ciclo ativo do PWM 0C0A
  OCR0B = 0;    //controle do ciclo ativo do PWM OC0B
  //    TCCR0A = 0b01100011; //TOP = OCR0A, OC0A e OC0B habilitados
  //    TCCR0B = 0b00001001; //liga TC0, prescaler = 1 e ajusta modo para comparação com OCR0A
  //    OCR0A = 100;    //controle da período do sinal no pino OC0A
  //    OCR0B = 10;    //controle do ciclo ativo do PWM 0C0B


  // CONFIGURAÇÃO DO PWM TIMER1
  TCCR1A = 0b10100010;    //PWM não invertido nos pinos OC1A e OC1B
  TCCR1B = 0b00011001;    //liga TC1, prescaler = 1
  ICR1 = 255;    //valor máximo para contagem
  //OCR1A = 2000;    //controle do ciclo ativo do PWM 0C1A
  //OCR1B = 100;



  // CONFIGURANDO A INTERRUPÇÃO DO TIMER2
  cli(); // desliga interrupções globais
  TCCR2B = 0b00000111; // TC2 com prescaler de 1024. Ti=16.384 ms
  TIMSK2 = 0b00000001; // interrupção do TC2 habilitada
  sei(); // ativa as interrupções globais


  // Inicializa a comunicação serial
  initUSART();

  // vetor que contém os valores da serial convertidos
  uint8_t digitos[tam_vetor];

  //uint8_t palavra[10];


  escreve_USART("Valores Sensor1 \n");
  //uint8_t cont=0;

  while (1) {

    if (estado) {
      set_bit(PORTB, LED);
    } else {
      clr_bit(PORTB, LED);
    }


    // put your main code here, to run repeatedly:

    // primeiro convertemos número a número os valores recebidos
    // exemplo: recebi 255, separa em 2 5 e 5 e os envia para a serial

    ident_num((unsigned int) sensor1, digitos);
    txByte(digitos[3]);
    txByte(digitos[2]);
    txByte(digitos[1]);
    txByte(digitos[0]);
    digitos[4] = ';';
    txByte(digitos[4]);
    //
    ident_num((unsigned int) sensor2, digitos);
    txByte(digitos[3]);
    txByte(digitos[2]);
    txByte(digitos[1]);
    txByte(digitos[0]);
    digitos[4] = ';';
    txByte(digitos[4]);

    ident_num((unsigned int) OCR0A, digitos);
    //txByte(digitos[3]);
    txByte(digitos[2]);
    txByte(digitos[1]);
    txByte(digitos[0]);
    //digitos[4] = ';';
    txByte(digitos[4]);



    //  	// por alguma razão o app android não está recebendo o último dígito. Assim
    //  	// estou enviando novamente o digito[0]. Esse é um bug a ser consertado.
    //txByte(digitos[0]);
    //    //txByte('\t');
    txByte('\n');

    //    while(rxByte() != '\n'){
    //      palavra[cont] += rxByte();
    //      cont++;
    //    }
    //
    //    if(palavra == "ldr"){
    //      set_bit(PORTB, LED);
    //    }else{
    //      clr_bit(PORTB, LED);
    //    }
    //    cont=0;

  }

}

void initUSART()
{
  // Configurado para 9600bps
  UBRR0L = 103;
  UBRR0H = 0;

  // U2X=1 - Dobra a velocidade
  //UCSRA = (1<<U2X);

  // UCSZ2 = 0 - 8 bits
  UCSR0B |= _BV(RXEN0) | _BV(TXEN0);


  // UCSZ1 = 1 e UCSZ0 = 1 -> 8 bits
  // USBS0 = 0 -> 1 - stop bits
  // UPM0 = 0 e UMP0 = 0 -> sem bit de paridade
  UCSR0C |= _BV(UCSZ01) | _BV(UCSZ00);
}

void txByte(uint8_t info)
{
  // Transmissгo de dados
  //Bit UDRE indica quando o buffer de tx pode enviar dados
  while (!(UCSR0A & (1 << UDRE0)));
  UDR0 = info;

}

// Realiza a leitura do buffer da serial e disponibiliza os dados
// no registrador UDR0
uint8_t rxByte()
{
  //Bit RXC sinaliza quando existem bytes nгo lidos no buffer
  while (!(UCSR0A & (1 << RXC0)));
  return UDR0;
}

uint8_t transceiver(uint8_t *data) {
  //txByte(data);
  escreve_USART(data);
  return rxByte();
}

//Conversão de um número em seus digitos individuais.
//O papel dessa função é poder exibir corretamente os valores na porta serial, já
// que algun serão maiores que 255 e alguns bugs estranhos acontecem mesmo com valores
// menores;
//------------------------------------------------------------------------------------
void ident_num(unsigned int valor, unsigned char *disp)
{
  unsigned char n;
  for (n = 0; n < tam_vetor; n++)
    disp[n] = 0 + conv_ascii;
  do
  {
    *disp = (valor % 10) + conv_ascii;
    valor /= 10;
    disp++;
    //limpa vetor para armazenagem dos digitos
    //pega o resto da divisão por 10
    //pega o inteiro da divisão por 10
  } while (valor != 0);
}
//-----------------------------------------------------------------------------------

//Usamos essa função apenas para percorrer um vetor de char (string)
//e escrevê-la na saída serial.
void escreve_USART(char *c)
//escreve String
{
  for (; *c != 0; c++) txByte(*c);
}

void escreve_USART(unsigned char c[], int t) {
  for (int i = t; i >= 0; i--) {
    txByte(c[i]);
  }
}

