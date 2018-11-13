/*
	O sistema possui um botão para iniciar o processo. Quando o sistema está ligado esse
	botao fica desabilitado.
	Os sensores são lidos na interrupção e o valor é mostrado em dois LEDs através de um pwm.
	A curva de secagem é calculada baseada nos sinais vindos dos sensores junto com o tempo.
	Uso de interrupção por estouro de timer.
  	Comunicação serial com o bluetooth ou com aplicação desk (funcionando no android via bluetooth)
  
*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

// definição das macros
#define set_bit(y, bit_x) (y |= (1 << bit_x)) // colocar o valor do pino em 1
#define clr_bit(y, bit_x) (y &= ~(1 << bit_x)) // colocar o valor do pino em 0
#define cpl_bit(y, bit_x) (y ^= (1 << bit_x)) // trocar estado lógico de um pino
#define tst_bit(y, bit_x) (y & (1 << bit_x)) // lê o valor de um pino

#define LED PB5 // setando LED como o pino 5 do PORTD
#define conv_ascii 48 // usado para converter em ascii os caracteres recebidos na comunicação serial
#define tam_vetor 5 // constante com o tamanho para vetores

// modificador volatile é exigido pelo compilador para
// variáveis usadas dentro de interrupções.
volatile uint8_t cont = 0, seg = 1; // variáveis de controle de tempo

volatile uint16_t sensor1, sensor2; // variáveis dos sensores. Podemos economizar memória usando inteiros de 16 bits para
				    // armazenar os valores.

/************ CONFIGURANDO O CANAL AD A SER LIDO ***********/

uint16_t readSensor(uint8_t sensor){
  
  // configura o AD no canal 0 mantendo AVCC como referência
  ADMUX &= 0b01000000;

  if(sensor != 0){
    ADMUX |= 0b00000001; // se for diferente do canal zero, define como canal 1
  }
  
  ADCSRA |= 0b01000000; // inicie a conversão
  while (!(ADCSRA & 0b00010000)); // espere a conversão ser finalizada (ADIF = 1)

  return ADC;

}
/********************* FINALIZADA A CONFIGURAÇÃO DO AD *********************/


/********************* CONFIGURANDO A INTERRUPÇÃO *********************/
// interrupção por estouro do timer2 
ISR(TIMER2_OVF_vect) { 
  
  sensor1 = readSensor(0); // leia o sensor no canal 0
  OCR1A = (sensor1>>2); // divide por 4 o valor lido (0 - 1023) para ficar dentro do range do pwm (0 - 255)
  sensor2 = readSensor(1); // leia o sensor no canal 1
  OCR1B = (sensor2>>2); // divide por 4 o valor lido (0 - 1023) para ficar dentro do range do pwm (0 - 255)
  
  cont++; // a cada 16.384 ms, conta 1 para controlar o tempo.
  if (cont == 61) {
    seg++; // 61 estouros do timer é aproximadamente 1 segundo.
    cont = 0;
  }
	
  // início do cálculo da curva.
  if (seg <= 10) { 
    OCR0A = 0;
    OCR0A = (int)(((40 * seg) / (100 * sensor1)) - (-0.4 * sensor2));
  } else {
    if (seg < 25) {
      OCR0A = 102;
    } else {
      if (seg < 35) {
        OCR0A = (int)(((40 * seg) / (100 * sensor1)) - (-0.4 * sensor2));
      } else {
        if (seg < 50) {
          OCR0A = 204;
        } else {
          if (seg < 60) {
            OCR0A = (int)(((-80 * seg) / (100 * sensor1)) - (-0.8) * sensor2);
          } else {
            seg = 1;
            OCR0A = 0;
            
          }
        }
      }
    }
  }

}

/********************* FIM DA INTERRUPÇÃO *********************/


int main() {

  // Configura o PORTD inteiro como saída
  DDRD = 0b11111111;
  
  // Configura os pinos 1, 2 e 3 do PORTB como saídas
  DDRB |= 0b00001110;
  PORTB |= 0xFF; // alta impedância nos pinos do PORTB que são entradas e nível alto nos pinos que estão como saída

  // Configurando o conversor A/D
  ADMUX = 0b01000000; // referência de tensão: AVCC
  ADCSRA = 0b10000111; // configurando para a frequência mais baixa para usar os 10 bits de precisão e habilitando o adc.

  // Configurando PWM usando o TIMER0
  TCCR0A = 0b10100011; //PWM não invertido nos pinos OC0A e OC0B
  TCCR0B = 0b00000011; //liga TC0, prescaler = 64
  OCR0A = 0;    //controle do ciclo ativo do PWM 0C0A

  // CONFIGURAÇÃO DO PWM TIMER1
  TCCR1A = 0b10100010;    //PWM não invertido nos pinos OC1A e OC1B
  TCCR1B = 0b00011001;    //liga TC1, prescaler = 1
  ICR1 = 255;    //valor máximo para contagem

  // CONFIGURANDO A INTERRUPÇÃO DO TIMER2
  cli(); // desliga interrupções globais
  TCCR2B = 0b00000111; // TC2 com prescaler de 1024. Ti=16.384 ms
  TIMSK2 = 0b00000001; // interrupção do TC2 habilitada
  sei(); // ativa as interrupções globais
  
  // Inicializa a comunicação serial
  initUSART();

  // vetor que contém os valores lidos da serial convertidos para ascii
  uint8_t digitos[tam_vetor];
  
  while (1) {
	  
    // primeiro convertemos número a número os valores recebidos 
    // exemplo: recebi 255, separa em 2 5 e 5 e os envia para a serial
    ident_num((unsigned int) sensor1, digitos);
    txByte(digitos[3]);
    txByte(digitos[2]);
    txByte(digitos[1]);
    txByte(digitos[0]);
    digitos[4] = ';';
    txByte(digitos[4]);

    ident_num((unsigned int) sensor2, digitos);
    txByte(digitos[3]);
    txByte(digitos[2]);
    txByte(digitos[1]);
    txByte(digitos[0]);
    digitos[4] = ';';
    txByte(digitos[4]);

    ident_num((unsigned int) OCR0A, digitos);
    txByte(digitos[3]);
    txByte(digitos[2]);
    txByte(digitos[1]);
    txByte(digitos[0]);
    //digitos[4] = ';';
    txByte(digitos[4]);
	  
// por alguma razão o app android não está recebendo o último dígito. Assim
// estou enviando novamente o digito[0]. Esse é um bug a ser consertado.	

    txByte('\n'); // final da comunicação
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
  UCSR0B |= _BV(RXEN0) | _BV(TXEN0); // habilita o transmissor e o receptor serial


  // UCSZ1 = 1 e UCSZ0 = 1 -> 8 bits
  // USBS0 = 0 -> 1 - stop bits
  // UPM0 = 0 e UMP0 = 0 -> sem bit de paridade
  UCSR0C |= _BV(UCSZ01) | _BV(UCSZ00); // frame de 8 bits
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

uint8_t transceiver(uint8_t *data){
  //txByte(data);
  escreve_USART(data);
  return rxByte();
}

//Conversão de um número em seus digitos individuais.
//O papel dessa função é poder exibir corretamente os valores na porta serial, já
// que alguns serão maiores que 255 e alguns bugs estranhos acontecem mesmo com valores
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
    //limpa vetor para armazenamento dos digitos
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

void escreve_USART(unsigned char c[], int t){
  for(int i=t; i>=0; i--){
    txByte(c[i]);
  }  
}
