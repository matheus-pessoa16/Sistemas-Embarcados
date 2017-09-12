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

volatile uint8_t cont = 0, seg = 1;

volatile uint16_t sensor1, sensor2;

ISR(TIMER2_OVF_vect) {
  cont++;
  if (cont == 61) {
    seg++;
    cont = 0;
  }

  // configura o AD no canal 0
  ADMUX &= 0b11111110;
  set_bit(ADCSRA, 6);
  //ADCSRA & 0b00010000
  while (!(tst_bit(ADCSRA, 4)));
  sensor1 = ADC;
  if (sensor1 == 0) sensor1 = 0;
  OCR1A = sensor1;

  //OCR2A = sensor1;

  // configura o AD no canal 1
  ADMUX |= 0b01000001;
  set_bit(ADCSRA, 6);
  //set_bit(ADMUX, 1);
  while (!(tst_bit(ADCSRA, 4)));
  sensor2 = ADC;
  OCR1B = sensor2;

  


  if (seg <= 10) {
    OCR0A = 0;

    OCR0A = (int)abs((40 * seg) / (100 * sensor1) - (-0.4 * sensor2));
  } else {
    if (seg < 25) {
      OCR0A = 102;
    } else {
      if (seg < 35) {
        OCR0A = (int)(abs((40 * seg) / (100 * sensor1) - (-0.4 * sensor2)));
      } else {
        if (seg < 50) {
          OCR0A = 204;
        } else {
          if (seg < 60) {
            OCR0A = (int)abs((-80 * seg) / (100 * sensor1) - (-0.8) * sensor2);
          } else {
            seg = 1;
          }
        }
      }
    }
  }

}




int main() {
 
  // Configura o PORTD inteiro como saída
  DDRD = 0xFF;

  // Configura os pinos 1, 2 e 3 do PORTB como saídas
  DDRB |= 0b00001110;
  PORTB = 0xFF; // alta impedância nos pinos do PORTB que são entradas e nível alto nos pinos que estão como saída

  // Apenas setando o bit do led do arduino para depois fazê-lo piscar
  set_bit(PORTB, LED);
  clr_bit(PORTB, LED);
  

  // Configurando o conversor A/D
  ADMUX = 0b01000000;
  ADCSRA = 0b10000111; // configurando para a frequência mais baixa para usar os 10 bits de precisão


  // Configurando PWM usando o TIMER0
  TCCR0A = 0b10100011; //PWM não invertido nos pinos OC0A e OC0B
  TCCR0B = 0b00000011; //liga TC0, prescaler = 64
  OCR0A = 0;    //controle do ciclo ativo do PWM 0C0A
  //    OCR0B = 50;    //controle do ciclo ativo do PWM OC0B
  //    TCCR0A = 0b01100011; //TOP = OCR0A, OC0A e OC0B habilitados
  //    TCCR0B = 0b00001001; //liga TC0, prescaler = 1 e ajusta modo para comparação com OCR0A
  //    OCR0A = 100;    //controle da período do sinal no pino OC0A
  //    OCR0B = 10;    //controle do ciclo ativo do PWM 0C0B


  // CONFIGURAÇÃO DO PWM TIMER1
  TCCR1A = 0b10100010;    //PWM não invertido nos pinos OC1A e OC1B
  TCCR1B = 0b00011001;    //liga TC1, prescaler = 1
  ICR1 = 1023;    //valor máximo para contagem
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

  escreve_USART("Valores Sensor1 \n");
  
  while (1) {
    // put your main code here, to run repeatedly:

    // primeiro convertemos número a número os valores recebidos 
    // exemplo: recebi 255, separa em 2 5 e 5 e os envia para a serial
    
    ident_num((unsigned int) sensor1, digitos);
    txByte(digitos[4]);
    txByte(digitos[3]);
    txByte(digitos[2]);
    txByte(digitos[1]);
    txByte(digitos[0]);
    
	// por alguma razão o app android não está recebendo o último dígito. Assim
	// estou enviando novamente o digito[0]. Esse é um bug a ser consertado.	
	txByte(digitos[0]);
    //escreve_USART(digitos, 4);
    
    //txByte('\t');
    
    txByte('\n');
    

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

void escreve_USART(unsigned char c[], int t){
  for(int i=t; i>=0; i--){
    txByte(c[i]);
  }  
}

