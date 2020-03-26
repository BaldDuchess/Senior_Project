#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

bool sync = 0;
uint8_t count = 0;
uint16_t long_pulse = 0;
uint16_t short_pulse = 0;
uint16_t timer_value = 0;

ISR(INT0_vect) 
{        
  count = count + 1;
  //Serial.print("count = ");
  //Serial.println(count);
  timer_value = TCNT1;
  
  if(TCNT1 > 10000)
  {
    count = 1;
    //Serial.println(TCNT1);
    //PORTB = 0xFF;               /*output signal*/
  }
  Serial.println(TCNT1);
  if(count == 9)
  {
    PORTB = 0xFF;
    //Serial.println("on");
    count = 0;
  }else
  {
    PORTB = 0x00;
  }
  
  TCNT1 = 0;
}

void initInterrupt0(void) 
{
  EIMSK |= (1 << INT0);                                 /* enable INT0 */
  EICRA = 0x03;                /* trigger when button changes */
  sei();                          /* set (global) interrupt enable bit */
}

static inline void initFreerunningADC(void) {
  ADMUX = 0x00;                           /* reference voltage on AREF */
  ADCSRA |= (1 << ADPS1) | (1 << ADPS0);    /* ADC clock prescaler /8 */
  ADCSRA |= (1 << ADEN);                                 /* enable ADC */
  ADCSRA |= (1 << ADATE);                       /* auto-trigger enable */
  ADCSRA |= (1 << ADSC);                     /* start first conversion */
}

void InitPortB(void)
{
  DDRB = 0xFF;                          /*change entire port to output*/
  DDRD = 0x00;
}

//NAME: StartTimer
//PURPOSE: initialize timer 1 in normal mode
//PARAMETERS: none
//OUTPUT: none
void StartTimer1()
{
  TCCR1A = 0x0; //put timer in normal mode
  TCCR1B = 0x03; //sys_clk/64 prescaler
  TCCR1B |= (1 << CS10); //no prescaler
}

int main(void) 
{
  // -------- Inits --------- //
  initInterrupt0();
  initFreerunningADC();
  InitPortB();
  StartTimer1();
  Serial.begin(9600);

  long_pulse = 10000; //starting value
  // ------ Event loop ------ //
  while (1) 
  {
    if(timer_value 
  }                                                
  return 0;                            /* This line is never reached */
}
