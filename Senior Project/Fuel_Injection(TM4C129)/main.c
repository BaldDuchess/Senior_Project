#include <stdint.h>
#include "TM4C129.h"


//defines


//void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
long StartCritical (void);    // previous I bit, disable interrupts
void EndCritical(long sr);    // restore I bit to previous value
void WaitForInterrupt(void);  // low power mode

// global variable visible in Watch window of debugger
// increments at least once per button press
volatile uint32_t crank_period_capture = 0;
volatile uint16_t crank_period = 0;
volatile uint32_t half_crank_period = 0;
char TDC = 1;
volatile uint32_t advance = 0;
volatile uint16_t spark_duration = 0x1;
volatile uint16_t fuel_duration = 0x1;

static uint16_t front_cyl_bank_spark = 0x01;
static uint16_t rear_cyl_bank_spark = 0x02;
static uint16_t cyl_1_fuel = 0x04;
static uint16_t cyl_2_fuel = 0x08;
static uint16_t cyl_3_fuel = 0x10;
static uint16_t cyl_4_fuel = 0x20;

volatile uint8_t cyl_to_fuel_front = 1;
volatile uint8_t cyl_to_fuel_rear = 2;


//interrupts
void PortM_Interrupt_Init(void){
  SYSCTL_RCGCGPIO |= 0x00000020; //activate clock for port F
  GPIO_PORTM_DIR_R &= ~0x01;    //make PF0 in (built-in button)
  GPIO_PORTM_AFSEL_R &= ~0x01;  //disable alt funct on PF4
  GPIO_PORTM_DEN_R |= 0x01;     //enable digital I/O on PF4
  GPIO_PORTM_PCTL_R &= ~0x0F; //configure PF4 as GPIO
  GPIO_PORTM_AMSEL_R = 0;       //disable analog functionality on PF
  GPIO_PORTM_PUR_R |= 0x01;     //enable weak pull-up on PF4
  GPIO_PORTM_IS_R &= ~0x01;     //PF4 is edge-sensitive
  GPIO_PORTM_IBE_R &= ~0x01;    //PF4 is not both edges
  GPIO_PORTM_IEV_R = 0x01;    //PF4 rising edge event
  GPIO_PORTM_ICR_R = 0x01;      //clear flag4
  GPIO_PORTM_IM_R |= 0x01;      //arm interrupt on PF4 *** No IME bit as mentioned in Book ***
  NVIC_PRI7_R = (NVIC_PRI7_R&0xFF00FFFF)|0x00A00000; //priority 5
  NVIC_EN0_R = 0x40000000;      //enable interrupt 30 in NVIC
  EnableInterrupts();           //Clears the I bit
}

//Purpose: calculates RPMs and starts the timer that starts the fuel and spark pulses
//Parameters: none
//Returns: none
void GPIOPortF_Handler(void){

  TDC = 1;   //if this interrupt has fired piston #1 is coming up on TDC

  GPIO_PORTF_ICR_R = 0x10;      // acknowledge flag4

  //re-start timer2 - determines crank period (RPMs)
  crank_period_capture = TIMER2_TAV_R;  //capture timer2's value
  crank_period = (crank_period_capture/16); //find crank rotation period in microseconds
  half_crank_period = crank_period_capture/2;
  TIMER2_TAV_R = 0;   //reset timer 2
  TIMER2_CTL_R |= 0x01;           //enable Timer A after initialization

  //re-start timer3 - controls signal for BDC event
  TIMER3_TAMATCHR_R = 0x00;           //set interrupt compare to 0
  TIMER3_TAILR_R = advance; //Timer A interval load value register
  TIMER3_CTL_R |= 0x01;           //enable Timer A after initialization

}

//Purpose: interrupt that turns off the spark pulse.
//Parameters: none
//Returns: none
void Timer0A_Handler(void)
{
    TIMER0_ICR_R = 0x11;        //acknowledge flag
    GPIO_PORTB_DATA_R &= ~0x3;      //shut off spark
}
//Purpose: interrupt that turns off the fuel pulse.
//Parameters: none
//Returns: none
void Timer1A_Handler(void)
{
    TIMER1_ICR_R = 0x11;        //acknowledge flag

    GPIO_PORTB_DATA_R &= ~0xFC;      //shut off fuel
}

//Purpose: starts fuel and spark pulses at the appropriate advance times, also re-arms itself for the BDC timing
//Parameters: none
//Returns: none
void Timer3A_Handler(void)
{


    if(TDC)
    {
        TIMER3_ICR_R = 0x11;        //acknowledge flag
        GPIO_PORTB_DATA_R |= front_cyl_bank_spark;      //start the spark

        switch(cyl_to_fuel_front)
        {
        case    1:
            GPIO_PORTB_DATA_R |= cyl_1_fuel;
            cyl_to_fuel_front = 3;
            break;
        case    3:
            GPIO_PORTB_DATA_R |= cyl_3_fuel;
            cyl_to_fuel_front = 1;
            break;
        default:
            GPIO_PORTF_DATA_R |= 0x0;   //do nothing
        }

        //re-start timer0 - controls duration of spark signal
        TIMER0_TAMATCHR_R = 0x00;           //set interrupt compare to 0
        TIMER0_TAILR_R = spark_duration; //Timer A interval load value register
        TIMER0_CTL_R |= 0x01;           //enable Timer A after initialization
        TDC = 0;    //signal that the next event for time 3

        //restart timer1 - controls duration for fuel signal
        TIMER1_TAMATCHR_R = 0x00;           //set interrupt compare to 0
        TIMER1_TAILR_R = fuel_duration; //Timer A interval load value register
        TIMER1_CTL_R |= 0x01;           //enable Timer A after initialization

        //re-start timer3 - controls spark signal for BDC event
        TIMER3_TAMATCHR_R = 0x00;           //set interrupt compare to 0
        TIMER3_TAILR_R = half_crank_period + advance; //Timer A interval load value register
        TIMER3_CTL_R |= 0x01;           //enable Timer A after initialization0
    }else
    {
        TIMER3_ICR_R = 0x11;        //acknowledge flag
        GPIO_PORTB_DATA_R |= rear_cyl_bank_spark;      //start the spark

        switch(cyl_to_fuel_rear)
        {
        case    2:
            GPIO_PORTB_DATA_R |= cyl_2_fuel;
            cyl_to_fuel_rear = 4;
            break;
        case    4:
            GPIO_PORTB_DATA_R |= cyl_4_fuel;
            cyl_to_fuel_rear = 2;
            break;
        default:
            GPIO_PORTF_DATA_R |= 0x0;   //do nothing
        }

        //re-start timer0 - controls duration of spark signal
        TIMER0_TAMATCHR_R = 0x00;           //set interrupt compare to 0
        TIMER0_TAILR_R = spark_duration; //Timer A interval load value register
        TIMER0_CTL_R |= 0x01;           //enable Timer A after initialization

        //restart timer1 - controls duration for fuel signal
        TIMER1_TAMATCHR_R = 0x00;           //set interrupt compare to 0
        TIMER1_TAILR_R = fuel_duration; //Timer A interval load value register
        TIMER1_CTL_R |= 0x01;           //enable Timer A after initialization
    }
}
void GPIO_Init(void)
{
    SYSCTL_RCGCGPIO |= 0x00001000;   //enable clock to GPIO N at clock gating control register
    GPIO_PORTM_DIR_R |= 0xEF;        //enable the GPIO pins on port M: PM0 is input for crank signal, all others are outputs for fuel or spark pulses
    GPIO_PORTM_AFSEL_R &= ~0XFF;    //disable alternate port functions
    GPIO_PORTM_DEN_R |= 0xFF;        //enable the GPIO pins for digital function

    //PortN is used for sending spark and fuel pulses, Pn4 and Pn5 are for spark
    SYSCTL_RCGCGPIO |= 0x0000800;   //enable clock to GPIO N, this will control spark and fuel pulses
    GPIO_PORTN_DIR_R |= 0xFF;        //enable the GPIO pins on port N as output
    GPIO_PORTN_AFSEL_R &= ~0XFF;    //disable alternate port functions
    GPIO_PORTN_DEN_R |= 0xFF;        //enable the GPIO pins for digital function

}
void Timer0_Init(void)
{
    SYSCTL_RCGCTIMER_R |= 0x01; //activate timer0
    TIMER0_CTL_R &= ~0x00000001;     //disable timer0A during setup
    TIMER0_CFG_R = 0x00;            //32-bit option
    TIMER0_TAMR_R = 0x01;           //one-shot mode and down-counter
    TIMER0_TAMR_R |= 0x20;          //enable interrupts for one-shot modeS
    TIMER0_IMR_R |= 0x1F;           // enable compare match interrupt
    NVIC_PRI4_R = (NVIC_PRI4_R&0x00FFFFFF)|0x40000000; // 8) priority 2
    NVIC_EN0_R = 0x80000;      //enable interrupt 19 in NVIC
}

void Timer1_Init(void)
{
    SYSCTL_RCGCTIMER_R |= 0x02; //activate timer1
    TIMER1_CTL_R &= ~0x00000001;     //disable timer3A during setup
    TIMER1_CFG_R = 0x00;            //32-bit option
    TIMER1_TAMR_R = 0x01;           //one-shot mode and down-counter
    TIMER1_TAMR_R |= 0x20;          //enable interrupts for one-shot mode
    TIMER1_IMR_R |= 0x1F;           // enable compare match interrupt
    NVIC_PRI5_R = (NVIC_PRI5_R&0x1FFF)|0x0008000; // 8) priority 2
    NVIC_EN0_R = 0x200000;      //enable interrupt
}

void Timer2_Init(void)
{
    SYSCTL_RCGCTIMER_R |= 0x04; //enable clock for timer2
    TIMER2_CTL_R &= ~0x00000001;     //disable timer2A during setup
    TIMER2_CFG_R = 0x00;            //32-bit option
    TIMER2_TAMR_R = 0x12;           //periodic mode and down-counter
    //TIMER2_TAPR_R |= 0x04;          //divide timer for sys_clk/2  21 20  19 18 17 16  15 14 13 12  11 10 9 8  7 6 5 4  3 2 1 0
}
void Timer3_Init(void)
{
    SYSCTL_RCGCTIMER_R |= 0x08; //activate timer3
    TIMER3_CTL_R &= ~0x00000001;     //disable timer0A during setup
    TIMER3_CFG_R = 0x00;            //32-bit option
    TIMER3_TAMR_R = 0x01;           //one-shot mode and down-counter

    TIMER3_TAMR_R |= 0x20;          //enable interrupts for one-shot mode
    TIMER3_IMR_R |= 0x1F;           // enable compare match interrupt
    NVIC_PRI8_R = (NVIC_PRI8_R&0x1FFFFFFF)|0x40000000; // 8) priority 2
    NVIC_EN1_R = 0x08;      //enable interrupt 35 in NVIC
}
//debug code
int main(void){
  Timer0_Init();
  Timer1_Init();
  Timer2_Init();
  Timer3_Init();
  PortM_Interrupt_Init();
  GPIO_Init();
  while(1){

          spark_duration = 0x1;
          fuel_duration = crank_period_capture/800;


  }
}
