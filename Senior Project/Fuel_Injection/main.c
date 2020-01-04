#include <stdint.h>
#include "tm4c123gh6pm.h"


//defines
//PWM


//void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
long StartCritical (void);    // previous I bit, disable interrupts
void EndCritical(long sr);    // restore I bit to previous value
void WaitForInterrupt(void);  // low power mode

// global variable visible in Watch window of debugger
// increments at least once per button press
volatile uint64_t FallingEdges = 0;
volatile uint32_t crank_period_capture = 0;
volatile uint16_t crank_period = 0;
volatile uint32_t half_crank_period = 0;
char TDC = 1;
volatile uint32_t advance = 0;

//interrupts
void PortF_Interrupt_Init(void){
  SYSCTL_RCGCGPIO_R |= 0x00000020; //activate clock for port F
  FallingEdges = 0;             //initialize counter
  GPIO_PORTF_DIR_R &= ~0x10;    //make PF4 in (built-in button)
  GPIO_PORTF_AFSEL_R &= ~0x10;  //disable alt funct on PF4
  GPIO_PORTF_DEN_R |= 0x10;     //enable digital I/O on PF4
  GPIO_PORTF_PCTL_R &= ~0x000F0000; //configure PF4 as GPIO
  GPIO_PORTF_AMSEL_R = 0;       //disable analog functionality on PF
  GPIO_PORTF_PUR_R |= 0x10;     //enable weak pull-up on PF4
  GPIO_PORTF_IS_R &= ~0x10;     //PF4 is edge-sensitive
  GPIO_PORTF_IBE_R &= ~0x10;    //PF4 is not both edges
  GPIO_PORTF_IEV_R = 0x01;    //PF4 rising edge event
  GPIO_PORTF_ICR_R = 0x10;      //clear flag4
  GPIO_PORTF_IM_R |= 0x10;      //arm interrupt on PF4 *** No IME bit as mentioned in Book ***
  NVIC_PRI7_R = (NVIC_PRI7_R&0xFF00FFFF)|0x00A00000; //priority 5
  NVIC_EN0_R = 0x40000000;      //enable interrupt 30 in NVIC
  EnableInterrupts();           //Clears the I bit
}
void GPIOPortF_Handler(void){

        FallingEdges = 0;
  TDC = 1;   //if this interrupt has fired piston #1 is coming up on TDC

  GPIO_PORTF_ICR_R = 0x10;      // acknowledge flag4

  //re-start timer2 - determines crank period (RPMs)
  crank_period_capture = TIMER2_TAV_R;  //capture timer2's value
  crank_period = (crank_period_capture/16); //find crank rotation period in microseconds
  half_crank_period = crank_period_capture/2;
  TIMER2_TAV_R = 0;   //reset timer 2
  TIMER2_CTL_R |= 0x01;           //enable Timer A after initialization

  //re-start timer3 - controls spark signal for BDC event
  TIMER3_TAMATCHR_R = 0x00;           //set interrupt compare to 0
  TIMER3_TAILR_R = advance; //Timer A interval load value register
  TIMER3_CTL_R |= 0x01;           //enable Timer A after initialization

}

void Timer0A_Handler(void)
{
    TIMER0_ICR_R = 0x11;        //acknowledge flag
    GPIO_PORTF_DATA_R = 0;      //turn off red LED
}
void Timer3A_Handler(void)
{
    if(TDC)
    {
        GPIO_PORTF_DATA_R = 0x02;      /* turn on red LED */

        TIMER3_ICR_R = 0x11;        //acknowledge flag

        //re-start timer0 - controls duration of spark signal
        TIMER0_TAMATCHR_R = 0x00;           //set interrupt compare to 0
        TIMER0_TAILR_R = 0x0000FFFF; //Timer A interval load value register
        TIMER0_CTL_R |= 0x01;           //enable Timer A after initialization
        TDC = 0;    //signal that the next event for time 3 will be BDC

        //re-start timer3 - controls spark signal for BDC event
        TIMER3_TAMATCHR_R = 0x00;           //set interrupt compare to 0
        TIMER3_TAILR_R = half_crank_period + advance; //Timer A interval load value register
        TIMER3_CTL_R |= 0x01;           //enable Timer A after initialization0
    }else
    {
        TIMER3_ICR_R = 0x11;        //acknowledge flag
        GPIO_PORTF_DATA_R = 0x04;      /* turn on blue LED */
        //re-start timer0 - controls duration of spark signal
        TIMER0_TAMATCHR_R = 0x00;           //set interrupt compare to 0
        TIMER0_TAILR_R = 0x0000FFFF; //Timer A interval load value register
        TIMER0_CTL_R |= 0x01;           //enable Timer A after initialization
    }
}
void GPIO_Init(void)
{
    SYSCTL_RCGC2_R |= 0x00000020;   //enable clock to GPIOF at clock gating control register
    GPIO_PORTF_DIR_R |= 0x0E;        //enable the GPIO pins for the LED (PF3, 2 1) as output
    GPIO_PORTF_DEN_R |= 0x0E;        //enable the GPIO pins for digital function
}
void Timer0_Init(void)
{
    SYSCTL_RCGC1_R |= SYSCTL_RCGC1_TIMER0; //activate timer0
    TIMER0_CTL_R &= ~0x00000001;     //disable timer0A during setup
    TIMER0_CFG_R = 0x00;            //32-bit option
    TIMER0_TAMR_R = 0x01;           //one-shot mode and down-counter
    TIMER0_TAMR_R |= 0x20;          //enable interrupts for one-shot mode
    TIMER0_IMR_R |= 0x1F;           // enable compare match interrupt
    NVIC_PRI4_R = (NVIC_PRI4_R&0x00FFFFFF)|0x40000000; // 8) priority 2
    NVIC_EN0_R = 0x80000;      //enable interrupt 19 in NVIC
}
void Timer2_Init(void)
{
    SYSCTL_RCGC1_R |= SYSCTL_RCGC1_TIMER2; //enable clock for timer2
    TIMER2_CTL_R &= ~0x00000001;     //disable timer2A during setup
    TIMER2_CFG_R = 0x00;            //32-bit option
    TIMER2_TAMR_R = 0x12;           //periodic mode and down-counter
    //TIMER2_TAPR_R |= 0x04;          //divide timer for sys_clk/2
}
void Timer3_Init(void)
{
    SYSCTL_RCGC1_R |= SYSCTL_RCGC1_TIMER3; //activate timer0
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
  Timer2_Init();
  Timer3_Init();
  PortF_Interrupt_Init();
  GPIO_Init();
  while(1){
  }
}
