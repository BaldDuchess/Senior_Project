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
volatile uint32_t FallingEdges = 0;

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
  GPIO_PORTF_ICR_R = 0x10;      // acknowledge flag4
  FallingEdges = FallingEdges + 1;  //increment counter
  GPIO_PORTF_DATA_R = 2;      /* turn on red LED */

  //initialize timer0
  TIMER0_TAMATCHR_R = 0x00;           //set interrupt compare to 0
  TIMER0_TAILR_R = 0x000FFFFF; //Timer A interval load value register
  TIMER0_CTL_R |= 0x01;           //enable Timer A after initialization
}

void Timer0A_Handler(void)
{
    TIMER0_ICR_R = 0x11;        //acknowledge flag
    GPIO_PORTF_DATA_R = 0;      //turn off red LED
}
void GPIO_Init(void)
{
    SYSCTL_RCGC2_R |= 0x00000020;   //enable clock to GPIOF at clock gating control register
    GPIO_PORTF_DIR_R |= 0x0E;        //enable the GPIO pins for the LED (PF3, 2 1) as output
    GPIO_PORTF_DEN_R |= 0x0E;        //enable the GPIO pins for digital function
}

void Timer_Init(void)
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

//debug code
int main(void){
  Timer_Init();
  PortF_Interrupt_Init();
  GPIO_Init();
  
  while(1){

  }
}



















































