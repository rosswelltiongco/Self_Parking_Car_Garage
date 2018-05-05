/*
function vs array - function ok
separate stepper function
what should be in systick -busy waiting ok as long as works
turning? back out while turning/pivot/tank
stdint? - defines aliases for uint
*/

// StepperTestMain.c
// Runs on LM4F120/TM4C123
// Test the functions provided by Stepper.c,
// 
// Before connecting a real stepper motor, remember to put the
// proper amount of delay between each CW() or CCW() step.
// Daniel Valvano
// September 12, 2013
// Modified by Min HE


// PD3 connected to driver for stepper motor coil A
// PD2 connected to driver for stepper motor coil A'
// PD1 connected to driver for stepper motor coil B
// PD0 connected to driver for stepper motor coil B'
#include <stdint.h>
#include "stepper.h"
#include "tm4c123gh6pm.h"

unsigned int enterFlag, exitFlag;
unsigned int obstacleFlag;
unsigned int mode;


void PortB_Init(void);
void PortF_Init(void);
void EnableInterrupts(void);
void WaitForInterrupt(void);



int main(void){
	//unsigned int i=0;
	obstacleFlag = 0;
	//unsigned char mode;
	mode = 0;	  //Mode selector

	PortB_Init();           
  PortF_Init();  
  Stepper_Init();
	EnableInterrupts();                    // The grader uses interrupts

  while(1){
		if (mode == 1){
			moveForward();
		}
		//moveBackward();
		//turnLeft();
		//turnRight();
		
  }
}

//Doorbell (Sw1) interrupt
void GPIOPortF_Handler(void){
	GPIO_PORTF_ICR_R = 0x11;
	//Why doesn't this set count to 0
	(GPIO_PORTF_DATA_R&0x10) ? (enterFlag = 1) : (enterFlag = 0); //If Sw2 pushed
	(GPIO_PORTF_DATA_R&0x01) ? (exitFlag = 1) : (exitFlag = 0); //If Sw2 pushed
	//(mode) ? (mode = 0) : (mode = 1);	
}

//Ultrasonic sensor interrupt
void GPIOPortB_Handler(void){
	GPIO_PORTB_ICR_R = 0x02;
	//Count = 0;
	(GPIO_PORTB_DATA_R&0x02) ? (mode = 0) : (mode = 1);
}

void PortF_Init(void){
	volatile unsigned long delay;
  SYSCTL_RCGC2_R |= 0x00000020;     // 1) F clock
  delay = SYSCTL_RCGC2_R;           // delay   
  GPIO_PORTF_LOCK_R = 0x4C4F434B;   // 2) unlock PortF PF0  
  GPIO_PORTF_CR_R |= 0x1E;           // allow changes to PF4-0       
	GPIO_PORTF_DIR_R &= ~0x11;    // (c) make PF4(sw2), PF0(sw1) inputs (built-in button)
  GPIO_PORTF_DIR_R |=  0x0E;    //  make PF3,PF2,PF1 output+++
  GPIO_PORTF_AFSEL_R &= ~0x11;  //     disable alt funct on PF4
  GPIO_PORTF_DEN_R |= 0x1E;     //     enable digital I/O on PF4 - PF0  
  GPIO_PORTF_PCTL_R &= ~0x000F0000; // configure PF4 as GPIO
  GPIO_PORTF_AMSEL_R = 0;       //     disable analog functionality on PF
  GPIO_PORTF_PUR_R |= 0x11;     //     enable weak pull-up on PF4
	GPIO_PORTF_IS_R &= ~0x11;     // (d) PF4 is edge-sensitive
  GPIO_PORTF_IBE_R &= ~0x11;    //     PF4 is not both edges
  GPIO_PORTF_IEV_R &= ~0x11;    //     PF4 falling edge event
  GPIO_PORTF_ICR_R = 0x11;      // (e) clear flag4
  GPIO_PORTF_IM_R |= 0x11;      // (f) arm interrupt on PF4
  NVIC_PRI7_R |= (NVIC_PRI7_R&0xFF00FFFF)|0x00A00000; // (g) priority 5
  NVIC_EN0_R |= 0x40000000;      // (h) enable interrupt 30 in NVIC
}

void PortB_Init(void){
	volatile unsigned long delay;
  SYSCTL_RCGC2_R |= 0x00000002;     // 1) B clock
  delay = SYSCTL_RCGC2_R;           // delay   
  GPIO_PORTB_CR_R |= 0x02;           // allow changes to PB0       
  GPIO_PORTB_DIR_R &=  ~0x02;    //  make PB0 input
  GPIO_PORTB_AFSEL_R &= ~0x02;  //     disable alt funct on PB0
  GPIO_PORTB_DEN_R |= 0x02;     //     enable digital I/O on PB0 
  GPIO_PORTB_PCTL_R &= ~0x000000F; // configure PB0 as GPIO
  GPIO_PORTB_AMSEL_R = 0;       //     disable analog functionality on PB
  GPIO_PORTB_PUR_R |= 0x02;     //     enable weak pull-up on PB0
	GPIO_PORTB_IS_R &= ~0x02;     // (d) PB0 is edge-sensitive
  GPIO_PORTB_IBE_R |= 0x02;    //     PB0 is both edges
  GPIO_PORTB_ICR_R = 0x02;      // (e) clear PB0
  GPIO_PORTB_IM_R |= 0x02;      // (f) arm interrupt on PB0
  NVIC_PRI0_R |= (NVIC_PRI0_R&0xFFFF00FFF)|0x00000000; // priority 0
  NVIC_EN0_R |= 0x00000002;      // (h) enable interrupt 1 in NVIC
}