// This is your first program to run on the LaunchPad
// You will run this program without modification as your Lab 2
// If the left switch SW1 is 
//      not pressed the LED toggles blue-red
//      pressed the LED toggles blue-green

// 0.Documentation Section 
// main.c
// Runs on LM4F120 or TM4C123
// Lab2_HelloLaunchPad, Input from PF4, output to PF3,PF2,PF1 (LED)
// Authors: Daniel Valvano, Jonathan Valvano and Ramesh Yerraballi
// Date: January 15, 2016

// LaunchPad built-in hardware
// SW1 left switch is negative logic PF4 on the Launchpad
// SW2 right switch is negative logic PF0 on the Launchpad
// red LED connected to PF1 on the Launchpad
// blue LED connected to PF2 on the Launchpad
// green LED connected to PF3 on the Launchpad

// 1. Pre-processor Directives Section
// Constant declarations to access port registers using 
// symbolic names instead of addresses

///////////////////////////////////////
//  Register definitions
///////////////////////////////////////
#include "TExaS.h"

#define NVIC_EN0_R              (*((volatile unsigned long *)0xE000E100))  // IRQ 0 to 31 Set Enable Register
#define NVIC_PRI7_R             (*((volatile unsigned long *)0xE000E41C))  // IRQ 28 to 31 Priority Register
#define NVIC_PRI0_R             (*((volatile unsigned long *)0xE000E400))
#define GPIO_PORTF_DATA_R       (*((volatile unsigned long *)0x400253FC))
#define GPIO_PORTF_DIR_R        (*((volatile unsigned long *)0x40025400))
#define GPIO_PORTF_AFSEL_R      (*((volatile unsigned long *)0x40025420))
#define GPIO_PORTF_PUR_R        (*((volatile unsigned long *)0x40025510))
#define GPIO_PORTF_DEN_R        (*((volatile unsigned long *)0x4002551C))
#define GPIO_PORTF_LOCK_R       (*((volatile unsigned long *)0x40025520))
#define GPIO_PORTF_CR_R         (*((volatile unsigned long *)0x40025524))
#define GPIO_PORTF_AMSEL_R      (*((volatile unsigned long *)0x40025528))
#define GPIO_PORTF_PCTL_R       (*((volatile unsigned long *)0x4002552C))
	
#define GPIO_PORTB_DATA_R       (*((volatile unsigned long *)0x400053FC))
#define GPIO_PORTB_DIR_R        (*((volatile unsigned long *)0x40005400))
#define GPIO_PORTB_AFSEL_R      (*((volatile unsigned long *)0x40005420))
#define GPIO_PORTB_PUR_R        (*((volatile unsigned long *)0x40005510))
#define GPIO_PORTB_DEN_R        (*((volatile unsigned long *)0x4000551C))
#define GPIO_PORTB_LOCK_R       (*((volatile unsigned long *)0x40005520))
#define GPIO_PORTB_CR_R         (*((volatile unsigned long *)0x40005524))
#define GPIO_PORTB_AMSEL_R      (*((volatile unsigned long *)0x40005528))
#define GPIO_PORTB_PCTL_R       (*((volatile unsigned long *)0x4000552C))
#define SYSCTL_RCGC2_R          (*((volatile unsigned long *)0x400FE108))
#define GPIO_PORTF_IS_R         (*((volatile unsigned long *)0x40025404))
#define GPIO_PORTF_IBE_R        (*((volatile unsigned long *)0x40025408))
#define GPIO_PORTF_IEV_R        (*((volatile unsigned long *)0x4002540C))
#define GPIO_PORTF_IM_R         (*((volatile unsigned long *)0x40025410))
#define GPIO_PORTF_RIS_R        (*((volatile unsigned long *)0x40025414))
#define GPIO_PORTF_ICR_R        (*((volatile unsigned long *)0x4002541C))
#define GPIO_PORTF_PDR_R        (*((volatile unsigned long *)0x40025514))

#define GPIO_PORTB_IS_R         (*((volatile unsigned long *)0x40005404))
#define GPIO_PORTB_IBE_R        (*((volatile unsigned long *)0x40005408))
#define GPIO_PORTB_IEV_R        (*((volatile unsigned long *)0x4000540C))
#define GPIO_PORTB_IM_R         (*((volatile unsigned long *)0x40005410))
#define GPIO_PORTB_RIS_R        (*((volatile unsigned long *)0x40005414))
#define GPIO_PORTB_ICR_R        (*((volatile unsigned long *)0x4000541C))
#define GPIO_PORTB_PDR_R        (*((volatile unsigned long *)0x40005514))
//systic
#define NVIC_ST_RELOAD_R        (*((volatile unsigned long *)0xE000E014))
#define NVIC_ST_CURRENT_R       (*((volatile unsigned long *)0xE000E018))
#define NVIC_SYS_PRI3_R         (*((volatile unsigned long *)0xE000ED20))  // Sys. Handlers 12 to 15 Priority
#define NVIC_ST_CTRL_R          (*((volatile unsigned long *)0xE000E010))
#define SYSCTL_RCGC2_GPIOF      0x00000020  // port F Clock Gating Control

#define GPIO_PORTD_AMSEL_R      (*((volatile unsigned long *)0x40007528))
#define GPIO_PORTD_PCTL_R       (*((volatile unsigned long *)0x4000752C))
#define GPIO_PORTD_DIR_R        (*((volatile unsigned long *)0x40007400))
#define GPIO_PORTD_AFSEL_R      (*((volatile unsigned long *)0x40007420))
#define GPIO_PORTD_DR8R_R       (*((volatile unsigned long *)0x40007508))
#define GPIO_PORTD_DEN_R        (*((volatile unsigned long *)0x4000751C))
#define SYSCTL_RCGCGPIO_R       (*((volatile unsigned long *)0x400FE608))

//Global Variables
#define STEPPER  (*((volatile unsigned long *)0x4000703C))
#define clockwise 0        // Next index
#define counterclockwise 1 // Next index
unsigned long In;  // input from PF4
unsigned long Out; // outputs to PF3,PF2,PF1 (multicolor LED)
unsigned int Count; 
unsigned int mode;
unsigned char s; // current state
unsigned long step_count;

//Function declaration
void PortB_Init(void);
void PortF_Init(void);
void EnableInterrupts(void);
void WaitForInterrupt(void);  
void Stepper_Init(void);
void SysTick_Init(unsigned long period);

struct State{
  unsigned long Out;     // Output
  unsigned long Next[2]; // CW/CCW
};
typedef const struct State StateType;
StateType fsm[4]={
  {12,{1,3}},
  { 6,{2,0}},
  { 3,{3,1}},
  { 1,{0,2}}
};

int main(void){  
	//Initialize ports and variables
	Count = 0;  //Flash counter
	mode = 0;	  //Mode selector
	PortB_Init();        
  PortF_Init();         
	Stepper_Init();
	SysTick_Init(160000);                 // initialize SysTick timer	
  EnableInterrupts();                    // The grader uses interrupts
	
	GPIO_PORTF_DATA_R = 0x08;						//Initialize led to green
  while(1){
		STEPPER = fsm[s].Out; // step motor
    WaitForInterrupt();          
  }
}

// Initialize Stepper interface
void Stepper_Init(void){
	s = 0;                      // 2) no need to unlock PD3-0
  SYSCTL_RCGCGPIO_R |= 0x08; // 1) activate port D
  GPIO_PORTD_AMSEL_R &= ~0x0F;      // 3) disable analog functionality on PD3-0
  GPIO_PORTD_PCTL_R &= ~0x0000FFFF; // 4) GPIO configure PD3-0 as GPIO
  GPIO_PORTD_DIR_R |= 0x0F;   // 5) make PD3-0 out
  GPIO_PORTD_AFSEL_R &= ~0x0F;// 6) disable alt funct on PD3-0
  GPIO_PORTD_DR8R_R |= 0x0F;  // enable 8 mA drive
  GPIO_PORTD_DEN_R |= 0x0F;   // 7) enable digital I/O on PD3-0 
}

//systic handler for counting
void SysTick_Handler(void){
// Color    LED(s) PortF
// dark     ---    0
// red      R--    0x02
// blue     --B    0x04
// green    -G-    0x08
// yellow   RG-    0x0A
// sky blue -GB    0x0C	
// white    RGB    0x0E
// pink     R-B    0x06
	
	//Flash red led	
	Count += 1;// count for flash
	if (Count>50){ // half a sec 
		GPIO_PORTF_DATA_R ^= 0x02; //Flash red
		GPIO_PORTF_DATA_R &= ~0x0D;  //clear bits //FIXME: How did you knwo 0d?
		Count = 0;
	}
	
	if(mode){//if mode is high go clockwise
		if (step_count<1000){
			 s = fsm[s].Next[clockwise]; // clock wise circular
			 step_count += 1;
		}
		else{ //Hit max range and set to blue
			//Count = 0;  //FIXME: does this do anything?
			GPIO_PORTF_DATA_R = 0x04;
			GPIO_PORTF_DATA_R &= ~0x0B;
		}
	}
	else{  //Else go counter-clockewise
		if (step_count>0){
			 s = fsm[s].Next[counterclockwise]; // clock wise circular
			 step_count -= 1;
		}
		else{ //Hit max range and set to green
			//Count = 0; //FIXME: does this do anything?
			GPIO_PORTF_DATA_R = 0x08;
			GPIO_PORTF_DATA_R &= ~0x07;//clear bits for color
		}
	}
}
//Doorbell (Sw1) interrupt
void GPIOPortF_Handler(void){
	GPIO_PORTF_ICR_R = 0x10;
	//Why doesn't this set count to 0
	(mode) ? (mode = 0) : (mode = 1);	
}
//Ultrasonic sensor interrupt
void GPIOPortB_Handler(void){
	GPIO_PORTB_ICR_R = 0x02;
	Count = 0;
	(GPIO_PORTB_DATA_R&0x02) ? (mode = 0) : (mode = 1);
}

void SysTick_Init(unsigned long period){
  NVIC_ST_CTRL_R = 0;         // disable SysTick during setup
  NVIC_ST_RELOAD_R = period-1;// reload value
  NVIC_ST_CURRENT_R = 0;      // any write to current clears it
  NVIC_SYS_PRI3_R = (NVIC_SYS_PRI3_R&0x00FFFFFF)|0x40000000; // priority 2
  NVIC_ST_CTRL_R = 0x07;
}

void PortF_Init(void){
	volatile unsigned long delay;
  SYSCTL_RCGC2_R |= 0x00000020;     // 1) F clock
  delay = SYSCTL_RCGC2_R;           // delay   
  GPIO_PORTF_LOCK_R = 0x4C4F434B;   // 2) unlock PortF PF0  
  GPIO_PORTF_CR_R |= 0x1E;           // allow changes to PF4-0       
	GPIO_PORTF_DIR_R &= ~0x10;    // (c) make PF4 in (built-in button)
  GPIO_PORTF_DIR_R |=  0x0E;    //  make PF3,PF2,PF1 output+++
  GPIO_PORTF_AFSEL_R &= ~0x10;  //     disable alt funct on PF4
  GPIO_PORTF_DEN_R |= 0x1E;     //     enable digital I/O on PF4 - PF0  
  GPIO_PORTF_PCTL_R &= ~0x000F0000; // configure PF4 as GPIO
  GPIO_PORTF_AMSEL_R = 0;       //     disable analog functionality on PF
  GPIO_PORTF_PUR_R |= 0x10;     //     enable weak pull-up on PF4
	GPIO_PORTF_IS_R &= ~0x10;     // (d) PF4 is edge-sensitive
  GPIO_PORTF_IBE_R &= ~0x10;    //     PF4 is not both edges
  GPIO_PORTF_IEV_R &= ~0x10;    //     PF4 falling edge event
  GPIO_PORTF_ICR_R = 0x10;      // (e) clear flag4
  GPIO_PORTF_IM_R |= 0x10;      // (f) arm interrupt on PF4
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