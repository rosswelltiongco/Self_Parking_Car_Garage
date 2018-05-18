#include "TExaS.h"
#include "tm4c123gh6pm.h"

// 2. Declarations Section
//   Global Variables
unsigned int enterFlag = 0;
unsigned int exitFlag = 0;
unsigned int i = 0;

//   Function Prototypes
void EnableInterrupts(void);
void WaitForInterrupt(void);  // low power mode
void PortF_Init(void);
void GPIOPortF_Handler(void);
//void GPIOPortB_Handler(void);
void SysTick_Init(unsigned long period);
void Init_PortB(void);
void flashLED(void);



	// 3. Subroutines Section
// MAIN: Mandatory for a C Program to be executable
int main(void){
	PortF_Init();
	Init_PortB();
	SysTick_Init(8000000);        // initialize SysTick timer
	EnableInterrupts();           // * AFTER inits, should be global
	enterFlag = 0;
	exitFlag = 0;
	GPIO_PORTF_DATA_R = 0x08;  // LED is green at start
  while(1){
		if (enterFlag){
			flashLED();
			GPIO_PORTF_DATA_R = 0x08;  // LED is set to blue
			enterFlag = 0;
		}
		else if (exitFlag){
			flashLED();
			GPIO_PORTF_DATA_R = 0x04;  // LED is set to green
			exitFlag = 0;
		}
  }
}

// Color    LED(s) PortF
// dark     ---    0
// red      R--    0x02
// blue     --B    0x04
// green    -G-    0x08
// yellow   RG-    0x0A
// sky blue -GB    0x0C	
// white    RGB    0x0E
// pink     R-B    0x06

void SysTick_Handler(void){
	//Nothing important - no handling
  //GPIO_PORTF_DATA_R ^= 0x04;       // toggle PF2
  //Counts = Counts + 1;
}

// global variable visible in Watch window of debugger
// increments at least once per button press
void PortF_Init(void){      
	volatile unsigned long delay;
  SYSCTL_RCGC2_R |= 0x00000020;     // 1) F clock
  delay = SYSCTL_RCGC2_R;           // delay   
  GPIO_PORTF_LOCK_R = 0x4C4F434B;   // 2) unlock PortF PF0  
  GPIO_PORTF_CR_R |= 0x1F;           // allow changes to PF4-0       
	GPIO_PORTF_DIR_R &= ~0x11;    // (c) make PF4(sw2), PF0(sw1) inputs (built-in button)
  GPIO_PORTF_DIR_R |=  0x0E;    //  make PF3,PF2,PF1 output+++
  GPIO_PORTF_AFSEL_R &= ~0x11;  //     disable alt funct on PF4
  GPIO_PORTF_DEN_R |= 0x1F;     //     enable digital I/O on PF4 - PF0  
  GPIO_PORTF_PCTL_R &= ~0x000F0000; // configure PF4 as GPIO
  GPIO_PORTF_AMSEL_R &= ~0x11;       //     disable analog functionality on PF
  GPIO_PORTF_PUR_R |= 0x11;     //     enable weak pull-up on PF4
	GPIO_PORTF_IS_R &= ~0x11;     // (d) PF4 is edge-sensitive
  GPIO_PORTF_IBE_R &= ~0x11;    //     PF4 is not both edges
  GPIO_PORTF_IEV_R &= ~0x11;    //     PF4 falling edge event
  GPIO_PORTF_ICR_R = 0x11;      // (e) clear flag4
  GPIO_PORTF_IM_R |= 0x11;      // (f) arm interrupt on PF4
  NVIC_PRI7_R |= (NVIC_PRI7_R&0xFF00FFFF)|0x00A00000; // (g) priority 5
  NVIC_EN0_R |= 0x40000000;      // (h) enable interrupt 30 in NVIC
}

void Init_PortB(void){
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


void SysTick_Init(unsigned long period){
  NVIC_ST_CTRL_R = 0;         // disable SysTick during setup
  NVIC_ST_RELOAD_R = period-1;// reload value
  NVIC_ST_CURRENT_R = 0;      // any write to current clears it
  NVIC_SYS_PRI3_R = (NVIC_SYS_PRI3_R&0x00FFFFFF)|0x40000000; // priority 2
                              // enable SysTick with core clock and interrupts
  NVIC_ST_CTRL_R = 0x07;
}


void GPIOPortF_Handler(void){
  if(GPIO_PORTF_RIS_R&0x01){  // SW2 touch
    GPIO_PORTF_ICR_R = 0x01;  // acknowledge flag0
    enterFlag = 1;
  }
  if(GPIO_PORTF_RIS_R&0x10){  // SW1 touch
    GPIO_PORTF_ICR_R = 0x10;  // acknowledge flag4
    exitFlag = 1;
	}
}

void GPIOPortB_Handler(void){	
	//If within range, else not in range
	GPIO_PORTB_ICR_R = 0x02;
	(GPIO_PORTB_DATA_R&0x02) ? (enterFlag = 1) : (exitFlag = 1);
	
}

void flashLED(void){
	for (i = 0; i < 3; i++){
      GPIO_PORTF_DATA_R = 0x02;  // LED is red
			WaitForInterrupt();        // wait 0.1 sec
			GPIO_PORTF_DATA_R = 0x00;  // LED is cleared
			WaitForInterrupt();        // wait 0.1 sec
	}	
}

