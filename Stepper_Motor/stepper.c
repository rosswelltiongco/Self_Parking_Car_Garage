//Stepper.c
// PD3 connected to driver for stepper motor coil A
// PD2 connected to driver for stepper motor coil A'
// PD1 connected to driver for stepper motor coil B
// PD0 connected to driver for stepper motor coil B'

#include <stdint.h>
#include "tm4c123gh6pm.h"
#include "systick.h"
struct State{
  uint8_t Out;     // Output
  uint8_t Next[2]; // CW/CCW
};
typedef const struct State StateType;

#define clockwise 0        // Next index
#define counterclockwise 1 // Next index
#define T1ms 16000    // assumes using 16 MHz PIOSC (default setting for clock source)
StateType motorLeft[4]={
  {12,{1,3}},
  { 6,{2,0}},
  { 3,{3,1}},
  { 1,{0,2}}
};
StateType motorRight[4]={
  {12,{1,3}},
  { 6,{2,0}},
  { 3,{3,1}},
  { 1,{0,2}}
};
unsigned int i;
unsigned char leftState; // current state
unsigned char rightState; // current state

#define STEPPERLEFT   (*((volatile unsigned long *)0x4000703C))
#define STEPPERRIGHT  (*((volatile unsigned long *)0x4002403C)) 

// Move 1.8 degrees clockwise, delay is the time to wait after each step
void StepperLeft_CW(unsigned long delay){
  leftState = motorLeft[leftState].Next[clockwise]; // clock wise circular
  STEPPERLEFT = motorLeft[leftState].Out; // step motor
  SysTick_Wait(delay);
}
// Move 1.8 degrees counterclockwise, delay is wait after each step
void StepperLeft_CCW(unsigned long delay){
  leftState = motorLeft[leftState].Next[counterclockwise]; // counter clock wise circular
  STEPPERLEFT = motorLeft[leftState].Out; // step motor
  SysTick_Wait(delay); // blind-cycle wait
}

// Move 1.8 degrees clockwise, delay is the time to wait after each step
void StepperRight_CW(unsigned long delay){
  rightState = motorRight[rightState].Next[clockwise]; // clock wise circular
  STEPPERRIGHT = motorLeft[rightState].Out; // step motor
  SysTick_Wait(delay);
}
// Move 1.8 degrees counterclockwise, delay is wait after each step
void StepperRight_CCW(unsigned long delay){
  rightState = motorRight[rightState].Next[counterclockwise]; // counter clock wise circular
  STEPPERRIGHT = motorRight[rightState].Out; // step motor
  SysTick_Wait(delay); // blind-cycle wait
}

// Initialize Stepper interface
void Stepper_Init(void){
	SysTick_Init();
	//Init Left Motor
  SYSCTL_RCGCGPIO_R |= 0x08; // 1) activate port D
  leftState = 0; 
                                    // 2) no need to unlock PD3-0
  GPIO_PORTD_AMSEL_R &= ~0x0F;      // 3) disable analog functionality on PD3-0
  GPIO_PORTD_PCTL_R &= ~0x0000FFFF; // 4) GPIO configure PD3-0 as GPIO
  GPIO_PORTD_DIR_R |= 0x0F;   // 5) make PD3-0 out
  GPIO_PORTD_AFSEL_R &= ~0x0F;// 6) disable alt funct on PD3-0
  GPIO_PORTD_DR8R_R |= 0x0F;  // enable 8 mA drive
  GPIO_PORTD_DEN_R |= 0x0F;   // 7) enable digital I/O on PD3-0 
	
	//Init Right Motor
	SYSCTL_RCGCGPIO_R |= 0x10; // 1) activate port E
  rightState = 0; 
                                    // 2) no need to unlock PD3-0
  GPIO_PORTE_AMSEL_R &= ~0x0F;      // 3) disable analog functionality on PDE-0
  GPIO_PORTE_PCTL_R &= ~0x0000FFFF; // 4) GPIO configure PE3-0 as GPIO
  GPIO_PORTE_DIR_R |= 0x0F;   // 5) make PDE-0 out
  GPIO_PORTE_AFSEL_R &= ~0x0F;// 6) disable alt funct on PDE-0
  GPIO_PORTE_DR8R_R |= 0x0F;  // enable 8 mA drive
  GPIO_PORTE_DEN_R |= 0x0F;   // 7) enable digital I/O on PE3-0 
}

void moveForward(void){
	//1000 = 180 degrees
	for (i=0; i<100; i++){
		StepperLeft_CW(10*T1ms);   // output every 10ms
		StepperRight_CCW(10*T1ms);   // output every 10ms
	}
}

void moveBackward(void){
	for (i=0; i<100; i++){
			StepperLeft_CCW(10*T1ms);   // output every 10ms
			StepperRight_CW(10*T1ms);   // output every 10ms
	}
}

void turnLeft(void){
	for (i=0; i<100; i++){
			StepperLeft_CCW(10*T1ms);   // output every 10ms
			StepperRight_CCW(10*T1ms);   // output every 10ms
	}
}

void turnRight(void){
	for (i=0; i<100; i++){
			StepperLeft_CW(10*T1ms);   // output every 10ms
			StepperRight_CW(10*T1ms);   // output every 10ms
	}
}
