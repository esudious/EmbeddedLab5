// TableTrafficLight.c
// Runs on LM4F120 or TM4C123
// Index implementation of a Moore finite state machine to operate
// a traffic light.


// east facing red light connected to PB5
// east facing yellow light connected to PB4
// east facing green light connected to PB3
// north facing red light connected to PB2
// north facing yellow light connected to PB1
// north facing green light connected to PB0
// north facing car detector connected to PE1 (1=car present)
// east facing car detector connected to PE0 (1=car present)

#include "TExaS.h"
#include "tm4c123gh6pm.h"
#include "PLL.h"
#include "SysTick.h"

#define STREET_LIGHT            (*((volatile unsigned long *)0x400050FC))
#define GPIO_PORTB_OUT          (*((volatile unsigned long *)0x400050FC)) // bits 5-0
#define GPIO_PORTB_DIR_R        (*((volatile unsigned long *)0x40005400))
#define GPIO_PORTB_AFSEL_R      (*((volatile unsigned long *)0x40005420))
#define GPIO_PORTB_DEN_R        (*((volatile unsigned long *)0x4000551C))
#define GPIO_PORTB_AMSEL_R      (*((volatile unsigned long *)0x40005528))
#define GPIO_PORTB_PCTL_R       (*((volatile unsigned long *)0x4000552C))
#define GPIO_PORTE_IN           (*((volatile unsigned long *)0x4002400C)) // bits 1-0
#define SENSOR                  (*((volatile unsigned long *)0x4002400C))

#define GPIO_PORTE_DIR_R        (*((volatile unsigned long *)0x40024400))
#define GPIO_PORTE_AFSEL_R      (*((volatile unsigned long *)0x40024420))
#define GPIO_PORTE_DEN_R        (*((volatile unsigned long *)0x4002451C))
#define GPIO_PORTE_AMSEL_R      (*((volatile unsigned long *)0x40024528))
#define GPIO_PORTE_PCTL_R       (*((volatile unsigned long *)0x4002452C))
#define SYSCTL_RCGC2_R          (*((volatile unsigned long *)0x400FE108))
#define SYSCTL_RCGC2_GPIOE      0x00000010  // port E Clock Gating Control
#define SYSCTL_RCGC2_GPIOB      0x00000002  // port B Clock Gating Control

#define GPIO_PORTF_DATA_R   (*((volatile unsigned long *)0x400253FC))
#define GPIO_PORTF_DIR_R  	(*((volatile unsigned long *)0x40025400))
#define GPIO_PORTF_AFSEL_R  (*((volatile unsigned long *)0x40025420))
#define GPIO_PORTF_PUR_R 		(*((volatile unsigned long *)0x40025510))
#define GPIO_PORTF_DEN_R 		(*((volatile unsigned long *)0x4002551C))
#define GPIO_PORTF_AMSEL_R  (*((volatile unsigned long *)0x40025528))
#define GPIO_PORTF_PCTL_R 	(*((volatile unsigned long *)0x4002552C))
#define GPIO_PORTF_CR_R 		(*((volatile unsigned long *)0x40025524))
#define SYSCTL_RCGCGPIO_R   (*((volatile unsigned long *)0x400FE608))
#define GPIO_PORTF_LOCK_R   (*((volatile unsigned long *)0x40025520))

#define lng_wt 3000
#define sht_wt 500

void DisableInterrupts(void);
void EnableInterrupts(void);
void ports_init(void);

// Linked data structure
struct State {
  unsigned long Out; //LEDS
	unsigned long Time;
	unsigned long PF;
  unsigned long Next[8];
}; 
typedef const struct State STyp;

#define goN   0
#define waitN 1
#define goE   2
#define waitE 3
#define walk  4
#define goNtoWlk 5
#define wtNtoWalk 6
#define goEtoWalk 7
#define wtEtoWalk 8
#define wlktoN 9
#define wlktoE 10
#define walkBlink1 11
#define walkBlink2 12
#define walkBlink3 13
#define walkBlink4 14
#define walkBlink5 15
#define walkBlink6 16
#define walkBlink7 17

#define northGreenLight  0x21 //0x01 , northRedLight 0x04
#define northYellowLight 0x22 //0x02
#define eastGreenLight   0x0C //0x08   eastRedLight 0x20
#define eastYellowLight  0x14 //0x10
#define allRed   				 0x24 //
#define greenWalk				 0x02
#define redWalk 				 0x08

//OUT, TIME, PF3 (walk green light)  N, E, N+E, P, E+P, N+P, All 
STyp FSM[11]={
 {northGreenLight	,lng_wt,redWalk,	{goN,waitN,goN,waitN}}, //Go North (Green) 
 {northYellowLight,sht_wt,redWalk,	{waitN,goE,goE,goE}},  //Wait North (Yellow) next is East
 {eastGreenLight	,lng_wt,redWalk,	{goE,goE,waitE,waitE}}, //Go East (Green)
 {eastYellowLight	,sht_wt,redWalk,	{waitE,goN,goN,goN}},  //Wait East (Yellow) next is North
 {allRed					,sht_wt,greenWalk,{walk,walk,walk,walk}}, //Walk
 {northGreenLight	,lng_wt,redWalk,	{goN,waitN,goN,waitN}}, 			//Go North Next is Wait to walk
 {northYellowLight,sht_wt,redWalk,	{walk,walk,walk,walk}},  //Wait North (Yellow) next is Walk
 {eastGreenLight	,lng_wt,redWalk,	{walk,walk,walk,walk}},			//Go East Next is Wait East to walk
 {eastYellowLight	,sht_wt,redWalk,	{walk,walk,walk,walk}}, 																//Wait East (Yellow) next is Walk 
 {allRed					,sht_wt,greenWalk,{walk,walk,walk,walk}},																		
 {allRed					,sht_wt,redWalk ,	{walk,walk,walk,walk}}
};  
	
unsigned long S;  // index to the current state 
unsigned long Input; 
volatile unsigned long delay;

int main(void){ 
  PLL_Initx();       // 80 MHz, Program 10.1
  SysTick_Init();   // Program 10.2
	EnableInterrupts();
  
  S = goN;  
	
  while(1){
    STREET_LIGHT = FSM[S].Out;  // set lights
    SysTick_Wait10ms(FSM[S].Time);
    Input = SENSOR;     // read sensors
    S = FSM[S].Next[Input];  
  }
}

void ports_init(){
	// Port B
  GPIO_PORTB_LOCK_R = 0x4C4F434B;   // unlock port
  GPIO_PORTB_CR_R = 0x3F;           // allow changes to PB5-0
	GPIO_PORTB_PCTL_R = 0x00000000;   // clear PCTL
  GPIO_PORTB_AMSEL_R &= ~0x3F;      // disable analog on PB5-0
  GPIO_PORTB_AFSEL_R &= ~0x3F;      // disable alt funct on PB5-0
  GPIO_PORTB_DEN_R |= 0x3F;         // enable digital I/O on PB5-0
	GPIO_PORTB_DIR_R |= 0x3F;         // PB5-0 outputs
	
	// Port E
  GPIO_PORTE_LOCK_R = 0x4C4F434B;   // unlock port
  GPIO_PORTE_CR_R = 0x07;           // allow changes to PE2-0
	GPIO_PORTE_PCTL_R = 0x00000000;   // clear PCTL
  GPIO_PORTE_AMSEL_R &= ~0x07;      // disable analog on PE2-0
  GPIO_PORTE_AFSEL_R &= ~0x07;      // disable alt funct on PE2-0
  GPIO_PORTE_PUR_R &= ~0x07;        // disableb pull-up on PE2-0
  GPIO_PORTE_DEN_R |= 0x07;         // enable digital I/O on PE2-0
	GPIO_PORTE_DIR_R &= ~0x07;        // PE2-0 inputs
	
	SYSCTL_RCGC2_R |= 0x12;      // 1) B E
  delay = SYSCTL_RCGC2_R;      // 2) no need to unlock
  GPIO_PORTE_AMSEL_R &= ~0x03; // 3) disable analog function on PE1-0
  GPIO_PORTE_PCTL_R &= ~0x000000FF; // 4) enable regular GPIO
  GPIO_PORTE_DIR_R &= ~0x03;   // 5) inputs on PE1-0
  GPIO_PORTE_AFSEL_R &= ~0x03; // 6) regular function on PE1-0
  GPIO_PORTE_DEN_R |= 0x03;    // 7) enable digital on PE1-0
  GPIO_PORTB_AMSEL_R &= ~0x3F; // 3) disable analog function on PB5-0
  GPIO_PORTB_PCTL_R &= ~0x00FFFFFF; // 4) enable regular GPIO
  GPIO_PORTB_DIR_R |= 0x3F;    // 5) outputs on PB5-0
  GPIO_PORTB_AFSEL_R &= ~0x3F; // 6) regular function on PB5-0
  GPIO_PORTB_DEN_R |= 0x3F;    // 7) enable digital on PB5-0
	GPIO_PORTF_LOCK_R = 0x4C4F434B;   // 8) unlock GPIO Port F
  GPIO_PORTF_CR_R = 0x0A;           // allow changes to PF3 and PF1
  GPIO_PORTF_AMSEL_R &= ~0x0A; 			// 9) disable analog function on PF3 abd PF1
  GPIO_PORTF_PCTL_R = 0x00000000;   // 10) PCTL GPIO on PF4-0
  GPIO_PORTF_DIR_R |= 0x0A;          // 11) PF3 and PF1 outputs
  GPIO_PORTF_AFSEL_R &= ~0x0A; 			// 12) regular function on PF3 and PF1
  GPIO_PORTF_DEN_R |= 0x0A;    			// 13) enable digital on PF3 and PF1
}




