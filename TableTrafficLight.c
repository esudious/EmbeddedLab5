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

#define lng_wt 30
#define sht_wt 5

void DisableInterrupts(void);
void EnableInterrupts(void);
void ports_init(void);

// Linked data structure
struct State {
  unsigned long Out; //LEDS
	unsigned long Time;
  unsigned long Next[8];
}; 
typedef const struct State STyp;

#define goN   0
#define waitN 1
#define goE   2
#define waitE 3
#define walk  4
#define walkBlink1 5
#define walkBlink2 6
#define walkBlink3 7
#define walkBlink4 8
#define walkBlink5 9
#define walkBlink6 10

#define northGreenLight  0xA1 //0x01 , northRedLight 0x04
#define northYellowLight 0xA2 //0x02
#define eastGreenLight   0x8C //0x08   eastRedLight 0x20
#define eastYellowLight  0x94 //0x10
#define allRed   		 		 0xA4 //0x80 + 0x20 + 0x04
#define greenWalk		     0x64 //0x40  walkRed = 0x40
//LIGHTS           WAIT    BUTTONS PRESSED N = 0x01, E = 0x02, W = 0x04
//OUT,             TIME,   None, N,  E,   N+E,   W, E+W, N+W, All 
STyp FSM[11]={
 {northGreenLight	,lng_wt,{goN,	goN,waitN,waitN,waitN,waitN,waitN,1}}, //Go North (Green) 
 {northYellowLight,sht_wt,{goE,goE,goE,goE,walk,walk,walk,2}},  			//Wait North Yellow
 {eastGreenLight	,lng_wt,{goE,	waitE,waitE,waitE,waitE,waitE,waitE,3}}, //Go East (Green)
 {eastYellowLight	,sht_wt,{walk,goN,goN,goN,walk,walk,walk,4}},  //East Yellow 
 {allRed					,sht_wt,{walkBlink1,walkBlink1,walkBlink1,walkBlink1,walkBlink1,walkBlink1,walkBlink1,5}}, //Walk
 {greenWalk				,lng_wt,{walkBlink2,walkBlink2,walkBlink2,walkBlink2,walkBlink2,walkBlink2,walkBlink2,6}}, 			
 {allRed					,sht_wt,{walkBlink3,walkBlink3,walkBlink3,walkBlink3,walkBlink3,walkBlink3,walkBlink3,7}},  
 {greenWalk				,lng_wt,{walkBlink4,walkBlink4,walkBlink4,walkBlink4,walkBlink4,walkBlink4,walkBlink4,8}},	
 {allRed					,sht_wt,{walkBlink5,walkBlink5,walkBlink5,walkBlink5,walkBlink5,walkBlink5,walkBlink5,9}}, 	
 {greenWalk				,lng_wt,{walkBlink6,walkBlink6,walkBlink6,walkBlink6,walkBlink6,walkBlink6,walkBlink6,10}},																		
 {allRed					,sht_wt,{goN,goN,goE,goN,walkBlink6,goE,goN,0}}
};  
	
unsigned long S;  // index to the current state 
unsigned long Input; 
volatile unsigned long delay;
unsigned long waittime;
unsigned long walk_light;

int main(void){ 
	
	SysTick_Init();   // Program 10.2
	PLL_Init();       // 80 MHz, Program 10.1
	ports_init();
  
  S = goN;  
	
	EnableInterrupts();
	
  while(1){
    STREET_LIGHT = FSM[S].Out;  // set street lights
		GPIO_PORTB_OUT = (STREET_LIGHT & 0x3F);
		walk_light = FSM[S].Out;
		GPIO_PORTF_DATA_R = 0x0F & (((walk_light & 0x80)>>6) | ((walk_light & 0x40)>>3)); //LIGHTS FOR PF, PF3 = GREEN, PF1 = RED
		waittime = FSM[S].Time;
    SysTick_Wait10ms(waittime);
    Input = GPIO_PORTE_DATA_R; 
    S = FSM[S].Next[Input];  
  }
}


void ports_init(){
	// Port B      
	SYSCTL_RCGC2_R |= 0x32;						
	delay = SYSCTL_RCGC2_R;   				// 1) B E
  
  //GPIO_PORTB_LOCK_R = 0x4C4F434B;   // unlock port
  //GPIO_PORTB_CR_R = 0x3F;           // allow changes to PB5-0
	GPIO_PORTB_PCTL_R = 0x00000000;   // clear PCTL
  GPIO_PORTB_AMSEL_R = 0x00;      // disable analog on PB5-0
  GPIO_PORTB_AFSEL_R &= ~0x3F;      // disable alt funct on PB5-0
  GPIO_PORTB_DEN_R |= 0x3F;         // enable digital I/O on PB5-0
	GPIO_PORTB_DIR_R |= 0x3F;         // PB5-0 outputs
	
	//PORT F
	GPIO_PORTF_LOCK_R = 0x4C4F434B;   // 8) unlock GPIO Port F
  GPIO_PORTF_CR_R = 0x0A;           // allow changes to PF3 and PF1
  GPIO_PORTF_AMSEL_R &= ~0x0A; 			// 9) disable analog function on PF3 abd PF1
  GPIO_PORTF_PCTL_R = 0x00000000;   // 10) PCTL GPIO on PF4-0
  GPIO_PORTF_DIR_R |= 0x1F;          // 11) PF3 and PF1 outputs
  GPIO_PORTF_AFSEL_R &= ~0x0A; 			// 12) regular function on PF3 and PF1
  GPIO_PORTF_DEN_R |= 0x0A;    			// 13) enable digital on PF3 and PF1
	GPIO_PORTF_PUR_R |= 0x0A;
	// Port E
	GPIO_PORTE_DIR_R		&= ~0x07; 
	GPIO_PORTE_AFSEL_R		&= 0x00;
	GPIO_PORTE_DEN_R		|= 0x07;
}




