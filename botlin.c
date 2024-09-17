#include "MKL25Z4.h"
#include <stdint.h>

//this is just blinky code

#define RED_LED 18 // PortB Pin 18
#define GREEN_LED 19 // PortB Pin 19
#define BLUE_LED 1 // PortD Pin 1
#define SW_POS 6 // PortD Pin 6
#define MASK(x) (1 << (x))

enum led_colour {
	RED, BLUE, GREEN
};

unsigned volatile int int_count = 0;
unsigned volatile int led_control = 0;

void initSwitch (void){
	SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;
	PORTD->PCR[SW_POS] |= (PORT_PCR_MUX(1) | PORT_PCR_PS_MASK |
												PORT_PCR_PE_MASK | PORT_PCR_IRQC(0x0a));
	
	PTD->PDDR &= ~MASK(SW_POS);
	
	NVIC_SetPriority(PORTD_IRQn, 2);
	NVIC_ClearPendingIRQ(PORTD_IRQn);
	NVIC_EnableIRQ(PORTD_IRQn);
												
}

void PORTD_IRQHandler()
{
	NVIC_ClearPendingIRQ(PORTD_IRQn);
	
	int_count = (int_count + 1) % 3;
	led_control ^= 1;
	
	PORTD->ISFR |= MASK(SW_POS);
}


void offAllLeds(){
	PTB->PSOR |= MASK(RED_LED);
	PTB->PSOR |= MASK(GREEN_LED);
	PTD->PSOR |= MASK(BLUE_LED);
}

void initGPIO(void) {
	// Enable Clock to PORTB and PORTD
	SIM->SCGC5 |= ((SIM_SCGC5_PORTB_MASK) | (SIM_SCGC5_PORTD_MASK));
	// Configure MUX settings to make all 3 pins GPIO
	PORTB->PCR[RED_LED] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[RED_LED] |= PORT_PCR_MUX(1);
	PORTB->PCR[GREEN_LED] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[GREEN_LED] |= PORT_PCR_MUX(1);
	PORTD->PCR[BLUE_LED] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[BLUE_LED] |= PORT_PCR_MUX(1);
	// Set Data Direction Registers for PortB and PortD
	PTB->PDDR |= (MASK(RED_LED) | MASK(GREEN_LED));
	PTD->PDDR |= MASK(BLUE_LED);
}


static void delay(volatile uint32_t nof){
	while(nof!=0){
		__ASM("NOP");
		nof--;
	}
}


void toggleLed(enum led_colour colour){
	switch (colour){
		case(RED):
			PTB->PTOR |= MASK(RED_LED);
			break;
		case(GREEN):
			PTB->PTOR |= MASK(GREEN_LED);
			break;
		case(BLUE):
			PTD->PTOR |= MASK(BLUE_LED);
			break;
		default:
			offAllLeds();
			break;
	}
}

int main(void)
{
	initSwitch();
	initGPIO();
	offAllLeds();
	
	while(1)
	{
		if(led_control)
		{
			toggleLed(int_count);
			delay(0x80000);
		}
		else{
			offAllLeds();
		}
	}
}
