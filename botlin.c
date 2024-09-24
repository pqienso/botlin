#include "MKL25Z4.h"
#include <stdint.h>
#include <stdio.h>
//this is just blinky code

#define RED_LED 18 // PortB Pin 18
#define GREEN_LED 19 // PortB Pin 19
#define BLUE_LED 1 // PortD Pin 1
#define SW_POS 6 // PortD Pin 6
#define MASK(x) (1 << (x))

#define BAUD_RATE 115200
#define UART_TX_PORTE22 22
#define UART_RX_PORTE23 23
#define UART2_INT_PRIO 3

volatile int8_t rx_data = 1;

static void delay(volatile uint32_t nof){
	while(nof!=0){
		__ASM("NOP");
		nof--;
	}
}

void offAllLeds(){
	PTB->PSOR |= MASK(RED_LED);
	PTB->PSOR |= MASK(GREEN_LED);
	PTD->PSOR |= MASK(BLUE_LED);
}

void initUart(void) {
	uint32_t divisor, bus_clock;
	// Enable Clock to UART and port
	SIM->SCGC4 |= SIM_SCGC4_UART2_MASK;
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
	
	//connect UART pins for PTE22, PTE23
	PORTE->PCR[UART_TX_PORTE22] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[UART_TX_PORTE22] |= PORT_PCR_MUX(4);
	
	PORTE->PCR[UART_RX_PORTE23] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[UART_RX_PORTE23] |= PORT_PCR_MUX(4);
	
	//disable Tx, Rx before config
	UART2->C2 &= ~((UART_C2_TE_MASK) | (UART_C2_RE_MASK));
	
	bus_clock = DEFAULT_SYSTEM_CLOCK / 2;
	divisor = bus_clock / (BAUD_RATE * 16);
	UART2->BDH = UART_BDH_SBR(divisor >> 8);
	UART2->BDL = UART_BDL_SBR(divisor);
	
	//Set UART config
	UART2->C1 = 0;
	UART2->S2 = 0;
	UART2->C3 = 0;
	
	NVIC_SetPriority(UART2_IRQn, UART2_INT_PRIO);
	NVIC_ClearPendingIRQ(UART2_IRQn);
	NVIC_EnableIRQ(UART2_IRQn);
	
	UART2->C2|= ((UART_C2_TE_MASK) | (UART_C2_RE_MASK) | (UART_C2_RIE_MASK));
}


void offLed(int8_t colour){
	switch (colour){
		case(1):
			PTB->PSOR |= MASK(RED_LED);
			break;
		case(2):
			PTB->PSOR |= MASK(GREEN_LED);
			break;
		case(3):
			PTD->PSOR |= MASK(BLUE_LED);
			break;
		default:
			offAllLeds();
			break;
	}
}

void onLed(int8_t colour){
	int8_t newColour = colour;
	switch (newColour){
		case(0):
			PTB->PCOR |= MASK(RED_LED);
			break;
		case(1):
			PTB->PCOR |= MASK(GREEN_LED);
			break;
		case(2):
			PTD->PCOR |= MASK(BLUE_LED);
			break;
		default:
			PTB->PCOR |= MASK(RED_LED);
			PTB->PCOR |= MASK(GREEN_LED);
			PTD->PCOR |= MASK(BLUE_LED);
			break;
	}
}


void UART2_IRQHandler() {
	// NVIC_ClearPendingIRQ(UART2_IRQn);
	
	// TEST BLOCK
	// PTB->PCOR |= MASK(RED_LED);
	// PTB->PCOR |= MASK(GREEN_LED);
	// PTD->PCOR |= MASK(BLUE_LED);
	// delay(200000);
	// offAllLeds();
	
	if (UART2->S1 & UART_S1_RDRF_MASK) {
		// PTB->PCOR |= MASK(RED_LED);
		// PTB->PCOR |= MASK(GREEN_LED);
		// PTD->PCOR |= MASK(BLUE_LED);
		// delay(200000);
		// offAllLeds();
		// printf("%d", UART2->D);
		rx_data = UART2->D;
	}
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


int main(void)
{
	initUart();
	initGPIO();
	offAllLeds();
	
	while(1)
	{
		onLed(rx_data);
		delay(100000);
		offAllLeds();
		delay(100000);
		
	}
}
