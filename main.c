/*----------------------------------------------------------------------------
 * CMSIS-RTOS 'main' function template
 *---------------------------------------------------------------------------*/
 
#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"
 
#include "MKL25Z4.h"
#include <stdint.h>
#include <stdio.h>

//this is just blinky code

#define RED_LED 18 // PortB Pin 18
#define GREEN_LED 19 // PortB Pin 19
#define BLUE_LED 1 // PortD Pin 1
#define SW_POS 6 // PortD Pin 6
#define MASK(x) (1 << (x))

#define BAUD_RATE 38400
#define UART_TX_PORTE22 22
#define UART_RX_PORTE23 23
#define UART2_INT_PRIO 3

#define LEFT_MOTOR1_PIN 0 // Pin for left motor Positive TPM1_CH0
#define LEFT_MOTOR2_PIN 1 // Pin for left motor Ground TPM1_CH1
#define RIGHT_MOTOR1_PIN 2 // Pin for right motor Positive TPM2_CH0
#define RIGHT_MOTOR2_PIN 3 // Pin for right motor Ground TPM2_CH1

#define PWM_FREQUENCY 50  // 50 Hz

volatile char rx_data = 1;
volatile int leftMotorValue, rightMotorValue;

osThreadId_t packetThreadId, rightPwmThreadId, leftPwmThreadId;
osMutexId_t rightPwmValueMutex, leftPwmValueMutex;
osEventFlagsId_t newPacketFlag, newPwmValueFlag;

void offAllLeds(){
  PTB->PSOR |= MASK(RED_LED);
  PTB->PSOR |= MASK(GREEN_LED);
  PTD->PSOR |= MASK(BLUE_LED);
}


void initPwm(void) {
  
  SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
  
  PORTB->PCR[LEFT_MOTOR1_PIN] &= ~PORT_PCR_MUX_MASK;
  PORTB->PCR[LEFT_MOTOR1_PIN] |= PORT_PCR_MUX(3);
  
  PORTB->PCR[LEFT_MOTOR2_PIN] &= ~PORT_PCR_MUX_MASK;
  PORTB->PCR[LEFT_MOTOR2_PIN] |= PORT_PCR_MUX(3);
  
  PORTB->PCR[RIGHT_MOTOR1_PIN] &= ~PORT_PCR_MUX_MASK;
  PORTB->PCR[RIGHT_MOTOR1_PIN] |= PORT_PCR_MUX(3);
  
  PORTB->PCR[RIGHT_MOTOR2_PIN] &= ~PORT_PCR_MUX_MASK;
  PORTB->PCR[RIGHT_MOTOR2_PIN] |= PORT_PCR_MUX(3);
  
  SIM->SCGC6 |= SIM_SCGC6_TPM1_MASK | SIM_SCGC6_TPM2_MASK;
  
  SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
  SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1);
  
  // Set MOD value for 50 Hz (assuming a 48 MHz clock)
  TPM1->MOD = 48000000 / (128 * PWM_FREQUENCY) - 1;
  TPM2->MOD = 48000000 / (128 * PWM_FREQUENCY) - 1;

  // Set TPM1 and TPM2 to up-counting mode with divide by 128 prescaler
  TPM1->SC = TPM_SC_CMOD(1) | TPM_SC_PS(7);
  TPM2->SC = TPM_SC_CMOD(1) | TPM_SC_PS(7);

  // Configure TPM1_CH0 and TPM1_CH1 for edge-aligned PWM, high-true pulses
  TPM1_C0SC = TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1);
  TPM1_C1SC = TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1);

  // Configure TPM2_CH0 and TPM2_CH1 for edge-aligned PWM, high-true pulses
  TPM2_C0SC = TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1);
  TPM2_C1SC = TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1);
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

// Function to set the duty cycle for a specific motor
void setMotorLeftPWM(uint32_t channel, float dutyCycle) {
    uint32_t mod = TPM1->MOD;
    TPM1->CONTROLS[channel].CnV = mod * ((dutyCycle < 0) ? 0 : dutyCycle);
    
}

void setMotorRightPWM(uint32_t channel, float dutyCycle) {
    uint32_t mod = TPM2->MOD;
    TPM2->CONTROLS[channel].CnV = mod * ((dutyCycle < 0) ? 0 : dutyCycle);
}
  
void processPacket() {
  int isMovement = rx_data & 0x80;
  if (isMovement) {
    int isLeft = rx_data & 0x40;
    int8_t value = ((int8_t) (rx_data << 2)) / 4;
    if (isLeft) {
			osMutexAcquire(leftPwmValueMutex, osWaitForever);
      leftMotorValue = ((int) value) + 1;
			osEventFlagsSet(newPwmValueFlag, 0x0001);
			osMutexRelease(leftPwmValueMutex);
    }
    else {
			osMutexAcquire(rightPwmValueMutex, osWaitForever);
      rightMotorValue = ((int) value) + 1;
			osEventFlagsSet(newPwmValueFlag, 0x0002);
			osMutexRelease(rightPwmValueMutex);
    }
  }
  else {
  }
}

void UART2_IRQHandler() {
  if (UART2->S1 & UART_S1_RDRF_MASK) {
    rx_data = UART2->D;
    osEventFlagsSet(newPacketFlag, 0x0001);
  }
}

void packetThread(void *argument){
    for (;;){
			osEventFlagsWait(newPacketFlag, 0x0001, osFlagsWaitAny, osWaitForever);
      processPacket();
    }
}

void rightPwmThread(void *argument){
    for (;;){
				osEventFlagsWait(newPwmValueFlag, 0x0002, osFlagsWaitAny, osWaitForever);
			
        osMutexAcquire(rightPwmValueMutex, osWaitForever);
        float rightDutyCycle = (float)rightMotorValue/(float)32; // duty cycle for right motors
				osMutexRelease(rightPwmValueMutex);
			
        // Set the PWM for right motors (both motors receive the same value)
        setMotorRightPWM(0, -rightDutyCycle); // Set right motor Positive
        setMotorRightPWM(1, rightDutyCycle); // Set right motor Ground
    }
}

void leftPwmThread(void *argument){
    for (;;){
				osEventFlagsWait(newPwmValueFlag, 0x0001, osFlagsWaitAny, osWaitForever);
			
        osMutexAcquire(leftPwmValueMutex, osWaitForever);
        float leftDutyCycle = (float)leftMotorValue/(float)32; // duty cycle for left motors
        osMutexRelease(leftPwmValueMutex);
				
        // Set the PWM for left motors (both motors receive the same value)
        setMotorLeftPWM(0, leftDutyCycle);  // Set left motor Positive
        setMotorLeftPWM(1, -leftDutyCycle);  // Set left motor Ground
    }
}


int main(void) {

    initUart();
    SystemCoreClockUpdate();
    initPwm();

    osKernelInitialize();

    leftPwmValueMutex = osMutexNew(NULL);
		rightPwmValueMutex = osMutexNew(NULL);
	
		newPacketFlag = osEventFlagsNew(NULL);
		newPwmValueFlag = osEventFlagsNew(NULL);

    packetThreadId = osThreadNew(packetThread, NULL, NULL);
    rightPwmThreadId = osThreadNew(rightPwmThread, NULL, NULL);
		leftPwmThreadId = osThreadNew(leftPwmThread, NULL, NULL);

    osKernelStart();

    for (;;){}
}


