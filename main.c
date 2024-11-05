/*----------------------------------------------------------------------------
 * CMSIS-RTOS 'main' function template
 *---------------------------------------------------------------------------*/
 
#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"
 
#include "MKL25Z4.h"
#include <stdint.h>
#include <stdio.h>

#define BAUD_RATE 38400
#define UART_TX_PORTE22 22
#define UART_RX_PORTE23 23
#define UART2_INT_PRIO 3

#define LEFT_MOTOR1_PIN 0 // Pin for left motor Positive TPM1_CH0
#define LEFT_MOTOR2_PIN 1 // Pin for left motor Ground TPM1_CH1
#define RIGHT_MOTOR1_PIN 2 // Pin for right motor Positive TPM2_CH0
#define RIGHT_MOTOR2_PIN 3 // Pin for right motor Ground TPM2_CH1

#define PWM_FREQUENCY 50  // 50 Hz

#define NUM_LEDS 10
#define RED_LED_PIN 22

#define BUZZER_PIN 2

volatile char uartData = 1;
volatile int leftMotorValue, rightMotorValue, isMovingTune = 1;

// Define the GPIO ports and pins for the LEDs
uint32_t ledPorts[NUM_LEDS] = {
  PORTB_BASE, PORTB_BASE, PORTB_BASE, PORTB_BASE, PORTE_BASE,
  PORTE_BASE, PORTE_BASE, PORTE_BASE, PORTE_BASE, PORTE_BASE
};
uint32_t ledPins[NUM_LEDS] = {8, 9, 10, 11, 2, 3, 4, 5, 1, 0};

osThreadId_t packetProcessingThreadId, rightPwmThreadId, leftPwmThreadId,
    ledControlThreadId, buzzerControlThreadId;
osMutexId_t rightPwmValueMutex, leftPwmValueMutex;
osEventFlagsId_t newPacketFlag, newPwmValueFlag;


void initGpio(void) {
    // Enable clocks for PORTB and PORTC
    SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTE_MASK;

    for (int i = 0; i < NUM_LEDS; i++) {
        // Configure the pin as GPIO
        if (ledPorts[i] == PORTB_BASE) {
            PORTB->PCR[ledPins[i]] = PORT_PCR_MUX(1);
            PTB->PDDR |= (1 << ledPins[i]);
        } else if (ledPorts[i] == PORTE_BASE) {
            PORTE->PCR[ledPins[i]] = PORT_PCR_MUX(1);
            PTE->PDDR |= (1 << ledPins[i]);
        }
    }
    PORTE->PCR[22] = PORT_PCR_MUX(1);
    PTE->PDDR |= (1 << 22);
}


void initPwm(void) {
  SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTC_MASK;

  PORTB->PCR[LEFT_MOTOR1_PIN] &= ~PORT_PCR_MUX_MASK;
  PORTB->PCR[LEFT_MOTOR1_PIN] |= PORT_PCR_MUX(3);
  
  PORTB->PCR[LEFT_MOTOR2_PIN] &= ~PORT_PCR_MUX_MASK;
  PORTB->PCR[LEFT_MOTOR2_PIN] |= PORT_PCR_MUX(3);
  
  PORTB->PCR[RIGHT_MOTOR1_PIN] &= ~PORT_PCR_MUX_MASK;
  PORTB->PCR[RIGHT_MOTOR1_PIN] |= PORT_PCR_MUX(3);
  
  PORTB->PCR[RIGHT_MOTOR2_PIN] &= ~PORT_PCR_MUX_MASK;
  PORTB->PCR[RIGHT_MOTOR2_PIN] |= PORT_PCR_MUX(3);

  PORTC->PCR[BUZZER_PIN] &= ~PORT_PCR_MUX_MASK;
  PORTC->PCR[BUZZER_PIN] |= PORT_PCR_MUX(4);
  
  SIM->SCGC6 |= SIM_SCGC6_TPM1_MASK | SIM_SCGC6_TPM2_MASK | SIM_SCGC6_TPM0_MASK;
  
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

  //Buzzer

  TPM0->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
  TPM0->SC = TPM_SC_CMOD(1) | TPM_SC_PS(7);
  TPM0->SC &= ~(TPM_SC_CPWMS_MASK);
  
  TPM0_C1SC = TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1);
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
  
  UART2->C2 |= ((UART_C2_TE_MASK) | (UART_C2_RE_MASK) | (UART_C2_RIE_MASK));
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
  

void UART2_IRQHandler() {
  if (UART2->S1 & UART_S1_RDRF_MASK) {
    uartData = UART2->D;
    osEventFlagsSet(newPacketFlag, 0x0001);
  }
}

void packetProcessingThread(void *argument){
    for (;;){
      osEventFlagsWait(newPacketFlag, 0x0001, osFlagsWaitAny, osWaitForever);
      
      int isMovement = uartData & 0x80;
      
      if (isMovement) {
        int isLeft = uartData & 0x40;
        int8_t value = ((int8_t) (uartData << 2)) / 4;
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
        isMovingTune = uartData & 1;
      }
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

void setRedLed(int turnOn) {
    if (turnOn) {
        PTE->PSOR = (1 << RED_LED_PIN);
    } else {
        PTE->PCOR = (1 << RED_LED_PIN);
    }
}

void turnOffAllGreenLeds() {
    for (int i = 0; i < NUM_LEDS; i++) {
        if (ledPorts[i] == PORTB_BASE) {
            PTB->PCOR = (1 << ledPins[i]);  
        } else if (ledPorts[i] == PORTE_BASE) {
            PTE->PCOR = (1 << ledPins[i]);
        }
    }
}


void turnOnGreenLed(int index) {
    if (ledPorts[index] == PORTB_BASE) {
        PTB->PSOR = (1 << ledPins[index]);
    } else if (ledPorts[index] == PORTE_BASE) {
        PTE->PSOR = (1 << ledPins[index]);
    }
}

void turnOnAllGreenLeds(void) {
    for (int i = 0; i < NUM_LEDS; i++) {
        turnOnGreenLed(i);
    }
}

void greenLedWalk(void) {
    for (int i = 0; i < NUM_LEDS; i++) {
        turnOnGreenLed(i);
        osDelay(50);
        turnOffAllGreenLeds();
    }
}

void ledControlThread(void *arg) {
    while (1) {
        int isMoving = leftMotorValue != 0 && rightMotorValue != 0;
      
        if (isMoving){
          setRedLed(1);
          greenLedWalk();
          setRedLed(0);
          greenLedWalk();
        }
        else {
          turnOnAllGreenLeds();
          setRedLed(1);
          osDelay(250);
          setRedLed(0);
          osDelay(250);
        }
    }
}

void playNote(uint16_t frequency, uint32_t duration) {
        if (frequency == 0) {
          TPM0_C1V = 0;
        }
        else{  
          TPM0->MOD = (48000000 / (128 * frequency)) - 1;
          TPM0_C1V = TPM0->MOD / 2;  // 75% duty cycle
        }
        osDelay(duration);
}


void buzzerControlThread(void *arg) {
    uint16_t movingNotes[] = {
        932, 0, 932, 0, 932, 988, 1244
    };
    uint32_t movingRhythm[] = {
        200, 200, 200, 200, 300, 300, 200
    };
    
    uint16_t finishNotes[] = {
        1318, 1174, 740, 830, 1108, 988, 588,
        660, 988, 880, 554, 660, 880
    };
    uint32_t finishRhythm[] = {
        150, 150, 300, 300, 150, 150, 300,
        300, 150, 150, 300, 300, 600
    };
  
    for (;;) {
        if(isMovingTune) {
          for (int i = 0; i < 7; i++) {
            playNote(movingNotes[i] * 8, movingRhythm[i]);
          }
        }
        else {
          for (int i = 0; i < 13; i++) {
            playNote(finishNotes[i] * 11, finishRhythm[i]);
          }
        }
    }
}


int main(void) {

    initUart();
    SystemCoreClockUpdate();
    initPwm();
    initGpio();

    osKernelInitialize();

    leftPwmValueMutex = osMutexNew(NULL);
    rightPwmValueMutex = osMutexNew(NULL);
  
    newPacketFlag = osEventFlagsNew(NULL);
    newPwmValueFlag = osEventFlagsNew(NULL);

    packetProcessingThreadId = osThreadNew(packetProcessingThread, NULL, NULL);
    rightPwmThreadId = osThreadNew(rightPwmThread, NULL, NULL);
    leftPwmThreadId = osThreadNew(leftPwmThread, NULL, NULL);
    ledControlThreadId = osThreadNew(ledControlThread, NULL, NULL);
    buzzerControlThreadId = osThreadNew(buzzerControlThread, NULL, NULL);

    osKernelStart();
    

    for (;;){}
}
