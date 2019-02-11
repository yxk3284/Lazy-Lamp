#include "PinDriver.h"
/**
Code that runs the GPIO. Can set up pins to be inputs/outputs, and write/read to pins
*/
/**
This function will enable a port for GPIO. Can enable ports A-E, pins 0-31
*/
void enablePin(uint8_t inOut, char port, uint32_t pin){
		if (pin > 32)
			return;
		switch(port){
			case 'A':
				SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;		//Set port clock
				PORTA->PCR[pin] = PORT_PCR_MUX(1u);		//Set pin to GPIO
				PTD->PDDR |= (inOut << pin);					//Set the pin to input/output
			break;
			case 'B':
				SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;		//Set port clock
				PORTB->PCR[pin] = PORT_PCR_MUX(1u);		//Set pin to GPIO
				PTB->PDDR |= (inOut << pin);					//Set the pin to input/output
			break;
			case 'C':
				SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;		//Set port clock
				PORTC->PCR[pin] = PORT_PCR_MUX(1u);		//Set pin to GPIO
				PTC->PDDR |= (inOut << pin);					//Set the pin to input/output
			break;
			case 'D':
				SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;		//Set port clock
				PORTD->PCR[pin] = PORT_PCR_MUX(1u);		//Set pin to GPIO
				PTD->PDDR |= (inOut << pin);					//Set the pin to input/output
			break;
			case 'E':
				SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;		//Set port clock
				PORTE->PCR[pin] = PORT_PCR_MUX(1u);		//Set pin to GPIO
				PTE->PDDR |= (inOut << pin);					//Set the pin to input/output
			break;
		}
}

/**
Method will turn of port at specific pin
*/
void turnOn(char port, uint32_t pin){
	if (pin > 32)
			return;
	switch(port){
		case 'A':
			PTA->PSOR |= (1<<pin);
		break;
		case 'B':
			PTB->PSOR |= (1<<pin);
		break;
		case 'C':
			PTC->PSOR |= (1<<pin);
		break;
		case 'D':
			PTD->PSOR |= (1<<pin);
		break;
		case 'E':
			PTE->PSOR |= (1<<pin);
		break;
	}
}

/**
Will turn of a pin at a specific port
*/
void turnOff(char port, uint32_t pin){
		if (pin > 32)
			return;
		switch(port){
		case 'A':
			PTA->PCOR |= (1<<pin);
		break;
		case 'B':
			PTB->PCOR |= (1<<pin);
		break;
		case 'C':
			PTC->PCOR |= (1<<pin);
		break;
		case 'D':
			PTD->PCOR |= (1<<pin);
		break;
		case 'E':
			PTE->PCOR |= (1<<pin);
		break;
	}
}
int readPin(char port, uint32_t pin){
		if (pin > 32)
			return -1;
		switch(port){
			case 'A':
				return (PTA->PDIR & (1<<pin)) > 0;
			case 'B':
				return (PTB->PDIR & (1<<pin)) > 0;
			case 'C':
				return (PTC->PDIR & (1<<pin)) > 0;
			case 'D':
				return (PTD->PDIR & (1<<pin)) > 0;
			case 'E':
				return (PTE->PDIR & (1<<pin)) > 0;
		}
		return -1;
}
