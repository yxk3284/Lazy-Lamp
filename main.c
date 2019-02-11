#include "main.h"
/**
Personal project to use the KL46Z to turn on and a light bulb with a relay using a button
*/


int ledPorts[] = {31,19,18,17,16,6,3,2,20,21,22};
boolean buttonInput = True;




void initPins(int leds[], char port, int button){
	for (int i = 0; i < NumLED; i++){
			enablePin(1, port, leds[i]);
	}
	enablePin(0, port, button);
}


int main(){
	__ASM ("CPSID I");  /* Mask all KL46 IRQs */
	Init_UART0_IRQ ();
	//Initialize timer 0
	initPIT(0);
  __ASM ("CPSIE I");  /* Unmask all KL46 IRQs */
	PutStringSB("start", MaxSize);
  initPins(ledPorts, 'E', 2);
	for(;;){
		__ASM("NOP");
			if(readPin('E', 2) == 1){
		PutStringSB("button worked", MaxSize);
		turnOn('E', 31);
	}
	else{
		turnOff('E', 31);
	}
	}

	return 1;
}

