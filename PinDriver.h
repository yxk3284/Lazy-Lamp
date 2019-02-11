#include "MKL46Z4.h"

void enablePin(uint8_t inOut, char port, uint32_t pin);
void turnOn(char port, uint32_t pin);
void turnOff(char port, uint32_t pin);
int readPin(char port, uint32_t pin);
