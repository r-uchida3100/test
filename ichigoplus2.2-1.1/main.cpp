//libraries

//application

//controller
#include "layer_controller/blink.hpp"
//base
#include "system.h"
#include "mcutime.h"

//board
#include "pin.hpp"

//circuit

int main(void)
{
	Led0 led;
	Blink blink(led);
	blink.setup();
	blink.time(500);
	while(1){
		blink.cycle();
	}
	return 0;
}
