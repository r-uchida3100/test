//libraries

//application

//controller
#include "layer_controller/blink.hpp"
#include "layer_controller/mini_md.hpp"
//base
#include "system.h"
#include "mcutime.h"

//board
#include "pin.hpp"

//circuit
#include "layer_driver/circuit/can_encoder.hpp"

#include "math.h"

/*Led0 led;
Blink blink(led);
blink.setup();
blink.time(500);
while(1){
	blink.cycle();
}
return 0;*/
/*
class statement{
	public:
	CW0 cw0;
	CCW0 ccw0;
	Pwm0 pwm0;
	CW1 cw1;
	CCW1 ccw1;
	Pwm1 pwm1;
	CW2 cw2;
	CCW2 ccw2;
	Pwm2 pwm2;
	Enc0 enc0;
	Enc1 enc1;
	Enc2 enc2;
	Serial0 serial;

	float distance0=0;
	float distance1=0;
	float distance2=0;
	float motor0X=0;
	float motor0Y=0;
	float motor1X=0;
	float motor1Y=0;
	float motor2X=0;
	float motor2Y=0;
	float machineX=0;
	float machineY=0;

	void position();
	void speak();
};

void statement::position(){
	float encoderMAX=200,diameter=40;//encoderMAX=���[�^�[����̃G���R�[�_�[�̒l,diameter=�^�C���̒��a
	enc0.setup();
	enc1.setup();
	enc2.setup();
	while (1)
	{
		distance0=enc0.count()*(diameter*M_PI/encoderMAX);
	    distance1=enc1.count()*(diameter*M_PI/encoderMAX);
   	    distance2=enc2.count()*(diameter*M_PI/encoderMAX);
   	    //���Ȉʒu
   	    //0�Ԃ�1�Ԃ̃O���t�̌�_
  	    motor0X=(distance1-distance0)/(2*sin(M_PI/3));
   	    motor0Y=(distance1+distance0)/(2*cos(M_PI/3));
   	    //0�Ԃ�2�Ԃ̃O���t�̌�_
   	    motor1X=(-1)*(distance0+(distance2*cos(M_PI/3)))/sin(M_PI/3);
   	    motor1Y=(-1)*distance2;
   	    //1�Ԃ�2�Ԃ̃O���t�̌�_
   	    motor2X=(distance1+(distance2*cos(M_PI/3)))/sin(M_PI/3);
   	    motor2Y=(-1)*distance2;

   	    //3�̌�_�̕���
   	    machineX=(motor0X+motor1X+motor2X)/3;
   	    machineY=(motor0Y+motor1Y+motor2Y)/3;
	}
return;
}

statement::statement1(){
	MiniMD motor1(cw0,ccw0,pwm0);
	MiniMD motor0(cw1,ccw1,pwm1);
	MiniMD motor2(cw2,ccw2,pwm2);
	enc0.setup();
	enc1.setup();
	enc2.setup();
	serial.setup(115200);
}

void statement::speak(){
	while (1)
		{
		if (millis()-time>10)
		{
			time=millis();
			serial.printf("%f,%f\n\r",machineX,machineY);
		}
	}
	return;
}*/

int main(){
	CW0 cw0;
	CCW0 ccw0;
	Pwm0 pwm0;
	CW1 cw1;
	CCW1 ccw1;
	Pwm1 pwm1;
	CW2 cw2;
	CCW2 ccw2;
	Pwm2 pwm2;
	Enc0 enc0;
	Enc1 enc1;
	Enc2 enc2;
	Serial0 serial;

	float distance0=0;
	float distance1=0;
	float distance2=0;
	float motor0X=0;
	float motor0Y=0;
	float motor1X=0;
	float motor1Y=0;
	float motor2X=0;
	float motor2Y=0;
	float machineX=0;
	float machineY=0;
	MiniMD motor1(cw0,ccw0,pwm0);
	MiniMD motor0(cw1,ccw1,pwm1);
	MiniMD motor2(cw2,ccw2,pwm2);
	enc0.setup();
	enc1.setup();
	enc2.setup();
	serial.setup(115200);

	float encoderMAX=200,diameter=40;//encoderMAX=���[�^�[����̃G���R�[�_�[�̒l,diameter=�^�C���̒��a

	/*A2 led;
	led.setupDigitalOut();
	int time=0;
	Serial0 serial;
	A0 a0;
	a0.setupAnalogIn();
	serial.setup(115200);
	Enc0 enc0;enc0.setup();
	Enc1 enc1;enc1.setup();
	Enc2 enc2;enc2.setup();
		while(1){
		if(millis()-time>33){
			time=millis();
			led.digitalToggle();
			serial.printf("%d,%d,%d,%f\n",enc0.count(),enc1.count(),enc2.count(),a0.analogRead());
		}
		}
		return 0;*/
	/*int time=0;
	Led0 pin;
	pin.setupDigitalOut();
		while (1)
	{
		if (millis()-time>=100)
		{
			time=millis();
			pin.digitalToggle();
		}
		pin.digitalRead();
		}
    return 0;*/
	int time=0;
	//float theta0,theta1,theta2,theta;
	motor0.setup();
	motor0.duty(-1.0);
	motor0.cycle();
	motor1.setup();
	motor1.duty(1.0);
	motor1.cycle();
	motor2.setup();
	motor2.duty(0.0);
	motor2.cycle();
	while (1)
	{
	   		distance0=enc0.count()*(diameter*M_PI/encoderMAX);
	   		distance1=enc1.count()*(diameter*M_PI/encoderMAX);
	   		distance2=enc2.count()*(diameter*M_PI/encoderMAX);
	   		//���Ȉʒu
	   		//0�Ԃ�1�Ԃ̃O���t�̌�_
	   		motor0X=(distance1-distance0)/(2*sin(M_PI/3));
	   		motor0Y=(distance1+distance0)/(2*cos(M_PI/3));
	   		//0�Ԃ�2�Ԃ̃O���t�̌�_
	   		motor1X=(-1)*(distance0+(distance2*cos(M_PI/3)))/sin(M_PI/3);
	   		motor1Y=(-1)*distance2;
	   		//1�Ԃ�2�Ԃ̃O���t�̌�_
	   		motor2X=(distance1+(distance2*cos(M_PI/3)))/sin(M_PI/3);
	   		motor2Y=(-1)*distance2;

	   		//3�̌�_�̕���
	   		machineX=(motor0X+motor1X+motor2X)/3;
	   		machineY=(motor0Y+motor1Y+motor2Y)/3;
	   		if (millis()-time>10)
	   			{
	   				time=millis();
	   				serial.printf("%f,%f\n\r",machineX,machineY);
                }
     }
return 0;
}
//(2000-����������)*0.0033333...