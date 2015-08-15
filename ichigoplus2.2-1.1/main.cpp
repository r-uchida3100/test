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
#include "layer_driver/circuit_drivers/can_encoder/can_encoder.hpp"
#include "layer_driver/circuit_drivers/mini_md_1.0/mini_md.hpp"

//driver

#include "math.h"
/*
Led0 led;
Blink blink(led);
blink.setup();
blink.time(500);
while(1){
	blink.cycle();
}
return 0;*/
/*float motorPDprogram(float current,float command,float Pgain,float Dgain)
{
	float output=0.00;
	float deviation=0.00;
	static float olddiviation=0.00;

	deviation=

}*/

class declaration
{
public:
	int time=0;
	float encoderMAX1=200;
	float encoderMAX2=1000;
	float diameter=30;
	float long0=115;
	float long1=115;
	float long2=115;
	float distance0=0;
	float distance1=0;
	float distance2=0;
	float motor0X=0;
	float motor0Y=0;
	float motor1X=0;
	float motor1Y=0;
	float motor2X=0;
	float motor2Y=0;
	float machineX1=0;
	float machineY1=0;
	float machinex1=0;
	float machiney1=0;
	float machinex2=0;
	float machiney2=0;
	float machinex=0;
	float machiney=0;
	float machinexdash=0;
	float machineydash=0;
	float machineX=0;
	float machineY=0;
	float a=0;
	float theta0=0,theta1=0,theta2=0,theta=0;
	float last0=0,last1=0,last2=0;

	Serial0 serial;
	Enc0 enc0;
	Enc1 enc1;
	Enc2 enc2;
	CW0 cw0;
	CCW0 ccw0;
	Pwm0 pwm0;
	CW1 cw1;
	CCW1 ccw1;
	Pwm1 pwm1;
	CW2 cw2;
	CCW2 ccw2;
	Pwm2 pwm2;
	void position();
	void print();
};

declaration::declaration2()
{
	MiniMD motor0(cw0,ccw0,pwm0);
	MiniMD motor1(cw1,ccw1,pwm1);
	MiniMD motor2(cw2,ccw2,pwm2);

	motor0.setup();
	motor0.duty(1.0);
	motor0.cycle();
	motor1.setup();
	motor1.duty(-1.0);
	motor1.cycle();
	motor2.setup();
	motor2.duty(0.0);
	motor2.cycle();
	enc0.setup();
	enc1.setup();
	enc2.setup();
	serial.setup(115200);
}

class EncoderUser{
public:
	EncoderUser(Encoder &enc_0,Encoder &enc_1,Encoder &enc_2){
		enc0=&enc_0;
		enc1=&enc_1;
		enc2=&enc_2;
	}
	int setup(){
		int i=0;
		i+=(enc0->setup()!=0);
		i+=(enc1->setup()!=0);
		i+=(enc2->setup()!=0);
	}
private:
	Encoder *enc0;
	Encoder *enc1;
	Encoder *enc2;
};

void declaration::position()
{
	distance0=enc0.count()*(diameter*M_PI/encoderMAX2)-last0;
	distance1=enc1.count()*(diameter*M_PI/encoderMAX1)-last1;
	distance2=enc2.count()*(diameter*M_PI/encoderMAX1)-last2;
	last0=canenc0.count()*(diameter*M_PI/encoderMAX2);
	last1=canenc1.count()*(diameter*M_PI/encoderMAX1);
	last2=canenc2.count()*(diameter*M_PI/encoderMAX1);

	theta0=atan2(distance0,long0);
	theta1=atan2(distance1,long1);
	theta2=atan2(distance2,long2);
	theta=(theta0+theta1+theta2)/3;

	motor0X=(distance1-distance0)/(2*sin(M_PI/3));
	motor0Y=(-1)*(distance1+distance0)/(2*cos(M_PI/3));

	motor1X=(-1)*(distance0+(distance2*cos(M_PI/3)))/sin(M_PI/3);
	motor1Y=distance2;

	motor2X=(distance1+(ditance2*cos(M_PI/3)))/sin(M_PI/3);
	motor2Y=distance2;

	machineX1=(motor0X+motor1X+motor2X)/3;//��L��3����x���W�̍��v
	machineY1=(motor0Y+motor1Y+motor2Y)/3;//��L��3����y���W�̍��v

	*machineX+=machineX1*cos(theta)-machineY1*sin(theta);//5�~���b��̍��Wx�ƌ��݂�x���W�Ƒ���
	*machineY+=machineX1*sin(theta)+machineY1*cos(theta);//5�~���b��̍��Wy�ƌ��݂�y���W�𑫂�
}

void declaration::print()
{
	if (millis()-time>50)
	 	{
	 	    time=millis();
	 		//serial.printf("%d %d %d\n\r",canenc0.count(),canenc1.count(),canenc2.count());
	 		serial.printf("%f %f\n\r",machineX,machineY);
	    }
}

int main()
{
	declaration declaration1;
	//float i,j;

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
	Can0 can;
	CanEncoder enc0(can , 0 , 5);
	CanEncoder enc1(can , 1 , 5);
	CanEncoder enc2(can , 2 , 5);

	EncoderUser enc(enc0,enc1,enc2);
	enc.setup();
	while (1)
	{
   		/*if (machineX>=1500)
   		{
   			i=2000-machineX;
   			j=i*0.002;
   			if (j>=1)
   			{
   				j=1;
   				motor0.duty(j);
   				motor1.duty(-j);
   			}
   		}*/
   		declaration1.print();
     }
return 0;
}
