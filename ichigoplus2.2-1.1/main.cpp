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
	declaration();
};

declaration::declaration()
{
	MiniMD motor0(cw0,ccw0,pwm0);
	MiniMD motor1(cw1,ccw1,pwm1);
	MiniMD motor2(cw2,ccw2,pwm2);

	motor0.setup();
	motor0.duty(-1.0);
	motor0.cycle();
	motor1.setup();
	motor1.duty(1.0);
	motor1.cycle();
	motor2.setup();
	motor2.duty(0.0);
	motor2.cycle();
	enc0.setup();
	enc1.setup();
	enc2.setup();
};

class EncoderUser{
public:
	float encoderMAX1;
	float encoderMAX2;
	float diameter;
	float long0;
	float long1;
	float long2;
	float motor0X;
	float motor0Y;
	float motor1X;
	float motor1Y;
	float motor2X;
	float motor2Y;
	float machineX1;
	float machineY1;
	float machinex1;
	float machiney1;
	float machinex2;
	float machiney2;
	float machinex;
	float machiney;
	float machinexdash;
	float machineydash;
	float machineX;
	float machineY;
	float a;
	float last0,last1,last2;
	float encAction0,encAction1,encAction2;
	float encActionOld0,encActionOld1,encActionOld2;
	float encActionRemainder0,encActionRemainder1,encActionRemainder2;
	float machineAngle;

	void cycle();
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
		return i;
	}
private:
	Encoder *enc0;
	Encoder *enc1;
	Encoder *enc2;
};

void EncoderUser::cycle()
{

	encAction0=enc0->count()/(encoderMAX2*diameter*M_PI);
	encAction1=enc1->count()/(encoderMAX1*diameter*M_PI);
	encAction2=enc2->count()/(encoderMAX1*diameter*M_PI);

	encAction0=encAction0*10000;
	encAction1=encAction1*10000;
	encAction2=encAction2*10000;

	encActionRemainder0=encAction0-encActionOld0;
	encActionRemainder1=encAction1-encActionOld1;
	encActionRemainder2=encAction2-encActionOld2;

	encActionOld0=encAction0;
	encActionOld1=encAction1;
	encActionOld2=encAction2;

	machineAngle=(encAction0+encAction1+encAction2)/3/long0;

    motor0X=(encActionRemainder0-encActionRemainder1)/(2*sin(M_PI/3));//0番と1番の交点
    motor0Y=(encActionRemainder0+encActionRemainder1)/(2*cos(M_PI/3));
    motor1X=(encActionRemainder0+(encActionRemainder2*cos(M_PI/3)))/sin(M_PI/3);//0番と2番の交点
    motor1Y=(-1)*encActionRemainder2;
    motor2X=(-1)*(encActionRemainder1+(encActionRemainder2*cos(M_PI/3)))/sin(M_PI/3);//１番と2番の交点
    motor2Y=(-1)*encActionRemainder2;

    machineX1=(motor0X+motor1X+motor2X)/3;//上記の3線のx座標の合計
    machineY1=(motor0Y+motor1Y+motor2Y)/3;//上記の3線のy座標の合計

    machineX+=machineX1*cos(machineAngle)-machineY1*sin(machineAngle);//5ミリ秒後の座標xと現在のx座標と足す
    machineY+=machineX1*sin(machineAngle)+machineY1*cos(machineAngle);//5ミリ秒後の座標yと現在のy座標を足す

}

int main()
{
	int time;
	//float enc0,enc1,enc2;

	Serial0 serial;
	serial.setup(115200);

	declaration declaration1;

	Can0 can;
	CanEncoder enc0(can , 0 , 5);
	CanEncoder enc1(can , 1 , 5);
	CanEncoder enc2(can , 2 , 5);

	EncoderUser enc(enc0,enc1,enc2);
	enc.setup();

	EncoderUser encuser(enc0,enc1,enc2);

	encuser.encoderMAX1=200;
	encuser.encoderMAX2=1000;
	encuser.diameter=30;
	encuser.long0=115;
	encuser.long1=115;
	encuser.long2=115;
	while (1){
		encuser.cycle();
		if (millis()-time>50)
		 	{
		 	    time=millis();
		 		//serial.printf("%d %d %d\n\r",canenc0.count(),canenc1.count(),canenc2.count());
		 		serial.printf("%f %f %f\n\r",encuser.machineX,encuser.machineY,encuser.machineAngle
		 				);
	        }
		if (encuser.machineX>=350)
		{
		}
	}
return 0;
}
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
