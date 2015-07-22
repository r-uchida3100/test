//libraries

//application

//controller

//base
#include "system.h"
#include "mcutime.h"
//board
#include "pin.hpp"

//circuit

#include "math.h"
#include "layer_controller/mini_md.hpp"

int main(void)
{
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
	float distance0,distance1,distance2,theta0,theta1,theta2,theta;
	float motor0X,motor0Y,motor1X,motor1Y,motor2X,motor2Y,machineX,machineY;
	float sin2=pow(sinf(M_PI/6),2);

	CW0 cw0;
	CCW0 ccw0;
	Pwm0 pwm0;
	MiniMD motor1(cw0,ccw0,pwm0);
	CW1 cw1;
	CCW1 ccw1;
	Pwm1 pwm1;
	MiniMD motor0(cw1,ccw1,pwm1);
	CW2 cw2;
	CCW2 ccw2;
	Pwm2 pwm2;
	MiniMD motor2(cw2,ccw2,pwm2);
	Enc0 enc0;enc0.setup();
	Enc1 enc1;enc1.setup();
	Enc2 enc2;enc2.setup();
	Sw0 sw0;sw0.setupDigitalOut();
	Serial0 serial;
	serial.setup(115200);

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
   	    //機体の角度
   	   	distance0=enc0.count()*(40*M_PI/200);
   	   	distance1=enc1.count()*(40*M_PI/200);
   	   	distance2=enc2.count()*(40*M_PI/200);
   	   	theta0=atan2(distance0,90.0);
   	   	theta1=atan2(distance1,90.0);
   	    theta2=atan2(distance2,90.0);
   	    theta=(theta0+theta1+theta2)/3;

   	    //自己位置もどき
   	   	//0番と1番のグラフの交点
   	   	motor0X=((distance1-distance0)*cosf(M_PI/6))/(2*sin2);
   	   	motor0Y=(distance1+distance0)/(2*sinf(M_PI/6));
   	   	//0番と2番のグラフの交点
   	   	motor1X=(-1)*((distance0+(distance2*sinf(M_PI/6)))*cosf(M_PI/6))/sin2;
   	   	motor1Y=(-1)*distance2;
   	   	//1番と2番のグラフの交点
   	    motor2X=(distance1+(distance2*sinf(M_PI/6)))*cosf(M_PI/6)/sin2;
   	   	motor2Y=(-1)*distance2;

   	    //3つの交点の平均
   	    machineX=(motor0X+motor1X+motor2X)/3;
   	    machineY=(motor0Y+motor1Y+motor2Y)/3;

        if (millis()-time>10)
	    {
	        time=millis();
	        serial.printf("%f,%f,%f,%f,%f\n\r",machineX,machineY,distance0,distance1,distance2);
	    }
    }
    return 0;
}
//(2000-走った距離)*0.0033333...
