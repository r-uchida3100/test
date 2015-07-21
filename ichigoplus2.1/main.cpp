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
	int time=0,b=0;
	float r0,r1,r2,theta0,theta1,theta2;
	float x0,y0,x1,y1,x2,y2,x,y;
	float c;
	float a=M_PI/6,sin2=pow(sinf(a),2);

	CW0 cw0;
	CCW0 ccw0;
	Pwm0 pwm0;
	MiniMD moter1(cw0,ccw0,pwm0);
	CW1 cw1;
	CCW1 ccw1;
	Pwm1 pwm1;
	MiniMD moter0(cw1,ccw1,pwm1);
	CW2 cw2;
	CCW2 ccw2;
	Pwm2 pwm2;
	MiniMD moter2(cw2,ccw2,pwm2);
	Enc0 enc0;enc0.setup();
	Enc1 enc1;enc1.setup();
	Enc2 enc2;enc2.setup();
	Sw0 sw0;sw0.setupDigitalOut();
	Serial0 serial;
	serial.setup(115200);

	moter0.setup();
	moter0.duty(-1.0);
	moter0.cycle();
	moter1.setup();
	moter1.duty(1.0);
	moter1.cycle();
	moter2.setup();
	moter2.duty(0.0);
	moter2.cycle();

   	while (1)
	{
   	    //機体の角度
   		r0=enc0.count()*(40*M_PI/200);
   		r1=enc1.count()*(40*M_PI/200);
   		r2=enc2.count()*(40*M_PI/200);
   	    theta0=atan2(r0,115.0);
   	    theta1=atan2(r1,115.0);
   	    theta2=atan2(r2,115.0);

   	    //0番と1番のグラフの交点
   	    x0=((r1-r0)*cosf(a))/(2*sin2);
   	    y0=(r1+r0)/(2*sinf(a));
   	    //0番と2番のグラフの交点
   	    x1=(-1)*((r0+(r2*sinf(a)))*cosf(a))/sin2;
   	    y1=(-1)*r2;
   	    //1番と2番のグラフの交点
   	    x2=(r1+(r2*sinf(a)))*cosf(a)/sin2;
   	    y2=(-1)*r2;

   	    x=(x0+x1+x2)/3; //3つの交点の平均
        y=(y0+y1+y2)/3;

        //(2000-走った距離)*0.0033333...

        if (millis()-time>33)
	    {
	        time=millis();
	        serial.printf("%f,%f,%f,%f,%f\n\r",x,y,r0,r1,r2);
	        wait (100);
	    }
    }
    return 0;
}

void KKD()
{

}
