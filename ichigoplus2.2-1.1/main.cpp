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
#include "layer_driver/circuit/can_encoder.hpp"
#include "layer_driver/circuit/mini_md.hpp"

//driver
#include "layer_driver/base/can.hpp"
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
	float encoderMAX=200,diameter=40;//encoderMAX=繝｢繝ｼ繧ｿ繝ｼ荳�捉縺ｮ繧ｨ繝ｳ繧ｳ繝ｼ繝��縺ｮ蛟､,diameter=繧ｿ繧､繝､縺ｮ逶ｴ蠕�	enc0.setup();
	enc1.setup();
	enc2.setup();
	while (1)
	{
		distance0=enc0.count()*(diameter*M_PI/encoderMAX);
	    distance1=enc1.count()*(diameter*M_PI/encoderMAX);
   	    distance2=enc2.count()*(diameter*M_PI/encoderMAX);
   	    //閾ｪ蟾ｱ菴咲ｽｮ
   	    //0逡ｪ縺ｨ1逡ｪ縺ｮ繧ｰ繝ｩ繝輔�莠､轤ｹ
  	    motor0X=(distance1-distance0)/(2*sin(M_PI/3));
   	    motor0Y=(distance1+distance0)/(2*cos(M_PI/3));
   	    //0逡ｪ縺ｨ2逡ｪ縺ｮ繧ｰ繝ｩ繝輔�莠､轤ｹ
   	    motor1X=(-1)*(distance0+(distance2*cos(M_PI/3)))/sin(M_PI/3);
   	    motor1Y=(-1)*distance2;
   	    //1逡ｪ縺ｨ2逡ｪ縺ｮ繧ｰ繝ｩ繝輔�莠､轤ｹ
   	    motor2X=(distance1+(distance2*cos(M_PI/3)))/sin(M_PI/3);
   	    motor2Y=(-1)*distance2;

   	    //3縺､縺ｮ莠､轤ｹ縺ｮ蟷ｳ蝮�   	    machineX=(motor0X+motor1X+motor2X)/3;
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
/*float motorPDprogram(float current,float command,float Pgain,float Dgain)
{
	float output=0.00;
	float deviation=0.00;
	static float olddiviation=0.00;

	deviation=

}*/

int main()
{
	Can0 can;
	CanEncoder enc0(can,0,5);
	CanEncoder enc1(can,1,5);
	CanEncoder enc2(can,2,5);
	CW0 cw0;
	CCW0 ccw0;
	Pwm0 pwm0;
	CW1 cw1;
	CCW1 ccw1;
	Pwm1 pwm1;
	CW2 cw2;
	CCW2 ccw2;
	Pwm2 pwm2;
	Serial0 serial;
	MiniMD motor0(cw0,ccw0,pwm0);
	MiniMD motor1(cw1,ccw1,pwm1);
	MiniMD motor2(cw2,ccw2,pwm2);

	motor0.setup();
	motor0.duty(0.0);
	motor0.cycle();
	motor1.setup();
	motor1.duty(0.0);
	motor1.cycle();
	motor2.setup();
	motor2.duty(0.0);
	motor2.cycle();
	enc0.setup();
	enc1.setup();
	enc2.setup();
	serial.setup(115200);

	int time=0;
	float encoderMAX1=200;//encoderMAX1=モーター①②の一周のエンコーダーカウント
	float encoderMAX2=1000;//encoderMAX2=モーター⓪の一周のエンコーダーカウント
	float diameter=30;//diameter=繧ｿ繧､繝､縺ｮ逶ｴ蠕�
	float long0=115;//繧ｿ繧､繝､�舌°繧芽ｻ贋ｽ薙�荳ｭ蠢�∪縺ｧ縺ｮ霍晞屬
	float long1=115;//繧ｿ繧､繝､�代°繧芽ｻ贋ｽ薙�荳ｭ蠢�∪縺ｧ縺ｮ霍晞屬
	float long2=115;//繧ｿ繧､繝､�偵°繧芽ｻ贋ｽ薙�荳ｭ蠢�∪縺ｧ縺ｮ霍晞屬
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
	float theta0,theta1,theta2,theta;
	float i,j;

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
	while (1)
	{
	   	distance0=enc0.count()*(diameter*M_PI/encoderMAX2);
	   	distance1=enc1.count()*(diameter*M_PI/encoderMAX1);
	   	distance2=enc2.count()*(diameter*M_PI/encoderMAX1);
	    //霆贋ｽ薙�隗貞ｺｦ
	   	theta0=atan2(distance0,long0);
	   	theta1=atan2(distance1,long1);
	   	theta2=atan2(distance2,long2);
	   	theta=(theta0+theta1+theta2)/3;
	   	//閾ｪ蟾ｱ菴咲ｽｮ
	   	//0逡ｪ縺ｨ1逡ｪ縺ｮ繧ｰ繝ｩ繝輔�莠､轤ｹ
	   	motor0X=(distance0-distance1)/(2*sin(M_PI/3));
	   	motor0Y=(distance0+distance1)/(2*cos(M_PI/3));
	   	//0逡ｪ縺ｨ2逡ｪ縺ｮ繧ｰ繝ｩ繝輔�莠､轤ｹ
	   	motor1X=(distance0+(distance2*cos(M_PI/3)))/sin(M_PI/3);
	   	motor1Y=(-1)*distance2;
	   	//1逡ｪ縺ｨ2逡ｪ縺ｮ繧ｰ繝ｩ繝輔�莠､轤ｹ
	   	motor2X=(-1)*(distance1+(distance2*cos(M_PI/3)))/sin(M_PI/3);
	   	motor2Y=(-1)*distance2;
   		//3縺､縺ｮ莠､轤ｹ縺ｮ蟷ｳ蝮�   		machineX=(motor0X+motor1X+motor2X)/3;
   		machineY=(motor0Y+motor1Y+motor2Y)/3;
   		//X,Y蠎ｧ讓呵ｻｸ繧貞崋螳�   		machineX=machineX*cos(theta);
   		machineY=machineY*sin(theta);

   		if (machineX>=1500)
   		{
   			i=2000-machineX;
   			j=i*0.002;
   			if (j>=1)
   			{
   				j=1;
   				motor0.duty(j);
   				motor1.duty(-j);
   			}
   		}
   		if (millis()-time>20)
   		{
   		    time=millis();
   			serial.printf("%f,%f %f\n\r",machineX,machineY,theta);
        }
     }
return 0;
}
//(2000-襍ｰ縺｣縺溯ｷ晞屬)*0.0033333...
