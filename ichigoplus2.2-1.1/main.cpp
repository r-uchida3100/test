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
	float encoderMAX=200,diameter=40;//encoderMAX=鬩幢ｽ｢隴趣ｽ｢�ｽ�ｽ�ｽ�｢鬩幢ｽ｢隴趣ｽ｢�ｽ�ｽ�ｽ�ｼ鬩幢ｽ｢�ｽ�ｧ�ｽ�ｽ�ｽ�ｿ鬩幢ｽ｢隴趣ｽ｢�ｽ�ｽ�ｽ�ｼ鬮｣蛹�ｽｽ�ｳ�ｽ�ｽ�ｽ�ｽ髫ｰ蜊�ｿｽ�ｽ�ｸ�ｽ�ｺ�ｽ�ｽ�ｽ�ｮ鬩幢ｽ｢�ｽ�ｧ�ｽ�ｽ�ｽ�ｨ鬩幢ｽ｢隴趣ｽ｢�ｽ�ｽ�ｽ�ｳ鬩幢ｽ｢�ｽ�ｧ�ｽ�ｽ�ｽ�ｳ鬩幢ｽ｢隴趣ｽ｢�ｽ�ｽ�ｽ�ｼ鬩幢ｽ｢隴趣ｽ｢�ｽ�ｿ�ｽ�ｽ�ｽ�ｽ�ｽ�ｽ鬩搾ｽｵ�ｽ�ｺ�ｽ�ｽ�ｽ�ｮ鬮ｯ蛹ｺ�ｻ繧托ｽｽ�ｽ�ｽ�､,diameter=鬩幢ｽ｢�ｽ�ｧ�ｽ�ｽ�ｽ�ｿ鬩幢ｽ｢�ｽ�ｧ�ｽ�ｽ�ｽ�､鬩幢ｽ｢隴趣ｽ｢�ｽ�ｽ�ｽ�､鬩搾ｽｵ�ｽ�ｺ�ｽ�ｽ�ｽ�ｮ鬯ｨ�ｾ�ｽ�ｶ�ｽ�ｽ�ｽ�ｴ鬮ｯ貅ｷ譯�ｿｽ�ｿ�ｽ�ｽ	enc0.setup();
	enc1.setup();
	enc2.setup();
	while (1)
	{
		distance0=enc0.count()*(diameter*M_PI/encoderMAX);
	    distance1=enc1.count()*(diameter*M_PI/encoderMAX);
   	    distance2=enc2.count()*(diameter*M_PI/encoderMAX);
   	    //鬯ｮ�｢�ｽ�ｾ�ｽ�ｽ�ｽ�ｪ鬮ｯ譎｢�ｽ�ｾ�ｽ�ｽ�ｽ�ｱ鬮｣蜴�ｽｽ�ｴ髯ｷ�･�ｽ�ｲ�ｽ�ｽ�ｽ�ｽ�ｽ�ｽ�ｽ�ｮ
   	    //0鬯ｨ�ｾ�ｽ�｡�ｽ�ｽ�ｽ�ｪ鬩搾ｽｵ�ｽ�ｺ�ｽ�ｽ�ｽ�ｨ1鬯ｨ�ｾ�ｽ�｡�ｽ�ｽ�ｽ�ｪ鬩搾ｽｵ�ｽ�ｺ�ｽ�ｽ�ｽ�ｮ鬩幢ｽ｢�ｽ�ｧ�ｽ�ｽ�ｽ�ｰ鬩幢ｽ｢隴趣ｽ｢�ｽ�ｽ�ｽ�ｩ鬩幢ｽ｢隴趣ｿｽ�ｽ�ｼ髮具ｽｻ�ｽ�ｿ�ｽ�ｽ鬮｣雋ｻ�ｿ�ｽ�ｽ�ｽ�ｽ�､鬮ｴ髮｣�ｽ�､�ｽ�ｽ�ｽ�ｹ
  	    motor0X=(distance1-distance0)/(2*sin(M_PI/3));
   	    motor0Y=(distance1+distance0)/(2*cos(M_PI/3));
   	    //0鬯ｨ�ｾ�ｽ�｡�ｽ�ｽ�ｽ�ｪ鬩搾ｽｵ�ｽ�ｺ�ｽ�ｽ�ｽ�ｨ2鬯ｨ�ｾ�ｽ�｡�ｽ�ｽ�ｽ�ｪ鬩搾ｽｵ�ｽ�ｺ�ｽ�ｽ�ｽ�ｮ鬩幢ｽ｢�ｽ�ｧ�ｽ�ｽ�ｽ�ｰ鬩幢ｽ｢隴趣ｽ｢�ｽ�ｽ�ｽ�ｩ鬩幢ｽ｢隴趣ｿｽ�ｽ�ｼ髮具ｽｻ�ｽ�ｿ�ｽ�ｽ鬮｣雋ｻ�ｿ�ｽ�ｽ�ｽ�ｽ�､鬮ｴ髮｣�ｽ�､�ｽ�ｽ�ｽ�ｹ
   	    motor1X=(-1)*(distance0+(distance2*cos(M_PI/3)))/sin(M_PI/3);
   	    motor1Y=(-1)*distance2;
   	    //1鬯ｨ�ｾ�ｽ�｡�ｽ�ｽ�ｽ�ｪ鬩搾ｽｵ�ｽ�ｺ�ｽ�ｽ�ｽ�ｨ2鬯ｨ�ｾ�ｽ�｡�ｽ�ｽ�ｽ�ｪ鬩搾ｽｵ�ｽ�ｺ�ｽ�ｽ�ｽ�ｮ鬩幢ｽ｢�ｽ�ｧ�ｽ�ｽ�ｽ�ｰ鬩幢ｽ｢隴趣ｽ｢�ｽ�ｽ�ｽ�ｩ鬩幢ｽ｢隴趣ｿｽ�ｽ�ｼ髮具ｽｻ�ｽ�ｿ�ｽ�ｽ鬮｣雋ｻ�ｿ�ｽ�ｽ�ｽ�ｽ�､鬮ｴ髮｣�ｽ�､�ｽ�ｽ�ｽ�ｹ
   	    motor2X=(distance1+(distance2*cos(M_PI/3)))/sin(M_PI/3);
   	    motor2Y=(-1)*distance2;

   	    //3鬩搾ｽｵ�ｽ�ｺ�ｽ�ｽ�ｽ�､鬩搾ｽｵ�ｽ�ｺ�ｽ�ｽ�ｽ�ｮ鬮｣雋ｻ�ｿ�ｽ�ｽ�ｽ�ｽ�､鬮ｴ髮｣�ｽ�､�ｽ�ｽ�ｽ�ｹ鬩搾ｽｵ�ｽ�ｺ�ｽ�ｽ�ｽ�ｮ鬮ｯ譎｢�ｽ�ｷ�ｽ�ｽ�ｽ�ｳ鬮ｯ諛ｶ�ｽ�ｮ�ｽ�ｽ�ｽ�ｽ   	    machineX=(motor0X+motor1X+motor2X)/3;
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
	float encoderMAX1=200;//encoderMAX1=驛｢譎｢�ｽ�｢驛｢譎｢�ｽ�ｼ驛｢�ｧ�ｽ�ｿ驛｢譎｢�ｽ�ｼ驕ｶ�ｭ�ｽ�ｽ髮崎ご�ｸ�ｺ�ｽ�ｮ髣包ｽｳ�ｽ�ｽ隰占��ｸ�ｺ�ｽ�ｮ驛｢�ｧ�ｽ�ｨ驛｢譎｢�ｽ�ｳ驛｢�ｧ�ｽ�ｳ驛｢譎｢�ｽ�ｼ驛｢譎｢�ｿ�ｽ�ｽ�ｽ驛｢�ｧ�ｽ�ｫ驛｢�ｧ�ｽ�ｦ驛｢譎｢�ｽ�ｳ驛｢譎｢�ｿ�ｽ
	float encoderMAX2=1000;//encoderMAX2=驛｢譎｢�ｽ�｢驛｢譎｢�ｽ�ｼ驛｢�ｧ�ｽ�ｿ驛｢譎｢�ｽ�ｼ髫ｨ繧托ｽｽ�ｪ驍ｵ�ｺ�ｽ�ｮ髣包ｽｳ�ｽ�ｽ隰占��ｸ�ｺ�ｽ�ｮ驛｢�ｧ�ｽ�ｨ驛｢譎｢�ｽ�ｳ驛｢�ｧ�ｽ�ｳ驛｢譎｢�ｽ�ｼ驛｢譎｢�ｿ�ｽ�ｽ�ｽ驛｢�ｧ�ｽ�ｫ驛｢�ｧ�ｽ�ｦ驛｢譎｢�ｽ�ｳ驛｢譎｢�ｿ�ｽ
	float diameter=30;//diameter=鬩幢ｽ｢�ｽ�ｧ�ｽ�ｽ�ｽ�ｿ鬩幢ｽ｢�ｽ�ｧ�ｽ�ｽ�ｽ�､鬩幢ｽ｢隴趣ｽ｢�ｽ�ｽ�ｽ�､鬩搾ｽｵ�ｽ�ｺ�ｽ�ｽ�ｽ�ｮ鬯ｨ�ｾ�ｽ�ｶ�ｽ�ｽ�ｽ�ｴ鬮ｯ貅ｷ譯�ｿｽ�ｿ�ｽ�ｽ
	float long0=115;//鬩幢ｽ｢�ｽ�ｧ�ｽ�ｽ�ｽ�ｿ鬩幢ｽ｢�ｽ�ｧ�ｽ�ｽ�ｽ�､鬩幢ｽ｢隴趣ｽ｢�ｽ�ｽ�ｽ�､�ｽ�ｽ�ｽ�ｽ鬮｣髮�ｿｽ�ｽ�ｰ鬩幢ｽ｢�ｽ�ｧ鬮｣�ｽ�ｽ�ｽ�ｽ�ｽ�ｽ�ｻ鬮ｮ讎奇ｿｽ�ｽ�ｽ�ｽ�ｽ鬮ｦ�ｮ陷ｻ�ｻ�ｽ�ｿ�ｽ�ｽ鬮｣蛹�ｽｽ�ｳ�ｽ�ｽ�ｽ�ｭ鬮ｯ貊ゑｽｽ�｢�ｽ�ｽ�ｽ�ｽ驕ｶ謫ｾ�ｽ�ｪ鬩搾ｽｵ�ｽ�ｺ�ｽ�ｽ�ｽ�ｧ鬩搾ｽｵ�ｽ�ｺ�ｽ�ｽ�ｽ�ｮ鬯ｮ�ｴ髢ｧ�ｴ髴趣ｽｨ髯橸ｽｻ�ｽ�ｬ
	float long1=115;//鬩幢ｽ｢�ｽ�ｧ�ｽ�ｽ�ｽ�ｿ鬩幢ｽ｢�ｽ�ｧ�ｽ�ｽ�ｽ�､鬩幢ｽ｢隴趣ｽ｢�ｽ�ｽ�ｽ�､�ｽ�ｽ�ｽ�ｽ髣比ｼ夲ｽｽ�｣�ｽ繧托ｽｽ�ｰ鬩幢ｽ｢�ｽ�ｧ鬮｣�ｽ�ｽ�ｽ�ｽ�ｽ�ｽ�ｻ鬮ｮ讎奇ｿｽ�ｽ�ｽ�ｽ�ｽ鬮ｦ�ｮ陷ｻ�ｻ�ｽ�ｿ�ｽ�ｽ鬮｣蛹�ｽｽ�ｳ�ｽ�ｽ�ｽ�ｭ鬮ｯ貊ゑｽｽ�｢�ｽ�ｽ�ｽ�ｽ驕ｶ謫ｾ�ｽ�ｪ鬩搾ｽｵ�ｽ�ｺ�ｽ�ｽ�ｽ�ｧ鬩搾ｽｵ�ｽ�ｺ�ｽ�ｽ�ｽ�ｮ鬯ｮ�ｴ髢ｧ�ｴ髴趣ｽｨ髯橸ｽｻ�ｽ�ｬ
	float long2=115;//鬩幢ｽ｢�ｽ�ｧ�ｽ�ｽ�ｽ�ｿ鬩幢ｽ｢�ｽ�ｧ�ｽ�ｽ�ｽ�､鬩幢ｽ｢隴趣ｽ｢�ｽ�ｽ�ｽ�､�ｽ�ｽ�ｽ�ｽ髯句ｹ｢�ｽ�ｵ�ｽ繧托ｽｽ�ｰ鬩幢ｽ｢�ｽ�ｧ鬮｣�ｽ�ｽ�ｽ�ｽ�ｽ�ｽ�ｻ鬮ｮ讎奇ｿｽ�ｽ�ｽ�ｽ�ｽ鬮ｦ�ｮ陷ｻ�ｻ�ｽ�ｿ�ｽ�ｽ鬮｣蛹�ｽｽ�ｳ�ｽ�ｽ�ｽ�ｭ鬮ｯ貊ゑｽｽ�｢�ｽ�ｽ�ｽ�ｽ驕ｶ謫ｾ�ｽ�ｪ鬩搾ｽｵ�ｽ�ｺ�ｽ�ｽ�ｽ�ｧ鬩搾ｽｵ�ｽ�ｺ�ｽ�ｽ�ｽ�ｮ鬯ｮ�ｴ髢ｧ�ｴ髴趣ｽｨ髯橸ｽｻ�ｽ�ｬ
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
	    //鬯ｮ�ｴ�ｽ�ｽ�ｽ�ｴ陷茨ｽｷ�ｽ�ｽ�ｽ�ｽ鬮ｦ�ｮ陷ｻ�ｻ�ｽ�ｿ�ｽ�ｽ鬯ｮ�ｫ髣埼屮�ｽ�ｲ隶厄ｽｸ�ｽ�ｽ�ｽ�ｺ�ｽ�ｽ�ｽ�ｦ
	   	theta0=atan2(distance0,long0);
	   	theta1=atan2(distance1,long1);
	   	theta2=atan2(distance2,long2);
	   	theta=(theta0+theta1+theta2)/3;
	   	//鬯ｮ�｢�ｽ�ｾ�ｽ�ｽ�ｽ�ｪ鬮ｯ譎｢�ｽ�ｾ�ｽ�ｽ�ｽ�ｱ鬮｣蜴�ｽｽ�ｴ髯ｷ�･�ｽ�ｲ�ｽ�ｽ�ｽ�ｽ�ｽ�ｽ�ｽ�ｮ
	   	//0鬯ｨ�ｾ�ｽ�｡�ｽ�ｽ�ｽ�ｪ鬩搾ｽｵ�ｽ�ｺ�ｽ�ｽ�ｽ�ｨ1鬯ｨ�ｾ�ｽ�｡�ｽ�ｽ�ｽ�ｪ鬩搾ｽｵ�ｽ�ｺ�ｽ�ｽ�ｽ�ｮ鬩幢ｽ｢�ｽ�ｧ�ｽ�ｽ�ｽ�ｰ鬩幢ｽ｢隴趣ｽ｢�ｽ�ｽ�ｽ�ｩ鬩幢ｽ｢隴趣ｿｽ�ｽ�ｼ髮具ｽｻ�ｽ�ｿ�ｽ�ｽ鬮｣雋ｻ�ｿ�ｽ�ｽ�ｽ�ｽ�､鬮ｴ髮｣�ｽ�､�ｽ�ｽ�ｽ�ｹ
	   	motor0X=(distance0-distance1)/(2*sin(M_PI/3));
	   	motor0Y=(distance0+distance1)/(2*cos(M_PI/3));
	   	//0鬯ｨ�ｾ�ｽ�｡�ｽ�ｽ�ｽ�ｪ鬩搾ｽｵ�ｽ�ｺ�ｽ�ｽ�ｽ�ｨ2鬯ｨ�ｾ�ｽ�｡�ｽ�ｽ�ｽ�ｪ鬩搾ｽｵ�ｽ�ｺ�ｽ�ｽ�ｽ�ｮ鬩幢ｽ｢�ｽ�ｧ�ｽ�ｽ�ｽ�ｰ鬩幢ｽ｢隴趣ｽ｢�ｽ�ｽ�ｽ�ｩ鬩幢ｽ｢隴趣ｿｽ�ｽ�ｼ髮具ｽｻ�ｽ�ｿ�ｽ�ｽ鬮｣雋ｻ�ｿ�ｽ�ｽ�ｽ�ｽ�､鬮ｴ髮｣�ｽ�､�ｽ�ｽ�ｽ�ｹ
	   	motor1X=(distance0+(distance2*cos(M_PI/3)))/sin(M_PI/3);
	   	motor1Y=(-1)*distance2;
	   	//1鬯ｨ�ｾ�ｽ�｡�ｽ�ｽ�ｽ�ｪ鬩搾ｽｵ�ｽ�ｺ�ｽ�ｽ�ｽ�ｨ2鬯ｨ�ｾ�ｽ�｡�ｽ�ｽ�ｽ�ｪ鬩搾ｽｵ�ｽ�ｺ�ｽ�ｽ�ｽ�ｮ鬩幢ｽ｢�ｽ�ｧ�ｽ�ｽ�ｽ�ｰ鬩幢ｽ｢隴趣ｽ｢�ｽ�ｽ�ｽ�ｩ鬩幢ｽ｢隴趣ｿｽ�ｽ�ｼ髮具ｽｻ�ｽ�ｿ�ｽ�ｽ鬮｣雋ｻ�ｿ�ｽ�ｽ�ｽ�ｽ�､鬮ｴ髮｣�ｽ�､�ｽ�ｽ�ｽ�ｹ
	   	motor2X=(-1)*(distance1+(distance2*cos(M_PI/3)))/sin(M_PI/3);
	   	motor2Y=(-1)*distance2;
   		//3鬩搾ｽｵ�ｽ�ｺ�ｽ�ｽ�ｽ�､鬩搾ｽｵ�ｽ�ｺ�ｽ�ｽ�ｽ�ｮ鬮｣雋ｻ�ｿ�ｽ�ｽ�ｽ�ｽ�､鬮ｴ髮｣�ｽ�､�ｽ�ｽ�ｽ�ｹ鬩搾ｽｵ�ｽ�ｺ�ｽ�ｽ�ｽ�ｮ鬮ｯ譎｢�ｽ�ｷ�ｽ�ｽ�ｽ�ｳ鬮ｯ諛ｶ�ｽ�ｮ�ｽ�ｽ�ｽ�ｽ


	   	machineX=(motor0X+motor1X+motor2X)/3;
   		machineY=(motor0Y+motor1Y+motor2Y)/3;
   		//X,Y鬮ｯ貅ｯ�ｶ�｣�ｽ�ｽ�ｽ�ｧ鬮ｫ�ｶ霓｣菫ｶ邏費ｿｽ�ｽ�ｽ�ｻ�ｽ�ｽ�ｽ�ｸ鬩幢ｽ｢�ｽ�ｧ鬮ｮ蛹ｺ�ｧ�ｫ�ｽ�ｴ驍�戟�･諛ｶ�ｿ�ｽ�ｽ�ｽ
   		machineX=machineX*cos(theta);
   		machineY=machineY*sin(theta);

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
   		if (millis()-time>20)
   		{
   		    time=millis();
   			serial.printf("%f,%f,%f,%f,%f,%f\n\r",machineX,machineY,theta,enc0.count(),enc1.count(),enc2.count());
        }
     }
return 0;
}
