//libraries
#include "math.h"

//application

//controller
#include "layer_controller/blink.hpp"
//base
#include "system.h"
#include "mcutime.h"
#include "software_reset.h"
//board
#include "pin.hpp"

//circuit
#include "layer_driver/circuit_drivers/can_encoder/can_encoder.hpp"
#include "layer_driver/circuit_drivers/mini_md_1.0/mini_md.hpp"

//driver

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
/*
class Control
{
public:
	Enc0 enc0;
	Enc1 enc1;
	Enc2 enc2;
	CW0 cw0;
	CW1 cw1;
	CW2 cw2;
	CCW0 ccw0;
	CCW1 ccw1;
	CCW2 ccw2;
	Pwm0 pwm0;
	Pwm1 pwm1;
	Pwm2 pwm2;
	Sw0 sw0;
	Sw1 sw1;
	Sw2 sw2;
	Sw3 sw3;

	Control();
	int count;
	int time1;
};

Control::Control(){
	MiniMD motor0(cw0,ccw0,pwm0);
	MiniMD motor1(cw1,ccw1,pwm1);
	MiniMD motor2(cw2,ccw2,pwm2);

	motor0.setup();
	motor1.setup();
	motor2.setup();
	enc0.setup();
	enc1.setup();
	enc2.setup();
	sw0.setupDigitalIn();
	sw1.setupDigitalIn();
	sw2.setupDigitalIn();
	sw3.setupDigitalIn();
}
*/
/*
class MiniMDuser{
public:

	MiniMDuser(MiniMD &motor_0,MiniMD &motor_1,MiniMD &motor_2){
		motor0=&motor_0;
		motor1=&motor_1;
		motor2=&motor_2;
	}
	int setup()
	{
		int j;
		j+=(motor0->setup()!=0);
		j+=(motor1->setup()!=0);
		j+=(motor2->setup()!=0);
		return j;
	}
private:
	MiniMD *motor0;
	MiniMD *motor1;
	MiniMD *motor2;
};
*/
/*
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
	float machineX;
	float machineY;
	float last0,last1,last2;
	float encAction0,encAction1,encAction2;
	float encActionOld0,encActionOld1,encActionOld2;
	float encActionRemainder0,encActionRemainder1,encActionRemainder2;
	float machineAngle;
	float EncRemainder0,EncRemainder1,EncRemainder2;
	float EncOld0,EncOld1,EncOld2;
	float EncDistance0,EncDistance1,EncDistance2;

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

	encAction0=diameter*M_PI*enc0->count()/encoderMAX2;
	encAction1=diameter*M_PI*enc1->count()/encoderMAX1;
	encAction2=diameter*M_PI*enc2->count()/encoderMAX1;

	encActionRemainder0=encAction0-encActionOld0;
	encActionRemainder1=encAction1-encActionOld1;
	encActionRemainder2=encAction2-encActionOld2;

	encActionOld0=encAction0;
	encActionOld1=encAction1;
	encActionOld2=encAction2;

	machineAngle=(encAction0+encAction1+encAction2)/3/long0;

    motor0X=(-1)*(encActionRemainder0-encActionRemainder1)/(2*sin(M_PI/3));//0番と1番の交点
    motor0Y=(-1)*(encActionRemainder0+encActionRemainder1)/(2*cos(M_PI/3));
    motor1X=(-1)*(encActionRemainder0+(encActionRemainder2*cos(M_PI/3)))/sin(M_PI/3);//0番と2番の交点
    motor1Y=encActionRemainder2;
    motor2X=(encActionRemainder1+(encActionRemainder2*cos(M_PI/3)))/sin(M_PI/3);//１番と2番の交点
    motor2Y=encActionRemainder2;

    machineX1=(motor0X+motor1X+motor2X)/3;//上記の3線のx座標の合計
    machineY1=(motor0Y+motor1Y+motor2Y)/3;//上記の3線のy座標の合計

    machineX+=machineX1*cos(machineAngle)-machineY1*sin(machineAngle);//5ミリ秒後の座標xと現在のx座標と足す
    machineY+=machineX1*sin(machineAngle)+machineY1*cos(machineAngle);//5ミリ秒後の座標yと現在のy座標を足す

    //machineX=machineX*5/4;
    //machineY=machineY*5/4;
}

class Omni0{
public:
	Omni0(MiniMD &MD0,MiniMD &MD1,MiniMD &MD2);
	int setup();
	void cycle();

	void order(float direction,float speed,float rotation);
};

Omni0::Omni0(MiniMD &MD0,MiniMD &MD1,MiniMD &MD2)
{}

void cycle()
{

}

void Omni0::order(float derection,float speed,float rotation)
{}

class Move{
public:
	float Output;
	float Output_Angle;
	Move(Omni0 &Omni,EncoderUser &User);
	float requestX,requestY;
	int setup();
	void request(float coordinateX,float coordinateY,float Angle);
	void PD_control(float Present_value,float Target_value,float P_gain,float D_gain);
	void lock();

	void stop();
	int busy();
	//float PD_control(float Present_value,float Target_value,float P_gain,D_gain);
	void cycle(float Present_Angle,float Target_Angle,float P_gain_Angle,float D_gain_Angle);
};

Move::Move(Omni0 &Omni,EncoderUser &User)
{}

void Move::PD_control(float Present_value,float Target_value,float P_gain,float D_gain)
{
	float Deviation=0.0;
	static float Deviation_Old=0.0;

	Deviation = Target_value - Present_value;

	Output = (P_gain*Deviation)+(D_gain*(Deviation - Deviation_Old));

	Deviation_Old = Deviation;
}

int Move::setup()
{}

void Move::cycle(float Present_Angle,float Target_Angle,float P_gain_Angle,float D_gain_Angle)
{
	//EncoderUser encuser;

	float Deviation_Angle;
	static float Deviation_Angle_Old;

	Deviation_Angle = Target_Angle - Present_Angle;

	Output_Angle = (P_gain_Angle*Deviation_Angle)+(D_gain_Angle*(Deviation_Angle-Deviation_Angle_Old));

	Deviation_Angle_Old=Deviation_Angle;
}

void Move::request(float coordinateX,float coordinateY,float Angle)
{
	float coordinateXdash=0.0,coordinateYdash=0.0;
	float differenceX=0.0,differenceY=0.0;
	coordinateXdash=coordinateX*0.8;
	differenceX=coordinateX-coordinateXdash;
	requestX=differenceX*0.0025;
	if (requestX>1.0)
	{
		requestX=1.0;
	}

	coordinateYdash=coordinateY*0.8;
	differenceY=coordinateY-coordinateYdash;
	requestY=differenceY*0.005;
	if (requestY>1.0)
	{
		requestY=1.0;
	}
}

void Move::stop()
{}

int Move::busy()
{}

void Move::lock()
{}
*/
int main()
{
	int time=0;
	float encoderMAX1=200;
	float encoderMAX2=1000;
	float diameter=30;
	float long0=115;
	float motor0X=0;
	float motor0Y=0;
	float motor1X=0;
	float motor1Y=0;
	float motor2X=0;
	float motor2Y=0;
	float machineX1=0;
	float machineY1=0;
	float machineX=0;
	float machineY=0;
	float encAction0=0,encAction1=0,encAction2=0;
	float encActionOld0=0,encActionOld1=0,encActionOld2=0;
	float encActionRemainder0=0,encActionRemainder1=0,encActionRemainder2=0;
	float machineAngle=0;

	//float enc0,enc1,enc2;


	Serial0 serial;
	serial.setup(115200);


/*
	MiniMDuser MDuser(motor0,motor1,motor2);

	MiniMD motor0(MDuser.cw0,MDuser.ccw0,MDuser.pwm0);
	MiniMD motor1(MDuser.cw1,MDuser.ccw1,MDuser.pwm1);
	MiniMD motor2(MDuser.cw2,MDuser.ccw2,MDuser.pwm2);

	MiniMDuser motor(motor0,motor1,motor2);
	motor.setup();
*/
	Can0 can;
	CanEncoder enc0(can , 0 , 5);
	CanEncoder enc1(can , 1 , 5);
	CanEncoder enc2(can , 2 , 5);

	//EncoderUser enc(enc0,enc1,enc2);
	//enc.setup();

	//EncoderUser encuser(enc0,enc1,enc2);

	CW0 cw0;
	CW1 cw1;
	CW2 cw2;
	CCW0 ccw0;
	CCW1 ccw1;
	CCW2 ccw2;
	Pwm0 pwm0;
	Pwm1 pwm1;
	Pwm2 pwm2;

	MiniMD motor0(cw0,ccw0,pwm0);
	MiniMD motor1(cw1,ccw1,pwm1);
	MiniMD motor2(cw2,ccw2,pwm2);

	motor0.setup();
	motor1.setup();
	motor2.setup();

	/*
	encuser.encoderMAX1=200;
	encuser.encoderMAX2=1000;
	encuser.diameter=30;
	encuser.long0=115;
	encuser.long1=115;
	encuser.long2=115;
	encuser.motor0X=0;
	encuser.motor0Y=0;
	encuser.motor1X=0;
	encuser.motor1Y=0;
	encuser.motor2X=0;
	encuser.motor2Y=0;
	encuser.machineX1=0;
	encuser.machineY1=0;
	encuser.machinex1=0;
	encuser.machiney1=0;
	encuser.machinex2=0;
	encuser.machiney2=0;
	encuser.machinex=0;
	encuser.machiney=0;
	encuser.machineX=0;
	encuser.machineY=0;
	encuser.last0=0;
	encuser.last1=0;
	encuser.last2=0;
	encuser.encAction0=0;
	encuser.encAction1=0;
	encuser.encAction2=0;
	encuser.encActionOld0=0;
	encuser.encActionOld1=0;
	encuser.encActionOld2=0;
	encuser.encActionRemainder0=0;
	encuser.encActionRemainder1=0;
	encuser.encActionRemainder2=0;
	encuser.machineAngle=0;

	Omni0 omni0(motor0,motor1,motor2);

	Move move(omni0,encuser);
	move.setup();
	move.Output=0;
	move.Output_Angle;

	move.request(2000,0,0);
	*/
	while (1){
		if (millis()-time>50)
		{
			time=millis();

			encAction0=diameter*M_PI*enc0.count()/encoderMAX2;
			encAction1=diameter*M_PI*enc1.count()/encoderMAX1;
			encAction2=diameter*M_PI*enc2.count()/encoderMAX1;

			encActionRemainder0=encAction0-encActionOld0;
			encActionRemainder1=encAction1-encActionOld1;
			encActionRemainder2=encAction2-encActionOld2;

			encActionOld0=encAction0;
			encActionOld1=encAction1;
			encActionOld2=encAction2;

			machineAngle=(encAction0+encAction1+encAction2)/3/long0;

		    motor0X=(-1)*(encActionRemainder0-encActionRemainder1)/(2*sin(M_PI/3));//0番と1番の交点
		    motor0Y=(-1)*(encActionRemainder0+encActionRemainder1)/(2*cos(M_PI/3));
		    motor1X=(-1)*(encActionRemainder0+(encActionRemainder2*cos(M_PI/3)))/sin(M_PI/3);//0番と2番の交点
		    motor1Y=encActionRemainder2;
		    motor2X=(encActionRemainder1+(encActionRemainder2*cos(M_PI/3)))/sin(M_PI/3);//１番と2番の交点
		    motor2Y=encActionRemainder2;

		    machineX1=(motor0X+motor1X+motor2X)/3;//上記の3線のx座標の合計
		    machineY1=(motor0Y+motor1Y+motor2Y)/3;//上記の3線のy座標の合計

		    machineX+=machineX1*cos(machineAngle)-machineY1*sin(machineAngle);//5ミリ秒後の座標xと現在のx座標と足す
		    machineY+=machineX1*sin(machineAngle)+machineY1*cos(machineAngle);//5ミリ秒後の座標yと現在のy座標を足す

		    motor0.duty(0.0);
		    motor1.duty(0.0);
		    motor2.duty(0.0);
		    motor0.cycle();
		    motor1.cycle();
		    motor2.cycle();

		    if (machineX>1600)
		    {

		    }

			/*if(!move.busy()) move.stop();
			encuser.cycle();
			move.cycle(encuser.machineAngle,0,1.0,1.0);
			move.PD_control(0.0,0.0,0.0,0.0);
			omni0.order(0.0,move.Output,move.Output_Angle);*/
			//serial.printf("%d %d %d\n\r",enc0.count(),enc1.count(),enc2.count());
			serial.printf("%f %f %f\n\r",machineX,machineY,machineAngle);
		}
	}
	/*
    while (1)
    {
	if (control.sw0.digitalRead()==0)
	{

	}
    }
*/return 0;
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
