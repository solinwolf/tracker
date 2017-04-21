
#include <stdio.h>
#include "pid_controler.h"
#include <math.h>
pid_controler::pid_controler(ros::NodeHandle& nh,float setdistance,float actualdistance){ 
	x=0;
	y=0;
	flag = -1;
	angle=-900;
	Setdistance=setdistance; 
	Actualdistance=actualdistance; 
	todistance = Setdistance - Actualdistance;
	drone = new DJIDrone(nh);	
}
pid_controler::~pid_controler(){
	printf("Landing..\n");
	drone->landing();
	sleep(3);
	drone->release_sdk_permission_control();
	sleep(2);
	delete drone;
}
float pid_controler::actions(float setdistance,float actualdistance,float x,float y){ 
	Setdistance=setdistance; 
	Actualdistance = actualdistance;
	todistance=Actualdistance-setdistance; 

	int degrees = (int)(asin(x/actualdistance)*180/3.14);


	
	float x_distance=0;
	float y_distance=0;
	//Go Forward if speed is positive
	//Go Back if speed is negetive
/*	if(todistance>=0.04)
		speed = 2;
	else if(todistance<=-0.04)
		speed = -2;
	int cout =  (int)(todistance/(0.02*speed)); //drone moves 0.02*speed in one loop
	for(int i = 0; i < cout; i++)
	{
	    if(i < cout-(int)(cout*0.1))
		drone->attitude_control(0x40, speed, 0, 0, 0);
	    else
		drone->attitude_control(0x40, 0, 0, 0, 0);
	    usleep(20000);
	}
	sleep(2);
*/
	int x_speed = 0;
	int y_speed = 0;
	x_distance = (-todistance*cos(degrees));
	y_distance = (-todistance*sin(degrees));	
	x_speed = 2;
	printf("x_distance = %f,y_distance = %f\n",x_distance,y_distance);
	int step =  abs((int)(x_distance/(0.02*2))); //drone moves 0.02*speed in one loop
	y_speed = y_distance/(x_distance/2);
	printf("x_speed = %d,y_speed = %d\n",x_speed,y_speed);
	printf("step = %d\n",step);
	for(int i = 0; i < step; i++)
	{
	    if(i < step-(int)(step*0.1))
		drone->attitude_control(0x40, x_speed, y_speed, 0, 0);
	    else
		drone->attitude_control(0x40, 0, 0, 0, 0);
	    usleep(20000);
	}
	sleep(3);
	//Turning Left/Right
	if(degrees<0)
		printf("Turning Left [%d] degrees...\n",degrees);
	else 
		printf("Turning Right [%d] degrees...\n",degrees);
	drone->attitude_control(0xA, 0, 0, 0, degrees);
	sleep(1);
	return Actualdistance; 
}
void pid_controler::exception_landing(void){
	drone->landing();
	sleep(3);
	drone->release_sdk_permission_control();
	sleep(2);
}
void pid_controler::flight_init(void){
	printf("Requesting sdk permission...\n");
	drone->request_sdk_permission_control();
	sleep(3);
	printf("Taking off...\n");
	drone->takeoff();
	sleep(4);
	drone->gimbal_angle_control(0, 0, 0, 20);
	sleep(1);

}

void pid_controler::gimbal_test(){
	int tmp =-900;
	drone->gimbal_angle_control(0, 0, 0, tmp);
	sleep(2);
}
void pid_controler::resetflag(void){
	flag = -1;
	angle = -900;
}
int pid_controler::getflag(void){
	return flag;
}
void pid_controler::setflag(void){
	flag = 1;
}
int pid_controler::getangle(void){
	return angle;
}
void pid_controler::setangle(int new_angle){
	angle = new_angle;
}
void pid_controler::setx(float x){
	this->x = x;
}
void pid_controler::sety(float y){
	this->y = y;
}



