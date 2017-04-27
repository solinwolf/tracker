
#include <stdio.h>
#include "controler.h"
#include <math.h>
#include <unistd.h>

controler::controler(ros::NodeHandle& nh,float setdistance,float actualdistance){ 
	x=0;
	y=0;
	flag = -1;
	angle=-900;
	Setdistance=setdistance; 
	Actualdistance=actualdistance; 
	todistance = Setdistance - Actualdistance;
	drone = new DJIDrone(nh);	
}
controler::~controler(){
	printf("Landing..\n");
	drone->landing();
	sleep(3);
	drone->release_sdk_permission_control();
	sleep(2);
	delete drone;
}
float controler::actions(float setdistance,float actualdistance,float x,float y){ 
	Setdistance=setdistance; 
	Actualdistance = actualdistance;
	todistance=Setdistance - Actualdistance; 

	int degrees = (int)(asin(x/Actualdistance)*180/3.14);
	int speed=0;

	
	//Turning Left/Right
	if(degrees<0)
		printf("Turning Left [%d] degrees...\n",degrees);
	else 
		printf("Turning Right [%d] degrees...\n",degrees);
	drone->attitude_control(0xA, 0, 0, 0, degrees);
	sleep(3);
	//Go Forward if todistance is negetive
	//Go Back if todistance is positive 
	if(todistance>=0.1){
		printf("Going Back %f m\n",todistance);
		int cout =  (int)(todistance/0.04); //drone moves 0.02*speed in one loop
		for(int i = 0; i < cout; i++)
		{
		    if(i < cout-(int)(cout*0.7))
			drone->attitude_control(0x4a, -2, 0, 0, 0);
		    else
			drone->attitude_control(0x4a, 0, 0, 0, 0);
		    usleep(20000);
		}
		//sleep(2);
	}	
	else if(todistance<=-0.1){
		printf("Going Forward %f m\n",-todistance);
		int cout =  (int)(-todistance/0.04); //drone moves 0.02*speed in one loop
		for(int i = 0; i < cout; i++)
		{
		    if(i < cout-(int)(cout*0.7))
			drone->attitude_control(0x4a, 2, 0, 0, 0);
		    else
			drone->attitude_control(0x4a, 0, 0, 0, 0);
		    usleep(20000);
		}
		//sleep(2);
	}


	return Actualdistance; 
}
void controler::exception_landing(void){
	drone->landing();
	sleep(3);
	drone->release_sdk_permission_control();
	sleep(2);
}
void controler::flight_init(void){
	printf("Requesting sdk permission...\n");
	drone->request_sdk_permission_control();
	sleep(3);
	printf("Taking off...\n");
	drone->takeoff();
	sleep(4);
	drone->gimbal_angle_control(0, -300, 0, 20);
	sleep(1);

}

void controler::gimbal_test(){
	/*drone->gimbal_angle_control(0, 0, 0, angle);

	angle+=10;
	if(angle>900)
		angle=-900;*/
}
void controler::resetflag(void){
	flag = -1;
	angle = -900;
}
int controler::getflag(void){
	return flag;
}
void controler::setflag(void){
	flag = 1;
}
int controler::getangle(void){
	return angle;
}
void controler::setangle(int new_angle){
	angle = new_angle;
}
void controler::setx(float x){
	this->x = x;
}
void controler::sety(float y){
	this->y = y;
}



