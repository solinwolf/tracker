
#include <stdio.h>
#include "pid_controler.h"
#include <math.h>
pid_controler::pid_controler(ros::NodeHandle& nh,float setdistance,float actualdistance){ 
	x=0;
	y=0;
	flag = -1;
	angle=0;
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
	todistance=Setdistance-Actualdistance; 
	//YAW 
	if(0>x){
		int cout = (int)(asin(x/actualdistance)*180/3.14/1.8);
		printf("Turning Left [%f] degrees...\n",asin(x/actualdistance)*180/3.14);

		for(int i = 0; i < cout; i ++){
		    if(i < cout-(int)(cout*0.1))
			drone->attitude_control(0xA, 0, 0, 0, -90);
		    else
			drone->attitude_control(0xA, 0, 0, 0, 0);
		    usleep(20000);
		}
		sleep(1);
	}
	else{
		int cout = (int)(asin(x/actualdistance)*180/3.14/1.8);
		printf("Turning Right [%f] degrees...\n",asin(x/actualdistance)*180/3.14);
		for(int i = 0; i < cout; i ++){
		    if(i < cout-(int)(cout*0.1))
			drone->attitude_control(0xA, 0, 0, 0, 90);
		    else
			drone->attitude_control(0xA, 0, 0, 0, 0);
		    usleep(20000);
		}
		sleep(1);
	}
		
//	sleep(2);
	//Too close
	if(todistance>=0.04) {
		// Action here
		printf("Going back distance:%f\n",todistance);
		//d->gimbal_angle_control(0, 0, 0, 20);
		int cout =  (int)(todistance/0.04);
		//printf("cout=%d\n",cout);
		for(int i = 0; i < cout; i++)
		{
		    if(i < cout-(int)(cout*0.1))
			drone->attitude_control(0x40, -2, 0, 0, 0);
		    else
			drone->attitude_control(0x40, 0, 0, 0, 0);
		    usleep(20000);
		}
		sleep(2);
	}
	//Too far
	else if(todistance<0.04){
		todistance = -todistance;
		printf("Going forward distance:%f\n",todistance);
		//d->gimbal_angle_control(0, 0, 0, 20);
		int cout =  (int)(todistance/0.04);
		//printf("cout=%d\n",cout);
		for(int i = 0; i < cout; i++)
		{
		    if(i < cout-(int)(cout*0.1))
			drone->attitude_control(0x40, 2, 0, 0, 0);
		    else
			drone->attitude_control(0x40, 0, 0, 0, 0);
		    usleep(20000);
		}
		sleep(2);
	
	}
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
	
	drone->gimbal_angle_control(0, 0, 450, 20);
	sleep(2);
	angle+=100;
	if(angle>=1800)
		angle=0;

}
void pid_controler::resetflag(void){
	flag = -1;
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
void pid_controler::setx(float x){
	this->x = x;
}
void pid_controler::sety(float y){
	this->y = y;
}



