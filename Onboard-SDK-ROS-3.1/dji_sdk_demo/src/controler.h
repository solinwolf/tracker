#ifndef __controler_H
#define __controler_H
#include <dji_sdk/dji_drone.h>

class controler{
private: 
	float Setdistance;         //定义设定值
	float Actualdistance;      //定义实际值
	float todistance;          //控制执行器的变量
   	DJIDrone* drone ;
	float x;
	float y;
	int flag;
	int angle;


public:
	controler(ros::NodeHandle& nh,float setdistance,float actualdistance);
	~controler();
	float actions(float setdistance,float actualdistance,float x,float y);
	void exception_landing(void);
	void flight_init(void);
	void setflag(void);
	void resetflag(void);
	int getflag();
	void setangle(int new_angle);
	void gimbal_test(void);
	int getangle();
	void setx(float x);
	void sety(float y);

};

#endif
