#ifndef __PID_CONTROLER_H
#define __PID_CONTROLER_H
#include <dji_sdk/dji_drone.h>

class pid_controler{
private: 
	float Setdistance;         //定义设定值
	float Actualdistance;      //定义实际值
//	float err;                 //定义偏差值
//	float err_last;            //定义上一个偏差值
//	float Kp,Ki,Kd;            //定义比例、积分、微分系数
	float todistance;          //定义电压值（控制执行器的变量）
//	float integral;            //定义积分值
//
   	DJIDrone* drone ;
	float x;
	float y;
	int flag;
	int angle;


public:
	pid_controler(ros::NodeHandle& nh,float setdistance,float actualdistance);
	~pid_controler();
	float actions(float setdistance,float actualdistance,float x,float y);
	void exception_landing(void);
	void flight_init(void);
	void setflag(void);
	void resetflag(void);
	int getflag();
	void gimbal_test(void);
	int getangle();
	void setx(float x);
	void sety(float y);
};

#endif
