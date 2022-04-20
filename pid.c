#include<stdio.h>
#include<stdlib.h>
#include<stdint.h>
#include<math.h>

#pragma warning(disable : 4305)  // 不显示4305和34号警告信息  
#pragma warning(disable : 4578)

#define	_PID_	98

#if _PID_ == 1
	//位置型PID
	struct _pid
	{
		float shedingzhi;            //定义设定值  SetSpeed
		float shijizhi;        //定义实际值  shijizhi
		float piancha;                //定义偏差值   piancha  
		float piancha_last;            //定义上一个偏差值   piancha_last
		float Kp, Ki, Kd;            //定义比例、积分、微分系数
		float voltage;          //定义电压值（控制执行器的变量）
		float jifenzhi;            //定义积分值 jifenzhi
	}pid;

	void PID_init()
	{
		printf("PID_init begin \n");
		pid.shedingzhi = 0.0;
		pid.shijizhi = 0.0;
		pid.piancha = 0.0;
		pid.piancha_last = 0.0;
		pid.voltage = 0.0;
		pid.jifenzhi = 0.0;
		pid.Kp = 0.2;
		pid.Ki = 0.015;
		pid.Kd = 0.2;
		printf("PID_init end \n");
	}

	float PID_realize(float speed)
	{
		pid.shedingzhi = speed;
		pid.piancha = pid.shedingzhi - pid.shijizhi;
		pid.jifenzhi += pid.piancha;
		pid.voltage = pid.Kp*pid.piancha + pid.Ki*pid.jifenzhi + pid.Kd*(pid.piancha - pid.piancha_last);
		pid.piancha_last = pid.piancha;
		pid.shijizhi = pid.voltage*1.0;
		return pid.shijizhi;
	}

	int main()
	{
		printf("System begin \n");
		PID_init();
		int count = 0;
		while (count < 1500)
		{
			float speed = PID_realize(200.0);
			printf("%f\n", speed);
			count++;
		}
		return 0;
	}

#elif _PID_ == 2
	//增量型PID
	struct _pid
	{
		float shedingzhi;            //定义设定值
		float shijizhi;        //定义实际值
		float piancha;                //定义偏差值
		float piancha_next;            //定义上一个偏差值
		float piancha_last;            //定义最上前的偏差值
		float Kp, Ki, Kd;            //定义比例、积分、微分系数
	}pid;

	void PID_init()
	{
		pid.shedingzhi = 0.0;
		pid.shijizhi = 0.0;
		pid.piancha = 0.0;
		pid.piancha_last = 0.0;
		pid.piancha_next = 0.0;
		pid.Kp = 0.2;
		pid.Ki = 0.015;
		pid.Kd = 0.2;
	}

	float PID_realize(float speed)
	{
		pid.shedingzhi = speed;
		pid.piancha = pid.shedingzhi - pid.shijizhi;
		float incrementSpeed = pid.Kp*(pid.piancha - pid.piancha_next) + pid.Ki*pid.piancha + pid.Kd*(pid.piancha - 2 * pid.piancha_next + pid.piancha_last);
		pid.shijizhi += incrementSpeed;
		pid.piancha_last = pid.piancha_next;
		pid.piancha_next = pid.piancha;
		return pid.shijizhi;
	}

	int main()
	{
		PID_init();
		int count = 0;
		while (count < 1000)
		{
			float speed = PID_realize(200.0);
			printf("%f\n", speed);
			count++;
		}
		return 0;
	}

#elif _PID_ ==3
	//积分分离的PID控制算法——位置型PID的变化
	struct _pid
	{
		float shedingzhi; //定义设定值
		float shijizhi; //定义实际值
		float piancha; //定义偏差值
		float piancha_last; //定义上一个偏差值
		float Kp, Ki, Kd; //定义比例、积分、微分系数
		float voltage; //定义电压值（控制执行器的变量）
		float jifenzhi; //定义积分值
	}pid;
	void PID_init()
	{
		printf("PID_init begin \n");
		pid.shedingzhi = 0.0;
		pid.shijizhi = 0.0;
		pid.piancha = 0.0;
		pid.piancha_last = 0.0;
		pid.voltage = 0.0;
		pid.jifenzhi = 0.0;
		pid.Kp = 0.2;
		pid.Ki = 0.04; //之前KI值是0.015比之前的大
		pid.Kd = 0.2;  //初始化过程
		printf("PID_init end \n");
	}

	float PID_realize(float speed)
	{
		int index;
		pid.shedingzhi = speed;
		pid.piancha = pid.shedingzhi - pid.shijizhi;
		pid.jifenzhi += pid.piancha;
		if (abs(pid.piancha) > 200)//***
		{
			index = 0;
		}
		else
		{
			index = 1;
			// pid.jifenzhi += pid.piancha;
		}	//*****
		pid.voltage = pid.Kp*pid.piancha + index * pid.Ki*pid.jifenzhi + pid.Kd*(pid.piancha - pid.piancha_last);//算法具体实现过程
		pid.piancha_last = pid.piancha;
		pid.shijizhi = pid.voltage*1.0;
		return pid.shijizhi;
	}

	int main()
	{
		printf("System begin \n");
		PID_init();
		int count = 0;
		while (count < 1500)
		{
			float speed = PID_realize(200.0);
			printf("%f\n", speed);
			count++;
		}
		return 0;
	}

#elif _PID_ ==4
	//积分分离的PID控制算法——增量型PID的变化v1.0
	//最后结果在两个数之间波动
	static uint16_t BetaGeneration(float pianchaor, float epsilon)
	{
		int beta;
		if (abs(pianchaor) > epsilon)
		{
			beta = 0;
		}
		else
		{
			beta = 1;
		}
		return beta;
	}

	//定义结构体和公用体
	typedef struct
	{
		float setpoint; //设定值
		float proportiongain; //比例系数
		float jifenzhigain; //积分系数
		float derivativegain; //微分系数
		float lastpianchaor; //前一拍偏差
		float prepianchaor; //前两拍偏差
		float deadband; //死区
		float result; //输出值
		float epsilon; //偏差检测阈值
	}PID;

	//接下来实现PID控制器：
	void PIDRegulation(PID *vPID, float processValue)
	{
		float thispianchaor;
		float increment;
		float ppianchaor, dpianchaor, ipianchaor;

		thispianchaor = vPID->setpoint - processValue; //得到偏差值
		ppianchaor = thispianchaor - vPID->lastpianchaor;
		ipianchaor = thispianchaor;
		dpianchaor = thispianchaor - 2 * (vPID->lastpianchaor) + vPID->prepianchaor;
		uint16_t beta = BetaGeneration(thispianchaor, vPID->epsilon);

		if (beta > 0)
		{
			increment = vPID->proportiongain*ppianchaor + vPID->derivativegain*dpianchaor; //增量计算
		}
		else
		{
			increment = vPID->proportiongain*ppianchaor + vPID->jifenzhigain*ipianchaor + vPID->derivativegain*dpianchaor; //增量计算
		}
		vPID->prepianchaor = vPID->lastpianchaor; //存放偏差用于下次运算
		vPID->lastpianchaor = thispianchaor;
		vPID->result += increment;
	}
	void PID_init(PID *pid)
	{
		printf("PID_init begin \n");
		pid->setpoint = 200.0; //设定值
		pid->proportiongain = 0.2; //比例系数
		pid->jifenzhigain = 0.35; //积分系数
		pid->derivativegain = 0.2; //微分系数
		pid->lastpianchaor = 0; //前一拍偏差
		pid->prepianchaor = 0; //前两拍偏差
		pid->deadband; //死区
		pid->result = 0; //输出值
		pid->epsilon = 0.1; //偏差检测阈值
		printf("PID_init end \n");
	}
	int main()
	{
		printf("System begin \n");
		PID pid1;
		PID_init(&pid1);
		int cout = 1000;
		while (cout)
		{
			PIDRegulation(&pid1, pid1.result);
			printf("%f\n", pid1.result);
			cout--;
		}
		return 0;
	}

#elif _PID_ ==5
	////积分分离的PID控制算法——增量型PID的变化v2.0
	struct _pid
	{
		float shedingzhi; //定义设定值
		float shijizhi; //定义实际值
		float piancha; //定义偏差值
		float piancha_next; //定义上一个偏差值
		float piancha_last; //定义最上前的偏差值
		float Kp, Ki, Kd; //定义比例、积分、微分系数
	}pid;

	void PID_init()
	{
		pid.shedingzhi = 0.0;
		pid.shijizhi = 0.0;
		pid.piancha = 0.0;
		pid.piancha_last = 0.0;
		pid.piancha_next = 0.0;
		pid.Kp = 0.2;
		pid.Ki = 0.015;
		pid.Kd = 0.2;
	}

	float PID_realize(float speed)
	{
		int index;
		pid.shedingzhi = speed;
		pid.piancha = pid.shedingzhi - pid.shijizhi;
		if (abs(pid.piancha) > 200)
		{
			index = 0;
		}
		else
		{
			index = 1;
		}

		float incrementSpeed = pid.Kp*(pid.piancha - pid.piancha_next) + index * pid.Ki*pid.piancha + pid.Kd*(pid.piancha - 2 * pid.piancha_next + pid.piancha_last);
		pid.shijizhi += incrementSpeed;
		pid.piancha_last = pid.piancha_next;
		pid.piancha_next = pid.piancha;
		return pid.shijizhi;
	}

	int main()
	{
		PID_init();
		int count = 0;
		while (count < 1000)
		{
			float speed = PID_realize(200.0);
			printf("%f\n", speed);
			count++;
		}
		return 0;
	}


#elif _PID_ ==6
	//抗积分饱和的PID
	struct _pid
	{
		float shedingzhi;            //定义设定值
		float shijizhi;        //定义实际值
		float piancha;                //定义偏差值
		float piancha_last;            //定义上一个偏差值
		float Kp, Ki, Kd;            //定义比例、积分、微分系数
		float voltage;            //定义电压值（控制执行器的变量）
		float jifenzhi;            //定义积分值
		float umax;
		float umin;
	}pid;

	void PID_init()
	{
		printf("PID_init begin \n");
		pid.shedingzhi = 0.0;
		pid.shijizhi = 0.0;
		pid.piancha = 0.0;
		pid.piancha_last = 0.0;
		pid.voltage = 0.0;
		pid.jifenzhi = 0.0;
		pid.Kp = 0.2;
		pid.Ki = 0.1;       //注意，和上几次相比，这里加大了积分环节的值
		pid.Kd = 0.2;
		pid.umax = 400;
		pid.umin = -200;
		printf("PID_init end \n");
	}
	float PID_realize(float speed) {
		int index;
		pid.shedingzhi = speed;
		pid.piancha = pid.shedingzhi - pid.shijizhi;

		if (pid.shijizhi > pid.umax)  //灰色底色表示抗积分饱和的实现
		{
			if (abs(pid.piancha) > 200)      //蓝色标注为积分分离过程
			{
				index = 0;
			}
			else {
				index = 1;
				if (pid.piancha < 0)
				{
					pid.jifenzhi += pid.piancha;
				}
			}
		}
		else if (pid.shijizhi < pid.umin) {
			if (abs(pid.piancha) > 200)      //积分分离过程
			{
				index = 0;
			}
			else {
				index = 1;
				if (pid.piancha > 0)
				{
					pid.jifenzhi += pid.piancha;
				}
			}
		}
		else {
			if (abs(pid.piancha) > 200)                    //积分分离过程
			{
				index = 0;
			}
			else {
				index = 1;
				pid.jifenzhi += pid.piancha;
			}
		}
		pid.voltage = pid.Kp*pid.piancha + index * pid.Ki*pid.jifenzhi + pid.Kd*(pid.piancha - pid.piancha_last);
		pid.piancha_last = pid.piancha;
		pid.shijizhi = pid.voltage*1.0;
		return pid.shijizhi;
	}

	int main()
	{
		printf("System begin \n");
		PID_init();
		int count = 0;
		while (count < 1500)
		{
			float speed = PID_realize(200.0);
			printf("%f\n", speed);
			count++;
		}
		return 0;
	}

#elif _PID_ ==7
	//梯形积分的PID
	//pid.voltage=pid.Kp*pid.piancha+index*pid.Ki*pid.jifenzhi/2+pid.Kd*(pid.piancha-pid.piancha_last);
	//梯形积分  相对于实现六的pid.voltage代码变换
	struct _pid
	{
		float shedingzhi;            //定义设定值
		float shijizhi;        //定义实际值
		float piancha;                //定义偏差值
		float piancha_last;            //定义上一个偏差值
		float Kp, Ki, Kd;            //定义比例、积分、微分系数
		float voltage;            //定义电压值（控制执行器的变量）
		float jifenzhi;            //定义积分值
		float umax;
		float umin;
	}pid;

	void PID_init()
	{
		printf("PID_init begin \n");
		pid.shedingzhi = 0.0;
		pid.shijizhi = 0.0;
		pid.piancha = 0.0;
		pid.piancha_last = 0.0;
		pid.voltage = 0.0;
		pid.jifenzhi = 0.0;
		pid.Kp = 0.2;
		pid.Ki = 0.1;       //注意，和上几次相比，这里加大了积分环节的值
		pid.Kd = 0.2;
		pid.umax = 400;
		pid.umin = -200;
		printf("PID_init end \n");
	}
	float PID_realize(float speed)
	{
		int index;
		pid.shedingzhi = speed;
		pid.piancha = pid.shedingzhi - pid.shijizhi;

		if (pid.shijizhi > pid.umax)  //灰色底色表示抗积分饱和的实现
		{
			if (abs(pid.piancha) > 200)      //蓝色标注为积分分离过程
			{
				index = 0;
			}
			else {
				index = 1;
				if (pid.piancha < 0)
				{
					pid.jifenzhi += pid.piancha;
				}
			}
		}
		else if (pid.shijizhi < pid.umin) {
			if (abs(pid.piancha) > 200)      //积分分离过程
			{
				index = 0;
			}
			else {
				index = 1;
				if (pid.piancha > 0)
				{
					pid.jifenzhi += pid.piancha;
				}
			}
		}
		else {
			if (abs(pid.piancha) > 200)                    //积分分离过程
			{
				index = 0;
			}
			else 
			{
				index = 1;
				pid.jifenzhi += pid.piancha;
			}
		}
		pid.voltage = pid.Kp*pid.piancha + index * pid.Ki*pid.jifenzhi / 2 + pid.Kd*(pid.piancha - pid.piancha_last);  //梯形积分 
		pid.piancha_last = pid.piancha;
		pid.shijizhi = pid.voltage*1.0;
		return pid.shijizhi;
	}

	int main()
	{
		printf("System begin \n");
		PID_init();
		int count = 0;
		while (count < 1500)
		{
			float speed = PID_realize(200.0);
			printf("%f\n", speed);
			count++;
		}
		return 0;
	}

#else
	//变积分的PID 系统的稳定速度非常快
	struct _pid
	{
		float shedingzhi; //定义设定值
		float shijizhi; //定义实际值
		float piancha; //定义偏差值
		float piancha_last; //定义上一个偏差值
		float Kp, Ki, Kd; //定义比例、积分、微分系数
		float voltage; //定义电压值（控制执行器的变量）
		float jifenzhi; //定义积分值
	}pid;

	void PID_init()
	{
		printf("PID_init begin \n");
		pid.shedingzhi = 0.0;
		pid.shijizhi = 0.0;
		pid.piancha = 0.0;
		pid.piancha_last = 0.0;
		pid.voltage = 0.0;
		pid.jifenzhi = 0.0;
		pid.Kp = 0.4;
		pid.Ki = 0.2;    //增加了积分系数
		pid.Kd = 0.2;
		printf("PID_init end \n");
	}

	float PID_realize(float speed)
	{
		float index;
		pid.shedingzhi = speed;
		pid.piancha = pid.shedingzhi - pid.shijizhi;

		if (abs(pid.piancha) > 200)           //变积分过程
		{
			index = 0.0;
		}
		else if (abs(pid.piancha) < 180) {
			index = 1.0;
			pid.jifenzhi += pid.piancha;
		}
		else {
			index = (200 - abs(pid.piancha)) / 20;
			pid.jifenzhi += pid.piancha;
		}
		pid.voltage = pid.Kp*pid.piancha + index * pid.Ki*pid.jifenzhi + pid.Kd*(pid.piancha - pid.piancha_last);
		pid.piancha_last = pid.piancha;
		pid.shijizhi = pid.voltage*1.0;
		return pid.shijizhi;
	}

	int main()
	{
		printf("System begin \n");
		PID_init();
		int count = 0;
		while (count < 1500)
		{
			float speed = PID_realize(200.0);
			printf("%f\n", speed);
			count++;
		}
		return 0;
	}

#endif