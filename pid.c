#include<stdio.h>
#include<stdlib.h>
#include<stdint.h>
#include<math.h>

#pragma warning(disable : 4305)  // ����ʾ4305��34�ž�����Ϣ  
#pragma warning(disable : 4578)

#define	_PID_	98

#if _PID_ == 1
	//λ����PID
	struct _pid
	{
		float shedingzhi;            //�����趨ֵ  SetSpeed
		float shijizhi;        //����ʵ��ֵ  shijizhi
		float piancha;                //����ƫ��ֵ   piancha  
		float piancha_last;            //������һ��ƫ��ֵ   piancha_last
		float Kp, Ki, Kd;            //������������֡�΢��ϵ��
		float voltage;          //�����ѹֵ������ִ�����ı�����
		float jifenzhi;            //�������ֵ jifenzhi
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
	//������PID
	struct _pid
	{
		float shedingzhi;            //�����趨ֵ
		float shijizhi;        //����ʵ��ֵ
		float piancha;                //����ƫ��ֵ
		float piancha_next;            //������һ��ƫ��ֵ
		float piancha_last;            //��������ǰ��ƫ��ֵ
		float Kp, Ki, Kd;            //������������֡�΢��ϵ��
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
	//���ַ����PID�����㷨����λ����PID�ı仯
	struct _pid
	{
		float shedingzhi; //�����趨ֵ
		float shijizhi; //����ʵ��ֵ
		float piancha; //����ƫ��ֵ
		float piancha_last; //������һ��ƫ��ֵ
		float Kp, Ki, Kd; //������������֡�΢��ϵ��
		float voltage; //�����ѹֵ������ִ�����ı�����
		float jifenzhi; //�������ֵ
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
		pid.Ki = 0.04; //֮ǰKIֵ��0.015��֮ǰ�Ĵ�
		pid.Kd = 0.2;  //��ʼ������
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
		pid.voltage = pid.Kp*pid.piancha + index * pid.Ki*pid.jifenzhi + pid.Kd*(pid.piancha - pid.piancha_last);//�㷨����ʵ�ֹ���
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
	//���ַ����PID�����㷨����������PID�ı仯v1.0
	//�������������֮�䲨��
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

	//����ṹ��͹�����
	typedef struct
	{
		float setpoint; //�趨ֵ
		float proportiongain; //����ϵ��
		float jifenzhigain; //����ϵ��
		float derivativegain; //΢��ϵ��
		float lastpianchaor; //ǰһ��ƫ��
		float prepianchaor; //ǰ����ƫ��
		float deadband; //����
		float result; //���ֵ
		float epsilon; //ƫ������ֵ
	}PID;

	//������ʵ��PID��������
	void PIDRegulation(PID *vPID, float processValue)
	{
		float thispianchaor;
		float increment;
		float ppianchaor, dpianchaor, ipianchaor;

		thispianchaor = vPID->setpoint - processValue; //�õ�ƫ��ֵ
		ppianchaor = thispianchaor - vPID->lastpianchaor;
		ipianchaor = thispianchaor;
		dpianchaor = thispianchaor - 2 * (vPID->lastpianchaor) + vPID->prepianchaor;
		uint16_t beta = BetaGeneration(thispianchaor, vPID->epsilon);

		if (beta > 0)
		{
			increment = vPID->proportiongain*ppianchaor + vPID->derivativegain*dpianchaor; //��������
		}
		else
		{
			increment = vPID->proportiongain*ppianchaor + vPID->jifenzhigain*ipianchaor + vPID->derivativegain*dpianchaor; //��������
		}
		vPID->prepianchaor = vPID->lastpianchaor; //���ƫ�������´�����
		vPID->lastpianchaor = thispianchaor;
		vPID->result += increment;
	}
	void PID_init(PID *pid)
	{
		printf("PID_init begin \n");
		pid->setpoint = 200.0; //�趨ֵ
		pid->proportiongain = 0.2; //����ϵ��
		pid->jifenzhigain = 0.35; //����ϵ��
		pid->derivativegain = 0.2; //΢��ϵ��
		pid->lastpianchaor = 0; //ǰһ��ƫ��
		pid->prepianchaor = 0; //ǰ����ƫ��
		pid->deadband; //����
		pid->result = 0; //���ֵ
		pid->epsilon = 0.1; //ƫ������ֵ
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
	////���ַ����PID�����㷨����������PID�ı仯v2.0
	struct _pid
	{
		float shedingzhi; //�����趨ֵ
		float shijizhi; //����ʵ��ֵ
		float piancha; //����ƫ��ֵ
		float piancha_next; //������һ��ƫ��ֵ
		float piancha_last; //��������ǰ��ƫ��ֵ
		float Kp, Ki, Kd; //������������֡�΢��ϵ��
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
	//�����ֱ��͵�PID
	struct _pid
	{
		float shedingzhi;            //�����趨ֵ
		float shijizhi;        //����ʵ��ֵ
		float piancha;                //����ƫ��ֵ
		float piancha_last;            //������һ��ƫ��ֵ
		float Kp, Ki, Kd;            //������������֡�΢��ϵ��
		float voltage;            //�����ѹֵ������ִ�����ı�����
		float jifenzhi;            //�������ֵ
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
		pid.Ki = 0.1;       //ע�⣬���ϼ�����ȣ�����Ӵ��˻��ֻ��ڵ�ֵ
		pid.Kd = 0.2;
		pid.umax = 400;
		pid.umin = -200;
		printf("PID_init end \n");
	}
	float PID_realize(float speed) {
		int index;
		pid.shedingzhi = speed;
		pid.piancha = pid.shedingzhi - pid.shijizhi;

		if (pid.shijizhi > pid.umax)  //��ɫ��ɫ��ʾ�����ֱ��͵�ʵ��
		{
			if (abs(pid.piancha) > 200)      //��ɫ��עΪ���ַ������
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
			if (abs(pid.piancha) > 200)      //���ַ������
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
			if (abs(pid.piancha) > 200)                    //���ַ������
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
	//���λ��ֵ�PID
	//pid.voltage=pid.Kp*pid.piancha+index*pid.Ki*pid.jifenzhi/2+pid.Kd*(pid.piancha-pid.piancha_last);
	//���λ���  �����ʵ������pid.voltage����任
	struct _pid
	{
		float shedingzhi;            //�����趨ֵ
		float shijizhi;        //����ʵ��ֵ
		float piancha;                //����ƫ��ֵ
		float piancha_last;            //������һ��ƫ��ֵ
		float Kp, Ki, Kd;            //������������֡�΢��ϵ��
		float voltage;            //�����ѹֵ������ִ�����ı�����
		float jifenzhi;            //�������ֵ
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
		pid.Ki = 0.1;       //ע�⣬���ϼ�����ȣ�����Ӵ��˻��ֻ��ڵ�ֵ
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

		if (pid.shijizhi > pid.umax)  //��ɫ��ɫ��ʾ�����ֱ��͵�ʵ��
		{
			if (abs(pid.piancha) > 200)      //��ɫ��עΪ���ַ������
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
			if (abs(pid.piancha) > 200)      //���ַ������
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
			if (abs(pid.piancha) > 200)                    //���ַ������
			{
				index = 0;
			}
			else 
			{
				index = 1;
				pid.jifenzhi += pid.piancha;
			}
		}
		pid.voltage = pid.Kp*pid.piancha + index * pid.Ki*pid.jifenzhi / 2 + pid.Kd*(pid.piancha - pid.piancha_last);  //���λ��� 
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
	//����ֵ�PID ϵͳ���ȶ��ٶȷǳ���
	struct _pid
	{
		float shedingzhi; //�����趨ֵ
		float shijizhi; //����ʵ��ֵ
		float piancha; //����ƫ��ֵ
		float piancha_last; //������һ��ƫ��ֵ
		float Kp, Ki, Kd; //������������֡�΢��ϵ��
		float voltage; //�����ѹֵ������ִ�����ı�����
		float jifenzhi; //�������ֵ
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
		pid.Ki = 0.2;    //�����˻���ϵ��
		pid.Kd = 0.2;
		printf("PID_init end \n");
	}

	float PID_realize(float speed)
	{
		float index;
		pid.shedingzhi = speed;
		pid.piancha = pid.shedingzhi - pid.shijizhi;

		if (abs(pid.piancha) > 200)           //����ֹ���
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