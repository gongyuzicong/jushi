/*******************************************************************************
 * @name    : ��Ԫ����̬���㷨
 * @version : V1.0
 * @date    : 2014-04-03
 * @MDK     : KEIL MDK4.72a & KEL MDK 5.10
 * @brief   : �������������ֵ������̬���㣬�õ�Ŀ������ĸ����Ǻͺ���� �ͺ����
 * ---------------------------------------------------------------------------- 
 * @copyright
 *
 * ----------------------------------------------------------------------------
 * @description
 * �������������ֵ������̬���㣬�õ�Ŀ������ĸ����Ǻͺ���� �ͺ����
 * 
 *-----------------------------------------------------------------------------
 * @history
 * ----------------------------------------------------------------------------
 * ����ʱ�䣺2016��7��9��    �����ˣ����Ӵ�
 * �汾��¼��V1.1
 * �������ݣ�V1.0 �½�
 			 V1.1 �޸������ϰ汾STM32���
 * ----------------------------------------------------------------------------
 *
 ******************************************************************************/
//#include "delay.h"
//#include "timer.h"
#include "imu.h"

volatile float exInt, eyInt, ezInt;	// ������
volatile float q0, q1, q2, q3; // ȫ����Ԫ��
volatile float integralFBhand,handdiff;
volatile u32 lastUpdate, now; // �������ڼ��� ��λ us


void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);

/**
  * @brief  ��ȡϵͳʱ��
  * @param  None
  * @retval ��ǰϵͳʱ�䣬��λ��us
	* @note   
	*/
u32 micros(void)
{
 	u32 temp = 0;
	
 	temp = TIM2->CNT;		//����16λʱ��
 	temp = temp << 16;
 	temp += TIM3->CNT;	//����16λʱ��
 	return temp;
}

/**
  * @brief  ��ʼ��MPU6050������̬������صı�������ʱ����
  * @param  None
  * @retval 1 - ��ʼ��ʧ��   0 - ��ʼ���ɹ�
	* @note
	*     -	��ʼ��MPU6050
	*     -	��ʼ����Ԫ��
	*     -	����������
	*     -	����ϵͳʱ��
	*/
u32 IMU_init(void)
{
	if(MPU6050_init() == 1) return 1;//MPU6050��ʼ��ʧ�ܣ�����1
	delay_ms(50);
	if(MPU6050_init() == 1) return 1;//MPU6050��ʼ��ʧ�ܣ�����1
	TIM3_TIM2_Init();	//��ʼ��TIM3��TIM2�ļ�����ʱ������������ϵͳʱ��
	
	//��ʼ����Ԫ��
	q0 = 1.0f;  
	q1 = 0.0f;
	q2 = 0.0f;
	q3 = 0.0f;
	exInt = 0.0;
	eyInt = 0.0;
	ezInt = 0.0;

	lastUpdate = micros();//����ʱ��
	now = micros();
	return 0;
}

/**
  * @brief  ���ټ���1/Sqrt(x)
  * @param  Ҫ�������
  * @retval ����ֵ
	* @note		
	*/
float invSqrt(float x) 
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

/**
  * @brief  ��ȡ�˶��������Ĳ�����
  * @param  ��������ŵ�ַ
  * @retval None
	* @note		�����ǵĲ�������ת��Ϊ��/��
	*/
void IMU_getValues(float * values) 
{  
	u16 mpu6050_adc[7];
	int i;
	
	//��ȡ���ٶȺ������ǵĵ�ǰADC
	MPU6050_getADC(&mpu6050_adc[0], &mpu6050_adc[1], &mpu6050_adc[2],
								 &mpu6050_adc[3], &mpu6050_adc[4], &mpu6050_adc[5],
								 &mpu6050_adc[7]);
	
	for(i = 0; i<6; i++)
	{
		values[i] = (float) mpu6050_adc[i];
	}
//	HMC58X3_mgetValues(&values[6]);	//��ȡ�����Ƶ�ADCֵ
}


/**
  * @brief  ���ݵ�ǰ����ֵ������Ԫ��
  * @param  ��ǰ�Ĳ���ֵ��������ֵΪ���ٶ�
  * @retval None
	* @note   
	*/
#define Kp 2.0f		// proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.01f	// integral gain governs rate of convergence of gyroscope biases

void IMU_AHRSupdate(float gx, float gy, float gz, 
										float ax, float ay, float az, 
										float mx, float my, float mz) 
{
  float norm;
  float hx, hy, hz, bx, bz;
  float vx, vy, vz, wx, wy, wz;
  float ex, ey, ez,halfT;
  float tempq0,tempq1,tempq2,tempq3;

  // �Ȱ���Щ�õõ���ֵ���
  float q0q0 = q0*q0;
  float q0q1 = q0*q1;
  float q0q2 = q0*q2;
  float q0q3 = q0*q3;
  float q1q1 = q1*q1;
  float q1q2 = q1*q2;
  float q1q3 = q1*q3;
  float q2q2 = q2*q2;   
  float q2q3 = q2*q3;
  float q3q3 = q3*q3;          
  
  now = micros();	//��ȡʱ��
  if(now < lastUpdate)
	{
		halfT =  ((float)(now + (0xffffffff - lastUpdate)) / 2000000.0f);//��λ: 500ms
  }
  else
	{
		halfT =  ((float)(now - lastUpdate) / 2000000.0f);
  }
	
  lastUpdate = now;	//����ʱ��

  norm = invSqrt(ax*ax + ay*ay + az*az);       
  ax = ax * norm;
  ay = ay * norm;
  az = az * norm;
  //�Ѽ��ٶȼƵ���ά����ת�ɵ�λ������

  norm = invSqrt(mx*mx + my*my + mz*mz);          
  mx = mx * norm;
  my = my * norm;
  mz = mz * norm;

  /*
	���ǰ���Ԫ������ɡ��������Ҿ����еĵ����е�����Ԫ�ء�
	�������Ҿ����ŷ���ǵĶ��壬��������ϵ������������ת����������ϵ��������������Ԫ�ء�
	���������vx\y\z����ʵ���ǵ�ǰ��ŷ���ǣ�����Ԫ�����Ļ����������ϵ�ϣ����������������λ������
  */
  // compute reference direction of flux
  hx = 2*mx*(0.5f - q2q2 - q3q3) + 2*my*(q1q2 - q0q3) + 2*mz*(q1q3 + q0q2);
  hy = 2*mx*(q1q2 + q0q3) + 2*my*(0.5f - q1q1 - q3q3) + 2*mz*(q2q3 - q0q1);
  hz = 2*mx*(q1q3 - q0q2) + 2*my*(q2q3 + q0q1) + 2*mz*(0.5f - q1q1 - q2q2);         
  bx = sqrt((hx*hx) + (hy*hy));
  bz = hz;     
  
  // estimated direction of gravity and flux (v and w)
  vx = 2*(q1q3 - q0q2);
  vy = 2*(q0q1 + q2q3);
  vz = q0q0 - q1q1 - q2q2 + q3q3;
  wx = 2*bx*(0.5 - q2q2 - q3q3) + 2*bz*(q1q3 - q0q2);
  wy = 2*bx*(q1q2 - q0q3) + 2*bz*(q0q1 + q2q3);
  wz = 2*bx*(q0q2 + q1q3) + 2*bz*(0.5 - q1q1 - q2q2);  
  
	//��ʹ�ô�����
	wx = 0;
	wy = 0;
	wz = 0;
	
	// error is sum of cross product between reference direction of fields and direction measured by sensors
  ex = (ay*vz - az*vy) + (my*wz - mz*wy);
  ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
  ez = (ax*vy - ay*vx) + (mx*wy - my*wx);

/*
  axyz�ǻ����������ϵ�ϣ����ٶȼƲ����������������Ҳ����ʵ�ʲ����������������
  axyz�ǲ����õ�������������vxyz�����ݻ��ֺ����̬����������������������Ƕ��ǻ����������ϵ�ϵ�����������
  ������֮�������������������ݻ��ֺ����̬�ͼӼƲ��������̬֮�����
  ������������������������Ҳ�������������ˣ�����ʾ��exyz�����������������Ĳ����
  �����������Ծ���λ�ڻ�������ϵ�ϵģ������ݻ������Ҳ���ڻ�������ϵ�����Ҳ���Ĵ�С�����ݻ����������ȣ�
	���������������ݡ���������Լ��ö�������һ�£����������ǶԻ���ֱ�ӻ��֣����Զ����ݵľ�������ֱ�������ڶ�
	��������ϵ�ľ�����
*/

	if(ex != 0.0f && ey != 0.0f && ez != 0.0f)
	{
		exInt = exInt + ex * Ki * halfT;
		eyInt = eyInt + ey * Ki * halfT;	
		ezInt = ezInt + ez * Ki * halfT;

		// �ò���������PI����������ƫ
		gx = gx + Kp*ex + exInt;
		gy = gy + Kp*ey + eyInt;
		gz = gz + Kp*ez + ezInt;
  }

  // ��Ԫ��΢�ַ���
  tempq0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
  tempq1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
  tempq2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
  tempq3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;  
  
  // ��Ԫ���淶��
  norm = invSqrt(tempq0*tempq0 + tempq1*tempq1 + tempq2*tempq2 + tempq3*tempq3);
  q0 = tempq0 * norm;
  q1 = tempq1 * norm;
  q2 = tempq2 * norm;
  q3 = tempq3 * norm;
}

/**
  * @brief  ������Ԫ��
  * @param  �����Ԫ�����׵�ַ
  * @retval None
	* @note   
	*/
void IMU_getQ(float * q)
{
	float mygetqval[9];	//���ڴ�Ŵ�����ת�����������

  IMU_getValues(mygetqval);	 
  //�������ǵĲ���ֵת�ɻ���ÿ��
  //���ٶȺʹ����Ʊ��� ADCֵ������Ҫת��
	//��������Ϊ1000��ÿ��  32.8 ��Ӧ 1��/��
	mygetqval[3] /= 32.8f;
	mygetqval[4] /= 32.8f;
	mygetqval[5] /= 32.8f;
	
	IMU_AHRSupdate(mygetqval[3] * M_PI/180, mygetqval[4] * M_PI/180, mygetqval[5] * M_PI/180,
								 mygetqval[0], mygetqval[1], mygetqval[2], 
	               mygetqval[6], mygetqval[7], mygetqval[8]);

  q[0] = q0; //���ص�ǰֵ
  q[1] = q1;
  q[2] = q2;
  q[3] = q3;
}


/**
  * @brief  ��ѯ������Ԫ����������Ԫ��������̬��
  * @param  ��̬�ǵ������׵�ַ
  * @retval None
	* @note   
	*/
void IMU_getYawPitchRoll(float * angles)
{
  float q[4]; //����Ԫ��
  volatile float gx=0.0, gy=0.0, gz=0.0; //������������
	
  IMU_getQ(q); //����ȫ����Ԫ��
  
  angles[0] = -atan2(2 * q[1] * q[2] + 2 * q[0] * q[3], - 2 * q[2]*q[2] - 2 * q[3] * q[3] + 1)* 180/M_PI; // yaw
  angles[1] = -asin(-2 * q[1] * q[3] + 2 * q[0] * q[2])* 180/M_PI; // pitch
  angles[2] = atan2(2 * q[2] * q[3] + 2 * q[0] * q[1], -2 * q[1] * q[1] - 2 * q[2] * q[2] + 1)* 180/M_PI; // roll
  //if(angles[0]<0)angles[0]+=360.0f;  //�� -+180��  ת��0-360��
}


/********************* (C) COPYRIGHT 2014 WWW.UCORTEX.COM **********END OF FILE**********/

