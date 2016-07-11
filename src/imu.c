/*******************************************************************************
 * @name    : 四元素姿态角算法
 * @version : V1.0
 * @date    : 2014-04-03
 * @MDK     : KEIL MDK4.72a & KEL MDK 5.10
 * @brief   : 将传感器的输出值进行姿态解算，得到目标载体的俯仰角和横滚角 和航向角
 * ---------------------------------------------------------------------------- 
 * @copyright
 *
 * ----------------------------------------------------------------------------
 * @description
 * 将传感器的输出值进行姿态解算，得到目标载体的俯仰角和横滚角 和航向角
 * 
 *-----------------------------------------------------------------------------
 * @history
 * ----------------------------------------------------------------------------
 * 更改时间：2016年7月9日    更改人：郭子聪
 * 版本记录：V1.1
 * 更改内容：V1.0 新建
 			 V1.1 修改适配老版本STM32框架
 * ----------------------------------------------------------------------------
 *
 ******************************************************************************/
//#include "delay.h"
//#include "timer.h"
#include "imu.h"

volatile float exInt, eyInt, ezInt;	// 误差积分
volatile float q0, q1, q2, q3; // 全局四元数
volatile float integralFBhand,handdiff;
volatile u32 lastUpdate, now; // 采样周期计数 单位 us


void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);

/**
  * @brief  读取系统时间
  * @param  None
  * @retval 当前系统时间，单位是us
	* @note   
	*/
u32 micros(void)
{
 	u32 temp = 0;
	
 	temp = TIM2->CNT;		//读高16位时间
 	temp = temp << 16;
 	temp += TIM3->CNT;	//读低16位时间
 	return temp;
}

/**
  * @brief  初始化MPU6050和与姿态计算相关的变量，定时器等
  * @param  None
  * @retval 1 - 初始化失败   0 - 初始化成功
	* @note
	*     -	初始化MPU6050
	*     -	初始化四元数
	*     -	将积分清零
	*     -	更新系统时间
	*/
u32 IMU_init(void)
{
	if(MPU6050_init() == 1) return 1;//MPU6050初始化失败，返回1
	delay_ms(50);
	if(MPU6050_init() == 1) return 1;//MPU6050初始化失败，返回1
	TIM3_TIM2_Init();	//初始话TIM3和TIM2的级联定时器，用来产生系统时间
	
	//初始化四元数
	q0 = 1.0f;  
	q1 = 0.0f;
	q2 = 0.0f;
	q3 = 0.0f;
	exInt = 0.0;
	eyInt = 0.0;
	ezInt = 0.0;

	lastUpdate = micros();//更新时间
	now = micros();
	return 0;
}

/**
  * @brief  快速计算1/Sqrt(x)
  * @param  要计算的数
  * @retval 计算值
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
  * @brief  读取运动传感器的测量量
  * @param  测量量存放地址
  * @retval None
	* @note		陀螺仪的测量数据转化为°/秒
	*/
void IMU_getValues(float * values) 
{  
	u16 mpu6050_adc[7];
	int i;
	
	//读取加速度和陀螺仪的当前ADC
	MPU6050_getADC(&mpu6050_adc[0], &mpu6050_adc[1], &mpu6050_adc[2],
								 &mpu6050_adc[3], &mpu6050_adc[4], &mpu6050_adc[5],
								 &mpu6050_adc[7]);
	
	for(i = 0; i<6; i++)
	{
		values[i] = (float) mpu6050_adc[i];
	}
//	HMC58X3_mgetValues(&values[6]);	//读取磁力计的ADC值
}


/**
  * @brief  根据当前测量值计算四元数
  * @param  当前的测量值，陀螺仪值为角速度
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

  // 先把这些用得到的值算好
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
  
  now = micros();	//读取时间
  if(now < lastUpdate)
	{
		halfT =  ((float)(now + (0xffffffff - lastUpdate)) / 2000000.0f);//单位: 500ms
  }
  else
	{
		halfT =  ((float)(now - lastUpdate) / 2000000.0f);
  }
	
  lastUpdate = now;	//更新时间

  norm = invSqrt(ax*ax + ay*ay + az*az);       
  ax = ax * norm;
  ay = ay * norm;
  az = az * norm;
  //把加速度计的三维向量转成单位向量。

  norm = invSqrt(mx*mx + my*my + mz*mz);          
  mx = mx * norm;
  my = my * norm;
  mz = mz * norm;

  /*
	这是把四元数换算成《方向余弦矩阵》中的第三列的三个元素。
	根据余弦矩阵和欧拉角的定义，地理坐标系的重力向量，转到机体坐标系，正好是这三个元素。
	所以这里的vx\y\z，其实就是当前的欧拉角（即四元数）的机体坐标参照系上，换算出来的重力单位向量。
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
  
	//不使用磁力计
	wx = 0;
	wy = 0;
	wz = 0;
	
	// error is sum of cross product between reference direction of fields and direction measured by sensors
  ex = (ay*vz - az*vy) + (my*wz - mz*wy);
  ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
  ez = (ax*vy - ay*vx) + (mx*wy - my*wx);

/*
  axyz是机体坐标参照系上，加速度计测出来的重力向量，也就是实际测出来的重力向量。
  axyz是测量得到的重力向量，vxyz是陀螺积分后的姿态来推算出的重力向量，它们都是机体坐标参照系上的重力向量。
  那它们之间的误差向量，就是陀螺积分后的姿态和加计测出来的姿态之间的误差。
  向量间的误差，可以用向量叉积（也叫向量外积、叉乘）来表示，exyz就是两个重力向量的叉积。
  这个叉积向量仍旧是位于机体坐标系上的，而陀螺积分误差也是在机体坐标系，而且叉积的大小与陀螺积分误差成正比，
	正好拿来纠正陀螺。（你可以自己拿东西想象一下）由于陀螺是对机体直接积分，所以对陀螺的纠正量会直接体现在对
	机体坐标系的纠正。
*/

	if(ex != 0.0f && ey != 0.0f && ez != 0.0f)
	{
		exInt = exInt + ex * Ki * halfT;
		eyInt = eyInt + ey * Ki * halfT;	
		ezInt = ezInt + ez * Ki * halfT;

		// 用叉积误差来做PI修正陀螺零偏
		gx = gx + Kp*ex + exInt;
		gy = gy + Kp*ey + eyInt;
		gz = gz + Kp*ez + ezInt;
  }

  // 四元数微分方程
  tempq0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
  tempq1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
  tempq2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
  tempq3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;  
  
  // 四元数规范化
  norm = invSqrt(tempq0*tempq0 + tempq1*tempq1 + tempq2*tempq2 + tempq3*tempq3);
  q0 = tempq0 * norm;
  q1 = tempq1 * norm;
  q2 = tempq2 * norm;
  q3 = tempq3 * norm;
}

/**
  * @brief  更新四元数
  * @param  存放四元数的首地址
  * @retval None
	* @note   
	*/
void IMU_getQ(float * q)
{
	float mygetqval[9];	//用于存放传感器转换结果的数组

  IMU_getValues(mygetqval);	 
  //将陀螺仪的测量值转成弧度每秒
  //加速度和磁力计保持 ADC值　不需要转换
	//这里量程为1000度每秒  32.8 对应 1°/秒
	mygetqval[3] /= 32.8f;
	mygetqval[4] /= 32.8f;
	mygetqval[5] /= 32.8f;
	
	IMU_AHRSupdate(mygetqval[3] * M_PI/180, mygetqval[4] * M_PI/180, mygetqval[5] * M_PI/180,
								 mygetqval[0], mygetqval[1], mygetqval[2], 
	               mygetqval[6], mygetqval[7], mygetqval[8]);

  q[0] = q0; //返回当前值
  q[1] = q1;
  q[2] = q2;
  q[3] = q3;
}


/**
  * @brief  轮询更新四元数，根据四元数解算姿态角
  * @param  姿态角的数组首地址
  * @retval None
	* @note   
	*/
void IMU_getYawPitchRoll(float * angles)
{
  float q[4]; //　四元数
  volatile float gx=0.0, gy=0.0, gz=0.0; //估计重力方向
	
  IMU_getQ(q); //更新全局四元数
  
  angles[0] = -atan2(2 * q[1] * q[2] + 2 * q[0] * q[3], - 2 * q[2]*q[2] - 2 * q[3] * q[3] + 1)* 180/M_PI; // yaw
  angles[1] = -asin(-2 * q[1] * q[3] + 2 * q[0] * q[2])* 180/M_PI; // pitch
  angles[2] = atan2(2 * q[2] * q[3] + 2 * q[0] * q[1], -2 * q[1] * q[1] - 2 * q[2] * q[2] + 1)* 180/M_PI; // roll
  //if(angles[0]<0)angles[0]+=360.0f;  //将 -+180度  转成0-360度
}


/********************* (C) COPYRIGHT 2014 WWW.UCORTEX.COM **********END OF FILE**********/

