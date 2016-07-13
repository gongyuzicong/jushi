/*******************************************************************************
 * @name    : MPU6050����
 * @author  : ���Ӵ�
 * @version : V1.1
 * @date    : 2014-04-03
 * @MDK     : KEIL MDK4.72a & KEL MDK 5.10
 * @brief   : MPU6050������
 * ---------------------------------------------------------------------------- 
 * @copyright
 *
 * ----------------------------------------------------------------------------
 * @description
 * MPU6050��ʵ���Էǳ�ǿ��6���˶���������������ʵ����MPU6050�ĳ�ʼ���Լ�����ֵ��
 * ��ȡ��
 *-----------------------------------------------------------------------------
 * @history
 * ----------------------------------------------------------------------------
 * ����ʱ�䣺2016��7��9��    �����ˣ����Ӵ�
 * �汾��¼��V1.1
 * �������ݣ�V1.0 �½�
 *			 V1.1 �����ϰ汾STM32������
 * ----------------------------------------------------------------------------
 *
 ******************************************************************************/
#include "mpu6050.h"
#include "timer_opts.h"
#include <math.h>
#include "i2c_opts.h"
#include "cfg_gpio.h"

u8 MPU6050_UPDATE = 0;
s16  MPU6050_FIFO[7][11];
s16 Gx_offset = 0, Gy_offset = 0, Gz_offset = 0;
MPU6050_ConfigTypedef MPU6050_Config;	//������Ϣ

/**
  * @brief ���µ����ݸ��µ�FIFO�У������˲�����
  * @param ax, ay, az : ���ٶ�ADCֵ
	* @param gx, gy, gz : ������ADCֵ
  * @param temp : �¶ȴ�����ADCֵ
  * @retval none
  * @note MPU6050_FIFO[6][11]�ṹ��
	*   ---------------+-----+-----+-----+-----+------+-----+-----+
	*									 |[ 0 ]|[ 1 ]|[ 2 ]|[ 3 ]|[ 4 ] |[ 5 ]|[ 6 ]|
	*   ---------------+-----+-----+-----+-----+------+-----+-----+
	*		MPU6050_FIFO[0]| ax0 | ay0 | az0 | gx0 |  gy0 | gz0 |temp0| 
	*   ---------------+-----+-----+-----+-----+------+-----+-----+
	*		MPU6050_FIFO[1]| ax1 | ay1 | az1 | gx1 |  gy1 | gz1 |temp1| 
	*   ---------------+-----+-----+-----+-----+------+-----+-----+
	*		MPU6050_FIFO[2]| ax2 | ay2 | az2 | gx2 |  gy2 | gz2 |temp2| 
	*   ---------------+-----+-----+-----+-----+------+-----+-----+
	*		MPU6050_FIFO[3]| ax3 | ay3 | az3 | gx3 |  gy3 | gz3 |temp3| 
	*   ---------------+-----+-----+-----+-----+------+-----+-----+
	*		MPU6050_FIFO[4]| ax4 | ay4 | az4 | gx4 |  gy4 | gz4 |temp4| 
	*   ---------------+-----+-----+-----+-----+------+-----+-----+
	*		MPU6050_FIFO[5]| ax5 | ay5 | az5 | gx5 |  gy5 | gz5 |temp5| 
	*   ---------------+-----+-----+-----+-----+------+-----+-----+
	*		MPU6050_FIFO[6]| ax6 | ay6 | az6 | gx6 |  gy6 | gz6 |temp6| 
	*   ---------------+-----+-----+-----+-----+------+-----+-----+
	*		MPU6050_FIFO[7]| ax7 | ay7 | az7 | gx7 |  gy7 | gz7 |temp7| 
	*   ---------------+-----+-----+-----+-----+------+-----+-----+
	*		MPU6050_FIFO[8]| ax8 | ay8 | az8 | gx8 |  gy8 | gz8 |temp8| 
	*   ---------------+-----+-----+-----+-----+------+-----+-----+
	*		MPU6050_FIFO[9]| ax9 | ay9 | az9 | gx9 |  gy9 | gz9 |temp9| <- ���һ�β�����ADCֵ
	*   ---------------+-----+-----+-----+-----+------+-----+-----+	
  *	 MPU6050_FIFO[10]| ax  | ay  | az  | gx  |  gy  | gz  |temp | <- ƽ��ֵ
	*   ---------------+-----+-----+-----+-----+------+-----+-----+
  */
void  MPU6050_PushFIFO(s16 ax, s16 ay, s16 az,
                        s16 gx, s16 gy, s16 gz,
                        s16 temp)
{
	u8 i ;
	s32 sum=0;
	
	for(i=1;i<10;i++)//FIFO������λ����
	{	
		MPU6050_FIFO[0][i-1]=MPU6050_FIFO[0][i];
		MPU6050_FIFO[1][i-1]=MPU6050_FIFO[1][i];
		MPU6050_FIFO[2][i-1]=MPU6050_FIFO[2][i];
		MPU6050_FIFO[3][i-1]=MPU6050_FIFO[3][i];
		MPU6050_FIFO[4][i-1]=MPU6050_FIFO[4][i];
		MPU6050_FIFO[5][i-1]=MPU6050_FIFO[5][i];
    	MPU6050_FIFO[6][i-1]=MPU6050_FIFO[6][i];
	}
	
	//���µ����ݷ��õ�FIFO�ĵ�9��
	MPU6050_FIFO[0][9]=ax;
	MPU6050_FIFO[1][9]=ay;
	MPU6050_FIFO[2][9]=az;
	MPU6050_FIFO[3][9]=gx;
	MPU6050_FIFO[4][9]=gy;
	MPU6050_FIFO[5][9]=gz;
  	MPU6050_FIFO[6][9]=temp;
	
	//����ƽ��ֵ�ŵ���10��
	sum=0;
	for(i=0;i<10;i++)
	{	
		 sum+=MPU6050_FIFO[0][i];
	}
	MPU6050_FIFO[0][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++)
	{
		 sum+=MPU6050_FIFO[1][i];
	}
	MPU6050_FIFO[1][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++)
	{
		 sum+=MPU6050_FIFO[2][i];
	}
	MPU6050_FIFO[2][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++)
	{
		 sum+=MPU6050_FIFO[3][i];
	}
	MPU6050_FIFO[3][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++)
	{
		 sum+=MPU6050_FIFO[4][i];
	}
	MPU6050_FIFO[4][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++)
	{
		 sum+=MPU6050_FIFO[5][i];
	}
	MPU6050_FIFO[5][10]=sum/10;
  
 	sum=0;
	for(i=0;i<10;i++)
	{
		 sum+=MPU6050_FIFO[6][i];
	}
	MPU6050_FIFO[6][10]=sum/10;
}

/**
  * @brief ����MPU6050ʱ��Դ
  * @param source: ʱ��Դ
  * @retval none
  * @note MPU6050_FIFO[6][11]�ṹ��
	*
	* CLK_SEL | Clock Source
	* --------+--------------------------------------
	* 0       | Internal oscillator
	* 1       | PLL with X Gyro reference
	* 2       | PLL with Y Gyro reference
	* 3       | PLL with Z Gyro reference
	* 4       | PLL with external 32.768kHz reference
	* 5       | PLL with external 19.2MHz reference
	* 6       | Reserved
	* 7       | Stops the clock and keeps the timing generator in reset
  */
void MPU6050_setClockSource(u8 source)
{
	I2C_StatusTypeDef status = I2C_ERROR;
	u8 tempreg = 0x00;
	
	status = I2C_ReadOneByte(MPU6050_DevAddr, MPU6050_PWR_MGMT_1, &tempreg);
	if(status != I2C_SUCCESS) return;
	tempreg &= ~0x07;	//PWR_MGMT_1[2:0]����
	tempreg |= source;//����ʱ��Դ
	I2C_WriteOneByte(MPU6050_DevAddr, MPU6050_PWR_MGMT_1, tempreg);//д�Ĵ���
}

/**
	* @brief ��������������
	* @param range: ����ֵ
	* 	@arg #define MPU6050_GYRO_FS_250		0x00
	* 	@arg #define MPU6050_GYRO_FS_500		0x01
	* 	@arg #define MPU6050_GYRO_FS_1000		0x02
	* 	@arg #define MPU6050_GYRO_FS_2000		0x03
	*	@retval None
	* @note: GYRO_CONFIG bit4~bit3
	*
	*/
void MPU6050_setFullScaleGyroRange(u8 range)
{
	I2C_StatusTypeDef status = I2C_ERROR;
	u8 tempreg = 0x00;
	
	status = I2C_ReadOneByte(MPU6050_DevAddr, MPU6050_GYRO_CONFIG, &tempreg);
	if(status != I2C_SUCCESS) return;
	tempreg &= ~(3<<3);			//GYRO_CONFIG[4:3]����
	tempreg |= (range<<3);	//����FS_SEL
	I2C_WriteOneByte(MPU6050_DevAddr, MPU6050_GYRO_CONFIG, tempreg);//д�Ĵ���
}

/**
	* @brief ���ü��ٶ�����
	* @param range: ����ֵ
	* 	@arg #define MPU6050_ACCEL_FS_2   0x00
	* 	@arg #define MPU6050_ACCEL_FS_4   0x01
	* 	@arg #define MPU6050_ACCEL_FS_8   0x02
	* 	@arg #define MPU6050_ACCEL_FS_16  0x03
	*	@retval None
	* @note: ACCEL_CONFIG bit4~bit3
	*
	*/
void MPU6050_setFullScaleAccelRange(u8 range)
{
	I2C_StatusTypeDef status = I2C_ERROR;
	u8 tempreg = 0x00;
	
	status = I2C_ReadOneByte(MPU6050_DevAddr, MPU6050_ACCEL_CONFIG, &tempreg);
	if(status != I2C_SUCCESS) return;
	tempreg &= ~(3<<3);			//ACCEL_CONFIG[4:3]����
	tempreg |= (range<<3);	//����AFS_SEL
	I2C_WriteOneByte(MPU6050_DevAddr, MPU6050_ACCEL_CONFIG, tempreg);//д�Ĵ���
}

/**
	* @brief ����MPU6050��������
	* @param eable: 1 - ��������; 0 - ���빤��״̬
	*	@retval None
	* @note: PWR_MGMT_1 bit6
	*
	*/
void MPU6050_setSleepEnabled(u8 enable)
{	
	I2C_WriteBit(MPU6050_DevAddr, MPU6050_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, enable);
}

/**
	* @brief ��ȡMPU6050�豸ID
	* @param None
	*	@retval None
	* @note: Ĭ�϶�ȡ����ֵ��0x68���Ĵ�������:
	*		WHO_AM_I(Address: 0X75):
	*		+---+---+---+---+---+---+---+---+
	*		| 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
	*		+---+---+---+---+---+---+---+---+
	*		| - |        WHO_AM_I       | - |
	*		+---+---+---+---+---+---+---+---+
	*/
u8 MPU6050_getDeviceID(void)
{
	u8 data;
	
	I2C_ReadOneByte(MPU6050_DevAddr, MPU6050_WHO_AM_I, &data);
	return data;
}

/**
	* @brief ��ѯMPU6050 ADC�����Ƿ�׼����
	* @param None
	*	@retval 1 - ׼����  0 - δ׼����
	* @note: 
	*		WHO_AM_I(Address: 0X75):
	*		+---+-------+---+------------------+-----------+---+---+------------+
	*		| 7 |   6   | 5 |         4        |     3     | 2 | 1 |     0      |
	*		+---+-------+---+------------------+-----------+---+---+------------+
	*		| - |MOT_INT| - |FIFO OVER_FLOW_INT|I2C_MST_INT| - | - |DATA_RDY_INT|
	*		+---+-------+---+------------------+-----------+---+---+------------+
	*/
u8 MPU6050_is_DRY(void)
{

	u8 int_status = 0;
	
	if(MPU6050_INT_PIN == 1)
	{
		//��һ��״̬�Ĵ��������INT��־λ
		I2C_ReadOneByte(MPU6050_DevAddr,MPU6050_INT_STATUS, &int_status);
		return 1;
	}
	else return 0;
}

/**
	* @brief ��ȡMPU6050����ADCֵ
	* @param None
	*	@retval None
	* @note: 
	*		ADCת��ֵ(�Ĵ���59~72):
	*		+---------------------------------------+
	*	59|          ACCEL_XOUT[15:8]             |
	*		+---------------------------------------+
	*	60|          ACCEL_XOUT[7:0]              |
	*		+---------------------------------------+
	*	61|          ACCEL_YOUT[15:8]             |
	*		+---------------------------------------+
	*	62|          ACCEL_YOUT[7:0]              |
	*		+---------------------------------------+
	*	63|          ACCEL_ZOUT[15:8]             |
	*		+---------------------------------------+
	*	64|          ACCEL_ZOUT[7:0]              |
	*		+---------------------------------------+
	*	65|          TEMP_OUT[15:8]               |
	*		+---------------------------------------+
	*	66|          TEMP_OUT[7:0]                |
	*		+---------------------------------------+
	*	67|          GYRO_XOUT[15:8]              |
	*		+---------------------------------------+
	*	68|          GYRO_XOUT[7:0]               |
	*		+---------------------------------------+
	*	69|          GYRO_YOUT[15:8]              |
	*		+---------------------------------------+
	*	79|          GYRO_YOUT[7:0]               |
	*		+---------------------------------------+
	*	71|          GYRO_ZOUT[15:8]              |
	*		+---------------------------------------+
	*	72|          GYRO_ZOUT[7:0]               |
	*		+---------------------------------------+
	*/
void MPU6050_getADC(s16* ax, s16* ay, s16* az,
										s16* gx, s16* gy, s16* gz,
										s16* temp)
{
	s16 MPU6050_Lastax,MPU6050_Lastay,MPU6050_Lastaz,
					MPU6050_LastTemp,
					MPU6050_Lastgx,MPU6050_Lastgy,MPU6050_Lastgz;
	u8 buffer[14];
	
	//����׼����
	if(MPU6050_is_DRY())
	{
		//��ȡADCֵ
		I2C_ReadBurst(MPU6050_DevAddr, MPU6050_ACCEL_XOUT_H, buffer, 14);
		//���ٶ�ADCֵ��
	    MPU6050_Lastax=(((s16)buffer[0]) << 8) | buffer[1];
	    MPU6050_Lastay=(((s16)buffer[2]) << 8) | buffer[3];
	    MPU6050_Lastaz=(((s16)buffer[4]) << 8) | buffer[5];
		//�¶�ADCֵ��
		MPU6050_LastTemp = (((s16)buffer[6]) << 8) | buffer[7];
		//������ADCֵ��
	    MPU6050_Lastgx=(((s16)buffer[8]) << 8) | buffer[9];
	    MPU6050_Lastgy=(((s16)buffer[10]) << 8) | buffer[11];
	    MPU6050_Lastgz=(((s16)buffer[12]) << 8) | buffer[13];
		
		//��ADCֵѹ��MPU6050_FIFO:
		MPU6050_PushFIFO(MPU6050_Lastax,MPU6050_Lastay,MPU6050_Lastaz
			,MPU6050_Lastgx,MPU6050_Lastgy,MPU6050_Lastgz,MPU6050_LastTemp);
		MPU6050_UPDATE = 1;
	}
		
		//��ȡADCƽ��ֵ
		*ax  =MPU6050_FIFO[0][10];
		*ay  =MPU6050_FIFO[1][10];
		*az = MPU6050_FIFO[2][10];
		*gx  =MPU6050_FIFO[3][10]-Gx_offset;
		*gy = MPU6050_FIFO[4][10]-Gy_offset;
		*gz = MPU6050_FIFO[5][10]-Gz_offset;
		*temp = MPU6050_FIFO[6][10];

		
}

void MPU6050_getADC1(s16* ax, s16* ay, s16* az,
										s16* gx, s16* gy, s16* gz,
										s16* temp)
{
	s16 MPU6050_Lastax,MPU6050_Lastay,MPU6050_Lastaz,
					MPU6050_LastTemp,
					MPU6050_Lastgx,MPU6050_Lastgy,MPU6050_Lastgz;
	u8 buffer[14];
	
	//����׼����
	if(MPU6050_is_DRY())
	{
		//��ȡADCֵ
		I2C_ReadBurst(MPU6050_DevAddr, MPU6050_ACCEL_XOUT_H, buffer, 14);
		//���ٶ�ADCֵ��
	    MPU6050_Lastax=(((s16)buffer[0]) << 8) | buffer[1];
	    MPU6050_Lastay=(((s16)buffer[2]) << 8) | buffer[3];
	    MPU6050_Lastaz=(((s16)buffer[4]) << 8) | buffer[5];
		//�¶�ADCֵ��
		MPU6050_LastTemp = (((s16)buffer[6]) << 8) | buffer[7];
		//������ADCֵ��
	    MPU6050_Lastgx=(((s16)buffer[8]) << 8) | buffer[9];
	    MPU6050_Lastgy=(((s16)buffer[10]) << 8) | buffer[11];
	    MPU6050_Lastgz=(((s16)buffer[12]) << 8) | buffer[13];
		
		
		MPU6050_UPDATE = 1;
	}

	*ax = MPU6050_Lastax;
	*ay = MPU6050_Lastay;
	*az = MPU6050_Lastaz;
	*gx = MPU6050_Lastgx;
	*gy = MPU6050_Lastgy;
	*gz = MPU6050_Lastgz;
	*temp = MPU6050_LastTemp;
			
}



/**
	* @brief ���㷽���
	* @param ax,ay,az : ���ٶȼ�ADCֵ
	* @param dir : 0,��Z��ĽǶ�;1,��X��ĽǶ�;2,��Y��ĽǶ�
	*	@retval �Ƕ�ֵ����λ0.1��
	* @note: 
	*/
short MPU6050_Get_Angle(float ax, float ay, float az, u8 dir)
{
	float temp;
 	float res=0;
	switch(dir)
	{
		case 0://����ȻZ��ĽǶ�
 			temp = sqrt((ax * ax + ay * ay)) / az;
 			res = atan(temp);
 			break;
		case 1://����ȻX��ĽǶ�
 			temp = ax/sqrt((ay * ay + az * az));
 			res = atan(temp);
 			break;
 		case 2://����ȻY��ĽǶ�
 			temp = ay / sqrt((ax*ax + az*az));
 			res = atan(temp);
 			break;
 	}
	return res * 1800 / 3.14;
}

/**
	* @brief  �����ǽ���
	* @param  None
	*	@retval None
	* @note   ���������뱣�ְ��Ӿ�ֹ
	*/
void MPU6050_InitGyro_Offset(void)
{
	u8 i;
	s16 temp[7];
	s32	tempgx=0,tempgy=0,tempgz=0;
	
	//����ǰ50�εĲ���ֵ
	for(i=0;i<50;i++)
	{
		Delay_us(100);
		MPU6050_getADC(&temp[0],&temp[1],&temp[2],&temp[3],&temp[4],&temp[5],&temp[6]);
	}
	
	//����100�������ǲ���ֵ��ƽ��ֵ
 	for(i=0;i<100;i++)
	{
		Delay_us(200);
		MPU6050_getADC(&temp[0],&temp[1],&temp[2],&temp[3],&temp[4],&temp[5],&temp[6]);
		tempgx+= temp[3];
		tempgy+= temp[4];
		tempgz+= temp[5];
	}
	
	MPU6050_Config.dGx_offset = Gx_offset = tempgx / 100;
	MPU6050_Config.dGy_offset = Gy_offset = tempgy / 100;
	MPU6050_Config.dGz_offset = Gz_offset = tempgz / 100;
	
	MPU6050_WriteConfig();//����������д��EEPROM
}


/**
  * @brief  MPU6050 INT���ų�ʼ��
  * @param  None
  * @retval 0 - ��ʼ���ɹ���1 - ��ʼ��ʧ��
  * @note  
  */
void MPU6050_GPIO_init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure; 

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);	/*��APB2�����ϵ�GPIOAʱ��*/

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;  
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	GPIO_ResetBits(GPIOE, GPIO_Pin_11);
}

/**
  * @brief  MPU6050��ʼ��
  * @param  None
  * @retval 0 - ��ʼ���ɹ���1 - ��ʼ��ʧ��
  * @note  
  */
u8 MPU6050_init(void)
{
	s16 temp[7];
	u8 i;

	MPU6050_GPIO_init();
	
	if(MPU6050_getDeviceID() != 0x68)
	{
		printf("FIND MPU6050 ERROR!\r\n");
		return 1;	//I2C����ʧ��
	}
	
	MPU6050_setClockSource(MPU6050_CLOCK_PLL_XGYRO);		//����ʱ��
	MPU6050_setFullScaleGyroRange(MPU6050_GYRO_FS_250);//������������� +-1000��ÿ��
	MPU6050_setFullScaleAccelRange(MPU6050_ACCEL_FS_2);	//���ٶȶ�������� +-2G
	MPU6050_setSleepEnabled(0);					//���빤��״̬

	//����MPU6050 ����AUXI2C
	I2C_WriteBit(MPU6050_DevAddr, MPU6050_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT, 0);
	//����������I2C��	MPU6050��AUXI2C	ֱͨ������������ֱ�ӷ���HMC5883L
	I2C_WriteBit(MPU6050_DevAddr, MPU6050_INT_PIN_CFG, MPU6050_INTCFG_I2C_BYPASS_EN_BIT, 1);

	//����MPU6050 ���ж�ģʽ ���жϵ�ƽģʽ
	I2C_WriteBit(MPU6050_DevAddr, MPU6050_INT_PIN_CFG, MPU6050_INTCFG_INT_LEVEL_BIT, 0);	//�����ж�ʱ��INTΪ�ߵ�ƽ
	I2C_WriteBit(MPU6050_DevAddr, MPU6050_INT_PIN_CFG, MPU6050_INTCFG_INT_OPEN_BIT, 0);		//�������
	I2C_WriteBit(MPU6050_DevAddr, MPU6050_INT_PIN_CFG, MPU6050_INTCFG_LATCH_INT_EN_BIT, 1);//ͨ������жϱ�־ʹINT�ָ̻ܽ��͵�ƽ
	I2C_WriteBit(MPU6050_DevAddr, MPU6050_INT_PIN_CFG, MPU6050_INTCFG_INT_RD_CLEAR_BIT, 1);//�κζ������������INT�ܽ�״̬
	//������ת������ж�
	I2C_WriteBit(MPU6050_DevAddr, MPU6050_INT_ENABLE, MPU6050_INTERRUPT_DATA_RDY_BIT, 1);	
	
	//MPU6050_LoadConfig();	//��EEPROM�ж�ȡ������Ϣ

	/*
	Gx_offset = MPU6050_Config.dGx_offset;
	Gy_offset = MPU6050_Config.dGy_offset;
	Gz_offset = MPU6050_Config.dGz_offset;
	*/

	Gx_offset = 0;
	Gy_offset = 0;
	Gz_offset = 0;

	for(i=0;i<10;i++)//����FIFO����
	{
		Delay_us(50);
		MPU6050_getADC(&temp[0],&temp[1],&temp[2],&temp[3],&temp[4],&temp[5],&temp[6]);
	}
	
	return 0;
}

/**
  * @brief  ��ȡEEPROM�е�������Ϣ�����������Ϣ��Ч����Ĭ��ֵд��EEPROM
  * @param  None
  * @retval None
	* @note   ���ݴ洢��EEPROM�����ַ0~9������10����ַ
	*
	* 	�����ַ|   MPU6050_Config:
  	* ----------+----------------------------
	*      0    | MPU6050_Config.is_good;	
	*      1    | MPU6050_Config.dGx_offset;
	*      2    | MPU6050_Config.dGy_offset;
	*      3    | MPU6050_Config.dGz_offset;   
	*      4    | MPU6050_Config.dMx_offset;
	*      5    | MPU6050_Config.dMy_offset;
	*      6    | MPU6050_Config.dMz_offset;
	*      7    | MPU6050_Config.dMx_scale;
	*      8    | MPU6050_Config.dMy_scale;
	*      9    | MPU6050_Config.dMz_scale;
	*/
void MPU6050_LoadConfig(void)
{
	//��ȡEEPROM�洢������������Ϣ
	//EE_ReadBuf(0, &MPU6050_Config.is_good, sizeof(MPU6050_Config)/2);
	
	if(MPU6050_Config.is_good != 0xABCD)//������Ч ����ʱ��Ҫװ��Ĭ��ֵ��
	{
		MPU6050_Config.is_good = 0xABCD;
		MPU6050_Config.dGx_offset = 0;
		MPU6050_Config.dGy_offset = 0;
		MPU6050_Config.dGz_offset = 0;
	
		MPU6050_Config.dMx_offset = 0;
		MPU6050_Config.dMy_offset = 0;
		MPU6050_Config.dMz_offset = 0;
	
		MPU6050_Config.dMx_scale = 1.0f;
		MPU6050_Config.dMy_scale = 1.0f;
		MPU6050_Config.dMz_scale = 1.0f;

		MPU6050_WriteConfig();	//��Ĭ��ֵд��flash
	}
}

/**
  * @brief  ����ǰ������Ϣд��EEPROM
  * @param  None
  * @retval None
	* @note   �����ݴ洢��EEPROM�������ַ0~9
	*/
void MPU6050_WriteConfig(void)
{
	//EE_WriteBuf(0, &MPU6050_Config.is_good, sizeof(MPU6050_Config)/2);
}

