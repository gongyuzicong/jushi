#include "i2c_opts.h"
#include "cfg_gpio.h"



// I2C�����ٶ� = I2C_SPEED_1K / i2c_speed
u32	i2c_speed = I2C_SPEED_1K / 100;

/**
  * @brief  ģ��I2C�ӿڳ�ʼ��
  * @param  None
  * @retval None
  * @note
	*		SCL: 	PB10
	*		SDA:	PB11
  */
void My_I2C_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	//ʹ��GPIOB����ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	
	
	//I2C��SDA��SCL����Ҫ��Ӳ���������������裬�����������Ϊ�������
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	//��©���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_SetBits(GPIOB, GPIO_Pin_6 | GPIO_Pin_7);//�����������ó�ʼ��GPIO

	//����SCL��SDA����״̬Ϊ�ߵ�ƽ
	I2C_SCL = 1;
	I2C_SDA = 1;
	
	I2C_SetSpeed(100);//����I2C�����ٶ�Ϊ100Kbps
}


void Change_SDA_IN(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	//ʹ��GPIOB����ʱ��
	
	//I2C��SDA��SCL����Ҫ��Ӳ���������������裬�����������Ϊ�������
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;		//��������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void Change_SDA_OUT(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	//ʹ��GPIOB����ʱ��
	
	//I2C��SDA��SCL����Ҫ��Ӳ���������������裬�����������Ϊ�������
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	//��©���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

/**
	* @brief ����I2C��ʼ�ź�
  * @param  None
  * @retval None
  * @note
	*		��ο�I2Cͨ��Э�飬I2C��ʼ�źţ���SCLΪ�ߵ�ƽʱ��SDA�ɸ߱��
	*		����ͼ��ʾ:���򲿷ֱ�ʾI2C��ʼ�ź�
	*           _____     |
	*        __|__   |    |  ___  ___  ___  ___  ___  ___  ___  ___  
	*   SDA:   |  \__|____|_/   \/   \/   \/   \/   \/   \/   \/   \     /
	*          |     |    | \___/\___/\___/\___/\___/\___/\___/\___/\___/
	*        __|_____|_   |   _    _    _    _    _    _    _    _    _
	*   SCL:   |     | \__|__/ \__/ \__/ \__/ \__/ \__/ \__/ \__/ \__/ \_
	*          |_____|    | 
	*           start         D7   D6   D5   D4   D3   D2   D1   D0   ACK
	*/
void I2C_Start(void)
{
	u32 i2c_delay = i2c_speed;
	
	SDA_OUT();	//SDA����Ϊ���
	I2C_SDA = 1;	//SDA: ��
	I2C_SCL = 1;	//SCL: ��
	i2c_delay = i2c_speed;//��ʱ>4.7us
	while(i2c_delay--){}
 	I2C_SDA = 0;	//��SCLΪ�ߵ�ƽʱ��SDA�ɸ߱��
	i2c_delay = i2c_speed;//��ʱ>4us
	while(i2c_delay--){}
	I2C_SCL = 0;	//SCL��ͣ�ǯסI2C���ߣ�׼�����ͻ�������� 
}

/**
	* @brief ����I2Cֹͣ�ź�
  * @param  None
  * @retval None
  * @note
	*		��ο�I2Cͨ��Э�飬I2Cֹͣ�źţ���SCLΪ�ߵ�ƽʱ��SDA�ɵͱ��
	*		������STOP�źź�SCL��SDA��Ϊ�ߵ�ƽ�����ͷ���I2C����
	*		����ͼ��ʾ:���򲿷ֱ�ʾI2Cֹͣ�ź�
	*                                    _____
	*         ___  ___  ___  ___        |   __|_
	*   SDA: /   \/   \/   \/   \       |  /  |
	*        \___/\___/\___/\___/\______|_/   |
	*         _    _    _    _    _    _|_____|_
	*   SCL: / \__/ \__/ \__/ \__/ \__/ |     |
	*                                   |_____|
	*        D3   D2   D1   D0   ACK     stop
	*/
void I2C_Stop(void)
{
	u32 i2c_delay = i2c_speed;
	
	SDA_OUT(); 		//SDA����Ϊ���
	I2C_SDA = 0;	//SDA�͵�ƽ
	I2C_SCL = 1;	//SCL�ߵ�ƽ
 	i2c_delay = i2c_speed;//��ʱ>4us
	while(i2c_delay--){}
	I2C_SDA = 1;	//STOP:��SCLΪ�ߵ�ƽʱ��SDA�ɵͱ��
	i2c_delay = i2c_speed;
	while(i2c_delay--){}//��ʱ>4.7us						   	
}

/**
	* @brief  �ȴ�ACKӦ���ź�
  * @param  None
  * @retval 1 - δ���յ�Ӧ���ź�ACK��0 - ���յ�Ӧ���ź�ACK
  * @note
	*		��ο�I2Cͨ��Э�飬���ACKӦ���źţ���SCLΪ�ߵ�ƽʱ����ȡSDAΪ�͵�ƽ
	*		����ͼ��ʾ:���򲿷ֱ�ʾI2C�յ�Ӧ���ź�
	*                             ________     _____
	*         ___  ___  ___  ___ | _      |   |   __|_
	*   SDA: /   \/   \/   \/   \|/ \     |   |  /  |
	*        \___/\___/\___/\___/|   \____|___|_/   |
	*         _    _    _    _   | _____  |  _|_____|
	*   SCL: / \__/ \__/ \__/ \__|/     \_|_/ |     |
	*                            |________|   |_____|
	*        D3   D2   D1   D0      ACK        stop
	*/
u8 I2C_Wait_ACK(void)
{
	u32 i2c_delay = i2c_speed;
	u8 timeout = 0;
	
	SDA_IN();			//SDA����Ϊ����
	I2C_SDA = 1;	//SDA��������
	I2C_SCL=1;	//SCL����Ϊ�ߵ�ƽ
	i2c_delay = i2c_speed;
	//printf("i2c_speed = %d\r\n", i2c_speed);
	while(i2c_delay--){}
	while(READ_SDA == 1)//�ȴ�ACK
	{
		if(timeout++ > 250)
		{
			I2C_Stop();
			return 1;
		}
	}
	
	I2C_SCL = 0;//ǯסI2C���ߣ�ʱ���ź���Ϊ�͵�ƽ
	return 0;  
}


/**
	* @brief  ����ACKӦ���ź�
  * @param  None
  * @retval None
  * @note
	*		��ο�I2Cͨ��Э�飬����ACKӦ���ź�: ��SDAΪ�͵�ƽʱ��SCL����һ��������
	*		����ͼ��ʾ:���򲿷ֱ�ʾI2CӦ���ź�
	*                             _____     _____
	*         ___  ___  ___  ___ |     |   |   __|_
	*   SDA: /   \/   \/   \/   \|     |   |  /  |
	*        \___/\___/\___/\___/|\____|___|_/   |
	*         _    _    _    _   |  _  |  _|_____|_
	*   SCL: / \__/ \__/ \__/ \__|_/ \_|_/ |     |
	*                            |_____|   |_____|
	*        D3   D2   D1   D0     ACK      stop
	*/
void I2C_ACK(void)
{
	u32 i2c_delay = i2c_speed;
	
	I2C_SCL = 0;	//�͵�ƽ
	SDA_OUT();		//����SDAΪ���
	I2C_SDA = 0;	//ACK�ź�
	i2c_delay = i2c_speed;
	while(i2c_delay--){}//��ʱ>4us
	I2C_SCL = 1;	//�ߵ�ƽ
	i2c_delay = i2c_speed;
	while(i2c_delay--){}//��ʱ>4us
	I2C_SCL = 0;	//ǯסI2C���ߣ�ʱ���ź���Ϊ�͵�ƽ
}


/**
	* @brief  ������Ӧ���ź�NACK
  * @param  None
  * @retval None
  * @note
	*		��ο�I2Cͨ��Э�飬����ACK��Ӧ���ź�: ��SDAΪ�ߵ�ƽʱ��SCL����һ��������
	*		����ͼ��ʾ:���򲿷ֱ�ʾI2C��Ӧ���ź�
	*                             _____      ______
	*         ___  ___  ___  ___ | ____|_   |    __|_
	*   SDA: /   \/   \/   \/   \|/    | \  |   /  |
	*        \___/\___/\___/\___/|     |  \_|__/   |
	*         _    _    _    _   |  _  |  __|______|_
	*   SCL: / \__/ \__/ \__/ \__|_/ \_|_/  |      |
	*                            |_____|    |______|
	*        D3   D2   D1   D0    NACK        stop
	*/	    
void I2C_NACK(void)
{
	u32 i2c_delay = i2c_speed;
	
	I2C_SCL = 0;	//�͵�ƽ
	SDA_OUT();		//SDA����Ϊ���
	I2C_SDA = 1;	//NACK�ź�
	i2c_delay = i2c_speed;
	while(i2c_delay--){}//��ʱ>4us
	I2C_SCL = 1;	//�ߵ�ƽ
	i2c_delay = i2c_speed;
	while(i2c_delay--){}//��ʱ>4us
	I2C_SCL = 0;	//ǯסI2C���ߣ�ʱ���ź���Ϊ�͵�ƽ
}


/**
	* @brief  I2C����һ���ֽ�
  * @param  None
  * @retval None
  * @note
	*		��ο�I2Cͨ��Э�飬����ACKӦ���ź�: ��SDAΪ�ߵ�ƽʱ��SCL����һ��������
	*		����ͼ��ʾ:���򲿷ֱ�ʾI2C��ʼ�ź�
	*
	*           _____     |<------------I2C���ݷ�������------------>|
	*        __|__   |    |  ___  ___  ___  ___  ___  ___  ___  ___ | _ 
	*   SDA:   |  \__|____|_/   \/   \/   \/   \/   \/   \/   \/   \|/ 
	*          |     |    | \___/\___/\___/\___/\___/\___/\___/\___/|\_
	*        __|_____|_   |   _    _    _    _    _    _    _    _  |  
	*   SCL:   |     | \__|__/ \__/ \__/ \__/ \__/ \__/ \__/ \__/ \_|_
	*          |_____|    |                                         |
	*           start     |   D7   D6   D5   D4   D3   D2   D1   D0 |
	*/
void I2C_Send_Byte(u8 data)
{                        
	u8 i = 0;
	u32 i2c_delay = i2c_speed;

	SDA_OUT();									//SDA��Ϊ���
	I2C_SCL = 0;								//ǯסI2C���ߣ�SCL��Ϊ�͵�ƽ
	for(i = 0; i < 8; i++)
	{
		if(data&0x80)I2C_SDA = 1;	//��λ�ȴ�
		else I2C_SDA = 0;
		
		i2c_delay = i2c_speed;
		while(i2c_delay--){}			//��ʱ>4us
	
		I2C_SCL = 1;							//��SCL�ϲ���һ��������
		i2c_delay = i2c_speed;
		while(i2c_delay--){}			//��ʱ>4us
			
		I2C_SCL=0;
		i2c_delay = i2c_speed/3;
		while(i2c_delay--){}			//��ʱ>1us
		data <<= 1;								//����һλ
	}
}


/**
	* @brief  ��I2C��ȡһ���ֽ�
  * @param  ack : 0 - NACK; 1 - ACK
  * @retval ���յ�������
  * @note
	*		��ο�I2Cͨ��Э�飬����ACKӦ���ź�: ��SDAΪ�ߵ�ƽʱ��SCL����һ��������
	*		����ͼ��ʾ:���򲿷ֱ�ʾI2C��ʼ�ź�
	*
	*           _____     |<------------I2C���ݶ�ȡ����(ACK)------------>|
	*        __|__   |    |  ___  ___  ___  ___  ___  ___  ___  ___      |
	*   SDA:   |  \__|____|_/   \/   \/   \/   \/   \/   \/   \/   \     |
	*          |     |    | \___/\___/\___/\___/\___/\___/\___/\___/\____|_
	*        __|_____|_   |   _    _    _    _    _    _    _    _    _  |
	*   SCL:   |     | \__|__/ \__/ \__/ \__/ \__/ \__/ \__/ \__/ \__/ \_|_
	*          |_____|    |                                              |
	*           start     |   D7   D6   D5   D4   D3   D2   D1   D0   ACK
	*
	*           _____     |<------------I2C���ݶ�ȡ����(NACK)----------->|
	*        __|__   |    |  ___  ___  ___  ___  ___  ___  ___  ___  ____|_
	*   SDA:   |  \__|____|_/   \/   \/   \/   \/   \/   \/   \/   \/    |
	*          |     |    | \___/\___/\___/\___/\___/\___/\___/\___/     |
	*        __|_____|_   |   _    _    _    _    _    _    _    _    _  |
	*   SCL:   |     | \__|__/ \__/ \__/ \__/ \__/ \__/ \__/ \__/ \__/ \_|_
	*          |_____|    |                                              |
	*           start     |   D7   D6   D5   D4   D3   D2   D1   D0  NACK
	*/
u8 I2C_Read_Byte(u8 ack)
{
	u8 i, receive = 0x00;
	u32 i2c_delay = i2c_speed;
	
	I2C_SCL = 0;									//SCL�͵�ƽ
	SDA_IN();											//SDA����Ϊ����
	for(i = 0; i < 8; i++)
	{
		i2c_delay = i2c_speed;
		while(i2c_delay--);
		I2C_SCL = 1;								//�ߵ�ƽ
		i2c_delay = i2c_speed;
		while(i2c_delay--);
		receive <<= 1;
		if(READ_SDA) receive |= 1;	//��λ��ǰ
		I2C_SCL = 0;
	}
	if (ack == 0) I2C_NACK();			//����NACK
	else I2C_ACK();								//����ACK
	
	return receive;								//���ؽ��յ�������
}

/**
  * @brief  ����I2C�ٶ�
  * @param  speed : I2C�ٶȣ���λKbps
  * @retval ��������ǰ��I2C�ٶ�
	* @note   I2C�ٶ����÷�Χ��: 1Kbps ~ 400Kbps
  */
u16 I2C_SetSpeed(u16 speed)
{
	u16 temp;
	
	//I2C�ٶȱ���С��400Kbps������ 1Kbps
	if((speed > 400)|| (speed < 1)) return 0;
	
	temp = I2C_SPEED_1K / i2c_speed;	//����ԭ����i2c�ٶ�
	i2c_speed = I2C_SPEED_1K / speed;	//�����µ�i2c�ٶ�

	return temp;	//��������ǰ��i2c�ٶ�
}

/* ---------------------------���²����Ƿ�װ�õ�I2C��д����--------------------------- */

//���嵽ĳһ������������ϸ�Ķ�������������I2C���ֵ�˵������ΪĳЩ����I2C�Ķ�д������
//��һЩ���죬����Ĵ��������ھ��������I2C�����У�������֤OK�ģ�

/**
  * @brief  ���豸ָ����ַд�뵥һByte����
  * @param  DevAddr : I2C���豸��ַ
  * @param  DataAddr: ��Ҫ���ʵ��豸�ڵ�ַ(��Ĵ�����ַ��EEPROM��ַ��)
  * @param  Data    : д�������
  * @retval I2C���ʵĽ��: I2C_SUCCESS / I2C_TIMEOUT / I2C_ERROR
	* @note   
	*   1 - �豸��ַDevAddr��7bit�ǹ̶��ģ����Ϊ�Ƕ�/д(R/W)λ��1Ϊ����0Ϊд
	*		2 - ʱ��
	*           _______________________________________
	*          | |         |   |        |   |    |   | |
	*   Master:|S|DevAddr+W|   |DataAddr|   |Data|   |P|
	*          |_|_________|___|________|___|____|___|_|
	*           _______________________________________
	*          | |         |   |        |   |    |   | |
	*   Slave: | |         |ACK|        |ACK|    |ACK| |
	*          |_|_________|___|________|___|____|___|_|
  */
I2C_StatusTypeDef I2C_WriteOneByte(u8 DevAddr, u8 DataAddr, u8 Data)
{
	I2C_Start();													//Master������ʼ�ź�
	I2C_Send_Byte(DevAddr);								//Master���ʹ��豸��ַ
	if(I2C_Wait_ACK()) return I2C_TIMEOUT;//�ȴ�ACK��ʱ����
	I2C_Send_Byte(DataAddr);							//�������ݵ�ַ
	if(I2C_Wait_ACK()) return I2C_TIMEOUT;//�ȴ�ACK��ʱ����
	I2C_Send_Byte(Data);									//��������
	if(I2C_Wait_ACK()) return I2C_TIMEOUT;//�ȴ�ACK��ʱ����
	I2C_Stop();														//����ֹͣ�ź�
	return I2C_SUCCESS;
}

/**
  * @brief  ���豸ָ����ַ����д������(Burstдģʽ)
  * @param  DevAddr : I2C���豸��ַ
  * @param  DataAddr: ��Ҫ���ʵ��豸�ڵ�ַ(��Ĵ�����ַ��EEPROM��ַ��)
	*                   ����Burstģʽ��DataAddrһ�����豸��FIFO,���棬��洢�豸�����ݵ�ַ
  * @param  *pData  : д��������׵�ַ
  * @param     Num  : ����д������ݸ���
  * @retval I2C���ʵĽ��: I2C_SUCCESS / I2C_TIMEOUT / I2C_ERROR
	* @note   
	*   1 - �豸��ַDevAddr��7bit�ǹ̶��ģ����Ϊ�Ƕ�/д(R/W)λ��1Ϊ����0Ϊд
	*		2 - ʱ��
	*           ____________________________________________________
	*          | |         |   |        |   |    |   |   |    |   | |
	*   Master:|S|DevAddr+W|   |DataAddr|   |Data|   |...|Data|   |P|
	*          |_|_________|___|________|___|____|___|___|____|___|_|
	*           ____________________________________________________
	*          | |         |   |        |   |    |   |   |    |   | |
	*   Slave: | |         |ACK|        |ACK|    |ACK|...|    |ACK| |
	*          |_|_________|___|________|___|____|___|___|____|___|_|
  */
I2C_StatusTypeDef I2C_WriteBurst(u8 DevAddr, u8 DataAddr, u8* pData, u32 Num)
{
	u32 i = 0;
	
	I2C_Start();													//Master������ʼ�ź�
	I2C_Send_Byte(DevAddr);								//Master���ʹ��豸��ַ
	if(I2C_Wait_ACK()) return I2C_TIMEOUT;//�ȴ�ACK��ʱ����
	I2C_Send_Byte(DataAddr);							//�������ݵ�ַ
	if(I2C_Wait_ACK()) return I2C_TIMEOUT;//�ȴ�ACK��ʱ����
	for(i = 0; i < Num; i++)
	{
		I2C_Send_Byte(*(pData+i));						//��������
		if(I2C_Wait_ACK()) return I2C_TIMEOUT;//�ȴ�ACK��ʱ����
	}
	
	I2C_Stop();	//����ֹͣ�ź�
	return I2C_SUCCESS;
}

/**
  * @brief  ��ָ���豸��ȡ1Byte����
  * @param  DevAddr : I2C���豸��ַ
  * @param  DataAddr: ��Ҫ���ʵ��豸�ڵ�ַ(��Ĵ�����ַ��EEPROM��ַ��)
  * @param  *Data   : ���ݵĴ�ŵ�ַ
  * @retval I2C���ʵĽ��: I2C_SUCCESS / I2C_TIMEOUT / I2C_ERROR
	* @note   
	*   1 - �豸��ַDevAddr��7bit�ǹ̶��ģ����Ϊ�Ƕ�/д(R/W)λ��1Ϊ����0Ϊд
	*		2 - ʱ��
	*           _________________________________________________________
	*          | |         |   |        |    | |         |   |    |    | |
	*   Master:|S|DevAddr+W|   |DataAddr|    |S|DevAddr+R|   |    |NACK|P|
	*          |_|_________|___|________|____|_|_________|___|____|____|_|
	*           _________________________________________________________
	*          | |         |   |        |    | |         |   |    |    | |
	*   Slave: | |         |ACK|        |ACK | |         |ACK|Data|    | |
	*          |_|_________|___|________|____|_|_________|___|____|____|_|
  */
I2C_StatusTypeDef I2C_ReadOneByte(u8 DevAddr, u8 DataAddr, u8* Data)
{
	I2C_Start();													//Master������ʼ�ź�
	I2C_Send_Byte(DevAddr);								//Master���ʹ��豸��ַ
	if(I2C_Wait_ACK()) 
	{
		printf("I2C_TIMEOUT1\r\n");
		return I2C_TIMEOUT;//�ȴ�ACK��ʱ����
	}
	I2C_Send_Byte(DataAddr);							//�������ݵ�ַ

	if(I2C_Wait_ACK())
	{
		printf("I2C_TIMEOUT2\r\n");
		return I2C_TIMEOUT;//�ȴ�ACK��ʱ����
	}
	I2C_Stop();
	//Delay_ms(1);
	I2C_Start();													//Master������ʼ�ź�
	I2C_Send_Byte(DevAddr+1);							//Master���ʹ��豸����ַ
	if(I2C_Wait_ACK())
	{
		printf("I2C_TIMEOUT3\r\n");
		return I2C_TIMEOUT;//�ȴ�ACK��ʱ����
	}
	*Data = I2C_Read_Byte(0);							//�����ݣ�NACK
	I2C_Stop();														//����ֹͣ�ź�
	return I2C_SUCCESS;
}

/**
  * @brief  ���豸ָ����ַ����д������(Burstдģʽ)
  * @param  DevAddr : I2C���豸��ַ
  * @param  DataAddr: ��Ҫ���ʵ��豸�ڵ�ַ(��Ĵ�����ַ��EEPROM��ַ��)
	*                   ����Burstģʽ��DataAddrһ�����豸��FIFO,���棬��洢�豸�����ݵ�ַ
  * @param  *pData  : д��������׵�ַ
  * @param     Num  : ����д������ݸ���
  * @retval	I2C���ʵĽ��: I2C_SUCCESS / I2C_TIMEOUT / I2C_ERROR
	* @note   
	*   1 - �豸��ַDevAddr��7bit�ǹ̶��ģ����Ϊ�Ƕ�/д(R/W)λ��1Ϊ����0Ϊд
	*		2 - ʱ��
	*           _____________________________________________________________________
	*          | |         |   |        |   | |         |   |    |   |   |    |    | |
	*   Master:|S|DevAddr+W|   |DataAddr|   |S|DevAddr+R|   |    |ACK|...|    |NACK|P|
	*          |_|_________|___|________|___|_|_________|___|____|___|___|____|____|_|
	*           _____________________________________________________________________
	*          | |         |   |        |   | |         |   |    |   |   |    |    | |
	*   Slave: | |         |ACK|        |ACK| |         |ACK|Data|   |...|Data|    | |
	*          |_|_________|___|________|___|_|_________|___|____|___|___|____|____|_|
  */
I2C_StatusTypeDef I2C_ReadBurst(u8 DevAddr, u8 DataAddr, u8* pData, u32 Num)
{
	u32 i = 0;
	
	I2C_Start();													//Master������ʼ�ź�
	I2C_Send_Byte(DevAddr);								//Master���ʹ��豸��ַ
	if(I2C_Wait_ACK()) return I2C_TIMEOUT;//�ȴ�ACK��ʱ����
	I2C_Send_Byte(DataAddr);							//�������ݵ�ַ
	if(I2C_Wait_ACK()) return I2C_TIMEOUT;//�ȴ�ACK��ʱ����
	
	I2C_Start();													//Master������ʼ�ź�
	I2C_Send_Byte(DevAddr+1);							//Master���ʹ��豸����ַ
	if(I2C_Wait_ACK()) return I2C_TIMEOUT;//�ȴ�ACK��ʱ����
	
	for(i = 0; i < (Num-1); i++)
	{
		*(pData+i) = I2C_Read_Byte(1);			//�����ݣ�ACK
	}
	*(pData+i) = I2C_Read_Byte(0);				//�����ݣ�NACK
	
	I2C_Stop();														//����ֹͣ�ź�
	return I2C_SUCCESS;
}


/**
  * @brief  �������ݵ�ĳһλ
  * @param  DevAddr : I2C���豸��ַ
  * @param  DataAddr: ��Ҫ���ʵ��豸�ڵ�ַ(��Ĵ�����ַ��EEPROM��ַ��)
  * @param  Bitx  : �ڼ�λ
  * @param  BitSet: ��Ҫ���õ�ֵ
  * @retval I2C���ʵĽ��: I2C_SUCCESS / I2C_TIMEOUT / I2C_ERROR
	* @note  
	*/
I2C_StatusTypeDef I2C_WriteBit(u8 DevAddr, u8 DataAddr, u8 Bitx, u8 BitSet)
{
	I2C_StatusTypeDef status = I2C_ERROR;
	u8 tempdata = 0;
	
	status = I2C_ReadOneByte(DevAddr, DataAddr, &tempdata);	//��ȡԭ������
	if(status != I2C_SUCCESS) return status;								//I2C�����򷵻�
	
	tempdata &= ~(1<<Bitx);																	//��Ҫ�趨��λ����
	tempdata |= (BitSet<<Bitx);															//����ָ����bit
	status = I2C_WriteOneByte(DevAddr, DataAddr, tempdata);	//д������
	
	return status;	//����״̬
}







