/*
 * ANO_DTP.c
 *
 *  Created on: 2022��1��10��
 *      Author: HANYING
 */

#include "ANO_DTP.h"
#include "../LIBRARY/HY_UART.h"//���޸ĳ����Լ��Ĵ��ں���ͷ�ļ�

//ע���޸�void ANO_Report(u8 fun, u8*data, u8 len)�������UART_PutChar(USART1, send_buf[i]);
//ע���޸�void ANO_Report(u8 fun, u8*data, u8 len)�������UART_PutChar(USART1, send_buf[i]);
//ע���޸�void ANO_Report(u8 fun, u8*data, u8 len)�������UART_PutChar(USART1, send_buf[i]);
//��Ҫ������˵����

/*************************************************�����ķֽ���************************************************/

///*********************************************************************************************************
//*  �������ƣ�void ANO_Report(u8 fun, u8*data, u8 len)
//*  ����˵�����ڲ�����/����ʹ��
//*  ����˵����֡ID,���飬����
//*  �������أ���
//*  �޸�ʱ�䣺2021��1��10��
//*  ��    ע����
//**********************************************************************************************************/
void ANO_Report(u8 fun, u8*data, u8 len)
{
	unsigned char send_buf[35];
	unsigned char i;
	if (len > 30)
		return; //���30�ֽ�����

	send_buf[len + 4] = 0;  //У��������
	send_buf[len + 5] = 0;  //У��������
	send_buf[0] = 0XAA;   //֡ͷ
	send_buf[1] = 0XFF;   //Ŀ���ַ
	send_buf[2] = fun;    //������
	send_buf[3] = len;    //���ݳ���

	for (i = 0; i < len; i++)
		send_buf[4 + i] = data[i];          //��������

	for (i = 0; i < len + 4; i++)
	{
		send_buf[len + 4] += send_buf[i];   //����У���
		send_buf[len + 5] += send_buf[len + 4];   //���㸽��У��
	}

	for (i = 0; i < len + 6; i++)
		UART_PutChar(USART1, send_buf[i]);
}

///*********************************************************************************************************
//*  �������ƣ�void CAR_Inductance(short L1, short L2, short L3,short L4, short L5, short L6,
//*				short L7, short L8, unsigned char Frame)
//*  ����˵�������Ͱ˸����ADCԭʼ����,���������ķ�ֶ�Ե��
//*  ����˵����L1~L8:ADCԭʼ���ݣ�Frame��֡ID:����ֱ��ʹ������ʽ֡0xF1~0xFA
//*  �������أ���
//*  �޸�ʱ�䣺2021��1��10��
//*  ��    ע��ʹ������ʽ֡0xF1
//**********************************************************************************************************/
void CAR_Inductance(short L1, short L2, short L3, short L4, short L5, short L6,
		short L7, short L8)
{
	unsigned char tbuf[16];

	tbuf[0] = L1 & 0XFF;
	tbuf[1] = (L1 >> 8) & 0XFF;

	tbuf[2] = L2 & 0XFF;
	tbuf[3] = (L2 >> 8) & 0XFF;

	tbuf[4] = L3 & 0XFF;
	tbuf[5] = (L3 >> 8) & 0XFF;

	tbuf[6] = L4 & 0XFF;
	tbuf[7] = (L4 >> 8) & 0XFF;

	tbuf[8] = L5 & 0XFF;
	tbuf[9] = (L5 >> 8) & 0XFF;

	tbuf[10] = L6 & 0XFF;
	tbuf[11] = (L6 >> 8) & 0XFF;

	tbuf[12] = L7 & 0XFF;
	tbuf[13] = (L7 >> 8) & 0XFF;

	tbuf[14] = L8 & 0XFF;
	tbuf[15] = (L8 >> 8) & 0XFF;

	ANO_Report(0xF1, tbuf, 16);
}

///*********************************************************************************************************
//*  �������ƣ�void CAR_MotorPID(float KP1, float KI1, float KD1,float KP2, float KI2, float KD2)
//*  ����˵�������͵��PID����
//*  ����˵��������PID����
//*  �������أ���
//*  �޸�ʱ�䣺2021��1��10��
//*  ��    ע��ʹ������ʽ֡0xF2
//**********************************************************************************************************/
void CAR_MotorPID(float KP1, float KI1, float KD1, float KP2, float KI2,
		float KD2)
{
	unsigned char tbuf[12];

	short kp1 = 100 * KP1;
	short ki1 = 100 * KI1;
	short kd1 = 100 * KD1;

	short kp2 = 100 * KP2;
	short ki2 = 100 * KI2;
	short kd2 = 100 * KD2;

	tbuf[0] = kp1 & 0XFF;
	tbuf[1] = (kp1 >> 8) & 0XFF;

	tbuf[2] = ki1 & 0XFF;
	tbuf[3] = (ki1 >> 8) & 0XFF;

	tbuf[4] = kd1 & 0XFF;
	tbuf[5] = (kd1 >> 8) & 0XFF;

	tbuf[6] = kp2 & 0XFF;
	tbuf[7] = (kp2 >> 8) & 0XFF;

	tbuf[8] = ki2 & 0XFF;
	tbuf[9] = (ki2 >> 8) & 0XFF;

	tbuf[10] = kd2 & 0XFF;
	tbuf[11] = (kd2 >> 8) & 0XFF;

	ANO_Report(0xF2, tbuf, 12);
}

///*********************************************************************************************************
//*  �������ƣ�CAR_ServoPID(float SKP, float SKI, float SKD)
//*  ����˵�������Ͷ��PID����
//*  ����˵�������PID����
//*  �������أ���
//*  �޸�ʱ�䣺2021��1��10��
//*  ��    ע��ʹ������ʽ֡0xF3
//**********************************************************************************************************/
void CAR_ServoPID(float SKP, float SKI, float SKD)
{
	unsigned char tbuf[6];

	short skp = 100 * SKP;
	short ski = 100 * SKI;
	short skd = 100 * SKD;

	tbuf[0] = skp & 0XFF;
	tbuf[1] = (skp >> 8) & 0XFF;

	tbuf[2] = ski & 0XFF;
	tbuf[3] = (ski >> 8) & 0XFF;

	tbuf[4] = skd & 0XFF;
	tbuf[5] = (skd >> 8) & 0XFF;

	ANO_Report(0xF3, tbuf, 6);
}

///*********************************************************************************************************
//*  �������ƣ�void CAR_PWM_ENCODER(short LPWM, short LENCODE, short RPWM, short RENCODE, short SPWM)
//*  ����˵�����������ҵ��PWM�����PWM�ͱ���������
//*  ����˵��������������������
//*  �������أ���
//*  �޸�ʱ�䣺2021��1��10��
//*  ��    ע��ʹ������ʽ֡0xF4
//**********************************************************************************************************/
void CAR_PWM_ENCODER(short LPWM, short LENCODE, short RPWM, short RENCODE,
		short SPWM)
{
	unsigned char tbuf[10];

	tbuf[0] = LPWM & 0XFF;
	tbuf[1] = (LPWM >> 8) & 0XFF;

	tbuf[2] = LENCODE & 0XFF;
	tbuf[3] = (LENCODE >> 8) & 0XFF;

	tbuf[4] = RPWM & 0XFF;
	tbuf[5] = (RPWM >> 8) & 0XFF;

	tbuf[6] = RENCODE & 0XFF;
	tbuf[7] = (RENCODE >> 8) & 0XFF;

	tbuf[8] = SPWM & 0XFF;
	tbuf[9] = (SPWM >> 8) & 0XFF;

	ANO_Report(0xF4, tbuf, 10);
}

///*********************************************************************************************************
//*  �������ƣ�void CAR_Raw_Data(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz)
//*  ����˵���������˶�������ԭʼ����
//*  ����˵����aacx-accy-accz:���ٶȼ�ԭʼ������gyrox-gyroy-gyroz�����ٶȼ�ԭʼ����
//*  �������أ���
//*  �޸�ʱ�䣺2021��1��10��
//*  ��    ע���̶�֡Э�飬�����޸�
//**********************************************************************************************************/
void CAR_Raw_Data(short aacx, short aacy, short aacz, short gyrox, short gyroy,
		short gyroz)
{
	unsigned char tbuf[13];

	tbuf[0] = aacx & 0XFF;
	tbuf[1] = (aacx >> 8) & 0XFF;

	tbuf[2] = aacy & 0XFF;
	tbuf[3] = (aacy >> 8) & 0XFF;
	tbuf[4] = aacz & 0XFF;
	tbuf[5] = (aacz >> 8) & 0XFF;

	tbuf[6] = gyrox & 0XFF;
	tbuf[7] = (gyrox >> 8) & 0XFF;
	tbuf[8] = gyroy & 0XFF;
	tbuf[9] = (gyroy >> 8) & 0XFF;
	tbuf[10] = gyroz & 0XFF;
	tbuf[11] = (gyroz >> 8) & 0XFF;

	tbuf[12] = 0xFF;

	ANO_Report(0x01, tbuf, 13);
}
///*********************************************************************************************************
//*  �������ƣ�void CAR_Angle_Data(float roll,float pitch,float yaw)
//*  ����˵��������ŷ����
//*  ����˵����ŷ���ǲ���
//*  �������أ���
//*  �޸�ʱ�䣺2021��1��10��
//*  ��    ע���̶�֡Э�飬�����޸�
//**********************************************************************************************************/
void CAR_Angle_Data(float roll_o, float pitch_o, float yaw_o)
{
	unsigned char tbuf[7];

	short roll = 100 * roll_o;
	short pitch = 100 * pitch_o;
	short yaw = 100 * yaw_o;

	tbuf[0] = roll & 0XFF;
	tbuf[1] = (roll >> 8) & 0XFF;

	tbuf[2] = pitch & 0XFF;
	tbuf[3] = (pitch >> 8) & 0XFF;

	tbuf[4] = yaw & 0XFF;
	tbuf[5] = (yaw >> 8) & 0XFF;

	tbuf[6] = 0x01;

	ANO_Report(0x03, tbuf, 7);
}
///*********************************************************************************************************
//*  �������ƣ�void CAR_Quaternion_Data(short V0, short V1, short V2, short V3)
//*  ����˵����������Ԫ��
//*  ����˵������Ԫ������
//*  �������أ���
//*  �޸�ʱ�䣺2021��1��10��
//*  ��    ע���̶�֡Э�飬�����޸�
//**********************************************************************************************************/
void CAR_Quaternion_Data(short V0, short V1, short V2, short V3)
{
	unsigned char tbuf[9];

	V0 *= 10000 * V0;
	V1 *= 10000 * V1;
	V2 *= 10000 * V2;
	V3 *= 10000 * V3;

	tbuf[0] = V0 & 0XFF;
	tbuf[1] = (V0 >> 8) & 0XFF;

	tbuf[2] = V1 & 0XFF;
	tbuf[3] = (V1 >> 8) & 0XFF;

	tbuf[4] = V2 & 0XFF;
	tbuf[5] = (V2 >> 8) & 0XFF;

	tbuf[6] = V3 & 0XFF;
	tbuf[7] = (V3 >> 8) & 0XFF;

	tbuf[8] = 0x01;

	ANO_Report(0x04, tbuf, 9);
}
