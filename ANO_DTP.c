/*
 * ANO_DTP.c
 *
 *  Created on: 2022年1月10日
 *      Author: HANYING
 */

#include "ANO_DTP.h"
#include "../LIBRARY/HY_UART.h"//请修改成你自己的串口函数头文件

//注意修改void ANO_Report(u8 fun, u8*data, u8 len)函数里的UART_PutChar(USART1, send_buf[i]);
//注意修改void ANO_Report(u8 fun, u8*data, u8 len)函数里的UART_PutChar(USART1, send_buf[i]);
//注意修改void ANO_Report(u8 fun, u8*data, u8 len)函数里的UART_PutChar(USART1, send_buf[i]);
//重要的事情说三遍

/*************************************************华丽的分界线************************************************/

///*********************************************************************************************************
//*  函数名称：void ANO_Report(u8 fun, u8*data, u8 len)
//*  功能说明：内部调用/单独使用
//*  参数说明：帧ID,数组，长度
//*  函数返回：无
//*  修改时间：2021年1月10日
//*  备    注：无
//**********************************************************************************************************/
void ANO_Report(u8 fun, u8*data, u8 len)
{
	unsigned char send_buf[35];
	unsigned char i;
	if (len > 30)
		return; //最多30字节数据

	send_buf[len + 4] = 0;  //校验数置零
	send_buf[len + 5] = 0;  //校验数置零
	send_buf[0] = 0XAA;   //帧头
	send_buf[1] = 0XFF;   //目标地址
	send_buf[2] = fun;    //功能码
	send_buf[3] = len;    //数据长度

	for (i = 0; i < len; i++)
		send_buf[4 + i] = data[i];          //复制数据

	for (i = 0; i < len + 4; i++)
	{
		send_buf[len + 4] += send_buf[i];   //计算校验和
		send_buf[len + 5] += send_buf[len + 4];   //计算附加校验
	}

	for (i = 0; i < len + 6; i++)
		UART_PutChar(USART1, send_buf[i]);
}

///*********************************************************************************************************
//*  函数名称：void CAR_Inductance(short L1, short L2, short L3,short L4, short L5, short L6,
//*				short L7, short L8, unsigned char Frame)
//*  功能说明：发送八个电感ADC原始数据,兼容麦克纳姆轮多对电感
//*  参数说明：L1~L8:ADC原始数据；Frame：帧ID:可以直接使用灵活格式帧0xF1~0xFA
//*  函数返回：无
//*  修改时间：2021年1月10日
//*  备    注：使用灵活格式帧0xF1
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
//*  函数名称：void CAR_MotorPID(float KP1, float KI1, float KD1,float KP2, float KI2, float KD2)
//*  功能说明：发送电机PID参数
//*  参数说明：两组PID参数
//*  函数返回：无
//*  修改时间：2021年1月10日
//*  备    注：使用灵活格式帧0xF2
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
//*  函数名称：CAR_ServoPID(float SKP, float SKI, float SKD)
//*  功能说明：发送舵机PID参数
//*  参数说明：舵机PID参数
//*  函数返回：无
//*  修改时间：2021年1月10日
//*  备    注：使用灵活格式帧0xF3
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
//*  函数名称：void CAR_PWM_ENCODER(short LPWM, short LENCODE, short RPWM, short RENCODE, short SPWM)
//*  功能说明：发送左右电机PWM，舵机PWM和编码器数据
//*  参数说明：电机舵机编码器参数
//*  函数返回：无
//*  修改时间：2021年1月10日
//*  备    注：使用灵活格式帧0xF4
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
//*  函数名称：void CAR_Raw_Data(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz)
//*  功能说明：发送运动传感器原始数据
//*  参数说明：aacx-accy-accz:加速度计原始参数；gyrox-gyroy-gyroz：角速度计原始数据
//*  函数返回：无
//*  修改时间：2021年1月10日
//*  备    注：固定帧协议，不可修改
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
//*  函数名称：void CAR_Angle_Data(float roll,float pitch,float yaw)
//*  功能说明：发送欧拉角
//*  参数说明：欧拉角参数
//*  函数返回：无
//*  修改时间：2021年1月10日
//*  备    注：固定帧协议，不可修改
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
//*  函数名称：void CAR_Quaternion_Data(short V0, short V1, short V2, short V3)
//*  功能说明：发送四元数
//*  参数说明：四元数参数
//*  函数返回：无
//*  修改时间：2021年1月10日
//*  备    注：固定帧协议，不可修改
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
