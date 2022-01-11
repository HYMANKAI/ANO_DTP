/*
 * ANO_DTP.h
 *
 *  Created on: 2022Äê1ÔÂ10ÈÕ
 *      Author: HANYING
 */

#ifndef LIBRARY_ANO_ANO_DTP_H_
#define LIBRARY_ANO_ANO_DTP_H_

void CAR_Inductance(short L1, short L2, short L3, short L4, short L5, short L6, short L7, short L8);
void CAR_MotorPID(float KP1, float KI1, float KD1, float KP2, float KI2,float KD2);
void CAR_ServoPID(float SKP, float SKI, float SKD);
void CAR_PWM_ENCODER(short LPWM, short LENCODE, short RPWM, short RENCODE,short SPWM);
void CAR_Raw_Data(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz);
void CAR_Angle_Data(float roll,float pitch,float yaw);
void CAR_Quaternion_Data(short V0, short V1, short V2, short V3);

#endif /* LIBRARY_ANO_ANO_DTP_H_ */
