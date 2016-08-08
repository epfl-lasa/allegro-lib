/*
 * control_AllegroHand.cpp
 *
 *  Created on: Nov 15, 2012
 *      Author:
 */

#include "controlAllegroHand.h"
#include <iostream>
#include <math.h>
#include <stdio.h>
using namespace std;

void PRINT_INFO(const char *msg)
{
	cout << msg << endl;
}

controlAllegroHand::controlAllegroHand()
{
	mEmergencyStop = false;

	mPWM_MAX[eJOINTNAME_INDEX_0] = PWM_LIMIT_ROLL;
	mPWM_MAX[eJOINTNAME_INDEX_1] = PWM_LIMIT_NEAR;
	mPWM_MAX[eJOINTNAME_INDEX_2] = PWM_LIMIT_MIDDLE;
	mPWM_MAX[eJOINTNAME_INDEX_3] = PWM_LIMIT_FAR;

	mPWM_MAX[eJOINTNAME_MIDDLE_0] = PWM_LIMIT_ROLL;
	mPWM_MAX[eJOINTNAME_MIDDLE_1] = PWM_LIMIT_NEAR;
	mPWM_MAX[eJOINTNAME_MIDDLE_2] = PWM_LIMIT_MIDDLE;
	mPWM_MAX[eJOINTNAME_MIDDLE_3] = PWM_LIMIT_FAR;

	mPWM_MAX[eJOINTNAME_PINKY_0] = PWM_LIMIT_ROLL;
	mPWM_MAX[eJOINTNAME_PINKY_1] = PWM_LIMIT_NEAR;
	mPWM_MAX[eJOINTNAME_PINKY_2] = PWM_LIMIT_MIDDLE;
	mPWM_MAX[eJOINTNAME_PINKY_3] = PWM_LIMIT_FAR;

	mPWM_MAX[eJOINTNAME_THUMB_0] = PWM_LIMIT_THUMB_ROLL;
	mPWM_MAX[eJOINTNAME_THUMB_1] = PWM_LIMIT_THUMB_NEAR;
	mPWM_MAX[eJOINTNAME_THUMB_2] = PWM_LIMIT_THUMB_MIDDLE;
	mPWM_MAX[eJOINTNAME_THUMB_3] = PWM_LIMIT_THUMB_FAR;

	mEncoderOffset[eJOINTNAME_INDEX_0]  =    351;
	mEncoderOffset[eJOINTNAME_INDEX_1]  =  1533-65536;
	mEncoderOffset[eJOINTNAME_INDEX_2]  =    2057;
	mEncoderOffset[eJOINTNAME_INDEX_3]  =     516;
	mEncoderOffset[eJOINTNAME_MIDDLE_0] =   100;
	mEncoderOffset[eJOINTNAME_MIDDLE_1] =  757-65536;
	mEncoderOffset[eJOINTNAME_MIDDLE_2] =   -2079;
	mEncoderOffset[eJOINTNAME_MIDDLE_3] =    -918;
	mEncoderOffset[eJOINTNAME_PINKY_0]  =  1774;
	mEncoderOffset[eJOINTNAME_PINKY_1]  = -398-65536;
	mEncoderOffset[eJOINTNAME_PINKY_2]  =   -284;
	mEncoderOffset[eJOINTNAME_PINKY_3]  =   -460;
	mEncoderOffset[eJOINTNAME_THUMB_0]  = 0;
	mEncoderOffset[eJOINTNAME_THUMB_1]  = 1288;
	mEncoderOffset[eJOINTNAME_THUMB_2]  =   96-65536;
	mEncoderOffset[eJOINTNAME_THUMB_3]  = -1519-65536;

	mEncoderDirection[eJOINTNAME_INDEX_0]  =  1;
	mEncoderDirection[eJOINTNAME_INDEX_1]  = -1;
	mEncoderDirection[eJOINTNAME_INDEX_2]  =  1;
	mEncoderDirection[eJOINTNAME_INDEX_3]  =  1;
	mEncoderDirection[eJOINTNAME_MIDDLE_0] =  1;
	mEncoderDirection[eJOINTNAME_MIDDLE_1] = -1;
	mEncoderDirection[eJOINTNAME_MIDDLE_2] =  1;
	mEncoderDirection[eJOINTNAME_MIDDLE_3] =  1;
	mEncoderDirection[eJOINTNAME_PINKY_0]  =  1;
	mEncoderDirection[eJOINTNAME_PINKY_1]  = -1;
	mEncoderDirection[eJOINTNAME_PINKY_2]  =  1;
	mEncoderDirection[eJOINTNAME_PINKY_3]  =  1;
	mEncoderDirection[eJOINTNAME_THUMB_0]  =  1;
	mEncoderDirection[eJOINTNAME_THUMB_1]  =  1;
	mEncoderDirection[eJOINTNAME_THUMB_2]  = -1;
	mEncoderDirection[eJOINTNAME_THUMB_3]  = -1;

	mMotorDirection[eJOINTNAME_INDEX_0]  =  1;
	mMotorDirection[eJOINTNAME_INDEX_1]  =  1;
	mMotorDirection[eJOINTNAME_INDEX_2]  =  1;
	mMotorDirection[eJOINTNAME_INDEX_3]  =  1;
	mMotorDirection[eJOINTNAME_MIDDLE_0] =  1;
	mMotorDirection[eJOINTNAME_MIDDLE_1] = -1;
	mMotorDirection[eJOINTNAME_MIDDLE_2] = -1;
	mMotorDirection[eJOINTNAME_MIDDLE_3] =  1;
	mMotorDirection[eJOINTNAME_PINKY_0]  = -1;
	mMotorDirection[eJOINTNAME_PINKY_1]  =  1;
	mMotorDirection[eJOINTNAME_PINKY_2]  =  1;
	mMotorDirection[eJOINTNAME_PINKY_3]  =  1;
	mMotorDirection[eJOINTNAME_THUMB_0]  =  1;
	mMotorDirection[eJOINTNAME_THUMB_1]  =  1;
	mMotorDirection[eJOINTNAME_THUMB_2]  =  1;
	mMotorDirection[eJOINTNAME_THUMB_3]  =  1;
}


controlAllegroHand::~controlAllegroHand()
{
	PRINT_INFO("Setting System OFF");
	_writeDeviceMsg(ID_CMD_SET_SYSTEM_OFF, ID_DEVICE_MAIN, ID_COMMON);
	usleep(10000);

	if(CAN_Close(CanHandle))
	{
		PRINT_INFO("Error in CAN_Close()");
	}
}

void controlAllegroHand::init(int mode)
{

	unsigned char data[8];
	int ret;
	TPCANRdMsg lmsg;

	PRINT_INFO("Opening CAN device");

	CanHandle = LINUX_CAN_Open("/dev/pcan32", O_RDWR);
	if (!CanHandle)
	{
		PRINT_INFO("Error in CAN_Open()");
	}

	char txt[VERSIONSTRING_LEN];
	ret = CAN_VersionInfo(CanHandle, txt);
	if (!ret)
	{
		PRINT_INFO(txt);
	}
	else {
		PRINT_INFO("Error getting CAN_VersionInfo()");
	}

	PRINT_INFO("Initializing CAN device");
	// init to an user defined bit rate
	ret = CAN_Init(CanHandle, CAN_BAUD_1M, CAN_INIT_TYPE_ST);
	if (ret)
	{
		PRINT_INFO("Error in CAN_Init()");
	}

	PRINT_INFO("Clear the can buffer");
	for(int i=0; i<100; i++){
		LINUX_CAN_Read_Timeout(CanHandle, &lmsg, 1000); // polding
	}

	PRINT_INFO("System off");
	_writeDeviceMsg(ID_CMD_SET_SYSTEM_OFF, ID_DEVICE_MAIN, ID_COMMON);
	usleep(100);

	PRINT_INFO("Setting loop period = 3 ms");
	//data[0] = (char)(ALLEGRO_CONTROL_TIME_INTERVAL * 1000.);
	data[0] = 3;
	_writeDeviceMsg(ID_CMD_SET_PERIOD, ID_DEVICE_MAIN, ID_COMMON, 1, data );
	usleep(100);

	PRINT_INFO("Setting task mode");
	_writeDeviceMsg(ID_CMD_SET_MODE_TASK, ID_DEVICE_MAIN, ID_COMMON);
	usleep(100);

	PRINT_INFO("Setting System ON");
	_writeDeviceMsg(ID_CMD_SET_SYSTEM_ON, ID_DEVICE_MAIN, ID_COMMON);
	usleep(100);

	for(int i=0; i<100; i++) ret=LINUX_CAN_Read_Timeout(CanHandle, &lmsg, 0);

	PRINT_INFO("Setting joint query command");
	_writeDeviceMsg(ID_CMD_QUERY_STATE_DATA, ID_DEVICE_MAIN, ID_COMMON);
	usleep(100);

	// wait for the first command
	PRINT_INFO("Wait for first joint command");
	int cnt = 0;
	int itr = 0;
	double q[4];
	char lID;
	while(true)
	{
		ret=LINUX_CAN_Read_Timeout(CanHandle, &lmsg, 1000000);
		lID  = _parseCANMsg( lmsg.Msg, q);
		if( (lID >= ID_DEVICE_SUB_01) && (lID <= ID_DEVICE_SUB_04) )
		{
			cnt++;
			if(cnt == 8) break;
		}
		else{
			itr++;
		}

		if(itr > 4){
			mEmergencyStop = true;
			break;
		}
	}

	cout << "started" << endl;
}

int controlAllegroHand::update(void)
{
	_readDevices();
	usleep(10);
	_writeDevices();

	if(mEmergencyStop == true)
	{
		return -1;
	}
	else
	{
		return 0;
	}
}

int  controlAllegroHand::command(const short& cmd, const int& arg)
{
	return 0;
}

void controlAllegroHand::setTorque(double *torque)
{
	for(int i=0; i<DOF_JOINTS; i++)
	{
		desired_torque[i] = torque[i];
	}
}

void controlAllegroHand::getJointInfo(double *position, double *torque)
{
	for(int i=0; i<DOF_JOINTS; i++)
	{
		position[i] = curr_position[i];
		torque[i] = curr_torque[i];
	}
}

void controlAllegroHand::_readDevices()
{
	double q[4];
	char lID;
	int ret = 0;
	int itr = 0;
	static int errorcnt = 0;
	TPCANRdMsg lmsg;

	while( itr<4 )
	//while( true)
	{
		ret=LINUX_CAN_Read_Timeout(CanHandle, &lmsg, 3000); // timeout in micro second
		//ret=LINUX_CAN_Read_Timeout(CanHandle, &lmsg, 0); // 0 : polling

		if (ret)
		{
			break;
		}
		else
		{
			lID  = _parseCANMsg( lmsg.Msg, q);
			if( (lID >= ID_DEVICE_SUB_01) && (lID <= ID_DEVICE_SUB_04) )
			{
				for(int i=0; i<4; i++)
				{
					curr_position[i+4*(lID-ID_DEVICE_SUB_01)] = q[i];
				}
				itr++;
				printf("%d, ", lID );
			}
			else if( lID == 0)
			{
				errorcnt = 0;
				//printf("(%d), ", lID );
			}
			else if( lID < 0 )
			{
				mEmergencyStop = true;
			}
		}
	}


	if( itr < 4)
	{
		printf(": %d  \n", itr );
		errorcnt++;
		if( errorcnt > 3 ){
			mEmergencyStop = true;
		}
	}
	else{
		printf(": %d  \n", itr );
		errorcnt = 0;
	}


}

void controlAllegroHand::_writeDevices()
{
	double pwmDouble[DOF_JOINTS];
	short pwm[DOF_JOINTS];
	unsigned char data[8];

	// convert to torque to pwm
	for(int i=0; i<DOF_JOINTS; i++ ){
		pwmDouble[i] =  desired_torque[i] *1.0 * (double)mMotorDirection[i] *800.0;

		mPWM_MAX[i] = 800.0;

		// limitation should be less than 800
		if     ( pwmDouble[i] >  mPWM_MAX[i] )
		{
			pwmDouble[i] =  mPWM_MAX[i];
			//cout <<i << " max"<< endl;
		}
		else if( pwmDouble[i] < -mPWM_MAX[i] ) {
			pwmDouble[i] = -mPWM_MAX[i];
			//cout <<i<< " min"<< endl;
		}

		pwm[i] = (short)pwmDouble[i];
	}
	//pwm[eJOINTNAME_PINKY_0] = 0;
	//pwm[eJOINTNAME_PINKY_1] = 0;
	//pwm[eJOINTNAME_PINKY_2] = 0;
	//pwm[eJOINTNAME_PINKY_3] = 0;

	for(int findex=0; findex<4; findex++ ){
//		data[0] = (unsigned char)( (pwm[0+findex*4] >> 8) & 0x00ff);
//		data[1] = (unsigned char)(  pwm[0+findex*4]       & 0x00ff);
//		data[2] = (unsigned char)( (pwm[1+findex*4] >> 8) & 0x00ff);
//		data[3] = (unsigned char)(  pwm[1+findex*4]       & 0x00ff);
//		data[4] = (unsigned char)( (pwm[2+findex*4] >> 8) & 0x00ff);
//		data[5] = (unsigned char)(  pwm[2+findex*4]       & 0x00ff);
//		data[6] = (unsigned char)( (pwm[3+findex*4] >> 8) & 0x00ff);
//		data[7] = (unsigned char)(  pwm[3+findex*4]       & 0x00ff);

		data[6] = (unsigned char)( (pwm[0+findex*4] >> 8) & 0x00ff);
		data[7] = (unsigned char)(  pwm[0+findex*4]       & 0x00ff);
		data[4] = (unsigned char)( (pwm[1+findex*4] >> 8) & 0x00ff);
		data[5] = (unsigned char)(  pwm[1+findex*4]       & 0x00ff);
		data[2] = (unsigned char)( (pwm[2+findex*4] >> 8) & 0x00ff);
		data[3] = (unsigned char)(  pwm[2+findex*4]       & 0x00ff);
		data[0] = (unsigned char)( (pwm[3+findex*4] >> 8) & 0x00ff);
		data[1] = (unsigned char)(  pwm[3+findex*4]       & 0x00ff);

		_writeDeviceMsg( (DWORD)(ID_CMD_SET_TORQUE_1 + findex), ID_DEVICE_MAIN, ID_COMMON, 8, data);
		usleep(10);
	}

	// send message to call joint position and torque query
	//_writeDeviceMsg(ID_CMD_QUERY_STATE_DATA, ID_DEVICE_MAIN, ID_COMMON);
	//usleep(10);
}

void controlAllegroHand::_writeDeviceMsg(DWORD command, DWORD from, DWORD to, BYTE len, unsigned char *data)
{
	TPCANMsg msg1;
	DWORD Txid;

	Txid = (command<<6) | (to <<3) | (from);
	msg1.ID  = Txid;
	msg1.MSGTYPE  = MSGTYPE_STANDARD;
	msg1.LEN  = len;
	for(BYTE i=0; i<8; i++) msg1.DATA[i] = 0;

	for(BYTE i=0; i<len; i++)
	{
		msg1.DATA[i] = data[i];
	}

	//if(LINUX_CAN_Write_Timeout(CanHandle, &msg1, 0.0))
	if(CAN_Write(CanHandle, &msg1))
	{
		cout << "CAN communication error (write)" << endl;
		mEmergencyStop = true;
	}

}

void controlAllegroHand::_writeDeviceMsg(DWORD command, DWORD from,DWORD to)
{
	_writeDeviceMsg(command, from, to, 0, NULL);
}



char controlAllegroHand::_parseCANMsg(TPCANMsg read_msg,  double *values)
{
	unsigned char cmd, src, to;
	unsigned char len;
	unsigned char tmpdata[8];
	int tmppos[4];
	int lIndexBase;

	cmd = (unsigned char)( (read_msg.ID >> 6) & 0x1f );
	to  = (unsigned char)( (read_msg.ID >> 3) & 0x07 );
	src = (unsigned char)( read_msg.ID & 0x07 );
	len = (unsigned char)( read_msg.LEN );
	for(unsigned int nd=0; nd<len; nd++)
		tmpdata[nd] = read_msg.DATA[nd];

	switch (cmd)
	{
	case ID_CMD_QUERY_CONTROL_DATA:
		if (src >= ID_DEVICE_SUB_01 && src <= ID_DEVICE_SUB_04)
		{

			tmppos[0] = (int)(tmpdata[0] | (tmpdata[1] << 8));
			tmppos[1] = (int)(tmpdata[2] | (tmpdata[3] << 8));
			tmppos[2] = (int)(tmpdata[4] | (tmpdata[5] << 8));
			tmppos[3] = (int)(tmpdata[6] | (tmpdata[7] << 8));

			lIndexBase = 4*(src-ID_DEVICE_SUB_01);

			//values[0] = (double)mEncoderDirection[lIndexBase+0] * (double)(tmppos[0] - 32768 - mEncoderOffset[lIndexBase+0]) * ( 333.3 / 65536.0 ) * ( M_PI/180.0);
			//values[1] = (double)mEncoderDirection[lIndexBase+1] * (double)(tmppos[1] - 32768 - mEncoderOffset[lIndexBase+1]) * ( 333.3 / 65536.0 ) * ( M_PI/180.0);
			//values[2] = (double)mEncoderDirection[lIndexBase+2] * (double)(tmppos[2] - 32768 - mEncoderOffset[lIndexBase+2]) * ( 333.3 / 65536.0 ) * ( M_PI/180.0);
			//values[3] = (double)mEncoderDirection[lIndexBase+3] * (double)(tmppos[3] - 32768 - mEncoderOffset[lIndexBase+3]) * ( 333.3 / 65536.0 ) * ( M_PI/180.0);
			values[0] = ((double)mEncoderDirection[lIndexBase+0] *(double)tmppos[0] - 32768.0 - (double)mEncoderOffset[lIndexBase+0]) * ( 333.3 / 65536.0 ) * ( M_PI/180.0);
			values[1] = ((double)mEncoderDirection[lIndexBase+1] *(double)tmppos[1] - 32768.0 - (double)mEncoderOffset[lIndexBase+1]) * ( 333.3 / 65536.0 ) * ( M_PI/180.0);
			values[2] = ((double)mEncoderDirection[lIndexBase+2] *(double)tmppos[2] - 32768.0 - (double)mEncoderOffset[lIndexBase+2]) * ( 333.3 / 65536.0 ) * ( M_PI/180.0);
			values[3] = ((double)mEncoderDirection[lIndexBase+3] *(double)tmppos[3] - 32768.0 - (double)mEncoderOffset[lIndexBase+3]) * ( 333.3 / 65536.0 ) * ( M_PI/180.0);

//			printf("IndexBase: %d, %d %d %d %d \n", lIndexBase,
//					tmppos[0]- 32768,
//					tmppos[1]- 32768,
//					tmppos[2]- 32768,
//					tmppos[3]- 32768 );
			return src;

		}
		else
		{
			cout << "No subdevice match!" << endl;
			return -1;
		}

		break;
		//TODO: Implement this
	case ID_CMD_QUERY_STATE_DATA:
		return 0;
		break;
	default:
		printf("unknown command %d, src %d, to %d, len %d \n", cmd, src, to, len);
		/*
		for(int nd=0; nd<len; nd++)
		{
			printf("%d \n ", tmpdata[nd]);
		}
		*/
		return -1;
		break;
	}

}
