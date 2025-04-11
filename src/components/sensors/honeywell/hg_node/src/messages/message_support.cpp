#include <message_support.h>


void setVariables(uint32_t sizeOfBuffer, uint8_t * transmitBuffer, int sh, ros::NodeHandle * n)
{
	bufSize = sizeOfBuffer;
	TxBuffer = transmitBuffer;
	serialHandle = sh;
	internalNodeHandle = n;
	return;
}


uint32_t getBufSize(void) {return bufSize;}
uint8_t * getTxBuffer(void) {return TxBuffer;}
int getSerialHandle(void) {return serialHandle;}
ros::NodeHandle * getRosHandle(void) {return internalNodeHandle;}

bool processMessage(uint8_t * buffer, uint32_t AddressID, uint32_t messageID)
{
	if (AddressID == 0x0E)
	{
		switch(messageID)
		{
			case 0x01: Msg_01_pub_callback(buffer);break;
			case 0x02: Msg_02_pub_callback(buffer);break;
			case 0x04: Msg_04_pub_callback(buffer);break;
			case 0x05: Msg_05_pub_callback(buffer);break;
			case 0x0C: Msg_0C_pub_callback(buffer);break;
			case 0x0D: Msg_0D_pub_callback(buffer);break;
			case 0xA1: Msg_A1_pub_callback(buffer);break;
			case 0xA2: Msg_A2_pub_callback(buffer);break;
			case 0xA3: Msg_A3_pub_callback(buffer);break;
			case 0xA9: Msg_A9_pub_callback(buffer);break;
			case 0xAC: Msg_AC_pub_callback(buffer);break;
			case 0xAD: Msg_AD_pub_callback(buffer);break;
			case 0xAE: Msg_AE_pub_callback(buffer);break;
			case 0xB0: Msg_B0_pub_callback(buffer);break;
			case 0xB1: Msg_B1_pub_callback(buffer);break;
			case 0xB2: Msg_B2_pub_callback(buffer);break;
			case 0xB3: Msg_B3_pub_callback(buffer);break;
			case 0xB4: Msg_B4_pub_callback(buffer);break;
			case 0xCA: Msg_CA_pub_callback(buffer);break;
			case 0xF0: Msg_F0_pub_callback(buffer);break;
			case 0xF1: Msg_F1_pub_callback(buffer);break;
			case 0xF2: Msg_F2_pub_callback(buffer);break;
			case 0xF3: Msg_F3_pub_callback(buffer);break;
			case 0xF4: Msg_F4_pub_callback(buffer);break;
			case 0xF5: Msg_F5_pub_callback(buffer);break;
			case 0xF6: Msg_F6_pub_callback(buffer);break;
			case 0xF7: Msg_F7_pub_callback(buffer);break;
			case 0xF8: Msg_F8_pub_callback(buffer);break;
			case 0xF9: Msg_F9_pub_callback(buffer);break;
			case 0xFA: Msg_FA_pub_callback(buffer);break;
			case 0xFB: Msg_FB_pub_callback(buffer);break;
			default:return false;
		}
	}
	return true;
}

void init_all_pubs(ros::NodeHandle * n)
{
	ROS_INFO("Starting All HGuide Publishers:");
	init_01(n);
	init_02(n);
	init_04(n);
	init_05(n);
	init_0C(n);
	init_0D(n);
	init_A1(n);
	init_A2(n);
	init_A3(n);
	init_A9(n);
	init_AC(n);
	init_AD(n);
	init_AE(n);
	init_B0(n);
	init_B1(n);
	init_B2(n);
	init_B3(n);
	init_B4(n);
	init_CA(n);
	init_F0(n);
	init_F1(n);
	init_F2(n);
	init_F3(n);
	init_F4(n);
	init_F5(n);
	init_F6(n);
	init_F7(n);
	init_F8(n);
	init_F9(n);
	init_FA(n);
	init_FB(n);
}


void init_all_subs(ros::NodeHandle * n)
{
	ROS_INFO("Starting All HGuide Subscribers:");
}


void stop_all_pubs(void)
{
	ROS_INFO("Stopping All HGuide Publishers");
	stop_01();
	stop_02();
	stop_04();
	stop_05();
	stop_0C();
	stop_0D();
	stop_A1();
	stop_A2();
	stop_A3();
	stop_A9();
	stop_AC();
	stop_AD();
	stop_AE();
	stop_B0();
	stop_B1();
	stop_B2();
	stop_B3();
	stop_B4();
	stop_CA();
	stop_F0();
	stop_F1();
	stop_F2();
	stop_F3();
	stop_F4();
	stop_F5();
	stop_F6();
	stop_F7();
	stop_F8();
	stop_F9();
	stop_FA();
	stop_FB();
}


void stop_all_subs(void)
{
	ROS_INFO("Stopping All HGuide Subscribers:");
}
