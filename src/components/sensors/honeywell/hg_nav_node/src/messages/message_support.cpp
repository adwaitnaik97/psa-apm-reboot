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
	if (AddressID == 0xA5C381FF)
	{
		switch(messageID)
		{
			case 0x2001: Msg_2001_pub_callback(buffer);break;
			case 0x2002: Msg_2002_pub_callback(buffer);break;
			case 0x2005: Msg_2005_pub_callback(buffer);break;
			case 0x2011: Msg_2011_pub_callback(buffer);break;
			case 0x2021: Msg_2021_pub_callback(buffer);break;
			case 0x20FF: Msg_20FF_pub_callback(buffer);break;
			case 0x2201: Msg_2201_pub_callback(buffer);break;
			case 0x2301: Msg_2301_pub_callback(buffer);break;
			case 0x2311: Msg_2311_pub_callback(buffer);break;
			case 0x2401: Msg_2401_pub_callback(buffer);break;
			case 0x2402: Msg_2402_pub_callback(buffer);break;
			case 0x2411: Msg_2411_pub_callback(buffer);break;
			case 0x2421: Msg_2421_pub_callback(buffer);break;
			case 0x2422: Msg_2422_pub_callback(buffer);break;
			case 0x2423: Msg_2423_pub_callback(buffer);break;
			case 0x2424: Msg_2424_pub_callback(buffer);break;
			case 0x2425: Msg_2425_pub_callback(buffer);break;
			case 0x2426: Msg_2426_pub_callback(buffer);break;
			case 0x2427: Msg_2427_pub_callback(buffer);break;
			case 0x2428: Msg_2428_pub_callback(buffer);break;
			case 0x2429: Msg_2429_pub_callback(buffer);break;
			case 0x2501: Msg_2501_pub_callback(buffer);break;
			case 0x2611: Msg_2611_pub_callback(buffer);break;
			case 0x5001: Msg_5001_pub_callback(buffer);break;
			case 0x5012: Msg_5012_pub_callback(buffer);break;
			case 0x5101: Msg_5101_pub_callback(buffer);break;
			case 0x5102: Msg_5102_pub_callback(buffer);break;
			case 0x5103: Msg_5103_pub_callback(buffer);break;
			case 0x5104: Msg_5104_pub_callback(buffer);break;
			case 0x5105: Msg_5105_pub_callback(buffer);break;
			case 0x5106: Msg_5106_pub_callback(buffer);break;
			case 0x5108: Msg_5108_pub_callback(buffer);break;
			case 0x5109: Msg_5109_pub_callback(buffer);break;
			case 0x5201: Msg_5201_pub_callback(buffer);break;
			case 0x6001: Msg_6001_pub_callback(buffer);break;
			case 0x6003: Msg_6003_pub_callback(buffer);break;
			case 0x6108: Msg_6108_pub_callback(buffer);break;
			case 0x6109: Msg_6109_pub_callback(buffer);break;
			case 0x6110: Msg_6110_pub_callback(buffer);break;
			case 0x6111: Msg_6111_pub_callback(buffer);break;
			case 0x6112: Msg_6112_pub_callback(buffer);break;
			case 0x6201: Msg_6201_pub_callback(buffer);break;
			case 0x6202: Msg_6202_pub_callback(buffer);break;
			case 0x6203: Msg_6203_pub_callback(buffer);break;
			case 0x6204: Msg_6204_pub_callback(buffer);break;
			case 0x6205: Msg_6205_pub_callback(buffer);break;
			case 0x6211: Msg_6211_pub_callback(buffer);break;
			case 0x6311: Msg_6311_pub_callback(buffer);break;
			case 0x6403: Msg_6403_pub_callback(buffer);break;
			case 0x6405: Msg_6405_pub_callback(buffer);break;
			case 0x6406: Msg_6406_pub_callback(buffer);break;
			case 0x6424: Msg_6424_pub_callback(buffer);break;
			case 0x6428: Msg_6428_pub_callback(buffer);break;
			case 0x6438: Msg_6438_pub_callback(buffer);break;
			case 0x6504: Msg_6504_pub_callback(buffer);break;
			case 0x6505: Msg_6505_pub_callback(buffer);break;
			case 0x6508: Msg_6508_pub_callback(buffer);break;
			case 0x6601: Msg_6601_pub_callback(buffer);break;
			case 0x6651: Msg_6651_pub_callback(buffer);break;
			case 0x6721: Msg_6721_pub_callback(buffer);break;
			case 0x6722: Msg_6722_pub_callback(buffer);break;
			case 0x6723: Msg_6723_pub_callback(buffer);break;
			case 0x6724: Msg_6724_pub_callback(buffer);break;
			case 0x6725: Msg_6725_pub_callback(buffer);break;
			case 0x6726: Msg_6726_pub_callback(buffer);break;
			case 0x6738: Msg_6738_pub_callback(buffer);break;
			case 0x9900: Msg_9900_pub_callback(buffer);break;
			case 0x9910: Msg_9910_pub_callback(buffer);break;
			case 0xCE01: Msg_CE01_pub_callback(buffer);break;
			case 0xFE01: Msg_FE01_pub_callback(buffer);break;
			case 0xFE02: Msg_FE02_pub_callback(buffer);break;
			default:return false;
		}
	}
	return true;
}

void init_all_pubs(ros::NodeHandle * n)
{
	ROS_INFO("Starting All HGuide Publishers:");
	init_2001(n);
	init_2002(n);
	init_2005(n);
	init_2011(n);
	init_2021(n);
	init_20FF(n);
	init_2201(n);
	init_2301(n);
	init_2311(n);
	init_2401(n);
	init_2402(n);
	init_2411(n);
	init_2421(n);
	init_2422(n);
	init_2423(n);
	init_2424(n);
	init_2425(n);
	init_2426(n);
	init_2427(n);
	init_2428(n);
	init_2429(n);
	init_2501(n);
	init_2611(n);
	init_5001(n);
	init_5012(n);
	init_5101(n);
	init_5102(n);
	init_5103(n);
	init_5104(n);
	init_5105(n);
	init_5106(n);
	init_5108(n);
	init_5109(n);
	init_5201(n);
	init_6001(n);
	init_6003(n);
	init_6108(n);
	init_6109(n);
	init_6110(n);
	init_6111(n);
	init_6112(n);
	init_6201(n);
	init_6202(n);
	init_6203(n);
	init_6204(n);
	init_6205(n);
	init_6211(n);
	init_6311(n);
	init_6403(n);
	init_6405(n);
	init_6406(n);
	init_6424(n);
	init_6428(n);
	init_6438(n);
	init_6504(n);
	init_6505(n);
	init_6508(n);
	init_6601(n);
	init_6651(n);
	init_6721(n);
	init_6722(n);
	init_6723(n);
	init_6724(n);
	init_6725(n);
	init_6726(n);
	init_6738(n);
	init_9900(n);
	init_9910(n);
	init_CE01(n);
	init_FE01(n);
	init_FE02(n);
}


void init_all_subs(ros::NodeHandle * n)
{
	ROS_INFO("Starting All HGuide Subscribers:");
	init_1001(n);
	init_1002(n);
	init_1003(n);
	init_1004(n);
	init_1005(n);
	init_1101(n);
	init_1105(n);
	init_1108(n);
	init_1111(n);
	init_1201(n);
	init_1401(n);
	init_4012(n);
	init_4109(n);
	init_4110(n);
	init_4201(n);
	init_4202(n);
	init_4204(n);
	init_4401(n);
	init_4404(n);
	init_4438(n);
	init_4651(n);
	init_4738(n);
}


void stop_all_pubs(void)
{
	ROS_INFO("Stopping All HGuide Publishers");
	stop_2001();
	stop_2002();
	stop_2005();
	stop_2011();
	stop_2021();
	stop_20FF();
	stop_2201();
	stop_2301();
	stop_2311();
	stop_2401();
	stop_2402();
	stop_2411();
	stop_2421();
	stop_2422();
	stop_2423();
	stop_2424();
	stop_2425();
	stop_2426();
	stop_2427();
	stop_2428();
	stop_2429();
	stop_2501();
	stop_2611();
	stop_5001();
	stop_5012();
	stop_5101();
	stop_5102();
	stop_5103();
	stop_5104();
	stop_5105();
	stop_5106();
	stop_5108();
	stop_5109();
	stop_5201();
	stop_6001();
	stop_6003();
	stop_6108();
	stop_6109();
	stop_6110();
	stop_6111();
	stop_6112();
	stop_6201();
	stop_6202();
	stop_6203();
	stop_6204();
	stop_6205();
	stop_6211();
	stop_6311();
	stop_6403();
	stop_6405();
	stop_6406();
	stop_6424();
	stop_6428();
	stop_6438();
	stop_6504();
	stop_6505();
	stop_6508();
	stop_6601();
	stop_6651();
	stop_6721();
	stop_6722();
	stop_6723();
	stop_6724();
	stop_6725();
	stop_6726();
	stop_6738();
	stop_9900();
	stop_9910();
	stop_CE01();
	stop_FE01();
	stop_FE02();
}


void stop_all_subs(void)
{
	ROS_INFO("Stopping All HGuide Subscribers:");
	stop_1001();
	stop_1002();
	stop_1003();
	stop_1004();
	stop_1005();
	stop_1101();
	stop_1105();
	stop_1108();
	stop_1111();
	stop_1201();
	stop_1401();
	stop_4012();
	stop_4109();
	stop_4110();
	stop_4201();
	stop_4202();
	stop_4204();
	stop_4401();
	stop_4404();
	stop_4438();
	stop_4651();
	stop_4738();
}
