// MavlinkTest.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"

#include "minimal\mavlink.h"
int _tmain(int argc, _TCHAR* argv[])
{
	mavlink_heartbeat_t t;
	mavlink_message_t message;
	uint8_t buf[1024];
	mavlink_msg_heartbeat_pack(1, 200, &message, MAV_TYPE_HELICOPTER, MAV_AUTOPILOT_GENERIC, MAV_MODE_FLAG_AUTO_ENABLED, 0, MAV_STATE_ACTIVE);

        // Copy the message to send buffer
        uint16_t len = mavlink_msg_to_send_buffer(buf, &message);
	mavlink_msg_heartbeat_decode(&message, &t);
	return 0;
}

