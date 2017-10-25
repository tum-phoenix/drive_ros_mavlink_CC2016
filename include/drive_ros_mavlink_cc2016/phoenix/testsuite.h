/** @file
 *	@brief MAVLink comm protocol testsuite generated from phoenix.xml
 *	@see http://qgroundcontrol.org/mavlink/
 */
#ifndef PHOENIX_TESTSUITE_H
#define PHOENIX_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL

static void mavlink_test_phoenix(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{

	mavlink_test_phoenix(system_id, component_id, last_msg);
}
#endif




static void mavlink_test_notification(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_notification_t packet_in = {
		963497464,
	17,
	"FGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZAB",
	"DEFGHIJKLMNOPQRSTUVWXYZABCDEF",
	};
	mavlink_notification_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.timestamp = packet_in.timestamp;
        	packet1.type = packet_in.type;
        
        	mav_array_memcpy(packet1.description, packet_in.description, sizeof(char)*50);
        	mav_array_memcpy(packet1.tag, packet_in.tag, sizeof(char)*30);
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_notification_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_notification_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_notification_pack(system_id, component_id, &msg , packet1.timestamp , packet1.type , packet1.description , packet1.tag );
	mavlink_msg_notification_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_notification_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.type , packet1.description , packet1.tag );
	mavlink_msg_notification_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_notification_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_notification_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.type , packet1.description , packet1.tag );
	mavlink_msg_notification_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_heartbeat(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_heartbeat_t packet_in = {
		963497464,
	45.0,
	73.0,
	101.0,
	18067,
	187,
	254,
	};
	mavlink_heartbeat_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.timestamp = packet_in.timestamp;
        	packet1.rc_velocity = packet_in.rc_velocity;
        	packet1.rc_steering_front = packet_in.rc_steering_front;
        	packet1.rc_steering_rear = packet_in.rc_steering_rear;
        	packet1.battery_voltage = packet_in.battery_voltage;
        	packet1.remote_control = packet_in.remote_control;
        	packet1.drive_mode = packet_in.drive_mode;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_heartbeat_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_heartbeat_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_heartbeat_pack(system_id, component_id, &msg , packet1.timestamp , packet1.battery_voltage , packet1.remote_control , packet1.drive_mode , packet1.rc_velocity , packet1.rc_steering_front , packet1.rc_steering_rear );
	mavlink_msg_heartbeat_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_heartbeat_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.battery_voltage , packet1.remote_control , packet1.drive_mode , packet1.rc_velocity , packet1.rc_steering_front , packet1.rc_steering_rear );
	mavlink_msg_heartbeat_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_heartbeat_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_heartbeat_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.battery_voltage , packet1.remote_control , packet1.drive_mode , packet1.rc_velocity , packet1.rc_steering_front , packet1.rc_steering_rear );
	mavlink_msg_heartbeat_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_debug(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_debug_t packet_in = {
		963497464,
	{ 45.0, 46.0, 47.0, 48.0, 49.0, 50.0, 51.0, 52.0, 53.0, 54.0, 55.0, 56.0 },
	};
	mavlink_debug_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.timestamp = packet_in.timestamp;
        
        	mav_array_memcpy(packet1.data, packet_in.data, sizeof(float)*12);
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_debug_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_debug_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_debug_pack(system_id, component_id, &msg , packet1.timestamp , packet1.data );
	mavlink_msg_debug_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_debug_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.data );
	mavlink_msg_debug_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_debug_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_debug_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.data );
	mavlink_msg_debug_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_telemetry(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_telemetry_t packet_in = {
		963497464,
	45.0,
	73.0,
	101.0,
	129.0,
	157.0,
	185.0,
	213.0,
	241.0,
	269.0,
	297.0,
	325.0,
	353.0,
	381.0,
	409.0,
	437.0,
	963500792,
	20771,
	20875,
	20979,
	21083,
	21187,
	21291,
	245,
	56,
	123,
	190,
	};
	mavlink_telemetry_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.timestamp = packet_in.timestamp;
        	packet1.xacc = packet_in.xacc;
        	packet1.yacc = packet_in.yacc;
        	packet1.zacc = packet_in.zacc;
        	packet1.xgyro = packet_in.xgyro;
        	packet1.ygyro = packet_in.ygyro;
        	packet1.zgyro = packet_in.zgyro;
        	packet1.dist_front = packet_in.dist_front;
        	packet1.dist_rear = packet_in.dist_rear;
        	packet1.dist_side = packet_in.dist_side;
        	packet1.odom = packet_in.odom;
        	packet1.odom_accumulated = packet_in.odom_accumulated;
        	packet1.xmotion_front = packet_in.xmotion_front;
        	packet1.ymotion_front = packet_in.ymotion_front;
        	packet1.xmotion_rear = packet_in.xmotion_rear;
        	packet1.ymotion_rear = packet_in.ymotion_rear;
        	packet1.current_motor = packet_in.current_motor;
        	packet1.current_servo_front = packet_in.current_servo_front;
        	packet1.current_servo_rear = packet_in.current_servo_rear;
        	packet1.current_total = packet_in.current_total;
        	packet1.pwm_servo_front = packet_in.pwm_servo_front;
        	packet1.pwm_servo_rear = packet_in.pwm_servo_rear;
        	packet1.battery_voltage = packet_in.battery_voltage;
        	packet1.motion_front_quality = packet_in.motion_front_quality;
        	packet1.motion_rear_quality = packet_in.motion_rear_quality;
        	packet1.remote_control = packet_in.remote_control;
        	packet1.drive_mode = packet_in.drive_mode;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_telemetry_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_telemetry_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_telemetry_pack(system_id, component_id, &msg , packet1.timestamp , packet1.xacc , packet1.yacc , packet1.zacc , packet1.xgyro , packet1.ygyro , packet1.zgyro , packet1.dist_front , packet1.dist_rear , packet1.dist_side , packet1.odom , packet1.odom_accumulated , packet1.xmotion_front , packet1.ymotion_front , packet1.xmotion_rear , packet1.ymotion_rear , packet1.motion_front_quality , packet1.motion_rear_quality , packet1.current_motor , packet1.current_servo_front , packet1.current_servo_rear , packet1.current_total , packet1.pwm_servo_front , packet1.pwm_servo_rear , packet1.battery_voltage , packet1.remote_control , packet1.drive_mode );
	mavlink_msg_telemetry_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_telemetry_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.xacc , packet1.yacc , packet1.zacc , packet1.xgyro , packet1.ygyro , packet1.zgyro , packet1.dist_front , packet1.dist_rear , packet1.dist_side , packet1.odom , packet1.odom_accumulated , packet1.xmotion_front , packet1.ymotion_front , packet1.xmotion_rear , packet1.ymotion_rear , packet1.motion_front_quality , packet1.motion_rear_quality , packet1.current_motor , packet1.current_servo_front , packet1.current_servo_rear , packet1.current_total , packet1.pwm_servo_front , packet1.pwm_servo_rear , packet1.battery_voltage , packet1.remote_control , packet1.drive_mode );
	mavlink_msg_telemetry_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_telemetry_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_telemetry_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.xacc , packet1.yacc , packet1.zacc , packet1.xgyro , packet1.ygyro , packet1.zgyro , packet1.dist_front , packet1.dist_rear , packet1.dist_side , packet1.odom , packet1.odom_accumulated , packet1.xmotion_front , packet1.ymotion_front , packet1.xmotion_rear , packet1.ymotion_rear , packet1.motion_front_quality , packet1.motion_rear_quality , packet1.current_motor , packet1.current_servo_front , packet1.current_servo_rear , packet1.current_total , packet1.pwm_servo_front , packet1.pwm_servo_rear , packet1.battery_voltage , packet1.remote_control , packet1.drive_mode );
	mavlink_msg_telemetry_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_control_lights(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_control_lights_t packet_in = {
		{ 963497464, 963497465, 963497466, 963497467, 963497468, 963497469, 963497470, 963497471, 963497472, 963497473, 963497474, 963497475, 963497476, 963497477, 963497478 },
	};
	mavlink_control_lights_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        
        	mav_array_memcpy(packet1.colors, packet_in.colors, sizeof(uint32_t)*15);
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_control_lights_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_control_lights_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_control_lights_pack(system_id, component_id, &msg , packet1.colors );
	mavlink_msg_control_lights_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_control_lights_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.colors );
	mavlink_msg_control_lights_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_control_lights_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_control_lights_send(MAVLINK_COMM_1 , packet1.colors );
	mavlink_msg_control_lights_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_control_command(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_control_command_t packet_in = {
		17.0,
	45.0,
	73.0,
	41,
	108,
	};
	mavlink_control_command_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.velocity = packet_in.velocity;
        	packet1.steering_front = packet_in.steering_front;
        	packet1.steering_rear = packet_in.steering_rear;
        	packet1.indicator_left = packet_in.indicator_left;
        	packet1.indicator_right = packet_in.indicator_right;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_control_command_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_control_command_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_control_command_pack(system_id, component_id, &msg , packet1.velocity , packet1.steering_front , packet1.steering_rear , packet1.indicator_left , packet1.indicator_right );
	mavlink_msg_control_command_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_control_command_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.velocity , packet1.steering_front , packet1.steering_rear , packet1.indicator_left , packet1.indicator_right );
	mavlink_msg_control_command_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_control_command_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_control_command_send(MAVLINK_COMM_1 , packet1.velocity , packet1.steering_front , packet1.steering_rear , packet1.indicator_left , packet1.indicator_right );
	mavlink_msg_control_command_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_imu(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_imu_t packet_in = {
		963497464,
	45.0,
	73.0,
	101.0,
	129.0,
	157.0,
	185.0,
	213.0,
	241.0,
	269.0,
	};
	mavlink_imu_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.timestamp = packet_in.timestamp;
        	packet1.xacc = packet_in.xacc;
        	packet1.yacc = packet_in.yacc;
        	packet1.zacc = packet_in.zacc;
        	packet1.xgyro = packet_in.xgyro;
        	packet1.ygyro = packet_in.ygyro;
        	packet1.zgyro = packet_in.zgyro;
        	packet1.xmag = packet_in.xmag;
        	packet1.ymag = packet_in.ymag;
        	packet1.zmag = packet_in.zmag;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_imu_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_imu_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_imu_pack(system_id, component_id, &msg , packet1.timestamp , packet1.xacc , packet1.yacc , packet1.zacc , packet1.xgyro , packet1.ygyro , packet1.zgyro , packet1.xmag , packet1.ymag , packet1.zmag );
	mavlink_msg_imu_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_imu_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.xacc , packet1.yacc , packet1.zacc , packet1.xgyro , packet1.ygyro , packet1.zgyro , packet1.xmag , packet1.ymag , packet1.zmag );
	mavlink_msg_imu_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_imu_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_imu_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.xacc , packet1.yacc , packet1.zacc , packet1.xgyro , packet1.ygyro , packet1.zgyro , packet1.xmag , packet1.ymag , packet1.zmag );
	mavlink_msg_imu_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_odometer_abs(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_odometer_abs_t packet_in = {
		963497464,
	45.0,
	73.0,
	101.0,
	129.0,
	157.0,
	185.0,
	18691,
	};
	mavlink_odometer_abs_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.timestamp = packet_in.timestamp;
        	packet1.xdist = packet_in.xdist;
        	packet1.ydist = packet_in.ydist;
        	packet1.zdist = packet_in.zdist;
        	packet1.xvelocity = packet_in.xvelocity;
        	packet1.yvelocity = packet_in.yvelocity;
        	packet1.zvelocity = packet_in.zvelocity;
        	packet1.quality = packet_in.quality;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_odometer_abs_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_odometer_abs_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_odometer_abs_pack(system_id, component_id, &msg , packet1.timestamp , packet1.xdist , packet1.ydist , packet1.zdist , packet1.xvelocity , packet1.yvelocity , packet1.zvelocity , packet1.quality );
	mavlink_msg_odometer_abs_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_odometer_abs_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.xdist , packet1.ydist , packet1.zdist , packet1.xvelocity , packet1.yvelocity , packet1.zvelocity , packet1.quality );
	mavlink_msg_odometer_abs_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_odometer_abs_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_odometer_abs_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.xdist , packet1.ydist , packet1.zdist , packet1.xvelocity , packet1.yvelocity , packet1.zvelocity , packet1.quality );
	mavlink_msg_odometer_abs_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_odometer_raw(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_odometer_raw_t packet_in = {
		963497464,
	963497672,
	963497880,
	963498088,
	18067,
	};
	mavlink_odometer_raw_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.timestamp = packet_in.timestamp;
        	packet1.xdist = packet_in.xdist;
        	packet1.ydist = packet_in.ydist;
        	packet1.zdist = packet_in.zdist;
        	packet1.quality = packet_in.quality;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_odometer_raw_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_odometer_raw_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_odometer_raw_pack(system_id, component_id, &msg , packet1.timestamp , packet1.xdist , packet1.ydist , packet1.zdist , packet1.quality );
	mavlink_msg_odometer_raw_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_odometer_raw_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.xdist , packet1.ydist , packet1.zdist , packet1.quality );
	mavlink_msg_odometer_raw_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_odometer_raw_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_odometer_raw_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.xdist , packet1.ydist , packet1.zdist , packet1.quality );
	mavlink_msg_odometer_raw_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_odometer_delta(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_odometer_delta_t packet_in = {
		963497464,
	45.0,
	73.0,
	101.0,
	129.0,
	157.0,
	185.0,
	213.0,
	18899,
	};
	mavlink_odometer_delta_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.timestamp = packet_in.timestamp;
        	packet1.delta = packet_in.delta;
        	packet1.xdist = packet_in.xdist;
        	packet1.ydist = packet_in.ydist;
        	packet1.zdist = packet_in.zdist;
        	packet1.xvelocity = packet_in.xvelocity;
        	packet1.yvelocity = packet_in.yvelocity;
        	packet1.zvelocity = packet_in.zvelocity;
        	packet1.quality = packet_in.quality;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_odometer_delta_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_odometer_delta_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_odometer_delta_pack(system_id, component_id, &msg , packet1.timestamp , packet1.delta , packet1.xdist , packet1.ydist , packet1.zdist , packet1.xvelocity , packet1.yvelocity , packet1.zvelocity , packet1.quality );
	mavlink_msg_odometer_delta_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_odometer_delta_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.delta , packet1.xdist , packet1.ydist , packet1.zdist , packet1.xvelocity , packet1.yvelocity , packet1.zvelocity , packet1.quality );
	mavlink_msg_odometer_delta_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_odometer_delta_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_odometer_delta_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.delta , packet1.xdist , packet1.ydist , packet1.zdist , packet1.xvelocity , packet1.yvelocity , packet1.zvelocity , packet1.quality );
	mavlink_msg_odometer_delta_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_odometer_delta_raw(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_odometer_delta_raw_t packet_in = {
		963497464,
	45.0,
	963497880,
	963498088,
	963498296,
	18275,
	};
	mavlink_odometer_delta_raw_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.timestamp = packet_in.timestamp;
        	packet1.delta = packet_in.delta;
        	packet1.xdist = packet_in.xdist;
        	packet1.ydist = packet_in.ydist;
        	packet1.zdist = packet_in.zdist;
        	packet1.quality = packet_in.quality;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_odometer_delta_raw_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_odometer_delta_raw_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_odometer_delta_raw_pack(system_id, component_id, &msg , packet1.timestamp , packet1.delta , packet1.xdist , packet1.ydist , packet1.zdist , packet1.quality );
	mavlink_msg_odometer_delta_raw_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_odometer_delta_raw_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.delta , packet1.xdist , packet1.ydist , packet1.zdist , packet1.quality );
	mavlink_msg_odometer_delta_raw_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_odometer_delta_raw_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_odometer_delta_raw_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.delta , packet1.xdist , packet1.ydist , packet1.zdist , packet1.quality );
	mavlink_msg_odometer_delta_raw_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_odometer(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_odometer_t packet_in = {
		963497464,
	45.0,
	73.0,
	101.0,
	129.0,
	157.0,
	185.0,
	213.0,
	241.0,
	269.0,
	297.0,
	19523,
	};
	mavlink_odometer_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.timestamp = packet_in.timestamp;
        	packet1.time_delta = packet_in.time_delta;
        	packet1.xdist_delta = packet_in.xdist_delta;
        	packet1.ydist_delta = packet_in.ydist_delta;
        	packet1.zdist_delta = packet_in.zdist_delta;
        	packet1.xdist_abs = packet_in.xdist_abs;
        	packet1.ydist_abs = packet_in.ydist_abs;
        	packet1.zdist_abs = packet_in.zdist_abs;
        	packet1.xvelocity = packet_in.xvelocity;
        	packet1.yvelocity = packet_in.yvelocity;
        	packet1.zvelocity = packet_in.zvelocity;
        	packet1.quality = packet_in.quality;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_odometer_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_odometer_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_odometer_pack(system_id, component_id, &msg , packet1.timestamp , packet1.time_delta , packet1.xdist_delta , packet1.ydist_delta , packet1.zdist_delta , packet1.xdist_abs , packet1.ydist_abs , packet1.zdist_abs , packet1.xvelocity , packet1.yvelocity , packet1.zvelocity , packet1.quality );
	mavlink_msg_odometer_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_odometer_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.time_delta , packet1.xdist_delta , packet1.ydist_delta , packet1.zdist_delta , packet1.xdist_abs , packet1.ydist_abs , packet1.zdist_abs , packet1.xvelocity , packet1.yvelocity , packet1.zvelocity , packet1.quality );
	mavlink_msg_odometer_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_odometer_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_odometer_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.time_delta , packet1.xdist_delta , packet1.ydist_delta , packet1.zdist_delta , packet1.xdist_abs , packet1.ydist_abs , packet1.zdist_abs , packet1.xvelocity , packet1.yvelocity , packet1.zvelocity , packet1.quality );
	mavlink_msg_odometer_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_proximity(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_proximity_t packet_in = {
		963497464,
	45.0,
	};
	mavlink_proximity_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.timestamp = packet_in.timestamp;
        	packet1.distance = packet_in.distance;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_proximity_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_proximity_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_proximity_pack(system_id, component_id, &msg , packet1.timestamp , packet1.distance );
	mavlink_msg_proximity_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_proximity_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.distance );
	mavlink_msg_proximity_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_proximity_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_proximity_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.distance );
	mavlink_msg_proximity_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_parking_lot(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_parking_lot_t packet_in = {
		963497464,
	45.0,
	73.0,
	};
	mavlink_parking_lot_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.timestamp = packet_in.timestamp;
        	packet1.parking_lot_size = packet_in.parking_lot_size;
        	packet1.parking_lot_position = packet_in.parking_lot_position;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_parking_lot_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_parking_lot_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_parking_lot_pack(system_id, component_id, &msg , packet1.timestamp , packet1.parking_lot_size , packet1.parking_lot_position );
	mavlink_msg_parking_lot_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_parking_lot_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.parking_lot_size , packet1.parking_lot_position );
	mavlink_msg_parking_lot_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_parking_lot_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_parking_lot_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.parking_lot_size , packet1.parking_lot_position );
	mavlink_msg_parking_lot_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_config_request_count(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_config_request_count_t packet_in = {
		5,
	};
	mavlink_config_request_count_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.dummy = packet_in.dummy;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_config_request_count_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_config_request_count_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_config_request_count_pack(system_id, component_id, &msg , packet1.dummy );
	mavlink_msg_config_request_count_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_config_request_count_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.dummy );
	mavlink_msg_config_request_count_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_config_request_count_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_config_request_count_send(MAVLINK_COMM_1 , packet1.dummy );
	mavlink_msg_config_request_count_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_config_request(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_config_request_t packet_in = {
		5,
	};
	mavlink_config_request_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.config_id = packet_in.config_id;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_config_request_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_config_request_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_config_request_pack(system_id, component_id, &msg , packet1.config_id );
	mavlink_msg_config_request_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_config_request_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.config_id );
	mavlink_msg_config_request_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_config_request_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_config_request_send(MAVLINK_COMM_1 , packet1.config_id );
	mavlink_msg_config_request_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_config_request_params(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_config_request_params_t packet_in = {
		5,
	72,
	};
	mavlink_config_request_params_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.config_id = packet_in.config_id;
        	packet1.param_id = packet_in.param_id;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_config_request_params_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_config_request_params_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_config_request_params_pack(system_id, component_id, &msg , packet1.config_id , packet1.param_id );
	mavlink_msg_config_request_params_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_config_request_params_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.config_id , packet1.param_id );
	mavlink_msg_config_request_params_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_config_request_params_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_config_request_params_send(MAVLINK_COMM_1 , packet1.config_id , packet1.param_id );
	mavlink_msg_config_request_params_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_config_count(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_config_count_t packet_in = {
		963497464,
	};
	mavlink_config_count_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.config_id_mask = packet_in.config_id_mask;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_config_count_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_config_count_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_config_count_pack(system_id, component_id, &msg , packet1.config_id_mask );
	mavlink_msg_config_count_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_config_count_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.config_id_mask );
	mavlink_msg_config_count_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_config_count_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_config_count_send(MAVLINK_COMM_1 , packet1.config_id_mask );
	mavlink_msg_config_count_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_config(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_config_t packet_in = {
		17235,
	139,
	"DEFGHIJKLMNOPQRSTUVWXYZABCDEF",
	};
	mavlink_config_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.param_id_mask = packet_in.param_id_mask;
        	packet1.config_id = packet_in.config_id;
        
        	mav_array_memcpy(packet1.name, packet_in.name, sizeof(char)*30);
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_config_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_config_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_config_pack(system_id, component_id, &msg , packet1.config_id , packet1.name , packet1.param_id_mask );
	mavlink_msg_config_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_config_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.config_id , packet1.name , packet1.param_id_mask );
	mavlink_msg_config_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_config_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_config_send(MAVLINK_COMM_1 , packet1.config_id , packet1.name , packet1.param_id_mask );
	mavlink_msg_config_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_config_param_int(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_config_param_int_t packet_in = {
		963497464,
	963497672,
	963497880,
	963498088,
	53,
	120,
	"STUVWXYZABCDEFGHIJKLMNOPQRSTU",
	};
	mavlink_config_param_int_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.value = packet_in.value;
        	packet1.default_value = packet_in.default_value;
        	packet1.min = packet_in.min;
        	packet1.max = packet_in.max;
        	packet1.config_id = packet_in.config_id;
        	packet1.param_id = packet_in.param_id;
        
        	mav_array_memcpy(packet1.name, packet_in.name, sizeof(char)*30);
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_config_param_int_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_config_param_int_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_config_param_int_pack(system_id, component_id, &msg , packet1.config_id , packet1.param_id , packet1.name , packet1.value , packet1.default_value , packet1.min , packet1.max );
	mavlink_msg_config_param_int_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_config_param_int_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.config_id , packet1.param_id , packet1.name , packet1.value , packet1.default_value , packet1.min , packet1.max );
	mavlink_msg_config_param_int_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_config_param_int_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_config_param_int_send(MAVLINK_COMM_1 , packet1.config_id , packet1.param_id , packet1.name , packet1.value , packet1.default_value , packet1.min , packet1.max );
	mavlink_msg_config_param_int_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_config_param_bool(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_config_param_bool_t packet_in = {
		5,
	72,
	"CDEFGHIJKLMNOPQRSTUVWXYZABCDE",
	101,
	168,
	};
	mavlink_config_param_bool_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.config_id = packet_in.config_id;
        	packet1.param_id = packet_in.param_id;
        	packet1.value = packet_in.value;
        	packet1.default_value = packet_in.default_value;
        
        	mav_array_memcpy(packet1.name, packet_in.name, sizeof(char)*30);
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_config_param_bool_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_config_param_bool_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_config_param_bool_pack(system_id, component_id, &msg , packet1.config_id , packet1.param_id , packet1.name , packet1.value , packet1.default_value );
	mavlink_msg_config_param_bool_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_config_param_bool_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.config_id , packet1.param_id , packet1.name , packet1.value , packet1.default_value );
	mavlink_msg_config_param_bool_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_config_param_bool_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_config_param_bool_send(MAVLINK_COMM_1 , packet1.config_id , packet1.param_id , packet1.name , packet1.value , packet1.default_value );
	mavlink_msg_config_param_bool_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_config_param_float(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_config_param_float_t packet_in = {
		17.0,
	45.0,
	73.0,
	101.0,
	53,
	120,
	"STUVWXYZABCDEFGHIJKLMNOPQRSTU",
	};
	mavlink_config_param_float_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.value = packet_in.value;
        	packet1.default_value = packet_in.default_value;
        	packet1.min = packet_in.min;
        	packet1.max = packet_in.max;
        	packet1.config_id = packet_in.config_id;
        	packet1.param_id = packet_in.param_id;
        
        	mav_array_memcpy(packet1.name, packet_in.name, sizeof(char)*30);
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_config_param_float_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_config_param_float_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_config_param_float_pack(system_id, component_id, &msg , packet1.config_id , packet1.param_id , packet1.name , packet1.value , packet1.default_value , packet1.min , packet1.max );
	mavlink_msg_config_param_float_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_config_param_float_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.config_id , packet1.param_id , packet1.name , packet1.value , packet1.default_value , packet1.min , packet1.max );
	mavlink_msg_config_param_float_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_config_param_float_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_config_param_float_send(MAVLINK_COMM_1 , packet1.config_id , packet1.param_id , packet1.name , packet1.value , packet1.default_value , packet1.min , packet1.max );
	mavlink_msg_config_param_float_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_config_param_set_int(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_config_param_set_int_t packet_in = {
		963497464,
	17,
	84,
	};
	mavlink_config_param_set_int_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.value = packet_in.value;
        	packet1.config_id = packet_in.config_id;
        	packet1.param_id = packet_in.param_id;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_config_param_set_int_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_config_param_set_int_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_config_param_set_int_pack(system_id, component_id, &msg , packet1.config_id , packet1.param_id , packet1.value );
	mavlink_msg_config_param_set_int_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_config_param_set_int_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.config_id , packet1.param_id , packet1.value );
	mavlink_msg_config_param_set_int_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_config_param_set_int_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_config_param_set_int_send(MAVLINK_COMM_1 , packet1.config_id , packet1.param_id , packet1.value );
	mavlink_msg_config_param_set_int_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_config_param_set_bool(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_config_param_set_bool_t packet_in = {
		5,
	72,
	139,
	};
	mavlink_config_param_set_bool_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.config_id = packet_in.config_id;
        	packet1.param_id = packet_in.param_id;
        	packet1.value = packet_in.value;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_config_param_set_bool_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_config_param_set_bool_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_config_param_set_bool_pack(system_id, component_id, &msg , packet1.config_id , packet1.param_id , packet1.value );
	mavlink_msg_config_param_set_bool_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_config_param_set_bool_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.config_id , packet1.param_id , packet1.value );
	mavlink_msg_config_param_set_bool_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_config_param_set_bool_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_config_param_set_bool_send(MAVLINK_COMM_1 , packet1.config_id , packet1.param_id , packet1.value );
	mavlink_msg_config_param_set_bool_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_config_param_set_float(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_config_param_set_float_t packet_in = {
		17.0,
	17,
	84,
	};
	mavlink_config_param_set_float_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.value = packet_in.value;
        	packet1.config_id = packet_in.config_id;
        	packet1.param_id = packet_in.param_id;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_config_param_set_float_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_config_param_set_float_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_config_param_set_float_pack(system_id, component_id, &msg , packet1.config_id , packet1.param_id , packet1.value );
	mavlink_msg_config_param_set_float_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_config_param_set_float_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.config_id , packet1.param_id , packet1.value );
	mavlink_msg_config_param_set_float_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_config_param_set_float_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_config_param_set_float_send(MAVLINK_COMM_1 , packet1.config_id , packet1.param_id , packet1.value );
	mavlink_msg_config_param_set_float_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_command(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_command_t packet_in = {
		5,
	};
	mavlink_command_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.command = packet_in.command;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_command_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_command_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_command_pack(system_id, component_id, &msg , packet1.command );
	mavlink_msg_command_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_command_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.command );
	mavlink_msg_command_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_command_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_command_send(MAVLINK_COMM_1 , packet1.command );
	mavlink_msg_command_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_phoenix(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_test_notification(system_id, component_id, last_msg);
	mavlink_test_heartbeat(system_id, component_id, last_msg);
	mavlink_test_debug(system_id, component_id, last_msg);
	mavlink_test_telemetry(system_id, component_id, last_msg);
	mavlink_test_control_lights(system_id, component_id, last_msg);
	mavlink_test_control_command(system_id, component_id, last_msg);
	mavlink_test_imu(system_id, component_id, last_msg);
	mavlink_test_odometer_abs(system_id, component_id, last_msg);
	mavlink_test_odometer_raw(system_id, component_id, last_msg);
	mavlink_test_odometer_delta(system_id, component_id, last_msg);
	mavlink_test_odometer_delta_raw(system_id, component_id, last_msg);
	mavlink_test_odometer(system_id, component_id, last_msg);
	mavlink_test_proximity(system_id, component_id, last_msg);
	mavlink_test_parking_lot(system_id, component_id, last_msg);
	mavlink_test_config_request_count(system_id, component_id, last_msg);
	mavlink_test_config_request(system_id, component_id, last_msg);
	mavlink_test_config_request_params(system_id, component_id, last_msg);
	mavlink_test_config_count(system_id, component_id, last_msg);
	mavlink_test_config(system_id, component_id, last_msg);
	mavlink_test_config_param_int(system_id, component_id, last_msg);
	mavlink_test_config_param_bool(system_id, component_id, last_msg);
	mavlink_test_config_param_float(system_id, component_id, last_msg);
	mavlink_test_config_param_set_int(system_id, component_id, last_msg);
	mavlink_test_config_param_set_bool(system_id, component_id, last_msg);
	mavlink_test_config_param_set_float(system_id, component_id, last_msg);
	mavlink_test_command(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // PHOENIX_TESTSUITE_H
