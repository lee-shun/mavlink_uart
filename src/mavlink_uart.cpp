/**
 * @file mavlink_uart.cpp
 *
 * @brief An example uart communication via mavlink, definition
 *
 * This process connects an external MAVLink UART device to send and receive data,
 * For multi-UAV mission plan
 *	
 * 参考自 c_uart_interface_example 修改而来
 *
 */

using namespace std;
#include <iostream>
#include <stdio.h>
#include <cstdlib>
#include <unistd.h>
#include <cmath>
#include <string.h>
#include <inttypes.h>
#include <fstream>
#include <signal.h>
#include <time.h>
#include <sys/time.h>
using std::string;
#include <stdlib.h>
#include <ros/ros.h>
#include <serial/serial.h>

#include <mavlink_uart.h>

// ros msg
//#include <std_msgs/Int32.h>
//#include <nav_msgs/Odometry.h>
//#include <geometry_msgs/Twist.h>
//#include <geometry_msgs/PoseStamped.h>
#include<mavlink_uart/Formation_fixed_wing.h>

//待处理， 用于多端口接收端头处理
const uint8_t CONFIG_Header_0 = 0xEB;               //帧头0 

//串口相关
serial::Serial sp;			//创建一个serial类
//默认串口参数
int uart_name_No;	//打开"/dev/ttyUSB*"的编号
int baudrate = 57600;

uint8_t Rbuffer_rx[1024];          //串口接收Raw 
size_t  Rn;                                         //串口接收Raw 字节数
uint8_t Wbuffer_tx[1024];		//串口发送Raw 
size_t  Wn;   									//串口发送Raw 字节数

mavlink_message_t message;

Mavlink_Messages current_messages;
Time_Stamps this_timestamps;

int system_id = 1;	//设备编号
int companion_id = 2; //用作接收编号？
int write_flag = 0;
int case0r_1s=0;

//函数申明
// mavlink解包相关
int read_message(mavlink_message_t &message, uint8_t cp);
void handle_message();

// quit handler 退出相关
void quit_handler(int sig);
// 时间 helper
uint64_t get_time_usec();


mavlink_uart::Formation_fixed_wing Leader_DroneState;                          //Leader位置速度
mavlink_uart::Formation_fixed_wing Receive_DroneState;                         //从机接收到的Leader位置速度
int flag_leader_state_send = 0;


ros::Publisher drone_state_pub;

//ROS回调函数
void drone_state_cb(const mavlink_uart::Formation_fixed_wing::ConstPtr& msg)
{
    Leader_DroneState = *msg;

	flag_leader_state_send = 1;//topic更新标志
}

// ------------------------------------------------------------------------------
//   Main
// ------------------------------------------------------------------------------
int main(int argc, char **argv)
{
	ros::init(argc, argv, "mavlink_mission");
	ros::NodeHandle nh("~");


	//【订阅】需要通过串口通信模块发送出去的信息
    // 接收leader的位置速度信息
    ros::Subscriber drone_state_sub = nh.subscribe<mavlink_uart::Formation_fixed_wing>("/mavros/fixed_wing_formation/status", 10, drone_state_cb);

    // 【发布】通过串口通信模块接收到的信息
	// 发布接收到的leader的位置速度信息
    drone_state_pub = nh.advertise<mavlink_uart::Formation_fixed_wing>("/mavros/fixed_wing_formation/status", 10);

	// 设置 Rate
	ros::Rate loop_rate(20);

	//Responds to early exits signaled with Ctrl-C. 
	signal(SIGINT, quit_handler);

	// ----------------------------------------------------------------------
	//   读取param参数
	// ----------------------------------------------------------------------
	nh.param<int>("baudrate", baudrate, 57600);
	nh.param<int>("system_id", system_id, 1);
	nh.param<int>("uart_name_No", uart_name_No, 0);
        nh.param<int>("case0r_1s", case0r_1s, 0);

	string serial_port_adr("/dev/ttyUSB" + to_string(uart_name_No));

	cout << "baudrate: " << baudrate << endl;
	cout << "system_id: " << system_id << endl;
        cout << "0read_1send: " << case0r_1s << endl;
	cout << "uart_name_No: " << uart_name_No << endl;
	cout << "serial_port_adr: " << serial_port_adr.c_str() << endl;

	// ----------------------------------------------------------------------
	//   串口serial设置&开启
	// ----------------------------------------------------------------------
	serial::Timeout to = serial::Timeout::simpleTimeout(10); //创建timeout
	sp.setPort(serial_port_adr.c_str());					 	//设置要打开的串口名称
	//sp.setPort("/dev/ttyUSB0");
	sp.setBaudrate(baudrate);									 	//设置串口通信的波特率
	sp.setTimeout(to);										 				//串口设置timeout
	//打开串口
	try
	{
		sp.open();
	}
	catch (serial::IOException &e)
	{
		ROS_ERROR_STREAM("Unable to open port.");
		return -1;
	}
	//判断串口是否打开成功
	if (sp.isOpen())
	{
		ROS_INFO_STREAM("/dev/ttyUSB"+to_string(uart_name_No)+ " is Opened.");
	}
	else
	{
		ROS_INFO_STREAM("/dev/ttyUSB"+to_string(uart_name_No)+ " Open Failed!.");
		return -1;
	}

	//初始化
	int i = 0;
	flag_leader_state_send = 0;

	while (	ros::ok()	)
	{	
		ros::spinOnce();

                switch (case0r_1s)
		{
                        //case1 发送		//case0 接收
			case 0:
				// ----------------------------------------------------------------------
				//   进行串口字节读取，并解包 
				// ----------------------------------------------------------------------
				Rn = sp.available();	//获取缓冲区内的字节数
				// 如果有待读取字节数
				if (Rn != 0)
				{
					Rn = sp.read(Rbuffer_rx, Rn); //读出数据

					// 打印读取的字节到屏幕上
					cout << endl << endl <<"Receive " << (int)Rn << " byte: " << endl;
					for (int i = 0; i < Rn; i++)
					{
						cout << hex << (Rbuffer_rx[i] & 0xff) << " ";
					}
					cout << endl;
				
					// ----------------------------------------------------------------------
					//   字节解析Message
					// ----------------------------------------------------------------------
					//  通信模块配置	ATS153=1 Address Tag 启用后， 可辨别来自多个模块的信息
					//	其消息格式 待确认
					/**Type 1: First 2 bytes - size (high byte first), one byte - RSSI, one reserved byte
					 *				 (0x00), last six bytes - source unit address (high byte first).
					 */
					bool success;	//mavlink 信息正确标志
					for (int i = 0; i < Rn; i++)
					{
						success = read_message(message, Rbuffer_rx[i] & 0xff);
						if (success)
						{		
							cout<<"Receive a complete Mavlink message"  << endl;
							handle_message();	//读取并转化为rostopic pub出去
						}
					}

					if (!success)
					{
						//cout<<"Not a complete"  << endl;
					}

				}// end 如果有接收到if (Rn != 0)

			break;			//end case 0:

			case 1:

				// ----------------------------------------------------------------------
				//   进行串口字节发送，并解包 
				// ----------------------------------------------------------------------
				/**
				 	Wbuffer_tx[0] = CONFIG_Header_0;     //配置帧头 0xEB
			 		Wbuffer_tx[1] = 0x99;           
				*/
				
					i = i+1;//心跳包计数

					mavlink_self_fax_formation_t sp_mavlink;
					mavlink_heartbeat_t hb_mavlink;
					mavlink_message_t message_to_send;//需要发送的mavlink message

					if ( flag_leader_state_send)	//rostopic 有更新
					{
    				// --------------------------------------------------------------------------
					//   PACK PAYLOAD
					sp_mavlink.ned_vel_x       = Leader_DroneState.ned_vel_x;
					sp_mavlink.ned_vel_y       = Leader_DroneState.ned_vel_y;
					sp_mavlink.ned_vel_z       = Leader_DroneState.ned_vel_z;
					sp_mavlink.latitude       = Leader_DroneState.latitude;
					sp_mavlink.altitude       = Leader_DroneState.altitude;
					sp_mavlink.longtitude  = Leader_DroneState.longtitude;

					flag_leader_state_send = 0;	//topic更新标志置 0 

				 	// --------------------------------------------------------------------------
					//   ENCODE
					mavlink_msg_self_fax_formation_encode(system_id, companion_id, &message_to_send, &sp_mavlink);
					write_flag = 1;
					}
					else if ( (i%20) == 19) //1s一个心跳包
					{
						// --------------------------------------------------------------------------
						//   PACK PAYLOAD
						hb_mavlink.custom_mode = 66;/*<  A bitfield for use for autopilot-specific flags*/
						hb_mavlink.type = MAV_TYPE_FIXED_WING;/*<  Type of the MAV (quadrotor, helicopter, etc.)*/
						hb_mavlink.autopilot = MAV_AUTOPILOT_PX4; /*<  Autopilot type / class.*/
						hb_mavlink.base_mode = MAV_MODE_FLAG_GUIDED_ENABLED;/*<  System mode bitmap.*/
						hb_mavlink.system_status = MAV_STATE_ACTIVE;/*<  System status flag.*/
					
						// --------------------------------------------------------------------------
						//   ENCODE
						mavlink_msg_heartbeat_encode(system_id, companion_id, &message_to_send, &hb_mavlink);
						write_flag = 1;

						i = 0;
					}
					
				// --------------------------------------------------------------------------
				//   WRITE
				// Translate message to buffer
				if (write_flag)
				{
					unsigned len;
					len  = mavlink_msg_to_send_buffer((uint8_t*)Wbuffer_tx, &message_to_send);
					// Write buffer to serial port
					sp.write(Wbuffer_tx, len);

					// 打印到屏幕上
					cout<<"Send " << (int)len << " byte: " << endl;
					for (int i = 0; i < len; i++)
					{
						cout << hex << (Wbuffer_tx[i] & 0xff) << " ";
					}
					cout << endl << endl;

					write_flag = 0;
				}
			break;			//end case 1

			default:
				cout << "Open port Error! " << endl;
				return -1;
			break;
		}

		loop_rate.sleep();

	}

	//关闭串口s
    sp.close();
	return 0;
}

// ------------------------------------------------------------------------------
//   Quit Signal Handler	This function is called when you press Ctrl-C
void quit_handler(int sig)
{
	printf("\n");
	printf("TERMINATING AT USER REQUEST\n");
	printf("\n");
	
	sp.close();
	// end program here
	exit(0);
}


int read_message(mavlink_message_t &message, uint8_t cp)
{
	mavlink_status_t status;
	uint8_t          msgReceived = false;

	// --------------------------------------------------------------------------
	//   PARSE MESSAGE
		msgReceived = mavlink_parse_char(MAVLINK_COMM_1, cp, &message, &status);
		//MAVLINK_COMM_1 处可更改为通道数，用于处理来自于不同终端的信息

	return msgReceived;
}


void handle_message()
{
	// Store message sysid and compid.
	// Note this doesn't handle multiple message sources.
	current_messages.sysid  = message.sysid;
	current_messages.compid = message.compid;

	// Handle Message ID
	switch (message.msgid)
		{

			case MAVLINK_MSG_ID_HEARTBEAT:
				{
					printf("MAVLINK_MSG_ID_HEARTBEAT\n");
					mavlink_msg_heartbeat_decode(&message, &(current_messages.heartbeat));
					current_messages.time_stamps.heartbeat = get_time_usec();
					this_timestamps.heartbeat = current_messages.time_stamps.heartbeat;
					
					mavlink_heartbeat_t heart_rc = current_messages.heartbeat;

					printf(" Datefrom  sysid=%i ,  compid=%i  \n",  current_messages.sysid,  current_messages.compid);
					printf(" custom_mode = %i ,",  heart_rc.custom_mode);
					printf(" type = %i ,",  heart_rc.type);
					printf(" autopilot = %i ,",  heart_rc.autopilot);
					printf(" \n ");
					printf(" base_mode = %i ,",  heart_rc.base_mode);
					printf(" system_status = %i ,",  heart_rc.system_status);
					printf(" mavlink_version = %i ,",  heart_rc.mavlink_version);
					printf(" \n ");
					break;
				}

			case MAVLINK_MSG_ID_SYS_STATUS:
				{
					//printf("MAVLINK_MSG_ID_SYS_STATUS\n");
					mavlink_msg_sys_status_decode(&message, &(current_messages.sys_status));
					current_messages.time_stamps.sys_status = get_time_usec();
					this_timestamps.sys_status = current_messages.time_stamps.sys_status;
					break;
				}

			case MAVLINK_MSG_ID_BATTERY_STATUS:
				{
					//printf("MAVLINK_MSG_ID_BATTERY_STATUS\n");
					mavlink_msg_battery_status_decode(&message, &(current_messages.battery_status));
					current_messages.time_stamps.battery_status = get_time_usec();
					this_timestamps.battery_status = current_messages.time_stamps.battery_status;
					break;
				}

			case MAVLINK_MSG_ID_RADIO_STATUS:
				{
					//printf("MAVLINK_MSG_ID_RADIO_STATUS\n");
					mavlink_msg_radio_status_decode(&message, &(current_messages.radio_status));
					current_messages.time_stamps.radio_status = get_time_usec();
					this_timestamps.radio_status = current_messages.time_stamps.radio_status;
					break;
				}

			case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
				{
					//printf("MAVLINK_MSG_ID_LOCAL_POSITION_NED\n");
					mavlink_msg_local_position_ned_decode(&message, &(current_messages.local_position_ned));
					current_messages.time_stamps.local_position_ned = get_time_usec();
					this_timestamps.local_position_ned = current_messages.time_stamps.local_position_ned;
					break;
				}

			case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
				{
					//printf("MAVLINK_MSG_ID_GLOBAL_POSITION_INT\n");
					mavlink_msg_global_position_int_decode(&message, &(current_messages.global_position_int));
					current_messages.time_stamps.global_position_int = get_time_usec();
					this_timestamps.global_position_int = current_messages.time_stamps.global_position_int;
					break;
				}

			case MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED:
				{
					//printf("MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED\n");
					mavlink_msg_position_target_local_ned_decode(&message, &(current_messages.position_target_local_ned));
					current_messages.time_stamps.position_target_local_ned = get_time_usec();
					this_timestamps.position_target_local_ned = current_messages.time_stamps.position_target_local_ned;
					break;
				}

			case MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT:
				{
					//printf("MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT\n");
					mavlink_msg_position_target_global_int_decode(&message, &(current_messages.position_target_global_int));
					current_messages.time_stamps.position_target_global_int = get_time_usec();
					this_timestamps.position_target_global_int = current_messages.time_stamps.position_target_global_int;
					break;
				}

			case MAVLINK_MSG_ID_HIGHRES_IMU:
				{
					//printf("MAVLINK_MSG_ID_HIGHRES_IMU\n");
					mavlink_msg_highres_imu_decode(&message, &(current_messages.highres_imu));
					current_messages.time_stamps.highres_imu = get_time_usec();
					this_timestamps.highres_imu = current_messages.time_stamps.highres_imu;
					break;
				}

			case MAVLINK_MSG_ID_ATTITUDE:
				{
					//printf("MAVLINK_MSG_ID_ATTITUDE\n");
					mavlink_msg_attitude_decode(&message, &(current_messages.attitude));
					current_messages.time_stamps.attitude = get_time_usec();
					this_timestamps.attitude = current_messages.time_stamps.attitude;
					break;
				}

			//add test
				case MAVLINK_MSG_ID_SELF_FAX_FORMATION:
				{
					printf("MAVLINK_MSG_ID_SELF_FAX_FORMATION\n");
					mavlink_msg_self_fax_formation_decode(&message, &(current_messages.self_fax_formation));
					current_messages.time_stamps.self_fax_formation = get_time_usec();
					this_timestamps.self_fax_formation = current_messages.time_stamps.self_fax_formation;
					
					mavlink_self_fax_formation_t position = current_messages.self_fax_formation;

					Receive_DroneState.ned_vel_x = position.ned_vel_x;
					Receive_DroneState.ned_vel_y = position.ned_vel_y;
					Receive_DroneState.ned_vel_z = position.ned_vel_z;
					Receive_DroneState.latitude 	= position.latitude;
					Receive_DroneState.altitude 	= position.altitude;
					Receive_DroneState.longtitude = position.longtitude;
										
					drone_state_pub.publish(Receive_DroneState);

					cout << "[vx, vy] = " << Receive_DroneState.ned_vel_x << ", " << Receive_DroneState.ned_vel_y<< endl;

					break;
				}

			default:
				{
					printf("Warning, did not handle message id %i\n",message.msgid);
					break;
				}

		} // end: switch msgid

};

// ----------------------------------------------------------------------------------
//   Time
// ------------------- ---------------------------------------------------------------
uint64_t
get_time_usec()
{
	static struct timeval _time_stamp;
	gettimeofday(&_time_stamp, NULL);
	return _time_stamp.tv_sec*1000000 + _time_stamp.tv_usec;
}
