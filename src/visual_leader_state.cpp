/**
	虚拟leader 发送和接收ROS
	用于测试
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
#include <stdlib.h>
#include <ros/ros.h>


// ros msg
//#include <std_msgs/Int32.h>
//#include <nav_msgs/Odometry.h>
//#include <geometry_msgs/Twist.h>
//#include <geometry_msgs/PoseStamped.h>
#include<mavlink_uart/Formation_fixed_wing.h>

// quit handler 退出相关
void quit_handler(int sig);

mavlink_uart::Formation_fixed_wing Leader_DroneState;                          //Leader位置速度
mavlink_uart::Formation_fixed_wing Receive_DroneState;                         //从机接收到的Leader位置速度
int flag_leader_state_send = 0;

ros::Publisher drone_state_pub;

//ROS回调函数
void drone_state_cb(const mavlink_uart::Formation_fixed_wing::ConstPtr& msg)
{
    Receive_DroneState = *msg;

	flag_leader_state_send = 1;//topic更新标志
}

// ------------------------------------------------------------------------------
//   Main
// ------------------------------------------------------------------------------
int main(int argc, char **argv)
{
	ros::init(argc, argv, "visual_reader");
	ros::NodeHandle nh("~");

	//【订阅】通过串口通信模块接收回来的信息	// 接收从串口收到的leader的信息
    ros::Subscriber drone_state_sub = nh.subscribe<mavlink_uart::Formation_fixed_wing>("/mavros/fixed_wing_formation/status1", 10, drone_state_cb);

    // 【发布】需要通过串口通信模块发送的信息	// 发布leader的信息
    drone_state_pub = nh.advertise<mavlink_uart::Formation_fixed_wing>("/mavros/fixed_wing_formation/status", 10);

	// 设置 Rate
	ros::Rate loop_rate(20);

	//Responds to early exits signaled with Ctrl-C. 
	signal(SIGINT, quit_handler);

	//初始化
	int i = 0;
	flag_leader_state_send = 0;

	while (	ros::ok()	)
	{	
		ros::spinOnce();


			i = i+1 ;
			if (i%20 ==3)
			{
			Leader_DroneState.ned_vel_x = i;
			Leader_DroneState.ned_vel_y = i*i;
			Leader_DroneState.ned_vel_z = 99999999;

			Leader_DroneState.altitude = 0.123456789;
			Leader_DroneState.latitude = 987654321.123456789;
			Leader_DroneState.longtitude = i+100;
			drone_state_pub.publish(Leader_DroneState);
			
			cout << "---------------------------------" << endl;
				cout<<"Send " <<  endl;
				cout << "[vx, vy, vz] = " << fixed << setprecision(9) <<  Leader_DroneState. ned_vel_x << ", " << Leader_DroneState. ned_vel_y << ", " << Leader_DroneState. ned_vel_z << endl;
				cout << "altitude =" << Leader_DroneState. altitude << endl;
				cout << "latitude =" << Leader_DroneState. latitude << endl;
				cout << "longtitude =" << Leader_DroneState. longtitude << endl;
				cout << endl << endl ;
			}
			//

			if (flag_leader_state_send)
			{
				// 打印到屏幕上
				cout<<"Received " <<  endl;
				cout << "[vx, vy, vz] = " << Receive_DroneState. ned_vel_x << ", " << Receive_DroneState. ned_vel_y << ", " << Receive_DroneState. ned_vel_z << endl;
				//cout << "altitude =" << Receive_DroneState. altitude << endl;
				//cout << "latitude =" << Receive_DroneState. latitude << endl;
				//cout << "longtitude =" << Receive_DroneState. longtitude << endl;
				cout << endl << endl ;

				flag_leader_state_send = 0;
			}

		loop_rate.sleep();

	}

	return 0;
}

// ------------------------------------------------------------------------------
//   Quit Signal Handler	This function is called when you press Ctrl-C
void quit_handler(int sig)
{
	printf("\n");
	printf("TERMINATING AT USER REQUEST\n");
	printf("\n");
	
	// end program here
	exit(0);
}


								
