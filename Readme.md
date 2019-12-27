
-------------------------------------------------------------
功能说明
ROS
本功能包利用与串口通讯硬件搭配使用，实现无人机集群任务层面通信。
    包括无人机和地面站之间的 
	1. 1对1通信

已经包含功能
	0. 接收&发布ROS话题
	1. 心跳包功能
	2. 发送用于固定翼无人机编队的 /位置和/速度信息

待完善/添加功能
	2. 1对多通信
	3. 多对多通信
	对于 2&3 需要先分辨出来自于不同端口的信息，然后调用mavlink函数处理 
	4. 是否通信握手应答 (用于确保指令正确传输到了)
	...
	
----------------------------------------------------------------------------------------------
信息打包编码方式采用 Mavlink 2.0 协议	
	Note 此处自定义了 (#304消息格式) 详细见 message_definitions 内common.xml文件
	
	#有关生成 h头文件详见mavlink教程
		-Generating MAVLink Libraries
		 [https://mavlink.io/en/getting_started/generate_libraries.html]
	#Mavlink消息查询
		[https://mavlink.io/en/messages/common.html]
	#Mavlink 手册 				
		[https://mavlink.io/en/]
	#Mavlink Github			
		[https://github.com/mavlink/mavlink]
	#c_uart_interface_example 程序例程	
		[https://github.com/mavlink/c_uart_interface_example]

------------------------------------------------------------
准备工作

1.安装ROS的串口通信包 serial包
  #install 
	ubuntu 14.04 LTS
 		sudo apt-get install ros-indigo-serial
	ubuntu 16.04 LTS
 		sudo apt-get install ros-kinetic-serial
	ubuntu 18.04 LTS
 		sudo apt-get install ros-melodic-serial

  #Github
	https://github.com/wjwwood/serial
	#Note# 某些设备无法使用 apt-get 的方式安装(eg 妙算Manifold), 需要从github下载，
		catkin_make编译【存疑!】,并添加路径

2.安装步骤
     打开终端(Ctrl+Alt+T)
     $ mkdir -p ~/mavlink_ws/src
     将文件夹 mavlink_uart 复制到目录 ~/mavlink_ws/src 下
     $ cd ~/mavlink_ws 
     $ catkin_make
     $ echo “source ~/mavlink_ws/devel/setup.bash” >> ~/.bashrc
	
3.使用
	需要配合串口通信模块使用 /两个USB串口也行

	#编码&读取
	   roslaunch mavlink_uart mavlink_uart_read.launch 
	#发送&解包
	   roslaunch mavlink_uart mavlink_uart_send.launch 
	#虚拟领机leader Rostopic 发布和订阅
	   roslaunch mavlink_uart visual_leader_state_node.launch

------------------------------------------------------------
其他
#1. Ubuntu 下串口调试助手
eg	CuteCom 
		sudo apt-get install cutecom

#2. 串口通信模块配置
	CUAV雷巡P9的详细配置见PDF文件夹
	1. 1对1通信
	2. 1对多通信
	3. 多对多通信(Mesh组网)

	1对多 和 多对多(Mesh) 需要进行测试和调试

------------------------------------------------------------
mavlink消息读取
mavlink_uart.h/cpp 文件内
	注释文字 "// add test" 下， 通过类似的添加 mavlink 相关函数，即可实现对应信息的解包
------------------------------------------------------------
mavlink消息发送
mavlink_uart.h/cpp 文件内
	while循环 "case 1" 条件中




 
