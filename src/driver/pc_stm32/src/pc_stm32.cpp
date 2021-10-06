#include"pc_stm.h"

#define __NAME__ "pc_stm_node"

using namespace std;
#define MAX_ARRAY 50
#define MAX_STEER_ANGLE 12.3
#define Wheel_D  0.355
#define AXIS_DISTANCE 0.88

#define __NAME__ "base_control"
	
Uppercontrol::Uppercontrol():
	m_reading_status(false),
	prase_flag_(true),
	m_pkg_buffer(new uint8_t[MAX_ARRAY])
{
}

Uppercontrol::~Uppercontrol()
{
	this->closeSerial();
	if(m_pkg_buffer!=NULL)
		delete [] m_pkg_buffer;
}

double Uppercontrol::generate_real_speed(double& temp1,double& temp2)
{	
	left_wheel_speed = M_PI * Wheel_D * temp1 / 60.0;
	right_wheel_speed = M_PI * Wheel_D * temp2 / 60.0;
	speed = (left_wheel_speed + right_wheel_speed) / 2.0;
	return speed;
}

bool Uppercontrol::openSerial(const std::string& port,int baudrate)
{
	try
	{
		m_serial_port = new serial::Serial(port,baudrate,serial::Timeout::simpleTimeout(10)); 
	}
	catch(serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port.");
        return -1;
    }
	if (!m_serial_port->isOpen())
	{
        std::cout << "Serial port: " << port << " failed to open." << std::endl;
		delete m_serial_port;
		m_serial_port = NULL;
		return false;
	} 
	else 
	{
		std::cout << "Serial port: " << port << " opened successfully." << std::endl;
	}

	m_serial_port->flush();
	return true;
}     

void Uppercontrol::closeSerial()
{
	if(m_serial_port!=NULL)
	{
		m_serial_port->close();
		delete m_serial_port;
		m_serial_port = NULL;
	}
}

bool Uppercontrol::init()
{
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");
	odom_pub_ = nh.advertise<nav_msgs::Odometry>("/wheel_odom", 50);
	m_pub_state_ = nh.advertise<driverless_common::VehicleState>(nh_private.param<std::string>("vehicle_state","/vehicle_state"),10);
	m_cmd_state_ = nh.advertise<driverless_common::VehicleCtrlCmd>(nh_private.param<std::string>("vehicle_cmd_state","/vehicle_cmd_state"),10);
	
	std::string pid_params = nh_private.param<std::string>("pid_params","/pid_params");
	
	std::string m_controlCmd_goal = nh_private.param<std::string>("vehicle_cmd","/vehicle_cmd");
	m_sub_controlCmd_ = nh.subscribe(m_controlCmd_goal ,1,&Uppercontrol::D_Cmd_callback, this);
	m_sub_pid_params_ = nh.subscribe(pid_params,1,&Uppercontrol::Pid_callback, this);
	
	nh_private.param<bool>("cmd_feedback", cmd_feedback_, false);
	
	std::string port_name = nh_private.param<std::string>("port_name","/dev/pts/23");
	int baudrate = nh_private.param<int>("baudrate",115200);
	if(!openSerial(port_name,baudrate))
		return false;
	return true;
}

void Uppercontrol::startReading()
{
	if(m_reading_status)
		return ;
	m_read_thread_ptr = boost::shared_ptr<boost::thread>(new boost::thread(&Uppercontrol::readSerialThread,this));   // 智能指针赋值时 调用读取线程函数
}

void Uppercontrol::readSerialThread()
{
	m_reading_status = true;
	
	const int Max_read_size = 200;
	uint8_t * const raw_data_buf = new uint8_t[Max_read_size];
	
	size_t get_len;
		   
	while(ros::ok() && m_reading_status)
	{	
		try
		{
			get_len = m_serial_port->read(raw_data_buf, Max_read_size);
		}
		catch(std::exception &e)
		{
			ROS_ERROR("Error reading from serial port: %s",e.what());
			ros::Duration(0.01).sleep();
			continue;
		}
		if(get_len == 0) 
		{
			ros::Duration(0.01).sleep();
			continue;
		}
		parseIncomingData(raw_data_buf, get_len);
		ros::Duration(0.01).sleep();
		//boost::this_thread::sleep(boost::posix_time::milliseconds(10));
		//print(raw_data_buf, get_len);
		//ROS_INFO("get_len: %d", get_len);
	}
	delete [] raw_data_buf;
}

void Uppercontrol::parseIncomingData(uint8_t* buffer,size_t len)
{	
	static unsigned char pkg_buffer[14];
	static size_t pkg_buffer_index = 0; //数据包缓存定位索引
	size_t pkg_len = 0;
	int pkgId;
	
	for(size_t i=0; i<len; ++i)
	{
		if(0 == pkg_buffer_index) 
		{
			if(0x66 == buffer[i])
				pkg_buffer[pkg_buffer_index++] = buffer[i];
		}
		else if(1 == pkg_buffer_index)
		{
			if(0xcc == buffer[i])
			{
				pkg_buffer[pkg_buffer_index++] = buffer[i];
			}
			else
				pkg_buffer_index = 0;
		}
		else
		{
			if(pkg_buffer_index >= MAX_ARRAY)
			{
				ROS_ERROR("subscript out of bounds!");
				pkg_buffer_index = 0;
				return;
			}
				
			pkg_buffer[pkg_buffer_index++] = buffer[i];
			
			if(pkg_buffer_index == 3)
				pkgId = pkg_buffer[2];
			else if(pkg_buffer_index == 4)
				pkg_len = pkg_buffer[3];
			else if(pkg_buffer_index == pkg_len+5)
			{
				//std::cout << "pkg_len: " << pkg_len << std::endl;
				if(pkg_buffer[pkg_len+4] != sumCheck(pkg_buffer+2,pkg_len+2))
				{
					ROS_ERROR("check failed!");
					pkg_buffer_index = 0;
					return ;
				}
				if((pkg_buffer[2] == 0x02))    //0x02代表下位机向上位机发布的消息包id; 0xe0代表车速信号、制动信号、转角信号有效
					parseFromStmVehicleState(pkg_buffer);
				else if(pkg_buffer[2] == 0x05) //PID
				{
					float kp = ((pkg_buffer[4] * 256 + pkg_buffer[5]) - 30000)/100.0;
					float ki = ((pkg_buffer[6] * 256 + pkg_buffer[7]) - 30000)/100.0;
					float kd = ((pkg_buffer[8] * 256 + pkg_buffer[9]) - 30000)/100.0;
					
					ROS_ERROR("[%s] NOT ERROR, vehicle speed pid: %.3f, %.3f,%.3f", __NAME__, kp,ki,kd);
				}
				pkg_buffer_index = 0;
			}
		}
	}
}

void Uppercontrol::run()
{
	if(init())
	{	
		prase_flag_ = true;
		startReading();
		ROS_INFO("%s initial ok...",ros::this_node::getName().c_str());
	}
	ros::spin();
}

uint8_t Uppercontrol::sumCheck(const uint8_t* buf, int len)
{	
	uint8_t sum = 0;
	for(int i=0; i<len; i++)
		sum += buf[i];
		
	return sum;
}

void Uppercontrol::parseFromStmVehicleState(const unsigned char* buffer)
{	
	real_speed_left        = ((buffer[4]* 256 + buffer[5]) - 30000);
	real_speed_right       = ((buffer[6]* 256 + buffer[7] ) - 30000);
	real_angle             = ((buffer[8]* 256 + buffer[9] ) - 30000)/10.0;
	
	real_touque            = buffer[10] *256 + buffer[11] - 30000;
	real_brake             = buffer[12];
	real_speed             = generate_real_speed(real_speed_left,real_speed_right);
	
	m_state_.speed = real_speed;
	m_state_.roadwheel_angle = real_angle;
	m_state_.driverless = true;
	m_state_.manualCtrlDetected = false;
	
	if(m_cmd_.gear == driverless_common::VehicleCtrlCmd::GEAR_DRIVE)
		m_state_.gear = driverless_common::VehicleState::GEAR_DRIVE;
	else if(m_cmd_.gear == driverless_common::VehicleCtrlCmd::GEAR_REVERSE)
		m_state_.gear = driverless_common::VehicleState::GEAR_REVERSE;
	else if(m_cmd_.gear == driverless_common::VehicleCtrlCmd::GEAR_PARKING)
		m_state_.gear = driverless_common::VehicleState::GEAR_PAKING;
	else if(m_cmd_.gear == driverless_common::VehicleCtrlCmd::GEAR_NEUTRAL)
		m_state_.gear = driverless_common::VehicleState::GEAR_NEUTRAL;
	else
		ROS_ERROR("[%s] The current gear was unknown!", __NAME__);
	m_pub_state_.publish(m_state_);
	
	/*
	if(prase_flag_)
	{
		double x = 0.0;
		double y = 0.0;
		double th = 0.0;
		last_time = ros::Time::now();
		prase_flag_ = false;
	}
	
		double dt = (current_time - last_time).toSec();
		vx = real_speed; // forward
		vy = 0;
		vth = (right_speed - left_speed) / AXIS_DISTANCE;  //左转角度为正
		
		double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
		double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
		double delta_th = vth * dt;

		current_time = ros::Time::now();
		
		x += delta_x;
		y += delta_y;
		th += delta_th;
		//里程计的偏航角转为四元数
		geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
		//首先,创建一个TransformStamped消息,通过tf发送;在current_time发布"odom"坐标到"base_link"的转换
		geometry_msgs::TransformStamped odom_trans;
		odom_trans.header.stamp = current_time;
		odom_trans.header.frame_id = "wheel_odom";
		odom_trans.child_frame_id = "base_link";

		odom_trans.transform.translation.x = x;
		odom_trans.transform.translation.y = y;
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation = odom_quat;

		//send the transform
		odom_broadcaster.sendTransform(odom_trans);

		//next, we'll publish the odometry message over ROS
		odom.header.frame_id = "odom";
		odom.child_frame_id = "base_link";
		odom.header.stamp = current_time;
		//里程数据填充消息
		//set the position
		odom.pose.pose.position.x = x;
		odom.pose.pose.position.y = y;
		odom.pose.pose.position.z = 0.0;
		odom.pose.pose.orientation = odom_quat;

		//set the velocity
		odom.twist.twist.linear.x = vx;
		odom.twist.twist.linear.y = vy;
		odom.twist.twist.angular.z = vth;

		//publish the message
		odom_pub_.publish(odom);
		ROS_DEBUG_STREAM("accumulation_x: " << x << "; accumulation_y: " << y <<"; accumulation_th: " << vth);
		last_time = current_time;
		*/
}

void Uppercontrol::D_Cmd_callback(const driverless_common::VehicleCtrlCmd::ConstPtr& msg)
{	
	static uint8_t pkgId = 0x01;
	const uint8_t dataLen = 6;
	static uint8_t buf[11] = {0x66, 0xcc, pkgId, dataLen };
	
	m_cmd_.gear = msg->gear;
	m_cmd_.speed = msg->speed;
	m_cmd_.roadwheel_angle = msg->roadwheel_angle;
	m_cmd_.brake = msg->brake;
	m_cmd_.emergency_brake = msg->emergency_brake;
	m_cmd_.hand_brake = msg->hand_brake;
	m_cmd_.left_turn_light = msg->left_turn_light;
	m_cmd_.right_turn_light = msg->right_turn_light;
	m_cmd_.brake_light = msg->brake_light;
	m_cmd_.horn = msg->horn;
	if(cmd_feedback_)
		m_cmd_state_.publish(m_cmd_);

	uint16_t u16_speed = msg->speed * 100.0 + 30000;
	buf[5]  = u16_speed >> 8;  
	buf[6]  = u16_speed;  
	
	float float_angle = msg->roadwheel_angle;
	if(float_angle > MAX_STEER_ANGLE)
		float_angle = MAX_STEER_ANGLE;
	else if(float_angle < -MAX_STEER_ANGLE)
		float_angle = -MAX_STEER_ANGLE;
	
	uint16_t u16_angle = float_angle * 100.0 + 30000;
	buf[7]  = u16_angle >> 8;
	buf[8]  = u16_angle;
	
	uint8_t checkVal = sumCheck(buf+2, dataLen+2);
	
	//std::cout << (buf[2]) << "\t" << dataLen+2 << "\t" << int(checkVal) << std::endl;
	buf[dataLen+4] = checkVal;
	//print(buf, dataLen+5);
	m_serial_port-> write(buf,dataLen+5);
}

void Uppercontrol::Pid_callback(const logistics_msgs::PidParams::ConstPtr& pid)
{	
	if(!pid->set) //only query
	{
		uint8_t buf[5] = {0x66, 0xcc, 0x06, 0x00, 0x06};
		m_serial_port-> write(buf,5);
		return;
	}

	uint8_t buf[11];
	buf[0]  = 0x66;
	buf[1]  = 0xcc;
	buf[2]  = 0x05;   //数据包id
	buf[3]  = 0x06;   //数据长度（bu含校验位）
	
	uint16_t sum = pid->kp * 100.0 + 30000;
	buf[4]  = sum >> 8;  
	buf[5]  = sum;  
	
	uint16_t sun = pid->ki * 100.0 + 30000;
	buf[6]  = sun >> 8;
	buf[7]  = sun;
	
	uint16_t sup = pid->kd * 100.0 + 30000;
	buf[8]  = sup >> 8;
	buf[9]  = sup;
	
	buf[10] = buf[2] + buf[3] + buf[4] + buf[5] + buf[6] + buf[7] + buf[8] + buf[9];   //校验位
	
	//std::cout << "set pid: " << pid->kp << "\t" << pid->ki << "\t" << pid->kd << "\n";
	
	m_serial_port-> write(buf,11);
}

void Uppercontrol::stopReading()
{
	m_reading_status = false;
}

int main(int argc,char** argv)
{
	ros::init(argc,argv,"Uppercontrol_node");
	Uppercontrol con;
	con.run();
}
