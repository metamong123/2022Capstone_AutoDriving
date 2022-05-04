#include "ERP42Interface/erp42_interface.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/Imu.h"
#include <cmath>
#include <iostream>

using namespace unmansol::erp42;

double yaw, old_yaw, yaw_angle, r_delta_time, old_time, before_yaw;
int i=0;
ERP42Interface::ERP42Interface()
    : m_odom_x(0.0),
      m_odom_y(0.0),
      m_odom_yaw(0.0),
      m_linear_vel(0.0),
      m_angular_vel(0.0),
      m_wheel_base(0.0),
      m_steer_angle(0.0),
      m_delta_encoder(0),
      m_nh("~"),
      m_encoder(0),
      m_last_encoder(0),
      m_wheel_pos(0.0)
{
    InitNode();
}
void YawCallback(std_msgs::Float64 msg)
{
    
    yaw=msg.data;
    //std::cout << "yaw_radian :" << yaw << " ";
}

void ImuCompleCallback(sensor_msgs::Imu data)
{
    double time=data.header.stamp.toSec();
    double r_delta_time=time-old_time;
    old_time=time;
    
    yaw += DEG2RAD(data.angular_velocity.z) * r_delta_time ;
    if (yaw >= DEG2RAD(180))
    {
	yaw=yaw-DEG2RAD(360);
    }
    else if (yaw <= DEG2RAD(-180))
    {
	yaw=yaw+DEG2RAD(360);
    }
    std::cout << "imu_delta_time :" << r_delta_time << " ";
    std::cout << "imu_angle_vel_z :" << data.angular_velocity.z << " " ;
    
    
    
}

/*void ImuCallback(sensor_msgs::Imu data)
{
    double time=data.header.stamp.toSec();
    double r_delta_time=time-old_time;
    old_time=time;
    
    yaw_angle += DEG2RAD(data.angular_velocity.z) * r_delta_time ;
    if (yaw_angle >= DEG2RAD(180))
    {
	yaw_angle=yaw_angle-DEG2RAD(360);
    }
    else if (yaw_angle <= DEG2RAD(-180))
    {
	yaw_angle=yaw_angle+DEG2RAD(360);
    }
    std::cout << "imu_delta_time :" << r_delta_time << " ";
    std::cout << "imu_angle_vel_z :" << data.angular_velocity.z << " " ;
    
    
    
}*/

void YawZCallback(std_msgs::Float64 msg)
{

   yaw=msg.data;
   before_yaw = msg.data;
    
}

void ERP42Interface::InitNode()
{
    ns_ = ros::this_node::getNamespace();

    if (this->ns_ == "/erp42_can")
    {
        m_sub_steer   = m_nh.subscribe(this->ns_ + "/feedback1",1, &ERP42Interface::CANSteerCallback, this);
        m_sub_encoder = m_nh.subscribe(this->ns_ + "/feedback2",1, &ERP42Interface::CANEncoderCallback, this);
    }
    else
    {
        m_sub_encoder = m_nh.subscribe(this->ns_ + "/feedback",1,&ERP42Interface::SerialDriveCallback, this);
	m_sub_imu=m_nh.subscribe("/imu_yaw",1,YawCallback);
	//m_sub_Imu=m_nh.subscribe("/imu_data",1,ImuCallback);
	//m_sub_yaw_z = m_nh.subscribe("/imu_yaw_z",1,YawZCallback);
        //m_sub_imu_comple=m_nh.subscribe("/imu/data",1,ImuCompleCallback);
    }
}




// *****************************************************************************
// Set params
void ERP42Interface::SetParams(const double &wheel_radius,
                               const double &wheel_base,
                               const double &wheel_tread,
                               const double &max_vel,
                               const double &min_vel,
                               const double &max_steer_angle,
                               const double &min_steer_angle)
{

    //! Wheel Radius
    m_wheel_radius = wheel_radius;
    ROS_INFO("Wheel Radius : %lf [m]", wheel_radius);

    //! Wheel Base
    m_wheel_base = wheel_base;
    ROS_INFO("Wheel Base : %lf [m]", m_wheel_base);

    //! Tread width[m]
    m_wheel_tread = wheel_tread;
    ROS_INFO("Wheel Tread : %lf [m]", m_wheel_tread);

    //! Max Vel
    m_max_vel = max_vel;
    ROS_INFO("Max Velocity : %lf [m/s]", m_max_vel);

    //! Min Vel
    m_min_vel = min_vel;
    ROS_INFO("Min Velocity : %lf [m/s]", m_min_vel);

    //! Max Steer Angle
    m_max_steer_angle = max_steer_angle;
    ROS_INFO("Max Steer Angle : %lf [degree]", m_max_steer_angle);

    //! Min Steer Angle
    m_min_steer_angle = min_steer_angle;
    ROS_INFO("Min Steer Angle : %lf [degree]", m_min_steer_angle);

}

void ERP42Interface::CANEncoderCallback(const erp42_msgs::CANFeedBack::Ptr &msg)
{
    m_encoder = msg->encoder;
}

void ERP42Interface::CANSteerCallback(const erp42_msgs::CANFeedBack::Ptr &msg)
{
    m_steer_angle = DEG2RAD(msg->steer);  
}

void ERP42Interface::SerialDriveCallback(const erp42_msgs::SerialFeedBack::Ptr &msg)
{
    m_encoder = msg->encoder;
    m_steer_angle = -DEG2RAD(msg->steer);
    if (m_steer_angle < 0) 
	m_steer_angle *= 0.9;
    else if (m_steer_angle > 0) 
	m_steer_angle *= 0.73;
}
// *****************************************************************************
// Calculate ERP42 odometry
void ERP42Interface::CalculateOdometry(const double &delta_time)
{
    m_delta_encoder = m_encoder - m_last_encoder;
    m_last_encoder = m_encoder;
    m_wheel_pos = TICK2RAD * m_delta_encoder;
    
    L = m_wheel_base / tan(m_steer_angle);
    r_center = sqrt(pow(L,2) + pow(m_wheel_base,2));
    r_in = sqrt(pow((L-m_wheel_tread/2),2) + pow(m_wheel_base,2));
    r_out = sqrt(pow((L+m_wheel_tread/2),2) + pow(m_wheel_base,2));
    theta_center = m_steer_angle;
    theta_in = atan(m_wheel_tread/(L-m_wheel_tread/2));
    theta_out = atan(m_wheel_tread/(L+m_wheel_tread/2));
    if (m_steer_angle < 0)
	m_delta_pos = m_wheel_radius * m_wheel_pos * r_center / r_in;
    
    else if (m_steer_angle > 0)
	m_delta_pos = m_wheel_radius * m_wheel_pos * r_center / r_out;
    
    else
	m_delta_pos = m_wheel_radius * m_wheel_pos;
    
    m_linear_vel = m_delta_pos / delta_time;
    m_angular_vel = tan(m_steer_angle) * m_linear_vel / m_wheel_base;
    
    if (i==0){
    	old_yaw =0;//0.349065850399;//0.174+1.57079632679; //+0.1;//-1.954768762;//1.954768762;// +1.18682389;
	before_yaw = yaw;
    }	
    /*else {
	if (abs(yaw-before_yaw)>0.1){
		before_yaw = yaw;
		yaw = yaw * 0.9;
	}
    }*/
    m_odom_yaw = old_yaw + yaw; //m_angular_vel * delta_time;
    m_odom_x -= m_delta_pos * cos(m_odom_yaw);
    m_odom_y -= m_delta_pos * sin(m_odom_yaw);
    

    std::cout << "Duration: " << delta_time << " ";
    std::cout << "Previous Encoder: " << m_last_encoder << " ";
    //  std::cout << "Delta Encoder: " << m_delta_encoder << " ";
    std::cout << "Wheel Pose: " << m_steer_angle << " ";
    std::cout << "Delta Pose: " << m_delta_pos << " ";
    std::cout << "Linear Vel: " << m_linear_vel<< " ";
    std::cout << "Angular Vel: " << m_angular_vel<< " ";
    std::cout << "Odom X : " << m_odom_x << " ";
    std::cout << " Y : " << m_odom_y << " ";
    std::cout << " Yaw : " << m_odom_yaw << std::endl;
    std::cout << std::endl;

    if (isnan(m_delta_pos)) m_delta_pos = 0.0;
    if (isnan(m_angular_vel)) m_angular_vel = 0.0;
    i++;
}

// *****************************************************************************
// Reset ERP42 odometry
void ERP42Interface::ResetOdometry()
{
    SetOdometry(0.0, 0.0, 0.0);
}

// *****************************************************************************
// Set ERP42 odometry
void ERP42Interface::SetOdometry(const double &new_x, const double &new_y, const double &new_yaw)
{
    m_odom_x   = new_x;
    m_odom_y   = new_y;
    m_odom_yaw = new_yaw;
}
