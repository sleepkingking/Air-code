#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <tf/transform_listener.h>
#include <color_tracking/position.h> 
#include <nav_msgs/Odometry.h>

#define ALTITUDE  2

using namespace std;

tf::Quaternion quat; 
double roll, pitch, yaw;
float init_position_x_take_off =0;
float init_position_y_take_off =0;
float init_position_z_take_off =0;
bool  flag_init_position = false;
nav_msgs::Odometry local_pos;
//回调函数接收无人机的里程计信息
void local_pos_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
    local_pos = *msg;
    if (flag_init_position==false && (local_pos.pose.pose.position.z!=0))
    {
		init_position_x_take_off = local_pos.pose.pose.position.x;
	    init_position_y_take_off = local_pos.pose.pose.position.y;
	    init_position_z_take_off = local_pos.pose.pose.position.z;
        flag_init_position = true;		    
    }
    tf::quaternionMsgToTF(local_pos.pose.pose.orientation, quat);	
	tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
}





mavros_msgs::PositionTarget setpoint_raw;//速度、高度、偏航角控制
 
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

float last_position_x = 0;
float last_position_y = 0;
color_tracking::position landmark;
void landmark_cb(const color_tracking::position::ConstPtr& msg)
{
    landmark = *msg;
}
 

int main(int argc, char **argv)
{
    ros::init(argc, argv, "landing_node");
    ros::NodeHandle nh;
 	
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Subscriber local_pos_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 10, local_pos_cb);
    ros::Subscriber landmark_sub = nh.subscribe<color_tracking::position>("landmark_position", 10, landmark_cb);
 
    ros::Publisher  mavros_setpoint_pos_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);   

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    
    ros::Rate rate(20.0);  
    while(ros::ok() && current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }

	setpoint_raw.type_mask = 1 + 2 + /*4 + 8 + 16 + 32*/ + 64 + 128 + 256 + 512 + 1024 + 2048;
	setpoint_raw.coordinate_frame = 1;
	setpoint_raw.position.x =init_position_x_take_off + 0;
	setpoint_raw.position.y =init_position_y_take_off + 0;
	setpoint_raw.position.z =init_position_z_take_off + ALTITUDE;
	mavros_setpoint_pos_pub.publish(setpoint_raw);
 
    for(int i = 100; ros::ok() && i > 0; --i)
    {
		mavros_setpoint_pos_pub.publish(setpoint_raw);
        ros::spinOnce();
        rate.sleep();
    }
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
 
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    
    ros::Time last_request = ros::Time::now();
 
    while(ros::ok())
    {
    	//请求进入OFFBOARD模式
        if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
            {
                ROS_INFO("Offboard enabled");
            }
           	last_request = ros::Time::now();
           	flag_init_position = false;		    
       	}
        else 
		{
			//请求解锁
			if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0)))
			{
		        if( arming_client.call(arm_cmd) && arm_cmd.response.success)
		       	{
		            ROS_INFO("Vehicle armed");
		        }
		        last_request = ros::Time::now();
		        flag_init_position = false;		    
			}
		}
	    //1、添加高度判断，使得无人机跳出模式切换循环
	    if(fabs(local_pos.pose.pose.position.z- init_position_z_take_off -ALTITUDE)<0.5)
		{	
			if(ros::Time::now() - last_request > ros::Duration(3.0))
			{
				break;
			}
		}
		//2、添加时间判断，使得无人机跳出模式切换循环
		if(ros::Time::now() - last_request > ros::Duration(8.0))
		{
			break;
		}		
		//此处添加是为增加无人机的安全性能，在实际测试过程中，采用某款国产的GPS和飞控，气压计和GPS定位误差极大，
		//导致了无人机起飞后直接飘走，高度和位置都不正常，无法跳出模式循环，导致遥控且无法接管
		//因此增加了时间判断，确保无人机在切入offboard模式和解锁后，确保任何情况下，8秒后遥控器都能切入其他模式接管无人机	
		//注意：一定要确定GPS和飞控传感器都是正常的
		//注意：一定要确定GPS和飞控传感器都是正常的
		//注意：一定要确定GPS和飞控传感器都是正常的
		//注意：一定要确定GPS和飞控传感器都是正常的
		//注意：一定要确定GPS和飞控传感器都是正常的				
		setpoint_raw.type_mask = /*1 + 2 + 4 + 8 + 16 + 32*/ + 64 + 128 + 256 + 512 + 1024 + 2048;
		setpoint_raw.coordinate_frame = 1;
		setpoint_raw.position.x =init_position_x_take_off + 0;
		setpoint_raw.position.y =init_position_y_take_off + 0;
		setpoint_raw.position.z =init_position_z_take_off + ALTITUDE;		
		mavros_setpoint_pos_pub.publish(setpoint_raw);
        ros::spinOnce();
        rate.sleep();
    }   
    last_request = ros::Time::now();

	//控制降落部分
	while(ros::ok())      
	{	
		printf("follow");
		//如果找到地标，控制方向
		if(landmark.iffind)
		{
			//无人机左右移动速度控制
			if(landmark.angleX > 5)
				//setpoint_raw.velocity.y = 0.05*(landmark.angleX - 5);
				setpoint_raw.velocity.y = 0.3;	
	    	else if(landmark.angleX < -5)
				//setpoint_raw.velocity.y = 0.05*(landmark.angleX + 5);
				setpoint_raw.velocity.y = -0.3;
		    else
				setpoint_raw.velocity.y = 0;

	        //无人机前后移动速度控制
			if(landmark.angleY > 5)
	    	    //setpoint_raw.velocity.x = 0.05*(landmark.angleY - 5);
	    	    setpoint_raw.velocity.x = 0.3;
			else if(landmark.angleY < -5)
	    	    //setpoint_raw.velocity.x = 0.05*(landmark.angleY + 5);
	    	    setpoint_raw.velocity.x = -0.3;
			else
				setpoint_raw.velocity.x = 0;
							
			setpoint_raw.type_mask = 1 + 2 +/* 4 + 8 + 16 + 32*/ + 64 + 128 + 256 + 512 + 1024 + 2048;
			setpoint_raw.coordinate_frame = 8;
			setpoint_raw.position.z = init_position_z_take_off+ALTITUDE;
						
		    //如果位置很正开始降落
			cout<<landmark.angleX<<endl;
			cout<<landmark.angleY<<endl;
			
			if((landmark.angleX<=5 && landmark.angleX>=-5) && (landmark.angleY<=5 && landmark.angleY>=-5))
	        {	
	        	setpoint_raw.type_mask = 1 + 2 +/* 4 + 8 + 16 + 32*/ + 64 + 128 + 256 + 512 + 1024 + 2048;
			    setpoint_raw.coordinate_frame = 8;
				setpoint_raw.velocity.x = 0;
				setpoint_raw.velocity.y = 0;
				setpoint_raw.position.z = init_position_z_take_off+ALTITUDE;
			    mavros_setpoint_pos_pub.publish(setpoint_raw);
			}
			last_position_x = local_pos.pose.pose.position.x;
			last_position_y = local_pos.pose.pose.position.y;		
    	}
    	//则保持当前位置不动
		else
		{
			setpoint_raw.type_mask = /*1 + 2 + 4 + 8 + 16 + 32*/ + 64 + 128 + 256 + 512 + 1024 + 2048;
			setpoint_raw.coordinate_frame = 1;
			setpoint_raw.position.x =last_position_x;
			setpoint_raw.position.y =last_position_y;
			setpoint_raw.position.z =init_position_z_take_off + ALTITUDE;		
		}
    	mavros_setpoint_pos_pub.publish(setpoint_raw);
		ros::spinOnce();
		rate.sleep();
	}
  
    return 0;
}

